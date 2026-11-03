//! Raw register operations shared by every engine that owns a PIO block.

use rp_pac as pac;

use crate::clock::PioClockDivider;
use crate::sm::PioBlock;

/// `IO_BANK0` function select for SIO, function 5 on every RP family.
pub(crate) const FUNCSEL_SIO: u8 = 5;
/// `IO_BANK0` function select for PIO0; PIO1 is the next function.
pub(crate) const FUNCSEL_PIO0: u8 = 6;
pub(crate) const FUNCSEL_PIO1: u8 = 7;

/// GPIOs the selected chip's bank 0 exposes.
pub(crate) const GPIO_COUNT: u8 = if cfg!(feature = "rp2040") { 30 } else { 32 };

/// Enable or disable one SM, leaving the block's others untouched.
pub(crate) fn set_enabled(pio: pac::pio::Pio, sm: usize, enabled: bool) {
    pio.ctrl().modify(|w| {
        let bits = w.sm_enable();
        w.set_sm_enable(if enabled {
            bits | 1 << sm
        } else {
            bits & !(1 << sm)
        });
    });
}

/// Execute one instruction immediately, ahead of whatever the SM was doing.
pub(crate) fn exec(pio: pac::pio::Pio, sm: usize, instr: u16) {
    pio.sm(sm).instr().write(|w| w.set_instr(instr));
}

/// `set pins, <high>` on one pad, restoring the pin mapping afterwards.
pub(crate) fn exec_set_pin(pio: pac::pio::Pio, sm: usize, gpio: u8, high: bool) {
    exec_set(pio, sm, gpio, 0xe000 | high as u16);
}

/// `set pindirs, <output>` on one pad, restoring the pin mapping afterwards.
pub(crate) fn exec_set_pindir(pio: pac::pio::Pio, sm: usize, gpio: u8, output: bool) {
    exec_set(pio, sm, gpio, 0xe080 | output as u16);
}

fn exec_set(pio: pac::pio::Pio, sm: usize, gpio: u8, instr: u16) {
    let regs = pio.sm(sm);
    let saved = regs.pinctrl().read();
    regs.pinctrl().modify(|w| {
        w.set_set_base(gpio);
        w.set_set_count(1);
    });
    exec(pio, sm, instr);
    regs.pinctrl().write_value(saved);
}

/// Toggling the FIFO join flushes both FIFOs; the register returns to its configured value.
pub(crate) fn clear_fifos(pio: pac::pio::Pio, sm: usize) {
    let regs = pio.sm(sm);
    let joined = regs.shiftctrl().read().fjoin_rx();
    regs.shiftctrl().modify(|w| w.set_fjoin_rx(!joined));
    regs.shiftctrl().modify(|w| w.set_fjoin_rx(joined));
}

pub(crate) fn restart(pio: pac::pio::Pio, sm: usize) {
    pio.ctrl().modify(|w| w.set_sm_restart(1 << sm));
}

/// Program one SM's divider and restart its clock-divider phase.
pub(crate) fn set_divider(pio: pac::pio::Pio, sm: usize, divider: PioClockDivider) {
    pio.sm(sm).clkdiv().write(|w| {
        w.set_int(divider.integer);
        w.set_frac(divider.fractional);
    });
    pio.ctrl().modify(|w| w.set_clkdiv_restart(1 << sm));
}

/// Write `words` into instruction memory from `offset`.
pub(crate) fn load(pio: pac::pio::Pio, offset: usize, words: &[u16]) {
    for (i, word) in words.iter().enumerate() {
        pio.instr_mem(offset + i).write(|w| w.set_instr_mem(*word));
    }
}

/// GPIO base 0 on RP2350: every pad this crate addresses is below 32.
pub(crate) fn gpio_base_zero(pio: pac::pio::Pio) {
    #[cfg(feature = "rp235x")]
    pio.gpiobase().write(|w| w.set_gpiobase(false));
    #[cfg(not(feature = "rp235x"))]
    let _ = pio;
}

/// Take the block out of reset and wait for it to answer.
pub(crate) fn unreset(block: PioBlock) {
    pac::RESETS.reset().modify(|w| match block {
        PioBlock::Pio0 => w.set_pio0(false),
        PioBlock::Pio1 => w.set_pio1(false),
    });
    loop {
        let done = pac::RESETS.reset_done().read();
        let ready = match block {
            PioBlock::Pio0 => done.pio0(),
            PioBlock::Pio1 => done.pio1(),
        };
        if ready {
            break;
        }
    }
}

pub(crate) fn set_funcsel(gpio: u8, funcsel: u8) {
    pac::IO_BANK0
        .gpio(gpio as usize)
        .ctrl()
        .modify(|w| w.set_funcsel(funcsel));
}
