//! The [`PioShifter`] mechanism over raw PIO registers, with no runtime or HAL in the path.

use daplink_rs::dap_rs::swj::Pins;
use daplink_rs::{spin_cycles, ScanChainStorage};
use rp_pac as pac;

use crate::clock::{clock_divider, CYCLES_PER_BIT, MIN_DIVIDER};
use crate::pins::{PinMap, PioSwdPins};
use crate::program::select_program;
use crate::shifter::{start_idle, PioShifter, SwjPinsSio};
use crate::swd::PioSwdDeps;

/// The SWCLK the state machine comes up at; the host reprograms it before it matters.
const DEFAULT_SWCLK_HZ: u32 = 1_000_000;

/// `IO_BANK0` function select for SIO, function 5 on every RP family.
const FUNCSEL_SIO: u8 = 5;
/// `IO_BANK0` function select for PIO0, function 6 on every RP family.
const FUNCSEL_PIO0: u8 = 6;

/// The state machine this backend runs on; index 0 of the chosen block.
const SM_INDEX: usize = 0;

/// Which PIO block drives the wire.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum PioBlock {
    /// PIO0.
    Pio0,
    /// PIO1.
    Pio1,
}

/// The raw-register state machine: the mechanism half of the PIO SWD backend.
pub struct PioSm {
    pio: pac::pio::Pio,
    funcsel: u8,
    swclk: u8,
    swdio_out: u8,
    swdio_in: u8,
    oen: Option<u8>,
    nreset: Option<u8>,
    sys_hz: u32,
}

/// Bring up the PIO SWD backend and hand back the DAP dependencies over it.
///
/// # Safety
///
/// The caller must own the PIO block, its state machine 0, and every pad in `pins` exclusively.
pub unsafe fn pio_swd_deps<const N: usize>(
    block: PioBlock,
    pins: PioSwdPins,
    sys_hz: u32,
    scan: &'static ScanChainStorage<N>,
) -> PioSwdDeps<PioSm> {
    let (sm, entries) = PioSm::new(block, pins, sys_hz);
    PioSwdDeps::new(sm, entries, sys_hz, scan)
}

impl PioSm {
    /// Load the program, configure the state machine, park it at the dispatcher and enable it.
    ///
    /// # Safety
    ///
    /// The caller must own the PIO block, its state machine 0, and every pad in `pins`.
    pub unsafe fn new(
        block: PioBlock,
        pins: PioSwdPins,
        sys_hz: u32,
    ) -> (Self, crate::program::ProgramEntries) {
        let (pio, funcsel) = match block {
            PioBlock::Pio0 => (pac::PIO0, FUNCSEL_PIO0),
            PioBlock::Pio1 => (pac::PIO1, FUNCSEL_PIO0 + 1),
        };
        unreset(block);

        let mut sm = Self {
            pio,
            funcsel,
            swclk: pins.swclk,
            swdio_out: pins.swdio_out,
            swdio_in: pins.readback(),
            oen: pins.oen,
            nreset: pins.nreset,
            sys_hz,
        };

        let prog = select_program(sm.oen.is_some());
        let map = PinMap::new(&pins, prog.side_set_bits);
        // The block is ours, so the program loads at offset 0 and the entries are its labels.
        let entries = prog.entries(0);
        sm.set_enabled(false);
        for (i, word) in prog.instructions.iter().enumerate() {
            pio.instr_mem(i).write(|w| w.set_instr_mem(*word));
        }

        // GPIO base 0 on RP2350: every pad this backend addresses is below 32.
        #[cfg(feature = "rp235x")]
        pio.gpiobase().write(|w| w.set_gpiobase(false));

        let regs = pio.sm(SM_INDEX);
        regs.execctrl().modify(|w| {
            w.set_wrap_bottom(prog.wrap_target);
            w.set_wrap_top(prog.wrap);
            w.set_side_en(prog.side_set_opt);
            w.set_side_pindir(false);
        });
        regs.shiftctrl().modify(|w| {
            // LSB first both ways; auto refill is off because every data word has a command word.
            w.set_in_shiftdir(true);
            w.set_out_shiftdir(true);
            w.set_autopush(false);
            w.set_autopull(false);
            // A threshold of 32 encodes as 0.
            w.set_push_thresh(0);
            w.set_pull_thresh(0);
        });
        regs.pinctrl().write(|w| {
            // A split wiring differs only here: `IN` on the readback pad, `OUT`/`SET` on the driven one.
            w.set_out_base(map.out_base);
            w.set_out_count(1);
            w.set_set_base(map.set_base);
            w.set_set_count(1);
            w.set_in_base(map.in_base);
            w.set_sideset_base(map.sideset_base);
            w.set_sideset_count(map.sideset_count);
        });

        let div = clock_divider(sys_hz, DEFAULT_SWCLK_HZ, CYCLES_PER_BIT, MIN_DIVIDER)
            .unwrap_or(MIN_DIVIDER);
        sm.set_clock_divider(div);

        // Claim the pads for PIO, then nRESET for SIO; nRESET is never PIO-owned.
        claim_pad(sm.swclk, funcsel);
        claim_pad(sm.swdio_out, funcsel);
        if sm.swdio_in != sm.swdio_out {
            claim_pad(sm.swdio_in, funcsel);
        }
        if let Some(oe) = sm.oen {
            claim_pad(oe, funcsel);
        }
        if let Some(nrst) = sm.nreset {
            claim_pad(nrst, FUNCSEL_SIO);
            // Open drain: the output register stays low, so asserting is enabling the driver.
            sio_drive_level(nrst, false);
            sio_set_oe(nrst, false);
        }

        start_idle(&mut sm, entries.get_next_cmd);
        (sm, entries)
    }

    /// The state machine's bit in the block-wide control masks.
    const fn mask(&self) -> u8 {
        1 << SM_INDEX
    }

    /// Execute one instruction immediately, ahead of whatever the SM was doing.
    fn exec(&mut self, instr: u16) {
        self.pio.sm(SM_INDEX).instr().write(|w| w.set_instr(instr));
    }

    /// Set one pad's direction from the SM's view, restoring the pin mapping afterwards.
    fn exec_set_pindir(&mut self, gpio: u8, output: bool) {
        let regs = self.pio.sm(SM_INDEX);
        let saved = regs.pinctrl().read();
        regs.pinctrl().modify(|w| {
            w.set_set_base(gpio);
            w.set_set_count(1);
        });
        // `set pindirs, <output>`: SET opcode, destination PINDIRS.
        self.exec(0xE080 | (output as u16));
        self.pio.sm(SM_INDEX).pinctrl().write_value(saved);
    }

    /// Set one pad's output latch from the SM's view, restoring the pin mapping afterwards.
    fn exec_set_pin(&mut self, gpio: u8, high: bool) {
        let regs = self.pio.sm(SM_INDEX);
        let saved = regs.pinctrl().read();
        regs.pinctrl().modify(|w| {
            w.set_set_base(gpio);
            w.set_set_count(1);
        });
        // `set pins, <high>`: SET opcode, destination PINS.
        self.exec(0xE000 | (high as u16));
        self.pio.sm(SM_INDEX).pinctrl().write_value(saved);
    }
}

impl PioShifter for PioSm {
    fn put(&mut self, word: u32) {
        while self.pio.fstat().read().txfull() & self.mask() != 0 {}
        self.pio.txf(SM_INDEX).write_value(word);
    }

    fn get(&mut self) -> u32 {
        while self.pio.fstat().read().rxempty() & self.mask() != 0 {}
        self.pio.rxf(SM_INDEX).read()
    }

    fn set_clock_divider(&mut self, divider: u16) {
        self.pio.sm(SM_INDEX).clkdiv().write(|w| {
            w.set_int(divider);
            w.set_frac(0);
        });
        self.pio
            .ctrl()
            .modify(|w| w.set_clkdiv_restart(self.mask()));
    }

    fn set_enabled(&mut self, enabled: bool) {
        let mask = self.mask();
        self.pio.ctrl().modify(|w| {
            let bits = w.sm_enable();
            w.set_sm_enable(if enabled { bits | mask } else { bits & !mask });
        });
    }

    fn clear_fifos(&mut self) {
        // Toggling the FIFO join flushes both FIFOs; the register returns to its configured value.
        let regs = self.pio.sm(SM_INDEX);
        let joined = regs.shiftctrl().read().fjoin_rx();
        regs.shiftctrl().modify(|w| w.set_fjoin_rx(!joined));
        regs.shiftctrl().modify(|w| w.set_fjoin_rx(joined));
    }

    fn restart(&mut self) {
        self.pio.ctrl().modify(|w| w.set_sm_restart(self.mask()));
    }

    fn exec_jmp(&mut self, addr: u8) {
        // An unconditional `jmp` is the address alone; opcode and condition are both zero.
        self.exec(addr as u16);
    }

    fn force_oen_released(&mut self) {
        if let Some(oe) = self.oen {
            self.exec_set_pin(oe, true);
        }
    }

    fn force_swclk_low(&mut self) {
        self.exec_set_pin(self.swclk, false);
    }

    fn force_pindirs_idle(&mut self) {
        self.exec_set_pindir(self.swclk, true);
        self.exec_set_pindir(self.swdio_out, false);
        if let Some(oe) = self.oen {
            self.exec_set_pindir(oe, true);
        }
    }

    fn pads_to_sio(&mut self) {
        set_funcsel(self.swclk, FUNCSEL_SIO);
        set_funcsel(self.swdio_out, FUNCSEL_SIO);
    }

    fn pads_to_pio(&mut self) {
        set_funcsel(self.swclk, self.funcsel);
        set_funcsel(self.swdio_out, self.funcsel);
    }
}

impl SwjPinsSio for PioSm {
    fn swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        // SWCLK is a driven output: high only when requested, low at idle.
        sio_set_oe(self.swclk, true);
        sio_drive_level(
            self.swclk,
            mask.contains(Pins::SWCLK) && output.contains(Pins::SWCLK),
        );

        // SWDIO drives only when requested; otherwise the pull-up holds the bus.
        if mask.contains(Pins::SWDIO) {
            sio_set_oe(self.swdio_out, true);
            sio_drive_level(self.swdio_out, output.contains(Pins::SWDIO));
        } else {
            sio_set_oe(self.swdio_out, false);
        }

        if let Some(nrst) = self.nreset {
            if mask.contains(Pins::NRESET) {
                // Open drain: assert drives low, release goes high-Z for the target pull-up.
                sio_set_oe(nrst, !output.contains(Pins::NRESET));
            }
        }

        if wait_us != 0 {
            spin_cycles(wait_us.saturating_mul(self.sys_hz / 1_000_000));
        }

        let mut read = Pins::empty();
        read.set(Pins::SWCLK, sio_read(self.swclk));
        read.set(Pins::SWDIO, sio_read(self.swdio_in));
        read.set(Pins::NRESET, self.nreset.is_none_or(sio_read));
        // No nTRST line on this backend; report it released.
        read.set(Pins::NTRST, true);
        read
    }
}

/// Take the block out of reset and wait for it to answer.
fn unreset(block: PioBlock) {
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

/// Enable the pad's input buffer and route it to `funcsel`; every other pad setting is shared
/// between the SIO and PIO functions, so a later handover moves the function select alone.
fn claim_pad(gpio: u8, funcsel: u8) {
    pac::PADS_BANK0.gpio(gpio as usize).modify(|w| {
        w.set_ie(true);
        w.set_od(false);
        // RP2350 pads come out of reset isolated.
        #[cfg(feature = "rp235x")]
        w.set_iso(false);
    });
    set_funcsel(gpio, funcsel);
}

fn set_funcsel(gpio: u8, funcsel: u8) {
    pac::IO_BANK0
        .gpio(gpio as usize)
        .ctrl()
        .modify(|w| w.set_funcsel(funcsel));
}

fn sio_set_oe(gpio: u8, output: bool) {
    let oe = pac::SIO.gpio_oe(0);
    if output {
        oe.value_set().write_value(1 << gpio);
    } else {
        oe.value_clr().write_value(1 << gpio);
    }
}

fn sio_drive_level(gpio: u8, high: bool) {
    let out = pac::SIO.gpio_out(0);
    if high {
        out.value_set().write_value(1 << gpio);
    } else {
        out.value_clr().write_value(1 << gpio);
    }
}

fn sio_read(gpio: u8) -> bool {
    (pac::SIO.gpio_in(0).read() >> gpio) & 1 != 0
}
