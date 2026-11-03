//! RP PIO1 state machine for complete ACK-aware JTAG-DP transfers.

use rp_pac as pac;

use crate::asm::{jmp, set, Cond, Dest, SideSet};
use crate::autonomous_program::{
    ir_scan_words, request_word, AutonomousProgram, ACK_SHIFT, IR_DONE_IRQ, IR_EXEC_BOTTOM,
    WAIT_REVERSED,
};
use crate::clock::{
    jtag_clock_divider, jtag_timing, JtagTiming, PioClockDivider, JTAG_MIN_DIVIDER, SLOWEST_DIVIDER,
};
use crate::composition::{PioJtagAccelerator, MAX_BYPASS_BEFORE};
use crate::pins::PioJtagPins;
use crate::regs::{self, set_funcsel, FUNCSEL_PIO0, FUNCSEL_PIO1, GPIO_COUNT};
use crate::sm::PioBlock;

const SM: usize = 0;
const IR_DONE_IRQ_MASK: u8 = 1 << IR_DONE_IRQ;
const SS2: SideSet = SideSet { data_bits: 2 };
const TX_FIFO_DEPTH: usize = 4;

/// Dedicated RP PIO1 accelerator for complete JTAG-DP data scans.
pub struct RpAutonomousJtag {
    pio: pac::pio::Pio,
    pins: PioJtagPins,
    sys_hz: u32,
    timing: JtagTiming,
    geometry: Option<(u8, u8)>,
    ir_pending: bool,
    active: bool,
}

impl RpAutonomousJtag {
    /// Configure PIO1 SM0 without taking the pads away from the generic PIO0 engine.
    ///
    /// # Safety
    /// Own the whole PIO1 block, including IRQ flags 0 and 1, for the engine's lifetime, and its pads while active.
    pub unsafe fn new(pins: PioJtagPins, sys_hz: u32) -> Self {
        regs::unreset(PioBlock::Pio1);
        Self::from_pio(pac::PIO1, pins, sys_hz)
    }

    /// Build on an exclusively owned, disabled PIO1 block.
    pub(crate) unsafe fn from_pio(pio: pac::pio::Pio, pins: PioJtagPins, sys_hz: u32) -> Self {
        assert!(valid_pins(&pins));
        let timing = jtag_timing(sys_hz).expect("unsupported PIO JTAG system clock");
        let program = AutonomousProgram::new(0, 1, timing);

        regs::set_enabled(pio, SM, false);
        load_program(pio, &program);
        regs::gpio_base_zero(pio);
        let regs = pio.sm(SM);
        regs.execctrl().write(|w| {
            w.set_wrap_bottom(program.wrap_bottom);
            w.set_wrap_top(program.wrap_top);
            w.set_side_en(true);
            w.set_side_pindir(false);
        });
        regs.shiftctrl().write(|w| {
            // Each path pushes its completion word only after returning the TAP to Idle.
            w.set_in_shiftdir(true);
            w.set_out_shiftdir(true);
            w.set_autopush(false);
            w.set_autopull(false);
            w.set_push_thresh(0);
            w.set_pull_thresh(0);
            w.set_fjoin_rx(false);
            w.set_fjoin_tx(false);
        });
        regs.pinctrl().write(|w| {
            w.set_out_base(pins.tdi);
            w.set_out_count(1);
            w.set_set_base(pins.tdi);
            w.set_set_count(1);
            w.set_in_base(pins.tdo);
            w.set_sideset_base(pins.tck);
            w.set_sideset_count(3);
        });

        // TDO must bypass the synchronizer for the late-sample cell.
        let bypass = pio.input_sync_bypass().read();
        pio.input_sync_bypass()
            .write_value(bypass | (1 << pins.tdo));

        let default_divider = jtag_clock_divider(sys_hz, 1_000_000)
            .unwrap_or(PioClockDivider::integer(JTAG_MIN_DIVIDER));
        regs::set_divider(pio, SM, default_divider);

        Self {
            pio,
            pins,
            sys_hz,
            timing,
            geometry: Some((0, 1)),
            ir_pending: false,
            active: false,
        }
    }

    /// Recover the disabled PIO1 block for another program.
    pub(crate) fn into_pio(self) -> pac::pio::Pio {
        assert!(!self.active, "cannot reload an active JTAG state machine");
        assert!(!self.ir_pending, "cannot reload a pending JTAG instruction");
        self.pio
    }

    fn flush_and_restart(&mut self) {
        regs::clear_fifos(self.pio, SM);
        regs::restart(self.pio, SM);
        self.pio.irq().write(|w| w.set_irq(IR_DONE_IRQ_MASK));
        self.ir_pending = false;
    }

    fn put(&mut self, word: u32) {
        while self.pio.fstat().read().txfull() & sm_mask() != 0 {}
        self.pio.txf(SM).write_value(word);
    }

    fn put_ir_words(&mut self, words: &[u32]) {
        // Establish FIFO headroom before falling back to per-word backpressure.
        let primed = tx_fifo_headroom(self.pio.flevel().read().tx0());
        for &word in &words[..primed] {
            self.pio.txf(SM).write_value(word);
        }
        for &word in &words[primed..] {
            self.put(word);
        }
    }

    fn get(&mut self) -> u32 {
        while self.pio.fstat().read().rxempty() & sm_mask() != 0 {}
        self.pio.rxf(SM).read()
    }

    fn finish_ir(&mut self) {
        if !self.ir_pending {
            return;
        }
        while self.pio.irq().read().irq() & IR_DONE_IRQ_MASK == 0 {}
        // Wait until the JMP after IRQ reaches the data program's blocking PULL before changing state.
        while self.pio.sm(SM).addr().read().addr() != 0 {}
        self.pio.irq().write(|w| w.set_irq(IR_DONE_IRQ_MASK));
        self.ir_pending = false;
    }
}

impl PioJtagAccelerator for RpAutonomousJtag {
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.finish_ir();
        let divider = jtag_clock_divider(self.sys_hz, max_frequency);
        regs::set_divider(self.pio, SM, divider.unwrap_or(SLOWEST_DIVIDER));
        divider.is_some()
    }

    fn enter(&mut self, bypass_before: u8, bypass_after: u8) {
        assert!(bypass_before <= MAX_BYPASS_BEFORE && bypass_after <= 1);
        debug_assert!(!self.active);
        debug_assert!(!self.ir_pending);
        regs::set_enabled(self.pio, SM, false);
        if self.geometry != Some((bypass_before, bypass_after)) {
            let program = AutonomousProgram::new(bypass_before, bypass_after, self.timing);
            load_program(self.pio, &program);
            self.pio.sm(SM).execctrl().modify(|w| {
                w.set_wrap_bottom(program.wrap_bottom);
                w.set_wrap_top(program.wrap_top);
            });
            self.geometry = Some((bypass_before, bypass_after));
        }
        self.flush_and_restart();

        // Stage Idle levels and directions before changing the GPIO function select.
        regs::exec_set_pin(self.pio, SM, self.pins.tck, false);
        regs::exec_set_pin(self.pio, SM, self.pins.tms, true);
        regs::exec_set_pin(self.pio, SM, self.pins.tdi, true);
        regs::exec_set_pindir(self.pio, SM, self.pins.tdo, false);
        regs::exec_set_pindir(self.pio, SM, self.pins.tdi, true);
        regs::exec_set_pindir(self.pio, SM, self.pins.tms, true);
        regs::exec_set_pindir(self.pio, SM, self.pins.tck, true);
        // The data program requires Y to hold the reversed WAIT ACK across transfers.
        regs::exec(self.pio, SM, set(SS2, Dest::Y, WAIT_REVERSED, None, 0));
        regs::exec(self.pio, SM, 0);

        set_funcsel(self.pins.tdo, FUNCSEL_PIO1);
        set_funcsel(self.pins.tdi, FUNCSEL_PIO1);
        set_funcsel(self.pins.tms, FUNCSEL_PIO1);
        set_funcsel(self.pins.tck, FUNCSEL_PIO1);
        regs::set_enabled(self.pio, SM, true);
        self.active = true;
    }

    fn exit(&mut self) {
        debug_assert!(self.active);
        self.finish_ir();
        regs::set_enabled(self.pio, SM, false);

        // Match staged PIO0 levels before FUNCSEL moves; TCK must stay driven low throughout.
        regs::exec_set_pin(self.pio, SM, self.pins.tck, false);
        regs::exec_set_pin(self.pio, SM, self.pins.tms, true);
        regs::exec_set_pin(self.pio, SM, self.pins.tdi, true);

        set_funcsel(self.pins.tdo, FUNCSEL_PIO0);
        set_funcsel(self.pins.tdi, FUNCSEL_PIO0);
        set_funcsel(self.pins.tms, FUNCSEL_PIO0);
        set_funcsel(self.pins.tck, FUNCSEL_PIO0);

        regs::exec_set_pindir(self.pio, SM, self.pins.tck, false);
        regs::exec_set_pindir(self.pio, SM, self.pins.tms, false);
        regs::exec_set_pindir(self.pio, SM, self.pins.tdi, false);
        regs::exec_set_pindir(self.pio, SM, self.pins.tdo, false);
        self.active = false;
    }

    fn transfer(&mut self, request: u8, data: u32) -> (u32, u32) {
        debug_assert!(self.active);
        let (before, _) = self.geometry.unwrap_or((0, 0));

        // B is queued before the PIO checks ACK, removing the CPU-visible rendezvous.
        self.put(request_word(before, request));
        self.put(data);
        let ack = self.get() >> ACK_SHIFT;
        // The data word, or an empty one after WAIT.
        let captured = self.get();
        let flags = self.pio.irq().read().irq() & IR_DONE_IRQ_MASK;
        if self.ir_pending {
            debug_assert!(flags != 0);
            self.ir_pending = false;
        }
        self.pio.irq().write(|w| w.set_irq(flags));
        (ack, captured)
    }

    fn shift_ir(&mut self, ir: u32) -> bool {
        if !self.active || self.geometry != Some((0, 1)) {
            return false;
        }
        self.finish_ir();
        let Some(words) = ir_scan_words(ir, self.timing) else {
            return false;
        };

        // Establish the next TMS value with TCK low before the executor's first rising edge.
        regs::exec(self.pio, SM, set(SS2, Dest::Pins, 1, Some(0b10), 0));
        regs::exec(
            self.pio,
            SM,
            jmp(SS2, Cond::Always, IR_EXEC_BOTTOM, None, 0),
        );
        self.put_ir_words(&words);
        self.ir_pending = true;
        true
    }
}

const fn sm_mask() -> u8 {
    1 << SM
}

const fn tx_fifo_headroom(level: u8) -> usize {
    TX_FIFO_DEPTH.saturating_sub(level as usize)
}

fn valid_pins(pins: &PioJtagPins) -> bool {
    pins.valid_for_gpio_count(GPIO_COUNT)
}

fn load_program(pio: pac::pio::Pio, program: &AutonomousProgram) {
    regs::load(pio, 0, &program.words[..usize::from(program.len)]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ir_priming_never_overfills_the_tx_fifo() {
        assert_eq!(tx_fifo_headroom(0), 4);
        assert_eq!(tx_fifo_headroom(1), 3);
        assert_eq!(tx_fifo_headroom(2), 2);
        assert_eq!(tx_fifo_headroom(3), 1);
        assert_eq!(tx_fifo_headroom(4), 0);
    }

    #[test]
    fn pin_validation_uses_the_selected_rp_family_limit() {
        let pico = PioJtagPins {
            tck: 2,
            tms: 3,
            tdi: 6,
            tdo: 7,
        };
        assert!(valid_pins(&pico));

        let upper = PioJtagPins {
            tck: if cfg!(feature = "rp2040") { 28 } else { 30 },
            tms: if cfg!(feature = "rp2040") { 29 } else { 31 },
            tdi: if cfg!(feature = "rp2040") { 26 } else { 28 },
            tdo: if cfg!(feature = "rp2040") { 27 } else { 29 },
        };
        assert!(valid_pins(&upper));

        let out_of_range = PioJtagPins {
            tdi: if cfg!(feature = "rp2040") { 30 } else { 32 },
            ..pico
        };
        assert!(!valid_pins(&out_of_range));
    }
}
