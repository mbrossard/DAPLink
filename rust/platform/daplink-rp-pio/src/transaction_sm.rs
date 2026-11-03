//! Raw PIO1 state machine for the ACK-continuation SWD transfer program.

use rp_pac as pac;

use daplink_rs::CommandFence;

use crate::clock::{
    swd_transaction_clock_divider, PioClockDivider, SLOWEST_DIVIDER, TRANSACTION_MIN_DIVIDER,
};
use crate::pins::{PinMap, PioSwdPins};
use crate::regs::{self, set_funcsel, FUNCSEL_PIO0, FUNCSEL_PIO1};
use crate::sequence::XferError;
use crate::sm::PioBlock;
use crate::swd_transaction_hybrid::PioSwdTransferAccelerator;
use crate::transaction::{PioSwdTransaction, PioSwdTransactionIo};
use crate::transaction_program::select_transaction_program;

const SM: usize = 0;
const DEFAULT_SWCLK_HZ: u32 = 1_000_000;

struct RpPioSwdTransactionIo {
    pio: pac::pio::Pio,
    pins: PioSwdPins,
    sys_hz: u32,
    active: bool,
    in_flight: bool,
}

/// PIO1 accelerator for fixed SWD register transfers, composed beside generic PIO0 SWJ.
pub struct RpPioSwdTransaction {
    engine: PioSwdTransaction<RpPioSwdTransactionIo>,
}

/// Sealed command-boundary token; its completion predicate holds only after the transfer path observed ACK.
pub struct RpPioSwdTransactionFence {
    _private: (),
}

impl RpPioSwdTransaction {
    /// Configure PIO1 without taking the pads from PIO0; the fence belongs on the DAP facade owning this engine.
    ///
    /// # Safety
    /// Own the whole PIO1 block; PIO0 alone shares the pads and must be parked before `enter`.
    pub unsafe fn new(pins: PioSwdPins, sys_hz: u32) -> (Self, RpPioSwdTransactionFence) {
        regs::unreset(PioBlock::Pio1);
        let engine = Self::from_pio(pac::PIO1, pins, sys_hz);
        (engine, RpPioSwdTransactionFence::new())
    }

    /// Build on an exclusively owned, disabled PIO1 block.
    pub(crate) unsafe fn from_pio(pio: pac::pio::Pio, pins: PioSwdPins, sys_hz: u32) -> Self {
        let program = select_transaction_program(pins.oen.is_some());
        assert!(pins.valid_side_set_layout(program.side_set_bits));
        let map = PinMap::new(&pins, program.side_set_bits);

        regs::set_enabled(pio, SM, false);
        for (index, word) in program.instructions.iter().enumerate() {
            pio.instr_mem(index).write(|w| w.set_instr_mem(*word));
        }
        regs::gpio_base_zero(pio);

        let regs = pio.sm(SM);
        regs.execctrl().write(|w| {
            w.set_wrap_bottom(0);
            w.set_wrap_top(31);
            w.set_side_en(true);
            w.set_side_pindir(false);
        });
        regs.shiftctrl().write(|w| {
            w.set_in_shiftdir(true);
            w.set_out_shiftdir(true);
            w.set_autopush(true);
            w.set_autopull(false);
            // A threshold of 32 encodes as zero.
            w.set_push_thresh(0);
            w.set_pull_thresh(0);
            w.set_fjoin_rx(false);
            w.set_fjoin_tx(false);
        });
        regs.pinctrl().write(|w| {
            w.set_out_base(map.out_base);
            w.set_out_count(1);
            w.set_set_base(map.set_base);
            w.set_set_count(1);
            w.set_in_base(map.in_base);
            w.set_sideset_base(map.sideset_base);
            w.set_sideset_count(map.sideset_count);
        });

        // SWDIO changes on rising SWCLK; bypass keeps the edge sample in the current bit cell.
        let bypass = pio.input_sync_bypass().read();
        pio.input_sync_bypass()
            .write_value(bypass | (1 << map.in_base));

        let divider = swd_transaction_clock_divider(sys_hz, DEFAULT_SWCLK_HZ)
            .unwrap_or(PioClockDivider::integer(TRANSACTION_MIN_DIVIDER));
        regs::set_divider(pio, SM, divider);

        let mut io = RpPioSwdTransactionIo {
            pio,
            pins,
            sys_hz,
            active: false,
            in_flight: false,
        };
        io.flush_and_restart();
        io.stage_released_low();
        Self {
            engine: PioSwdTransaction::new(io),
        }
    }

    /// Recover the disabled PIO1 block for another program.
    pub(crate) fn into_pio(self) -> pac::pio::Pio {
        let io = self.engine.into_inner();
        assert!(!io.active, "cannot reload an active SWD state machine");
        io.pio
    }
}

impl RpPioSwdTransactionFence {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

impl CommandFence for RpPioSwdTransactionFence {
    fn finish_command(&self) {
        let pio = pac::PIO1;
        loop {
            let enabled = pio.ctrl().read().sm_enable() & sm_mask() != 0;
            let parked = pio.sm(SM).addr().read().addr() == 0;
            let tx_empty = pio.fstat().read().txempty() & sm_mask() != 0;
            if transaction_command_finished(enabled, parked, tx_empty) {
                break;
            }
        }
    }
}

fn transaction_command_finished(enabled: bool, parked: bool, tx_empty: bool) -> bool {
    !enabled || (parked && tx_empty)
}

impl RpPioSwdTransactionIo {
    fn flush_and_restart(&mut self) {
        regs::clear_fifos(self.pio, SM);
        regs::restart(self.pio, SM);
        self.in_flight = false;
    }

    fn stage_released_low(&mut self) {
        // Stage OEN before making it an output, then release SWDIO before the pads move here.
        if let Some(oen) = self.pins.oen {
            regs::exec_set_pin(self.pio, SM, oen, true);
        }
        regs::exec_set_pin(self.pio, SM, self.pins.swclk, false);
        regs::exec_set_pindir(self.pio, SM, self.pins.swdio_out, false);
        regs::exec_set_pindir(self.pio, SM, self.pins.swclk, true);
        if let Some(oen) = self.pins.oen {
            regs::exec_set_pindir(self.pio, SM, oen, true);
        }
        regs::exec(self.pio, SM, 0);
    }

    fn select_pads(&self, funcsel: u8) {
        // Data moves first, the staged-released OEN follows, and the staged-low clock moves last.
        if self.pins.readback() != self.pins.swdio_out {
            set_funcsel(self.pins.readback(), funcsel);
        }
        set_funcsel(self.pins.swdio_out, funcsel);
        if let Some(oen) = self.pins.oen {
            set_funcsel(oen, funcsel);
        }
        set_funcsel(self.pins.swclk, funcsel);
    }

    fn enter(&mut self) {
        debug_assert!(!self.active);
        regs::set_enabled(self.pio, SM, false);
        self.flush_and_restart();
        self.stage_released_low();
        self.select_pads(FUNCSEL_PIO1);
        regs::set_enabled(self.pio, SM, true);
        self.active = true;
    }

    fn exit(&mut self) {
        debug_assert!(self.active);
        self.finish();
        regs::set_enabled(self.pio, SM, false);
        self.stage_released_low();
        self.select_pads(FUNCSEL_PIO0);
        self.active = false;
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        if self.active {
            self.finish();
        }
        let divider = swd_transaction_clock_divider(self.sys_hz, max_frequency);
        regs::set_divider(self.pio, SM, divider.unwrap_or(SLOWEST_DIVIDER));
        divider.is_some()
    }

    fn finish(&mut self) {
        debug_assert!(self.active);
        if !self.in_flight {
            return;
        }
        loop {
            let parked = self.pio.sm(SM).addr().read().addr() == 0;
            let tx_empty = self.pio.fstat().read().txempty() & sm_mask() != 0;
            // `in_flight` separates completion from the initial park; TX-empty proves every queued word was consumed.
            if parked && tx_empty {
                break;
            }
        }
        self.in_flight = false;
    }
}

impl PioSwdTransactionIo for RpPioSwdTransactionIo {
    fn put(&mut self, word: u32) {
        debug_assert!(self.active);
        while self.pio.fstat().read().txfull() & sm_mask() != 0 {}
        self.pio.txf(SM).write_value(word);
        self.in_flight = true;
    }

    fn get(&mut self) -> u32 {
        debug_assert!(self.active);
        while self.pio.fstat().read().rxempty() & sm_mask() != 0 {}
        self.pio.rxf(SM).read()
    }
}

impl PioSwdTransferAccelerator for RpPioSwdTransaction {
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.engine.io_mut().set_clock(max_frequency)
    }

    fn enter(&mut self) {
        self.engine.io_mut().enter();
    }

    fn exit(&mut self) {
        self.engine.io_mut().exit();
    }

    fn read_transfer(&mut self, ap: bool, addr: u8) -> Result<u32, XferError> {
        self.engine.read_transfer(ap, addr)
    }

    fn write_transfer(&mut self, ap: bool, addr: u8, data: u32) -> Result<(), XferError> {
        self.engine.write_transfer(ap, addr, data)
    }
}

const fn sm_mask() -> u8 {
    1 << SM
}

#[cfg(test)]
mod tests {
    use super::transaction_command_finished;

    #[test]
    fn disabled_transaction_sm_never_blocks_jtag_or_inactive_modes() {
        for parked in [false, true] {
            for tx_empty in [false, true] {
                assert!(transaction_command_finished(false, parked, tx_empty));
            }
        }
    }

    #[test]
    fn active_transaction_sm_requires_park_and_empty_fifo() {
        assert!(!transaction_command_finished(true, false, false));
        assert!(!transaction_command_finished(true, false, true));
        assert!(!transaction_command_finished(true, true, false));
        assert!(transaction_command_finished(true, true, true));
    }
}
