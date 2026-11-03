//! One PIO1 owner shared by the complete-transfer SWD and JTAG programs.

use core::mem;
use core::sync::atomic::{AtomicU8, Ordering};

use daplink_rs::CommandFence;

use crate::clock::{jtag_clock_divider, swd_transaction_clock_divider};
use crate::composition::PioJtagAccelerator;
use crate::pins::{PioJtagPins, PioSwdPins};
use crate::regs::GPIO_COUNT;
use crate::sequence::XferError;
use crate::swd_transaction_hybrid::PioSwdTransferAccelerator;
use crate::transaction_sm::RpPioSwdTransactionFence;
use crate::{RpAutonomousJtag, RpPioSwdTransaction};

const MODE_NONE: u8 = 0;
const MODE_SWD: u8 = 1;
const MODE_JTAG: u8 = 2;

// Exclusive PIO1 ownership also protects this mode token shared with the command fence.
static PIO1_MODE: AtomicU8 = AtomicU8::new(MODE_NONE);

enum LoadedEngine {
    Invalid,
    Swd(RpPioSwdTransaction),
    Jtag(RpAutonomousJtag),
}

/// One PIO1 transfer engine that reloads its program when the active wire protocol changes.
pub struct RpPioTransferEngine {
    loaded: LoadedEngine,
    swd_pins: PioSwdPins,
    jtag_pins: PioJtagPins,
    sys_hz: u32,
    max_frequency: u32,
    active: bool,
}

/// Waits for SWD suffixes; its dispatcher predicate must not be used for a loaded JTAG program.
pub struct RpPioTransferFence {
    _private: (),
}

impl RpPioTransferEngine {
    /// Configure PIO1 with SWD initially loaded while leaving all pads with PIO0.
    ///
    /// # Safety
    /// Own PIO1, its IRQ flags 0 and 1 and the PIO1 mode state; PIO0 alone shares the pads, parked before entry.
    pub unsafe fn new(
        swd_pins: PioSwdPins,
        jtag_pins: PioJtagPins,
        sys_hz: u32,
    ) -> (Self, RpPioTransferFence) {
        assert!(jtag_pins.valid_with_gpio_count(&swd_pins, GPIO_COUNT));
        assert!(swd_transaction_clock_divider(sys_hz, 1_000_000).is_some());
        assert!(jtag_clock_divider(sys_hz, 1_000_000).is_some());
        let (swd, _) = RpPioSwdTransaction::new(swd_pins, sys_hz);
        PIO1_MODE.store(MODE_SWD, Ordering::Relaxed);
        (
            Self {
                loaded: LoadedEngine::Swd(swd),
                swd_pins,
                jtag_pins,
                sys_hz,
                max_frequency: 1_000_000,
                active: false,
            },
            RpPioTransferFence { _private: () },
        )
    }

    fn supports_clock(&self, max_frequency: u32) -> bool {
        swd_transaction_clock_divider(self.sys_hz, max_frequency).is_some()
            && jtag_clock_divider(self.sys_hz, max_frequency).is_some()
    }

    fn set_transfer_clock(&mut self, max_frequency: u32) -> bool {
        let accepted = match &mut self.loaded {
            LoadedEngine::Swd(engine) => {
                PioSwdTransferAccelerator::set_clock(engine, max_frequency)
            }
            LoadedEngine::Jtag(engine) => PioJtagAccelerator::set_clock(engine, max_frequency),
            LoadedEngine::Invalid => unreachable!(),
        };
        // Saved even when clamped, so a reload reapplies the same ceiling.
        self.max_frequency = max_frequency;
        accepted && self.supports_clock(max_frequency)
    }

    fn ensure_swd_loaded(&mut self) {
        // Exit drains wire work before reload; restore the saved ceiling before publishing mode.
        if matches!(self.loaded, LoadedEngine::Swd(_)) {
            return;
        }
        assert!(!self.active, "cannot reload an active PIO1 engine");
        PIO1_MODE.store(MODE_NONE, Ordering::Relaxed);
        let old = mem::replace(&mut self.loaded, LoadedEngine::Invalid);
        let LoadedEngine::Jtag(jtag) = old else {
            unreachable!();
        };
        let pio = jtag.into_pio();
        let mut swd = unsafe { RpPioSwdTransaction::from_pio(pio, self.swd_pins, self.sys_hz) };
        PioSwdTransferAccelerator::set_clock(&mut swd, self.max_frequency);
        self.loaded = LoadedEngine::Swd(swd);
        PIO1_MODE.store(MODE_SWD, Ordering::Relaxed);
    }

    fn ensure_jtag_loaded(&mut self) {
        // Reload only after SWD has returned its pads, before the next JTAG entry.
        if matches!(self.loaded, LoadedEngine::Jtag(_)) {
            return;
        }
        assert!(!self.active, "cannot reload an active PIO1 engine");
        PIO1_MODE.store(MODE_NONE, Ordering::Relaxed);
        let old = mem::replace(&mut self.loaded, LoadedEngine::Invalid);
        let LoadedEngine::Swd(swd) = old else {
            unreachable!();
        };
        let pio = swd.into_pio();
        let mut jtag = unsafe { RpAutonomousJtag::from_pio(pio, self.jtag_pins, self.sys_hz) };
        PioJtagAccelerator::set_clock(&mut jtag, self.max_frequency);
        self.loaded = LoadedEngine::Jtag(jtag);
        PIO1_MODE.store(MODE_JTAG, Ordering::Relaxed);
    }
}

impl Drop for RpPioTransferEngine {
    fn drop(&mut self) {
        PIO1_MODE.store(MODE_NONE, Ordering::Relaxed);
    }
}

impl PioSwdTransferAccelerator for RpPioTransferEngine {
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.set_transfer_clock(max_frequency)
    }

    fn enter(&mut self) {
        assert!(!self.active, "PIO1 transfer engine is already active");
        self.ensure_swd_loaded();
        let LoadedEngine::Swd(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.enter();
        self.active = true;
    }

    fn exit(&mut self) {
        assert!(self.active, "PIO1 transfer engine is not active");
        let LoadedEngine::Swd(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.exit();
        self.active = false;
    }

    fn read_transfer(&mut self, ap: bool, addr: u8) -> Result<u32, XferError> {
        assert!(self.active, "PIO1 transfer engine is not active");
        let LoadedEngine::Swd(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.read_transfer(ap, addr)
    }

    fn write_transfer(&mut self, ap: bool, addr: u8, data: u32) -> Result<(), XferError> {
        assert!(self.active, "PIO1 transfer engine is not active");
        let LoadedEngine::Swd(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.write_transfer(ap, addr, data)
    }
}

impl PioJtagAccelerator for RpPioTransferEngine {
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.set_transfer_clock(max_frequency)
    }

    fn enter(&mut self, bypass_before: u8, bypass_after: u8) {
        assert!(!self.active, "PIO1 transfer engine is already active");
        self.ensure_jtag_loaded();
        let LoadedEngine::Jtag(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.enter(bypass_before, bypass_after);
        self.active = true;
    }

    fn exit(&mut self) {
        assert!(self.active, "PIO1 transfer engine is not active");
        let LoadedEngine::Jtag(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.exit();
        self.active = false;
    }

    fn transfer(&mut self, request: u8, data: u32) -> (u32, u32) {
        assert!(self.active, "PIO1 transfer engine is not active");
        let LoadedEngine::Jtag(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.transfer(request, data)
    }

    fn shift_ir(&mut self, ir: u32) -> bool {
        assert!(self.active, "PIO1 transfer engine is not active");
        let LoadedEngine::Jtag(engine) = &mut self.loaded else {
            unreachable!();
        };
        engine.shift_ir(ir)
    }
}

impl CommandFence for RpPioTransferFence {
    fn finish_command(&self) {
        if PIO1_MODE.load(Ordering::Relaxed) == MODE_SWD {
            RpPioSwdTransactionFence::new().finish_command();
        }
    }
}
