//! GPIO bit-bang lines driven directly through the LPC55 GPIO registers.

use core::mem::offset_of;

use daplink_rs::InputOutputPin;
use lpc55_hal::raw;

// The port-indexed GPIO registers sit at fixed byte deltas from `set[port]`.
const D_CLR: isize = offset_of!(raw::gpio::RegisterBlock, clr) as isize
    - offset_of!(raw::gpio::RegisterBlock, set) as isize;
const D_PIN: isize = offset_of!(raw::gpio::RegisterBlock, pin) as isize
    - offset_of!(raw::gpio::RegisterBlock, set) as isize;
const D_DIRSET: isize = offset_of!(raw::gpio::RegisterBlock, dirset) as isize
    - offset_of!(raw::gpio::RegisterBlock, set) as isize;
const D_DIRCLR: isize = offset_of!(raw::gpio::RegisterBlock, dirclr) as isize
    - offset_of!(raw::gpio::RegisterBlock, set) as isize;

/// A GPIO line addressed by resolved register address and bit mask; the pad must already be in GPIO
/// mode. A zero `mask` is a no-connect; `txen_mask` names an external buffer's enable pad.
#[derive(Clone, Copy)]
pub struct IoPin {
    set: usize,
    txen: usize,
    mask: u32,
    txen_mask: u32,
    open_drain: bool,
}

impl IoPin {
    /// Resolve the `set[port]` register address once, at construction.
    fn set_addr(port: usize) -> usize {
        // SAFETY: the GPIO block is a fixed peripheral; `addr_of!` forms no reference.
        let gpio = raw::GPIO::ptr();
        (unsafe { core::ptr::addr_of!((*gpio).set[port]) }) as usize
    }

    /// A plain push-pull output (SWCLK); the pad must already be a GPIO output.
    pub fn output(port: usize, pin: u8) -> Self {
        IoPin {
            set: Self::set_addr(port),
            txen: 0,
            mask: 1 << pin,
            txen_mask: 0,
            open_drain: false,
        }
    }

    /// A buffered, direction-controlled line (SWDIO): a data pad plus the buffer's TXEN pad.
    pub fn buffered(data_port: usize, data_pin: u8, txen_port: usize, txen_pin: u8) -> Self {
        IoPin {
            set: Self::set_addr(data_port),
            txen: Self::set_addr(txen_port),
            mask: 1 << data_pin,
            txen_mask: 1 << txen_pin,
            open_drain: false,
        }
    }

    /// A buffered open-drain line (nRESET): asserted by enabling the buffer, released by
    /// tri-stating it so the target's own pull-up raises the wire.
    pub fn open_drain(data_port: usize, data_pin: u8, txen_port: usize, txen_pin: u8) -> Self {
        IoPin {
            open_drain: true,
            ..Self::buffered(data_port, data_pin, txen_port, txen_pin)
        }
    }

    /// A plain read-only input (JTAG TDO).
    pub fn input(port: usize, pin: u8) -> Self {
        IoPin {
            set: Self::set_addr(port),
            txen: 0,
            mask: 1 << pin,
            txen_mask: 0,
            open_drain: false,
        }
    }

    /// A no-connect line: every operation is a harmless no-op.
    pub fn nc() -> Self {
        IoPin {
            set: Self::set_addr(0),
            txen: 0,
            mask: 0,
            txen_mask: 0,
            open_drain: false,
        }
    }
}

/// The board's nRESET line, defined once so wiring and drive policy cannot drift apart.
pub(crate) fn nreset() -> IoPin {
    IoPin::open_drain(0, 19, 0, 13)
}

impl InputOutputPin for IoPin {
    #[inline(always)]
    fn set_high(&mut self, high: bool) {
        // Open-drain: the level write is the whole turnaround, as the adapter writes only levels.
        if self.open_drain {
            if high {
                self.set_as_input();
            } else {
                // Latch the low level before enabling the buffer, so it cannot glitch high.
                // SAFETY: resolved GPIO register; write-1-to-act bit.
                unsafe {
                    (self.set.wrapping_add_signed(D_CLR) as *mut u32).write_volatile(self.mask)
                };
                self.drive();
            }
            return;
        }
        let addr = if high {
            self.set
        } else {
            self.set.wrapping_add_signed(D_CLR)
        };
        // SAFETY: `set` resolved at construction and `D_CLR` stays in the block; no shared RMW.
        unsafe { (addr as *mut u32).write_volatile(self.mask) };
    }

    #[inline(always)]
    fn is_high(&mut self) -> bool {
        // SAFETY: as above; PIN is a plain read.
        let level = unsafe { (self.set.wrapping_add_signed(D_PIN) as *const u32).read_volatile() };
        (level & self.mask) != 0
    }

    #[inline]
    fn set_as_output(&mut self) {
        // Open-drain: drive lives in `set_high`, so parking as an output cannot hold the buffer on.
        if self.open_drain {
            return;
        }
        self.drive();
    }

    #[inline]
    fn set_as_input(&mut self) {
        // The reverse of `drive`: pad to input, then tri-state; an unbuffered line drives high.
        if self.txen_mask != 0 {
            // SAFETY: resolved GPIO registers; dedicated write-1-to-act bits.
            unsafe {
                (self.set.wrapping_add_signed(D_DIRCLR) as *mut u32).write_volatile(self.mask);
                (self.txen.wrapping_add_signed(D_CLR) as *mut u32).write_volatile(self.txen_mask);
            }
        } else {
            // SAFETY: as above.
            unsafe { (self.set as *mut u32).write_volatile(self.mask) };
        }
    }
}

impl IoPin {
    #[inline]
    fn drive(&mut self) {
        // Buffer on before the pad drives, so probe and target never both drive the shared wire.
        if self.txen_mask != 0 {
            // SAFETY: resolved GPIO registers; dedicated write-1-to-act bits.
            unsafe {
                (self.txen as *mut u32).write_volatile(self.txen_mask);
                (self.set.wrapping_add_signed(D_DIRSET) as *mut u32).write_volatile(self.mask);
            }
        }
    }
}
