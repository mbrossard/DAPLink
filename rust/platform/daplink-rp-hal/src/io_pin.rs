//! SIO-register GPIO line used to bit-bang SWD.

use daplink_rs::InputOutputPin;

/// A GPIO line driven by SIO register mask; a zero mask is a no-connect, `open_drain` marks nRESET.
/// Stateless, so a second handle to the same pad is safe to hold; the masks differ only when split.
#[derive(Clone, Copy)]
pub struct IoPin {
    out_mask: u32,
    in_mask: u32,
    open_drain: bool,
}

const RP2040_GPIO_COUNT: u8 = 30;

impl IoPin {
    /// A single bidirectional line; the caller must already have put the pad into SIO mode.
    pub fn new(pin: u8) -> Self {
        debug_assert!(
            pin < RP2040_GPIO_COUNT,
            "pin out of range for RP2040 (GPIO0..=GPIO29)"
        );
        Self {
            out_mask: 1 << pin,
            in_mask: 1 << pin,
            open_drain: false,
        }
    }

    /// A level-shifted line: drive/direction on `out_pin`, readback on `in_pin`.
    pub fn split(out_pin: u8, in_pin: u8) -> Self {
        debug_assert!(
            out_pin < RP2040_GPIO_COUNT && in_pin < RP2040_GPIO_COUNT,
            "pin out of range for RP2040 (GPIO0..=GPIO29)"
        );
        Self {
            out_mask: 1 << out_pin,
            in_mask: 1 << in_pin,
            open_drain: false,
        }
    }

    /// An open-drain line (nRESET): asserted low by driving, released by going high impedance.
    pub fn open_drain(pin: u8) -> Self {
        Self {
            open_drain: true,
            ..Self::new(pin)
        }
    }

    /// A no-connect pin (unused signal): every operation is a no-op.
    pub fn nc() -> Self {
        Self {
            out_mask: 0,
            in_mask: 0,
            open_drain: false,
        }
    }
}

impl InputOutputPin for IoPin {
    #[inline]
    fn set_as_output(&mut self) {
        let sio = rp2040_hal::pac::SIO::ptr();
        unsafe { (*sio).gpio_oe_set().write(|w| w.bits(self.out_mask)) };
    }
    #[inline(always)]
    fn set_high(&mut self, high: bool) {
        let sio = rp2040_hal::pac::SIO::ptr();
        if self.open_drain {
            if high {
                self.set_as_input();
            } else {
                unsafe { (*sio).gpio_out_clr().write(|w| w.bits(self.out_mask)) };
                self.set_as_output();
            }
        } else if high {
            unsafe { (*sio).gpio_out_set().write(|w| w.bits(self.out_mask)) };
        } else {
            unsafe { (*sio).gpio_out_clr().write(|w| w.bits(self.out_mask)) };
        }
    }
    #[inline]
    fn set_as_input(&mut self) {
        let sio = rp2040_hal::pac::SIO::ptr();
        unsafe { (*sio).gpio_oe_clr().write(|w| w.bits(self.out_mask)) };
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        let sio = rp2040_hal::pac::SIO::ptr();
        unsafe { (*sio).gpio_in().read().bits() & self.in_mask != 0 }
    }
}
