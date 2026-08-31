//! SIO-register GPIO line for bit-banging SWD.

use daplink_rs::InputOutputPin;
use embassy_rp::gpio::{Flex, Pin, Pull};
use embassy_rp::pac::SIO;
use embassy_rp::Peri;

/// A GPIO line driven by SIO register mask; a zero mask is a no-connect, `open_drain` marks nRESET.
/// Stateless, so a second handle to the same pad is safe to hold.
#[derive(Clone, Copy)]
pub struct IoPin {
    mask: u32,
    open_drain: bool,
}

impl IoPin {
    /// A single bidirectional line.
    pub fn new(pin: u8) -> Self {
        Self {
            mask: 1 << pin,
            open_drain: false,
        }
    }

    /// A no-connect line: every operation is a no-op.
    pub fn nc() -> Self {
        Self {
            mask: 0,
            open_drain: false,
        }
    }

    /// An open-drain line: asserts by pulling low, releases by floating.
    pub fn open_drain(pin: u8) -> Self {
        Self {
            mask: 1 << pin,
            open_drain: true,
        }
    }
}

impl InputOutputPin for IoPin {
    #[inline(always)]
    fn set_as_output(&mut self) {
        SIO.gpio_oe(0).value_set().write_value(self.mask);
    }
    #[inline(always)]
    fn set_high(&mut self, high: bool) {
        if self.open_drain {
            if high {
                self.set_as_input();
            } else {
                SIO.gpio_out(0).value_clr().write_value(self.mask);
                self.set_as_output();
            }
        } else if high {
            SIO.gpio_out(0).value_set().write_value(self.mask);
        } else {
            SIO.gpio_out(0).value_clr().write_value(self.mask);
        }
    }
    #[inline(always)]
    fn set_as_input(&mut self) {
        SIO.gpio_oe(0).value_clr().write_value(self.mask);
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        SIO.gpio_in(0).read() & self.mask != 0
    }
}

/// A line whose drive and readback are separate pads; a zero mask is a no-connect. Direction and
/// level go to the driven pad, so the SWD read turnaround tristates that pad alone.
#[derive(Clone, Copy)]
pub struct SplitIoPin {
    out_mask: u32,
    in_mask: u32,
}

impl SplitIoPin {
    /// A line driven and read on the same pad.
    pub fn new(pin: u8) -> Self {
        Self {
            out_mask: 1 << pin,
            in_mask: 1 << pin,
        }
    }

    /// A line driven on `out_pin` and always read on `in_pin`.
    pub fn split(out_pin: u8, in_pin: u8) -> Self {
        Self {
            out_mask: 1 << out_pin,
            in_mask: 1 << in_pin,
        }
    }

    /// A no-connect line: writes are no-ops and reads are low.
    pub fn nc() -> Self {
        Self {
            out_mask: 0,
            in_mask: 0,
        }
    }
}

impl InputOutputPin for SplitIoPin {
    #[inline(always)]
    fn set_as_output(&mut self) {
        SIO.gpio_oe(0).value_set().write_value(self.out_mask);
    }
    #[inline(always)]
    fn set_high(&mut self, high: bool) {
        if high {
            SIO.gpio_out(0).value_set().write_value(self.out_mask);
        } else {
            SIO.gpio_out(0).value_clr().write_value(self.out_mask);
        }
    }
    #[inline(always)]
    fn set_as_input(&mut self) {
        SIO.gpio_oe(0).value_clr().write_value(self.out_mask);
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        SIO.gpio_in(0).read() & self.in_mask != 0
    }
}

/// Put `pin` into SIO mode as an input and hand back the register-level [`IoPin`]; leaks the `Flex`.
pub fn sio_pin<P: Pin>(pin: Peri<'static, P>, pull: Pull) -> IoPin {
    let index = pin.pin();
    let mut flex = Flex::new(pin);
    flex.set_pull(pull);
    flex.set_as_input();
    core::mem::forget(flex);
    IoPin::new(index)
}
