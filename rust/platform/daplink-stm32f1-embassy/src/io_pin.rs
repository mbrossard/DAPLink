//! PAC-register GPIO line for the bit-bang SWJ driver.

use daplink_rs::InputOutputPin;
use embassy_stm32::gpio::Pin;
use embassy_stm32::pac::gpio::{regs, Gpio};
use embassy_stm32::Peri;

/// GPIO ports sit `0x400` apart on every STM32.
const PORT_STRIDE: usize = 0x400;

/// CNF+MODE nibbles (RM0008); for input with pull, ODR selects up/down.
const CFG_OUTPUT_PP_50M: u32 = 0b0011;
pub(crate) const CFG_INPUT_PULL: u32 = 0b1000;
const CFG_OUTPUT_OD_2M: u32 = 0b0110;

#[derive(Clone, Copy)]
pub struct Line {
    block: usize,
    pin: usize,
}

impl Line {
    pub(crate) fn new(port: u8, pin: u8) -> Self {
        Self {
            block: embassy_stm32::pac::GPIOA.as_ptr() as usize + PORT_STRIDE * port as usize,
            pin: pin as usize,
        }
    }

    #[inline(always)]
    fn block(&self) -> Gpio {
        // SAFETY: the PAC's own GPIOA block plus the fixed stride, for a port the caller owns.
        unsafe { Gpio::from_ptr(self.block as *mut ()) }
    }

    /// Drive the line, or release an open-drain one, through BSRR.
    #[inline(always)]
    pub(crate) fn set_high(&self, high: bool) {
        let bit = if high {
            1 << self.pin
        } else {
            1 << (self.pin + 16)
        };
        self.block().bsrr().write_value(regs::Bsrr(bit));
    }

    #[inline(always)]
    fn is_high(&self) -> bool {
        self.block().idr().read().0 & (1 << self.pin) != 0
    }

    pub(crate) fn set_cfg(&self, cfg: u32) {
        let shift = 4 * (self.pin % 8);
        let cr = self.block().cr(self.pin / 8);
        let v = (cr.read().0 & !(0b1111 << shift)) | (cfg << shift);
        cr.write_value(regs::Cr(v));
    }
}

/// A bit-bang SWJ line; `OpenDrain` stays an output and releases through ODR alone.
#[derive(Clone, Copy)]
pub enum IoPin {
    Pin(Line),
    /// An open-drain line: low is driven, high releases.
    OpenDrain(Line),
    Nc,
}

impl IoPin {
    /// A bidirectional push-pull line, starting as a pulled-up input.
    pub fn new(p: Peri<'static, impl Pin>) -> Self {
        let line = Line::new(p.port(), p.pin());
        // The pull follows ODR in input mode, so raise it before the mode change.
        line.set_high(true);
        line.set_cfg(CFG_INPUT_PULL);
        IoPin::Pin(line)
    }

    /// An open-drain line (typically nRESET), released and permanently an output.
    pub fn new_open_drain(p: Peri<'static, impl Pin>) -> Self {
        let line = Line::new(p.port(), p.pin());
        // Release before entering output mode, so a stale low never asserts the net.
        line.set_high(true);
        line.set_cfg(CFG_OUTPUT_OD_2M);
        IoPin::OpenDrain(line)
    }
}

impl InputOutputPin for IoPin {
    fn set_as_output(&mut self) {
        match self {
            IoPin::Pin(l) => l.set_cfg(CFG_OUTPUT_PP_50M),
            IoPin::OpenDrain(_) | IoPin::Nc => {}
        }
    }
    #[inline(always)]
    fn set_high(&mut self, high: bool) {
        if let IoPin::Pin(l) | IoPin::OpenDrain(l) = self {
            l.set_high(high);
        }
    }
    fn set_as_input(&mut self) {
        match self {
            // ODR selects the pull in input mode.
            IoPin::Pin(l) => {
                l.set_high(true);
                l.set_cfg(CFG_INPUT_PULL);
            }
            // Released; the net's pull-up raises it.
            IoPin::OpenDrain(l) => l.set_high(true),
            IoPin::Nc => {}
        }
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        match self {
            // IDR reads the net level even in output mode.
            IoPin::Pin(l) | IoPin::OpenDrain(l) => l.is_high(),
            IoPin::Nc => false,
        }
    }
}
