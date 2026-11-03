//! PAC-register GPIO line for the bit-bang SWJ driver.

use daplink_rs::InputOutputPin;
use embassy_stm32::gpio::Pin;
use embassy_stm32::pac::gpio::{regs, Gpio};
use embassy_stm32::Peri;

/// GPIO ports sit `0x400` apart on every STM32.
const PORT_STRIDE: usize = 0x400;

#[derive(Clone, Copy)]
pub struct Line {
    block: usize,
    pin: usize,
}

impl Line {
    fn new(port: u8, pin: u8) -> Self {
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
    fn set_high(&self, high: bool) {
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

    /// Rewrite this pin's MODER bits; speed and pull are set once at wiring.
    fn set_mode(&self, mode: u32) {
        let shift = 2 * self.pin;
        let moder = self.block().moder();
        let v = (moder.read().0 & !(0b11 << shift)) | (mode << shift);
        moder.write_value(regs::Moder(v));
    }
}

const MODE_INPUT: u32 = 0b00;
const MODE_OUTPUT: u32 = 0b01;

/// A bit-bang SWJ line; `OpenDrain` stays an output and releases through ODR alone.
#[derive(Clone, Copy)]
pub enum IoPin {
    Pin(Line),
    /// An open-drain line: low is driven, high releases.
    OpenDrain(Line),
    Nc,
}

impl IoPin {
    /// A push-pull line at very-high speed, pulled up, starting as an input.
    pub fn new(p: Peri<'static, impl Pin>) -> Self {
        Self::with_speed(p, 0b11)
    }

    /// [`new`](Self::new) at a caller-chosen `OSPEEDR` class.
    pub fn with_speed(p: Peri<'static, impl Pin>, speed: u32) -> Self {
        let line = Line::new(p.port(), p.pin());
        let block = line.block();
        let shift = 2 * line.pin;
        block
            .ospeedr()
            .modify(|w| w.0 = (w.0 & !(0b11 << shift)) | ((speed & 0b11) << shift));
        block
            .pupdr()
            .modify(|w| w.0 = (w.0 & !(0b11 << shift)) | (0b01 << shift));
        line.set_high(true);
        line.set_mode(MODE_INPUT);
        IoPin::Pin(line)
    }

    /// An open-drain line (typically nRESET), released and permanently an output.
    pub fn new_open_drain(p: Peri<'static, impl Pin>) -> Self {
        let line = Line::new(p.port(), p.pin());
        line.block().otyper().modify(|w| w.0 |= 1 << line.pin);
        // Release before entering output mode, so a stale low never asserts the net.
        line.set_high(true);
        line.set_mode(MODE_OUTPUT);
        IoPin::OpenDrain(line)
    }
}

impl InputOutputPin for IoPin {
    fn set_as_output(&mut self) {
        match self {
            IoPin::Pin(l) => l.set_mode(MODE_OUTPUT),
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
            IoPin::Pin(l) => l.set_mode(MODE_INPUT),
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

/// Park `pins` of `port` as unpulled inputs, so a pad sharing a net drives nothing.
pub fn park_inputs(port: Gpio, pins: u16) {
    for pin in (0..16).filter(|n| pins & (1 << n) != 0) {
        let shift = 2 * pin;
        port.pupdr().modify(|w| w.0 &= !(0b11 << shift));
        port.moder().modify(|w| w.0 &= !(0b11 << shift));
    }
}
