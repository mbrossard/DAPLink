//! PAC-register GPIO line for the bit-bang SWJ driver; handles are `Copy`.

use daplink_rs::InputOutputPin;
use embassy_stm32::gpio::Pin;
use embassy_stm32::pac::gpio::{regs, Gpio};
use embassy_stm32::Peri;

/// GPIO ports sit `0x400` apart on every STM32.
const PORT_STRIDE: usize = 0x400;

/// Register access for one line: the port block address plus the pin index.
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

    /// Drive the line (or, open-drain, release it) through the atomic set/reset register.
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

    /// Rewrite this pin's two MODER bits; the other config registers are set once at wiring.
    fn set_mode(&self, mode: u32) {
        let shift = 2 * self.pin;
        let moder = self.block().moder();
        let v = (moder.read().0 & !(0b11 << shift)) | (mode << shift);
        moder.write_value(regs::Moder(v));
    }
}

/// MODER field values: general-purpose input / output.
const MODE_INPUT: u32 = 0b00;
const MODE_OUTPUT: u32 = 0b01;

/// A bit-bang SWJ line: `Pin` is bidirectional push-pull, `OpenDrain` drives a wired-AND net and
/// stays an output permanently (the data register alone releases it), `Nc` is a no-op placeholder.
#[derive(Clone, Copy)]
pub enum IoPin {
    /// A single bidirectional push-pull line.
    Pin(Line),
    /// A bidirectional open-drain line: low is driven, high releases (wired-AND).
    OpenDrain(Line),
    /// A no-connect pin: every operation is a no-op.
    Nc,
}

impl IoPin {
    /// Wire a bidirectional push-pull line: very-high speed, pulled up, idle high, input for now.
    pub fn new(p: Peri<'static, impl Pin>) -> Self {
        let line = Line::new(p.port(), p.pin());
        let block = line.block();
        let shift = 2 * line.pin;
        // Speed and pull are set once here; only MODER flips at runtime.
        block.ospeedr().modify(|w| w.0 |= 0b11 << shift);
        block
            .pupdr()
            .modify(|w| w.0 = (w.0 & !(0b11 << shift)) | (0b01 << shift));
        line.set_high(true);
        line.set_mode(MODE_INPUT);
        IoPin::Pin(line)
    }

    /// Wire a bidirectional open-drain line (typically nRESET), released, permanently an output.
    pub fn new_open_drain(p: Peri<'static, impl Pin>) -> Self {
        let line = Line::new(p.port(), p.pin());
        line.block().otyper().modify(|w| w.0 |= 1 << line.pin);
        // Release before entering output mode, so a stale low can never assert the net.
        line.set_high(true);
        line.set_mode(MODE_OUTPUT);
        IoPin::OpenDrain(line)
    }
}

impl InputOutputPin for IoPin {
    fn set_as_output(&mut self) {
        match self {
            IoPin::Pin(l) => l.set_mode(MODE_OUTPUT),
            // Already (and permanently) an open-drain output.
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
            // "Input" for the open-drain line means released; the net's pull-up raises it.
            IoPin::OpenDrain(l) => l.set_high(true),
            IoPin::Nc => {}
        }
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        match self {
            // IDR reads the real net level even in output mode.
            IoPin::Pin(l) | IoPin::OpenDrain(l) => l.is_high(),
            IoPin::Nc => false,
        }
    }
}
