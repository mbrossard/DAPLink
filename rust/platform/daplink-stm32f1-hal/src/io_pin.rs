//! Port-register GPIO line for the bit-bang SWJ driver.

use daplink_rs::InputOutputPin;
use stm32f1xx_hal::pac;

/// CNF+MODE nibbles (RM0008); for input with pull, ODR selects up/down.
const CFG_OUTPUT_PP_50M: u32 = 0b0011;
const CFG_OUTPUT_OD_50M: u32 = 0b0111;
const CFG_INPUT_PULL: u32 = 0b1000;

/// Which GPIO port a bit-bang pin lives on; [`IoPin::new`] clocks it, so no `split` is needed.
#[derive(Clone, Copy)]
pub enum Port {
    A,
    B,
    C,
}

/// A GPIO line driven through the port registers; a no-op when not connected.
#[derive(Clone, Copy)]
pub struct IoPin {
    port: Port,
    pin: u8,
    connected: bool,
    open_drain: bool,
}

impl IoPin {
    /// A connected bidirectional line on `port`/`pin`, enabling that port's GPIO clock.
    ///
    /// # Panics
    ///
    /// If `pin` is 16 or more.
    pub fn new(port: Port, pin: u8) -> Self {
        assert!(pin < 16, "F1 GPIO index must be below 16");
        // SAFETY: one RMW of RCC APB2ENR during single-threaded bring-up, per-port bits.
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.apb2enr().modify(|_, w| match port {
            Port::A => w.iopaen().set_bit(),
            Port::B => w.iopben().set_bit(),
            Port::C => w.iopcen().set_bit(),
        });
        Self {
            port,
            pin,
            connected: true,
            open_drain: false,
        }
    }

    /// A connected open-drain line (nRESET): low is driven, high only releases the net.
    pub fn open_drain(port: Port, pin: u8) -> Self {
        let mut line = Self {
            open_drain: true,
            ..Self::new(port, pin)
        };
        // Released and an output from here on: the SWJ wrapper never switches nRESET's mode.
        line.set_as_input();
        line
    }

    /// A no-connect pin: every operation is a no-op.
    pub fn nc() -> Self {
        Self {
            port: Port::A,
            pin: 0,
            connected: false,
            open_drain: false,
        }
    }

    /// Every F1 GPIO port shares one register layout.
    fn block(&self) -> *const pac::gpioa::RegisterBlock {
        match self.port {
            Port::A => pac::GPIOA::ptr(),
            Port::B => pac::GPIOB::ptr(),
            Port::C => pac::GPIOC::ptr(),
        }
    }

    fn set_mode(&self, mode: u32) {
        if !self.connected {
            return;
        }
        // SAFETY: the port was clocked by `new`; bring-up and the engine are strictly sequential.
        let r = unsafe { &*self.block() };
        if self.pin < 8 {
            let shift = self.pin as u32 * 4;
            r.crl()
                .modify(|rd, w| unsafe { w.bits((rd.bits() & !(0xf << shift)) | (mode << shift)) });
        } else {
            let shift = (self.pin as u32 - 8) * 4;
            r.crh()
                .modify(|rd, w| unsafe { w.bits((rd.bits() & !(0xf << shift)) | (mode << shift)) });
        }
    }
}

impl InputOutputPin for IoPin {
    #[inline]
    fn set_as_output(&mut self) {
        // Push-pull, or open-drain for wired-AND.
        self.set_mode(if self.open_drain {
            CFG_OUTPUT_OD_50M
        } else {
            CFG_OUTPUT_PP_50M
        });
    }
    #[inline(always)]
    fn set_high(&mut self, high: bool) {
        if !self.connected {
            return;
        }
        // SAFETY: BSRR is write-only and atomic per bit, so no RMW hazard exists.
        let r = unsafe { &*self.block() };
        let bit = if high { self.pin } else { self.pin + 16 };
        r.bsrr().write(|w| unsafe { w.bits(1 << bit) });
    }
    #[inline]
    fn set_as_input(&mut self) {
        if !self.connected {
            return;
        }
        // ODR = 1 before the input switch selects the pull-up, so SWDIO never reads pulled down.
        // SAFETY: see `set_high`.
        let r = unsafe { &*self.block() };
        r.bsrr().write(|w| unsafe { w.bits(1 << self.pin) });
        self.set_mode(if self.open_drain {
            CFG_OUTPUT_OD_50M
        } else {
            CFG_INPUT_PULL
        });
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        if !self.connected {
            return false;
        }
        // SAFETY: IDR is read-only.
        let r = unsafe { &*self.block() };
        r.idr().read().bits() & (1 << self.pin) != 0
    }
}
