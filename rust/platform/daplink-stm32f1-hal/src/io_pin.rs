//! Port-register GPIO line for the bit-bang SWJ driver on the F1's CRL/CRH-style ports.

use daplink_rs::InputOutputPin;
use stm32f1xx_hal::pac;

/// Which GPIO port a bit-bang pin lives on; [`IoPin::new`] clocks it, so no `split` is needed.
#[derive(Clone, Copy)]
pub enum Port {
    /// GPIO port A.
    A,
    /// GPIO port B.
    B,
    /// GPIO port C.
    C,
}

/// A GPIO line driven through the port registers (CRL/CRH for direction, BSRR for level);
/// `Copy`, holding an identity rather than ownership, and a no-op when `connected` is false.
#[derive(Clone, Copy)]
pub struct IoPin {
    port: Port,
    pin: u8,
    connected: bool,
    open_drain: bool,
}

impl IoPin {
    /// A connected bidirectional line on `port`/`pin`, enabling that port's GPIO clock.
    pub fn new(port: Port, pin: u8) -> Self {
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

    /// A connected **open-drain** line (nRESET): low is driven, high only releases the net. Such a
    /// line stays in output mode from its first direction call, so a bare BSRR write reaches it.
    pub fn open_drain(port: Port, pin: u8) -> Self {
        Self {
            open_drain: true,
            ..Self::new(port, pin)
        }
    }

    /// A no-connect pin (unused signal): every operation is a no-op.
    pub fn nc() -> Self {
        Self {
            port: Port::A,
            pin: 0,
            connected: false,
            open_drain: false,
        }
    }

    /// The register block this pin's port lives in; every F1 GPIO port shares one layout.
    fn block(&self) -> *const pac::gpioa::RegisterBlock {
        match self.port {
            Port::A => pac::GPIOA::ptr(),
            Port::B => pac::GPIOB::ptr(),
            Port::C => pac::GPIOC::ptr(),
        }
    }

    /// Write the 4-bit CNF+MODE field for this pin in CRL (pins 0..7) or CRH (8..15).
    fn set_mode(&self, mode: u32) {
        if !self.connected {
            return;
        }
        // SAFETY: a port `new` clocked, whose config bits nothing else rewrites concurrently
        // (bring-up and the engine are strictly sequential).
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
        // MODE 0b11 (output, 50 MHz) with CNF 0b00 push-pull, or 0b01 open-drain for wired-AND.
        self.set_mode(if self.open_drain { 0b0111 } else { 0b0011 });
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
        // MODE 0b00 (input) with CNF 0b10 (pull up/down); ODR = 1 first selects the pull-up, so
        // SWDIO idles high and never briefly reads as pull-down. SAFETY: see `set_high`.
        let r = unsafe { &*self.block() };
        r.bsrr().write(|w| unsafe { w.bits(1 << self.pin) });
        // An open-drain line is released by the ODR = 1 above and stays an output.
        self.set_mode(if self.open_drain { 0b0111 } else { 0b1000 });
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        if !self.connected {
            return false;
        }
        // SAFETY: IDR is read-only and reads the real net level even in output mode.
        let r = unsafe { &*self.block() };
        r.idr().read().bits() & (1 << self.pin) != 0
    }
}
