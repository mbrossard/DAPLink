//! STM32F103 clock configuration and the two board flavors of USB re-enumeration.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::{peripherals, Config, Peri};

/// Core clock produced by [`clocks_72m_hse8m`]; the cycle-counting delay counts against it.
pub const CPU_HZ: u32 = 72_000_000;

/// 72 MHz off an 8 MHz HSE crystal (PLL x9); APB1 /2 at its 36 MHz cap, APB2 undivided. The
/// USB 48 MHz comes off the PLL's fixed /1.5 tap, so 72 MHz is the only legal sysclk here.
pub fn clocks_72m_hse8m() -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    config
}

/// A switched D+ pull-up: the board is absent from the bus until [`attach`](UsbConnect::attach).
pub struct UsbConnect<'d> {
    pin: Output<'d>,
    /// True when pulling the pin low is what connects the pull-up (a PNP switch).
    active_low: bool,
}

impl UsbConnect<'_> {
    /// Present the D+ pull-up; call only once the USB device can answer enumeration.
    pub fn attach(&mut self) {
        if self.active_low {
            self.pin.set_low();
        } else {
            self.pin.set_high();
        }
    }

    /// Remove the D+ pull-up, so the host sees a detach.
    pub fn detach(&mut self) {
        if self.active_low {
            self.pin.set_high();
        } else {
            self.pin.set_low();
        }
    }
}

/// The F1 device controller has no internal D+ pull-up, so this net is the only re-enumeration.
impl daplink_rs::daplink_core::UsbReconnect for UsbConnect<'_> {
    fn detach(&mut self) {
        UsbConnect::detach(self);
    }

    fn attach(&mut self) {
        UsbConnect::attach(self);
    }
}

/// USB connect control for the nanoDAP V2.3 wiring: PB15, asserted low. Returned detached.
pub fn usb_connect_pb15(pin: Peri<'_, peripherals::PB15>) -> UsbConnect<'_> {
    UsbConnect {
        // Detached = high for an active-low control.
        pin: Output::new(pin, Level::High, Speed::VeryHigh),
        active_low: true,
    }
}

/// USB connect control on PA15, driven high to connect. Returned detached; the caller must free
/// PA15 from its reset-state JTAG role (`Config::swj`) before `init`.
pub fn usb_connect_pa15(pin: Peri<'_, peripherals::PA15>) -> UsbConnect<'_> {
    UsbConnect {
        // Detached = low for an active-high control.
        pin: Output::new(pin, Level::Low, Speed::VeryHigh),
        active_low: false,
    }
}
