//! Pico status LED and carrier LED bank.

use daplink_rs::{BankLed, StatusLed};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIN_10, PIN_11, PIN_12, PIN_14, PIN_15, PIN_25};
use embassy_rp::Peri;

pub const STATUS_LED_PIN: u8 = 25;

pub fn status_led(pin: Peri<'static, PIN_25>) -> StatusLed<Output<'static>> {
    StatusLed::new(Output::new(pin, Level::Low))
}

pub struct CarrierPins {
    pub gpio10: Peri<'static, PIN_10>,
    pub gpio11: Peri<'static, PIN_11>,
    pub gpio12: Peri<'static, PIN_12>,
    pub gpio14: Peri<'static, PIN_14>,
    pub gpio15: Peri<'static, PIN_15>,
}

pub type CarrierBankLed = BankLed<Output<'static>>;

pub struct CarrierLeds {
    pub heartbeat: CarrierBankLed,
    pub dap: CarrierBankLed,
    pub flash: CarrierBankLed,
    pub vcom: CarrierBankLed,
    pub swo: CarrierBankLed,
}

/// Consume the carrier pins and initialize its five active-low LEDs dark.
pub fn carrier_leds(pins: CarrierPins) -> CarrierLeds {
    CarrierLeds {
        heartbeat: BankLed::new(Output::new(pins.gpio10, Level::High)),
        dap: BankLed::new(Output::new(pins.gpio11, Level::High)),
        flash: BankLed::new(Output::new(pins.gpio12, Level::High)),
        vcom: BankLed::new(Output::new(pins.gpio14, Level::High)),
        swo: BankLed::new(Output::new(pins.gpio15, Level::High)),
    }
}
