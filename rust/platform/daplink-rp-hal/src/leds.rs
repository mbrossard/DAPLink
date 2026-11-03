//! Pico status LED and carrier LED bank.

use daplink_rs::{BankLed, StatusLed};
use rp2040_hal::gpio::bank0::{Gpio10, Gpio11, Gpio12, Gpio14, Gpio25};
use rp2040_hal::gpio::{FunctionNull, FunctionSio, Pin, PullDown, SioOutput};

pub const STATUS_LED_PIN: u8 = 25;
pub type StatusLedPin = Pin<Gpio25, FunctionSio<SioOutput>, PullDown>;

pub fn status_led(pin: StatusLedPin) -> StatusLed<StatusLedPin> {
    StatusLed::new(pin)
}

pub struct CarrierPins {
    pub gpio10: Pin<Gpio10, FunctionNull, PullDown>,
    pub gpio11: Pin<Gpio11, FunctionNull, PullDown>,
    pub gpio12: Pin<Gpio12, FunctionNull, PullDown>,
    pub gpio14: Pin<Gpio14, FunctionNull, PullDown>,
}

pub type CarrierBankLed<I> = BankLed<Pin<I, FunctionSio<SioOutput>, PullDown>>;

pub struct CarrierLeds {
    pub heartbeat: CarrierBankLed<Gpio10>,
    pub dap: CarrierBankLed<Gpio11>,
    pub flash: CarrierBankLed<Gpio12>,
    pub vcom: CarrierBankLed<Gpio14>,
}

/// Consume the carrier pins and initialize its four active-low LEDs dark.
pub fn carrier_leds(pins: CarrierPins) -> CarrierLeds {
    CarrierLeds {
        heartbeat: BankLed::new(pins.gpio10.into_push_pull_output()),
        dap: BankLed::new(pins.gpio11.into_push_pull_output()),
        flash: BankLed::new(pins.gpio12.into_push_pull_output()),
        vcom: BankLed::new(pins.gpio14.into_push_pull_output()),
    }
}
