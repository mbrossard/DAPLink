//! Onboard status LED, and (behind `carrier-leds`) the carrier debug bank.

use daplink_rs::StatusLed;
use rp2040_hal::gpio::bank0::Gpio25;
use rp2040_hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};

/// Onboard status LED pin: GP25 on both the Pico and Pico 2, active-high.
pub const STATUS_LED_PIN: u8 = 25;

/// The onboard status LED's pin type, once switched to a push-pull output.
pub type StatusLedPin = Pin<Gpio25, FunctionSio<SioOutput>, PullDown>;

/// Build the onboard status LED, dark until the host connects or runs the target.
pub fn status_led(pin: StatusLedPin) -> StatusLed<StatusLedPin> {
    StatusLed::new(pin)
}

#[cfg(feature = "carrier-leds")]
pub use carrier::*;

#[cfg(feature = "carrier-leds")]
mod carrier {
    use super::{FunctionSio, Pin, PullDown, SioOutput};
    use daplink_rs::BankLed;
    use rp2040_hal::gpio::bank0::{Gpio10, Gpio11, Gpio12, Gpio14};

    /// Carrier debug-bank pin numbers, one line each so reassignment stays cheap.
    pub const HEARTBEAT_LED_PIN: u8 = 10;
    pub const DAP_LED_PIN: u8 = 11;
    pub const FLASH_LED_PIN: u8 = 12;
    pub const VCOM_LED_PIN: u8 = 14;
    /// GP15: wired on the carrier, but this firmware has no cheap SWO tap, so the role is unwired.
    pub const SWO_LED_PIN: u8 = 15;

    type Out<I> = Pin<I, FunctionSio<SioOutput>, PullDown>;

    /// One carrier-bank role LED, active-low (each pulls to 3V3 through 2 kOhm).
    pub type CarrierBankLed<I> = BankLed<Out<I>>;

    /// The four wired carrier-bank pins (GP15/SWO has no cheap tap on this firmware).
    pub struct CarrierPins {
        pub heartbeat: Out<Gpio10>,
        pub dap: Out<Gpio11>,
        pub flash: Out<Gpio12>,
        pub vcom: Out<Gpio14>,
    }

    /// The four wired carrier-bank LEDs, dark until a role wiring drives them.
    pub struct CarrierLeds {
        pub heartbeat: CarrierBankLed<Gpio10>,
        pub dap: CarrierBankLed<Gpio11>,
        pub flash: CarrierBankLed<Gpio12>,
        pub vcom: CarrierBankLed<Gpio14>,
    }

    /// Build the carrier debug bank's four wired LEDs.
    pub fn carrier_leds(pins: CarrierPins) -> CarrierLeds {
        CarrierLeds {
            heartbeat: BankLed::new(pins.heartbeat),
            dap: BankLed::new(pins.dap),
            flash: BankLed::new(pins.flash),
            vcom: BankLed::new(pins.vcom),
        }
    }
}
