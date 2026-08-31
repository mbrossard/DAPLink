//! Onboard status LED, and (behind `carrier-leds`) the carrier debug bank.

use daplink_rs::StatusLed;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIN_25;
use embassy_rp::Peri;

/// Onboard status LED pin: GP25 on both the Pico and Pico 2, active-high.
pub const STATUS_LED_PIN: u8 = 25;

/// Build the onboard status LED, dark until the host connects or runs the target.
pub fn status_led(pin: Peri<'static, PIN_25>) -> StatusLed<Output<'static>> {
    StatusLed::new(Output::new(pin, Level::Low))
}

#[cfg(feature = "carrier-leds")]
pub use carrier::*;

#[cfg(feature = "carrier-leds")]
mod carrier {
    use super::{Level, Output};
    use daplink_rs::BankLed;
    use embassy_rp::peripherals::{PIN_10, PIN_11, PIN_12, PIN_14, PIN_15};
    use embassy_rp::Peri;

    /// Carrier debug-bank pin numbers, one line each so reassignment stays cheap.
    pub const HEARTBEAT_LED_PIN: u8 = 10;
    pub const DAP_LED_PIN: u8 = 11;
    pub const FLASH_LED_PIN: u8 = 12;
    pub const VCOM_LED_PIN: u8 = 14;
    pub const SWO_LED_PIN: u8 = 15;

    /// One carrier-bank role LED, active-low (each pulls to 3V3 through 2 kOhm).
    pub type CarrierBankLed = BankLed<Output<'static>>;

    /// The five carrier-bank pins, taken from `Peripherals` before probe bring-up claims the rest.
    pub struct CarrierPins {
        pub heartbeat: Peri<'static, PIN_10>,
        pub dap: Peri<'static, PIN_11>,
        pub flash: Peri<'static, PIN_12>,
        pub vcom: Peri<'static, PIN_14>,
        pub swo: Peri<'static, PIN_15>,
    }

    /// The five carrier-bank LEDs, dark until a role wiring drives them.
    pub struct CarrierLeds {
        pub heartbeat: CarrierBankLed,
        pub dap: CarrierBankLed,
        pub flash: CarrierBankLed,
        pub vcom: CarrierBankLed,
        pub swo: CarrierBankLed,
    }

    /// Build the carrier debug bank from its five pins.
    pub fn carrier_leds(pins: CarrierPins) -> CarrierLeds {
        CarrierLeds {
            heartbeat: BankLed::new(Output::new(pins.heartbeat, Level::High)),
            dap: BankLed::new(Output::new(pins.dap, Level::High)),
            flash: BankLed::new(Output::new(pins.flash, Level::High)),
            vcom: BankLed::new(Output::new(pins.vcom, Level::High)),
            swo: BankLed::new(Output::new(pins.swo, Level::High)),
        }
    }
}
