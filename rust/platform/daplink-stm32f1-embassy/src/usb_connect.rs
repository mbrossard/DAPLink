//! The switched D+ pull-up that attaches the board to USB.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{peripherals, Peri};

/// A switched D+ pull-up: the board is absent from the bus until [`attach`](UsbConnect::attach).
pub struct UsbConnect<'d> {
    pin: Output<'d>,
    /// True when driving the pin low connects the pull-up.
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

impl daplink_rs::daplink_core::UsbReconnect for UsbConnect<'_> {
    fn detach(&mut self) {
        UsbConnect::detach(self);
    }

    fn attach(&mut self) {
        UsbConnect::attach(self);
    }
}

/// USB connect control on PB15, active low; returned detached.
pub fn usb_connect_pb15(pin: Peri<'_, peripherals::PB15>) -> UsbConnect<'_> {
    UsbConnect {
        pin: Output::new(pin, Level::High, Speed::VeryHigh),
        active_low: true,
    }
}

/// USB connect control on PA15, active high; returned detached. Free PA15 from JTAG first.
pub fn usb_connect_pa15(pin: Peri<'_, peripherals::PA15>) -> UsbConnect<'_> {
    UsbConnect {
        pin: Output::new(pin, Level::Low, Speed::VeryHigh),
        active_low: false,
    }
}
