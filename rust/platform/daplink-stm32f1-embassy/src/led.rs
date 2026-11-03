//! Tri-state driver for a bicolor LED pair tapped at its midpoint.

use daplink_rs::dap_rs::dap::{DapLeds, HostStatus};
use embassy_stm32::gpio::{Flex, Pin, Pull, Speed};
use embassy_stm32::Peri;

/// A bicolor LED on one pin: high is activity, low idle, floating off.
pub struct BicolorLed {
    pin: Flex<'static>,
    connected: bool,
    running: bool,
}

impl BicolorLed {
    /// Wrap the pin, starting off.
    pub fn new(pin: Peri<'static, impl Pin>) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input(Pull::None);
        Self {
            pin,
            connected: false,
            running: false,
        }
    }

    pub fn idle(&mut self) {
        self.pin.set_low();
        self.pin.set_as_output(Speed::Low);
    }

    pub fn activity(&mut self) {
        self.pin.set_high();
        self.pin.set_as_output(Speed::Low);
    }

    pub fn off(&mut self) {
        self.pin.set_as_input(Pull::None);
    }
}

impl DapLeds for BicolorLed {
    fn react_to_host_status(&mut self, status: HostStatus) {
        match status {
            HostStatus::Connected(on) => self.connected = on,
            HostStatus::Running(on) => self.running = on,
        }
        if self.connected || self.running {
            self.activity();
        } else {
            self.idle();
        }
    }
}
