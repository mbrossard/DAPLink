//! LED host-status and activity hooks for the DAP engine and firmware loops.

use dap_rs::dap::{DapLeds, HostStatus};
use embedded_hal::digital::OutputPin;

/// A `DapLeds` implementation for probes with no status LED: every host-status update is ignored.
pub struct NoopLeds;

impl DapLeds for NoopLeds {
    fn react_to_host_status(&mut self, _status: HostStatus) {}
}

/// Drives one status LED from the host's connect/running state, lit while either holds. A pin
/// error is ignored -- a stuck LED must never abort a session. Polarity is chosen at construction.
pub struct StatusLed<P: OutputPin> {
    led: P,
    connected: bool,
    running: bool,
    active_low: bool,
}

impl<P: OutputPin> StatusLed<P> {
    /// Wrap an active-high output pin (drive high to light the LED), starting with the LED off.
    pub fn new(led: P) -> Self {
        Self::with_polarity(led, false)
    }

    /// Wrap an active-low output pin (drive low to light the LED), starting with the LED off.
    pub fn new_active_low(led: P) -> Self {
        Self::with_polarity(led, true)
    }

    fn with_polarity(mut led: P, active_low: bool) -> Self {
        let _ = if active_low {
            led.set_high()
        } else {
            led.set_low()
        };
        Self {
            led,
            connected: false,
            running: false,
            active_low,
        }
    }

    /// Drive the pin to reflect the current connect/running state, honoring the wired polarity.
    fn apply(&mut self) {
        let on = self.connected || self.running;
        let drive_high = on ^ self.active_low;
        let _ = if drive_high {
            self.led.set_high()
        } else {
            self.led.set_low()
        };
    }
}

impl<P: OutputPin> DapLeds for StatusLed<P> {
    fn react_to_host_status(&mut self, status: HostStatus) {
        match status {
            HostStatus::Connected(on) => self.connected = on,
            HostStatus::Running(on) => self.running = on,
        }
        self.apply();
    }
}

/// A cheap activity/level sink a firmware loop can call on every iteration without cost when
/// nothing is wired: `()` is the no-op implementation every non-LED build passes.
pub trait Activity {
    /// Toggle: called once per activity event (a command processed, a byte moved, a tick).
    fn pulse(&mut self) {}
    /// Track a level directly: called on a session boundary (e.g. a flash session opening).
    fn level(&mut self, _on: bool) {}
}

impl Activity for () {}

/// A `'static mut` no-op [`Activity`], for a call site that needs the trait-object shape (e.g. an
/// embassy task, which cannot be generic) but has no LED wired on this build.
///
/// # Panics
/// Panics if called a second time from the same call site: each call promotes a fresh function-
/// local static, so one call site must call it at most once (firmware `main` runs once).
pub fn noop_activity() -> &'static mut dyn Activity {
    static CELL: static_cell::StaticCell<()> = static_cell::StaticCell::new();
    CELL.init(())
}

/// One carrier debug-bank LED, wired active-low. [`pulse`](BankLed::pulse) flickers per event;
/// [`level`](BankLed::level) tracks an explicit state, for a session boundary.
pub struct BankLed<P: OutputPin> {
    led: P,
    lit: bool,
}

impl<P: OutputPin> BankLed<P> {
    /// Wrap an active-low output pin, starting with the LED dark (pin high).
    pub fn new(mut led: P) -> Self {
        let _ = led.set_high();
        Self { led, lit: false }
    }

    fn drive(&mut self, lit: bool) {
        self.lit = lit;
        let _ = if lit {
            self.led.set_low()
        } else {
            self.led.set_high()
        };
    }
}

impl<P: OutputPin> Activity for BankLed<P> {
    fn pulse(&mut self) {
        let lit = !self.lit;
        self.drive(lit);
    }

    fn level(&mut self, on: bool) {
        self.drive(on);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::digital::{Mock, State, Transaction};

    /// The LED lights on connect or running and goes dark only once both flags clear.
    #[test]
    fn led_tracks_connected_or_running() {
        let expectations = [
            Transaction::set(State::Low),
            Transaction::set(State::High),
            Transaction::set(State::High),
            Transaction::set(State::High),
            Transaction::set(State::Low),
        ];
        let mut leds = StatusLed::new(Mock::new(&expectations));

        leds.react_to_host_status(HostStatus::Connected(true));
        leds.react_to_host_status(HostStatus::Running(true));
        leds.react_to_host_status(HostStatus::Connected(false));
        leds.react_to_host_status(HostStatus::Running(false));

        leds.led.done();
    }

    /// An active-low LED inverts every drive: dark is the high level, lit is the low level.
    #[test]
    fn active_low_led_inverts_drive() {
        let expectations = [
            Transaction::set(State::High),
            Transaction::set(State::Low),
            Transaction::set(State::Low),
            Transaction::set(State::High),
        ];
        let mut leds = StatusLed::new_active_low(Mock::new(&expectations));

        leds.react_to_host_status(HostStatus::Connected(true));
        leds.react_to_host_status(HostStatus::Running(false));
        leds.react_to_host_status(HostStatus::Connected(false));

        leds.led.done();
    }

    /// A bank LED starts dark (pin high, since it is active-low), then alternates on `pulse`.
    #[test]
    fn bank_led_pulse_toggles_from_dark() {
        let expectations = [
            Transaction::set(State::High),
            Transaction::set(State::Low),
            Transaction::set(State::High),
            Transaction::set(State::Low),
        ];
        let mut led = BankLed::new(Mock::new(&expectations));

        led.pulse();
        led.pulse();
        led.pulse();

        led.led.done();
    }

    /// `level` drives an explicit state regardless of the previous one, unlike `pulse`.
    #[test]
    fn bank_led_level_sets_explicit_state() {
        let expectations = [
            Transaction::set(State::High),
            Transaction::set(State::Low),
            Transaction::set(State::Low),
            Transaction::set(State::High),
        ];
        let mut led = BankLed::new(Mock::new(&expectations));

        led.level(true);
        led.level(true);
        led.level(false);

        led.led.done();
    }

    /// `()` is the no-op `Activity`: it never panics and never touches hardware.
    #[test]
    fn unit_activity_is_a_true_no_op() {
        let mut sink = ();
        sink.pulse();
        sink.level(true);
        sink.level(false);
    }
}
