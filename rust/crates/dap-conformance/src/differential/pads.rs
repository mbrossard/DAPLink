//! GPIO, delay and LED handles on the simulated pads, for engines built on dap-rs's traits.

use dap_rs::dap::{DapLeds, DelayNs, HostStatus};
use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin};

use crate::target::{Line, Shared};

/// The core clock every simulated delay claims.
pub const CPU_HZ: u32 = 100_000_000;

/// A GPIO handle on one simulated pad: push-pull with a direction, or open-drain.
#[derive(Clone)]
pub struct SimPin {
    sim: Shared,
    line: Line,
    open_drain: bool,
}

impl SimPin {
    /// A push-pull handle on `line`.
    pub fn new(sim: &Shared, line: Line) -> Self {
        Self {
            sim: sim.clone(),
            line,
            open_drain: false,
        }
    }

    /// An open-drain handle on `line`, as firmwares wire nRESET: low drives, high releases.
    pub fn open_drain(sim: &Shared, line: Line) -> Self {
        Self {
            open_drain: true,
            ..Self::new(sim, line)
        }
    }
}

impl InputOutputPin for SimPin {
    fn set_as_output(&mut self) {
        if !self.open_drain {
            self.sim.borrow_mut().set_output(self.line, true);
        }
    }

    fn set_high(&mut self, high: bool) {
        let mut s = self.sim.borrow_mut();
        s.set_latch(self.line, high);
        if self.open_drain {
            s.set_output(self.line, !high);
        }
    }

    fn set_as_input(&mut self) {
        self.sim.borrow_mut().set_output(self.line, false);
    }

    fn is_high(&mut self) -> bool {
        self.sim.borrow().level(self.line)
    }
}

/// A delay that takes no time: the simulation has no clock, so no check covers wire timing.
#[derive(Clone, Copy, Debug, Default)]
pub struct SimDelay;

impl DelayNs for SimDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

impl DelayCycles for SimDelay {
    fn cpu_clock(&self) -> u32 {
        CPU_HZ
    }

    fn delay_cycles(&mut self, _cycles: u32) {}
}

/// LEDs that ignore host status.
pub struct NoLeds;

impl DapLeds for NoLeds {
    fn react_to_host_status(&mut self, _host_status: HostStatus) {}
}
