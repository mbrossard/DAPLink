//! Cycle-counting SWD bit-timing delay.

use crate::ClockProfile;
#[cfg(feature = "rp2040")]
use daplink_rs::PreparedSpin;
use daplink_rs::{spin_cycles, DelayCycles, DelayNs};

/// Cycles a bit cell already spends outside the residual spin.
pub const SWD_FIXED_WORK_CYCLES: u32 = 0;

/// Floor on the residual spin, in core cycles: shorter half-periods corrupt data on the wire.
pub const MIN_HALF_PERIOD_CYCLES: u32 = 6;

#[cfg(feature = "rp2040")]
const FAST_HALF_SPIN: PreparedSpin = PreparedSpin::new(MIN_HALF_PERIOD_CYCLES);

/// Cycle-counting delay against an explicitly selected system clock.
#[derive(Clone, Copy)]
pub struct CycleDelay {
    hz: u32,
    #[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
    min_half_period_cycles: u32,
}

impl CycleDelay {
    /// Build a delay matched to the profile passed to [`crate::init`].
    pub const fn new(profile: ClockProfile) -> Self {
        Self {
            hz: profile.cpu_hz(),
            #[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
            min_half_period_cycles: MIN_HALF_PERIOD_CYCLES,
        }
    }
}

impl DelayNs for CycleDelay {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        spin_cycles((u64::from(ns) * u64::from(self.hz)).div_ceil(1_000_000_000) as u32);
    }
}

impl DelayCycles for CycleDelay {
    #[inline(always)]
    fn cpu_clock(&self) -> u32 {
        self.hz
    }
    /// A request is a maximum: a residual under the floor realizes at the floor.
    #[inline(always)]
    fn delay_cycles(&mut self, cycles: u32) {
        #[cfg(feature = "rp2040")]
        {
            let cycles = cycles
                .saturating_sub(SWD_FIXED_WORK_CYCLES)
                .max(MIN_HALF_PERIOD_CYCLES);
            if cycles == MIN_HALF_PERIOD_CYCLES {
                FAST_HALF_SPIN.run();
            } else {
                spin_cycles(cycles);
            }
        }
        #[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
        spin_cycles(cycles.max(self.min_half_period_cycles));
    }
}
