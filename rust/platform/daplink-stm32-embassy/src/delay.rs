//! Cycle-counting SWD bit-timing delay against a firmware-supplied core clock.

use daplink_rs::{spin_cycles, DelayCycles, DelayNs};

/// Cycle-counting delay for a core clock only known at runtime; `Copy`, so one value feeds both
/// the deps and the engine.
#[derive(Clone, Copy)]
pub struct DynCycleDelay {
    hz: u32,
    fixed_work: u32,
    min_half_period: u32,
}

impl DynCycleDelay {
    /// Build the delay for a core clock of `hz`, spinning the full requested count.
    pub const fn new(hz: u32) -> Self {
        Self {
            hz,
            fixed_work: 0,
            min_half_period: 0,
        }
    }

    /// [`new`](Self::new) with the bit cell's fixed work (core cycles) subtracted from every
    /// request, and a floor on the residual spin: shorter half-periods corrupt data on the wire.
    pub const fn with_fixed_work(hz: u32, fixed_work: u32, min_half_period_cycles: u32) -> Self {
        Self {
            hz,
            fixed_work,
            min_half_period: min_half_period_cycles,
        }
    }
}

impl DelayNs for DynCycleDelay {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        // Widened: the product overflows `u32` past a few tens of ns at these clock rates.
        spin_cycles((u64::from(ns) * u64::from(self.hz)).div_ceil(1_000_000_000) as u32);
    }
}

impl DelayCycles for DynCycleDelay {
    #[inline(always)]
    fn cpu_clock(&self) -> u32 {
        self.hz
    }
    /// A request is a maximum, so a residual under the floor realizes AT the floor: never faster.
    #[inline(always)]
    fn delay_cycles(&mut self, cycles: u32) {
        spin_cycles(
            cycles
                .saturating_sub(self.fixed_work)
                .max(self.min_half_period),
        );
    }
}
