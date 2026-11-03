//! Cycle-counting delay for a core clock supplied at runtime.

use crate::{spin_cycles, DelayCycles, DelayNs, PreparedSpin};

/// Copyable cycle-counting delay shared by the probe dependencies and engine.
#[derive(Clone, Copy)]
pub struct DynCycleDelay {
    hz: u32,
    fixed_work: u32,
    min_half_period: u32,
}

impl DynCycleDelay {
    /// Build a delay that spins the full requested cycle count.
    pub const fn new(hz: u32) -> Self {
        Self {
            hz,
            fixed_work: 0,
            min_half_period: 0,
        }
    }

    /// Subtract fixed bit-cell work and floor the residual half-period delay.
    pub const fn with_fixed_work(hz: u32, fixed_work: u32, min_half_period_cycles: u32) -> Self {
        Self {
            hz,
            fixed_work,
            min_half_period: min_half_period_cycles,
        }
    }

    /// Prepare a reusable half-period with this delay's fixed-work subtraction and floor.
    #[inline(always)]
    pub fn prepare(&self, cycles: u32) -> PreparedSpin {
        PreparedSpin::new(
            cycles
                .saturating_sub(self.fixed_work)
                .max(self.min_half_period),
        )
    }
}

impl DelayNs for DynCycleDelay {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        spin_cycles((u64::from(ns) * u64::from(self.hz)).div_ceil(1_000_000_000) as u32);
    }
}

impl DelayCycles for DynCycleDelay {
    #[inline(always)]
    fn cpu_clock(&self) -> u32 {
        self.hz
    }

    #[inline(always)]
    fn delay_cycles(&mut self, cycles: u32) {
        spin_cycles(
            cycles
                .saturating_sub(self.fixed_work)
                .max(self.min_half_period),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prepared_delay_preserves_fixed_work_floor_and_full_range() {
        for (work, floor, request, expected) in [
            (0, 0, 0, 0),
            (0, 0, u32::MAX, u32::MAX),
            (8, 4, 1, 4),
            (8, 4, 8, 4),
            (8, 4, 9, 4),
            (8, 4, 15, 7),
            (u32::MAX, 1, 36, 1),
        ] {
            let delay = DynCycleDelay::with_fixed_work(72_000_000, work, floor);
            assert_eq!(delay.prepare(request), PreparedSpin::new(expected));
        }
    }
}
