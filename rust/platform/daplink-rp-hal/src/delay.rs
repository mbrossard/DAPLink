//! Cycle-counting delay against the fixed RP2040 core clock.

use daplink_rs::{spin_cycles, DelayCycles, DelayNs, PreparedSpin};

/// RP2040 core clock produced by [`crate::clocks`]; SWD and `WAIT` delays count cycles against it.
#[cfg(not(feature = "clock-200mhz"))]
pub const CPU_HZ: u32 = 125_000_000;
#[cfg(feature = "clock-200mhz")]
pub const CPU_HZ: u32 = 200_000_000;

/// Cycles a request already spends in the bit cell: request = this plus the residual spin.
pub const SWD_FIXED_WORK_CYCLES: u32 = 0;

/// Floor on the residual spin, in core cycles: shorter half-periods corrupt data on the wire.
pub const MIN_HALF_PERIOD_CYCLES: u32 = 6;

const FAST_HALF_SPIN: PreparedSpin = PreparedSpin::new(MIN_HALF_PERIOD_CYCLES);

/// Cycle-counting delay against [`CPU_HZ`].
#[derive(Clone, Copy)]
pub struct CycleDelay;

impl DelayNs for CycleDelay {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        #[cfg(not(feature = "clock-200mhz"))]
        spin_cycles(ns.div_ceil(8));
        #[cfg(feature = "clock-200mhz")]
        spin_cycles((u64::from(ns) * u64::from(CPU_HZ)).div_ceil(1_000_000_000) as u32);
    }
}

impl DelayCycles for CycleDelay {
    #[inline(always)]
    fn cpu_clock(&self) -> u32 {
        CPU_HZ
    }
    /// A request is a maximum, so a residual under the floor realizes AT the floor: never faster.
    #[inline(always)]
    fn delay_cycles(&mut self, cycles: u32) {
        let cycles = cycles
            .saturating_sub(SWD_FIXED_WORK_CYCLES)
            .max(MIN_HALF_PERIOD_CYCLES);
        if cycles == MIN_HALF_PERIOD_CYCLES {
            FAST_HALF_SPIN.run();
        } else {
            spin_cycles(cycles);
        }
    }
}
