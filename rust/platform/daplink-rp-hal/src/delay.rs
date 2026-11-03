//! Cycle-counting delay against the fixed RP2040 core clock.

use daplink_rs::{spin_cycles, DelayCycles, DelayNs, PreparedSpin};

/// Cycles a bit cell already spends outside the residual spin.
pub const SWD_FIXED_WORK_CYCLES: u32 = 0;

/// Floor on the residual spin, in core cycles: shorter half-periods corrupt data on the wire.
pub const MIN_HALF_PERIOD_CYCLES: u32 = 6;

const FAST_HALF_SPIN: PreparedSpin = PreparedSpin::new(MIN_HALF_PERIOD_CYCLES);

/// Cycle-counting delay against the firmware's core frequency in Hz.
#[derive(Clone, Copy)]
pub struct CycleDelay<const CPU_HZ: u32>;

impl<const CPU_HZ: u32> DelayNs for CycleDelay<CPU_HZ> {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        if CPU_HZ == 125_000_000 {
            spin_cycles(ns.div_ceil(8));
        } else {
            spin_cycles((u64::from(ns) * u64::from(CPU_HZ)).div_ceil(1_000_000_000) as u32);
        }
    }
}

impl<const CPU_HZ: u32> DelayCycles for CycleDelay<CPU_HZ> {
    #[inline(always)]
    fn cpu_clock(&self) -> u32 {
        CPU_HZ
    }
    /// A request is a maximum: a residual under the floor realizes at the floor.
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
