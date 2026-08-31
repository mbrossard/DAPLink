//! Cycle-counting SWD bit-timing delay.

#[cfg(feature = "rp2040")]
use daplink_rs::PreparedSpin;
use daplink_rs::{spin_cycles, DelayCycles, DelayNs};

/// Core clock selected by the firmware composition; delays count cycles against it.
#[cfg(all(
    feature = "rp2040",
    not(any(feature = "clock-180mhz", feature = "clock-200mhz"))
))]
pub const CPU_HZ: u32 = 125_000_000;
#[cfg(feature = "clock-180mhz")]
pub const CPU_HZ: u32 = 180_000_000;
#[cfg(all(not(feature = "clock-180mhz"), feature = "clock-200mhz"))]
pub const CPU_HZ: u32 = 200_000_000;
#[cfg(all(
    not(feature = "rp2040"),
    feature = "rp2350",
    not(any(feature = "clock-180mhz", feature = "clock-200mhz"))
))]
pub const CPU_HZ: u32 = 150_000_000;

/// Cycles a request already spends in the bit cell: request = this plus the residual spin.
pub const SWD_FIXED_WORK_CYCLES: u32 = 0;

/// Floor on the residual spin, in core cycles: shorter half-periods corrupt data on the wire.
pub const MIN_HALF_PERIOD_CYCLES: u32 = 6;

#[cfg(feature = "rp2040")]
const FAST_HALF_SPIN: PreparedSpin = PreparedSpin::new(MIN_HALF_PERIOD_CYCLES);

/// Cycle-counting delay against the fixed core clock.
#[cfg(feature = "rp2040")]
#[derive(Clone, Copy)]
pub struct CycleDelay;

/// Cycle-counting delay against the selected RP2350 core clock.
#[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
#[derive(Clone, Copy)]
pub struct CycleDelay {
    hz: u32,
    min_half_period_cycles: u32,
}

impl CycleDelay {
    /// Build the delay for the selected chip and clock profile.
    pub const fn new() -> Self {
        #[cfg(feature = "rp2040")]
        return Self;

        #[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
        Self {
            hz: CPU_HZ,
            min_half_period_cycles: MIN_HALF_PERIOD_CYCLES,
        }
    }
}

impl Default for CycleDelay {
    fn default() -> Self {
        Self::new()
    }
}

impl DelayNs for CycleDelay {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        #[cfg(all(
            feature = "rp2040",
            not(any(feature = "clock-180mhz", feature = "clock-200mhz"))
        ))]
        spin_cycles(ns.div_ceil(8));
        #[cfg(all(
            feature = "rp2040",
            any(feature = "clock-180mhz", feature = "clock-200mhz")
        ))]
        spin_cycles((u64::from(ns) * u64::from(CPU_HZ)).div_ceil(1_000_000_000) as u32);
        #[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
        spin_cycles((u64::from(ns) * u64::from(self.hz)).div_ceil(1_000_000_000) as u32);
    }
}

impl DelayCycles for CycleDelay {
    #[inline(always)]
    fn cpu_clock(&self) -> u32 {
        #[cfg(feature = "rp2040")]
        return CPU_HZ;

        #[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
        self.hz
    }
    /// A request is a maximum, so a residual under the floor realizes AT the floor: never faster.
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
