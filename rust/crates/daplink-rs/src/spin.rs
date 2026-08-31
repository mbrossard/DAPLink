//! Alignment-pinned cycle spin for SWD bit timing.

use core::num::NonZeroU32;

/// Core cycles one loop iteration costs on Cortex-M0+: `subs` 1, taken `bne` 2.
pub const CYCLES_PER_ITERATION: u32 = 3;

/// A cycle request converted once into the trip count used by the aligned delay loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PreparedSpin(NonZeroU32);

impl PreparedSpin {
    /// Prepare a delay, rounding up to whole loop iterations and keeping one iteration minimum.
    pub const fn new(cycles: u32) -> Self {
        Self(nonzero(iterations_for(cycles)))
    }

    /// Run the prepared delay without repeating the cycle-to-iteration conversion.
    #[inline(always)]
    pub fn run(self) {
        spin_iterations(self.0.get());
    }
}

/// Spins for approximately `cycles` core cycles, never less than one iteration.
#[inline(always)]
pub fn spin_cycles(cycles: u32) {
    spin_iterations(iterations_for(cycles));
}

/// Iterations to spin for `cycles`, rounded up, at least one: a zero count would wrap the loop.
const fn iterations_for(cycles: u32) -> u32 {
    let iterations = ceil_div3(cycles);
    if iterations == 0 {
        1
    } else {
        iterations
    }
}

/// `cycles / 3` rounded up, for every `u32`; thumbv6m has no divide instruction.
const fn ceil_div3(cycles: u32) -> u32 {
    // 16384 = 3 * 5461 + 1, so the high part divides exactly and its remainder joins the low part.
    let high = cycles >> 14;
    5461 * high + small_ceil_div3(high + (cycles & 16_383))
}

/// `n / 3` rounded up, for `n < 2^19`, by a reciprocal whose 32-bit product cannot overflow.
const fn small_ceil_div3(n: u32) -> u32 {
    // 512 = 3 * 170 + 2, so the high part divides exactly and twice its value joins the low part.
    let high = n >> 9;
    let rest = 2 * high + (n & 511) + 2;
    // 43691 = ceil(2^17 / 3), which floors exactly for arguments below 2^17.
    170 * high + ((rest * 43_691) >> 17)
}

const fn nonzero(value: u32) -> NonZeroU32 {
    match NonZeroU32::new(value) {
        Some(value) => value,
        None => NonZeroU32::MIN,
    }
}

#[cfg(target_arch = "arm")]
#[inline(always)]
fn spin_iterations(iterations: u32) {
    // SAFETY: register arithmetic and a local backward branch only -- no memory, no stack, no
    // call; the count is non-zero, and the flags clobber is why `preserves_flags` is absent.
    unsafe {
        core::arch::asm!(
            ".p2align 4",
            // A local label: named labels would emit relocations thumbv6m cannot take.
            "2:",
            "subs {i}, #1",
            "bne 2b",
            i = inout(reg) iterations => _,
            options(nomem, nostack),
        );
    }
}

/// Host fallback for builds and tests off the target: same shape, no timing guarantee.
#[cfg(not(target_arch = "arm"))]
#[inline(always)]
fn spin_iterations(iterations: u32) {
    for _ in 0..iterations {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::{ceil_div3, iterations_for, spin_cycles, PreparedSpin, CYCLES_PER_ITERATION};

    /// The reciprocal divides exactly over every request a bit cell can plausibly make.
    #[test]
    fn the_reciprocal_divides_exactly_over_every_plausible_request() {
        for cycles in 0..=1u32 << 21 {
            assert_eq!(ceil_div3(cycles), cycles.div_ceil(3), "cycles = {cycles}");
        }
    }

    /// And over the rest of `u32`: both decomposition boundaries, plus all three residues.
    #[test]
    fn the_reciprocal_divides_exactly_over_the_whole_range() {
        let mut checked = 0u32;
        for high in (0..=u32::MAX >> 14).step_by(31) {
            for low in [0, 1, 2, 510, 511, 512, 16_381, 16_382, 16_383] {
                let cycles = (high << 14) | low;
                assert_eq!(ceil_div3(cycles), cycles.div_ceil(3), "cycles = {cycles}");
                checked += 1;
            }
        }
        for cycles in (0..=u32::MAX).step_by(65_537) {
            assert_eq!(ceil_div3(cycles), cycles.div_ceil(3), "cycles = {cycles}");
            checked += 1;
        }
        for cycles in [u32::MAX - 2, u32::MAX - 1, u32::MAX] {
            assert_eq!(ceil_div3(cycles), cycles.div_ceil(3), "cycles = {cycles}");
            checked += 1;
        }
        assert!(checked > 100_000, "the sweep must cover the range");
    }

    /// The conversion never yields zero, and never undershoots the request.
    #[test]
    fn the_conversion_never_undershoots_by_more_than_one_iteration() {
        for cycles in 0..100_000u32 {
            let iterations = iterations_for(cycles);
            assert!(iterations >= 1);
            let realized = iterations * CYCLES_PER_ITERATION;
            assert!(realized >= cycles && realized - cycles < 2 * CYCLES_PER_ITERATION);
        }
    }

    /// The primitive itself runs for the degenerate and the clamped request.
    #[test]
    fn the_spin_runs() {
        spin_cycles(0);
        spin_cycles(99);
        PreparedSpin::new(99).run();
    }
}
