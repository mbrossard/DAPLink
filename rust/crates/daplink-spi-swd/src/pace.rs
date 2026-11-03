//! Whole-period plans for bit-bang cells paced by a counted spin loop.

use core::num::NonZeroU32;

/// Core cycles one iteration of the spin these plans count: `subs` and a taken `bne`.
pub const SPIN_CYCLES: u32 = 3;

/// Shortest low half, high half and period of a scaled cell, in picoseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Floors {
    /// Clock fall to the next rise, including any sample.
    pub low_ps: u32,
    /// Clock rise to the next fall.
    pub high_ps: u32,
    /// Rise to rise.
    pub period_ps: u32,
}

/// `ps` rounded up to whole core cycles.
pub const fn ps_cycles(ps: u32, cpu_hz: u32) -> u32 {
    (ps as u64 * cpu_hz as u64).div_ceil(1_000_000_000_000) as u32
}

/// Target cycles of one cell: the request period rounded up once, with the half floors beside it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Targets {
    period: u32,
    low: u32,
    low_floor: u32,
    high_floor: u32,
}

/// The targets for a request under `floors`.
const fn targets(cpu_hz: u32, request_hz: u32, floors: Floors) -> Targets {
    let period = cpu_hz.div_ceil(if request_hz == 0 { 1 } else { request_hz });
    period_targets(cpu_hz, period, floors)
}

/// The targets for a request of `period` whole cycles.
const fn period_targets(cpu_hz: u32, period: u32, floors: Floors) -> Targets {
    let low_floor = ps_cycles(floors.low_ps, cpu_hz);
    Targets {
        period: max(period, ps_cycles(floors.period_ps, cpu_hz)),
        low: max(period.div_ceil(2), low_floor),
        low_floor,
        high_floor: ps_cycles(floors.high_ps, cpu_hz),
    }
}

/// A class's least fixed work besides its spins.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fixed {
    /// Cycles the low half spends outside its spin.
    pub low: i32,
    /// Cycles the high half spends outside its spin.
    pub high: i32,
    /// Cycles the whole cell spends outside its spins.
    pub period: i32,
}

/// Spins for a class: the fewest that meet the period and both floors, low taking the larger half.
const fn halves(t: Targets, f: Fixed) -> (NonZeroU32, NonZeroU32) {
    let low_min = spins(t.low_floor as i32 - f.low);
    let high_min = spins(t.high_floor as i32 - f.high);
    let all = max(spins(t.period as i32 - f.period), low_min + high_min);
    let low = min(spins(t.low as i32 - f.low), all - high_min);
    (at_least_one(low), at_least_one(all - low))
}

const fn at_least_one(n: u32) -> NonZeroU32 {
    match NonZeroU32::new(n) {
        Some(n) => n,
        None => NonZeroU32::MIN,
    }
}

/// One cell class's fit: its fixed work and the floors its halves keep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClassFit {
    /// Work besides the spins.
    pub fixed: Fixed,
    /// Floors the halves keep.
    pub floors: Floors,
}

impl ClassFit {
    /// The class's (low, high) spin iterations for a request.
    pub const fn halves(&self, cpu_hz: u32, request_hz: u32) -> (NonZeroU32, NonZeroU32) {
        halves(targets(cpu_hz, request_hz, self.floors), self.fixed)
    }

    /// Fail the build if a request plans a cell fast, below a floor, or with a spare spin.
    pub const fn verify(&self, cpu_hz: u32) {
        let (f, fl) = (self.fixed, self.floors);
        let low_floor = ps_cycles(fl.low_ps, cpu_hz) as i32;
        let high_floor = ps_cycles(fl.high_ps, cpu_hz) as i32;
        let period_floor = ps_cycles(fl.period_ps, cpu_hz) as i32;
        let least = least_spins(low_floor, f.low) + least_spins(high_floor, f.high);
        // Past this period no floor binds, and every plan is the period split in two.
        let fixed = f.low.abs() + f.high.abs() + f.period.abs();
        let last = 2 * (low_floor + high_floor + period_floor + fixed) + 16;
        let mut period = 1;
        while period <= last {
            let (lo, hi) = halves(period_targets(cpu_hz, period as u32, fl), f);
            let (lo, hi) = (lo.get() as i32, hi.get() as i32);
            let cycles = (lo + hi) * SPIN_CYCLES as i32 + f.period;
            assert!(
                cycles >= period && cycles >= period_floor,
                "a cell runs fast"
            );
            assert!(
                lo * SPIN_CYCLES as i32 + f.low >= low_floor,
                "a low half is short"
            );
            assert!(
                hi * SPIN_CYCLES as i32 + f.high >= high_floor,
                "a high half is short"
            );
            let fewer = (lo + hi - 1) * SPIN_CYCLES as i32 + f.period;
            assert!(
                lo + hi - 1 < least || fewer < period || fewer < period_floor,
                "a cell takes a spare spin"
            );
            period += 1;
        }
    }
}

/// Fewest spins that lift `fixed` cycles of work to `floor`.
const fn least_spins(floor: i32, fixed: i32) -> i32 {
    spins(floor - fixed) as i32
}

const fn spins(cycles: i32) -> u32 {
    if cycles <= SPIN_CYCLES as i32 {
        1
    } else {
        (cycles as u32).div_ceil(SPIN_CYCLES)
    }
}

const fn max(a: u32, b: u32) -> u32 {
    if a > b {
        a
    } else {
        b
    }
}

const fn min(a: u32, b: u32) -> u32 {
    if a < b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn counts((lo, hi): (NonZeroU32, NonZeroU32)) -> (u32, u32) {
        (lo.get(), hi.get())
    }

    const F: Floors = Floors {
        low_ps: 50_000,
        high_ps: 28_000,
        period_ps: 70_000,
    };

    const fn fixed(low: i32, high: i32, period: i32) -> Fixed {
        Fixed { low, high, period }
    }

    #[test]
    fn realized_cells_never_outrun_the_request_or_the_floors() {
        let cpu = 250_000_000;
        for request in (500_000..40_000_000).step_by(997) {
            let t = targets(cpu, request, F);
            for f in [
                fixed(0, 0, 0),
                fixed(-3, -4, -1),
                fixed(-1, -1, 10),
                fixed(2, 6, 8),
            ] {
                let (lo, hi) = halves(t, f);
                let (lo, hi) = (lo.get(), hi.get());
                let low = (lo * SPIN_CYCLES) as i32 + f.low;
                let high = (hi * SPIN_CYCLES) as i32 + f.high;
                let period = ((lo + hi) * SPIN_CYCLES) as i32 + f.period;
                assert!(period as u64 * u64::from(request) >= u64::from(cpu));
                assert!(low >= ps_cycles(F.low_ps, cpu) as i32);
                assert!(high >= ps_cycles(F.high_ps, cpu) as i32);
                assert!(period >= ps_cycles(F.period_ps, cpu) as i32);
            }
        }
    }

    #[test]
    fn the_period_is_split_once() {
        let t = targets(250_000_000, 12_000_000, F);
        assert_eq!(
            t,
            Targets {
                period: 21,
                low: 13,
                low_floor: 13,
                high_floor: 7
            }
        );
        // 21 cycles less 3 of fixed work is six iterations, not the seven per-half rounding costs.
        assert_eq!(counts(halves(t, fixed(1, 2, 3))), (4, 2));
        // A class whose high half does no work keeps its floor whatever its period credit.
        assert_eq!(counts(halves(t, fixed(-1, -1, 10))), (5, 3));
    }

    const fn floors(low_ps: u32, high_ps: u32, period_ps: u32) -> Floors {
        Floors {
            low_ps,
            high_ps,
            period_ps,
        }
    }

    const CPU: u32 = 250_000_000;
    const DRIVE: Floors = floors(20_000, 20_000, 0);
    const BLOCK_SAMPLE: Floors = floors(20_000, 20_000, 44_000);
    const JTAG_SAMPLE: Floors = floors(36_000, 20_000, 0);

    /// The H523 block and lean JTAG classes at 250 MHz: control, ACK, read, write, then IR.
    const H523: [(Floors, Fixed); 9] = [
        (DRIVE, fixed(-1, 0, 2)),
        (BLOCK_SAMPLE, fixed(-1, 0, 2)),
        (BLOCK_SAMPLE, fixed(5, 2, 8)),
        (DRIVE, fixed(-1, 3, 4)),
        (DRIVE, fixed(-2, 0, 2)),
        (JTAG_SAMPLE, fixed(1, 5, 8)),
        (JTAG_SAMPLE, fixed(2, 2, 5)),
        (DRIVE, fixed(-1, 7, 8)),
        (DRIVE, fixed(-4, 7, 5)),
    ];

    #[test]
    fn the_h523_fits_pass_their_compile_time_check() {
        for (floors, fixed) in H523 {
            let fit = ClassFit { fixed, floors };
            fit.verify(CPU);
            assert_eq!(
                fit.halves(CPU, 20_000_000),
                halves(targets(CPU, 20_000_000, floors), fixed)
            );
        }
    }

    /// `CPU / p` rounded up for every period `p` to 2^20 cycles (239 Hz), then the powers of two.
    fn requests() -> impl Iterator<Item = u32> {
        let periods = (1..=1u32 << 20).map(|p| CPU.div_ceil(p));
        periods.chain((0..32).map(|k| 1u32 << k))
    }

    /// FNV-1a, a word per plan, over each H523 class's plan for every request in [`requests`].
    fn h523_plan_digest() -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325u64;
        for (floors, fixed) in H523 {
            let fit = ClassFit { fixed, floors };
            for request in requests() {
                let (lo, hi) = counts(fit.halves(CPU, request));
                hash = (hash ^ (u64::from(lo) << 32 | u64::from(hi))).wrapping_mul(0x100_0000_01b3);
            }
        }
        hash
    }

    /// A changed plan changes the realized wire clock, and needs recapturing before it is blessed.
    #[test]
    fn the_h523_plan_table_is_unchanged() {
        assert_eq!(h523_plan_digest(), 0x685e_5a12_2fa8_ca3c);
    }

    /// Realized (low, high, period) cycles of a class at `request`.
    fn cell(request: u32, (fl, f): (Floors, Fixed)) -> (i32, i32, i32) {
        let (lo, hi) = counts(halves(targets(CPU, request, fl), f));
        let (lo, hi) = (lo as i32 * 3, hi as i32 * 3);
        (lo + f.low, hi + f.high, lo + hi + f.period)
    }

    /// Fewest spins that lift `fixed` cycles of work to `floor`.
    fn least(floor: u32, fixed: i32) -> i32 {
        (1..).find(|s| s * 3 + fixed >= floor as i32).unwrap()
    }

    #[test]
    fn every_request_from_1_hz_to_40_mhz_gets_the_fewest_spins_that_are_never_faster() {
        let slow = (0..=24).map(|k| (1u64 << k) as u32);
        let periods = (7..=200_000).map(|p: u32| CPU.div_ceil(p));
        for request in slow.chain(periods).filter(|&r| r <= 40_000_000) {
            let period = CPU.div_ceil(request);
            for class in H523 {
                let (fl, f) = class;
                let (low, high, realized) = cell(request, class);
                let floor = |ps| ps_cycles(ps, CPU) as i32;
                assert!(realized >= period as i32 && realized >= floor(fl.period_ps));
                assert!(low >= floor(fl.low_ps) && high >= floor(fl.high_ps));
                // One spin fewer breaks the period or leaves a half below its floor.
                let spins = (realized - f.period) / 3;
                let halves = least(ps_cycles(fl.low_ps, CPU), f.low)
                    + least(ps_cycles(fl.high_ps, CPU), f.high);
                let short = (spins - 1) * 3 + f.period;
                assert!(
                    spins - 1 < halves || short < period.max(ps_cycles(fl.period_ps, CPU)) as i32,
                    "{request} Hz, {class:?}: {spins} spins"
                );
            }
        }
    }

    #[test]
    fn a_request_between_a_class_plateau_and_25_mhz_gets_the_plateau() {
        for class in &H523[..4] {
            let top = cell(25_000_000, *class).2 as u32;
            for request in (CPU.div_ceil(top)..=25_000_000).step_by(1_000) {
                assert_eq!(
                    cell(request, *class).2 as u32,
                    top,
                    "{request} Hz, {class:?}"
                );
            }
        }
        // 20 MHz: 14-cycle control and ACK cells, 13-cycle writes.
        let at = |i: usize| cell(20_000_000, H523[i]).2;
        assert_eq!([at(0), at(1), at(2), at(3)], [14, 14, 14, 13]);
    }
}
