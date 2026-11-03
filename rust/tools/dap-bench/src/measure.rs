//! The timed loop: integrity guards, the floors that end a window, and repeat statistics.

use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};

use crate::adi::Adiv5;

/// Wall-clock budget a bare `--window` (or no flag) resolves to.
pub const DEFAULT_WINDOW_MS: u64 = 500;
/// Minimum wall-clock floor at the default window, alongside `MIN_ITERATIONS`.
const DEFAULT_MIN_TIME: Duration = Duration::from_millis(100);
/// Minimum iteration-count floor, alongside the time floor, at any window.
const MIN_ITERATIONS: u32 = 4;

/// SRAM write pattern: neither zero nor erased-flash ones, so read-back cannot pass by accident.
const WRITE_PATTERN_WORD: u32 = 0x5AA5_5AA5;

/// Which direction a [`Step`] measured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Block reads.
    Read,
    /// Block writes; the end-of-run system reset re-initializes RAM afterward.
    Write,
}

impl Direction {
    /// Short label used in the table's `dir` column.
    pub const fn label(self) -> &'static str {
        match self {
            Direction::Read => "read",
            Direction::Write => "write",
        }
    }
}

/// One measured run: direction, requested clock, achieved throughput, iteration count.
pub struct Step {
    /// Direction measured.
    pub direction: Direction,
    /// Requested SWD or JTAG clock, in Hz.
    pub clock_hz: u32,
    /// Achieved throughput, in KiB/s.
    pub kib_per_s: f64,
    /// Number of whole buffer transfers timed, not individual protocol commands.
    pub iterations: u32,
    /// Total wall-clock time of the timed transfers, in milliseconds.
    pub elapsed_ms: f64,
    /// Effective pipeline depth used for this row's timed transfers (1 for lockstep).
    pub depth: u32,
}

/// The minimum wall-clock floor for `budget`: 100 ms, scaled down for a shorter window.
fn min_time_for_budget(budget: Duration) -> Duration {
    let default_budget = Duration::from_millis(DEFAULT_WINDOW_MS);
    if budget >= default_budget {
        return DEFAULT_MIN_TIME;
    }
    let ratio = budget.as_secs_f64() / default_budget.as_secs_f64();
    DEFAULT_MIN_TIME.mul_f64(ratio).min(budget)
}

/// Whether measurement may stop: the budget has elapsed and both floors are met.
fn stop(elapsed: Duration, iterations: u32, budget: Duration, min_time: Duration) -> bool {
    elapsed >= budget && elapsed >= min_time && iterations >= MIN_ITERATIONS
}

/// Read `words` from `addr` twice and require the two agree.
pub fn read_twice_and_compare(adi: &mut Adiv5, addr: u32, words: usize) -> Result<Vec<u32>> {
    let mut a = vec![0u32; words];
    let mut b = vec![0u32; words];
    adi.read_block(addr, &mut a)
        .context("read integrity guard: first read failed")?;
    adi.read_block(addr, &mut b)
        .context("read integrity guard: second read failed")?;
    if a != b {
        bail!("read integrity guard failed at 0x{addr:08X}: repeated reads disagree");
    }
    Ok(a)
}

/// Read twice at the timed loop's `depth`, requiring agreement and non-zero data.
pub fn integrity_guard_read(adi: &mut Adiv5, addr: u32, words: usize, depth: usize) -> Result<()> {
    let mut a = vec![0u32; words];
    let mut b = vec![0u32; words];
    adi.read_block_depth(addr, &mut a, depth)
        .context("read integrity guard: first read failed")?;
    adi.read_block_depth(addr, &mut b, depth)
        .context("read integrity guard: second read failed")?;
    if a != b {
        bail!("read integrity guard failed at 0x{addr:08X}: repeated reads disagree");
    }
    if a.iter().all(|&w| w == 0) {
        bail!("read integrity guard failed at 0x{addr:08X}: region reads all-zero");
    }
    Ok(())
}

/// [`time_block_read`] behind [`integrity_guard_read`] at the same `depth`.
pub fn measure_block_read(
    adi: &mut Adiv5,
    addr: u32,
    buf: &mut [u32],
    clock_hz: u32,
    window: Duration,
    depth: usize,
) -> Result<Step> {
    integrity_guard_read(adi, addr, buf.len(), depth)?;
    time_block_read(adi, addr, buf, clock_hz, window, depth)
}

/// Time block reads until `window` and both floors, a time floor and four iterations, are met.
/// Run [`integrity_guard_read`] first: the timed loop checks no data.
pub fn time_block_read(
    adi: &mut Adiv5,
    addr: u32,
    buf: &mut [u32],
    clock_hz: u32,
    window: Duration,
    depth: usize,
) -> Result<Step> {
    let min_time = min_time_for_budget(window);
    let mut iterations: u32 = 0;
    let mut elapsed = Duration::ZERO;
    loop {
        let started = Instant::now();
        adi.read_block_depth(addr, buf, depth)?;
        elapsed += started.elapsed();
        iterations += 1;
        if stop(elapsed, iterations, window, min_time) {
            break;
        }
    }
    let bytes = (buf.len() * 4) as f64;
    let kib_per_s = (iterations as f64 * bytes / 1024.0) / elapsed.as_secs_f64();
    Ok(Step {
        direction: Direction::Read,
        clock_hz,
        kib_per_s,
        iterations,
        elapsed_ms: elapsed.as_secs_f64() * 1000.0,
        depth: depth.max(1) as u32,
    })
}

/// Write the fixed pattern at `depth` and read it back before any write is timed.
pub fn integrity_guard_write(adi: &mut Adiv5, addr: u32, words: usize, depth: usize) -> Result<()> {
    let pattern = vec![WRITE_PATTERN_WORD; words];
    adi.write_block_depth(addr, &pattern, depth)?;
    let mut check = vec![0u32; words];
    adi.read_block_depth(addr, &mut check, depth)?;
    if check != pattern {
        bail!("write integrity guard failed at 0x{addr:08X}: pattern read-back mismatch");
    }
    Ok(())
}

/// Time pattern writes until `window` and both floors, a time floor and four iterations, are met.
/// Run [`integrity_guard_write`] first, and reset the target afterward: the pattern overwrites RAM.
pub fn time_block_write(
    adi: &mut Adiv5,
    addr: u32,
    words: usize,
    clock_hz: u32,
    window: Duration,
    depth: usize,
) -> Result<Step> {
    let pattern = vec![WRITE_PATTERN_WORD; words];
    let min_time = min_time_for_budget(window);
    let mut iterations: u32 = 0;
    let mut elapsed = Duration::ZERO;
    loop {
        let started = Instant::now();
        adi.write_block_depth(addr, &pattern, depth)?;
        elapsed += started.elapsed();
        iterations += 1;
        if stop(elapsed, iterations, window, min_time) {
            break;
        }
    }
    let bytes = (words * 4) as f64;
    let kib_per_s = (iterations as f64 * bytes / 1024.0) / elapsed.as_secs_f64();
    Ok(Step {
        direction: Direction::Write,
        clock_hz,
        kib_per_s,
        iterations,
        elapsed_ms: elapsed.as_secs_f64() * 1000.0,
        depth: depth.max(1) as u32,
    })
}

/// Repeats per rung a bare `--repeat` (or no flag) resolves to.
pub const DEFAULT_REPEATS: u32 = 32;

/// Which phase of one repeat failed. A repeat that aborts is counted, never retried.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AbortClass {
    /// The debug port never answered: attach failed before the core was touched.
    Attach,
    /// The core did not acknowledge the halt.
    Halt,
    /// The integrity guard failed: reads disagreed, read all-zero, or the pattern did not read back.
    Guard,
    /// A transfer inside the timed loop failed.
    Transfer,
}

impl AbortClass {
    /// Short label used in the abort summary and in JSON.
    pub const fn label(self) -> &'static str {
        match self {
            AbortClass::Attach => "attach",
            AbortClass::Halt => "halt",
            AbortClass::Guard => "guard",
            AbortClass::Transfer => "transfer",
        }
    }

    /// Every class, in report order.
    pub const ALL: [AbortClass; 4] = [
        AbortClass::Attach,
        AbortClass::Halt,
        AbortClass::Guard,
        AbortClass::Transfer,
    ];
}

/// One rung measured `repeats` times; `samples.len() + aborts.len() == repeats`.
pub struct RepeatSet {
    /// Direction measured.
    pub direction: Direction,
    /// Requested clock, in Hz.
    pub clock_hz: u32,
    /// Effective pipeline depth (1 for lockstep).
    pub depth: u32,
    /// Attempts made, which is exactly `--repeat`.
    pub repeats: u32,
    /// The attempts that produced a number, in run order.
    pub samples: Vec<Step>,
    /// The attempts that did not, classified by the phase that failed.
    pub aborts: Vec<AbortClass>,
}

impl RepeatSet {
    /// Every sample's throughput, in run order.
    pub fn throughputs(&self) -> Vec<f64> {
        self.samples.iter().map(|s| s.kib_per_s).collect()
    }

    /// Median throughput, `None` when every repeat aborted.
    pub fn median_kib_per_s(&self) -> Option<f64> {
        median(&self.throughputs())
    }

    /// Lowest and highest sampled throughput, `None` when every repeat aborted.
    pub fn range_kib_per_s(&self) -> Option<(f64, f64)> {
        range(&self.throughputs())
    }

    /// The sample the median names (the lower middle at even count).
    pub fn median_sample(&self) -> Option<&Step> {
        if self.samples.is_empty() {
            return None;
        }
        let mut order: Vec<usize> = (0..self.samples.len()).collect();
        order.sort_by(|&a, &b| {
            self.samples[a]
                .kib_per_s
                .total_cmp(&self.samples[b].kib_per_s)
        });
        Some(&self.samples[order[(order.len() - 1) / 2]])
    }

    /// Abort count per class, in [`AbortClass::ALL`] order, zeroes included.
    pub fn abort_counts(&self) -> Vec<(AbortClass, u32)> {
        AbortClass::ALL
            .iter()
            .map(|&c| (c, self.aborts.iter().filter(|&&a| a == c).count() as u32))
            .collect()
    }
}

/// Median of `values`, the mean of the two middles at even count; `None` when empty.
pub fn median(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let mut v = values.to_vec();
    v.sort_by(f64::total_cmp);
    let mid = v.len() / 2;
    if v.len() % 2 == 1 {
        Some(v[mid])
    } else {
        Some((v[mid - 1] + v[mid]) / 2.0)
    }
}

/// Lowest and highest of `values`; `None` for an empty series.
pub fn range(values: &[f64]) -> Option<(f64, f64)> {
    let lo = values.iter().copied().reduce(f64::min)?;
    let hi = values.iter().copied().reduce(f64::max)?;
    Some((lo, hi))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[test]
    fn stop_requires_budget_and_both_floors() {
        let budget = Duration::from_millis(500);
        let min_time = min_time_for_budget(budget);
        assert!(!stop(
            Duration::from_millis(50),
            MIN_ITERATIONS,
            budget,
            min_time
        )); // under budget
        assert!(!stop(
            Duration::from_millis(600),
            MIN_ITERATIONS - 1,
            budget,
            min_time
        )); // under MIN_ITERATIONS
        assert!(stop(
            Duration::from_millis(600),
            MIN_ITERATIONS,
            budget,
            min_time
        )); // all satisfied
    }

    #[test]
    fn stop_keeps_reading_past_budget_for_floors() {
        let budget = Duration::from_millis(500);
        let min_time = min_time_for_budget(budget);
        // Budget met but MIN_ITERATIONS not yet reached: must keep going.
        assert!(!stop(budget, 1, budget, min_time));
        // Budget and MIN_TIME met, MIN_ITERATIONS now reached: may stop.
        assert!(stop(budget, MIN_ITERATIONS, budget, min_time));
    }

    #[test]
    fn min_time_for_budget_keeps_default_floor_at_or_above_default_window() {
        assert_eq!(
            min_time_for_budget(Duration::from_millis(500)),
            DEFAULT_MIN_TIME
        );
        assert_eq!(
            min_time_for_budget(Duration::from_millis(1000)),
            DEFAULT_MIN_TIME
        );
    }

    #[test]
    fn min_time_for_budget_scales_down_below_default_window() {
        // Half the default window scales the floor to half, and never exceeds the budget.
        let budget = Duration::from_millis(250);
        let min_time = min_time_for_budget(budget);
        assert_eq!(min_time, Duration::from_millis(50));
        assert!(min_time <= budget);

        // A window smaller than the default floor still keeps the floor within the budget.
        let tiny = Duration::from_millis(20);
        assert!(min_time_for_budget(tiny) <= tiny);
    }

    #[test]
    fn direction_label_matches_table_column() {
        assert_eq!(Direction::Read.label(), "read");
        assert_eq!(Direction::Write.label(), "write");
    }

    /// A [`RepeatSet`] with the given throughputs and aborts, each sample distinguishable.
    pub(crate) fn repeat_set(
        direction: Direction,
        clock_hz: u32,
        throughputs: &[f64],
        aborts: &[AbortClass],
    ) -> RepeatSet {
        RepeatSet {
            direction,
            clock_hz,
            depth: 1,
            repeats: (throughputs.len() + aborts.len()) as u32,
            samples: throughputs
                .iter()
                .enumerate()
                .map(|(i, &kib_per_s)| Step {
                    direction,
                    clock_hz,
                    kib_per_s,
                    iterations: i as u32 + 1,
                    elapsed_ms: 100.0 + i as f64,
                    depth: 1,
                })
                .collect(),
            aborts: aborts.to_vec(),
        }
    }

    #[test]
    fn median_of_an_odd_series_is_the_middle_value() {
        assert_eq!(median(&[3.0, 1.0, 2.0]), Some(2.0));
        assert_eq!(median(&[5.0]), Some(5.0));
        assert_eq!(median(&[1.0, 2.0, 3.0, 4.0, 5.0]), Some(3.0));
    }

    #[test]
    fn median_of_an_even_series_is_the_mean_of_the_two_middles() {
        assert_eq!(median(&[1.0, 2.0, 3.0, 4.0]), Some(2.5));
        let recorded = [108.6, 99.2, 114.0, 122.4, 96.3, 107.7, 80.5, 99.4];
        let m = median(&recorded).unwrap();
        assert!((m - 103.55).abs() < 1e-9, "median was {m}");
    }

    #[test]
    fn median_and_range_are_empty_for_an_empty_series() {
        assert_eq!(median(&[]), None);
        assert_eq!(range(&[]), None);
    }

    #[test]
    fn range_reports_the_full_spread_regardless_of_order() {
        let recorded = [108.6, 99.2, 114.0, 122.4, 96.3, 107.7, 80.5, 99.4];
        assert_eq!(range(&recorded), Some((80.5, 122.4)));
        assert_eq!(range(&[7.0]), Some((7.0, 7.0)));
    }

    #[test]
    fn median_sample_names_the_lower_middle_run_at_even_count() {
        // Sorted: 1.0, 2.0, 3.0, 4.0 -- the lower middle is 2.0, submitted second.
        let set = repeat_set(Direction::Read, 1_000_000, &[4.0, 2.0, 3.0, 1.0], &[]);
        let sample = set.median_sample().expect("samples present");
        assert_eq!(sample.kib_per_s, 2.0);
        assert_eq!(sample.iterations, 2);
    }

    #[test]
    fn median_sample_names_the_middle_run_at_odd_count() {
        let set = repeat_set(Direction::Read, 1_000_000, &[4.0, 2.0, 3.0], &[]);
        assert_eq!(set.median_sample().unwrap().kib_per_s, 3.0);
    }

    #[test]
    fn an_all_abort_set_has_no_median_no_range_and_no_sample() {
        let set = repeat_set(
            Direction::Read,
            1_000_000,
            &[],
            &[AbortClass::Attach, AbortClass::Halt],
        );
        assert_eq!(set.median_kib_per_s(), None);
        assert_eq!(set.range_kib_per_s(), None);
        assert!(set.median_sample().is_none());
        assert_eq!(set.repeats, 2);
    }
}
