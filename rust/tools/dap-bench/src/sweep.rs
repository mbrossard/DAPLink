//! Pure clock-sweep helpers: ladder generation, pass/fail interval narrowing, plateau detection.

/// Verdict of one sweep rung's cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Attach, guard, and verify all succeeded.
    Pass,
    /// Attach, guard, or verify failed, confirmed by a re-attach retry.
    Fail,
}

/// One measured sweep rung: requested clock, achieved throughput, iteration count, verdict.
pub struct SweepRow {
    /// Requested SWD/JTAG clock, in Hz.
    pub clock_hz: u32,
    /// Achieved throughput, in KiB/s; meaningless (0.0) on a failed rung.
    pub kib_per_s: f64,
    /// Number of block transfers timed; 0 on a failed rung.
    pub iterations: u32,
    /// Pass or fail.
    pub verdict: Verdict,
    /// Effective pipeline depth used for this rung's timed reads (1 for lockstep).
    pub depth: u32,
}

/// The ascending clock ladder from `start_hz` to `max_hz`, doubling each rung. The caller
/// validates `1 <= start_hz <= max_hz`.
pub fn geometric_ladder(start_hz: u32, max_hz: u32) -> Vec<u32> {
    let mut rungs = Vec::new();
    let mut hz = start_hz.min(max_hz);
    loop {
        rungs.push(hz);
        if hz == max_hz {
            break;
        }
        hz = hz.saturating_mul(2).min(max_hz);
    }
    rungs
}

/// Geometric midpoint of `(low, high)`, or `low` when no integer lies strictly between them.
pub fn geometric_midpoint(low: u32, high: u32) -> u32 {
    let mid = (f64::from(low) * f64::from(high)).sqrt().round() as u32;
    if mid > low && mid < high {
        mid
    } else {
        low
    }
}

/// Narrow a `(low, high)` bisection interval given whether the geometric midpoint passed.
pub fn narrow_interval(low: u32, high: u32, midpoint_passed: bool) -> (u32, u32) {
    let mid = geometric_midpoint(low, high);
    if midpoint_passed {
        (mid, high)
    } else {
        (low, mid)
    }
}

/// Plateau onset over passing rungs in ascending clock order: the first rung whose next two
/// improvements are both below 5%. `None` with fewer than 3 rows or a curve still climbing.
pub fn detect_plateau(passing_rows: &[(u32, f64)]) -> Option<u32> {
    const THRESHOLD: f64 = 0.05;
    let improvement = |prev: f64, cur: f64| {
        if prev <= 0.0 {
            0.0
        } else {
            (cur - prev) / prev
        }
    };

    if passing_rows.len() < 3 {
        return None;
    }
    for i in 0..passing_rows.len() - 2 {
        let step1 = improvement(passing_rows[i].1, passing_rows[i + 1].1);
        let step2 = improvement(passing_rows[i + 1].1, passing_rows[i + 2].1);
        if step1 < THRESHOLD && step2 < THRESHOLD {
            return Some(passing_rows[i].0);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometric_ladder_doubles_from_start_to_max() {
        assert_eq!(
            geometric_ladder(500_000, 32_000_000),
            vec![500_000, 1_000_000, 2_000_000, 4_000_000, 8_000_000, 16_000_000, 32_000_000]
        );
    }

    #[test]
    fn geometric_ladder_stops_immediately_when_start_equals_max() {
        assert_eq!(geometric_ladder(1_000_000, 1_000_000), vec![1_000_000]);
    }

    #[test]
    fn geometric_ladder_clamps_a_non_power_of_two_gap() {
        assert_eq!(
            geometric_ladder(1_000_000, 3_000_000),
            vec![1_000_000, 2_000_000, 3_000_000]
        );
    }

    #[test]
    fn geometric_midpoint_lands_between_the_bounds() {
        let mid = geometric_midpoint(1_000_000, 4_000_000);
        assert_eq!(mid, 2_000_000);
        assert!(mid > 1_000_000 && mid < 4_000_000);
    }

    #[test]
    fn geometric_midpoint_falls_back_to_low_when_adjacent() {
        assert_eq!(geometric_midpoint(1_000_000, 1_000_001), 1_000_000);
    }

    #[test]
    fn narrow_interval_raises_low_on_pass() {
        assert_eq!(
            narrow_interval(1_000_000, 4_000_000, true),
            (2_000_000, 4_000_000)
        );
    }

    #[test]
    fn narrow_interval_lowers_high_on_fail() {
        assert_eq!(
            narrow_interval(1_000_000, 4_000_000, false),
            (1_000_000, 2_000_000)
        );
    }

    #[test]
    fn detect_plateau_finds_the_first_flattening_pair() {
        let rows = [
            (1_000_000, 100.0),
            (2_000_000, 190.0),
            (4_000_000, 195.0),
            (8_000_000, 196.0),
        ];
        assert_eq!(detect_plateau(&rows), Some(2_000_000));
    }

    #[test]
    fn detect_plateau_none_when_still_climbing() {
        let rows = [(1_000_000, 100.0), (2_000_000, 200.0), (4_000_000, 400.0)];
        assert_eq!(detect_plateau(&rows), None);
    }

    #[test]
    fn detect_plateau_none_with_fewer_than_three_rows() {
        let rows = [(1_000_000, 100.0), (2_000_000, 101.0)];
        assert_eq!(detect_plateau(&rows), None);
    }

    #[test]
    fn detect_plateau_treats_nonpositive_prev_as_zero_improvement() {
        let rows = [
            (1_000_000, 0.0),
            (2_000_000, 0.0),
            (4_000_000, 0.0),
            (8_000_000, 0.0),
        ];
        assert_eq!(detect_plateau(&rows), Some(1_000_000));
    }
}
