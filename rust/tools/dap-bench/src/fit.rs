//! The clock-sweep fit: per-block time as a line in the reciprocal clock.

use std::collections::BTreeMap;

use crate::dap::{transfer_block_read_capacity, transfer_block_write_capacity};
use crate::measure::{Direction, Step};

/// Approximate payload-only SWCLK cycles per 32-bit SWD transfer.
pub const SWD_CYCLES_PER_WORD: f64 = 46.0;

/// Approximate TCK cycles one 32-bit DPACC/APACC transfer costs over JTAG.
pub const JTAG_CYCLES_PER_WORD: f64 = 41.0;

/// Per-block time within this fraction of the sweep's best counts as "no longer dropping".
const WIRE_PLATEAU_TOLERANCE: f64 = 0.03;

/// A least-squares straight-line fit `y = intercept + slope * x`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearFit {
    /// The `y`-intercept; for the throughput model, the clock-independent per-block cost.
    pub intercept: f64,
    /// The line's slope.
    pub slope: f64,
    /// Coefficient of determination `1 - SS_res / SS_tot`; `1.0` when every `y` is identical.
    pub r_squared: f64,
}

/// Least-squares fit of `y = intercept + slope * x`; `None` for fewer than two distinct `x`.
pub fn least_squares(points: &[(f64, f64)]) -> Option<LinearFit> {
    let n = points.len();
    if n < 2 {
        return None;
    }
    let nf = n as f64;
    let sx: f64 = points.iter().map(|p| p.0).sum();
    let sy: f64 = points.iter().map(|p| p.1).sum();
    let sxx: f64 = points.iter().map(|p| p.0 * p.0).sum();
    let sxy: f64 = points.iter().map(|p| p.0 * p.1).sum();
    let denom = nf * sxx - sx * sx;
    if denom == 0.0 {
        return None;
    }
    let slope = (nf * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / nf;

    let mean_y = sy / nf;
    let ss_tot: f64 = points.iter().map(|p| (p.1 - mean_y).powi(2)).sum();
    let ss_res: f64 = points
        .iter()
        .map(|p| (p.1 - (intercept + slope * p.0)).powi(2))
        .sum();
    let r_squared = if ss_tot == 0.0 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };
    Some(LinearFit {
        intercept,
        slope,
        r_squared,
    })
}

/// The lowest clock within [`WIRE_PLATEAU_TOLERANCE`] of the minimum `t_block`, if below the top.
fn detect_wire_plateau(clocks_hz: &[u32], t_block: &[f64]) -> Option<u32> {
    if clocks_hz.len() != t_block.len() || clocks_hz.len() < 2 {
        return None;
    }
    let mut pairs: Vec<(u32, f64)> = clocks_hz
        .iter()
        .copied()
        .zip(t_block.iter().copied())
        .filter(|&(_, t)| t.is_finite() && t > 0.0)
        .collect();
    if pairs.len() < 2 {
        return None;
    }
    pairs.sort_by_key(|&(c, _)| c);
    let min_t = pairs.iter().map(|&(_, t)| t).fold(f64::INFINITY, f64::min);
    if !min_t.is_finite() || min_t <= 0.0 {
        return None;
    }
    let tol = min_t * (1.0 + WIRE_PLATEAU_TOLERANCE);
    let ceiling_idx = pairs.iter().position(|&(_, t)| t <= tol)?;
    if ceiling_idx < pairs.len() - 1 {
        Some(pairs[ceiling_idx].0)
    } else {
        None
    }
}

/// One fixed workload's time fit per full-capacity-equivalent block, not per actual command.
/// Comparable only across matching workloads and host policies; it never measures the wire clock.
#[derive(Clone, Debug, PartialEq)]
pub struct FitRow {
    /// Transfer direction of the fitted group.
    pub direction: Direction,
    /// Pipeline depth of the fitted group (1 for lockstep).
    pub depth: u32,
    /// Per-packet `DAP_TransferBlock` capacity at the negotiated packet size, for this direction.
    pub block_words: usize,
    /// Intercept `a`, in microseconds per full-capacity-equivalent block.
    pub intercept_us: f64,
    /// Slope `b`, in cycle units per equivalent block; includes clock-dependent overhead.
    pub slope: f64,
    /// The ideal slope for a clock-honoring probe (`block_words * cycles_per_word`).
    pub ideal_slope: f64,
    /// Goodness of fit (see [`LinearFit::r_squared`]).
    pub r_squared: f64,
    /// The top swept clock, in Hz (where the effective clock and wire cost are reported).
    pub top_clock_hz: u32,
    /// Payload-model clock estimate in Hz, not a measured wire clock; invalid on a plateau.
    pub effective_clock_hz_top: f64,
    /// Fitted clock-dependent cost per equivalent block at the top clock, in microseconds.
    pub wire_us_top: f64,
    /// The clock beyond which `T_block` no longer drops; `None` while it is still dropping.
    pub plateau_ceiling_hz: Option<u32>,
}

/// The clock-sweep fit table's column-header line.
pub fn fit_column_header() -> String {
    format!(
        "{:<6}  {:>5}  {:>6}  {:>10}  {:>10}  {:>10}  {:>6}  {:>12}  {:>11}  plateau",
        "dir", "depth", "blk-w", "a-us", "b-cyc", "ideal-b", "R^2", "eff-clk-top", "wire-us-top",
    )
}

impl FitRow {
    /// Render this fit as one aligned line under [`fit_column_header`].
    pub fn render(&self) -> String {
        let plateau = self
            .plateau_ceiling_hz
            .map_or_else(|| "-".to_string(), |hz| format!("~{hz} Hz"));
        format!(
            "{:<6}  {:>5}  {:>6}  {:>10.3}  {:>10.1}  {:>10.1}  {:>6.3}  {:>12.0}  {:>11.3}  {}",
            self.direction.label(),
            self.depth,
            self.block_words,
            self.intercept_us,
            self.slope,
            self.ideal_slope,
            self.r_squared,
            self.effective_clock_hz_top,
            self.wire_us_top,
            plateau,
        )
    }
}

/// Fit groups from one fixed policy/target, time normalized per `block_words`.
pub fn compute_fits(
    rows: &[Step],
    chunk_bytes: usize,
    packet_size: u16,
    cycles_per_word: f64,
) -> Vec<FitRow> {
    type Key = (bool, u32);
    let mut groups: BTreeMap<Key, Vec<&Step>> = BTreeMap::new();
    for row in rows {
        let key = (row.direction == Direction::Write, row.depth);
        groups.entry(key).or_default().push(row);
    }

    let mut fits = Vec::new();
    for ((is_write, depth), grp) in groups {
        let direction = if is_write {
            Direction::Write
        } else {
            Direction::Read
        };
        let block_words = match direction {
            Direction::Read => transfer_block_read_capacity(packet_size),
            Direction::Write => transfer_block_write_capacity(packet_size),
        };
        if block_words == 0 {
            continue;
        }
        let blocks_per_row = (chunk_bytes / 4) as f64 / block_words as f64;
        if blocks_per_row <= 0.0 {
            continue;
        }

        let usable: Vec<&&Step> = grp
            .iter()
            .filter(|r| r.clock_hz > 0 && r.iterations > 0 && r.elapsed_ms > 0.0)
            .collect();
        if usable.len() < 2 {
            continue;
        }

        let mut points = Vec::with_capacity(usable.len());
        let mut clocks = Vec::with_capacity(usable.len());
        let mut tblocks = Vec::with_capacity(usable.len());
        for r in &usable {
            let secs_per_row = (r.elapsed_ms / 1000.0) / f64::from(r.iterations);
            let t_block = secs_per_row / blocks_per_row;
            points.push((1.0 / f64::from(r.clock_hz), t_block));
            clocks.push(r.clock_hz);
            tblocks.push(t_block);
        }
        if points.len() < 2 {
            continue;
        }
        let Some(fit) = least_squares(&points) else {
            continue;
        };
        let top_clock_hz = *clocks.iter().max().expect("at least two points");
        let ideal_slope = block_words as f64 * cycles_per_word;
        // Payload-only normalization cancels in this ratio; protocol and host costs do not.
        let effective_clock_hz_top = if fit.slope > 0.0 {
            f64::from(top_clock_hz) * ideal_slope / fit.slope
        } else {
            f64::INFINITY
        };
        let wire_us_top = fit.slope / f64::from(top_clock_hz) * 1e6;
        let plateau_ceiling_hz = detect_wire_plateau(&clocks, &tblocks);
        fits.push(FitRow {
            direction,
            depth,
            block_words,
            intercept_us: fit.intercept * 1e6,
            slope: fit.slope,
            ideal_slope,
            r_squared: fit.r_squared,
            top_clock_hz,
            effective_clock_hz_top,
            wire_us_top,
            plateau_ceiling_hz,
        });
    }
    fits
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adi::{TarPolicy, TransferShape};

    #[test]
    fn least_squares_recovers_an_exact_line() {
        let a = 3.0;
        let b = 2.0;
        let points: Vec<(f64, f64)> = (1..=5).map(|x| (x as f64, a + b * x as f64)).collect();
        let fit = least_squares(&points).unwrap();
        assert!(
            (fit.intercept - a).abs() < 1e-9,
            "intercept {}",
            fit.intercept
        );
        assert!((fit.slope - b).abs() < 1e-9, "slope {}", fit.slope);
        assert!((fit.r_squared - 1.0).abs() < 1e-9, "r^2 {}", fit.r_squared);
    }

    #[test]
    fn least_squares_needs_at_least_two_points() {
        assert!(least_squares(&[]).is_none());
        assert!(least_squares(&[(1.0, 1.0)]).is_none());
    }

    #[test]
    fn least_squares_rejects_a_vertical_line() {
        assert!(least_squares(&[(1.0, 1.0), (1.0, 2.0)]).is_none());
    }

    #[test]
    fn detect_wire_plateau_finds_a_genuine_plateau() {
        let clocks = [1_000_000, 2_000_000, 4_000_000, 8_000_000];
        let t_block = [10.0e-6, 5.0e-6, 4.98e-6, 4.99e-6];
        assert_eq!(detect_wire_plateau(&clocks, &t_block), Some(2_000_000));
    }

    #[test]
    fn detect_wire_plateau_none_when_still_dropping() {
        let clocks = [1_000_000, 2_000_000, 4_000_000];
        let t_block = [10.0e-6, 5.0e-6, 2.5e-6];
        assert_eq!(detect_wire_plateau(&clocks, &t_block), None);
    }

    fn fit_step(direction: Direction, clock_hz: u32, depth: u32, secs_per_row: f64) -> Step {
        // iterations=1 with elapsed_ms holding one row's time keeps the per-row math trivial.
        Step {
            direction,
            clock_hz,
            kib_per_s: 0.0,
            iterations: 1,
            elapsed_ms: secs_per_row * 1000.0,
            depth,
        }
    }

    #[test]
    fn fit_literal_payload_model_recovers_requested_clock() {
        let rows = [
            fit_step(Direction::Read, 1_000_000, 1, 0.048104),
            fit_step(Direction::Read, 2_000_000, 1, 0.024552),
            fit_step(Direction::Read, 4_000_000, 1, 0.012776),
        ];
        let fits = compute_fits(&rows, 4096, 64, 46.0);
        let f = &fits[0];
        assert_eq!(f.block_words, 15);
        assert!((f.intercept_us - 14.6484375).abs() < 1e-8);
        assert!((f.slope - 690.0).abs() < 1e-8);
        assert!((f.effective_clock_hz_top - 4_000_000.0).abs() < 1e-6);
        assert!((f.wire_us_top - 172.5).abs() < 1e-8);
    }

    #[test]
    fn fit_literal_mixed_write_overhead_is_not_a_physical_clock() {
        assert_eq!(TransferShape::Mixed.capacity(64, false), 11);
        assert_eq!(
            crate::adi::shaped_command_count(
                0,
                1024,
                11,
                TarPolicy::PerChunk,
                TransferShape::Mixed,
            ),
            96
        );
        let rows = [
            fit_step(Direction::Write, 1_000_000, 1, 0.053520),
            fit_step(Direction::Write, 2_000_000, 1, 0.027760),
            fit_step(Direction::Write, 4_000_000, 1, 0.014880),
        ];
        let fits = compute_fits(&rows, 4096, 64, 46.0);
        let f = &fits[0];
        assert_eq!(f.block_words, 14);
        assert!((f.intercept_us - 27.34375).abs() < 1e-8);
        assert!((f.slope - 704.375).abs() < 1e-8);
        assert!((f.effective_clock_hz_top - 3_657_142.857142857).abs() < 1e-6);
        assert!((f.wire_us_top - 176.09375).abs() < 1e-8);
    }

    #[test]
    fn fit_literal_per_window_changes_wire_overhead_not_clock() {
        assert_eq!(
            crate::adi::shaped_command_count(
                0,
                1024,
                15,
                TarPolicy::PerWindow,
                TransferShape::Block,
            ),
            76
        );
        let rows = [
            fit_step(Direction::Read, 1_000_000, 1, 0.048288),
            fit_step(Direction::Read, 2_000_000, 1, 0.024644),
            fit_step(Direction::Read, 4_000_000, 1, 0.012822),
        ];
        let fits = compute_fits(&rows, 4096, 64, 46.0);
        let f = &fits[0];
        assert!((f.slope - 692.6953125).abs() < 1e-8);
        assert!((f.effective_clock_hz_top - 3_984_435.7976653697).abs() < 1e-6);
    }

    #[test]
    fn compute_fits_recovers_the_per_block_model() {
        // packet_size=64 -> 15-word read blocks; synthesized so the fit must recover a and b.
        let packet_size = 64u16;
        let block_words = transfer_block_read_capacity(packet_size);
        let chunk_bytes = 4096usize;
        let blocks_per_row = (chunk_bytes / 4) as f64 / block_words as f64;
        let a_row = 500.0e-6; // 500 us clock-independent cost per row
        let b_block = 100.0; // wire cycles per block
        let clocks = [1_000_000u32, 2_000_000, 4_000_000, 8_000_000];
        let rows: Vec<Step> = clocks
            .iter()
            .map(|&hz| {
                let t_row = a_row + blocks_per_row * b_block / f64::from(hz);
                fit_step(Direction::Read, hz, 1, t_row)
            })
            .collect();

        let fits = compute_fits(&rows, chunk_bytes, packet_size, SWD_CYCLES_PER_WORD);
        assert_eq!(fits.len(), 1);
        let f = &fits[0];
        assert_eq!(f.direction, Direction::Read);
        assert_eq!(f.depth, 1);
        assert_eq!(f.block_words, block_words);
        let expected_intercept_us = (a_row / blocks_per_row) * 1e6;
        assert!(
            (f.intercept_us - expected_intercept_us).abs() < 1.0,
            "intercept {} (expected {expected_intercept_us})",
            f.intercept_us
        );
        assert!((f.slope - b_block).abs() < 1e-3, "slope {}", f.slope);
        assert!((f.r_squared - 1.0).abs() < 1e-6, "r^2 {}", f.r_squared);
        assert_eq!(f.top_clock_hz, 8_000_000);
        assert!(f.plateau_ceiling_hz.is_none());
    }

    #[test]
    fn compute_fits_groups_read_and_write_and_depth_separately() {
        let packet_size = 64u16;
        let chunk_bytes = 4096usize;
        let mut rows = Vec::new();
        for &hz in &[1_000_000u32, 2_000_000, 4_000_000] {
            rows.push(fit_step(
                Direction::Read,
                hz,
                1,
                1.0e-3 + 1.0 / f64::from(hz),
            ));
            rows.push(fit_step(
                Direction::Write,
                hz,
                1,
                1.0e-3 + 1.0 / f64::from(hz),
            ));
            rows.push(fit_step(
                Direction::Read,
                hz,
                4,
                5.0e-4 + 1.0 / f64::from(hz),
            ));
        }
        let fits = compute_fits(&rows, chunk_bytes, packet_size, SWD_CYCLES_PER_WORD);
        // Three distinct (direction, depth) groups, each with 3 usable clocks.
        assert_eq!(fits.len(), 3);
        let read_depth1 = fits
            .iter()
            .find(|f| f.direction == Direction::Read && f.depth == 1)
            .expect("read depth-1 group present");
        let read_depth4 = fits
            .iter()
            .find(|f| f.direction == Direction::Read && f.depth == 4)
            .expect("read depth-4 group present");
        let write_depth1 = fits
            .iter()
            .find(|f| f.direction == Direction::Write && f.depth == 1)
            .expect("write depth-1 group present");
        assert_ne!(read_depth1.intercept_us, read_depth4.intercept_us);
        assert_eq!(write_depth1.depth, 1);
    }

    #[test]
    fn compute_fits_skips_groups_with_fewer_than_two_clocks() {
        let rows = [fit_step(Direction::Read, 1_000_000, 1, 1.0e-3)];
        assert!(compute_fits(&rows, 4096, 64, SWD_CYCLES_PER_WORD).is_empty());
    }

    #[test]
    fn compute_fits_ignores_unusable_rows() {
        let mut rows = vec![
            fit_step(Direction::Read, 1_000_000, 1, 1.0e-3),
            fit_step(Direction::Read, 2_000_000, 1, 0.9e-3),
        ];
        // A zero-clock or zero-iteration row must never enter the fit (division by zero).
        rows.push(Step {
            direction: Direction::Read,
            clock_hz: 0,
            kib_per_s: 0.0,
            iterations: 1,
            elapsed_ms: 1.0,
            depth: 1,
        });
        let fits = compute_fits(&rows, 4096, 64, SWD_CYCLES_PER_WORD);
        assert_eq!(fits.len(), 1);
    }
}
