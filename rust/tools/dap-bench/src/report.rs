//! Timing loop, table and JSON rendering, and the clock-sweep fit decomposition.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};

use crate::adi::Adiv5;
use crate::dap::{transfer_block_read_capacity, transfer_block_write_capacity};

/// Wall-clock budget a bare `--window` (or no flag) resolves to.
pub const DEFAULT_WINDOW_MS: u64 = 500;
/// Minimum wall-clock floor at the default window, alongside `MIN_ITERATIONS`.
const DEFAULT_MIN_TIME: Duration = Duration::from_millis(100);
/// Minimum iteration-count floor, alongside the time floor, at any window.
const MIN_ITERATIONS: u32 = 4;

/// SRAM write pattern: neither zero nor an erased-flash all-ones word, so the read-back
/// integrity check cannot pass by accident.
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
    /// Requested SWD clock, in Hz.
    pub clock_hz: u32,
    /// Achieved throughput, in KiB/s.
    pub kib_per_s: f64,
    /// Number of block transfers timed.
    pub iterations: u32,
    /// Total wall-clock time of the timed transfers, in milliseconds.
    pub elapsed_ms: f64,
    /// Effective pipeline depth used for this row's timed transfers (1 for lockstep).
    pub depth: u32,
}

/// The minimum wall-clock floor for a `budget`: the default 100 ms floor when `budget` is at
/// least the default 500 ms window, else scaled down proportionally (never exceeding `budget`).
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

/// Read `words` from `addr` twice and require the two agree, so no step passes on a stuck or
/// corrupting bus.
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

/// Read twice at `depth`, requiring the two agree and not be all-zero. Running at the timed
/// loop's own depth is what catches a windowed submission's response misordering.
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

/// Read `buf` from `addr` repeatedly until [`stop`] and report the throughput. The caller runs
/// the integrity guard first; nothing is timed until it has been verified.
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

/// Write the fixed pattern at `depth` and read it back, so no write is timed before the path
/// has been verified at the depth the timed loop will use.
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

/// [`time_block_write`] behind [`integrity_guard_write`] at the same `depth`.
pub fn measure_block_write(
    adi: &mut Adiv5,
    addr: u32,
    words: usize,
    clock_hz: u32,
    window: Duration,
    depth: usize,
) -> Result<Step> {
    integrity_guard_write(adi, addr, words, depth)?;
    time_block_write(adi, addr, words, clock_hz, window, depth)
}

/// Write and time a fixed pattern at `addr` until [`stop`]. RAM is not saved: the caller's
/// end-of-run system reset re-initializes it.
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

// --- Repeats, aggregation, and aborts ----------------------------------------------------------

/// Repeats per rung a bare `--repeat` (or no flag) resolves to.
pub const DEFAULT_REPEATS: u32 = 32;

/// Which phase of one repeat failed. A repeat that aborts is counted, never retried.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AbortClass {
    /// The debug port never answered: attach failed before the core was touched.
    Attach,
    /// The core did not acknowledge the halt.
    Halt,
    /// The integrity guard failed: repeated reads disagreed, read all-zero, or a written
    /// pattern did not read back.
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

/// One rung measured `repeats` times: the samples that produced a number, and the attempts that
/// aborted. `samples.len() + aborts.len() == repeats` always -- an abort is never refilled.
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

    /// The sample the median names -- the lower of the two middles at even count -- so the
    /// iteration count and elapsed time reported alongside a median belong to one real run.
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

/// Median of `values`: the middle value, or the mean of the two middles at even count. `None`
/// for an empty series.
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

/// Column header for [`render`].
pub fn column_header() -> String {
    format!(
        "{:>5}  {:>10}  {:>9}  {:>9}  {:>9}  {:>5}  {:>6}  {:>5}",
        "dir", "clock_hz", "median", "min", "max", "n", "aborts", "depth"
    )
}

/// Render one [`RepeatSet`] as an aligned table row: the median with its full range, the number
/// of samples behind it, and the aborts that did not become samples.
pub fn render(set: &RepeatSet) -> String {
    let cell = |v: Option<f64>| v.map_or_else(|| "-".to_string(), |v| format!("{v:.1}"));
    let (lo, hi) = match set.range_kib_per_s() {
        Some((lo, hi)) => (Some(lo), Some(hi)),
        None => (None, None),
    };
    format!(
        "{:>5}  {:>10}  {:>9}  {:>9}  {:>9}  {:>5}  {:>6}  {:>5}",
        set.direction.label(),
        set.clock_hz,
        cell(set.median_kib_per_s()),
        cell(lo),
        cell(hi),
        set.samples.len(),
        set.aborts.len(),
        set.depth
    )
}

/// The per-class abort tally for a rung, e.g. `halt 2, transfer 1`; empty when none aborted.
pub fn abort_detail(set: &RepeatSet) -> String {
    set.abort_counts()
        .into_iter()
        .filter(|&(_, n)| n > 0)
        .map(|(c, n)| format!("{} {n}", c.label()))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Format seconds since the Unix epoch as ISO-8601 UTC (`YYYY-MM-DDTHH:MM:SSZ`).
pub fn format_iso8601_utc(secs_since_epoch: u64) -> String {
    let days = (secs_since_epoch / 86_400) as i64;
    let tod = secs_since_epoch % 86_400;
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        tod / 3600,
        (tod / 60) % 60,
        tod % 60
    )
}

/// Days since 1970-01-01 to a proleptic-Gregorian `(year, month, day)`, for non-negative `z`.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097) as u64; // day of era, [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // year of era
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // day of year, March-based
    let mp = (5 * doy + 2) / 153; // month index, March = 0
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe as i64 + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

/// Run-level context the JSON document states once.
pub struct RunContext<'a> {
    /// Run start time, ISO-8601 UTC.
    pub timestamp_utc: String,
    /// `DAP_Info` vendor string, when the probe exposes one.
    pub probe_vendor: Option<String>,
    /// Probe product string (`DAP_Info` value, falling back to the USB descriptor upstream).
    pub probe_product: Option<String>,
    /// Probe serial string (`DAP_Info` value, falling back to the USB descriptor upstream).
    pub probe_serial: Option<String>,
    /// `DAP_Info` firmware version string.
    pub firmware_version: Option<String>,
    /// `DAP_PACKET_SIZE`, in bytes.
    pub packet_size: Option<u16>,
    /// `DAP_PACKET_COUNT`, outstanding packets.
    pub packet_count: Option<u8>,
    /// Wire protocol label (`"swd"` or `"jtag"`).
    pub protocol: &'a str,
    /// Target profile name.
    pub target: &'a str,
    /// Measurement budget per rung, in milliseconds.
    pub window_ms: u64,
    /// Bytes moved per block-transfer iteration.
    pub chunk_bytes: usize,
    /// Repeats attempted per rung.
    pub repeats: u32,
    /// Requested clock sweep, in Hz, in sweep order.
    pub clocks_hz: &'a [u32],
    /// Pipeline-depth sweep, post-clamp, in sweep order.
    pub depths: &'a [usize],
}

/// Serialize a run to `schema_version` 3. A field is `null` only when the probe answered no
/// value for it, never because the query was skipped.
pub fn to_json(ctx: &RunContext, rows: &[RepeatSet], fit: &[FitRow]) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "{{");
    let _ = writeln!(out, "  \"schema_version\": 3,");
    let _ = writeln!(
        out,
        "  \"timestamp_utc\": {},",
        json_str(&ctx.timestamp_utc)
    );

    let _ = writeln!(out, "  \"probe\": {{");
    let _ = writeln!(out, "    \"vendor\": {},", json_opt_str(&ctx.probe_vendor));
    let _ = writeln!(
        out,
        "    \"product\": {},",
        json_opt_str(&ctx.probe_product)
    );
    let _ = writeln!(out, "    \"serial\": {},", json_opt_str(&ctx.probe_serial));
    let _ = writeln!(
        out,
        "    \"firmware_version\": {},",
        json_opt_str(&ctx.firmware_version)
    );
    let _ = writeln!(
        out,
        "    \"packet_size\": {},",
        json_opt_num(ctx.packet_size)
    );
    let _ = writeln!(
        out,
        "    \"packet_count\": {}",
        json_opt_num(ctx.packet_count)
    );
    let _ = writeln!(out, "  }},");

    let _ = writeln!(out, "  \"protocol\": {},", json_str(ctx.protocol));
    let _ = writeln!(out, "  \"target\": {},", json_str(ctx.target));
    let _ = writeln!(out, "  \"window_ms\": {},", ctx.window_ms);
    let _ = writeln!(out, "  \"chunk_bytes\": {},", ctx.chunk_bytes);
    let _ = writeln!(out, "  \"repeats\": {},", ctx.repeats);

    let _ = write!(out, "  \"clocks_hz\": [");
    for (i, hz) in ctx.clocks_hz.iter().enumerate() {
        let comma = if i + 1 < ctx.clocks_hz.len() {
            ", "
        } else {
            ""
        };
        let _ = write!(out, "{hz}{comma}");
    }
    let _ = writeln!(out, "],");

    let _ = write!(out, "  \"depths\": [");
    for (i, depth) in ctx.depths.iter().enumerate() {
        let comma = if i + 1 < ctx.depths.len() { ", " } else { "" };
        let _ = write!(out, "{depth}{comma}");
    }
    let _ = writeln!(out, "],");

    let _ = writeln!(out, "  \"rows\": [");
    for (i, row) in rows.iter().enumerate() {
        let comma = if i + 1 < rows.len() { "," } else { "" };
        let (lo, hi) = match row.range_kib_per_s() {
            Some((lo, hi)) => (Some(lo), Some(hi)),
            None => (None, None),
        };
        let _ = writeln!(out, "    {{");
        let _ = writeln!(
            out,
            "      \"direction\": {},",
            json_str(row.direction.label())
        );
        let _ = writeln!(out, "      \"clock_hz\": {},", row.clock_hz);
        let _ = writeln!(out, "      \"depth\": {},", row.depth);
        let _ = writeln!(out, "      \"repeats\": {},", row.repeats);
        let _ = writeln!(out, "      \"samples\": {},", row.samples.len());
        let _ = writeln!(out, "      \"aborts\": {},", row.aborts.len());
        let _ = write!(out, "      \"abort_classes\": {{");
        for (j, (class, n)) in row.abort_counts().into_iter().enumerate() {
            let comma = if j + 1 < AbortClass::ALL.len() {
                ", "
            } else {
                ""
            };
            let _ = write!(out, "{}: {n}{comma}", json_str(class.label()));
        }
        let _ = writeln!(out, "}},");
        let _ = writeln!(out, "      \"kib_per_s\": {{");
        let _ = writeln!(
            out,
            "        \"median\": {},",
            json_opt_f64(row.median_kib_per_s())
        );
        let _ = writeln!(out, "        \"min\": {},", json_opt_f64(lo));
        let _ = writeln!(out, "        \"max\": {}", json_opt_f64(hi));
        let _ = writeln!(out, "      }},");
        let _ = write!(out, "      \"samples_kib_per_s\": [");
        for (j, s) in row.samples.iter().enumerate() {
            let comma = if j + 1 < row.samples.len() { ", " } else { "" };
            let _ = write!(out, "{}{comma}", json_f64(s.kib_per_s));
        }
        let _ = writeln!(out, "],");
        let median_sample = row.median_sample();
        let _ = writeln!(
            out,
            "      \"median_iterations\": {},",
            json_opt_num(median_sample.map(|s| s.iterations))
        );
        let _ = writeln!(
            out,
            "      \"median_elapsed_ms\": {}",
            json_opt_f64(median_sample.map(|s| s.elapsed_ms))
        );
        let _ = writeln!(out, "    }}{comma}");
    }
    let _ = writeln!(out, "  ],");

    let _ = writeln!(out, "  \"fit\": [");
    for (i, f) in fit.iter().enumerate() {
        let comma = if i + 1 < fit.len() { "," } else { "" };
        let _ = writeln!(out, "    {{");
        let _ = writeln!(
            out,
            "      \"direction\": {},",
            json_str(f.direction.label())
        );
        let _ = writeln!(out, "      \"depth\": {},", f.depth);
        let _ = writeln!(out, "      \"block_words\": {},", f.block_words);
        let _ = writeln!(out, "      \"intercept_us\": {},", json_f64(f.intercept_us));
        let _ = writeln!(out, "      \"slope\": {},", json_f64(f.slope));
        let _ = writeln!(out, "      \"ideal_slope\": {},", json_f64(f.ideal_slope));
        let _ = writeln!(out, "      \"r_squared\": {},", json_f64(f.r_squared));
        let _ = writeln!(out, "      \"top_clock_hz\": {},", f.top_clock_hz);
        let _ = writeln!(
            out,
            "      \"effective_clock_hz_top\": {},",
            json_f64(f.effective_clock_hz_top)
        );
        let _ = writeln!(out, "      \"wire_us_top\": {},", json_f64(f.wire_us_top));
        let _ = writeln!(
            out,
            "      \"plateau_ceiling_hz\": {}",
            json_opt_num(f.plateau_ceiling_hz)
        );
        let _ = writeln!(out, "    }}{comma}");
    }
    let _ = writeln!(out, "  ]");
    let _ = write!(out, "}}");
    out.push('\n');
    out
}

/// Sweep-level context: [`RunContext`] minus the clock list a sweep discovers rather than
/// requests.
pub struct SweepContext<'a> {
    /// Run start time, ISO-8601 UTC.
    pub timestamp_utc: String,
    /// `DAP_Info` vendor string, when the probe exposes one.
    pub probe_vendor: Option<String>,
    /// Probe product string (`DAP_Info` value, falling back to the USB descriptor upstream).
    pub probe_product: Option<String>,
    /// Probe serial string (`DAP_Info` value, falling back to the USB descriptor upstream).
    pub probe_serial: Option<String>,
    /// `DAP_Info` firmware version string.
    pub firmware_version: Option<String>,
    /// `DAP_PACKET_SIZE`, in bytes.
    pub packet_size: Option<u16>,
    /// `DAP_PACKET_COUNT`, outstanding packets.
    pub packet_count: Option<u8>,
    /// Wire protocol label (`"swd"` or `"jtag"`).
    pub protocol: &'a str,
    /// Target profile name.
    pub target: &'a str,
    /// Measurement budget per rung, in milliseconds.
    pub window_ms: u64,
    /// Bytes moved per block-transfer iteration.
    pub chunk_bytes: usize,
}

/// Serialize a sweep to `schema_version` 3: [`to_json`]'s context plus a `"sweep"` object of
/// rows, `"plateau_hz"`, and `"failure_interval_hz"` (high end `null` if the ladder never failed).
pub fn sweep_to_json(
    ctx: &SweepContext,
    rows: &[crate::sweep::SweepRow],
    plateau_hz: Option<u32>,
    failure_interval_hz: (u32, Option<u32>),
) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "{{");
    let _ = writeln!(out, "  \"schema_version\": 3,");
    let _ = writeln!(
        out,
        "  \"timestamp_utc\": {},",
        json_str(&ctx.timestamp_utc)
    );

    let _ = writeln!(out, "  \"probe\": {{");
    let _ = writeln!(out, "    \"vendor\": {},", json_opt_str(&ctx.probe_vendor));
    let _ = writeln!(
        out,
        "    \"product\": {},",
        json_opt_str(&ctx.probe_product)
    );
    let _ = writeln!(out, "    \"serial\": {},", json_opt_str(&ctx.probe_serial));
    let _ = writeln!(
        out,
        "    \"firmware_version\": {},",
        json_opt_str(&ctx.firmware_version)
    );
    let _ = writeln!(
        out,
        "    \"packet_size\": {},",
        json_opt_num(ctx.packet_size)
    );
    let _ = writeln!(
        out,
        "    \"packet_count\": {}",
        json_opt_num(ctx.packet_count)
    );
    let _ = writeln!(out, "  }},");

    let _ = writeln!(out, "  \"protocol\": {},", json_str(ctx.protocol));
    let _ = writeln!(out, "  \"target\": {},", json_str(ctx.target));
    let _ = writeln!(out, "  \"window_ms\": {},", ctx.window_ms);
    let _ = writeln!(out, "  \"chunk_bytes\": {},", ctx.chunk_bytes);

    let _ = writeln!(out, "  \"sweep\": {{");
    let _ = writeln!(out, "    \"rows\": [");
    for (i, row) in rows.iter().enumerate() {
        let comma = if i + 1 < rows.len() { "," } else { "" };
        let verdict = match row.verdict {
            crate::sweep::Verdict::Pass => "pass",
            crate::sweep::Verdict::Fail => "fail",
        };
        let _ = writeln!(out, "      {{");
        let _ = writeln!(out, "        \"clock_hz\": {},", row.clock_hz);
        let _ = writeln!(out, "        \"kib_per_s\": {},", json_f64(row.kib_per_s));
        let _ = writeln!(out, "        \"iterations\": {},", row.iterations);
        let _ = writeln!(out, "        \"verdict\": {},", json_str(verdict));
        let _ = writeln!(out, "        \"depth\": {}", row.depth);
        let _ = writeln!(out, "      }}{comma}");
    }
    let _ = writeln!(out, "    ],");
    let _ = writeln!(out, "    \"plateau_hz\": {},", json_opt_num(plateau_hz));
    let _ = writeln!(
        out,
        "    \"failure_interval_hz\": [{}, {}]",
        failure_interval_hz.0,
        json_opt_num(failure_interval_hz.1)
    );
    let _ = writeln!(out, "  }}");
    let _ = write!(out, "}}");
    out.push('\n');
    out
}

/// Quote and escape a string as a JSON string literal.
fn json_str(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(o, "\\u{:04x}", c as u32);
            }
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

/// A JSON string literal, or `null` when the option is empty.
fn json_opt_str(s: &Option<String>) -> String {
    s.as_deref().map_or_else(|| "null".to_string(), json_str)
}

/// A JSON number, or `null` when the option is empty.
fn json_opt_num<T: std::fmt::Display>(v: Option<T>) -> String {
    v.map_or_else(|| "null".to_string(), |n| n.to_string())
}

/// A finite JSON number, or `null` for an absent or non-finite value.
fn json_opt_f64(x: Option<f64>) -> String {
    x.map_or_else(|| "null".to_string(), json_f64)
}

/// A finite JSON number, or `null` for a non-finite value (JSON has no NaN/Infinity).
fn json_f64(x: f64) -> String {
    if x.is_finite() {
        x.to_string()
    } else {
        "null".to_string()
    }
}

// --- Clock-sweep fit decomposition -------------------------------------------------------------

/// Approximate SWCLK cycles one 32-bit SWD transfer costs: 8 request, two turnarounds, 3 ACK,
/// 32 data, 1 parity, a couple idle.
pub const SWD_CYCLES_PER_WORD: f64 = 46.0;

/// Approximate TCK cycles one 32-bit DPACC/APACC transfer costs over JTAG.
pub const JTAG_CYCLES_PER_WORD: f64 = 41.0;

/// Per-block time within this fraction of the sweep's best counts as "no longer dropping".
const WIRE_PLATEAU_TOLERANCE: f64 = 0.03;

/// A least-squares straight-line fit `y = intercept + slope * x`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearFit {
    /// The `y`-intercept (the `x = 0` value; for the throughput model, the clock-independent
    /// per-block cost).
    pub intercept: f64,
    /// The line's slope.
    pub slope: f64,
    /// Coefficient of determination `1 - SS_res / SS_tot`; `1.0` when every `y` is identical.
    pub r_squared: f64,
}

/// Least-squares fit of `y = intercept + slope * x`; `None` with fewer than two points or when
/// every `x` is identical.
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

/// The lowest clock whose `t_block` is already within [`WIRE_PLATEAU_TOLERANCE`] of the sweep's
/// minimum; `None` unless that clock is strictly below the top swept one.
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
        // Best t_block at (or tied at) the top clock: still wire-bound, no plateau.
        None
    }
}

/// One `(direction, depth)` group's per-block time fit as `T_block = a + b * (1/f)`, a "block"
/// being one `DAP_TransferBlock` at the negotiated capacity. A plateau breaks the linear model.
#[derive(Clone, Debug, PartialEq)]
pub struct FitRow {
    /// Transfer direction of the fitted group.
    pub direction: Direction,
    /// Pipeline depth of the fitted group (1 for lockstep).
    pub depth: u32,
    /// Per-packet `DAP_TransferBlock` capacity at the negotiated packet size, for this direction.
    pub block_words: usize,
    /// Intercept `a` in microseconds: the clock-independent per-block cost.
    pub intercept_us: f64,
    /// Slope `b` (in cycle units): the wire cost coefficient of `1/f`.
    pub slope: f64,
    /// The ideal slope for a clock-honoring probe (`block_words * cycles_per_word`).
    pub ideal_slope: f64,
    /// Goodness of fit (see [`LinearFit::r_squared`]).
    pub r_squared: f64,
    /// The top swept clock, in Hz (where the effective clock and wire cost are reported).
    pub top_clock_hz: u32,
    /// Achieved clock at the top swept one (`top_clock * ideal_slope / slope`); meaningless under
    /// a plateau.
    pub effective_clock_hz_top: f64,
    /// The wire cost at the top swept clock, in microseconds (`slope / top_clock`).
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

/// Decompose `rows` into a [`FitRow`] per `(direction, depth)` group with two or more usable
/// clocks; `cycles_per_word` is the wire protocol's constant.
pub fn compute_fits(
    rows: &[Step],
    chunk_bytes: usize,
    packet_size: u16,
    cycles_per_word: f64,
) -> Vec<FitRow> {
    // BTreeMap keeps the summary ordered regardless of the run order.
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
        // b approximates the ideal slope when the probe honors the clock, so the ratio recovers
        // the achieved clock and b/f is the wire time there.
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

    #[test]
    fn format_iso8601_utc_matches_a_known_instant() {
        // 2024-01-02T03:04:05Z.
        assert_eq!(format_iso8601_utc(1_704_164_645), "2024-01-02T03:04:05Z");
        assert_eq!(format_iso8601_utc(0), "1970-01-01T00:00:00Z");
    }

    #[test]
    fn to_json_emits_the_versioned_schema() {
        let ctx = RunContext {
            timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
            probe_vendor: Some("ARM".to_string()),
            probe_product: Some("DAPLink".to_string()),
            probe_serial: None,
            firmware_version: Some("0260".to_string()),
            packet_size: Some(512),
            packet_count: Some(4),
            protocol: "swd",
            target: "stm32",
            window_ms: 500,
            chunk_bytes: 4096,
            repeats: 3,
            clocks_hz: &[1_000_000, 2_000_000],
            depths: &[1],
        };
        let rows = [
            repeat_set(Direction::Read, 1_000_000, &[100.0, 123.4, 150.0], &[]),
            repeat_set(
                Direction::Read,
                2_000_000,
                &[234.5, 200.0],
                &[AbortClass::Halt],
            ),
        ];
        let json = to_json(&ctx, &rows, &[]);

        assert!(json.starts_with("{\n  \"schema_version\": 3,\n"));
        assert!(json.contains("\"timestamp_utc\": \"2024-01-02T03:04:05Z\""));
        assert!(json.contains("\"vendor\": \"ARM\""));
        assert!(json.contains("\"serial\": null"));
        assert!(json.contains("\"protocol\": \"swd\""));
        assert!(json.contains("\"repeats\": 3,"));
        assert!(json.contains("\"clocks_hz\": [1000000, 2000000]"));
        assert!(json.contains("\"depths\": [1]"));
        assert!(json.contains("\"direction\": \"read\""));
        assert!(json.contains("\"median\": 123.4"));
        assert!(json.contains("\"min\": 100"));
        assert!(json.contains("\"max\": 150"));
        assert!(json.contains("\"samples_kib_per_s\": [100, 123.4, 150]"));
        assert!(json.contains("\"samples\": 2,"));
        assert!(json.contains("\"aborts\": 1,"));
        assert!(json.contains(
            "\"abort_classes\": {\"attach\": 0, \"halt\": 1, \"guard\": 0, \"transfer\": 0}"
        ));
        assert!(json.contains("\"depth\": 1"));
        assert!(json.contains("\"fit\": [\n  ]"));
        assert!(json.ends_with("}\n"));

        // The DAP_Info fields nest under "probe" and serialize as values, never as null.
        let probe_block = json
            .split("\"probe\": {")
            .nth(1)
            .and_then(|s| s.split("},").next())
            .expect("probe object present");
        assert!(probe_block.contains("\"firmware_version\": \"0260\""));
        assert!(probe_block.contains("\"packet_size\": 512"));
        assert!(probe_block.contains("\"packet_count\": 4"));
    }

    #[test]
    fn sweep_to_json_emits_the_nested_sweep_object() {
        let ctx = SweepContext {
            timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
            probe_vendor: None,
            probe_product: Some("DAPLink".to_string()),
            probe_serial: None,
            firmware_version: None,
            packet_size: None,
            packet_count: None,
            protocol: "swd",
            target: "stm32",
            window_ms: 500,
            chunk_bytes: 4096,
        };
        let rows = [
            crate::sweep::SweepRow {
                clock_hz: 1_000_000,
                kib_per_s: 123.4,
                iterations: 10,
                verdict: crate::sweep::Verdict::Pass,
                depth: 1,
            },
            crate::sweep::SweepRow {
                clock_hz: 2_000_000,
                kib_per_s: 0.0,
                iterations: 0,
                verdict: crate::sweep::Verdict::Fail,
                depth: 1,
            },
        ];
        let json = sweep_to_json(&ctx, &rows, Some(1_000_000), (500_000, Some(2_000_000)));

        assert!(json.starts_with("{\n  \"schema_version\": 3,\n"));
        assert!(json.contains("\"sweep\": {"));
        assert!(json.contains("\"clock_hz\": 1000000"));
        assert!(json.contains("\"verdict\": \"pass\""));
        assert!(json.contains("\"verdict\": \"fail\""));
        assert!(json.contains("\"plateau_hz\": 1000000"));
        assert!(json.contains("\"failure_interval_hz\": [500000, 2000000]"));
        assert!(json.contains("\"depth\": 1"));
        assert!(json.ends_with("}\n"));
    }

    #[test]
    fn sweep_to_json_emits_null_plateau_and_open_failure_interval() {
        let ctx = SweepContext {
            timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
            probe_vendor: None,
            probe_product: None,
            probe_serial: None,
            firmware_version: None,
            packet_size: None,
            packet_count: None,
            protocol: "swd",
            target: "stm32",
            window_ms: 500,
            chunk_bytes: 4096,
        };
        let json = sweep_to_json(&ctx, &[], None, (32_000_000, None));
        assert!(json.contains("\"plateau_hz\": null"));
        assert!(json.contains("\"failure_interval_hz\": [32000000, null]"));
        assert!(json.contains("\"rows\": [\n    ]"));
    }

    #[test]
    fn to_json_escapes_string_fields() {
        let ctx = RunContext {
            timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
            probe_vendor: None,
            probe_product: Some("quote\"here".to_string()),
            probe_serial: None,
            firmware_version: None,
            packet_size: None,
            packet_count: None,
            protocol: "swd",
            target: "stm32",
            window_ms: 500,
            chunk_bytes: 4096,
            repeats: 1,
            clocks_hz: &[],
            depths: &[],
        };
        let json = to_json(&ctx, &[], &[]);
        assert!(json.contains("\"product\": \"quote\\\"here\""));
        assert!(json.contains("\"clocks_hz\": []"));
        assert!(json.contains("\"rows\": [\n  ]"));
    }

    // -- Least-squares fit --------------------------------------------------------------------

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
        // t_block stops dropping meaningfully after the second clock.
        let t_block = [10.0e-6, 5.0e-6, 4.98e-6, 4.99e-6];
        assert_eq!(detect_wire_plateau(&clocks, &t_block), Some(2_000_000));
    }

    #[test]
    fn detect_wire_plateau_none_when_still_dropping() {
        let clocks = [1_000_000, 2_000_000, 4_000_000];
        let t_block = [10.0e-6, 5.0e-6, 2.5e-6];
        assert_eq!(detect_wire_plateau(&clocks, &t_block), None);
    }

    // -- Fit decomposition ---------------------------------------------------------------------

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

    /// A [`RepeatSet`] carrying the given throughputs and aborts; the per-sample iteration and
    /// elapsed values are made distinct so a test can tell which sample was picked.
    fn repeat_set(
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
        // The eight rp2040-pico reads on record, whose median the tool's page states as ~104.
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

    #[test]
    fn aborts_are_counted_by_class_and_never_refill_the_set() {
        let set = repeat_set(
            Direction::Read,
            1_000_000,
            &[100.0, 120.0],
            &[AbortClass::Halt, AbortClass::Halt, AbortClass::Transfer],
        );
        assert_eq!(set.samples.len() + set.aborts.len(), set.repeats as usize);
        assert_eq!(
            set.abort_counts(),
            vec![
                (AbortClass::Attach, 0),
                (AbortClass::Halt, 2),
                (AbortClass::Guard, 0),
                (AbortClass::Transfer, 1),
            ]
        );
        assert_eq!(abort_detail(&set), "halt 2, transfer 1");
    }

    #[test]
    fn abort_detail_is_empty_when_every_repeat_produced_a_number() {
        let set = repeat_set(Direction::Read, 1_000_000, &[100.0], &[]);
        assert_eq!(abort_detail(&set), "");
    }

    #[test]
    fn render_shows_the_median_with_its_full_range_and_the_abort_count() {
        let set = repeat_set(
            Direction::Read,
            25_000_000,
            &[108.6, 99.2, 114.0, 122.4, 96.3, 107.7, 80.5, 99.4],
            &[AbortClass::Halt],
        );
        let row = render(&set);
        assert!(row.contains("103.6"), "{row}");
        assert!(row.contains("80.5"), "{row}");
        assert!(row.contains("122.4"), "{row}");
        assert_eq!(
            row.split_whitespace().collect::<Vec<_>>(),
            vec!["read", "25000000", "103.6", "80.5", "122.4", "8", "1", "1"]
        );
    }

    #[test]
    fn render_marks_an_all_abort_rung_rather_than_printing_a_number() {
        let set = repeat_set(Direction::Write, 1_000_000, &[], &[AbortClass::Attach]);
        let row = render(&set);
        assert_eq!(
            row.split_whitespace().collect::<Vec<_>>(),
            vec!["write", "1000000", "-", "-", "-", "0", "1", "1"]
        );
    }
}
