//! Wire-clock throughput sweep for the two speed cases: an instrument, never a gate.

use std::time::{Duration, Instant};

use probe_rs::probe::list::Lister;
use probe_rs::probe::{DebugProbeSelector, WireProtocol};
use probe_rs::{Core, MemoryInterface, Permissions, Session};

use crate::swd::{scratch, HALT_TIMEOUT, SCRATCH_BYTES};
use crate::util::chain;
use crate::{Ctx, Failure};

/// Requested wire clocks swept at each step, in Hz. Fixed so results compare across runs.
const CLOCKS_HZ: [u32; 4] = [1_000_000, 4_000_000, 12_000_000, 25_000_000];
/// The `--speed-extended` ladder, in Hz.
const EXTENDED_CLOCKS_HZ: [u32; 11] = [
    1_000_000, 2_000_000, 4_000_000, 8_000_000, 12_000_000, 16_000_000, 20_000_000, 25_000_000,
    32_000_000, 40_000_000, 50_000_000,
];
/// Bytes per read or write call unless `--speed-access` says otherwise.
const DEFAULT_ACCESS: usize = 4096;
/// Wall-clock budget per step.
const BUDGET: Duration = Duration::from_millis(500);
/// Wall-clock budget per repeat of an extended step.
const EXTENDED_BUDGET: Duration = Duration::from_millis(100);
/// Repeats per extended step, reported as a median with its full range.
const EXTENDED_REPEATS: u32 = 5;
/// Minimum wall-clock floor per step, alongside `MIN_CHUNKS`.
const MIN_TIME: Duration = Duration::from_millis(100);
/// Minimum chunk-count floor per step, alongside `MIN_TIME`.
const MIN_CHUNKS: u32 = 4;

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Options {
    pub(crate) extended: bool,
    /// Bytes per call, each swept in turn.
    pub(crate) access: Vec<usize>,
}

impl Options {
    /// `access` deduplicated in order, [`DEFAULT_ACCESS`] when empty.
    pub(crate) fn new(extended: bool, access: &[usize]) -> Self {
        let mut sizes = Vec::new();
        for &n in access {
            if !sizes.contains(&n) {
                sizes.push(n);
            }
        }
        if sizes.is_empty() {
            sizes.push(DEFAULT_ACCESS);
        }
        Options {
            extended,
            access: sizes,
        }
    }

    fn ladder(&self) -> &'static [u32] {
        if self.extended {
            &EXTENDED_CLOCKS_HZ
        } else {
            &CLOCKS_HZ
        }
    }

    fn repeats(&self) -> u32 {
        if self.extended {
            EXTENDED_REPEATS
        } else {
            1
        }
    }
}

impl Default for Options {
    fn default() -> Self {
        Options::new(false, &[])
    }
}

/// Parse one `--speed-access` value: whole words, at most the RAM scratch area.
pub(crate) fn parse_access(s: &str) -> Result<usize, String> {
    let n: usize = s
        .trim()
        .parse()
        .map_err(|_| format!("`{s}` is not a byte count"))?;
    if n == 0 || !n.is_multiple_of(4) || n > SCRATCH_BYTES {
        return Err(format!(
            "access size {n} must be a multiple of 4 from 4 to {SCRATCH_BYTES}"
        ));
    }
    Ok(n)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Direction {
    Read,
    Write,
}

impl Direction {
    pub(crate) fn label(self) -> &'static str {
        match self {
            Direction::Read => "read",
            Direction::Write => "write",
        }
    }
}

/// One repeat: timed calls and their summed elapsed time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Sample {
    pub(crate) calls: u32,
    pub(crate) ns: u128,
}

impl Sample {
    /// Throughput at `access` bytes per call; `None` for a zero-time sample.
    pub(crate) fn kib_per_s(&self, access: usize) -> Option<f64> {
        (self.ns > 0).then(|| {
            (self.calls as f64 * access as f64 / 1024.0) / (self.ns as f64 / 1_000_000_000.0)
        })
    }
}

/// One rung, direction and access size.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Row {
    pub(crate) requested_hz: u32,
    pub(crate) direction: Direction,
    pub(crate) access_bytes: usize,
    pub(crate) samples: Vec<Sample>,
    /// Raw per-call nanoseconds, so a clamped clock's equal-time signature survives; basic sweep only.
    pub(crate) chunk_ns: Vec<u128>,
    /// Why the rung failed or what it mismatched; its samples, if any, still stand.
    pub(crate) error: Option<String>,
}

impl Row {
    fn new(requested_hz: u32, direction: Direction, access_bytes: usize) -> Self {
        Row {
            requested_hz,
            direction,
            access_bytes,
            samples: Vec::new(),
            chunk_ns: Vec::new(),
            error: None,
        }
    }

    fn failed(requested_hz: u32, direction: Direction, access_bytes: usize, e: String) -> Self {
        Row {
            error: Some(e),
            ..Row::new(requested_hz, direction, access_bytes)
        }
    }

    fn throughputs(&self) -> Vec<f64> {
        self.samples
            .iter()
            .filter_map(|s| s.kib_per_s(self.access_bytes))
            .collect()
    }

    /// Median KiB/s with its min and max, `None` without samples.
    pub(crate) fn stats(&self) -> Option<(f64, f64, f64)> {
        let t = self.throughputs();
        let (lo, hi) = range(&t)?;
        Some((median(&t)?, lo, hi))
    }
}

pub(crate) struct Sweep {
    pub(crate) extended: bool,
    pub(crate) repeats: u32,
    pub(crate) rows: Vec<Row>,
}

fn median(values: &[f64]) -> Option<f64> {
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

fn range(values: &[f64]) -> Option<(f64, f64)> {
    let lo = values.iter().copied().reduce(f64::min)?;
    let hi = values.iter().copied().reduce(f64::max)?;
    Some((lo, hi))
}

/// Whether a step may stop: the budget has elapsed and both floors are met.
fn stop(budget: Duration, elapsed: Duration, chunks: u32) -> bool {
    elapsed >= budget && elapsed >= MIN_TIME && chunks >= MIN_CHUNKS
}

/// Run `op`, which times its own transfer, until [`stop`]; returns the sample and per-call times.
fn repeat_until<E>(
    budget: Duration,
    mut op: impl FnMut() -> Result<Duration, E>,
) -> Result<(Sample, Vec<u128>), E> {
    let mut calls = 0;
    let mut elapsed = Duration::ZERO;
    let mut chunk_ns = Vec::new();
    loop {
        let step = op()?;
        elapsed += step;
        chunk_ns.push(step.as_nanos());
        calls += 1;
        if stop(budget, elapsed, calls) {
            let ns = elapsed.as_nanos();
            return Ok((Sample { calls, ns }, chunk_ns));
        }
    }
}

/// Take the main session, sweep, stash the rows on `cx`, and always hand back a fresh SWD session.
pub(crate) fn run(
    cx: &mut Ctx,
    protocol: WireProtocol,
    before: impl FnOnce(&mut Session) -> Result<(), Failure>,
) -> Result<(), Failure> {
    let name = format!("speed-{}", crate::run_record::transport_name(protocol));
    let mut session = cx
        .session
        .take()
        .ok_or_else(|| Failure::Fatal(format!("{name} needs a successful connect")))?;
    let prepared = before(&mut session);
    drop(session);

    let result = prepared.and_then(|()| {
        let sweep = sweep(&cx.selector, &cx.chip, protocol, cx.read_addr, &cx.speed)?;
        print(&sweep);
        let verdict = verdict(&sweep);
        cx.last_speed = Some((protocol, sweep));
        verdict
    });

    match crate::jtag::reopen_swd(cx) {
        Ok(session) => cx.session = Some(session),
        Err(e) => return Err(Failure::Fatal(format!("re-open SWD: {e}"))),
    }
    result
}

/// An extended sweep fails when no rung moved any data or a rung that moved data mismatched.
fn verdict(sweep: &Sweep) -> Result<(), Failure> {
    if !sweep.extended {
        return Ok(());
    }
    if sweep.rows.iter().all(|r| r.samples.is_empty()) {
        return Err(Failure::Failed("no rung completed a transfer".into()));
    }
    if let Some(r) = sweep.rows.iter().find(|r| {
        !r.samples.is_empty()
            && r.error
                .as_deref()
                .is_some_and(|e| e.starts_with("mismatch"))
    }) {
        let e = r.error.as_deref().unwrap_or_default();
        return Err(Failure::Failed(format!("{} Hz: {e}", r.requested_hz)));
    }
    Ok(())
}

/// Sweep over `protocol`, one fresh session per step: probe-rs sets speed only before attach.
pub(crate) fn sweep(
    selector: &DebugProbeSelector,
    chip: &str,
    protocol: WireProtocol,
    read_addr: u64,
    opts: &Options,
) -> Result<Sweep, Failure> {
    let mut rows = Vec::new();
    for &hz in opts.ladder() {
        if opts.extended {
            rows.extend(extended_step(selector, chip, protocol, hz, &opts.access));
        } else {
            rows.extend(basic_step(
                selector,
                chip,
                protocol,
                hz,
                read_addr,
                &opts.access,
            )?);
        }
    }
    Ok(Sweep {
        extended: opts.extended,
        repeats: opts.repeats(),
        rows,
    })
}

fn attach(
    selector: &DebugProbeSelector,
    chip: &str,
    protocol: WireProtocol,
    hz: u32,
) -> Result<Session, Failure> {
    let mut probe = Lister::new()
        .open(selector.clone())
        .map_err(|e| Failure::Fatal(format!("open probe: {}", chain(e))))?;
    probe
        .select_protocol(protocol)
        .map_err(|e| Failure::Failed(format!("select protocol: {}", chain(e))))?;
    // Requests above the probe's internal delay clamp realize flat; that is expected.
    probe
        .set_speed(hz / 1000)
        .map_err(|e| Failure::Failed(format!("set speed: {}", chain(e))))?;
    probe
        .attach(chip, Permissions::default())
        .map_err(|e| Failure::Failed(format!("attach at {hz} Hz: {}", chain(e))))
}

/// Read `access`-byte blocks from `read_addr` for [`BUDGET`], once per access size.
fn basic_step(
    selector: &DebugProbeSelector,
    chip: &str,
    protocol: WireProtocol,
    hz: u32,
    read_addr: u64,
    access: &[usize],
) -> Result<Vec<Row>, Failure> {
    let mut session = attach(selector, chip, protocol, hz)?;
    let mut core = session
        .core(0)
        .map_err(|e| Failure::Fatal(format!("core(0) at {hz} Hz: {}", chain(e))))?;
    let mut rows = Vec::with_capacity(access.len());
    for &n in access {
        let mut buf = vec![0u32; n / 4];
        let (sample, chunk_ns) = repeat_until(BUDGET, || {
            let started = Instant::now();
            core.read_32(read_addr, &mut buf)
                .map_err(|e| Failure::Failed(format!("read at {hz} Hz: {}", chain(e))))?;
            Ok(started.elapsed())
        })?;
        rows.push(Row {
            requested_hz: hz,
            direction: Direction::Read,
            access_bytes: n,
            samples: vec![sample],
            chunk_ns,
            error: None,
        });
    }
    Ok(rows)
}

/// Halt, time pattern-checked writes and reads in the RAM scratch area, then restore; failures land on rows.
fn extended_step(
    selector: &DebugProbeSelector,
    chip: &str,
    protocol: WireProtocol,
    hz: u32,
    access: &[usize],
) -> Vec<Row> {
    let all_failed = |e: String| -> Vec<Row> {
        access
            .iter()
            .flat_map(|&n| {
                [Direction::Write, Direction::Read].map(|d| Row::failed(hz, d, n, e.clone()))
            })
            .collect()
    };
    let mut session = match attach(selector, chip, protocol, hz) {
        Ok(s) => s,
        Err(Failure::Failed(e) | Failure::Fatal(e) | Failure::Skip(e)) => return all_failed(e),
    };
    let mut core = match session.core(0) {
        Ok(c) => c,
        Err(e) => return all_failed(format!("core(0): {}", chain(e))),
    };
    let (addr, saved) = match backup(&mut core) {
        Ok(v) => v,
        Err(e) => {
            core.run().ok();
            return all_failed(e);
        }
    };
    let mut rows = Vec::with_capacity(access.len() * 2);
    for &n in access {
        rows.push(write_row(&mut core, hz, addr, n));
        rows.push(read_row(&mut core, hz, addr, n));
    }
    let restored = core
        .write_32(addr, &saved)
        .and_then(|()| core.run())
        .map_err(chain);
    if let Err(e) = restored {
        for r in &mut rows {
            r.error.get_or_insert_with(|| format!("restore: {e}"));
        }
    }
    rows
}

/// Halt and save the scratch area, read twice so a garbled read is never written back.
fn backup(core: &mut Core) -> Result<(u64, Vec<u32>), String> {
    let addr = scratch(core)?;
    core.halt(HALT_TIMEOUT)
        .map_err(|e| format!("halt: {}", chain(e)))?;
    let mut first = vec![0u32; SCRATCH_BYTES / 4];
    let mut second = vec![0u32; SCRATCH_BYTES / 4];
    core.read_32(addr, &mut first)
        .and_then(|()| core.read_32(addr, &mut second))
        .map_err(|e| format!("backup: {}", chain(e)))?;
    if first != second {
        return Err("backup: two reads of the scratch area disagree".into());
    }
    Ok((addr, first))
}

/// Pattern seed, distinct per clock, size and direction so stale data never passes.
fn seed(hz: u32, access: usize, direction: Direction) -> u32 {
    hz.wrapping_mul(0x9E37_79B9) ^ (access as u32).wrapping_mul(0x85EB_CA6B) ^ direction as u32
}

/// The scratch area's expected contents for `seed`.
fn pattern(seed: u32) -> Vec<u32> {
    (0..(SCRATCH_BYTES / 4) as u32)
        .map(|i| i.wrapping_mul(0x9E37_79B9) ^ seed)
        .collect()
}

/// The first word where `got` differs from `expected`, as an error.
fn mismatch(addr: u64, expected: &[u32], got: &[u32]) -> Option<String> {
    let i = expected.iter().zip(got).position(|(e, g)| e != g)?;
    Some(format!(
        "mismatch at {:#x}: expected {:#010x}, read {:#010x}",
        addr + 4 * i as u64,
        expected[i],
        got[i]
    ))
}

/// Byte offset of the `call`th access, cycling through the scratch area.
fn offset(call: usize, access: usize) -> usize {
    call % (SCRATCH_BYTES / access) * access
}

/// Timed writes of this step's pattern, then an untimed readback of what they covered.
fn write_row(core: &mut Core, hz: u32, addr: u64, n: usize) -> Row {
    let pattern = pattern(seed(hz, n, Direction::Write));
    let mut row = Row::new(hz, Direction::Write, n);
    let mut call = 0;
    for _ in 0..EXTENDED_REPEATS {
        let timed = repeat_until(EXTENDED_BUDGET, || {
            let at = offset(call, n);
            call += 1;
            let words = &pattern[at / 4..(at + n) / 4];
            let started = Instant::now();
            core.write_32(addr + at as u64, words).map_err(chain)?;
            Ok::<_, String>(started.elapsed())
        });
        match timed {
            Ok((sample, _)) => row.samples.push(sample),
            Err(e) => {
                row.error = Some(format!("write: {e}"));
                return row;
            }
        }
    }
    let covered = call.min(SCRATCH_BYTES / n) * n / 4;
    let mut back = vec![0u32; covered];
    row.error = match core.read_32(addr, &mut back) {
        Ok(()) => mismatch(addr, &pattern[..covered], &back),
        Err(e) => Some(format!("write readback: {}", chain(e))),
    };
    row
}

/// Write this step's pattern untimed, then time reads, comparing every block against it.
fn read_row(core: &mut Core, hz: u32, addr: u64, n: usize) -> Row {
    let pattern = pattern(seed(hz, n, Direction::Read));
    let mut row = Row::new(hz, Direction::Read, n);
    if let Err(e) = core.write_32(addr, &pattern) {
        row.error = Some(format!("pattern write: {}", chain(e)));
        return row;
    }
    let mut buf = vec![0u32; n / 4];
    let mut call = 0;
    let mut bad = None;
    for _ in 0..EXTENDED_REPEATS {
        let timed = repeat_until(EXTENDED_BUDGET, || {
            let at = offset(call, n);
            call += 1;
            let started = Instant::now();
            core.read_32(addr + at as u64, &mut buf).map_err(chain)?;
            let took = started.elapsed();
            if bad.is_none() {
                bad = mismatch(addr + at as u64, &pattern[at / 4..(at + n) / 4], &buf);
            }
            Ok::<_, String>(took)
        });
        match timed {
            Ok((sample, _)) => row.samples.push(sample),
            Err(e) => {
                row.error = Some(format!("read: {e}"));
                return row;
            }
        }
    }
    row.error = bad;
    row
}

fn header() -> String {
    format!(
        "    {:>5}  {:>10}  {:>6}  {:>9}  {:>9}  {:>9}  {:>4}  {:>6}",
        "dir", "clock_hz", "access", "med KiB/s", "min", "max", "reps", "calls"
    )
}

/// One row: median KiB/s with its range, repeats and total calls, then any failure.
fn render(r: &Row) -> String {
    let cell = |v: Option<f64>| v.map_or_else(|| "-".to_string(), |v| format!("{v:.1}"));
    let stats = r.stats();
    let mut line = format!(
        "    {:>5}  {:>10}  {:>6}  {:>9}  {:>9}  {:>9}  {:>4}  {:>6}",
        r.direction.label(),
        r.requested_hz,
        r.access_bytes,
        cell(stats.map(|s| s.0)),
        cell(stats.map(|s| s.1)),
        cell(stats.map(|s| s.2)),
        r.samples.len(),
        r.samples.iter().map(|s| s.calls).sum::<u32>()
    );
    if let Some(e) = &r.error {
        line.push_str(&format!("  FAILED: {e}"));
    }
    line
}

fn print(sweep: &Sweep) {
    println!("{}", header());
    for r in &sweep.rows {
        println!("{}", render(r));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_requires_budget_and_both_floors() {
        assert!(!stop(BUDGET, Duration::from_millis(50), MIN_CHUNKS)); // under budget
        assert!(!stop(BUDGET, Duration::from_millis(600), MIN_CHUNKS - 1)); // under MIN_CHUNKS
        assert!(stop(BUDGET, Duration::from_millis(600), MIN_CHUNKS)); // all satisfied
    }

    #[test]
    fn stop_keeps_reading_past_budget_for_floors() {
        assert!(!stop(BUDGET, BUDGET, 1));
        assert!(stop(BUDGET, BUDGET, MIN_CHUNKS));
        assert!(stop(EXTENDED_BUDGET, EXTENDED_BUDGET, MIN_CHUNKS));
    }

    #[test]
    fn repeat_until_counts_calls_and_sums_time() {
        let (s, ns) =
            repeat_until(EXTENDED_BUDGET, || Ok::<_, ()>(Duration::from_millis(30))).unwrap();
        assert_eq!(s.calls, 4);
        assert_eq!(s.ns, 120_000_000);
        assert_eq!(ns, vec![30_000_000; 4]);
    }

    #[test]
    fn the_default_run_keeps_the_basic_ladder_and_one_size() {
        let o = Options::default();
        assert_eq!(o.ladder(), &CLOCKS_HZ);
        assert_eq!(o.access, vec![DEFAULT_ACCESS]);
        assert_eq!(o.repeats(), 1);
    }

    #[test]
    fn the_extended_ladder_ascends_to_50_mhz_through_every_basic_rung() {
        let o = Options::new(true, &[]);
        assert_eq!(o.ladder(), &EXTENDED_CLOCKS_HZ);
        assert!(EXTENDED_CLOCKS_HZ.windows(2).all(|w| w[0] < w[1]));
        assert_eq!(EXTENDED_CLOCKS_HZ.last(), Some(&50_000_000));
        assert!(CLOCKS_HZ.iter().all(|hz| EXTENDED_CLOCKS_HZ.contains(hz)));
        assert_eq!(o.repeats(), EXTENDED_REPEATS);
    }

    #[test]
    fn access_sizes_keep_order_and_drop_repeats() {
        assert_eq!(
            Options::new(false, &[64, 4, 64, 4096]).access,
            vec![64, 4, 4096]
        );
    }

    #[test]
    fn access_sizes_are_whole_words_within_the_scratch_area() {
        assert_eq!(parse_access("4"), Ok(4));
        assert_eq!(parse_access("508"), Ok(508));
        assert_eq!(parse_access("4096"), Ok(4096));
        for bad in ["0", "6", "4100", "8192", "-4", "x"] {
            assert!(parse_access(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn median_and_range_follow_dap_bench() {
        assert_eq!(median(&[]), None);
        assert_eq!(median(&[3.0, 1.0, 2.0]), Some(2.0));
        assert_eq!(median(&[4.0, 1.0, 3.0, 2.0]), Some(2.5));
        assert_eq!(range(&[4.0, 1.0, 3.0]), Some((1.0, 4.0)));
        assert_eq!(range(&[]), None);
    }

    #[test]
    fn a_row_reports_median_min_and_max_in_kib_per_s() {
        let sample = |ms: u128| Sample {
            calls: 10,
            ns: ms * 1_000_000,
        };
        let row = Row {
            requested_hz: 1_000_000,
            direction: Direction::Write,
            access_bytes: 1024,
            samples: vec![sample(100), sample(50), sample(200)],
            chunk_ns: Vec::new(),
            error: None,
        };
        assert_eq!(row.stats(), Some((100.0, 50.0, 200.0)));
        let failed = Row::failed(1, Direction::Read, 4, "x".into());
        assert_eq!(failed.stats(), None);
        assert!(render(&failed).contains("FAILED: x"));
    }

    #[test]
    fn offsets_cycle_inside_the_scratch_area() {
        for n in [4, 64, 508, 4096] {
            for call in 0..3000 {
                assert!(offset(call, n) + n <= SCRATCH_BYTES, "{n} {call}");
                assert_eq!(offset(call, n) % 4, 0);
            }
        }
        assert_eq!(offset(8, 508), 0);
        assert_eq!(offset(7, 508), 7 * 508);
    }

    #[test]
    fn every_extended_step_writes_a_pattern_no_earlier_step_left() {
        let mut seen = std::collections::BTreeSet::new();
        for &hz in &EXTENDED_CLOCKS_HZ {
            for n in [4, 64, 508, 4096] {
                for d in [Direction::Write, Direction::Read] {
                    assert!(seen.insert(seed(hz, n, d)), "{hz} {n} {d:?}");
                }
            }
        }
    }

    #[test]
    fn mismatch_names_the_first_differing_word() {
        assert_eq!(mismatch(0x100, &[1, 2, 3], &[1, 2, 3]), None);
        let e = mismatch(0x100, &[1, 2, 3], &[1, 9, 7]).unwrap();
        assert!(e.contains("0x104"), "{e}");
    }

    #[test]
    fn only_an_extended_sweep_that_moved_nothing_fails() {
        let failed = vec![Row::failed(1, Direction::Read, 4, "x".into())];
        let sweep = |extended, rows| Sweep {
            extended,
            repeats: 1,
            rows,
        };
        assert!(verdict(&sweep(true, failed.clone())).is_err());
        assert!(verdict(&sweep(false, failed.clone())).is_ok());
        let mut some = failed;
        some.push(Row {
            samples: vec![Sample { calls: 1, ns: 1 }],
            ..Row::failed(2, Direction::Read, 4, "y".into())
        });
        assert!(verdict(&sweep(true, some.clone())).is_ok());
        some.push(Row {
            samples: vec![Sample { calls: 1, ns: 1 }],
            ..Row::failed(3, Direction::Write, 4, "mismatch at 0x0".into())
        });
        assert!(verdict(&sweep(true, some)).is_err());
    }
}
