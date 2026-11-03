//! JSONL run records: one line per measured run or window, plus a summary line per ladder.

use std::fs::OpenOptions;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Serialize, Serializer};
use serde_json::ser::{CharEscape, Formatter};
use serde_json::value::RawValue;

use daplink_test_protocol::v1::Direction;
use daplink_test_protocol::Stats;

use crate::budget::Clamp;
use crate::contract::CaseResult;
use crate::counters;
use crate::stream::{self, Leg, Run};

mod summary;
pub(crate) use summary::staged_summary;

pub const SCHEMA_VERSION: u32 = 15;

/// The image a record measured.
#[derive(Clone, Copy)]
pub struct Firmware<'a> {
    /// The firmware registry row, when anything names it.
    pub row: Option<&'a str>,
    /// What named it: `image`, `operator` or `none`.
    pub source: &'a str,
    /// The probe's `DAP_Info` firmware version.
    pub version: Option<&'a str>,
}

const DEFAULT_FILE_NAME: &str = "vcom-bench.jsonl";

const CHECKOUT_MARKER: &str = "rust/tools/vcom-bench/Cargo.toml";

pub struct RunRecord<'a> {
    pub probe: &'a str,
    pub firmware: Firmware<'a>,
    pub verb: &'a str,
    /// Which phase of the verb; `run` is the only one an echo has.
    pub phase: &'a str,
    /// The rate both ends were asked for.
    pub baud_nominal: u32,
    pub dut_realized: Option<u32>,
    /// The rate the probe reported programming, when its firmware serves the query.
    pub probe_realized: Option<u32>,
    pub len: u64,
    pub chunk: usize,
    /// `paced`, `batched`, or `out`.
    pub mode: &'a str,
    /// Always 0; an echo run is not a window.
    pub secs: f64,
    pub host_written: u64,
    pub host_read: u64,
    /// Whether the echo came back byte-exact and complete.
    pub byte_exact: bool,
    /// Offset of the first divergence, when not byte-exact.
    pub first_divergence: Option<u64>,
    pub throughput_bps: f64,
    /// The DUT counter delta over the run.
    pub dut_stats: Option<Stats>,
    pub dap_roundtrips: Option<u64>,
    /// The run's verdict under the caller's criterion, when the run had one.
    pub pass: Option<bool>,
    pub note: Option<&'a str>,
}

/// Prefer the checkout's artifacts directory, falling back to the executable directory.
pub fn default_path() -> PathBuf {
    let start = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(Path::to_path_buf))
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_else(|| PathBuf::from("."));
    for ancestor in start.ancestors() {
        if ancestor.join(CHECKOUT_MARKER).is_file() {
            return ancestor.join("artifacts").join(DEFAULT_FILE_NAME);
        }
    }
    start.join("artifacts").join(DEFAULT_FILE_NAME)
}

pub fn append(path: &Path, record: &RunRecord) -> std::io::Result<()> {
    append_line(path, &render(record, unix_now()))
}

/// Append a raw pre-rendered JSON line (the ladder summary uses this).
pub fn append_line(path: &Path, line: &str) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    file.write_all(line.as_bytes())?;
    file.write_all(b"\n")
}

/// Seconds since the epoch, 0 when the clock is unreadable.
pub fn unix_now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

pub fn append_contract(
    path: &Path,
    probe: &str,
    firmware: Firmware,
    result: &CaseResult,
) -> std::io::Result<()> {
    append_line(path, &render_contract(probe, firmware, result, unix_now()))
}

#[derive(Serialize)]
struct LatencyWire {
    samples: usize,
    p50_us: Option<u64>,
    p99_us: Option<u64>,
    max_us: Option<u64>,
}

#[derive(Serialize)]
struct ContractWire<'a> {
    schema_version: u32,
    tool: &'static str,
    unix_s: u64,
    probe: &'a str,
    firmware_row: Option<&'a str>,
    firmware_row_source: &'a str,
    firmware_version: Option<&'a str>,
    verb: &'static str,
    case: &'a str,
    tier: Option<&'a str>,
    baud_nominal: u32,
    latency: LatencyWire,
    byte_exact: bool,
    counter_classification: Option<&'static str>,
    dap_progress: bool,
    recovery_us: Option<u64>,
    pass: bool,
    failure_reason: Option<&'a str>,
}

pub fn render_contract(probe: &str, firmware: Firmware, r: &CaseResult, unix_s: u64) -> String {
    let wire = ContractWire {
        schema_version: SCHEMA_VERSION,
        tool: "vcom-bench",
        unix_s,
        probe,
        firmware_row: firmware.row,
        firmware_row_source: firmware.source,
        firmware_version: firmware.version,
        verb: "contract",
        case: r.case,
        tier: r.tier.as_deref(),
        baud_nominal: r.baud,
        latency: LatencyWire {
            samples: r.latencies.count(),
            p50_us: r.latencies.percentile_us(50),
            p99_us: r.latencies.percentile_us(99),
            max_us: r.latencies.max_us(),
        },
        byte_exact: r.exact,
        counter_classification: r.counter_classification.map(|c| c.key()),
        dap_progress: r.dap_progress,
        recovery_us: r
            .recovery
            .map(|d| d.as_micros().min(u128::from(u64::MAX)) as u64),
        pass: r.pass,
        failure_reason: r.failure.as_deref(),
    };
    to_json(&wire)
}

#[derive(Serialize)]
struct DutStatsWire {
    rx: u32,
    tx: u32,
    ferr: u32,
    ovr: u32,
    drop: u32,
    stuck: u32,
}

impl From<Stats> for DutStatsWire {
    fn from(s: Stats) -> Self {
        DutStatsWire {
            rx: s.rx,
            tx: s.tx,
            ferr: s.ferr,
            ovr: s.ovr,
            drop: s.drop,
            stuck: s.stuck,
        }
    }
}

#[derive(Serialize)]
struct RunRecordWire<'a> {
    schema_version: u32,
    tool: &'static str,
    unix_s: u64,
    probe: &'a str,
    firmware_row: Option<&'a str>,
    firmware_row_source: &'a str,
    firmware_version: Option<&'a str>,
    verb: &'a str,
    phase: &'a str,
    shape: &'static str,
    baud_nominal: u32,
    dut_realized: Option<u32>,
    probe_realized: Option<u32>,
    len: u64,
    chunk: usize,
    mode: &'a str,
    #[serde(serialize_with = "ser_bare")]
    secs: f64,
    host_written: u64,
    host_read: u64,
    byte_exact: bool,
    first_divergence: Option<u64>,
    #[serde(serialize_with = "ser_f1")]
    throughput_bps: f64,
    dut_stats: Option<DutStatsWire>,
    dap_roundtrips: Option<u64>,
    pass: Option<bool>,
    note: Option<&'a str>,
}

/// No trailing newline.
pub fn render(r: &RunRecord, unix_s: u64) -> String {
    let wire = RunRecordWire {
        schema_version: SCHEMA_VERSION,
        tool: "vcom-bench",
        unix_s,
        probe: r.probe,
        firmware_row: r.firmware.row,
        firmware_row_source: r.firmware.source,
        firmware_version: r.firmware.version,
        verb: r.verb,
        phase: r.phase,
        shape: "echo",
        baud_nominal: r.baud_nominal,
        dut_realized: r.dut_realized,
        probe_realized: r.probe_realized,
        len: r.len,
        chunk: r.chunk,
        mode: r.mode,
        secs: r.secs,
        host_written: r.host_written,
        host_read: r.host_read,
        byte_exact: r.byte_exact,
        first_divergence: r.first_divergence,
        throughput_bps: r.throughput_bps,
        dut_stats: r.dut_stats.map(DutStatsWire::from),
        dap_roundtrips: r.dap_roundtrips,
        pass: r.pass,
        note: r.note,
    };
    to_json(&wire)
}

pub struct V1Record<'a> {
    pub probe: &'a str,
    pub firmware: Firmware<'a>,
    pub verb: &'a str,
    /// The measurement phase, including unscored warmup windows.
    pub phase: &'a str,
    /// The rate both ends were asked for.
    pub baud_nominal: u32,
    pub probe_realized: Option<u32>,
    pub run: &'a Run,
    /// The aggregate-budget decision the point was admitted under.
    pub clamp: Option<Clamp>,
    pub note: Option<&'a str>,
}

pub fn append_v1(path: &Path, record: &V1Record) -> std::io::Result<()> {
    append_line(path, &render_v1(record, unix_now()))
}

#[derive(Serialize)]
struct SeedsWire {
    h2d: u32,
    d2h: u32,
}

#[derive(Serialize)]
struct BpsPairWire {
    #[serde(serialize_with = "ser_f1")]
    h2d: f64,
    #[serde(serialize_with = "ser_f1")]
    d2h: f64,
}

#[derive(Serialize)]
struct AbandonWire<'a> {
    #[serde(serialize_with = "ser_f3")]
    at_s: f64,
    checkpoint: usize,
    reason: &'a str,
}

#[derive(Serialize)]
struct TrendWire {
    host_to_dut: &'static str,
    dut_to_host: &'static str,
}

#[derive(Serialize)]
struct SliceWire {
    payload: u64,
    #[serde(serialize_with = "ser_f4")]
    frac: f64,
    corrupt: u64,
    missing: u64,
    duplicate: u64,
    unresolved: u64,
    resyncs: u32,
}

impl From<stream::Slice> for SliceWire {
    fn from(s: stream::Slice) -> Self {
        SliceWire {
            payload: s.payload,
            frac: s.frac,
            corrupt: s.corrupt,
            missing: s.missing,
            duplicate: s.duplicate,
            unresolved: s.unresolved,
            resyncs: s.resyncs,
        }
    }
}

#[derive(Serialize)]
struct PointWire {
    #[serde(serialize_with = "ser_f3")]
    at_s: f64,
    host_to_dut: Option<SliceWire>,
    dut_to_host: Option<SliceWire>,
}

#[derive(Serialize)]
struct CheckpointsWire {
    #[serde(serialize_with = "ser_bare")]
    period_s: f64,
    trend: TrendWire,
    points: Vec<PointWire>,
}

#[derive(Serialize)]
struct BurstWire {
    bytes: u32,
    idle_ms: u32,
    host: bool,
    dut: bool,
}

#[derive(Serialize)]
struct CreditWire {
    window: u64,
    granted: u64,
    obeyed: u64,
}

#[derive(Serialize)]
struct DapWire {
    profile: &'static str,
    commands: u64,
    bytes: u64,
    #[serde(serialize_with = "ser_f4")]
    secs: f64,
    #[serde(serialize_with = "ser_f1")]
    cmd_per_s: f64,
    #[serde(serialize_with = "ser_f1")]
    bytes_per_s: f64,
}

#[derive(Serialize)]
struct ClampWire {
    #[serde(serialize_with = "ser_f3")]
    demand_slots_per_ms: f64,
    #[serde(serialize_with = "ser_f3")]
    budget_slots_per_ms: f64,
    fits: bool,
    high_speed: bool,
}

#[derive(Serialize)]
struct CountersWire {
    uart_rx: u32,
    cdc_in: u32,
    host_dropped: u32,
    ring_full: u32,
    frame_err: u32,
    overrun: u32,
    cdc_out: u32,
    uart_tx: u32,
    rx_dma_err: u32,
    valid: Option<u32>,
    grade: Option<u32>,
    units: Option<u32>,
}

impl From<counters::VcomCounters> for CountersWire {
    fn from(c: counters::VcomCounters) -> Self {
        CountersWire {
            uart_rx: c.uart_rx,
            cdc_in: c.cdc_in,
            host_dropped: c.host_dropped,
            ring_full: c.ring_full,
            frame_err: c.frame_err,
            overrun: c.overrun,
            cdc_out: c.cdc_out,
            uart_tx: c.uart_tx,
            rx_dma_err: c.rx_dma_err,
            valid: c.valid,
            grade: c.grade,
            units: c.units,
        }
    }
}

#[derive(Serialize)]
struct ProbeCountersWire {
    classification: &'static str,
    before: CountersWire,
    cumulative: CountersWire,
    bracket: CountersWire,
}

#[derive(Serialize)]
struct LegWire {
    claimed: bool,
    tx_payload: u64,
    tx_frames: u32,
    verified: u64,
    corrupt: u64,
    missing: u64,
    duplicate: u64,
    unresolved: u64,
    rx_frames: u32,
    resyncs: u32,
    eos: bool,
    eos_total: Option<u64>,
    credit: u64,
    elapsed_us: u32,
    onset_us: u32,
    maxgap_us: u32,
    reconciles: bool,
    #[serde(serialize_with = "ser_f1")]
    goodput_bps: f64,
    #[serde(serialize_with = "ser_f4")]
    budget_frac: f64,
}

impl LegWire {
    fn from_leg(leg: &Leg, run: &Run) -> Self {
        let rx = &leg.rx;
        LegWire {
            claimed: leg.claimed,
            tx_payload: leg.tx_payload,
            tx_frames: leg.tx_frames,
            verified: rx.verified,
            corrupt: rx.corrupt,
            missing: rx.missing,
            duplicate: rx.duplicate,
            unresolved: rx.unresolved,
            rx_frames: rx.frames,
            resyncs: rx.resyncs,
            eos: rx.eos,
            eos_total: leg.rx_eos_total,
            credit: rx.credit,
            elapsed_us: rx.elapsed_us,
            onset_us: rx.onset_us,
            maxgap_us: rx.maxgap_us,
            reconciles: leg.reconciles(),
            goodput_bps: run.goodput_bps(leg.dir),
            budget_frac: run.budget_frac(leg.dir),
        }
    }
}

#[derive(Serialize)]
struct V1Wire<'a> {
    schema_version: u32,
    tool: &'static str,
    unix_s: u64,
    probe: &'a str,
    firmware_row: Option<&'a str>,
    firmware_row_source: &'a str,
    firmware_version: Option<&'a str>,
    verb: &'a str,
    phase: &'a str,
    shape: &'static str,
    mode: &'static str,
    baud_nominal: u32,
    dut_realized: u32,
    probe_realized: Option<u32>,
    interval: u32,
    #[serde(serialize_with = "ser_f6")]
    payload_fraction: f64,
    #[serde(serialize_with = "ser_bare")]
    window_s: f64,
    #[serde(serialize_with = "ser_f4")]
    measured_s: f64,
    #[serde(serialize_with = "ser_bare")]
    host_frac: f64,
    seeds: SeedsWire,
    budget_h2d: u32,
    budget_d2h: u32,
    payload_budget_bps: BpsPairWire,
    pass_bar_bps: BpsPairWire,
    #[serde(serialize_with = "ser_f3")]
    sustained_s: f64,
    timed_out: bool,
    closed: bool,
    dut_ferr: u32,
    dut_ovr: u32,
    dut_drop: u32,
    dut_stuck: u32,
    abandoned: Option<AbandonWire<'a>>,
    checkpoints: CheckpointsWire,
    burst: Option<BurstWire>,
    credit: Option<CreditWire>,
    dap: DapWire,
    clamp: Option<ClampWire>,
    probe_counters: Option<ProbeCountersWire>,
    host_to_dut: Option<LegWire>,
    dut_to_host: Option<LegWire>,
    pass: bool,
    reasons: Vec<String>,
    note: Option<&'a str>,
}

pub fn render_v1(r: &V1Record, unix_s: u64) -> String {
    // The wire field names are literal; keep `stream::key` in step with them.
    debug_assert_eq!(stream::key(Direction::HostToDut), "host_to_dut");
    debug_assert_eq!(stream::key(Direction::DutToHost), "dut_to_host");

    let run = r.run;
    let plan = &run.plan;
    let verdict = run.verdict();

    let checkpoints = CheckpointsWire {
        period_s: run.series.period,
        trend: TrendWire {
            host_to_dut: run.series.trend(Direction::HostToDut).key(),
            dut_to_host: run.series.trend(Direction::DutToHost).key(),
        },
        points: run
            .series
            .points
            .iter()
            .map(|p| PointWire {
                at_s: p.at_secs,
                host_to_dut: p.h2d.map(SliceWire::from),
                dut_to_host: p.d2h.map(SliceWire::from),
            })
            .collect(),
    };

    let probe_counters = run.conservation.map(|c| ProbeCountersWire {
        classification: c.classification().key(),
        before: CountersWire::from(c.before),
        cumulative: CountersWire::from(c.after),
        bracket: CountersWire::from(c.delta()),
    });

    let wire = V1Wire {
        schema_version: SCHEMA_VERSION,
        tool: "vcom-bench",
        unix_s,
        probe: r.probe,
        firmware_row: r.firmware.row,
        firmware_row_source: r.firmware.source,
        firmware_version: r.firmware.version,
        verb: r.verb,
        phase: r.phase,
        shape: "v1",
        mode: plan.shape.key(),
        baud_nominal: r.baud_nominal,
        dut_realized: plan.dut_baud,
        probe_realized: r.probe_realized,
        interval: plan.interval,
        payload_fraction: plan.payload_fraction(),
        window_s: plan.secs,
        measured_s: run.measured_secs,
        host_frac: plan.host_frac,
        seeds: SeedsWire {
            h2d: plan.seeds.h2d,
            d2h: plan.seeds.d2h,
        },
        budget_h2d: plan.budget_h2d(),
        budget_d2h: plan.budget_d2h(),
        payload_budget_bps: BpsPairWire {
            h2d: plan.payload_budget_bps(Direction::HostToDut),
            d2h: plan.payload_budget_bps(Direction::DutToHost),
        },
        pass_bar_bps: BpsPairWire {
            h2d: plan.pass_bar_bps(Direction::HostToDut),
            d2h: plan.pass_bar_bps(Direction::DutToHost),
        },
        sustained_s: run.sustained_secs(),
        timed_out: run.timed_out,
        closed: run.closed,
        dut_ferr: run.ferr,
        dut_ovr: run.ovr,
        dut_drop: run.drop,
        dut_stuck: run.stuck,
        abandoned: run.abandoned.as_ref().map(|a| AbandonWire {
            at_s: a.at_secs,
            checkpoint: a.checkpoint,
            reason: &a.reason,
        }),
        checkpoints,
        burst: plan.burst.map(|burst| BurstWire {
            bytes: burst.bytes,
            idle_ms: burst.idle_ms,
            host: plan.burst_host,
            dut: plan.burst_dut,
        }),
        credit: run.credit.map(|credit| CreditWire {
            window: plan.credit_window,
            granted: credit.granted,
            obeyed: credit.obeyed,
        }),
        dap: DapWire {
            profile: run.dap.profile.key(),
            commands: run.dap.commands,
            bytes: run.dap.bytes,
            secs: run.dap.secs,
            cmd_per_s: run.dap.cps(),
            bytes_per_s: run.dap.bps(),
        },
        clamp: r.clamp.map(|clamp| ClampWire {
            demand_slots_per_ms: clamp.demand,
            budget_slots_per_ms: clamp.budget,
            fits: clamp.fits,
            high_speed: clamp.high_speed,
        }),
        probe_counters,
        host_to_dut: run
            .leg(Direction::HostToDut)
            .map(|leg| LegWire::from_leg(leg, run)),
        dut_to_host: run
            .leg(Direction::DutToHost)
            .map(|leg| LegWire::from_leg(leg, run)),
        pass: verdict.pass,
        reasons: verdict.reasons,
        note: r.note,
    };
    to_json(&wire)
}

/// Emit `text`, already a JSON number, as a raw literal.
fn ser_raw<S: Serializer>(text: String, s: S) -> Result<S::Ok, S::Error> {
    RawValue::from_string(text)
        .map_err(|e| serde::ser::Error::custom(e.to_string()))?
        .serialize(s)
}

fn ser_f1<S: Serializer>(v: &f64, s: S) -> Result<S::Ok, S::Error> {
    ser_raw(format!("{v:.1}"), s)
}

fn ser_f2<S: Serializer>(v: &f64, s: S) -> Result<S::Ok, S::Error> {
    ser_raw(format!("{v:.2}"), s)
}

fn ser_f3<S: Serializer>(v: &f64, s: S) -> Result<S::Ok, S::Error> {
    ser_raw(format!("{v:.3}"), s)
}

fn ser_f4<S: Serializer>(v: &f64, s: S) -> Result<S::Ok, S::Error> {
    ser_raw(format!("{v:.4}"), s)
}

fn ser_f6<S: Serializer>(v: &f64, s: S) -> Result<S::Ok, S::Error> {
    ser_raw(format!("{v:.6}"), s)
}

/// Plain `{}` display for an `f64`: shortest form, no forced decimal.
fn ser_bare<S: Serializer>(v: &f64, s: S) -> Result<S::Ok, S::Error> {
    ser_raw(format!("{v}"), s)
}

/// Compact formatter that writes every escaped character as `\u00xx`.
struct HexControlFormatter;

impl Formatter for HexControlFormatter {
    fn write_char_escape<W: ?Sized + std::io::Write>(
        &mut self,
        writer: &mut W,
        char_escape: CharEscape,
    ) -> std::io::Result<()> {
        use CharEscape::*;
        let byte = match char_escape {
            Quote => return writer.write_all(b"\\\""),
            ReverseSolidus => return writer.write_all(b"\\\\"),
            Solidus => return writer.write_all(b"/"),
            Backspace => 0x08,
            FormFeed => 0x0C,
            LineFeed => 0x0A,
            CarriageReturn => 0x0D,
            Tab => 0x09,
            AsciiControl(byte) => byte,
        };
        write!(writer, "\\u{byte:04x}")
    }
}

/// Render one wire struct as its committed JSONL line: no trailing newline.
fn to_json<T: Serialize>(value: &T) -> String {
    let mut buf = Vec::with_capacity(512);
    let mut ser = serde_json::Serializer::with_formatter(&mut buf, HexControlFormatter);
    value
        .serialize(&mut ser)
        .expect("record serializes to JSON");
    String::from_utf8(buf).expect("JSON output is valid UTF-8")
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::budget::{BusBudget, DapAchieved, DapProfile};
    use crate::stream::{legs, Clock, Credit, Plan, Seeds, Shape, DEFAULT_INTERVAL};
    use daplink_test_protocol::v1::Direction;
    use daplink_test_protocol::v1::{RxCounters, TxCounters, V1Report};

    pub(crate) const FIRMWARE: Firmware<'static> = Firmware {
        row: Some("firmware/mculink"),
        source: "image",
        version: Some("2.1.1.gfeedc0ffee12"),
    };

    pub(crate) fn assert_record(name: &str, line: &str) {
        let actual: serde_json::Value = serde_json::from_str(line).expect("complete JSON record");
        let prefix = format!("{{\"case\":\"{name}\",\"record\":");
        let fixture = include_str!("record/corpus.jsonl")
            .lines()
            .find(|row| row.starts_with(&prefix))
            .expect("named corpus case");
        let expected: serde_json::Value = serde_json::from_str(fixture).unwrap();
        assert_eq!(actual, expected["record"], "{name}");
        assert_eq!(actual["schema_version"], SCHEMA_VERSION);
        assert_eq!(
            format!("{prefix}{line}}}"),
            fixture,
            "{name}: JSONL formatting"
        );
    }

    fn sample() -> RunRecord<'static> {
        RunRecord {
            probe: "0d28:0204:AA",
            firmware: FIRMWARE,
            verb: "echo",
            phase: "run",
            baud_nominal: 2_625_000,
            dut_realized: Some(2_625_000),
            probe_realized: Some(2_666_666),
            len: 256,
            chunk: 64,
            mode: "batched",
            secs: 0.0,
            host_written: 256,
            host_read: 256,
            byte_exact: true,
            first_divergence: None,
            throughput_bps: 123_456.7,
            dut_stats: Some(Stats {
                rx: 256,
                tx: 256,
                ..Stats::default()
            }),
            dap_roundtrips: None,
            pass: Some(true),
            note: None,
        }
    }

    #[test]
    fn a_record_renders_as_one_json_line_with_the_schema_version() {
        let line = render(&sample(), 1_700_000_000);
        assert_record(
            "a_record_renders_as_one_json_line_with_the_schema_version",
            &line,
        );
    }

    #[test]
    fn absent_fields_render_as_null() {
        let mut r = sample();
        r.firmware = Firmware {
            row: None,
            source: "none",
            version: None,
        };
        r.dut_realized = None;
        r.probe_realized = None;
        r.dut_stats = None;
        r.pass = None;
        let line = render(&r, 0);
        assert_record("absent_fields_render_as_null", &line);
    }

    /// A clean duplex run of `secs`.
    fn v1_run(secs: f64) -> Run {
        let plan = Plan {
            shape: Shape::Duplex,
            seeds: Seeds::default(),
            interval: DEFAULT_INTERVAL,
            secs,
            dut_baud: 2_625_000,
            probe_baud: 2_625_000,
            host_frac: 1.0,
            burst: None,
            burst_host: false,
            burst_dut: false,
            credit_window: 4096,
        };
        let bytes = plan.budget_d2h();
        let frames = bytes / plan.interval + 1;
        let report = V1Report {
            tx: bytes,
            tx_frames: frames,
            verified: bytes,
            corrupt: 0,
            missing: 0,
            duplicate: 0,
            unresolved: 0,
            rx_frames: frames,
            resyncs: 0,
            eos: true,
            elapsed_us: 30_000_000,
            onset_us: 120,
            maxgap_us: 900,
            ferr: 0,
            ovr: 0,
            drop: 0,
            credit: 0,
        };
        let host_rx = RxCounters {
            verified: u64::from(bytes),
            frames,
            eos: true,
            eos_total: u64::from(bytes),
            elapsed_us: 30_000_000,
            onset_us: 90,
            maxgap_us: 1_100,
            ..RxCounters::default()
        };
        let host_tx = TxCounters {
            payload: u64::from(plan.budget_h2d()),
            frames,
            credit: 0,
        };
        Run {
            legs: legs(&plan, Some(host_tx), Some(host_rx), &report),
            plan,
            ferr: 0,
            ovr: 0,
            drop: 0,
            stuck: 0,
            measured_secs: secs + 1.0,
            timed_out: false,
            closed: false,
            abandoned: None,
            series: stream::Series::default(),
            dap: DapAchieved {
                profile: DapProfile::Representative,
                commands: 3_000,
                bytes: 192_000,
                secs,
            },
            credit: None,
            conservation: None,
        }
    }

    fn v1_sample(run: &Run) -> V1Record<'_> {
        V1Record {
            probe: "0d28:0204:AA",
            firmware: FIRMWARE,
            verb: "run",
            phase: "run",
            baud_nominal: 2_625_000,
            probe_realized: Some(2_666_666),
            run,
            clamp: Some(crate::budget::clamp(
                &run.plan.demand(run.dap.profile.nominal_bps()),
                &BusBudget::for_packet(64),
            )),
            note: None,
        }
    }

    #[test]
    fn a_v1_window_renders_both_ledgers_and_its_verdict_on_one_line() {
        let run = v1_run(30.0);
        let line = render_v1(&v1_sample(&run), 1_700_000_000);
        assert_record(
            "a_v1_window_renders_both_ledgers_and_its_verdict_on_one_line",
            &line,
        );
    }

    #[test]
    fn a_failed_window_carries_every_reason_it_failed() {
        let mut run = v1_run(30.0);
        run.ovr = 3;
        run.legs[1].rx.eos = false;
        let line = render_v1(&v1_sample(&run), 0);
        assert_record("a_failed_window_carries_every_reason_it_failed", &line);
    }

    #[test]
    fn a_stuck_dma_abort_is_recorded_and_fails_the_window() {
        let mut run = v1_run(30.0);
        run.stuck = 2;
        let line = render_v1(&v1_sample(&run), 0);
        assert_record("a_stuck_dma_abort_is_recorded_and_fails_the_window", &line);
    }

    #[test]
    fn a_window_records_the_probe_ledger_cumulative_and_over_its_bracket() {
        let mut run = v1_run(30.0);
        run.conservation = Some(counters::Conservation {
            before: counters::VcomCounters {
                uart_rx: 4096,
                cdc_in: 4096,
                ..counters::VcomCounters::default()
            },
            after: counters::VcomCounters {
                uart_rx: 1_004_096,
                cdc_in: 1_004_096,
                ..counters::VcomCounters::default()
            },
        });
        let line = render_v1(&v1_sample(&run), 0);
        assert_record(
            "a_window_records_the_probe_ledger_cumulative_and_over_its_bracket",
            &line,
        );
    }

    #[test]
    fn a_window_records_the_checkpoint_series_that_diagnoses_it() {
        use crate::stream::{Abandon, Checkpoint, Series, Slice};
        let mut run = v1_run(60.0);
        run.series = Series {
            period: 5.0,
            points: [1.0, 0.71]
                .into_iter()
                .enumerate()
                .map(|(i, frac)| Checkpoint {
                    at_secs: (i + 1) as f64 * 5.0,
                    h2d: Some(Slice {
                        payload: 455_000,
                        frac: 1.0,
                        ..Slice::default()
                    }),
                    d2h: Some(Slice {
                        payload: 323_000,
                        frac,
                        missing: if frac < 1.0 { 41_216 } else { 0 },
                        ..Slice::default()
                    }),
                })
                .collect(),
        };
        run.closed = true;
        run.abandoned = Some(Abandon {
            at_secs: 12.4,
            checkpoint: 2,
            reason: "DUT -> host: 41216 missing".to_string(),
        });
        let line = render_v1(&v1_sample(&run), 0);
        assert_record(
            "a_window_records_the_checkpoint_series_that_diagnoses_it",
            &line,
        );
    }

    #[test]
    fn a_window_that_took_no_checkpoint_records_an_empty_series() {
        let line = render_v1(&v1_sample(&v1_run(2.0)), 0);
        assert_record(
            "a_window_that_took_no_checkpoint_records_an_empty_series",
            &line,
        );
    }

    #[test]
    fn a_window_from_a_firmware_without_the_query_records_a_null_ledger() {
        let line = render_v1(&v1_sample(&v1_run(30.0)), 0);
        assert_record(
            "a_window_from_a_firmware_without_the_query_records_a_null_ledger",
            &line,
        );
    }

    #[test]
    fn a_simplex_window_records_the_silent_direction_as_null() {
        let mut run = v1_run(30.0);
        run.legs.retain(|leg| leg.dir == Direction::DutToHost);
        let line = render_v1(&v1_sample(&run), 0);
        assert_record(
            "a_simplex_window_records_the_silent_direction_as_null",
            &line,
        );
    }

    #[test]
    fn a_credit_window_records_both_sides_of_the_grant_ledger() {
        let mut run = v1_run(30.0);
        run.credit = Some(Credit {
            granted: 1_000_000,
            obeyed: 999_424,
        });
        let line = render_v1(&v1_sample(&run), 0);
        assert_record(
            "a_credit_window_records_both_sides_of_the_grant_ledger",
            &line,
        );
    }

    #[test]
    fn escaped_failed_echo_record() {
        let mut r = sample();
        r.probe = "probe\"\\\n\t";
        r.note = Some("failed \"exchange\"\r\n\\");
        r.pass = Some(false);
        r.byte_exact = false;
        r.first_divergence = Some(17);
        r.dut_stats.as_mut().unwrap().stuck = 1;
        assert_record("escaped-failed-echo", &render(&r, 0));
    }

    #[test]
    fn burst_timeout_record() {
        let mut run = v1_run(30.0);
        run.plan.burst = Some(daplink_test_protocol::v1::BurstSpec {
            bytes: 256,
            idle_ms: 100,
        });
        run.plan.burst_host = true;
        run.plan.burst_dut = true;
        run.timed_out = true;
        let mut r = v1_sample(&run);
        r.clamp = None;
        r.note = Some("missing \"EOS\"\n");
        assert_record("burst-timeout", &render_v1(&r, 0));
    }

    #[test]
    fn a_contract_failure_records_latency_recovery_and_reason() {
        let mut latencies = crate::contract::Latencies::default();
        latencies.push(std::time::Duration::from_micros(400));
        latencies.push(std::time::Duration::from_micros(900));
        let result = CaseResult {
            case: "reader-pause",
            tier: Some("250ms".into()),
            baud: 115_200,
            exact: false,
            counter_classification: Some(crate::counters::ConservationClass::ObservedLoss),
            dap_progress: true,
            recovery: Some(std::time::Duration::from_micros(1_200)),
            latencies,
            pass: false,
            failure: Some("named loss".into()),
        };
        let line = render_contract("probe", FIRMWARE, &result, 12);
        assert_record(
            "a_contract_failure_records_latency_recovery_and_reason",
            &line,
        );
    }

    #[test]
    fn a_line_transport_failure_reaches_the_contract_record() {
        let failure = crate::contract::line_runtime_failure(
            14,
            921_600,
            "target control",
            "transfer cancelled",
        );
        let result = CaseResult {
            case: "line-coding",
            tier: Some("100-cycles".into()),
            baud: 115_200,
            exact: true,
            counter_classification: None,
            dap_progress: false,
            recovery: None,
            latencies: crate::contract::Latencies::default(),
            pass: false,
            failure: Some(failure),
        };
        let line = render_contract("probe", FIRMWARE, &result, 12);
        assert_record(
            "a_line_transport_failure_reaches_the_contract_record",
            &line,
        );
    }

    #[test]
    fn the_clock_reads_microseconds_from_where_it_started() {
        let clock = Clock::new();
        std::thread::sleep(std::time::Duration::from_millis(2));
        assert!(clock.now_us() >= 2_000);
    }
}
