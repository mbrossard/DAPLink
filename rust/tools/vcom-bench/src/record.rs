//! JSONL run records: one line per measured run or window, plus a summary line per ladder.

use std::fmt::Write as _;
use std::fs::OpenOptions;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use daplink_test_protocol::v1::Direction;
use daplink_test_protocol::Stats;

use crate::budget::Clamp;
use crate::contract::CaseResult;
use crate::stream::{self, Leg, Run};

pub const SCHEMA_VERSION: u32 = 13;

const DEFAULT_FILE_NAME: &str = "vcom-bench.jsonl";

const CHECKOUT_MARKER: &str = "rust/tools/vcom-bench/Cargo.toml";

pub struct RunRecord<'a> {
    pub probe: &'a str,
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

/// Search upward from the running binary for the repository root and use `artifacts/` inside it,
/// or fall back to `./artifacts/` when no root is found.
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

pub fn append_contract(path: &Path, probe: &str, result: &CaseResult) -> std::io::Result<()> {
    append_line(path, &render_contract(probe, result, unix_now()))
}

pub fn render_contract(probe: &str, r: &CaseResult, unix_s: u64) -> String {
    let class = r
        .counter_classification
        .map(|class| quote(class.key()))
        .unwrap_or_else(|| "null".into());
    let failure = r
        .failure
        .as_deref()
        .map(quote)
        .unwrap_or_else(|| "null".into());
    let tier = r
        .tier
        .as_deref()
        .map(quote)
        .unwrap_or_else(|| "null".into());
    format!(
        "{{\"schema_version\":{SCHEMA_VERSION},\"tool\":\"vcom-bench\",\"unix_s\":{unix_s},\"probe\":{},\"verb\":\"contract\",\"case\":{},\"tier\":{tier},\"baud_nominal\":{},\"latency\":{{\"samples\":{},\"p50_us\":{},\"p99_us\":{},\"max_us\":{}}},\"byte_exact\":{},\"counter_classification\":{class},\"dap_progress\":{},\"recovery_us\":{},\"pass\":{},\"failure_reason\":{failure}}}",
        quote(probe), quote(r.case), r.baud, r.latencies.count(),
        opt_num(r.latencies.percentile_us(50)), opt_num(r.latencies.percentile_us(99)),
        opt_num(r.latencies.max_us()), r.exact, r.dap_progress,
        opt_num(r.recovery.map(|d| d.as_micros().min(u128::from(u64::MAX)) as u64)), r.pass,
    )
}

/// No trailing newline.
pub fn render(r: &RunRecord, unix_s: u64) -> String {
    let mut s = String::with_capacity(512);
    let _ = write!(
        s,
        "{{\"schema_version\":{SCHEMA_VERSION},\"tool\":\"vcom-bench\",\"unix_s\":{unix_s},\
         \"probe\":{},\"verb\":{},\"phase\":{},\"shape\":\"echo\",\"baud_nominal\":{},\
         \"dut_realized\":{},\"probe_realized\":{},\"len\":{},\"chunk\":{},\"mode\":{},\
         \"secs\":{},\"host_written\":{},\"host_read\":{},\"byte_exact\":{},\
         \"first_divergence\":{},\"throughput_bps\":{:.1}",
        quote(r.probe),
        quote(r.verb),
        quote(r.phase),
        r.baud_nominal,
        opt_num(r.dut_realized.map(u64::from)),
        opt_num(r.probe_realized.map(u64::from)),
        r.len,
        r.chunk,
        quote(r.mode),
        r.secs,
        r.host_written,
        r.host_read,
        r.byte_exact,
        opt_num(r.first_divergence),
        r.throughput_bps,
    );
    if let Some(st) = &r.dut_stats {
        let _ = write!(
            s,
            ",\"dut_stats\":{{\"rx\":{},\"tx\":{},\"ferr\":{},\"ovr\":{},\"drop\":{},\"stuck\":{}}}",
            st.rx, st.tx, st.ferr, st.ovr, st.drop, st.stuck
        );
    } else {
        s.push_str(",\"dut_stats\":null");
    }
    let _ = write!(s, ",\"dap_roundtrips\":{}", opt_num(r.dap_roundtrips));
    match r.pass {
        Some(p) => {
            let _ = write!(s, ",\"pass\":{p}");
        }
        None => s.push_str(",\"pass\":null"),
    }
    match r.note {
        Some(n) => {
            let _ = write!(s, ",\"note\":{}", quote(n));
        }
        None => s.push_str(",\"note\":null"),
    }
    s.push('}');
    s
}

pub struct V1Record<'a> {
    pub probe: &'a str,
    pub verb: &'a str,
    /// Which phase of the verb: `warmup`, `sweep`, `screen`, `sustain`, `return-priority`,
    /// `discovery`, `qualify`, `soak`, or `run`.
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

pub fn render_v1(r: &V1Record, unix_s: u64) -> String {
    let run = r.run;
    let plan = &run.plan;
    let verdict = run.verdict();
    let mut s = String::with_capacity(2048);
    let _ = write!(
        s,
        "{{\"schema_version\":{SCHEMA_VERSION},\"tool\":\"vcom-bench\",\"unix_s\":{unix_s},\
         \"probe\":{},\"verb\":{},\"phase\":{},\"shape\":\"v1\",\"mode\":{},\
         \"baud_nominal\":{},\"dut_realized\":{},\"probe_realized\":{},\"interval\":{},\
         \"payload_fraction\":{:.6},\"window_s\":{},\"measured_s\":{:.4},\"host_frac\":{},\
         \"seeds\":{{\"h2d\":{},\"d2h\":{}}},\"budget_h2d\":{},\"budget_d2h\":{},\
         \"payload_budget_bps\":{{\"h2d\":{:.1},\"d2h\":{:.1}}},\"pass_bar_bps\":{{\"h2d\":{:.1},\"d2h\":{:.1}}},\"sustained_s\":{:.3},\
         \"timed_out\":{},\"closed\":{},\"dut_ferr\":{},\"dut_ovr\":{},\"dut_drop\":{},\
         \"dut_stuck\":{}",
        quote(r.probe),
        quote(r.verb),
        quote(r.phase),
        quote(plan.shape.key()),
        r.baud_nominal,
        plan.dut_baud,
        opt_num(r.probe_realized.map(u64::from)),
        plan.interval,
        plan.payload_fraction(),
        plan.secs,
        run.measured_secs,
        plan.host_frac,
        plan.seeds.h2d,
        plan.seeds.d2h,
        plan.budget_h2d(),
        plan.budget_d2h(),
        plan.payload_budget_bps(Direction::HostToDut),
        plan.payload_budget_bps(Direction::DutToHost),
        plan.pass_bar_bps(Direction::HostToDut),
        plan.pass_bar_bps(Direction::DutToHost),
        run.sustained_secs(),
        run.timed_out,
        run.closed,
        run.ferr,
        run.ovr,
        run.drop,
        run.stuck,
    );
    match &run.abandoned {
        Some(a) => {
            let _ = write!(
                s,
                ",\"abandoned\":{{\"at_s\":{:.3},\"checkpoint\":{},\"reason\":{}}}",
                a.at_secs,
                a.checkpoint,
                quote(&a.reason),
            );
        }
        None => s.push_str(",\"abandoned\":null"),
    }
    series_json(&mut s, run);
    match plan.burst {
        Some(burst) => {
            let _ = write!(
                s,
                ",\"burst\":{{\"bytes\":{},\"idle_ms\":{},\"host\":{},\"dut\":{}}}",
                burst.bytes, burst.idle_ms, plan.burst_host, plan.burst_dut
            );
        }
        None => s.push_str(",\"burst\":null"),
    }
    match run.credit {
        Some(credit) => {
            let _ = write!(
                s,
                ",\"credit\":{{\"window\":{},\"granted\":{},\"obeyed\":{}}}",
                plan.credit_window, credit.granted, credit.obeyed
            );
        }
        None => s.push_str(",\"credit\":null"),
    }
    let dap = &run.dap;
    let _ = write!(
        s,
        ",\"dap\":{{\"profile\":{},\"commands\":{},\"bytes\":{},\"secs\":{:.4},\
         \"cmd_per_s\":{:.1},\"bytes_per_s\":{:.1}}}",
        quote(dap.profile.key()),
        dap.commands,
        dap.bytes,
        dap.secs,
        dap.cps(),
        dap.bps(),
    );
    match r.clamp {
        Some(clamp) => {
            let _ = write!(
                s,
                ",\"clamp\":{{\"demand_slots_per_ms\":{:.3},\"budget_slots_per_ms\":{:.3},\
                 \"fits\":{},\"high_speed\":{}}}",
                clamp.demand, clamp.budget, clamp.fits, clamp.high_speed
            );
        }
        None => s.push_str(",\"clamp\":null"),
    }
    match run.conservation {
        Some(c) => {
            let d = c.delta();
            let _ = write!(
                s,
                ",\"probe_counters\":{{\"classification\":{},",
                quote(c.classification().key())
            );
            counters_json(&mut s, "cumulative", c.after);
            s.push(',');
            counters_json(&mut s, "bracket", d);
            s.push('}');
        }
        None => s.push_str(",\"probe_counters\":null"),
    }
    for dir in stream::DIRECTIONS {
        let _ = write!(s, ",{}:", quote(stream::key(dir)));
        match run.leg(dir) {
            Some(leg) => leg_json(&mut s, leg, run),
            None => s.push_str("null"),
        }
    }
    let _ = write!(s, ",\"pass\":{},\"reasons\":[", verdict.pass);
    for (i, reason) in verdict.reasons.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&quote(reason));
    }
    s.push(']');
    match r.note {
        Some(n) => {
            let _ = write!(s, ",\"note\":{}", quote(n));
        }
        None => s.push_str(",\"note\":null"),
    }
    s.push('}');
    s
}

fn series_json(s: &mut String, run: &Run) {
    let series = &run.series;
    let _ = write!(
        s,
        ",\"checkpoints\":{{\"period_s\":{},\"trend\":{{",
        series.period
    );
    for (i, dir) in stream::DIRECTIONS.into_iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let _ = write!(
            s,
            "{}:{}",
            quote(stream::key(dir)),
            quote(series.trend(dir).key())
        );
    }
    s.push_str("},\"points\":[");
    for (i, point) in series.points.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let _ = write!(s, "{{\"at_s\":{:.3}", point.at_secs);
        for dir in stream::DIRECTIONS {
            let _ = write!(s, ",{}:", quote(stream::key(dir)));
            match point.slice(dir) {
                Some(slice) => {
                    let _ = write!(
                        s,
                        "{{\"payload\":{},\"frac\":{:.4},\"corrupt\":{},\"missing\":{},\
                         \"duplicate\":{},\"unresolved\":{},\"resyncs\":{}}}",
                        slice.payload,
                        slice.frac,
                        slice.corrupt,
                        slice.missing,
                        slice.duplicate,
                        slice.unresolved,
                        slice.resyncs,
                    );
                }
                None => s.push_str("null"),
            }
        }
        s.push('}');
    }
    s.push_str("]}");
}

fn counters_json(s: &mut String, name: &str, c: stream::VcomCounters) {
    let _ = write!(
        s,
        "{}:{{\"uart_rx\":{},\"cdc_in\":{},\"host_dropped\":{},\"ring_full\":{},\
         \"frame_err\":{},\"overrun\":{},\"cdc_out\":{},\"uart_tx\":{},\
         \"rx_dma_err\":{},\"valid\":{},\"grade\":{},\"units\":{}}}",
        quote(name),
        c.uart_rx,
        c.cdc_in,
        c.host_dropped,
        c.ring_full,
        c.frame_err,
        c.overrun,
        c.cdc_out,
        c.uart_tx,
        c.rx_dma_err,
        opt_num(c.valid.map(u64::from)),
        opt_num(c.grade.map(u64::from)),
        opt_num(c.units.map(u64::from)),
    );
}

fn leg_json(s: &mut String, leg: &Leg, run: &Run) {
    let rx = &leg.rx;
    let _ = write!(
        s,
        "{{\"claimed\":{},\"tx_payload\":{},\"tx_frames\":{},\"verified\":{},\"corrupt\":{},\
         \"missing\":{},\"duplicate\":{},\"unresolved\":{},\"rx_frames\":{},\"resyncs\":{},\
         \"eos\":{},\"eos_total\":{},\"credit\":{},\"elapsed_us\":{},\"onset_us\":{},\
         \"maxgap_us\":{},\"reconciles\":{},\"goodput_bps\":{:.1},\"budget_frac\":{:.4}}}",
        leg.claimed,
        leg.tx_payload,
        leg.tx_frames,
        rx.verified,
        rx.corrupt,
        rx.missing,
        rx.duplicate,
        rx.unresolved,
        rx.frames,
        rx.resyncs,
        rx.eos,
        opt_num(leg.rx_eos_total),
        rx.credit,
        rx.elapsed_us,
        rx.onset_us,
        rx.maxgap_us,
        leg.reconciles(),
        run.goodput_bps(leg.dir),
        run.budget_frac(leg.dir),
    );
}

pub fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn opt_num(v: Option<u64>) -> String {
    v.map_or_else(|| "null".to_string(), |n| n.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::budget::{BusBudget, DapAchieved, DapProfile};
    use crate::stream::{legs, Clock, Credit, Plan, Seeds, Shape, DEFAULT_INTERVAL};
    use daplink_test_protocol::v1::Direction;
    use daplink_test_protocol::v1::{RxCounters, TxCounters, V1Report};

    fn sample() -> RunRecord<'static> {
        RunRecord {
            probe: "0d28:0204:AA",
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
        assert!(line.starts_with("{\"schema_version\":13,\"tool\":\"vcom-bench\""));
        assert!(!line.contains('\n'));
        assert!(line.ends_with('}'));
        for needle in [
            "\"shape\":\"echo\"",
            "\"baud_nominal\":2625000",
            "\"probe_realized\":2666666",
            "\"dut_stats\":{\"rx\":256,",
            "\"stuck\":0",
            "\"byte_exact\":true",
            "\"pass\":true",
            "\"first_divergence\":null",
            "\"note\":null",
        ] {
            assert!(line.contains(needle), "{line} is missing {needle}");
        }
    }

    #[test]
    fn absent_fields_render_as_null() {
        let mut r = sample();
        r.dut_realized = None;
        r.probe_realized = None;
        r.dut_stats = None;
        r.pass = None;
        let line = render(&r, 0);
        for needle in [
            "\"dut_realized\":null",
            "\"probe_realized\":null",
            "\"dut_stats\":null",
            "\"pass\":null",
        ] {
            assert!(line.contains(needle), "{line} is missing {needle}");
        }
    }

    /// A clean duplex run of `secs` at the F401's edge rate.
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
            complete: true,
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
        assert!(line.starts_with("{\"schema_version\":13,\"tool\":\"vcom-bench\""));
        assert!(!line.contains('\n'));
        assert!(line.ends_with('}'));
        for needle in [
            "\"shape\":\"v1\"",
            "\"mode\":\"duplex\"",
            "\"interval\":2048",
            "\"payload_fraction\":0.988417",
            "\"seeds\":{\"h2d\":1592590337,\"d2h\":1592590338}",
            "\"host_to_dut\":{\"claimed\":true,",
            "\"dut_to_host\":{\"claimed\":true,",
            "\"unresolved\":0",
            "\"onset_us\":90",
            "\"maxgap_us\":1100",
            "\"reconciles\":true",
            "\"dap\":{\"profile\":\"representative\"",
            "\"clamp\":{\"demand_slots_per_ms\":8.403",
            "\"fits\":true",
            "\"credit\":null",
            "\"burst\":null",
            "\"pass\":true",
            "\"reasons\":[]",
        ] {
            assert!(line.contains(needle), "{line} is missing {needle}");
        }
    }

    #[test]
    fn a_failed_window_carries_every_reason_it_failed() {
        let mut run = v1_run(30.0);
        run.ovr = 3;
        run.legs[1].rx.eos = false;
        let line = render_v1(&v1_sample(&run), 0);
        assert!(line.contains("\"pass\":false"), "{line}");
        assert!(line.contains("overruns"), "{line}");
        assert!(line.contains("end-of-stream"), "{line}");
        assert!(line.contains("\"eos\":false"), "{line}");
    }

    #[test]
    fn a_stuck_dma_abort_is_recorded_and_fails_the_window() {
        let mut run = v1_run(30.0);
        run.stuck = 2;
        let line = render_v1(&v1_sample(&run), 0);
        assert!(line.contains("\"dut_stuck\":2"), "{line}");
        assert!(line.contains("\"pass\":false"), "{line}");
        assert!(line.contains("stuck DMA aborts"), "{line}");
    }

    #[test]
    fn a_window_records_the_probe_ledger_cumulative_and_over_its_bracket() {
        let mut run = v1_run(30.0);
        run.conservation = Some(stream::Conservation {
            before: stream::VcomCounters {
                uart_rx: 4096,
                cdc_in: 4096,
                ..stream::VcomCounters::default()
            },
            after: stream::VcomCounters {
                uart_rx: 1_004_096,
                cdc_in: 1_004_096,
                ..stream::VcomCounters::default()
            },
        });
        let line = render_v1(&v1_sample(&run), 0);
        assert!(
            line.contains("\"probe_counters\":{\"classification\":\"partial-clean\","),
            "{line}"
        );
        assert!(
            line.contains("\"cumulative\":{\"uart_rx\":1004096,"),
            "{line}"
        );
        assert!(line.contains("\"bracket\":{\"uart_rx\":1000000,"), "{line}");
        assert!(line.contains("\"overrun\":0"), "{line}");
        assert!(
            line.contains("\"rx_dma_err\":0,\"valid\":null,\"grade\":null"),
            "{line}"
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
        for needle in [
            "\"sustained_s\":12.400",
            "\"closed\":true",
            "\"abandoned\":{\"at_s\":12.400,\"checkpoint\":2,",
            "\"checkpoints\":{\"period_s\":5,",
            "\"trend\":{\"host_to_dut\":\"holding\",\"dut_to_host\":\"declining\"}",
            "\"points\":[{\"at_s\":5.000,",
            "\"missing\":41216",
        ] {
            assert!(line.contains(needle), "{line} is missing {needle}");
        }
    }

    #[test]
    fn a_window_that_took_no_checkpoint_records_an_empty_series() {
        let line = render_v1(&v1_sample(&v1_run(2.0)), 0);
        assert!(line.contains("\"points\":[]"), "{line}");
        assert!(line.contains("\"dut_to_host\":\"unsampled\""), "{line}");
        assert!(line.contains("\"abandoned\":null"), "{line}");
    }

    #[test]
    fn a_window_from_a_firmware_without_the_query_records_a_null_ledger() {
        let line = render_v1(&v1_sample(&v1_run(30.0)), 0);
        assert!(line.contains("\"probe_counters\":null"), "{line}");
    }

    #[test]
    fn a_simplex_window_records_the_silent_direction_as_null() {
        let mut run = v1_run(30.0);
        run.legs.retain(|leg| leg.dir == Direction::DutToHost);
        let line = render_v1(&v1_sample(&run), 0);
        assert!(line.contains("\"host_to_dut\":null"), "{line}");
        assert!(line.contains("\"dut_to_host\":{"), "{line}");
    }

    #[test]
    fn a_credit_window_records_both_sides_of_the_grant_ledger() {
        let mut run = v1_run(30.0);
        run.credit = Some(Credit {
            granted: 1_000_000,
            obeyed: 999_424,
        });
        let line = render_v1(&v1_sample(&run), 0);
        assert!(
            line.contains("\"credit\":{\"window\":4096,\"granted\":1000000,\"obeyed\":999424}"),
            "{line}"
        );
    }

    #[test]
    fn quoting_escapes_what_json_requires() {
        assert_eq!(quote("plain"), "\"plain\"");
        assert_eq!(quote("a\"b\\c"), "\"a\\\"b\\\\c\"");
        assert_eq!(quote("x\ny"), "\"x\\u000ay\"");
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
            counter_classification: Some(crate::stream::ConservationClass::ObservedLoss),
            dap_progress: true,
            recovery: Some(std::time::Duration::from_micros(1_200)),
            latencies,
            pass: false,
            failure: Some("named loss".into()),
        };
        let line = render_contract("probe", &result, 12);
        for needle in [
            "\"schema_version\":13",
            "\"verb\":\"contract\"",
            "\"case\":\"reader-pause\"",
            "\"p50_us\":400",
            "\"p99_us\":900",
            "\"counter_classification\":\"observed-loss\"",
            "\"recovery_us\":1200",
            "\"pass\":false",
            "\"failure_reason\":\"named loss\"",
        ] {
            assert!(line.contains(needle), "{line} is missing {needle}");
        }
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
        let line = render_contract("probe", &result, 12);
        assert!(line.contains("\"case\":\"line-coding\""), "{line}");
        assert!(
            line.contains("cycle 14 rate 921600 target control stage"),
            "{line}"
        );
        assert!(line.contains("\"pass\":false"), "{line}");
    }

    #[test]
    fn the_clock_reads_microseconds_from_where_it_started() {
        let clock = Clock::new();
        std::thread::sleep(std::time::Duration::from_millis(2));
        assert!(clock.now_us() >= 2_000);
    }
}
