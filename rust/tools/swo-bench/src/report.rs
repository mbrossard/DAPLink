//! Per-tier rows, the human table, and the JSON document.

use std::fmt::Write;

use crate::stream::{Framing, Gap};

/// Capture window per tier, long enough that every tier laps the probe's trace ring many times;
/// a shorter window lets the slower tiers never fill it, and a lap-loss bug reports clean.
pub const DEFAULT_WINDOW_MS: u64 = 5_000;

/// Gaps listed under a row before the rest are summarized.
pub const GAPS_SHOWN: usize = 4;

/// Payload fraction of the captured bytes below which a tier is judged misframed. The 8-byte
/// records ride in 10 ITM wire bytes, so a healthy tier yields 0.80; every recorded clean tier
/// sits at 0.77 or above and every recorded misframed one at 0.53 or below.
pub const MIN_PAYLOAD_YIELD: f64 = 0.65;

/// Captures smaller than this are too short for the yield fraction to mean anything; a tier
/// with no capture at all is silent, not misframed.
pub const YIELD_GATE_MIN_BYTES: usize = 4096;

/// A tier's overall judgement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Every record arrived in sequence, and the capture decoded to record payload.
    Clean,
    /// Records arrived, but the sequence has holes.
    Gaps,
    /// Nothing framed, and nothing (or almost nothing) was on the wire.
    Silent,
    /// Bytes were captured but too few decoded to payload: bit misalignment, not a pass.
    Misframed,
}

impl Verdict {
    /// Table label.
    pub const fn label(self) -> &'static str {
        match self {
            Verdict::Clean => "clean",
            Verdict::Gaps => "GAPS",
            Verdict::Silent => "SILENT",
            Verdict::Misframed => "MISFRAMED",
        }
    }

    /// Judge a tier. The sequence tags are the verification, but an order-only check grades a
    /// misframed capture clean, so the yield gate demotes a full capture that decoded to little.
    pub fn of(framing: &Framing, captured_bytes: usize, payload_bytes: usize) -> Verdict {
        if framing.records == 0 {
            if captured_bytes >= YIELD_GATE_MIN_BYTES {
                Verdict::Misframed
            } else {
                Verdict::Silent
            }
        } else if !framing.is_clean() {
            Verdict::Gaps
        } else if below_yield_gate(captured_bytes, payload_bytes) {
            Verdict::Misframed
        } else {
            Verdict::Clean
        }
    }
}

/// True when a capture large enough to judge yielded under [`MIN_PAYLOAD_YIELD`] of its bytes.
pub fn below_yield_gate(captured_bytes: usize, payload_bytes: usize) -> bool {
    captured_bytes >= YIELD_GATE_MIN_BYTES
        && (payload_bytes as f64) < MIN_PAYLOAD_YIELD * captured_bytes as f64
}

/// One baud tier's measurement.
#[derive(Debug, Clone)]
pub struct Tier {
    /// Baud requested of both the DUT and the probe.
    pub requested_baud: u32,
    /// Baud the probe reported realizing.
    pub probe_baud: u32,
    /// Baud the DUT's integer prescaler realizes.
    pub dut_baud: Option<u32>,
    /// Capture window, in milliseconds.
    pub window_ms: f64,
    /// Trace bytes captured off the wire.
    pub captured_bytes: usize,
    /// Captured wire throughput.
    pub kib_per_s: f64,
    /// Stimulus-port payload bytes the ITM decode yielded.
    pub payload_bytes: usize,
    /// ITM overflow packets: the target's own trace FIFO dropped data.
    pub itm_overflow: usize,
    /// Bytes the ITM decoder could not attribute to a packet.
    pub decode_errors: usize,
    /// Record framing and sequence result.
    pub framing: Framing,
    /// `DAP_SWO_Data` exchanges issued.
    pub polls: u64,
    /// Polls that came back full, i.e. the host was already behind.
    pub full_polls: u64,
    /// Polls the probe answered with the capture-active bit clear. Nonzero means the probe stopped
    /// capturing mid-window, which loss counters alone cannot distinguish from bytes going missing.
    pub inactive_polls: u64,
    /// Bytes the probe still held when the window closed.
    pub probe_bytes_left: u32,
    /// SWD throughput achieved by the interleaved load, when `--swd-load` was on.
    pub swd_kib_per_s: Option<f64>,
    /// Load reads whose data differed from this tier's first chunk.
    pub swd_mismatches: u64,
    /// Load transfers the probe did not acknowledge cleanly.
    pub swd_errors: u64,
    /// The tier's judgement.
    pub verdict: Verdict,
}

/// Column header for the tier table.
pub fn column_header() -> String {
    format!(
        "{:>9}  {:>9}  {:>9}  {:>8}  {:>8}  {:>7}  {:>6}  {:>4}  {:>6}  {:>4}  {:>4}  {:>5}  {:>8}  {:>9}  {:>7}",
        "baud",
        "probe",
        "dut",
        "KiB/s",
        "bytes",
        "records",
        "lost",
        "gaps",
        "resync",
        "ovf",
        "derr",
        "full%",
        "polls/s",
        "swd_KiB/s",
        "verdict",
    )
}

/// Render one tier as an aligned table row.
pub fn render(t: &Tier) -> String {
    let dash = "-".to_string();
    format!(
        "{:>9}  {:>9}  {:>9}  {:>8.1}  {:>8}  {:>7}  {:>6}  {:>4}  {:>6}  {:>4}  {:>4}  {:>5.0}  {:>8.0}  {:>9}  {:>7}",
        t.requested_baud,
        t.probe_baud,
        t.dut_baud.map_or(dash.clone(), |b| b.to_string()),
        t.kib_per_s,
        t.captured_bytes,
        t.framing.records,
        lost_display(t),
        t.framing.gaps.len(),
        t.framing.resync_events,
        t.itm_overflow,
        t.decode_errors,
        full_poll_percent(t),
        polls_per_s(t),
        t.swd_kib_per_s.map_or(dash, |v| format!("{v:.1}")),
        t.verdict.label(),
    )
}

/// Lost-record total for a row: unknowable once resyncs corrupted the tags.
pub fn lost_display(t: &Tier) -> String {
    if t.framing.resync_events > 0 {
        "?".into()
    } else {
        t.framing.lost_records.to_string()
    }
}

/// A gap's lost-record count: unknowable once resyncs corrupted the tags it was derived from.
pub fn gap_missing_display(t: &Tier, gap: &Gap) -> String {
    if t.framing.resync_events > 0 {
        "?".into()
    } else {
        gap.missing.to_string()
    }
}

/// `DAP_SWO_Data` exchange rate. On a full-speed probe an exchange carries 60 payload bytes, so
/// this is what bounds sustained capture, not the ring.
pub fn polls_per_s(t: &Tier) -> f64 {
    if t.window_ms <= 0.0 {
        return 0.0;
    }
    t.polls as f64 * 1000.0 / t.window_ms
}

/// Share of polls that came back full, as a percentage.
pub fn full_poll_percent(t: &Tier) -> f64 {
    if t.polls == 0 {
        return 0.0;
    }
    100.0 * t.full_polls as f64 / t.polls as f64
}

/// The loud part: everything a non-clean tier lost, named with its position. Returns nothing for
/// a clean tier with no probe-side complaint.
pub fn notes(t: &Tier) -> Vec<String> {
    let mut out = Vec::new();
    if t.inactive_polls > 0 {
        // The probe stopped capturing mid-window; loss counters cannot say that on their own.
        out.push(format!(
            "  {} baud: the probe answered {} of {} poll(s) with capture inactive -- it stopped \
             capturing mid-window, so loss below is not only bytes missing the host",
            t.requested_baud, t.inactive_polls, t.polls
        ));
    }
    if t.verdict == Verdict::Silent {
        out.push(format!(
            "  {} baud: no record framed in {:.0} ms ({} bytes captured, {} payload)",
            t.requested_baud, t.window_ms, t.captured_bytes, t.payload_bytes
        ));
    }
    if below_yield_gate(t.captured_bytes, t.payload_bytes) || t.verdict == Verdict::Misframed {
        out.push(format!(
            "  {} baud: {} payload byte(s) from {} captured ({:.0}%, gate {:.0}%) -- the capture \
             is misframed, not evidence",
            t.requested_baud,
            t.payload_bytes,
            t.captured_bytes,
            100.0 * t.payload_bytes as f64 / (t.captured_bytes as f64).max(1.0),
            100.0 * MIN_PAYLOAD_YIELD
        ));
    }
    for gap in t.framing.gaps.iter().take(GAPS_SHOWN) {
        out.push(format!(
            "  {} baud: {} record(s) lost after seq {} (payload offset {})",
            t.requested_baud,
            gap_missing_display(t, gap),
            gap.after_seq,
            gap.byte_offset
        ));
    }
    if t.framing.gaps.len() > GAPS_SHOWN {
        if t.framing.resync_events > 0 {
            out.push(format!(
                "  {} baud: {} further gap(s) not listed; lost total unknowable past corrupted tags",
                t.requested_baud,
                t.framing.gaps.len() - GAPS_SHOWN
            ));
        } else {
            out.push(format!(
                "  {} baud: {} further gap(s) not listed, {} record(s) lost in total",
                t.requested_baud,
                t.framing.gaps.len() - GAPS_SHOWN,
                t.framing.lost_records
            ));
        }
    }
    if t.framing.resync_events > 0 {
        out.push(format!(
            "  {} baud: {} resync(s) over {} byte(s) -- the trace line lost data mid-record",
            t.requested_baud, t.framing.resync_events, t.framing.resync_bytes
        ));
    }
    if t.framing.out_of_order > 0 {
        out.push(format!(
            "  {} baud: {} record(s) whose sequence did not advance",
            t.requested_baud, t.framing.out_of_order
        ));
    }
    if t.itm_overflow > 0 {
        out.push(format!(
            "  {} baud: {} ITM overflow packet(s) -- the target's own trace FIFO dropped data",
            t.requested_baud, t.itm_overflow
        ));
    }
    if t.swd_mismatches > 0 || t.swd_errors > 0 {
        out.push(format!(
            "  {} baud: SWD load had {} mismatching read(s) and {} unacked transfer(s)",
            t.requested_baud, t.swd_mismatches, t.swd_errors
        ));
    }
    if let Some(dut) = t.dut_baud {
        if dut != t.requested_baud || t.probe_baud != t.requested_baud {
            out.push(format!(
                "  {} baud: realized {} on the DUT and {} on the probe",
                t.requested_baud, dut, t.probe_baud
            ));
        }
    }
    out
}

/// Run-level context the JSON document states once.
pub struct Context<'a> {
    /// Run start time, ISO-8601 UTC.
    pub timestamp_utc: String,
    /// Probe product string.
    pub probe_product: Option<String>,
    /// Probe serial string.
    pub probe_serial: Option<String>,
    /// `DAP_Info` firmware version string.
    pub firmware_version: Option<String>,
    /// `DAP_PACKET_SIZE`, in bytes.
    pub packet_size: Option<u16>,
    /// `DAP_PACKET_COUNT`, outstanding packets.
    pub packet_count: Option<u8>,
    /// Probe SWO trace buffer size, in bytes.
    pub trace_buffer_bytes: Option<u32>,
    /// Target profile name.
    pub target: &'a str,
    /// Capture window per tier, in milliseconds.
    pub window_ms: u64,
    /// Effective pipeline depth.
    pub depth: usize,
    /// Why that depth: requested, or the auto policy's stated reason.
    pub depth_rationale: &'a str,
    /// Whether a concurrent SWD load ran.
    pub swd_load: bool,
    /// Whether the sweep reset the DUT before its first tier.
    pub dut_reset: bool,
    /// Requested baud tiers, in sweep order.
    pub bauds: &'a [u32],
}

/// Serialize the context and tier rows to the versioned JSON schema (`schema_version` 10, which
/// dropped `probe_overflow`, `probe_trace_error` and `poll_cadence`; 9 added the idle share of
/// each stall and `gaps_total` to `poll_cadence`; 8 made `lost_records` `null` once resyncs
/// corrupted the tags and added `poll_cadence`; 6 added `dut_reset`; 5 added `depth_rationale`;
/// 4 added the `MISFRAMED` verdict; 3 made the two probe status rows `null` when clear).
pub fn to_json(ctx: &Context, rows: &[Tier]) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "{{");
    let _ = writeln!(out, "  \"schema_version\": 10,");
    let _ = writeln!(
        out,
        "  \"timestamp_utc\": {},",
        json_str(&ctx.timestamp_utc)
    );
    let _ = writeln!(out, "  \"probe\": {{");
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
        "    \"packet_count\": {},",
        json_opt_num(ctx.packet_count)
    );
    let _ = writeln!(
        out,
        "    \"trace_buffer_bytes\": {}",
        json_opt_num(ctx.trace_buffer_bytes)
    );
    let _ = writeln!(out, "  }},");
    let _ = writeln!(out, "  \"target\": {},", json_str(ctx.target));
    let _ = writeln!(out, "  \"window_ms\": {},", ctx.window_ms);
    let _ = writeln!(out, "  \"depth\": {},", ctx.depth);
    let _ = writeln!(
        out,
        "  \"depth_rationale\": {},",
        json_str(ctx.depth_rationale)
    );
    let _ = writeln!(out, "  \"swd_load\": {},", ctx.swd_load);
    let _ = writeln!(out, "  \"dut_reset\": {},", ctx.dut_reset);

    let _ = write!(out, "  \"bauds\": [");
    for (i, baud) in ctx.bauds.iter().enumerate() {
        let comma = if i + 1 < ctx.bauds.len() { ", " } else { "" };
        let _ = write!(out, "{baud}{comma}");
    }
    let _ = writeln!(out, "],");

    let _ = writeln!(out, "  \"rows\": [");
    for (i, row) in rows.iter().enumerate() {
        let comma = if i + 1 < rows.len() { "," } else { "" };
        let _ = write!(out, "{}", json_row(row));
        let _ = writeln!(out, "{comma}");
    }
    let _ = writeln!(out, "  ]");
    let _ = write!(out, "}}");
    out.push('\n');
    out
}

/// One tier object, indented for the `rows` array.
fn json_row(t: &Tier) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "    {{");
    let _ = writeln!(out, "      \"requested_baud\": {},", t.requested_baud);
    let _ = writeln!(out, "      \"probe_baud\": {},", t.probe_baud);
    let _ = writeln!(out, "      \"dut_baud\": {},", json_opt_num(t.dut_baud));
    let _ = writeln!(out, "      \"window_ms\": {},", json_f64(t.window_ms));
    let _ = writeln!(out, "      \"captured_bytes\": {},", t.captured_bytes);
    let _ = writeln!(out, "      \"kib_per_s\": {},", json_f64(t.kib_per_s));
    let _ = writeln!(out, "      \"payload_bytes\": {},", t.payload_bytes);
    let _ = writeln!(out, "      \"records\": {},", t.framing.records);
    // Corrupted tags make the total nonsense, so it is withheld rather than serialized raw.
    let _ = writeln!(out, "      \"lost_records\": {},", json_lost(t));
    let _ = writeln!(out, "      \"out_of_order\": {},", t.framing.out_of_order);
    let _ = writeln!(out, "      \"resync_events\": {},", t.framing.resync_events);
    let _ = writeln!(out, "      \"resync_bytes\": {},", t.framing.resync_bytes);
    let _ = writeln!(out, "      \"itm_overflow\": {},", t.itm_overflow);
    let _ = writeln!(out, "      \"decode_errors\": {},", t.decode_errors);
    let _ = writeln!(out, "      \"polls\": {},", t.polls);
    let _ = writeln!(out, "      \"full_polls\": {},", t.full_polls);
    let _ = writeln!(out, "      \"inactive_polls\": {},", t.inactive_polls);
    let _ = writeln!(out, "      \"polls_per_s\": {},", json_f64(polls_per_s(t)));
    let _ = writeln!(out, "      \"probe_bytes_left\": {},", t.probe_bytes_left);
    let _ = writeln!(
        out,
        "      \"swd_kib_per_s\": {},",
        t.swd_kib_per_s.map_or("null".to_string(), json_f64)
    );
    let _ = writeln!(out, "      \"swd_mismatches\": {},", t.swd_mismatches);
    let _ = writeln!(out, "      \"swd_errors\": {},", t.swd_errors);
    let _ = writeln!(out, "      \"verdict\": {},", json_str(t.verdict.label()));
    let _ = writeln!(out, "      \"gaps\": [");
    for (i, gap) in t.framing.gaps.iter().enumerate() {
        let comma = if i + 1 < t.framing.gaps.len() {
            ","
        } else {
            ""
        };
        let _ = writeln!(out, "        {}{comma}", json_gap(t, gap));
    }
    let _ = writeln!(out, "      ]");
    let _ = write!(out, "    }}");
    out
}

/// One gap as a compact JSON object; `missing` is withheld with the tier's total.
fn json_gap(t: &Tier, gap: &Gap) -> String {
    let missing = if t.framing.resync_events > 0 {
        "null".to_string()
    } else {
        gap.missing.to_string()
    };
    format!(
        "{{\"after_seq\": {}, \"missing\": {missing}, \"byte_offset\": {}}}",
        gap.after_seq, gap.byte_offset
    )
}

/// The tier's lost-record total, `null` once a resync corrupted the tags it is summed from.
fn json_lost(t: &Tier) -> String {
    if t.framing.resync_events > 0 {
        "null".to_string()
    } else {
        t.framing.lost_records.to_string()
    }
}

/// A JSON string literal, escaping what the spec requires.
fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// An optional string as a JSON string or `null`.
fn json_opt_str(s: &Option<String>) -> String {
    s.as_deref().map_or("null".to_string(), json_str)
}

/// An optional number as a JSON number or `null`.
fn json_opt_num<T: std::fmt::Display>(v: Option<T>) -> String {
    v.map_or("null".to_string(), |v| v.to_string())
}

/// A float as a JSON number; a non-finite value becomes `null` rather than invalid JSON.
fn json_f64(v: f64) -> String {
    if v.is_finite() {
        format!("{v:.3}")
    } else {
        "null".to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stream::Gap;

    fn framing(records: u64, gaps: Vec<Gap>) -> Framing {
        let lost = gaps.iter().map(|g| g.missing).sum();
        Framing {
            records,
            first_seq: (records > 0).then_some(1),
            last_seq: (records > 0).then_some(records as u32),
            gaps,
            lost_records: lost,
            ..Framing::default()
        }
    }

    fn tier(framing: Framing) -> Tier {
        let verdict = Verdict::of(&framing, 250_000, 200_000);
        Tier {
            requested_baud: 2_000_000,
            probe_baud: 2_000_000,
            dut_baud: Some(2_000_000),
            window_ms: 1000.0,
            captured_bytes: 250_000,
            kib_per_s: 244.1,
            payload_bytes: 200_000,
            itm_overflow: 0,
            decode_errors: 0,
            framing,
            polls: 400,
            full_polls: 100,
            inactive_polls: 0,
            probe_bytes_left: 0,
            swd_kib_per_s: None,
            swd_mismatches: 0,
            swd_errors: 0,
            verdict,
        }
    }

    #[test]
    fn verdict_follows_the_sequence_tags() {
        assert_eq!(Verdict::of(&framing(10, vec![]), 100, 80), Verdict::Clean);
        assert_eq!(Verdict::of(&framing(0, vec![]), 0, 0), Verdict::Silent);
        let gap = Gap {
            after_seq: 4,
            missing: 3,
            byte_offset: 40,
        };
        assert_eq!(Verdict::of(&framing(10, vec![gap]), 100, 80), Verdict::Gaps);
    }

    #[test]
    fn a_full_capture_that_decodes_to_little_is_misframed_not_clean() {
        // The recorded shape: line-rate byte volume, consecutive decoded subset, low yield.
        let f = framing(1_266, vec![]);
        assert!(f.is_clean());
        assert_eq!(Verdict::of(&f, 500_111, 10_128), Verdict::Misframed);
        // Zero records from a full byte count is a misframe too, never SILENT.
        assert_eq!(
            Verdict::of(&framing(0, vec![]), 57_620, 0),
            Verdict::Misframed
        );
    }

    #[test]
    fn a_legitimately_silent_tier_is_not_swept_into_the_misframe_bucket() {
        // Nothing on the wire, or only a ragged scrap below the gate's floor.
        assert_eq!(Verdict::of(&framing(0, vec![]), 0, 0), Verdict::Silent);
        assert_eq!(Verdict::of(&framing(0, vec![]), 64, 0), Verdict::Silent);
    }

    #[test]
    fn the_yield_gate_sits_between_the_recorded_populations() {
        // Worst recorded genuine clean tier: 0.7745; best recorded misframed tier: 0.5232.
        assert!(!below_yield_gate(1_960_209, 1_518_152));
        assert!(below_yield_gate(250_025, 130_824));
        assert!(!below_yield_gate(250_000, 200_000));
        // Below the floor the fraction means nothing.
        assert!(!below_yield_gate(100, 0));
    }

    #[test]
    fn the_header_and_a_row_have_the_same_column_count() {
        let header = column_header();
        let row = render(&tier(framing(10, vec![])));
        assert_eq!(
            header.split_whitespace().count(),
            row.split_whitespace().count()
        );
        assert!(row.contains("clean"));
    }

    #[test]
    fn a_row_without_a_load_prints_a_dash_for_swd_throughput() {
        let row = render(&tier(framing(10, vec![])));
        assert!(row.contains(" - "));
        let mut t = tier(framing(10, vec![]));
        t.swd_kib_per_s = Some(512.25);
        assert!(render(&t).contains("512.2"));
    }

    #[test]
    fn a_clean_tier_says_nothing_and_a_gapped_one_names_positions() {
        assert!(notes(&tier(framing(10, vec![]))).is_empty());
        let gap = Gap {
            after_seq: 4,
            missing: 3,
            byte_offset: 40,
        };
        let notes = notes(&tier(framing(10, vec![gap])));
        assert_eq!(notes.len(), 1);
        assert!(notes[0].contains("3 record(s) lost after seq 4"));
        assert!(notes[0].contains("offset 40"));
    }

    #[test]
    fn many_gaps_are_truncated_with_a_total() {
        let gaps: Vec<Gap> = (0..10)
            .map(|i| Gap {
                after_seq: i,
                missing: 1,
                byte_offset: i as usize * 8,
            })
            .collect();
        let notes = notes(&tier(framing(20, gaps)));
        assert_eq!(notes.len(), GAPS_SHOWN + 1);
        assert!(notes.last().unwrap().contains("6 further gap(s)"));
    }

    #[test]
    fn a_silent_tier_is_called_out_first() {
        let mut t = tier(framing(0, vec![]));
        t.captured_bytes = 0;
        t.payload_bytes = 0;
        t.verdict = Verdict::of(&t.framing, 0, 0);
        let notes = notes(&t);
        assert!(notes[0].contains("no record framed"));
    }

    #[test]
    fn a_misframed_tier_names_its_yield() {
        let mut t = tier(framing(1_266, vec![]));
        t.captured_bytes = 500_111;
        t.payload_bytes = 10_128;
        t.verdict = Verdict::of(&t.framing, t.captured_bytes, t.payload_bytes);
        assert_eq!(t.verdict, Verdict::Misframed);
        assert!(render(&t).contains("MISFRAMED"));
        let notes = notes(&t);
        assert!(notes.iter().any(|n| n.contains("misframed")), "{notes:?}");
        assert!(notes.iter().any(|n| n.contains("2%")), "{notes:?}");
    }

    #[test]
    fn polls_per_s_scales_the_count_to_the_window() {
        let mut t = tier(framing(1, vec![]));
        t.polls = 400;
        t.window_ms = 1000.0;
        assert_eq!(polls_per_s(&t), 400.0);
        t.window_ms = 500.0;
        assert_eq!(polls_per_s(&t), 800.0);
        t.window_ms = 0.0;
        assert_eq!(polls_per_s(&t), 0.0);
    }

    #[test]
    fn an_inactive_capture_is_named_before_the_loss_it_explains() {
        let mut t = tier(framing(1, vec![]));
        t.polls = 400;
        assert!(notes(&t).iter().all(|n| !n.contains("capture inactive")));
        t.inactive_polls = 7;
        let n = notes(&t);
        assert!(n[0].contains("7 of 400 poll(s) with capture inactive"));
    }

    #[test]
    fn full_poll_percent_is_zero_without_polls() {
        let mut t = tier(framing(1, vec![]));
        t.polls = 0;
        assert_eq!(full_poll_percent(&t), 0.0);
        t.polls = 4;
        t.full_polls = 1;
        assert_eq!(full_poll_percent(&t), 25.0);
    }

    #[test]
    fn json_states_the_context_and_every_row() {
        let ctx = Context {
            timestamp_utc: "2026-01-01T00:00:00Z".to_string(),
            probe_product: Some("probe".to_string()),
            probe_serial: None,
            firmware_version: Some("2.1.0".to_string()),
            packet_size: Some(512),
            packet_count: Some(4),
            trace_buffer_bytes: Some(1023),
            target: "stm32f401",
            window_ms: 1000,
            depth: 2,
            depth_rationale: "requested",
            swd_load: true,
            dut_reset: true,
            bauds: &[1_000_000, 2_000_000],
        };
        let gap = Gap {
            after_seq: 4,
            missing: 3,
            byte_offset: 40,
        };
        let json = to_json(&ctx, &[tier(framing(10, vec![gap]))]);
        assert!(json.starts_with("{\n"));
        assert!(json.ends_with("}\n"));
        assert!(json.contains("\"schema_version\": 10"));
        assert!(json.contains("\"depth_rationale\": \"requested\""));
        assert!(json.contains("\"dut_reset\": true"));
        assert!(json.contains("\"serial\": null"));
        assert!(json.contains("\"swd_kib_per_s\": null"));
        assert!(json.contains("\"verdict\": \"GAPS\""));
        assert!(json.contains("\"after_seq\": 4"));
        assert!(json.contains("\"lost_records\": 3"));
        assert_eq!(json.matches("\"requested_baud\"").count(), 1);
    }

    /// The context the JSON tests share.
    fn context() -> Context<'static> {
        Context {
            timestamp_utc: "2026-01-01T00:00:00Z".to_string(),
            probe_product: None,
            probe_serial: None,
            firmware_version: None,
            packet_size: Some(64),
            packet_count: Some(4),
            trace_buffer_bytes: Some(1023),
            target: "stm32f401",
            window_ms: 5000,
            depth: 4,
            depth_rationale: "requested",
            swd_load: false,
            dut_reset: true,
            bauds: &[4_000_000],
        }
    }

    /// A tier whose tags a resync corrupted, as a 4 MBd RTIC row is.
    fn resynced_tier() -> Tier {
        let gap = Gap {
            after_seq: 12,
            missing: 3_221_225_472,
            byte_offset: 4_096,
        };
        let mut t = tier(framing(200_000, vec![gap]));
        t.framing.resync_events = 5;
        t.framing.resync_bytes = 37;
        t
    }

    #[test]
    fn a_corrupted_lost_total_is_withheld_from_the_json_not_serialized_raw() {
        let json = to_json(&context(), &[resynced_tier()]);
        assert!(json.contains("\"lost_records\": null"), "{json}");
        assert!(!json.contains("3221225472"), "{json}");
        // The gap's position is still evidence; only the count it implies is unknowable.
        assert!(json.contains("\"missing\": null"), "{json}");
        assert!(json.contains("\"after_seq\": 12"));
        assert!(json.contains("\"resync_events\": 5"));
    }

    #[test]
    fn the_notes_mark_a_corrupted_gap_the_way_the_table_does() {
        let t = resynced_tier();
        assert_eq!(lost_display(&t), "?");
        let notes = notes(&t);
        assert!(
            notes
                .iter()
                .any(|n| n.contains("? record(s) lost after seq 12")),
            "{notes:?}"
        );
        assert!(!notes.iter().any(|n| n.contains("3221225472")), "{notes:?}");
    }

    #[test]
    fn json_escapes_strings_and_rejects_non_finite_numbers() {
        assert_eq!(json_str("a\"b\\c\n"), "\"a\\\"b\\\\c\\n\"");
        assert_eq!(json_f64(f64::NAN), "null");
        assert_eq!(json_f64(1.5), "1.500");
        assert_eq!(json_opt_num::<u16>(None), "null");
    }
}
