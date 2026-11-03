//! Table and JSON rendering of measured runs and sweeps.

use std::fmt::Write as _;

use crate::adi::{TarPolicy, TransferShape};
use crate::fit::FitRow;
use crate::measure::{AbortClass, RepeatSet};
use crate::transport::{RefillPolicy, SubmitPolicy};

/// Column header for [`render`].
pub fn column_header() -> String {
    format!(
        "{:>5}  {:>10}  {:>9}  {:>9}  {:>9}  {:>5}  {:>6}  {:>5}",
        "dir", "clock_hz", "median", "min", "max", "n", "aborts", "depth"
    )
}

/// Render one [`RepeatSet`] as an aligned table row.
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

/// The probe and run identity both JSON documents open with.
pub struct Header<'a> {
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
    /// The firmware registry row the record measured, when anything names it.
    pub firmware_row: Option<String>,
    /// What named it: `image`, `operator` or `none`.
    pub firmware_row_source: &'static str,
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

/// Run-level context the JSON document states once.
pub struct RunContext<'a> {
    /// The shared opening fields.
    pub header: Header<'a>,
    /// Repeats attempted per rung.
    pub repeats: u32,
    /// Requested clock sweep, in Hz, in sweep order.
    pub clocks_hz: &'a [u32],
    /// Pipeline-depth sweep, post-clamp, in sweep order.
    pub depths: &'a [usize],
    /// The host-policy flags this run was measured under.
    pub host_policy: HostPolicy,
    /// The error that ended the run before its last rung; `rows` then holds what was measured.
    pub error: Option<&'a str>,
}

/// The host-side submission and addressing policy a run used.
#[derive(Debug, Clone, Copy)]
pub struct HostPolicy {
    /// Where block transfers re-arm TAR.
    pub tar: TarPolicy,
    /// The command shape block transfers were issued as.
    pub shape: TransferShape,
    /// URB order at depth 1; depth >= 2 always posts IN before OUT.
    pub submit: SubmitPolicy,
    /// Whether each pipelined batch's IN URBs were held back until every OUT had completed.
    pub out_burst: bool,
    /// When a completed pipelined command's credit was replaced.
    pub refill: RefillPolicy,
    /// Commands a 4 KiB read costs under `tar` and `shape` at the negotiated packet size.
    pub commands_per_chunk_read: usize,
    /// Commands a 4 KiB write costs under `tar` and `shape` at the negotiated packet size.
    pub commands_per_chunk_write: usize,
}

/// Serialize a run to schema 5; absent values and nonfinite numbers become `null`.
/// Any change to the emitted fields or their order is a schema version bump.
pub fn to_json(ctx: &RunContext, rows: &[RepeatSet], fit: &[FitRow]) -> String {
    let mut out = String::new();
    write_header(&mut out, &ctx.header);
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

    let hp = &ctx.host_policy;
    let _ = writeln!(out, "  \"host_policy\": {{");
    let _ = writeln!(out, "    \"tar\": {},", json_str(hp.tar.label()));
    let _ = writeln!(out, "    \"shape\": {},", json_str(hp.shape.label()));
    let _ = writeln!(out, "    \"submit\": {},", json_str(hp.submit.label()));
    let _ = writeln!(out, "    \"out_burst\": {},", hp.out_burst);
    let _ = writeln!(out, "    \"refill\": {},", json_str(hp.refill.label()));
    let _ = writeln!(out, "    \"commands_per_chunk\": {{");
    let _ = writeln!(out, "      \"read\": {},", hp.commands_per_chunk_read);
    let _ = writeln!(out, "      \"write\": {}", hp.commands_per_chunk_write);
    let _ = writeln!(out, "    }}");
    let _ = writeln!(out, "  }},");
    let _ = writeln!(
        out,
        "  \"error\": {},",
        ctx.error.map_or_else(|| "null".to_string(), json_str)
    );

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

/// Serialize a sweep to schema 5: the [`Header`] plus a `"sweep"` object.
pub fn sweep_to_json(
    ctx: &Header,
    rows: &[crate::sweep::SweepRow],
    plateau_hz: Option<u32>,
    failure_interval_hz: (u32, Option<u32>),
) -> String {
    let mut out = String::new();
    write_header(&mut out, ctx);

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

/// Open a schema-6 document with the [`Header`] fields; the caller's fields follow.
fn write_header(out: &mut String, h: &Header) {
    let _ = writeln!(out, "{{");
    let _ = writeln!(out, "  \"schema_version\": 6,");
    let _ = writeln!(out, "  \"timestamp_utc\": {},", json_str(&h.timestamp_utc));

    let _ = writeln!(out, "  \"probe\": {{");
    let _ = writeln!(out, "    \"vendor\": {},", json_opt_str(&h.probe_vendor));
    let _ = writeln!(out, "    \"product\": {},", json_opt_str(&h.probe_product));
    let _ = writeln!(out, "    \"serial\": {},", json_opt_str(&h.probe_serial));
    let _ = writeln!(
        out,
        "    \"firmware_version\": {},",
        json_opt_str(&h.firmware_version)
    );
    let _ = writeln!(
        out,
        "    \"firmware_row\": {},",
        json_opt_str(&h.firmware_row)
    );
    let _ = writeln!(
        out,
        "    \"firmware_row_source\": {},",
        json_str(h.firmware_row_source)
    );
    let _ = writeln!(out, "    \"packet_size\": {},", json_opt_num(h.packet_size));
    let _ = writeln!(
        out,
        "    \"packet_count\": {}",
        json_opt_num(h.packet_count)
    );
    let _ = writeln!(out, "  }},");

    let _ = writeln!(out, "  \"protocol\": {},", json_str(h.protocol));
    let _ = writeln!(out, "  \"target\": {},", json_str(h.target));
    let _ = writeln!(out, "  \"window_ms\": {},", h.window_ms);
    let _ = writeln!(out, "  \"chunk_bytes\": {},", h.chunk_bytes);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measure::tests::repeat_set;
    use crate::measure::Direction;

    #[test]
    fn format_iso8601_utc_matches_a_known_instant() {
        // 2024-01-02T03:04:05Z.
        assert_eq!(format_iso8601_utc(1_704_164_645), "2024-01-02T03:04:05Z");
        assert_eq!(format_iso8601_utc(0), "1970-01-01T00:00:00Z");
    }

    /// The host policy of a default `run`.
    fn default_host_policy() -> HostPolicy {
        HostPolicy {
            tar: TarPolicy::PerChunk,
            shape: TransferShape::Block,
            submit: SubmitPolicy::Lockstep,
            out_burst: false,
            refill: RefillPolicy::AfterHistory,
            commands_per_chunk_read: 144,
            commands_per_chunk_write: 152,
        }
    }

    #[test]
    fn to_json_states_the_default_host_policy() {
        let ctx = RunContext {
            header: Header {
                timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
                probe_vendor: None,
                probe_product: None,
                probe_serial: None,
                firmware_version: None,
                firmware_row: Some("firmware/mculink".to_string()),
                firmware_row_source: "image",
                packet_size: Some(64),
                packet_count: Some(4),
                protocol: "swd",
                target: "stm32",
                window_ms: 500,
                chunk_bytes: 4096,
            },
            repeats: 1,
            clocks_hz: &[25_000_000],
            depths: &[1],
            host_policy: default_host_policy(),
            error: None,
        };
        let rows = [repeat_set(Direction::Read, 25_000_000, &[100.0], &[])];
        let json = to_json(&ctx, &rows, &[]);
        assert!(json.contains("\"tar\": \"per-chunk\""));
        assert!(json.contains("\"shape\": \"block\""));
        assert!(json.contains("\"submit\": \"lockstep\""));
        assert!(json.contains("\"out_burst\": false"));
        assert!(json.contains("\"refill\": \"after-history\""));
        assert!(json.contains("\"read\": 144"));
        assert!(json.contains("\"write\": 152"));
    }

    #[test]
    fn to_json_states_a_changed_host_policy_and_its_command_counts() {
        let ctx = RunContext {
            header: Header {
                timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
                probe_vendor: None,
                probe_product: None,
                probe_serial: None,
                firmware_version: None,
                firmware_row: Some("firmware/mculink".to_string()),
                firmware_row_source: "image",
                packet_size: Some(64),
                packet_count: Some(4),
                protocol: "swd",
                target: "stm32",
                window_ms: 500,
                chunk_bytes: 4096,
            },
            repeats: 1,
            clocks_hz: &[25_000_000],
            depths: &[4],
            host_policy: HostPolicy {
                tar: TarPolicy::PerWindow,
                shape: TransferShape::Mixed,
                submit: SubmitPolicy::InFirst,
                out_burst: true,
                refill: RefillPolicy::Prompt,
                commands_per_chunk_read: 72,
                commands_per_chunk_write: 96,
            },
            error: None,
        };
        let rows = [repeat_set(Direction::Read, 25_000_000, &[100.0], &[])];
        let json = to_json(&ctx, &rows, &[]);
        assert!(json.contains("\"tar\": \"per-window\""));
        assert!(json.contains("\"shape\": \"mixed\""));
        assert!(json.contains("\"submit\": \"in-first\""));
        assert!(json.contains("\"out_burst\": true"));
        assert!(json.contains("\"refill\": \"prompt\""));
        assert!(json.contains("\"read\": 72"));
        assert!(json.contains("\"write\": 96"));
    }

    #[test]
    fn to_json_emits_the_versioned_schema() {
        let ctx = RunContext {
            header: Header {
                timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
                probe_vendor: Some("ARM".to_string()),
                probe_product: Some("DAPLink".to_string()),
                probe_serial: None,
                firmware_version: Some("0260".to_string()),
                firmware_row: Some("firmware/mculink".to_string()),
                firmware_row_source: "image",
                packet_size: Some(512),
                packet_count: Some(4),
                protocol: "swd",
                target: "stm32",
                window_ms: 500,
                chunk_bytes: 4096,
            },
            repeats: 3,
            clocks_hz: &[1_000_000, 2_000_000],
            depths: &[1],
            host_policy: default_host_policy(),
            error: None,
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

        assert!(json.starts_with("{\n  \"schema_version\": 6,\n"));
        assert!(json.contains("\"timestamp_utc\": \"2024-01-02T03:04:05Z\""));
        assert!(json.contains("\"vendor\": \"ARM\""));
        assert!(json.contains("\"serial\": null"));
        assert!(json.contains("\"protocol\": \"swd\""));
        assert!(json.contains("\"repeats\": 3,"));
        assert!(json.contains("\"clocks_hz\": [1000000, 2000000]"));
        assert!(json.contains("\"depths\": [1]"));
        assert!(json.contains("\"error\": null,\n  \"rows\": ["));
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
        assert!(probe_block.contains("\"firmware_row\": \"firmware/mculink\""));
        assert!(probe_block.contains("\"firmware_row_source\": \"image\""));
        assert!(probe_block.contains("\"packet_size\": 512"));
        assert!(probe_block.contains("\"packet_count\": 4"));
    }

    #[test]
    fn sweep_to_json_emits_the_nested_sweep_object() {
        let ctx = Header {
            timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
            probe_vendor: None,
            probe_product: Some("DAPLink".to_string()),
            probe_serial: None,
            firmware_version: None,
            firmware_row: Some("firmware/mculink".to_string()),
            firmware_row_source: "image",
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

        assert!(json.starts_with("{\n  \"schema_version\": 6,\n"));
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
        let ctx = Header {
            timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
            probe_vendor: None,
            probe_product: None,
            probe_serial: None,
            firmware_version: None,
            firmware_row: Some("firmware/mculink".to_string()),
            firmware_row_source: "image",
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
            header: Header {
                timestamp_utc: "2024-01-02T03:04:05Z".to_string(),
                probe_vendor: None,
                probe_product: Some("quote\"here".to_string()),
                probe_serial: None,
                firmware_version: None,
                firmware_row: Some("firmware/mculink".to_string()),
                firmware_row_source: "image",
                packet_size: None,
                packet_count: None,
                protocol: "swd",
                target: "stm32",
                window_ms: 500,
                chunk_bytes: 4096,
            },
            repeats: 1,
            clocks_hz: &[],
            depths: &[],
            host_policy: default_host_policy(),
            error: None,
        };
        let json = to_json(&ctx, &[], &[]);
        assert!(json.contains("\"product\": \"quote\\\"here\""));
        assert!(json.contains("\"clocks_hz\": []"));
        assert!(json.contains("\"rows\": [\n  ]"));
    }

    #[test]
    fn record_corpus_matches_byte_for_byte() {
        let ctx = RunContext {
            header: Header {
                timestamp_utc: "2026-01-02T03:04:05Z".to_string(),
                probe_vendor: Some("vendor\"\\\n\t\u{0001}".to_string()),
                probe_product: None,
                probe_serial: Some("serial\rnumber".to_string()),
                firmware_version: None,
                firmware_row: Some("firmware/mculink".to_string()),
                firmware_row_source: "image",
                packet_size: None,
                packet_count: Some(4),
                protocol: "swd",
                target: "target\"name",
                window_ms: 500,
                chunk_bytes: 4096,
            },
            repeats: 5,
            clocks_hz: &[1_000_000, 2_000_000],
            depths: &[1, 4],
            host_policy: default_host_policy(),
            error: Some(
                "write at 2000000 Hz, depth 1: all 2 repeats aborted (halt 2); first: \"x\"",
            ),
        };
        let rows = [
            repeat_set(
                Direction::Read,
                1_000_000,
                &[100.0, f64::NAN],
                &[AbortClass::Attach, AbortClass::Guard, AbortClass::Transfer],
            ),
            repeat_set(
                Direction::Write,
                2_000_000,
                &[],
                &[AbortClass::Halt, AbortClass::Halt],
            ),
        ];
        let fit = [FitRow {
            direction: Direction::Read,
            depth: 4,
            block_words: 15,
            intercept_us: f64::NAN,
            slope: f64::INFINITY,
            ideal_slope: 690.0,
            r_squared: 0.75,
            top_clock_hz: 2_000_000,
            effective_clock_hz_top: f64::NEG_INFINITY,
            wire_us_top: 1.25,
            plateau_ceiling_hz: None,
        }];
        let run = to_json(&ctx, &rows, &fit);
        assert_eq!(run, include_str!("report/run-corpus.json"));

        let sweep_rows = [
            crate::sweep::SweepRow {
                clock_hz: 1_000_000,
                kib_per_s: 123.4,
                iterations: 10,
                verdict: crate::sweep::Verdict::Pass,
                depth: 1,
            },
            crate::sweep::SweepRow {
                clock_hz: 2_000_000,
                kib_per_s: f64::NAN,
                iterations: 0,
                verdict: crate::sweep::Verdict::Fail,
                depth: 4,
            },
        ];
        let sweep = sweep_to_json(&ctx.header, &sweep_rows, None, (1_000_000, None));
        assert_eq!(sweep, include_str!("report/sweep-corpus.json"));
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
