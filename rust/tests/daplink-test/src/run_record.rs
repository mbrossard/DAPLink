//! JSONL recording for a HIL run: one appended record per case, under git-ignored `artifacts/`.

use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use probe_rs::probe::{DebugProbeSelector, WireProtocol};

use crate::speed::{self, CHUNK_BYTES};

/// Record format version: bumped on a field's meaning changing, so a reader can refuse a record.
pub(crate) const SCHEMA_VERSION: u32 = 1;

/// Default record file name under the resolved `artifacts/` directory.
const DEFAULT_FILE_NAME: &str = "hil-runs.jsonl";

/// The marker identifying the checkout root while walking up from a starting path.
const CHECKOUT_MARKER: &str = "rust/tests/daplink-test/Cargo.toml";

/// What every case record of one invocation shares: probe, images, target, harness version.
pub(crate) struct RunMeta {
    /// Groups the records of one invocation exactly, rather than by timestamp proximity.
    pub(crate) run_id: String,
    /// Probe USB vendor id.
    pub(crate) vid: u16,
    /// Probe USB product id.
    pub(crate) pid: u16,
    /// Probe USB serial, when it reports one.
    pub(crate) serial: Option<String>,
    /// The probe's own reported firmware-version string, when the read succeeded.
    pub(crate) firmware_version: Option<String>,
    /// This harness binary's own linked version string (`daplink_core::DAP_FW_VERSION`).
    pub(crate) harness_version: &'static str,
    /// Target registry name.
    pub(crate) target: &'static str,
    /// probe-rs chip name the run attaches with.
    pub(crate) chip: String,
    /// DUT ELF path, when the run was given one.
    pub(crate) dut_elf: Option<String>,
    /// DUT uf2 path, when the run was given one.
    pub(crate) dut_uf2: Option<String>,
}

impl RunMeta {
    /// Build the shared run header from what `main` already read; nothing is re-queried here.
    pub(crate) fn new(
        run_id: String,
        selector: &DebugProbeSelector,
        firmware_version: Option<&str>,
        target: &'static str,
        chip: &str,
        dut_elf: Option<&str>,
        dut_uf2: Option<&str>,
    ) -> Self {
        RunMeta {
            run_id,
            vid: selector.vendor_id,
            pid: selector.product_id,
            serial: selector.serial_number.clone(),
            firmware_version: firmware_version.map(str::to_string),
            harness_version: daplink_core::DAP_FW_VERSION,
            target,
            chip: chip.to_string(),
            dut_elf: dut_elf.map(str::to_string),
            dut_uf2: dut_uf2.map(str::to_string),
        }
    }
}

/// One case's outcome and cost.
pub(crate) struct CaseRecord<'a> {
    /// Registry case name.
    pub(crate) case: &'a str,
    /// `ok`, `failed`, `fatal` or `skip` -- the harness's `Failure` taxonomy.
    pub(crate) outcome: &'static str,
    /// Why it failed, was fatal, or was skipped; `None` for `ok`.
    pub(crate) detail: Option<&'a str>,
    /// Wall clock the case took, including a skip's near-zero.
    pub(crate) duration_ms: u64,
    /// The speed sweep's rungs, on the record of a case that swept (`speed-swd`, `speed-jtag`).
    pub(crate) speed: Option<SpeedPayload>,
}

/// A speed case's own measurements.
pub(crate) struct SpeedPayload {
    /// Transport the sweep ran over.
    pub(crate) transport: &'static str,
    /// Bytes read per timed chunk, constant across rungs.
    pub(crate) chunk_bytes: usize,
    /// The rungs, in sweep order.
    pub(crate) rows: Vec<RowRecord>,
}

impl SpeedPayload {
    /// Build a payload from the raw sweep this run stashed on `Ctx::last_speed`.
    pub(crate) fn from_steps(protocol: WireProtocol, steps: &[speed::Step]) -> Self {
        SpeedPayload {
            transport: transport_name(protocol),
            chunk_bytes: CHUNK_BYTES,
            rows: steps
                .iter()
                .map(|s| RowRecord {
                    requested_hz: s.requested_hz,
                    chunk_ns: s.chunk_ns.clone(),
                })
                .collect(),
        }
    }
}

/// One rung's raw measurement, as recorded.
pub(crate) struct RowRecord {
    /// Requested wire clock, Hz.
    pub(crate) requested_hz: u32,
    /// Elapsed nanoseconds of each timed read, in the order they ran.
    pub(crate) chunk_ns: Vec<u128>,
}

/// The record's spelling of a wire transport.
fn transport_name(protocol: WireProtocol) -> &'static str {
    match protocol {
        WireProtocol::Swd => "swd",
        WireProtocol::Jtag => "jtag",
    }
}

/// An id shared by every record of one invocation.
pub(crate) fn run_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    format!("{:x}-{nanos:x}", std::process::id())
}

/// The default record path: `artifacts/` under the checkout root above `start`, or under `start`
/// itself when no ancestor matches.
pub(crate) fn default_path(start: &Path) -> PathBuf {
    for ancestor in start.ancestors() {
        if ancestor.join(CHECKOUT_MARKER).is_file() {
            return ancestor.join("artifacts").join(DEFAULT_FILE_NAME);
        }
    }
    start.join("artifacts").join(DEFAULT_FILE_NAME)
}

/// [`default_path`] from the running binary's location, else the current directory.
pub(crate) fn default_path_for_process() -> PathBuf {
    let start = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(Path::to_path_buf))
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_else(|| PathBuf::from("."));
    default_path(&start)
}

/// Append one case record to `path`, creating the file and its directory; never truncates.
pub(crate) fn append(path: &Path, meta: &RunMeta, case: &CaseRecord) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let line = render(meta, case, secs);
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    file.write_all(line.as_bytes())
}

/// Render one case record as a single JSON line, terminated by a newline.
pub(crate) fn render(meta: &RunMeta, case: &CaseRecord, unix_s: u64) -> String {
    let rows = case.speed.as_ref().map(|s| s.rows.len()).unwrap_or(0);
    let mut s = String::with_capacity(384 + rows * 96);
    s.push_str(&format!(
        "{{\"schema_version\":{SCHEMA_VERSION},\"run_id\":{},\"unix_s\":{unix_s},",
        quote(&meta.run_id)
    ));
    s.push_str(&format!("\"utc\":{},", quote(&utc_iso(unix_s))));
    s.push_str(&format!(
        "\"harness_version\":{},",
        quote(meta.harness_version)
    ));
    s.push_str(&format!(
        "\"probe\":{{\"vid\":\"{:04x}\",\"pid\":\"{:04x}\",\"serial\":{}}},",
        meta.vid,
        meta.pid,
        opt_str(meta.serial.as_deref())
    ));
    s.push_str(&format!(
        "\"firmware_version\":{},",
        opt_str(meta.firmware_version.as_deref())
    ));
    s.push_str(&format!(
        "\"target\":{},\"chip\":{},",
        quote(meta.target),
        quote(&meta.chip)
    ));
    s.push_str(&format!(
        "\"dut_elf\":{},\"dut_uf2\":{},",
        opt_str(meta.dut_elf.as_deref()),
        opt_str(meta.dut_uf2.as_deref())
    ));
    s.push_str(&format!(
        "\"case\":{},\"outcome\":{},\"detail\":{},\"duration_ms\":{}",
        quote(case.case),
        quote(case.outcome),
        opt_str(case.detail),
        case.duration_ms
    ));
    if let Some(p) = &case.speed {
        s.push_str(&format!(
            ",\"speed\":{{\"transport\":{},\"chunk_bytes\":{},\"rows\":[",
            quote(p.transport),
            p.chunk_bytes
        ));
        for (i, r) in p.rows.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&render_row(r));
        }
        s.push_str("]}");
    }
    s.push_str("}\n");
    s
}

/// Render one rung object.
fn render_row(r: &RowRecord) -> String {
    format!(
        "{{\"requested_hz\":{},\"chunk_ns\":{}}}",
        r.requested_hz,
        ns_array(&r.chunk_ns)
    )
}

/// Render a list of elapsed times as a JSON array.
fn ns_array(ns: &[u128]) -> String {
    let mut s = String::from("[");
    for (i, v) in ns.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&v.to_string());
    }
    s.push(']');
    s
}

/// A JSON string, or `null` for an absent value.
fn opt_str(v: Option<&str>) -> String {
    match v {
        Some(s) => quote(s),
        None => "null".to_string(),
    }
}

/// Quote and escape a JSON string.
fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// Format a Unix timestamp as `YYYY-MM-DDTHH:MM:SSZ`.
fn utc_iso(unix_s: u64) -> String {
    let days = (unix_s / 86_400) as i64;
    let rem = unix_s % 86_400;
    let (y, m, d) = civil_from_days(days);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z",
        rem / 3600,
        (rem % 3600) / 60,
        rem % 60
    )
}

/// Days since the Unix epoch to a civil `(year, month, day)`, by the shift-the-era method.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The exact line the renderer emits for a swept case, as a reader parses it back.
    const PINNED_SWEEP: &str = r#"{"schema_version":1,"run_id":"1f4-17a2b","unix_s":1754378400,"utc":"2025-08-05T07:20:00Z","harness_version":"2.1.1.gfeedc0ffee12","probe":{"vid":"0d28","pid":"0204","serial":"0000ABCDEF012345"},"firmware_version":"2.1.1.gfeedc0ffee12","target":"rp2040","chip":"RP2040","dut_elf":"target/rp2040.elf","dut_uf2":null,"case":"speed-swd","outcome":"ok","detail":null,"duration_ms":8421,"speed":{"transport":"swd","chunk_bytes":4096,"rows":[{"requested_hz":25000000,"chunk_ns":[71500,71600,71400]}]}}"#;

    /// The exact line for a case that swept nothing: the `speed` key is absent, not null.
    const PINNED_PLAIN: &str = r#"{"schema_version":1,"run_id":"1f4-17a2b","unix_s":1754378400,"utc":"2025-08-05T07:20:00Z","harness_version":"2.1.1.gfeedc0ffee12","probe":{"vid":"0d28","pid":"0204","serial":"0000ABCDEF012345"},"firmware_version":"2.1.1.gfeedc0ffee12","target":"rp2040","chip":"RP2040","dut_elf":"target/rp2040.elf","dut_uf2":null,"case":"msc","outcome":"failed","detail":"drop 2 of 3 never re-enumerated","duration_ms":61230}"#;

    /// The rendered line is pinned in full, raw per-read nanoseconds included: a derived rate
    /// would round away the equal-time signature a clamped clock leaves.
    #[test]
    fn a_swept_case_renders_one_line_with_raw_times() {
        let case = CaseRecord {
            case: "speed-swd",
            outcome: "ok",
            detail: None,
            duration_ms: 8421,
            speed: Some(SpeedPayload {
                transport: "swd",
                chunk_bytes: 4096,
                rows: vec![RowRecord {
                    requested_hz: 25_000_000,
                    chunk_ns: vec![71_500, 71_600, 71_400],
                }],
            }),
        };
        let line = render(&meta_for_test(), &case, 1_754_378_400);
        assert_eq!(line, format!("{PINNED_SWEEP}\n"));
    }

    /// Every case records its cost, not only the ones that sweep.
    #[test]
    fn a_case_that_swept_nothing_records_its_cost() {
        let case = CaseRecord {
            case: "msc",
            outcome: "failed",
            detail: Some("drop 2 of 3 never re-enumerated"),
            duration_ms: 61_230,
            speed: None,
        };
        let line = render(&meta_for_test(), &case, 1_754_378_400);
        assert_eq!(line, format!("{PINNED_PLAIN}\n"));
        assert!(!line.contains("\"speed\""), "{line}");
    }

    /// A sweep runs over its own transport, whatever the rest of the run attached with.
    #[test]
    fn a_sweeps_own_transport_is_recorded() {
        let payload = SpeedPayload::from_steps(
            WireProtocol::Jtag,
            &[speed::Step {
                requested_hz: 1_000_000,
                kib_per_s: 123.0,
                chunks: 4,
                chunk_ns: vec![1, 2, 3, 4],
            }],
        );
        assert_eq!(payload.transport, "jtag");
        assert_eq!(payload.rows[0].chunk_ns, vec![1, 2, 3, 4]);
    }

    /// The taxonomy's four outcomes render as themselves; nothing infers one.
    #[test]
    fn outcomes_render_as_given() {
        for outcome in ["ok", "failed", "fatal", "skip"] {
            let case = CaseRecord {
                case: "connect",
                outcome,
                detail: None,
                duration_ms: 1,
                speed: None,
            };
            let line = render(&meta_for_test(), &case, 0);
            assert!(
                line.contains(&format!("\"outcome\":\"{outcome}\"")),
                "{line}"
            );
        }
    }

    /// An absent field renders as `null`, never the string `"None"`.
    #[test]
    fn absent_keys_render_as_null() {
        let mut meta = meta_for_test();
        meta.serial = None;
        meta.firmware_version = None;
        meta.dut_elf = None;
        let case = CaseRecord {
            case: "connect",
            outcome: "ok",
            detail: None,
            duration_ms: 1,
            speed: None,
        };
        let line = render(&meta, &case, 0);
        assert!(line.contains("\"serial\":null"), "{line}");
        assert!(line.contains("\"firmware_version\":null"), "{line}");
        assert!(line.contains("\"dut_elf\":null"), "{line}");
        assert!(line.contains("\"detail\":null"), "{line}");
    }

    /// A failure reason lands in the record verbatim, so it has to be escaped.
    #[test]
    fn strings_are_escaped() {
        assert_eq!(quote("a\"b\\c"), "\"a\\\"b\\\\c\"");
        assert_eq!(quote("a\nb"), "\"a\\nb\"");
        assert_eq!(quote("\u{1}"), "\"\\u0001\"");
        let case = CaseRecord {
            case: "msc",
            outcome: "failed",
            detail: Some("mount \"/Volumes/DAPLINK\" went away"),
            duration_ms: 1,
            speed: None,
        };
        let line = render(&meta_for_test(), &case, 0);
        assert!(line.contains(r#"\"/Volumes/DAPLINK\""#), "{line}");
    }

    #[test]
    fn utc_covers_epoch_leap_years_and_century_rules() {
        assert_eq!(utc_iso(0), "1970-01-01T00:00:00Z");
        assert_eq!(utc_iso(951_782_400), "2000-02-29T00:00:00Z");
        assert_eq!(utc_iso(1_709_164_800), "2024-02-29T00:00:00Z");
        assert_eq!(utc_iso(1_735_689_599), "2024-12-31T23:59:59Z");
    }

    /// Records of one invocation share a run id, so two rigs recording into one tree do not mix.
    #[test]
    fn a_run_id_is_stable_within_a_run_and_differs_between_runs() {
        let a = run_id();
        assert_eq!(a, a.clone());
        assert!(
            (0..16).any(|_| run_id() != a),
            "a fresh run id must not repeat {a}"
        );
    }

    /// Resolution walks up from a buried binary to the checkout root, no further in.
    #[test]
    fn default_path_resolves_against_a_synthetic_layout() {
        let dir = std::env::temp_dir().join(format!("daplink-record-test-{}", run_id()));
        let checkout = dir.join("DAPLink");
        let deep = checkout.join("rust/target/debug/deps");
        std::fs::create_dir_all(&deep).unwrap();
        std::fs::create_dir_all(checkout.join("rust/tests/daplink-test")).unwrap();
        std::fs::write(
            checkout.join("rust/tests/daplink-test/Cargo.toml"),
            "[package]\n",
        )
        .unwrap();

        let resolved = default_path(&deep);
        assert_eq!(resolved, checkout.join("artifacts").join("hil-runs.jsonl"));

        std::fs::remove_dir_all(&dir).ok();
    }

    /// Outside a recognizable checkout, resolution falls back rather than panicking.
    #[test]
    fn default_path_falls_back_when_no_checkout_is_found() {
        let start = std::env::temp_dir().join("nowhere-near-a-checkout");
        let resolved = default_path(&start);
        assert_eq!(resolved, start.join("artifacts").join("hil-runs.jsonl"));
    }

    /// Appending twice grows the file: a later run's records are history, not a replacement.
    #[test]
    fn append_grows_the_file_rather_than_truncating_it() {
        let dir = std::env::temp_dir().join(format!("daplink-record-append-test-{}", run_id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("hil-runs.jsonl");

        let case = CaseRecord {
            case: "connect",
            outcome: "ok",
            detail: None,
            duration_ms: 1,
            speed: None,
        };
        append(&path, &meta_for_test(), &case).unwrap();
        append(&path, &meta_for_test(), &case).unwrap();

        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents.lines().count(), 2, "{contents}");

        std::fs::remove_dir_all(&dir).ok();
    }

    /// Appending creates the directory: a fresh checkout has no `artifacts/` until the first run.
    #[test]
    fn append_creates_a_missing_directory() {
        let dir = std::env::temp_dir().join(format!("daplink-record-mkdir-test-{}", run_id()));
        std::fs::remove_dir_all(&dir).ok();
        let path = dir.join("nested").join("hil-runs.jsonl");

        let case = CaseRecord {
            case: "connect",
            outcome: "ok",
            detail: None,
            duration_ms: 1,
            speed: None,
        };
        append(&path, &meta_for_test(), &case).unwrap();
        assert!(path.is_file());

        std::fs::remove_dir_all(&dir).ok();
    }

    /// Minimal run identity for the cases that only exercise one field.
    fn meta_for_test() -> RunMeta {
        RunMeta {
            run_id: "1f4-17a2b".to_string(),
            vid: 0x0d28,
            pid: 0x0204,
            serial: Some("0000ABCDEF012345".to_string()),
            firmware_version: Some("2.1.1.gfeedc0ffee12".to_string()),
            harness_version: "2.1.1.gfeedc0ffee12",
            target: "rp2040",
            chip: "RP2040".to_string(),
            dut_elf: Some("target/rp2040.elf".to_string()),
            dut_uf2: None,
        }
    }
}
