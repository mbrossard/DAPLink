//! JSONL recording for a HIL run: a start line, then one appended record per case, in a fixed ledger.

use std::ffi::OsString;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use probe_rs::probe::WireProtocol;

use crate::speed;

/// Record format version, bumped when a field's meaning changes.
pub(crate) const SCHEMA_VERSION: u32 = 5;

/// The record file name, in the ledger and under a fallback `artifacts/`.
const DEFAULT_FILE_NAME: &str = "hil-runs.jsonl";

/// The marker identifying the checkout root while walking up from a starting path.
const CHECKOUT_MARKER: &str = "rust/tests/daplink-test/Cargo.toml";

/// What every case record of one invocation shares: probe, images, target, harness version.
pub(crate) struct RunMeta {
    /// Groups the records of one invocation exactly, rather than by timestamp proximity.
    pub(crate) run_id: String,
    /// `full` for the default set, `list` for named cases.
    pub(crate) mode: &'static str,
    /// The runner's token for one event across its attempts.
    pub(crate) series: Option<String>,
    pub(crate) vid: u16,
    pub(crate) pid: u16,
    pub(crate) serial: Option<String>,
    /// The firmware registry row the record measured, when anything names it.
    pub(crate) firmware_row: Option<String>,
    /// What named it: `image`, `operator` or `none`.
    pub(crate) firmware_row_source: &'static str,
    /// The probe's own reported firmware-version string, when the read succeeded.
    pub(crate) firmware_version: Option<String>,
    /// This harness binary's own linked version string (`daplink_core::DAP_FW_VERSION`).
    pub(crate) harness_version: &'static str,
    pub(crate) target: &'static str,
    /// probe-rs chip name the run attaches with.
    pub(crate) chip: String,
    /// DUT ELF path, when the run was given one.
    pub(crate) dut_elf: Option<String>,
    /// DUT uf2 path, when the run was given one.
    pub(crate) dut_uf2: Option<String>,
    /// What the DUT reported about itself over the test protocol, `None` when it did not answer.
    pub(crate) dut: Option<DutIdRecord>,
}

/// The DUT image's own identity, as its `ID` reply gave it.
pub(crate) struct DutIdRecord {
    pub(crate) name: Option<String>,
    pub(crate) proto: Option<u32>,
    /// Build revision of the DUT image; `None` for an image predating the field.
    pub(crate) revision: Option<String>,
}

impl RunMeta {
    /// Build the shared run header from what `main` already read; nothing is re-queried here.
    pub(crate) fn new(
        run_id: String,
        (mode, series): (&'static str, Option<String>),
        cx: &crate::Ctx,
        firmware_row: &firmware_row::Resolved,
        firmware_version: Option<&str>,
        target: &'static str,
        dut: Option<&crate::dutid::DutId>,
    ) -> Self {
        RunMeta {
            run_id,
            mode,
            series,
            vid: cx.selector.vendor_id,
            pid: cx.selector.product_id,
            serial: cx.selector.serial_number.clone(),
            firmware_row: firmware_row.row.clone(),
            firmware_row_source: firmware_row.source.key(),
            firmware_version: firmware_version.map(str::to_string),
            harness_version: daplink_core::DAP_FW_VERSION,
            target,
            chip: cx.chip.clone(),
            dut_elf: cx.dut_elf.clone(),
            dut_uf2: cx.dut_uf2.clone(),
            dut: dut.map(|d| DutIdRecord {
                name: d.name.clone(),
                proto: d.proto,
                revision: d.revision.clone(),
            }),
        }
    }
}

/// One case's outcome and cost.
pub(crate) struct CaseRecord<'a> {
    pub(crate) case: &'a str,
    /// `ok`, `failed`, `fatal` or `skip` -- the harness's `Failure` taxonomy.
    pub(crate) outcome: &'static str,
    /// Why it failed, was fatal, or was skipped; on an `ok`, a note on how it was reached.
    pub(crate) detail: Option<&'a str>,
    /// Wall clock the case took, including a skip's near-zero.
    pub(crate) duration_ms: u64,
    /// The first `/proc/loadavg` field at case start, where the host has one.
    pub(crate) load1: Option<f64>,
    /// The deadline-bound waits the case noted.
    pub(crate) waits: Vec<WaitRecord>,
    /// The speed sweep's rungs, on the record of a case that swept (`speed-swd`, `speed-jtag`).
    pub(crate) speed: Option<SpeedPayload>,
}

/// One deadline-bound wait, the worst of `n`.
pub(crate) struct WaitRecord {
    pub(crate) name: &'static str,
    pub(crate) budget_ms: u64,
    pub(crate) max_ms: u64,
    pub(crate) n: u32,
}

/// A speed case's own measurements.
pub(crate) struct SpeedPayload {
    pub(crate) transport: &'static str,
    /// `basic` or `extended`.
    pub(crate) mode: &'static str,
    pub(crate) repeats: u32,
    pub(crate) rows: Vec<RowRecord>,
}

impl SpeedPayload {
    /// Build a payload from the raw sweep this run stashed on `Ctx::last_speed`.
    pub(crate) fn from_sweep(protocol: WireProtocol, sweep: &speed::Sweep) -> Self {
        SpeedPayload {
            transport: transport_name(protocol),
            mode: if sweep.extended { "extended" } else { "basic" },
            repeats: sweep.repeats,
            rows: sweep
                .rows
                .iter()
                .map(|r| RowRecord {
                    requested_hz: r.requested_hz,
                    direction: r.direction.label(),
                    access_bytes: r.access_bytes,
                    samples: r.samples.iter().map(|s| (s.calls, s.ns)).collect(),
                    stats: r.stats(),
                    error: r.error.clone(),
                    chunk_ns: (!sweep.extended).then(|| r.chunk_ns.clone()),
                })
                .collect(),
        }
    }
}

/// One row's measurement, as recorded.
pub(crate) struct RowRecord {
    /// Requested wire clock, Hz.
    pub(crate) requested_hz: u32,
    /// `read` or `write`.
    pub(crate) direction: &'static str,
    /// Bytes per call.
    pub(crate) access_bytes: usize,
    /// Calls and summed elapsed nanoseconds of each repeat.
    pub(crate) samples: Vec<(u32, u128)>,
    /// Median, min and max KiB/s over the repeats.
    pub(crate) stats: Option<(f64, f64, f64)>,
    /// Why the row failed; its samples, if any, still stand.
    pub(crate) error: Option<String>,
    /// Elapsed nanoseconds of each timed call; basic mode only, as the extended one is too long.
    pub(crate) chunk_ns: Option<Vec<u128>>,
}

/// The record's spelling of a wire transport.
pub(crate) fn transport_name(protocol: WireProtocol) -> &'static str {
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

/// The fixed ledger, wherever the binary was built: under `$XDG_STATE_HOME`, else `~/.local/state`.
pub(crate) fn ledger_path(state: Option<OsString>, home: Option<OsString>) -> Option<PathBuf> {
    let state = state.filter(|s| !s.is_empty()).map(PathBuf::from);
    let base = state.or_else(|| home.map(|h| PathBuf::from(h).join(".local/state")))?;
    Some(base.join("daplink").join(DEFAULT_FILE_NAME))
}

/// Where a recording run appends: the ledger, and `--record` as well.
pub(crate) fn record_paths(record: Option<PathBuf>) -> Vec<PathBuf> {
    let ledger = ledger_path(std::env::var_os("XDG_STATE_HOME"), std::env::var_os("HOME"))
        .unwrap_or_else(default_path_for_process);
    std::iter::once(ledger).chain(record).collect()
}

/// The first `/proc/loadavg` field, `None` where the host has no such file.
pub(crate) fn load1() -> Option<f64> {
    let text = std::fs::read_to_string("/proc/loadavg").ok()?;
    text.split_whitespace().next()?.parse().ok()
}

/// The ledger's fallback path: `artifacts/` under the checkout root above `start`, or under `start`
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
    append_line(path, &render(meta, case, now_s()))
}

/// Append the run's start line, which names what it selected: a name with no row and no `fatal` died.
pub(crate) fn append_start(path: &Path, meta: &RunMeta, selected: &[&str]) -> std::io::Result<()> {
    append_line(path, &render_start(meta, selected, load1(), now_s()))
}

fn append_line(path: &Path, line: &str) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    file.write_all(line.as_bytes())
}

fn now_s() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// Render the start line as a single JSON line, terminated by a newline.
pub(crate) fn render_start(
    meta: &RunMeta,
    selected: &[&str],
    load1: Option<f64>,
    unix_s: u64,
) -> String {
    let names: Vec<String> = selected.iter().map(|name| quote(name)).collect();
    format!(
        "{}\"event\":\"start\",\"selected\":[{}],\"load1\":{}}}\n",
        render_head(meta, unix_s),
        names.join(","),
        opt_f64(load1)
    )
}

/// Render one case record as a single JSON line, terminated by a newline.
pub(crate) fn render(meta: &RunMeta, case: &CaseRecord, unix_s: u64) -> String {
    let rows = case.speed.as_ref().map(|s| s.rows.len()).unwrap_or(0);
    let mut s = String::with_capacity(384 + rows * 96);
    s.push_str(&render_head(meta, unix_s));
    s.push_str(&format!(
        "\"case\":{},\"outcome\":{},\"detail\":{},\"duration_ms\":{},\"load1\":{}",
        quote(case.case),
        quote(case.outcome),
        opt_str(case.detail),
        case.duration_ms,
        opt_f64(case.load1)
    ));
    if !case.waits.is_empty() {
        let waits: Vec<String> = case
            .waits
            .iter()
            .map(|w| {
                format!(
                    "{{\"name\":{},\"budget_ms\":{},\"max_ms\":{},\"n\":{}}}",
                    quote(w.name),
                    w.budget_ms,
                    w.max_ms,
                    w.n
                )
            })
            .collect();
        s.push_str(&format!(",\"waits\":[{}]", waits.join(",")));
    }
    if let Some(p) = &case.speed {
        s.push_str(&format!(
            ",\"speed\":{{\"transport\":{},\"mode\":{},\"repeats\":{},\"rows\":[",
            quote(p.transport),
            quote(p.mode),
            p.repeats
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

/// What every line of a run opens with.
fn render_head(meta: &RunMeta, unix_s: u64) -> String {
    let mut s = String::with_capacity(384);
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
        "\"firmware_row\":{},\"firmware_row_source\":{},",
        opt_str(meta.firmware_row.as_deref()),
        quote(meta.firmware_row_source)
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
    match &meta.dut {
        Some(d) => s.push_str(&format!(
            "\"dut\":{{\"name\":{},\"proto\":{},\"revision\":{}}},",
            opt_str(d.name.as_deref()),
            d.proto
                .map_or_else(|| "null".to_string(), |p| p.to_string()),
            opt_str(d.revision.as_deref())
        )),
        None => s.push_str("\"dut\":null,"),
    }
    s.push_str(&format!(
        "\"mode\":{},\"series\":{},",
        quote(meta.mode),
        opt_str(meta.series.as_deref())
    ));
    s
}

fn opt_f64(v: Option<f64>) -> String {
    v.map_or_else(|| "null".to_string(), |v| v.to_string())
}

fn render_row(r: &RowRecord) -> String {
    let samples: Vec<String> = r
        .samples
        .iter()
        .map(|(calls, ns)| format!("{{\"calls\":{calls},\"ns\":{ns}}}"))
        .collect();
    let stat = |pick: fn((f64, f64, f64)) -> f64| {
        r.stats
            .map_or_else(|| "null".to_string(), |s| format!("{:.3}", pick(s)))
    };
    let mut s = format!(
        "{{\"requested_hz\":{},\"direction\":{},\"access_bytes\":{},\"samples\":[{}],\
         \"median_kib_s\":{},\"min_kib_s\":{},\"max_kib_s\":{},\"error\":{}",
        r.requested_hz,
        quote(r.direction),
        r.access_bytes,
        samples.join(","),
        stat(|s| s.0),
        stat(|s| s.1),
        stat(|s| s.2),
        opt_str(r.error.as_deref())
    );
    if let Some(ns) = &r.chunk_ns {
        s.push_str(&format!(",\"chunk_ns\":{}", ns_array(ns)));
    }
    s.push('}');
    s
}

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
    const PINNED_SWEEP: &str = r#"{"schema_version":5,"run_id":"1f4-17a2b","unix_s":1754378400,"utc":"2025-08-05T07:20:00Z","harness_version":"2.1.1.gfeedc0ffee12","probe":{"vid":"0d28","pid":"0204","serial":"0000ABCDEF012345"},"firmware_row":"firmware/mculink","firmware_row_source":"image","firmware_version":"2.1.1.gfeedc0ffee12","target":"rp2040","chip":"RP2040","dut_elf":"target/rp2040.elf","dut_uf2":null,"dut":{"name":"rp2040","proto":8,"revision":"gfeedc0ffee12"},"mode":"list","series":null,"case":"speed-swd","outcome":"ok","detail":null,"duration_ms":8421,"load1":null,"speed":{"transport":"swd","mode":"basic","repeats":1,"rows":[{"requested_hz":25000000,"direction":"read","access_bytes":4096,"samples":[{"calls":3,"ns":214500}],"median_kib_s":55944.056,"min_kib_s":55944.056,"max_kib_s":55944.056,"error":null,"chunk_ns":[71500,71600,71400]}]}}"#;

    /// A swept case as schema 4 wrote it, before `mode`, `series` and `load1`.
    const PINNED_V4_SWEEP: &str = r#"{"schema_version":4,"run_id":"1f4-17a2b","unix_s":1754378400,"utc":"2025-08-05T07:20:00Z","harness_version":"2.1.1.gfeedc0ffee12","probe":{"vid":"0d28","pid":"0204","serial":"0000ABCDEF012345"},"firmware_row":"firmware/mculink","firmware_row_source":"image","firmware_version":"2.1.1.gfeedc0ffee12","target":"rp2040","chip":"RP2040","dut_elf":"target/rp2040.elf","dut_uf2":null,"dut":{"name":"rp2040","proto":8,"revision":"gfeedc0ffee12"},"case":"speed-swd","outcome":"ok","detail":null,"duration_ms":8421,"speed":{"transport":"swd","mode":"basic","repeats":1,"rows":[{"requested_hz":25000000,"direction":"read","access_bytes":4096,"samples":[{"calls":3,"ns":214500}],"median_kib_s":55944.056,"min_kib_s":55944.056,"max_kib_s":55944.056,"error":null,"chunk_ns":[71500,71600,71400]}]}}"#;

    /// A swept case as schema 2 wrote it, before rows carried direction and access size.
    const PINNED_V2_SWEEP: &str = r#"{"schema_version":2,"run_id":"1f4-17a2b","unix_s":1754378400,"utc":"2025-08-05T07:20:00Z","harness_version":"2.1.1.gfeedc0ffee12","probe":{"vid":"0d28","pid":"0204","serial":"0000ABCDEF012345"},"firmware_version":"2.1.1.gfeedc0ffee12","target":"rp2040","chip":"RP2040","dut_elf":"target/rp2040.elf","dut_uf2":null,"dut":{"name":"rp2040","proto":8,"revision":"gfeedc0ffee12"},"case":"speed-swd","outcome":"ok","detail":null,"duration_ms":8421,"speed":{"transport":"swd","chunk_bytes":4096,"rows":[{"requested_hz":25000000,"chunk_ns":[71500,71600,71400]}]}}"#;

    /// The exact line for a case that swept nothing: the `speed` key is absent, not null.
    const PINNED_PLAIN: &str = r#"{"schema_version":5,"run_id":"1f4-17a2b","unix_s":1754378400,"utc":"2025-08-05T07:20:00Z","harness_version":"2.1.1.gfeedc0ffee12","probe":{"vid":"0d28","pid":"0204","serial":"0000ABCDEF012345"},"firmware_row":"firmware/mculink","firmware_row_source":"image","firmware_version":"2.1.1.gfeedc0ffee12","target":"rp2040","chip":"RP2040","dut_elf":"target/rp2040.elf","dut_uf2":null,"dut":{"name":"rp2040","proto":8,"revision":"gfeedc0ffee12"},"mode":"list","series":null,"case":"msc","outcome":"failed","detail":"drop 2 of 3 never re-enumerated","duration_ms":61230,"load1":null}"#;

    /// The rendered line is pinned in full, raw per-read nanoseconds included.
    #[test]
    fn a_swept_case_renders_one_line_with_raw_times() {
        let case = CaseRecord {
            case: "speed-swd",
            outcome: "ok",
            detail: None,
            duration_ms: 8421,
            load1: None,
            waits: Vec::new(),
            speed: Some(SpeedPayload {
                transport: "swd",
                mode: "basic",
                repeats: 1,
                rows: vec![RowRecord {
                    requested_hz: 25_000_000,
                    direction: "read",
                    access_bytes: 4096,
                    samples: vec![(3, 214_500)],
                    stats: Some((55_944.056, 55_944.056, 55_944.056)),
                    error: None,
                    chunk_ns: Some(vec![71_500, 71_600, 71_400]),
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
            load1: None,
            waits: Vec::new(),
            speed: None,
        };
        let line = render(&meta_for_test(), &case, 1_754_378_400);
        assert_eq!(line, format!("{PINNED_PLAIN}\n"));
        assert!(!line.contains("\"speed\""), "{line}");
    }

    /// A sweep runs over its own transport, whatever the rest of the run attached with.
    #[test]
    fn a_sweeps_own_transport_is_recorded() {
        let payload = SpeedPayload::from_sweep(WireProtocol::Jtag, &basic_sweep());
        assert_eq!(payload.transport, "jtag");
        assert_eq!(payload.mode, "basic");
        assert_eq!(payload.rows[0].chunk_ns, Some(vec![1, 2, 3, 4]));
        assert_eq!(payload.rows[0].samples, vec![(4, 10)]);
    }

    /// An extended row carries its direction, size, repeats and failure, but no per-call times.
    #[test]
    fn an_extended_sweep_round_trips_through_a_reader() {
        let sweep = speed::Sweep {
            extended: true,
            repeats: 2,
            rows: vec![
                speed::Row {
                    requested_hz: 50_000_000,
                    direction: speed::Direction::Write,
                    access_bytes: 64,
                    samples: vec![
                        speed::Sample {
                            calls: 400,
                            ns: 100_000_000,
                        },
                        speed::Sample {
                            calls: 500,
                            ns: 100_000_000,
                        },
                    ],
                    chunk_ns: vec![9, 9],
                    error: Some("mismatch at 0x20041000".into()),
                },
                speed::Row {
                    requested_hz: 50_000_000,
                    direction: speed::Direction::Read,
                    access_bytes: 64,
                    samples: Vec::new(),
                    chunk_ns: Vec::new(),
                    error: Some("attach at 50000000 Hz: no ack".into()),
                },
            ],
        };
        let case = CaseRecord {
            speed: Some(SpeedPayload::from_sweep(WireProtocol::Swd, &sweep)),
            ..plain_case()
        };
        let read = read_speed(&render(&meta_for_test(), &case, 0));
        assert_eq!(read.schema, 5);
        assert_eq!((read.mode.as_str(), read.repeats), ("extended", 2));
        assert_eq!(
            read.rows,
            vec![
                ReadRow {
                    requested_hz: 50_000_000,
                    direction: "write".into(),
                    access_bytes: 64,
                    samples: vec![(400, 100_000_000), (500, 100_000_000)],
                    median_kib_s: Some(281.25),
                    error: Some("mismatch at 0x20041000".into()),
                    chunk_ns: None,
                },
                ReadRow {
                    requested_hz: 50_000_000,
                    direction: "read".into(),
                    access_bytes: 64,
                    samples: Vec::new(),
                    median_kib_s: None,
                    error: Some("attach at 50000000 Hz: no ack".into()),
                    chunk_ns: None,
                },
            ]
        );
    }

    /// A basic sweep reads back with its per-call times.
    #[test]
    fn a_basic_sweep_round_trips_through_a_reader() {
        let case = CaseRecord {
            speed: Some(SpeedPayload::from_sweep(WireProtocol::Swd, &basic_sweep())),
            ..plain_case()
        };
        let read = read_speed(&render(&meta_for_test(), &case, 0));
        assert_eq!((read.mode.as_str(), read.repeats), ("basic", 1));
        assert_eq!(read.rows[0].samples, vec![(4, 10)]);
        assert_eq!(read.rows[0].chunk_ns, Some(vec![1, 2, 3, 4]));
        assert_eq!(read.rows[0].direction, "read");
    }

    /// A schema-4 record, which carries no mode, series or load, still reads.
    #[test]
    fn a_schema_4_sweep_still_reads() {
        assert_eq!(read_speed(PINNED_V4_SWEEP).schema, 4);
    }

    /// A schema-3 record, which names no firmware row, still reads.
    #[test]
    fn a_schema_3_sweep_still_reads() {
        let v3 = PINNED_V4_SWEEP
            .replace("\"schema_version\":4", "\"schema_version\":3")
            .replace(
                "\"firmware_row\":\"firmware/mculink\",\"firmware_row_source\":\"image\",",
                "",
            );
        assert_eq!(read_speed(&v3).schema, 3);
    }

    /// A schema-2 record still reads: its rows are basic reads at the payload's `chunk_bytes`.
    #[test]
    fn a_schema_2_sweep_still_reads() {
        let read = read_speed(PINNED_V2_SWEEP);
        assert_eq!(read.schema, 2);
        assert_eq!((read.mode.as_str(), read.repeats), ("basic", 1));
        assert_eq!(
            read.rows,
            vec![ReadRow {
                requested_hz: 25_000_000,
                direction: "read".into(),
                access_bytes: 4096,
                samples: vec![(3, 214_500)],
                median_kib_s: None,
                error: None,
                chunk_ns: Some(vec![71_500, 71_600, 71_400]),
            }]
        );
    }

    /// One basic row of four calls.
    fn basic_sweep() -> speed::Sweep {
        speed::Sweep {
            extended: false,
            repeats: 1,
            rows: vec![speed::Row {
                requested_hz: 1_000_000,
                direction: speed::Direction::Read,
                access_bytes: 4096,
                samples: vec![speed::Sample { calls: 4, ns: 10 }],
                chunk_ns: vec![1, 2, 3, 4],
                error: None,
            }],
        }
    }

    /// A reader's view of a speed payload, whichever schema wrote it.
    struct ReadSpeed {
        schema: u64,
        mode: String,
        repeats: u64,
        rows: Vec<ReadRow>,
    }

    #[derive(Debug, PartialEq)]
    struct ReadRow {
        requested_hz: u64,
        direction: String,
        access_bytes: u64,
        samples: Vec<(u64, u64)>,
        median_kib_s: Option<f64>,
        error: Option<String>,
        chunk_ns: Option<Vec<u64>>,
    }

    /// Parse a record line's speed payload, normalizing a schema-2 one.
    fn read_speed(line: &str) -> ReadSpeed {
        let v: serde_json::Value = serde_json::from_str(line).expect("a record is JSON");
        let schema = v["schema_version"].as_u64().unwrap();
        let p = &v["speed"];
        let ns = |a: &serde_json::Value| -> Option<Vec<u64>> {
            a.as_array()
                .map(|a| a.iter().map(|n| n.as_u64().unwrap()).collect())
        };
        let rows = p["rows"].as_array().unwrap().iter();
        if schema < 3 {
            let access = p["chunk_bytes"].as_u64().unwrap();
            return ReadSpeed {
                schema,
                mode: "basic".into(),
                repeats: 1,
                rows: rows
                    .map(|r| {
                        let chunk_ns = ns(&r["chunk_ns"]).unwrap();
                        ReadRow {
                            requested_hz: r["requested_hz"].as_u64().unwrap(),
                            direction: "read".into(),
                            access_bytes: access,
                            samples: vec![(chunk_ns.len() as u64, chunk_ns.iter().sum())],
                            median_kib_s: None,
                            error: None,
                            chunk_ns: Some(chunk_ns),
                        }
                    })
                    .collect(),
            };
        }
        ReadSpeed {
            schema,
            mode: p["mode"].as_str().unwrap().into(),
            repeats: p["repeats"].as_u64().unwrap(),
            rows: rows
                .map(|r| ReadRow {
                    requested_hz: r["requested_hz"].as_u64().unwrap(),
                    direction: r["direction"].as_str().unwrap().into(),
                    access_bytes: r["access_bytes"].as_u64().unwrap(),
                    samples: r["samples"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(|s| (s["calls"].as_u64().unwrap(), s["ns"].as_u64().unwrap()))
                        .collect(),
                    median_kib_s: r["median_kib_s"].as_f64(),
                    error: r["error"].as_str().map(str::to_string),
                    chunk_ns: ns(&r["chunk_ns"]),
                })
                .collect(),
        }
    }

    /// A killed run shows as a selected name with no row and no fatal, so a start line is not a case.
    #[test]
    fn the_start_line_names_the_selection_and_is_not_a_case() {
        let line = render_start(&meta_for_test(), &["identity", "info"], Some(1.5), 0);
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["schema_version"], SCHEMA_VERSION);
        assert_eq!(v["event"], "start");
        assert_eq!(v["selected"][1], "info");
        assert_eq!(v["load1"], 1.5);
        assert_eq!(v["mode"], "list");
        assert!(v.get("case").is_none(), "{line}");
    }

    #[test]
    fn waits_render_only_when_present() {
        let mut case = plain_case();
        assert!(!render(&meta_for_test(), &case, 0).contains("waits"));
        case.waits = vec![WaitRecord {
            name: "rtt.expect",
            budget_ms: 3000,
            max_ms: 412,
            n: 2,
        }];
        assert!(render(&meta_for_test(), &case, 0)
            .contains(r#""waits":[{"name":"rtt.expect","budget_ms":3000,"max_ms":412,"n":2}]"#));
    }

    #[test]
    fn a_case_carries_its_load_and_a_run_its_series() {
        let mut meta = meta_for_test();
        meta.mode = "full";
        meta.series = Some("attempt-a".into());
        let case = CaseRecord {
            load1: Some(0.25),
            ..plain_case()
        };
        let v: serde_json::Value = serde_json::from_str(&render(&meta, &case, 0)).unwrap();
        assert_eq!(v["mode"], "full");
        assert_eq!(v["series"], "attempt-a");
        assert_eq!(v["load1"], 0.25);
    }

    #[test]
    fn the_ledger_path_ignores_the_binary_and_prefers_xdg_state() {
        let path = |state: Option<&str>, home: Option<&str>| {
            ledger_path(state.map(Into::into), home.map(Into::into))
        };
        assert_eq!(
            path(Some("/s"), Some("/h")),
            Some(PathBuf::from("/s/daplink/hil-runs.jsonl"))
        );
        assert_eq!(
            path(None, Some("/h")),
            Some(PathBuf::from("/h/.local/state/daplink/hil-runs.jsonl"))
        );
        assert_eq!(
            path(Some(""), Some("/h")),
            Some(PathBuf::from("/h/.local/state/daplink/hil-runs.jsonl"))
        );
        assert_eq!(path(None, None), None);
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
                load1: None,
                waits: Vec::new(),
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
        meta.firmware_row = None;
        meta.firmware_row_source = "none";
        meta.firmware_version = None;
        meta.dut_elf = None;
        let case = CaseRecord {
            case: "connect",
            outcome: "ok",
            detail: None,
            duration_ms: 1,
            load1: None,
            waits: Vec::new(),
            speed: None,
        };
        let line = render(&meta, &case, 0);
        assert!(line.contains("\"serial\":null"), "{line}");
        assert!(
            line.contains("\"firmware_row\":null,\"firmware_row_source\":\"none\""),
            "{line}"
        );
        assert!(line.contains("\"firmware_version\":null"), "{line}");
        assert!(line.contains("\"dut_elf\":null"), "{line}");
        assert!(line.contains("\"detail\":null"), "{line}");
    }

    /// A DUT that never answered records as absent, not as a blank identity.
    #[test]
    fn an_unread_dut_identity_renders_as_null() {
        let mut meta = meta_for_test();
        meta.dut = None;
        let line = render(&meta, &plain_case(), 0);
        assert!(line.contains(r#""dut":null,"mode""#), "{line}");
    }

    /// A DUT image predating the revision field records a null revision.
    #[test]
    fn a_dut_without_a_revision_records_the_rest() {
        let mut meta = meta_for_test();
        meta.dut = Some(DutIdRecord {
            name: Some("rp2040".to_string()),
            proto: Some(7),
            revision: None,
        });
        let line = render(&meta, &plain_case(), 0);
        assert!(
            line.contains(r#""dut":{"name":"rp2040","proto":7,"revision":null}"#),
            "{line}"
        );
    }

    /// One outcome-only case record, for the tests that only vary the run identity.
    fn plain_case() -> CaseRecord<'static> {
        CaseRecord {
            case: "connect",
            outcome: "ok",
            detail: None,
            duration_ms: 1,
            load1: None,
            waits: Vec::new(),
            speed: None,
        }
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
            load1: None,
            waits: Vec::new(),
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

    /// Records of one invocation share a run id.
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
            load1: None,
            waits: Vec::new(),
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
            load1: None,
            waits: Vec::new(),
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
            mode: "list",
            series: None,
            vid: 0x0d28,
            pid: 0x0204,
            serial: Some("0000ABCDEF012345".to_string()),
            firmware_row: Some("firmware/mculink".to_string()),
            firmware_row_source: "image",
            firmware_version: Some("2.1.1.gfeedc0ffee12".to_string()),
            harness_version: "2.1.1.gfeedc0ffee12",
            target: "rp2040",
            chip: "RP2040".to_string(),
            dut_elf: Some("target/rp2040.elf".to_string()),
            dut_uf2: None,
            dut: Some(DutIdRecord {
                name: Some("rp2040".to_string()),
                proto: Some(8),
                revision: Some("gfeedc0ffee12".to_string()),
            }),
        }
    }
}
