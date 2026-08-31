//! `dap-bench` command-line entry point.

use std::time::Duration;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand, ValueEnum};

use dap_bench::{adi, dap, report, sweep, target, transport};

/// Bytes moved per block-transfer iteration.
const CHUNK_BYTES: usize = 4096;
/// Default clock sweep; a bare `run` must stay comparable with recorded rows.
const DEFAULT_CLOCK_LIST: &str = "1M,4M,12M,25M";

/// Measure CMSIS-DAP block-transfer throughput against a target over SWD or JTAG.
#[derive(Parser, Debug)]
#[command(name = "dap-bench", version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Sweep clock/direction, reading or writing 4 KiB blocks and reporting throughput.
    Run(RunArgs),
    /// Ascend a clock ladder read-only, finding the throughput plateau and the failure edge.
    Sweep(SweepArgs),
}

/// Direction of a `run` measurement.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum DirectionArg {
    /// Read from the target's flash window only.
    Read,
    /// Write to the target's SRAM window only; the end-of-run reset re-initializes RAM.
    Write,
    /// Read rows, then write rows.
    Both,
}

/// Wire protocol of a `run` measurement.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum ProtocolArg {
    /// Serial Wire Debug.
    Swd,
    /// JTAG.
    Jtag,
}

/// Arguments for `dap-bench run`.
#[derive(clap::Args, Debug)]
struct RunArgs {
    /// Probe selector (`VID:PID[:serial]`); required when several probes are attached.
    #[arg(long = "probe")]
    probe: Option<String>,
    /// Comma-separated SWJ clock sweep (`1M,4M,12M,25M`); accepts `k`/`M` suffixes.
    #[arg(long = "clock", default_value = DEFAULT_CLOCK_LIST)]
    clock: String,
    /// Which direction(s) to measure.
    #[arg(long = "direction", value_enum, default_value_t = DirectionArg::Read)]
    direction: DirectionArg,
    /// Wire protocol.
    #[arg(long = "protocol", value_enum, default_value_t = ProtocolArg::Swd)]
    protocol: ProtocolArg,
    /// Measurement budget per rung, in milliseconds. Records keep the default so rows compare.
    #[arg(long = "window", default_value_t = report::DEFAULT_WINDOW_MS)]
    window: u64,
    /// Target address-profile name; an unknown name errors listing the known ones.
    #[arg(long = "target", default_value = target::DEFAULT_TARGET)]
    target: String,
    /// Comma-separated sweep of chunks kept outstanding on the bulk pair (`1,2,4,8`), each
    /// clamped to the probe's advertised `DAP_PACKET_COUNT`. `1` is lockstep.
    #[arg(long = "depth", default_value = "1")]
    depth: String,
    /// Repeats per rung, reported as a median with its full range. The default matches the
    /// validation procedure: a 32-sample set at 25 MHz has spanned more than 2.5x, so one sample
    /// is not a number, and a repeat costs roughly half a second at the default window.
    /// `--repeat 1` takes a single ad-hoc sample, which no recorded figure may be quoted from.
    #[arg(long = "repeat", default_value_t = report::DEFAULT_REPEATS)]
    repeat: u32,
    /// Decompose the clock sweep into a clock-independent intercept and a wire slope per
    /// `(direction, depth)` group; needs two or more clocks per group. Each group's fit uses
    /// the rung's median sample.
    #[arg(long = "fit", default_value_t = false)]
    fit: bool,
    /// Emit one JSON document to stdout instead of the table; other output moves to stderr.
    #[arg(long = "json")]
    json: bool,
}

/// Arguments for `dap-bench sweep`.
#[derive(clap::Args, Debug)]
struct SweepArgs {
    /// Probe selector (`VID:PID[:serial]`); required when several probes are attached.
    #[arg(long = "probe")]
    probe: Option<String>,
    /// Wire protocol.
    #[arg(long = "protocol", value_enum, default_value_t = ProtocolArg::Swd)]
    protocol: ProtocolArg,
    /// Target address-profile name; an unknown name errors listing the known ones.
    #[arg(long = "target", default_value = target::DEFAULT_TARGET)]
    target: String,
    /// Measurement budget per rung, in milliseconds.
    #[arg(long = "window", default_value_t = report::DEFAULT_WINDOW_MS)]
    window: u64,
    /// Ladder's first rung, in Hz.
    #[arg(long = "start", default_value_t = 500_000)]
    start: u32,
    /// Ladder's ceiling, in Hz; the ascent never probes above it.
    #[arg(long = "max", default_value_t = 32_000_000)]
    max: u32,
    /// Chunks kept outstanding on the bulk pair at once, clamped to the probe's advertised
    /// `DAP_PACKET_COUNT`. `1` is lockstep.
    #[arg(long = "depth", default_value_t = 1)]
    depth: usize,
    /// Emit one JSON document to stdout instead of the table; other output moves to stderr.
    #[arg(long = "json")]
    json: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Run(args) => cmd_run(&args),
        Command::Sweep(args) => cmd_sweep(&args),
    }
}

/// Open the probe named by `selector` (`VID:PID[:serial]`), or the only one attached if `None`.
fn open_probe(selector: Option<&str>) -> Result<transport::Probe> {
    let probes = transport::enumerate();
    let info = transport::select_cli(&probes, selector)?.clone();
    transport::Probe::open(info)
}

/// Parse one clock token (`4M`, `500k`, `4000000`) into Hz.
fn parse_clock_hz(token: &str) -> Result<u32> {
    let (digits, scale) = match token.chars().last() {
        Some('M') | Some('m') => (&token[..token.len() - 1], 1_000_000),
        Some('k') | Some('K') => (&token[..token.len() - 1], 1_000),
        _ => (token, 1),
    };
    digits
        .parse::<u32>()
        .map(|v| v * scale)
        .map_err(|e| anyhow::anyhow!("invalid clock '{token}': {e}"))
}

/// Parse a comma-separated `--clock` list into an ordered, deduplicated Hz sweep.
fn parse_clock_list(list: &str) -> Result<Vec<u32>> {
    let mut out = Vec::new();
    for token in list.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        let hz = parse_clock_hz(token)?;
        if !out.contains(&hz) {
            out.push(hz);
        }
    }
    if out.is_empty() {
        anyhow::bail!("no clocks selected (expected a list like '1M,2M,5M,10M')");
    }
    Ok(out)
}

/// Parse a comma-separated `--depth` list into an ordered, deduplicated ladder.
fn parse_depth_list(list: &str) -> Result<Vec<usize>> {
    let mut out = Vec::new();
    for token in list.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        let depth: usize = token
            .parse()
            .map_err(|e| anyhow::anyhow!("invalid depth '{token}': {e}"))?;
        if !out.contains(&depth) {
            out.push(depth);
        }
    }
    if out.is_empty() {
        anyhow::bail!("no depths selected (expected a list like '1,2,4,8')");
    }
    Ok(out)
}

/// [`resolve_depth`] over a ladder, dropping the duplicates a clamp introduces.
fn resolve_depth_list(requested: &[usize], advertised: Option<u8>) -> (Vec<usize>, Vec<String>) {
    let (mut depths, mut warnings) = (Vec::new(), Vec::new());
    for &r in requested {
        let (depth, warning) = resolve_depth(r, advertised);
        if !depths.contains(&depth) {
            depths.push(depth);
        }
        warnings.extend(warning);
    }
    (depths, warnings)
}

/// Clamp `--depth` to the advertised `DAP_PACKET_COUNT`, treating a probe that reports no count
/// as single-packet; a clamp that changed the request returns a warning line for the caller.
fn resolve_depth(requested: usize, advertised: Option<u8>) -> (usize, Option<String>) {
    let advertised = usize::from(advertised.unwrap_or(1)).max(1);
    let requested = requested.max(1);
    if requested > advertised {
        (
            advertised,
            Some(format!(
                "warning: --depth {requested} exceeds the probe's advertised DAP_PACKET_COUNT \
                 ({advertised}); using {advertised}"
            )),
        )
    } else {
        (requested, None)
    }
}

/// The `DAP_Info` identity fields the header shows; a rejected info id leaves its field empty.
struct Identity {
    vendor: Option<String>,
    product: Option<String>,
    serial: Option<String>,
    firmware_version: Option<String>,
    packet_size: Option<u16>,
    packet_count: Option<u8>,
}

/// Query the identity fields via a series of `DAP_Info` requests.
fn query_identity(probe: &mut transport::Probe) -> Identity {
    const INFO_TIMEOUT: Duration = Duration::from_millis(500);
    let mut query = |id: dap::InfoId| probe.transfer(&dap::encode_info(id), INFO_TIMEOUT).ok();
    // A zero-length string means "read it from the USB descriptor instead".
    let non_empty = |s: String| if s.is_empty() { None } else { Some(s) };

    Identity {
        vendor: query(dap::InfoId::VendorId)
            .and_then(|r| dap::decode_info_string(&r).ok())
            .and_then(non_empty),
        product: query(dap::InfoId::ProductId)
            .and_then(|r| dap::decode_info_string(&r).ok())
            .and_then(non_empty),
        serial: query(dap::InfoId::SerialNumber)
            .and_then(|r| dap::decode_info_string(&r).ok())
            .and_then(non_empty),
        firmware_version: query(dap::InfoId::FirmwareVersion)
            .and_then(|r| dap::decode_info_string(&r).ok())
            .and_then(non_empty),
        packet_size: query(dap::InfoId::PacketSize)
            .and_then(|r| dap::decode_info_packet_size(&r).ok()),
        packet_count: query(dap::InfoId::PacketCount)
            .and_then(|r| dap::decode_info_packet_count(&r).ok()),
    }
}

/// Print the identity header every run carries; in `--json` mode it goes to stderr, keeping
/// stdout the sole JSON document.
fn print_identity_header(
    probe: &transport::Probe,
    identity: &Identity,
    profile: &target::TargetProfile,
    protocol: ProtocolArg,
    json: bool,
) {
    let dash = || "-".to_string();
    let info = probe.info();
    let line = |s: String| {
        if json {
            eprintln!("{s}");
        } else {
            println!("{s}");
        }
    };
    line(format!(
        "probe    {} (VID:PID {:04x}:{:04x}, serial {})",
        identity
            .product
            .clone()
            .or_else(|| info.product.clone())
            .unwrap_or_else(dash),
        info.vendor_id,
        info.product_id,
        identity
            .serial
            .clone()
            .or_else(|| info.serial.clone())
            .unwrap_or_else(dash),
    ));
    if let Some(vendor) = identity.vendor.as_ref().filter(|v| !v.is_empty()) {
        line(format!("vendor   {vendor}"));
    }
    line(format!(
        "firmware {}",
        identity.firmware_version.clone().unwrap_or_else(dash)
    ));
    line(format!(
        "packet   {} bytes x {} outstanding",
        identity.packet_size.map_or_else(dash, |v| v.to_string()),
        identity.packet_count.map_or_else(dash, |v| v.to_string()),
    ));
    line(format!("target   {}", profile.name));
    line(format!(
        "protocol {}",
        match protocol {
            ProtocolArg::Swd => "swd",
            ProtocolArg::Jtag => "jtag",
        }
    ));
}

/// `dap-bench run ...`.
fn cmd_run(args: &RunArgs) -> Result<()> {
    let clocks = parse_clock_list(&args.clock)?;
    let depths_requested = parse_depth_list(&args.depth)?;
    let window = Duration::from_millis(args.window);
    let profile = target::lookup(&args.target)?;
    let directions: &[report::Direction] = match args.direction {
        DirectionArg::Read => &[report::Direction::Read],
        DirectionArg::Write => &[report::Direction::Write],
        DirectionArg::Both => &[report::Direction::Read, report::Direction::Write],
    };

    // Bounds-check the fixed windows this run will touch once, up front, rather than per rung.
    if !profile.flash_contains(profile.flash_base, CHUNK_BYTES) {
        anyhow::bail!(
            "target '{}' flash window is smaller than the {CHUNK_BYTES}-byte read chunk",
            profile.name
        );
    }
    if !profile.sram_contains(profile.sram_base, CHUNK_BYTES) {
        anyhow::bail!(
            "target '{}' SRAM window is smaller than the {CHUNK_BYTES}-byte write chunk",
            profile.name
        );
    }

    // The interface claim is exclusive, so the handle must be opened once and held.
    let mut probe = open_probe(args.probe.as_deref())?;
    let identity = query_identity(&mut probe);
    print_identity_header(&probe, &identity, profile, args.protocol, args.json);
    let repeats = args.repeat.max(1);
    let rungs = directions.len() * clocks.len();
    let line = |s: String| {
        if args.json {
            eprintln!("{s}");
        } else {
            println!("{s}");
        }
    };
    line(format!(
        "repeats  {repeats} per rung, {rungs} rungs per depth (median with full range; \
         --repeat 1 for a single ad-hoc sample)"
    ));
    let (depths, depth_warnings) = resolve_depth_list(&depths_requested, identity.packet_count);
    for warning in &depth_warnings {
        line(warning.clone());
    }
    if !args.json {
        println!();
        println!("{}", report::column_header());
    }

    let result = run_sweep(
        &mut probe,
        profile,
        directions,
        &clocks,
        window,
        &RunOptions {
            protocol: args.protocol,
            depths: depths.clone(),
            repeats,
            json: args.json,
        },
    );

    // Always return the DUT to normal operation, whether the sweep above finished or aborted.
    reset_target(&mut probe, clocks[0], args.protocol, profile, args.json);

    let rows = result?;
    let fits = if args.fit {
        let packet_size = identity.packet_size.unwrap_or(0);
        let cycles_per_word = match args.protocol {
            ProtocolArg::Swd => report::SWD_CYCLES_PER_WORD,
            ProtocolArg::Jtag => report::JTAG_CYCLES_PER_WORD,
        };
        // One representative run per rung: the sample the median names.
        let medians: Vec<report::Step> = rows
            .iter()
            .filter_map(|r| r.median_sample())
            .map(|s| report::Step {
                direction: s.direction,
                clock_hz: s.clock_hz,
                kib_per_s: s.kib_per_s,
                iterations: s.iterations,
                elapsed_ms: s.elapsed_ms,
                depth: s.depth,
            })
            .collect();
        report::compute_fits(&medians, CHUNK_BYTES, packet_size, cycles_per_word)
    } else {
        Vec::new()
    };

    if args.json {
        let ctx = report::RunContext {
            timestamp_utc: report::format_iso8601_utc(unix_time_now()),
            probe_vendor: identity.vendor.filter(|v| !v.is_empty()),
            probe_product: identity.product.or_else(|| probe.info().product.clone()),
            probe_serial: identity.serial.or_else(|| probe.info().serial.clone()),
            firmware_version: identity.firmware_version,
            packet_size: identity.packet_size,
            packet_count: identity.packet_count,
            protocol: match args.protocol {
                ProtocolArg::Swd => "swd",
                ProtocolArg::Jtag => "jtag",
            },
            target: profile.name,
            window_ms: args.window,
            chunk_bytes: CHUNK_BYTES,
            repeats,
            clocks_hz: &clocks,
            depths: &depths,
        };
        print!("{}", report::to_json(&ctx, &rows, &fits));
    } else if args.fit {
        print_fit_summary(&fits);
    }
    Ok(())
}

/// R^2 below which the summary notes call the linear wire model a poor fit.
const FIT_GOOD_R_SQUARED: f64 = 0.9;

/// Print the clock-sweep fit table and its explanatory notes (human mode only).
fn print_fit_summary(fits: &[report::FitRow]) {
    println!();
    println!(
        "FIT  T_block = a + b/f  (least squares over the clock sweep; a = clock-independent \
         per-block cost, b = wire term)"
    );
    if fits.is_empty() {
        println!("     no (direction, depth) groups with two or more usable clocks to fit");
        return;
    }
    println!("{}", report::fit_column_header());
    for f in fits {
        println!("{}", f.render());
    }
    for f in fits {
        if let Some(ceiling) = f.plateau_ceiling_hz {
            println!(
                "  note: {}/depth {} plateaus -- T_block stops dropping by ~{} Hz; a higher \
                 clock will not help (the wire is not the bottleneck, and b above is not \
                 meaningful)",
                f.direction.label(),
                f.depth,
                ceiling
            );
        } else if f.r_squared < FIT_GOOD_R_SQUARED {
            println!(
                "  note: {}/depth {} fits the linear wire model poorly (R^2 {:.3})",
                f.direction.label(),
                f.depth,
                f.r_squared
            );
        }
    }
}

/// Seconds since the Unix epoch; a clock before 1970 reports 0 rather than panicking.
fn unix_time_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

/// Attach at `clock_hz` with each path's default config; JTAG is single-drop and ignores the
/// profile's `targetsel`. A failure carries the profile, which is otherwise unrecoverable from
/// the wire symptom.
fn attach_at<'a>(
    probe: &'a mut transport::Probe,
    protocol: ProtocolArg,
    clock_hz: u32,
    profile: &target::TargetProfile,
) -> Result<adi::Adiv5<'a>> {
    match protocol {
        ProtocolArg::Swd => adi::Adiv5::attach(
            probe,
            adi::AttachConfig {
                clock_hz,
                targetsel: profile.targetsel,
                ap: profile.ap,
                ..adi::AttachConfig::default()
            },
        )
        .with_context(|| profile.attach_context()),
        ProtocolArg::Jtag => adi::Adiv5::attach_jtag(
            probe,
            adi::JtagAttachConfig {
                clock_hz,
                ..adi::JtagAttachConfig::default()
            },
        ),
    }
}

/// The remaining per-run knobs [`run_sweep`]'s rungs need, bundled for the arity lint.
struct RunOptions {
    protocol: ProtocolArg,
    depths: Vec<usize>,
    repeats: u32,
    json: bool,
}

/// One repeat of one rung: attach, halt, guard, then time. The phase that failed comes back
/// with the error so the caller can count the abort by class instead of ending the run.
fn one_repeat(
    probe: &mut transport::Probe,
    profile: &target::TargetProfile,
    direction: report::Direction,
    clock_hz: u32,
    depth: usize,
    window: Duration,
    protocol: ProtocolArg,
) -> Result<report::Step, (report::AbortClass, anyhow::Error)> {
    use report::AbortClass;

    // A bitbang probe may only latch a new clock on (re)connect.
    let mut adi = attach_at(probe, protocol, clock_hz, profile)
        .with_context(|| format!("attaching at {clock_hz} Hz failed"))
        .map_err(|e| (AbortClass::Attach, e))?;

    // Reads included: a running core contends on the flash/AHB matrix.
    adi.halt_core()
        .with_context(|| format!("halting the core at {clock_hz} Hz failed"))
        .map_err(|e| (AbortClass::Halt, e))?;

    let words = CHUNK_BYTES / 4;
    match direction {
        report::Direction::Read => {
            report::integrity_guard_read(&mut adi, profile.flash_base, words, depth)
                .map_err(|e| (AbortClass::Guard, e))?;
            let mut buf = [0u32; CHUNK_BYTES / 4];
            report::time_block_read(
                &mut adi,
                profile.flash_base,
                &mut buf,
                clock_hz,
                window,
                depth,
            )
            .map_err(|e| (AbortClass::Transfer, e))
        }
        report::Direction::Write => {
            report::integrity_guard_write(&mut adi, profile.sram_base, words, depth)
                .map_err(|e| (AbortClass::Guard, e))?;
            report::time_block_write(&mut adi, profile.sram_base, words, clock_hz, window, depth)
                .map_err(|e| (AbortClass::Transfer, e))
        }
    }
}

/// Measure one rung `repeats` times. An aborted repeat is counted and classified, never retried:
/// refilling the set would report a healthy rig that is not.
fn measure_rung(
    probe: &mut transport::Probe,
    profile: &target::TargetProfile,
    direction: report::Direction,
    clock_hz: u32,
    depth: usize,
    window: Duration,
    options: &RunOptions,
) -> (report::RepeatSet, Option<String>) {
    let mut set = report::RepeatSet {
        direction,
        clock_hz,
        depth: depth.max(1) as u32,
        repeats: options.repeats,
        samples: Vec::with_capacity(options.repeats as usize),
        aborts: Vec::new(),
    };
    let mut first_error = None;
    for _ in 0..options.repeats {
        match one_repeat(
            probe,
            profile,
            direction,
            clock_hz,
            depth,
            window,
            options.protocol,
        ) {
            Ok(step) => set.samples.push(step),
            Err((class, e)) => {
                set.aborts.push(class);
                first_error.get_or_insert_with(|| format!("{e:#}"));
            }
        }
    }
    (set, first_error)
}

/// Run every `(direction, depth, clock)` rung `--repeat` times -- fresh attach, halt, guard,
/// measure per repeat -- returning one aggregate row per rung in sweep order; `--json`
/// suppresses the per-row table line. A rung ends the run only when every repeat aborted.
fn run_sweep(
    probe: &mut transport::Probe,
    profile: &target::TargetProfile,
    directions: &[report::Direction],
    clocks: &[u32],
    window: Duration,
    options: &RunOptions,
) -> Result<Vec<report::RepeatSet>> {
    let mut rows = Vec::with_capacity(directions.len() * options.depths.len() * clocks.len());
    for &direction in directions {
        for &depth in &options.depths {
            for &clock_hz in clocks {
                let (set, first_error) =
                    measure_rung(probe, profile, direction, clock_hz, depth, window, options);
                if set.samples.is_empty() {
                    anyhow::bail!(
                        "{} at {clock_hz} Hz: all {} repeats aborted ({}); first: {}",
                        direction.label(),
                        set.repeats,
                        report::abort_detail(&set),
                        first_error.unwrap_or_else(|| "no error recorded".to_string())
                    );
                }
                if !set.aborts.is_empty() {
                    eprintln!(
                        "note: {} at {clock_hz} Hz: {} of {} repeats aborted ({}); first: {}",
                        direction.label(),
                        set.aborts.len(),
                        set.repeats,
                        report::abort_detail(&set),
                        first_error.unwrap_or_else(|| "no error recorded".to_string())
                    );
                }
                if !options.json {
                    println!("{}", report::render(&set));
                }
                rows.push(set);
            }
        }
    }
    Ok(rows)
}

/// Reboot the target via `SYSRESETREQ`, since the halted core never resumes on its own. The
/// reset drops the debug connection, so a transfer error here warns and never fails.
fn reset_target(
    probe: &mut transport::Probe,
    clock_hz: u32,
    protocol: ProtocolArg,
    profile: &target::TargetProfile,
    json: bool,
) {
    let outcome =
        attach_at(probe, protocol, clock_hz, profile).and_then(|mut adi| adi.system_reset());
    match outcome {
        Ok(()) if json => eprintln!("target reset (SYSRESETREQ)"),
        Ok(()) => println!("target reset (SYSRESETREQ)"),
        Err(e) => eprintln!("note: end-of-run target reset not acked (expected): {e:#}"),
    }
}

/// Validate `--start`/`--max` before any hardware is touched.
fn validate_sweep_range(start_hz: u32, max_hz: u32) -> Result<()> {
    if start_hz < 1 {
        anyhow::bail!("--start must be at least 1 Hz");
    }
    if start_hz > max_hz {
        anyhow::bail!("--start ({start_hz}) must not exceed --max ({max_hz})");
    }
    Ok(())
}

/// Column header for the sweep table.
fn sweep_column_header() -> String {
    format!(
        "{:>10}  {:>9}  {:>5}  {:>7}  {:>5}",
        "clock_hz", "KiB/s", "iters", "verdict", "depth"
    )
}

/// Render one measured [`sweep::SweepRow`] as an aligned table row.
fn sweep_render(row: &sweep::SweepRow) -> String {
    let verdict = match row.verdict {
        sweep::Verdict::Pass => "pass",
        sweep::Verdict::Fail => "fail",
    };
    format!(
        "{:>10}  {:>9.1}  {:>5}  {:>7}  {:>5}",
        row.clock_hz, row.kib_per_s, row.iterations, verdict, row.depth
    )
}

/// The state one sweep rung's cell needs, bundled so per-rung helpers take one argument instead
/// of threading probe/protocol/profile/window through each of them separately.
struct SweepSession<'a> {
    probe: &'a mut transport::Probe,
    protocol: ProtocolArg,
    profile: &'a target::TargetProfile,
    window: Duration,
    depth: usize,
}

/// One rung's read-only cell: attach, halt, timed read, then a read-twice verify after timing --
/// the corruption check that makes a sweep a failure search rather than a benchmark.
fn sweep_cell(session: &mut SweepSession, clock_hz: u32) -> Result<report::Step> {
    let mut adi = attach_at(session.probe, session.protocol, clock_hz, session.profile)
        .with_context(|| format!("attaching at {clock_hz} Hz failed"))?;
    adi.halt_core()
        .with_context(|| format!("halting the core at {clock_hz} Hz failed"))?;
    let mut buf = [0u32; CHUNK_BYTES / 4];
    let step = report::measure_block_read(
        &mut adi,
        session.profile.flash_base,
        &mut buf,
        clock_hz,
        session.window,
        session.depth,
    )
    .with_context(|| format!("read measurement at {clock_hz} Hz failed"))?;
    report::read_twice_and_compare(&mut adi, session.profile.flash_base, CHUNK_BYTES / 4)
        .with_context(|| format!("post-measurement verify at {clock_hz} Hz failed"))?;
    Ok(step)
}

/// [`sweep_cell`] with one re-attach retry, so a single marginal reading never kills the ascent;
/// a confirmed fail reports the retry's own error.
fn sweep_cell_confirmed(session: &mut SweepSession, clock_hz: u32) -> Result<report::Step> {
    match sweep_cell(session, clock_hz) {
        Ok(step) => Ok(step),
        Err(_) => sweep_cell(session, clock_hz),
    }
}

/// One confirmed cell as a [`sweep::SweepRow`]; a failing rung is a result, never an error.
fn record_sweep_cell(session: &mut SweepSession, clock_hz: u32, json: bool) -> sweep::SweepRow {
    let row = match sweep_cell_confirmed(session, clock_hz) {
        Ok(step) => sweep::SweepRow {
            clock_hz,
            kib_per_s: step.kib_per_s,
            iterations: step.iterations,
            verdict: sweep::Verdict::Pass,
            depth: step.depth,
        },
        Err(e) => {
            eprintln!("note: rung at {clock_hz} Hz failed (confirmed): {e:#}");
            sweep::SweepRow {
                clock_hz,
                kib_per_s: 0.0,
                iterations: 0,
                verdict: sweep::Verdict::Fail,
                depth: session.depth.max(1) as u32,
            }
        }
    };
    if !json {
        println!("{}", sweep_render(&row));
    }
    row
}

/// Bisect twice inside `(pass_hz, fail_hz)`, each midpoint running the ascent's confirmed cell.
fn refine_failure_interval(
    session: &mut SweepSession,
    pass_hz: u32,
    fail_hz: u32,
    rows: &mut Vec<sweep::SweepRow>,
    json: bool,
) -> (u32, u32) {
    let mut low = pass_hz;
    let mut high = fail_hz;
    for _ in 0..2 {
        let mid = sweep::geometric_midpoint(low, high);
        let row = record_sweep_cell(session, mid, json);
        let passed = row.verdict == sweep::Verdict::Pass;
        rows.push(row);
        (low, high) = sweep::narrow_interval(low, high, passed);
    }
    (low, high)
}

/// A `sweep` ascent's rungs, plateau onset, failure interval, and highest clock attempted.
struct SweepOutcome {
    rows: Vec<sweep::SweepRow>,
    plateau_hz: Option<u32>,
    failure_interval_hz: (u32, Option<u32>),
    last_clock_hz: u32,
}

/// Ascend the ladder to the first confirmed fail, then bisect toward the edge and find the
/// plateau.
fn run_sweep_ascent(
    session: &mut SweepSession,
    start_hz: u32,
    max_hz: u32,
    json: bool,
) -> SweepOutcome {
    let ladder = sweep::geometric_ladder(start_hz, max_hz);
    let mut rows: Vec<sweep::SweepRow> = Vec::with_capacity(ladder.len());
    let mut last_pass_hz: Option<u32> = None;
    let mut confirmed_fail_hz: Option<u32> = None;
    let mut last_clock_hz = start_hz;

    for &clock_hz in &ladder {
        last_clock_hz = clock_hz;
        let row = record_sweep_cell(session, clock_hz, json);
        let verdict = row.verdict;
        rows.push(row);
        match verdict {
            sweep::Verdict::Pass => last_pass_hz = Some(clock_hz),
            sweep::Verdict::Fail => {
                confirmed_fail_hz = Some(clock_hz);
                break;
            }
        }
    }

    let failure_interval_hz = match confirmed_fail_hz {
        None => (max_hz, None),
        Some(fail_hz) => match last_pass_hz {
            // The very first rung was already a confirmed fail: no interval to bisect.
            None => (0, Some(fail_hz)),
            Some(pass_hz) => {
                let (low, high) =
                    refine_failure_interval(session, pass_hz, fail_hz, &mut rows, json);
                (low, Some(high))
            }
        },
    };

    let passing: Vec<(u32, f64)> = rows
        .iter()
        .filter(|r| r.verdict == sweep::Verdict::Pass)
        .map(|r| (r.clock_hz, r.kib_per_s))
        .collect();
    let plateau_hz = sweep::detect_plateau(&passing);

    SweepOutcome {
        rows,
        plateau_hz,
        failure_interval_hz,
        last_clock_hz,
    }
}

/// `dap-bench sweep ...`.
fn cmd_sweep(args: &SweepArgs) -> Result<()> {
    validate_sweep_range(args.start, args.max)?;
    let window = Duration::from_millis(args.window);
    let profile = target::lookup(&args.target)?;

    // Bounds-check the fixed flash window this sweep will read, up front, rather than per rung.
    if !profile.flash_contains(profile.flash_base, CHUNK_BYTES) {
        anyhow::bail!(
            "target '{}' flash window is smaller than the {CHUNK_BYTES}-byte read chunk",
            profile.name
        );
    }

    let mut probe = open_probe(args.probe.as_deref())?;
    let identity = query_identity(&mut probe);
    print_identity_header(&probe, &identity, profile, args.protocol, args.json);
    let (depth, depth_warning) = resolve_depth(args.depth, identity.packet_count);
    if let Some(warning) = &depth_warning {
        if args.json {
            eprintln!("{warning}");
        } else {
            println!("{warning}");
        }
    }
    if !args.json {
        println!();
        println!("{}", sweep_column_header());
    }

    let outcome = {
        let mut session = SweepSession {
            probe: &mut probe,
            protocol: args.protocol,
            profile,
            depth,
            window,
        };
        run_sweep_ascent(&mut session, args.start, args.max, args.json)
    };

    // Always return the DUT to normal operation, whether the ascent finished, failed, or aborted.
    reset_target(
        &mut probe,
        outcome.last_clock_hz,
        args.protocol,
        profile,
        args.json,
    );

    if !args.json {
        println!();
        match outcome.plateau_hz {
            Some(hz) => println!("plateau  {hz}"),
            None => println!("plateau  none"),
        }
        match outcome.failure_interval_hz.1 {
            Some(high) => println!("failure  [{}, {}]", outcome.failure_interval_hz.0, high),
            None => println!("failure  [{}, none]", outcome.failure_interval_hz.0),
        }
    } else {
        let ctx = report::SweepContext {
            timestamp_utc: report::format_iso8601_utc(unix_time_now()),
            probe_vendor: identity.vendor.filter(|v| !v.is_empty()),
            probe_product: identity.product.or_else(|| probe.info().product.clone()),
            probe_serial: identity.serial.or_else(|| probe.info().serial.clone()),
            firmware_version: identity.firmware_version,
            packet_size: identity.packet_size,
            packet_count: identity.packet_count,
            protocol: match args.protocol {
                ProtocolArg::Swd => "swd",
                ProtocolArg::Jtag => "jtag",
            },
            target: profile.name,
            window_ms: args.window,
            chunk_bytes: CHUNK_BYTES,
        };
        print!(
            "{}",
            report::sweep_to_json(
                &ctx,
                &outcome.rows,
                outcome.plateau_hz,
                outcome.failure_interval_hz
            )
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_depth_passes_through_within_the_advertised_count() {
        assert_eq!(resolve_depth(4, Some(8)), (4, None));
        assert_eq!(resolve_depth(8, Some(8)), (8, None));
    }

    #[test]
    fn resolve_depth_clamps_and_warns_above_the_advertised_count() {
        let (depth, warning) = resolve_depth(16, Some(4));
        assert_eq!(depth, 4);
        let warning = warning.expect("clamping warns");
        assert!(warning.contains("16"));
        assert!(warning.contains("4"));
    }

    #[test]
    fn resolve_depth_treats_missing_advertised_count_as_one() {
        let (depth, warning) = resolve_depth(4, None);
        assert_eq!(depth, 1);
        assert!(warning.unwrap().contains("1"));
    }

    #[test]
    fn resolve_depth_floors_a_zero_request_at_one() {
        assert_eq!(resolve_depth(0, Some(4)), (1, None));
    }

    #[test]
    fn parse_depth_list_preserves_order_and_dedups() {
        assert_eq!(parse_depth_list("1,4,2,4").unwrap(), vec![1, 4, 2]);
        assert_eq!(parse_depth_list(" 8 ,, 2 ").unwrap(), vec![8, 2]);
    }

    #[test]
    fn parse_depth_list_rejects_garbage_and_all_empty() {
        assert!(parse_depth_list("banana").is_err());
        assert!(parse_depth_list("").is_err());
        assert!(parse_depth_list(" , ").is_err());
    }

    #[test]
    fn resolve_depth_list_clamps_each_entry_and_drops_clamp_duplicates() {
        let (depths, warnings) = resolve_depth_list(&[1, 4, 8, 16], Some(8));
        assert_eq!(depths, vec![1, 4, 8]);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].contains("16"));
    }

    #[test]
    fn parse_clock_hz_accepts_suffixes_and_plain_integers() {
        assert_eq!(parse_clock_hz("4000000").unwrap(), 4_000_000);
        assert_eq!(parse_clock_hz("4M").unwrap(), 4_000_000);
        assert_eq!(parse_clock_hz("4m").unwrap(), 4_000_000);
        assert_eq!(parse_clock_hz("500k").unwrap(), 500_000);
        assert_eq!(parse_clock_hz("500K").unwrap(), 500_000);
    }

    #[test]
    fn parse_clock_hz_rejects_garbage() {
        assert!(parse_clock_hz("banana").is_err());
        assert!(parse_clock_hz("").is_err());
    }

    #[test]
    fn parse_clock_list_preserves_order_and_dedups() {
        let list = parse_clock_list("1M,2M,1000000,5M").unwrap();
        assert_eq!(list, vec![1_000_000, 2_000_000, 5_000_000]);
    }

    #[test]
    fn parse_clock_list_trims_whitespace_and_skips_empty_tokens() {
        let list = parse_clock_list(" 1M ,, 2M ").unwrap();
        assert_eq!(list, vec![1_000_000, 2_000_000]);
    }

    #[test]
    fn parse_clock_list_rejects_all_empty() {
        assert!(parse_clock_list("").is_err());
        assert!(parse_clock_list(" , , ").is_err());
    }

    #[test]
    fn default_clock_list_matches_the_recorded_hil_ladder() {
        let list = parse_clock_list(DEFAULT_CLOCK_LIST).unwrap();
        assert_eq!(list, vec![1_000_000, 4_000_000, 12_000_000, 25_000_000]);
    }

    #[test]
    fn validate_sweep_range_rejects_zero_start() {
        assert!(validate_sweep_range(0, 1_000_000).is_err());
    }

    #[test]
    fn validate_sweep_range_rejects_start_above_max() {
        assert!(validate_sweep_range(2_000_000, 1_000_000).is_err());
    }

    #[test]
    fn validate_sweep_range_accepts_start_at_or_below_max() {
        assert!(validate_sweep_range(1, 1_000_000).is_ok());
        assert!(validate_sweep_range(1_000_000, 1_000_000).is_ok());
    }
}
