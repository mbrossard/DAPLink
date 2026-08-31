//! `swo-bench` command-line entry point.

use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use clap::{Parser, Subcommand};
use dap_bench::{adi, dap, transport};

use swo_bench::{capture, dut, interrupt, itm, report, stream, swo, target};

/// Default baud sweep: the rates the DUT's 84 MHz trace clock divides plus the slow rung a
/// serial-console user would pick.
const DEFAULT_BAUD_LIST: &str = "115200,500k,1M,2M,4M";
/// SWJ clock for the `--swd-load` attach, in Hz.
const DEFAULT_SWD_CLOCK_HZ: u32 = 10_000_000;
/// Attempts at reclaiming the bulk interface after the DUT setup session releases it.
const REOPEN_ATTEMPTS: u32 = 30;
/// Pause between those attempts.
const REOPEN_INTERVAL: Duration = Duration::from_millis(100);
/// Timeout for the identity queries.
const INFO_TIMEOUT: Duration = Duration::from_millis(500);

/// Measure the probe's SWO capture per baud tier, with drop and gap detection.
#[derive(Parser, Debug)]
#[command(name = "swo-bench", version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Sweep baud tiers, capturing the DUT's sequence-tagged ITM stream at each.
    Run(RunArgs),
}

/// Arguments for `swo-bench run`.
#[derive(clap::Args, Debug)]
struct RunArgs {
    /// Probe selector (`VID:PID[:serial]`); required when several probes are attached.
    #[arg(long = "probe")]
    probe: Option<String>,
    /// Emitting target profile; an unknown name errors listing the known ones.
    #[arg(long = "target", default_value = target::DEFAULT_TARGET)]
    target: String,
    /// DUT ELF, so RTT attaches at its `_SEGGER_RTT` symbol instead of scanning target RAM.
    #[arg(long = "dut-elf")]
    dut_elf: Option<String>,
    /// Comma-separated baud sweep (`115200,1M,2M`); accepts `k`/`M` suffixes.
    #[arg(long = "baud", default_value = DEFAULT_BAUD_LIST)]
    baud: String,
    /// Capture window per tier, in milliseconds.
    #[arg(long = "window-ms", default_value_t = report::DEFAULT_WINDOW_MS)]
    window_ms: u64,
    /// Interleave pipelined `DAP_TransferBlock` reads with the capture polls.
    #[arg(long = "swd-load", default_value_t = false)]
    swd_load: bool,
    /// SWJ clock for the `--swd-load` attach, in Hz.
    #[arg(long = "swd-clock", default_value_t = DEFAULT_SWD_CLOCK_HZ)]
    swd_clock: u32,
    /// Commands kept outstanding on the bulk pair, clamped to the probe's advertised
    /// `DAP_PACKET_COUNT`; `1` is lockstep. Default: chosen by transport class, stated in
    /// the header.
    #[arg(long = "depth")]
    depth: Option<usize>,
    /// Skip the one DUT reset the sweep takes before its first tier.
    #[arg(long = "no-dut-reset", default_value_t = false)]
    no_dut_reset: bool,
    /// Emit a single JSON document to stdout instead of the human table; all other run output
    /// moves to stderr.
    #[arg(long = "json")]
    json: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Run(args) => cmd_run(&args),
    }
}

/// Parse one baud token (`4M`, `500k`, `115200`).
fn parse_baud(token: &str) -> Result<u32> {
    let (digits, scale) = match token.chars().last() {
        Some('M') | Some('m') => (&token[..token.len() - 1], 1_000_000),
        Some('k') | Some('K') => (&token[..token.len() - 1], 1_000),
        _ => (token, 1),
    };
    let value = digits
        .parse::<u32>()
        .map_err(|e| anyhow!("invalid baud '{token}': {e}"))?;
    value
        .checked_mul(scale)
        .filter(|v| *v > 0)
        .ok_or_else(|| anyhow!("invalid baud '{token}'"))
}

/// Parse a comma-separated `--baud` list into an ordered, deduplicated sweep.
fn parse_baud_list(list: &str) -> Result<Vec<u32>> {
    let mut out = Vec::new();
    for token in list.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        let baud = parse_baud(token)?;
        if !out.contains(&baud) {
            out.push(baud);
        }
    }
    if out.is_empty() {
        bail!("no baud selected (expected a list like '115200,1M,2M')");
    }
    Ok(out)
}

/// Auto depth for a full-speed probe: measured to lift the exchange rate past the 6667/s that
/// 4 MBd needs, where depth 1 is drain-bound.
const FULL_SPEED_AUTO_DEPTH: usize = 4;
/// Packet size at or above which a probe is treated as high-speed.
const HIGH_SPEED_PACKET_SIZE: u16 = 512;

/// Resolve `--depth`. An explicit request is clamped to the probe's advertised
/// `DAP_PACKET_COUNT` with a warning; absent one, the transport decides -- depth helps when the
/// exchange rate binds and hurts when the payloads are already unfilled -- and the returned
/// rationale states the choice so a recorded run carries its own reasoning.
fn resolve_depth(
    requested: Option<usize>,
    packet_size: Option<u16>,
    packet_count: Option<u8>,
) -> (usize, String, Option<String>) {
    let advertised = usize::from(packet_count.unwrap_or(1)).max(1);
    if let Some(requested) = requested {
        let requested = requested.max(1);
        return if requested > advertised {
            (
                advertised,
                format!("requested {requested}, clamped"),
                Some(format!(
                    "warning: --depth {requested} exceeds the probe's advertised DAP_PACKET_COUNT \
                     ({advertised}); using {advertised}"
                )),
            )
        } else {
            (requested, "requested".to_string(), None)
        };
    }
    match packet_size {
        Some(size) if size >= HIGH_SPEED_PACKET_SIZE => (
            1,
            format!(
                "auto: {}-byte high-speed payloads are unfilled at depth 1, so more depth only \
                 thins the exchanges",
                swo::data_capacity(size)
            ),
            None,
        ),
        Some(size) => {
            let depth = FULL_SPEED_AUTO_DEPTH.min(advertised);
            (
                depth,
                format!(
                    "auto: {}-byte full-speed payloads make the exchange rate the bound, and \
                     pipelining lifts it",
                    swo::data_capacity(size)
                ),
                None,
            )
        }
        None => (
            1,
            "auto: packet size unknown, staying lockstep".to_string(),
            None,
        ),
    }
}

/// The probe's identity, as both benches print it plus the SWO-specific fields.
struct Identity {
    product: Option<String>,
    serial: Option<String>,
    firmware_version: Option<String>,
    packet_size: Option<u16>,
    packet_count: Option<u8>,
    trace_buffer_bytes: Option<u32>,
    capabilities: Option<u8>,
}

/// One optional identity query. A refused query must not leave its answer stranded for the next
/// one to pick up, so it resynchronizes before moving on.
fn optional_query(probe: &mut transport::Probe, req: Vec<u8>) -> Option<Vec<u8>> {
    let out = probe.transfer(&req, INFO_TIMEOUT).ok();
    if out.is_none() || probe.is_desynced() {
        let _ = probe.resynchronize();
    }
    out
}

/// Query the identity fields; a probe that rejects one leaves that field empty.
fn query_identity(probe: &mut transport::Probe) -> Identity {
    let mut query = |req: Vec<u8>| optional_query(probe, req);
    let non_empty = |s: String| if s.is_empty() { None } else { Some(s) };
    Identity {
        product: query(dap::encode_info(dap::InfoId::ProductId))
            .and_then(|r| dap::decode_info_string(&r).ok())
            .and_then(non_empty),
        serial: query(dap::encode_info(dap::InfoId::SerialNumber))
            .and_then(|r| dap::decode_info_string(&r).ok())
            .and_then(non_empty),
        firmware_version: query(dap::encode_info(dap::InfoId::FirmwareVersion))
            .and_then(|r| dap::decode_info_string(&r).ok())
            .and_then(non_empty),
        packet_size: query(dap::encode_info(dap::InfoId::PacketSize))
            .and_then(|r| dap::decode_info_packet_size(&r).ok()),
        packet_count: query(dap::encode_info(dap::InfoId::PacketCount))
            .and_then(|r| dap::decode_info_packet_count(&r).ok()),
        trace_buffer_bytes: query(swo::encode_info(swo::INFO_SWO_TRACE_BUFFER_SIZE))
            .and_then(|r| swo::decode_info_u32(&r).ok()),
        capabilities: query(swo::encode_info(swo::INFO_CAPABILITIES))
            .and_then(|r| swo::decode_info_u8(&r).ok()),
    }
}

/// Print the self-describing header. In `--json` mode it goes to stderr, keeping stdout as the
/// sole JSON document.
fn print_header(
    identity: &Identity,
    profile: &target::TargetProfile,
    args: &RunArgs,
    depth: usize,
    depth_rationale: &str,
) {
    let dash = || "-".to_string();
    let line = |s: String| {
        if args.json {
            eprintln!("{s}");
        } else {
            println!("{s}");
        }
    };
    line(format!(
        "probe    {} (serial {})",
        identity.product.clone().unwrap_or_else(dash),
        identity.serial.clone().unwrap_or_else(dash)
    ));
    line(format!(
        "firmware {}",
        identity.firmware_version.clone().unwrap_or_else(dash)
    ));
    line(format!(
        "packet   {} bytes x {} outstanding",
        identity.packet_size.map_or_else(dash, |v| v.to_string()),
        identity.packet_count.map_or_else(dash, |v| v.to_string()),
    ));
    line(format!("depth    {depth} ({depth_rationale})"));
    line(format!(
        "swo      trace buffer {} bytes, uart capture {}",
        identity
            .trace_buffer_bytes
            .map_or_else(dash, |v| v.to_string()),
        identity
            .capabilities
            .map_or_else(dash, |c| (c & swo::CAP_SWO_UART != 0).to_string()),
    ));
    line(format!("target   {} ({})", profile.name, profile.chip));
    line(format!(
        "load     {}",
        if args.swd_load {
            format!("swd block reads at {} Hz", args.swd_clock)
        } else {
            "none".to_string()
        }
    ));
    // A run that skipped the reset measured whatever it inherited, so the record must say so.
    line(format!(
        "dut      {}",
        if args.no_dut_reset {
            "not reset (--no-dut-reset); the sweep measures inherited state"
        } else {
            "reset once before the first tier"
        }
    ));
}

/// A `VID:PID[:serial]` string both the raw-bulk transport and probe-rs accept.
fn resolve_selector(requested: Option<&str>) -> Result<String> {
    if let Some(sel) = requested {
        // Parsed once here so a malformed selector fails before any hardware is touched.
        let _: transport::ProbeSelector = sel.parse()?;
        return Ok(sel.to_string());
    }
    let probes = transport::enumerate();
    let info = transport::select_cli(&probes, None)?;
    Ok(match &info.serial {
        Some(serial) => format!("{:04x}:{:04x}:{serial}", info.vendor_id, info.product_id),
        None => format!("{:04x}:{:04x}", info.vendor_id, info.product_id),
    })
}

/// Open the probe's bulk interface, retrying while a just-dropped setup session still holds it.
fn open_probe(selector: &str) -> Result<transport::Probe> {
    let parsed: transport::ProbeSelector = selector.parse()?;
    let mut last = None;
    for _ in 0..REOPEN_ATTEMPTS {
        let probes = transport::enumerate();
        match transport::select(&probes, &parsed).map(|info| transport::Probe::open(info.clone())) {
            Ok(Ok(probe)) => return Ok(probe),
            Ok(Err(e)) => last = Some(e),
            Err(e) => last = Some(e),
        }
        std::thread::sleep(REOPEN_INTERVAL);
    }
    Err(last.unwrap_or_else(|| anyhow!("no probe matched '{selector}'")))
        .context("claiming the probe's bulk interface failed")
}

/// Runs its action on drop unless disarmed, so a panic still reaches the DUT teardown.
struct DropGuard<F: FnOnce()> {
    action: Option<F>,
}

impl<F: FnOnce()> DropGuard<F> {
    fn new(action: F) -> Self {
        Self {
            action: Some(action),
        }
    }

    fn disarm(&mut self) {
        self.action = None;
    }
}

impl<F: FnOnce()> Drop for DropGuard<F> {
    fn drop(&mut self) {
        if let Some(action) = self.action.take() {
            action();
        }
    }
}

/// Put the DUT into (or out of) its continuous stream at `baud`, resetting it first when
/// `reset` is set.
fn setup_dut(
    selector: &str,
    profile: &target::TargetProfile,
    elf: Option<&str>,
    baud: Option<u32>,
    reset: bool,
) -> Result<()> {
    let mut d = dut::Dut::attach(selector, profile.chip, elf)?;
    if reset {
        d.reset()
            .context("resetting the DUT before the sweep (--no-dut-reset skips it)")?;
    }
    // Always stop first: a tier's rate change must not land mid-stream.
    d.set_stream(false)?;
    if let Some(baud) = baud {
        d.set_swo_baud(baud)?;
        d.set_stream(true)?;
    }
    Ok(())
}

/// Attach over raw bulk and confirm one block read, leaving the MEM-AP programmed for the
/// interleaved load; the handle is dropped so the capture loop owns the one command stream.
fn prepare_load(
    probe: &mut transport::Probe,
    profile: &target::TargetProfile,
    clock_hz: u32,
) -> Result<capture::LoadPlan> {
    let mut adi = adi::Adiv5::attach(
        probe,
        adi::AttachConfig {
            clock_hz,
            ..adi::AttachConfig::default()
        },
    )
    .context("SWD attach for the concurrent load failed")?;
    let words = dap::transfer_block_read_capacity(adi.packet_size()).max(1);
    if !profile.flash_contains(words * 4) {
        bail!(
            "target '{}' flash window is smaller than one load chunk",
            profile.name
        );
    }
    let mut probe_buf = vec![0u32; words];
    adi.read_block(profile.flash_base, &mut probe_buf)
        .context("the load's first block read failed")?;
    Ok(capture::LoadPlan {
        addr: profile.flash_base,
        words,
        dap_index: 0,
    })
}

/// Assert the DUT's trace-pin enable over raw DAP, leaving no attach behind it.
fn route_trace_raw(
    probe: &mut transport::Probe,
    profile: &target::TargetProfile,
    clock_hz: u32,
) -> Result<()> {
    let reg = profile.trace_route_reg as u32;
    let mut adi = adi::Adiv5::attach(
        probe,
        adi::AttachConfig {
            clock_hz,
            ..Default::default()
        },
    )?;
    let cr = adi.read_mem32(reg)?;
    // Asynchronous SWO: trace enabled, TRACE_MODE cleared.
    adi.write_mem32(reg, (cr & !(0b11 << 6)) | (1 << 5))?;
    Ok(())
}

/// Measure one baud tier end to end: DUT setup, capture, decode, judge.
fn run_tier(
    selector: &str,
    profile: &target::TargetProfile,
    args: &RunArgs,
    baud: u32,
    depth: usize,
    reset: bool,
) -> Result<report::Tier> {
    setup_dut(
        selector,
        profile,
        args.dut_elf.as_deref(),
        Some(baud),
        reset,
    )
    .with_context(|| format!("preparing the DUT for {baud} baud failed"))?;

    let mut probe = open_probe(selector)?;
    // The trace-pin enable lives in the debug domain, which a probe-rs attach rewrites
    // wholesale, so it must be routed after the tier's last attach.
    route_trace_raw(&mut probe, profile, args.swd_clock)
        .with_context(|| format!("routing the {baud} baud tier's trace pin failed"))?;
    if probe.stale_on_claim() > 0 {
        // These packets answered commands this handle never sent: a tier boundary was crossed.
        eprintln!(
            "note: the {baud} baud tier's claim found {} stale response packet(s) on the probe",
            probe.stale_on_claim()
        );
    }
    let packet_size = probe_packet_size(&mut probe);
    let load = if args.swd_load {
        Some(prepare_load(&mut probe, profile, args.swd_clock)?)
    } else {
        None
    };
    let cfg = capture::Config {
        baud,
        window: Duration::from_millis(args.window_ms),
        depth,
        data_count: swo::data_capacity(packet_size),
        load,
    };
    let captured = capture::run(&mut probe, &cfg)
        .with_context(|| format!("capturing at {baud} baud failed"))?;
    release_probe(probe, baud);

    Ok(tier_from(profile, baud, &captured, args.swd_load))
}

/// Release a tier's bulk handle only once the wire is quiet: dropping it does not cancel a
/// response the probe already armed, which the next tier's claim would inherit.
fn release_probe(mut probe: transport::Probe, baud: u32) {
    if probe.is_desynced() {
        match probe.resynchronize() {
            Ok(0) => {}
            Ok(n) => eprintln!(
                "note: the {baud} baud tier left {n} unread response(s) on the probe; \
                 discarded before release"
            ),
            Err(e) => eprintln!("note: draining the {baud} baud tier's probe failed: {e:#}"),
        }
    }
    drop(probe);
}

/// The probe's `DAP_PACKET_SIZE`, falling back to the CMSIS-DAP minimum. As the tier's first
/// command, a silent fallback would absorb a leftover response, so it announces and resyncs.
fn probe_packet_size(probe: &mut transport::Probe) -> u16 {
    let decoded = optional_query(probe, dap::encode_info(dap::InfoId::PacketSize))
        .and_then(|r| dap::decode_info_packet_size(&r).ok())
        .filter(|v| *v >= 64);
    match decoded {
        Some(size) => size,
        None => {
            eprintln!(
                "warning: the probe did not report a usable DAP_PACKET_SIZE; assuming 64 bytes"
            );
            probe.mark_desynced();
            let _ = probe.resynchronize();
            64
        }
    }
}

/// Decode a capture into its tier row.
fn tier_from(
    profile: &target::TargetProfile,
    baud: u32,
    captured: &capture::Capture,
    swd_load: bool,
) -> report::Tier {
    let decoded = itm::decode_port(&captured.bytes, profile.stimulus_port);
    let framing = stream::scan(&decoded.payload);
    let seconds = captured.elapsed.as_secs_f64().max(f64::MIN_POSITIVE);
    let verdict = report::Verdict::of(&framing, captured.bytes.len(), decoded.payload.len());
    report::Tier {
        requested_baud: baud,
        probe_baud: captured.actual_baud,
        dut_baud: profile.realized_baud(baud),
        window_ms: captured.elapsed.as_secs_f64() * 1000.0,
        captured_bytes: captured.bytes.len(),
        kib_per_s: captured.bytes.len() as f64 / 1024.0 / seconds,
        payload_bytes: decoded.payload.len(),
        itm_overflow: decoded.overflow,
        decode_errors: decoded.errors,
        verdict,
        framing,
        polls: captured.polls,
        full_polls: captured.full_polls,
        inactive_polls: captured.inactive_polls,
        probe_bytes_left: captured.final_status.bytes_available,
        swd_kib_per_s: swd_load.then(|| captured.swd_words as f64 * 4.0 / 1024.0 / seconds),
        swd_mismatches: captured.swd_mismatches,
        swd_errors: captured.swd_errors,
    }
}

/// `swo-bench run ...`.
fn cmd_run(args: &RunArgs) -> Result<()> {
    let bauds = parse_baud_list(&args.baud)?;
    let profile = target::lookup(&args.target)?;
    for &baud in &bauds {
        if profile.realized_baud(baud).is_none() {
            bail!(
                "target '{}' cannot divide its {} Hz trace clock down to {baud} baud",
                profile.name,
                profile.trace_clock_hz
            );
        }
    }
    let selector = resolve_selector(args.probe.as_deref())?;

    let identity = {
        let mut probe = open_probe(&selector)?;
        query_identity(&mut probe)
    };
    let (depth, depth_rationale, warning) =
        resolve_depth(args.depth, identity.packet_size, identity.packet_count);
    print_header(&identity, profile, args, depth, &depth_rationale);
    if let Some(warning) = warning {
        if args.json {
            eprintln!("{warning}");
        } else {
            println!("{warning}");
        }
    }
    if identity
        .capabilities
        .is_some_and(|c| c & swo::CAP_SWO_UART == 0)
    {
        bail!("probe does not advertise SWO UART capture");
    }
    if !args.json {
        println!();
        println!("{}", report::column_header());
    }

    interrupt::install();
    let started = Instant::now();
    let mut rows = Vec::with_capacity(bauds.len());
    // A panic must not skip the teardown; the normal path disarms this and stops the DUT loudly.
    let mut guard = DropGuard::new(|| {
        if let Err(e) = setup_dut(&selector, profile, args.dut_elf.as_deref(), None, false) {
            eprintln!("error: stopping the DUT stream failed: {e:#}");
        }
    });
    // Once per invocation, never per tier: on a rig where the DUT hangs off its own USB lead a
    // reset churns that port, and one per case has taken a host controller down. The first tier
    // consumes it; the tiers after it are already starting from a DUT this sweep configured.
    let mut pending_reset = !args.no_dut_reset;
    let result = (|| -> Result<()> {
        for &baud in &bauds {
            if interrupt::triggered() {
                bail!("interrupted before the {baud} baud tier");
            }
            let tier = run_tier(
                &selector,
                profile,
                args,
                baud,
                depth,
                std::mem::take(&mut pending_reset),
            )?;
            if !args.json {
                println!("{}", report::render(&tier));
                for note in report::notes(&tier) {
                    println!("{note}");
                }
            }
            rows.push(tier);
        }
        Ok(())
    })();
    guard.disarm();

    // Always leave the DUT quiet, whether the sweep finished or aborted. A DUT left streaming
    // poses as a dead one to the next session, so a failed stop fails the run.
    let teardown = setup_dut(&selector, profile, args.dut_elf.as_deref(), None, false)
        .context("stopping the DUT stream failed; the DUT may still be streaming");
    if let Err(e) = result {
        if let Err(t) = teardown {
            eprintln!("error: {t:#}");
        }
        return Err(e);
    }

    if args.json {
        let ctx = report::Context {
            timestamp_utc: dap_bench::report::format_iso8601_utc(unix_time_now()),
            probe_product: identity.product,
            probe_serial: identity.serial,
            firmware_version: identity.firmware_version,
            packet_size: identity.packet_size,
            packet_count: identity.packet_count,
            trace_buffer_bytes: identity.trace_buffer_bytes,
            target: profile.name,
            window_ms: args.window_ms,
            depth,
            depth_rationale: &depth_rationale,
            swd_load: args.swd_load,
            dut_reset: !args.no_dut_reset,
            bauds: &bauds,
        };
        print!("{}", report::to_json(&ctx, &rows));
    } else {
        println!();
        println!(
            "{} tier(s) in {:.1} s; {} clean",
            rows.len(),
            started.elapsed().as_secs_f64(),
            rows.iter()
                .filter(|r| r.verdict == report::Verdict::Clean)
                .count()
        );
    }
    teardown
}

/// Seconds since the Unix epoch, for the JSON run timestamp.
fn unix_time_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_baud_accepts_suffixes_and_plain_integers() {
        assert_eq!(parse_baud("115200").unwrap(), 115_200);
        assert_eq!(parse_baud("2M").unwrap(), 2_000_000);
        assert_eq!(parse_baud("2m").unwrap(), 2_000_000);
        assert_eq!(parse_baud("500k").unwrap(), 500_000);
        assert_eq!(parse_baud("500K").unwrap(), 500_000);
    }

    #[test]
    fn parse_baud_rejects_garbage_and_zero() {
        assert!(parse_baud("banana").is_err());
        assert!(parse_baud("").is_err());
        assert!(parse_baud("0").is_err());
        assert!(parse_baud("M").is_err());
        assert!(parse_baud("5000M").is_err(), "overflows u32");
    }

    #[test]
    fn parse_baud_list_preserves_order_and_dedups() {
        let list = parse_baud_list("115200,1M,1000000,2M").unwrap();
        assert_eq!(list, vec![115_200, 1_000_000, 2_000_000]);
    }

    #[test]
    fn parse_baud_list_trims_whitespace_and_skips_empty_tokens() {
        assert_eq!(
            parse_baud_list(" 1M ,, 2M ").unwrap(),
            vec![1_000_000, 2_000_000]
        );
        assert!(parse_baud_list("").is_err());
        assert!(parse_baud_list(" , , ").is_err());
    }

    #[test]
    fn the_default_sweep_parses_and_the_target_can_realize_every_rung() {
        let list = parse_baud_list(DEFAULT_BAUD_LIST).unwrap();
        assert_eq!(
            list,
            vec![115_200, 500_000, 1_000_000, 2_000_000, 4_000_000]
        );
        let profile = target::lookup(target::DEFAULT_TARGET).unwrap();
        assert!(list.iter().all(|&b| profile.realized_baud(b).is_some()));
    }

    #[test]
    fn the_drop_guard_fires_on_panic_and_not_after_disarm() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;

        let fired = Arc::new(AtomicUsize::new(0));
        let observed = fired.clone();
        let mut guard = DropGuard::new(move || {
            fired.fetch_add(1, Ordering::SeqCst);
        });
        guard.disarm();
        drop(guard);
        assert_eq!(observed.load(Ordering::SeqCst), 0);

        let fired = Arc::new(AtomicUsize::new(0));
        let armed = fired.clone();
        let outcome = std::panic::catch_unwind(move || {
            let _guard = DropGuard::new(move || {
                armed.fetch_add(1, Ordering::SeqCst);
            });
            panic!("sweep died");
        });
        assert!(outcome.is_err());
        assert_eq!(
            fired.load(Ordering::SeqCst),
            1,
            "the teardown must survive a panic"
        );
    }

    #[test]
    fn an_explicit_depth_wins_but_clamps_to_the_advertised_count() {
        assert_eq!(resolve_depth(Some(4), Some(64), Some(8)).0, 4);
        assert_eq!(resolve_depth(Some(0), Some(64), Some(4)).0, 1);
        let (depth, rationale, warning) = resolve_depth(Some(16), Some(64), Some(4));
        assert_eq!(depth, 4);
        assert!(rationale.contains("clamped"));
        assert!(warning.unwrap().contains("16"));
        assert_eq!(resolve_depth(Some(4), Some(64), None).0, 1);
    }

    #[test]
    fn auto_depth_pipelines_full_speed_and_keeps_high_speed_lockstep() {
        // Full speed: 60-byte payloads, the exchange rate binds, pipeline up to 4.
        let (depth, rationale, warning) = resolve_depth(None, Some(64), Some(4));
        assert_eq!(depth, 4);
        assert!(rationale.contains("60-byte"));
        assert!(warning.is_none());
        // High speed: 508-byte payloads already unfilled, stay lockstep.
        let (depth, rationale, _) = resolve_depth(None, Some(512), Some(8));
        assert_eq!(depth, 1);
        assert!(rationale.contains("508-byte"));
        // The advertised count still bounds the full-speed choice.
        assert_eq!(resolve_depth(None, Some(64), Some(2)).0, 2);
        // Without a packet size the class is unknown; lockstep, and say why.
        let (depth, rationale, _) = resolve_depth(None, None, Some(8));
        assert_eq!(depth, 1);
        assert!(rationale.contains("unknown"));
    }
}
