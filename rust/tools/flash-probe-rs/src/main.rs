//! Flash a target through `daplink-flash`, with probe-rs acting only as the SWD wire.

mod cli;

use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use daplink_flash::{DefaultCallbacks, EraseMode, FlashManager, Target, TargetFlash};
use daplink_probe_rs::{ProbeRsSwd, RawDapAccess};
use daplink_targets::targets::{rp2040::RP2040, stm32f401ce::STM32F401CE};
use embedded_hal::delay::DelayNs;
use probe_rs::probe::list::Lister;
use probe_rs::probe::{DebugProbeSelector, Probe, WireProtocol};

use cli::{Args, TargetKind};

/// Self-test pattern size and verify granularity: a multiple of four, and within every
/// supported target's program buffer.
const CHUNK: usize = 256;

/// A blocking [`DelayNs`] backed by the host thread's sleep clock.
struct StdDelay;

impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        std::thread::sleep(Duration::from_nanos(u64::from(ns)));
    }
}

fn main() {
    // `daplink-flash` and this tool at debug, the probe-rs/nusb layers at warn.
    let filter = tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
        tracing_subscriber::EnvFilter::new("warn,flash_probe_rs=info,daplink_flash=debug")
    });
    tracing_subscriber::fmt().with_env_filter(filter).init();
    if let Err(err) = run() {
        eprintln!("error: {err:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let args = Args::from_env()?;

    let image = match &args.image {
        Some(path) => {
            Some(std::fs::read(path).with_context(|| format!("reading image file '{path}'"))?)
        }
        None => None,
    };

    let mut probe = open_probe(&args)?;
    probe
        .attach_to_unspecified()
        .context("attaching to target")?;
    let dap = probe
        .try_as_dap_probe()
        .ok_or_else(|| anyhow!("probe does not expose a CMSIS-DAP interface"))?;
    // A `DapProbe` is a `RawDapAccess`; upcast to the raw surface the SWD shim consumes.
    let raw: &mut dyn RawDapAccess = dap;

    let base = args.target.flash_base();
    let erase_mode = args.erase_mode;
    match args.target {
        TargetKind::Stm32f401 => {
            flash_target(raw, &STM32F401CE, base, image.as_deref(), erase_mode)
        }
        TargetKind::Rp2040 => flash_target(raw, &RP2040, base, image.as_deref(), erase_mode),
    }
}

/// Open the probe named by `--probe` (or the first one), select SWD, and set the speed.
fn open_probe(args: &Args) -> Result<Probe> {
    let lister = Lister::new();

    let mut probe = match &args.probe {
        Some(sel) => {
            let selector: DebugProbeSelector = sel
                .parse()
                .with_context(|| format!("parsing --probe selector '{sel}'"))?;
            lister
                .open(selector)
                .with_context(|| format!("opening probe '{sel}'"))?
        }
        None => {
            let all = lister.list_all();
            let info = all
                .first()
                .ok_or_else(|| anyhow!("no debug probes found"))?;
            info.open().context("opening the first available probe")?
        }
    };

    probe
        .select_protocol(WireProtocol::Swd)
        .context("selecting the SWD wire protocol")?;
    let achieved = probe
        .set_speed(args.speed_khz)
        .context("setting the probe speed")?;
    if achieved != args.speed_khz {
        tracing::info!(
            "requested {} kHz, probe set {} kHz",
            args.speed_khz,
            achieved
        );
    }
    Ok(probe)
}

/// One target's flash session: a self-test with no image, else program-then-verify.
fn flash_target<T: Target>(
    raw: &mut dyn RawDapAccess,
    target: &T,
    base: u32,
    image: Option<&[u8]>,
    erase_mode: EraseMode,
) -> Result<()> {
    let swd = ProbeRsSwd::new(raw);
    let mut flash = TargetFlash::new(swd, StdDelay, target, DefaultCallbacks);
    // RP2040's generic reset sequence drops the multidrop DP and wedges the verify reads that
    // follow, so the run stays on the live halted core.
    flash.set_post_flash_reset(false);

    match image {
        None => {
            let pattern = self_test_pattern();
            tracing::info!(
                "self-test at {base:#010x}: erase, program and verify {} bytes",
                pattern.len()
            );
            // Single-sector erase whatever `--erase-mode` says: the self-test bypasses
            // `FlashManager` and smoke-tests init/reset/halt at one location.
            flash.init().map_err(flash_err)?;
            flash.erase_sector(base).map_err(flash_err)?;
            flash.program_page(base, &pattern).map_err(flash_err)?;
            flash.verify(base, &pattern).map_err(flash_err)?;
            flash.uninit().map_err(flash_err)?;
            tracing::info!("self-test passed");
        }
        Some(data) => {
            tracing::info!("programming {} bytes at {base:#010x}", data.len());
            // One MSC sector at a time, the most the on-probe path ever holds of the file.
            let mut buffer = [0u8; 512];
            let mut manager = FlashManager::new(flash, &mut buffer).map_err(flash_err)?;
            manager.set_erase_mode(erase_mode);
            // Per-page verify, because the on-probe path cannot retain the image to verify at
            // the end.
            manager.set_verify(true);
            manager.write(base, data).map_err(flash_err)?;
            // `finish` flushes and verifies the final page under the same no-reset policy.
            let (result, _parts) = manager.finish(false);
            result.map_err(flash_err)?;
            tracing::info!("program and verify complete");
        }
    }
    Ok(())
}

/// A word-aligned alternating `0xAA`/`0x55` pattern for the self-test.
fn self_test_pattern() -> [u8; CHUNK] {
    let mut pattern = [0u8; CHUNK];
    for (i, byte) in pattern.iter_mut().enumerate() {
        *byte = if i % 2 == 0 { 0xAA } else { 0x55 };
    }
    pattern
}

/// Wrap a flash-stack error (which carries no `std::error::Error` impl) as an [`anyhow::Error`].
fn flash_err(err: daplink_flash::Error) -> anyhow::Error {
    anyhow!("flash operation failed: {err:?}")
}
