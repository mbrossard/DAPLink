//! Flash a target through `daplink-flash`, with probe-rs acting only as the SWD wire.

mod cli;

use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use daplink_flash::{DefaultCallbacks, EraseMode, FlashManager, Target, TargetFlash};
use daplink_flash::{Error as FlashError, FlashInterface};
use daplink_probe_rs::{ProbeRsSwd, RawDapAccess};
use daplink_targets::targets::{rp2040::RP2040, stm32f401ce::STM32F401CE};
use embedded_hal::delay::DelayNs;
use probe_rs::architecture::arm::ArmError;
use probe_rs::probe::list::Lister;
use probe_rs::probe::{DebugProbeSelector, Probe, WireProtocol};

use cli::{Args, TargetKind};

/// Self-test pattern size: a multiple of four, within every target's program buffer.
const CHUNK: usize = 256;

/// A blocking [`DelayNs`] backed by the host thread's sleep clock.
struct StdDelay;

impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        std::thread::sleep(Duration::from_nanos(u64::from(ns)));
    }
}

fn main() {
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
    let Some(args) = Args::from_env()? else {
        println!("{}", cli::USAGE);
        return Ok(());
    };

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
    let raw: &mut dyn RawDapAccess = dap;

    let erase_mode = args.erase_mode;
    match args.target {
        TargetKind::Stm32f401 => flash_target(raw, &STM32F401CE, image.as_deref(), erase_mode),
        TargetKind::Rp2040 => flash_target(raw, &RP2040, image.as_deref(), erase_mode),
    }
}

/// Open the selected or sole available probe, select SWD, and set the speed.
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
            sole_probe(&all)?
                .open()
                .context("opening the sole available probe")?
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

fn sole_probe<T>(probes: &[T]) -> Result<&T> {
    match probes {
        [probe] => Ok(probe),
        [] => Err(anyhow!("no debug probes found")),
        _ => Err(anyhow!(
            "multiple debug probes found; pass --probe <selector>"
        )),
    }
}

#[derive(Debug, PartialEq, Eq)]
struct SelfTestError {
    primary: FlashError,
    teardown: Option<FlashError>,
}

fn self_test(flash: &mut impl FlashInterface, base: u32) -> Result<(), SelfTestError> {
    flash.init().map_err(|primary| SelfTestError {
        primary,
        teardown: None,
    })?;
    let pattern = self_test_pattern();
    let operation = (|| {
        flash.erase_sector(base)?;
        flash.program_page(base, &pattern)?;
        flash.verify(base, &pattern)
    })();
    let teardown = flash.uninit();
    match (operation, teardown) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(primary), cleanup) => Err(SelfTestError {
            primary,
            teardown: cleanup.err(),
        }),
        (Ok(()), Err(primary)) => Err(SelfTestError {
            primary,
            teardown: None,
        }),
    }
}

/// One target's flash session: a self-test with no image, else program-then-verify.
fn flash_target<T: Target>(
    raw: &mut dyn RawDapAccess,
    target: &T,
    image: Option<&[u8]>,
    erase_mode: EraseMode,
) -> Result<()> {
    let regions = target.flash_regions();
    let base = regions
        .iter()
        .find(|r| r.is_default)
        .or_else(|| regions.first())
        .ok_or_else(|| anyhow!("{} has no flash region", target.name()))?
        .start;
    let swd = ProbeRsSwd::new(raw);
    let mut flash = TargetFlash::new(swd, StdDelay, target, DefaultCallbacks);
    // RP2040's generic reset drops the multidrop DP and wedges the verify reads.
    flash.set_post_flash_reset(false);

    let mut buffer = [0u8; 512];
    let (result, flash) = match image {
        None => {
            tracing::info!(
                "self-test at {base:#010x}: erase, program and verify {} bytes",
                CHUNK
            );
            // The self-test bypasses `FlashManager`, so it always erases a single sector.
            (self_test(&mut flash, base), flash)
        }
        Some(data) => {
            tracing::info!("programming {} bytes at {base:#010x}", data.len());
            let mut manager =
                FlashManager::new(flash, &mut buffer).map_err(|err| flash_err(err, None))?;
            manager.set_erase_mode(erase_mode);
            // Per-page verify: the on-probe path cannot retain the image to verify at the end.
            manager.set_verify(true);
            let (result, (flash, _)) = match manager.write(base, data) {
                Ok(()) => manager.finish(false),
                Err(err) => (Err(err), manager.abort()),
            };
            (
                result.map_err(|primary| SelfTestError {
                    primary,
                    teardown: None,
                }),
                flash,
            )
        }
    };
    let (mut swd, _) = flash.into_parts();
    result.map_err(|err| {
        let cause = swd.take_transport_error();
        match err.teardown {
            Some(teardown) => {
                let primary = flash_err(err.primary, None)
                    .context(format!("teardown also failed: {teardown:?}"));
                match cause {
                    Some(cause) => primary.context(format!(
                        "last transport failure during operation or teardown: {cause:?}"
                    )),
                    None => primary,
                }
            }
            None => flash_err(err.primary, cause),
        }
    })?;
    match image {
        Some(_) => tracing::info!("program and verify complete"),
        None => tracing::info!("self-test passed"),
    }
    Ok(())
}

fn self_test_pattern() -> [u8; CHUNK] {
    let mut pattern = [0u8; CHUNK];
    for (i, byte) in pattern.iter_mut().enumerate() {
        *byte = if i % 2 == 0 { 0xAA } else { 0x55 };
    }
    pattern
}

/// Wrap a flash-stack error (which has no `std::error::Error` impl) over its probe-rs cause, if any.
fn flash_err(err: daplink_flash::Error, cause: Option<ArmError>) -> anyhow::Error {
    let context = format!("flash operation failed: {err:?}");
    match cause {
        Some(cause) => anyhow::Error::new(cause).context(context),
        None => anyhow!(context),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selection_requires_exactly_one_probe() {
        assert!(sole_probe::<u8>(&[]).is_err());
        assert_eq!(*sole_probe(&[7]).unwrap(), 7);
        assert!(sole_probe(&[7, 8]).is_err());
    }

    #[derive(Default)]
    struct Flash {
        calls: Vec<&'static str>,
        fail_at: Option<&'static str>,
        fail_teardown: bool,
    }

    impl Flash {
        fn step(&mut self, name: &'static str, error: FlashError) -> Result<(), FlashError> {
            self.calls.push(name);
            if self.fail_at == Some(name) {
                Err(error)
            } else {
                Ok(())
            }
        }
    }

    impl FlashInterface for Flash {
        fn init(&mut self) -> Result<(), FlashError> {
            self.step("init", FlashError::InitFailed)
        }
        fn uninit(&mut self) -> Result<(), FlashError> {
            self.calls.push("uninit");
            if self.fail_teardown {
                Err(FlashError::UninitFailed)
            } else {
                Ok(())
            }
        }
        fn erase_sector(&mut self, _: u32) -> Result<(), FlashError> {
            self.step("erase", FlashError::EraseFailed)
        }
        fn program_page(&mut self, _: u32, data: &[u8]) -> Result<(), FlashError> {
            assert_eq!(data, self_test_pattern());
            self.step("program", FlashError::WriteFailed)
        }
        fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashError> {
            assert_eq!(data, self_test_pattern());
            self.step("verify", FlashError::VerifyFailed { addr })
        }
        fn set_post_flash_reset(&mut self, _: bool) {}
        fn erase_chip(&mut self) -> Result<(), FlashError> {
            unreachable!()
        }
        fn program_page_min_size(&self, _: u32) -> u32 {
            unreachable!()
        }
        fn erase_sector_size(&self, _: u32) -> Option<u32> {
            unreachable!()
        }
        fn erase_sector_base(&self, _: u32) -> Option<u32> {
            unreachable!()
        }
        fn supports_chip_erase(&self) -> bool {
            unreachable!()
        }
        fn supports_sector_erase(&self) -> bool {
            unreachable!()
        }
    }

    #[test]
    fn self_test_tears_down_once_after_every_operation_outcome() {
        for (step, error, expected) in [
            (
                None,
                None,
                vec!["init", "erase", "program", "verify", "uninit"],
            ),
            (
                Some("erase"),
                Some(FlashError::EraseFailed),
                vec!["init", "erase", "uninit"],
            ),
            (
                Some("program"),
                Some(FlashError::WriteFailed),
                vec!["init", "erase", "program", "uninit"],
            ),
            (
                Some("verify"),
                Some(FlashError::VerifyFailed { addr: 0 }),
                vec!["init", "erase", "program", "verify", "uninit"],
            ),
        ] {
            for fail_teardown in [false, true] {
                let mut flash = Flash {
                    fail_at: step,
                    fail_teardown,
                    ..Flash::default()
                };
                let result = self_test(&mut flash, 0);
                assert_eq!(flash.calls, expected);
                let primary = error.or(fail_teardown.then_some(FlashError::UninitFailed));
                assert_eq!(
                    result,
                    primary.map_or(Ok(()), |primary| Err(SelfTestError {
                        primary,
                        teardown: (error.is_some() && fail_teardown)
                            .then_some(FlashError::UninitFailed),
                    }))
                );
            }
        }
    }

    #[test]
    fn failed_init_does_not_start_operations_or_teardown() {
        let mut flash = Flash {
            fail_at: Some("init"),
            fail_teardown: true,
            ..Flash::default()
        };
        assert_eq!(
            self_test(&mut flash, 0),
            Err(SelfTestError {
                primary: FlashError::InitFailed,
                teardown: None
            })
        );
        assert_eq!(flash.calls, ["init"]);
    }
}
