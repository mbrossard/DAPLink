//! Hardware-in-the-loop harness: run named cases against a probe and its wired target.

use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;
use probe_rs::probe::{DebugProbeSelector, WireProtocol};
use probe_rs::{Core, MemoryInterface, Session};

mod bench;
mod connect;
mod control;
mod dap;
mod flash;
mod fwver;
mod identity;
mod info;
mod jtag;
// macOS's storage driver claims the MSC interface exclusively, so raw BOT is impossible there.
#[cfg(not(target_os = "macos"))]
mod msc;
// The msc case's platform-independent half, so it stays under test on macOS too.
mod msc_wire;
mod nreset;
mod profile;
mod reset;
mod rtt;
mod run_record;
mod speed;
mod speed_jtag;
mod speed_swd;
mod swd;
mod swo;
mod target;
mod uf2;
mod util;
mod vcom;
mod vendor_flash;

use profile::Req;
use target::Target;

/// Parse a `--target` value against the target table, erroring with the known names.
fn parse_target(s: &str) -> Result<&'static Target, String> {
    target::target(s).ok_or_else(|| {
        let known: Vec<&str> = target::TARGETS.iter().map(|t| t.name).collect();
        format!("unknown target `{s}`; targets: {}", known.join(", "))
    })
}

pub(crate) struct Ctx {
    pub(crate) selector: DebugProbeSelector,
    pub(crate) chip: String,
    pub(crate) dut_elf: Option<String>,
    pub(crate) dut_uf2: Option<String>,
    pub(crate) session: Option<Session>,
    pub(crate) read_addr: u64,
    /// The target's trace-routing register, absent where its silicon has none.
    pub(crate) dbgmcu_cr: Option<u64>,
    /// The most recent speed sweep's rungs, kept off `CaseFn`'s shared return type; the recorder
    /// takes and clears it right after the case returns.
    pub(crate) last_speed: Option<(WireProtocol, Vec<speed::Step>)>,
}

/// A case verdict short of ok: `Failed` and `Skip` both continue the run -- `Skip` naming an
/// absent capability rather than a fault -- while `Fatal` stops it, the substrate being gone.
pub(crate) enum Failure {
    Failed(String),
    Skip(String),
    Fatal(String),
}

type CaseFn = fn(&mut Ctx, bool) -> Result<(), Failure>;

/// Name, body, whether the case needs `--dut-elf` or `--dut-uf2`, and the rig wiring it requires.
/// Ordered so each case adds one layer: insert a new case at its layer, never append.
const CASES: &[(&str, CaseFn, bool, bool, &[Req])] = &[
    // Enumeration identity from the bus listing, then the vendor descriptors over control.
    ("identity", identity::run, false, false, &[]),
    ("info", info::run, false, false, &[]),
    ("connect", connect::run, false, false, &[]),
    ("swd", swd::run, false, false, &[]),
    ("flash", flash::run, true, false, &[]),
    ("rtt", rtt::run, true, false, &[]),
    ("reset", reset::run, true, false, &[]),
    ("nreset", nreset::run, true, false, &[Req::Nreset]),
    ("vcom", vcom::run, true, false, &[Req::Vcom]),
    ("swo", swo::run, true, false, &[Req::Swo]),
    ("jtag", jtag::run, false, false, &[Req::Jtag]),
    // Not profile-gated: the firmware's answer to the first vendor command is the capability probe.
    ("vendor-flash", vendor_flash::run, false, true, &[]),
    // Not profile-gated either: the mass-storage interface's presence is the capability probe.
    #[cfg(not(target_os = "macos"))]
    ("msc", msc::run, false, true, &[]),
    ("speed-swd", speed_swd::run, false, false, &[]),
    ("speed-jtag", speed_jtag::run, false, false, &[Req::Jtag]),
];

/// SCB AIRCR; writing VECTKEY | SYSRESETREQ is the architecture-generic reset fallback.
const AIRCR: u64 = 0xE000_ED0C;
const AIRCR_SYSRESETREQ: u32 = 0x05FA_0004;

/// Reset the target and leave it running: the chip-specific method, else a raw SYSRESETREQ.
pub(crate) fn reset(core: &mut Core) -> Result<(), String> {
    core.reset().or_else(|first| {
        core.write_word_32(AIRCR, AIRCR_SYSRESETREQ)
            .map_err(|second| {
                format!(
                    "reset failed ({}); SYSRESETREQ failed ({})",
                    util::chain(first),
                    util::chain(second)
                )
            })
    })?;
    if core.core_halted().map_err(util::chain)? {
        core.run().map_err(util::chain)?;
    }
    Ok(())
}

/// Probe test cases, run against a probe wired to a live target.
#[derive(Parser, Debug)]
struct Args {
    /// Probe under test, as a full vid:pid:serial selector
    #[arg(long)]
    probe: String,
    /// Wired target the probe is attached to
    #[arg(long, value_parser = parse_target)]
    target: &'static Target,
    /// DUT image for the flash case; without it, DUT-dependent cases are left out
    #[arg(long)]
    dut_elf: Option<String>,
    /// DUT uf2 image for the vendor-flash case; without it, that case is left out
    #[arg(long)]
    dut_uf2: Option<String>,
    /// Rig profile gating cases on the wiring it provides; without it, no case is skipped
    #[arg(long)]
    profile: Option<String>,
    /// Abilities to withhold beyond profile ∩ target, comma-separated (e.g. `swo`); for a bench
    /// where a normally-supported ability is unwired or temporarily broken
    #[arg(long, value_delimiter = ',')]
    disable_caps: Vec<Req>,
    /// Transport probe for the bench defmt capture: a second session reading RTT off the PUT
    #[arg(long, requires = "probe_elf")]
    via: Option<String>,
    /// Probe firmware ELF (a `defmt` build) the bench capture reads its defmt table from
    #[arg(long, requires = "via")]
    probe_elf: Option<String>,
    /// JSONL run record path, overriding the default under the git-ignored `artifacts/`
    #[arg(long, conflicts_with = "no_record")]
    record: Option<std::path::PathBuf>,
    /// Disable JSONL run recording entirely
    #[arg(long)]
    no_record: bool,
    /// Cases to run (all applicable when empty)
    cases: Vec<String>,
}

fn main() -> ExitCode {
    let Args {
        probe,
        target,
        dut_elf,
        dut_uf2,
        profile,
        disable_caps,
        via,
        probe_elf,
        record,
        no_record,
        cases,
    } = Args::parse();
    let profile = match profile.as_deref() {
        Some(name) => match profile::profile(name) {
            Some(p) => Some(p),
            None => {
                let known: Vec<&str> = profile::PROFILES.iter().map(|p| p.name).collect();
                eprintln!("unknown profile `{name}`; profiles: {}", known.join(", "));
                return ExitCode::FAILURE;
            }
        },
        None => None,
    };
    let full_run = cases.is_empty();
    let selected: Vec<&(&str, CaseFn, bool, bool, &[Req])> = if full_run {
        CASES
            .iter()
            .filter(|(_, _, needs_elf, needs_uf2, _)| {
                (dut_elf.is_some() || !needs_elf) && (dut_uf2.is_some() || !needs_uf2)
            })
            .collect()
    } else {
        let mut sel = Vec::new();
        for name in &cases {
            match CASES.iter().find(|(n, _, _, _, _)| n == name) {
                Some(c) => sel.push(c),
                None => {
                    let known: Vec<&str> = CASES.iter().map(|(n, _, _, _, _)| *n).collect();
                    eprintln!("unknown case `{name}`; cases: {}", known.join(", "));
                    return ExitCode::FAILURE;
                }
            }
        }
        sel
    };

    let selector: DebugProbeSelector = match probe.parse() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("bad probe selector `{probe}`: {e}");
            return ExitCode::FAILURE;
        }
    };
    let mut cx = Ctx {
        selector,
        chip: target.chip.to_string(),
        dut_elf,
        dut_uf2,
        session: None,
        read_addr: target.read_addr,
        dbgmcu_cr: target.dbgmcu_cr,
        last_speed: None,
    };

    let record_path = if no_record {
        None
    } else {
        Some(record.unwrap_or_else(run_record::default_path_for_process))
    };

    // Bench defmt capture: opt in via `--via` + `--probe-elf`, zero impact otherwise.
    let capture = match (via.as_deref(), probe_elf.as_deref()) {
        (Some(via), Some(elf)) => match bench::start(via, std::path::Path::new(elf), &cx.chip) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("bench: defmt capture setup failed: {e}");
                return ExitCode::FAILURE;
            }
        },
        _ => None,
    };

    println!("probe {}  target {}", cx.selector, target.name);
    let firmware_version = match fwver::read(&cx.selector) {
        Ok(version) => {
            println!("firmware {version}");
            if let Some(warning) = fwver::staleness_warning(&version) {
                println!("{warning}");
            }
            Some(version)
        }
        Err(e) => {
            println!("firmware version unavailable ({e})");
            None
        }
    };
    if let Some(path) = &record_path {
        println!("record {}", path.display());
    }

    let meta = run_record::RunMeta::new(
        run_record::run_id(),
        &cx.selector,
        firmware_version.as_deref(),
        target.name,
        &cx.chip,
        cx.dut_elf.as_deref(),
        cx.dut_uf2.as_deref(),
    );

    let record_one = |name: &str,
                      outcome: &'static str,
                      detail: Option<&str>,
                      ms: u64,
                      speed: Option<(WireProtocol, Vec<speed::Step>)>| {
        let Some(path) = &record_path else { return };
        let case = run_record::CaseRecord {
            case: name,
            outcome,
            detail,
            duration_ms: ms,
            speed: speed
                .map(|(protocol, steps)| run_record::SpeedPayload::from_steps(protocol, &steps)),
        };
        if let Err(e) = run_record::append(path, &meta, &case) {
            eprintln!("record: could not append to {}: {e}", path.display());
        }
    };

    let mut failed = false;
    for (name, case, _, _, requires) in selected {
        if let Some((req, side)) =
            profile::missing_requirement(profile, Some(target.supports), &disable_caps, requires)
        {
            let reason = match side {
                profile::Side::Profile => format!("{req}"),
                profile::Side::Target => format!("{req}: target"),
                profile::Side::Disabled => format!("{req}: disabled"),
            };
            println!("{name:<12} skip ({reason})");
            record_one(name, "skip", Some(&reason), 0, None);
            continue;
        }
        let started = Instant::now();
        let outcome = case(&mut cx, full_run);
        let ms = started.elapsed().as_millis() as u64;
        let speed = cx.last_speed.take();
        match outcome {
            Ok(()) => {
                println!("{name:<12} ok");
                record_one(name, "ok", None, ms, speed);
            }
            Err(Failure::Skip(reason)) => {
                println!("{name:<12} skip ({reason})");
                record_one(name, "skip", Some(&reason), ms, speed);
            }
            Err(Failure::Failed(e)) => {
                failed = true;
                println!("{name:<12} FAILED: {e}");
                record_one(name, "failed", Some(&e), ms, speed);
            }
            Err(Failure::Fatal(e)) => {
                failed = true;
                println!("{name:<12} FATAL: {e}");
                record_one(name, "fatal", Some(&e), ms, speed);
                // The substrate later cases depend on is gone; the rest is cascade noise.
                break;
            }
        }
    }
    if let Some(capture) = capture {
        capture.finish();
    }
    if failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASE: &[&str] = &["daplink-test", "--probe", "1:2:x"];

    #[test]
    fn target_parses_known_name() {
        let args = Args::try_parse_from(BASE.iter().copied().chain(["--target", "rp2040"]))
            .expect("valid target parses");
        assert_eq!(args.target.name, "rp2040");
    }

    #[test]
    fn target_rejects_unknown_name() {
        let err = Args::try_parse_from(BASE.iter().copied().chain(["--target", "bogus"]))
            .expect_err("unknown target must fail to parse");
        assert!(err.to_string().contains("unknown target"));
        assert!(err.to_string().contains("stm32f401"));
    }

    #[test]
    fn target_is_required() {
        let err = Args::try_parse_from(BASE.iter().copied())
            .expect_err("missing target must fail to parse");
        assert_eq!(err.kind(), clap::error::ErrorKind::MissingRequiredArgument);
    }

    #[test]
    fn disable_caps_parses_comma_list() {
        let args = Args::try_parse_from(BASE.iter().copied().chain([
            "--target",
            "rp2040",
            "--disable-caps",
            "vcom,nreset",
        ]))
        .expect("valid disable-caps parses");
        assert_eq!(args.disable_caps, vec![Req::Vcom, Req::Nreset]);
    }

    #[test]
    fn disable_caps_absent_is_empty() {
        let args = Args::try_parse_from(BASE.iter().copied().chain(["--target", "rp2040"]))
            .expect("no disable-caps parses");
        assert!(args.disable_caps.is_empty());
    }

    #[test]
    fn disable_caps_rejects_unknown_name() {
        let err = Args::try_parse_from(BASE.iter().copied().chain([
            "--target",
            "rp2040",
            "--disable-caps",
            "bogus",
        ]))
        .expect_err("unknown requirement must fail to parse");
        assert!(err.to_string().contains("unknown requirement"));
    }
}
