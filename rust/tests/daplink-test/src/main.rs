//! Hardware-in-the-loop harness: run named cases against a probe and its wired target.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::Parser;
use probe_rs::probe::{DebugProbeSelector, WireProtocol};
use probe_rs::{Core, MemoryInterface, Session};

mod bench;
mod connect;
mod control;
mod dap;
mod dutid;
mod fault;
mod flash;
mod fwver;
mod halt;
mod identity;
mod info;
mod jtag;
mod jtag_chain;
mod load;
// macOS's storage driver claims the MSC interface exclusively, so raw BOT is impossible there.
#[cfg(not(target_os = "macos"))]
mod msc;
// The msc case's platform-independent half, so it stays under test on macOS too.
mod msc_wire;
mod nreset;
mod pipeline;
mod profile;
mod reattach;
mod reset;
mod rtt;
mod run_record;
mod scan;
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
mod wait;

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
    /// The `connect` case's session; a case that drops it for raw USB reconnects or fails FATAL.
    pub(crate) session: Option<Session>,
    pub(crate) read_addr: u64,
    /// An address the target maps to nothing, for the fault case.
    pub(crate) fault_addr: Option<u64>,
    /// IR lengths of the target's JTAG chain, for the jtag-chain case.
    pub(crate) jtag_ir_lens: &'static [u8],
    /// The target's trace-routing register, absent where its silicon has none.
    pub(crate) dbgmcu_cr: Option<u64>,
    /// The latest speed sweep's rungs; the recorder takes and clears it after the case.
    pub(crate) last_speed: Option<(WireProtocol, speed::Sweep)>,
    /// What the speed cases sweep.
    pub(crate) speed: speed::Options,
    /// How the last case succeeded (a retried `flash`, `fault`'s error, `msc-corrupt`'s hold), recorded as its `detail`.
    pub(crate) last_note: Option<String>,
    /// The previous case reset the DUT and left its boot event unread, for `rtt`.
    pub(crate) boot_pending: bool,
}

/// A case verdict short of ok: `Failed` and `Skip` continue the run, `Fatal` stops it.
pub(crate) enum Failure {
    Failed(String),
    Skip(String),
    Fatal(String),
}

type CaseFn = fn(&mut Ctx, bool) -> Result<(), Failure>;

/// One case: its name, body, the DUT images it needs, and the wiring it requires.
struct Case {
    name: &'static str,
    run: CaseFn,
    /// Left out of a full run without `--dut-elf`.
    needs_elf: bool,
    /// Left out of a full run without `--dut-uf2`.
    needs_uf2: bool,
    /// Ends in a DUT reset whose boot event is left for the next case.
    boots: bool,
    requires: &'static [Req],
}

impl Case {
    const fn new(name: &'static str, run: CaseFn) -> Self {
        Self {
            name,
            run,
            needs_elf: false,
            needs_uf2: false,
            boots: false,
            requires: &[],
        }
    }

    const fn boots(mut self) -> Self {
        self.boots = true;
        self
    }

    const fn elf(mut self) -> Self {
        self.needs_elf = true;
        self
    }

    const fn uf2(mut self) -> Self {
        self.needs_uf2 = true;
        self
    }

    const fn requires(mut self, requires: &'static [Req]) -> Self {
        self.requires = requires;
        self
    }
}

/// Ordered by layer, so insert, never append.
const CASES: &[Case] = &[
    // Enumeration identity from the bus listing, then the vendor descriptors over control.
    Case::new("identity", identity::run),
    Case::new("info", info::run),
    // Still raw bulk, so both run before a probe-rs session claims the interface.
    Case::new("pipeline", pipeline::run),
    Case::new("reattach", reattach::run),
    Case::new("reattach-suspend", reattach::run_suspend),
    // Ends in a USB reset, so a stack it wedges cannot take the later cases with it.
    Case::new("halt", halt::run),
    Case::new("connect", connect::run),
    Case::new("swd", swd::run),
    Case::new("fault", fault::run),
    Case::new("flash", flash::run).elf().boots(),
    Case::new("rtt", rtt::run).elf(),
    Case::new("reset", reset::run).elf(),
    Case::new("nreset", nreset::run)
        .elf()
        .requires(&[Req::Nreset]),
    Case::new("vcom", vcom::run).elf().requires(&[Req::Vcom]),
    Case::new("vcom-load", vcom::run_loaded)
        .elf()
        .requires(&[Req::Vcom]),
    Case::new("swo", swo::run).elf().requires(&[Req::Swo]),
    Case::new("swo-load", swo::run_loaded)
        .elf()
        .requires(&[Req::Swo]),
    Case::new("jtag", jtag::run).requires(&[Req::Jtag]),
    Case::new("jtag-chain", jtag_chain::run).requires(&[Req::Jtag]),
    // Not profile-gated: the firmware's answer to the first vendor command is the capability probe.
    Case::new("vendor-flash", vendor_flash::run).uf2(),
    // Not profile-gated either: the mass-storage interface's presence is the capability probe.
    #[cfg(not(target_os = "macos"))]
    Case::new("msc", msc::run).uf2(),
    #[cfg(not(target_os = "macos"))]
    Case::new("msc-corrupt", msc::run_corrupt).uf2(),
    Case::new("speed-swd", speed_swd::run),
    Case::new("speed-jtag", speed_jtag::run).requires(&[Req::Jtag]),
];

/// SCB AIRCR; writing VECTKEY | SYSRESETREQ is the architecture-generic reset fallback.
const AIRCR: u64 = 0xE000_ED0C;
const AIRCR_SYSRESETREQ: u32 = 0x05FA_0004;
/// Outlasts a DUT's boot up to `main`.
const BOOT_SETTLE: Duration = Duration::from_millis(200);

/// Reset the target, leave it running and wait out its boot: the chip-specific method, else a raw SYSRESETREQ.
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
    // Attach reads RTT channel names from flash, and on the RP2040 that XIP read can hang the bootrom's flash load.
    std::thread::sleep(BOOT_SETTLE);
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
    /// Abilities to withhold beyond profile ∩ target, comma-separated (e.g. `swo`)
    ///
    /// For wiring where a normally supported ability is absent or temporarily broken.
    #[arg(long, value_delimiter = ',')]
    disable_caps: Vec<Req>,
    /// Transport probe for bench defmt capture or the GPIO scan's PUT RTT channel
    #[arg(long, requires = "probe_elf")]
    via: Option<String>,
    /// Probe firmware ELF for defmt capture or GPIO scan RTT attachment
    #[arg(long, requires = "via")]
    probe_elf: Option<String>,
    /// Sweep the speed cases over 1-50 MHz with a target RAM read/write pattern check
    ///
    /// Each step reports its median and range over repeats.
    #[arg(long)]
    speed_extended: bool,
    /// Bytes per speed-case read or write call, each swept in turn (default 4096)
    ///
    /// Comma-separated or repeated; multiples of 4 up to 4096.
    #[arg(long, value_delimiter = ',', value_parser = speed::parse_access)]
    speed_access: Vec<usize>,
    /// A second JSONL run record path, beside the ledger under the user's state directory
    #[arg(long, conflicts_with = "no_record")]
    record: Option<std::path::PathBuf>,
    /// Disable JSONL run recording entirely
    #[arg(long)]
    no_record: bool,
    /// Token for one event across its attempts, or an arm from one tree, copied into every record
    #[arg(long)]
    series: Option<String>,
    /// Firmware registry row on the probe, for an image too old to name its own
    ///
    /// `cargo xtask list` names the rows; `external:<name>` names an image outside them.
    #[arg(long)]
    firmware_row: Option<String>,
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
        speed_extended,
        speed_access,
        record,
        no_record,
        series,
        firmware_row,
        cases,
    } = Args::parse();
    if cases.iter().any(|case| case == "scan") {
        return scan::run(scan::Options {
            probe: &probe,
            via: via.as_deref(),
            probe_elf: probe_elf.as_deref(),
            dut_elf: dut_elf.as_deref(),
            target,
            profile: profile.as_deref(),
            cases: &cases,
            firmware_row: firmware_row.as_deref(),
            series: series.as_deref(),
            record: if no_record {
                Vec::new()
            } else {
                run_record::record_paths(record)
            },
        });
    }
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
    let selected: Vec<&Case> = if full_run {
        CASES
            .iter()
            .filter(|c| (dut_elf.is_some() || !c.needs_elf) && (dut_uf2.is_some() || !c.needs_uf2))
            .collect()
    } else {
        let mut sel = Vec::new();
        for name in &cases {
            match CASES.iter().find(|c| c.name == name) {
                Some(c) => sel.push(c),
                None => {
                    let known: Vec<&str> = CASES.iter().map(|c| c.name).collect();
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
        fault_addr: target.fault_addr,
        jtag_ir_lens: target.jtag_ir_lens,
        dbgmcu_cr: target.dbgmcu_cr,
        last_speed: None,
        speed: speed::Options::new(speed_extended, &speed_access),
        last_note: None,
        boot_pending: false,
    };

    let record_paths = if no_record {
        Vec::new()
    } else {
        run_record::record_paths(record)
    };

    let image_row = fwver::read_row(&cx.selector);
    let firmware_row = firmware_row::resolve(image_row.as_deref(), firmware_row.as_deref());

    // Defmt capture: opt in via `--via` + `--probe-elf`.
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
    println!("firmware row {}", firmware_row.describe());
    for warning in &firmware_row.warnings {
        println!("WARNING: {warning}");
    }
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
    // One identity exchange per run, before raw-bulk cases; a silent DUT is recorded absent.
    let dut_id = match dutid::read(&cx) {
        Ok(id) => {
            println!("dut {}", id.describe());
            Some(id)
        }
        Err(e) => {
            println!("dut identity unavailable ({e})");
            None
        }
    };
    for path in &record_paths {
        println!("record {}", path.display());
    }

    let meta = run_record::RunMeta::new(
        run_record::run_id(),
        (if full_run { "full" } else { "list" }, series),
        &cx,
        &firmware_row,
        firmware_version.as_deref(),
        target.name,
        dut_id.as_ref(),
    );

    // The first write is the preflight: a path that cannot take it would lose every row.
    let names: Vec<&str> = selected.iter().map(|c| c.name).collect();
    for path in &record_paths {
        if let Err(e) = run_record::append_start(path, &meta, &names) {
            eprintln!("record: cannot write {}: {e}", path.display());
            if let Some(capture) = capture {
                capture.finish();
            }
            return ExitCode::FAILURE;
        }
    }

    let record_one = |name: &str,
                      outcome: &'static str,
                      detail: Option<&str>,
                      ms: u64,
                      load1: Option<f64>,
                      speed: Option<(WireProtocol, speed::Sweep)>| {
        let case = run_record::CaseRecord {
            case: name,
            outcome,
            detail,
            duration_ms: ms,
            load1,
            waits: wait::take(),
            speed: speed
                .map(|(protocol, sweep)| run_record::SpeedPayload::from_sweep(protocol, &sweep)),
        };
        for path in &record_paths {
            if let Err(e) = run_record::append(path, &meta, &case) {
                eprintln!("record: could not append to {}: {e}", path.display());
            }
        }
    };

    let mut failed = false;
    for &Case {
        name,
        run: case,
        boots,
        requires,
        ..
    } in selected
    {
        let load1 = run_record::load1();
        if let Some((req, side)) =
            profile::missing_requirement(profile, Some(target.supports), &disable_caps, requires)
        {
            let reason = match side {
                profile::Side::Profile => format!("{req}"),
                profile::Side::Target => format!("{req}: target"),
                profile::Side::Disabled => format!("{req}: disabled"),
            };
            println!("{name:<12} skip ({reason})");
            record_one(name, "skip", Some(&reason), 0, load1, None);
            continue;
        }
        let started = Instant::now();
        let outcome = case(&mut cx, full_run);
        cx.boot_pending = boots && outcome.is_ok();
        let ms = started.elapsed().as_millis() as u64;
        let speed = cx.last_speed.take();
        let note = cx.last_note.take();
        match outcome {
            Ok(()) => {
                match &note {
                    Some(n) => println!("{name:<12} ok ({n})"),
                    None => println!("{name:<12} ok"),
                }
                record_one(name, "ok", note.as_deref(), ms, load1, speed);
            }
            Err(Failure::Skip(reason)) => {
                println!("{name:<12} skip ({reason})");
                record_one(name, "skip", Some(&reason), ms, load1, speed);
            }
            Err(Failure::Failed(e)) => {
                failed = true;
                println!("{name:<12} FAILED: {e}");
                record_one(name, "failed", Some(&e), ms, load1, speed);
            }
            Err(Failure::Fatal(e)) => {
                failed = true;
                println!("{name:<12} FATAL: {e}");
                record_one(name, "fatal", Some(&e), ms, load1, speed);
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
    fn case_names_are_unique() {
        for (i, case) in CASES.iter().enumerate() {
            assert!(
                CASES[..i].iter().all(|c| c.name != case.name),
                "{} is listed twice",
                case.name
            );
        }
    }

    #[test]
    fn raw_bulk_cases_run_before_connect_and_session_cases_after() {
        let at = |name: &str| CASES.iter().position(|c| c.name == name).expect(name);
        for raw in [
            "identity",
            "info",
            "pipeline",
            "reattach",
            "reattach-suspend",
        ] {
            assert!(at(raw) < at("connect"), "{raw} runs after connect");
        }
        for session in ["swd", "fault", "flash", "rtt", "reset"] {
            assert!(at("connect") < at(session), "{session} runs before connect");
        }
    }

    /// `rtt` reads the boot event `flash` leaves, so the one booting case sits right before it.
    #[test]
    fn the_booting_case_precedes_rtt() {
        let booting: Vec<&str> = CASES.iter().filter(|c| c.boots).map(|c| c.name).collect();
        assert_eq!(booting, ["flash"]);
        let at = |name: &str| CASES.iter().position(|c| c.name == name).expect(name);
        assert_eq!(at("flash") + 1, at("rtt"));
    }

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
    fn speed_options_default_to_the_basic_sweep() {
        let args = Args::try_parse_from(BASE.iter().copied().chain(["--target", "rp2040"]))
            .expect("no speed options parses");
        assert!(!args.speed_extended);
        assert!(args.speed_access.is_empty());
    }

    #[test]
    fn speed_access_takes_a_list_and_repeats() {
        let args = Args::try_parse_from(BASE.iter().copied().chain([
            "--target",
            "rp2040",
            "--speed-extended",
            "--speed-access",
            "4,64",
            "--speed-access",
            "508",
        ]))
        .expect("valid speed options parse");
        assert!(args.speed_extended);
        assert_eq!(args.speed_access, vec![4, 64, 508]);
    }

    #[test]
    fn speed_access_rejects_a_partial_word() {
        let err = Args::try_parse_from(BASE.iter().copied().chain([
            "--target",
            "rp2040",
            "--speed-access",
            "6",
        ]))
        .expect_err("a partial word must fail to parse");
        assert!(err.to_string().contains("multiple of 4"), "{err}");
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
