//! Build orchestration for the firmware fleet, run as `cargo xtask <verb>` from `rust/`.

mod bootsel;
// The scan runs only as a test; nothing in the built tool reaches it.
#[cfg(test)]
mod debug_assert;
mod dep_pins;
mod elf;
mod gate;
mod install;
mod manifest_facts;
mod matrix;
mod reattach;
mod registry;
mod revision;
mod sources;
mod stack;
mod tree;
mod usb;
mod workspaces;

use clap::{Args, Parser, Subcommand};
use install::{build, install, put_selector, route};
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, ExitCode};

/// Resolve a registry row by full path or unambiguous basename.
fn find_firmware(name: &str) -> Option<&'static registry::Firmware> {
    if let Some(fw) = registry::FIRMWARE.iter().find(|f| f.name == name) {
        return Some(fw);
    }
    let mut hits = registry::FIRMWARE
        .iter()
        .filter(|f| f.name.rsplit('/').next() == Some(name));
    let first = hits.next()?;
    match hits.next() {
        None => Some(first),
        Some(second) => {
            eprintln!("`{name}` is ambiguous: {}, {}", first.name, second.name);
            for f in hits {
                eprintln!("                  {}", f.name);
            }
            None
        }
    }
}

fn unknown_firmware(name: &str) {
    eprintln!("unknown firmware `{name}`; `cargo xtask list` names them");
    let near: Vec<&str> = registry::FIRMWARE
        .iter()
        .map(|f| f.name)
        .filter(|n| {
            n.rsplit('/')
                .next()
                .is_some_and(|b| b.contains(name) || name.contains(b))
        })
        .collect();
    if !near.is_empty() {
        eprintln!("did you mean: {}", near.join(", "));
    }
}

/// The `rust/` tree root; registry paths resolve against it.
fn root() -> PathBuf {
    tree::checked()
        .expect("checked first in main")
        .to_path_buf()
}

/// `list`: firmware rows, shapes, targets and the gate tables; a closed reader ends it cleanly.
fn list(out: &mut impl Write) -> ExitCode {
    let shapes = match matrix::shapes(&root()) {
        Ok(shapes) => shapes,
        Err(error) => {
            eprintln!("could not read feature matrix: {error}");
            return ExitCode::FAILURE;
        }
    };
    // Columns align within each table, so long shape labels do not widen the others.
    let firmware: Vec<[String; 3]> = registry::FIRMWARE
        .iter()
        .map(|fw| [fw.name.into(), fw.target.into(), fw.chip.into()])
        .collect();
    let mut shaped = Vec::new();
    for shape in shapes {
        shaped.push([
            format!("shape  {}", shape.label()),
            shape.base.target.into(),
            shape.features.join(","),
        ]);
    }
    let mut targets = Vec::new();
    for t in registry::TARGETS {
        let dut_chip = registry::FIRMWARE
            .iter()
            .find(|f| f.name == t.dut)
            .map(|f| f.chip)
            .unwrap_or("?");
        targets.push([format!("target {}", t.name), dut_chip.into(), t.dut.into()]);
    }
    let mut gate = Vec::new();
    for ws in workspaces::WORKSPACES {
        gate.push([
            format!("workspace {}", ws.name),
            ws.path.into(),
            ws.features.join(","),
        ]);
    }
    for ws in workspaces::embedded() {
        gate.push([
            format!("embedded  {}", ws.name),
            ws.path.into(),
            String::new(),
        ]);
    }
    for c in workspaces::CROSS {
        gate.push([
            format!("cross     {}", c.name),
            c.path.into(),
            c.target.into(),
        ]);
    }
    for ex in workspaces::EXCLUDED {
        gate.push([
            format!("excluded  {}", ex.path),
            String::new(),
            ex.reason.into(),
        ]);
    }
    let written = [firmware, shaped, targets, gate]
        .iter()
        .try_for_each(|rows| write_columns(out, rows));
    match written {
        Err(error) if error.kind() != std::io::ErrorKind::BrokenPipe => {
            eprintln!("could not write the list: {error}");
            ExitCode::FAILURE
        }
        _ => ExitCode::SUCCESS,
    }
}

/// Each column but the last padded to its widest cell.
fn write_columns(out: &mut impl Write, rows: &[[String; 3]]) -> std::io::Result<()> {
    let width = |i: usize| rows.iter().map(|r| r[i].len()).max().unwrap_or(0);
    let (first, second) = (width(0), width(1));
    for [a, b, c] in rows {
        let line = format!("{a:<first$} {b:<second$} {c}");
        writeln!(out, "{}", line.trim_end())?;
    }
    out.flush()
}

/// Build orchestration for the firmware fleet.
#[derive(Parser)]
#[command(name = "cargo xtask", bin_name = "cargo xtask")]
struct Cli {
    #[command(subcommand)]
    verb: Verb,
}

#[derive(Subcommand)]
enum Verb {
    /// Print the firmware rows, feature shapes, targets and gate tables
    List,
    /// Release-build one row, or with no name the default compositions
    Build { name: Option<String> },
    /// Build every row, then the feature shapes
    BuildAll,
    /// Compile-check the feature shapes alone
    Matrix {
        /// Build instead, catching section overflows and duplicate symbols
        #[arg(long)]
        link: bool,
    },
    /// Print dependencies pinned in several workspaces; fail when their revisions differ
    Pins,
    /// Check tracked lockfiles and pins, then run fmt, clippy and tests in every workspace row
    Test {
        /// Also run build-all
        #[arg(long)]
        build: bool,
    },
    /// Build a row, flash it and reset
    Flash(FlashArgs),
    /// Build a probe row and the target's DUT image, flash the probe, run the HIL cases
    Hil(HilArgs),
}

#[derive(Args)]
struct FlashArgs {
    /// Registry path or unambiguous basename
    name: String,
    /// Transport probe, as vid:pid:serial
    #[arg(long, alias = "probe")]
    via: Option<String>,
    /// The flashed board's own USB identity, as vid:pid:serial
    #[arg(long)]
    put: Option<String>,
    /// Where the row's BOOTSEL drive is mounted
    #[arg(long)]
    bootsel_mount: Option<PathBuf>,
}

#[derive(Args)]
struct HilArgs {
    /// Registry path or unambiguous basename
    name: String,
    /// Transport probe, as vid:pid:serial
    #[arg(long)]
    via: Option<String>,
    /// Probe under test, as vid:pid:serial
    #[arg(long, alias = "probe")]
    put: String,
    /// Wired target; `cargo xtask list` names them
    #[arg(long)]
    target: String,
    /// Where the row's BOOTSEL drive is mounted
    #[arg(long)]
    bootsel_mount: Option<PathBuf>,
    /// Test the image the probe already holds
    #[arg(long)]
    no_put_flash: bool,
    /// Run only this case, in the order given; repeatable
    #[arg(long = "case", value_name = "NAME")]
    cases: Vec<String>,
    /// Passed to the harness's speed cases
    #[arg(long)]
    speed_extended: bool,
    /// Passed to the harness's speed cases; repeatable
    #[arg(long, value_name = "BYTES[,BYTES...]")]
    speed_access: Vec<String>,
}

impl HilArgs {
    /// What follows the harness's own arguments: its options, then the cases.
    fn harness_args(&self) -> Vec<String> {
        let mut args = Vec::new();
        if self.speed_extended {
            args.push("--speed-extended".to_string());
        }
        for access in &self.speed_access {
            args.extend(["--speed-access".to_string(), access.clone()]);
        }
        args.extend(self.cases.iter().cloned());
        args
    }
}

fn exit_code(ok: bool) -> ExitCode {
    if ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

fn build_verb(name: Option<&str>) -> ExitCode {
    let Some(name) = name else {
        let mut ok = true;
        for fw in registry::FIRMWARE.iter().filter(|f| f.name == f.path) {
            if !build(fw) {
                println!("{:<40} FAILED", fw.name);
                ok = false;
            }
        }
        return exit_code(ok);
    };
    let Some(fw) = find_firmware(name) else {
        unknown_firmware(name);
        return ExitCode::FAILURE;
    };
    exit_code(build(fw))
}

fn flash(args: &FlashArgs) -> ExitCode {
    let name = args.name.as_str();
    let Some(fw) = find_firmware(name) else {
        unknown_firmware(name);
        return ExitCode::FAILURE;
    };
    let put_sel = match put_selector(fw, args.put.as_deref()) {
        Ok(sel) => sel,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    let via = args.via.as_deref();
    let route = match route(fw, via, put_sel.as_ref(), args.bootsel_mount.as_deref()) {
        Ok(route) => route,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    exit_code(build(fw) && install(fw, route, put_sel.as_ref()))
}

fn hil(args: &HilArgs) -> ExitCode {
    let (name, target) = (args.name.as_str(), args.target.as_str());
    let Some(fw) = find_firmware(name) else {
        unknown_firmware(name);
        return ExitCode::FAILURE;
    };
    let Some(tgt) = registry::TARGETS.iter().find(|t| t.name == target) else {
        eprintln!("unknown target `{target}`; `cargo xtask list` names them");
        return ExitCode::FAILURE;
    };
    let Some(dut) = registry::FIRMWARE.iter().find(|f| f.name == tgt.dut) else {
        eprintln!("target `{target}` names an unregistered DUT `{}`", tgt.dut);
        return ExitCode::FAILURE;
    };
    let Some(profile) = fw.profile else {
        eprintln!("firmware `{name}` is a DUT image with no rig profile; it cannot be a PUT");
        return ExitCode::FAILURE;
    };
    let put_sel = match put_selector(fw, Some(args.put.as_str())) {
        Ok(sel) => sel,
        Err(e) => {
            eprintln!("cannot follow the PUT on the bus: {e}");
            return ExitCode::FAILURE;
        }
    };
    let via = args.via.as_deref();
    let route = if args.no_put_flash {
        None
    } else {
        match route(fw, via, put_sel.as_ref(), args.bootsel_mount.as_deref()) {
            Ok(route) => Some(route),
            Err(e) => {
                eprintln!("{e}");
                return ExitCode::FAILURE;
            }
        }
    };
    if !((route.is_none() || build(fw)) && build(dut)) {
        return ExitCode::FAILURE;
    }
    if let Some(route) = route {
        if !install(fw, route, put_sel.as_ref()) {
            return ExitCode::FAILURE;
        }
    }
    let dut_elf = root().join(dut.elf());
    let mut hil_args = vec![
        "run".to_string(),
        "--quiet".to_string(),
        // The speed cases time the host's probe-rs as much as the probe.
        "--release".to_string(),
        "--locked".to_string(),
        "--".to_string(),
        "--probe".to_string(),
        args.put.clone(),
        "--target".to_string(),
        tgt.name.to_string(),
        "--dut-elf".to_string(),
        dut_elf.to_str().unwrap_or_default().to_string(),
        "--profile".to_string(),
        profile.to_string(),
        "--firmware-row".to_string(),
        fw.name.to_string(),
    ];
    if dut.uf2_family.is_some() {
        let dut_uf2 = PathBuf::from(format!("{}.uf2", dut_elf.display()));
        hil_args.push("--dut-uf2".to_string());
        hil_args.push(dut_uf2.to_str().unwrap_or_default().to_string());
    }
    hil_args.extend(args.harness_args());
    let ok = Command::new("cargo")
        .args(hil_args)
        .current_dir(root().join("tests/daplink-test"))
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    exit_code(ok)
}

fn pins() -> ExitCode {
    let mismatched = dep_pins::check(&root());
    for group in dep_pins::GROUPS {
        for path in group.manifests {
            let manifest = root().join(path).join("Cargo.toml");
            let pin = dep_pins::pin(&manifest, group.dep);
            let rendered = match &pin {
                Ok(Some(identity)) => identity.to_string(),
                Ok(None) => "absent".to_string(),
                Err(error) => format!("ERROR: {error}"),
            };
            println!("{:<40} {:<12} {}", path, group.dep, rendered);
        }
    }
    for line in &mismatched {
        println!("MISMATCH: {line}");
    }
    exit_code(mismatched.is_empty())
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    if let Err(error) = tree::checked() {
        eprintln!("{error}");
        return ExitCode::FAILURE;
    }
    match cli.verb {
        Verb::List => list(&mut std::io::stdout().lock()),
        Verb::Build { name } => build_verb(name.as_deref()),
        Verb::BuildAll => exit_code(gate::build_all()),
        Verb::Matrix { link } => gate::matrix(link),
        Verb::Pins => pins(),
        Verb::Test { build } => gate::test(build),
        Verb::Flash(args) => flash(&args),
        Verb::Hil(args) => hil(&args),
    }
}

#[cfg(test)]
mod tests {
    use super::{list, write_columns, Cli, HilArgs, Verb};
    use clap::{CommandFactory, Parser};
    use std::io::{self, Write};
    use std::process::ExitCode;

    /// A reader that has gone away.
    struct Closed;

    impl Write for Closed {
        fn write(&mut self, _: &[u8]) -> io::Result<usize> {
            Err(io::ErrorKind::BrokenPipe.into())
        }

        fn flush(&mut self) -> io::Result<()> {
            Err(io::ErrorKind::BrokenPipe.into())
        }
    }

    #[test]
    fn list_ends_cleanly_when_its_reader_closes() {
        assert_eq!(list(&mut Closed), ExitCode::SUCCESS);
    }

    #[test]
    fn the_widest_cell_sets_its_column() {
        let rows = [
            ["a".into(), "bb".into(), "c".into()],
            ["aaaa".into(), "b".into(), String::new()],
        ];
        let mut out = Vec::new();
        write_columns(&mut out, &rows).unwrap();
        assert_eq!(String::from_utf8(out).unwrap(), "a    bb c\naaaa b\n");
    }

    fn parse(args: &[&str]) -> Result<Verb, clap::Error> {
        let argv = std::iter::once(&"cargo xtask").chain(args);
        Cli::try_parse_from(argv).map(|cli| cli.verb)
    }

    fn parse_hil(args: &[&str]) -> HilArgs {
        match parse(args).expect("valid options parse") {
            Verb::Hil(hil) => hil,
            _ => panic!("not hil"),
        }
    }

    #[test]
    fn the_command_definition_is_consistent() {
        Cli::command().debug_assert();
    }

    const HIL: [&str; 6] = ["hil", "fw", "--put", "P", "--target", "t"];

    /// Harness options come before the cases, which keep the order given.
    #[test]
    fn hil_forwards_speed_options_and_cases_to_the_harness() {
        let mut args = HIL.to_vec();
        args.extend([
            "--case",
            "connect",
            "--speed-access",
            "4,64",
            "--speed-extended",
            "--case",
            "vcom",
            "--speed-access",
            "508",
        ]);
        assert_eq!(
            parse_hil(&args).harness_args(),
            [
                "--speed-extended",
                "--speed-access",
                "4,64",
                "--speed-access",
                "508",
                "connect",
                "vcom",
            ]
        );
        assert!(parse_hil(&HIL).harness_args().is_empty());
    }

    #[test]
    fn hil_needs_the_put_and_the_target() {
        assert!(parse(&["hil", "fw", "--target", "t"]).is_err());
        assert!(parse(&["hil", "fw", "--put", "P"]).is_err());
        assert!(parse(&["hil", "--put", "P", "--target", "t"]).is_err());
    }

    #[test]
    fn no_put_flash_defaults_off() {
        assert!(!parse_hil(&HIL).no_put_flash);
        let mut args = HIL.to_vec();
        args.push("--no-put-flash");
        assert!(parse_hil(&args).no_put_flash);
    }

    /// `--via` and `--put` mean the same in both verbs; `--probe` keeps each verb's older meaning.
    #[test]
    fn both_verbs_name_the_transport_and_the_put_alike() {
        let Verb::Flash(flash) = parse(&["flash", "fw", "--probe", "T", "--put", "P"]).unwrap()
        else {
            panic!("not flash");
        };
        assert_eq!(flash.via.as_deref(), Some("T"));
        assert_eq!(flash.put.as_deref(), Some("P"));
        let hil = parse_hil(&["hil", "fw", "--via", "T", "--probe", "P", "--target", "t"]);
        assert_eq!(hil.via.as_deref(), Some("T"));
        assert_eq!(hil.put, "P");
    }

    /// A flag given twice, under either spelling, or missing its value, is a usage error.
    #[test]
    fn a_repeated_or_valueless_option_is_refused() {
        assert!(parse(&["flash", "fw", "--probe", "A", "--via", "B"]).is_err());
        assert!(parse(&["hil", "fw", "--target", "t", "--probe", "A", "--put", "B"]).is_err());
        assert!(parse(&["flash", "fw", "--put"]).is_err());
        let mut args = HIL.to_vec();
        args.extend(["--no-put-flash", "--no-put-flash"]);
        assert!(parse(&args).is_err());
        let mut args = HIL.to_vec();
        args.push("--case");
        assert!(parse(&args).is_err());
    }

    #[test]
    fn flash_refuses_hil_only_options() {
        for opt in [
            &["--target", "rp2040"][..],
            &["--speed-extended"],
            &["--speed-access", "4"],
            &["--case", "vcom"],
            &["--no-put-flash"],
        ] {
            let mut args = vec!["flash", "fw"];
            args.extend(opt);
            assert!(parse(&args).is_err(), "{opt:?}");
        }
    }

    #[test]
    fn a_verb_refuses_arguments_it_does_not_take() {
        assert!(parse(&[]).is_err());
        assert!(parse(&["list", "extra"]).is_err());
        assert!(parse(&["test", "--link"]).is_err());
        assert!(matches!(
            parse(&["matrix", "--link"]),
            Ok(Verb::Matrix { link: true })
        ));
        assert!(matches!(
            parse(&["test", "--build"]),
            Ok(Verb::Test { build: true })
        ));
    }
}
