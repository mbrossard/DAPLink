//! Build orchestration for the firmware fleet, run as `cargo xtask <verb>` from `rust/`.

mod bootsel;
// The scan runs only as a test; nothing in the built tool reaches it.
#[cfg(test)]
mod debug_assert;
mod elf;
mod matrix;
mod reattach;
mod registry;
mod revision;
mod usb;
mod workspaces;

use registry::{Bootloader, Firmware};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};
use std::time::Duration;
use workspaces::Workspace;

/// Resolve a registry row by name, accepting the bare basename as well as the full `firmware/...`
/// path. An ambiguous basename names its candidates rather than guessing.
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

/// Report an unresolvable row, suggesting rows whose basename contains the typed name.
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

/// The `rust/` tree root; registry paths resolve against it, not the invocation directory.
fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

fn build(fw: &Firmware) -> bool {
    println!("building {} for {}", fw.name, fw.target);
    let mut cmd = Command::new("cargo");
    cmd.args(["build", "--release", "--target", fw.target]);
    // Absolute: the build runs in the crate directory, and output belongs under `rust/target/`.
    cmd.arg("--target-dir").arg(root().join(fw.target_dir()));
    if fw.no_default_features {
        cmd.arg("--no-default-features");
    }
    if !fw.features.is_empty() {
        cmd.args(["--features", &fw.features.join(",")]);
    }
    revision::stamp(&mut cmd, &root());
    let ok = cmd
        .current_dir(root().join(fw.path))
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    ok && emit_uf2(fw)
}

/// For a row that names a uf2 family, write the uf2-encoded flash image beside the built ELF;
/// a no-op for every other row.
fn emit_uf2(fw: &Firmware) -> bool {
    let Some(family) = fw.uf2_family else {
        return true;
    };
    let elf_path = root().join(fw.elf());
    let elf_bytes = match std::fs::read(&elf_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{}: could not read {}: {e}", fw.name, elf_path.display());
            return false;
        }
    };
    let (base, image) = match elf::flash_image(&elf_bytes) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("{}: could not extract flash image: {e}", fw.name);
            return false;
        }
    };
    let uf2 = daplink_decoder::encode::bin_to_uf2(&image, base, family);
    let uf2_path = PathBuf::from(format!("{}.uf2", elf_path.display()));
    if let Err(e) = std::fs::write(&uf2_path, &uf2) {
        eprintln!("{}: could not write {}: {e}", fw.name, uf2_path.display());
        return false;
    }
    println!(
        "wrote {} ({} bytes, {} blocks)",
        uf2_path.display(),
        uf2.len(),
        uf2.len() / 512
    );
    true
}

/// Compile one generated feature shape for its row's target. `check` is the default: what the
/// matrix asks is whether the combination compiles, and every shape of a crate shares one
/// directory, so the cost is one dependency graph per crate rather than one per shape. `link`
/// runs the full build instead, which is the only way a section overflow or a symbol collision
/// in a shape nobody flashes is reached.
fn check_shape(shape: &matrix::Shape, link: bool) -> bool {
    let fw = shape.base;
    println!("checking {} for {}", shape.label(), fw.target);
    let mut cmd = Command::new("cargo");
    cmd.args([if link { "build" } else { "check" }, "--release"]);
    cmd.args(["--target", fw.target]);
    cmd.arg("--target-dir")
        .arg(root().join("target/matrix").join(fw.path));
    if shape.no_default_features {
        cmd.arg("--no-default-features");
    }
    if !shape.features.is_empty() {
        cmd.args(["--features", &shape.features.join(",")]);
    }
    revision::stamp(&mut cmd, &root());
    cmd.current_dir(root().join(fw.path))
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Build every registry row, then compile-check every shape the matrix generates on top of them.
/// Returns whether all of it passed; a failure does not stop the rest.
fn build_all() -> bool {
    let mut ok = true;
    for fw in registry::FIRMWARE {
        let built = build(fw);
        ok &= built;
        println!("{:<40} {}", fw.name, if built { "ok" } else { "FAILED" });
    }
    for shape in matrix::shapes(&root()) {
        let checked = check_shape(&shape, false);
        ok &= checked;
        println!(
            "{:<40} {}",
            shape.label(),
            if checked { "ok" } else { "FAILED" }
        );
    }
    ok
}

/// Allow-by-default lint every row denies on top of `-D warnings`: a side-effecting call inside
/// `debug_assert!` is dropped entirely in release, so the statement ships as dead code.
const EXTRA_LINTS: &str = "clippy::debug_assert_with_mut_call";

/// Run one check (`fmt`, `clippy`, or `test`) for a workspace row, in its own directory. Prints
/// a single status line and returns whether it passed.
fn check(ws: &Workspace, label: &str, mut cmd: Command) -> bool {
    // One directory for all rows: they run sequentially, so nothing contends, and they share one
    // dependency graph instead of building it per row. Set through the environment rather than
    // `--target-dir`, which `cargo fmt` rejects outright and which lands after the `--` of a
    // clippy row and reaches rustc instead of cargo. An explicit `CARGO_TARGET_DIR` still wins.
    if std::env::var_os("CARGO_TARGET_DIR").is_none() {
        cmd.env("CARGO_TARGET_DIR", root().join("target/workspace"));
    }
    let ok = cmd
        .current_dir(root().join(ws.path))
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    println!(
        "{:<24} {:<8} {}",
        ws.name,
        label,
        if ok { "ok" } else { "FAILED" }
    );
    ok
}

/// The triple `cargo` builds for by default here, from `rustc -vV`.
fn host_triple() -> Option<String> {
    let out = Command::new("rustc").arg("-vV").output().ok()?;
    String::from_utf8(out.stdout)
        .ok()?
        .lines()
        .find_map(|l| l.strip_prefix("host: "))
        .map(str::to_string)
}

/// The `--target` a host row needs. A crate that pins an embedded target in its own
/// `.cargo/config.toml` would otherwise cross-compile the test harness instead of running it, and
/// only an explicit `--target` on the invocation outranks that pin.
fn host_target(ws: &Workspace) -> Option<String> {
    let cfg = std::fs::read_to_string(root().join(ws.path).join(".cargo/config.toml")).ok()?;
    if !cfg.lines().any(|l| l.starts_with("target = ")) {
        return None;
    }
    host_triple()
}

/// `fmt --check`, `clippy -D warnings`, then `test` for one row, returning the labels that
/// failed; a failing check does not stop the others.
fn test_workspace(ws: &Workspace) -> Vec<&'static str> {
    let mut failed = Vec::new();
    let host = host_target(ws);

    let mut fmt = Command::new("cargo");
    fmt.args(["fmt", "--check"]);
    if !check(ws, "fmt", fmt) {
        failed.push("fmt");
    }

    let mut clippy = Command::new("cargo");
    clippy.args(["clippy", "--all-targets"]);
    if !ws.features.is_empty() {
        clippy.args(["--features", &ws.features.join(",")]);
    }
    if let Some(target) = &host {
        clippy.args(["--target", target]);
    }
    clippy.args(["--", "-W", EXTRA_LINTS, "-D", "warnings"]);
    if !check(ws, "clippy", clippy) {
        failed.push("clippy");
    }

    let mut test = Command::new("cargo");
    test.arg("test");
    if !ws.features.is_empty() {
        test.args(["--features", &ws.features.join(",")]);
    }
    if let Some(target) = &host {
        test.args(["--target", target]);
    }
    if !check(ws, "test", test) {
        failed.push("test");
    }

    failed
}

/// `fmt --check` then `clippy -D warnings` for an embedded row, built for the crate's own pinned
/// target; there is no `test` crate there, so test targets are neither linted nor run.
fn lint_workspace(ws: &Workspace) -> Vec<&'static str> {
    let mut failed = Vec::new();

    let mut fmt = Command::new("cargo");
    fmt.args(["fmt", "--check"]);
    if !check(ws, "fmt", fmt) {
        failed.push("fmt");
    }

    let mut clippy = Command::new("cargo");
    clippy.arg("clippy");
    if !ws.features.is_empty() {
        clippy.args(["--features", &ws.features.join(",")]);
    }
    clippy.args(["--", "-W", EXTRA_LINTS, "-D", "warnings"]);
    revision::stamp(&mut clippy, &root());
    if !check(ws, "clippy", clippy) {
        failed.push("clippy");
    }

    failed
}

fn probe_rs(args: &[&str]) -> bool {
    Command::new("probe-rs")
        .args(args)
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn probe_rs_owned(args: &[String]) -> bool {
    Command::new("probe-rs")
        .args(args)
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// 32-bit words per `probe-rs write` invocation in the [`zero_via_write_loop`] fallback --
/// enough to make real progress without stretching the command line unreasonably.
const ZERO_CHUNK_WORDS: u32 = 64;

/// Zero `size` bytes at `addr` through a bounded loop of `probe-rs write` calls (RAM-only, but
/// works on a target whose `download` command refuses a `--base-address` outside flash).
fn zero_via_write_loop(fw: &Firmware, probe: &str, addr: u32, size: u32) -> bool {
    let base = ["write", "b32", "--chip", fw.chip, "--probe", probe].map(String::from);
    let whole_words = size / 4;
    let mut word = 0;
    while word < whole_words {
        let n = ZERO_CHUNK_WORDS.min(whole_words - word);
        let mut args = base.to_vec();
        args.push(format!("{:#x}", addr + word * 4));
        args.extend(std::iter::repeat_n("0".to_string(), n as usize));
        if !probe_rs_owned(&args) {
            return false;
        }
        word += n;
    }
    let tail = size % 4;
    if tail > 0 {
        let mut args: Vec<String> = ["write", "b8", "--chip", fw.chip, "--probe", probe]
            .map(String::from)
            .to_vec();
        args.push(format!("{:#x}", addr + whole_words * 4));
        args.extend(std::iter::repeat_n("0".to_string(), tail as usize));
        if !probe_rs_owned(&args) {
            return false;
        }
    }
    true
}

/// Zero the built ELF's `.uninit` region, so a reset-surviving RAM region boots clean; an absent
/// section is fine, and a target refusing a RAM `--base-address` falls back to a write loop.
fn zero_uninit(fw: &Firmware, elf: &Path, probe: &str) -> bool {
    let bytes = match std::fs::read(elf) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{}: could not read {}: {e}", fw.name, elf.display());
            return false;
        }
    };
    let (addr, size) = match elf::uninit_range(&bytes) {
        Ok(Some(range)) => range,
        Ok(None) => return true,
        Err(e) => {
            eprintln!("{}: could not read .uninit section: {e}", fw.name);
            return false;
        }
    };
    println!("zeroing .uninit @{addr:#010x}+{size:#x}");

    let zeros_path =
        std::env::temp_dir().join(format!("xtask-uninit-zero-{}.bin", std::process::id()));
    if let Err(e) = std::fs::write(&zeros_path, vec![0u8; size as usize]) {
        eprintln!("{}: could not write zero-fill scratch file: {e}", fw.name);
        return false;
    }
    let downloaded = probe_rs(&[
        "download",
        "--binary-format",
        "bin",
        "--base-address",
        &format!("{addr:#x}"),
        "--chip",
        fw.chip,
        "--probe",
        probe,
        zeros_path.to_str().unwrap_or_default(),
    ]);
    let _ = std::fs::remove_file(&zeros_path);

    downloaded || zero_via_write_loop(fw, probe, addr, size)
}

/// Flash a built row over the transport `probe` selects, zero its `.uninit` RAM, then reset it;
/// `--preverify` skips flashing content already byte-identical.
fn flash(fw: &Firmware, probe: &str) -> bool {
    let elf = root().join(fw.elf());
    println!("flashing {} ({})", fw.name, fw.chip);
    probe_rs(&[
        "download",
        "--preverify",
        "--chip",
        fw.chip,
        "--probe",
        probe,
        elf.to_str().unwrap_or_default(),
    ]) && zero_uninit(fw, &elf, probe)
        && probe_rs(&["reset", "--chip", fw.chip, "--probe", probe])
}

/// Flash a built row by its `bootloader` disposition, then leave the PUT enumerated under its
/// own identity: `via` is the transport, `put` the flashed board's own USB selector. A DUT image
/// has no USB role of its own and passes `None`.
fn install(
    fw: &Firmware,
    via: Option<&str>,
    put: Option<&usb::Selector>,
    bootsel_mount: Option<&Path>,
) -> bool {
    let flashed = match fw.bootloader {
        Bootloader::Probe => flash(fw, via.expect("a probe row without --via is rejected")),
        Bootloader::BootselUf2 => {
            let elf = root().join(fw.elf());
            let uf2 = PathBuf::from(format!("{}.uf2", elf.display()));
            bootsel::flash_bootsel(
                &uf2,
                put.expect("a bootsel row without --probe is rejected"),
                bootsel_mount,
            )
        }
    };
    if !flashed {
        return false;
    }
    let Some(put) = put else {
        return true;
    };
    // Without a re-attach the host keeps the pre-flash descriptors, and an interface the new
    // image gained would be absent -- which a capability case reads as firmware that does not
    // compose it.
    if fw.bootloader == Bootloader::Probe
        && reattach::applies(fw.chip)
        && !reattach::run(fw, via.expect("a probe row without --via is rejected"), put)
    {
        return false;
    }
    wait_for_put(put)
}

/// The flashed board's own USB identity, for the rows that have one. A DUT image is not on the
/// bus at all, so `None` there is right; a PUT without one cannot be followed across the reflash
/// and is refused rather than slept through.
/// `flag` differs per verb: `flash` names the board `--put` and keeps `--probe` for the
/// transport, as probe-rs does; `hil` has always named the board `--probe`.
fn put_selector(
    fw: &Firmware,
    spec: Option<&str>,
    flag: &str,
) -> Result<Option<usb::Selector>, String> {
    match (spec, fw.profile) {
        (Some(spec), _) => usb::parse_selector(spec).map(Some),
        (None, None) => Ok(None),
        (None, Some(_)) => Err(format!(
            "firmware `{}` is a probe under test; pass {flag} <vid:pid:serial> so the reflash \
             can be followed on the bus",
            fw.name
        )),
    }
}

/// How long the reflashed PUT must hold its place on the bus before the harness may open it.
const PUT_SETTLE: Duration = Duration::from_millis(750);
/// How long to wait for it to get there. The enumeration delay is the host's and varies.
const PUT_REENUM_TIMEOUT: Duration = Duration::from_secs(20);

/// Wait for the reflashed PUT to be back on the bus under its own serial. A fixed sleep either
/// runs long or loses the race, and losing it reports a probe that is merely late as absent.
fn wait_for_put(sel: &usb::Selector) -> bool {
    println!("waiting for the PUT to settle on the bus: {sel}");
    if usb::wait_settled(sel, PUT_SETTLE, PUT_REENUM_TIMEOUT) {
        return true;
    }
    eprintln!("PUT {sel} did not settle on the bus within {PUT_REENUM_TIMEOUT:?}");
    false
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("list") => {
            for fw in registry::FIRMWARE {
                println!("{:<40} {:<21} {}", fw.name, fw.target, fw.chip);
            }
            for shape in matrix::shapes(&root()) {
                println!(
                    "shape  {:<33} {:<21} {}",
                    shape.label(),
                    shape.base.target,
                    shape.features.join(",")
                );
            }
            for t in registry::TARGETS {
                let dut_chip = registry::FIRMWARE
                    .iter()
                    .find(|f| f.name == t.dut)
                    .map(|f| f.chip)
                    .unwrap_or("?");
                println!("target {:<33} {:<21} {}", t.name, dut_chip, t.dut);
            }
            for ws in workspaces::WORKSPACES {
                let features = ws.features.join(",");
                println!("workspace {:<27} {:<21} {}", ws.name, ws.path, features);
            }
            for ws in workspaces::EMBEDDED {
                println!("embedded  {:<27} {}", ws.name, ws.path);
            }
            for ex in workspaces::EXCLUDED {
                println!("excluded  {:<27} {}", ex.path, ex.reason);
            }
            ExitCode::SUCCESS
        }
        Some("build") => {
            let Some(name) = args.get(1) else {
                let mut ok = true;
                for fw in registry::FIRMWARE.iter().filter(|f| f.name == f.path) {
                    if !build(fw) {
                        println!("{:<40} FAILED", fw.name);
                        ok = false;
                    }
                }
                return if ok {
                    ExitCode::SUCCESS
                } else {
                    ExitCode::FAILURE
                };
            };
            let Some(fw) = find_firmware(name) else {
                unknown_firmware(name);
                return ExitCode::FAILURE;
            };
            if build(fw) {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Some("flash") => {
            let usage = || {
                eprintln!(
                    "usage: cargo xtask flash <name> [--probe <transport-sel>] \
                     [--put <vid:pid:serial>] [--bootsel-mount <path>]"
                );
                ExitCode::FAILURE
            };
            let Some(name) = args.get(1) else {
                return usage();
            };
            let (mut via, mut put, mut bootsel_mount) = (None, None, None);
            let mut it = args[2..].iter();
            while let Some(a) = it.next() {
                match a.as_str() {
                    // The transport, as `--probe` means for probe-rs, which this verb wraps.
                    "--probe" => via = it.next(),
                    "--put" => put = it.next(),
                    "--bootsel-mount" => bootsel_mount = it.next(),
                    _ => return usage(),
                }
            }
            let Some(fw) = find_firmware(name) else {
                unknown_firmware(name);
                return ExitCode::FAILURE;
            };
            if fw.bootloader == Bootloader::Probe && via.is_none() {
                eprintln!("firmware `{name}` flashes over a probe transport; pass --probe <sel>");
                return usage();
            }
            let put_sel = match put_selector(fw, put.map(String::as_str), "--put") {
                Ok(sel) => sel,
                Err(e) => {
                    eprintln!("{e}");
                    return ExitCode::FAILURE;
                }
            };
            let mount = bootsel_mount.map(PathBuf::from);
            if build(fw)
                && install(
                    fw,
                    via.map(String::as_str),
                    put_sel.as_ref(),
                    mount.as_deref(),
                )
            {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Some("hil") => {
            let usage = || {
                eprintln!(
                    "usage: cargo xtask hil <name> [--via <transport-sel>] --probe <put-sel> \
                     --target <name> [--bootsel-mount <path>]"
                );
                ExitCode::FAILURE
            };
            let Some(name) = args.get(1) else {
                return usage();
            };
            let (mut via, mut put, mut target, mut bootsel_mount) = (None, None, None, None);
            let mut it = args[2..].iter();
            while let Some(a) = it.next() {
                match a.as_str() {
                    "--via" => via = it.next(),
                    "--probe" => put = it.next(),
                    "--target" => target = it.next(),
                    "--bootsel-mount" => bootsel_mount = it.next(),
                    _ => return usage(),
                }
            }
            let (Some(put), Some(target)) = (put, target) else {
                return usage();
            };
            let Some(fw) = find_firmware(name) else {
                unknown_firmware(name);
                return ExitCode::FAILURE;
            };
            if fw.bootloader == Bootloader::Probe && via.is_none() {
                eprintln!("firmware `{name}` flashes over a probe transport; pass --via <sel>");
                return usage();
            }
            let Some(tgt) = registry::TARGETS.iter().find(|t| t.name == target) else {
                eprintln!("unknown target `{target}`; `cargo xtask list` names them");
                return ExitCode::FAILURE;
            };
            let Some(dut) = registry::FIRMWARE.iter().find(|f| f.name == tgt.dut) else {
                eprintln!("target `{target}` names an unregistered DUT `{}`", tgt.dut);
                return ExitCode::FAILURE;
            };
            let Some(profile) = fw.profile else {
                eprintln!(
                    "firmware `{name}` is a DUT image with no rig profile; it cannot be a PUT"
                );
                return ExitCode::FAILURE;
            };
            let put_sel = match put_selector(fw, Some(put.as_str()), "--probe") {
                Ok(sel) => sel,
                Err(e) => {
                    eprintln!("cannot follow the PUT on the bus: {e}");
                    return ExitCode::FAILURE;
                }
            };
            if !(build(fw) && build(dut)) {
                return ExitCode::FAILURE;
            }
            let mount = bootsel_mount.map(PathBuf::from);
            if !install(
                fw,
                via.map(String::as_str),
                put_sel.as_ref(),
                mount.as_deref(),
            ) {
                return ExitCode::FAILURE;
            }
            let dut_elf = root().join(dut.elf());
            let mut hil_args = vec![
                "run".to_string(),
                "--quiet".to_string(),
                "--".to_string(),
                "--probe".to_string(),
                put.clone(),
                "--target".to_string(),
                tgt.name.to_string(),
                "--dut-elf".to_string(),
                dut_elf.to_str().unwrap_or_default().to_string(),
                "--profile".to_string(),
                profile.to_string(),
            ];
            if dut.uf2_family.is_some() {
                let dut_uf2 = PathBuf::from(format!("{}.uf2", dut_elf.display()));
                hil_args.push("--dut-uf2".to_string());
                hil_args.push(dut_uf2.to_str().unwrap_or_default().to_string());
            }
            let ok = Command::new("cargo")
                .args(hil_args)
                .current_dir(root().join("tests/daplink-test"))
                .status()
                .map(|s| s.success())
                .unwrap_or(false);
            if ok {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Some("build-all") => {
            if build_all() {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Some("matrix") => {
            let link = args.get(1).map(String::as_str) == Some("--link");
            let mut ok = true;
            for shape in matrix::shapes(&root()) {
                let checked = check_shape(&shape, link);
                ok &= checked;
                println!(
                    "{:<40} {}",
                    shape.label(),
                    if checked { "ok" } else { "FAILED" }
                );
            }
            for i in matrix::INCOMPATIBLE {
                println!(
                    "{:<40} skipped: {}",
                    format!("{} +{}", i.path, i.added.join(",")),
                    i.reason
                );
            }
            for u in matrix::UNREACHED {
                println!(
                    "{:<40} unreached: {}",
                    format!("{} {}", u.path, u.feature),
                    u.reason
                );
            }
            for (path, feat) in matrix::unreached(&root()) {
                println!("{:<40} compiled by nothing", format!("{path} {feat}"));
                ok = false;
            }
            if ok {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Some("test") => {
            let with_build = args.get(1).map(String::as_str) == Some("--build");
            let mut failures: Vec<(&str, &str)> = Vec::new();
            for ws in workspaces::WORKSPACES {
                for label in test_workspace(ws) {
                    failures.push((ws.name, label));
                }
            }
            for ws in workspaces::EMBEDDED {
                for label in lint_workspace(ws) {
                    failures.push((ws.name, label));
                }
            }
            let mut build_failed = false;
            if with_build {
                println!("--- build-all ---");
                build_failed = !build_all();
            }
            println!("--- summary ---");
            if failures.is_empty() && !build_failed {
                println!(
                    "all {} workspaces clean",
                    workspaces::WORKSPACES.len() + workspaces::EMBEDDED.len()
                );
                ExitCode::SUCCESS
            } else {
                for (name, label) in &failures {
                    println!("FAILED: {name} {label}");
                }
                if build_failed {
                    println!("FAILED: build-all");
                }
                ExitCode::FAILURE
            }
        }
        _ => {
            eprintln!(
                "usage: cargo xtask <list | build [<name>] | build-all | \
                 matrix [--link] | test [--build] | \
                 flash <name> --probe <sel> [--put <sel>] | \
                 hil <name> --via <sel> --probe <sel> --target <name>>"
            );
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{find_firmware, put_selector};
    use crate::registry;
    use crate::workspaces;
    use daplink_decoder::{detect_format, Decoder, DecoderContext, Error, Format, Uf2Decoder};
    use std::path::Path;

    /// Crate directories under `dir` (relative to `rust/`), skipping build-artifact trees.
    fn crate_dirs(dir: &Path, rel: &str, out: &mut Vec<String>) {
        for entry in std::fs::read_dir(dir).unwrap() {
            let entry = entry.unwrap();
            let name = entry.file_name().to_string_lossy().into_owned();
            let path = entry.path();
            if path.is_dir() {
                if name == "target" || name.starts_with('.') {
                    continue;
                }
                let rel = if rel.is_empty() {
                    name
                } else {
                    format!("{rel}/{name}")
                };
                crate_dirs(&path, &rel, out);
            } else if name == "Cargo.toml" {
                out.push(if rel.is_empty() {
                    ".".to_string()
                } else {
                    rel.to_string()
                });
            }
        }
    }

    /// Whether a registry entry (exact path, or a `<prefix>/*` glob) names this crate directory.
    fn covers(entry: &str, dir: &str) -> bool {
        match entry.strip_suffix("/*") {
            Some(prefix) => dir.strip_prefix(prefix).is_some_and(|r| r.starts_with('/')),
            None => entry == dir,
        }
    }

    /// Every crate in the tree must hold a place in exactly one of the gate's lists, so a new
    /// directory cannot land unreached by every check without failing here.
    #[test]
    fn every_crate_is_named_by_the_gate_registry() {
        let mut dirs = Vec::new();
        crate_dirs(&crate::root(), "", &mut dirs);
        dirs.sort();
        assert!(dirs.len() > 30, "the walk found too few crates: {dirs:?}");
        let entries: Vec<&str> = workspaces::WORKSPACES
            .iter()
            .chain(workspaces::EMBEDDED)
            .map(|ws| ws.path)
            .chain(workspaces::EXCLUDED.iter().map(|ex| ex.path))
            .collect();
        for dir in &dirs {
            // `xtask` is the root workspace's only member and is gated through its row.
            let dir = if dir == "xtask" { "." } else { dir };
            let named: Vec<&&str> = entries.iter().filter(|e| covers(e, dir)).collect();
            assert_eq!(
                named.len(),
                1,
                "`{dir}` must be named by exactly one gate list, found {named:?}"
            );
        }
        for entry in &entries {
            let listed = entry.strip_suffix("/*").unwrap_or(entry);
            assert!(
                dirs.iter()
                    .any(|d| covers(entry, d) || listed == d || *entry == "."),
                "`{entry}` names no crate in the tree"
            );
        }
    }

    /// Reassembles decoded fragments into a flat image starting at `base`, so the test can
    /// check the payload landed back exactly where and as it started.
    struct Rebuild {
        base: u32,
        image: Vec<u8>,
        family: Option<u32>,
    }

    impl DecoderContext for Rebuild {
        fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
            let off = (addr - self.base) as usize;
            if self.image.len() < off + data.len() {
                self.image.resize(off + data.len(), 0);
            }
            self.image[off..off + data.len()].copy_from_slice(data);
            Ok(())
        }

        fn set_uf2_family(&mut self, family: Option<u32>) {
            self.family = family;
        }
    }

    /// What `emit_uf2` writes must detect as uf2 and decode back to the exact image, base and
    /// family id, which is what a flash case checks against the real files.
    #[test]
    fn uf2_round_trips_through_daplink_decoder() {
        let base = 0x1000_0000u32;
        let bin: Vec<u8> = (0..2500u32).map(|i| (i * 31) as u8).collect();
        let family = registry::UF2_FAMILY_RP2040;

        let uf2 = daplink_decoder::encode::bin_to_uf2(&bin, base, family);
        assert_eq!(uf2.len() % 512, 0);
        assert_eq!(uf2.len() / 512, bin.len().div_ceil(256));

        assert_eq!(detect_format(None, &uf2), Format::Uf2);
        assert_eq!(detect_format(Some("test-rp2040.uf2"), &[]), Format::Uf2);

        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rebuild {
            base,
            image: Vec::new(),
            family: None,
        };
        dec.input(&uf2, &mut sp, &mut ctx).unwrap();
        dec.finish(&mut sp, &mut ctx).unwrap();

        assert_eq!(ctx.family, Some(family));
        assert!(ctx.image.len() >= bin.len());
        assert_eq!(&ctx.image[..bin.len()], &bin[..]);
    }

    /// Two rows sharing a build directory would evict each other's output every time the gate
    /// alternated between them.
    #[test]
    fn every_row_builds_in_its_own_directory() {
        let mut seen = std::collections::BTreeSet::new();
        for fw in registry::FIRMWARE {
            assert!(
                seen.insert(fw.target_dir()),
                "{} shares a target dir",
                fw.name
            );
        }
    }

    /// A PUT reflash that cannot be followed on the bus is refused, not slept through; a DUT
    /// image has no USB identity to follow in the first place.
    #[test]
    fn only_a_put_row_demands_its_own_selector() {
        let put = find_firmware("firmware/rp2040-pico-rtic").expect("a registered PUT row");
        let dut = find_firmware("tests/firmwares/test-rp2040").expect("a registered DUT row");
        assert!(put_selector(put, None, "--put").is_err());
        assert!(put_selector(dut, None, "--put")
            .expect("a DUT needs none")
            .is_none());
        assert!(put_selector(put, Some("0d28:0204:SERIAL"), "--put")
            .expect("a full selector parses")
            .is_some());
        // probe-rs accepts a serial-less selector; the bus wait cannot use one.
        assert!(put_selector(put, Some("0d28:0204"), "--put").is_err());
    }
}
