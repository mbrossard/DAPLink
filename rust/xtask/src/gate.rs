//! The host gate and the firmware sweep: workspace checks, feature shapes, and build-all.

use std::path::Path;
use std::process::{Command, ExitCode};

use crate::install::build;
use crate::workspaces::{self, Workspace};
use crate::{dep_pins, matrix, registry, revision, root, stack};

/// `test`: every workspace row, embedded lint and cross row, then with `with_build` the sweep.
pub(crate) fn test(with_build: bool) -> ExitCode {
    // First and fatal: the unlocked rows below would rewrite a stale lock and hide it.
    let stale = stale_locks();
    if !stale.is_empty() {
        for line in &stale {
            println!("FAILED: lockfile: {line}");
        }
        return ExitCode::FAILURE;
    }
    let mut failures: Vec<(&str, &str)> = Vec::new();
    let mismatched = dep_pins::check(&root());
    for ws in workspaces::WORKSPACES {
        for label in test_workspace(ws) {
            failures.push((ws.name, label));
        }
    }
    for ws in workspaces::embedded() {
        for label in lint_workspace(&ws) {
            failures.push((ws.name, label));
        }
    }
    for c in workspaces::CROSS {
        for label in lint_cross(c) {
            failures.push((c.name, label));
        }
    }
    let mut build_failed = false;
    if with_build {
        println!("--- build-all ---");
        build_failed = !build_all();
    }
    println!("--- summary ---");
    for line in &mismatched {
        println!("FAILED: dependency pins: {line}");
    }
    if failures.is_empty() && !build_failed && mismatched.is_empty() {
        println!(
            "all {} rows clean",
            workspaces::WORKSPACES.len() + registry::FAMILIES.len() + workspaces::CROSS.len()
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

/// Directories holding a tracked `Cargo.lock`, relative to `rust/`.
fn tracked_lock_dirs() -> Result<Vec<String>, String> {
    let out = Command::new("git")
        .args(["ls-files", "--", "*Cargo.lock"])
        .current_dir(root())
        .output()
        .map_err(|e| format!("git ls-files: {e}"))?;
    if !out.status.success() {
        return Err("git ls-files failed".to_string());
    }
    let listing = String::from_utf8_lossy(&out.stdout);
    Ok(listing
        .lines()
        .map(|l| l.strip_suffix("Cargo.lock").unwrap_or(l))
        .map(|d| d.trim_end_matches('/').to_string())
        .collect())
}

/// Whether cargo accepts `dir`'s lockfile unchanged.
fn lock_is_current(dir: &Path) -> bool {
    Command::new("cargo")
        .args(["metadata", "--locked", "--format-version", "1"])
        .current_dir(dir)
        .output()
        .is_ok_and(|out| out.status.success())
}

/// Every tracked lockfile cargo would have to change.
fn stale_locks() -> Vec<String> {
    match tracked_lock_dirs() {
        Ok(dirs) => dirs
            .into_iter()
            .filter(|dir| !lock_is_current(&root().join(dir)))
            .map(|dir| format!("`{dir}/Cargo.lock` does not match its manifests"))
            .collect(),
        Err(error) => vec![error],
    }
}

/// `matrix`: lint, or with `link` build, every generated shape, then the incompatible pairs.
pub(crate) fn matrix(link: bool) -> ExitCode {
    let mut ok = true;
    let shapes = match matrix::shapes(&root()) {
        Ok(shapes) => shapes,
        Err(error) => {
            eprintln!("could not read feature matrix: {error}");
            return ExitCode::FAILURE;
        }
    };
    for shape in shapes {
        let checked = check_shape(&shape, link);
        ok &= checked;
        println!(
            "{:<40} {}",
            shape.label(),
            if checked { "ok" } else { "FAILED" }
        );
    }
    for i in matrix::INCOMPATIBLE {
        ok &= check_incompatible(i);
    }
    if ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// Lint (or with `link`, build) one generated feature shape for its row's target.
fn check_shape(shape: &matrix::Shape, link: bool) -> bool {
    let fw = shape.base;
    println!("checking {} for {}", shape.label(), fw.target);
    let mut cmd = Command::new("cargo");
    cmd.args([
        if link { "build" } else { "clippy" },
        "--release",
        "--locked",
    ]);
    cmd.args(["--target", fw.target]);
    cmd.arg("--target-dir")
        .arg(root().join("target/matrix").join(fw.path));
    if shape.no_default_features {
        cmd.arg("--no-default-features");
    }
    if !shape.features.is_empty() {
        cmd.args(["--features", &shape.features.join(",")]);
    }
    if !link {
        cmd.args(["--", "-W", EXTRA_LINTS, "-D", "warnings"]);
    }
    revision::stamp(&mut cmd, &root());
    cmd.current_dir(root().join(fw.path))
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Whether an incompatible combination still fails to compile, and with its own guard.
fn check_incompatible(i: &matrix::Incompatible) -> bool {
    let label = format!("{} +{}", i.path, i.added.join(","));
    let Some(fw) = registry::FIRMWARE.iter().find(|f| f.path == i.path) else {
        println!("{label:<40} FAILED: no registry row");
        return false;
    };
    let output = Command::new("cargo")
        .args(["check", "--release", "--locked", "--target", fw.target])
        .arg("--target-dir")
        .arg(root().join("target/matrix").join(fw.path))
        .args(["--features", &i.added.join(",")])
        .current_dir(root().join(fw.path))
        .output();
    let stderr = output
        .as_ref()
        .map(|out| String::from_utf8_lossy(&out.stderr).into_owned())
        .unwrap_or_default();
    let rejected =
        stderr.contains(i.reason) && output.as_ref().is_ok_and(|out| !out.status.success());
    let status = if rejected {
        "rejected"
    } else {
        eprintln!("{stderr}");
        "FAILED: guard did not fire"
    };
    println!("{label:<40} {status}");
    rejected
}

/// Build every registry row and check its stack floor, then every matrix shape; one failure does not stop the rest.
pub(crate) fn build_all() -> bool {
    let shapes = match matrix::shapes(&root()) {
        Ok(shapes) => shapes,
        Err(error) => {
            eprintln!("FAILED: feature matrix: {error}");
            return false;
        }
    };
    let mut ok = true;
    for fw in registry::FIRMWARE {
        let row = matrix::Shape {
            base: fw,
            features: fw.features.iter().map(|f| f.to_string()).collect(),
            added: Vec::new(),
            no_default_features: fw.no_default_features,
        };
        let built = build(fw) && check_shape(&row, false) && stack::check(fw);
        ok &= built;
        println!("{:<40} {}", fw.name, if built { "ok" } else { "FAILED" });
    }
    for shape in shapes {
        let checked = check_shape(&shape, false);
        ok &= checked;
        println!(
            "{:<40} {}",
            shape.label(),
            if checked { "ok" } else { "FAILED" }
        );
    }
    for i in matrix::INCOMPATIBLE {
        ok &= check_incompatible(i);
    }
    ok
}

/// A side-effecting call inside `debug_assert!` vanishes in release.
const EXTRA_LINTS: &str = "clippy::debug_assert_with_mut_call";

/// Run one check for a workspace row in its own directory, printing a status line.
fn check(ws: &Workspace, label: &str, mut cmd: Command) -> bool {
    // Shared target dir via the environment: `cargo fmt` rejects `--target-dir`.
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

fn host_triple() -> Option<String> {
    let out = Command::new("rustc").arg("-vV").output().ok()?;
    String::from_utf8(out.stdout)
        .ok()?
        .lines()
        .find_map(|l| l.strip_prefix("host: "))
        .map(str::to_string)
}

/// Explicit host `--target`, which outranks a crate's pinned embedded target.
fn host_target(ws: &Workspace) -> Option<String> {
    let cfg = std::fs::read_to_string(root().join(ws.path).join(".cargo/config.toml")).ok()?;
    if !cfg.lines().any(|l| l.starts_with("target = ")) {
        return None;
    }
    host_triple()
}

/// `fmt --check`, `clippy`, `doc`, then `test` for one row; returns the failures.
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
    feature_args(&mut clippy, ws);
    if let Some(target) = &host {
        clippy.args(["--target", target]);
    }
    clippy.args(["--", "-W", EXTRA_LINTS, "-D", "warnings"]);
    if !check(ws, "clippy", clippy) {
        failed.push("clippy");
    }

    let chips = workspaces::DOC_CHIPS
        .iter()
        .filter(|(path, _)| *path == ws.path);
    let features: Vec<&str> = ws
        .features
        .iter()
        .copied()
        .chain(chips.map(|c| c.1))
        .collect();
    let doc = workspaces::doc_filter(ws).map(|f| {
        let mut doc = host_doc(f, host.as_deref(), &features);
        if ws.no_default_features {
            doc.arg("--no-default-features");
        }
        doc
    });
    if !doc.is_some_and(|doc| check(ws, "doc", doc)) {
        failed.push("doc");
    }

    let mut test = Command::new("cargo");
    test.arg("test");
    feature_args(&mut test, ws);
    if let Some(target) = &host {
        test.args(["--target", target]);
    }
    if !check(ws, "test", test) {
        failed.push("test");
    }

    failed
}

fn feature_args(cmd: &mut Command, ws: &Workspace) {
    if ws.no_default_features {
        cmd.arg("--no-default-features");
    }
    if !ws.features.is_empty() {
        cmd.args(["--features", &ws.features.join(",")]);
    }
}

/// `fmt --check` then `clippy -D warnings` for an embedded row, on its pinned target.
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

/// `cargo doc` with rustdoc warnings denied, on `target` when given.
fn host_doc(filter: &str, target: Option<&str>, features: &[&str]) -> Command {
    let mut doc = Command::new("cargo");
    doc.args(["doc", "--no-deps", filter]);
    if !features.is_empty() {
        doc.args(["--features", &features.join(",")]);
    }
    if let Some(target) = target {
        doc.args(["--target", target]);
    }
    let flags = std::env::var("RUSTDOCFLAGS").unwrap_or_default();
    doc.env("RUSTDOCFLAGS", format!("{flags} -D warnings").trim_start());
    doc
}

/// `rustdoc -D warnings` for a library, on `target` when given.
fn rustdoc(target: Option<&str>, features: &[&str]) -> Command {
    let mut doc = Command::new("cargo");
    doc.args(["rustdoc", "--lib"]);
    if !features.is_empty() {
        doc.args(["--features", &features.join(",")]);
    }
    if let Some(target) = target {
        doc.args(["--target", target]);
    }
    doc.args(["--", "-D", "warnings"]);
    doc
}

/// `clippy` then `rustdoc`, both `-D warnings`, for one chip's build of a host-gated library.
fn lint_cross(c: &workspaces::Cross) -> Vec<&'static str> {
    let mut failed = Vec::new();
    let ws = Workspace {
        name: c.name,
        path: c.path,
        features: c.features,
        no_default_features: false,
    };
    let mut clippy = Command::new("cargo");
    clippy.args(["clippy", "--target", c.target]);
    clippy.args(["--features", &c.features.join(",")]);
    clippy.args(["--", "-W", EXTRA_LINTS, "-D", "warnings"]);
    if !check(&ws, "clippy", clippy) {
        failed.push("clippy");
    }
    if !check(&ws, "doc", rustdoc(Some(c.target), c.features)) {
        failed.push("doc");
    }
    failed
}

#[cfg(test)]
mod tests {
    use crate::manifest_facts::ManifestFacts;
    use crate::{registry, workspaces};

    /// `--locked` refuses a row whose crate has no lockfile on a fresh clone.
    #[test]
    fn every_firmware_crate_has_a_tracked_lockfile() {
        let dirs = super::tracked_lock_dirs().unwrap();
        for family in registry::FAMILIES {
            assert!(
                dirs.iter().any(|d| d == family.path),
                "`{}` has no tracked lockfile",
                family.path
            );
        }
    }

    #[test]
    fn a_lockfile_that_does_not_match_its_manifest_is_refused() {
        let dir = crate::root().join(format!("target/lock-fixture-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("src")).unwrap();
        std::fs::write(dir.join("src/lib.rs"), "").unwrap();
        std::fs::write(
            dir.join("Cargo.toml"),
            "[package]\nname = \"fixture\"\nversion = \"0.1.0\"\n[workspace]\n",
        )
        .unwrap();
        let lock = |name: &str| {
            format!("version = 4\n\n[[package]]\nname = \"{name}\"\nversion = \"0.1.0\"\n")
        };
        std::fs::write(dir.join("Cargo.lock"), lock("fixture")).unwrap();
        let current = super::lock_is_current(&dir);
        std::fs::write(dir.join("Cargo.lock"), lock("renamed")).unwrap();
        let stale = super::lock_is_current(&dir);
        std::fs::remove_dir_all(&dir).unwrap();
        assert!(current);
        assert!(!stale);
    }

    /// Whether a registry entry (exact path, or a `<prefix>/*` glob) names this crate directory.
    fn covers(entry: &str, dir: &str) -> bool {
        match entry.strip_suffix("/*") {
            Some(prefix) => dir.strip_prefix(prefix).is_some_and(|r| r.starts_with('/')),
            None => entry == dir,
        }
    }

    /// Every crate in the tree must hold a place in exactly one of the gate's lists.
    #[test]
    fn every_crate_is_named_by_the_gate_registry() {
        let mut dirs = Vec::new();
        workspaces::crate_dirs(&crate::root(), "", &mut dirs);
        dirs.sort();
        assert!(dirs.len() > 30, "the walk found too few crates: {dirs:?}");
        // A crate may hold several host rows, one per feature set; it still counts once here.
        let mut host: Vec<&str> = workspaces::WORKSPACES.iter().map(|ws| ws.path).collect();
        host.dedup();
        let entries: Vec<&str> = host
            .into_iter()
            .chain(workspaces::embedded().map(|ws| ws.path))
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

    /// A cross row re-lints a host-gated crate with a feature it declares, on a pinned target.
    #[test]
    fn every_cross_row_names_a_gated_crate_feature_and_target() {
        let toolchain = std::fs::read_to_string(crate::root().join("rust-toolchain.toml")).unwrap();
        for c in workspaces::CROSS {
            assert!(
                workspaces::WORKSPACES.iter().any(|ws| ws.path == c.path),
                "`{}` is not a host-gated crate",
                c.path
            );
            let manifest = crate::manifest_facts::ManifestFacts::read(
                crate::root().join(c.path).join("Cargo.toml"),
            )
            .unwrap();
            let declared = manifest.features().unwrap();
            for f in c.features {
                assert!(declared.contains_key(*f), "`{}` declares no `{f}`", c.path);
            }
            assert!(
                toolchain.contains(&format!("\"{}\"", c.target)),
                "`{}` is not a pinned target",
                c.target
            );
        }
    }

    /// Every host row is documented, its targets found from Cargo's conventional paths alone.
    #[test]
    fn every_host_row_documents_targets_at_conventional_paths() {
        for ws in workspaces::WORKSPACES {
            assert!(
                workspaces::doc_filter(ws).is_some(),
                "`{}` documents nothing",
                ws.name
            );
            for dir in workspaces::package_dirs(&crate::root().join(ws.path)) {
                let text = std::fs::read_to_string(dir.join("Cargo.toml")).unwrap();
                let manifest: toml::Table = text.parse().unwrap();
                assert!(
                    !manifest.contains_key("lib") && !manifest.contains_key("bin"),
                    "`{}` declares its targets, which `doc_filter` does not read",
                    dir.display()
                );
            }
        }
    }

    /// A doc chip is the one the crate's tests select, on a dependency the library itself has.
    #[test]
    fn every_doc_chip_is_the_dev_dependency_chip_of_a_documented_row() {
        for (path, feature) in workspaces::DOC_CHIPS {
            assert!(
                workspaces::WORKSPACES
                    .iter()
                    .any(|ws| ws.path == *path && workspaces::doc_filter(ws).is_some()),
                "`{path}` has no documented host row"
            );
            let (dep, chip) = feature.split_once('/').unwrap();
            let text =
                std::fs::read_to_string(crate::root().join(path).join("Cargo.toml")).unwrap();
            let manifest: toml::Table = text.parse().unwrap();
            assert!(
                manifest["dependencies"].get(dep).is_some(),
                "`{path}` does not depend on `{dep}`"
            );
            let dev = &manifest["dev-dependencies"][dep]["features"];
            assert!(
                dev.as_array()
                    .unwrap()
                    .iter()
                    .any(|f| f.as_str() == Some(chip)),
                "`{path}`'s `{dep}` dev-dependency does not select `{chip}`"
            );
        }
    }

    /// Every crate outside the root workspace declares its own `[workspace]` table.
    #[test]
    fn every_crate_outside_the_root_workspace_declares_its_own() {
        let mut dirs = Vec::new();
        workspaces::crate_dirs(&crate::root(), "", &mut dirs);
        for dir in dirs.iter().filter(|d| *d != "." && *d != "xtask") {
            let text = std::fs::read_to_string(crate::root().join(dir).join("Cargo.toml")).unwrap();
            assert!(
                text.lines().any(|l| l.trim() == "[workspace]"),
                "`{dir}` declares no `[workspace]` of its own"
            );
        }
    }

    /// Why `manifest` could link `dap-conformance` into an image, if it could.
    fn links_conformance(manifest: &ManifestFacts) -> Option<String> {
        match manifest.dependency("dap-conformance") {
            Ok(None) => None,
            Ok(Some(dep)) if dep.declaration.ends_with("dev-dependencies") => None,
            Ok(Some(dep)) => Some(format!("declares it under `{}`", dep.declaration)),
            Err(e) => Some(e.to_string()),
        }
    }

    /// Firmware-linked crates take `dap-conformance` only as a dev-dependency.
    #[test]
    fn no_firmware_linked_crate_depends_on_dap_conformance() {
        let mut dirs = Vec::new();
        workspaces::crate_dirs(&crate::root(), "", &mut dirs);
        let linked = ["crates/", "platform/", "firmware/", "tests/firmwares/"];
        let (mut users, mut found) = (0, Vec::new());
        for dir in dirs
            .iter()
            .filter(|d| linked.iter().any(|p| d.starts_with(p)) && *d != "crates/dap-conformance")
        {
            let manifest = ManifestFacts::read(crate::root().join(dir).join("Cargo.toml")).unwrap();
            if let Ok(Some(_)) = manifest.dependency("dap-conformance") {
                users += 1;
            }
            if let Some(why) = links_conformance(&manifest) {
                found.push(format!("`{dir}` {why}"));
            }
        }
        assert!(found.is_empty(), "{}", found.join("; "));
        assert!(users >= 5, "only {users} crates use dap-conformance");
    }

    #[test]
    fn dap_conformance_is_refused_outside_dev_dependencies() {
        let dep =
            r#"dap-differential = { package = "dap-conformance", path = "../dap-conformance" }"#;
        for (table, refused) in [
            ("[dev-dependencies]", false),
            ("[target.'cfg(test)'.dev-dependencies]", false),
            ("[dependencies]", true),
            ("[build-dependencies]", true),
            ("[target.'cfg(unix)'.dependencies]", true),
        ] {
            let manifest =
                ManifestFacts::parse("fixture/Cargo.toml", &format!("{table}\n{dep}\n")).unwrap();
            assert_eq!(links_conformance(&manifest).is_some(), refused, "{table}");
        }
    }
}
