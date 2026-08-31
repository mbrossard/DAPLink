//! Stamps the source revision into `DAPLINK_FW_REVISION`, consumed by `DAP_FW_VERSION`.
//!
//! A revision without `-dirty` is a promise that the binary was built from exactly that commit,
//! so this script must never emit one it cannot vouch for. Left to itself it re-runs over every
//! tracked file under `rust/`, which makes the promise true but rebuilds every dependent on any
//! edit; a caller that already knows the revision hands it over in `DAPLINK_FW_REVISION` instead,
//! and then nothing is watched. `xtask` does that for fleet builds.

use std::path::Path;
use std::process::Command;

/// The variable read as an override and emitted for `env!`.
const VAR: &str = "DAPLINK_FW_REVISION";

fn main() {
    let dir = std::env::var("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR");
    let rust_root = Path::new(&dir)
        .parent()
        .and_then(Path::parent)
        .expect("the crate sits at rust/crates/daplink-core")
        .to_path_buf();

    println!("cargo:rerun-if-env-changed={VAR}");
    if let Ok(injected) = std::env::var(VAR) {
        println!("cargo:rustc-env={VAR}={injected}");
        return;
    }

    let Some(git_dir) = git(&rust_root, &["rev-parse", "--absolute-git-dir"]) else {
        // Silence here would reach the bench as a firmware version nothing can judge.
        println!(
            "cargo:warning=no git revision: this build stamps `unknown`, and a staleness check \
             against it stays silent rather than warning"
        );
        println!("cargo:rustc-env={VAR}=unknown");
        return;
    };
    // A commit moves neither the worktree nor its mtimes, so the hash needs its own triggers.
    println!("cargo:rerun-if-changed={git_dir}/HEAD");
    println!("cargo:rerun-if-changed={git_dir}/index");
    watch_tree(&rust_root);
    println!("cargo:rustc-env={VAR}={}", revision(&rust_root));
}

/// Re-run over every tracked file under `rust/`, so no worktree edit can leave the stamp behind.
/// A tracked file already deleted is skipped: cargo treats a missing path as changed, which would
/// re-run the script on every build until the deletion is committed.
fn watch_tree(rust_root: &Path) {
    let Some(list) = git(rust_root, &["ls-files", "-z"]) else {
        return;
    };
    for rel in list.split('\0').filter(|p| !p.is_empty()) {
        let path = rust_root.join(rel);
        if path.exists() {
            println!("cargo:rerun-if-changed={}", path.display());
        }
    }
}

/// `g` + short commit hash, `-dirty` if tracked files under `rust/` changed, or `unknown`
/// without git. Changes outside `rust/` cannot reach the binary and do not mark it dirty.
fn revision(rust_root: &Path) -> String {
    let Some(hash) = git(rust_root, &["rev-parse", "--short=12", "HEAD"]) else {
        return "unknown".to_string();
    };
    // Untracked files never mark the build dirty.
    let dirty = match git(
        rust_root,
        &["status", "--porcelain", "--untracked-files=no", "--", "."],
    ) {
        Some(status) if !status.is_empty() => "-dirty",
        _ => "",
    };
    format!("g{hash}{dirty}")
}

/// Run `git -C dir <args>`, returning trimmed stdout on success and `None` on any failure.
fn git(dir: &Path, args: &[&str]) -> Option<String> {
    let out = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(args)
        .output()
        .ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}
