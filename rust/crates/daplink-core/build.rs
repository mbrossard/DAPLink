//! Stamps the revision into `DAPLINK_FW_REVISION` and passes `DAPLINK_FW_ROW` through.

use std::path::Path;
use std::process::Command;

/// The variable read as an override and emitted for `env!`.
const VAR: &str = "DAPLINK_FW_REVISION";
/// The registry row `xtask` builds; any other build stamps `unknown`.
const ROW_VAR: &str = "DAPLINK_FW_ROW";

fn main() {
    println!("cargo:rerun-if-env-changed={ROW_VAR}");
    let row = std::env::var(ROW_VAR)
        .ok()
        .filter(|row| !row.is_empty())
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env={ROW_VAR}={row}");

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

/// Re-run on every tracked file under `rust/`; a deleted one is skipped or cargo re-runs forever.
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

/// `g` + short hash, `-dirty` if tracked files under `rust/` changed, or `unknown` without git.
fn revision(rust_root: &Path) -> String {
    let Some(hash) = git(rust_root, &["rev-parse", "--short=12", "HEAD"]) else {
        return "unknown".to_string();
    };
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
