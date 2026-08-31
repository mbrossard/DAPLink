//! The source revision handed to a firmware build, so its version stamp is fresh without
//! `daplink-core`'s build script having to watch the tree for every image in the fleet.

use std::path::Path;
use std::process::Command;
use std::sync::OnceLock;

/// The variable `daplink-core`'s build script reads in place of asking git itself.
const VAR: &str = "DAPLINK_FW_REVISION";

/// Hand `cmd` the revision of the `rust/` tree, computed once per invocation. Host workspaces
/// are deliberately left out: they are also built by plain `cargo`, and a variable that appears
/// and disappears between the two would rebuild the crate each time it flipped.
pub fn stamp(cmd: &mut Command, rust_root: &Path) {
    if let Some(rev) = current(rust_root) {
        cmd.env(VAR, rev);
    }
}

/// The revision, or `None` when git cannot answer -- the build script then stamps `unknown`.
fn current(rust_root: &Path) -> Option<&'static str> {
    static REVISION: OnceLock<Option<String>> = OnceLock::new();
    REVISION
        .get_or_init(|| {
            let rev = compute(rust_root);
            if rev.is_none() {
                eprintln!(
                    "warning: no git revision here; images stamp `unknown` and a staleness \
                     check against them stays silent"
                );
            }
            rev
        })
        .as_deref()
}

fn compute(rust_root: &Path) -> Option<String> {
    let hash = git(rust_root, &["rev-parse", "--short=12", "HEAD"])?;
    let dirty = git(
        rust_root,
        &["status", "--porcelain", "--untracked-files=no", "--", "."],
    )
    .is_some_and(|s| !s.is_empty());
    Some(format(&hash, dirty))
}

/// `g` + short hash, `-dirty` when tracked files under `rust/` differ from the commit.
fn format(hash: &str, dirty: bool) -> String {
    format!("g{hash}{}", if dirty { "-dirty" } else { "" })
}

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

#[cfg(test)]
mod tests {
    use super::*;

    /// A dirty tree must not stamp a bare commit: that string promises the binary is that commit.
    #[test]
    fn dirty_never_stamps_a_bare_commit() {
        assert_eq!(format("0123456789ab", false), "g0123456789ab");
        assert_eq!(format("0123456789ab", true), "g0123456789ab-dirty");
    }

    /// `stamp` hands the build script this tree's revision, or leaves the variable unset for it
    /// to resolve; what it must never do is pass an empty or malformed one.
    #[test]
    fn stamp_passes_this_trees_revision_or_nothing() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
        let mut cmd = Command::new("cargo");
        stamp(&mut cmd, root);
        let passed = cmd
            .get_envs()
            .find(|(k, _)| *k == VAR)
            .map(|(_, v)| v.expect("a set variable").to_string_lossy().into_owned());
        assert_eq!(passed, current(root).map(str::to_string));
        if let Some(rev) = passed {
            assert!(rev.starts_with('g') && rev.len() > 1, "{rev}");
        }
    }
}
