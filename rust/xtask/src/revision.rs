//! The source revision handed to firmware builds, so `daplink-core` need not watch the tree.

use std::ffi::OsString;
use std::path::Path;
use std::process::Command;
use std::sync::OnceLock;

const VAR: &str = "DAPLINK_FW_REVISION";
const ROW_VAR: &str = "DAPLINK_FW_ROW";

/// Name `row` in the image; only a build of a registry row can.
pub fn stamp_row(cmd: &mut Command, row: &str) {
    cmd.env(ROW_VAR, row);
}

/// Hand `cmd` the `rust/` revision; host workspaces skip it so plain `cargo` does not rebuild.
pub fn stamp(cmd: &mut Command, rust_root: &Path) {
    stamp_from(cmd, std::env::var_os(VAR), rust_root);
}

/// A non-empty inherited revision passes through unchanged; only its absence consults git.
fn stamp_from(cmd: &mut Command, inherited: Option<OsString>, rust_root: &Path) {
    if let Some(rev) = inherited.filter(|v| !v.is_empty()) {
        cmd.env(VAR, rev);
    } else if let Some(rev) = current(rust_root) {
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

    /// `stamp` passes this tree's revision or nothing, never an empty or malformed one.
    #[test]
    fn stamp_passes_this_trees_revision_or_nothing() {
        let root = &crate::root();
        let stamped = passed(None, root);
        assert_eq!(stamped, current(root).map(str::to_string));
        assert_eq!(passed(Some(""), root), stamped);
        if let Some(rev) = stamped {
            assert!(rev.starts_with('g') && rev.len() > 1, "{rev}");
        }
    }

    /// A caller's `DAPLINK_FW_REVISION` wins over git, so a pinned identity build stays pinned.
    #[test]
    fn stamp_passes_an_inherited_revision_through() {
        let root = &crate::root();
        assert_eq!(
            passed(Some("gpinned0000"), root).as_deref(),
            Some("gpinned0000")
        );
    }

    fn passed(inherited: Option<&str>, root: &Path) -> Option<String> {
        let mut cmd = Command::new("cargo");
        stamp_from(&mut cmd, inherited.map(OsString::from), root);
        cmd.get_envs()
            .find(|(k, _)| *k == VAR)
            .map(|(_, v)| v.expect("a set variable").to_string_lossy().into_owned())
    }
}
