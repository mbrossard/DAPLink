//! The `rust/` tree xtask works on, resolved when it runs rather than when it was compiled.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

/// The `rust/` tree, or why xtask must not run.
pub fn checked() -> Result<&'static Path, &'static str> {
    static ROOT: OnceLock<Result<PathBuf, String>> = OnceLock::new();
    ROOT.get_or_init(|| {
        let cwd = std::env::current_dir().map_err(|e| format!("no working directory: {e}"))?;
        let root = locate(std::env::var_os("CARGO_MANIFEST_DIR"), &cwd).ok_or_else(|| {
            format!(
                "{} is not inside a DAPLink rust/ tree; run `cargo xtask` from rust/",
                cwd.display()
            )
        })?;
        verify(&root, BUILT)?;
        Ok(root)
    })
    .as_deref()
    .map_err(String::as_str)
}

/// Cargo's run-time manifest directory, else the working directory's enclosing tree.
fn locate(manifest_dir: Option<OsString>, cwd: &Path) -> Option<PathBuf> {
    let is_tree = |dir: &Path| dir.join("xtask/Cargo.toml").is_file();
    manifest_dir
        .as_deref()
        .and_then(|dir| Path::new(dir).parent())
        .filter(|dir| is_tree(dir))
        .or_else(|| cwd.ancestors().find(|dir| is_tree(dir)))
        .map(Path::to_path_buf)
}

/// The sources this binary was compiled from, by path relative to `xtask/`.
const BUILT: &[(&str, &[u8])] = include!(concat!(env!("OUT_DIR"), "/sources.rs"));

/// Refuse a binary built from other sources: its registry would describe another tree.
fn verify(root: &Path, built: &[(&str, &[u8])]) -> Result<(), String> {
    let xtask = root.join("xtask");
    let files = crate::sources::files(&xtask)
        .map_err(|e| format!("could not read {}: {e}", xtask.display()))?;
    let names: Vec<&str> = built.iter().map(|(name, _)| *name).collect();
    let differs = if files != names {
        Some("the file list".to_string())
    } else {
        built
            .iter()
            .find(|(name, bytes)| std::fs::read(xtask.join(name)).ok().as_deref() != Some(*bytes))
            .map(|(name, _)| format!("xtask/{name}"))
    };
    match differs {
        None => Ok(()),
        Some(what) => Err(format!(
            "this xtask binary was not built from {}: {what} differs; rebuild it with \
             `touch {}/src/main.rs`, or give each tree its own CARGO_TARGET_DIR",
            xtask.display(),
            xtask.display()
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn this_tree() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .to_path_buf()
    }

    #[test]
    fn cargo_manifest_dir_names_the_tree() {
        let root = this_tree();
        let dir = root.join("xtask").into_os_string();
        assert_eq!(locate(Some(dir), Path::new("/")), Some(root));
    }

    #[test]
    fn working_directory_names_the_tree_without_cargo() {
        let root = this_tree();
        let cwd = root.join("crates/daplink-decoder/src");
        assert_eq!(locate(None, &cwd), Some(root.clone()));
        assert_eq!(locate(None, &root), Some(root));
    }

    /// Another package's manifest directory must not pull the root out of the working tree.
    #[test]
    fn a_foreign_manifest_dir_falls_back_to_the_working_directory() {
        let root = this_tree();
        let foreign = root.join("crates/daplink-decoder").into_os_string();
        assert_eq!(locate(Some(foreign), &root.join("firmware")), Some(root));
    }

    #[test]
    fn outside_any_tree_there_is_no_root() {
        assert_eq!(locate(None, Path::new("/")), None);
    }

    #[test]
    fn the_built_sources_match_this_tree() {
        assert_eq!(verify(&this_tree(), BUILT), Ok(()));
        assert_eq!(checked(), Ok(this_tree().as_path()));
    }

    #[test]
    fn a_binary_from_other_sources_is_refused() {
        let mut other = BUILT.to_vec();
        let main = other
            .iter()
            .position(|(name, _)| *name == "src/main.rs")
            .unwrap();
        other[main].1 = b"fn main() {}";
        let error = verify(&this_tree(), &other).unwrap_err();
        assert!(error.contains("xtask/src/main.rs differs"), "{error}");
        other.remove(main);
        let error = verify(&this_tree(), &other).unwrap_err();
        assert!(error.contains("the file list differs"), "{error}");
    }
}
