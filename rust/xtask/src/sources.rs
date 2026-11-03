//! The files an xtask binary is built from, as paths relative to `xtask/`.

use std::io;
use std::path::Path;

/// `Cargo.toml`, `build.rs` and everything under `src/`, sorted.
pub fn files(xtask: &Path) -> io::Result<Vec<String>> {
    let mut out = vec!["Cargo.toml".to_string(), "build.rs".to_string()];
    walk(xtask, "src", &mut out)?;
    Ok(out)
}

fn walk(xtask: &Path, rel: &str, out: &mut Vec<String>) -> io::Result<()> {
    let mut names = std::fs::read_dir(xtask.join(rel))?
        .map(|e| e.map(|e| e.file_name().to_string_lossy().into_owned()))
        .collect::<io::Result<Vec<_>>>()?;
    names.sort();
    for name in names {
        let path = format!("{rel}/{name}");
        if xtask.join(&path).is_dir() {
            walk(xtask, &path, out)?;
        } else {
            out.push(path);
        }
    }
    Ok(())
}
