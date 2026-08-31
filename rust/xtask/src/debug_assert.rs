//! A source scan for statements written as `debug_assert!` arguments. The macro and its siblings
//! expand to nothing in release, so a call placed there never runs in a shipped image. Clippy's
//! `debug_assert_with_mut_call` decides the same question from types and is exact where it
//! applies, but it skips proc-macro expansions -- so it sees nothing inside `#[rtic::app]` or
//! `#[embassy_executor::task]`, which is where every firmware task body lives. This scan reads
//! the source text and so reaches there; it pays for that with a syntactic rule, flagging an
//! operand that is a call and carries no comparison.

use std::path::{Path, PathBuf};

/// A flagged operand that is in fact effect-free, and why. Keyed by the operand text rather than
/// a line, so the entry survives edits above it and dies with the assertion it names.
pub struct Allowed {
    pub path: &'static str,
    pub argument: &'static str,
    pub reason: &'static str,
}

pub const ALLOWED: &[Allowed] = &[Allowed {
    path: "crates/daplink-vfs/src/transfer.rs",
    argument: "self.is_pending()",
    reason: "a pure `&self` predicate; the syntactic rule cannot tell it from a mutating call",
}];

pub struct Finding {
    pub path: String,
    pub line: usize,
    pub argument: String,
}

/// Operators that make an operand a comparison rather than a bare call.
const COMPARISONS: &[&str] = &["==", "!=", "<", ">", "&&", "||"];

/// Every `.rs` file under `dir`, skipping build-artifact and hidden trees.
fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        let path = entry.path();
        if path.is_dir() {
            if name != "target" && !name.starts_with('.') {
                sources(&path, out);
            }
        } else if name.ends_with(".rs") {
            out.push(path);
        }
    }
}

/// The text between the parenthesis at `open` and its match, or `None` if unbalanced.
fn balanced(src: &str, open: usize) -> Option<&str> {
    let mut depth = 0usize;
    for (i, c) in src[open..].char_indices() {
        match c {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&src[open + 1..open + i]);
                }
            }
            _ => {}
        }
    }
    None
}

/// Split on commas outside parentheses, brackets, and braces.
fn top_level_commas(arg: &str) -> Vec<&str> {
    let mut parts = Vec::new();
    let mut depth = 0usize;
    let mut start = 0usize;
    for (i, c) in arg.char_indices() {
        match c {
            '(' | '[' | '{' => depth += 1,
            ')' | ']' | '}' => depth = depth.saturating_sub(1),
            ',' if depth == 0 => {
                parts.push(&arg[start..i]);
                start = i + 1;
            }
            _ => {}
        }
    }
    parts.push(&arg[start..]);
    parts
}

/// Flag operands of `debug_assert*!` that are calls with no comparison around them. Returns the
/// line number and the operand text of each.
pub fn scan_source(src: &str) -> Vec<(usize, String)> {
    let mut out = Vec::new();
    let mut at = 0usize;
    while let Some(found) = src[at..].find("debug_assert") {
        let start = at + found;
        at = start + "debug_assert".len();
        // The equality forms carry two operands; every form may carry trailing format arguments,
        // which are never operands.
        let (operands, rest) = match &src[at..] {
            r if r.starts_with("_eq!") || r.starts_with("_ne!") => (2, &r[4..]),
            r if r.starts_with('!') => (1, &r[1..]),
            _ => continue,
        };
        let open = src.len() - rest.trim_start().len();
        if !src[open..].starts_with('(') {
            continue;
        }
        let Some(arg) = balanced(src, open) else {
            continue;
        };
        let line = src[..start].matches('\n').count() + 1;
        for part in top_level_commas(arg).into_iter().take(operands) {
            let text = part.split_whitespace().collect::<Vec<_>>().join(" ");
            if text.contains('(') && !COMPARISONS.iter().any(|op| text.contains(op)) {
                out.push((line, text));
            }
        }
    }
    out
}

/// Scan every source file under `root`, dropping the operands [`ALLOWED`] names.
pub fn scan(root: &Path) -> Vec<Finding> {
    let mut files = Vec::new();
    sources(root, &mut files);
    files.sort();
    let mut out = Vec::new();
    for file in files {
        // This module's own tests hold the flagged shapes as fixtures.
        if file.ends_with("xtask/src/debug_assert.rs") {
            continue;
        }
        let Ok(src) = std::fs::read_to_string(&file) else {
            continue;
        };
        let rel = file
            .strip_prefix(root)
            .unwrap_or(&file)
            .to_string_lossy()
            .replace('\\', "/");
        for (line, argument) in scan_source(&src) {
            if ALLOWED
                .iter()
                .any(|a| a.path == rel && a.argument == argument)
            {
                continue;
            }
            out.push(Finding {
                path: rel.clone(),
                line,
                argument,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::{scan, scan_source, ALLOWED};

    /// The shape that shipped dead: the ring push was the assertion's argument, so the bridge leg
    /// pushed nothing in release.
    #[test]
    fn a_push_written_as_the_assertion_argument_is_flagged() {
        let found = scan_source("debug_assert!(vcom.tx.push(byte));");
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].1, "vcom.tx.push(byte)");
    }

    #[test]
    fn comparisons_and_bare_conditions_are_not_flagged() {
        let src = "\
debug_assert!(num_bits <= data.len() * 8, \"num_bits exceeds data\");
debug_assert!(bits >= 1 && bits <= MAX_BITS);
debug_assert!(false, \"response ring full after a reserved slot\");
debug_assert_eq!(staged, n);
debug_assert!(programmed);
";
        assert!(scan_source(src).is_empty(), "{:?}", scan_source(src));
    }

    /// Both operands of the equality forms are read, and the trailing message is not.
    #[test]
    fn an_equality_operand_is_read_and_the_message_is_not() {
        let found = scan_source("debug_assert_eq!(n, ring.push_all(buf), \"push(buf) short\");");
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].1, "ring.push_all(buf)");
    }

    #[test]
    fn no_statement_is_written_inside_a_debug_assert_in_the_tree() {
        let found = scan(&crate::root());
        assert!(
            found.is_empty(),
            "a `debug_assert!` argument would be dropped in release: {:?}",
            found
                .iter()
                .map(|f| format!("{}:{} {}", f.path, f.line, f.argument))
                .collect::<Vec<_>>()
        );
    }

    /// An exemption outlives its assertion silently, so each one must still match live source.
    #[test]
    fn every_allowed_operand_still_exists() {
        for allowed in ALLOWED {
            let src = std::fs::read_to_string(crate::root().join(allowed.path))
                .unwrap_or_else(|e| panic!("{}: {e}", allowed.path));
            assert!(
                scan_source(&src).iter().any(|(_, a)| a == allowed.argument),
                "{} no longer holds `{}`, exempted because {}",
                allowed.path,
                allowed.argument,
                allowed.reason
            );
        }
    }
}
