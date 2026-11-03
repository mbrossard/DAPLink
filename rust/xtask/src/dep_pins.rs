//! Keeps dependency pins repeated across independent workspaces identical.

use std::path::Path;

use crate::manifest_facts::{DependencyIdentity, DependencySource, Error, ManifestFacts};

/// One dependency and every manifest directory that pins it, relative to `rust/`.
pub struct Group {
    pub dep: &'static str,
    pub manifests: &'static [&'static str],
}

pub const GROUPS: &[Group] = &[
    // A mismatch compiles two incompatible `dap_rs` trait sets into one firmware.
    Group {
        dep: "dap-rs",
        manifests: &[
            "crates/dap-conformance",
            "crates/daplink-flash",
            "crates/daplink-rs",
            "crates/daplink-spi-swd",
        ],
    },
    // The LPC55 HAL fork, pinned by the platform crate and the firmware.
    Group {
        dep: "lpc55-hal",
        manifests: &["platform/daplink-lpc55-hal", "firmware/mculink"],
    },
];

pub fn pin(path: &Path, dep: &str) -> Result<Option<DependencyIdentity>, Error> {
    let identity = ManifestFacts::read(path)?.dependency(dep)?;
    comparable(path, dep, identity)
}

fn comparable(
    path: &Path,
    dep: &str,
    identity: Option<DependencyIdentity>,
) -> Result<Option<DependencyIdentity>, Error> {
    if identity
        .as_ref()
        .is_some_and(|identity| identity.source == DependencySource::Workspace)
    {
        return Err(Error::new(
            path,
            format!("cannot compare workspace-inherited `{dep}` without resolving its workspace"),
        ));
    }
    Ok(identity)
}

/// Every way a group's manifests disagree, including facts that could not be read unambiguously.
fn disagreements(
    dep: &str,
    pins: &[(&str, Result<Option<DependencyIdentity>, Error>)],
) -> Vec<String> {
    let mut out = Vec::new();
    let mut agreed: Option<(&str, &DependencyIdentity)> = None;
    for (path, pin) in pins {
        let pin = match pin {
            Ok(Some(pin)) => pin,
            Ok(None) => {
                out.push(format!("`{path}` declares no `{dep}` dependency"));
                continue;
            }
            Err(error) => {
                out.push(error.to_string());
                continue;
            }
        };
        match agreed {
            None => agreed = Some((path, pin)),
            Some((first, expected)) if pin != expected => out.push(format!(
                "`{path}` pins `{dep}` as {pin}, `{first}` as {expected}"
            )),
            Some(_) => {}
        }
    }
    out
}

/// Check every group against the tree, returning one line per disagreement or manifest error.
pub fn check(root: &Path) -> Vec<String> {
    let mut out = Vec::new();
    for group in GROUPS {
        let pins: Vec<_> = group
            .manifests
            .iter()
            .map(|path| {
                let manifest = root.join(path).join("Cargo.toml");
                (*path, pin(&manifest, group.dep))
            })
            .collect();
        out.extend(disagreements(group.dep, &pins));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(text: &str, dep: &str) -> Result<Option<DependencyIdentity>, Error> {
        ManifestFacts::parse("fixture/Cargo.toml", text)?.dependency(dep)
    }

    const PINNED: &str = r#"
[package]
name = "example"

[dependencies]
dap-rs = { git = "https://example.invalid/dap-rs.git", rev = "aaaa" }
other = { version = "1", default-features = false }
"#;

    const MOVED: &str = r#"
[dependencies]
dap-rs = { git = "https://example.invalid/dap-rs.git", rev = "bbbb" }
"#;

    const OTHER_REPOSITORY: &str = r#"
[dependencies]
dap-rs = { git = "https://mirror.invalid/dap-rs.git", rev = "aaaa" }
"#;

    const ABSENT: &str = r#"
[dependencies]
other = "1"
"#;

    #[test]
    fn the_tree_pins_every_shared_dependency_at_one_identity() {
        let found = check(&crate::root());
        assert!(found.is_empty(), "{}", found.join("; "));
    }

    #[test]
    fn every_manifest_pinning_a_grouped_dependency_is_in_its_group() {
        let root = crate::root();
        let mut dirs = Vec::new();
        crate::workspaces::crate_dirs(&root, "", &mut dirs);
        for dir in &dirs {
            let manifest = ManifestFacts::read(root.join(dir).join("Cargo.toml")).unwrap();
            for group in GROUPS {
                if manifest.dependency(group.dep).unwrap().is_some() {
                    assert!(
                        group.manifests.contains(&dir.as_str()),
                        "`{dir}` pins `{}` outside its group",
                        group.dep
                    );
                }
            }
        }
    }

    #[test]
    fn declarations_yield_revision_and_registry_identity() {
        let dap = fixture(PINNED, "dap-rs").unwrap().unwrap();
        assert_eq!(
            dap.to_string(),
            "git https://example.invalid/dap-rs.git; rev aaaa"
        );
        let other = fixture(PINNED, "other").unwrap().unwrap();
        assert_eq!(other.to_string(), "registry crates.io; version 1");
        assert!(fixture(PINNED, "dap").unwrap().is_none());
        assert!(fixture(ABSENT, "dap-rs").unwrap().is_none());
    }

    #[test]
    fn identical_identities_agree() {
        let pins = [
            ("a", fixture(PINNED, "dap-rs")),
            ("b", fixture(PINNED, "dap-rs")),
            ("c", fixture(PINNED, "dap-rs")),
        ];
        assert!(disagreements("dap-rs", &pins).is_empty());
    }

    #[test]
    fn a_moved_revision_names_both_identities() {
        let pins = [
            ("a", fixture(PINNED, "dap-rs")),
            ("b", fixture(MOVED, "dap-rs")),
        ];
        let found = disagreements("dap-rs", &pins);
        assert_eq!(found.len(), 1);
        assert!(found[0].contains("`b`"), "{}", found[0]);
        assert!(found[0].contains("rev bbbb"), "{}", found[0]);
        assert!(found[0].contains("rev aaaa"), "{}", found[0]);
    }

    #[test]
    fn equal_revisions_from_different_repositories_disagree() {
        let pins = [
            ("a", fixture(PINNED, "dap-rs")),
            ("b", fixture(OTHER_REPOSITORY, "dap-rs")),
        ];
        let found = disagreements("dap-rs", &pins);
        assert_eq!(found.len(), 1);
        assert!(found[0].contains("example.invalid"), "{}", found[0]);
        assert!(found[0].contains("mirror.invalid"), "{}", found[0]);
    }

    #[test]
    fn aliases_do_not_change_identity_but_packages_do() {
        let renamed = r#"
[dependencies.driver]
package = "dap-rs"
git = "https://example.invalid/dap-rs.git"
rev = "aaaa"
"#;
        let misleading = r#"
[dependencies.dap-rs]
package = "other-driver"
git = "https://example.invalid/dap-rs.git"
rev = "aaaa"
"#;
        let same = [
            ("a", fixture(PINNED, "dap-rs")),
            ("b", fixture(renamed, "dap-rs")),
        ];
        assert!(disagreements("dap-rs", &same).is_empty());

        let different = [
            ("a", fixture(PINNED, "dap-rs")),
            ("b", fixture(misleading, "dap-rs")),
        ];
        assert_eq!(disagreements("dap-rs", &different).len(), 1);
    }

    #[test]
    fn target_qualifiers_are_part_of_identity() {
        let unix = r#"
[target.'cfg(unix)'.dependencies]
dap-rs = { git = "https://example.invalid/dap-rs.git", rev = "aaaa" }
"#;
        let windows = r#"
[target.'cfg(windows)'.dependencies]
dap-rs = { git = "https://example.invalid/dap-rs.git", rev = "aaaa" }
"#;
        let pins = [
            ("unix", fixture(unix, "dap-rs")),
            ("windows", fixture(windows, "dap-rs")),
        ];
        let found = disagreements("dap-rs", &pins);
        assert_eq!(found.len(), 1);
        assert!(found[0].contains("target.cfg(unix).dependencies"));
        assert!(found[0].contains("target.cfg(windows).dependencies"));
    }

    #[test]
    fn a_manifest_missing_the_dependency_fails() {
        let pins = [
            ("a", fixture(PINNED, "dap-rs")),
            ("b", fixture(ABSENT, "dap-rs")),
        ];
        let found = disagreements("dap-rs", &pins);
        assert_eq!(found.len(), 1);
        assert!(
            found[0].contains("`b` declares no `dap-rs`"),
            "{}",
            found[0]
        );
    }

    #[test]
    fn malformed_pin_fails_without_running_matrix() {
        let pins = [
            ("a", fixture(PINNED, "dap-rs")),
            ("bad", fixture("[dependencies", "dap-rs")),
        ];
        let found = disagreements("dap-rs", &pins);
        assert_eq!(found.len(), 1);
        assert!(found[0].contains("fixture/Cargo.toml"), "{}", found[0]);
        assert!(found[0].contains("parse TOML"), "{}", found[0]);
    }

    #[test]
    fn workspace_inheritance_is_not_compared_without_resolution() {
        let identity = fixture("[dependencies]\ndap-rs = { workspace = true }", "dap-rs").unwrap();
        let error = comparable(Path::new("fixture/Cargo.toml"), "dap-rs", identity)
            .unwrap_err()
            .to_string();
        assert!(error.contains("fixture/Cargo.toml"), "{error}");
        assert!(error.contains("workspace-inherited"), "{error}");
    }
}
