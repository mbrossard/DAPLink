//! TOML facts for xtask's matrix and pin checks; Cargo validates everything else.

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use toml::{Table, Value};

#[derive(Debug)]
pub struct Error {
    path: PathBuf,
    detail: String,
}

impl Error {
    pub(crate) fn new(path: &Path, detail: impl Into<String>) -> Self {
        Self {
            path: path.to_path_buf(),
            detail: detail.into(),
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.path.display(), self.detail)
    }
}

impl std::error::Error for Error {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DependencySource {
    Registry(Option<String>),
    // Keep repository spelling exact: an unproven URL equivalence must not hide a moved pin.
    Git(String),
    // Relative paths are made lexically absolute from the manifest; symlinks are not resolved.
    Path(String),
    Workspace,
}

impl fmt::Display for DependencySource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Registry(None) => f.write_str("registry crates.io"),
            Self::Registry(Some(name)) => write!(f, "registry {name}"),
            Self::Git(url) => write!(f, "git {url}"),
            Self::Path(path) => write!(f, "path {path}"),
            Self::Workspace => f.write_str("workspace"),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DependencySelector {
    Revision(String),
    Tag(String),
    Branch(String),
    Unpinned,
}

impl fmt::Display for DependencySelector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Revision(value) => write!(f, "rev {value}"),
            Self::Tag(value) => write!(f, "tag {value}"),
            Self::Branch(value) => write!(f, "branch {value}"),
            Self::Unpinned => f.write_str("unpinned"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct DependencyIdentity {
    pub alias: String,
    pub package: String,
    pub declaration: String,
    pub source: DependencySource,
    pub selector: DependencySelector,
    pub version: Option<String>,
}

impl PartialEq for DependencyIdentity {
    fn eq(&self, other: &Self) -> bool {
        // Aliases may differ; compare the effective package identity.
        self.package == other.package
            && self.declaration == other.declaration
            && self.source == other.source
            && self.selector == other.selector
            && self.version == other.version
    }
}

impl Eq for DependencyIdentity {}

impl fmt::Display for DependencyIdentity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let registry_version = matches!(self.source, DependencySource::Registry(_))
            .then(|| self.version.as_deref())
            .flatten();
        let selector = match registry_version {
            Some(version) => format!("version {version}"),
            None => self.selector.to_string(),
        };
        let version = match (&self.source, &self.version) {
            (DependencySource::Registry(_), _) | (_, None) => String::new(),
            (_, Some(value)) => format!("; version {value}"),
        };
        let declaration = if self.declaration == "dependencies" {
            String::new()
        } else {
            format!("; declaration {}", self.declaration)
        };
        if self.alias == self.package {
            write!(f, "{}; {selector}{version}{declaration}", self.source)
        } else {
            write!(
                f,
                "alias {} for {}; {}; {}{version}{declaration}",
                self.alias, self.package, self.source, selector,
            )
        }
    }
}

#[derive(Debug)]
struct DependencyDeclaration {
    location: String,
    identity: DependencyIdentity,
}

#[derive(Debug)]
pub struct ManifestFacts {
    path: PathBuf,
    document: Table,
}

impl ManifestFacts {
    pub fn read(path: impl AsRef<Path>) -> Result<Self, Error> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|source| Error::new(path, format!("could not read manifest: {source}")))?;
        Self::parse(path, &text)
    }

    pub(crate) fn parse(path: impl AsRef<Path>, text: &str) -> Result<Self, Error> {
        Self::parse_inner(path.as_ref(), text)
    }

    fn parse_inner(path: &Path, text: &str) -> Result<Self, Error> {
        let document = text
            .parse::<Table>()
            .map_err(|source| Error::new(path, format!("could not parse TOML: {source}")))?;
        Ok(Self {
            path: path.to_path_buf(),
            document,
        })
    }

    pub fn features(&self) -> Result<BTreeMap<String, Vec<String>>, Error> {
        let Some(value) = self.document.get("features") else {
            return Ok(BTreeMap::new());
        };
        let table = value
            .as_table()
            .ok_or_else(|| self.invalid("features", "expected a table"))?;
        table
            .iter()
            .map(|(name, value)| {
                let entries = value.as_array().ok_or_else(|| {
                    self.invalid(&format!("features.{name}"), "expected an array")
                })?;
                let entries = entries
                    .iter()
                    .enumerate()
                    .map(|(index, value)| {
                        value.as_str().map(str::to_owned).ok_or_else(|| {
                            self.invalid(&format!("features.{name}[{index}]"), "expected a string")
                        })
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                Ok((name.clone(), entries))
            })
            .collect()
    }

    pub fn dependency(&self, name: &str) -> Result<Option<DependencyIdentity>, Error> {
        let mut declarations = Vec::new();
        for kind in ["dependencies", "dev-dependencies", "build-dependencies"] {
            self.dependencies_at(&self.document, kind, kind, &mut declarations)?;
        }
        if let Some(targets) = self.document.get("target") {
            let targets = targets
                .as_table()
                .ok_or_else(|| self.invalid("target", "expected a table"))?;
            for (target, value) in targets {
                let table = value
                    .as_table()
                    .ok_or_else(|| self.invalid(&format!("target.{target}"), "expected a table"))?;
                for kind in ["dependencies", "dev-dependencies", "build-dependencies"] {
                    let location = format!("target.{target}.{kind}");
                    self.dependencies_at(table, kind, &location, &mut declarations)?;
                }
            }
        }

        let mut matches = declarations.into_iter().filter(|declaration| {
            declaration.identity.alias == name || declaration.identity.package == name
        });
        let Some(first) = matches.next() else {
            return Ok(None);
        };
        let rest: Vec<_> = matches.collect();
        if rest.is_empty() {
            return Ok(Some(first.identity));
        }
        let locations = std::iter::once(first.location)
            .chain(rest.into_iter().map(|declaration| declaration.location))
            .collect::<Vec<_>>()
            .join(", ");
        Err(self.invalid(
            name,
            format!("dependency declaration is ambiguous across {locations}"),
        ))
    }

    fn dependencies_at(
        &self,
        table: &Table,
        kind: &str,
        location: &str,
        out: &mut Vec<DependencyDeclaration>,
    ) -> Result<(), Error> {
        let Some(value) = table.get(kind) else {
            return Ok(());
        };
        let dependencies = value
            .as_table()
            .ok_or_else(|| self.invalid(location, "expected a table"))?;
        for (alias, value) in dependencies {
            let entry = format!("{location}.{alias}");
            out.push(DependencyDeclaration {
                location: entry.clone(),
                identity: self.dependency_identity(alias, value, &entry)?,
            });
        }
        Ok(())
    }

    fn dependency_identity(
        &self,
        alias: &str,
        value: &Value,
        location: &str,
    ) -> Result<DependencyIdentity, Error> {
        if let Some(version) = value.as_str() {
            return Ok(DependencyIdentity {
                alias: alias.to_owned(),
                package: alias.to_owned(),
                declaration: location
                    .strip_suffix(&format!(".{alias}"))
                    .unwrap_or(location)
                    .to_owned(),
                source: DependencySource::Registry(None),
                selector: DependencySelector::Unpinned,
                version: Some(version.to_owned()),
            });
        }
        let table = value
            .as_table()
            .ok_or_else(|| self.invalid(location, "expected a version string or table"))?;
        let package = optional_string(self, table, "package", location)?
            .unwrap_or(alias)
            .to_owned();
        let git = optional_string(self, table, "git", location)?;
        let path = optional_string(self, table, "path", location)?;
        let registry = optional_string(self, table, "registry", location)?;
        let workspace = match table.get("workspace") {
            None => false,
            Some(Value::Boolean(value)) => *value,
            Some(_) => {
                return Err(self.invalid(&format!("{location}.workspace"), "expected a boolean"))
            }
        };
        let source_count = usize::from(git.is_some())
            + usize::from(path.is_some())
            + usize::from(registry.is_some())
            + usize::from(workspace);
        if source_count > 1 {
            return Err(self.invalid(location, "dependency declares multiple sources"));
        }

        let selectors = [
            ("rev", optional_string(self, table, "rev", location)?),
            ("tag", optional_string(self, table, "tag", location)?),
            ("branch", optional_string(self, table, "branch", location)?),
        ];
        let version = optional_string(self, table, "version", location)?.map(str::to_owned);
        let selected: Vec<_> = selectors
            .into_iter()
            .filter_map(|(kind, value)| value.map(|value| (kind, value)))
            .collect();
        if selected.len() > 1 {
            let kinds = selected.iter().map(|(kind, _)| *kind).collect::<Vec<_>>();
            return Err(self.invalid(
                location,
                format!(
                    "dependency declares distinct selectors: {}",
                    kinds.join(", ")
                ),
            ));
        }
        let selector = match selected.first() {
            Some(("rev", value)) => DependencySelector::Revision((*value).to_owned()),
            Some(("tag", value)) => DependencySelector::Tag((*value).to_owned()),
            Some(("branch", value)) => DependencySelector::Branch((*value).to_owned()),
            Some((kind, _)) => unreachable!("known selector {kind}"),
            None => DependencySelector::Unpinned,
        };
        let source = if let Some(url) = git {
            DependencySource::Git(url.to_owned())
        } else if let Some(path) = path {
            if !matches!(selector, DependencySelector::Unpinned) {
                return Err(self.invalid(location, "path dependency declares a git selector"));
            }
            DependencySource::Path(self.normalized_dependency_path(path, location)?)
        } else if workspace {
            if !matches!(selector, DependencySelector::Unpinned) || version.is_some() {
                return Err(self.invalid(location, "workspace dependency adds a local selector"));
            }
            DependencySource::Workspace
        } else {
            if !matches!(selector, DependencySelector::Unpinned) {
                return Err(self.invalid(location, "registry dependency declares a git selector"));
            }
            if version.is_none() {
                return Err(self.invalid(location, "dependency declares no supported source"));
            }
            DependencySource::Registry(registry.map(str::to_owned))
        };
        Ok(DependencyIdentity {
            alias: alias.to_owned(),
            package,
            declaration: location
                .strip_suffix(&format!(".{alias}"))
                .unwrap_or(location)
                .to_owned(),
            source,
            selector,
            version,
        })
    }

    fn normalized_dependency_path(&self, path: &str, location: &str) -> Result<String, Error> {
        use std::path::Component;

        let source = Path::new(path);
        let joined = if source.is_absolute() {
            source.to_path_buf()
        } else {
            self.path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(source)
        };
        let absolute = if joined.is_absolute() {
            joined
        } else {
            std::env::current_dir()
                .map_err(|source| {
                    self.invalid(
                        location,
                        format!("could not resolve relative dependency path: {source}"),
                    )
                })?
                .join(joined)
        };
        let mut normalized = PathBuf::new();
        for component in absolute.components() {
            match component {
                Component::CurDir => {}
                Component::ParentDir => {
                    normalized.pop();
                }
                other => normalized.push(other.as_os_str()),
            }
        }
        Ok(normalized.display().to_string())
    }

    fn invalid(&self, key: &str, detail: impl fmt::Display) -> Error {
        Error::new(&self.path, format!("invalid `{key}`: {detail}"))
    }
}

fn optional_string<'a>(
    facts: &ManifestFacts,
    table: &'a Table,
    key: &str,
    location: &str,
) -> Result<Option<&'a str>, Error> {
    match table.get(key) {
        None => Ok(None),
        Some(Value::String(value)) => Ok(Some(value)),
        Some(_) => Err(facts.invalid(&format!("{location}.{key}"), "expected a string")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn facts(text: &str) -> ManifestFacts {
        ManifestFacts::parse("fixture/Cargo.toml", text).unwrap()
    }

    #[test]
    fn features_accept_indentation_quotes_and_brackets_in_comments() {
        let manifest = facts(
            r#"
                [features] # unmatched brackets do not matter: [[{
                default = ['accel']
                defmt = ["dep:defmt", 'other/defmt'] # ]}]
            "#,
        );
        let features = manifest.features().unwrap();
        assert_eq!(features["default"], ["accel"]);
        assert_eq!(features["defmt"], ["dep:defmt", "other/defmt"]);
    }

    #[test]
    fn table_dependency_and_renamed_package_keep_complete_identity() {
        let manifest = facts(
            r#"
                [dependencies.driver]
                package = 'dap-rs'
                git = 'https://example.invalid/dap-rs.git'
                rev = 'abc123'
            "#,
        );
        let identity = manifest.dependency("dap-rs").unwrap().unwrap();
        assert_eq!(identity.alias, "driver");
        assert_eq!(identity.package, "dap-rs");
        assert_eq!(
            identity.source,
            DependencySource::Git("https://example.invalid/dap-rs.git".to_owned())
        );
        assert_eq!(
            identity.selector,
            DependencySelector::Revision("abc123".to_owned())
        );
    }

    #[test]
    fn target_specific_duplicates_are_ambiguous() {
        let manifest = facts(
            r#"
                [dependencies]
                dap-rs = { git = 'https://example.invalid/a.git', rev = 'same' }

                [target.'cfg(unix)'.dependencies]
                dap-rs = { git = 'https://example.invalid/b.git', rev = 'same' }
            "#,
        );
        let error = manifest.dependency("dap-rs").unwrap_err().to_string();
        assert!(error.contains("fixture/Cargo.toml"), "{error}");
        assert!(error.contains("ambiguous"), "{error}");
        assert!(
            error.contains("target.cfg(unix).dependencies.dap-rs"),
            "{error}"
        );
    }

    #[test]
    fn malformed_and_unreadable_manifests_name_the_path() {
        let malformed = ManifestFacts::parse("bad/Cargo.toml", "features = [")
            .unwrap_err()
            .to_string();
        assert!(malformed.contains("bad/Cargo.toml"), "{malformed}");
        assert!(malformed.contains("parse TOML"), "{malformed}");

        let unreadable = ManifestFacts::read("does-not-exist/Cargo.toml")
            .unwrap_err()
            .to_string();
        assert!(
            unreadable.contains("does-not-exist/Cargo.toml"),
            "{unreadable}"
        );
        assert!(unreadable.contains("read manifest"), "{unreadable}");
    }

    #[test]
    fn distinct_selectors_are_rejected() {
        let manifest = facts(
            r#"
                [dependencies]
                dap-rs = { git = 'https://example.invalid/dap-rs.git', rev = 'a', branch = 'main' }
            "#,
        );
        let error = manifest.dependency("dap-rs").unwrap_err().to_string();
        assert!(error.contains("distinct selectors: rev, branch"), "{error}");
    }

    #[test]
    fn git_version_is_an_additional_constraint() {
        let manifest = facts(
            r#"
                [dependencies]
                dap-rs = { git = 'https://example.invalid/dap-rs.git', rev = 'a', version = '1' }
            "#,
        );
        let identity = manifest.dependency("dap-rs").unwrap().unwrap();
        assert_eq!(
            identity.to_string(),
            "git https://example.invalid/dap-rs.git; rev a; version 1"
        );
    }

    #[test]
    fn path_sources_are_normalized_from_each_manifest() {
        let first = ManifestFacts::parse(
            "one/Cargo.toml",
            "[dependencies]\ndep = { path = '../shared' }",
        )
        .unwrap()
        .dependency("dep")
        .unwrap()
        .unwrap();
        let second = ManifestFacts::parse(
            "two/nested/Cargo.toml",
            "[dependencies]\ndep = { path = '../../shared' }",
        )
        .unwrap()
        .dependency("dep")
        .unwrap()
        .unwrap();
        assert_eq!(first, second);

        let left = ManifestFacts::parse(
            "left/project/Cargo.toml",
            "[dependencies]\ndep = { path = '../shared' }",
        )
        .unwrap()
        .dependency("dep")
        .unwrap()
        .unwrap();
        let right = ManifestFacts::parse(
            "right/project/Cargo.toml",
            "[dependencies]\ndep = { path = '../shared' }",
        )
        .unwrap()
        .dependency("dep")
        .unwrap()
        .unwrap();
        assert_ne!(left, right);
    }

    #[test]
    fn unsupported_source_forms_fail_visibly() {
        let manifest = facts("[dependencies]\ndep = { index = 'https://example.invalid' }");
        let error = manifest.dependency("dep").unwrap_err().to_string();
        assert!(error.contains("fixture/Cargo.toml"), "{error}");
        assert!(error.contains("no supported source"), "{error}");
    }

    #[test]
    fn feature_values_are_checked_at_the_precise_key() {
        let manifest = facts("[features]\ndefmt = ['ok', 1]");
        let error = manifest.features().unwrap_err().to_string();
        assert!(error.contains("fixture/Cargo.toml"), "{error}");
        assert!(error.contains("features.defmt[1]"), "{error}");
    }
}
