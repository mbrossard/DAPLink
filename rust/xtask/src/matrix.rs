//! Generated feature shapes. The registry names the images the fleet flashes; this crosses those
//! rows with the features no row names, so a combination nobody flashes still has to compile.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use crate::registry::{Firmware, FIRMWARE};
use crate::workspaces::{EMBEDDED, EXCLUDED, WORKSPACES};

/// A combination the matrix deliberately does not build, and why. Naming one here is how the
/// gate states its bound instead of silently dropping the shape.
pub struct Incompatible {
    pub path: &'static str,
    pub added: &'static [&'static str],
    pub reason: &'static str,
}

pub const INCOMPATIBLE: &[Incompatible] = &[];

/// A declared feature no gate row reaches, and why that is allowed to stand. The reason must
/// name what keeps the feature alive -- an out-of-tree consumer, a path only a test takes --
/// because an excuse nobody can check is how a feature stays uncompiled.
pub struct Unreached {
    pub path: &'static str,
    pub feature: &'static str,
    pub reason: &'static str,
}

pub const UNREACHED: &[Unreached] = &[];

/// A feature pair no registry row builds together: the row it extends, and the features added on
/// top of that row's own. Named by hand rather than generated, because crossing every pair of a
/// crate's features costs far more build time than the combinations are worth.
pub struct Pair {
    pub base: &'static str,
    pub added: &'static [&'static str],
}

pub const PAIRS: &[Pair] = &[
    // `carrier-leds` next to `flash`, on both LED backends: the embassy seam drives the bank,
    // the RTIC one drives it from `daplink-rp-hal`.
    Pair {
        base: "firmware/rp2040-pico-embassy-flash",
        added: &["carrier-leds"],
    },
    Pair {
        base: "firmware/rp2040-pico-rtic-flash",
        added: &["carrier-leds"],
    },
    // The Debug Probe overclock remains orthogonal to its bitbang control.
    Pair {
        base: "firmware/debugprobe-overclock-180mhz",
        added: &["bitbang"],
    },
    // Both Pico runtimes retain one exact overclock, crossed with each optional composition.
    Pair {
        base: "firmware/rp2040-pico-embassy-overclock-200mhz",
        added: &["bitbang"],
    },
    Pair {
        base: "firmware/rp2040-pico-embassy-overclock-200mhz",
        added: &["flash-rp2040"],
    },
    Pair {
        base: "firmware/rp2040-pico-embassy-overclock-200mhz",
        added: &["carrier-leds"],
    },
    Pair {
        base: "firmware/rp2040-pico-rtic-overclock-200mhz",
        added: &["bitbang"],
    },
    Pair {
        base: "firmware/rp2040-pico-rtic-overclock-200mhz",
        added: &["flash-rp2040"],
    },
    Pair {
        base: "firmware/rp2040-pico-rtic-overclock-200mhz",
        added: &["carrier-leds"],
    },
    // `usb-counters` next to `flash`: both forward into `daplink-embassy-usb`, so one row
    // compiles the combination the two seams share.
    Pair {
        base: "firmware/rp2350-pico2-embassy-flash",
        added: &["usb-counters"],
    },
    // The Pico 2 overclock remains orthogonal to its control and optional compositions.
    Pair {
        base: "firmware/rp2350-pico2-embassy-overclock-200mhz",
        added: &["bitbang"],
    },
    Pair {
        base: "firmware/rp2350-pico2-embassy-overclock-200mhz",
        added: &["flash-rp2350"],
    },
    Pair {
        base: "firmware/rp2350-pico2-embassy-overclock-200mhz",
        added: &["carrier-leds"],
    },
    Pair {
        base: "firmware/rp2350-pico2-embassy-overclock-200mhz",
        added: &["usb-counters"],
    },
];

/// A no-default feature proof generated from a registry row without becoming a flashed image.
pub struct NoDefault {
    pub base: &'static str,
    pub features: &'static [&'static str],
}

pub const NO_DEFAULTS: &[NoDefault] = &[
    NoDefault {
        base: "firmware/rp2040-pico-embassy",
        features: &["bitbang", "guard"],
    },
    NoDefault {
        base: "firmware/debugprobe",
        features: &["bitbang", "guard"],
    },
    NoDefault {
        base: "firmware/rp2040-pico-rtic",
        features: &["bitbang", "guard"],
    },
    NoDefault {
        base: "firmware/rp2350-pico2-embassy",
        features: &["bitbang", "guard"],
    },
];

/// One generated shape: a registry row's crate and default-feature disposition, plus the
/// features the crossing adds on top of that row's own.
pub struct Shape {
    pub base: &'static Firmware,
    pub features: Vec<String>,
    pub added: Vec<String>,
    pub no_default_features: bool,
}

impl Shape {
    /// How the shape prints in a gate summary: the row it extends and what was added.
    pub fn label(&self) -> String {
        let defaults = if self.no_default_features {
            " --no-default-features"
        } else {
            ""
        };
        format!("{}{defaults} +{}", self.base.name, self.added.join(","))
    }
}

/// One entry of a `[features]` value: a same-crate feature, an optional dependency the feature
/// activates, or a feature turned on in another crate.
enum Entry {
    Own(String),
    Dep(String),
    Forward {
        pkg: String,
        feat: String,
        weak: bool,
    },
}

impl Entry {
    /// The same-crate feature this entry names, if it names one.
    fn own(&self) -> Option<&str> {
        match self {
            Entry::Own(f) => Some(f),
            _ => None,
        }
    }

    /// Read one `[features]` string: `dep:x`, `pkg/feat`, `pkg?/feat`, or a plain feature.
    fn parse(s: &str) -> Entry {
        if let Some(pkg) = s.strip_prefix("dep:") {
            return Entry::Dep(pkg.to_string());
        }
        match s.split_once('/') {
            Some((pkg, feat)) => Entry::Forward {
                pkg: pkg.trim_end_matches('?').to_string(),
                feat: feat.to_string(),
                weak: pkg.ends_with('?'),
            },
            None => Entry::Own(s.to_string()),
        }
    }
}

/// One dependency declaration and what the declaring crate turns on in it. A `dev` entry is
/// compiled only for that crate's own test targets, so only a gate row on the crate sees it.
struct Dep {
    name: String,
    optional: bool,
    default_features: bool,
    features: Vec<String>,
    dev: bool,
}

/// A parsed manifest: enough of one to follow a feature from a gate row into the crates it
/// forwards to.
#[derive(Default)]
struct Manifest {
    name: String,
    members: Vec<String>,
    features: BTreeMap<String, Vec<Entry>>,
    deps: Vec<Dep>,
}

/// Parse a crate's manifest. Hand-read rather than deserialized: the walk needs three tables of
/// manifests this tree writes, which is cheaper than pulling a TOML parser into the root lockfile.
fn manifest(root: &Path, path: &str) -> Manifest {
    let text = match std::fs::read_to_string(root.join(path).join("Cargo.toml")) {
        Ok(s) => s,
        Err(_) => return Manifest::default(),
    };
    let mut out = Manifest::default();
    let mut section = String::new();
    let (mut key, mut value, mut depth) = (String::new(), String::new(), 0i32);
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('#') {
            continue;
        }
        if depth == 0 {
            if trimmed.starts_with('[') {
                section = trimmed.to_string();
                continue;
            }
            let Some((name, rest)) = line.split_once('=') else {
                continue;
            };
            if name.starts_with(char::is_whitespace) {
                continue;
            }
            key = name.trim().trim_matches('"').to_string();
            value = rest.to_string();
            depth += value_depth(&value);
        } else {
            value.push_str(line);
            depth += value_depth(line);
        }
        if depth != 0 {
            continue;
        }
        if section == "[package]" && key == "name" {
            out.name = value.trim().trim_matches('"').to_string();
        } else if section == "[workspace]" && key == "members" {
            out.members = strings(&value);
        } else if section == "[features]" {
            let entries = strings(&value).iter().map(|s| Entry::parse(s)).collect();
            out.features.insert(key.clone(), entries);
        } else if section.ends_with("dependencies]") {
            out.deps.push(Dep {
                name: key.clone(),
                optional: value.contains("optional = true"),
                default_features: !value.contains("default-features = false"),
                features: dep_features(&value),
                dev: section.contains("dev-dependencies"),
            });
        }
    }
    out
}

/// The crate's own `[features]` table: each feature and the same-crate features it turns on.
/// `dep:` and `<pkg>/<feat>` entries name other crates and carry no closure here -- following
/// those is [`reach`]'s job, not a shape's.
fn feature_table(root: &Path, path: &str) -> BTreeMap<String, Vec<String>> {
    manifest(root, path)
        .features
        .iter()
        .map(|(k, v)| {
            let own = v
                .iter()
                .filter_map(Entry::own)
                .map(str::to_string)
                .collect();
            (k.clone(), own)
        })
        .collect()
}

/// Net bracket depth a manifest line adds, so a multi-line value reads as one.
fn value_depth(text: &str) -> i32 {
    text.chars()
        .map(|c| match c {
            '[' | '{' => 1,
            ']' | '}' => -1,
            _ => 0,
        })
        .sum()
}

/// The quoted strings of a manifest value.
fn strings(value: &str) -> Vec<String> {
    value
        .split('"')
        .skip(1)
        .step_by(2)
        .map(str::to_string)
        .collect()
}

/// The `features = [...]` array of a dependency declaration; `default-features` is not it.
fn dep_features(value: &str) -> Vec<String> {
    let mut rest = value;
    while let Some(at) = rest.find("features") {
        let dashed = rest[..at].ends_with('-');
        rest = &rest[at + "features".len()..];
        if dashed {
            continue;
        }
        match (rest.find('['), rest.find(']')) {
            (Some(open), Some(close)) if open < close => return strings(&rest[open..close]),
            _ => continue,
        }
    }
    Vec::new()
}

/// Every same-crate feature a set of seeds turns on, the table's implications followed through.
fn closure(table: &BTreeMap<String, Vec<String>>, seeds: &[String]) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut queue: Vec<String> = seeds.to_vec();
    while let Some(f) = queue.pop() {
        if !out.insert(f.clone()) {
            continue;
        }
        if let Some(deps) = table.get(&f) {
            queue.extend(deps.iter().cloned());
        }
    }
    out
}

/// The features a registry row actually compiles with, defaults included unless it dropped them.
fn row_features(table: &BTreeMap<String, Vec<String>>, fw: &Firmware) -> BTreeSet<String> {
    let mut seeds: Vec<String> = fw.features.iter().map(|f| f.to_string()).collect();
    if !fw.no_default_features {
        seeds.push("default".to_string());
    }
    closure(table, &seeds)
}

/// Whether a shape is one the matrix has been told not to build.
fn is_incompatible(path: &str, added: &[String]) -> bool {
    INCOMPATIBLE.iter().any(|i| {
        i.path == path
            && i.added
                .iter()
                .map(|a| a.to_string())
                .eq(added.iter().cloned())
    })
}

fn shape(base: &'static Firmware, added: Vec<String>, no_default_features: bool) -> Shape {
    let mut features: Vec<String> = base.features.iter().map(|f| f.to_string()).collect();
    features.extend(added.iter().cloned());
    Shape {
        base,
        features,
        added,
        no_default_features,
    }
}

/// A shape a registry row already compiles is dropped: the variant rows the fleet flashes are
/// themselves crossings, so generating them again would only double the gate's cost.
fn is_registry_row(
    table: &BTreeMap<String, Vec<String>>,
    rows: &[&'static Firmware],
    shape: &Shape,
) -> bool {
    let mut seeds = shape.features.clone();
    if !shape.no_default_features {
        seeds.push("default".to_string());
    }
    let wanted = closure(table, &seeds);
    rows.iter().any(|r| {
        r.no_default_features == shape.no_default_features && row_features(table, r) == wanted
    })
}

/// The shapes the matrix adds to the registry: every row of a `defmt`-capable crate crossed with
/// `defmt`, then each declared feature still built by nothing, on that crate's canonical row.
pub fn shapes(root: &Path) -> Vec<Shape> {
    let mut out = Vec::new();
    let mut done: BTreeSet<&str> = BTreeSet::new();
    for fw in FIRMWARE {
        if !done.insert(fw.path) {
            continue;
        }
        let table = feature_table(root, fw.path);
        if table.is_empty() {
            continue;
        }
        let rows: Vec<&'static Firmware> = FIRMWARE.iter().filter(|f| f.path == fw.path).collect();
        let mut covered: BTreeSet<String> = BTreeSet::new();
        for row in &rows {
            covered.extend(row_features(&table, row));
        }

        if table.contains_key("defmt") {
            for row in &rows {
                let added = vec!["defmt".to_string()];
                if row_features(&table, row).contains("defmt") || is_incompatible(row.path, &added)
                {
                    continue;
                }
                let s = shape(row, added, row.no_default_features);
                if !is_registry_row(&table, &rows, &s) {
                    out.push(s);
                }
            }
        }

        let canonical = rows.iter().find(|f| f.name == f.path).unwrap_or(&rows[0]);
        for feat in table.keys().filter(|f| *f != "default") {
            let added = vec![feat.clone()];
            if covered.contains(feat) || is_incompatible(fw.path, &added) {
                continue;
            }
            covered.extend(closure(&table, &added));
            let s = shape(canonical, added, canonical.no_default_features);
            if !is_registry_row(&table, &rows, &s) {
                out.push(s);
            }
        }
    }
    out.extend(pair_shapes(root));
    out.extend(no_default_shapes(root));
    out
}

/// The shapes [`PAIRS`] names, each on the registry row it extends. An entry naming a row or a
/// feature that does not exist generates nothing; a test says so instead.
fn pair_shapes(root: &Path) -> Vec<Shape> {
    let mut out = Vec::new();
    for p in PAIRS {
        let Some(base) = FIRMWARE.iter().find(|f| f.name == p.base) else {
            continue;
        };
        let added: Vec<String> = p.added.iter().map(|a| a.to_string()).collect();
        if is_incompatible(base.path, &added) {
            continue;
        }
        let table = feature_table(root, base.path);
        let rows: Vec<&'static Firmware> =
            FIRMWARE.iter().filter(|f| f.path == base.path).collect();
        let s = shape(base, added, base.no_default_features);
        if !is_registry_row(&table, &rows, &s) {
            out.push(s);
        }
    }
    out
}

fn no_default_shapes(root: &Path) -> Vec<Shape> {
    let mut out = Vec::new();
    for n in NO_DEFAULTS {
        let Some(base) = FIRMWARE.iter().find(|f| f.name == n.base) else {
            continue;
        };
        let added: Vec<String> = n.features.iter().map(|f| f.to_string()).collect();
        let table = feature_table(root, base.path);
        let rows: Vec<&'static Firmware> =
            FIRMWARE.iter().filter(|f| f.path == base.path).collect();
        let s = shape(base, added, true);
        if !is_registry_row(&table, &rows, &s) {
            out.push(s);
        }
    }
    out
}

/// Every gated workspace's manifest, keyed by its path under `rust/`. The three lists in
/// `workspaces.rs` name every `Cargo.toml` in the tree -- a closure test there says so -- so a
/// crate cannot escape this map by being unlisted.
struct Universe {
    by_path: BTreeMap<String, Manifest>,
    by_name: BTreeMap<String, String>,
}

fn universe(root: &Path) -> Universe {
    let paths = WORKSPACES
        .iter()
        .chain(EMBEDDED)
        .map(|w| w.path)
        .chain(EXCLUDED.iter().map(|e| e.path));
    let mut by_path = BTreeMap::new();
    let mut by_name = BTreeMap::new();
    for path in paths {
        let mut m = manifest(root, path);
        // A row on a virtual manifest stands for its single member: the root row runs cargo at
        // `rust/`, and `xtask` is the crate that resolves there.
        if m.name.is_empty() {
            if let [member] = m.members.as_slice() {
                m = manifest(root, &format!("{path}/{member}"));
            }
        }
        by_name.insert(m.name.clone(), path.to_string());
        by_path.insert(path.to_string(), m);
    }
    Universe { by_path, by_name }
}

/// The `(path, feature)` pairs one gate invocation compiles: the row's own features, followed
/// through every `dep:` and `<pkg>/<feat>` entry and through the dependency declarations of the
/// crates they name. `dev` says whether the row compiles test targets, since a dev-dependency's
/// features reach a crate only then.
///
/// A `<pkg>?/<feat>` forward counts only where the row already activated `<pkg>`: a weak forward
/// compiles nothing on its own, and counting it would be coverage the gate cannot back.
fn reach(
    u: &Universe,
    path: &str,
    features: &[String],
    defaults: bool,
    dev: bool,
) -> BTreeSet<(String, String)> {
    let mut pkgs: BTreeSet<String> = BTreeSet::from([path.to_string()]);
    let mut with_defaults: BTreeSet<String> = BTreeSet::new();
    if defaults {
        with_defaults.insert(path.to_string());
    }
    let mut feats: BTreeSet<(String, String)> = features
        .iter()
        .map(|f| (path.to_string(), f.clone()))
        .collect();
    loop {
        let before = (pkgs.len(), with_defaults.len(), feats.len());
        for p in pkgs.clone() {
            let Some(m) = u.by_path.get(&p) else {
                continue;
            };
            if with_defaults.contains(&p) && m.features.contains_key("default") {
                feats.insert((p.clone(), "default".to_string()));
            }
            for d in &m.deps {
                if d.dev && !(dev && p == path) {
                    continue;
                }
                let Some(dp) = u.by_name.get(&d.name) else {
                    continue;
                };
                if !d.optional {
                    pkgs.insert(dp.clone());
                }
                if !pkgs.contains(dp) {
                    continue;
                }
                if d.default_features {
                    with_defaults.insert(dp.clone());
                }
                feats.extend(d.features.iter().map(|f| (dp.clone(), f.clone())));
            }
        }
        for (p, f) in feats.clone() {
            let Some(entries) = u.by_path.get(&p).and_then(|m| m.features.get(&f)) else {
                continue;
            };
            for e in entries {
                match e {
                    Entry::Own(g) => {
                        feats.insert((p.clone(), g.clone()));
                    }
                    Entry::Dep(x) => {
                        if let Some(xp) = u.by_name.get(x) {
                            pkgs.insert(xp.clone());
                        }
                    }
                    Entry::Forward { pkg, feat, weak } => {
                        let Some(xp) = u.by_name.get(pkg) else {
                            continue;
                        };
                        if *weak && !pkgs.contains(xp) {
                            continue;
                        }
                        pkgs.insert(xp.clone());
                        feats.insert((xp.clone(), feat.clone()));
                    }
                }
            }
        }
        if (pkgs.len(), with_defaults.len(), feats.len()) == before {
            return feats;
        }
    }
}

/// Every `(path, feature)` the tree compiles anywhere: the host gate's rows with their test
/// targets, the embedded rows at their defaults, every registry row, and every generated shape.
fn compiled(root: &Path) -> BTreeSet<(String, String)> {
    let u = universe(root);
    let mut out = BTreeSet::new();
    let owned = |fs: &[&str]| fs.iter().map(|f| f.to_string()).collect::<Vec<String>>();
    for w in WORKSPACES {
        out.extend(reach(&u, w.path, &owned(w.features), true, true));
    }
    for w in EMBEDDED {
        out.extend(reach(&u, w.path, &owned(w.features), true, false));
    }
    for f in FIRMWARE {
        out.extend(reach(
            &u,
            f.path,
            &owned(f.features),
            !f.no_default_features,
            false,
        ));
    }
    for s in shapes(root) {
        out.extend(reach(
            &u,
            s.base.path,
            &s.features,
            !s.no_default_features,
            false,
        ));
    }
    out
}

/// Every declared feature nothing in the tree compiles and no list excuses. The gate's whole
/// claim is that this is empty: a feature here is dead as far as every check in the tree goes.
pub fn unreached(root: &Path) -> Vec<(String, String)> {
    let compiled = compiled(root);
    let excused = |path: &str, feat: &String| {
        UNREACHED
            .iter()
            .any(|u| u.path == path && u.feature == feat)
            || INCOMPATIBLE
                .iter()
                .any(|i| i.path == path && i.added.contains(&feat.as_str()))
    };
    let mut out = Vec::new();
    for (path, m) in &universe(root).by_path {
        for feat in m.features.keys() {
            if feat == "default"
                || compiled.contains(&(path.clone(), feat.clone()))
                || excused(path, feat)
            {
                continue;
            }
            out.push((path.clone(), feat.clone()));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn forwards(entries: &[Entry]) -> BTreeSet<(&str, &str, bool)> {
        entries
            .iter()
            .filter_map(|entry| match entry {
                Entry::Forward { pkg, feat, weak } => Some((pkg.as_str(), feat.as_str(), *weak)),
                _ => None,
            })
            .collect()
    }

    /// The gate's feature closure: a feature no row, shape or dependency declaration compiles is
    /// dead as far as every check in the tree is concerned, whether a firmware or a library
    /// declares it. Excusing one means naming it in [`UNREACHED`] or [`INCOMPATIBLE`].
    #[test]
    fn every_declared_feature_is_compiled_by_something() {
        let orphans = unreached(&crate::root());
        let listed: Vec<String> = orphans
            .iter()
            .map(|(p, f)| format!("`{p}` declares `{f}`"))
            .collect();
        assert!(
            orphans.is_empty(),
            "no row, shape or dependency compiles: {}",
            listed.join(", ")
        );
    }

    /// The `defmt` crossing is the rule the matrix exists for: no image ships logging on, so
    /// nothing but the crossing ever compiles a bench feature next to it.
    #[test]
    fn every_row_of_a_defmt_capable_crate_is_crossed_with_defmt() {
        let root = crate::root();
        let shapes = shapes(&root);
        for fw in FIRMWARE {
            let table = feature_table(&root, fw.path);
            if !table.contains_key("defmt") || row_features(&table, fw).contains("defmt") {
                continue;
            }
            let mut seeds: Vec<String> = fw.features.iter().map(|f| f.to_string()).collect();
            seeds.push("defmt".to_string());
            if !fw.no_default_features {
                seeds.push("default".to_string());
            }
            let wanted = closure(&table, &seeds);
            let compiled = FIRMWARE
                .iter()
                .filter(|r| r.path == fw.path)
                .any(|r| row_features(&table, r) == wanted)
                || shapes
                    .iter()
                    .any(|s| s.base.name == fw.name && s.added == ["defmt"]);
            assert!(compiled, "`{}` is never compiled with defmt", fw.name);
        }
    }

    /// A manifest whose `[features]` table the parser misreads would silently shrink the gate.
    #[test]
    fn the_feature_table_reads_multi_line_arrays_and_drops_foreign_features() {
        let table = feature_table(&crate::root(), "firmware/mculink");
        assert_eq!(table["default"], ["accel", "guard"]);
        assert_eq!(table["flash-stm32f401"], ["flash"]);
        assert!(
            table["defmt"].is_empty(),
            "defmt forwards to other crates only"
        );
        assert!(table.contains_key("pass-trace"));
    }

    #[test]
    fn rp_firmware_models_are_consistent_and_fully_crossed() {
        struct Model {
            path: &'static str,
            overclock: &'static str,
            accel_forwards: &'static [(&'static str, &'static str, bool)],
            clock_forwards: &'static [(&'static str, &'static str, bool)],
            crossings: &'static [&'static str],
        }

        const MODELS: &[Model] = &[
            Model {
                path: "firmware/rp2040-pico-embassy",
                overclock: "overclock-200mhz",
                accel_forwards: &[("daplink-rs", "rp2040-jtag-accel", false)],
                clock_forwards: &[
                    ("daplink-rp-embassy", "clock-200mhz", false),
                    ("daplink-rp-pio", "clock-200mhz", true),
                ],
                crossings: &["bitbang", "flash-rp2040", "carrier-leds"],
            },
            Model {
                path: "firmware/rp2040-pico-rtic",
                overclock: "overclock-200mhz",
                accel_forwards: &[("daplink-rs", "rp2040-jtag-accel", false)],
                clock_forwards: &[
                    ("daplink-rp-hal", "clock-200mhz", false),
                    ("daplink-rp-pio", "clock-200mhz", true),
                ],
                crossings: &["bitbang", "flash-rp2040", "carrier-leds"],
            },
            Model {
                path: "firmware/debugprobe",
                overclock: "overclock-180mhz",
                accel_forwards: &[],
                clock_forwards: &[
                    ("daplink-rp-embassy", "clock-180mhz", false),
                    ("daplink-rp-pio", "clock-180mhz", true),
                ],
                crossings: &["bitbang"],
            },
            Model {
                path: "firmware/rp2350-pico2-embassy",
                overclock: "overclock-200mhz",
                accel_forwards: &[("daplink-rs", "rp2040-jtag-accel", false)],
                clock_forwards: &[
                    ("daplink-rp-embassy", "clock-200mhz", false),
                    ("daplink-rp-pio", "clock-200mhz", true),
                ],
                crossings: &["bitbang", "flash-rp2350", "carrier-leds", "usb-counters"],
            },
        ];

        let root = crate::root();
        let shapes = shapes(&root);
        for model in MODELS {
            let table = feature_table(&root, model.path);
            let manifest = manifest(&root, model.path);
            assert_eq!(table["default"], ["accel", "guard"], "{}", model.path);
            assert!(manifest.features["bitbang"].is_empty(), "{}", model.path);
            assert!(
                manifest.features["accel"]
                    .iter()
                    .any(|entry| matches!(entry, Entry::Dep(name) if name == "daplink-rp-pio")),
                "{} accel does not activate daplink-rp-pio",
                model.path
            );
            assert_eq!(
                forwards(&manifest.features["accel"]),
                model.accel_forwards.iter().copied().collect(),
                "{} accel forwarding",
                model.path
            );
            assert_eq!(
                forwards(&manifest.features["guard"]),
                [("daplink-rs", "guard", false)].into_iter().collect(),
                "{} guard forwarding",
                model.path
            );

            let default = FIRMWARE
                .iter()
                .find(|fw| fw.name == model.path)
                .unwrap_or_else(|| panic!("`{}` has no default registry row", model.path));
            assert!(!default.no_default_features && default.features.is_empty());
            assert_eq!(
                row_features(&table, default),
                ["accel", "default", "guard"]
                    .into_iter()
                    .map(str::to_string)
                    .collect(),
                "{} default",
                model.path
            );

            let bitbang_name = format!("{}-bitbang", model.path);
            let bitbang = FIRMWARE
                .iter()
                .find(|fw| fw.name == bitbang_name)
                .unwrap_or_else(|| panic!("`{}` has no bitbang registry row", model.path));
            assert!(!bitbang.no_default_features);
            assert_eq!(bitbang.features, ["bitbang"]);
            assert_eq!(
                row_features(&table, bitbang),
                ["accel", "bitbang", "default", "guard"]
                    .into_iter()
                    .map(str::to_string)
                    .collect(),
                "{} default plus bitbang",
                model.path
            );

            let no_default = shapes
                .iter()
                .find(|s| {
                    s.base.name == model.path
                        && s.no_default_features
                        && s.added == ["bitbang", "guard"]
                })
                .unwrap_or_else(|| panic!("`{}` has no no-default bitbang proof", model.path));
            assert_eq!(
                closure(&table, &no_default.features),
                ["bitbang", "guard"]
                    .into_iter()
                    .map(str::to_string)
                    .collect(),
                "{} no-default bitbang",
                model.path
            );

            let overclocks: Vec<&str> = table
                .keys()
                .filter(|f| f.starts_with("overclock-"))
                .map(String::as_str)
                .collect();
            assert_eq!(overclocks, [model.overclock], "{}", model.path);
            let overclock_name = format!("{}-{}", model.path, model.overclock);
            let overclock = FIRMWARE
                .iter()
                .find(|fw| fw.name == overclock_name)
                .unwrap_or_else(|| panic!("`{}` has no overclock registry row", model.path));
            assert!(!overclock.no_default_features);
            assert_eq!(overclock.features, [model.overclock]);

            assert_eq!(
                forwards(&manifest.features[model.overclock]),
                model.clock_forwards.iter().copied().collect(),
                "{} {} forwarding",
                model.path,
                model.overclock
            );

            for added in model.crossings {
                assert!(
                    shapes.iter().any(|s| {
                        s.base.name == overclock.name
                            && !s.no_default_features
                            && s.added == [*added]
                    }),
                    "`{overclock_name} +{added}` generates no shape"
                );
            }
            if table.contains_key("defmt") {
                assert!(
                    shapes.iter().any(|s| {
                        s.base.name == overclock.name
                            && !s.no_default_features
                            && s.added == ["defmt"]
                    }),
                    "`{overclock_name} +defmt` generates no shape"
                );
            }
        }
    }

    /// The dependency half of the same parser: a misread declaration would drop a whole crate
    /// out of the walk, or hand it features its dependent never asked for.
    #[test]
    fn the_dependency_table_reads_optionality_defaults_and_features() {
        let m = manifest(&crate::root(), "platform/daplink-usb-device");
        let core = m.deps.iter().find(|d| d.name == "daplink-core").unwrap();
        assert!(core.dev, "the fixture dependency is a dev-dependency");
        assert_eq!(core.features, ["bot-fixtures"]);
        let m = manifest(&crate::root(), "firmware/mculink");
        assert_eq!(m.name, "mculink");
        let targets = m.deps.iter().find(|d| d.name == "daplink-targets").unwrap();
        assert!(targets.optional && !targets.default_features && targets.features.is_empty());
    }

    /// A manifest the walk cannot name is a crate no forward can reach, so every gated workspace
    /// has to parse to its own name.
    #[test]
    fn every_gated_workspace_parses_to_a_distinct_package_name() {
        let u = universe(&crate::root());
        assert_eq!(u.by_name.len(), u.by_path.len());
        assert!(!u.by_name.contains_key(""));
    }

    /// What the walk reaches from one row, checked against a forward the gate depends on: the
    /// firmware's `accel` is the only thing that compiles the LPC55 seam's masked-GPIO engine.
    #[test]
    fn the_walk_follows_a_firmware_feature_into_a_platform_crate() {
        let u = universe(&crate::root());
        let feats = reach(&u, "firmware/mculink", &[], true, false);
        assert!(feats.contains(&(
            "platform/daplink-lpc55-hal".to_string(),
            "masked-swd".to_string()
        )));
        assert!(!feats.contains(&("crates/daplink-vfs".to_string(), "defmt".to_string())));
    }

    /// A pair entry that names no row, no feature, or a combination a registry row already
    /// builds is an entry the gate silently drops.
    #[test]
    fn every_pair_entry_names_a_real_row_and_generates_a_shape() {
        let root = crate::root();
        let shapes = shapes(&root);
        for p in PAIRS {
            let base = FIRMWARE
                .iter()
                .find(|f| f.name == p.base)
                .unwrap_or_else(|| panic!("`{}` is no registry row", p.base));
            let table = feature_table(&root, base.path);
            for a in p.added {
                assert!(table.contains_key(*a), "`{}` declares no `{a}`", base.path);
            }
            assert!(
                shapes
                    .iter()
                    .any(|s| s.base.name == base.name && s.added == p.added),
                "`{} +{}` generates no shape",
                p.base,
                p.added.join(",")
            );
        }
    }

    #[test]
    fn every_no_default_entry_names_a_real_row_and_generates_a_shape() {
        let root = crate::root();
        let shapes = shapes(&root);
        for n in NO_DEFAULTS {
            let base = FIRMWARE
                .iter()
                .find(|f| f.name == n.base)
                .unwrap_or_else(|| panic!("`{}` is no registry row", n.base));
            let table = feature_table(&root, base.path);
            for feature in n.features {
                assert!(
                    table.contains_key(*feature),
                    "`{}` declares no `{feature}`",
                    base.path
                );
            }
            assert!(
                shapes.iter().any(|s| {
                    s.base.name == base.name && s.no_default_features && s.added == n.features
                }),
                "`{} --no-default-features +{}` generates no shape",
                n.base,
                n.features.join(",")
            );
        }
    }

    #[test]
    fn an_incompatible_entry_names_a_real_crate_and_feature() {
        let root = crate::root();
        for i in INCOMPATIBLE {
            let table = feature_table(&root, i.path);
            assert!(!table.is_empty(), "`{}` names no crate", i.path);
            for a in i.added {
                assert!(table.contains_key(*a), "`{}` declares no `{a}`", i.path);
            }
        }
    }

    /// An excuse for a feature that is compiled, or for one no crate declares, is an excuse
    /// nobody will ever revisit.
    #[test]
    fn an_unreached_entry_names_a_real_feature_that_is_still_unreached() {
        let root = crate::root();
        let compiled = compiled(&root);
        for u in UNREACHED {
            let table = feature_table(&root, u.path);
            assert!(
                table.contains_key(u.feature),
                "`{}` declares no `{}`",
                u.path,
                u.feature
            );
            assert!(
                !compiled.contains(&(u.path.to_string(), u.feature.to_string())),
                "`{}`'s `{}` is compiled; drop the excuse",
                u.path,
                u.feature
            );
        }
    }
}
