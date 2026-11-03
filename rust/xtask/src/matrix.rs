//! Generated feature shapes: registry rows crossed with `defmt` and hand-named combinations.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use crate::manifest_facts::{Error, ManifestFacts};
use crate::registry::{Firmware, FIRMWARE};

/// A combination the matrix deliberately does not build; `reason` is its `compile_error!` text.
pub struct Incompatible {
    pub path: &'static str,
    pub added: &'static [&'static str],
    pub reason: &'static str,
}

pub const INCOMPATIBLE: &[Incompatible] = &[
    Incompatible {
        path: "firmware/stm32f103-embassy",
        added: &["accel"],
        reason: "`accel` composes only on the `nanodap` board wiring",
    },
    Incompatible {
        path: "firmware/stm32f103-embassy",
        added: &["flash"],
        reason: "`flash` composes only on the `nanodap` board wiring",
    },
    Incompatible {
        path: "firmware/stlinkv2",
        added: &["accel", "conntest"],
        reason: "accel and conntest reserve incompatible probe-pad compositions",
    },
    Incompatible {
        path: "firmware/stm32h523-embassy",
        added: &["spi-block", "flash"],
        reason: "spi-block runs half-duplex and holds the bus without the flash mux",
    },
    Incompatible {
        path: "firmware/stm32h523-embassy",
        added: &["spi-block", "accel-duplex"],
        reason: "spi-block runs half-duplex and holds the bus without the flash mux",
    },
    Incompatible {
        path: "firmware/stm32h523-embassy",
        added: &["plain-bitbang", "accel"],
        reason: "plain-bitbang replaces the fast bit-bang engine; accel is a third engine",
    },
];

/// A hand-named feature pair no registry row builds: the row it extends and the features added.
pub struct Pair {
    pub base: &'static str,
    pub added: &'static [&'static str],
}

pub const PAIRS: &[Pair] = &[
    // `carrier-leds` next to `flash`, on both LED backends.
    Pair {
        base: "firmware/rp2040-pico-embassy-flash",
        added: &["carrier-leds"],
    },
    Pair {
        base: "firmware/rp2040-pico-rtic-flash",
        added: &["carrier-leds"],
    },
    // Both Pico runtimes retain one exact overclock, crossed with the ancillary compositions.
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
        added: &["flash-rp2040"],
    },
    Pair {
        base: "firmware/rp2040-pico-rtic-overclock-200mhz",
        added: &["carrier-leds"],
    },
    // `carrier-leds` on the Pico 2 flash row, the RP2040 pairs' counterpart.
    Pair {
        base: "firmware/rp2350-pico2-embassy-flash",
        added: &["carrier-leds"],
    },
    // The duplex and plain engines under the flash mux, which no H523 row composes.
    Pair {
        base: "firmware/stm32h523-embassy-accel-duplex",
        added: &["flash", "flash-stm32f401"],
    },
    Pair {
        base: "firmware/stm32h523-embassy-plain",
        added: &["flash", "flash-stm32f401"],
    },
    // `usb-counters` next to `flash`: both forward into `daplink-embassy-usb`.
    Pair {
        base: "firmware/rp2350-pico2-embassy-flash",
        added: &["usb-counters"],
    },
    // The Pico 2 overclock remains orthogonal to its ancillary compositions.
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
    // The Pico 2 flash target no registry row selects.
    Pair {
        base: "firmware/rp2350-pico2-embassy",
        added: &["flash-rp2040"],
    },
    // Diagnostic features, on the H523 accel image they instrument.
    Pair {
        base: "firmware/stm32h523-embassy-accel",
        added: &["dap-trace"],
    },
    Pair {
        base: "firmware/stm32h523-embassy-accel",
        added: &["wire-probe"],
    },
    Pair {
        base: "firmware/stm32h523-embassy-accel",
        added: &["usb-counters"],
    },
    Pair {
        base: "firmware/mculink",
        added: &["probe-timing"],
    },
    // Bit-banged flash on MCU-Link: `bitbang` under the flash mux.
    Pair {
        base: "firmware/mculink-bitbang",
        added: &["flash", "flash-stm32f401"],
    },
    // The nRF52 targets, which no bench row flashes, beside each flash row's own target.
    Pair {
        base: "firmware/rp2040-pico-embassy-flash",
        added: &["flash-nrf52840", "flash-nrf52833"],
    },
    Pair {
        base: "firmware/rp2040-pico-rtic-flash",
        added: &["flash-nrf52840", "flash-nrf52833"],
    },
    Pair {
        base: "firmware/rp2350-pico2-embassy-flash",
        added: &["flash-nrf52840", "flash-nrf52833"],
    },
    Pair {
        base: "firmware/stm32h523-embassy-flash",
        added: &["flash-nrf52840", "flash-nrf52833"],
    },
    Pair {
        base: "firmware/mculink-flash",
        added: &["flash-nrf52840", "flash-nrf52833"],
    },
    Pair {
        base: "firmware/stm32f103-embassy-flash",
        added: &["flash-nrf52840", "flash-nrf52833"],
    },
    Pair {
        base: "firmware/stm32f103-embassy-accel-flash",
        added: &["flash-nrf52840", "flash-nrf52833"],
    },
];

/// A no-default feature proof generated from a registry row without becoming a flashed image.
pub struct NoDefault {
    pub base: &'static str,
    pub features: &'static [&'static str],
}

pub const NO_DEFAULTS: &[NoDefault] = &[
    // Overclocking is orthogonal to the engine composition selected without defaults.
    NoDefault {
        base: "firmware/rp2040-pico-embassy-bitbang",
        features: &["overclock-200mhz"],
    },
    NoDefault {
        base: "firmware/rp2040-pico-rtic-bitbang",
        features: &["overclock-200mhz"],
    },
    NoDefault {
        base: "firmware/debugprobe-bitbang",
        features: &["overclock-180mhz"],
    },
    NoDefault {
        base: "firmware/rp2350-pico2-embassy-bitbang",
        features: &["overclock-200mhz"],
    },
    // Bit-banged flash: `cfg(all(flash, not(accel)))`, otherwise compiled only by the RTIC crate.
    NoDefault {
        base: "firmware/rp2040-pico-embassy-bitbang",
        features: &["flash", "flash-rp2040"],
    },
    NoDefault {
        base: "firmware/rp2350-pico2-embassy-bitbang",
        features: &["flash", "flash-rp2350"],
    },
];

/// One generated shape: a registry row plus the features the crossing adds.
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

/// Whether a `[features]` entry names a same-crate feature rather than another crate's.
fn is_own(entry: &str) -> bool {
    !entry.starts_with("dep:") && !entry.contains('/')
}

/// The crate's own `[features]` table: each feature and the same-crate features it turns on.
fn feature_table(root: &Path, path: &str) -> Result<BTreeMap<String, Vec<String>>, Error> {
    Ok(ManifestFacts::read(root.join(path).join("Cargo.toml"))?
        .features()?
        .into_iter()
        .map(|(k, v)| {
            let own = v.into_iter().filter(|entry| is_own(entry)).collect();
            (k, own)
        })
        .collect())
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

/// A shape a registry row already compiles is dropped.
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

/// The shapes the matrix adds: `defmt` crossings, then [`PAIRS`] and [`NO_DEFAULTS`].
pub fn shapes(root: &Path) -> Result<Vec<Shape>, Error> {
    let mut out = Vec::new();
    let mut done: BTreeSet<&str> = BTreeSet::new();
    for fw in FIRMWARE {
        if !done.insert(fw.path) {
            continue;
        }
        let table = feature_table(root, fw.path)?;
        if table.is_empty() {
            continue;
        }
        let rows: Vec<&'static Firmware> = FIRMWARE.iter().filter(|f| f.path == fw.path).collect();

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
    }
    out.extend(pair_shapes(root)?);
    out.extend(no_default_shapes(root)?);
    Ok(out)
}

/// The shapes [`PAIRS`] names, each on the registry row it extends.
fn pair_shapes(root: &Path) -> Result<Vec<Shape>, Error> {
    let mut out = Vec::new();
    for p in PAIRS {
        let Some(base) = FIRMWARE.iter().find(|f| f.name == p.base) else {
            continue;
        };
        let added: Vec<String> = p.added.iter().map(|a| a.to_string()).collect();
        if is_incompatible(base.path, &added) {
            continue;
        }
        let table = feature_table(root, base.path)?;
        let rows: Vec<&'static Firmware> =
            FIRMWARE.iter().filter(|f| f.path == base.path).collect();
        let s = shape(base, added, base.no_default_features);
        if !is_registry_row(&table, &rows, &s) {
            out.push(s);
        }
    }
    Ok(out)
}

fn no_default_shapes(root: &Path) -> Result<Vec<Shape>, Error> {
    let mut out = Vec::new();
    for n in NO_DEFAULTS {
        let Some(base) = FIRMWARE.iter().find(|f| f.name == n.base) else {
            continue;
        };
        let added: Vec<String> = n.features.iter().map(|f| f.to_string()).collect();
        let table = feature_table(root, base.path)?;
        let rows: Vec<&'static Firmware> =
            FIRMWARE.iter().filter(|f| f.path == base.path).collect();
        let s = shape(base, added, true);
        if !is_registry_row(&table, &rows, &s) {
            out.push(s);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manifest(root: &Path, path: &str) -> BTreeMap<String, Vec<String>> {
        ManifestFacts::read(root.join(path).join("Cargo.toml"))
            .unwrap()
            .features()
            .unwrap()
    }

    /// The `<pkg>/<feat>` entries of a `[features]` value, weak forwards flagged.
    fn forwards(entries: &[String]) -> BTreeSet<(String, String, bool)> {
        entries
            .iter()
            .filter_map(|entry| entry.split_once('/'))
            .map(|(pkg, feat)| {
                (
                    pkg.trim_end_matches('?').to_string(),
                    feat.to_string(),
                    pkg.ends_with('?'),
                )
            })
            .collect()
    }

    /// No image ships `defmt`, so only the crossing compiles bench features next to it.
    #[test]
    fn every_row_of_a_defmt_capable_crate_is_crossed_with_defmt() {
        let root = crate::root();
        let shapes = shapes(&root).unwrap();
        for fw in FIRMWARE {
            let table = feature_table(&root, fw.path).unwrap();
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

    #[test]
    fn the_feature_table_reads_multi_line_arrays_and_drops_foreign_features() {
        let table = feature_table(&crate::root(), "firmware/mculink").unwrap();
        assert_eq!(table["default"], ["accel"]);
        assert_eq!(table["flash-stm32f401"], ["flash"]);
        assert!(
            table["defmt"].is_empty(),
            "defmt forwards to other crates only"
        );
        assert!(table.contains_key("probe-timing"));
    }

    #[test]
    fn rp_firmware_models_are_consistent_and_fully_crossed() {
        struct Model {
            path: &'static str,
            overclock: &'static str,
            crossings: &'static [&'static str],
        }

        const MODELS: &[Model] = &[
            Model {
                path: "firmware/rp2040-pico-embassy",
                overclock: "overclock-200mhz",
                crossings: &["flash-rp2040", "carrier-leds"],
            },
            Model {
                path: "firmware/rp2040-pico-rtic",
                overclock: "overclock-200mhz",
                crossings: &["flash-rp2040", "carrier-leds"],
            },
            Model {
                path: "firmware/debugprobe",
                overclock: "overclock-180mhz",
                crossings: &[],
            },
            Model {
                path: "firmware/rp2350-pico2-embassy",
                overclock: "overclock-200mhz",
                crossings: &["flash-rp2350", "carrier-leds", "usb-counters"],
            },
        ];

        let root = crate::root();
        let shapes = shapes(&root).unwrap();
        let retired = [
            "bitbang",
            "pio-swd",
            "pio-jtag",
            "pio-swd-fast",
            "pio-swd-transaction",
            "pio-jtag-autonomous",
            "optimized-pio",
        ];
        for model in MODELS {
            let table = feature_table(&root, model.path).unwrap();
            let manifest = manifest(&root, model.path);
            assert_eq!(table["default"], ["accel"], "{}", model.path);
            for feature in retired {
                assert!(
                    !table.contains_key(feature),
                    "{} still declares retired `{feature}`",
                    model.path
                );
            }
            assert!(
                manifest["accel"]
                    .iter()
                    .any(|entry| entry == "dep:daplink-rp-pio"),
                "{} accel does not activate daplink-rp-pio",
                model.path
            );
            assert_eq!(
                forwards(&manifest["accel"]),
                BTreeSet::new(),
                "{} accel forwarding",
                model.path
            );
            assert!(!manifest.contains_key("guard"));

            let default = FIRMWARE
                .iter()
                .find(|fw| fw.name == model.path)
                .unwrap_or_else(|| panic!("`{}` has no default registry row", model.path));
            assert!(!default.no_default_features && default.features.is_empty());
            assert_eq!(
                row_features(&table, default),
                ["accel", "default"]
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
            assert!(bitbang.no_default_features);
            assert!(bitbang.features.is_empty());
            assert_eq!(
                row_features(&table, bitbang),
                BTreeSet::new(),
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
            assert!(
                matches!(overclock.features, [f] | [f, "dap-packet-512"] if *f == model.overclock),
                "{overclock_name} features"
            );
            for (pkg, feature, _) in forwards(&manifest[model.overclock]) {
                assert!(
                    pkg != "daplink-rp-embassy" && pkg != "daplink-rp-pio",
                    "{} {} forwards retired {pkg}/{feature}",
                    model.path,
                    model.overclock
                );
            }

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
            assert!(
                shapes.iter().any(|s| {
                    s.base.name == bitbang.name
                        && s.no_default_features
                        && s.added == [model.overclock]
                }),
                "`{bitbang_name}` never compiles with {}",
                model.overclock
            );
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

        assert!(!FIRMWARE
            .iter()
            .filter(|fw| MODELS.iter().any(|model| model.path == fw.path))
            .any(|fw| retired.iter().any(|feature| fw.features.contains(feature))));
        assert!(!manifest(&root, "crates/daplink-rs").contains_key("rp2040-jtag-accel"));
        for path in ["platform/daplink-rp-embassy", "platform/daplink-rp-pio"] {
            let table = feature_table(&root, path).unwrap();
            assert!(!table.contains_key("clock-180mhz"), "{path}");
            assert!(!table.contains_key("clock-200mhz"), "{path}");
        }
        for model in MODELS {
            let build = std::fs::read_to_string(root.join(model.path).join("build.rs")).unwrap();
            assert!(!build.contains("rp_accel"), "{} build.rs", model.path);
        }
    }

    /// A pair entry naming no row, no feature, or an existing registry row is silently dropped.
    #[test]
    fn every_pair_entry_names_a_real_row_and_generates_a_shape() {
        let root = crate::root();
        let shapes = shapes(&root).unwrap();
        for p in PAIRS {
            let base = FIRMWARE
                .iter()
                .find(|f| f.name == p.base)
                .unwrap_or_else(|| panic!("`{}` is no registry row", p.base));
            let table = feature_table(&root, base.path).unwrap();
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
        let shapes = shapes(&root).unwrap();
        for n in NO_DEFAULTS {
            let base = FIRMWARE
                .iter()
                .find(|f| f.name == n.base)
                .unwrap_or_else(|| panic!("`{}` is no registry row", n.base));
            let table = feature_table(&root, base.path).unwrap();
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
            let table = feature_table(&root, i.path).unwrap();
            assert!(!table.is_empty(), "`{}` names no crate", i.path);
            for a in i.added {
                assert!(table.contains_key(*a), "`{}` declares no `{a}`", i.path);
            }
        }
    }

    /// The gate matches `reason` against the failing build, so it must be a guard's exact text.
    #[test]
    fn every_incompatible_reason_is_a_guard_in_its_crate() {
        let root = crate::root();
        for i in INCOMPATIBLE {
            let main = std::fs::read_to_string(root.join(i.path).join("src/main.rs")).unwrap();
            let guard = format!("compile_error!(\"{}\")", i.reason);
            assert!(main.contains(&guard), "`{}` has no guard `{guard}`", i.path);
        }
    }
}
