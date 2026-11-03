//! The gate registry: every workspace under `rust/`; a crate in no list fails the closure test.

use std::path::{Path, PathBuf};

/// One gated workspace: label, directory under `rust/`, and `clippy`/`test` features.
pub struct Workspace {
    pub name: &'static str,
    pub path: &'static str,
    pub features: &'static [&'static str],
    pub no_default_features: bool,
}

/// A workspace under `rust/` that is not part of the host gate, and why.
pub struct Excluded {
    pub path: &'static str,
    pub reason: &'static str,
}

pub const WORKSPACES: &[Workspace] = &[
    Workspace {
        name: "daplink-core",
        path: "crates/daplink-core",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-decoder",
        path: "crates/daplink-decoder",
        features: &[],
        no_default_features: false,
    },
    // The tests build without `std` too, which only a row without the default features keeps.
    Workspace {
        name: "daplink-decoder-no-std",
        path: "crates/daplink-decoder",
        features: &[],
        no_default_features: true,
    },
    Workspace {
        name: "daplink-spi-swd",
        path: "crates/daplink-spi-swd",
        features: &[],
        no_default_features: false,
    },
    // `std` publishes the host mock the crate's own tests reach through `cfg(test)`.
    Workspace {
        name: "daplink-flash",
        path: "crates/daplink-flash",
        features: &["std"],
        no_default_features: false,
    },
    // The defmt build's diagnostics must not add core accesses the tests pin down.
    Workspace {
        name: "daplink-flash-defmt",
        path: "crates/daplink-flash",
        features: &["std", "defmt"],
        no_default_features: false,
    },
    // The RP families run algorithms with interrupts masked; the tests pin both resume streams.
    Workspace {
        name: "daplink-flash-mask-interrupts",
        path: "crates/daplink-flash",
        features: &["std", "mask-interrupts"],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-targets",
        path: "crates/daplink-targets",
        features: &[
            "synthetic",
            "rp2040",
            "rp2350",
            "stm32f401ce",
            "nrf52840",
            "nrf52833",
        ],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-vfs",
        path: "crates/daplink-vfs",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-rs",
        path: "crates/daplink-rs",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-probe-rs",
        path: "crates/daplink-probe-rs",
        features: &[],
        no_default_features: false,
    },
    // Which firmware row a bench record names, against the registry.
    Workspace {
        name: "firmware-row",
        path: "crates/firmware-row",
        features: &[],
        no_default_features: false,
    },
    // Pure PL011 divisor arithmetic, host-tested here.
    Workspace {
        name: "pl011-baud",
        path: "crates/pl011-baud",
        features: &[],
        no_default_features: false,
    },
    // The model alone, then with the dap-rs reference the transfer-path crates' tests compare to.
    Workspace {
        name: "dap-conformance",
        path: "crates/dap-conformance",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "dap-conformance-differential",
        path: "crates/dap-conformance",
        features: &["differential"],
        no_default_features: false,
    },
    // Both MSC and non-MSC compositions are always available to host tests.
    Workspace {
        name: "daplink-usb-device",
        path: "platform/daplink-usb-device",
        features: &[],
        no_default_features: false,
    },
    // Chip-free, then each chip, whose register paths carry tests of their own.
    Workspace {
        name: "daplink-rp-pio",
        path: "platform/daplink-rp-pio",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-rp-pio-rp2040-host",
        path: "platform/daplink-rp-pio",
        features: &["rp2040"],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-rp-pio-rp235x-host",
        path: "platform/daplink-rp-pio",
        features: &["rp235x"],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-rp-dma-uart",
        path: "platform/daplink-rp-dma-uart",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-rp-dma-uart-rp2040-host",
        path: "platform/daplink-rp-dma-uart",
        features: &["rp2040"],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-rp-dma-uart-rp235x-host",
        path: "platform/daplink-rp-dma-uart",
        features: &["rp235x"],
        no_default_features: false,
    },
    // The host target excludes the unique-id flash read's ARM assembly.
    Workspace {
        name: "daplink-rp-hal",
        path: "platform/daplink-rp-hal",
        features: &[],
        no_default_features: false,
    },
    // No tests; the host `--target` overrides its embedded `.cargo/config.toml` for lint coverage.
    Workspace {
        name: "daplink-stm32f1-hal",
        path: "platform/daplink-stm32f1-hal",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-stm32f1-spi-swd",
        path: "platform/daplink-stm32f1-spi-swd",
        features: &[],
        no_default_features: false,
    },
    // Both clock profiles are always available beside the masked engine.
    Workspace {
        name: "daplink-lpc55-hal",
        path: "platform/daplink-lpc55-hal",
        features: &["masked-swd", "probe-timing"],
        no_default_features: false,
    },
    // The dev-dependency picks the chip; the features reach the block and SPI-session tests.
    Workspace {
        name: "daplink-stm32-embassy",
        path: "platform/daplink-stm32-embassy",
        features: &["block-swd", "spi-swd", "spi-block"],
        no_default_features: false,
    },
    // The dev-dependency picks the chip, as for `daplink-stm32-embassy`.
    Workspace {
        name: "daplink-stm32f1-embassy",
        path: "platform/daplink-stm32f1-embassy",
        features: &[],
        no_default_features: false,
    },
    // Both USB products use the host critical-section implementation and mock time driver;
    // `multi-packet` compiles the reassembly path its framing tests exercise.
    Workspace {
        name: "daplink-embassy-usb",
        path: "platform/daplink-embassy-usb",
        features: &["multi-packet"],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-embassy-usb-trace",
        path: "platform/daplink-embassy-usb",
        features: &["dap-trace", "usb-counters"],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-rp-embassy",
        path: "platform/daplink-rp-embassy",
        features: &["rp2040"],
        no_default_features: false,
    },
    Workspace {
        name: "dap-bench",
        path: "tools/dap-bench",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "flash-probe-rs",
        path: "tools/flash-probe-rs",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "probe-reboot",
        path: "tools/probe-reboot",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "vcom-bench",
        path: "tools/vcom-bench",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "swo-bench",
        path: "tools/swo-bench",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-test",
        path: "tests/daplink-test",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "daplink-test-protocol",
        path: "tests/crates/daplink-test-protocol",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "stm32-usart-brr",
        path: "tests/crates/stm32-usart-brr",
        features: &[],
        no_default_features: false,
    },
    Workspace {
        name: "xtask",
        path: ".",
        features: &[],
        no_default_features: false,
    },
];

/// A host-gated library linted again on one chip's target: a firmware's clippy skips path deps.
pub struct Cross {
    pub name: &'static str,
    pub path: &'static str,
    pub target: &'static str,
    pub features: &'static [&'static str],
}

pub const CROSS: &[Cross] = &[
    Cross {
        name: "daplink-rp-pio-rp2040",
        path: "platform/daplink-rp-pio",
        target: "thumbv6m-none-eabi",
        features: &["rp2040"],
    },
    Cross {
        name: "daplink-rp-pio-rp235x",
        path: "platform/daplink-rp-pio",
        target: "thumbv8m.main-none-eabihf",
        features: &["rp235x"],
    },
    Cross {
        name: "daplink-rp-dma-uart-rp2040",
        path: "platform/daplink-rp-dma-uart",
        target: "thumbv6m-none-eabi",
        features: &["rp2040"],
    },
    Cross {
        name: "daplink-rp-dma-uart-rp235x",
        path: "platform/daplink-rp-dma-uart",
        target: "thumbv8m.main-none-eabihf",
        features: &["rp235x"],
    },
    Cross {
        name: "daplink-rp-embassy-rp2040",
        path: "platform/daplink-rp-embassy",
        target: "thumbv6m-none-eabi",
        features: &["rp2040"],
    },
    Cross {
        name: "daplink-rp-embassy-rp2350",
        path: "platform/daplink-rp-embassy",
        target: "thumbv8m.main-none-eabihf",
        features: &["rp2350"],
    },
];

/// The `cargo doc` target filter covering a host row: `--bins` documents the library too.
// A binary named like its library shares its doc directory; cargo warns, and the check holds.
pub fn doc_filter(ws: &Workspace) -> Option<&'static str> {
    let packages = package_dirs(&crate::root().join(ws.path));
    let any = |rel: &str| packages.iter().any(|dir| dir.join(rel).exists());
    if any("src/main.rs") || any("src/bin") {
        Some("--bins")
    } else if any("src/lib.rs") {
        Some("--lib")
    } else {
        None
    }
}

/// A row's package directories: its own, or its virtual workspace's members.
pub fn package_dirs(dir: &Path) -> Vec<PathBuf> {
    let manifest: toml::Table = std::fs::read_to_string(dir.join("Cargo.toml"))
        .ok()
        .and_then(|text| text.parse().ok())
        .unwrap_or_default();
    if manifest.contains_key("package") {
        return vec![dir.to_path_buf()];
    }
    manifest
        .get("workspace")
        .and_then(|w| w.get("members"))
        .and_then(|m| m.as_array())
        .into_iter()
        .flatten()
        .filter_map(|m| m.as_str())
        .map(|m| dir.join(m))
        .collect()
}

/// The chip a library's docs need, which its host row gets from a dev-dependency.
pub const DOC_CHIPS: &[(&str, &str)] = &[
    (
        "platform/daplink-stm32-embassy",
        "embassy-stm32/stm32h523ce",
    ),
    (
        "platform/daplink-stm32f1-embassy",
        "embassy-stm32/stm32f103cb",
    ),
];

/// Embedded workspaces, derived from the firmware families.
pub fn embedded() -> impl Iterator<Item = Workspace> {
    crate::registry::FAMILIES.iter().map(|family| {
        let path = family.path;
        Workspace {
            name: path.rsplit('/').next().unwrap_or(path),
            path,
            features: &[],
            no_default_features: false,
        }
    })
}

/// Each reason states what actually stops the host gate, from a build that failed.
pub const EXCLUDED: &[Excluded] = &[];

/// Crate directories under `dir` (relative to `rust/`), skipping build-artifact trees.
#[cfg(test)]
pub fn crate_dirs(dir: &Path, rel: &str, out: &mut Vec<String>) {
    for entry in std::fs::read_dir(dir).unwrap() {
        let entry = entry.unwrap();
        let name = entry.file_name().to_string_lossy().into_owned();
        let path = entry.path();
        if path.is_dir() {
            if name == "target" || name.starts_with('.') {
                continue;
            }
            let rel = if rel.is_empty() {
                name
            } else {
                format!("{rel}/{name}")
            };
            crate_dirs(&path, &rel, out);
        } else if name == "Cargo.toml" {
            out.push(if rel.is_empty() {
                ".".to_string()
            } else {
                rel.to_string()
            });
        }
    }
}
