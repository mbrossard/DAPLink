//! The gate registry: every workspace under `rust/` and the checks that reach it. A crate in no
//! list fails the closure test, so a new directory must take a disposition the day it lands.

/// One gated workspace: its label, its own workspace directory relative to `rust/`, and the
/// features `clippy`/`test` enable there; an empty slice runs with default features.
pub struct Workspace {
    pub name: &'static str,
    pub path: &'static str,
    pub features: &'static [&'static str],
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
    },
    Workspace {
        name: "daplink-decoder",
        path: "crates/daplink-decoder",
        features: &[],
    },
    Workspace {
        name: "daplink-spi-swd",
        path: "crates/daplink-spi-swd",
        features: &[],
    },
    // `std` publishes the host mock the crate's own tests reach through `cfg(test)`; on the row
    // it is the only thing that compiles the arm an out-of-tree consumer gets.
    Workspace {
        name: "daplink-flash",
        path: "crates/daplink-flash",
        features: &["std"],
    },
    Workspace {
        name: "daplink-targets",
        path: "crates/daplink-targets",
        features: &["synthetic", "rp2040", "rp2350", "stm32f401ce"],
    },
    Workspace {
        name: "daplink-vfs",
        path: "crates/daplink-vfs",
        features: &[],
    },
    // `guard` is the crate's one host-visible feature; enabling it exercises the guarded path.
    Workspace {
        name: "daplink-rs",
        path: "crates/daplink-rs",
        features: &["guard"],
    },
    Workspace {
        name: "daplink-probe-rs",
        path: "crates/daplink-probe-rs",
        features: &[],
    },
    // The PL011 divisor arithmetic `daplink-rp-hal` and both RP DUT images call; pure, so its
    // tests run here.
    Workspace {
        name: "pl011-baud",
        path: "crates/pl011-baud",
        features: &[],
    },
    // `flash` composes the mass-storage class; the default build is DAP+VCOM only.
    Workspace {
        name: "daplink-usb-device",
        path: "platform/daplink-usb-device",
        features: &["flash"],
    },
    // Chip-agnostic by default: without a chip feature the runtime-neutral half host-tests.
    Workspace {
        name: "daplink-rp-pio",
        path: "platform/daplink-rp-pio",
        features: &[],
    },
    // The receive cursor's ring arithmetic is pure and host-tested here.
    Workspace {
        name: "daplink-rp-dma-uart",
        path: "platform/daplink-rp-dma-uart",
        features: &[],
    },
    // `carrier-leds` is the crate's one feature and host-builds like the rest of it. The
    // unique-id read is behind a target cfg, since rp2040-flash's ARM inline asm has no host
    // build, and the row overrides the crate's own thumbv6m pin so its tests run rather than
    // cross-compile.
    Workspace {
        name: "daplink-rp-hal",
        path: "platform/daplink-rp-hal",
        features: &["carrier-leds"],
    },
    // Neither of the next two crates carries a test, so the row buys lint coverage only. Both
    // host-build; what kept them out was the embedded target their own `.cargo/config.toml`
    // pins, which the row's host `--target` overrides.
    Workspace {
        name: "daplink-stm32f1-hal",
        path: "platform/daplink-stm32f1-hal",
        features: &[],
    },
    // The default feature shape compiles the PLL0 clock arm and the masked engine together.
    Workspace {
        name: "daplink-lpc55-hal",
        path: "platform/daplink-lpc55-hal",
        features: &["masked-swd", "probe-timing"],
    },
    // `flash` composes the mass-storage class, as on the polled adapter; the crate's
    // dev-dependencies supply the critical-section impl and the mock time driver its tests need.
    Workspace {
        name: "daplink-embassy-usb",
        path: "platform/daplink-embassy-usb",
        features: &["flash"],
    },
    // Its divisor arithmetic is pure and host-tested; no chip feature is needed to reach it.
    Workspace {
        name: "daplink-rp-embassy",
        path: "platform/daplink-rp-embassy",
        features: &[],
    },
    Workspace {
        name: "dap-bench",
        path: "tools/dap-bench",
        features: &[],
    },
    Workspace {
        name: "flash-probe-rs",
        path: "tools/flash-probe-rs",
        features: &[],
    },
    Workspace {
        name: "probe-reboot",
        path: "tools/probe-reboot",
        features: &[],
    },
    Workspace {
        name: "vcom-bench",
        path: "tools/vcom-bench",
        features: &[],
    },
    Workspace {
        name: "swo-bench",
        path: "tools/swo-bench",
        features: &[],
    },
    Workspace {
        name: "daplink-test",
        path: "tests/daplink-test",
        features: &[],
    },
    Workspace {
        name: "daplink-test-protocol",
        path: "tests/crates/daplink-test-protocol",
        features: &[],
    },
    Workspace {
        name: "stm32-usart-brr",
        path: "tests/crates/stm32-usart-brr",
        features: &[],
    },
    // The root workspace: only xtask itself resolves here.
    Workspace {
        name: "xtask",
        path: ".",
        features: &[],
    },
];

/// Embedded workspaces: `fmt --check` and `clippy` build for the crate's own pinned target, and
/// no test target exists there, so `cargo test` is never run.
pub const EMBEDDED: &[Workspace] = &[
    Workspace {
        name: "debugprobe",
        path: "firmware/debugprobe",
        features: &[],
    },
    Workspace {
        name: "mculink",
        path: "firmware/mculink",
        features: &[],
    },
    Workspace {
        name: "rp2040-pico-embassy",
        path: "firmware/rp2040-pico-embassy",
        features: &[],
    },
    Workspace {
        name: "rp2040-pico-rtic",
        path: "firmware/rp2040-pico-rtic",
        features: &[],
    },
    Workspace {
        name: "rp2350-pico2-embassy",
        path: "firmware/rp2350-pico2-embassy",
        features: &[],
    },
    Workspace {
        name: "stlinkv2",
        path: "firmware/stlinkv2",
        features: &[],
    },
    Workspace {
        name: "stm32f103-embassy",
        path: "firmware/stm32f103-embassy",
        features: &[],
    },
    Workspace {
        name: "stm32h523-embassy",
        path: "firmware/stm32h523-embassy",
        features: &[],
    },
    Workspace {
        name: "test-rp2040",
        path: "tests/firmwares/test-rp2040",
        features: &[],
    },
    Workspace {
        name: "test-rp2350",
        path: "tests/firmwares/test-rp2350",
        features: &[],
    },
    Workspace {
        name: "test-stm32f401",
        path: "tests/firmwares/test-stm32f401",
        features: &[],
    },
];

/// Each reason states what actually stops the host gate, not what the dependency list suggests:
/// an exclusion nobody can check is how a crate stays unreached, and every reason here must be
/// written from a build that failed.
pub const EXCLUDED: &[Excluded] = &[
    Excluded {
        path: "platform/daplink-stm32-embassy",
        reason: "pulls embassy-stm32, which needs a firmware-selected chip feature",
    },
    Excluded {
        path: "platform/daplink-stm32f1-embassy",
        reason: "pulls embassy-stm32, which needs a firmware-selected chip feature",
    },
];
