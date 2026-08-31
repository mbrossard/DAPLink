//! The firmware registry: the one table that knows every image in the tree.

/// One firmware image: its name, crate path, target triple, probe-rs chip, HIL rig profile and
/// feature shape. Rows sharing a crate path get distinct build directories from `name`, or their
/// builds evict.
pub struct Firmware {
    pub name: &'static str,
    pub path: &'static str,
    pub target: &'static str,
    pub chip: &'static str,
    /// The HIL rig profile a PUT is flashed for; `None` for a DUT image, which has no rig
    /// profile of its own.
    pub profile: Option<&'static str>,
    pub features: &'static [&'static str],
    pub no_default_features: bool,
    /// The uf2 family id to tag a build's flash image with, for rows a drag-n-drop bootloader
    /// (or the flash test harness) accepts as uf2. `None` for rows that never emit one.
    pub uf2_family: Option<u32>,
    /// How `flash` reaches this row's PUT: a probe-rs transport, or the ROM UF2 bootloader.
    pub bootloader: Bootloader,
}

/// How a firmware row is flashed onto its running board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bootloader {
    /// The default: `probe-rs download` over the `--probe` transport selector.
    Probe,
    /// The board is its own PUT: `0x9F` into the ROM UF2 bootloader, then the BOOTSEL drive.
    /// `--probe` names the PUT's own selector, not a transport.
    BootselUf2,
}

/// uf2 family id for RP2040 targets (rp2040.us: "RP2040 (absolute, single flash bank)").
pub const UF2_FAMILY_RP2040: u32 = 0xe48b_ff56;
/// uf2 family id for RP2350 targets (Arm secure image).
pub const UF2_FAMILY_RP2350: u32 = 0xe48b_ff59;
/// uf2 family id for STM32F401 targets, shared by the whole STM32F4 family.
pub const UF2_FAMILY_STM32F4: u32 = 0x5775_5a57;

impl Firmware {
    /// Where this row builds, relative to `rust/`: one subdirectory of the tree's single
    /// `target/`, named for the row, so rows sharing a crate path never share a directory.
    pub fn target_dir(&self) -> String {
        format!("target/{}", self.name)
    }

    /// The built ELF, relative to `rust/`: the binary is named after the crate directory.
    pub fn elf(&self) -> String {
        let name = self.path.rsplit('/').next().unwrap_or(self.path);
        format!("{}/{}/release/{}", self.target_dir(), self.target, name)
    }
}

pub const FIRMWARE: &[Firmware] = &[
    Firmware {
        name: "firmware/rp2040-pico-embassy",
        path: "firmware/rp2040-pico-embassy",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &[],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "firmware/rp2040-pico-embassy-overclock-200mhz",
        path: "firmware/rp2040-pico-embassy",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["overclock-200mhz"],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::Probe,
    },
    // The same crate with `bitbang` overriding the accelerated default, so an A/B against the
    // row above isolates the wire engine alone.
    Firmware {
        name: "firmware/rp2040-pico-embassy-bitbang",
        path: "firmware/rp2040-pico-embassy",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["bitbang"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same shape targeting an F401, which is what front 9's DUT is: the row that lets the
    // mass-storage drive be exercised end to end, programming included.
    Firmware {
        name: "firmware/rp2040-pico-embassy-flash-f401",
        path: "firmware/rp2040-pico-embassy",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["flash", "flash-stm32f401"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the mass-storage drive and vendor flash on, targeting the rig's RP2040.
    Firmware {
        name: "firmware/rp2040-pico-embassy-flash",
        path: "firmware/rp2040-pico-embassy",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["flash", "flash-rp2040"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the carrier's five-LED debug bank on.
    Firmware {
        name: "firmware/rp2040-pico-embassy-leds",
        path: "firmware/rp2040-pico-embassy",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["carrier-leds"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with RTT-backed defmt logging on, so build-all covers it.
    Firmware {
        name: "firmware/rp2040-pico-embassy-defmt",
        path: "firmware/rp2040-pico-embassy",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["defmt"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The Raspberry Pi Debug Probe: SWD only, and no debug header, so it takes the row's uf2.
    Firmware {
        name: "firmware/debugprobe",
        path: "firmware/debugprobe",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-debugprobe"),
        features: &[],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::BootselUf2,
    },
    Firmware {
        name: "firmware/debugprobe-overclock-180mhz",
        path: "firmware/debugprobe",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-debugprobe"),
        features: &["overclock-180mhz"],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::BootselUf2,
    },
    // The same crate with `bitbang` overriding the accelerated default.
    Firmware {
        name: "firmware/debugprobe-bitbang",
        path: "firmware/debugprobe",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-debugprobe"),
        features: &["bitbang"],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::BootselUf2,
    },
    Firmware {
        name: "firmware/rp2040-pico-rtic",
        path: "firmware/rp2040-pico-rtic",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &[],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "firmware/rp2040-pico-rtic-overclock-200mhz",
        path: "firmware/rp2040-pico-rtic",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["overclock-200mhz"],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::Probe,
    },
    // The same crate with `bitbang` overriding the accelerated default.
    Firmware {
        name: "firmware/rp2040-pico-rtic-bitbang",
        path: "firmware/rp2040-pico-rtic",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["bitbang"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the mass-storage drive and vendor flash on, targeting the rig's RP2040.
    Firmware {
        name: "firmware/rp2040-pico-rtic-flash",
        path: "firmware/rp2040-pico-rtic",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["flash", "flash-rp2040"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The flash row with `bitbang` overriding the accelerated default.
    Firmware {
        name: "firmware/rp2040-pico-rtic-flash-bitbang",
        path: "firmware/rp2040-pico-rtic",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["bitbang", "flash", "flash-rp2040"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the carrier's four wired debug-bank LEDs on.
    Firmware {
        name: "firmware/rp2040-pico-rtic-leds",
        path: "firmware/rp2040-pico-rtic",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["carrier-leds"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with RTT-backed defmt logging on, so build-all covers it.
    Firmware {
        name: "firmware/rp2040-pico-rtic-defmt",
        path: "firmware/rp2040-pico-rtic",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: Some("rp2040-pico"),
        features: &["defmt"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "firmware/rp2350-pico2-embassy",
        path: "firmware/rp2350-pico2-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "RP235x",
        profile: Some("rp2350-pico2"),
        features: &[],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2350),
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "firmware/rp2350-pico2-embassy-overclock-200mhz",
        path: "firmware/rp2350-pico2-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "RP235x",
        profile: Some("rp2350-pico2"),
        features: &["overclock-200mhz"],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2350),
        bootloader: Bootloader::Probe,
    },
    // The same crate with `bitbang` overriding the accelerated default.
    Firmware {
        name: "firmware/rp2350-pico2-embassy-bitbang",
        path: "firmware/rp2350-pico2-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "RP235x",
        profile: Some("rp2350-pico2"),
        features: &["bitbang"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the mass-storage drive and vendor flash on, targeting the rig's RP2350.
    Firmware {
        name: "firmware/rp2350-pico2-embassy-flash",
        path: "firmware/rp2350-pico2-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "RP235x",
        profile: Some("rp2350-pico2"),
        features: &["flash", "flash-rp2350"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the carrier's five-LED debug bank on.
    Firmware {
        name: "firmware/rp2350-pico2-embassy-leds",
        path: "firmware/rp2350-pico2-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "RP235x",
        profile: Some("rp2350-pico2"),
        features: &["carrier-leds"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with RTT-backed defmt logging on, so build-all covers it.
    Firmware {
        name: "firmware/rp2350-pico2-embassy-defmt",
        path: "firmware/rp2350-pico2-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "RP235x",
        profile: Some("rp2350-pico2"),
        features: &["defmt"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // probe-rs auto-detect does not identify this part, so the chip name is load-bearing.
    Firmware {
        name: "firmware/stm32h523-embassy",
        path: "firmware/stm32h523-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "STM32H523CE",
        profile: Some("stm32h523"),
        features: &[],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the SPI-session wire backend on, for A/B against the row above.
    Firmware {
        name: "firmware/stm32h523-embassy-accel",
        path: "firmware/stm32h523-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "STM32H523CE",
        profile: Some("stm32h523"),
        features: &["accel"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the mass-storage drive and vendor flash on, targeting the rig's STM32F4.
    Firmware {
        name: "firmware/stm32h523-embassy-flash",
        path: "firmware/stm32h523-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "STM32H523CE",
        profile: Some("stm32h523"),
        features: &["flash", "flash-stm32f401"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with RTT-backed defmt logging on, so build-all covers it.
    Firmware {
        name: "firmware/stm32h523-embassy-defmt",
        path: "firmware/stm32h523-embassy",
        target: "thumbv8m.main-none-eabihf",
        chip: "STM32H523CE",
        profile: Some("stm32h523"),
        features: &["defmt"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The NXP MCU-Link; probe-rs auto-detect misses this part, so the chip name is load-bearing.
    Firmware {
        name: "firmware/mculink",
        path: "firmware/mculink",
        target: "thumbv8m.main-none-eabihf",
        chip: "LPC55S69JBD64",
        profile: Some("mculink"),
        features: &[],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The accelerated engine on the alternate 96 MHz FRO profile.
    Firmware {
        name: "firmware/mculink-96mhz",
        path: "firmware/mculink",
        target: "thumbv8m.main-none-eabihf",
        chip: "LPC55S69JBD64",
        profile: Some("mculink"),
        features: &["core-96mhz"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The bit-bang engine on the default 150 MHz core profile.
    Firmware {
        name: "firmware/mculink-bitbang",
        path: "firmware/mculink",
        target: "thumbv8m.main-none-eabihf",
        chip: "LPC55S69JBD64",
        profile: Some("mculink"),
        features: &["bitbang", "guard"],
        no_default_features: true,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The bit-bang engine crossed with the alternate 96 MHz FRO profile.
    Firmware {
        name: "firmware/mculink-bitbang-96mhz",
        path: "firmware/mculink",
        target: "thumbv8m.main-none-eabihf",
        chip: "LPC55S69JBD64",
        profile: Some("mculink"),
        features: &["bitbang", "core-96mhz", "guard"],
        no_default_features: true,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with RTT-backed defmt logging on, so build-all covers it.
    Firmware {
        name: "firmware/mculink-defmt",
        path: "firmware/mculink",
        target: "thumbv8m.main-none-eabihf",
        chip: "LPC55S69JBD64",
        profile: Some("mculink"),
        features: &["defmt"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the mass-storage drive and vendor flash on, targeting the rig's STM32F4.
    Firmware {
        name: "firmware/mculink-flash",
        path: "firmware/mculink",
        target: "thumbv8m.main-none-eabihf",
        chip: "LPC55S69JBD64",
        profile: Some("mculink"),
        features: &["flash", "flash-stm32f401"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The same crate with the high-speed chirp retry on; no benched row enables it yet, so
    // this row alone keeps it under build-all.
    Firmware {
        name: "firmware/mculink-hs-retry",
        path: "firmware/mculink",
        target: "thumbv8m.main-none-eabihf",
        chip: "LPC55S69JBD64",
        profile: Some("mculink"),
        features: &["usb-hs-retry"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "tests/firmwares/test-stm32f401",
        path: "tests/firmwares/test-stm32f401",
        target: "thumbv7em-none-eabihf",
        chip: "STM32F401CEUx",
        profile: None,
        features: &[],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_STM32F4),
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "tests/firmwares/test-rp2040",
        path: "tests/firmwares/test-rp2040",
        target: "thumbv6m-none-eabi",
        chip: "RP2040",
        profile: None,
        features: &[],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2040),
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "firmware/stm32f103-embassy",
        path: "firmware/stm32f103-embassy",
        target: "thumbv7m-none-eabi",
        chip: "STM32F103CB",
        profile: Some("stm32f103"),
        features: &[],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The nanoDAP V2.3 wiring: PB15 D+ switch, PC13 LED, JTAG with TDO on PA5, no SWO route.
    Firmware {
        name: "firmware/stm32f103-embassy-nanodap",
        path: "firmware/stm32f103-embassy",
        target: "thumbv7m-none-eabi",
        chip: "STM32F103CB",
        profile: Some("stm32f103-nanodap"),
        features: &["nanodap"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The nanoDAP build with the SPI-session SWD backend on, for A/B against the row above.
    Firmware {
        name: "firmware/stm32f103-embassy-accel",
        path: "firmware/stm32f103-embassy",
        target: "thumbv7m-none-eabi",
        chip: "STM32F103CB",
        profile: Some("stm32f103-nanodap"),
        features: &["nanodap", "accel"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The nanoDAP build with the mass-storage drive and vendor flash on, targeting the STM32F4.
    Firmware {
        name: "firmware/stm32f103-embassy-flash",
        path: "firmware/stm32f103-embassy",
        target: "thumbv7m-none-eabi",
        chip: "STM32F103CB",
        profile: Some("stm32f103-nanodap"),
        features: &["nanodap", "flash", "flash-stm32f401"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // The nanoDAP build with RTT-backed defmt logging on, so build-all covers it.
    Firmware {
        name: "firmware/stm32f103-embassy-defmt",
        path: "firmware/stm32f103-embassy",
        target: "thumbv7m-none-eabi",
        chip: "STM32F103CB",
        profile: Some("stm32f103-nanodap"),
        features: &["nanodap", "defmt"],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    // Original ST-Link/V2 hardware on the polled stack; no benched rig carries this profile yet.
    Firmware {
        name: "firmware/stlinkv2",
        path: "firmware/stlinkv2",
        target: "thumbv7m-none-eabi",
        chip: "STM32F103C8",
        profile: Some("stlinkv2"),
        features: &[],
        no_default_features: false,
        uf2_family: None,
        bootloader: Bootloader::Probe,
    },
    Firmware {
        name: "tests/firmwares/test-rp2350",
        path: "tests/firmwares/test-rp2350",
        target: "thumbv8m.main-none-eabihf",
        chip: "RP235x",
        profile: None,
        features: &[],
        no_default_features: false,
        uf2_family: Some(UF2_FAMILY_RP2350),
        bootloader: Bootloader::Probe,
    },
];

/// One test target: its short name and DUT firmware row; the harness owns chip and wiring.
pub struct Target {
    pub name: &'static str,
    pub dut: &'static str,
}

pub const TARGETS: &[Target] = &[
    Target {
        name: "stm32f401",
        dut: "tests/firmwares/test-stm32f401",
    },
    Target {
        name: "rp2040",
        dut: "tests/firmwares/test-rp2040",
    },
    Target {
        name: "rp2350",
        dut: "tests/firmwares/test-rp2350",
    },
];
