//! The firmware registry: the one table that knows every image in the tree.

/// How a firmware row is flashed onto its running board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bootloader {
    /// The default: `probe-rs download` over the `--via` transport selector.
    Probe,
    /// The board is its own PUT: `0x9F` into the ROM UF2 bootloader; `--put` names the PUT.
    BootselUf2,
}

/// Properties shared by every build variant of one firmware crate.
pub struct FirmwareFamily {
    pub path: &'static str,
    target: &'static str,
    chip: &'static str,
    profile: Option<&'static str>,
    uf2_family: Option<u32>,
    bootloader: Bootloader,
}

/// Properties that distinguish one build of a firmware crate.
pub struct FirmwareVariant {
    name: &'static str,
    profile: Option<&'static str>,
    features: &'static [&'static str],
    no_default_features: bool,
    uf2_family: Option<Option<u32>>,
}

impl FirmwareVariant {
    pub const fn new(name: &'static str) -> Self {
        Self {
            name,
            profile: None,
            features: &[],
            no_default_features: false,
            uf2_family: None,
        }
    }

    pub const fn profile(mut self, profile: &'static str) -> Self {
        self.profile = Some(profile);
        self
    }

    pub const fn features(mut self, features: &'static [&'static str]) -> Self {
        self.features = features;
        self
    }

    pub const fn without_default_features(mut self) -> Self {
        self.no_default_features = true;
        self
    }

    pub const fn without_uf2(mut self) -> Self {
        self.uf2_family = Some(None);
        self
    }
}

/// One resolved firmware image; rows sharing a crate path build in directories named by `name`.
pub struct Firmware {
    pub name: &'static str,
    pub path: &'static str,
    pub target: &'static str,
    pub chip: &'static str,
    /// The HIL profile a PUT is flashed for; `None` for a DUT image.
    pub profile: Option<&'static str>,
    pub features: &'static [&'static str],
    pub no_default_features: bool,
    /// The uf2 family id for rows that emit a uf2 image.
    pub uf2_family: Option<u32>,
    /// How `flash` reaches this row's PUT.
    pub bootloader: Bootloader,
}

/// uf2 family id for RP2040 targets.
pub const UF2_FAMILY_RP2040: u32 = 0xe48b_ff56;
/// uf2 family id for RP2350 targets (Arm secure image).
pub const UF2_FAMILY_RP2350: u32 = 0xe48b_ff59;
/// uf2 family id for STM32F401 targets, shared by the whole STM32F4 family.
pub const UF2_FAMILY_STM32F4: u32 = 0x5775_5a57;

impl FirmwareFamily {
    pub const fn variant(&self, variant: FirmwareVariant) -> Firmware {
        Firmware {
            name: variant.name,
            path: self.path,
            target: self.target,
            chip: self.chip,
            profile: match variant.profile {
                Some(profile) => Some(profile),
                None => self.profile,
            },
            features: variant.features,
            no_default_features: variant.no_default_features,
            uf2_family: match variant.uf2_family {
                Some(family) => family,
                None => self.uf2_family,
            },
            bootloader: self.bootloader,
        }
    }
}

impl Firmware {
    /// Where this row builds, relative to `rust/`.
    pub fn target_dir(&self) -> String {
        format!("target/{}", self.name)
    }

    /// Whether this is an RP probe whose UF2 its own ROM takes, so BOOTSEL can install it.
    pub fn rom_installable(&self) -> bool {
        self.profile.is_some()
            && matches!(self.uf2_family, Some(UF2_FAMILY_RP2040 | UF2_FAMILY_RP2350))
    }

    /// The built ELF, relative to `rust/`.
    pub fn elf(&self) -> String {
        let name = self.path.rsplit('/').next().unwrap_or(self.path);
        format!("{}/{}/release/{}", self.target_dir(), self.target, name)
    }
}

const RP2040_PICO_EMBASSY: FirmwareFamily = FirmwareFamily {
    path: "firmware/rp2040-pico-embassy",
    target: "thumbv6m-none-eabi",
    chip: "RP2040",
    profile: Some("rp2040-pico"),
    uf2_family: Some(UF2_FAMILY_RP2040),
    bootloader: Bootloader::Probe,
};
const DEBUGPROBE: FirmwareFamily = FirmwareFamily {
    path: "firmware/debugprobe",
    target: "thumbv6m-none-eabi",
    chip: "RP2040",
    profile: Some("rp2040-debugprobe"),
    uf2_family: Some(UF2_FAMILY_RP2040),
    bootloader: Bootloader::BootselUf2,
};
const RP2040_PICO_RTIC: FirmwareFamily = FirmwareFamily {
    path: "firmware/rp2040-pico-rtic",
    target: "thumbv6m-none-eabi",
    chip: "RP2040",
    profile: Some("rp2040-pico"),
    uf2_family: Some(UF2_FAMILY_RP2040),
    bootloader: Bootloader::Probe,
};
const RP2350_PICO2_EMBASSY: FirmwareFamily = FirmwareFamily {
    path: "firmware/rp2350-pico2-embassy",
    target: "thumbv8m.main-none-eabihf",
    chip: "RP235x",
    profile: Some("rp2350-pico2"),
    uf2_family: Some(UF2_FAMILY_RP2350),
    bootloader: Bootloader::Probe,
};
const STM32H523_EMBASSY: FirmwareFamily = FirmwareFamily {
    path: "firmware/stm32h523-embassy",
    target: "thumbv8m.main-none-eabihf",
    chip: "STM32H523CE",
    profile: Some("stm32h523"),
    uf2_family: None,
    bootloader: Bootloader::Probe,
};
const MCULINK: FirmwareFamily = FirmwareFamily {
    path: "firmware/mculink",
    target: "thumbv8m.main-none-eabihf",
    chip: "LPC55S69JBD64",
    profile: Some("mculink"),
    uf2_family: None,
    bootloader: Bootloader::Probe,
};
const TEST_STM32F401: FirmwareFamily = FirmwareFamily {
    path: "tests/firmwares/test-stm32f401",
    target: "thumbv7em-none-eabihf",
    chip: "STM32F401CEUx",
    profile: None,
    uf2_family: Some(UF2_FAMILY_STM32F4),
    bootloader: Bootloader::Probe,
};
const TEST_RP2040: FirmwareFamily = FirmwareFamily {
    path: "tests/firmwares/test-rp2040",
    target: "thumbv6m-none-eabi",
    chip: "RP2040",
    profile: None,
    uf2_family: Some(UF2_FAMILY_RP2040),
    bootloader: Bootloader::Probe,
};
const STM32F103_EMBASSY: FirmwareFamily = FirmwareFamily {
    path: "firmware/stm32f103-embassy",
    target: "thumbv7m-none-eabi",
    chip: "STM32F103CB",
    profile: Some("stm32f103"),
    uf2_family: None,
    bootloader: Bootloader::Probe,
};
const STLINKV2: FirmwareFamily = FirmwareFamily {
    path: "firmware/stlinkv2",
    target: "thumbv7m-none-eabi",
    chip: "STM32F103C8",
    profile: Some("stlinkv2"),
    uf2_family: None,
    bootloader: Bootloader::Probe,
};
const TEST_RP2350: FirmwareFamily = FirmwareFamily {
    path: "tests/firmwares/test-rp2350",
    target: "thumbv8m.main-none-eabihf",
    chip: "RP235x",
    profile: None,
    uf2_family: Some(UF2_FAMILY_RP2350),
    bootloader: Bootloader::Probe,
};

/// Every embedded crate, in gate display order.
pub const FAMILIES: &[&FirmwareFamily] = &[
    &DEBUGPROBE,
    &MCULINK,
    &RP2040_PICO_EMBASSY,
    &RP2040_PICO_RTIC,
    &RP2350_PICO2_EMBASSY,
    &STLINKV2,
    &STM32F103_EMBASSY,
    &STM32H523_EMBASSY,
    &TEST_RP2040,
    &TEST_RP2350,
    &TEST_STM32F401,
];

pub const FIRMWARE: &[Firmware] = &[
    RP2040_PICO_EMBASSY.variant(FirmwareVariant::new("firmware/rp2040-pico-embassy")),
    RP2040_PICO_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2040-pico-embassy-overclock-200mhz")
            .features(&["overclock-200mhz", "dap-packet-512"]),
    ),
    RP2040_PICO_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2040-pico-embassy-bitbang")
            .without_default_features()
            .without_uf2(),
    ),
    RP2040_PICO_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2040-pico-embassy-flash-f401")
            .features(&["flash", "flash-stm32f401"])
            .without_uf2(),
    ),
    RP2040_PICO_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2040-pico-embassy-flash")
            .features(&["flash", "flash-rp2040"])
            .without_uf2(),
    ),
    DEBUGPROBE.variant(FirmwareVariant::new("firmware/debugprobe")),
    DEBUGPROBE.variant(
        FirmwareVariant::new("firmware/debugprobe-overclock-180mhz")
            .features(&["overclock-180mhz", "dap-packet-512"]),
    ),
    DEBUGPROBE
        .variant(FirmwareVariant::new("firmware/debugprobe-bitbang").without_default_features()),
    RP2040_PICO_RTIC.variant(FirmwareVariant::new("firmware/rp2040-pico-rtic")),
    RP2040_PICO_RTIC.variant(
        FirmwareVariant::new("firmware/rp2040-pico-rtic-overclock-200mhz")
            .features(&["overclock-200mhz"]),
    ),
    RP2040_PICO_RTIC.variant(
        FirmwareVariant::new("firmware/rp2040-pico-rtic-bitbang")
            .without_default_features()
            .without_uf2(),
    ),
    RP2040_PICO_RTIC.variant(
        FirmwareVariant::new("firmware/rp2040-pico-rtic-flash")
            .features(&["flash", "flash-rp2040"])
            .without_uf2(),
    ),
    RP2040_PICO_RTIC.variant(
        FirmwareVariant::new("firmware/rp2040-pico-rtic-flash-bitbang")
            .features(&["flash", "flash-rp2040"])
            .without_default_features()
            .without_uf2(),
    ),
    RP2350_PICO2_EMBASSY.variant(FirmwareVariant::new("firmware/rp2350-pico2-embassy")),
    RP2350_PICO2_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2350-pico2-embassy-overclock-200mhz")
            .features(&["overclock-200mhz", "dap-packet-512"]),
    ),
    RP2350_PICO2_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2350-pico2-embassy-bitbang")
            .without_default_features()
            .without_uf2(),
    ),
    RP2350_PICO2_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2350-pico2-embassy-flash-f401")
            .features(&["flash", "flash-stm32f401"])
            .without_uf2(),
    ),
    RP2350_PICO2_EMBASSY.variant(
        FirmwareVariant::new("firmware/rp2350-pico2-embassy-flash")
            .features(&["flash", "flash-rp2350"])
            .without_uf2(),
    ),
    STM32H523_EMBASSY.variant(FirmwareVariant::new("firmware/stm32h523-embassy")),
    STM32H523_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32h523-embassy-pkt512").features(&["dap-packet-512"]),
    ),
    STM32H523_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32h523-embassy-accel").features(&[
            "accel",
            "spi-block",
            "dap-packet-512",
        ]),
    ),
    STM32H523_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32h523-embassy-accel-duplex").features(&[
            "accel",
            "accel-duplex",
            "dap-packet-512",
        ]),
    ),
    STM32H523_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32h523-embassy-flash").features(&[
            "flash",
            "flash-stm32f401",
            "dap-packet-512",
        ]),
    ),
    STM32H523_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32h523-embassy-accel-flash").features(&[
            "accel",
            "flash",
            "flash-stm32f401",
            "dap-packet-512",
        ]),
    ),
    STM32H523_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32h523-embassy-plain").features(&["plain-bitbang"]),
    ),
    MCULINK.variant(FirmwareVariant::new("firmware/mculink")),
    MCULINK.variant(FirmwareVariant::new("firmware/mculink-96mhz").features(&["core-96mhz"])),
    MCULINK.variant(
        FirmwareVariant::new("firmware/mculink-bitbang")
            .features(&["bitbang"])
            .without_default_features(),
    ),
    MCULINK.variant(
        FirmwareVariant::new("firmware/mculink-bitbang-96mhz")
            .features(&["bitbang", "core-96mhz"])
            .without_default_features(),
    ),
    MCULINK.variant(
        FirmwareVariant::new("firmware/mculink-flash").features(&["flash", "flash-stm32f401"]),
    ),
    TEST_STM32F401.variant(FirmwareVariant::new("tests/firmwares/test-stm32f401")),
    TEST_STM32F401.variant(
        FirmwareVariant::new("tests/firmwares/test-stm32f401-conntest").features(&["conntest"]),
    ),
    TEST_RP2040.variant(FirmwareVariant::new("tests/firmwares/test-rp2040")),
    STM32F103_EMBASSY.variant(FirmwareVariant::new("firmware/stm32f103-embassy")),
    STM32F103_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32f103-embassy-nanodap")
            .profile("stm32f103-nanodap")
            .features(&["nanodap"]),
    ),
    STM32F103_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32f103-embassy-accel")
            .profile("stm32f103-nanodap")
            .features(&["nanodap", "accel"]),
    ),
    STM32F103_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32f103-embassy-flash")
            .profile("stm32f103-nanodap")
            .features(&["nanodap", "flash", "flash-stm32f401"]),
    ),
    STM32F103_EMBASSY.variant(
        FirmwareVariant::new("firmware/stm32f103-embassy-accel-flash")
            .profile("stm32f103-nanodap")
            .features(&["nanodap", "accel", "flash", "flash-stm32f401"]),
    ),
    STLINKV2.variant(FirmwareVariant::new("firmware/stlinkv2")),
    STLINKV2.variant(
        FirmwareVariant::new("firmware/stlinkv2-accel")
            .profile("stlinkv2")
            .features(&["accel"]),
    ),
    STLINKV2.variant(
        FirmwareVariant::new("firmware/stlinkv2-conntest")
            .profile("stlinkv2-conntest")
            .features(&["conntest"]),
    ),
    TEST_RP2350.variant(FirmwareVariant::new("tests/firmwares/test-rp2350")),
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
