//! Synthetic test-fixture target: a fabricated layout for host tests, describing no real device.

use daplink_flash::identity::IdentityProbe;

use crate::algorithms::synthetic::SYNTHETIC_ALGO;
use crate::{FlashRegion, RamRegion, SectorInfo, Target};

/// Fabricated UF2 family id for the synthetic fixture (not any real Microsoft-registered family).
pub const SYNTHETIC_UF2_FAMILY: u32 = 0x0F0F_F0F0;

/// Fabricated identity probe: one made-up signature at a fake address, describing no real register.
static IDENTITY_PROBES: [IdentityProbe; 1] =
    [IdentityProbe::new(0x2000_0000, 0xFFFF_FFFF, 0xDEAD_BEEF)];

/// Fabricated flash region: 16 KiB at a fictional flash base, 1 KiB sectors.
static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
    start: 0x0800_0000,
    end: 0x0800_4000,
    is_default: true,
    is_secure: false,
    algo: &SYNTHETIC_ALGO,
}];

/// Fabricated RAM region matching the algorithm's fake RAM window.
static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
    start: 0x2000_0000,
    end: 0x2000_2000,
}];

/// Fabricated sector map: sixteen 1 KiB sectors covering the flash region.
static SECTORS: [SectorInfo; 1] = [SectorInfo {
    start: 0x0800_0000,
    size: 0x400,
    count: 16,
}];

/// Synthetic test-fixture target; not a real device.
#[derive(Clone, Copy, Debug, Default)]
pub struct SyntheticTarget;

impl Target for SyntheticTarget {
    fn name(&self) -> &'static str {
        "synthetic"
    }
    fn flash_regions(&self) -> &[FlashRegion<'_>] {
        &FLASH_REGIONS
    }

    fn ram_regions(&self) -> &[RamRegion] {
        &RAM_REGIONS
    }

    fn sectors(&self) -> &[SectorInfo] {
        &SECTORS
    }

    fn identity_probes(&self) -> &[IdentityProbe] {
        &IDENTITY_PROBES
    }

    fn uf2_family(&self) -> Option<u32> {
        Some(SYNTHETIC_UF2_FAMILY)
    }
}
