//! Target description for the Nordic nRF52840 (1 MiB internal NVMC flash, 256 KiB RAM).

use daplink_flash::identity::IdentityProbe;
use daplink_flash::reset_sequence::ResetSequence;
use daplink_flash::target::ResetType;

use crate::algorithms::nrf52::NRF52_ALGO;
use crate::families::nrf52::{info_part_probe, CODEPAGESIZE_PROBE, NRF52_RESET_SEQUENCE};
use crate::{FlashRegion, RamRegion, SectorInfo, Target};

/// `FICR.INFO.PART` on every nRF52840.
pub const NRF52840_INFO_PART: u32 = 0x0005_2840;

/// Official Microsoft UF2 family id for the nRF52840 (`uf2families.json`, "Nordic NRF52840").
pub const NRF52840_UF2_FAMILY: u32 = 0xADA5_2840;

/// Identity: an exact `FICR.INFO.PART` match, then the 4 KiB `FICR.CODEPAGESIZE`.
pub static IDENTITY_PROBES: [IdentityProbe; 2] =
    [info_part_probe(NRF52840_INFO_PART), CODEPAGESIZE_PROBE];

/// Flash (programmable) regions of the device: 1 MiB of code flash at `0x0`.
pub static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
    start: 0x0000_0000,
    end: 0x0010_0000,
    is_default: true,
    algo: &NRF52_ALGO,
}];

/// RAM regions of the device (256 KiB Data RAM from the SRAM base).
pub static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
    start: 0x2000_0000,
    end: 0x2004_0000,
}];

/// Sector layout: uniform 4 KiB pages.
pub static SECTORS: [SectorInfo; 1] = [SectorInfo {
    start: 0x0000_0000,
    size: 0x0000_1000,
    count: 256,
}];

/// Page-staging scratch size: the algorithm's program buffer, one 4 KiB page.
pub const PROGRAM_BUFFER_SIZE: usize = NRF52_ALGO.program_buffer_size as usize;

/// The target description type for this device.
#[derive(Clone, Copy, Debug, Default)]
pub struct NRF52840;

impl Target for NRF52840 {
    fn name(&self) -> &'static str {
        "nRF52840"
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
    fn reset_type(&self) -> ResetType {
        ResetType::SoftwareSysReset
    }
    fn reset_sequence(&self) -> &dyn ResetSequence {
        &NRF52_RESET_SEQUENCE
    }
    fn identity_probes(&self) -> &[IdentityProbe] {
        &IDENTITY_PROBES
    }
    fn uf2_family(&self) -> Option<u32> {
        Some(NRF52840_UF2_FAMILY)
    }
}
