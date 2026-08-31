//! Target description for the STMicroelectronics STM32F401CE (512 KiB flash, 96 KiB RAM).

use daplink_flash::identity::IdentityProbe;
use daplink_flash::target::ResetType;

use crate::algorithms::stm32f4::STM32F4_ALGO;
use crate::{FlashRegion, RamRegion, SectorInfo, Target};

/// Official Microsoft UF2 family id for the STM32F4 family (`uf2families.json`, "ST STM32F401").
pub const STM32F4_UF2_FAMILY: u32 = 0x5775_5A57;

/// Identity probe: `DBGMCU_IDCODE` at `0xE004_2000` (RM0368 8.6.1). The mask is deliberately
/// `0x0FFD`, clearing `DEV_ID` bit 1 so both F401xD/xE (`0x433`) and F411 (`0x431`) pass.
pub static IDENTITY_PROBES: [IdentityProbe; 1] =
    [IdentityProbe::new(0xE004_2000, 0x0000_0FFD, 0x0000_0431)];

/// Flash (programmable) regions of the device: 512 KiB at the STM32 flash base.
pub static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
    start: 0x0800_0000,
    end: 0x0808_0000,
    is_default: true,
    is_secure: false,
    algo: &STM32F4_ALGO,
}];

/// RAM regions of the device (96 KiB SRAM from the SRAM base).
pub static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
    start: 0x2000_0000,
    end: 0x2001_8000,
}];

/// Sector layout, one entry per uniform sector group. The F4 map is non-uniform: four 16 KiB
/// sectors, then one 64 KiB sector, then three 128 KiB sectors -- 512 KiB in eight sectors.
pub static SECTORS: [SectorInfo; 3] = [
    SectorInfo {
        start: 0x0800_0000,
        size: 0x0000_4000,
        count: 4,
    },
    SectorInfo {
        start: 0x0801_0000,
        size: 0x0001_0000,
        count: 1,
    },
    SectorInfo {
        start: 0x0802_0000,
        size: 0x0002_0000,
        count: 3,
    },
];

/// Page-staging scratch size: the algorithm's 1 KiB program buffer, not the 16 KiB erase unit.
pub const PROGRAM_BUFFER_SIZE: usize = 0x400;

/// The target description type for this device.
#[derive(Clone, Copy, Debug, Default)]
pub struct STM32F401CE;

impl Target for STM32F401CE {
    fn name(&self) -> &'static str {
        "STM32F401CE"
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
        // Never nRESET: a pulse clears core debug configuration, wiping the armed vector catch.
        ResetType::SoftwareSysReset
    }
    fn identity_probes(&self) -> &[IdentityProbe] {
        &IDENTITY_PROBES
    }
    fn uf2_family(&self) -> Option<u32> {
        Some(STM32F4_UF2_FAMILY)
    }
}
