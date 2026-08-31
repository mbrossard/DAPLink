//! Target description for the Raspberry Pi RP2040 (external QSPI flash, RAM-resident algorithm).

use daplink_flash::reset_sequence::ResetSequence;
use daplink_flash::target::{ApAddressing, DebugPort, DpWake};

use daplink_flash::identity::IdentityProbe;

use crate::algorithms::rp2040_rom::RP2040_ROM_ALGO as SELECTED_ALGO;
use crate::families::rp2040::RP2040_RESET_SEQUENCE;
use crate::{FlashRegion, RamRegion, SectorInfo, Target};

/// Official Microsoft UF2 family id for the RP2040 (`uf2families.json`, "Raspberry Pi RP2040").
pub const RP2040_UF2_FAMILY: u32 = 0xE48B_FF56;

/// Multidrop `TARGETSEL` for the RP2040's core 0; core 1 is `0x1100_2927`, the rescue DP
/// `0xF100_2927`. A cold chip may not answer DPIDR until one of them is selected.
pub const RP2040_CORE0_TARGETSEL: u32 = 0x0100_2927;

/// Target-identity probes: the bootrom magic at `0x0000_0010` (`'M'`, `'u'`, ROM version), masked
/// to `0x0001_754D` (RP2040 datasheet 2.8.3).
pub static IDENTITY_PROBES: [IdentityProbe; 1] =
    [IdentityProbe::new(0x0000_0010, 0x00FF_FFFF, 0x0001_754D)];

/// Flash (programmable) regions of the device.
pub static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
    start: 0x1000_0000,
    end: 0x1020_0000,
    is_default: true,
    is_secure: false,
    algo: &SELECTED_ALGO,
}];

/// RAM regions of the device.
pub static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
    start: 0x2000_0000,
    end: 0x2004_2000,
}];

/// Sector layout, one entry per uniform sector group.
pub static SECTORS: [SectorInfo; 1] = [SectorInfo {
    start: 0x1000_0000,
    size: 0x0000_1000,
    count: 512,
}];

/// Page-staging scratch size: the algorithm's program buffer, one 4 KiB QSPI sector.
pub const PROGRAM_BUFFER_SIZE: usize = 0x1000;

/// The target description type for this device.
#[derive(Clone, Copy, Debug, Default)]
pub struct RP2040;

impl Target for RP2040 {
    fn name(&self) -> &'static str {
        "RP2040"
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
    fn reset_sequence(&self) -> &dyn ResetSequence {
        &RP2040_RESET_SEQUENCE
    }
    fn identity_probes(&self) -> &[IdentityProbe] {
        &IDENTITY_PROBES
    }
    fn uf2_family(&self) -> Option<u32> {
        Some(RP2040_UF2_FAMILY)
    }
    fn debug_port(&self) -> DebugPort {
        // A DPv2 that ignores the JTAG-to-SWD switch: dormant wake, then a core-0 TARGETSEL.
        DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: Some(RP2040_CORE0_TARGETSEL),
            ap: ApAddressing::V5,
        }
    }
}
