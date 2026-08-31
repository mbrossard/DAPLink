//! Target description for the Raspberry Pi RP2350 (Pico 2 family; external QSPI flash).

use daplink_flash::identity::IdentityProbe;
use daplink_flash::reset_sequence::ResetSequence;
use daplink_flash::target::{ApAddressing, DebugPort, DpWake};

use crate::algorithms::rp235x::RP2350_ALGO as SELECTED_ALGO;
use crate::families::rp2350::RP2350_RESET_SEQUENCE;
use crate::{FlashRegion, RamRegion, SectorInfo, Target};

/// Official Microsoft UF2 family id for the RP2350 in Arm secure mode (`uf2families.json`,
/// "Raspberry Pi RP2350, Arm Secure"). This is the only directly-bootable family accepted here.
pub const RP2350_ARM_S_UF2_FAMILY: u32 = 0xE48B_FF59;
/// UF2 family id for the RP2350 RISC-V (Hazard3) image. Rejected: this Arm entry cannot flash a
/// RISC-V image (it would land the wrong vector layout / boot mode).
pub const RP2350_RISCV_UF2_FAMILY: u32 = 0xE48B_FF5A;
/// UF2 family id for the RP2350 Arm non-secure image. Rejected: an NS image is not directly
/// bootable (it must be launched by a prior secure image), so it is not a valid drag-n-drop drop.
pub const RP2350_ARM_NS_UF2_FAMILY: u32 = 0xE48B_FF5B;

/// ADIv6 base address of the RP2350 core-0 mem-AP (AHB5-AP), programmed into DP `SELECT`; core 1
/// is `0x4000`, the RISC-V APB-AP `0xA000`, the RP-AP `0x8_0000`.
pub const RP2350_CORE0_AP_BASE: u32 = 0x2000;

/// Multidrop `TARGETSEL` for the RP2350 core-0 debug port: instance `0x0` in `[31:28]`, part
/// `0x0040` in `[27:12]`, JEP-106 designer `0x493` in `[11:1]`, stop bit set.
pub const RP2350_CORE0_TARGETSEL: u32 = 0x0004_0927;

/// `SYSINFO.CHIP_ID` at `0x4000_0000` is `REVISION[31:28] | PART[27:12] | MANUFACTURER[11:1] |
/// STOP[0]`; masking off the stepping-dependent `REVISION` nibble leaves this.
pub const RP2350_CHIP_ID_EXPECTED: u32 = 0x0000_4927;

/// Target-identity probes for the RP2350.
pub static IDENTITY_PROBES: [IdentityProbe; 1] = [IdentityProbe::new(
    0x4000_0000,
    0x0FFF_FFFF,
    RP2350_CHIP_ID_EXPECTED,
)];

/// Flash regions: the Pico 2's 4 MiB of external QSPI flash at the XIP base `0x1000_0000`.
pub static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
    start: 0x1000_0000,
    end: 0x1040_0000,
    is_default: true,
    is_secure: false,
    algo: &SELECTED_ALGO,
}];

/// RAM regions of the device (520 KiB main SRAM, striped banks plus the two 4 KiB banks).
pub static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
    start: 0x2000_0000,
    end: 0x2008_2000,
}];

/// Sector layout: uniform 4 KiB sectors across the 4 MiB flash.
pub static SECTORS: [SectorInfo; 1] = [SectorInfo {
    start: 0x1000_0000,
    size: 0x0000_1000,
    count: 1024,
}];

/// Page-staging scratch size: the algorithm's program buffer, one 4 KiB QSPI sector.
pub const PROGRAM_BUFFER_SIZE: usize = 0x1000;

/// The target description type for this device.
#[derive(Clone, Copy, Debug, Default)]
pub struct RP235x;

impl Target for RP235x {
    fn name(&self) -> &'static str {
        "RP2350"
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
    fn reset_sequence(&self) -> &dyn ResetSequence {
        // Any reset drops the multidrop DP and clears the vector catch; see `families::rp2350`.
        &RP2350_RESET_SEQUENCE
    }
    fn uf2_family(&self) -> Option<u32> {
        Some(RP2350_ARM_S_UF2_FAMILY)
    }
    fn debug_port(&self) -> DebugPort {
        // Dormant at power-up, ADIv6 AP addressing, and silent until `TARGETSEL` selects core 0.
        DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: Some(RP2350_CORE0_TARGETSEL),
            ap: ApAddressing::V6 {
                ap_base: RP2350_CORE0_AP_BASE,
            },
        }
    }
}

/// Assert at compile time that the rejected UF2 families are distinct from the accepted one, so a
/// future edit cannot silently collapse them.
const _: () = {
    assert!(RP2350_ARM_S_UF2_FAMILY != RP2350_RISCV_UF2_FAMILY);
    assert!(RP2350_ARM_S_UF2_FAMILY != RP2350_ARM_NS_UF2_FAMILY);
};
