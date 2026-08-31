//! Host tests pinning the STM32F401CE record, including its non-uniform sector map.

use daplink_flash::target::ResetType;

use crate::algorithms::stm32f4::{STM32F4_ALGO, STM32F4_BLOB};
use crate::targets::stm32f401ce::{STM32F401CE, STM32F4_UF2_FAMILY};
use crate::{check_identity, RamRegion, Target};

/// The record must not reset this part over nRESET, which would clear the armed vector catch.
#[test]
fn reset_type_is_software_sysreset() {
    assert_eq!(STM32F401CE.reset_type(), ResetType::SoftwareSysReset);
}

#[test]
fn uf2_family_is_official_stm32f4_id() {
    // Microsoft uf2families.json: "ST STM32F401" (STM32F4) -> 0x57755a57.
    assert_eq!(STM32F4_UF2_FAMILY, 0x5775_5A57);
    assert_eq!(STM32F401CE.uf2_family(), Some(0x5775_5A57));
}

#[test]
fn identity_probe_matches_f401xe_dev_id() {
    let probes = STM32F401CE.identity_probes();
    assert_eq!(probes.len(), 1);
    let p = probes[0];
    // DBGMCU_IDCODE DEV_ID is bits [11:0]; the mask clears bit 1, so 0x433 and 0x431 both pass.
    assert_eq!(p.addr, 0xE004_2000);
    assert_eq!(p.mask, 0x0000_0FFD);
    assert_eq!(p.expected, 0x0000_0431);

    // REV_ID is masked off; F411 is accepted as an F401 superset, and F401xB/xC stays rejected.
    assert!(check_identity(probes, |_| Ok(0x1000_0433)).is_ok());
    assert!(check_identity(probes, |_| Ok(0x1000_0431)).is_ok());
    assert!(check_identity(probes, |_| Ok(0x1000_0423)).is_err());

    // Only those two of 4096 DEV_IDs: 0x413, 0x419, 0x421 and 0x441 are all real F4 neighbours.
    let mut accepted = [0u32; 2];
    let mut n = 0usize;
    for v in 0..0x1000u32 {
        if check_identity(probes, |_| Ok(v)).is_ok() {
            assert!(n < 2, "widened mask admits more than two DEV_IDs");
            accepted[n] = v;
            n += 1;
        }
    }
    assert_eq!((n, accepted), (2, [0x431, 0x433]));
}

#[test]
fn flash_region_is_512k_at_flash_base() {
    let target = STM32F401CE;
    let regions = target.flash_regions();
    assert_eq!(regions.len(), 1);
    let r = &regions[0];
    // STM32F401CE: 512 KiB of flash at the STM32 flash base.
    assert_eq!(r.start, 0x0800_0000);
    assert_eq!(r.end, 0x0808_0000);
    assert_eq!(r.end - r.start, 512 * 1024);
    assert!(r.is_default);
    assert!(!r.is_secure);
}

#[test]
fn ram_region_pinned() {
    let target = STM32F401CE;
    let ram = target.ram_regions();
    assert_eq!(ram.len(), 1);
    assert_eq!(ram[0].start, 0x2000_0000);
    assert_eq!(ram[0].end, 0x2001_8000);
    assert_eq!(ram[0].end - ram[0].start, 96 * 1024);
}

#[test]
fn sector_map_is_non_uniform_and_covers_the_whole_region() {
    let target = STM32F401CE;
    let sectors = target.sectors();
    // Four 16 KiB, one 64 KiB, three 128 KiB -- the F4 layout, not a uniform run.
    assert_eq!(sectors.len(), 3);
    assert_eq!(
        (sectors[0].start, sectors[0].size, sectors[0].count),
        (0x0800_0000, 0x4000, 4)
    );
    assert_eq!(
        (sectors[1].start, sectors[1].size, sectors[1].count),
        (0x0801_0000, 0x1_0000, 1)
    );
    assert_eq!(
        (sectors[2].start, sectors[2].size, sectors[2].count),
        (0x0802_0000, 0x2_0000, 3)
    );

    // The runs abut in order and end exactly at the region end: no gap, and no overhang.
    let region = &target.flash_regions()[0];
    let mut cursor = region.start;
    for s in sectors {
        assert_eq!(
            s.start, cursor,
            "sector run at 0x{:08x} does not abut",
            s.start
        );
        cursor = s.end();
    }
    assert_eq!(cursor, region.end);
    assert_eq!(sectors.iter().map(|s| s.count).sum::<u32>(), 8);
}

#[test]
fn sector_lookup_returns_the_size_of_the_containing_run() {
    let sectors = STM32F401CE.sectors();
    let size_at = |addr: u32| sectors.iter().find_map(|s| s.sector_size_at(addr));
    let base_at = |addr: u32| sectors.iter().find_map(|s| s.sector_base_at(addr));

    // One probe inside each run, including the boundary addresses where the size changes.
    assert_eq!(size_at(0x0800_0000), Some(0x4000));
    assert_eq!(size_at(0x0800_FFFF), Some(0x4000));
    assert_eq!(size_at(0x0801_0000), Some(0x1_0000));
    assert_eq!(size_at(0x0801_FFFF), Some(0x1_0000));
    assert_eq!(size_at(0x0802_0000), Some(0x2_0000));
    assert_eq!(size_at(0x0807_FFFF), Some(0x2_0000));
    // Past the device there is no sector at all.
    assert_eq!(size_at(0x0808_0000), None);

    // Erase bases snap to the containing sector, whose size differs by run.
    assert_eq!(base_at(0x0800_5001), Some(0x0800_4000));
    assert_eq!(base_at(0x0801_8000), Some(0x0801_0000));
    assert_eq!(base_at(0x0805_0000), Some(0x0804_0000));
}

#[test]
fn algo_entry_offsets_pinned() {
    let algo = &STM32F4_ALGO;
    // Entry points are OFFSETS into the blob (small values), not absolute addresses.
    assert_eq!(algo.init, 0x3d);
    assert_eq!(algo.uninit, 0x6f);
    assert_eq!(algo.erase_chip, 0x7d);
    assert_eq!(algo.erase_sector, 0xa9);
    assert_eq!(algo.program_page, 0xf5);
    // The vendor FLM exports no `Verify` symbol; the generic read-back is used instead.
    assert!(algo.verify.is_none());
}

#[test]
fn algo_entries_are_offsets_not_addresses() {
    let algo = &STM32F4_ALGO;
    let blob_len = STM32F4_BLOB.len() as u32;
    for &offset in &[
        algo.init,
        algo.uninit,
        algo.erase_chip,
        algo.erase_sector,
        algo.program_page,
    ] {
        // A value inside the blob is an offset; one near 0x2000_0000 is the absolute-address bug.
        assert!(offset < blob_len, "entry 0x{offset:x} is not a blob offset");
    }
}

#[test]
fn algo_ram_invariants() {
    let algo = &STM32F4_ALGO;
    let ram = RamRegion {
        start: 0x2000_0000,
        end: 0x2001_8000,
    };

    // Stack pointer, static base and the algorithm load address live in RAM.
    assert!(ram.contains(algo.syscall.stack_pointer));
    assert!(ram.contains(algo.syscall.static_base));
    assert!(ram.contains(algo.algo_start));

    // Program buffer lies fully within RAM.
    assert!(ram.contains(algo.program_buffer));
    let pb_end = algo.program_buffer + algo.program_buffer_size;
    assert!(pb_end <= ram.end);
    assert!(algo.program_buffer_size > 0);

    // Program buffer must not overlap the loaded algorithm blob.
    let blob_start = algo.algo_start;
    let blob_end = algo.algo_start + STM32F4_BLOB.len() as u32;
    assert!(algo.program_buffer >= blob_end || pb_end <= blob_start);
}

#[test]
fn program_buffer_size_matches_the_record() {
    // The record's staging size is the algorithm's own program buffer, not a sector size.
    assert_eq!(
        crate::targets::stm32f401ce::PROGRAM_BUFFER_SIZE as u32,
        STM32F4_ALGO.program_buffer_size
    );
}

#[test]
fn target_resolves_algo_handle() {
    let target = STM32F401CE;
    let regions = target.flash_regions();
    assert_eq!(regions.len(), 1);
    // The flash region's algorithm handle points at the shared stm32f4 algorithm.
    assert!(core::ptr::eq(regions[0].algo, &STM32F4_ALGO));
}
