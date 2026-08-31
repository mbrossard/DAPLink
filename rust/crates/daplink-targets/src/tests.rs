//! Host tests for the registry and target-selection seam (require the `synthetic` feature).

use crate::algorithms::synthetic::SYNTHETIC_ALGO;
use crate::selector::{FixedTarget, ImageHint, TargetSelector};
use crate::targets::synthetic::{SyntheticTarget, SYNTHETIC_UF2_FAMILY};
use crate::{check_identity, FlashRegion, RamRegion, SectorInfo, Target};

/// A dummy target with empty memory maps, used to exercise the selector.
struct DummyTarget;

impl Target for DummyTarget {
    fn flash_regions(&self) -> &[FlashRegion<'_>] {
        &[]
    }
    fn ram_regions(&self) -> &[RamRegion] {
        &[]
    }
    fn sectors(&self) -> &[SectorInfo] {
        &[]
    }
}

#[test]
fn fixed_target_selects_first() {
    let first = DummyTarget;
    let second = DummyTarget;
    let table: [&dyn Target; 2] = [&first, &second];

    let selected = FixedTarget.select(ImageHint::NONE, &table);
    assert!(selected.is_ok());
    // The first entry is chosen (identity via pointer equality).
    let chosen = selected.expect("a target was selected");
    assert!(core::ptr::eq(
        chosen as *const dyn Target as *const (),
        &first as &dyn Target as *const dyn Target as *const ()
    ));
}

#[test]
fn fixed_target_empty_table_none() {
    let table: [&dyn Target; 0] = [];
    assert!(FixedTarget.select(ImageHint::NONE, &table).is_err());
}

#[test]
fn synthetic_algo_invariants() {
    let algo = &SYNTHETIC_ALGO;
    let ram = RamRegion {
        start: 0x2000_0000,
        end: 0x2000_2000,
    };

    // Stack pointer within RAM.
    assert!(ram.contains(algo.syscall.stack_pointer));
    // Static base within RAM.
    assert!(ram.contains(algo.syscall.static_base));

    // Program buffer within RAM, including its full extent.
    assert!(ram.contains(algo.program_buffer));
    let pb_end = algo.program_buffer + algo.program_buffer_size;
    assert!(pb_end <= ram.end);
    assert!(algo.program_buffer_size > 0);

    // Program buffer must not overlap the loaded algorithm blob.
    let blob_start = algo.algo_start;
    let blob_end = algo.algo_start + algo.algo_blob.len() as u32;
    assert!(algo.program_buffer >= blob_end || pb_end <= blob_start);

    // Entry offsets fall within the blob.
    let blob_len = algo.algo_blob.len() as u32;
    for &offset in &[
        algo.init,
        algo.uninit,
        algo.erase_chip,
        algo.erase_sector,
        algo.program_page,
    ] {
        assert!(offset < blob_len);
    }
    assert!(algo.verify.is_none());
}

#[test]
fn synthetic_target_resolves_algo() {
    let target = SyntheticTarget;
    let regions = target.flash_regions();
    assert!(!regions.is_empty());

    // The region's algorithm handle points at the synthetic algorithm.
    assert!(core::ptr::eq(regions[0].algo, &SYNTHETIC_ALGO));

    assert!(!target.sectors().is_empty());
    assert!(!target.ram_regions().is_empty());
}

#[test]
fn synthetic_carries_fake_identity_data() {
    let target = SyntheticTarget;
    // A fabricated, non-registered UF2 family id, so no real device is impersonated.
    assert_eq!(target.uf2_family(), Some(SYNTHETIC_UF2_FAMILY));

    // A single fabricated identity probe; its made-up signature matches only its made-up read.
    let probes = target.identity_probes();
    assert_eq!(probes.len(), 1);
    assert!(check_identity(probes, |a| {
        assert_eq!(a, probes[0].addr);
        Ok(0xDEAD_BEEF)
    })
    .is_ok());
    assert!(check_identity(probes, |_| Ok(0x0000_0000)).is_err());
}

/// A target with the default empty identity descriptor performs no reads and always returns `Ok`.
#[test]
fn empty_identity_descriptor_skips_check() {
    let target = DummyTarget;
    assert!(target.identity_probes().is_empty());
    assert_eq!(target.uf2_family(), None);
    let mut reads = 0u32;
    assert!(check_identity(target.identity_probes(), |_| {
        reads += 1;
        Ok(0)
    })
    .is_ok());
    assert_eq!(reads, 0);
}
