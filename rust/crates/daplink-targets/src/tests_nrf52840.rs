//! Host tests pinning the nRF52840 record's geometry, identity, UF2 family and family wiring.

use daplink_flash::target::ResetType;

use crate::algorithms::nrf52::{NRF52_ALGO, NRF52_BLOB};
use crate::families::nrf52::{FICR_CODEPAGESIZE, FICR_INFO_PART, NRF52_RESET_SEQUENCE};
use crate::targets::nrf52840::{NRF52840, NRF52840_INFO_PART, NRF52840_UF2_FAMILY};
use crate::{check_identity, Target};

#[test]
fn geometry_is_1m_flash_256k_ram_4k_pages() {
    let r = NRF52840.flash_regions();
    assert_eq!(r.len(), 1);
    assert_eq!((r[0].start, r[0].end), (0x0000_0000, 0x0010_0000));
    assert!(r[0].is_default);

    let ram = NRF52840.ram_regions();
    assert_eq!(ram.len(), 1);
    assert_eq!((ram[0].start, ram[0].end), (0x2000_0000, 0x2004_0000));

    let s = NRF52840.sectors();
    assert_eq!(s.len(), 1);
    assert_eq!((s[0].start, s[0].size, s[0].count), (0, 0x1000, 256));
    assert_eq!(s[0].end(), r[0].end);
}

#[test]
fn identity_is_ficr_part_then_codepagesize() {
    let probes = NRF52840.identity_probes();
    assert_eq!(probes.len(), 2);
    assert_eq!(FICR_INFO_PART, 0x1000_0100);
    assert_eq!(
        (probes[0].addr, probes[0].mask, probes[0].expected),
        (FICR_INFO_PART, 0xFFFF_FFFF, 0x0005_2840)
    );
    assert_eq!(NRF52840_INFO_PART, 0x0005_2840);
    assert_eq!(FICR_CODEPAGESIZE, 0x1000_0010);
    assert_eq!(
        (probes[1].addr, probes[1].mask, probes[1].expected),
        (FICR_CODEPAGESIZE, 0xFFFF_FFFF, 0x1000)
    );

    let ficr = |part: u32, page: u32| {
        move |a: u32| match a {
            FICR_INFO_PART => Ok(part),
            FICR_CODEPAGESIZE => Ok(page),
            other => panic!("unexpected read {other:#x}"),
        }
    };
    assert!(check_identity(probes, ficr(0x0005_2840, 0x1000)).is_ok());
    assert!(check_identity(probes, ficr(0x0005_2833, 0x1000)).is_err());
    assert!(check_identity(probes, ficr(0x0005_2840, 0x0400)).is_err());
}

#[test]
fn uf2_family_is_the_nrf52840_id_not_the_generic_one() {
    // Microsoft uf2families.json: "Nordic NRF52840" -> 0xada52840; "Nordic NRF52" -> 0x1b57745f.
    assert_eq!(NRF52840_UF2_FAMILY, 0xADA5_2840);
    assert_eq!(NRF52840.uf2_family(), Some(0xADA5_2840));
    assert_ne!(NRF52840_UF2_FAMILY, 0x1B57_745F);
}

#[test]
fn reset_is_software_through_the_family_sequence() {
    assert_eq!(NRF52840.name(), "nRF52840");
    assert_eq!(NRF52840.reset_type(), ResetType::SoftwareSysReset);
    assert!(core::ptr::eq(
        NRF52840.reset_sequence() as *const _ as *const (),
        &NRF52_RESET_SEQUENCE as *const _ as *const (),
    ));
}

#[test]
fn region_uses_the_shared_algorithm_inside_ram() {
    let algo = NRF52840.flash_regions()[0].algo;
    assert!(core::ptr::eq(algo, &NRF52_ALGO));
    let ram = NRF52840.ram_regions()[0];
    assert!(ram.contains(algo.algo_start));
    assert!(algo.algo_start + NRF52_BLOB.len() as u32 <= ram.end);
    assert!(ram.contains(algo.syscall.stack_pointer));
    assert!(algo.program_buffer + algo.program_buffer_size <= ram.end);
}
