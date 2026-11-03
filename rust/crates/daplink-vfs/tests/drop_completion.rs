//! When a finished drop's medium goes away.

mod common;

use common::{dir_entry, uf2_numbered};
use daplink_core::{FlashBackend, FlashPhase};
use daplink_targets::{targets::rp2040::RP2040, Target};
use daplink_vfs::{
    fat, FlashSink, MediaState, NoObserver, TransferOutcome, VfsConfig, VfsFile, MSC_RECONNECT_MS,
    MSC_TIMEOUT_FINISHABLE_MS, MSC_TIMEOUT_FINISHABLE_UNSIZED_MS,
};

const FAM_A: u32 = 0xE48B_FF56;
const FAM_B: u32 = 0x5775_5A57;
const FLAG_FAMILY: u32 = 0x2000;

/// Records programs and session closes over the RP2040 window; `family` is what it accepts.
struct Rec {
    ranges: [(u32, u32); 1],
    family: Option<u32>,
    programmed: Vec<u32>,
    finishes: Vec<bool>,
    fail_finish: bool,
}

impl Rec {
    fn new(family: Option<u32>) -> Self {
        let flash = RP2040
            .flash_regions()
            .iter()
            .find(|r| r.is_default)
            .expect("default flash region");
        Self {
            ranges: [(flash.start, flash.end)],
            family,
            programmed: Vec::new(),
            finishes: Vec::new(),
            fail_finish: false,
        }
    }
}

impl FlashBackend for Rec {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        Ok(())
    }
    fn program(&mut self, addr: u32, _data: &[u8]) -> Result<(), FlashPhase> {
        self.programmed.push(addr);
        Ok(())
    }
    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        self.finishes.push(success);
        if success && self.fail_finish {
            return Err(FlashPhase::Verify);
        }
        Ok(())
    }
    fn finish_with_reset(&mut self, success: bool, _reset: bool) -> Result<(), FlashPhase> {
        self.finish(success)
    }
    fn flash_ranges(&self) -> &[(u32, u32)] {
        &self.ranges
    }
    fn expected_uf2_family(&self) -> Option<u32> {
        self.family
    }
}

type V<'a> = daplink_vfs::Vfs<'a, FlashSink<Rec>, NoObserver>;

fn vfs<'a>(files: &'a mut [&'a mut dyn VfsFile], family: Option<u32>) -> V<'a> {
    daplink_vfs::Vfs::new(
        VfsConfig::default(),
        files,
        FlashSink::new(Rec::new(family)),
        NoObserver,
    )
}

const XIP: u32 = 0x1000_0000;

/// Block `n` of `of` at its place in flash, tagged `family` when given.
fn block(n: u32, of: u32, family: Option<u32>) -> [u8; 512] {
    let mut b = uf2_numbered(n, of, XIP + n * 256, &[n as u8; 256]);
    if let Some(f) = family {
        b[8..12].copy_from_slice(&FLAG_FAMILY.to_le_bytes());
        b[28..32].copy_from_slice(&f.to_le_bytes());
    }
    b
}

/// The dropped file's entry at cluster 2, sized `size` (0 = unknown).
fn dirent(vfs: &mut V<'_>, size: u32) {
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, size));
    vfs.write_sector(131, &root);
}

/// A FAT-copy sector: the kind of housekeeping a host lands after its data.
fn fat_write(vfs: &mut V<'_>) {
    vfs.write_sector(1, &[0xF8u8; 512]);
}

fn stream(vfs: &mut V<'_>, blocks: &[[u8; 512]]) {
    for (i, b) in blocks.iter().enumerate() {
        vfs.write_sector(fat::DATA_START_LBA + i as u32, b);
    }
}

fn run(n: u32, family: Option<u32>) -> Vec<[u8; 512]> {
    (0..n).map(|i| block(i, n, family)).collect()
}

/// A host still writing after `Done` holds the medium up until it is quiet.
#[test]
fn finished_medium_waits_for_write_silence() {
    // The size-bearing entry before the data (Windows-like) and after it (macOS-like).
    for dirent_first in [true, false] {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut v = vfs(&mut files, None);
        if dirent_first {
            dirent(&mut v, 4 * 512);
        }
        stream(&mut v, &run(4, None));
        if !dirent_first {
            dirent(&mut v, 4 * 512);
        }
        assert!(!v.is_transfer_active(), "dirent_first={dirent_first}");
        assert_eq!(v.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(
            v.sink().backend().finishes,
            [true],
            "flashed and reset at completion"
        );

        // The host's tail: a FAT copy every 100 ms for a second.
        let mut landed_detached = 0;
        for _ in 0..10 {
            assert_eq!(
                v.tick(100),
                MediaState::Ready,
                "dirent_first={dirent_first}"
            );
            if v.media() == MediaState::Detached {
                landed_detached += 1;
            }
            fat_write(&mut v);
        }
        assert_eq!(
            landed_detached, 0,
            "no tail write lands on a missing medium"
        );

        // Then silence: the medium goes away after exactly the sized idle window.
        assert_eq!(v.tick(MSC_TIMEOUT_FINISHABLE_MS - 1), MediaState::Ready);
        assert_eq!(v.tick(1), MediaState::Detached);
        assert!(v.take_transfer_finished());
        assert_eq!(v.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(v.take_media_changed());

        // The tail reopened nothing: one session, closed once, four blocks.
        assert_eq!(v.sink().backend().finishes, [true]);
        assert_eq!(v.sink().backend().programmed.len(), 4);
        assert_eq!(v.transfer_timeout(), 0, "nothing pending after the remount");
    }
}

/// Data and a new entry during the settle are absorbed, as a finished volume absorbs them in C.
#[test]
fn a_second_drop_during_the_settle_binds_nothing() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut v = vfs(&mut files, None);
    dirent(&mut v, 2 * 512);
    stream(&mut v, &run(2, None));
    assert_eq!(v.sink().outcome(), Some(TransferOutcome::Done));
    v.tick(100);
    // A second file: its data lands past the first, its entry in slot 1.
    v.write_sector(fat::DATA_START_LBA + 2, &block(0, 2, None));
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 2 * 512));
    root[32..64].copy_from_slice(&dir_entry(b"OTHER   UF2", 0x20, 4, 2 * 512));
    v.write_sector(131, &root);
    assert!(!v.is_transfer_active());
    v.tick(MSC_TIMEOUT_FINISHABLE_MS);
    assert_eq!(v.media(), MediaState::Detached);
    assert_eq!(v.tick(MSC_RECONNECT_MS), MediaState::Ready);
    assert_eq!(
        v.transfer_timeout(),
        0,
        "the absorbed entry binds no pending drop"
    );
    assert_eq!(v.sink().backend().programmed.len(), 2);
    assert_eq!(v.sink().backend().finishes, [true]);
}

/// An idle completion has already proven the silence, so it detaches at once.
#[test]
fn an_idle_completion_detaches_without_a_second_wait() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut v = vfs(&mut files, None);
    dirent(&mut v, 0);
    stream(&mut v, &run(2, None)[..2]);
    assert!(v.is_transfer_active());
    assert_eq!(
        v.tick(MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached
    );
    assert_eq!(v.sink().outcome(), Some(TransferOutcome::Done));
}

/// A failed drop keeps failing loud: the host's tail hits a missing medium, as today.
#[test]
fn a_failed_drop_detaches_at_once() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut v = vfs(&mut files, Some(FAM_A));
    dirent(&mut v, 2 * 512);
    stream(&mut v, &run(2, Some(FAM_B)));
    assert_eq!(
        v.sink().outcome(),
        Some(TransferOutcome::FlashError(FlashPhase::Family))
    );
    assert_eq!(v.tick(1), MediaState::Detached);
}

/// `Done` from the stream does not settle when the flash then refused it.
#[test]
fn a_failed_final_flush_detaches_at_once() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut v = daplink_vfs::Vfs::new(
        VfsConfig::default(),
        &mut files,
        FlashSink::new(Rec {
            fail_finish: true,
            ..Rec::new(None)
        }),
        NoObserver,
    );
    dirent(&mut v, 2 * 512);
    stream(&mut v, &run(2, None));
    assert_eq!(
        v.sink().outcome(),
        Some(TransferOutcome::FlashError(FlashPhase::Verify))
    );
    assert_eq!(v.tick(1), MediaState::Detached);
}

/// A drop that flashed nothing did not succeed, so it does not wait either.
#[test]
fn a_drop_with_nothing_programmable_detaches_at_once() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut v = vfs(&mut files, None);
    dirent(&mut v, 512);
    let mut meta = block(0, 1, None);
    meta[8..12].copy_from_slice(&1u32.to_le_bytes());
    stream(&mut v, &[meta]);
    assert_eq!(v.sink().outcome(), Some(TransferOutcome::NoData));
    assert_eq!(v.tick(1), MediaState::Detached);
}

/// A universal file in cat order, the target's run first, ends at its last byte and then settles.
#[test]
fn a_two_run_file_in_cat_order_ends_done_and_settles() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut v = vfs(&mut files, Some(FAM_A));
    dirent(&mut v, 0);
    let image: Vec<[u8; 512]> = run(2, Some(FAM_A))
        .into_iter()
        .chain(run(2, Some(FAM_B)))
        .collect();
    stream(&mut v, &image);
    assert!(v.is_transfer_active(), "the target's run does not end it");
    dirent(&mut v, 4 * 512);
    assert_eq!(v.sink().outcome(), Some(TransferOutcome::Done));
    assert_eq!(v.sink().backend().programmed.len(), 2);
    assert_eq!(v.tick(MSC_TIMEOUT_FINISHABLE_MS - 1), MediaState::Ready);
    assert_eq!(v.tick(1), MediaState::Detached);
}
