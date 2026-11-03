//! Real-host write sequences: early metadata, size-0 entries, out-of-order and data-first drops.

mod common;

use common::{dir_entry, uf2_numbered};
use daplink_core::{FlashBackend, FlashPhase};
use daplink_targets::{targets::rp2040::RP2040, Target};
use daplink_vfs::{
    fat, DetailsFile, DeviceInfo, FlashSink, MediaState, NoObserver, TransferOutcome, VfsConfig,
    VfsFile,
};

/// A flash backend recording every programmed fragment, over the RP2040 XIP window.
struct RecordingBackend {
    ranges: [(u32, u32); 1],
    writes: Vec<(u32, Vec<u8>)>,
    begun: bool,
    finished: Option<bool>,
}

impl RecordingBackend {
    /// The window comes from the RP2040 chip record so it cannot drift.
    fn new() -> Self {
        let flash = RP2040
            .flash_regions()
            .iter()
            .find(|r| r.is_default)
            .expect("the RP2040 record has a default flash region");
        Self {
            ranges: [(flash.start, flash.end)],
            writes: Vec::new(),
            begun: false,
            finished: None,
        }
    }

    /// The programmed bytes at `base`, concatenated in program order.
    fn programmed_from(&self, base: u32) -> Vec<u8> {
        let mut out = Vec::new();
        for (addr, data) in &self.writes {
            if *addr >= base {
                out.extend_from_slice(data);
            }
        }
        out
    }
}

impl FlashBackend for RecordingBackend {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        self.begun = true;
        Ok(())
    }
    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
        self.writes.push((addr, data.to_vec()));
        Ok(())
    }
    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        self.finished = Some(success);
        Ok(())
    }
    fn finish_with_reset(&mut self, success: bool, _reset: bool) -> Result<(), FlashPhase> {
        self.finish(success)
    }
    fn flash_ranges(&self) -> &[(u32, u32)] {
        &self.ranges
    }
    fn selected_target_name(&self) -> Option<&str> {
        Some("RP2040")
    }
}

/// Write a single 32-byte entry into root-directory `slot` (0..15 of sector 131) and apply it.
fn write_dir_slot(
    vfs: &mut daplink_vfs::Vfs<'_, FlashSink<RecordingBackend>, NoObserver>,
    slot: usize,
    entry: &[u8; 32],
) {
    let mut root = [0u8; 512];
    root[slot * 32..slot * 32 + 32].copy_from_slice(entry);
    vfs.write_sector(131, &root);
}

const XIP_BASE: u32 = 0x1000_0000;

/// Block `block_no` of `num_blocks` at its XIP page; `good_end: false` corrupts the end magic.
fn uf2_block(block_no: u32, num_blocks: u32, payload: &[u8], good_end: bool) -> [u8; 512] {
    let mut b = uf2_numbered(block_no, num_blocks, XIP_BASE + block_no * 256, payload);
    if !good_end {
        b[508..512].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
    }
    b
}

/// A magicless sector standing in for host metadata written into the data region.
fn metadata_sector(fill: u8) -> [u8; 512] {
    let mut s = [fill; 512];
    // Zero the leading word so the sector can never be mistaken for an image start.
    s[0..4].copy_from_slice(&0x0000_0000u32.to_le_bytes());
    s
}

/// Build a two-block UF2 image and its expected 512-byte decoded payload.
fn two_block_uf2() -> ([u8; 512], [u8; 512], Vec<u8>) {
    let p0 = [0xA1u8; 256];
    let p1 = [0xB2u8; 256];
    let mut expected = Vec::new();
    expected.extend_from_slice(&p0);
    expected.extend_from_slice(&p1);
    (
        uf2_block(0, 2, &p0, true),
        uf2_block(1, 2, &p1, true),
        expected,
    )
}

/// Find a file's `(first_cluster, size)` in the root directory, reading through the volume.
fn find(
    vfs: &mut daplink_vfs::Vfs<'_, FlashSink<RecordingBackend>, NoObserver>,
    name: &[u8; 11],
) -> Option<(u32, u32)> {
    for root in 131..=132 {
        let mut buf = [0u8; 512];
        vfs.read_sector(root, &mut buf);
        for i in 0..16 {
            let e = &buf[i * 32..i * 32 + 32];
            if &e[0..11] == name {
                let fc = u16::from_le_bytes([e[26], e[27]]) as u32;
                let sz = u32::from_le_bytes([e[28], e[29], e[30], e[31]]);
                return Some((fc, sz));
            }
        }
    }
    None
}

fn read_file(
    vfs: &mut daplink_vfs::Vfs<'_, FlashSink<RecordingBackend>, NoObserver>,
    name: &[u8; 11],
) -> Vec<u8> {
    let (cluster, size) = find(vfs, name).expect("file present");
    let mut out = Vec::new();
    let mut sector = 0;
    while out.len() < size as usize {
        let lba = fat::cluster_to_lba(cluster) + sector;
        let mut buf = [0u8; 512];
        vfs.read_sector(lba, &mut buf);
        out.extend_from_slice(&buf);
        sector += 1;
    }
    out.truncate(size as usize);
    out
}

/// The `Status:` line `DETAILS.TXT` currently publishes, trimmed of its field padding.
fn status_line(vfs: &mut daplink_vfs::Vfs<'_, FlashSink<RecordingBackend>, NoObserver>) -> String {
    let body = read_file(vfs, b"DETAILS TXT");
    let text = std::str::from_utf8(&body).unwrap();
    let off = text.find("Status: ").unwrap() + "Status: ".len();
    text[off..off + 64].trim_end().to_string()
}

/// Drive the detach and remount so status files refresh.
fn cycle_remount(vfs: &mut daplink_vfs::Vfs<'_, FlashSink<RecordingBackend>, NoObserver>) {
    // A finished transfer detaches on the next tick.
    for _ in 0..2 {
        if vfs.tick(1) == MediaState::Detached {
            break;
        }
    }
    assert_eq!(vfs.media(), MediaState::Detached, "transfer should detach");
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_RECONNECT_MS),
        MediaState::Ready,
        "volume should remount"
    );
}

/// The dropped user file lives well past the generated files and any metadata clusters.
const USER_CLUSTER: u16 = 10;

fn user_lba(sector: u32) -> u32 {
    fat::cluster_to_lba(USER_CLUSTER as u32) + sector
}

macro_rules! with_vfs {
    ($vfs:ident, $body:block) => {{
        let mut details = DetailsFile::new(DeviceInfo {
            version: "0.1.0",
            serial: "SER123",
            reset_count: 0,
        });
        let mut files: [&mut dyn VfsFile; 1] = [&mut details];
        let mut $vfs: daplink_vfs::Vfs<'_, _, _> = daplink_vfs::Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(RecordingBackend::new()),
            NoObserver,
        );
        $body
    }};
}

#[test]
fn metadata_written_before_file_does_not_start_bogus_transfer() {
    with_vfs!(vfs, {
        let (b0, b1, expected) = two_block_uf2();

        // The host creates the directory entry (UF2, cluster 10, size 1024)...
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );

        // ...then writes metadata into the data region first.
        vfs.write_sector(fat::cluster_to_lba(6), &metadata_sector(0x5A));
        vfs.write_sector(fat::cluster_to_lba(7), &metadata_sector(0xC3));
        // No transfer must have started from metadata.
        assert!(
            !vfs.is_transfer_active(),
            "host metadata must not start a transfer"
        );

        // Then the real UF2 sectors arrive in order at the file's own cluster.
        vfs.write_sector(user_lba(0), &b0);
        vfs.write_sector(user_lba(1), &b1);

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
        assert_eq!(vfs.sink().backend().finished, Some(true));
    });
}

#[test]
fn dir_entry_size_zero_then_updated_after_data() {
    with_vfs!(vfs, {
        let (b0, b1, expected) = two_block_uf2();

        // The entry is first created with size 0 and no cluster (the common macOS create).
        write_dir_slot(&mut vfs, 5, &dir_entry(b"APP     UF2", 0x20, 0, 0));
        // Metadata to the data region (must not start the bound UF2 transfer via the guard).
        vfs.write_sector(fat::cluster_to_lba(6), &metadata_sector(0x11));
        assert!(!vfs.is_transfer_active());

        // The data streams while the entry still reads size 0, so it completes on the unsized timeout.
        vfs.write_sector(user_lba(0), &b0);
        vfs.write_sector(user_lba(1), &b1);
        // The host rewrites the entry with the real cluster and size on close.
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );

        // Both blocks were programmed during streaming; the finishable UF2 completes on timeout.
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
        assert_eq!(
            vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_MS),
            MediaState::Detached
        );
        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
    });
}

#[test]
fn interleaved_metadata_mid_stream_is_ignored() {
    with_vfs!(vfs, {
        let (b0, b1, expected) = two_block_uf2();
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );

        vfs.write_sector(user_lba(0), &b0);
        // A metadata flush lands between the file's own sectors: it must be ignored, not abort.
        vfs.write_sector(fat::cluster_to_lba(6), &metadata_sector(0x77));
        vfs.write_sector(user_lba(1), &b1);

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
    });
}

#[test]
fn duplicate_sector_rewrite_is_tolerated() {
    with_vfs!(vfs, {
        let (b0, b1, expected) = two_block_uf2();
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );

        vfs.write_sector(user_lba(0), &b0);
        // The host re-flushes the first cluster (identical bytes): tolerate, do not abort.
        vfs.write_sector(user_lba(0), &b0);
        vfs.write_sector(user_lba(1), &b1);

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
    });
}

#[test]
fn apple_double_sidecar_with_same_basename_is_ignored() {
    with_vfs!(vfs, {
        let (b0, b1, expected) = two_block_uf2();

        // A hidden AppleDouble sidecar sharing the basename must neither bind nor corrupt the transfer.
        write_dir_slot(&mut vfs, 4, &dir_entry(b"._APP   UF2", 0x22, 6, 4096));
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );
        // The sidecar's data (at its own cluster 6) is written first.
        vfs.write_sector(fat::cluster_to_lba(6), &metadata_sector(0x2E));
        assert!(!vfs.is_transfer_active());

        vfs.write_sector(user_lba(0), &b0);
        vfs.write_sector(user_lba(1), &b1);

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
    });
}

#[test]
fn details_status_populated_on_decode_error() {
    with_vfs!(vfs, {
        // A valid start magic with a corrupt end magic fails with DecodeError.
        let bad = uf2_block(0, 1, &[0xAAu8; 256], false);
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"BAD     UF2", 0x20, USER_CLUSTER, 512),
        );
        vfs.write_sector(user_lba(0), &bad);

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::DecodeError));

        // After the remount, the published status names the failure.
        cycle_remount(&mut vfs);
        let text = status_line(&mut vfs);
        assert!(
            text.contains("DecodeError"),
            "the status names the failure: {text:?}"
        );
    });
}

#[test]
fn data_first_sniff_starts_and_late_dirent_completes_it() {
    with_vfs!(vfs, {
        // No directory entry at all: the block magic alone starts the transfer.
        let (b0, b1, expected) = two_block_uf2();

        vfs.write_sector(user_lba(0), &b0);
        assert!(
            vfs.is_transfer_active(),
            "block magic must sniff-start an unbound image"
        );
        vfs.write_sector(user_lba(1), &b1);

        // The entry lands last with the met size and the starting cluster: it binds and completes.
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
        assert_eq!(vfs.sink().backend().finished, Some(true));
    });
}

#[test]
fn interleaved_metadata_during_a_data_first_drop_is_ignored() {
    with_vfs!(vfs, {
        // Data-first, with housekeeping ahead of the front: a magicless sector is foreign, not a gap.
        let (b0, b1, expected) = two_block_uf2();

        vfs.write_sector(user_lba(0), &b0);
        assert!(vfs.is_transfer_active());

        vfs.write_sector(fat::cluster_to_lba(12), &metadata_sector(0x99));
        assert!(
            vfs.is_transfer_active(),
            "foreign housekeeping ahead of the front must not abort the stream"
        );

        vfs.write_sector(user_lba(1), &b1);
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
        assert_eq!(
            vfs.sink().backend().writes.len(),
            2,
            "the foreign sector must never reach the backend"
        );
    });
}

#[test]
fn a_reordered_data_first_drop_reports_out_of_order_not_timeout() {
    with_vfs!(vfs, {
        // Data-first with the third sector before the second: the verdict names the reorder.
        vfs.write_sector(user_lba(0), &uf2_block(0, 3, &[0xA1u8; 256], true));
        assert!(vfs.is_transfer_active());
        vfs.write_sector(user_lba(2), &uf2_block(2, 3, &[0xC3u8; 256], true));
        vfs.write_sector(user_lba(1), &uf2_block(1, 3, &[0xA5u8; 256], true));
        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1536),
        );
        assert!(
            vfs.is_transfer_active(),
            "a stream short of its known size stays open until the active timeout"
        );

        assert_eq!(
            vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS),
            MediaState::Detached
        );
        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::OutOfOrder));
        assert_eq!(
            vfs.sink().backend().finished,
            Some(false),
            "the flash session closes as failed, not successful"
        );

        cycle_remount(&mut vfs);
        let text = status_line(&mut vfs);
        assert!(
            text.contains("OutOfOrder"),
            "the status must name the reorder: {text:?}"
        );
    });
}

#[test]
fn apple_double_sidecar_does_not_claim_a_data_first_drop() {
    with_vfs!(vfs, {
        // The sidecar lands first, then the real payload sniff-starts; the sidecar must not claim it.
        write_dir_slot(&mut vfs, 4, &dir_entry(b"._APP   UF2", 0x22, 6, 4096));
        vfs.write_sector(fat::cluster_to_lba(6), &metadata_sector(0x2E));
        assert!(!vfs.is_transfer_active());

        let (b0, b1, expected) = two_block_uf2();

        vfs.write_sector(user_lba(0), &b0);
        assert!(
            vfs.is_transfer_active(),
            "the sidecar must not have consumed the real file's sniff-start"
        );
        vfs.write_sector(user_lba(1), &b1);

        write_dir_slot(
            &mut vfs,
            5,
            &dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, 1024),
        );

        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
    });
}
