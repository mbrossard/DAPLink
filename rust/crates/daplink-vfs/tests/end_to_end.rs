//! End-to-end tests: drop an image, cycle the remount, inspect the volume via a host reader.
#![allow(dead_code, unused_imports)]

mod common;

use common::{dir_entry, uf2_block};
use daplink_vfs::transfer::TransferOutcome;
use daplink_vfs::{
    fat, DecoderSet, MediaState, NoObserver, NullSink, TransferSink, VfsConfig, VfsFile,
    VfsObserver,
};

/// A minimal host FAT16 reader over a `Vfs`, used to extract a named file's bytes.
struct HostReader;

impl HostReader {
    fn fat_next<S, O, D>(vfs: &mut daplink_vfs::Vfs<'_, S, O, D>, cluster: u32) -> u32
    where
        S: TransferSink,
        O: VfsObserver,
        D: DecoderSet,
    {
        let fat_sector = 1 + (cluster / 256);
        let idx = (cluster % 256) as usize;
        let mut buf = [0u8; 512];
        vfs.read_sector(fat_sector, &mut buf);
        u16::from_le_bytes([buf[idx * 2], buf[idx * 2 + 1]]) as u32
    }

    /// Find a file by 8.3 name and return (first_cluster, size).
    fn find<S, O, D>(vfs: &mut daplink_vfs::Vfs<'_, S, O, D>, name: &[u8; 11]) -> Option<(u32, u32)>
    where
        S: TransferSink,
        O: VfsObserver,
        D: DecoderSet,
    {
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

    fn read_file<S, O, D>(
        vfs: &mut daplink_vfs::Vfs<'_, S, O, D>,
        name: &[u8; 11],
    ) -> Option<Vec<u8>>
    where
        S: TransferSink,
        O: VfsObserver,
        D: DecoderSet,
    {
        let (mut cluster, size) = Self::find(vfs, name)?;
        let mut out = Vec::new();
        while (2..0xFFF8).contains(&cluster) {
            for s in 0..fat::SECTORS_PER_CLUSTER {
                let lba = fat::cluster_to_lba(cluster) + s;
                let mut buf = [0u8; 512];
                vfs.read_sector(lba, &mut buf);
                out.extend_from_slice(&buf);
            }
            cluster = Self::fat_next(vfs, cluster);
        }
        out.truncate(size as usize);
        Some(out)
    }
}

#[test]
fn registered_details_file_extracts() {
    use daplink_vfs::{DetailsFile, DeviceInfo};
    let mut details = DetailsFile::new(DeviceInfo {
        version: "9.9.9",
        serial: "SER123",
        reset_count: 0,
    });
    let mut files: [&mut dyn VfsFile; 1] = [&mut details];
    let mut vfs: daplink_vfs::Vfs<'_, _, _> =
        daplink_vfs::Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);

    let body = HostReader::read_file(&mut vfs, b"DETAILS TXT").unwrap();
    assert_eq!(body.len(), 512);
    let text = std::str::from_utf8(&body).unwrap();
    assert!(text.contains("Interface Version: 9.9.9"));
    assert!(text.contains("Unique ID: SER123"));
}

#[test]
fn drop_image_flows_and_remounts_with_stats() {
    use daplink_vfs::{FatShortName, StatsSink};
    assert_eq!(
        FatShortName::new("APP.UF2").unwrap().as_bytes(),
        b"APP     UF2"
    );

    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs: daplink_vfs::Vfs<'_, _, _> = daplink_vfs::Vfs::new(
        VfsConfig::default(),
        &mut files[..],
        StatsSink::new(),
        NoObserver,
    );

    // The engine is directory-driven: data streams from the declared cluster's first sector.
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 3, 512));
    vfs.write_sector(131, &root);
    vfs.write_sector(
        fat::cluster_to_lba(3),
        &uf2_block(0x0800_0000, &[1, 2, 3, 4]),
    );

    // The sized transfer completes; the volume detaches once the host is quiet, then remounts.
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_MS),
        MediaState::Detached
    );
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);

    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
}

#[test]
fn multi_file_session_decodes_independently() {
    // Two transfers in the same Vfs decode independently; no format or size state carries over.
    use daplink_decoder::Format;
    use daplink_vfs::StatsSink;

    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs: daplink_vfs::Vfs<'_, _, _> = daplink_vfs::Vfs::new(
        VfsConfig::default(),
        &mut files[..],
        StatsSink::new(),
        NoObserver,
    );

    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::DATA_START_LBA, &uf2_block(0x0800_0000, &[1, 2, 3, 4]));
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
    assert_eq!(vfs.sink().fmt, Some(Format::Uf2));
    assert_eq!(vfs.sink().addr_min, 0x0800_0000);

    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_MS),
        MediaState::Detached
    );
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);

    let mut root2 = [0u8; 512];
    root2[0..32].copy_from_slice(&dir_entry(b"NEXT    UF2", 0x20, 2, 512));
    vfs.write_sector(131, &root2);
    vfs.write_sector(fat::DATA_START_LBA, &uf2_block(0x1000_0000, &[9, 9]));
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
    assert_eq!(vfs.sink().fmt, Some(Format::Uf2));
    assert_eq!(vfs.sink().addr_min, 0x1000_0000);
    assert_eq!(vfs.sink().stats.bytes, 2);
}

#[test]
fn details_status_names_the_failure_then_the_success() {
    // Each remount patches `Status:` in DETAILS.TXT without moving any offset.
    use daplink_vfs::{DetailsFile, DeviceInfo, StatsSink};

    let mut details = DetailsFile::new(DeviceInfo {
        version: "0.1.0",
        serial: "SER123",
        reset_count: 0,
    });
    let mut files: [&mut dyn VfsFile; 1] = [&mut details];
    let mut vfs: daplink_vfs::Vfs<'_, _, _> = daplink_vfs::Vfs::new(
        VfsConfig::default(),
        &mut files[..],
        StatsSink::new(),
        NoObserver,
    );

    let status_of = |vfs: &mut daplink_vfs::Vfs<'_, StatsSink, NoObserver>| -> String {
        let body = HostReader::read_file(vfs, b"DETAILS TXT").unwrap();
        assert_eq!(body.len(), 512, "the file's size never moves");
        let text = std::str::from_utf8(&body).unwrap().to_string();
        let off = text.find("Status: ").unwrap() + "Status: ".len();
        text[off..off + 64].trim_end().to_string()
    };

    // A drop whose declared size the stream never meets stalls and fails at the active timeout.
    let mut root = [0u8; 512];
    root[32..64].copy_from_slice(&dir_entry(b"BAD     UF2", 0x20, 3, 4096));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000, &[1, 2]));
    assert!(vfs.is_transfer_active());
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS),
        MediaState::Detached
    );
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Timeout));
    assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);
    let text = status_of(&mut vfs);
    assert!(
        text.contains("Timeout"),
        "status names the failure: {text:?}"
    );

    // An image whose data is never recognized fails loud with the no-data reason.
    let mut root3 = [0u8; 512];
    root3[32..64].copy_from_slice(&dir_entry(b"GHOST   UF2", 0x20, 0, 4096));
    vfs.write_sector(131, &root3);
    vfs.write_sector(fat::cluster_to_lba(3), &[0u8; 512]); // fails the block-magic guard
    assert!(!vfs.is_transfer_active(), "nothing must have started");
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS),
        MediaState::Detached,
        "the bound-but-never-streamed drop fails at the active timeout"
    );
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::NoData));
    assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);
    let text = status_of(&mut vfs);
    assert!(
        text.contains("no image data recognized"),
        "status names the no-data failure: {text:?}"
    );

    // A subsequent successful transfer replaces the failure line on the next remount.
    let mut root2 = [0u8; 512];
    root2[32..64].copy_from_slice(&dir_entry(b"OK      UF2", 0x20, 3, 512));
    vfs.write_sector(131, &root2);
    vfs.write_sector(
        fat::cluster_to_lba(3),
        &uf2_block(0x0800_0000, &[1, 2, 3, 4]),
    );
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_MS),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);
    let text = status_of(&mut vfs);
    assert!(text.ends_with("-> Done"), "status: {text:?}");
    assert!(
        !text.contains("Timeout"),
        "no residue of the failure: {text:?}"
    );
}
