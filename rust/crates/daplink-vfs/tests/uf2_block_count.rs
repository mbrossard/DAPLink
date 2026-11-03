//! A uf2 stream that ends short of its own `numBlocks` fails, whichever signal ended it.

mod common;

use common::{dir_entry, uf2_numbered};
use daplink_vfs::transfer::TransferOutcome;
use daplink_vfs::{fat, MediaState, NoObserver, StatsSink, VfsConfig, VfsFile};

type V<'a> = daplink_vfs::Vfs<'a, StatsSink, NoObserver>;

fn make_vfs<'a>(files: &'a mut [&'a mut dyn VfsFile]) -> V<'a> {
    daplink_vfs::Vfs::new(VfsConfig::default(), files, StatsSink::new(), NoObserver)
}

fn root(size: u32) -> [u8; 512] {
    let mut r = [0u8; 512];
    r[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, size));
    r
}

/// Write blocks `0..sent` of `of` as consecutive sectors of the file at cluster 2.
fn stream(vfs: &mut V<'_>, sent: u32, of: u32) {
    for n in 0..sent {
        let b = uf2_numbered(n, of, 0x1000_0000 + n * 256, &[n as u8; 256]);
        vfs.write_sector(fat::DATA_START_LBA + n, &b);
    }
}

#[test]
fn a_sized_copy_short_of_its_block_count_fails() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    vfs.write_sector(131, &root(2 * 512));
    stream(&mut vfs, 2, 4);
    assert!(!vfs.is_transfer_active());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::DecodeError));
}

#[test]
fn an_unsized_stream_that_stops_short_fails_at_its_timeout() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    vfs.write_sector(131, &root(0));
    stream(&mut vfs, 2, 4);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached
    );
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::DecodeError));
}

#[test]
fn a_complete_numbered_stream_still_succeeds() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    vfs.write_sector(131, &root(4 * 512));
    stream(&mut vfs, 4, 4);
    assert!(!vfs.is_transfer_active());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
}
