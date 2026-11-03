//! Host write orders replayed against the transfer engine over every chip record's geometry.

use daplink_core::{FlashBackend, FlashPhase};
use daplink_targets::{targets, Target};
use daplink_vfs::{
    fat, DetailsFile, DeviceInfo, FlashSink, MediaState, NoObserver, TransferOutcome, VfsConfig,
    VfsFile,
};

/// Every chip record the registry composes, in registry order.
fn fleet() -> [&'static dyn Target; 2] {
    [&targets::rp2040::RP2040, &targets::stm32f401ce::STM32F401CE]
}

/// One row's default flash region, taken from the record.
struct Geometry {
    name: &'static str,
    flash: (u32, u32),
}

impl Geometry {
    fn of(target: &'static dyn Target) -> Self {
        let flash = target
            .flash_regions()
            .iter()
            .find(|r| r.is_default)
            .or_else(|| target.flash_regions().first())
            .expect("every chip record has a flash region");
        Self {
            name: target.name(),
            flash: (flash.start, flash.end),
        }
    }

    /// Block `n` of an image addressed for THIS row, carrying a 256-byte payload of `fill`.
    fn block(&self, n: u32, num_blocks: u32, fill: u8) -> [u8; 512] {
        let mut b = [0u8; 512];
        b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        b[12..16].copy_from_slice(&(self.flash.0 + n * 256).to_le_bytes()); // target addr
        b[16..20].copy_from_slice(&256u32.to_le_bytes()); // payload size
        b[20..24].copy_from_slice(&n.to_le_bytes());
        b[24..28].copy_from_slice(&num_blocks.to_le_bytes());
        b[32..288].fill(fill);
        b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        b
    }
}

/// A flash backend over one chip record's geometry, recording what it programs.
struct RowBackend {
    ranges: [(u32, u32); 1],
    name: &'static str,
    writes: Vec<(u32, Vec<u8>)>,
    finished: Option<bool>,
    /// How many times the session was closed, so an early release can be shown to happen once.
    finishes: usize,
}

impl RowBackend {
    fn new(g: &Geometry) -> Self {
        Self {
            ranges: [g.flash],
            name: g.name,
            writes: Vec::new(),
            finished: None,
            finishes: 0,
        }
    }

    /// Bytes programmed, concatenated in program order.
    fn programmed(&self) -> Vec<u8> {
        self.writes.iter().flat_map(|(_, d)| d.clone()).collect()
    }
}

impl FlashBackend for RowBackend {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        Ok(())
    }
    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
        self.writes.push((addr, data.to_vec()));
        Ok(())
    }
    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        self.finished = Some(success);
        self.finishes += 1;
        Ok(())
    }
    fn finish_with_reset(&mut self, success: bool, _reset: bool) -> Result<(), FlashPhase> {
        self.finish(success)
    }
    fn flash_ranges(&self) -> &[(u32, u32)] {
        &self.ranges
    }
    fn selected_target_name(&self) -> Option<&str> {
        Some(self.name)
    }
}

type RowVfs<'a> = daplink_vfs::Vfs<'a, FlashSink<RowBackend>, NoObserver>;

/// The dropped file's clusters sit well past the generated files.
const USER_CLUSTER: u16 = 10;

fn user_lba(sector: u32) -> u32 {
    fat::cluster_to_lba(USER_CLUSTER as u32) + sector
}

/// A magicless sector: host housekeeping written into the data region.
fn housekeeping() -> [u8; 512] {
    [0xC3u8; 512]
}

/// Write the dropped file's entry, sized `size`, into root-directory slot 5 (sector 131).
fn write_dirent(vfs: &mut RowVfs<'_>, size: u32) {
    let mut e = [0u8; 32];
    e[0..11].copy_from_slice(b"APP     UF2");
    e[11] = 0x20;
    e[26..28].copy_from_slice(&USER_CLUSTER.to_le_bytes());
    e[28..32].copy_from_slice(&size.to_le_bytes());
    let mut root = [0u8; 512];
    root[5 * 32..5 * 32 + 32].copy_from_slice(&e);
    vfs.write_sector(131, &root);
}

/// Drive one write order through a fresh volume composed for `g`, and hand the closure the VFS.
fn replay(
    g: &Geometry,
    body: impl FnOnce(&mut RowVfs<'_>),
) -> (Option<TransferOutcome>, Vec<u8>, Option<bool>) {
    let mut details = DetailsFile::new(DeviceInfo {
        version: "0.1.0",
        serial: "SER123",
        reset_count: 0,
    });
    let mut files: [&mut dyn VfsFile; 1] = [&mut details];
    let mut vfs: RowVfs<'_> = daplink_vfs::Vfs::new(
        VfsConfig::default(),
        &mut files[..],
        FlashSink::new(RowBackend::new(g)),
        NoObserver,
    );
    body(&mut vfs);
    let outcome = vfs.sink().outcome();
    let backend = vfs.sink().backend();
    (outcome, backend.programmed(), backend.finished)
}

/// Data clusters before the directory entry: content starts it, the late entry completes it.
#[test]
fn data_first_with_the_dirent_last_completes_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, programmed, finished) = replay(&g, |vfs| {
            vfs.write_sector(user_lba(0), &g.block(0, 2, 0xA1));
            assert!(
                vfs.is_transfer_active(),
                "{}: block magic must sniff-start an unbound image",
                g.name
            );
            vfs.write_sector(user_lba(1), &g.block(1, 2, 0xB2));
            write_dirent(vfs, 1024);
        });
        assert_eq!(outcome, Some(TransferOutcome::Done), "{}", g.name);
        assert_eq!(finished, Some(true), "{}", g.name);
        let mut expected = vec![0xA1u8; 256];
        expected.extend_from_slice(&[0xB2u8; 256]);
        assert_eq!(programmed, expected, "{}", g.name);
    }
}

/// The third payload sector before the second: a host reorder, reported as `OutOfOrder`.
#[test]
fn a_reordered_data_first_drop_reports_out_of_order_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, _programmed, finished) = replay(&g, |vfs| {
            vfs.write_sector(user_lba(0), &g.block(0, 3, 0xA1));
            assert!(vfs.is_transfer_active(), "{}", g.name);
            vfs.write_sector(user_lba(2), &g.block(2, 3, 0xC3));
            vfs.write_sector(user_lba(1), &g.block(1, 3, 0xA5));
            write_dirent(vfs, 1536);
            assert!(
                vfs.is_transfer_active(),
                "{}: a stream short of its known size stays open to the active timeout",
                g.name
            );
            assert_eq!(
                vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS),
                MediaState::Detached,
                "{}",
                g.name
            );
        });
        assert_eq!(outcome, Some(TransferOutcome::OutOfOrder), "{}", g.name);
        assert_eq!(
            finished,
            Some(false),
            "{}: the flash session closes as failed",
            g.name
        );
    }
}

/// The same reorder with no entry ever: it must not complete as `Done` with a hole.
#[test]
fn a_reordered_drop_with_no_dirent_fails_loud_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, programmed, finished) = replay(&g, |vfs| {
            vfs.write_sector(user_lba(0), &g.block(0, 6, 0xA1));
            assert!(vfs.is_transfer_active(), "{}", g.name);
            // The host writes past the hole and never comes back for it.
            vfs.write_sector(user_lba(2), &g.block(2, 6, 0xC3));
            vfs.write_sector(user_lba(1), &g.block(1, 6, 0xA5));
            for sector in 3..6 {
                vfs.write_sector(user_lba(sector), &g.block(sector, 6, 0xA5));
            }
            assert_eq!(
                vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
                MediaState::Detached,
                "{}",
                g.name
            );
        });
        assert_eq!(
            outcome,
            Some(TransferOutcome::OutOfOrder),
            "{}: a hole in the stream is not a Done",
            g.name
        );
        assert_eq!(finished, Some(false), "{}", g.name);
        assert!(
            programmed.len() < 6 * 256,
            "{}: the image was short, which is exactly why it must not report success",
            g.name
        );
    }
}

/// Two skipped sectors never returned: a hole however early the later block arrived.
#[test]
fn a_hole_below_the_stalled_front_still_fails_loud_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, programmed, finished) = replay(&g, |vfs| {
            vfs.write_sector(user_lba(0), &g.block(0, 5, 0xA1));
            assert!(vfs.is_transfer_active(), "{}", g.name);
            // Block 4 arrives early; block 3 never arrives at all.
            vfs.write_sector(user_lba(4), &g.block(4, 5, 0xC3));
            vfs.write_sector(user_lba(1), &g.block(1, 5, 0xA5));
            vfs.write_sector(user_lba(2), &g.block(2, 5, 0xA5));
            write_dirent(vfs, 5 * 512);
            assert_eq!(
                vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS),
                MediaState::Detached,
                "{}",
                g.name
            );
        });
        assert_eq!(outcome, Some(TransferOutcome::OutOfOrder), "{}", g.name);
        assert_eq!(finished, Some(false), "{}", g.name);
        assert!(
            programmed.len() < 5 * 256,
            "{}: the image was short, which is what the report has to say",
            g.name
        );
    }
}

/// A magicless early write instead keeps the `Timeout` verdict of a host that stopped.
#[test]
fn housekeeping_beside_a_stalled_stream_stays_a_timeout_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, _programmed, finished) = replay(&g, |vfs| {
            vfs.write_sector(user_lba(0), &g.block(0, 5, 0xA1));
            vfs.write_sector(user_lba(4), &housekeeping());
            vfs.write_sector(user_lba(1), &g.block(1, 5, 0xA5));
            vfs.write_sector(user_lba(2), &g.block(2, 5, 0xA5));
            write_dirent(vfs, 5 * 512);
            assert_eq!(
                vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS),
                MediaState::Detached,
                "{}",
                g.name
            );
        });
        assert_eq!(outcome, Some(TransferOutcome::Timeout), "{}", g.name);
        assert_eq!(finished, Some(false), "{}", g.name);
    }
}

/// Re-delivering the skipped sector at the front heals the stream.
#[test]
fn a_re_delivered_sector_heals_the_reorder_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, programmed, finished) = replay(&g, |vfs| {
            vfs.write_sector(user_lba(0), &g.block(0, 4, 0xA1));
            vfs.write_sector(user_lba(2), &g.block(2, 4, 0xC3));
            for sector in 1..4 {
                vfs.write_sector(user_lba(sector), &g.block(sector, 4, 0xA5));
            }
            write_dirent(vfs, 4 * 512);
        });
        assert_eq!(outcome, Some(TransferOutcome::Done), "{}", g.name);
        assert_eq!(finished, Some(true), "{}", g.name);
        assert_eq!(programmed.len(), 4 * 256, "{}", g.name);
    }
}

/// The file's own block with its magic destroyed fails at that write as a decode failure.
#[test]
fn a_corrupt_block_inside_the_window_fails_at_that_write_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, programmed, finished) = replay(&g, |vfs| {
            write_dirent(vfs, 3 * 512);
            vfs.write_sector(user_lba(0), &g.block(0, 3, 0xA1));
            assert!(vfs.is_transfer_active(), "{}", g.name);
            // Block 2 of the file's own data lands with the magic gone.
            vfs.write_sector(user_lba(2), &housekeeping());
            assert!(
                !vfs.is_transfer_active(),
                "{}: the corrupt block ends the drop at once",
                g.name
            );
            assert_eq!(
                vfs.tick(1),
                MediaState::Detached,
                "{}: with no idle wait before the volume remounts",
                g.name
            );
        });
        assert_eq!(outcome, Some(TransferOutcome::DecodeError), "{}", g.name);
        assert_eq!(finished, Some(false), "{}", g.name);
        assert_eq!(
            programmed.len(),
            256,
            "{}: only the blocks before the corruption are programmed",
            g.name
        );
    }
}

/// That corruption closes the flash session at the corrupt write, exactly once.
#[test]
fn a_corrupt_block_releases_the_session_once_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let mut details = DetailsFile::new(DeviceInfo {
            version: "0.1.0",
            serial: "SER123",
            reset_count: 0,
        });
        let mut files: [&mut dyn VfsFile; 1] = [&mut details];
        let mut vfs: RowVfs<'_> = daplink_vfs::Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(RowBackend::new(&g)),
            NoObserver,
        );
        write_dirent(&mut vfs, 3 * 512);
        vfs.write_sector(user_lba(0), &g.block(0, 3, 0xA1));
        vfs.write_sector(user_lba(2), &housekeeping());
        assert_eq!(vfs.sink().backend().finishes, 1, "{}", g.name);
        // Whatever the host writes next, the closed session is not closed again.
        vfs.write_sector(user_lba(1), &g.block(1, 3, 0xA5));
        vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS);
        assert_eq!(
            vfs.sink().backend().finishes,
            1,
            "{}: the session is released exactly once",
            g.name
        );
    }
}

/// A sized reorder is judged on the short gapped window, not the active one.
#[test]
fn a_sized_forward_gap_reports_out_of_order_without_the_active_wait_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, _programmed, finished) = replay(&g, |vfs| {
            write_dirent(vfs, 3 * 512);
            vfs.write_sector(user_lba(0), &g.block(0, 3, 0xA1));
            vfs.write_sector(user_lba(2), &g.block(2, 3, 0xC3));
            assert_eq!(
                vfs.transfer_timeout(),
                daplink_vfs::MSC_TIMEOUT_GAPPED_MS,
                "{}: the discarded sector shortens the idle window",
                g.name
            );
            assert_eq!(
                vfs.tick(daplink_vfs::MSC_TIMEOUT_GAPPED_MS),
                MediaState::Detached,
                "{}",
                g.name
            );
        });
        assert_eq!(outcome, Some(TransferOutcome::OutOfOrder), "{}", g.name);
        assert_eq!(finished, Some(false), "{}", g.name);
    }
}

/// A gap healed within that window still completes.
#[test]
fn a_healed_gap_survives_the_shortened_window_on_every_row() {
    for target in fleet() {
        let g = Geometry::of(target);
        let (outcome, programmed, finished) = replay(&g, |vfs| {
            write_dirent(vfs, 3 * 512);
            vfs.write_sector(user_lba(0), &g.block(0, 3, 0xA1));
            vfs.write_sector(user_lba(2), &g.block(2, 3, 0xC3));
            assert_eq!(
                vfs.tick(daplink_vfs::MSC_TIMEOUT_GAPPED_MS - 1),
                MediaState::Ready,
                "{}",
                g.name
            );
            vfs.write_sector(user_lba(1), &g.block(1, 3, 0xA5));
            vfs.write_sector(user_lba(2), &g.block(2, 3, 0xC3));
        });
        assert_eq!(outcome, Some(TransferOutcome::Done), "{}", g.name);
        assert_eq!(finished, Some(true), "{}", g.name);
        assert_eq!(programmed.len(), 3 * 256, "{}", g.name);
    }
}

/// The sweep reads each record's own geometry, and the rows differ.
#[test]
fn the_swept_geometry_is_the_registry_and_the_rows_differ() {
    let rows: Vec<Geometry> = fleet().into_iter().map(Geometry::of).collect();
    assert_eq!(rows.len(), fleet().len());
    for g in &rows {
        assert!(g.flash.1 > g.flash.0, "{}: empty flash window", g.name);
    }
    let flashes: Vec<(u32, u32)> = rows.iter().map(|g| g.flash).collect();
    assert!(
        flashes.windows(2).any(|w| w[0] != w[1]),
        "a sweep over identical windows is one window"
    );
}
