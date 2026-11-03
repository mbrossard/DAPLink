//! Tick-edge tests: timeouts and remount delay over 1 ms steps, pinned at their thresholds.

mod common;

use common::{dir_entry, uf2_block};
use daplink_vfs::transfer::TransferOutcome;
use daplink_vfs::{fat, MediaState, NoObserver, StatsSink, VfsConfig, VfsFile};

fn make_vfs<'a>(
    files: &'a mut [&'a mut dyn VfsFile],
) -> daplink_vfs::Vfs<'a, StatsSink, NoObserver> {
    daplink_vfs::Vfs::new(VfsConfig::default(), files, StatsSink::new(), NoObserver)
}

/// Start a transfer far short of its declared size, which can only time out.
fn start_unfinishable_stream(vfs: &mut daplink_vfs::Vfs<'_, StatsSink, NoObserver>) {
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"BAD     UF2", 0x20, 3, 4096));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000, &[0; 4]));
    assert!(vfs.is_transfer_active());
}

/// Start an unsized uf2 transfer the finishable idle timeout completes.
fn start_finishable_stream(vfs: &mut daplink_vfs::Vfs<'_, StatsSink, NoObserver>) {
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 0));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::DATA_START_LBA, &uf2_block(0x0800_0000, &[0; 4]));
    assert!(vfs.is_transfer_active());
}

#[test]
fn active_timeout_accumulates_in_small_steps() {
    // The active timeout accumulates over 1 ms ticks exactly: 19,999 stay Ready, 20,000 fire.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_unfinishable_stream(&mut vfs);

    for _ in 0..daplink_vfs::MSC_TIMEOUT_ACTIVE_MS - 1 {
        assert_eq!(vfs.tick(1), MediaState::Ready);
    }
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Timeout));
}

#[test]
fn finishable_timeout_boundary() {
    // An unsized transfer completes on the unsized idle timeout, not the sized one.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_finishable_stream(&mut vfs);

    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS - 1),
        MediaState::Ready
    );
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
}

#[test]
fn paused_unsized_stream_does_not_complete_early() {
    // A paused unsized copy must not be declared Done during a 500 ms gap.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_finishable_stream(&mut vfs);

    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_MS),
        MediaState::Ready,
        "a gap at the old tight threshold must not complete an unsized stream early"
    );
    assert!(vfs.is_transfer_active(), "the transfer is still open");
    // The host resumes streaming: the same transfer continues rather than restarting.
    vfs.write_sector(fat::DATA_START_LBA + 1, &uf2_block(0x0800_0100, &[0; 4]));
    assert!(
        vfs.is_transfer_active(),
        "resumed data extends the same transfer, not a second one"
    );
}

#[test]
fn known_size_unmet_stall_uses_active_timeout_and_fails_loud() {
    // A sized stream stalled short of its size times out loud on the active timeout, not Done.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 3, 2048));
    vfs.write_sector(131, &root);
    // 512 of the declared 2048 bytes, then silence.
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000, &[0; 4]));
    assert!(vfs.is_transfer_active());

    // Well past the finishable threshold: the short stream must NOT complete.
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_ACTIVE_MS - 1),
        MediaState::Ready,
        "a stream short of its known size must not complete on the finishable timeout"
    );
    assert_eq!(vfs.sink().outcome(), None);
    // The active timeout fires: fail loud.
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Timeout));
}

#[test]
fn bound_never_streamed_boundary() {
    // A bound image with no accepted data fails at exactly the active timeout.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 4096));
    vfs.write_sector(131, &root);
    // The only data the host writes fails the content guard: nothing starts.
    vfs.write_sector(fat::DATA_START_LBA, &[0u8; 512]);
    assert!(!vfs.is_transfer_active());

    for _ in 0..daplink_vfs::MSC_TIMEOUT_ACTIVE_MS - 1 {
        assert_eq!(vfs.tick(1), MediaState::Ready);
    }
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::NoData));
}

#[test]
fn reconnect_boundary_and_write_resets_idle() {
    // Remount boundary: detached at 2,499 ms, remounted with the media-changed latch at 2,500.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached
    );
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_RECONNECT_MS - 1),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(1), MediaState::Ready);
    assert!(vfs.take_media_changed(), "a remount raises the latch");

    // A mid-transfer write restarts the idle window.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS - 100),
        MediaState::Ready
    );
    // The stream's next sector, restarting the idle window.
    vfs.write_sector(fat::DATA_START_LBA + 1, &uf2_block(0x0800_0100, &[0; 4]));
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS - 1),
        MediaState::Ready,
        "the write reset the idle accumulation"
    );
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
}

/// An idle-timeout detach after a prior remount cycle re-arms exactly `MSC_RECONNECT_MS`.
#[test]
fn idle_timeout_detach_after_prior_remount_schedules_exact_reconnect() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);

    // Cycle 1: a transfer completes on the finishable idle timeout, detaches, and remounts.
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);
    assert!(vfs.take_media_changed());

    // Cycle 2: another transfer ends via the idle-timeout detach edge.
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached,
        "idle-timeout path detaches the volume"
    );
    // The delay is exactly MSC_RECONNECT_MS.
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_RECONNECT_MS - 1),
        MediaState::Detached,
        "remount must not be early"
    );
    assert_eq!(
        vfs.tick(1),
        MediaState::Ready,
        "remount must land at exactly MSC_RECONNECT_MS, not the far future"
    );
    assert!(vfs.take_media_changed(), "the remount republishes status");
}

/// The finished-latch detach edge also re-arms exactly `MSC_RECONNECT_MS` after a cycle.
#[test]
fn finished_latch_detach_after_prior_remount_schedules_exact_reconnect() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);

    // Cycle 1: complete a transfer and remount.
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);

    // Cycle 2: a sized transfer completed by its last sector detaches after the write silence.
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 3, 512));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000, &[0; 4]));
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_MS),
        MediaState::Detached,
        "finished-latch detach"
    );

    assert_eq!(
        vfs.tick(daplink_vfs::MSC_RECONNECT_MS - 1),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(1), MediaState::Ready);
}

/// Zero-elapsed ticks leave the remount schedule intact; an oversized delta saturates.
#[test]
fn detach_schedule_survives_zero_and_oversized_ticks() {
    // Zero-elapsed storm: the countdown must not move (no early remount, no deferral).
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached
    );
    for _ in 0..10_000 {
        assert_eq!(vfs.tick(0), MediaState::Detached);
    }
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_RECONNECT_MS - 1),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(1), MediaState::Ready);

    // Oversized elapsed: saturates to "due now"; no wraparound can postpone the remount.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(u32::MAX), MediaState::Ready);
}

/// A corrupt block in the declared window ends the transfer at once; remount on the next tick.
#[test]
fn a_corrupt_block_needs_no_idle_wait() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 3, 4 * 512));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000, &[0; 4]));
    assert!(vfs.is_transfer_active());

    vfs.write_sector(fat::cluster_to_lba(3) + 2, &[0x5Au8; 512]);
    assert!(!vfs.is_transfer_active(), "the corrupt write ended it");
    assert_eq!(vfs.transfer_timeout(), 0, "nothing is waiting on a timeout");
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::DecodeError));
}

/// A sized stream with a gap waits only the gapped window before reporting the reorder.
#[test]
fn gapped_timeout_boundary() {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 3, 4 * 512));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000, &[0; 4]));
    vfs.write_sector(fat::cluster_to_lba(3) + 2, &uf2_block(0x0800_0200, &[0; 4]));
    assert_eq!(
        vfs.transfer_timeout(),
        daplink_vfs::MSC_TIMEOUT_GAPPED_MS,
        "the discarded sector shortens the wait"
    );

    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_GAPPED_MS - 1),
        MediaState::Ready
    );
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert!(vfs.take_transfer_finished());
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::OutOfOrder));
}
