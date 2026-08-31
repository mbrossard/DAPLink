//! Tick-edge tests: the transfer timeouts and the remount delay accumulated across many small
//! `dt` steps (the real firmware shape: 1 ms SysTick deltas), with the threshold boundaries
//! pinned exactly (threshold - 1 does not fire; one more millisecond does).

use daplink_vfs::transfer::TransferOutcome;
use daplink_vfs::{fat, MediaState, NoObserver, StatsSink, VfsConfig, VfsFile};

/// Build a root-directory entry for a dropped file.
fn dir_entry(name: &[u8; 11], attr: u8, first_cluster: u16, size: u32) -> [u8; 32] {
    let mut e = [0u8; 32];
    e[0..11].copy_from_slice(name);
    e[11] = attr;
    e[26..28].copy_from_slice(&first_cluster.to_le_bytes());
    e[28..32].copy_from_slice(&size.to_le_bytes());
    e
}

fn make_vfs<'a>(
    files: &'a mut [&'a mut dyn VfsFile],
) -> daplink_vfs::Vfs<'a, StatsSink, NoObserver> {
    daplink_vfs::Vfs::new(VfsConfig::default(), files, StatsSink::new(), NoObserver)
}

/// One well-formed 512-byte UF2 block carrying four payload bytes at `addr`.
fn uf2_block(addr: u32) -> [u8; 512] {
    let mut b = [0u8; 512];
    b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
    b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
    b[12..16].copy_from_slice(&addr.to_le_bytes()); // target addr
    b[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
    b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
    b
}

/// Start an unfinishable transfer: a stream whose dirent declared a size it is far short of
/// cannot complete and can only time out (the active-timeout path).
fn start_unfinishable_stream(vfs: &mut daplink_vfs::Vfs<'_, StatsSink, NoObserver>) {
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"BAD     UF2", 0x20, 3, 4096));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000));
    assert!(vfs.is_transfer_active());
}

/// Start a finishable transfer: an unsized uf2 stream has no in-stream terminator and ends at a
/// block boundary, so the finishable idle timeout completes it. The dirent is bound first with
/// the cluster still unallocated, so the window is unknown and the block-magic content guard is
/// what lets the first sector start it.
fn start_finishable_stream(vfs: &mut daplink_vfs::Vfs<'_, StatsSink, NoObserver>) {
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 0));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::DATA_START_LBA, &uf2_block(0x0800_0000));
    assert!(vfs.is_transfer_active());
}

#[test]
fn active_timeout_accumulates_in_small_steps() {
    // The 20 s active timeout must accumulate across many 1 ms deltas exactly as it would from
    // one big jump: 19,999 single-millisecond ticks stay Ready, the 20,000th fires the timeout.
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
    // An unsized transfer -- no dirent has ever supplied a size -- completes on the
    // longer unsized idle timeout, not the tight sized-and-met one: one ms short of it stays
    // Ready, one more completes the transfer and detaches the volume. This is also the
    // regression case for a paused-but-not-finished unsized copy: a >= 500 ms gap (the OLD
    // threshold, still exercised by `paused_unsized_stream_does_not_complete_early` below) must
    // NOT complete it early.
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
    // The composed failure this backs out of: a paused (not finished) unsized copy must
    // survive a gap at least as long as the OLD 500 ms finishable threshold without being
    // declared Done. A premature Done here would flash whatever prefix had streamed so far and
    // report success, then the remount would clear the directory shadow so the host's resumed
    // writes re-diff as a fresh Created and start a second transfer over the same drop.
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
    vfs.write_sector(fat::DATA_START_LBA + 1, &uf2_block(0x0800_0100));
    assert!(
        vfs.is_transfer_active(),
        "resumed data extends the same transfer, not a second one"
    );
}

#[test]
fn known_size_unmet_stall_uses_active_timeout_and_fails_loud() {
    // A stream whose dirent size is known but which stalls SHORT of it must not complete a
    // truncated prefix on the 500 ms finishable timeout: the 20 s active timeout applies, and
    // when it fires the outcome is a loud Timeout, not Done.
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 3, 2048));
    vfs.write_sector(131, &root);
    // 512 of the declared 2048 bytes, then silence.
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000));
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
    // A recognized image dirent with no accepted data is under the active timeout: 19,999 ms of
    // silence stays Ready, one more millisecond fails the drop loud and detaches the volume.
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
    // Remount boundary: 2,499 ms after detaching the volume is still detached; one more
    // millisecond remounts it and raises the media-changed latch.
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

    // A mid-transfer data-sector write resets the accumulated idle: after part of the unsized
    // idle window plus a write, the window restarts (one ms short of it stays Ready, one more
    // fires).
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs = make_vfs(&mut files);
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS - 100),
        MediaState::Ready
    );
    // The stream's next sector, restarting the idle window.
    vfs.write_sector(fat::DATA_START_LBA + 1, &uf2_block(0x0800_0100));
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS - 1),
        MediaState::Ready,
        "the write reset the idle accumulation"
    );
    assert_eq!(vfs.tick(1), MediaState::Detached);
    assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
}

/// Regression (bench): a transfer that ends via the IDLE-TIMEOUT detach edge after a prior
/// remount cycle must schedule its remount at exactly `MSC_RECONNECT_MS`. The first cycle
/// leaves the volume's internal remount timer in a used state; the second detach must re-arm
/// the schedule from the constant rather than inherit anything stale -- a stale value here
/// once pushed the remount into the far future and the drive never came back.
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

    // Cycle 2 (the bench sequence): another transfer ends via the idle-timeout detach edge.
    start_finishable_stream(&mut vfs);
    assert_eq!(
        vfs.tick(daplink_vfs::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
        MediaState::Detached,
        "idle-timeout path detaches the volume"
    );
    // The scheduled delay is exactly MSC_RECONNECT_MS: one millisecond short stays detached,
    // one more remounts.
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

/// The FINISHED-LATCH detach edge (a self-terminating stream, e.g. a final uf2 block) re-arms
/// the same exact `MSC_RECONNECT_MS` schedule after a prior remount cycle.
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

    // Cycle 2: a sized transfer whose last sector completes it -- the finished latch fires on
    // the next tick without any idle accumulation.
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 3, 512));
    vfs.write_sector(131, &root);
    vfs.write_sector(fat::cluster_to_lba(3), &uf2_block(0x0800_0000));
    assert_eq!(vfs.tick(1), MediaState::Detached, "finished-latch detach");

    assert_eq!(
        vfs.tick(daplink_vfs::MSC_RECONNECT_MS - 1),
        MediaState::Detached
    );
    assert_eq!(vfs.tick(1), MediaState::Ready);
}

/// Degenerate tick sequences around a detach must neither remount early nor defer the remount:
/// a burst of zero-elapsed ticks (a sub-millisecond service loop) leaves the schedule intact,
/// and an oversized elapsed value (a caller's stale-timestamp delta) saturates into an
/// immediate remount instead of wrapping into a far-future one.
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
