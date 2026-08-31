//! Sector-I/O seam between a USB mass-storage class and the virtual volume.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::queue::{MscOp, OpConsumer, OpProducer, ReplySlot, Sector};
use crate::transfer::{TransferSink, VfsObserver};
use crate::{MediaState, Vfs};

/// Whether a sector operation completed or must be retried.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SectorPoll {
    /// The operation completed.
    Ready,
    /// The operation is not yet complete; poll again.
    Pending,
}

/// A block device presented to the USB mass-storage class.
pub trait SectorIo {
    /// Whether the medium is present and ready.
    fn media_ready(&self) -> bool;
    /// Read logical block `lba` into `buf`.
    fn read(&mut self, lba: u32, buf: &mut [u8; 512]) -> SectorPoll;
    /// Write `buf` to logical block `lba`.
    fn write(&mut self, lba: u32, buf: &[u8; 512]) -> SectorPoll;
    /// Return and clear a pending media-changed edge, for the class to raise SCSI UNIT ATTENTION.
    /// Defaults `false`, for a backend that signals remounts some other way (the epoch).
    fn media_changed(&mut self) -> bool {
        false
    }
    /// Note that the transport entered (`true`) or left (`false`) a host command's data phase.
    /// A backend whose medium can change on its own must hold that change until the phase closes.
    fn set_data_phase(&mut self, _open: bool) {}
}

/// A [`SectorIo`] that services each request against the volume inline (always [`SectorPoll::Ready`]).
pub struct DirectIo<'v, 'a, S: TransferSink, O: VfsObserver> {
    vfs: &'v mut Vfs<'a, S, O>,
}

impl<'v, 'a, S: TransferSink, O: VfsObserver> DirectIo<'v, 'a, S, O> {
    /// Wrap a mutable reference to `vfs`.
    pub fn new(vfs: &'v mut Vfs<'a, S, O>) -> Self {
        Self { vfs }
    }
}

impl<S: TransferSink, O: VfsObserver> SectorIo for DirectIo<'_, '_, S, O> {
    fn media_ready(&self) -> bool {
        self.vfs.media() == MediaState::Ready
    }

    fn read(&mut self, lba: u32, buf: &mut [u8; 512]) -> SectorPoll {
        self.vfs.read_sector(lba, buf);
        SectorPoll::Ready
    }

    fn write(&mut self, lba: u32, buf: &[u8; 512]) -> SectorPoll {
        self.vfs.write_sector(lba, buf);
        SectorPoll::Ready
    }

    fn media_changed(&mut self) -> bool {
        self.vfs.take_media_changed()
    }

    fn set_data_phase(&mut self, open: bool) {
        self.vfs.set_data_phase(open);
    }
}

/// A [`SectorIo`] that offloads each request to an application-side worker: reads park as
/// [`SectorPoll::Pending`] until a reply lands, writes enqueue or report `Pending` when full.
pub struct QueuedIo<'a, 'q, 'r, 'o, const DEPTH: usize = 4> {
    ops: &'a mut OpProducer<'q, DEPTH>,
    replies: &'r ReplySlot,
    media_ready: bool,
    epoch: u32,
    /// The `(lba, epoch)` of the read in flight, if any. Caller-owned so it survives a service
    /// pass; the caller must also clear it on the USB and Bulk-Only reset paths.
    outstanding: &'o mut Option<(u32, u32)>,
}

impl<'a, 'q, 'r, 'o, const DEPTH: usize> QueuedIo<'a, 'q, 'r, 'o, DEPTH> {
    /// Wrap the producer and reply slot for one service pass at media epoch `epoch`.
    pub fn new(
        ops: &'a mut OpProducer<'q, DEPTH>,
        replies: &'r ReplySlot,
        media_ready: bool,
        epoch: u32,
        outstanding: &'o mut Option<(u32, u32)>,
    ) -> Self {
        Self {
            ops,
            replies,
            media_ready,
            epoch,
            outstanding,
        }
    }
}

impl<const DEPTH: usize> SectorIo for QueuedIo<'_, '_, '_, '_, DEPTH> {
    fn media_ready(&self) -> bool {
        self.media_ready
    }

    fn read(&mut self, lba: u32, buf: &mut [u8; 512]) -> SectorPoll {
        if let Some(data) = self.replies.take(lba, self.epoch) {
            *buf = data;
            *self.outstanding = None;
            SectorPoll::Ready
        } else {
            // Re-emit only when this exact request is not already in flight.
            if *self.outstanding != Some((lba, self.epoch)) {
                // Marked only when the push succeeds: a dropped request must stay re-emittable.
                if self
                    .ops
                    .try_push(MscOp::Read {
                        lba,
                        epoch: self.epoch,
                    })
                    .is_ok()
                {
                    *self.outstanding = Some((lba, self.epoch));
                }
            }
            SectorPoll::Pending
        }
    }

    fn write(&mut self, lba: u32, buf: &[u8; 512]) -> SectorPoll {
        match self.ops.try_push(MscOp::Write(Sector {
            lba,
            data: *buf,
            epoch: self.epoch,
        })) {
            Ok(()) => SectorPoll::Ready,
            Err(_) => SectorPoll::Pending,
        }
    }

    fn set_data_phase(&mut self, open: bool) {
        // A full queue drops the marker; `MSC_DETACH_DEFER_MS` still ends a lost close's wait.
        let _ = self.ops.try_push(MscOp::DataPhase(open));
    }
}

/// The outcome of one [`msc_worker_step`] call.
pub struct WorkerStep {
    /// Whether an operation was dequeued and serviced this call.
    pub did_work: bool,
    /// The media state after advancing the volume's timers.
    pub media: MediaState,
}

/// Advance the worker one step: service at most one queued operation, tick the volume by
/// `elapsed_ms`, and republish the shared media state, bumping `media_epoch` on each remount.
pub fn msc_worker_step<S: TransferSink, O: VfsObserver, const DEPTH: usize>(
    vfs: &mut Vfs<'_, S, O>,
    ops: &mut OpConsumer<'_, DEPTH>,
    replies: &ReplySlot,
    media_ready: &AtomicBool,
    media_epoch: &AtomicU32,
    elapsed_ms: u32,
) -> WorkerStep {
    let mut did_work = false;
    if let Some(op) = ops.pop() {
        did_work = true;
        match op {
            MscOp::Read { lba, epoch } => {
                let current = media_epoch.load(Ordering::Acquire);
                // A read from a previous mount is stale; the USB side re-emits it.
                if epoch == current {
                    let mut buf = [0u8; 512];
                    vfs.read_sector(lba, &mut buf);
                    replies.post(Sector {
                        lba,
                        data: buf,
                        epoch: current,
                    });
                }
            }
            MscOp::Write(s) => {
                let current = media_epoch.load(Ordering::Acquire);
                // A write from a previous mount must never reach the new one.
                if s.epoch == current {
                    vfs.write_sector(s.lba, &s.data);
                }
            }
            // Transport state, not a mounting: applied whatever the epoch.
            MscOp::DataPhase(open) => vfs.set_data_phase(open),
        }
    }

    let was_detached = vfs.media() == MediaState::Detached;
    let media = vfs.tick(elapsed_ms);
    if was_detached && media == MediaState::Ready {
        // Sole writer, so load/store is race-free (thumbv6m has no read-modify-write atomic).
        let next = media_epoch.load(Ordering::Relaxed).wrapping_add(1);
        media_epoch.store(next, Ordering::Release);
    }
    media_ready.store(media == MediaState::Ready, Ordering::Release);

    WorkerStep { did_work, media }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::queue::OpQueue;
    use crate::{NoObserver, NullSink, Vfs, VfsConfig};

    fn make_vfs<'a>(files: &'a mut [&'a mut dyn crate::VfsFile]) -> Vfs<'a, NullSink, NoObserver> {
        Vfs::new(VfsConfig::default(), files, NullSink, NoObserver)
    }

    /// One well-formed 512-byte UF2 block, the smallest thing that starts a transfer.
    fn uf2_block() -> [u8; 512] {
        let mut b = [0u8; 512];
        b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        b[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        b[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        b
    }

    fn dir_entry(name: &[u8; 11], attr: u8, first_cluster: u16, size: u32) -> [u8; 32] {
        let mut e = [0u8; 32];
        e[0..11].copy_from_slice(name);
        e[11] = attr;
        e[26..28].copy_from_slice(&first_cluster.to_le_bytes());
        e[28..32].copy_from_slice(&size.to_le_bytes());
        e
    }

    #[test]
    fn direct_io_reads_the_volume_and_its_writes_reach_the_transfer_engine() {
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 0));
        {
            let mut io = DirectIo::new(&mut vfs);
            assert!(io.media_ready());

            let mut buf = [0u8; 512];
            assert_eq!(io.read(0, &mut buf), SectorPoll::Ready);
            assert_eq!((buf[510], buf[511]), (0x55, 0xAA), "boot-sector signature");

            assert_eq!(io.write(131, &root), SectorPoll::Ready);
            assert_eq!(
                io.write(crate::fat::DATA_START_LBA, &uf2_block()),
                SectorPoll::Ready
            );
        }
        assert!(vfs.is_transfer_active());
    }

    #[test]
    fn direct_io_reports_the_detach_and_the_remounts_media_changed_edge() {
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 0));
        vfs.write_sector(131, &root);
        {
            let mut io = DirectIo::new(&mut vfs);
            assert!(!io.media_changed(), "the initial mount raises no edge");
            assert_eq!(
                io.write(crate::fat::DATA_START_LBA, &uf2_block()),
                SectorPoll::Ready
            );
        }
        assert_eq!(
            vfs.tick(crate::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS + 1),
            MediaState::Detached
        );
        assert!(!DirectIo::new(&mut vfs).media_ready());

        assert_eq!(vfs.tick(crate::MSC_RECONNECT_MS), MediaState::Ready);
        let mut io = DirectIo::new(&mut vfs);
        assert!(io.media_ready());
        assert!(io.media_changed(), "the remount raises one edge");
        assert!(!io.media_changed(), "and taking it clears it");
    }

    #[test]
    fn direct_io_data_phase_holds_the_detach_to_the_command_boundary() {
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
        {
            let mut io = DirectIo::new(&mut vfs);
            io.set_data_phase(true);
            assert_eq!(io.write(131, &root), SectorPoll::Ready);
            assert_eq!(
                io.write(crate::fat::DATA_START_LBA, &uf2_block()),
                SectorPoll::Ready
            );
        }
        // The image met its size, but the tick under the open phase must only arm the detach.
        assert_eq!(vfs.tick(1), MediaState::Ready);
        assert_eq!(vfs.tick(1), MediaState::Ready);

        DirectIo::new(&mut vfs).set_data_phase(false);
        assert_eq!(vfs.media(), MediaState::Detached, "the close commits it");
    }

    #[test]
    fn queued_data_phase_marker_is_dropped_when_the_queue_is_full() {
        let mut q: OpQueue<2> = OpQueue::new(); // usable capacity: 1
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let mut outstanding = None;
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.write(9, &[0u8; 512]), SectorPoll::Ready);
            io.set_data_phase(true);
        }
        assert_eq!(c.len(), 1, "the marker is dropped, not queued");
        assert!(matches!(c.pop(), Some(MscOp::Write(_))));
        assert!(c.pop().is_none());
    }

    #[test]
    fn a_lost_data_phase_close_still_detaches_at_the_defer_bound() {
        // What the dropped marker above costs: the close never arrives, so the bound ends the wait.
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut q: OpQueue<8> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let ready = AtomicBool::new(true);
        let epoch = AtomicU32::new(0);
        let step = |vfs: &mut _, c: &mut _, dt| {
            msc_worker_step(vfs, c, &replies, &ready, &epoch, dt).media
        };

        p.try_push(MscOp::DataPhase(true)).ok().expect("open");
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
        p.try_push(MscOp::Write(Sector {
            lba: 131,
            data: root,
            epoch: 0,
        }))
        .ok()
        .expect("dirent");
        p.try_push(MscOp::Write(Sector {
            lba: crate::fat::DATA_START_LBA,
            data: uf2_block(),
            epoch: 0,
        }))
        .ok()
        .expect("image");

        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Ready); // phase opens
        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Ready); // dirent
        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Ready); // image, detach armed

        assert_eq!(
            step(&mut vfs, &mut c, crate::MSC_DETACH_DEFER_MS - 1),
            MediaState::Ready
        );
        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Detached);
        assert!(!ready.load(Ordering::Acquire));
    }

    #[test]
    fn queued_io_read_parks_then_delivers_reply() {
        let mut q: OpQueue<4> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();

        let mut buf = [0u8; 512];
        let mut outstanding = None;
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.read(0, &mut buf), SectorPoll::Pending);
            // A second read for the same block must not enqueue again.
            assert_eq!(io.read(0, &mut buf), SectorPoll::Pending);
        }
        assert_eq!(c.len(), 1, "the read must be enqueued exactly once");

        match c.pop() {
            Some(MscOp::Read { lba, epoch }) => assert_eq!((lba, epoch), (0, 0)),
            _ => panic!("expected a read op"),
        }
        replies.post(Sector {
            lba: 0,
            data: [0xABu8; 512],
            epoch: 0,
        });

        let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
        assert_eq!(io.read(0, &mut buf), SectorPoll::Ready);
        assert_eq!(buf, [0xABu8; 512]);
        assert_eq!(outstanding, None, "delivery clears the outstanding marker");
    }

    #[test]
    fn queued_read_not_reemitted_across_passes() {
        // The caller-owned marker survives a pass, so an unfulfilled read is enqueued once.
        let mut q: OpQueue<8> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let mut buf = [0u8; 512];
        let mut outstanding = None;

        for _ in 0..2 {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.read(3, &mut buf), SectorPoll::Pending);
        }
        assert_eq!(c.len(), 1, "the read is enqueued once across passes");

        replies.post(Sector {
            lba: 3,
            data: [0x5Au8; 512],
            epoch: 0,
        });
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.read(3, &mut buf), SectorPoll::Ready);
        }
        assert_eq!(buf, [0x5Au8; 512]);

        // After an epoch bump (remount) the same lba is a NEW request: it re-emits.
        let mut io = QueuedIo::new(&mut p, &replies, true, 1, &mut outstanding);
        assert_eq!(io.read(3, &mut buf), SectorPoll::Pending);
        match c.pop() {
            Some(MscOp::Read { lba, epoch }) => assert_eq!((lba, epoch), (3, 0)),
            _ => panic!("expected the original read op"),
        }
        match c.pop() {
            Some(MscOp::Read { lba, epoch }) => assert_eq!((lba, epoch), (3, 1)),
            _ => panic!("expected the post-remount read op"),
        }
    }

    #[test]
    fn queued_read_full_queue_drop_retries() {
        // A dropped read must leave the marker clear, so the next poll re-emits it.
        let mut q: OpQueue<2> = OpQueue::new(); // usable capacity: 1
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let mut buf = [0u8; 512];
        let mut outstanding = None;

        // Fill the queue with a write, then attempt a read: the push is dropped.
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.write(9, &[0u8; 512]), SectorPoll::Ready);
            assert_eq!(io.read(3, &mut buf), SectorPoll::Pending);
        }
        assert_eq!(outstanding, None, "a dropped push must not mark the read");

        // The worker drains the write; the next pass re-emits the read.
        assert!(matches!(c.pop(), Some(MscOp::Write(_))));
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.read(3, &mut buf), SectorPoll::Pending);
        }
        assert_eq!(outstanding, Some((3, 0)));
        match c.pop() {
            Some(MscOp::Read { lba, epoch }) => assert_eq!((lba, epoch), (3, 0)),
            _ => panic!("expected the retried read op"),
        }
    }

    #[test]
    fn queued_write_full_queue_pending_retries_and_lands() {
        // A write is backpressured, never dropped: unlike a read, the caller must resubmit it.
        let mut q: OpQueue<2> = OpQueue::new(); // usable capacity: 1
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let mut outstanding = None;

        // Fill the queue with an unrelated read, then attempt the write: it is backpressured.
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.read(1, &mut [0u8; 512]), SectorPoll::Pending);
            assert_eq!(io.write(9, &[0x5Au8; 512]), SectorPoll::Pending);
        }
        assert_eq!(
            c.len(),
            1,
            "the backpressured write must not have been enqueued"
        );

        // The worker drains the read; the retried write now has room.
        assert!(matches!(c.pop(), Some(MscOp::Read { .. })));
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            assert_eq!(io.write(9, &[0x5Au8; 512]), SectorPoll::Ready);
        }
        match c.pop() {
            Some(MscOp::Write(s)) => {
                assert_eq!(s.lba, 9);
                assert_eq!(s.data, [0x5Au8; 512]);
            }
            _ => panic!("expected the retried write op to land in the queue"),
        }
    }

    #[test]
    fn reply_slot_post_overwrites_unclaimed_reply() {
        // A single mailbox: a post overwrites any previous unclaimed reply.
        let replies = ReplySlot::new();
        replies.post(Sector {
            lba: 1,
            data: [0xA1u8; 512],
            epoch: 0,
        });
        replies.post(Sector {
            lba: 2,
            data: [0xB2u8; 512],
            epoch: 0,
        });
        assert!(
            replies.take(1, 0).is_none(),
            "the overwritten reply is gone"
        );
        assert_eq!(replies.take(2, 0), Some([0xB2u8; 512]));
    }

    #[test]
    fn queued_io_write_enqueues_sector_by_value() {
        let mut q: OpQueue<4> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let mut outstanding = None;
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 7, &mut outstanding);
            assert_eq!(io.write(9, &[0x5Au8; 512]), SectorPoll::Ready);
        }
        match c.pop() {
            Some(MscOp::Write(s)) => {
                assert_eq!(s.lba, 9);
                assert_eq!(s.epoch, 7);
                assert_eq!(s.data, [0x5Au8; 512]);
            }
            _ => panic!("expected a write op"),
        }
    }

    #[test]
    fn worker_serves_read_and_posts_boot_sector() {
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut q: OpQueue<4> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let ready = AtomicBool::new(false);
        let epoch = AtomicU32::new(0);

        let mut outstanding = None;
        {
            let mut io = QueuedIo::new(&mut p, &replies, true, 0, &mut outstanding);
            let mut buf = [0u8; 512];
            assert_eq!(io.read(0, &mut buf), SectorPoll::Pending);
        }

        let step = msc_worker_step(&mut vfs, &mut c, &replies, &ready, &epoch, 0);
        assert!(step.did_work);
        assert_eq!(step.media, MediaState::Ready);
        assert!(ready.load(Ordering::Acquire));

        let data = replies
            .take(0, 0)
            .expect("reply posted under current epoch");
        // Boot-sector signature.
        assert_eq!(data[510], 0x55);
        assert_eq!(data[511], 0xAA);
    }

    #[test]
    fn worker_drops_read_with_stale_epoch() {
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut q: OpQueue<4> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let ready = AtomicBool::new(false);
        let epoch = AtomicU32::new(1); // media has already remounted once

        // A read enqueued under the old epoch 0.
        p.try_push(MscOp::Read { lba: 0, epoch: 0 })
            .ok()
            .expect("enqueue");

        let step = msc_worker_step(&mut vfs, &mut c, &replies, &ready, &epoch, 0);
        assert!(step.did_work, "the stale op is still consumed");
        assert!(
            replies.take(0, 1).is_none(),
            "no reply must be posted for a stale-epoch read"
        );
    }

    #[test]
    fn worker_write_drives_transfer_and_epoch_bumps_on_remount() {
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut q: OpQueue<4> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let ready = AtomicBool::new(true);
        let epoch = AtomicU32::new(0);

        // Cluster left unallocated, so the block-magic content guard is what starts the transfer.
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 0));
        vfs.write_sector(131, &root);
        let sector = uf2_block();

        p.try_push(MscOp::Write(Sector {
            lba: crate::fat::DATA_START_LBA,
            data: sector,
            epoch: 0,
        }))
        .ok()
        .expect("enqueue write");
        let step = msc_worker_step(&mut vfs, &mut c, &replies, &ready, &epoch, 0);
        assert!(step.did_work);
        assert!(vfs.is_transfer_active());

        // The dirent never supplied a size, so the unsized finishable timeout completes it.
        let step = msc_worker_step(
            &mut vfs,
            &mut c,
            &replies,
            &ready,
            &epoch,
            crate::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS + 1,
        );
        assert_eq!(step.media, MediaState::Detached);
        assert_eq!(epoch.load(Ordering::Acquire), 0, "no bump while detaching");
        assert!(!ready.load(Ordering::Acquire));

        // Detached -> Ready bumps the epoch.
        let step = msc_worker_step(
            &mut vfs,
            &mut c,
            &replies,
            &ready,
            &epoch,
            crate::MSC_RECONNECT_MS,
        );
        assert_eq!(step.media, MediaState::Ready);
        assert_eq!(epoch.load(Ordering::Acquire), 1, "remount bumps the epoch");
        assert!(ready.load(Ordering::Acquire));
    }

    #[test]
    fn queued_data_phase_holds_the_worker_detach_to_the_phase_end() {
        // A completion latched mid-command must survive to the marker closing the phase.
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut q: OpQueue<8> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let ready = AtomicBool::new(true);
        let epoch = AtomicU32::new(0);
        let step = |vfs: &mut _, c: &mut _, dt| {
            msc_worker_step(vfs, c, &replies, &ready, &epoch, dt).media
        };

        // One WRITE(10) carrying the dirent and the whole image, its phase not yet closed.
        p.try_push(MscOp::DataPhase(true)).ok().expect("open");
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
        p.try_push(MscOp::Write(Sector {
            lba: 131,
            data: root,
            epoch: 0,
        }))
        .ok()
        .expect("dirent");
        let sector = uf2_block();
        p.try_push(MscOp::Write(Sector {
            lba: crate::fat::DATA_START_LBA,
            data: sector,
            epoch: 0,
        }))
        .ok()
        .expect("image");

        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Ready); // phase opens
        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Ready); // dirent
        assert_eq!(
            step(&mut vfs, &mut c, 1),
            MediaState::Ready,
            "the image met its size, but the command is still in flight"
        );
        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Ready, "still open");
        assert_eq!(epoch.load(Ordering::Acquire), 0);

        // Closing the phase is what the detach lands on.
        p.try_push(MscOp::DataPhase(false)).ok().expect("close");
        assert_eq!(step(&mut vfs, &mut c, 1), MediaState::Detached);
        assert!(!ready.load(Ordering::Acquire));
        // The remount the drop earned still arrives, bumping the epoch.
        assert_eq!(
            step(&mut vfs, &mut c, crate::MSC_RECONNECT_MS),
            MediaState::Ready
        );
        assert_eq!(epoch.load(Ordering::Acquire), 1);
    }

    #[test]
    fn worker_drops_write_with_stale_epoch() {
        let mut files: [&mut dyn crate::VfsFile; 0] = [];
        let mut vfs = make_vfs(&mut files);
        let mut q: OpQueue<4> = OpQueue::new();
        let (mut p, mut c) = q.split();
        let replies = ReplySlot::new();
        let ready = AtomicBool::new(true);
        let epoch = AtomicU32::new(1); // media has already remounted once

        // Bound, so this sector would start a transfer if it were applied.
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 0));
        vfs.write_sector(131, &root);

        let sector = uf2_block();

        // A write from the old epoch must not be applied to the new mount.
        p.try_push(MscOp::Write(Sector {
            lba: crate::fat::DATA_START_LBA,
            data: sector,
            epoch: 0,
        }))
        .ok()
        .expect("enqueue write");

        let step = msc_worker_step(&mut vfs, &mut c, &replies, &ready, &epoch, 0);
        assert!(step.did_work, "the stale op is still consumed");
        assert!(
            !vfs.is_transfer_active(),
            "a stale-epoch write must not be applied to the volume"
        );
    }
}
