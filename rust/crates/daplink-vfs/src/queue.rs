//! Worker-offload seam: an SPSC operation queue and an epoch-tagged reply slot.

use core::cell::RefCell;
use critical_section::Mutex;
use heapless::spsc::{Consumer, Producer, Queue};

/// A 512-byte sector tagged with its logical block and the media epoch it belongs to.
#[derive(Clone, Copy)]
pub struct Sector {
    /// The logical block address.
    pub lba: u32,
    /// The sector payload.
    pub data: [u8; 512],
    /// The media epoch this sector belongs to.
    pub epoch: u32,
}

/// A queued sector operation; a write carries its 512 bytes by value, so the queue is the buffer.
#[allow(clippy::large_enum_variant)]
pub enum MscOp {
    /// Read logical block `lba`, produced under media epoch `epoch`.
    Read {
        /// The logical block to read.
        lba: u32,
        /// The media epoch the request was issued under.
        epoch: u32,
    },
    /// Write the carried sector.
    Write(Sector),
    /// The transport entered (`true`) or left (`false`) a host command's data phase. In-band, so
    /// the worker sees it in order with the sectors it brackets; no epoch, it is not the volume.
    DataPhase(bool),
}

/// A single-producer / single-consumer queue of pending sector operations. `DEPTH` is the backing
/// buffer size; the usable capacity is `DEPTH - 1` operations.
pub struct OpQueue<const DEPTH: usize = 4> {
    inner: Queue<MscOp, DEPTH>,
}

impl<const DEPTH: usize> OpQueue<DEPTH> {
    /// Create an empty queue.
    pub const fn new() -> Self {
        Self {
            inner: Queue::new(),
        }
    }

    /// Split the queue into its producer (USB side) and consumer (worker side) halves.
    pub fn split(&mut self) -> (OpProducer<'_, DEPTH>, OpConsumer<'_, DEPTH>) {
        let (producer, consumer) = self.inner.split();
        (
            OpProducer { inner: producer },
            OpConsumer { inner: consumer },
        )
    }
}

impl<const DEPTH: usize> Default for OpQueue<DEPTH> {
    fn default() -> Self {
        Self::new()
    }
}

/// Whether the operation queue can still accept work. [`Backpressure::YieldToWorker`] is an
/// obligation: a USB service outranking the worker must yield, or nothing can free a slot.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[must_use = "a full operation queue obliges a USB service that outranks its sector worker to \
              yield to it; ignoring this livelocks the mass-storage path on the first host write"]
pub enum Backpressure {
    /// The queue has a free slot: keep servicing.
    None,
    /// The queue is full: the caller must yield to the worker before servicing again.
    YieldToWorker,
}

/// The producer half of an [`OpQueue`], held by the USB poll context.
pub struct OpProducer<'a, const DEPTH: usize> {
    inner: Producer<'a, MscOp, DEPTH>,
}

impl<const DEPTH: usize> OpProducer<'_, DEPTH> {
    /// Enqueue `op`; a full queue hands it back as `Err`, which the caller treats as
    /// backpressure and retries on the next poll.
    // Handed back by value: there is no heap in firmware to box it into.
    #[allow(clippy::result_large_err)]
    pub fn try_push(&mut self, op: MscOp) -> Result<(), MscOp> {
        self.inner.enqueue(op)
    }

    /// Whether the queue can still accept work, or the caller now owes the worker a yield.
    pub fn backpressure(&self) -> Backpressure {
        if self.inner.ready() {
            Backpressure::None
        } else {
            Backpressure::YieldToWorker
        }
    }
}

/// The consumer half of an [`OpQueue`], held by the worker context.
pub struct OpConsumer<'a, const DEPTH: usize> {
    inner: Consumer<'a, MscOp, DEPTH>,
}

impl<const DEPTH: usize> OpConsumer<'_, DEPTH> {
    /// Pop the oldest pending operation, if any.
    pub fn pop(&mut self) -> Option<MscOp> {
        self.inner.dequeue()
    }

    /// The number of operations currently queued.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Whether the queue is currently empty.
    pub fn is_empty(&self) -> bool {
        self.inner.len() == 0
    }
}

/// A one-slot mailbox for a read reply. [`take`](ReplySlot::take) matches LBA and media epoch,
/// so a reply left over from a previous mount is discarded once a remount bumps the epoch.
pub struct ReplySlot {
    inner: Mutex<RefCell<Option<Sector>>>,
}

impl ReplySlot {
    /// Create an empty reply slot.
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(None)),
        }
    }

    /// Publish a read reply, overwriting any previous unclaimed one.
    pub fn post(&self, s: Sector) {
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(s);
        });
    }

    /// Take the stored reply if it matches both `lba` and `epoch`, clearing the slot; a mismatch
    /// leaves it untouched.
    pub fn take(&self, lba: u32, epoch: u32) -> Option<[u8; 512]> {
        critical_section::with(|cs| {
            let slot = self.inner.borrow(cs);
            let mut guard = slot.borrow_mut();
            if let Some(s) = guard.as_ref() {
                if s.lba == lba && s.epoch == epoch {
                    let data = s.data;
                    *guard = None;
                    return Some(data);
                }
            }
            None
        })
    }
}

impl Default for ReplySlot {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queue_backpressure_hands_op_back() {
        // DEPTH 2 -> usable capacity 1.
        let mut q: OpQueue<2> = OpQueue::new();
        let (mut p, mut c) = q.split();
        assert!(c.is_empty());
        assert_eq!(p.backpressure(), Backpressure::None);
        assert!(p.try_push(MscOp::Read { lba: 1, epoch: 0 }).is_ok());
        // The obligation must be reported before the next push proves it.
        assert_eq!(p.backpressure(), Backpressure::YieldToWorker);
        // The next push overflows and must return the op unchanged.
        let handed_back = p.try_push(MscOp::Read { lba: 2, epoch: 0 });
        match handed_back {
            Err(MscOp::Read { lba, epoch }) => {
                assert_eq!((lba, epoch), (2, 0));
            }
            _ => panic!("expected the overflowing op to be handed back"),
        }
        assert_eq!(c.len(), 1);
        // Only the worker can clear the obligation.
        assert!(c.pop().is_some());
        assert_eq!(p.backpressure(), Backpressure::None);
    }

    #[test]
    fn reply_slot_matches_lba_and_epoch() {
        let slot = ReplySlot::new();
        slot.post(Sector {
            lba: 5,
            data: [7u8; 512],
            epoch: 0,
        });
        // Wrong LBA: no take.
        assert!(slot.take(6, 0).is_none());
        // Correct LBA and epoch: take and clear.
        assert_eq!(slot.take(5, 0), Some([7u8; 512]));
        // Already cleared.
        assert!(slot.take(5, 0).is_none());
    }

    #[test]
    fn reply_slot_stale_epoch_not_taken_after_remount() {
        let slot = ReplySlot::new();
        slot.post(Sector {
            lba: 5,
            data: [9u8; 512],
            epoch: 0,
        });
        // A remount advanced the epoch: the stale reply must not be taken.
        assert!(slot.take(5, 1).is_none());
    }
}
