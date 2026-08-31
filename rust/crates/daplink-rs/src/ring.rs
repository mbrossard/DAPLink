//! Lock-free SPSC byte ring, from the interrupt draining a hardware RX FIFO to the forwarding task.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

/// SPSC byte ring plus an overrun counter, buffering `N - 1` bytes; split once with
/// [`split`](ByteRing::split). Plain atomic load/store only, never RMW, so it is sound on thumbv6m.
pub struct ByteRing<const N: usize> {
    /// Backing storage; each cell is written only by the producer and read only by the consumer,
    /// at disjoint indices, so per-cell interior mutability is sound.
    buf: [UnsafeCell<u8>; N],
    /// Next write index (owned by the producer, observed by the consumer).
    head: AtomicUsize,
    /// Next read index (owned by the consumer, observed by the producer).
    tail: AtomicUsize,
    /// Bytes the producer dropped into a full ring, ever; written only by the producer, so the
    /// consumer snapshots it rather than clearing it.
    overruns: AtomicU32,
}

// SAFETY: cells are accessed only through the producer/consumer split at disjoint indices, with
// head/tail publishing the hand-off via Acquire/Release ordering.
unsafe impl<const N: usize> Sync for ByteRing<N> {}

impl<const N: usize> ByteRing<N> {
    /// Create an empty ring; `const` so it can back a `static`.
    pub const fn new() -> Self {
        Self {
            buf: [const { UnsafeCell::new(0) }; N],
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            overruns: AtomicU32::new(0),
        }
    }

    /// Split into producer and consumer halves; the `&mut` receiver guarantees a single split.
    pub fn split(&mut self) -> (ByteProducer<'_, N>, ByteConsumer<'_, N>) {
        let ring: &Self = self;
        (
            ByteProducer { ring },
            ByteConsumer {
                ring,
                overruns_seen: 0,
            },
        )
    }

    /// Advance a ring index by one, wrapping at `N`.
    #[inline]
    fn wrap_next(idx: usize) -> usize {
        let next = idx + 1;
        if next == N {
            0
        } else {
            next
        }
    }
}

impl<const N: usize> Default for ByteRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Producer half of a [`ByteRing`], held by the interrupt that drains the hardware FIFO.
pub struct ByteProducer<'a, const N: usize> {
    ring: &'a ByteRing<N>,
}

// SAFETY: the producer only writes `head` and its own ring cells and only reads `tail`.
unsafe impl<const N: usize> Send for ByteProducer<'_, N> {}

impl<const N: usize> ByteProducer<'_, N> {
    /// Bytes that can be pushed without overflowing the ring.
    pub fn remaining_capacity(&self) -> usize {
        let head = self.ring.head.load(Ordering::Relaxed);
        let tail = self.ring.tail.load(Ordering::Acquire);
        if head >= tail {
            N - 1 - (head - tail)
        } else {
            tail - head - 1
        }
    }

    /// Push one byte, reporting whether it fit; on a full ring the byte is dropped and the
    /// overrun counted.
    pub fn push(&mut self, b: u8) -> bool {
        let head = self.ring.head.load(Ordering::Relaxed);
        let next = ByteRing::<N>::wrap_next(head);
        // Acquire pairs with the consumer's Release store of `tail`.
        if next == self.ring.tail.load(Ordering::Acquire) {
            // Load-add-store: the producer is the counter's only writer.
            let n = self.ring.overruns.load(Ordering::Relaxed).wrapping_add(1);
            self.ring.overruns.store(n, Ordering::Relaxed);
            return false;
        }
        // SAFETY: only the producer advances `head`, and the consumer never reads this cell until
        // the Release store below publishes the new `head`.
        unsafe { *self.ring.buf[head].get() = b };
        self.ring.head.store(next, Ordering::Release);
        true
    }
}

/// Bytes one interrupt entry moves out of a hardware RX FIFO before it returns.
pub const FIFO_DRAIN_BUDGET: usize = 32;

/// Push up to `budget` bytes from `next` into `producer`, returning the count. The budget is what
/// keeps a line that refills the FIFO as fast as the loop empties it from holding the interrupt.
pub fn drain_fifo<const N: usize>(
    producer: &mut ByteProducer<'_, N>,
    budget: usize,
    mut next: impl FnMut() -> Option<u8>,
) -> usize {
    let mut n = 0;
    while n < budget {
        match next() {
            Some(b) => {
                producer.push(b);
                n += 1;
            }
            None => break,
        }
    }
    n
}

/// Consumer half of a [`ByteRing`], held by the task that forwards the bytes on.
pub struct ByteConsumer<'a, const N: usize> {
    ring: &'a ByteRing<N>,
    /// The overrun total as of the last snapshot, so taking one never erases a concurrent event.
    overruns_seen: u32,
}

// SAFETY: the consumer only writes `tail` and reads its own ring cells and `head`.
unsafe impl<const N: usize> Send for ByteConsumer<'_, N> {}

impl<const N: usize> ByteConsumer<'_, N> {
    /// Pop one byte, or `None` if the ring is empty.
    pub fn pop(&mut self) -> Option<u8> {
        let tail = self.ring.tail.load(Ordering::Relaxed);
        // Acquire pairs with the producer's Release store of `head`.
        if tail == self.ring.head.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: `head != tail` means this cell was fully written and published; only the
        // consumer advances `tail`, so the read cannot race.
        let b = unsafe { *self.ring.buf[tail].get() };
        self.ring
            .tail
            .store(ByteRing::<N>::wrap_next(tail), Ordering::Release);
        Some(b)
    }

    /// Drain up to `buf.len()` bytes into `buf`; returns the count copied.
    pub fn drain(&mut self, buf: &mut [u8]) -> usize {
        let mut n = 0;
        while n < buf.len() {
            match self.pop() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    /// Bytes currently buffered.
    pub fn len(&self) -> usize {
        let head = self.ring.head.load(Ordering::Acquire);
        let tail = self.ring.tail.load(Ordering::Relaxed);
        if head >= tail {
            head - tail
        } else {
            N - tail + head
        }
    }

    /// `true` if no bytes are buffered.
    pub fn is_empty(&self) -> bool {
        self.ring.head.load(Ordering::Acquire) == self.ring.tail.load(Ordering::Relaxed)
    }

    /// Discard all buffered bytes and any overruns recorded so far.
    pub fn clear(&mut self) {
        while self.pop().is_some() {}
        self.overruns_seen = self.ring.overruns.load(Ordering::Relaxed);
    }

    /// Whether the producer overran since the last call; a snapshot of the producer-owned total,
    /// so an overrun landing concurrently is reported next time rather than erased.
    pub fn take_overrun(&mut self) -> bool {
        let total = self.ring.overruns.load(Ordering::Relaxed);
        let overran = total != self.overruns_seen;
        self.overruns_seen = total;
        overran
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    #[test]
    fn ring_drains_in_order_and_flags_overrun_on_full() {
        // Capacity is N - 1, so a ByteRing<4> buffers 3 bytes.
        let mut ring: ByteRing<4> = ByteRing::new();
        let (mut p, mut c) = ring.split();
        let fit: Vec<bool> = [1u8, 2, 3, 4, 5].into_iter().map(|b| p.push(b)).collect();
        assert_eq!(fit, [true, true, true, false, false]);
        assert!(c.take_overrun());
        assert!(!c.take_overrun(), "overrun clears on read");
        let mut buf = [0u8; 8];
        let n = c.drain(&mut buf);
        assert_eq!(&buf[..n], &[1, 2, 3]);
        assert!(c.is_empty());
    }

    #[test]
    fn overruns_after_a_clear_or_a_take_are_still_reported() {
        let mut ring: ByteRing<2> = ByteRing::new();
        let (mut p, mut c) = ring.split();
        assert!(p.push(1));
        assert!(!p.push(2));
        c.clear();
        assert!(!c.take_overrun(), "clear discards recorded overruns");
        assert!(p.push(3));
        assert!(!p.push(4));
        assert!(
            c.take_overrun(),
            "a fresh overrun survives the earlier clear"
        );
        assert!(!p.push(5));
        assert!(
            c.take_overrun(),
            "each take reports only what followed the last"
        );
    }

    #[test]
    fn ring_wraps_around_capacity() {
        let mut ring: ByteRing<4> = ByteRing::new();
        let (mut p, mut c) = ring.split();
        for b in [10u8, 20, 30] {
            p.push(b);
        }
        let mut buf = [0u8; 2];
        assert_eq!(c.drain(&mut buf), 2);
        assert_eq!(&buf, &[10, 20]);
        for b in [40u8, 50] {
            p.push(b);
        }
        assert_eq!(c.len(), 3);
        let mut out = [0u8; 8];
        let n = c.drain(&mut out);
        assert_eq!(&out[..n], &[30, 40, 50]);
    }

    #[test]
    fn producer_capacity_tracks_empty_full_wrap_and_recovery() {
        let mut ring: ByteRing<4> = ByteRing::new();
        let (mut p, mut c) = ring.split();
        assert_eq!(p.remaining_capacity(), 3);
        assert!(p.push(1));
        assert!(p.push(2));
        assert_eq!(p.remaining_capacity(), 1);
        assert!(p.push(3));
        assert_eq!(p.remaining_capacity(), 0);
        assert!(!p.push(4));
        let mut buf = [0; 2];
        assert_eq!(c.drain(&mut buf), 2);
        assert_eq!(p.remaining_capacity(), 2);
        assert!(p.push(4));
        assert!(p.push(5));
        assert_eq!(p.remaining_capacity(), 0);
        let mut out = [0; 3];
        assert_eq!(c.drain(&mut out), 3);
        assert_eq!(out, [3, 4, 5]);
        assert_eq!(p.remaining_capacity(), 3);
    }

    #[test]
    fn a_fifo_drain_stops_at_its_budget_and_leaves_the_rest() {
        // A source that never runs dry: a line refilling as fast as the loop empties it.
        let mut ring: ByteRing<64> = ByteRing::new();
        let (mut p, mut c) = ring.split();
        let mut k = 0u8;
        let n = drain_fifo(&mut p, FIFO_DRAIN_BUDGET, || {
            k = k.wrapping_add(1);
            Some(k)
        });
        assert_eq!(n, FIFO_DRAIN_BUDGET, "the loop returns at the budget");
        assert_eq!(c.len(), FIFO_DRAIN_BUDGET);
        let mut buf = [0u8; 64];
        let got = c.drain(&mut buf);
        assert_eq!(&buf[..got], (1..=32u8).collect::<Vec<_>>().as_slice());
    }

    #[test]
    fn a_fifo_drain_stops_early_when_the_fifo_empties() {
        let mut ring: ByteRing<64> = ByteRing::new();
        let (mut p, mut c) = ring.split();
        let mut left = [7u8, 8, 9].into_iter();
        assert_eq!(drain_fifo(&mut p, FIFO_DRAIN_BUDGET, || left.next()), 3);
        let mut buf = [0u8; 8];
        let n = c.drain(&mut buf);
        assert_eq!(&buf[..n], &[7, 8, 9]);
        // A zero budget takes nothing and does not consume from the source.
        let mut more = [1u8].into_iter();
        assert_eq!(drain_fifo(&mut p, 0, || more.next()), 0);
        assert_eq!(more.next(), Some(1));
    }

    #[test]
    fn a_fifo_drain_into_a_full_ring_flags_overrun_and_still_returns() {
        let mut ring: ByteRing<4> = ByteRing::new();
        let (mut p, mut c) = ring.split();
        let mut k = 0u8;
        assert_eq!(
            drain_fifo(&mut p, FIFO_DRAIN_BUDGET, || {
                k = k.wrapping_add(1);
                Some(k)
            }),
            FIFO_DRAIN_BUDGET,
            "a full ring drops bytes but the budget still bounds the loop"
        );
        assert!(c.take_overrun());
        assert_eq!(c.len(), 3);
    }
}
