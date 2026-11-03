//! The receive ring and the cursor that tracks the controller's free-running production count.

use core::cell::UnsafeCell;
use daplink_core::DmaRingCursor;

/// Receive ring entries; the controller's wrap region must match this buffer.
pub const RX_RING_LEN: usize = 512;
const RX_RING_BYTES: usize = RX_RING_LEN * 2;
/// `log2` of the ring in bytes: the controller's write-address wrap width.
pub const RX_RING_LOG2: u32 = 10;
const _: () = assert!(RX_RING_BYTES == 1 << RX_RING_LOG2);

/// Receive cursor over the channel's production count since [`reset`](Self::reset), sampled as a
/// `u32`: it must advance by less than `2^32` between observations.
pub struct RxTracker {
    cursor: DmaRingCursor,
    produced: u32,
    lost: u32,
}

impl RxTracker {
    pub const fn new() -> Self {
        Self {
            cursor: DmaRingCursor::new(RX_RING_LEN),
            produced: 0,
            lost: 0,
        }
    }

    /// Start over: the channel was re-armed, so its count restarts and the ring's contents go.
    pub fn reset(&mut self) {
        self.cursor.reset();
        self.produced = 0;
        self.lost = 0;
    }

    /// Fold the channel's current total in, banking whatever a lap overwrote.
    pub fn observe(&mut self, produced: u32) {
        let advanced = produced.wrapping_sub(self.produced) as usize;
        self.produced = produced;
        self.lost = self
            .lost
            .wrapping_add(self.cursor.observe_produced(advanced) as u32);
    }

    pub fn available(&self) -> u32 {
        self.cursor.available() as u32
    }

    pub fn take_lost(&mut self) -> u32 {
        core::mem::take(&mut self.lost)
    }

    /// Ring slot of the next entry to read, or `None` once the reader has caught up.
    pub fn next_index(&mut self) -> Option<usize> {
        self.cursor.next_index()
    }
}

impl Default for RxTracker {
    fn default() -> Self {
        Self::new()
    }
}

/// DMA receive ring aligned to the controller's address-wrap region.
#[repr(C, align(1024))]
pub struct RxRing {
    buf: UnsafeCell<[u16; RX_RING_LEN]>,
}

// SAFETY: shared access exposes only a raw pointer, whose reads and writes remain unsafe.
unsafe impl Sync for RxRing {}

impl RxRing {
    pub const fn new() -> Self {
        Self {
            buf: UnsafeCell::new([0; RX_RING_LEN]),
        }
    }

    /// Raw storage for DMA; the caller must coordinate all accesses without creating slot references.
    pub fn as_mut_ptr(&self) -> *mut u16 {
        self.buf.get().cast()
    }
}

impl Default for RxRing {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drain_at(produced: u32) -> (usize, usize, usize, u32) {
        let mut rx = RxTracker::new();
        rx.observe(produced);
        let lost = rx.take_lost();
        let (mut n, mut first, mut last) = (0, 0, 0);
        while let Some(i) = rx.next_index() {
            if n == 0 {
                first = i;
            }
            last = i;
            n += 1;
        }
        (n, first, last, lost)
    }

    #[test]
    fn indices_stay_in_bounds_and_wrap_at_the_ring_length() {
        let (n, first, last, lost) = drain_at(RX_RING_LEN as u32);
        assert_eq!(lost, 0);
        assert_eq!(n, RX_RING_LEN);
        assert_eq!(first, 0);
        assert_eq!(last, RX_RING_LEN - 1);
        let mut rx = RxTracker::new();
        rx.observe(RX_RING_LEN as u32);
        while rx.next_index().is_some() {}
        rx.observe(RX_RING_LEN as u32 + 1);
        assert_eq!(rx.next_index(), Some(0));
    }

    #[test]
    fn nothing_waits_until_the_channel_produces() {
        let mut rx = RxTracker::new();
        rx.observe(0);
        assert_eq!(rx.available(), 0);
        assert_eq!(rx.next_index(), None);
        rx.observe(1);
        assert_eq!(rx.available(), 1);
    }

    #[test]
    fn a_lap_found_mid_drain_is_banked_rather_than_dropped() {
        let mut rx = RxTracker::new();
        rx.observe(4);
        assert_eq!(rx.take_lost(), 0);
        rx.next_index();
        rx.observe(4 + 2 * RX_RING_LEN as u32);
        assert_eq!(rx.take_lost(), RX_RING_LEN as u32 + 3);
        assert_eq!(rx.take_lost(), 0, "reported once");
    }

    #[test]
    fn a_rearm_discards_the_ring_and_restarts_the_count() {
        let mut rx = RxTracker::new();
        rx.observe(3 * RX_RING_LEN as u32);
        assert!(rx.available() > 0);
        rx.reset();
        assert_eq!(rx.available(), 0);
        assert_eq!(rx.take_lost(), 0, "the discarded lap belongs to no session");
        assert_eq!(rx.next_index(), None);
        rx.observe(2);
        assert_eq!(rx.available(), 2);
        assert_eq!(rx.next_index(), Some(0));
    }
}
