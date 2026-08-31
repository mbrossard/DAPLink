//! The receive ring and the cursor that tracks the controller's free-running production count.

use daplink_core::DmaRingCursor;

/// Receive ring entries; the wrap region [`RxRing`] is aligned to has to be the buffer itself.
pub const RX_RING_LEN: usize = 512;
const RX_RING_BYTES: usize = RX_RING_LEN * 2;
/// `log2` of the ring in bytes: the controller's write-address wrap width.
pub const RX_RING_LOG2: u32 = 10;
const _: () = assert!(RX_RING_BYTES == 1 << RX_RING_LOG2);

/// Receive cursor over the channel's free-running production count, which has no aliasing limit.
pub struct RxTracker {
    cursor: DmaRingCursor,
    /// The production total as of the last fold, so each fold takes a difference.
    produced: u32,
    /// Lapped entries seen since the caller last asked; a lap can be found mid-drain.
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

    /// Entries waiting as of the last fold.
    pub fn available(&self) -> u32 {
        self.cursor.available() as u32
    }

    /// Take the lapped count and clear it.
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

/// The ring itself, aligned so the controller's wrap region lands on it.
#[repr(C, align(1024))]
pub struct RxRing {
    pub buf: [u16; RX_RING_LEN],
}

impl RxRing {
    pub const fn new() -> Self {
        Self {
            buf: [0; RX_RING_LEN],
        }
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

    /// Drain a tracker at a production total: entries handed out, first and last slot, lapped.
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
        // One more entry lands back at the base rather than past the buffer.
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
    fn an_exactly_full_ring_is_not_a_lap() {
        let (n, _, _, lost) = drain_at(RX_RING_LEN as u32);
        assert_eq!(lost, 0, "a full ring is intact, not overwritten");
        assert_eq!(n, RX_RING_LEN);
    }

    #[test]
    fn one_entry_past_full_loses_exactly_one() {
        let (n, first, _, lost) = drain_at(RX_RING_LEN as u32 + 1);
        assert_eq!(lost, 1);
        assert_eq!(n, RX_RING_LEN);
        // Resynchronized onto the oldest survivor, the entry after the one dropped.
        assert_eq!(first, 1);
    }

    /// A free-running count separates a double wrap from a single one by magnitude alone.
    #[test]
    fn a_double_wrap_is_distinguished_from_a_single_one() {
        let ring = RX_RING_LEN as u32;
        let (_, _, _, single) = drain_at(ring + 1);
        let (_, _, _, double) = drain_at(2 * ring);
        assert_eq!(single, 1);
        assert_eq!(double, ring);
        assert_ne!(single, double);
    }

    #[test]
    fn past_a_double_wrap_the_whole_excess_is_charged() {
        let ring = RX_RING_LEN as u32;
        let (n, _, _, lost) = drain_at(2 * ring + 5);
        assert_eq!(lost, ring + 5);
        assert_eq!(n, RX_RING_LEN);
    }

    #[test]
    fn a_lap_found_mid_drain_is_banked_rather_than_dropped() {
        // The fold that discovers a lap is often not the one the caller asked about.
        let mut rx = RxTracker::new();
        rx.observe(4);
        assert_eq!(rx.take_lost(), 0);
        rx.next_index();
        // Three waiting entries with two rings on top: the whole overwrite, not new production.
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
        // The channel's counter restarted with it, so the next fold measures from zero.
        rx.observe(2);
        assert_eq!(rx.available(), 2);
        assert_eq!(rx.next_index(), Some(0));
    }
}
