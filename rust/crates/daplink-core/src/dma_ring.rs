//! Cursor arithmetic for a circular DMA receive ring, with lap detection.

use core::sync::atomic::{compiler_fence, Ordering};

/// Read/write cursors and backlog for one circular DMA ring, in transfer units (bytes here).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaRingCursor {
    /// Ring length in units. Never zero.
    len: usize,
    /// Index of the next unit the consumer will take.
    read: usize,
    /// The write position as of the last [`observe`](DmaRingCursor::observe).
    prev_write: usize,
    /// Units produced but not yet consumed; exceeding `len` is what makes a lap detectable.
    avail: usize,
}

impl DmaRingCursor {
    /// A cursor for a ring of `len` units, positioned at the start with nothing available.
    ///
    /// # Panics
    ///
    /// If `len` is zero.
    pub const fn new(len: usize) -> Self {
        assert!(len > 0, "a DMA ring must have a non-zero length");
        Self {
            len,
            read: 0,
            prev_write: 0,
            avail: 0,
        }
    }

    /// Ring length in units.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Units produced and not yet taken, as of the last [`observe`](DmaRingCursor::observe).
    #[inline]
    pub const fn available(&self) -> usize {
        self.avail
    }

    /// Whether nothing is waiting.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.avail == 0
    }

    /// Units waiting at `write_pos`, without folding it in. Advisory: a plain cursor difference,
    /// so it never exceeds `len` and cannot report a lap.
    #[inline]
    pub const fn pending_at(&self, write_pos: usize) -> usize {
        let write = if write_pos >= self.len { 0 } else { write_pos };
        if write >= self.read {
            write - self.read
        } else {
            self.len - self.read + write
        }
    }

    /// Fold the write position into the backlog, returning the units a lap overwrote. A position
    /// of `len` (the wrap instant) or beyond folds to `0`; an exactly full ring is not a lap.
    pub fn observe(&mut self, write_pos: usize) -> usize {
        self.observe_reloaded(write_pos, false)
    }

    /// [`observe`](Self::observe) for a caller that also knows whether the transfer counter
    /// reloaded since the last observation.
    pub fn observe_reloaded(&mut self, write_pos: usize, reloaded: bool) -> usize {
        let len = self.len;
        let w = if write_pos >= len { 0 } else { write_pos };
        // Modular distance: the position wraps, the running count does not.
        let mut produced = (w + len - self.prev_write) % len;
        if reloaded && w >= self.prev_write {
            produced += len;
        }
        self.observe_produced(produced)
    }

    /// [`observe`](Self::observe) for a caller whose channel reports an absolute production count
    /// rather than a position: no aliasing limit, so a stall of several rings is charged in full.
    pub fn observe_produced(&mut self, produced: usize) -> usize {
        let len = self.len;
        // Wrap by subtraction: this runs per unit drained and ARMv6-M has no divide instruction.
        let step = if produced < len {
            produced
        } else {
            produced % len
        };
        let w = self.prev_write + step;
        self.prev_write = if w >= len { w - len } else { w };
        self.avail += produced;
        if self.avail > len {
            let lost = self.avail - len;
            self.avail = len;
            self.read = self.prev_write;
            lost
        } else {
            0
        }
    }

    /// Take the index of the next unit to read, or `None` when the backlog is empty. Consuming:
    /// the caller must read that index before calling again.
    #[inline]
    pub fn next_index(&mut self) -> Option<usize> {
        if self.avail == 0 {
            return None;
        }
        let index = self.read;
        self.read = if index + 1 == self.len { 0 } else { index + 1 };
        self.avail -= 1;
        Some(index)
    }

    /// Discard the backlog and re-base both cursors at the ring's start. Correct only when the
    /// caller reloads the transfer counter in the same breath.
    #[inline]
    pub fn reset(&mut self) {
        self.read = 0;
        self.prev_write = 0;
        self.avail = 0;
    }
}

/// One circular DMA receive channel, reduced to what a ring needs: the platform seam.
pub trait DmaRxChannel {
    /// Units still to be written before the counter reloads, within the current lap.
    fn remaining(&self) -> usize;

    /// [`remaining`](Self::remaining), plus whether the counter reloaded since the previous call.
    /// Consuming: an implementation latching a hardware flag clears it here.
    fn sample(&mut self) -> (usize, bool) {
        (self.remaining(), false)
    }

    /// Stop the transfer, reload the counter to `len` units, and start it again.
    fn reload(&mut self, len: usize);

    /// Whether the transfer is running. A stopped channel is not observed, so a backlog captured
    /// before the stop stays readable.
    fn running(&self) -> bool {
        true
    }
}

/// A circular DMA receive ring: a [`DmaRingCursor`] plus the counter read, the volatile data read
/// and the restart. Owns no memory; the caller passes the buffer the controller fills.
pub struct DmaRxRing<C> {
    cursor: DmaRingCursor,
    chan: C,
}

impl<C: DmaRxChannel> DmaRxRing<C> {
    /// A ring of `len` units on `chan`, positioned at the start.
    ///
    /// # Panics
    ///
    /// If `len` is zero (see [`DmaRingCursor::new`]).
    pub fn new(chan: C, len: usize) -> Self {
        Self {
            cursor: DmaRingCursor::new(len),
            chan,
        }
    }

    /// Ring length in units.
    #[inline]
    pub const fn capacity(&self) -> usize {
        self.cursor.len()
    }

    /// Units folded in and not yet taken.
    #[inline]
    pub const fn available(&self) -> usize {
        self.cursor.available()
    }

    /// Whether nothing was waiting as of the last [`observe`](DmaRxRing::observe).
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.cursor.is_empty()
    }

    /// Units waiting right now, read from the counter without folding it in. Advisory: see
    /// [`DmaRingCursor::pending_at`].
    #[inline]
    pub fn pending(&self) -> usize {
        self.cursor.pending_at(self.write_pos())
    }

    /// The index the controller will write next. A counter reading of zero maps to `len`, and a
    /// larger reading is clamped; [`DmaRingCursor::observe`] folds both back to position 0.
    #[inline]
    pub fn write_pos(&self) -> usize {
        let len = self.cursor.len();
        // The counter runs DOWN from the ring length, so the write cursor is len - remaining.
        len - self.chan.remaining().min(len)
    }

    /// Fold the transfer counter into the backlog, returning the units a lap overwrote. Folding
    /// on every call, the empty ones included, is what turns a wrapping position into a count.
    #[inline]
    pub fn observe(&mut self) -> usize {
        if !self.chan.running() {
            return 0;
        }
        let len = self.cursor.len();
        let (remaining, reloaded) = self.chan.sample();
        self.cursor
            .observe_reloaded(len - remaining.min(len), reloaded)
    }

    /// Take the next unit out of `mem`, or `None` once the reader has caught up. A slice shorter
    /// than [`capacity`](DmaRxRing::capacity) yields `None` rather than a read past its end.
    #[inline]
    pub fn next_byte(&mut self, mem: &[u8]) -> Option<u8> {
        let index = self.cursor.next_index()?;
        // The counter read must be ordered before the data read, or the compiler may reuse it.
        compiler_fence(Ordering::Acquire);
        let slot = mem.get(index)?;
        // SAFETY: in bounds by the lookup above; volatile because the controller's write to it
        // is invisible to the compiler.
        Some(unsafe { core::ptr::read_volatile(slot) })
    }

    /// Fold the counter in and take one unit, with the units a lap overwrote since the last call.
    #[inline]
    pub fn pop(&mut self, mem: &[u8]) -> (Option<u8>, usize) {
        let lost = self.observe();
        (self.next_byte(mem), lost)
    }

    /// Fold the counter in and copy up to `out.len()` units out of `mem`: the batched receive
    /// path. Returns the units copied and the units a lap overwrote.
    pub fn read(&mut self, mem: &[u8], out: &mut [u8]) -> (usize, usize) {
        let lost = self.observe();
        let mut n = 0;
        while n < out.len() {
            match self.next_byte(mem) {
                Some(b) => {
                    out[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        (n, lost)
    }

    /// Re-base the cursor at the ring's base. Correct only when the caller reloads the transfer
    /// counter in the same breath; [`restart`](DmaRxRing::restart) does both.
    pub fn reset_cursor(&mut self) {
        // A latched reload belongs to the discarded session; consume it so it isn't charged later.
        let _ = self.chan.sample();
        self.cursor.reset();
        compiler_fence(Ordering::Release);
    }

    /// Restart the transfer, discarding everything in flight. The counter reload and the cursor
    /// re-base happen together, which is the only reason re-basing at the ring's base is correct.
    pub fn restart(&mut self) {
        self.chan.reload(self.cursor.len());
        self.reset_cursor();
    }
}

#[cfg(test)]
mod pending_tests {
    use super::DmaRingCursor;

    #[test]
    fn pending_reports_the_gap_without_moving_the_cursor() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.pending_at(3), 3);
        // Advisory only: asking does not advance anything.
        assert_eq!(c.available(), 0);
        c.observe(3);
        assert_eq!(c.available(), 3);
    }

    #[test]
    fn pending_wraps_with_the_ring_and_never_exceeds_it() {
        let mut c = DmaRingCursor::new(8);
        c.observe(6);
        for _ in 0..6 {
            c.next_index();
        }
        // Read sits at 6; a write position of 2 means four units across the seam.
        assert_eq!(c.pending_at(2), 4);
        // The wrap instant reads as the base, and a full ring is len - read, never more.
        assert_eq!(c.pending_at(8), 2);
    }
}

#[cfg(test)]
mod tests {
    use super::{DmaRingCursor, DmaRxChannel, DmaRxRing};
    use core::cell::Cell;

    /// Drain the whole backlog into `out`, returning the filled prefix.
    fn drain<'a>(c: &mut DmaRingCursor, out: &'a mut [usize; 16]) -> &'a [usize] {
        let mut n = 0;
        while let Some(i) = c.next_index() {
            out[n] = i;
            n += 1;
        }
        &out[..n]
    }

    #[test]
    fn a_fresh_cursor_is_empty() {
        let c = DmaRingCursor::new(8);
        assert_eq!(c.len(), 8);
        assert_eq!(c.available(), 0);
        assert!(c.is_empty());
    }

    #[test]
    fn production_accumulates_and_drains_in_order() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(3), 0);
        assert_eq!(c.available(), 3);
        assert_eq!(drain(&mut c, &mut [0; 16]), [0, 1, 2]);
        assert!(c.is_empty());
        assert_eq!(c.observe(5), 0);
        assert_eq!(drain(&mut c, &mut [0; 16]), [3, 4]);
    }

    #[test]
    fn the_cursor_wraps_without_reporting_a_lap() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(7), 0);
        assert_eq!(drain(&mut c, &mut [0; 16]), [0, 1, 2, 3, 4, 5, 6]);
        // A reading of `len` is the wrap instant: position 0, i.e. one more byte since 7.
        assert_eq!(c.observe(8), 0);
        assert_eq!(drain(&mut c, &mut [0; 16]), [7]);
        assert_eq!(c.observe(2), 0);
        assert_eq!(drain(&mut c, &mut [0; 16]), [0, 1]);
    }

    #[test]
    fn an_exactly_full_ring_is_not_a_lap() {
        // A loss here would be a false positive on a boundary healthy traffic actually visits.
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(4), 0);
        assert_eq!(c.observe(0), 0);
        assert_eq!(c.available(), 8);
        assert_eq!(drain(&mut c, &mut [0; 16]), [0, 1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn a_lap_is_reported_with_its_exact_loss_and_resyncs() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(4), 0);
        assert_eq!(c.observe(0), 0);
        assert_eq!(c.observe(3), 3);
        assert_eq!(c.available(), 8);
        assert_eq!(drain(&mut c, &mut [0; 16]), [3, 4, 5, 6, 7, 0, 1, 2]);
    }

    #[test]
    fn a_lap_is_reported_once_not_on_every_later_observation() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(4), 0);
        assert_eq!(c.observe(0), 0);
        assert_eq!(c.observe(2), 2);
        assert_eq!(c.observe(2), 0);
        assert_eq!(c.observe(2), 0);
    }

    #[test]
    fn draining_between_observations_prevents_a_lap() {
        let mut c = DmaRingCursor::new(8);
        for step in 1..=3 {
            assert_eq!(c.observe(step * 2 % 8), 0);
            let mut buf = [0; 16];
            assert_eq!(drain(&mut c, &mut buf).len(), 2);
        }
        assert!(c.is_empty());
    }

    #[test]
    fn a_stall_longer_than_a_whole_ring_aliases_and_understates() {
        // The documented residual limit: 11 units into an 8-ring land the writer at position 3,
        // and a position-only fold cannot tell that stall from 3 units of healthy traffic.
        let mut aliased = DmaRingCursor::new(8);
        assert_eq!(aliased.observe(11 % 8), 0, "the lap goes unreported");
        assert_eq!(aliased.available(), 3, "five surviving units go unseen");
        // An absolute production count resolves the same stall in full.
        let mut counted = DmaRingCursor::new(8);
        assert_eq!(counted.observe_produced(11), 3);
        assert_eq!(counted.available(), 8);
    }

    #[test]
    fn a_reported_reload_resolves_the_ring_a_full_lap_of_starvation_hides() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe_reloaded(0, true), 0, "one ring is not yet a loss");
        assert_eq!(c.available(), 8);
        assert_eq!(drain(&mut c, &mut [0; 16]), [0, 1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn a_reload_past_an_advanced_position_adds_the_whole_ring_and_reports_the_overshoot() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(3), 0);
        // 2 units by the position, plus the ring the reload proves: 10 produced into 8.
        assert_eq!(c.observe_reloaded(5, true), 5);
        assert_eq!(c.available(), 8);
        assert_eq!(drain(&mut c, &mut [0; 16]), [5, 6, 7, 0, 1, 2, 3, 4]);
    }

    #[test]
    fn a_reload_the_modular_distance_already_carries_is_not_counted_twice() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(6), 0);
        assert_eq!(c.observe_reloaded(2, true), 2);
        assert_eq!(c.available(), 8);
    }

    #[test]
    fn an_absolute_count_charges_a_stall_of_several_rings_in_full() {
        // The stall a position fold understates: 8 units of 24 survive, 16 are reported.
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe_produced(24), 16);
        assert_eq!(c.available(), 8);
        // Resynchronized onto the oldest survivor, which is where the writer left off.
        assert_eq!(drain(&mut c, &mut [0; 16]), [0, 1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn an_absolute_count_agrees_with_the_position_fold_while_it_can_see() {
        // Below one ring the two entry points must not diverge.
        for len in 2..=MAX_LEN {
            for produced in 0..len {
                let mut by_pos = DmaRingCursor::new(len);
                let mut by_count = DmaRingCursor::new(len);
                assert_eq!(
                    by_pos.observe(produced),
                    by_count.observe_produced(produced)
                );
                assert_eq!(by_pos.available(), by_count.available());
                assert_eq!(by_pos, by_count);
            }
        }
    }

    #[test]
    fn an_absolute_count_leaves_the_cursor_where_the_writer_is() {
        // Every span must land the write cursor where the position would have.
        for len in 2..=MAX_LEN {
            for produced in 0..3 * len {
                let mut c = DmaRingCursor::new(len);
                c.observe_produced(produced);
                let mut walked = DmaRingCursor::new(len);
                for _ in 0..produced {
                    walked.observe_produced(1);
                }
                assert_eq!(c.prev_write, walked.prev_write, "len {len}, {produced}");
                assert_eq!(c.prev_write, produced % len);
            }
        }
    }

    #[test]
    fn an_exactly_full_ring_by_count_is_not_a_lap_either() {
        for len in 2..=MAX_LEN {
            let mut c = DmaRingCursor::new(len);
            assert_eq!(c.observe_produced(len), 0);
            assert_eq!(c.available(), len);
            assert_eq!(c.observe_produced(1), 1);
        }
    }

    #[test]
    fn reset_discards_everything() {
        let mut c = DmaRingCursor::new(8);
        assert_eq!(c.observe(5), 0);
        assert_eq!(c.next_index(), Some(0));
        c.reset();
        assert!(c.is_empty());
        assert_eq!(c.observe(2), 0);
        assert_eq!(drain(&mut c, &mut [0; 16]), [0, 1]);
    }

    #[test]
    fn a_single_unit_ring_degenerates_safely() {
        // `len == 1` makes every modular distance zero. Not useful, but it must not panic.
        let mut c = DmaRingCursor::new(1);
        assert_eq!(c.observe(0), 0);
        assert_eq!(c.observe(1), 0);
        assert!(c.is_empty());
    }

    #[test]
    #[should_panic(expected = "non-zero length")]
    fn a_zero_length_ring_is_rejected() {
        let _ = DmaRingCursor::new(0);
    }

    /// Largest ring the properties below sweep; none of the arithmetic is size-sensitive.
    const MAX_LEN: usize = 24;

    /// The modelled byte stream, distinct over a span far longer than any ring here.
    const fn stream_byte(k: usize) -> u8 {
        (k % 251) as u8
    }

    /// Counter, memory and cursor modelled together: the `len - remaining` derivation callers
    /// perform is where the boundaries live.
    struct DmaModel {
        len: usize,
        /// The controller's next write index, always below `len`.
        w: usize,
        /// Whether the model presents the transient `0` at the wrap rather than the reloaded `len`.
        wrap_reads_zero: bool,
        buf: [u8; MAX_LEN],
        /// Stream index of the unit in each slot: the truth the cursor can only infer.
        slot: [usize; MAX_LEN],
        produced: usize,
        cursor: DmaRingCursor,
        delivered: usize,
        skipped: usize,
        reported: usize,
        /// Stream index the consumer expects next.
        expect: usize,
    }

    impl DmaModel {
        fn new(len: usize) -> Self {
            Self::with_wrap_reading(len, false)
        }

        fn with_wrap_reading(len: usize, wrap_reads_zero: bool) -> Self {
            assert!(len <= MAX_LEN);
            Self {
                len,
                w: 0,
                wrap_reads_zero,
                buf: [0; MAX_LEN],
                slot: [0; MAX_LEN],
                produced: 0,
                cursor: DmaRingCursor::new(len),
                delivered: 0,
                skipped: 0,
                reported: 0,
                expect: 0,
            }
        }

        /// The transfer counter as a poll would read it.
        fn cndtr(&self) -> usize {
            if self.w == 0 && self.produced > 0 && self.wrap_reads_zero {
                0
            } else {
                self.len - self.w
            }
        }

        fn write_pos(&self) -> usize {
            self.len - self.cndtr().min(self.len)
        }

        fn produce(&mut self, n: usize) {
            for _ in 0..n {
                self.buf[self.w] = stream_byte(self.produced);
                self.slot[self.w] = self.produced;
                self.produced += 1;
                self.w += 1;
                if self.w == self.len {
                    self.w = 0;
                }
            }
        }

        fn observe(&mut self) -> usize {
            let lost = self.cursor.observe(self.write_pos());
            self.reported += lost;
            lost
        }

        /// Take one unit, checking it is intact and that the reader never goes backwards.
        fn take(&mut self) -> Option<u8> {
            let index = self.cursor.next_index()?;
            assert!(
                index < self.len,
                "index {index} outside a ring of {}",
                self.len
            );
            let k = self.slot[index];
            assert!(
                k < self.produced && self.produced - k <= self.len,
                "index {index} holds stream unit {k}, overwritten by unit {}",
                self.produced,
            );
            assert!(
                k >= self.expect,
                "the reader went backwards: {k} after {}",
                self.expect
            );
            self.skipped += k - self.expect;
            self.expect = k + 1;
            self.delivered += 1;
            Some(self.buf[index])
        }

        fn drain(&mut self) -> usize {
            let mut n = 0;
            while self.take().is_some() {
                n += 1;
            }
            n
        }

        /// A restart: counter reloaded, cursor re-based, ring memory left stale.
        fn restart(&mut self) {
            self.w = 0;
            self.produced = 0;
            self.delivered = 0;
            self.skipped = 0;
            self.reported = 0;
            self.expect = 0;
            self.cursor.reset();
        }

        /// Fold in, drain, and check the accounting invariants.
        fn settle_and_audit(&mut self) {
            self.observe();
            self.drain();
            assert_eq!(self.cursor.available(), 0);
            let truth = self.produced - self.delivered;
            assert!(
                self.reported <= truth,
                "reported {} lost against {truth} actually lost: the reading invented loss",
                self.reported,
            );
            assert_eq!(
                (truth - self.reported) % self.len,
                0,
                "the shortfall between {} reported and {truth} actual is not whole rings",
                self.reported,
            );
            assert!(self.expect <= self.produced);
            if self.delivered > 0 {
                // Every drain ends current: the newest unit, or whole rings behind it.
                assert_eq!(self.expect % self.len, self.produced % self.len);
            }
        }
    }

    /// A tiny xorshift: the stall patterns need to vary, not to be unpredictable.
    struct Rng(u32);

    impl Rng {
        fn next_below(&mut self, n: usize) -> usize {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            self.0 as usize % n
        }
    }

    #[test]
    fn the_transfer_counter_maps_onto_the_position_on_every_ring_size() {
        for len in 1..=MAX_LEN {
            for wrap_reads_zero in [false, true] {
                let mut m = DmaModel::with_wrap_reading(len, wrap_reads_zero);
                for k in 0..3 * len {
                    assert_eq!(m.produced, k);
                    if wrap_reads_zero && k > 0 && k % len == 0 {
                        // The wrap instant: `len - remaining` reads `len`, out of range by one.
                        assert_eq!(m.write_pos(), len);
                    } else {
                        assert_eq!(m.write_pos(), k % len);
                    }
                    assert_eq!(m.observe(), 0);
                    m.drain();
                    m.produce(1);
                }
                m.settle_and_audit();
                // A one-unit ring is the exception: the mapping carries no information.
                if len > 1 {
                    assert_eq!(m.delivered, m.produced);
                } else {
                    assert_eq!(m.delivered, 0);
                }
            }
        }
    }

    #[test]
    fn the_two_counter_readings_at_the_wrap_are_interchangeable() {
        // Which one a poll catches is a race no caller controls.
        for len in 2..=MAX_LEN {
            let mut reloaded = DmaModel::with_wrap_reading(len, false);
            let mut zero = DmaModel::with_wrap_reading(len, true);
            let mut rng = Rng(0x1234_5678);
            for _ in 0..128 {
                let n = rng.next_below(2 * len);
                reloaded.produce(n);
                zero.produce(n);
                assert_eq!(reloaded.observe(), zero.observe());
                assert_eq!(reloaded.cursor.available(), zero.cursor.available());
                let taken = rng.next_below(len + 1);
                for _ in 0..taken {
                    assert_eq!(reloaded.take(), zero.take());
                }
            }
            reloaded.settle_and_audit();
            zero.settle_and_audit();
            assert_eq!(reloaded.delivered, zero.delivered);
            assert_eq!(reloaded.reported, zero.reported);
        }
    }

    #[test]
    fn every_unit_arrives_once_and_in_order_while_the_reader_keeps_up() {
        for len in 2..=MAX_LEN {
            let mut rng = Rng(0x9e37_79b9);
            let mut m = DmaModel::new(len);
            for _ in 0..256 {
                m.produce(rng.next_below(len));
                assert_eq!(m.observe(), 0, "no loss while the reader keeps up");
                m.drain();
            }
            m.settle_and_audit();
            assert_eq!(m.delivered, m.produced);
            assert_eq!(m.reported, 0);
        }
    }

    #[test]
    fn an_exactly_full_ring_delivers_every_unit_where_the_position_alone_reads_empty() {
        for len in 2..=MAX_LEN {
            let mut m = DmaModel::new(len);
            m.produce(len - 1);
            assert_eq!(m.observe(), 0);
            m.produce(1);
            // From the position alone a full ring is indistinguishable from an empty one.
            assert_eq!(m.write_pos(), 0);
            assert_eq!(m.observe(), 0, "an exactly full ring is not a loss");
            assert_eq!(m.cursor.available(), len);
            assert_eq!(m.drain(), len);
            assert_eq!(m.expect, len, "every unit of the stream, in order");
            m.settle_and_audit();
            assert_eq!(m.reported, 0);
        }
    }

    #[test]
    fn a_lap_hands_back_the_newest_whole_ring_and_reports_the_exact_loss() {
        for len in 2..=MAX_LEN {
            for chunk in 1..len {
                for over in 1..=len {
                    let mut m = DmaModel::new(len);
                    let total = len + over;
                    while m.produced < total {
                        m.produce(chunk.min(total - m.produced));
                        m.observe();
                    }
                    assert_eq!(m.reported, over, "the loss is the overshoot, exactly");
                    assert_eq!(m.cursor.available(), len);
                    assert_eq!(m.drain(), len);
                    // Resynchronized to the oldest survivor: the newest whole ring, contiguous.
                    assert_eq!(m.expect, total);
                    assert_eq!(
                        m.skipped, m.reported,
                        "the count matches what the reader skipped"
                    );
                    m.settle_and_audit();
                }
            }
        }
    }

    #[test]
    fn a_stall_of_whole_rings_under_reports_by_whole_rings_and_never_invents_loss() {
        for len in 2..=MAX_LEN {
            for delta in 1..=3 * len {
                let mut m = DmaModel::new(len);
                m.produce(delta);
                // A single fold cannot exceed the ring, so it cannot flag a lap however far it ran.
                assert_eq!(m.observe(), 0);
                assert_eq!(m.cursor.available(), delta % len);
                assert_eq!(m.drain(), delta % len);
                if delta % len == 0 {
                    assert!(m.cursor.is_empty());
                    assert_eq!(m.delivered, 0);
                } else {
                    // The understatement costs the count, not the content.
                    assert_eq!(m.expect, delta);
                }
                assert_eq!(m.reported, 0);
                m.settle_and_audit();
                assert_eq!(m.produced - m.delivered, delta - delta % len);
            }
        }
    }

    #[test]
    fn the_units_handed_out_are_the_newest_under_any_stall_pattern() {
        // Stalls past a whole ring, and drains leaving a backlog for the next lap to eat into.
        for len in 2..=MAX_LEN {
            for seed in 1..=8u32 {
                let mut rng = Rng(seed.wrapping_mul(0x9e37_79b9) | 1);
                let mut m = DmaModel::new(len);
                for _ in 0..256 {
                    m.produce(rng.next_below(2 * len + 1));
                    m.observe();
                    let taken = rng.next_below(len + 1);
                    for _ in 0..taken {
                        if m.take().is_none() {
                            break;
                        }
                    }
                }
                m.settle_and_audit();
            }
        }
    }

    #[test]
    fn observing_before_every_unit_holds_the_same_invariants() {
        // A controller that keeps writing through the drain loop.
        for len in 2..=MAX_LEN {
            for seed in 1..=4u32 {
                let mut rng = Rng(seed.wrapping_mul(0x85eb_ca6b) | 1);
                let mut m = DmaModel::new(len);
                for _ in 0..128 {
                    m.produce(rng.next_below(len + 2));
                    for _ in 0..8 * len {
                        if rng.next_below(4) == 0 {
                            m.produce(1);
                        }
                        m.observe();
                        if m.take().is_none() {
                            break;
                        }
                    }
                }
                m.settle_and_audit();
            }
        }
    }

    #[test]
    fn a_restart_rebases_the_cursor_on_the_reloaded_counter() {
        for len in 2..=MAX_LEN {
            for first in 1..len {
                for taken in 0..first {
                    let mut m = DmaModel::new(len);
                    // Leave the cursor awkward: mid-ring, partly drained, sometimes a lap behind.
                    m.produce(first);
                    m.observe();
                    for _ in 0..taken {
                        assert!(m.take().is_some());
                    }
                    m.produce(len - 1);
                    m.observe();
                    assert!(!m.cursor.is_empty());

                    m.restart();
                    assert!(m.cursor.is_empty());
                    assert_eq!(m.cursor.len(), len);
                    assert_eq!(m.write_pos(), 0, "the reloaded counter reads the base");
                    // Nothing from before the restart comes back out, though the memory holds it,
                    assert_eq!(m.observe(), 0);
                    assert!(m.take().is_none());
                    // and what arrives after is measured from the base, in order.
                    m.produce(len - 1);
                    assert_eq!(m.observe(), 0);
                    assert_eq!(m.drain(), len - 1);
                    assert_eq!(m.expect, len - 1);
                    m.settle_and_audit();
                    assert_eq!(m.reported, 0);
                }
            }
        }
    }

    #[test]
    fn a_reset_not_paired_with_a_counter_reload_mis_measures_the_next_fold() {
        // Pinning the mismatch keeps the pairing from being separated by a later edit.
        for len in 2..=MAX_LEN {
            for pos in 1..len {
                let mut c = DmaRingCursor::new(len);
                assert_eq!(c.observe(pos), 0);
                c.reset();
                assert!(c.is_empty());
                assert_eq!(c.observe(pos), 0);
                assert_eq!(c.available(), pos, "{pos} units conjured from a stale base");
            }
        }
    }

    /// A model channel over plain cells: `remaining` is the counter as a poll reads it.
    struct ModelChannel<'c> {
        ndtr: &'c Cell<usize>,
        running: &'c Cell<bool>,
    }

    impl DmaRxChannel for ModelChannel<'_> {
        fn remaining(&self) -> usize {
            self.ndtr.get()
        }

        fn reload(&mut self, len: usize) {
            self.ndtr.set(len);
            self.running.set(true);
        }

        fn running(&self) -> bool {
            self.running.get()
        }
    }

    /// A sticky reload latch like the F1 transfer-complete flag: cleared by the read, and
    /// saturating -- two reloads between samples read as one.
    struct LapChannel<'c> {
        ndtr: &'c Cell<usize>,
        running: &'c Cell<bool>,
        reloaded: &'c Cell<bool>,
    }

    impl DmaRxChannel for LapChannel<'_> {
        fn remaining(&self) -> usize {
            self.ndtr.get()
        }

        fn reload(&mut self, len: usize) {
            self.ndtr.set(len);
            self.reloaded.set(false);
            self.running.set(true);
        }

        fn running(&self) -> bool {
            self.running.get()
        }

        fn sample(&mut self) -> (usize, bool) {
            (self.ndtr.get(), self.reloaded.replace(false))
        }
    }

    /// The controller side of the model, held apart from the ring so a test can produce between
    /// the ring's counter read and its data read.
    struct Controller<'c> {
        len: usize,
        w: usize,
        /// Units written since the last restart -- what the counter reflects.
        produced: usize,
        /// Units written since construction, monotone across restarts.
        stream: usize,
        ndtr: &'c Cell<usize>,
        /// The reload latch, for a test driving a [`LapChannel`].
        reloaded: Option<&'c Cell<bool>>,
        mem: [u8; MAX_LEN],
    }

    impl<'c> Controller<'c> {
        fn new(len: usize, ndtr: &'c Cell<usize>) -> Self {
            ndtr.set(len);
            Self {
                len,
                w: 0,
                produced: 0,
                stream: 0,
                ndtr,
                reloaded: None,
                mem: [0; MAX_LEN],
            }
        }

        /// A controller that also latches each counter reload, for a lap-counting channel.
        fn with_reloads(len: usize, ndtr: &'c Cell<usize>, reloaded: &'c Cell<bool>) -> Self {
            reloaded.set(false);
            Self {
                reloaded: Some(reloaded),
                ..Self::new(len, ndtr)
            }
        }

        fn produce(&mut self, n: usize) {
            for _ in 0..n {
                self.mem[self.w] = stream_byte(self.stream);
                self.stream += 1;
                self.produced += 1;
                self.w = if self.w + 1 == self.len {
                    0
                } else {
                    self.w + 1
                };
                if self.w == 0 {
                    if let Some(latch) = self.reloaded {
                        latch.set(true);
                    }
                }
                self.ndtr.set(self.len - self.w);
            }
        }

        /// The controller's half of a restart; the ring's `reload` already wrote the counter.
        fn restarted(&mut self) {
            self.w = 0;
            self.produced = 0;
        }
    }

    /// Which stream unit a delivered byte is; unique because `stream_byte` outlives the ring.
    fn live_unit(b: u8, stream: usize, len: usize) -> Option<usize> {
        (stream.saturating_sub(len)..stream).find(|&k| stream_byte(k) == b)
    }

    /// A ring over a fresh model channel, with the cells the test drives it through.
    fn model_ring<'c>(
        len: usize,
        ndtr: &'c Cell<usize>,
        run: &'c Cell<bool>,
    ) -> DmaRxRing<ModelChannel<'c>> {
        run.set(true);
        DmaRxRing::new(ModelChannel { ndtr, running: run }, len)
    }

    /// A ring over a channel that reports its counter reloads: the seam the VCOM bridge uses.
    fn lap_ring<'c>(
        len: usize,
        ndtr: &'c Cell<usize>,
        run: &'c Cell<bool>,
        reloaded: &'c Cell<bool>,
    ) -> DmaRxRing<LapChannel<'c>> {
        run.set(true);
        DmaRxRing::new(
            LapChannel {
                ndtr,
                running: run,
                reloaded,
            },
            len,
        )
    }

    #[test]
    fn the_ring_delivers_the_controller_stream_in_order() {
        for len in 2..=MAX_LEN {
            let ndtr = Cell::new(len);
            let run = Cell::new(true);
            let mut ctl = Controller::new(len, &ndtr);
            let mut ring = model_ring(len, &ndtr, &run);
            let mut expect = 0usize;
            for step in 0..4 * len {
                // Stalls short of a whole ring, drained each pass: the healthy path.
                ctl.produce(step % (len - 1) + 1);
                loop {
                    let (byte, lost) = ring.pop(&ctl.mem);
                    assert_eq!(lost, 0, "the reader keeps up, so nothing can lap");
                    let Some(b) = byte else { break };
                    assert_eq!(
                        b,
                        stream_byte(expect),
                        "unit {expect} out of order or stale"
                    );
                    expect += 1;
                }
            }
            assert_eq!(expect, ctl.stream, "every unit written came out");
        }
    }

    #[test]
    fn the_batched_read_delivers_the_same_stream_as_byte_at_a_time() {
        for len in 2..=MAX_LEN {
            let ndtr = Cell::new(len);
            let run = Cell::new(true);
            let mut ctl = Controller::new(len, &ndtr);
            let mut ring = model_ring(len, &ndtr, &run);
            let mut expect = 0usize;
            let mut out = [0u8; MAX_LEN];
            for step in 0..4 * len {
                ctl.produce(step % (len - 1) + 1);
                loop {
                    let (n, lost) = ring.read(&ctl.mem, &mut out);
                    assert_eq!(lost, 0, "the reader keeps up, so nothing can lap");
                    if n == 0 {
                        break;
                    }
                    for &b in &out[..n] {
                        assert_eq!(b, stream_byte(expect));
                        expect += 1;
                    }
                }
            }
            assert_eq!(expect, ctl.stream);
        }
    }

    #[test]
    fn a_batched_read_stops_at_the_caller_buffer_and_resumes_where_it_left_off() {
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let mut ctl = Controller::new(len, &ndtr);
        let mut ring = model_ring(len, &ndtr, &run);
        ctl.produce(5);
        let mut out = [0u8; 2];
        assert_eq!(ring.read(&ctl.mem, &mut out), (2, 0));
        assert_eq!(out, [stream_byte(0), stream_byte(1)]);
        assert_eq!(ring.available(), 3);
        let mut rest = [0u8; 8];
        assert_eq!(ring.read(&ctl.mem, &mut rest), (3, 0));
        assert_eq!(
            &rest[..3],
            &[stream_byte(2), stream_byte(3), stream_byte(4)]
        );
    }

    #[test]
    fn an_exactly_full_ring_delivers_every_byte_where_the_position_alone_reads_empty() {
        for len in 2..=MAX_LEN {
            let ndtr = Cell::new(len);
            let run = Cell::new(true);
            let mut ctl = Controller::new(len, &ndtr);
            let mut ring = model_ring(len, &ndtr, &run);
            ctl.produce(len - 1);
            assert_eq!(ring.observe(), 0);
            ctl.produce(1);
            assert_eq!(ring.write_pos(), 0);
            assert_eq!(ring.observe(), 0, "an exactly full ring is not a loss");
            assert_eq!(ring.available(), len);
            for k in 0..len {
                assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(k)));
            }
            assert_eq!(ring.next_byte(&ctl.mem), None);
        }
    }

    #[test]
    fn a_lap_delivers_the_newest_whole_ring_of_bytes_and_reports_the_exact_loss() {
        for len in 2..=MAX_LEN {
            for over in 1..=len {
                let ndtr = Cell::new(len);
                let run = Cell::new(true);
                let mut ctl = Controller::new(len, &ndtr);
                let mut ring = model_ring(len, &ndtr, &run);
                let total = len + over;
                let mut lost = 0;
                while ctl.stream < total {
                    ctl.produce((total - ctl.stream).min(len - 1));
                    lost += ring.observe();
                }
                assert_eq!(lost, over, "the loss is the overshoot, exactly");
                assert_eq!(ring.available(), len);
                // The newest whole ring -- not the modular difference, which mixes old with new.
                for k in 0..len {
                    assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(over + k)));
                }
                assert_eq!(ring.next_byte(&ctl.mem), None);
            }
        }
    }

    #[test]
    fn a_restart_reloads_the_counter_and_discards_what_was_in_flight() {
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let mut ctl = Controller::new(len, &ndtr);
        let mut ring = model_ring(len, &ndtr, &run);
        ctl.produce(3);
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.available(), 3);

        ring.restart();
        ctl.restarted();
        assert_eq!(ndtr.get(), len, "the counter was reloaded by the restart");
        assert!(ring.is_empty());
        assert_eq!(ring.write_pos(), 0);
        // The memory still holds the pre-restart bytes; none of them comes back.
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.next_byte(&ctl.mem), None);
        ctl.produce(2);
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(3)));
        assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(4)));
    }

    #[test]
    fn a_cursor_reset_between_a_stop_and_a_rearm_opens_a_clean_session() {
        // Nothing observes between the reset and the reload, so the base is the truth for both.
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let mut ctl = Controller::new(len, &ndtr);
        let mut ring = model_ring(len, &ndtr, &run);
        ctl.produce(5);
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.available(), 5);

        run.set(false);
        ring.reset_cursor();
        assert!(ring.is_empty());
        // Re-armed: the counter is reloaded and the controller starts over at the base.
        ndtr.set(len);
        ctl.restarted();
        run.set(true);
        assert_eq!(
            ring.observe(),
            0,
            "nothing carried over from before the stop"
        );
        ctl.produce(2);
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(5)));
        assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(6)));
        assert_eq!(ring.next_byte(&ctl.mem), None);
    }

    #[test]
    fn a_stopped_channel_is_not_observed_and_keeps_its_backlog_readable() {
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let mut ctl = Controller::new(len, &ndtr);
        let mut ring = model_ring(len, &ndtr, &run);
        ctl.produce(3);
        assert_eq!(ring.observe(), 0);
        // Stopping leaves the counter wherever it halted; folding that in would invent bytes.
        run.set(false);
        ndtr.set(1);
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.available(), 3);
        for k in 0..3 {
            assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(k)));
        }
    }

    #[test]
    fn a_writer_racing_the_first_byte_after_a_resync_replaces_it_and_is_not_counted() {
        // A resync leaves the reader at the write position, which is the slot written next.
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let mut ctl = Controller::new(len, &ndtr);
        let mut ring = model_ring(len, &ndtr, &run);
        ctl.produce(len - 1);
        assert_eq!(ring.observe(), 0);
        ctl.produce(3);
        assert_eq!(ring.observe(), 2);
        ctl.produce(1);
        assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(len + 2)));
        // The substitution costs a byte but is invisible to the accounting.
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.available(), len);
    }

    #[test]
    fn a_buffer_shorter_than_the_ring_yields_nothing_rather_than_reading_past_it() {
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let mut ring = model_ring(len, &ndtr, &run);
        ndtr.set(len - 5);
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.available(), 5);
        let short = [7u8; 3];
        for _ in 0..short.len() {
            assert_eq!(ring.next_byte(&short), Some(7));
        }
        assert_eq!(ring.next_byte(&short), None, "stops at the caller's slice");
    }

    #[test]
    fn both_counter_readings_at_the_wrap_map_onto_the_ring_base() {
        for len in 2..=MAX_LEN {
            let ndtr = Cell::new(len);
            let run = Cell::new(true);
            let mut ring = model_ring(len, &ndtr, &run);
            assert_eq!(ring.write_pos(), 0);
            // The transient zero a poll can catch just before the reload reads `len`.
            ndtr.set(0);
            assert_eq!(ring.write_pos(), len);
            assert_eq!(ring.observe(), 0);
            assert!(ring.is_empty());
            // A reading above the ring length is malformed; clamping keeps the subtraction sane.
            ndtr.set(len + 9);
            assert_eq!(ring.write_pos(), 0);
        }
    }

    #[test]
    fn a_starved_full_lap_is_delivered_where_the_position_alone_reads_empty() {
        for len in 2..=MAX_LEN {
            let ndtr = Cell::new(len);
            let run = Cell::new(true);
            let reloaded = Cell::new(false);
            let mut ctl = Controller::with_reloads(len, &ndtr, &reloaded);
            let mut ring = lap_ring(len, &ndtr, &run, &reloaded);
            ctl.produce(len);
            assert_eq!(ring.write_pos(), 0, "the position alone reads empty");
            assert_eq!(ring.observe(), 0, "one ring produced is not yet a loss");
            assert_eq!(ring.available(), len);
            for k in 0..len {
                assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(k)));
            }
            assert_eq!(ring.next_byte(&ctl.mem), None);
        }
    }

    #[test]
    fn a_starved_span_past_one_ring_reports_its_overshoot_and_hands_back_the_newest_ring() {
        for len in 2..=MAX_LEN {
            for over in 1..len {
                let ndtr = Cell::new(len);
                let run = Cell::new(true);
                let reloaded = Cell::new(false);
                let mut ctl = Controller::with_reloads(len, &ndtr, &reloaded);
                let mut ring = lap_ring(len, &ndtr, &run, &reloaded);
                ctl.produce(len + over);
                assert_eq!(ring.observe(), over, "the loss is the overshoot, exactly");
                assert_eq!(ring.available(), len);
                for k in 0..len {
                    assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(over + k)));
                }
            }
        }
    }

    #[test]
    fn a_starved_span_of_several_rings_aliases_down_to_one_and_never_invents_loss() {
        // The latch saturates: the count understates by whole rings, the window stays newest.
        for len in 2..=MAX_LEN {
            for laps in 2..=4 {
                let ndtr = Cell::new(len);
                let run = Cell::new(true);
                let reloaded = Cell::new(false);
                let mut ctl = Controller::with_reloads(len, &ndtr, &reloaded);
                let mut ring = lap_ring(len, &ndtr, &run, &reloaded);
                ctl.produce(laps * len);
                assert_eq!(
                    ring.observe(),
                    0,
                    "one ring of the {laps} is all a flag can say"
                );
                assert_eq!(ring.available(), len);
                for k in 0..len {
                    let newest = (laps - 1) * len + k;
                    assert_eq!(ring.next_byte(&ctl.mem), Some(stream_byte(newest)));
                }
            }
        }
    }

    #[test]
    fn the_batched_read_surfaces_the_loss_a_starved_ring_took() {
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let reloaded = Cell::new(false);
        let mut ctl = Controller::with_reloads(len, &ndtr, &reloaded);
        let mut ring = lap_ring(len, &ndtr, &run, &reloaded);
        ctl.produce(len + 3);
        let mut out = [0u8; 16];
        assert_eq!(ring.read(&ctl.mem, &mut out), (len, 3));
        assert_eq!(out[0], stream_byte(3));
        assert_eq!(ring.read(&ctl.mem, &mut out), (0, 0), "reported once");
    }

    /// Every ring length a seam hands this cursor: the LPC55 VCOM ring, and the two trace rings.
    const SHIPPING_LENS: [usize; 3] = [1023, 2048, 8192];

    /// One of them, for tests needing a compile-time buffer; divisible by their 64-byte bite.
    const SHIPPING_LEN: usize = 2048;

    #[test]
    fn every_shipping_ring_length_folds_a_full_ring_a_lap_and_a_starved_reload() {
        for len in SHIPPING_LENS {
            let mut c = DmaRingCursor::new(len);
            // Filled in the 64-byte bites a bridge service takes, then closed on the wrap.
            for pos in (64..len).step_by(64) {
                assert_eq!(c.observe(pos), 0, "len {len}");
            }
            assert_eq!(
                c.observe(0),
                0,
                "an exactly full ring is not a lap, len {len}"
            );
            assert_eq!(c.available(), len, "len {len}");
            // Three bytes past it, and the reader resynchronizes onto the oldest survivor.
            assert_eq!(c.observe(3), 3, "len {len}");
            assert_eq!(c.available(), len, "len {len}");
            assert_eq!(c.next_index(), Some(3), "len {len}");

            let mut c = DmaRingCursor::new(len);
            assert_eq!(
                c.observe_reloaded(0, true),
                0,
                "one starved lap is not a loss, len {len}"
            );
            assert_eq!(c.available(), len, "len {len}");
        }
    }

    #[test]
    fn the_shipping_ring_delivers_a_starved_lap_in_service_sized_bites() {
        let ndtr = Cell::new(SHIPPING_LEN);
        let run = Cell::new(true);
        // The counter is back at its reload value, so only the latch says a whole lap was written.
        let reloaded = Cell::new(true);
        let mut mem = [0u8; SHIPPING_LEN];
        for (k, b) in mem.iter_mut().enumerate() {
            *b = stream_byte(k);
        }
        let mut ring = lap_ring(SHIPPING_LEN, &ndtr, &run, &reloaded);
        assert_eq!(ring.observe(), 0);
        assert_eq!(ring.available(), SHIPPING_LEN);
        let mut out = [0u8; 64];
        for bite in 0..SHIPPING_LEN / out.len() {
            assert_eq!(ring.read(&mem, &mut out), (out.len(), 0));
            for (i, &b) in out.iter().enumerate() {
                assert_eq!(b, stream_byte(bite * out.len() + i));
            }
        }
        assert_eq!(ring.read(&mem, &mut out), (0, 0));
    }

    #[test]
    fn a_reload_reporting_channel_delivers_the_stream_without_inventing_loss() {
        // The healthy path: ordinary reloads a keeping-up reader crosses must not count as a lap.
        for len in 2..=MAX_LEN {
            let ndtr = Cell::new(len);
            let run = Cell::new(true);
            let reloaded = Cell::new(false);
            let mut ctl = Controller::with_reloads(len, &ndtr, &reloaded);
            let mut ring = lap_ring(len, &ndtr, &run, &reloaded);
            let mut expect = 0usize;
            for step in 0..4 * len {
                ctl.produce(step % (len - 1) + 1);
                loop {
                    let (byte, lost) = ring.pop(&ctl.mem);
                    assert_eq!(lost, 0, "the reader keeps up, so nothing can lap");
                    let Some(b) = byte else { break };
                    assert_eq!(b, stream_byte(expect));
                    expect += 1;
                }
            }
            assert_eq!(expect, ctl.stream, "every unit written came out");
        }
    }

    #[test]
    fn a_restart_discards_a_latched_reload_with_the_backlog() {
        let len = 8;
        let ndtr = Cell::new(len);
        let run = Cell::new(true);
        let reloaded = Cell::new(false);
        let mut ctl = Controller::with_reloads(len, &ndtr, &reloaded);
        let mut ring = lap_ring(len, &ndtr, &run, &reloaded);
        ctl.produce(len + 1);
        ring.restart();
        ctl.restarted();
        assert!(ring.is_empty());
        assert_eq!(
            ring.observe(),
            0,
            "the discarded lap is not charged to the new session"
        );
        assert_eq!(ring.next_byte(&ctl.mem), None);
    }

    #[test]
    fn the_bytes_handed_out_are_the_newest_under_any_stall_pattern() {
        for len in 2..=MAX_LEN {
            for seed in 1..=4u32 {
                let mut rng = Rng(seed.wrapping_mul(0x9e37_79b9) | 1);
                let ndtr = Cell::new(len);
                let run = Cell::new(true);
                let mut ctl = Controller::new(len, &ndtr);
                let mut ring = model_ring(len, &ndtr, &run);
                let (mut expect, mut delivered, mut reported) = (0usize, 0usize, 0usize);
                for _ in 0..256 {
                    ctl.produce(rng.next_below(2 * len + 1));
                    for _ in 0..rng.next_below(len + 1) {
                        let (byte, lost) = ring.pop(&ctl.mem);
                        reported += lost;
                        let Some(b) = byte else { break };
                        let k = live_unit(b, ctl.stream, len)
                            .expect("a byte the controller had already overwritten");
                        assert!(k >= expect, "the reader went backwards: {k} after {expect}");
                        expect = k + 1;
                        delivered += 1;
                    }
                }
                ring.observe();
                while let Some(b) = ring.next_byte(&ctl.mem) {
                    let k = live_unit(b, ctl.stream, len).expect("an overwritten byte");
                    assert!(k >= expect, "the reader went backwards: {k} after {expect}");
                    expect = k + 1;
                    delivered += 1;
                }
                assert!(
                    reported <= ctl.stream - delivered,
                    "the reading invented loss"
                );
                if delivered > 0 {
                    assert_eq!(expect % len, ctl.stream % len);
                }
            }
        }
    }
}
