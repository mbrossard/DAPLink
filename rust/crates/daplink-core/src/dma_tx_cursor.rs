//! Cursor arithmetic for a circular DMA transmit ring armed one contiguous run at a time.

/// What [`DmaTxCursor::accept`] took, split at the ring's end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Accepted {
    /// Bytes taken in total, `first + second`.
    pub taken: usize,
    /// Bytes to copy at the old head.
    pub first: usize,
    /// Bytes to copy at the ring's base; zero when the run does not wrap.
    pub second: usize,
}

/// Append, arm and in-flight positions over a `LEN`-byte ring; head == tail is empty.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaTxCursor<const LEN: usize, const MAX_XFER: usize = { usize::MAX }> {
    head: usize,
    tail: usize,
    inflight: usize,
}

impl<const LEN: usize, const MAX_XFER: usize> DmaTxCursor<LEN, MAX_XFER> {
    /// A cursor over the caller's three positions.
    #[inline]
    pub const fn from_parts(head: usize, tail: usize, inflight: usize) -> Self {
        Self {
            head,
            tail,
            inflight,
        }
    }

    /// Head, tail and in-flight count, in that order.
    #[inline]
    pub const fn parts(&self) -> (usize, usize, usize) {
        (self.head, self.tail, self.inflight)
    }

    /// Bytes staged in the ring, sent and unsent both.
    #[inline]
    pub const fn queued(&self) -> usize {
        (self.head + LEN - self.tail) % LEN
    }

    /// Bytes the ring will still take.
    #[inline]
    pub const fn free(&self) -> usize {
        LEN - 1 - self.queued()
    }

    /// Whether a transfer is in flight.
    #[inline]
    pub const fn is_armed(&self) -> bool {
        self.inflight != 0
    }

    /// Stage up to `n` bytes, advancing the head; the split tells the caller where to copy.
    #[inline]
    pub fn accept(&mut self, n: usize) -> Accepted {
        let taken = n.min(self.free());
        let first = (LEN - self.head).min(taken);
        self.head = (self.head + taken) % LEN;
        Accepted {
            taken,
            first,
            second: taken - first,
        }
    }

    /// The next contiguous run to arm, or `None` if one is in flight or nothing is queued.
    #[inline]
    pub fn arm(&mut self) -> Option<(usize, usize)> {
        if self.inflight != 0 || self.head == self.tail {
            return None;
        }
        let run = if self.head > self.tail {
            self.head - self.tail
        } else {
            LEN - self.tail
        };
        let len = run.min(MAX_XFER);
        self.inflight = len;
        Some((self.tail, len))
    }

    /// Retire the transfer in flight, returning the bytes it carried for the caller to credit.
    #[inline]
    pub fn reap(&mut self) -> usize {
        let done = self.inflight;
        self.tail = (self.tail + done) % LEN;
        self.inflight = 0;
        done
    }

    /// Retire the in-flight transfer on an error path, advancing as [`reap`](Self::reap) does; the
    /// count never reached the wire, so it is not to be credited.
    #[inline]
    pub fn drop_inflight(&mut self) -> usize {
        self.reap()
    }

    /// Empty the ring and forget the transfer in flight; the caller stops the channel itself.
    #[inline]
    pub fn reset(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.inflight = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type C8 = DmaTxCursor<8>;
    type C64 = DmaTxCursor<64>;

    /// Deterministic, dependency-free source for the property run.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }

        fn below(&mut self, n: usize) -> usize {
            (self.next() % n as u64) as usize
        }
    }

    #[test]
    fn empty_and_full_invariants() {
        let mut c = C8::from_parts(0, 0, 0);
        assert_eq!(c.queued(), 0);
        assert_eq!(c.free(), 7);
        assert!(!c.is_armed());
        assert_eq!(c.arm(), None);

        let a = c.accept(100);
        assert_eq!(a.taken, 7);
        assert_eq!(c.free(), 0);
        assert_eq!(c.accept(1).taken, 0);
    }

    #[test]
    fn accept_wraps_at_every_head() {
        for head in 0..8 {
            let mut c = C8::from_parts(head, head, 0);
            let a = c.accept(7);
            assert_eq!(a.taken, 7);
            assert_eq!(a.first, (8 - head).min(7));
            assert_eq!(a.second, 7 - a.first);
            assert_eq!(c.parts().0, (head + 7) % 8);
        }
    }

    #[test]
    fn accept_without_wrap_has_no_second() {
        let mut c = C8::from_parts(2, 2, 0);
        let a = c.accept(4);
        assert_eq!((a.taken, a.first, a.second), (4, 4, 0));
    }

    #[test]
    fn arm_never_crosses_the_ring_end() {
        for head in 0..8 {
            for tail in 0..8 {
                let mut c = C8::from_parts(head, tail, 0);
                match c.arm() {
                    None => assert_eq!(head, tail),
                    Some((start, len)) => {
                        assert_eq!(start, tail);
                        assert!(len > 0 && start + len <= 8);
                        assert!(c.is_armed());
                        assert_eq!(c.arm(), None);
                    }
                }
            }
        }
    }

    #[test]
    fn arm_caps_at_max_xfer() {
        for tail in 0..64 {
            for head in 0..64 {
                let mut c = DmaTxCursor::<64, 7>::from_parts(head, tail, 0);
                if let Some((start, len)) = c.arm() {
                    assert_eq!(start, tail);
                    assert!(len <= 7);
                    assert!(start + len <= 64);
                }
            }
        }
    }

    #[test]
    fn reap_after_a_capped_run_leaves_the_remainder_armable() {
        // Head wrapped past the base, so the first run stops at the ring's end and is capped.
        let mut c = DmaTxCursor::<64, 7>::from_parts(4, 60, 0);
        assert_eq!(c.queued(), 8);
        assert_eq!(c.arm(), Some((60, 4)));
        assert_eq!(c.reap(), 4);
        assert_eq!(c.parts(), (4, 0, 0));
        assert_eq!(c.arm(), Some((0, 4)));
        assert_eq!(c.reap(), 4);
        assert_eq!(c.queued(), 0);
        assert_eq!(c.arm(), None);
    }

    #[test]
    fn drop_inflight_advances_like_reap() {
        let mut dropped = C64::from_parts(30, 10, 0);
        let mut reaped = dropped;
        assert_eq!(dropped.arm(), Some((10, 20)));
        assert_eq!(reaped.arm(), Some((10, 20)));
        assert_eq!(dropped.drop_inflight(), 20);
        assert_eq!(reaped.reap(), 20);
        assert_eq!(dropped.parts(), reaped.parts());
    }

    #[test]
    fn reset_from_any_state() {
        for head in 0..8 {
            for tail in 0..8 {
                let mut c = C8::from_parts(head, tail, 0);
                c.arm();
                c.reset();
                assert_eq!(c.parts(), (0, 0, 0));
                assert_eq!(c.queued(), 0);
                assert_eq!(c.free(), 7);
            }
        }
    }

    /// One property run: random accept/arm/reap/drop, checking the ledger and every index.
    fn property_run<const LEN: usize, const MAX_XFER: usize>(seed: u64) {
        let mut rng = Rng(seed);
        let mut c = DmaTxCursor::<LEN, MAX_XFER>::from_parts(0, 0, 0);
        let (mut accepted, mut reaped, mut dropped) = (0usize, 0usize, 0usize);

        for _ in 0..100_000 {
            match rng.below(4) {
                0 => accepted += c.accept(rng.below(2 * LEN)).taken,
                1 => {
                    let armed = c.is_armed();
                    if let Some((start, len)) = c.arm() {
                        assert!(!armed);
                        assert!(len > 0 && len <= MAX_XFER);
                        assert!(start < LEN && start + len <= LEN);
                    } else {
                        assert!(armed || c.queued() == 0);
                    }
                }
                2 => reaped += c.reap(),
                _ => dropped += c.drop_inflight(),
            }
            let (head, tail, inflight) = c.parts();
            assert!(head < LEN && tail < LEN);
            assert!(inflight <= c.queued());
            assert_eq!(c.queued(), accepted - reaped - dropped);
            assert_eq!(c.free(), LEN - 1 - c.queued());
        }
    }

    #[test]
    fn property_runs_hold_the_ledger() {
        property_run::<8, { usize::MAX }>(0x2545_F491_4F6C_DD1D);
        property_run::<8, 7>(0x1234_5678_9ABC_DEF1);
        property_run::<64, { usize::MAX }>(0x0BAD_C0DE_DEAD_BEEF);
        property_run::<64, 1023>(0x9E37_79B9_7F4A_7C15);
        property_run::<64, 7>(0x0000_0000_0000_0BAD);
        property_run::<1024, { usize::MAX }>(0xDEAD_BEEF_CAFE_F00D);
        property_run::<1024, 1023>(0x5DEE_CE66_D000_0001);
        property_run::<1024, 7>(0x0123_4567_89AB_CDEF);
    }
}
