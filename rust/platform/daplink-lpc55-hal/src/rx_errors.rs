use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

pub(super) const FRAME_ERRORS: u32 = (1 << 13) | (1 << 14) | (1 << 15);

pub(super) struct Counters {
    pub frame: AtomicU32,
    pub overrun: AtomicU32,
    charged: AtomicBool,
}

impl Counters {
    pub const fn new() -> Self {
        Self {
            frame: AtomicU32::new(0),
            overrun: AtomicU32::new(0),
            charged: AtomicBool::new(false),
        }
    }
}

pub(super) trait Access {
    fn exclude(&self, body: impl FnOnce());
    fn status(&self) -> u32;
    fn ack_status(&self, bits: u32);
    fn rxerr(&self) -> bool;
    fn owned(&self) -> bool;
    fn flush(&self);
    fn ack_rxerr(&self);
    fn boundary(&self) {}
}

pub(super) fn observe(io: &impl Access, counters: &Counters) {
    io.exclude(|| {
        io.boundary();
        let bits = io.status() & FRAME_ERRORS;
        io.boundary();
        if bits != 0 {
            counters.frame.fetch_add(1, Ordering::Relaxed);
            io.boundary();
            io.ack_status(bits);
            io.boundary();
        }
        let rxerr = io.rxerr();
        io.boundary();
        if rxerr {
            let owned = io.owned();
            io.boundary();
            if owned {
                io.flush();
                io.boundary();
            }
            io.ack_rxerr();
            io.boundary();
            let charge = overrun_episode(&counters.charged, true, owned);
            io.boundary();
            if charge {
                counters.overrun.fetch_add(1, Ordering::Relaxed);
                io.boundary();
            }
        } else {
            overrun_episode(&counters.charged, false, false);
            io.boundary();
        }
    });
}

#[inline(always)]
pub(super) fn overrun_episode(charged: &AtomicBool, rxerr: bool, owned: bool) -> bool {
    if !rxerr {
        charged.store(false, Ordering::Relaxed);
        return false;
    }
    // A flushed FIFO ends the episode; a DMA-owned FIFO needs a clear observation.
    !charged.swap(!owned, Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};
    extern crate std;
    use std::vec::Vec;

    #[derive(Default)]
    struct Registers {
        stat: u32,
        enables: u32,
        rxerr: bool,
        dma: bool,
        fifo: usize,
        flushes: usize,
        acknowledgements: Vec<u32>,
    }

    struct Fake {
        regs: RefCell<Registers>,
        counters: Counters,
        masked: Cell<bool>,
        pending: Cell<bool>,
        boundary: Cell<usize>,
        preempt_at: Cell<Option<usize>>,
        arrive_at: Cell<Option<(usize, u32, bool)>>,
    }

    impl Fake {
        fn new(dma: bool, stat: u32, rxerr: bool) -> Self {
            Self {
                regs: RefCell::new(Registers {
                    dma,
                    stat,
                    rxerr,
                    fifo: 9,
                    ..Registers::default()
                }),
                counters: Counters::new(),
                masked: Cell::new(false),
                pending: Cell::new(false),
                boundary: Cell::new(0),
                preempt_at: Cell::new(None),
                arrive_at: Cell::new(None),
            }
        }

        fn observe(&self) {
            observe(self, &self.counters);
        }

        fn counts(&self) -> (u32, u32) {
            (
                self.counters.frame.load(Ordering::Relaxed),
                self.counters.overrun.load(Ordering::Relaxed),
            )
        }
    }

    impl Access for Fake {
        fn exclude(&self, body: impl FnOnce()) {
            assert!(!self.masked.replace(true));
            body();
            self.masked.set(false);
            if self.pending.replace(false) {
                self.observe();
            }
        }
        fn status(&self) -> u32 {
            self.regs.borrow().stat
        }
        fn ack_status(&self, bits: u32) {
            let mut regs = self.regs.borrow_mut();
            regs.acknowledgements.push(bits);
            regs.stat &= !bits;
        }
        fn rxerr(&self) -> bool {
            self.regs.borrow().rxerr
        }
        fn owned(&self) -> bool {
            !self.regs.borrow().dma
        }
        fn flush(&self) {
            let mut regs = self.regs.borrow_mut();
            assert!(!regs.dma);
            regs.fifo = 0;
            regs.flushes += 1;
        }
        fn ack_rxerr(&self) {
            self.regs.borrow_mut().rxerr = false;
        }
        fn boundary(&self) {
            let point = self.boundary.get();
            self.boundary.set(point + 1);
            if let Some((at, bits, rxerr)) = self.arrive_at.get() {
                if point == at {
                    let mut regs = self.regs.borrow_mut();
                    regs.stat |= bits;
                    regs.rxerr |= rxerr;
                    self.arrive_at.set(None);
                }
            }
            if self.preempt_at.get() == Some(point) {
                self.preempt_at.set(None);
                assert!(self.masked.get());
                self.pending.set(true);
            }
        }
    }

    #[test]
    fn every_observer_boundary_defers_the_preempting_observer() {
        for dma in [false, true] {
            let reference = Fake::new(dma, FRAME_ERRORS, true);
            reference.observe();
            for point in 0..reference.boundary.get() {
                let fake = Fake::new(dma, FRAME_ERRORS, true);
                fake.regs.borrow_mut().enables = FRAME_ERRORS;
                fake.preempt_at.set(Some(point));
                fake.observe();
                assert_eq!(fake.counts(), (1, 1), "boundary {point}, dma {dma}");
                assert_eq!(fake.regs.borrow().flushes, usize::from(!dma));
                assert_eq!(fake.regs.borrow().fifo, if dma { 9 } else { 0 });
            }
        }
    }

    #[test]
    fn raw_flags_are_visible_without_interrupt_enables_and_only_sampled_bits_are_cleared() {
        for bit in [1 << 13, 1 << 14, 1 << 15] {
            let fake = Fake::new(true, bit, false);
            assert_eq!(fake.regs.borrow().stat & fake.regs.borrow().enables, 0);
            fake.observe();
            fake.observe();
            assert_eq!(fake.counts(), (1, 0));
            assert_eq!(fake.regs.borrow().acknowledgements, [bit]);
        }
        let fake = Fake::new(true, 1 << 13, false);
        fake.arrive_at.set(Some((1, 1 << 14, false)));
        fake.observe();
        assert_eq!(fake.regs.borrow().stat, 1 << 14);
        fake.observe();
        assert_eq!(fake.counts(), (2, 0));
    }

    #[test]
    fn dma_episodes_need_a_clear_observation_and_never_empty_the_fifo() {
        let fake = Fake::new(true, 0, true);
        for (error, expected) in [(true, 1), (true, 1), (false, 1), (true, 2)] {
            fake.regs.borrow_mut().rxerr = error;
            fake.observe();
            assert_eq!(fake.counts(), (0, expected));
        }
        assert_eq!(fake.regs.borrow().fifo, 9);
        assert_eq!(fake.regs.borrow().flushes, 0);
    }

    #[test]
    fn quiet_observer_boundaries_also_exclude_the_preempting_reader() {
        let reference = Fake::new(true, 0, false);
        reference.observe();
        for point in 0..reference.boundary.get() {
            let fake = Fake::new(true, 0, false);
            fake.arrive_at.set(Some((point, 1 << 13, true)));
            fake.preempt_at.set(Some(point));
            fake.observe();
            assert_eq!(fake.counts(), (1, 1));
            assert_eq!(fake.regs.borrow().flushes, 0);
        }
    }

    #[test]
    fn dma_reassertion_after_ack_remains_a_conservative_episode() {
        let fake = Fake::new(true, 0, true);
        fake.arrive_at.set(Some((4, 0, true)));
        fake.preempt_at.set(Some(4));
        fake.observe();
        assert_eq!(fake.counts(), (0, 1));
        assert_eq!(fake.regs.borrow().flushes, 0);
        assert!(fake.counters.charged.load(Ordering::Relaxed));
    }

    #[test]
    fn new_events_after_ack_are_observed_but_same_bit_before_ack_coalesces() {
        for (point, expected) in [(1, 1), (3, 2)] {
            let fake = Fake::new(true, 1 << 13, false);
            fake.arrive_at.set(Some((point, 1 << 13, false)));
            fake.preempt_at.set(Some(point));
            fake.observe();
            assert_eq!(fake.counts(), (expected, 0));
        }
    }

    #[test]
    fn stale_clear_cannot_erase_a_preempting_observers_episode_latch() {
        let fake = Fake::new(true, 0, false);
        fake.arrive_at.set(Some((2, 0, true)));
        fake.preempt_at.set(Some(2));
        fake.observe();
        assert_eq!(fake.counts(), (0, 1));
        assert!(fake.counters.charged.load(Ordering::Relaxed));
        fake.regs.borrow_mut().rxerr = true;
        fake.observe();
        assert_eq!(fake.counts(), (0, 1));
    }

    #[test]
    fn owned_new_episode_after_ack_is_not_stale_flushed_or_double_charged() {
        let fake = Fake::new(false, 0, true);
        fake.arrive_at.set(Some((5, 0, true)));
        fake.preempt_at.set(Some(5));
        fake.observe();
        assert_eq!(fake.counts(), (0, 2));
        assert_eq!(fake.regs.borrow().flushes, 2);
    }

    #[test]
    fn counters_keep_wrapping_and_quiet_observations_reopen_the_latch() {
        let fake = Fake::new(true, FRAME_ERRORS, true);
        fake.counters.frame.store(u32::MAX, Ordering::Relaxed);
        fake.counters.overrun.store(u32::MAX, Ordering::Relaxed);
        fake.observe();
        assert_eq!(fake.counts(), (0, 0));
        fake.observe();
        assert!(!fake.counters.charged.load(Ordering::Relaxed));
    }
}
