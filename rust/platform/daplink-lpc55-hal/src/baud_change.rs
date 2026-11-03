use core::sync::atomic::{AtomicU32, Ordering};

pub(super) trait Access {
    fn exclude(&self, body: impl FnOnce());
    fn drained(&self) -> bool;
    fn tx_idle(&self) -> bool;
    fn program(&self, baud: u32) -> bool;
    fn tx_idle_interrupt(&self, enabled: bool);
    fn pend(&self);
    fn validating(&self) {}
    fn boundary(&self) {}
}

#[inline(always)]
pub(super) fn request(access: &impl Access, applied: &AtomicU32, pending: &AtomicU32, baud: u32) {
    // Sample pending first: its IRQ can consume a rate between the two loads.
    let settled = pending.load(Ordering::Acquire) == 0;
    access.boundary();
    if settled && applied.load(Ordering::Acquire) == baud {
        return;
    }
    access.validating();
    if crate::baud::flexcomm_osr_brg(crate::baud::VCOM_CLOCK_HZ, baud, crate::baud::VCOM_OSR_MIN)
        .is_none()
    {
        return;
    }
    // FLEXCOMM0 must not apply an older request between this decision and publication.
    access.exclude(|| {
        access.boundary();
        if applied.load(Ordering::Acquire) == baud {
            access.boundary();
            if pending.load(Ordering::Relaxed) != 0 {
                pending.store(0, Ordering::Release);
                access.boundary();
                access.tx_idle_interrupt(false);
            }
            return;
        }
        // Publish before attempting the change so the baud query cannot report it settled early.
        pending.store(baud, Ordering::Release);
        access.boundary();
        if apply_pending(access, pending, access.drained()) {
            return;
        }
        access.tx_idle_interrupt(true);
        access.pend();
    });
}

#[inline(always)]
fn apply_pending(access: &impl Access, pending: &AtomicU32, drained: bool) -> bool {
    let baud = pending.load(Ordering::Relaxed);
    access.boundary();
    if drained && baud != 0 && access.tx_idle() && access.program(baud) {
        access.boundary();
        pending
            .compare_exchange(baud, 0, Ordering::Release, Ordering::Relaxed)
            .ok();
        return true;
    }
    false
}

#[inline(always)]
pub(super) fn service(access: &impl Access, pending: &AtomicU32) {
    let drained = access.drained();
    apply_pending(access, pending, drained);
    access
        .tx_idle_interrupt(drained && (pending.load(Ordering::Relaxed) != 0 || !access.tx_idle()));
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};
    extern crate std;
    use std::vec::Vec;

    const A: u32 = 115_200;
    const B: u32 = 921_600;
    const C: u32 = 2_000_000;

    struct Registers {
        applied: AtomicU32,
        pending: AtomicU32,
        realized: Cell<u32>,
        drained: Cell<bool>,
        idle: Cell<bool>,
        tx_irq: Cell<bool>,
        programs: RefCell<Vec<u32>>,
        irq_writes: Cell<usize>,
        pends: Cell<usize>,
        validations: Cell<usize>,
        exclusions: Cell<usize>,
        excluded: Cell<bool>,
        steps: Cell<usize>,
        interrupt_at: Cell<Option<usize>>,
        deferred: Cell<bool>,
        refuse_program: Cell<bool>,
    }

    impl Registers {
        fn new() -> Self {
            Self {
                applied: AtomicU32::new(A),
                pending: AtomicU32::new(0),
                realized: Cell::new(115_385),
                drained: Cell::new(false),
                idle: Cell::new(false),
                tx_irq: Cell::new(false),
                programs: RefCell::new(Vec::new()),
                irq_writes: Cell::new(0),
                pends: Cell::new(0),
                validations: Cell::new(0),
                exclusions: Cell::new(0),
                excluded: Cell::new(false),
                steps: Cell::new(0),
                interrupt_at: Cell::new(None),
                deferred: Cell::new(false),
                refuse_program: Cell::new(false),
            }
        }

        fn request(&self, baud: u32) {
            request(self, &self.applied, &self.pending, baud);
        }

        fn service(&self) {
            service(self, &self.pending);
        }

        fn quiet(&self) {
            self.drained.set(true);
            self.idle.set(true);
        }

        fn state(&self) -> (u32, u32) {
            (
                self.applied.load(Ordering::Acquire),
                self.pending.load(Ordering::Acquire),
            )
        }

        fn schedule(&self, step: usize) {
            self.steps.set(0);
            self.interrupt_at.set(Some(step));
        }
    }

    impl Access for Registers {
        fn exclude(&self, body: impl FnOnce()) {
            self.exclusions.set(self.exclusions.get() + 1);
            assert!(!self.excluded.replace(true));
            body();
            self.excluded.set(false);
            if self.deferred.replace(false) {
                self.service();
            }
        }

        fn drained(&self) -> bool {
            self.boundary();
            self.drained.get()
        }

        fn tx_idle(&self) -> bool {
            self.boundary();
            self.idle.get()
        }

        fn program(&self, baud: u32) -> bool {
            self.boundary();
            if self.refuse_program.get() {
                return false;
            }
            let Some((osr, brg)) = crate::baud::flexcomm_osr_brg(
                crate::baud::VCOM_CLOCK_HZ,
                baud,
                crate::baud::VCOM_OSR_MIN,
            ) else {
                panic!("unrepresentable rate reached divisor programming");
            };
            self.programs.borrow_mut().push(baud);
            self.realized
                .set(crate::baud::realized(crate::baud::VCOM_CLOCK_HZ, osr, brg));
            self.applied.store(baud, Ordering::Release);
            self.boundary();
            true
        }

        fn tx_idle_interrupt(&self, enabled: bool) {
            self.boundary();
            self.tx_irq.set(enabled);
            self.irq_writes.set(self.irq_writes.get() + 1);
        }

        fn pend(&self) {
            self.boundary();
            self.pends.set(self.pends.get() + 1);
        }

        fn validating(&self) {
            self.validations.set(self.validations.get() + 1);
        }

        fn boundary(&self) {
            let step = self.steps.get();
            self.steps.set(step + 1);
            if self.interrupt_at.get() == Some(step) {
                self.interrupt_at.set(None);
                self.quiet();
                if self.excluded.get() {
                    self.deferred.set(true);
                } else {
                    self.service();
                }
            }
        }
    }

    #[test]
    fn returning_to_the_applied_rate_cancels_a_busy_pending_change() {
        let regs = Registers::new();
        regs.request(B);
        assert_eq!(regs.state(), (A, B));
        assert!(regs.tx_irq.get());
        regs.request(A);
        assert_eq!(regs.state(), (A, 0));
        assert!(!regs.tx_irq.get());
        regs.service();
        regs.quiet();
        regs.service();
        regs.service();
        assert_eq!(regs.state(), (A, 0));
        assert_eq!(regs.realized.get(), 115_385);
        assert!(regs.programs.borrow().is_empty());
    }

    #[test]
    fn settled_repeats_do_not_program_or_rearm_service() {
        let regs = Registers::new();
        for _ in 0..3 {
            regs.request(A);
        }
        regs.quiet();
        regs.request(A);
        assert_eq!(regs.state(), (A, 0));
        assert!(regs.programs.borrow().is_empty());
        assert_eq!(regs.irq_writes.get(), 0);
        assert_eq!(regs.pends.get(), 0);
        assert_eq!(regs.validations.get(), 0);
        assert_eq!(regs.exclusions.get(), 0);
    }

    #[test]
    fn invalid_rates_preserve_applied_pending_and_interrupt_state() {
        let regs = Registers::new();
        for pending in [0, B] {
            if pending != 0 {
                regs.request(pending);
            }
            let before = (regs.state(), regs.irq_writes.get(), regs.pends.get());
            for baud in [0, 1, 6_000_000, u32::MAX] {
                regs.request(baud);
                assert_eq!(
                    (regs.state(), regs.irq_writes.get(), regs.pends.get()),
                    before,
                    "invalid rate {baud} changed state"
                );
            }
        }
        regs.quiet();
        regs.service();
        assert_eq!(regs.state(), (B, 0));
        assert_eq!(*regs.programs.borrow(), [B]);
    }

    #[test]
    fn the_latest_valid_pending_rate_is_the_only_one_programmed() {
        let regs = Registers::new();
        regs.request(B);
        regs.request(C);
        regs.request(C);
        assert_eq!(regs.state(), (A, C));
        regs.drained.set(true);
        regs.service();
        assert_eq!(regs.state(), (A, C));
        assert!(regs.tx_irq.get());
        regs.idle.set(true);
        regs.service();
        assert_eq!(regs.state(), (C, 0));
        assert_eq!(*regs.programs.borrow(), [C]);
        assert_eq!(regs.realized.get(), C);
        assert!(!regs.tx_irq.get());
        regs.request(5_250_000);
        assert_eq!(regs.state(), (5_250_000, 0));
        assert_eq!(regs.realized.get(), 5_333_333);
    }

    #[test]
    fn dma_drain_and_uart_idle_are_both_required() {
        for (drained, idle) in [(false, false), (false, true), (true, false)] {
            let regs = Registers::new();
            regs.drained.set(drained);
            regs.idle.set(idle);
            regs.request(B);
            regs.service();
            assert_eq!(regs.state(), (A, B));
            assert!(regs.programs.borrow().is_empty());
            assert_eq!(regs.tx_irq.get(), drained);
            regs.quiet();
            regs.service();
            assert_eq!(regs.state(), (B, 0));
            assert_eq!(*regs.programs.borrow(), [B]);
        }
    }

    #[test]
    fn a_failed_programming_attempt_keeps_the_request_pending() {
        let regs = Registers::new();
        regs.quiet();
        regs.refuse_program.set(true);
        regs.request(B);
        regs.service();
        assert_eq!(regs.state(), (A, B));
        assert!(regs.tx_irq.get());
        assert!(regs.programs.borrow().is_empty());
        regs.refuse_program.set(false);
        regs.service();
        assert_eq!(regs.state(), (B, 0));
    }

    #[test]
    fn cancellation_defers_interrupts_at_every_request_boundary() {
        for step in 1..5 {
            let regs = Registers::new();
            regs.request(B);
            regs.schedule(step);
            regs.request(A);
            assert_eq!(regs.interrupt_at.get(), None, "boundary {step} missed");
            assert_eq!(regs.state(), (A, 0), "boundary {step}");
            assert!(regs.programs.borrow().is_empty(), "boundary {step}");
            assert!(!regs.tx_irq.get());
            regs.service();
            assert!(regs.programs.borrow().is_empty());
        }
    }

    #[test]
    fn replacing_a_pending_request_defers_interrupts_through_programming() {
        let probe = Registers::new();
        probe.quiet();
        probe.request(C);
        let boundaries = probe.steps.get();
        for step in 1..boundaries {
            let regs = Registers::new();
            regs.request(B);
            regs.quiet();
            regs.schedule(step);
            regs.request(C);
            assert_eq!(regs.interrupt_at.get(), None, "boundary {step} missed");
            assert_eq!(regs.state(), (C, 0), "boundary {step}");
            assert_eq!(*regs.programs.borrow(), [C], "boundary {step}");
            regs.service();
            assert_eq!(*regs.programs.borrow(), [C]);
        }
    }

    #[test]
    fn an_interrupt_completed_before_the_request_is_superseded_normally() {
        let regs = Registers::new();
        regs.request(B);
        regs.quiet();
        regs.service();
        regs.request(A);
        assert_eq!(regs.state(), (A, 0));
        assert_eq!(*regs.programs.borrow(), [B, A]);
    }

    #[test]
    fn pending_first_check_survives_interrupt_consumption_before_applied_read() {
        let regs = Registers::new();
        regs.request(B);
        regs.schedule(0);
        regs.request(A);
        assert_eq!(regs.interrupt_at.get(), None);
        assert_eq!(regs.state(), (A, 0));
        assert_eq!(*regs.programs.borrow(), [B, A]);
    }
}
