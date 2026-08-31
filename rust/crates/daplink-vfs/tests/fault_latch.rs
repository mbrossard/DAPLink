//! The panic/hard-fault latch, end to end: arm, latch, survive a reset, bound a reset loop.
//!
//! One test in its own binary on purpose. The latch is a process-wide static armed by
//! `boot_in_place`, so a second test running in parallel would arm a different stamp and the
//! two would write over each other's evidence.

use core::mem::MaybeUninit;

use daplink_vfs::stamp::{
    clear_fault_streak, latch_hard_fault, latch_panic, FAULT_RESET_LIMIT as LIMIT,
};
use daplink_vfs::ResetStamp;

/// Boot the stamp again in place, the way a reset does: the RAM keeps its contents.
fn reboot(slot: *mut MaybeUninit<ResetStamp>) -> &'static mut ResetStamp {
    // SAFETY (test only): the slot is leaked, so it outlives every borrow taken from it, and
    // the previous borrow has ended before this is called.
    ResetStamp::boot_in_place(unsafe { &mut *slot })
}

#[test]
fn a_fault_latches_a_reason_that_survives_the_reset_it_asks_for() {
    let slot: *mut MaybeUninit<ResetStamp> =
        Box::leak(Box::new(MaybeUninit::zeroed())) as *mut MaybeUninit<ResetStamp>;

    // A fault before anything armed the latch still asks for a reset -- the recovery does not
    // depend on the evidence -- and records nothing.
    assert!(latch_panic(None).resets());

    {
        let stamp = reboot(slot);
        assert!(stamp.fault().is_none(), "a power-on boot carries no fault");
        assert_eq!(stamp.fault_streak(), 0);
    }

    // A panic latches its location and asks for the reset.
    assert!(latch_panic(Some(("firmware/rp2040-pico-embassy/src/main.rs", 412))).resets());
    {
        let stamp = reboot(slot);
        let reason = core::str::from_utf8(stamp.fault().expect("a fault was latched")).unwrap();
        assert!(reason.starts_with("panic "), "{reason}");
        assert!(reason.ends_with("src/main.rs:412"), "{reason}");
        assert_eq!(stamp.reset_count(), 1, "the reset itself is counted");
        assert_eq!(stamp.fault_streak(), 1);
    }

    // A hard fault records the stacked program counter instead.
    assert!(latch_hard_fault(0x0800_a1b2).resets());
    {
        let stamp = reboot(slot);
        let reason = core::str::from_utf8(stamp.fault().unwrap()).unwrap();
        assert_eq!(reason, "hardfault pc=0800a1b2");
        assert_eq!(stamp.fault_streak(), 2);
    }

    // Without a location -- the default composition, which keeps every panic site's file
    // string out of the image -- the kind is still recorded and still overwrites the last one.
    assert!(latch_panic(None).resets());
    {
        let stamp = reboot(slot);
        assert_eq!(stamp.fault().unwrap(), b"panic");
    }

    // Faults that keep recurring stop asking for a reset, so a probe that faults every boot
    // ends up halted (the older behavior) rather than cycling on the bus forever.
    loop {
        if reboot(slot).fault_streak() >= LIMIT {
            break;
        }
        assert!(latch_panic(None).resets(), "up to the limit, still a reset");
    }
    assert!(!latch_panic(None).resets(), "past the limit");
    {
        let stamp = reboot(slot);
        assert_eq!(stamp.fault_streak(), LIMIT + 1);
    }

    // A boot that reaches steady state forgives the streak, so the next fault is recovered
    // from again -- the bound is on repeated faults, not on a probe's lifetime.
    clear_fault_streak();
    {
        let stamp = reboot(slot);
        assert_eq!(stamp.fault_streak(), 0);
        assert!(
            stamp.fault().is_some(),
            "the reason itself outlives the streak, so the host still sees it"
        );
    }
    assert!(latch_panic(None).resets());
}
