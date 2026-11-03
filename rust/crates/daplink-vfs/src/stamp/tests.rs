use super::*;
use std::panic::{catch_unwind, AssertUnwindSafe};

fn stop_boot() {
    panic!("terminal boot boundary");
}

fn bytes(status_len: u32, fault_len: u32) -> [u8; RETAINED_SIZE] {
    let mut bytes = [b'A'; RETAINED_SIZE];
    bytes[..4].copy_from_slice(&MAGIC.to_le_bytes());
    bytes[4..8].copy_from_slice(&u32::MAX.to_le_bytes());
    bytes[8..12].copy_from_slice(&9u32.to_le_bytes());
    bytes[12..16].copy_from_slice(&status_len.to_le_bytes());
    bytes[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
    bytes[84..88].copy_from_slice(&fault_len.to_le_bytes());
    bytes
}

fn record(endpoint: FaultEndpoint<'_>, reason: Reason) -> FaultAction {
    let recording = AtomicBool::new(false);
    // SAFETY: fixture writes are single-threaded, disjoint, and no live record reference exists.
    unsafe { terminal(endpoint.0.ptr, &recording, |dst| *dst = reason, &mut |_| {}) }
}

#[test]
fn invalid_magic_initializes_every_byte() {
    for fill in [0, 0xff, 0x64] {
        let mut fixture = InitializedStamp::from_bytes([fill; RETAINED_SIZE]);
        let boot = fixture.boot();
        assert_eq!(boot.snapshot.reset_count(), 0);
        assert_eq!(boot.snapshot.fault_streak(), 0);
        assert_eq!(boot.snapshot.status(), None);
        assert_eq!(boot.snapshot.fault(), None);
        assert_eq!(boot.snapshot.0.magic, MAGIC);
        assert_eq!(boot.snapshot.0.failed, 0);
        assert_eq!(boot.snapshot.0.status, [0; STATUS_MAX]);
        assert_eq!(boot.snapshot.0.fault, [0; FAULT_MAX]);
    }
}

#[test]
fn v1_fixture_validates_fields_independently_and_wraps_count() {
    for status in [0, STATUS_MAX as u32, STATUS_MAX as u32 + 1, u32::MAX] {
        for fault in [0, FAULT_MAX as u32, FAULT_MAX as u32 + 1, u32::MAX] {
            let mut fixture = InitializedStamp::from_bytes(bytes(status, fault));
            let boot = fixture.boot();
            assert_eq!(boot.snapshot.reset_count(), 0);
            assert_eq!(boot.snapshot.fault_streak(), u32::MAX);
            assert_eq!(boot.snapshot.0.failed, 1);
            assert_eq!(
                boot.snapshot.status().map(|s| s.0.len()),
                (status == STATUS_MAX as u32).then_some(STATUS_MAX)
            );
            assert_eq!(
                boot.snapshot.fault().map(|s| s.len()),
                (fault == FAULT_MAX as u32).then_some(FAULT_MAX)
            );
        }
    }
}

#[test]
fn snapshots_are_independent_and_steady_only_clears_streak() {
    let mut fixture = InitializedStamp::zeroed();
    {
        let mut boot = fixture.boot();
        boot.status.store(b"old status", true);
        assert_eq!(record(boot.fault, Reason::panic(None)), FaultAction::Reset);
    }
    let saved = {
        let mut boot = fixture.boot();
        let old_status = boot.snapshot.status().unwrap();
        let old_fault = boot.snapshot.fault().unwrap();
        boot.status.store(b"new status", false);
        assert_eq!(
            record(boot.fault, Reason::hard_fault(0x0800_a1b2)),
            FaultAction::Reset
        );
        boot.steady.reached();
        assert_eq!(old_status, (&b"old status"[..], true));
        assert_eq!(old_fault, b"panic");
        boot.snapshot
    };
    let boot = fixture.boot();
    assert_eq!(boot.snapshot.reset_count(), 2);
    assert_eq!(boot.snapshot.status(), Some((&b"new status"[..], false)));
    assert_eq!(boot.snapshot.fault(), Some(&b"hardfault pc=0800a1b2"[..]));
    assert_eq!(boot.snapshot.fault_streak(), 0);
    assert_eq!(saved.fault(), Some(&b"panic"[..]));
}

#[test]
fn fault_threshold_and_saturation_are_preserved() {
    let mut fixture = InitializedStamp::zeroed();
    for n in 1..=FAULT_RESET_LIMIT + 1 {
        let boot = fixture.boot();
        assert_eq!(boot.snapshot.fault_streak(), n - 1);
        assert_eq!(
            record(boot.fault, Reason::panic(None)).resets(),
            n <= FAULT_RESET_LIMIT
        );
    }
    let mut fixture = InitializedStamp::from_bytes(bytes(0, 0));
    assert_eq!(
        record(fixture.boot().fault, Reason::panic(None)),
        FaultAction::Halt
    );
    assert_eq!(fixture.boot().snapshot.fault_streak(), u32::MAX);
}

#[test]
fn status_truncation_and_empty_commits() {
    let mut fixture = InitializedStamp::zeroed();
    fixture.boot().status.store(&[b'x'; 100], true);
    {
        let mut boot = fixture.boot();
        assert_eq!(
            boot.snapshot.status(),
            Some((&[b'x'; STATUS_MAX][..], true))
        );
        boot.status.store(b"", false);
    }
    assert_eq!(fixture.boot().snapshot.status(), None);
}

#[test]
fn every_status_store_interruption_advertises_complete_or_absent_text() {
    let new = [b'B'; STATUS_MAX];
    for stop in 1..=STATUS_MAX + 3 {
        let mut fixture = InitializedStamp::from_bytes(bytes(STATUS_MAX as u32, FAULT_MAX as u32));
        {
            let boot = fixture.boot();
            let mut n = 0;
            let result = catch_unwind(AssertUnwindSafe(|| unsafe {
                write_status(boot.status.region.ptr, &new, false, &mut || {
                    n += 1;
                    if n == stop {
                        panic!("reset");
                    }
                });
            }));
            assert!(result.is_err());
        }
        let boot = fixture.boot();
        let expected = (stop == STATUS_MAX + 3).then_some((&new[..], false));
        assert_eq!(boot.snapshot.status(), expected);
        assert_eq!(boot.snapshot.fault(), Some(&[b'A'; FAULT_MAX][..]));
    }
}

#[test]
fn fault_interruption_and_nested_entry_never_splice_or_resume() {
    let total = FAULT_MAX + 6;
    for nested in [false, true] {
        for stop in 1..=total {
            let mut fixture =
                InitializedStamp::from_bytes(bytes(STATUS_MAX as u32, FAULT_MAX as u32));
            let mut final_step = Step::BeforeClaim;
            {
                let boot = fixture.boot();
                let p = boot.fault.0.ptr;
                let recording = AtomicBool::new(false);
                let mut n = 0;
                let result = catch_unwind(AssertUnwindSafe(|| unsafe {
                    terminal(
                        p,
                        &recording,
                        |reason| reason.bytes(&[b'B'; FAULT_MAX]),
                        &mut |step| {
                            n += 1;
                            if n == stop {
                                final_step = step;
                                if nested {
                                    let result = terminal(
                                        p,
                                        &recording,
                                        |reason| reason.write_panic(None),
                                        &mut |_| {},
                                    );
                                    assert_eq!(result, FaultAction::Halt);
                                }
                                panic!("terminal interruption");
                            }
                        },
                    );
                }));
                assert!(result.is_err());
                assert_eq!(n, stop, "the outer writer must not resume");
            }
            let boot = fixture.boot();
            let expected = match final_step {
                Step::BeforeClaim | Step::Unclaimed if nested => Some(&b"panic"[..]),
                Step::BeforeClaim | Step::Unclaimed | Step::Claimed | Step::Streak => {
                    Some(&[b'A'; FAULT_MAX][..])
                }
                Step::Committed => Some(&[b'B'; FAULT_MAX][..]),
                Step::Invalidated | Step::Byte => None,
            };
            assert_eq!(
                boot.snapshot.fault(),
                expected,
                "nested={nested} stop={stop}"
            );
            assert_eq!(
                boot.snapshot.status(),
                Some((&[b'A'; STATUS_MAX][..], true))
            );
            assert_eq!(boot.snapshot.fault_streak(), u32::MAX);
        }
    }
}

#[test]
fn fault_during_status_write_preserves_disjoint_fields() {
    for stop in 1..=7 {
        let mut fixture = InitializedStamp::zeroed();
        {
            let boot = fixture.boot();
            boot.steady.reached();
            let mut n = 0;
            let result = catch_unwind(AssertUnwindSafe(|| unsafe {
                write_status(boot.status.region.ptr, b"done", false, &mut || {
                    n += 1;
                    if n == stop {
                        assert_eq!(
                            terminal(
                                boot.fault.0.ptr,
                                &AtomicBool::new(false),
                                |reason| reason.write_panic(None),
                                &mut |_| {}
                            ),
                            FaultAction::Reset
                        );
                        panic!("terminal fault");
                    }
                });
            }));
            assert!(result.is_err());
        }
        let boot = fixture.boot();
        assert_eq!(boot.snapshot.fault(), Some(&b"panic"[..]));
        assert_eq!(boot.snapshot.fault_streak(), 1);
        assert_eq!(
            boot.snapshot.status(),
            (stop == 7).then_some((&b"done"[..], false))
        );
    }
}

#[test]
fn busy_terminal_never_constructs_reason_or_writes_again() {
    let mut fixture = InitializedStamp::zeroed();
    {
        let boot = fixture.boot();
        let recording = AtomicBool::new(false);
        unsafe {
            assert_eq!(
                terminal(
                    boot.fault.0.ptr,
                    &recording,
                    |reason| reason.write_panic(None),
                    &mut |_| {}
                ),
                FaultAction::Reset
            );
            assert_eq!(
                terminal(
                    boot.fault.0.ptr,
                    &recording,
                    |_| panic!("busy must not format"),
                    &mut |_| {}
                ),
                FaultAction::Halt
            );
            assert_eq!(
                terminal(
                    ptr::null_mut(),
                    &recording,
                    |_| panic!("unarmed must not format"),
                    &mut |_| {}
                ),
                FaultAction::Reset
            );
        }
    }
    let boot = fixture.boot();
    assert_eq!(boot.snapshot.fault(), Some(&b"panic"[..]));
    assert_eq!(boot.snapshot.fault_streak(), 1);
}

#[test]
fn steady_clear_only_precedes_a_fault_when_normal_code_reaches_it() {
    for clear_first in [false, true] {
        let mut initial = bytes(0, 0);
        initial[80..84].copy_from_slice(&2u32.to_le_bytes());
        let mut fixture = InitializedStamp::from_bytes(initial);
        {
            let boot = fixture.boot();
            if clear_first {
                boot.steady.reached();
                assert!(record(boot.fault, Reason::panic(None)).resets());
            } else {
                let interrupted = catch_unwind(AssertUnwindSafe(|| {
                    assert!(record(boot.fault, Reason::panic(None)).resets());
                    stop_boot();
                    boot.steady.reached();
                }));
                assert!(interrupted.is_err());
            }
        }
        assert_eq!(
            fixture.boot().snapshot.fault_streak(),
            if clear_first { 1 } else { 3 }
        );
    }
}

#[test]
fn reason_formatting_preserves_tail_and_fixed_width() {
    let r = Reason::panic(Some((
        "/a/very/long/path/firmware/rp2040-pico-embassy/src/main.rs",
        412,
    )));
    let text = core::str::from_utf8(r.written()).unwrap();
    assert!(text.starts_with("panic "));
    assert!(text.ends_with("pico-embassy/src/main.rs:412"));
    assert!(r.len <= FAULT_MAX);
    assert_eq!(
        Reason::hard_fault(0x0800_a1b2).written(),
        b"hardfault pc=0800a1b2"
    );
    let mut digits = [0; 10];
    assert_eq!(dec(&mut digits, 0), b"0");
    assert_eq!(dec(&mut digits, u32::MAX), b"4294967295");
}
