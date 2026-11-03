//! DWT event ring: halt all writers for dumps; critical sections must exclude all writers.

/// Reader's `read_command` returned a packet; arg is its length.
pub const EV_READ_RET: u8 = 1;
/// Reader's `read_command` failed; the host went away.
pub const EV_READ_ERR: u8 = 2;
/// `submit` enqueued the packet; arg is its length.
pub const EV_SUBMIT_ENQ: u8 = 3;
/// `submit` intercepted or dropped the packet; arg is its length.
pub const EV_SUBMIT_DROP: u8 = 4;
/// The queue was full and the reader is about to await `slot_freed`; aux bit 0 if already raised.
pub const EV_WAIT_SLOT: u8 = 5;
/// `execute_next_with` entered the closure; aux is the ordinal, arg the request's command byte.
pub const EV_EXEC_ENTER: u8 = 6;
/// `execute_next_with` left the closure; aux is the ordinal, arg the response length.
pub const EV_EXEC_EXIT: u8 = 7;
/// `execute_next_with` found nothing to run.
pub const EV_EXEC_IDLE: u8 = 8;
/// The executor is about to await `request_available`.
pub const EV_WAIT_REQ: u8 = 9;
/// `write_response` entered; arg is the response length.
pub const EV_WRITE_ENTER: u8 = 10;
/// `write_response` returned; arg is the response length.
pub const EV_WRITE_RET: u8 = 11;
/// `write_response` failed; the host went away.
pub const EV_WRITE_ERR: u8 = 12;
/// The response ring was full and the executor is about to await `response_slot_freed`.
pub const EV_WAIT_RESP_SLOT: u8 = 13;
/// USB CTR interrupt for an OUT transaction; aux is the endpoint, arg the `USB_FNR` low half.
pub const EV_CTR_OUT: u8 = 16;
/// USB CTR interrupt for an IN transaction; aux is the endpoint, arg the `USB_FNR` low half.
pub const EV_CTR_IN: u8 = 17;
/// A start of frame was seen; arg is the `USB_FNR` low half. One per millisecond of bus time.
pub const EV_SOF: u8 = 18;
/// The reader's `slot_freed` await returned; the span from [`EV_WAIT_SLOT`] is the suspension.
pub const EV_RESUME_SLOT: u8 = 19;
/// The executor's `request_available` await returned.
pub const EV_RESUME_REQ: u8 = 20;
/// The executor's `response_slot_freed` await returned.
pub const EV_RESUME_RESP_SLOT: u8 = 21;
/// First code a firmware may use for its own marks.
pub const EV_USER: u8 = 128;

/// Aux bit set beside an ordinal when the ring the site is about to touch is full.
pub const AUX_FULL: u8 = 0x80;
/// Aux bit set on a wait mark when the signal was already raised, so nothing suspends.
pub const AUX_READY: u8 = 1;

/// Record `code` with `arg` and the current cycle count, if the ring has room.
#[inline(always)]
pub fn mark(code: u8, arg: u16) {
    mark_aux(code, 0, arg);
}

/// Record `code` with an ordinal or endpoint in `aux` beside `arg`.
#[inline(always)]
pub fn mark_aux(code: u8, aux: u8, arg: u16) {
    #[cfg(feature = "dap-trace")]
    imp::mark(code, aux, arg);
    #[cfg(not(feature = "dap-trace"))]
    let _ = (code, aux, arg);
}

/// Combine an ordinal with the ring-full flag; `full` is only called when tracing is on.
#[inline(always)]
pub fn aux_full(ordinal: u8, full: impl FnOnce() -> bool) -> u8 {
    #[cfg(feature = "dap-trace")]
    return ordinal | if full() { AUX_FULL } else { 0 };
    #[cfg(not(feature = "dap-trace"))]
    {
        let _ = (ordinal, full);
        0
    }
}

/// The aux byte of a wait mark: set when the signal was already raised, so nothing suspends.
#[inline(always)]
pub fn aux_ready(signalled: impl FnOnce() -> bool) -> u8 {
    #[cfg(feature = "dap-trace")]
    return if signalled() { AUX_READY } else { 0 };
    #[cfg(not(feature = "dap-trace"))]
    {
        let _ = signalled;
        0
    }
}

/// The ordinal of the command the reader is enqueuing now.
#[inline(always)]
pub fn next_submit_ordinal() -> u8 {
    #[cfg(feature = "dap-trace")]
    return imp::next_ordinal(0);
    #[cfg(not(feature = "dap-trace"))]
    0
}

/// The ordinal of the command the executor is about to run.
#[inline(always)]
pub fn next_exec_ordinal() -> u8 {
    #[cfg(feature = "dap-trace")]
    return imp::next_ordinal(1);
    #[cfg(not(feature = "dap-trace"))]
    0
}

/// The ordinal of the response the writer is about to stage.
#[inline(always)]
pub fn next_stage_ordinal() -> u8 {
    #[cfg(feature = "dap-trace")]
    return imp::next_ordinal(2);
    #[cfg(not(feature = "dap-trace"))]
    0
}

/// Empty the ring so the next run captures a fresh window.
#[inline(always)]
pub fn reset() {
    #[cfg(feature = "dap-trace")]
    imp::reset();
}

/// Start the DWT cycle counter; call once during firmware init.
#[inline(always)]
pub fn enable_cyccnt() {
    #[cfg(feature = "dap-trace")]
    imp::enable_cyccnt();
}

#[cfg(feature = "dap-trace")]
pub use imp::{TraceEntry, TraceRing, DAP_TRACE, FORMAT_VERSION, TRACE_CAPACITY, TRACE_MAGIC};

#[cfg(feature = "dap-trace")]
mod imp {
    use core::cell::UnsafeCell;
    use core::sync::atomic::Ordering;

    use portable_atomic::AtomicU32;

    /// Entries in the ring.
    pub const TRACE_CAPACITY: usize = 2048;
    /// Header word 0, identifying an initialized dump.
    pub const TRACE_MAGIC: u32 = 0x4441_5054;
    /// Header word 3, bit 0: at least one mark was dropped because the ring was full.
    const FLAG_FROZEN: u32 = 1;
    /// Header word 3, bits 8..16: the entry layout, so a decoder can read older dumps too.
    pub const FORMAT_VERSION: u32 = 2;

    /// Ordinals wrap here; the window that matters holds a handful of commands in flight.
    const ORDINAL_MODULO: u32 = 128;

    /// One event: the packed code and argument, and the DWT cycle count when it was taken.
    #[derive(Clone, Copy)]
    #[repr(C)]
    pub struct TraceEntry {
        /// `code` in bits 0..8, `aux` in bits 8..16, `arg` in bits 16..32.
        pub event: u32,
        pub cycles: u32,
    }

    /// The dumped layout: a four-word header followed by `TRACE_CAPACITY` two-word entries.
    #[repr(C)]
    pub struct TraceRing {
        magic: AtomicU32,
        capacity: AtomicU32,
        count: AtomicU32,
        flags: AtomicU32,
        entries: UnsafeCell<[TraceEntry; TRACE_CAPACITY]>,
    }

    // SAFETY: writes and resets share a critical section; no references to entries escape.
    unsafe impl Sync for TraceRing {}

    /// Dump over SWD only with all writers halted; header words are stamped at runtime.
    #[no_mangle]
    #[used]
    pub static DAP_TRACE: TraceRing = TraceRing::new();

    impl TraceRing {
        const fn new() -> Self {
            Self {
                magic: AtomicU32::new(0),
                capacity: AtomicU32::new(0),
                count: AtomicU32::new(0),
                flags: AtomicU32::new(0),
                entries: UnsafeCell::new(
                    [TraceEntry {
                        event: 0,
                        cycles: 0,
                    }; TRACE_CAPACITY],
                ),
            }
        }

        #[inline(always)]
        fn mark(&self, code: u8, aux: u8, arg: u16) {
            self.record(|| TraceEntry {
                event: code as u32 | ((aux as u32) << 8) | ((arg as u32) << 16),
                cycles: cyccnt(),
            });
        }

        #[inline(always)]
        fn record(&self, entry: impl FnOnce() -> TraceEntry) {
            critical_section::with(|_| {
                let cur = self.count.load(Ordering::Relaxed);
                if cur as usize >= TRACE_CAPACITY {
                    self.flags.fetch_or(FLAG_FROZEN, Ordering::Relaxed);
                    return;
                }
                let entry = entry();
                // SAFETY: the critical section excludes every other writer and reset.
                unsafe {
                    self.entries
                        .get()
                        .cast::<TraceEntry>()
                        .add(cur as usize)
                        .write(entry)
                };
                self.count.store(cur + 1, Ordering::Release);
            });
        }

        fn reset(&self, ordinals: &[AtomicU32; 3]) {
            critical_section::with(|_| {
                self.magic.store(TRACE_MAGIC, Ordering::Relaxed);
                self.capacity
                    .store(TRACE_CAPACITY as u32, Ordering::Relaxed);
                self.flags.store(FORMAT_VERSION << 8, Ordering::Relaxed);
                for counter in ordinals {
                    counter.store(0, Ordering::Relaxed);
                }
                self.count.store(0, Ordering::Release);
            });
        }
    }

    const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
    const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
    const DWT_CYCCNT: *mut u32 = 0xE000_1004 as *mut u32;
    const DWT_LAR: *mut u32 = 0xE000_1FB0 as *mut u32;

    #[cfg(target_arch = "arm")]
    #[inline(always)]
    fn cyccnt() -> u32 {
        // SAFETY: an architecturally fixed read-only-here debug register.
        unsafe { core::ptr::read_volatile(DWT_CYCCNT) }
    }

    /// Off-target builds have no DWT, so the host test ring counts marks instead of cycles.
    #[cfg(not(target_arch = "arm"))]
    #[inline(always)]
    fn cyccnt() -> u32 {
        static FAKE: AtomicU32 = AtomicU32::new(0);
        FAKE.fetch_add(1, Ordering::Relaxed)
    }

    /// The three ordinal counters, indexed submit, execute, stage.
    static ORDINALS: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];

    #[inline(always)]
    pub fn next_ordinal(which: usize) -> u8 {
        critical_section::with(|_| {
            (ORDINALS[which].fetch_add(1, Ordering::Relaxed) % ORDINAL_MODULO) as u8
        })
    }

    #[inline(always)]
    pub fn mark(code: u8, aux: u8, arg: u16) {
        DAP_TRACE.mark(code, aux, arg);
    }

    pub fn reset() {
        DAP_TRACE.reset(&ORDINALS);
    }

    #[cfg(target_arch = "arm")]
    pub fn enable_cyccnt() {
        reset();
        // SAFETY: architecturally fixed debug registers, written once before anything marks.
        unsafe {
            core::ptr::write_volatile(DEMCR, core::ptr::read_volatile(DEMCR) | 1 << 24);
            // Armv8-M gates the DWT behind a software lock when one is implemented.
            core::ptr::write_volatile(DWT_LAR, 0xC5AC_CE55);
            core::ptr::write_volatile(DWT_CYCCNT, 0);
            core::ptr::write_volatile(DWT_CTRL, core::ptr::read_volatile(DWT_CTRL) | 1);
        }
    }

    #[cfg(not(target_arch = "arm"))]
    pub fn enable_cyccnt() {
        let _ = (DEMCR, DWT_CTRL, DWT_CYCCNT, DWT_LAR);
        reset();
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use std::sync::{mpsc, Arc};
        use std::thread;
        use std::time::Duration;

        fn ordinals() -> [AtomicU32; 3] {
            [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)]
        }

        fn entry(ring: &TraceRing, index: usize) -> TraceEntry {
            assert!(index < TRACE_CAPACITY);
            critical_section::with(|_| {
                // SAFETY: the fixture reads an initialized entry while all writers are excluded.
                unsafe { ring.entries.get().cast::<TraceEntry>().add(index).read() }
            })
        }

        #[test]
        fn dump_layout_is_unchanged() {
            assert_eq!(core::mem::offset_of!(TraceRing, magic), 0);
            assert_eq!(core::mem::offset_of!(TraceRing, capacity), 4);
            assert_eq!(core::mem::offset_of!(TraceRing, count), 8);
            assert_eq!(core::mem::offset_of!(TraceRing, flags), 12);
            assert_eq!(core::mem::offset_of!(TraceRing, entries), 16);
            assert_eq!(core::mem::size_of::<TraceEntry>(), 8);
            assert_eq!(core::mem::offset_of!(TraceEntry, cycles), 4);
            assert_eq!(core::mem::size_of::<TraceRing>(), 16 + TRACE_CAPACITY * 8);
            assert_eq!(core::mem::align_of::<TraceRing>(), 4);
            assert_eq!(TRACE_CAPACITY, 2048);
            assert_eq!(TRACE_MAGIC, 0x4441_5054);
            assert_eq!(FORMAT_VERSION, 2);
        }

        #[test]
        fn ring_fills_then_freezes() {
            let ring = TraceRing::new();
            ring.reset(&ordinals());
            for _ in 0..TRACE_CAPACITY + 8 {
                ring.mark(crate::trace::EV_READ_RET, 0, 64);
            }
            assert_eq!(ring.magic.load(Ordering::Relaxed), TRACE_MAGIC);
            assert_eq!(
                ring.capacity.load(Ordering::Relaxed) as usize,
                TRACE_CAPACITY
            );
            assert_eq!(ring.count.load(Ordering::Acquire) as usize, TRACE_CAPACITY);
            let flags = ring.flags.load(Ordering::Relaxed);
            assert_eq!(flags & FLAG_FROZEN, FLAG_FROZEN);
            assert_eq!(flags >> 8, FORMAT_VERSION);
            assert_eq!(entry(&ring, 0).event, 1 | 64 << 16);
            assert_eq!(entry(&ring, TRACE_CAPACITY - 1).event, 1 | 64 << 16);
            ring.record(|| panic!("a frozen ring must not construct an entry"));
        }

        #[test]
        fn count_commits_only_completed_entries() {
            let ring = TraceRing::new();
            ring.reset(&ordinals());
            for index in 0..4 {
                ring.record(|| {
                    assert_eq!(ring.count.load(Ordering::Acquire), index);
                    TraceEntry {
                        event: index + 1,
                        cycles: index + 10,
                    }
                });
                assert_eq!(ring.count.load(Ordering::Acquire), index + 1);
                let completed = entry(&ring, index as usize);
                assert_eq!(completed.event, index + 1);
                assert_eq!(completed.cycles, index + 10);
            }
        }

        #[test]
        fn reset_clears_freeze_count_and_ordinals_without_erasing_entries() {
            let ring = TraceRing::new();
            let ordinals = [AtomicU32::new(7), AtomicU32::new(8), AtomicU32::new(9)];
            for _ in 0..TRACE_CAPACITY + 1 {
                ring.mark(5, 6, 7);
            }
            ring.reset(&ordinals);
            assert_eq!(ring.count.load(Ordering::Acquire), 0);
            assert_eq!(ring.flags.load(Ordering::Relaxed), FORMAT_VERSION << 8);
            assert!(ordinals
                .iter()
                .all(|counter| counter.load(Ordering::Relaxed) == 0));
            assert_eq!(entry(&ring, 0).event, 5 | 6 << 8 | 7 << 16);
            ring.mark(8, 9, 10);
            assert_eq!(ring.count.load(Ordering::Acquire), 1);
            assert_eq!(entry(&ring, 0).event, 8 | 9 << 8 | 10 << 16);
        }

        #[test]
        fn reset_waits_for_reserved_entry_write_and_commit() {
            let ring = Arc::new(TraceRing::new());
            ring.reset(&ordinals());
            let (reserved_tx, reserved_rx) = mpsc::channel();
            let (continue_tx, continue_rx) = mpsc::channel();
            let (reset_start_tx, reset_start_rx) = mpsc::channel();
            let (reset_done_tx, reset_done_rx) = mpsc::channel();
            let writer_ring = Arc::clone(&ring);
            let writer = thread::spawn(move || {
                writer_ring.record(|| {
                    reserved_tx.send(()).expect("announce reservation");
                    continue_rx.recv().expect("release writer");
                    TraceEntry {
                        event: 7,
                        cycles: 8,
                    }
                });
            });
            reserved_rx.recv().expect("writer reserved a slot");
            let reset_ring = Arc::clone(&ring);
            let resetter = thread::spawn(move || {
                reset_start_tx.send(()).expect("announce reset");
                reset_ring.reset(&ordinals());
                reset_done_tx.send(()).expect("announce completed reset");
            });
            reset_start_rx.recv().expect("reset thread started");
            let premature = reset_done_rx.recv_timeout(Duration::from_millis(50));
            continue_tx.send(()).expect("release reserved writer");
            writer.join().expect("writer completed");
            resetter.join().expect("reset completed");
            assert_eq!(premature, Err(mpsc::RecvTimeoutError::Timeout));
            assert_eq!(ring.count.load(Ordering::Acquire), 0);
            ring.mark(9, 10, 11);
            assert_eq!(ring.count.load(Ordering::Acquire), 1);
            assert_eq!(entry(&ring, 0).event, 9 | 10 << 8 | 11 << 16);
        }

        #[test]
        fn concurrent_writers_commit_distinct_slots() {
            let ring = Arc::new(TraceRing::new());
            ring.reset(&ordinals());
            thread::scope(|scope| {
                for code in 0..8 {
                    let ring = &ring;
                    scope.spawn(move || {
                        for arg in 0..16 {
                            ring.mark(code, 0x81, arg);
                        }
                    });
                }
            });
            assert_eq!(ring.count.load(Ordering::Acquire), 128);
            let mut seen = [[false; 16]; 8];
            for index in 0..128 {
                let event = entry(&ring, index).event;
                assert_eq!((event >> 8) & 0xff, 0x81);
                let seen = &mut seen[(event & 0xff) as usize][(event >> 16) as usize];
                assert!(!*seen);
                *seen = true;
            }
            assert!(seen.iter().flatten().all(|seen| *seen));
        }

        // The pipeline tests share these counters, so only the aux-byte bound is asserted.
        #[test]
        fn ordinals_stay_within_the_aux_byte() {
            for which in 0..ORDINALS.len() {
                for _ in 0..2 * ORDINAL_MODULO {
                    assert!(u32::from(next_ordinal(which)) < ORDINAL_MODULO);
                }
            }
        }
    }
}

#[cfg(all(test, not(feature = "dap-trace")))]
mod tests {
    #[test]
    fn disabled_trace_does_not_evaluate_aux_inputs() {
        assert_eq!(super::aux_full(7, || panic!("trace is disabled")), 0);
        assert_eq!(super::aux_ready(|| panic!("trace is disabled")), 0);
        assert_eq!(super::next_submit_ordinal(), 0);
        assert_eq!(super::next_exec_ordinal(), 0);
        assert_eq!(super::next_stage_ordinal(), 0);
        super::mark_aux(1, 2, 3);
        super::reset();
    }
}
