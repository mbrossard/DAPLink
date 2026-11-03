extern crate std;

use super::*;
use std::{boxed::Box, cell::RefCell, vec::Vec};

#[derive(Default)]
struct Fake {
    busy_reads: u32,
    clear_at: Option<u32>,
    channel: usize,
    log: Vec<(usize, u32)>,
}

std::thread_local! {
    static STATE: RefCell<Fake> = RefCell::new(Fake::default());
    static MEMORY: *mut DmaMem = Box::into_raw(Box::new(unsafe { core::mem::zeroed() }));
    static REGS: *mut lpc55_hal::raw::dma0::RegisterBlock = Box::into_raw(Box::new(unsafe { core::mem::zeroed() }));
    static USART: *mut lpc55_hal::raw::usart0::RegisterBlock = Box::into_raw(Box::new(unsafe { core::mem::zeroed() }));
}

pub(super) fn mem() -> *mut DmaMem {
    MEMORY.with(|p| *p)
}
pub(super) fn regs() -> &'static lpc55_hal::raw::dma0::RegisterBlock {
    REGS.with(|p| unsafe { &**p })
}
pub(crate) fn usart() -> &'static lpc55_hal::raw::usart0::RegisterBlock {
    USART.with(|p| unsafe { &**p })
}

pub(crate) fn script(channel: usize, clear_at: Option<u32>) {
    STATE.with(|s| {
        *s.borrow_mut() = Fake {
            channel,
            clear_at,
            ..Fake::default()
        }
    });
}

pub(super) fn stop_busy() -> u32 {
    STATE.with(|s| {
        let mut s = s.borrow_mut();
        s.busy_reads += 1;
        if s.clear_at.is_some_and(|n| s.busy_reads >= n) {
            0
        } else {
            1 << s.channel
        }
    })
}

pub(super) fn stop_write(offset: usize, value: u32) {
    STATE.with(|s| s.borrow_mut().log.push((offset, value)));
}

struct StopFailed;

pub(super) fn stop_failed() -> ! {
    std::panic::panic_any(StopFailed)
}
pub(crate) fn arm(channel: usize) {
    stop_write(usize::MAX, channel as u32);
}

pub(crate) fn failed(f: impl FnOnce()) {
    let err = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
        .expect_err("stop must not return");
    assert!(err.is::<StopFailed>());
}

pub(crate) fn assert_unreleased(channel: usize) {
    STATE.with(|s| {
        let s = s.borrow();
        assert_eq!(s.busy_reads, BUSY_SPIN_LIMIT);
        assert_eq!(s.log, [(0x028, 1 << channel)]);
    });
}

#[test]
fn checked_stop_and_all_channel_owners_preserve_unreleased_memory() {
    for channel in [4, 5, 8] {
        for clear_at in [1, BUSY_SPIN_LIMIT / 2, BUSY_SPIN_LIMIT] {
            script(channel, Some(clear_at));
            assert_eq!(try_stop(channel), Ok(()));
            STATE.with(|s| {
                let s = s.borrow();
                assert_eq!(s.busy_reads, clear_at);
                assert_eq!(
                    s.log,
                    [
                        (0x028, 1 << channel),
                        (0x078, 1 << channel),
                        (0x058, 1 << channel)
                    ]
                );
            });
        }
        script(channel, Some(BUSY_SPIN_LIMIT + 1));
        failed(|| stop(channel));
        assert_unreleased(channel);
    }
    crate::dma_rx::stop_test();
    crate::dma_tx::stop_test();
    crate::dma_swo::stop_test();
}
