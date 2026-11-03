extern crate std;

#[test]
fn checked_suspend_preserves_the_cursor_and_never_resets_a_busy_owner() {
    let dma = registers();
    let ch = dma.ch(0);
    let faults = AtomicU32::new(0);
    ch.br1().write(|w| w.set_bndt(123));
    script(ch, (true, false, false), Some(SPINS), false);
    suspend(ch, &faults);
    assert_eq!(reset_count(ch), 0);
    assert_eq!(ch.br1().read().bndt(), 123);
    assert_eq!(faults.load(Ordering::Relaxed), 0);

    script(ch, (true, false, false), None, false);
    trapped(|| suspend(ch, &faults));
    assert_eq!(reset_count(ch), 0);
    assert_eq!(ch.br1().read().bndt(), 123);
    assert_eq!(faults.load(Ordering::Relaxed), 1);
}

use super::*;
use embassy_stm32::pac;
use std::{boxed::Box, cell::RefCell, collections::BTreeMap, vec::Vec};

#[derive(Clone, Copy)]
struct Script {
    state: State,
    clear_at: Option<u32>,
    polls: u32,
    reset_clear_at: Option<u32>,
    reset_polls: u32,
    reset: bool,
}

#[derive(Default)]
struct Fake {
    channels: BTreeMap<usize, Script>,
    log: Vec<(usize, &'static str)>,
}

std::thread_local! { static MMIO: RefCell<Fake> = RefCell::new(Fake::default()); }

pub(crate) fn registers() -> pac::gpdma::Gpdma {
    let mem = Box::leak(Box::new([0u32; 1024]));
    unsafe { pac::gpdma::Gpdma::from_ptr(mem.as_mut_ptr().cast()) }
}

pub(crate) fn uart() -> pac::usart::Usart {
    let mem = Box::leak(Box::new([0u32; 64]));
    unsafe { pac::usart::Usart::from_ptr(mem.as_mut_ptr().cast()) }
}

pub(crate) fn script(
    ch: Channel,
    state: (bool, bool, bool),
    clear_at: Option<u32>,
    reset_stuck: bool,
) {
    MMIO.with(|m| {
        m.borrow_mut().channels.insert(
            ch.as_ptr() as usize,
            Script {
                state: State {
                    enabled: state.0,
                    idle: state.1,
                    suspended: state.2,
                },
                clear_at,
                polls: 0,
                reset_clear_at: (!reset_stuck).then_some(1),
                reset_polls: 0,
                reset: false,
            },
        );
    });
}

pub(super) fn sample(ch: Channel) -> State {
    MMIO.with(|m| {
        let mut m = m.borrow_mut();
        let s = m
            .channels
            .get_mut(&(ch.as_ptr() as usize))
            .expect("scripted channel");
        s.polls += 1;
        if s.reset {
            s.reset_polls += 1;
            let clear = s.reset_clear_at.is_some_and(|n| s.reset_polls >= n);
            State {
                enabled: !clear,
                idle: clear,
                suspended: false,
            }
        } else if s.clear_at.is_some_and(|n| s.polls >= n) {
            State {
                enabled: true,
                idle: true,
                suspended: true,
            }
        } else {
            s.state
        }
    })
}

pub(super) fn request_suspend(ch: Channel) {
    MMIO.with(|m| m.borrow_mut().log.push((ch.as_ptr() as usize, "suspend")));
}

pub(super) fn reset(ch: Channel) {
    MMIO.with(|m| {
        let mut m = m.borrow_mut();
        m.log.push((ch.as_ptr() as usize, "reset"));
        m.channels.get_mut(&(ch.as_ptr() as usize)).unwrap().reset = true;
    });
}

struct StopFailed;

pub(super) fn failed() -> ! {
    std::panic::panic_any(StopFailed)
}
pub(crate) fn trapped(f: impl FnOnce()) {
    let err = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
        .expect_err("stop must not return");
    assert!(err.is::<StopFailed>());
}
pub(crate) fn reset_count(ch: Channel) -> usize {
    MMIO.with(|m| {
        m.borrow()
            .log
            .iter()
            .filter(|&&(id, op)| id == ch.as_ptr() as usize && op == "reset")
            .count()
    })
}

#[test]
fn full_steady_predicate_and_reset_confirmation_gate_every_owner() {
    let dma = registers();
    for state in [
        (true, false, false),
        (true, false, true),
        (false, false, false),
        (true, true, false),
    ] {
        let ch = dma.ch(0);
        script(ch, state, None, false);
        let faults = AtomicU32::new(7);
        let resets = reset_count(ch);
        trapped(|| halt(ch, &faults));
        assert_eq!(faults.load(Ordering::Relaxed), 8);
        assert_eq!(reset_count(ch), resets);
    }
    for state in [(true, true, true), (false, true, false)] {
        let ch = dma.ch(1);
        script(ch, state, None, false);
        assert!(try_halt(ch).is_ok());
    }
    for clear in [1, SPINS / 2, SPINS] {
        script(dma.ch(2), (true, false, false), Some(clear), false);
        assert!(try_halt(dma.ch(2)).is_ok());
    }
    script(dma.ch(2), (true, false, false), Some(SPINS + 1), false);
    assert!(try_halt(dma.ch(2)).is_err());
    script(dma.ch(3), (true, true, true), None, true);
    let faults = AtomicU32::new(0);
    trapped(|| halt(dma.ch(3), &faults));
    assert_eq!(faults.load(Ordering::Relaxed), 1);
    assert_eq!(reset_count(dma.ch(3)), 1);
    for clear_at in [1, SPINS, SPINS + 1] {
        script(dma.ch(3), (true, true, true), None, false);
        MMIO.with(|m| {
            m.borrow_mut()
                .channels
                .get_mut(&(dma.ch(3).as_ptr() as usize))
                .unwrap()
                .reset_clear_at = Some(clear_at);
        });
        assert_eq!(try_halt(dma.ch(3)).is_ok(), clear_at <= SPINS);
        MMIO.with(|m| {
            assert_eq!(
                m.borrow().channels[&(dma.ch(3).as_ptr() as usize)].reset_polls,
                clear_at.min(SPINS)
            )
        });
    }
    crate::vcom::stop_test();
    crate::swo::stop_test();
}
