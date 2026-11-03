//! Checked GPDMA suspension and release.

use core::sync::atomic::{fence, AtomicU32, Ordering};
use embassy_stm32::pac::gpdma::Channel;

const SPINS: u32 = 10_000;

#[derive(Clone, Copy, Debug)]
struct State {
    enabled: bool,
    idle: bool,
    suspended: bool,
}

fn try_suspend(ch: Channel) -> Result<(), ()> {
    request_suspend(ch);
    for _ in 0..SPINS {
        let state = sample(ch);
        if state.idle && (!state.enabled || state.suspended) {
            return Ok(());
        }
    }
    Err(())
}

fn try_halt(ch: Channel) -> Result<(), ()> {
    try_suspend(ch)?;
    reset(ch);
    for _ in 0..SPINS {
        let state = sample(ch);
        if !state.enabled && state.idle {
            fence(Ordering::SeqCst);
            return Ok(());
        }
    }
    Err(())
}

pub(crate) fn suspend(ch: Channel, faults: &AtomicU32) {
    if try_suspend(ch).is_err() {
        faults.fetch_add(1, Ordering::Relaxed);
        failed();
    }
}

pub(crate) fn halt(ch: Channel, faults: &AtomicU32) {
    if try_halt(ch).is_err() {
        faults.fetch_add(1, Ordering::Relaxed);
        failed();
    }
}

pub(crate) fn masked<T>(f: impl FnOnce() -> T) -> T {
    #[cfg(not(test))]
    {
        cortex_m::interrupt::free(|_| f())
    }
    #[cfg(test)]
    {
        f()
    }
}

#[cfg(not(test))]
fn sample(ch: Channel) -> State {
    let sr = ch.sr().read();
    State {
        enabled: ch.cr().read().en(),
        idle: sr.idlef(),
        suspended: sr.suspf(),
    }
}

#[cfg(not(test))]
fn request_suspend(ch: Channel) {
    if ch.cr().read().en() {
        ch.cr().modify(|w| w.set_susp(true));
    }
}

#[cfg(not(test))]
fn reset(ch: Channel) {
    ch.cr().write(|w| w.set_reset(true));
}

#[cfg(not(test))]
fn failed() -> ! {
    cortex_m::interrupt::disable();
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
use tests::{failed, request_suspend, reset, sample};

#[cfg(test)]
pub(crate) mod tests;
