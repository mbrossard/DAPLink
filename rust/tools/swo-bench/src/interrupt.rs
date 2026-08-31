//! SIGINT latch: the sweep aborts at the next poll batch and the DUT teardown still runs.

use std::sync::atomic::{AtomicBool, Ordering};

static TRIGGERED: AtomicBool = AtomicBool::new(false);

/// True once an interrupt arrived; the capture and tier loops poll this.
pub fn triggered() -> bool {
    TRIGGERED.load(Ordering::Relaxed)
}

#[cfg(unix)]
extern "C" fn latch(_: libc::c_int) {
    if TRIGGERED.swap(true, Ordering::Relaxed) {
        // A second interrupt means the teardown itself is stuck; die immediately.
        unsafe { libc::_exit(130) };
    }
}

/// Install the latch. Without it SIGINT kills the process mid-sweep with the DUT still streaming.
pub fn install() {
    #[cfg(unix)]
    unsafe {
        libc::signal(libc::SIGINT, latch as *const () as libc::sighandler_t);
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;

    #[test]
    fn a_first_interrupt_latches_instead_of_killing() {
        install();
        assert!(!triggered());
        // raise() delivers synchronously on this thread; a dead latch would kill the harness.
        unsafe { libc::raise(libc::SIGINT) };
        assert!(triggered());
    }
}
