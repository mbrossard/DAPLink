use core::cell::Cell;

struct Epoch {
    blocks: Cell<u64>,
    previous: Cell<u64>,
}

impl Epoch {
    const fn new() -> Self {
        Self {
            blocks: Cell::new(0),
            previous: Cell::new(0),
        }
    }
}

#[cfg(not(test))]
fn with_epoch<R>(body: impl FnOnce(&Epoch) -> R) -> R {
    use cortex_m::interrupt::{free, Mutex};
    static EPOCH: Mutex<Epoch> = Mutex::new(Epoch::new());
    // Only core 0's ordinary interrupts access this epoch; NMI and faults never do.
    free(|cs| body(EPOCH.borrow(cs)))
}

#[cfg(test)]
fn with_epoch<R>(body: impl FnOnce(&Epoch) -> R) -> R {
    extern crate std;
    std::thread_local! { static EPOCH: Epoch = const { Epoch::new() }; }
    EPOCH.with(body)
}

pub(super) fn blocks() -> u64 {
    with_epoch(|epoch| epoch.blocks.get())
}

pub(super) fn previous() -> u64 {
    with_epoch(|epoch| epoch.previous.get())
}

pub(super) fn publish(position: u64) {
    with_epoch(|epoch| epoch.previous.set(position));
}

pub(super) fn complete() {
    with_epoch(|epoch| epoch.blocks.set(epoch.blocks.get().wrapping_add(1)));
}

// The stopped channel's USB owner resets both words before arming the next capture.
pub(super) fn reset() {
    with_epoch(|epoch| {
        epoch.blocks.set(0);
        epoch.previous.set(0);
    });
}

#[cfg(test)]
pub(super) fn seed(blocks: u64, previous: u64) {
    with_epoch(|epoch| {
        epoch.blocks.set(blocks);
        epoch.previous.set(previous);
    });
}
