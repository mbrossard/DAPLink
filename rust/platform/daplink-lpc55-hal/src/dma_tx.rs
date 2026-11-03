//! Flexcomm0 transmit over DMA0 channel 5 from a byte ring, one transfer per contiguous run.

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use daplink_core::DmaTxCursor;
use lpc55_hal::raw;

use crate::dma::{self, TX_MAX_XFER, TX_RING_LEN};

/// Flexcomm0 transmit; receive is 4 and transmit is 5.
const CHANNEL: usize = 5;

/// Below the receive channels: a transmit can be asked to wait a transfer, a receive cannot.
const PRIORITY: u8 = 2;

/// Writer and controller cursors. Equal means empty, so one byte is always left unused.
static HEAD: AtomicUsize = AtomicUsize::new(0);
static TAIL: AtomicUsize = AtomicUsize::new(0);
/// Bytes reaped from completed transfers.
static SENT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// The wire floor [`sent`] last reported, so a transfer in flight cannot walk it backwards.
static WIRE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Latched channel errors, readable over SWD by symbol.
#[no_mangle]
pub static VCOM_TX_DMA_ERR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Bytes the controller is reading now, zero when idle; what `TAIL` advances by on completion.
static INFLIGHT: AtomicUsize = AtomicUsize::new(0);
/// Whether [`start`] has run: DMA0 is unclocked before it, and touching a channel register faults.
static STARTED: AtomicBool = AtomicBool::new(false);

/// Transfers armed and the bytes in them, readable over SWD.
#[cfg(feature = "probe-timing")]
#[no_mangle]
pub static PROBE_TX_ARMED: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "probe-timing")]
#[no_mangle]
pub static PROBE_TX_ARMED_BYTES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
/// Why an arming attempt declined: the controller was still sending, or there was nothing to send.
#[cfg(feature = "probe-timing")]
#[no_mangle]
pub static PROBE_KICK_BUSY: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "probe-timing")]
#[no_mangle]
pub static PROBE_KICK_EMPTY: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn usart() -> &'static raw::usart0::RegisterBlock {
    #[cfg(test)]
    {
        dma::stop_tests::usart()
    }
    #[cfg(not(test))]
    // SAFETY: USART0 is owned by the VCOM seam, this module's only caller.
    unsafe {
        &*raw::USART0::ptr()
    }
}

/// Ring arithmetic over the three cursors above, capped at what one descriptor covers.
type Cursor = DmaTxCursor<TX_RING_LEN, TX_MAX_XFER>;

/// Read the three cursors into a value the arithmetic works on. Callers hold the mask.
fn cursor() -> Cursor {
    Cursor::from_parts(
        HEAD.load(Ordering::Relaxed),
        TAIL.load(Ordering::Relaxed),
        INFLIGHT.load(Ordering::Relaxed),
    )
}

/// Hand the transmit FIFO to the controller. Called once, alongside the receive hand-over.
pub(crate) fn start() {
    // Shares FIFOCFG with the FLEXCOMM0 handler, which preempts this.
    dma::masked(|| {
        usart().fifocfg.modify(|_, w| w.dmatx().set_bit());
    });
    // Re-arm from the completion: waiting for the next USB pass idles the line at every boundary.
    dma::enable_int(CHANNEL);
    STARTED.store(true, Ordering::Relaxed);
}

/// Reap a finished transfer and arm the next; runs from DMA0 and the USB pass, so masked.
pub(crate) fn service() {
    if !STARTED.load(Ordering::Relaxed) {
        return;
    }
    dma::masked(|| {
        let mut reaped = false;
        // A faulted run is dropped rather than re-sent, which would duplicate bytes on the wire.
        if dma::take_error(CHANNEL) {
            VCOM_TX_DMA_ERR.fetch_add(1, Ordering::Relaxed);
            dma::stop(CHANNEL);
            let mut cursor = cursor();
            cursor.drop_inflight();
            TAIL.store(cursor.parts().1, Ordering::Relaxed);
            INFLIGHT.store(0, Ordering::Relaxed);
            reaped = true;
        }
        let mut cursor = cursor();
        if cursor.is_armed() && dma::tx_complete(CHANNEL) {
            let done = cursor.reap();
            TAIL.store(cursor.parts().1, Ordering::Relaxed);
            INFLIGHT.store(0, Ordering::Relaxed);
            SENT.fetch_add(done as u32, Ordering::Relaxed);
            reaped = true;
        }
        kick();
        // Wake FLEXCOMM0's deferred rate change, only on the transition to empty and from a reap.
        if reaped && INFLIGHT.load(Ordering::Relaxed) == 0 {
            #[cfg(not(test))]
            cortex_m::peripheral::NVIC::pend(lpc55_hal::raw::Interrupt::FLEXCOMM0);
        }
    });
}

/// Arm from `TAIL` to the ring end or the writer, whichever is first. Callers hold the mask.
fn kick() {
    let mut cursor = cursor();
    #[cfg(feature = "probe-timing")]
    let (busy, empty) = (cursor.is_armed(), cursor.queued() == 0);
    let Some((tail, len)) = cursor.arm() else {
        #[cfg(feature = "probe-timing")]
        if busy {
            PROBE_KICK_BUSY.fetch_add(1, Ordering::Relaxed);
        } else if empty {
            PROBE_KICK_EMPTY.fetch_add(1, Ordering::Relaxed);
        }
        return;
    };
    INFLIGHT.store(len, Ordering::Relaxed);
    #[cfg(feature = "probe-timing")]
    {
        PROBE_TX_ARMED.fetch_add(1, Ordering::Relaxed);
        PROBE_TX_ARMED_BYTES.fetch_add(len as u32, Ordering::Relaxed);
    }
    // SAFETY: the run lies between `TAIL` and `HEAD`, which the writer never stages into.
    let src = unsafe { core::ptr::addr_of!((*dma::mem()).tx.buf[tail]) };
    dma::arm_tx(
        CHANNEL,
        src,
        len,
        core::ptr::addr_of!(usart().fifowr) as u32,
        PRIORITY,
    );
}

/// Bytes the ring will take, which the caller may offer in one call: [`write`] wraps for itself.
pub(crate) fn capacity() -> usize {
    if !STARTED.load(Ordering::Relaxed) {
        return 0;
    }
    service();
    cursor().free()
}

/// Stage bytes for transmission, returning how many were taken.
pub(crate) fn write(data: &[u8]) -> usize {
    if !STARTED.load(Ordering::Relaxed) {
        return 0;
    }
    service();
    let mut cursor = cursor();
    let head = cursor.parts().0;
    if data.is_empty() || cursor.free() == 0 {
        return 0;
    }
    // Copied before `HEAD` moves: a completion mid-copy arms only what was already published.
    let taken = cursor.accept(data.len());
    // SAFETY: the bytes lie beyond `HEAD` within the free count, unread by the controller.
    unsafe {
        let base = core::ptr::addr_of_mut!((*dma::mem()).tx.buf) as *mut u8;
        core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(head), taken.first);
        if taken.second != 0 {
            core::ptr::copy_nonoverlapping(data.as_ptr().add(taken.first), base, taken.second);
        }
    }
    // Publishing and arming together: the completion preempts this and would miss the copy above.
    dma::masked(|| {
        HEAD.store(cursor.parts().0, Ordering::Relaxed);
        kick();
    });
    taken.taken
}

/// Bytes past the transmitter since boot: a floor, short by any transfer in flight.
pub(crate) fn sent() -> u32 {
    if !STARTED.load(Ordering::Relaxed) {
        return 0;
    }
    // `SENT` first: a completion landing mid-read can then only subtract more than it should.
    let sent = SENT.load(Ordering::Relaxed);
    let usart = usart();
    let resident = usart.fifostat.read().txlvl().bits() as u32
        + u32::from(usart.stat.read().txidle().bit_is_clear());
    let wire = wire_floor(sent, resident, WIRE.load(Ordering::Relaxed));
    WIRE.store(wire, Ordering::Relaxed);
    wire
}

/// Bytes past the transmitter, never below the `prev` floor even across the counter wrapping.
#[inline(always)]
fn wire_floor(sent: u32, resident: u32, prev: u32) -> u32 {
    let floor = sent.wrapping_sub(resident);
    // Signed distance, so the clamp against a mid-transfer dip still survives the counter wrapping.
    if (floor.wrapping_sub(prev) as i32) > 0 {
        floor
    } else {
        prev
    }
}

/// Whether everything staged reached the transmitter; the shift register may still be draining.
pub(crate) fn drained() -> bool {
    if !STARTED.load(Ordering::Relaxed) {
        return true;
    }
    service();
    let cursor = cursor();
    !cursor.is_armed() && cursor.queued() == 0
}

#[cfg(test)]
pub(crate) fn stop_test() {
    use dma::stop_tests::{assert_unreleased, failed, script};
    start();
    assert_eq!(write(&[1, 2, 3, 4]), 4);
    assert_eq!(write(&[5, 6]), 2);
    let before = cursor().parts();
    let bytes = unsafe { (*dma::mem()).tx.buf };
    script(CHANNEL, None);
    dma::error_sample();
    failed(service);
    assert_unreleased(CHANNEL);
    assert_eq!(cursor().parts(), before);
    assert_eq!(unsafe { (*dma::mem()).tx.buf }, bytes);
    assert_eq!(SENT.load(Ordering::Relaxed), 0);
    assert_eq!(VCOM_TX_DMA_ERR.load(Ordering::Relaxed), 1);
    script(CHANNEL, Some(1));
    dma::error_sample();
    service();
    assert_eq!(cursor().parts(), (6, 4, 2));
    assert_eq!(SENT.load(Ordering::Relaxed), 0);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The floor advances with the counter, holds against a dip, and survives the wrap.
    #[test]
    fn the_wire_floor_never_walks_backwards_across_the_wrap() {
        assert_eq!(wire_floor(100, 3, 90), 97);
        assert_eq!(wire_floor(100, 20, 90), 90);
        assert_eq!(wire_floor(100, 10, 90), 90);
        assert_eq!(wire_floor(5, 10, u32::MAX - 100), u32::MAX - 4);
        assert_eq!(wire_floor(10, 0, u32::MAX - 5), 10);
        assert_eq!(
            wire_floor(u32::MAX - 20, 0, 3),
            3,
            "a dip across the wrap still holds"
        );
    }
}
