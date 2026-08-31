//! Flexcomm0 transmit over DMA0 channel 5: no reload chain, and a byte ring rather than staged
//! packets, so one transfer carries every contiguous byte waiting.

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

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
/// Channel errors the controller latched. The `0x9D` reply has no slot for a transmit fault, so
/// this is the SWD name a bench reads.
#[no_mangle]
pub static VCOM_TX_DMA_ERR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Bytes the controller is reading now, zero when idle; what `TAIL` advances by on completion.
static INFLIGHT: AtomicUsize = AtomicUsize::new(0);
/// Whether [`start`] has run. Every entry point checks it: DMA0 is unclocked until the port is
/// handed over, and reading a channel register before that faults the core.
static STARTED: AtomicBool = AtomicBool::new(false);

/// Transfers armed and bytes in them: their ratio is the mean transfer, read out over SWD.
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
    // SAFETY: USART0 is owned by the VCOM seam, this module's only caller.
    unsafe { &*raw::USART0::ptr() }
}

/// Bytes waiting, sent and unsent both.
fn queued(head: usize, tail: usize) -> usize {
    (head + TX_RING_LEN - tail) % TX_RING_LEN
}

/// Hand the transmit FIFO to the controller. Called once, alongside the receive hand-over.
pub(crate) fn start() {
    // Shares FIFOCFG with the FLEXCOMM0 handler, which preempts this.
    cortex_m::interrupt::free(|_| {
        usart().fifocfg.modify(|_, w| w.dmatx().set_bit());
    });
    // Re-arm from the completion: waiting for the next USB pass idles the line at every boundary.
    dma::enable_int(CHANNEL);
    STARTED.store(true, Ordering::Relaxed);
}

/// Reap a finished transfer and start the next run of bytes; usually one register read. Runs from
/// the DMA0 completion and the USB pass both, so the sequence is masked against arming twice.
pub(crate) fn service() {
    if !STARTED.load(Ordering::Relaxed) {
        return;
    }
    cortex_m::interrupt::free(|_| {
        let mut reaped = false;
        // The flag is read rather than cleared blindly, as on the receive and trace channels. The
        // run in flight is dropped instead of re-sent: `cdc_out - uart_tx` then shows the loss,
        // where a re-send would put duplicate bytes on the wire.
        if dma::take_error(CHANNEL) {
            VCOM_TX_DMA_ERR.fetch_add(1, Ordering::Relaxed);
            dma::stop(CHANNEL);
            let dropped = INFLIGHT.swap(0, Ordering::Relaxed);
            let tail = TAIL.load(Ordering::Relaxed);
            TAIL.store((tail + dropped) % TX_RING_LEN, Ordering::Relaxed);
            reaped = true;
        }
        let inflight = INFLIGHT.load(Ordering::Relaxed);
        if inflight != 0 && dma::tx_complete(CHANNEL) {
            let tail = TAIL.load(Ordering::Relaxed);
            TAIL.store((tail + inflight) % TX_RING_LEN, Ordering::Relaxed);
            INFLIGHT.store(0, Ordering::Relaxed);
            SENT.fetch_add(inflight as u32, Ordering::Relaxed);
            reaped = true;
        }
        kick();
        // Wake FLEXCOMM0's deferred rate change, only on the transition to empty and from a reap.
        if reaped && INFLIGHT.load(Ordering::Relaxed) == 0 {
            cortex_m::peripheral::NVIC::pend(lpc55_hal::raw::Interrupt::FLEXCOMM0);
        }
    });
}

/// Arm the run of bytes from `TAIL` up to the end of the ring or the writer, whichever comes
/// first; a wrap simply costs one extra transfer. Callers hold the mask.
fn kick() {
    if INFLIGHT.load(Ordering::Relaxed) != 0 {
        #[cfg(feature = "probe-timing")]
        PROBE_KICK_BUSY.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let head = HEAD.load(Ordering::Relaxed);
    let tail = TAIL.load(Ordering::Relaxed);
    if head == tail {
        #[cfg(feature = "probe-timing")]
        PROBE_KICK_EMPTY.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let run = if head > tail {
        head - tail
    } else {
        TX_RING_LEN - tail
    };
    let len = run.min(TX_MAX_XFER);
    INFLIGHT.store(len, Ordering::Relaxed);
    #[cfg(feature = "probe-timing")]
    {
        PROBE_TX_ARMED.fetch_add(1, Ordering::Relaxed);
        PROBE_TX_ARMED_BYTES.fetch_add(len as u32, Ordering::Relaxed);
    }
    // SAFETY: the run is `MEM`'s, and it lies between `TAIL` and `HEAD`, which the writer never
    // stages into, so nothing else touches it until the transfer completes.
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
    let head = HEAD.load(Ordering::Relaxed);
    let tail = TAIL.load(Ordering::Relaxed);
    TX_RING_LEN - 1 - queued(head, tail)
}

/// Stage bytes for transmission, returning how many were taken.
pub(crate) fn write(data: &[u8]) -> usize {
    if !STARTED.load(Ordering::Relaxed) {
        return 0;
    }
    service();
    let head = HEAD.load(Ordering::Relaxed);
    let free = TX_RING_LEN - 1 - queued(head, TAIL.load(Ordering::Relaxed));
    let n = data.len().min(free);
    if n == 0 {
        return 0;
    }
    // Copied before `HEAD` moves: a completion mid-copy arms only what was already published.
    let first = (TX_RING_LEN - head).min(n);
    // SAFETY: the bytes lie beyond `HEAD` and within the free count taken above, so the
    // controller is not reading them.
    unsafe {
        let base = core::ptr::addr_of_mut!((*dma::mem()).tx.buf) as *mut u8;
        core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(head), first);
        if n > first {
            core::ptr::copy_nonoverlapping(data.as_ptr().add(first), base, n - first);
        }
    }
    // Publishing and arming together: the completion preempts this and would miss the copy above.
    cortex_m::interrupt::free(|_| {
        HEAD.store((head + n) % TX_RING_LEN, Ordering::Relaxed);
        kick();
    });
    n
}

/// Bytes past the transmitter, free-running since boot: the reaped total less what the FIFO and
/// the shift register still hold, which makes it a floor rather than a credit into `FIFOWR`. It is
/// tight whenever the line is quiet, and short by the transfer in flight while one is running.
pub(crate) fn sent() -> u32 {
    if !STARTED.load(Ordering::Relaxed) {
        return 0;
    }
    // `SENT` first: a completion landing mid-read can then only subtract more than it should.
    let sent = SENT.load(Ordering::Relaxed);
    let usart = usart();
    let resident = usart.fifostat.read().txlvl().bits() as u32
        + u32::from(usart.stat.read().txidle().bit_is_clear());
    let floor = sent.wrapping_sub(resident);
    let prev = WIRE.load(Ordering::Relaxed);
    // Signed distance, so the clamp against a mid-transfer dip still survives the counter wrapping.
    let wire = if (floor.wrapping_sub(prev) as i32) > 0 {
        floor
    } else {
        prev
    };
    WIRE.store(wire, Ordering::Relaxed);
    wire
}

/// Whether everything staged has been handed to the transmitter; the shift register may still be
/// draining, so a caller needing the line quiet checks `STAT.TXIDLE` as well.
pub(crate) fn drained() -> bool {
    if !STARTED.load(Ordering::Relaxed) {
        return true;
    }
    service();
    INFLIGHT.load(Ordering::Relaxed) == 0
        && HEAD.load(Ordering::Relaxed) == TAIL.load(Ordering::Relaxed)
}
