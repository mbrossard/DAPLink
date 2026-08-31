//! VCOM transmit over UART1 TX via a DMA byte ring, re-armed from its own completion.

use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use daplink_rp_dma_uart::seq::DmaChannel;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::Peri;

use embassy_rp::interrupt;
use embassy_rp::interrupt::typelevel::{Handler, Interrupt};
use embassy_rp::interrupt::InterruptExt;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_time::{Duration, Timer};
use static_cell::ConstStaticCell;

use crate::dma_rx::RX_RING_LEN;

/// Transmit ring capacity: matches the receive ring's.
const TX_RING_LEN: usize = RX_RING_LEN;

/// Channel claimed for transmit, taken from the peripheral token so the claim cannot disagree.
static TX_CHAN_NUM: AtomicUsize = AtomicUsize::new(0);

fn tx_chan() -> usize {
    TX_CHAN_NUM.load(Ordering::Relaxed)
}
/// `INTE1`/`INTS1` index: transmit completion is routed to `DMA_IRQ_1`, as receive is.
const IRQ_LINE: usize = 1;

/// Interval a writer waits on when the ring is full: no waker backs this seam.
const TX_POLL: Duration = Duration::from_micros(500);

/// Writer append point and armed-run start; equal means empty, so one byte is never used.
static TX_HEAD: AtomicUsize = AtomicUsize::new(0);
static TX_TAIL: AtomicUsize = AtomicUsize::new(0);
/// Bytes the armed run covers, zero when idle; `TAIL` advances by this on reap.
static TX_INFLIGHT: AtomicUsize = AtomicUsize::new(0);
/// Bytes reaped from completed bursts; load/store only (thumbv6m), written under the interrupt
/// mask its site already holds.
static TX_SENT: AtomicU32 = AtomicU32::new(0);

/// Ring base and the fixed `UARTDR` destination, set once by [`new`].
static TX_BASE: AtomicU32 = AtomicU32::new(0);
static TX_DST: AtomicU32 = AtomicU32::new(0);

/// Bytes queued in the ring, sent and unsent both.
fn queued(head: usize, tail: usize) -> usize {
    (head + TX_RING_LEN - tail) % TX_RING_LEN
}

fn chan() -> DmaChannel {
    DmaChannel::new(tx_chan())
}

/// Arm a one-shot burst of `len` bytes starting at ring offset `tail`.
fn arm(tail: usize, len: usize) {
    let chan = chan();
    chan.set_addrs(
        TX_BASE.load(Ordering::Relaxed) + tail as u32,
        TX_DST.load(Ordering::Relaxed),
    );
    chan.set_count(len as u32);
    chan.regs().ctrl_trig().write(|w| {
        w.set_en(true);
        w.set_data_size(DataSize::SIZE_BYTE);
        w.set_incr_read(true);
        w.set_treq_sel(TreqSel::UART1_TX);
        // Chained to itself, which means not chained: one burst, one completion.
        w.set_chain_to(tx_chan() as u8);
        // Raise the completion the re-arm hangs off (RP2040 section 2.5.4, RP2350 section 12.6.5).
        w.set_irq_quiet(false);
    });
}

/// Arm the queued run from `TAIL` up to `HEAD` or the ring's end, whichever comes first, if the
/// channel is idle. Caller holds the interrupt mask.
fn kick() {
    if TX_INFLIGHT.load(Ordering::Relaxed) != 0 {
        return;
    }
    let head = TX_HEAD.load(Ordering::Relaxed);
    let tail = TX_TAIL.load(Ordering::Relaxed);
    if head == tail {
        return;
    }
    let run = if head > tail {
        head - tail
    } else {
        TX_RING_LEN - tail
    };
    TX_INFLIGHT.store(run, Ordering::Relaxed);
    arm(tail, run);
}

/// Stage as much of `data` as the ring holds, kicking an idle channel; returns the bytes taken.
fn push(data: &[u8]) -> usize {
    let head = TX_HEAD.load(Ordering::Relaxed);
    let tail = TX_TAIL.load(Ordering::Relaxed);
    let free = TX_RING_LEN - 1 - queued(head, tail);
    let n = data.len().min(free);
    if n == 0 {
        return 0;
    }
    let base = TX_BASE.load(Ordering::Relaxed) as *mut u8;
    let first = (TX_RING_LEN - head).min(n);
    // SAFETY: the bytes lie beyond `head` and within the free count, and the channel only ever
    // reads between `TAIL` and `HEAD`.
    unsafe {
        core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(head), first);
        if n > first {
            core::ptr::copy_nonoverlapping(data.as_ptr().add(first), base, n - first);
        }
    }
    // Together: a completion between them would re-arm against a `HEAD` short of the copy above.
    cortex_m::interrupt::free(|_| {
        TX_HEAD.store((head + n) % TX_RING_LEN, Ordering::Relaxed);
        kick();
    });
    n
}

/// Bytes every reaped burst carried, free-running since boot. The channel reaps at `UARTDR`, so
/// this runs ahead of the wire by at most 33 bytes -- the 32-entry PL011 transmit FIFO plus the
/// shifter -- and is exact whenever the line is quiet.
pub(crate) fn sent() -> u32 {
    TX_SENT.load(Ordering::Relaxed)
}

/// Reap a finished burst and arm the next run queued behind it. Runs from `DMA_IRQ_1`.
fn service() {
    let mask = 1u32 << tx_chan();
    let ints = pac::DMA.ints(IRQ_LINE).read();
    if ints & mask == 0 {
        return;
    }
    pac::DMA.ints(IRQ_LINE).write_value(mask);
    cortex_m::interrupt::free(|_| {
        let inflight = TX_INFLIGHT.load(Ordering::Relaxed);
        if inflight != 0 {
            let tail = TX_TAIL.load(Ordering::Relaxed);
            TX_TAIL.store((tail + inflight) % TX_RING_LEN, Ordering::Relaxed);
            TX_INFLIGHT.store(0, Ordering::Relaxed);
            let sent = TX_SENT
                .load(Ordering::Relaxed)
                .wrapping_add(inflight as u32);
            TX_SENT.store(sent, Ordering::Relaxed);
        }
        kick();
    });
}

/// Handler for `DMA_IRQ_1`; bind that line to it and nothing else, as both VCOM halves use it.
pub struct VcomDmaInterruptHandler;

impl<I: Interrupt> Handler<I> for VcomDmaInterruptHandler {
    unsafe fn on_interrupt() {
        service();
        crate::dma_rx::service();
    }
}

/// Claim the transmit channel and ring, and enable completion; `uart_dr` is `UART1`'s `UARTDR`.
pub(crate) fn new<T: ChannelInstance>(uart_dr: u32, _tx: Peri<'static, T>) {
    TX_CHAN_NUM.store(T::number() as usize, Ordering::Relaxed);
    static MEM: ConstStaticCell<[u8; TX_RING_LEN]> = ConstStaticCell::new([0; TX_RING_LEN]);
    let base = MEM.take().as_mut_ptr() as u32;
    TX_BASE.store(base, Ordering::Relaxed);
    TX_DST.store(uart_dr, Ordering::Relaxed);

    interrupt::DMA_IRQ_1.set_priority(interrupt::Priority::P3);
    pac::DMA
        .inte(IRQ_LINE)
        .modify(|v| *v |= 1 << tx_chan() as u32);
    // SAFETY: the firmware's `bind_interrupts!` must name `DMA_IRQ_1 => VcomDmaInterruptHandler`.
    unsafe { interrupt::typelevel::DMA_IRQ_1::enable() };
}

/// Stage `data` in the ring, waiting for room; returns once queued, not once on the wire.
pub(crate) async fn write_all(data: &[u8]) {
    let mut sent = 0;
    while sent < data.len() {
        sent += push(&data[sent..]);
        if sent < data.len() {
            Timer::after(TX_POLL).await;
        }
    }
}

/// Resolve once the ring is empty and the last burst reaped; the PL011 may still be shifting.
pub(crate) async fn drain() {
    loop {
        let empty = cortex_m::interrupt::free(|_| {
            TX_HEAD.load(Ordering::Relaxed) == TX_TAIL.load(Ordering::Relaxed)
                && TX_INFLIGHT.load(Ordering::Relaxed) == 0
        });
        if empty {
            return;
        }
        Timer::after(TX_POLL).await;
    }
}

/// Stop the channel and drop what it holds, for a baud change: it was framed at the old rate.
/// Clears any completion the abort raised, so nothing re-arms against the discarded ring.
pub(crate) fn halt_for_baud_change() {
    cortex_m::interrupt::free(|_| {
        pac::DMA
            .inte(IRQ_LINE)
            .modify(|v| *v &= !(1 << tx_chan() as u32));
        // Completion disarmed first, because an abort can raise one (RP2040-E13, RP2350-E5).
        chan().abort();
        pac::DMA.ints(IRQ_LINE).write_value(1 << tx_chan());
        TX_HEAD.store(0, Ordering::Relaxed);
        TX_TAIL.store(0, Ordering::Relaxed);
        TX_INFLIGHT.store(0, Ordering::Relaxed);
        pac::DMA
            .inte(IRQ_LINE)
            .modify(|v| *v |= 1 << tx_chan() as u32);
    });
}
