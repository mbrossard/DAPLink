//! VCOM transmit over UART1 TX via a DMA byte ring, re-armed from its own completion.

use core::sync::atomic::{compiler_fence, AtomicU32, AtomicUsize, Ordering};

use daplink_core::DmaTxCursor;
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

/// Transmit ring capacity, independent of the receive ring's.
const TX_RING_LEN: usize = 2048;

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
/// Bytes reaped from completed bursts; load/store only, written under the interrupt mask.
static TX_SENT: AtomicU32 = AtomicU32::new(0);

/// Ring base and the fixed `UARTDR` destination, set once by [`new`].
static TX_BASE: AtomicU32 = AtomicU32::new(0);
static TX_DST: AtomicU32 = AtomicU32::new(0);

/// Ring arithmetic over the three cursors above; one transfer, so no per-transfer cap.
type Cursor = DmaTxCursor<TX_RING_LEN>;

/// Never unwind or restore interrupts while a DMA memory lease is unresolved.
pub(crate) fn abort_failed() -> ! {
    cortex_m::interrupt::disable();
    loop {
        core::hint::spin_loop();
    }
}

/// Read the three cursors into a value the arithmetic works on. Caller holds the mask.
fn cursor() -> Cursor {
    Cursor::from_parts(
        TX_HEAD.load(Ordering::Relaxed),
        TX_TAIL.load(Ordering::Relaxed),
        TX_INFLIGHT.load(Ordering::Relaxed),
    )
}

fn chan() -> DmaChannel {
    // SAFETY: new consumes the token; task and handler coordinate through the interrupt mask.
    unsafe { DmaChannel::new(tx_chan()) }
}

/// Arm a one-shot burst of `len` bytes starting at ring offset `tail`.
fn arm(tail: usize, len: usize) {
    let chan = chan();
    // SAFETY: the cursor reserves this initialized contiguous run until completion or abort.
    let regs = unsafe {
        chan.set_addrs(
            TX_BASE.load(Ordering::Relaxed) + tail as u32,
            TX_DST.load(Ordering::Relaxed),
        );
        chan.set_count(len as u32);
        chan.regs()
    };
    compiler_fence(Ordering::SeqCst);
    regs.ctrl_trig().write(|w| {
        w.set_en(true);
        w.set_data_size(DataSize::SIZE_BYTE);
        w.set_incr_read(true);
        w.set_treq_sel(TreqSel::UART1_TX);
        // Chained to itself, which means not chained: one burst, one completion.
        w.set_chain_to(tx_chan() as u8);
        // Raise the completion the re-arm hangs off (RP2040 section 2.5.4, RP2350 section 12.6.5).
        w.set_irq_quiet(false);
    });
    compiler_fence(Ordering::SeqCst);
}

/// Arm the queued run from `TAIL` to `HEAD` or the ring's end if the channel is idle; caller holds the mask.
fn kick() {
    let mut cursor = cursor();
    let Some((tail, run)) = cursor.arm() else {
        return;
    };
    TX_INFLIGHT.store(run, Ordering::Relaxed);
    arm(tail, run);
}

/// Stage as much of `data` as the ring holds, kicking an idle channel; returns the bytes taken.
fn push(data: &[u8]) -> usize {
    let mut cursor = cursor();
    let head = cursor.parts().0;
    if data.is_empty() || cursor.free() == 0 {
        return 0;
    }
    let taken = cursor.accept(data.len());
    let base = TX_BASE.load(Ordering::Relaxed) as *mut u8;
    // SAFETY: the copied free spans are disjoint from DMA's reserved TAIL-to-HEAD source.
    unsafe {
        core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(head), taken.first);
        if taken.second != 0 {
            core::ptr::copy_nonoverlapping(data.as_ptr().add(taken.first), base, taken.second);
        }
    }
    // Together: a completion between them would re-arm against a `HEAD` short of the copy above.
    cortex_m::interrupt::free(|_| {
        TX_HEAD.store(cursor.parts().0, Ordering::Relaxed);
        kick();
    });
    taken.taken
}

/// Bytes delivered to UARTDR since boot; FIFO and shifter may still hold up to 33 bytes.
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
    cortex_m::interrupt::free(|_| {
        if chan().busy() {
            return;
        }
        pac::DMA.ints(IRQ_LINE).write_value(mask);
        let mut cursor = cursor();
        let reaped = cursor.reap();
        if reaped != 0 {
            let (_, tail, _) = cursor.parts();
            TX_TAIL.store(tail, Ordering::Relaxed);
            TX_INFLIGHT.store(0, Ordering::Relaxed);
            let sent = TX_SENT.load(Ordering::Relaxed).wrapping_add(reaped as u32);
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
///
/// # Safety
/// The caller owns UART1 and binds DMA_IRQ_1 exclusively to VcomDmaInterruptHandler.
pub(crate) unsafe fn new<T: ChannelInstance>(uart_dr: u32, _tx: Peri<'static, T>) {
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

/// Stop and discard TX for a baud change; abort timeout permanently halts without releasing the ring.
pub(crate) fn halt_for_baud_change() {
    cortex_m::interrupt::free(|_| {
        pac::DMA
            .inte(IRQ_LINE)
            .modify(|v| *v &= !(1 << tx_chan() as u32));
        // SAFETY: completion is masked before abort can raise it; the static ring remains reserved.
        if !unsafe { chan().abort() } {
            abort_failed();
        }
        pac::DMA.ints(IRQ_LINE).write_value(1 << tx_chan());
        let mut cursor = cursor();
        cursor.reset();
        let (head, tail, inflight) = cursor.parts();
        TX_HEAD.store(head, Ordering::Relaxed);
        TX_TAIL.store(tail, Ordering::Relaxed);
        TX_INFLIGHT.store(inflight, Ordering::Relaxed);
        pac::DMA
            .inte(IRQ_LINE)
            .modify(|v| *v |= 1 << tx_chan() as u32);
    });
}
