//! CDC-ACM VCOM USART seam: GPDMA on both legs, with the USART interrupt reduced to wakes.

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::sync::atomic::{fence, AtomicU32, AtomicUsize, Ordering};
use core::task::Poll;

use daplink_embassy_usb::{SeamStats, VcomUart};
use daplink_rs::daplink_core::DEFAULT_VCOM_BAUD;
use daplink_rs::{
    vcom_episodes, vcom_grade, DmaRxChannel, DmaRxRing, VCOM_GRADE_LOWER_BOUND,
    VCOM_SLOT_FRAME_ERR, VCOM_SLOT_OVERRUN, VCOM_SLOT_RING_FULL, VCOM_SLOT_RX_DMA_ERR,
    VCOM_SLOT_UART_TX, VCOM_VALID_FRAME_ERR, VCOM_VALID_OVERRUN, VCOM_VALID_RING_FULL,
    VCOM_VALID_RX_DMA_ERR, VCOM_VALID_UART_TX,
};
use embassy_futures::select::{select, Either};
use embassy_stm32::dma::ChannelInstance;
use embassy_stm32::interrupt::typelevel::Handler;
use embassy_stm32::interrupt::{Interrupt, InterruptExt};
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::pac::gpdma::vals::{Ap, Dreq, Dw};
use embassy_stm32::pac::usart::vals::Over8;
use embassy_stm32::usart::Uart;
use embassy_stm32::Peri;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Timer};
use static_cell::ConstStaticCell;

const RX_RING: usize = 2048;
const TX_RING: usize = 2048;

// `BR1.BNDT` is 16 bits of bytes, so one block transfer cannot exceed this either side.
const _: () = assert!(RX_RING <= 0xFFFF);
const _: () = assert!(TX_RING <= 0xFFFF);

/// Fallback poll: a saturated stream never fires IDLE, and a ring a baud change emptied has no TC.
const DMA_POLL: Duration = Duration::from_micros(500);

/// Bound on the suspend handshake spin, so a wedged channel cannot hang the caller.
const SUSPEND_SPINS: u32 = 10_000;

/// Suspends whose bound expired with the channel still busy, either direction: the reset that
/// follows lands on running hardware, so nothing the seam counted across one is evidence.
static DMA_FAULTS: AtomicU32 = AtomicU32::new(0);

/// Suspend `ch` and wait for it to let go, charging a fault when the bound expires; the caller
/// resets the channel either way, so an expired bound is recorded rather than passed over.
fn suspend(ch: pac::gpdma::Channel) {
    if !ch.cr().read().en() {
        return;
    }
    ch.cr().modify(|w| w.set_susp(true));
    let mut spins = 0;
    while spins < SUSPEND_SPINS {
        let sr = ch.sr().read();
        if sr.suspf() || sr.idlef() {
            return;
        }
        spins += 1;
    }
    DMA_FAULTS.fetch_add(1, Ordering::Relaxed);
}

/// Baud rate represented by the USART divisor currently on the wire.
static REALIZED_BAUD: AtomicU32 = AtomicU32::new(DEFAULT_VCOM_BAUD);

/// Frame/parity/noise episodes; the flagged byte still reaches the ring through DMA.
static FRAME_ERR: AtomicU32 = AtomicU32::new(0);
/// Overrun episodes: the first RX FIFO entry survives, the shift register and what follows do not.
static OVERRUN: AtomicU32 = AtomicU32::new(0);
/// Bytes whose burst the transmit-complete interrupt reaped, so every counted byte left the wire.
static UART_TX: AtomicU32 = AtomicU32::new(0);

static RX_WAKER: AtomicWaker = AtomicWaker::new();
static TX_WAKER: AtomicWaker = AtomicWaker::new();

/// Append and armed-run positions; equal means empty, so one byte of the ring stays unused.
static TX_HEAD: AtomicUsize = AtomicUsize::new(0);
static TX_TAIL: AtomicUsize = AtomicUsize::new(0);
/// Bytes the current burst covers, zero when the channel is idle; `TAIL` advances by this on reap.
static TX_INFLIGHT: AtomicUsize = AtomicUsize::new(0);

/// Bytes queued in the transmit ring, sent and unsent both.
fn tx_queued(head: usize, tail: usize) -> usize {
    (head + TX_RING - tail) % TX_RING
}

/// Return the baud rate represented by the divisor the VCOM USART currently holds.
pub fn vcom_realized_baud() -> u32 {
    REALIZED_BAUD.load(Ordering::Relaxed)
}

/// The seam's answer to the `0x9E` query: the realized rate plus the bridge's applied-ness.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    daplink_embassy_usb::vcom_baud_state(vcom_realized_baud())
}

/// Divisors the USART clock prescaler field selects, indexed by the field value (RM0481 PRESC).
const PRESCALERS: [u32; 12] = [1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256];

/// Linked-list item closing the receive ring: `CDAR` then `CLLR`, in fetch order. 8-byte aligned
/// so it cannot straddle the 64 KiB window `CLBAR` addresses.
#[repr(C, align(8))]
struct RxLli([u32; 2]);

/// `CLLR` image for the item at `lli`: reload `CDAR` and `CLLR`; `UB1` clear reloads `BNDT`.
fn rx_llr_value(lli: u32) -> pac::gpdma::regs::ChLlr {
    let mut v = pac::gpdma::regs::ChLlr(0);
    v.set_uda(true);
    v.set_ull(true);
    v.set_la(((lli & 0xFFFF) >> 2) as u16);
    v
}

/// The GPDMA channel filling the receive ring, looped through a one-item list (no address wrap).
#[derive(Clone, Copy)]
struct VcomRxChannel {
    dma: pac::gpdma::Gpdma,
    ch: usize,
    request: u8,
    base: u32,
    src: u32,
    lli: u32,
}

impl VcomRxChannel {
    fn regs(&self) -> pac::gpdma::Channel {
        self.dma.ch(self.ch)
    }

    fn enabled(&self) -> bool {
        self.regs().cr().read().en()
    }

    /// Arm the transfer at the ring's base.
    fn start(&self) {
        let ch = self.regs();
        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| {
            w.set_dtef(true);
            w.set_htf(true);
            w.set_suspf(true);
            w.set_tcf(true);
            w.set_tof(true);
            w.set_ulef(true);
            w.set_usef(true);
        });
        ch.lbar().write(|w| w.set_lba((self.lli >> 16) as u16));
        ch.tr1().write(|w| {
            w.set_sdw(Dw::BYTE);
            w.set_ddw(Dw::BYTE);
            w.set_sinc(false);
            w.set_dinc(true);
            w.set_sap(Ap::PORT1);
            w.set_dap(Ap::PORT0);
        });
        ch.tr2().write(|w| {
            w.set_dreq(Dreq::SOURCE_PERIPHERAL);
            w.set_reqsel(self.request);
        });
        ch.tr3().write(|_w| {});
        ch.br1().write(|w| w.set_bndt(RX_RING as u16));
        ch.sar().write_value(self.src);
        ch.dar().write_value(self.base);
        ch.llr().write_value(rx_llr_value(self.lli));
        // The linked-list item and the configuration must be visible before the enable.
        fence(Ordering::SeqCst);
        ch.cr().write(|w| w.set_en(true));
    }

    /// Stop the transfer, ring memory untouched; the divisor may only change while it is stopped.
    fn halt(&self) {
        let ch = self.regs();
        suspend(ch);
        ch.cr().write(|w| w.set_reset(true));
        fence(Ordering::SeqCst);
    }
}

impl DmaRxChannel for VcomRxChannel {
    fn remaining(&self) -> usize {
        self.regs().br1().read().bndt() as usize
    }

    fn reload(&mut self, _len: usize) {
        self.halt();
        self.start();
    }

    fn running(&self) -> bool {
        self.enabled()
    }
}

/// The GPDMA channel draining one transmit burst; no linked list, so [`service`] arms each run.
#[derive(Clone, Copy)]
struct VcomTxChannel {
    dma: pac::gpdma::Gpdma,
    ch: usize,
    request: u8,
    dst: u32,
}

impl VcomTxChannel {
    fn regs(&self) -> pac::gpdma::Channel {
        self.dma.ch(self.ch)
    }

    /// Arm a one-shot burst of `len` bytes starting at `src`.
    fn start(&self, src: u32, len: usize) {
        let ch = self.regs();
        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| {
            w.set_dtef(true);
            w.set_htf(true);
            w.set_suspf(true);
            w.set_tcf(true);
            w.set_tof(true);
            w.set_ulef(true);
            w.set_usef(true);
        });
        ch.tr1().write(|w| {
            w.set_sdw(Dw::BYTE);
            w.set_ddw(Dw::BYTE);
            w.set_sinc(true);
            w.set_dinc(false);
            w.set_sap(Ap::PORT0);
            w.set_dap(Ap::PORT1);
        });
        ch.tr2().write(|w| {
            w.set_dreq(Dreq::DESTINATION_PERIPHERAL);
            w.set_reqsel(self.request);
        });
        ch.tr3().write(|_w| {});
        ch.br1().write(|w| w.set_bndt(len as u16));
        ch.sar().write_value(src);
        ch.dar().write_value(self.dst);
        // The configuration must be visible before the enable.
        fence(Ordering::SeqCst);
        ch.cr().write(|w| w.set_en(true));
    }

    /// Stop a burst mid-flight, same discipline as [`VcomRxChannel::halt`].
    fn halt(&self) {
        let ch = self.regs();
        suspend(ch);
        ch.cr().write(|w| w.set_reset(true));
        fence(Ordering::SeqCst);
    }
}

/// What the interrupt handler needs to reap a transmit burst and arm the next.
#[derive(Clone, Copy)]
struct IrqShared {
    regs: pac::usart::Usart,
    tx_chan: VcomTxChannel,
    tx_base: u32,
}

struct IrqCell(UnsafeCell<Option<IrqShared>>);

// SAFETY: written once by `StmVcom::new` with the USART line masked, and read only by that line's
// handler afterwards.
unsafe impl Sync for IrqCell {}

static IRQ: IrqCell = IrqCell(UnsafeCell::new(None));

/// Interrupt handler for the VCOM USART; bind that line to it and nothing else.
pub struct VcomInterruptHandler;

impl<I: embassy_stm32::interrupt::typelevel::Interrupt> Handler<I> for VcomInterruptHandler {
    unsafe fn on_interrupt() {
        // SAFETY: published before the line is unmasked, and reached only by this handler.
        if let Some(shared) = unsafe { *IRQ.0.get() } {
            service(shared);
        }
    }
}

/// One interrupt pass: count a receive error, wake a reader on IDLE, and on TC reap the finished
/// transmit burst and arm the next. No byte crosses this handler as data.
fn service(shared: IrqShared) {
    let r = shared.regs;
    let sr = r.isr().read();
    if sr.ore() {
        r.icr().write(|w| w.set_ore(true));
        OVERRUN.fetch_add(1, Ordering::Relaxed);
    }
    if sr.pe() || sr.fe() || sr.ne() {
        r.icr().write(|w| {
            w.set_pe(true);
            w.set_fe(true);
            w.set_ne(true);
        });
        FRAME_ERR.fetch_add(1, Ordering::Relaxed);
    }
    if sr.idle() {
        r.icr().write(|w| w.set_idle(true));
        RX_WAKER.wake();
    }
    if sr.tc() {
        // TC stays set with nothing left to send, so TCIE is disarmed here and re-armed per burst.
        r.icr().write(|w| w.set_tc(true));
        r.cr1().modify(|w| w.set_tcie(false));
        let inflight = TX_INFLIGHT.load(Ordering::Relaxed);
        if inflight != 0 {
            let tail = TX_TAIL.load(Ordering::Relaxed);
            TX_TAIL.store((tail + inflight) % TX_RING, Ordering::Relaxed);
            TX_INFLIGHT.store(0, Ordering::Relaxed);
            UART_TX.fetch_add(inflight as u32, Ordering::Relaxed);
        }
        tx_kick_regs(r, shared.tx_chan, shared.tx_base);
        TX_WAKER.wake();
    }
}

/// Arm the run from `TAIL` to `HEAD` or the ring's end, if the channel is idle; caller holds the mask.
fn tx_kick_regs(regs: pac::usart::Usart, tx_chan: VcomTxChannel, tx_base: u32) {
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
        TX_RING - tail
    };
    TX_INFLIGHT.store(run, Ordering::Relaxed);
    regs.icr().write(|w| w.set_tc(true));
    regs.cr1().modify(|w| w.set_tcie(true));
    tx_chan.start(tx_base + tail as u32, run);
}

/// The USART seam the shared bridge drives: a GPDMA ring on receive, GPDMA bursts on transmit.
pub struct StmVcom {
    /// Holds the pins, clock and framing; the seam uses it only to reprogram the divisor.
    uart: Uart<'static, Blocking>,
    /// The USART registers, for the divisor read-back, the TC mask, and the rejection undo.
    regs: pac::usart::Usart,
    /// The USART's NVIC line, left disabled by the HAL's baud-rejection path.
    nvic: Interrupt,
    /// Kernel clock feeding this USART, ahead of its own prescaler.
    kernel_clock: u32,
    /// Lap accounting over the receive ring the GPDMA channel fills.
    rx: DmaRxRing<VcomRxChannel>,
    /// A second handle to the same channel, for the direct halt a baud change needs.
    rx_chan: VcomRxChannel,
    /// The ring buffer the channel fills; read volatile through `rx`, never written here.
    mem: &'static [u8],
    /// Bytes a lap overwrote before the bridge drained them, saturating.
    rx_lost: u32,
    tx_chan: VcomTxChannel,
    /// Base of the transmit ring; written only under the `TX_HEAD`/`TX_TAIL` guard.
    tx_base: u32,
}

impl StmVcom {
    /// Take over `uart`'s interrupt and two GPDMA channels; `regs`, `nvic` and `kernel_clock` must
    /// name the same USART, whose line must be bound to [`VcomInterruptHandler`]. Call once.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        uart: Uart<'static, Blocking>,
        regs: pac::usart::Usart,
        nvic: Interrupt,
        kernel_clock: u32,
        rx_dma: pac::gpdma::Gpdma,
        rx_ch_index: usize,
        rx_request: u8,
        rx_channel: Peri<'static, impl ChannelInstance>,
        tx_dma: pac::gpdma::Gpdma,
        tx_ch_index: usize,
        tx_request: u8,
        tx_channel: Peri<'static, impl ChannelInstance>,
    ) -> Self {
        static RX_MEM: ConstStaticCell<[u8; RX_RING]> = ConstStaticCell::new([0; RX_RING]);
        let mem = RX_MEM.take();
        let base = mem.as_ptr() as u32;
        let mem: &'static [u8] = &*mem;

        static LLI: ConstStaticCell<RxLli> = ConstStaticCell::new(RxLli([0; 2]));
        let lli = LLI.take();
        let lli_addr = lli as *const RxLli as u32;
        // SAFETY: `lli` is exclusively owned here; the words are `CDAR` then `CLLR`, the fetch
        // order.
        unsafe {
            core::ptr::write_volatile(&mut lli.0[0], base);
            core::ptr::write_volatile(&mut lli.0[1], rx_llr_value(lli_addr).0);
        }

        // The channel peripherals are claimed for ownership; the registers go through `dma`/`ch`.
        let _ = (rx_channel, tx_channel);
        let rx_chan = VcomRxChannel {
            dma: rx_dma,
            ch: rx_ch_index,
            request: rx_request,
            base,
            src: regs.rdr().as_ptr() as u32,
            lli: lli_addr,
        };
        rx_chan.start();

        static TX_MEM: ConstStaticCell<[u8; TX_RING]> = ConstStaticCell::new([0; TX_RING]);
        let tx_base = TX_MEM.take().as_mut_ptr() as u32;
        let tx_chan = VcomTxChannel {
            dma: tx_dma,
            ch: tx_ch_index,
            request: tx_request,
            dst: regs.tdr().as_ptr() as u32,
        };

        nvic.disable();
        // SAFETY: the line is masked, so no handler can be reading the cell.
        unsafe {
            *IRQ.0.get() = Some(IrqShared {
                regs,
                tx_chan,
                tx_base,
            });
        }
        regs.cr3().modify(|w| {
            // EIE is how frame/noise/overrun still reach `service`: DMA cannot see those flags.
            w.set_dmar(true);
            w.set_dmat(true);
            w.set_eie(true);
        });
        // RXNEIE stays clear: every good byte leaves through DMA. TCIE is armed per burst.
        regs.cr1().modify(|w| {
            w.set_peie(true);
            w.set_idleie(true);
        });
        nvic.unpend();
        // SAFETY: the handler's state is published, so the line is ready to be serviced.
        unsafe { nvic.enable() };

        let vcom = Self {
            uart,
            regs,
            nvic,
            kernel_clock,
            rx: DmaRxRing::new(rx_chan, RX_RING),
            rx_chan,
            mem,
            rx_lost: 0,
            tx_chan,
            tx_base,
        };
        vcom.record_realized_baud();
        vcom
    }

    /// Publish the rate the divisor chain the USART holds produces, read back not recomputed.
    fn record_realized_baud(&self) {
        let presc = PRESCALERS
            .get(usize::from(self.regs.presc().read().prescaler().to_bits()))
            .copied()
            .unwrap_or(1);
        let brr = u32::from(self.regs.brr().read().brr());
        // Oversampling by 8: the register holds twice the divisor, bit 3 zeroed, low bits in place.
        let divisor = match self.regs.cr1().read().over8() {
            Over8::OVERSAMPLING8 => ((brr & !0xf) >> 1) | (brr & 0x7),
            Over8::OVERSAMPLING16 => brr,
        };
        let clock = self.kernel_clock / presc;
        if let Some(realized) = (clock + divisor / 2).checked_div(divisor) {
            REALIZED_BAUD.store(realized, Ordering::Relaxed);
        }
    }

    /// One non-blocking pass over the ring, charging what a lap lost to the ledger.
    fn drain_rx(&mut self, buf: &mut [u8]) -> usize {
        let lost = self.rx.observe();
        self.rx_lost = self.rx_lost.saturating_add(lost as u32);
        let (n, lost) = self.rx.read(self.mem, buf);
        self.rx_lost = self.rx_lost.saturating_add(lost as u32);
        n
    }

    fn tx_kick(&self) {
        tx_kick_regs(self.regs, self.tx_chan, self.tx_base);
    }

    /// Stage as much of `data` as the ring has room for; returns how many bytes were taken.
    fn tx_push(&self, data: &[u8]) -> usize {
        let head = TX_HEAD.load(Ordering::Relaxed);
        let tail = TX_TAIL.load(Ordering::Relaxed);
        let free = TX_RING - 1 - tx_queued(head, tail);
        let n = data.len().min(free);
        if n == 0 {
            return 0;
        }
        // Copy before `TX_HEAD` moves: a completion mid-copy must arm only what is published.
        let first = (TX_RING - head).min(n);
        // SAFETY: the bytes lie beyond `TX_HEAD` within the free count, so the channel is idle on them.
        unsafe {
            let base = self.tx_base as *mut u8;
            core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(head), first);
            if n > first {
                core::ptr::copy_nonoverlapping(data.as_ptr().add(first), base, n - first);
            }
        }
        // Publish and kick under one mask, or a completion re-arms against a stale `TX_HEAD`.
        cortex_m::interrupt::free(|_| {
            TX_HEAD.store((head + n) % TX_RING, Ordering::Relaxed);
            self.tx_kick();
        });
        n
    }

    /// Wait for the ring to free a byte, racing the fallback poll a baud-emptied ring needs.
    async fn tx_wait_room(&self) {
        loop {
            let woken = poll_fn(|cx| {
                TX_WAKER.register(cx.waker());
                let queued = tx_queued(
                    TX_HEAD.load(Ordering::Relaxed),
                    TX_TAIL.load(Ordering::Relaxed),
                );
                if queued < TX_RING - 1 {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            });
            match select(woken, Timer::after(DMA_POLL)).await {
                Either::First(()) => return,
                Either::Second(()) => continue,
            }
        }
    }
}

impl VcomUart for StmVcom {
    /// Ring-full is this seam's own lap accounting; frame and overrun come from the error interrupt.
    const SEAM_VALID: u32 = VCOM_VALID_RING_FULL
        | VCOM_VALID_FRAME_ERR
        | VCOM_VALID_OVERRUN
        | VCOM_VALID_UART_TX
        | VCOM_VALID_RX_DMA_ERR;
    /// Lower bounds throughout: the position-only cursor aliases whole laps, one interrupt pass
    /// can fold several error episodes, a baud change drops the transmit ring uncounted, and the
    /// DMA slot counts only expired suspends -- the GPDMA's own `DTEF`/`ULEF`/`USEF` flags are
    /// cleared unobserved at every arm.
    const SEAM_GRADE: u32 = vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_LOWER_BOUND)
        | vcom_grade(VCOM_SLOT_FRAME_ERR, VCOM_GRADE_LOWER_BOUND)
        | vcom_grade(VCOM_SLOT_OVERRUN, VCOM_GRADE_LOWER_BOUND)
        | vcom_grade(VCOM_SLOT_UART_TX, VCOM_GRADE_LOWER_BOUND)
        | vcom_grade(VCOM_SLOT_RX_DMA_ERR, VCOM_GRADE_LOWER_BOUND);
    const SEAM_UNITS: u32 = vcom_episodes(VCOM_SLOT_FRAME_ERR)
        | vcom_episodes(VCOM_SLOT_OVERRUN)
        | vcom_episodes(VCOM_SLOT_RX_DMA_ERR);

    /// Cancel-safe: bytes leave the ring only in the synchronous drain of a `Ready` poll.
    async fn read(&mut self, buf: &mut [u8]) -> usize {
        loop {
            let woken = poll_fn(|cx| {
                RX_WAKER.register(cx.waker());
                match self.drain_rx(buf) {
                    0 => Poll::Pending,
                    n => Poll::Ready(n),
                }
            });
            match select(woken, Timer::after(DMA_POLL)).await {
                Either::First(n) => return n,
                Either::Second(()) => {
                    let n = self.drain_rx(buf);
                    if n > 0 {
                        return n;
                    }
                }
            }
        }
    }

    fn read_ready(&mut self, buf: &mut [u8]) -> usize {
        self.drain_rx(buf)
    }

    fn seam_stats(&self) -> SeamStats {
        SeamStats {
            ring_full: self.rx_lost,
            frame_err: FRAME_ERR.load(Ordering::Relaxed),
            overrun: OVERRUN.load(Ordering::Relaxed),
            uart_tx: UART_TX.load(Ordering::Relaxed),
            // Either VCOM channel: a baud change re-arms the receive ring across every such reset.
            rx_dma_err: DMA_FAULTS.load(Ordering::Relaxed),
        }
    }

    /// Stage `data` in the transmit ring; returns once it is queued, not once it is on the wire.
    async fn write_all(&mut self, data: &[u8]) {
        let mut sent = 0;
        while sent < data.len() {
            sent += self.tx_push(&data[sent..]);
            if sent < data.len() {
                self.tx_wait_room().await;
            }
        }
    }

    /// Empty-and-reaped is only ever established by TC, the completion of the last frame on the
    /// wire, so this resolves at wire-idle with nothing further to prove.
    async fn drain_tx(&mut self) {
        loop {
            let idle = poll_fn(|cx| {
                TX_WAKER.register(cx.waker());
                let empty = TX_HEAD.load(Ordering::Relaxed) == TX_TAIL.load(Ordering::Relaxed)
                    && TX_INFLIGHT.load(Ordering::Relaxed) == 0;
                if empty {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            });
            match select(idle, Timer::after(DMA_POLL)).await {
                Either::First(()) => return,
                Either::Second(()) => continue,
            }
        }
    }

    fn set_baudrate(&mut self, baud: u32) {
        // Both channels stop first: a divisor change under a running transfer invalidates its counter.
        self.rx_chan.halt();
        // Mask TCIE across the halt: a suspended burst can still finish a latched byte and raise TC.
        cortex_m::interrupt::free(|_| {
            self.regs.cr1().modify(|w| w.set_tcie(false));
            self.tx_chan.halt();
            self.regs.icr().write(|w| w.set_tc(true));
            // Queued bytes were framed at the old rate; the whole ring is dropped.
            TX_HEAD.store(0, Ordering::Relaxed);
            TX_TAIL.store(0, Ordering::Relaxed);
            TX_INFLIGHT.store(0, Ordering::Relaxed);
        });
        // Outside the mask: wakes a writer waiting on room this just freed.
        TX_WAKER.wake();
        // A rate the USART cannot realize leaves the previous divisor in force.
        if self.uart.set_baudrate(baud).is_err() {
            // The HAL's rejection path leaves UE and the NVIC line disabled; restore both.
            self.regs.cr1().modify(|w| w.set_ue(true));
            self.nvic.unpend();
            // SAFETY: the seam's handler state has been published since construction.
            unsafe { self.nvic.enable() };
        }
        self.record_realized_baud();
        // Restart discards the old framing and re-bases the cursor, closing any accumulated lap.
        self.rx.restart();
    }
}
