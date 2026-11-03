//! CDC-ACM VCOM USART seam: GPDMA on both legs, the USART interrupt reduced to wakes.

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::sync::atomic::{fence, AtomicU32, AtomicUsize, Ordering};
use core::task::Poll;

use daplink_core::DmaTxCursor;
use daplink_embassy_usb::{BaudApply, SeamStats, VcomUart};
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

use crate::baud::PRESCALERS;

const RX_RING: usize = 2048;
const TX_RING: usize = 2048;

// `BR1.BNDT` is 16 bits of bytes.
const _: () = assert!(RX_RING <= 0xFFFF);
const _: () = assert!(TX_RING <= 0xFFFF);

/// Fallback poll: a saturated stream never fires IDLE, and a baud-emptied ring has no TC.
const DMA_POLL: Duration = Duration::from_micros(500);

/// Suspends, either direction, whose bound expired with the channel still busy.
static DMA_FAULTS: AtomicU32 = AtomicU32::new(0);

/// Baud rate represented by the USART divisor currently on the wire.
static REALIZED_BAUD: AtomicU32 = AtomicU32::new(DEFAULT_VCOM_BAUD);

/// Frame/parity/noise episodes; with `DDRE` clear, the flagged byte never reaches the ring.
static FRAME_ERR: AtomicU32 = AtomicU32::new(0);
/// Overrun episodes.
static OVERRUN: AtomicU32 = AtomicU32::new(0);
/// Bytes of reaped bursts, each fully off the wire.
static UART_TX: AtomicU32 = AtomicU32::new(0);

static RX_WAKER: AtomicWaker = AtomicWaker::new();
static TX_WAKER: AtomicWaker = AtomicWaker::new();

/// Append and armed-run positions; equal means empty, so one ring byte stays unused.
static TX_HEAD: AtomicUsize = AtomicUsize::new(0);
static TX_TAIL: AtomicUsize = AtomicUsize::new(0);
/// Bytes the current burst covers, zero when idle.
static TX_INFLIGHT: AtomicUsize = AtomicUsize::new(0);

/// Ring arithmetic over the three cursors.
type TxCursor = DmaTxCursor<TX_RING>;

/// Snapshot the three cursors; callers hold the mask.
fn tx_cursor() -> TxCursor {
    TxCursor::from_parts(
        TX_HEAD.load(Ordering::Relaxed),
        TX_TAIL.load(Ordering::Relaxed),
        TX_INFLIGHT.load(Ordering::Relaxed),
    )
}

pub fn vcom_realized_baud() -> u32 {
    REALIZED_BAUD.load(Ordering::Relaxed)
}

/// The `0x9E` query's answer: the realized rate plus whether it was applied.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    daplink_embassy_usb::vcom_baud_state(vcom_realized_baud())
}

/// Linked-list item closing the receive ring: `CDAR` then `CLLR`; aligned so it never straddles 64 KiB.
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

/// The GPDMA channel filling the receive ring, looped through a one-item list.
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

    fn start(&self) {
        let ch = self.regs();
        crate::gpdma::halt(ch, &DMA_FAULTS);
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
        // The linked-list item and configuration must be visible before the enable.
        fence(Ordering::SeqCst);
        ch.cr().write(|w| w.set_en(true));
    }

    /// Undo suspension without resetting the channel.
    fn resume(&self) {
        let ch = self.regs();
        ch.fcr().write(|w| w.set_suspf(true));
        ch.cr().modify(|w| w.set_susp(false));
    }

    /// Stop the transfer, ring memory untouched; the divisor changes only while stopped.
    fn halt(&self) {
        crate::gpdma::halt(self.regs(), &DMA_FAULTS);
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

/// The GPDMA channel draining one transmit burst; [`service`] arms each run.
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

    fn start_released(&self, src: u32, len: usize) {
        let ch = self.regs();
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

    /// Stop a burst mid-flight, as [`VcomRxChannel::halt`] does.
    fn halt(&self) {
        crate::gpdma::halt(self.regs(), &DMA_FAULTS);
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

// SAFETY: written once by `StmVcom::new` with the line masked, then read only by its handler.
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

/// One interrupt pass: count receive errors, wake a reader on IDLE, reap and re-arm on TC.
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
        let mut cursor = tx_cursor();
        if cursor.is_armed() {
            shared.tx_chan.halt();
        }
        // TC stays set once idle, so TCIE is disarmed here and re-armed per burst.
        r.icr().write(|w| w.set_tc(true));
        r.cr1().modify(|w| w.set_tcie(false));
        let reaped = cursor.reap();
        if reaped != 0 {
            TX_TAIL.store(cursor.parts().1, Ordering::Relaxed);
            TX_INFLIGHT.store(0, Ordering::Relaxed);
            UART_TX.fetch_add(reaped as u32, Ordering::Relaxed);
        }
        tx_kick_regs(r, shared.tx_chan, shared.tx_base);
        TX_WAKER.wake();
    }
}

/// Arm the run from `TAIL` if the channel is idle; caller holds the mask.
fn tx_kick_regs(regs: pac::usart::Usart, tx_chan: VcomTxChannel, tx_base: u32) {
    let mut cursor = tx_cursor();
    let Some((tail, run)) = cursor.arm() else {
        return;
    };
    tx_chan.halt();
    TX_INFLIGHT.store(run, Ordering::Relaxed);
    regs.icr().write(|w| w.set_tc(true));
    regs.cr1().modify(|w| w.set_tcie(true));
    tx_chan.start_released(tx_base + tail as u32, run);
}

fn halt_for_baud(rx: VcomRxChannel, tx: VcomTxChannel, regs: pac::usart::Usart) {
    rx.halt();
    crate::gpdma::masked(|| {
        // A suspended burst can finish a latched UART byte and raise TC.
        regs.cr1().modify(|w| w.set_tcie(false));
        tx.halt();
        regs.icr().write(|w| w.set_tc(true));
        let mut cursor = tx_cursor();
        cursor.reset();
        let (head, tail, inflight) = cursor.parts();
        TX_HEAD.store(head, Ordering::Relaxed);
        TX_TAIL.store(tail, Ordering::Relaxed);
        TX_INFLIGHT.store(inflight, Ordering::Relaxed);
    });
}

/// The USART seam the shared bridge drives: a GPDMA ring on receive, GPDMA bursts on transmit.
pub struct StmVcom {
    /// Used only to reprogram the divisor.
    uart: Uart<'static, Blocking>,
    /// For the divisor read-back, the TC mask and the rejection undo.
    regs: pac::usart::Usart,
    /// Left disabled by the HAL's baud-rejection path.
    nvic: Interrupt,
    /// Kernel clock ahead of the USART's own prescaler.
    kernel_clock: u32,
    rx: DmaRxRing<VcomRxChannel>,
    /// A second handle to the channel, for the halt a baud change needs.
    rx_chan: VcomRxChannel,
    /// Read volatile through `rx`, never written here.
    mem: &'static [u8],
    /// Bytes lost to ring laps, saturating.
    rx_lost: u32,
    tx_chan: VcomTxChannel,
    /// Written only under the `TX_HEAD`/`TX_TAIL` guard.
    tx_base: u32,
}

impl StmVcom {
    /// Take over `uart`'s interrupt and two GPDMA channels; `regs`, `nvic` and `kernel_clock` must
    /// name that USART, its line bound to [`VcomInterruptHandler`]. Call once.
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
        // SAFETY: `lli` is exclusively owned here.
        unsafe {
            core::ptr::write_volatile(&mut lli.0[0], base);
            core::ptr::write_volatile(&mut lli.0[1], rx_llr_value(lli_addr).0);
        }

        // Claimed for ownership; registers go through `dma`/`ch`.
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
            // EIE is how frame/noise/overrun reach `service`: DMA cannot see those flags.
            w.set_dmar(true);
            w.set_dmat(true);
            w.set_eie(true);
        });
        // RXNEIE stays clear; TCIE is armed per burst.
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

    /// Publish the rate the USART's divisor chain produces, read back.
    fn record_realized_baud(&self) {
        let presc = PRESCALERS
            .get(usize::from(self.regs.presc().read().prescaler().to_bits()))
            .copied()
            .unwrap_or(1);
        let brr = u32::from(self.regs.brr().read().brr());
        // OVER8: the register holds twice the divisor, bit 3 zeroed.
        let divisor = match self.regs.cr1().read().over8() {
            Over8::OVERSAMPLING8 => ((brr & !0xf) >> 1) | (brr & 0x7),
            Over8::OVERSAMPLING16 => brr,
        };
        let clock = self.kernel_clock / presc;
        if let Some(realized) = (clock + divisor / 2).checked_div(divisor) {
            REALIZED_BAUD.store(realized, Ordering::Relaxed);
        }
    }

    /// One non-blocking pass over the ring, charging lap losses.
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

    /// Stage as much of `data` as fits; returns the bytes taken.
    fn tx_push(&self, data: &[u8]) -> usize {
        let mut cursor = tx_cursor();
        let head = cursor.parts().0;
        if data.is_empty() || cursor.free() == 0 {
            return 0;
        }
        // Copy before `TX_HEAD` moves, so a completion arms only published bytes.
        let taken = cursor.accept(data.len());
        // SAFETY: the bytes lie beyond `TX_HEAD` within the free count, so the channel is not on them.
        unsafe {
            let base = self.tx_base as *mut u8;
            core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(head), taken.first);
            if taken.second != 0 {
                core::ptr::copy_nonoverlapping(data.as_ptr().add(taken.first), base, taken.second);
            }
        }
        // Publish and kick under one mask, or a completion re-arms against a stale `TX_HEAD`.
        cortex_m::interrupt::free(|_| {
            TX_HEAD.store(cursor.parts().0, Ordering::Relaxed);
            self.tx_kick();
        });
        taken.taken
    }

    /// Wait for ring room, racing the fallback poll a baud-emptied ring needs.
    async fn tx_wait_room(&self) {
        loop {
            let woken = poll_fn(|cx| {
                TX_WAKER.register(cx.waker());
                if tx_cursor().free() != 0 {
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
    /// Ring-full from lap accounting; frame and overrun from the error interrupt.
    const SEAM_VALID: u32 = VCOM_VALID_RING_FULL
        | VCOM_VALID_FRAME_ERR
        | VCOM_VALID_OVERRUN
        | VCOM_VALID_UART_TX
        | VCOM_VALID_RX_DMA_ERR;
    /// Lower bounds: cursors alias whole laps, one pass can fold several errors, and a baud change drops TX uncounted.
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
            // Either channel: a baud change re-arms the receive ring across every such reset.
            rx_dma_err: DMA_FAULTS.load(Ordering::Relaxed),
        }
    }

    /// Stage `data` in the transmit ring; returns once queued, not once on the wire.
    async fn write_all(&mut self, data: &[u8]) {
        let mut sent = 0;
        while sent < data.len() {
            sent += self.tx_push(&data[sent..]);
            if sent < data.len() {
                self.tx_wait_room().await;
            }
        }
    }

    /// Resolves at wire-idle: empty-and-reaped is established only by TC.
    async fn drain_tx(&mut self) {
        loop {
            let idle = poll_fn(|cx| {
                TX_WAKER.register(cx.waker());
                let cursor = tx_cursor();
                if cursor.queued() == 0 && !cursor.is_armed() {
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

    /// Suspended, not reset: the counter stays readable and later bytes count as overruns.
    fn halt_rx(&mut self) {
        crate::gpdma::suspend(self.rx_chan.regs(), &DMA_FAULTS);
    }

    fn set_baudrate(&mut self, baud: u32) -> BaudApply {
        // Reject before stopping DMA or letting the HAL disable the USART.
        if !crate::baud::supported(self.kernel_clock, baud) {
            self.rx_chan.resume();
            return BaudApply::Rejected;
        }
        // Stop both channels first: a divisor change invalidates a running transfer's counter.
        halt_for_baud(self.rx_chan, self.tx_chan, self.regs);
        // Outside the mask: wakes a writer waiting on the freed room.
        TX_WAKER.wake();
        // An unrealizable rate leaves the previous divisor in force.
        let applied = self.uart.set_baudrate(baud).is_ok();
        if !applied {
            // The HAL's rejection path leaves UE and the NVIC line disabled; restore both.
            self.regs.cr1().modify(|w| w.set_ue(true));
            self.nvic.unpend();
            // SAFETY: the handler state has been published since construction.
            unsafe { self.nvic.enable() };
        }
        self.record_realized_baud();
        // Restart re-bases the cursor, closing any accumulated lap.
        self.rx.restart();
        if applied {
            BaudApply::Applied
        } else {
            BaudApply::Rejected
        }
    }
}

#[cfg(test)]
pub(crate) fn stop_test() {
    use crate::gpdma::tests::{registers, reset_count, script, trapped, uart};
    let dma = registers();
    let regs = uart();
    let rx = VcomRxChannel {
        dma,
        ch: 1,
        request: 2,
        base: 0x2000_0100,
        src: 0x4000_0000,
        lli: 0x2000_1000,
    };
    let tx = VcomTxChannel {
        dma,
        ch: 2,
        request: 3,
        dst: 0x4000_0004,
    };
    for failed_rx in [true, false] {
        TX_HEAD.store(23, Ordering::Relaxed);
        TX_TAIL.store(4, Ordering::Relaxed);
        TX_INFLIGHT.store(7, Ordering::Relaxed);
        script(rx.regs(), (true, !failed_rx, !failed_rx), None, false);
        script(tx.regs(), (true, false, false), None, false);
        let before = tx_cursor().parts();
        let faults = DMA_FAULTS.load(Ordering::Relaxed);
        trapped(|| halt_for_baud(rx, tx, regs));
        assert_eq!(tx_cursor().parts(), before);
        assert_eq!(DMA_FAULTS.load(Ordering::Relaxed), faults + 1);
        assert_eq!(reset_count(tx.regs()), 0);
        assert!(!rx.regs().cr().read().en());
        assert_eq!(regs.brr().read().brr(), 0);
    }
    let before = tx_cursor().parts();
    let sent = UART_TX.load(Ordering::Relaxed);
    unsafe { (regs.isr().as_ptr() as *mut u32).write_volatile(1 << 6) };
    trapped(|| {
        service(IrqShared {
            regs,
            tx_chan: tx,
            tx_base: 0x2000_2000,
        })
    });
    assert_eq!(tx_cursor().parts(), before);
    assert_eq!(UART_TX.load(Ordering::Relaxed), sent);
    TX_INFLIGHT.store(0, Ordering::Relaxed);
    let before = tx_cursor().parts();
    trapped(|| tx_kick_regs(regs, tx, 0x2000_2000));
    assert_eq!(tx_cursor().parts(), before);
    assert_eq!(tx.regs().sar().read(), 0);
    script(rx.regs(), (false, true, false), None, false);
    script(tx.regs(), (false, true, false), None, false);
    halt_for_baud(rx, tx, regs);
    assert_eq!(tx_cursor().parts(), (0, 0, 0));
    let mut rx = rx;
    rx.reload(RX_RING);
    assert_eq!(rx.regs().dar().read(), rx.base);
    assert!(rx.regs().cr().read().en());
    TX_HEAD.store(17, Ordering::Relaxed);
    tx_kick_regs(regs, tx, 0x2000_2000);
    assert_eq!(tx.regs().sar().read(), 0x2000_2000);
    assert_eq!(tx.regs().br1().read().bndt(), 17);
    assert_eq!(TX_INFLIGHT.load(Ordering::Relaxed), 17);
}
