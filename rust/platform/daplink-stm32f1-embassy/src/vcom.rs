//! CDC-ACM VCOM seam with DMA on both legs; nothing here ever reads the USART data register.

use core::sync::atomic::{AtomicU32, Ordering};
use daplink_embassy_usb::{SeamStats, VcomUart};
use daplink_rs::daplink_core::{CDC_PACKET_SIZE, DEFAULT_VCOM_BAUD};
use daplink_rs::{
    vcom_grade, DmaRxRing, VCOM_GRADE_LOWER_BOUND, VCOM_SLOT_RING_FULL, VCOM_SLOT_UART_TX,
    VCOM_VALID_RING_FULL, VCOM_VALID_UART_TX,
};
use embassy_stm32::dma::{Channel, ChannelInstance, Transfer, TransferOptions};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::interrupt::{Interrupt, InterruptExt};
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{DMA1_CH6, DMA1_CH7, PA2, PA3, USART2};
use embassy_stm32::usart::{Config, UartTx};
use embassy_stm32::Peri;
use embassy_time::{Duration, Timer};
use static_cell::ConstStaticCell;

use crate::dma::BdmaRx;

/// Re-check period for an empty receive ring; a ring holding bytes never waits.
const RX_POLL: Duration = Duration::from_micros(500);

/// Transmit staging: one CDC packet, which is the largest the bridge ever hands over at once.
const TX_STAGE: usize = CDC_PACKET_SIZE as usize;

/// Baud rate represented by the USART divisor currently on the wire.
static REALIZED_BAUD: AtomicU32 = AtomicU32::new(DEFAULT_VCOM_BAUD);

/// Baud a `BRR` divisor yields off the kernel clock, rounded to nearest: the reply describes the
/// wire, not the divide that reached it. `brr` must be nonzero.
const fn realized_baud(kernel_hz: u32, brr: u32) -> u32 {
    (kernel_hz + brr / 2) / brr
}

/// USART2 runs off PCLK1, 36 MHz on this board; these are the rounding's two directions and an
/// exact rung.
const _: () = {
    assert!(realized_baud(36_000_000, 312) == 115_385);
    assert!(realized_baud(36_000_000, 39) == 923_077);
    assert!(realized_baud(36_000_000, 7) == 5_142_857);
    assert!(realized_baud(36_000_000, 36) == 1_000_000);
};

/// Return the baud rate represented by the last divisor successfully programmed.
pub fn vcom_realized_baud() -> u32 {
    REALIZED_BAUD.load(Ordering::Relaxed)
}

/// The seam's answer to the `0x9E` query: the realized rate plus the bridge's applied-ness.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    daplink_embassy_usb::vcom_baud_state(vcom_realized_baud())
}

/// The STM32F1 CDC-ACM VCOM seam: DMA on both legs, driven by
/// [`vcom_bridge`](daplink_embassy_usb::vcom_bridge).
pub struct DmaVcom {
    /// Owns the USART handle and the baud-rate setter; the bytes go out over `chan`, not here.
    tx: UartTx<'static, Blocking>,
    /// The USART registers, kept to undo the HAL's disable when it rejects a rate.
    regs: pac::usart::Usart,
    /// The USART kernel clock, sampled from RCC at bring-up.
    kernel_hz: u32,
    /// This USART's NVIC line, re-disabled after every baud change (no handler is bound).
    nvic: Interrupt,
    /// The receive ring over the RX DMA channel.
    rx: DmaRxRing<BdmaRx>,
    /// The buffer the channel fills; read volatile through the ring, never written here.
    mem: &'static [u8],
    /// Bytes a lap overwrote before the bridge drained them, saturating.
    lost: u32,
    /// Bytes of transfers already reaped; the packet still on the wire is not among them.
    sent: u32,
    /// The transmit channel, armed one staged packet at a time.
    chan: Channel<'static>,
    /// The packet on the wire, kept past the call that armed it so the bridge takes the next one
    /// from the host while this one is still going out.
    pending: Option<Transfer<'static>>,
    /// Bytes `pending` carries, charged to `sent` when it is reaped.
    inflight: usize,
    /// The staging buffer, written only while `pending` is `None`.
    stage: &'static mut [u8; TX_STAGE],
    /// The receive pin: a plain input, since a USART RX pad on this family takes no AF selection.
    _rx_pin: Input<'static>,
}

impl DmaVcom {
    /// Bring up the VCOM seam on USART2 (TX = PA2, RX = PA3): TX on DMA1 CH7, RX ring on DMA1
    /// CH6 (fixed in silicon, RM0008 table 78). The receive channel must stay unbound.
    pub fn new_usart2(
        usart: Peri<'static, USART2>,
        tx_pin: Peri<'static, PA2>,
        rx_pin: Peri<'static, PA3>,
        tx_dma: Peri<'static, DMA1_CH7>,
        rx_dma: Peri<'static, DMA1_CH6>,
        irq: impl Binding<
                <DMA1_CH7 as ChannelInstance>::Interrupt,
                embassy_stm32::dma::InterruptHandler<DMA1_CH7>,
            > + 'static,
        rx_buf: &'static mut [u8],
    ) -> Self {
        // Consumed so nothing else can drive the channel.
        let _ = rx_dma;
        let tx = UartTx::new_blocking(usart, tx_pin, vcom_config())
            .expect("the VCOM USART configuration is valid");
        // USART2_RX is DMA1 channel 6, i.e. index 5.
        Self::assemble(
            tx,
            Channel::new(tx_dma, irq),
            pac::USART2,
            embassy_stm32::rcc::frequency::<USART2>().0,
            Interrupt::USART2,
            Input::new(rx_pin, Pull::None),
            5,
            rx_buf,
        )
    }

    /// Enable the receiver, program the circular receive transfer, and assemble the seam.
    #[allow(clippy::too_many_arguments)]
    fn assemble(
        tx: UartTx<'static, Blocking>,
        tx_chan: Channel<'static>,
        usart: pac::usart::Usart,
        kernel_hz: u32,
        nvic: Interrupt,
        rx_pin: Input<'static>,
        ch: usize,
        rx_buf: &'static mut [u8],
    ) -> Self {
        // The transmit-only constructor left the receiver disabled.
        usart.cr1().modify(|w| w.set_re(true));

        let mem: &'static [u8] = rx_buf;
        // Lap ticks on: a starved span of whole rings must not fold to an empty backlog.
        let chan = BdmaRx::new(ch, usart.dr().as_ptr() as u32, mem).with_lap_ticks();
        chan.start();
        // The transmit-only constructor left `DMAT` clear, since this seam drives its own channel.
        usart.cr3().modify(|w| {
            w.set_dmar(true);
            w.set_dmat(true);
        });

        static STAGE: ConstStaticCell<[u8; TX_STAGE]> = ConstStaticCell::new([0; TX_STAGE]);
        let vcom = Self {
            tx,
            regs: usart,
            kernel_hz,
            nvic,
            rx: DmaRxRing::new(chan, mem.len()),
            mem,
            lost: 0,
            sent: 0,
            chan: tx_chan,
            pending: None,
            inflight: 0,
            stage: STAGE.take(),
            _rx_pin: rx_pin,
        };
        vcom.record_realized_baud();
        vcom
    }

    /// Publish the rate the divisor currently in `BRR` produces off the kernel clock.
    fn record_realized_baud(&self) {
        let divisor = u32::from(self.regs.brr().read().brr());
        if divisor != 0 {
            REALIZED_BAUD.store(realized_baud(self.kernel_hz, divisor), Ordering::Relaxed);
        }
    }

    /// Bytes the receive ring lost to a lap since boot, saturating.
    pub fn lost(&self) -> u32 {
        self.lost
    }

    /// Arm the transmit channel over the first `n` staged bytes, leaving the transfer pending.
    fn arm(&mut self, n: usize) {
        let src = core::ptr::slice_from_raw_parts(self.stage.as_ptr(), n);
        let dst = self.regs.dr().as_ptr() as *mut u8;
        self.inflight = n;
        // SAFETY: `pending` is `None`, so nothing else drives the channel or reads the staging
        // bytes; the transfer holds the channel by value, so it outlives this borrow of `self`.
        self.pending = Some(unsafe {
            (*core::ptr::addr_of_mut!(self.chan)).write_raw(
                (),
                src,
                dst,
                TransferOptions::default(),
            )
        });
    }

    /// Fold a finished transfer's bytes into the ledger.
    fn charge(&mut self) {
        self.sent = self.sent.wrapping_add(self.inflight as u32);
        self.inflight = 0;
    }

    /// Wait out the pending transfer, so the staging buffer is the CPU's again.
    async fn reap(&mut self) {
        if let Some(transfer) = self.pending.take() {
            transfer.await;
            self.charge();
        }
    }

    /// Charge the pending transfer if it has already finished; a running one is left alone.
    fn reap_ready(&mut self) {
        if self.pending.as_mut().is_some_and(|t| !t.is_running()) {
            if let Some(transfer) = self.pending.take() {
                transfer.blocking_wait();
                self.charge();
            }
        }
    }

    /// One non-blocking pass over the ring, charging what it lost to the ledger.
    fn drain(&mut self, buf: &mut [u8]) -> usize {
        // The only pass an idle bridge still makes, so the last packet is charged without traffic.
        self.reap_ready();
        let lost = self.rx.observe();
        self.lost = self.lost.saturating_add(lost as u32);
        let (n, lost) = self.rx.read(self.mem, buf);
        self.lost = self.lost.saturating_add(lost as u32);
        n
    }
}

/// The fixed VCOM framing: 8N1 at the fleet default rate, which the bridge then follows.
fn vcom_config() -> Config {
    let mut cfg = Config::default();
    cfg.baudrate = DEFAULT_VCOM_BAUD;
    cfg
}

impl VcomUart for DmaVcom {
    /// The ring's own lap accounting; a seam that never reads `DR` reaches no USART error flag.
    const SEAM_VALID: u32 = VCOM_VALID_RING_FULL | VCOM_VALID_UART_TX;
    /// Lower bounds: the sticky reload flag closes exactly one ring per drain, and a packet is
    /// charged only once its transfer completes at `DR`.
    const SEAM_GRADE: u32 = vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_LOWER_BOUND)
        | vcom_grade(VCOM_SLOT_UART_TX, VCOM_GRADE_LOWER_BOUND);
    const SEAM_UNITS: u32 = 0;

    /// Cancel-safe: bytes leave the ring only in a synchronous drain never crossed by an await.
    async fn read(&mut self, buf: &mut [u8]) -> usize {
        loop {
            let n = self.drain(buf);
            if n > 0 {
                return n;
            }
            Timer::after(RX_POLL).await;
        }
    }

    /// The same drain without the wait.
    fn read_ready(&mut self, buf: &mut [u8]) -> usize {
        self.drain(buf)
    }

    fn seam_stats(&self) -> SeamStats {
        SeamStats {
            ring_full: self.lost,
            uart_tx: self.sent,
            ..SeamStats::default()
        }
    }

    /// One DMA transfer per CDC packet, returning once the bytes are staged rather than once they
    /// are on the wire: the bridge then reads the next packet during this one's transit, which is
    /// what keeps the USB turnaround off the UART line. Ordering is the single slot's.
    async fn write_all(&mut self, data: &[u8]) {
        for chunk in data.chunks(TX_STAGE) {
            self.reap().await;
            self.stage[..chunk.len()].copy_from_slice(chunk);
            self.arm(chunk.len());
        }
    }

    /// DMA completion stops at `DR`, up to two frames short of the wire; `TC` covers the shifter,
    /// and a `DR` write clears it, so a set flag means the last frame is out.
    async fn drain_tx(&mut self) {
        self.reap().await;
        while !self.regs.sr().read().tc() {
            Timer::after(RX_POLL).await;
        }
    }

    fn set_baudrate(&mut self, baud: u32) {
        // Anything still staged carries the old framing; the bridge drains to wire-idle first.
        self.pending = None;
        self.inflight = 0;
        let applied = self.tx.set_baudrate(baud).is_ok();
        // `UartTx::set_baudrate` NVIC-enables the line; this seam binds no handler, so undo it.
        self.nvic.disable();
        if applied {
            self.record_realized_baud();
            // Restart discards the old framing and re-bases the cursor, closing any accumulated lap.
            self.rx.restart();
        } else {
            // A rejected rate leaves UE cleared; restore it or the next TX DMA write never lands.
            self.regs.cr1().modify(|w| w.set_ue(true));
        }
    }
}
