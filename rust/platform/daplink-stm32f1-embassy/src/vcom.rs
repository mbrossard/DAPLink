//! CDC-ACM VCOM seam with DMA on both legs.

use core::sync::atomic::{AtomicU32, Ordering};
use daplink_embassy_usb::{BaudApply, SeamStats, VcomUart};
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

/// Transmit staging: one CDC packet, the most the bridge hands over at once.
const TX_STAGE: usize = CDC_PACKET_SIZE as usize;

/// Baud rate represented by the USART divisor currently on the wire.
static REALIZED_BAUD: AtomicU32 = AtomicU32::new(DEFAULT_VCOM_BAUD);

/// Baud a nonzero `BRR` divisor yields, rounded to nearest.
const fn realized_baud(kernel_hz: u32, brr: u32) -> u32 {
    (kernel_hz + brr / 2) / brr
}

/// USART2 runs off a 36 MHz PCLK1 on this board.
const _: () = {
    assert!(realized_baud(36_000_000, 312) == 115_385);
    assert!(realized_baud(36_000_000, 39) == 923_077);
    assert!(realized_baud(36_000_000, 7) == 5_142_857);
    assert!(realized_baud(36_000_000, 36) == 1_000_000);
};

pub fn vcom_realized_baud() -> u32 {
    REALIZED_BAUD.load(Ordering::Relaxed)
}

/// The `0x9E` query's answer: the realized rate plus whether it was applied.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    daplink_embassy_usb::vcom_baud_state(vcom_realized_baud())
}

/// The STM32F1 CDC-ACM VCOM seam, driven by [`vcom_bridge`](daplink_embassy_usb::vcom_bridge).
pub struct DmaVcom {
    /// Owns the USART and its baud setter; bytes go out over `chan`.
    tx: UartTx<'static, Blocking>,
    /// Kept to undo the HAL's disable when it rejects a rate.
    regs: pac::usart::Usart,
    kernel_hz: u32,
    /// Re-disabled after every baud change; no handler is bound.
    nvic: Interrupt,
    rx: DmaRxRing<BdmaRx>,
    /// Read volatile through the ring, never written here.
    mem: &'static [u8],
    /// Bytes lost to ring laps, saturating.
    lost: u32,
    // TX lease: `stage` is rewritten only with no `pending` transfer; `inflight` is charged once.
    /// Bytes of reaped transfers.
    sent: u32,
    chan: Channel<'static>,
    /// The packet on the wire, so the bridge fetches the next one meanwhile.
    pending: Option<Transfer<'static>>,
    inflight: usize,
    /// Written only while `pending` is `None`.
    stage: &'static mut [u8; TX_STAGE],
    /// A plain input: an RX pad on this family takes no AF selection.
    _rx_pin: Input<'static>,
}

impl DmaVcom {
    /// VCOM on USART2 (PA2/PA3), TX on DMA1 CH7, RX ring on DMA1 CH6; the RX channel stays unbound.
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
        let _ = rx_dma;
        let tx = UartTx::new_blocking(usart, tx_pin, vcom_config())
            .expect("the VCOM USART configuration is valid");
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
        usart.cr1().modify(|w| w.set_re(true));

        let mem: &'static [u8] = rx_buf;
        // Lap ticks on: a starved span of whole rings must not fold to an empty backlog.
        let chan = BdmaRx::new(ch, usart.dr().as_ptr() as u32, mem).with_lap_ticks();
        chan.start();
        // The transmit-only constructor left `DMAT` clear.
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

    fn record_realized_baud(&self) {
        let divisor = u32::from(self.regs.brr().read().brr());
        if divisor != 0 {
            REALIZED_BAUD.store(realized_baud(self.kernel_hz, divisor), Ordering::Relaxed);
        }
    }

    pub fn lost(&self) -> u32 {
        self.lost
    }

    fn arm(&mut self, n: usize) {
        let src = core::ptr::slice_from_raw_parts(self.stage.as_ptr(), n);
        let dst = self.regs.dr().as_ptr() as *mut u8;
        self.inflight = n;
        // SAFETY: `pending` is `None`, so nothing else uses the channel or staging bytes.
        self.pending = Some(unsafe {
            (*core::ptr::addr_of_mut!(self.chan)).write_raw(
                (),
                src,
                dst,
                TransferOptions::default(),
            )
        });
    }

    fn charge(&mut self) {
        self.sent = self.sent.wrapping_add(self.inflight as u32);
        self.inflight = 0;
    }

    async fn reap(&mut self) {
        if let Some(transfer) = self.pending.take() {
            transfer.await;
            self.charge();
        }
    }

    /// Charge the pending transfer only if it already finished.
    fn reap_ready(&mut self) {
        if self.pending.as_mut().is_some_and(|t| !t.is_running()) {
            if let Some(transfer) = self.pending.take() {
                // A finished transfer still holds the channel until it is waited on.
                transfer.blocking_wait();
                self.charge();
            }
        }
    }

    fn drain(&mut self, buf: &mut [u8]) -> usize {
        // An idle bridge still drains, so the last packet is charged without traffic.
        self.reap_ready();
        let lost = self.rx.observe();
        self.lost = self.lost.saturating_add(lost as u32);
        let (n, lost) = self.rx.read(self.mem, buf);
        self.lost = self.lost.saturating_add(lost as u32);
        n
    }
}

/// 8N1 at the default rate; the bridge then follows the host.
fn vcom_config() -> Config {
    let mut cfg = Config::default();
    cfg.baudrate = DEFAULT_VCOM_BAUD;
    cfg
}

impl VcomUart for DmaVcom {
    /// A seam that never reads `DR` sees no USART error flag.
    const SEAM_VALID: u32 = VCOM_VALID_RING_FULL | VCOM_VALID_UART_TX;
    /// Lower bounds: one ring per drain, and a packet counts only once its transfer completes.
    const SEAM_GRADE: u32 = vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_LOWER_BOUND)
        | vcom_grade(VCOM_SLOT_UART_TX, VCOM_GRADE_LOWER_BOUND);
    const SEAM_UNITS: u32 = 0;

    /// Cancel-safe: bytes leave the ring only in a synchronous drain.
    async fn read(&mut self, buf: &mut [u8]) -> usize {
        loop {
            let n = self.drain(buf);
            if n > 0 {
                return n;
            }
            Timer::after(RX_POLL).await;
        }
    }

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

    /// Returns once staged, so the bridge reads the next packet during this one's transit.
    async fn write_all(&mut self, data: &[u8]) {
        for chunk in data.chunks(TX_STAGE) {
            self.reap().await;
            self.stage[..chunk.len()].copy_from_slice(chunk);
            self.arm(chunk.len());
        }
    }

    /// DMA completion stops at `DR`; `TC` covers the shifter.
    async fn drain_tx(&mut self) {
        self.reap().await;
        while !self.regs.sr().read().tc() {
            Timer::after(RX_POLL).await;
        }
    }

    /// The receiver stops; the still-running ring channel keeps what it already took.
    fn halt_rx(&mut self) {
        self.regs.cr1().modify(|w| w.set_re(false));
    }

    fn set_baudrate(&mut self, baud: u32) -> BaudApply {
        if baud == 0 {
            self.regs.cr1().modify(|w| w.set_re(true));
            return BaudApply::Rejected;
        }
        // Anything staged carries the old framing; the bridge drains to idle first.
        self.pending = None;
        self.inflight = 0;
        let applied = self.tx.set_baudrate(baud).is_ok();
        // `set_baudrate` NVIC-enables the line; no handler is bound, so undo it.
        self.nvic.disable();
        if applied {
            self.record_realized_baud();
            // Restart re-bases the cursor, closing any accumulated lap.
            self.rx.restart();
            self.regs.cr1().modify(|w| w.set_re(true));
            BaudApply::Applied
        } else {
            // A rejected rate leaves UE cleared; restore it or TX DMA never lands.
            self.regs.cr1().modify(|w| {
                w.set_ue(true);
                w.set_re(true);
            });
            BaudApply::Rejected
        }
    }
}
