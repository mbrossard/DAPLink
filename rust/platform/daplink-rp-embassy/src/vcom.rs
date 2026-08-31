//! RP halves of the shared USB stack: the driver alias, the device task, and the VCOM seam.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::dma_rx::{self, DmaVcomRx};
use crate::dma_tx;
use daplink_embassy_usb::{vcom_bridge, SeamStats, VcomUart};
use daplink_rs::daplink_core::DEFAULT_VCOM_BAUD;
use daplink_rs::{
    vcom_grade, DmaRingCursor, VCOM_GRADE_EXACT, VCOM_GRADE_LOWER_BOUND, VCOM_SLOT_FRAME_ERR,
    VCOM_SLOT_OVERRUN, VCOM_SLOT_RING_FULL, VCOM_SLOT_UART_TX, VCOM_VALID_FRAME_ERR,
    VCOM_VALID_OVERRUN, VCOM_VALID_RING_FULL, VCOM_VALID_UART_TX,
};
use embassy_rp::clocks::clk_peri_freq;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::pac;
use embassy_rp::peripherals::USB;
use embassy_rp::uart::BufferedUart;
use embassy_rp::Peri;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{ControlChanged, Receiver, Sender};
use embassy_usb::UsbDevice;

/// Concrete USB driver type for the RP-family USB controller.
pub type UsbDrv = embassy_rp::usb::Driver<'static, USB>;

/// Baud rate represented by the PL011 divisor currently on the wire.
static REALIZED_BAUD: AtomicU32 = AtomicU32::new(DEFAULT_VCOM_BAUD);

/// Return the baud rate represented by the last divisor programmed.
pub fn vcom_realized_baud() -> u32 {
    REALIZED_BAUD.load(Ordering::Relaxed)
}

/// The seam's answer to the `0x9E` query: the realized rate plus the bridge's applied-ness.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    daplink_embassy_usb::vcom_baud_state(vcom_realized_baud())
}

/// Publish the rate the divisor pair the PL011 holds produces, read back not recomputed.
fn record_realized_baud() {
    let regs = pac::UART1;
    let dx64 = (u32::from(regs.uartibrd().read().baud_divint()) << 6)
        | u32::from(regs.uartfbrd().read().baud_divfrac());
    let clk = clk_peri_freq();
    // An unprogrammed divisor or a stopped clock leaves the previous report standing.
    if dx64 != 0 && clk != 0 {
        REALIZED_BAUD.store(pl011_baud::realized(clk, dx64), Ordering::Relaxed);
    }
}

/// Drive the USB device: enumeration, control transfers, and endpoint FIFOs.
#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, UsbDrv>) -> ! {
    usb.run().await
}

/// `UARTDR` overrun flag, one byte above the data (RP2040 section 4.2.8, RP2350 section 12.1.8).
const UARTDR_OE: u16 = 1 << (8 + 3);
/// Framing, parity and break -- the flags that condemn the character; `OE` is deliberately out.
const UARTDR_FRAME_MASK: u16 = 0x7 << 8;

/// Nap between ring checks: no interrupt backs this seam, so a bounded poll is what sees bytes.
const RX_POLL: Duration = Duration::from_micros(500);

/// DMA rings over UART1 in both directions, wrapped as the [`VcomUart`] seam the bridge drives.
pub struct RpVcom {
    /// Kept only for its divisor arithmetic; its interrupt path serves neither direction under DMA.
    handle: BufferedUart,
    cursor: DmaRingCursor,
    chan: DmaVcomRx,
    /// The channel's production total as of the last drain, so each drain folds in a difference.
    produced: u32,
    mem: &'static [u16],
    stats: SeamStats,
}

impl RpVcom {
    /// Take `uart` over: silence its buffered interrupt and start both DMA rings.
    pub fn new<R: ChannelInstance, C: ChannelInstance, T: ChannelInstance>(
        uart: BufferedUart,
        rx_dma: Peri<'static, R>,
        chain_dma: Peri<'static, C>,
        tx_dma: Peri<'static, T>,
    ) -> Self {
        let regs = pac::UART1;
        regs.uartimsc().write(|_w| {});
        regs.uartdmacr().write(|w| {
            w.set_rxdmae(true);
            w.set_txdmae(true);
        });
        let (chan, mem) = dma_rx::new(regs.uartdr().as_ptr() as u32, rx_dma, chain_dma);
        dma_tx::new(regs.uartdr().as_ptr() as u32, tx_dma);
        Self {
            handle: uart,
            cursor: DmaRingCursor::new(dma_rx::RX_RING_LEN),
            chan,
            produced: 0,
            mem,
            stats: SeamStats::default(),
        }
    }

    /// Fold production into the cursor and drain the ring into `buf`, charging losses to their
    /// sites. Synchronous, and the only place bytes leave the ring.
    fn drain(&mut self, buf: &mut [u8]) -> usize {
        let produced = self.chan.produced();
        // Production cannot run backwards: a difference in the top half is a fault, not a stall.
        let advanced = match produced.wrapping_sub(self.produced) {
            d if d > u32::MAX / 2 => 0,
            d => d as usize,
        };
        self.produced = produced;
        let lost = self.cursor.observe_produced(advanced);
        self.stats.ring_full = self.stats.ring_full.wrapping_add(lost as u32);
        let mut n = 0;
        while n < buf.len() {
            let Some(idx) = self.cursor.next_index() else {
                break;
            };
            // SAFETY: `idx` is behind the write position, so the entry is committed; volatile
            // because the controller's write is invisible to alias analysis.
            let entry = unsafe { core::ptr::read_volatile(&self.mem[idx]) };
            // `OE` marks a byte lost before this one, which is itself valid; both can be set.
            if entry & UARTDR_OE != 0 {
                self.stats.overrun = self.stats.overrun.wrapping_add(1);
            }
            if entry & UARTDR_FRAME_MASK != 0 {
                self.stats.frame_err = self.stats.frame_err.wrapping_add(1);
            } else {
                buf[n] = entry as u8;
                n += 1;
            }
        }
        n
    }
}

impl VcomUart for RpVcom {
    /// Every bit is counted: laps make `ring_full` exact, and each entry carries its own flags.
    const SEAM_VALID: u32 =
        VCOM_VALID_RING_FULL | VCOM_VALID_FRAME_ERR | VCOM_VALID_OVERRUN | VCOM_VALID_UART_TX;
    /// An `OE` entry proves at least one lost byte and `uart_tx` is credited at `UARTDR`;
    /// everything else is counted at its site.
    const SEAM_GRADE: u32 = vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_EXACT)
        | vcom_grade(VCOM_SLOT_FRAME_ERR, VCOM_GRADE_EXACT)
        | vcom_grade(VCOM_SLOT_OVERRUN, VCOM_GRADE_LOWER_BOUND)
        | vcom_grade(VCOM_SLOT_UART_TX, VCOM_GRADE_LOWER_BOUND);
    const SEAM_UNITS: u32 = 0;

    /// Cancel-safe: bytes leave the ring only in a synchronous drain, never across the await.
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
            uart_tx: dma_tx::sent(),
            ..self.stats
        }
    }

    async fn write_all(&mut self, data: &[u8]) {
        dma_tx::write_all(data).await
    }

    /// Ring drained first; `BUSY` then covers the PL011's FIFO and shifter out to wire-idle.
    async fn drain_tx(&mut self) {
        dma_tx::drain().await;
        while pac::UART1.uartfr().read().busy() {
            Timer::after(RX_POLL).await;
        }
    }

    fn set_baudrate(&mut self, baud: u32) {
        // Reconfiguring a busy channel leaves its counter no longer describing what was written.
        self.chan.halt();
        // Empty after the bridge's drain; the halt still parks the channel across the divisor write.
        dma_tx::halt_for_baud_change();
        self.handle.set_baudrate(baud);
        record_realized_baud();
        self.chan.start();
        // The restart zeroes the epoch, so the cursor base and the total must go back with it.
        self.cursor.reset();
        self.produced = 0;
    }
}

/// Bridge the CDC-ACM VCOM port to the wired UART, following the host's line-coding baud.
#[embassy_executor::task]
pub async fn vcom_task(
    mut cdc_tx: Sender<'static, UsbDrv>,
    mut cdc_rx: Receiver<'static, UsbDrv>,
    control: ControlChanged<'static>,
    mut uart: RpVcom,
    applied_baud: u32,
    led: &'static mut dyn daplink_rs::Activity,
) -> ! {
    // The constructor programmed this rate, so the divisor it left is what to report.
    record_realized_baud();
    vcom_bridge(
        &mut cdc_tx,
        &mut cdc_rx,
        &control,
        &mut uart,
        applied_baud,
        led,
    )
    .await
}

#[cfg(test)]
mod tests {
    /// The divisor pair `embassy-rp` 0.10.0 leaves behind: its split lets `fbrd` reach 64, and
    /// the 6-bit register field keeps only the low bits.
    fn held_dx64(clk: u32, baud: u32) -> u32 {
        let div = (8 * clk) / baud;
        let (ibrd, fbrd) = match (div >> 7, (div & 0x7f).div_ceil(2)) {
            (0, _) => (1, 0),
            (int_part, _) if int_part >= 65535 => (65535, 0),
            (int_part, frac_part) => (int_part, frac_part),
        };
        (ibrd << 6) | (fbrd & 0x3f)
    }

    fn report(clk: u32, baud: u32) -> u32 {
        pl011_baud::realized(clk, held_dx64(clk, baud))
    }

    #[test]
    fn overflow_band_reports_the_wire_rate() {
        // RP2040 at 125 MHz: every request in 2604167..=2610966 leaves fbrd = 64, held as 0.
        assert_eq!(report(125_000_000, 2_604_167), 3_906_250);
        assert_eq!(report(125_000_000, 2_610_966), 3_906_250);
        // RP2350 at 150 MHz: requests 3125001..=3133159 are the same band.
        assert_eq!(report(150_000_000, 3_125_001), 4_687_500);
        assert_eq!(report(150_000_000, 3_133_159), 4_687_500);
    }

    #[test]
    fn overflow_with_odd_ibrd_holds_the_carried_divisor() {
        // fbrd = 64 with ibrd odd: bit 6 of the x64 divisor is already set, so the held pair
        // equals the carried value and the report tracks the request.
        assert_eq!(report(125_000_000, 1_956_947), 2_604_167);
    }

    #[test]
    fn saturation_ends() {
        assert_eq!(report(125_000_000, u32::MAX), 7_812_500);
        // 500e6 / (65535 << 6) = 119.21, the slowest rate the widest divisor reaches.
        assert_eq!(report(125_000_000, 100), 119);
    }
}
