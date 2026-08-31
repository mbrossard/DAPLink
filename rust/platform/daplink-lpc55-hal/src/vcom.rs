//! The VCOM UART: Flexcomm0 USART on PIO0_24 (RX) / PIO0_25 (TX), the MCU-Link's DUT UART pins.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use daplink_rs::FIFO_DRAIN_BUDGET;
use lpc55_hal as hal;
use lpc55_hal::drivers::pins::{Pio0_24, Pio0_25};
use lpc55_hal::drivers::Serial;
use lpc55_hal::prelude::*;
use lpc55_hal::raw;
use lpc55_hal::typestates::pin::state;
use lpc55_hal::Pin;

/// Handle to the Flexcomm0 USART transmitter and the receive ring's consumer half.
pub struct Vcom {
    /// The controller fills `mem`; the ring only tracks how far it has got.
    rx: daplink_rs::DmaRxRing<crate::dma_rx::Lpc55DmaRx>,
    mem: &'static [u8],
}

/// The rate [`program_vcom_baud`] last programmed, for the realized-baud vendor query.
static VCOM_REALIZED_BAUD: AtomicU32 = AtomicU32::new(0);
/// Nominal rate acknowledged only after FLEXCOMM0 programs its divisors.
static VCOM_APPLIED_BAUD: AtomicU32 = AtomicU32::new(0);
/// Latest nonzero line-coding rate awaiting application by FLEXCOMM0.
static VCOM_PENDING_BAUD: AtomicU32 = AtomicU32::new(0);

/// Bytes [`vcom_drain`] could not push into a full receive ring.
static RX_RING_FULL: AtomicU32 = AtomicU32::new(0);
/// Error-flagged frames [`vcom_drain`] popped and dropped.
static RX_FRAME_ERR: AtomicU32 = AtomicU32::new(0);
/// Receive-FIFO overrun episodes, charged once each.
static RX_OVERRUN: AtomicU32 = AtomicU32::new(0);
/// Whether the episode [`RX_OVERRUN`] last charged is still the one `RXERR` is showing.
static RX_OVERRUN_CHARGED: AtomicBool = AtomicBool::new(false);

/// Every place the receive path drops a byte, for the conservation ledger.
#[derive(Clone, Copy, Default)]
pub struct VcomRxDiscards {
    pub ring_full: u32,
    pub frame_err: u32,
    pub overrun: u32,
}

/// Read the receive-path discard counters.
pub fn vcom_rx_discards() -> VcomRxDiscards {
    VcomRxDiscards {
        ring_full: RX_RING_FULL.load(Ordering::Relaxed),
        frame_err: RX_FRAME_ERR.load(Ordering::Relaxed),
        overrun: RX_OVERRUN.load(Ordering::Relaxed),
    }
}

/// Bytes past the transmitter, free-running since boot; a floor, not a credit into the FIFO.
pub fn vcom_tx_sent() -> u32 {
    crate::dma_tx::sent()
}

/// Receive-channel errors the controller latched, each recovered by a re-arm.
pub fn vcom_rx_dma_errors() -> u32 {
    crate::dma_rx::VCOM_RX_DMA_ERR.load(Ordering::Relaxed)
}

/// Trace-channel errors the controller latched.
pub fn swo_dma_errors() -> u32 {
    crate::dma_swo::SWO_DMA_ERR.load(Ordering::Relaxed)
}

#[inline]
fn regs() -> &'static raw::usart0::RegisterBlock {
    // SAFETY: USART0 is owned by this seam; the transmitter is touched only from the USB task and
    // the receive registers only from `vcom_drain`.
    unsafe { &*raw::USART0::ptr() }
}

/// Reap a finished transmit and start the next staged one, from the DMA0 completion.
pub fn vcom_tx_service_irq() {
    crate::dma_tx::service();
}

/// FLEXCOMM0 interrupt body: the deferred baud change and the errors the controller cannot report.
pub fn vcom_service_irq() {
    let regs = regs();
    // Bytes arriving before the controller is armed hold the level interrupt asserted.
    if !crate::dma_rx::armed() {
        let mut n = 0;
        while n < FIFO_DRAIN_BUDGET && regs.fifostat.read().rxnotempty().bit_is_set() {
            let _ = regs.fiford.read();
            n += 1;
        }
    }
    observe_rx_errors();

    let drained = crate::dma_tx::drained();
    let pending = VCOM_PENDING_BAUD.load(Ordering::Relaxed);
    if drained && pending != 0 && regs.stat.read().txidle().bit_is_set() {
        program_vcom_baud(pending);
        VCOM_PENDING_BAUD
            .compare_exchange(pending, 0, Ordering::Relaxed, Ordering::Relaxed)
            .ok();
    }
    if drained {
        if VCOM_PENDING_BAUD.load(Ordering::Relaxed) == 0 && regs.stat.read().txidle().bit_is_set()
        {
            regs.intenclr.write(|w| w.txidleclr().set_bit());
        } else {
            regs.intenset.write(|w| w.txidleen().set_bit());
        }
    } else {
        regs.intenclr.write(|w| w.txidleclr().set_bit());
    }
}

/// Fold the receive path's status-level errors into the ledger. Call from a context that actually
/// runs: once the controller owns the FIFO, FLEXCOMM0 fires only for the deferred baud change.
pub fn observe_rx_errors() {
    let regs = regs();
    let pending = regs.intstat.read();
    if pending.framerrint().bit_is_set()
        || pending.parityerrint().bit_is_set()
        || pending.rxnoiseint().bit_is_set()
    {
        RX_FRAME_ERR.fetch_add(1, Ordering::Relaxed);
        regs.stat.write(|w| {
            w.framerrint()
                .set_bit()
                .parityerrint()
                .set_bit()
                .rxnoiseint()
                .set_bit()
        });
    }
    // Write-1-clear or the FIFO wedges; flush alongside it only while this seam still owns it.
    if regs.fifostat.read().rxerr().bit_is_set() {
        let owned = !crate::dma_rx::armed();
        if owned {
            // FIFOCFG is shared with the preempting FLEXCOMM0 handler, so its read-modify-write
            // is masked.
            cortex_m::interrupt::free(|_| {
                regs.fifocfg.modify(|_, w| w.emptyrx().set_bit());
            });
        }
        regs.fifostat.write(|w| w.rxerr().set_bit());
        // One charge per episode: the flush makes the clear stick, so a later set bit is a new
        // overrun; without one, only a pass that sees the flag clear reopens the latch.
        if !RX_OVERRUN_CHARGED.swap(!owned, Ordering::Relaxed) {
            RX_OVERRUN.fetch_add(1, Ordering::Relaxed);
        }
    } else {
        RX_OVERRUN_CHARGED.store(false, Ordering::Relaxed);
    }
}

/// Iterations to await RXIDLE: covers a frame at 9600 Bd and up; a slower wire loses its
/// in-flight frame, which the rate change was about to garble anyway.
const RX_IDLE_SPIN: u32 = 16_384;

/// Reprogram the Flexcomm0 baud divisors -- quiesce per UM11126 33.6.6, write the divisors,
/// then pulse CFG.ENABLE to restart the state machines on them; an unrepresentable rate leaves
/// the old one. Callers hold TX drained and idle.
// Outlining this shifts the boot path enough that the 150 MHz row stops enumerating on the bench.
#[inline(always)]
fn program_vcom_baud(baud: u32) -> bool {
    let Some((osr, brg)) =
        crate::baud::flexcomm_osr_brg(crate::baud::VCOM_CLOCK_HZ, baud, crate::baud::VCOM_OSR_MIN)
    else {
        return false;
    };
    let regs = regs();
    let mut spin = RX_IDLE_SPIN;
    while regs.stat.read().rxidle().bit_is_clear() && spin != 0 {
        spin -= 1;
    }
    // Written before the bracket: 33.6.1 permits only CFG writes while ENABLE is low, and a
    // divisor write inside the disabled window can be dropped, leaving the wire at the old rate.
    regs.brg
        .write(|w| unsafe { w.brgval().bits((brg - 1) as u16) });
    regs.osr
        .write(|w| unsafe { w.osrval().bits((osr - 1) as u8) });
    regs.cfg.modify(|_, w| w.enable().disabled());
    regs.cfg.modify(|_, w| w.enable().enabled());
    VCOM_REALIZED_BAUD.store(
        crate::baud::realized(crate::baud::VCOM_CLOCK_HZ, osr, brg),
        Ordering::Relaxed,
    );
    VCOM_APPLIED_BAUD.store(baud, Ordering::Release);
    true
}

/// The line rate the VCOM UART is actually programmed to (0 before the first programming).
pub fn vcom_realized_baud() -> u32 {
    VCOM_REALIZED_BAUD.load(Ordering::Relaxed)
}

/// The seam's answer to the `0x9E` query. FLEXCOMM0 defers a requested rate until the transmit
/// queue drains and `STAT.TXIDLE` is set, so pending is loaded first: a change starting between
/// the two loads then reads unsettled, never settled at the rate it is leaving.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    let pending = VCOM_PENDING_BAUD.load(Ordering::Acquire);
    daplink_rs::VcomBaudState::deferred(
        VCOM_REALIZED_BAUD.load(Ordering::Relaxed),
        VCOM_APPLIED_BAUD.load(Ordering::Relaxed),
        pending == 0,
    )
}

impl Vcom {
    /// Bring Flexcomm0 up as an 8N1 USART at `baud` with the per-byte RX FIFO interrupt armed on
    /// the FLEXCOMM0 NVIC line; call once per image. Returns the handle and its ring producer.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        flexcomm0: hal::peripherals::flexcomm::Flexcomm0,
        tx_pin: Pin<Pio0_25, state::Unused>,
        rx_pin: Pin<Pio0_24, state::Unused>,
        iocon: &mut hal::Iocon<hal::typestates::init_state::Enabled>,
        syscon: &mut hal::Syscon,
        clocks_token: &hal::typestates::ClocksSupportFlexcommToken,
        baud: u32,
    ) -> Self {
        let usart = flexcomm0.enabled_as_usart(syscon, clocks_token);

        // Rehome FC0's function clock off the HAL's 12 MHz FRO onto the Flexcomm source via FRG0.
        // SAFETY: single-threaded bring-up; the writes touch only FCCLKSEL0 and FLEXFRG0CTRL.
        let syscon_raw = unsafe { &*raw::SYSCON::ptr() };
        syscon_raw.fcclksel0().modify(|_, w| w.sel().enum_0x3());
        syscon_raw
            .flexfrg0ctrl()
            .write(|w| unsafe { w.div().bits(crate::clocks::FRG_DIV).mult().bits(0) });

        let tx = tx_pin.into_usart0_tx_pin(iocon);
        let rx = rx_pin.into_usart0_rx_pin(iocon);
        let config = hal::drivers::serial::config::Config::default().speed(baud.Hz());
        // No `Drop` glue: falling out of scope leaves every configured register as set.
        let _serial = Serial::new(usart, (tx, rx), config);
        // The HAL sized the divisors against its assumed 12 MHz kernel; redo them on the real one.
        let programmed = program_vcom_baud(baud);
        debug_assert!(programmed);

        let regs = regs();
        // Trigger at >= 1 entry: at the HAL's >= 2 default a lone final byte sits stranded.
        regs.fifotrig.modify(|_, w| unsafe { w.rxlvl().bits(0) });

        // Unmasked from boot, and DMA not armed until enumeration: either alone costs the attach.
        regs.fifointenset.write(|w| w.rxlvl().set_bit());
        let (chan, mem) = crate::dma_rx::Lpc55DmaRx::new();
        let rx = daplink_rs::DmaRxRing::new(chan, mem.len());
        Vcom { rx, mem }
    }

    /// Pop up to `buf.len()` received bytes out of what the controller has written; bytes a lap
    /// overwrote unread are charged here, the only place that loss can be seen.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        let lost = self.rx.observe();
        if lost != 0 {
            RX_RING_FULL.fetch_add(lost as u32, Ordering::Relaxed);
        }
        let (n, lost) = self.rx.read(self.mem, buf);
        if lost != 0 {
            RX_RING_FULL.fetch_add(lost as u32, Ordering::Relaxed);
        }
        n
    }

    /// Sample the channel without shipping anything. A pass that finds the staging full must still
    /// do this: skipping it for a whole buffer loses the cursor rather than merely delaying it.
    pub fn service_rx(&mut self) {
        self.read(&mut []);
    }

    /// Bring the receive channel up, or back up after an error quiesced it. Deferred rather than
    /// done at boot: enabling the DMA controller before the host has finished enumerating costs
    /// this probe its attach.
    pub fn start_rx_dma(&mut self) {
        if crate::dma_rx::armed() {
            return;
        }
        crate::dma::init();
        crate::dma_rx::arm();
        crate::dma_tx::start();
        self.rx.reset_cursor();
    }

    /// Capacity available for a whole CDC OUT packet.
    pub fn write_capacity(&self) -> usize {
        crate::dma_tx::capacity()
    }

    /// Apply the rate now if the line is already quiet, and otherwise ask FLEXCOMM0 to apply it
    /// once the transmit queue and shifter are. The quiet case must not be deferred as well.
    pub fn request_baud(&mut self, baud: u32) {
        if baud == 0 || VCOM_APPLIED_BAUD.load(Ordering::Acquire) == baud {
            return;
        }
        let regs = regs();
        // Published before the attempt, not after it fails: clearing pending first left a window
        // where a host reading 0x9E saw the old rate with nothing marked in flight.
        VCOM_PENDING_BAUD.store(baud, Ordering::Release);
        if crate::dma_tx::drained()
            && regs.stat.read().txidle().bit_is_set()
            && program_vcom_baud(baud)
        {
            VCOM_PENDING_BAUD
                .compare_exchange(baud, 0, Ordering::Release, Ordering::Relaxed)
                .ok();
            return;
        }
        regs.intenset.write(|w| w.txidleen().set_bit());
        cortex_m::peripheral::NVIC::pend(raw::Interrupt::FLEXCOMM0);
    }

    /// Whether FLEXCOMM0 has acknowledged programming this nominal rate.
    pub fn baud_applied(&self, baud: u32) -> bool {
        VCOM_APPLIED_BAUD.load(Ordering::Acquire) == baud
    }

    /// Stage bytes for the controller to transmit.
    pub fn write(&mut self, data: &[u8]) -> usize {
        crate::dma_tx::write(data)
    }
}
