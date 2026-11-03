//! The VCOM UART: Flexcomm0 USART on PIO0_24 (RX) / PIO0_25 (TX), the MCU-Link's DUT UART pins.

use core::sync::atomic::{AtomicU32, Ordering};

#[path = "rx_errors.rs"]
mod rx_errors;

#[path = "baud_change.rs"]
mod baud_change;

use daplink_rs::FIFO_DRAIN_BUDGET;
use lpc55_hal as hal;
use lpc55_hal::drivers::pins::{Pio0_24, Pio0_25};
use lpc55_hal::drivers::Serial;
use lpc55_hal::prelude::*;
use lpc55_hal::raw;
use lpc55_hal::typestates::pin::state;
use lpc55_hal::Pin;

/// Handle to the Flexcomm0 USART transmitter and the receive ring's consumer half.
/// Arm receive with [`Vcom::start_rx_dma`] once USB is configured; drain it before a rate change.
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

/// Bytes lost to observed receive-DMA ring overwrites.
static RX_RING_FULL: AtomicU32 = AtomicU32::new(0);
/// Separate frame/parity/noise and FIFO-overrun episodes; neither is a per-byte count.
static RX_ERRORS: rx_errors::Counters = rx_errors::Counters::new();

/// Receive loss indicators; byte counts and error episodes have distinct units.
#[derive(Clone, Copy, Default)]
pub struct VcomRxDiscards {
    pub ring_full: u32,
    pub frame_err: u32,
    pub overrun: u32,
}

pub fn vcom_rx_discards() -> VcomRxDiscards {
    VcomRxDiscards {
        ring_full: RX_RING_FULL.load(Ordering::Relaxed),
        frame_err: RX_ERRORS.frame.load(Ordering::Relaxed),
        overrun: RX_ERRORS.overrun.load(Ordering::Relaxed),
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

#[inline]
fn regs() -> &'static raw::usart0::RegisterBlock {
    // SAFETY: USART0 is this seam's; FLEXCOMM0 preempts the USB pass, so FIFOCFG RMWs are masked.
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

    baud_change::service(&BaudRegisters, &VCOM_PENDING_BAUD);
}

struct BaudRegisters;

impl baud_change::Access for BaudRegisters {
    fn exclude(&self, body: impl FnOnce()) {
        cortex_m::interrupt::free(|_| body());
    }
    fn drained(&self) -> bool {
        crate::dma_tx::drained()
    }
    fn tx_idle(&self) -> bool {
        regs().stat.read().txidle().bit_is_set()
    }
    #[inline(always)]
    fn program(&self, baud: u32) -> bool {
        program_vcom_baud(baud)
    }
    fn tx_idle_interrupt(&self, enabled: bool) {
        if enabled {
            regs().intenset.write(|w| w.txidleen().set_bit());
        } else {
            regs().intenclr.write(|w| w.txidleclr().set_bit());
        }
    }
    fn pend(&self) {
        cortex_m::peripheral::NVIC::pend(raw::Interrupt::FLEXCOMM0);
    }
}

/// Fold receive status errors into the ledger; runs in FLEXCOMM0 and in the USB pass it preempts.
pub fn observe_rx_errors() {
    rx_errors::observe(&RxErrorRegisters, &RX_ERRORS);
}

struct RxErrorRegisters;

impl rx_errors::Access for RxErrorRegisters {
    fn exclude(&self, body: impl FnOnce()) {
        cortex_m::interrupt::free(|_| body());
    }
    fn status(&self) -> u32 {
        regs().stat.read().bits()
    }
    fn ack_status(&self, bits: u32) {
        regs().stat.write(|w| unsafe { w.bits(bits) });
    }
    fn rxerr(&self) -> bool {
        regs().fifostat.read().rxerr().bit_is_set()
    }
    fn owned(&self) -> bool {
        !crate::dma_rx::armed()
    }
    fn flush(&self) {
        regs().fifocfg.modify(|_, w| w.emptyrx().set_bit());
    }
    fn ack_rxerr(&self) {
        regs().fifostat.write(|w| w.rxerr().set_bit());
    }
}

/// RXIDLE wait budget: covers one frame at 9600 Bd and up.
const RX_IDLE_SPIN: u32 = 16_384;

/// Reprogram the Flexcomm0 divisors; an unrepresentable rate keeps the old one.
// Kept inline: outlining it stops the 150 MHz image enumerating.
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
    // Written before the disable: UM11126 33.6.1 permits only CFG writes while ENABLE is low.
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

/// The `0x9E` answer; pending is loaded first so a change in flight never reads settled.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    let pending = VCOM_PENDING_BAUD.load(Ordering::Acquire);
    daplink_rs::VcomBaudState::deferred(
        VCOM_REALIZED_BAUD.load(Ordering::Relaxed),
        VCOM_APPLIED_BAUD.load(Ordering::Relaxed),
        pending == 0,
    )
}

impl Vcom {
    /// Initialize FLEXCOMM0 as 8N1 at `baud` with receive IRQ service; call once per image.
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

    /// Pop received bytes; bytes a lap overwrote unread are charged here.
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

    /// Sample the channel without shipping; must run even with staging full, or the cursor is lost.
    pub fn service_rx(&mut self) {
        self.read(&mut []);
    }

    /// Start the receive channel, or restart it after an error; deferred past enumeration.
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

    /// Apply when quiet; unattainable rates leave the applied and pending rates unchanged.
    pub fn request_baud(&mut self, baud: u32) {
        baud_change::request(&BaudRegisters, &VCOM_APPLIED_BAUD, &VCOM_PENDING_BAUD, baud);
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

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::AtomicBool;
    use rx_errors::overrun_episode;
    extern crate std;
    use daplink_rs::{VCOM_BAUD_APPLIED_VALID, VCOM_BAUD_SETTLED};

    /// The controller's FIFO keeps one episode until `RXERR` reads clear; a flushed one ends it.
    #[test]
    fn an_overrun_episode_is_charged_once_while_the_controller_owns_the_fifo() {
        let latch = AtomicBool::new(false);
        let dma: std::vec::Vec<bool> = [true, true, false, true]
            .iter()
            .map(|&rxerr| overrun_episode(&latch, rxerr, false))
            .collect();
        assert_eq!(dma, [true, false, false, true]);

        let latch = AtomicBool::new(false);
        assert!(overrun_episode(&latch, true, true));
        assert!(overrun_episode(&latch, true, true));
    }

    /// One test: the statics are process state, so split tests would race.
    #[test]
    fn a_deferred_rate_reads_unsettled_until_the_applied_rate_catches_up() {
        // Nothing requested yet: settled, with no rate to report.
        VCOM_PENDING_BAUD.store(0, Ordering::Relaxed);
        VCOM_APPLIED_BAUD.store(0, Ordering::Relaxed);
        VCOM_REALIZED_BAUD.store(0, Ordering::Relaxed);
        let s = vcom_baud_state();
        assert_eq!(s.flags, VCOM_BAUD_APPLIED_VALID | VCOM_BAUD_SETTLED);
        assert_eq!((s.realized, s.applied), (0, 0));

        // A request published before it is attempted reads unsettled at the rate it is leaving.
        VCOM_PENDING_BAUD.store(115_200, Ordering::Release);
        let s = vcom_baud_state();
        assert_eq!(s.flags, VCOM_BAUD_APPLIED_VALID);
        assert_eq!((s.realized, s.applied), (0, 0));

        // FLEXCOMM0 programs the divisors, then clears exactly the rate it applied.
        VCOM_REALIZED_BAUD.store(115_384, Ordering::Relaxed);
        VCOM_APPLIED_BAUD.store(115_200, Ordering::Release);
        assert!(VCOM_PENDING_BAUD
            .compare_exchange(115_200, 0, Ordering::Release, Ordering::Relaxed)
            .is_ok());
        let s = vcom_baud_state();
        assert_eq!(s.flags, VCOM_BAUD_APPLIED_VALID | VCOM_BAUD_SETTLED);
        assert_eq!((s.realized, s.applied), (115_384, 115_200));
        assert_eq!(VCOM_APPLIED_BAUD.load(Ordering::Acquire), 115_200);

        // A newer request survives the stale clear: the compare-exchange names its rate.
        VCOM_PENDING_BAUD.store(9_600, Ordering::Release);
        assert!(VCOM_PENDING_BAUD
            .compare_exchange(115_200, 0, Ordering::Release, Ordering::Relaxed)
            .is_err());
        assert_eq!(VCOM_PENDING_BAUD.load(Ordering::Acquire), 9_600);
        let s = vcom_baud_state();
        assert_eq!(s.flags, VCOM_BAUD_APPLIED_VALID);
        assert_eq!(
            (s.realized, s.applied),
            (115_384, 115_200),
            "an unsettled state still reports the rate on the wire"
        );

        // The realized rate is the divisor's, not the nominal one the host asked for.
        assert_ne!(s.realized, s.applied);

        VCOM_PENDING_BAUD.store(0, Ordering::Relaxed);
        VCOM_APPLIED_BAUD.store(0, Ordering::Relaxed);
        VCOM_REALIZED_BAUD.store(0, Ordering::Relaxed);
    }
}
