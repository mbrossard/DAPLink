//! A PL011 streamed over two DMA channels: raw registers, no interrupts.

use core::ptr::read_volatile;
use core::sync::atomic::{compiler_fence, Ordering};

use rp_pac::dma::regs::CtrlTrig;
use rp_pac::dma::vals::{DataSize, TreqSel};
use rp_pac::uart::Uart;

use crate::rx::{RxRing, RxTracker, RX_RING_LOG2};
use crate::seq::DmaChannel;

/// Maximum RX count common to both chips; exhaustion requires an explicit rearm.
const RX_ARM_COUNT: u32 = 0x0FFF_FFFF;

/// The abort wait expired; the old transfer's memory remains reserved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AbortTimeout;

/// A PL011 and the two DMA channels driving it; dropping it stops neither channel, so armed buffers
/// stay reserved.
pub struct UartDma {
    uart: Uart,
    rx_chan: DmaChannel,
    tx_chan: DmaChannel,
    dreq_rx: TreqSel,
    dreq_tx: TreqSel,
    rx: RxTracker,
    /// The armed ring, or null before [`arm`](Self::arm).
    ring: *mut u16,
    abort_timeouts: u32,
    rx_abort_pending: bool,
}

impl UartDma {
    /// Bind an exclusively reserved PL011 and its distinct RX/TX DMA channels.
    ///
    /// # Safety
    /// Own the UART and both idle channels; DREQs must match that UART and no other owner may reconfigure them.
    pub const unsafe fn new(
        uart: Uart,
        rx_chan: usize,
        tx_chan: usize,
        dreq_rx: TreqSel,
        dreq_tx: TreqSel,
    ) -> Self {
        assert!(rx_chan != tx_chan);
        Self {
            uart,
            // SAFETY: the constructor requires ownership of both channels.
            rx_chan: unsafe { DmaChannel::new(rx_chan) },
            // SAFETY: the constructor requires ownership of both channels.
            tx_chan: unsafe { DmaChannel::new(tx_chan) },
            dreq_rx,
            dreq_tx,
            rx: RxTracker::new(),
            ring: core::ptr::null_mut(),
            abort_timeouts: 0,
            rx_abort_pending: false,
        }
    }

    /// `CTRL_TRIG` for the receive channel: halfword ring writer, chained to itself.
    fn rx_ctrl(&self) -> CtrlTrig {
        let mut ctrl = CtrlTrig::default();
        ctrl.set_en(true);
        // Halfwords, so the PL011's per-character flags travel with their byte.
        ctrl.set_data_size(DataSize::SIZE_HALFWORD);
        ctrl.set_incr_write(true);
        ctrl.set_ring_sel(true);
        ctrl.set_ring_size(RX_RING_LOG2 as u8);
        // A channel chained to itself is a channel with chaining disabled.
        ctrl.set_chain_to(self.rx_chan.number() as u8);
        ctrl.set_treq_sel(self.dreq_rx);
        ctrl
    }

    /// `CTRL_TRIG` for the transmit channel: byte reader walking the send buffer.
    fn tx_ctrl(&self) -> CtrlTrig {
        let mut ctrl = CtrlTrig::default();
        ctrl.set_en(true);
        ctrl.set_incr_read(true);
        // A channel chained to itself is a channel with chaining disabled.
        ctrl.set_chain_to(self.tx_chan.number() as u8);
        ctrl.set_treq_sel(self.dreq_tx);
        ctrl
    }

    /// Whether any abort wait has expired since construction.
    pub fn abort_stuck(&self) -> bool {
        self.abort_timeouts != 0
    }

    /// Failed abort attempts since construction, saturating independently of stream counters.
    pub fn abort_timeouts(&self) -> u32 {
        self.abort_timeouts
    }

    /// RX service must remain paused until a later successful arm resolves the failed abort.
    pub fn rx_abort_pending(&self) -> bool {
        self.rx_abort_pending
    }

    /// Arm RX after a divisor change, discarding the previous ring only after a successful abort.
    ///
    /// # Errors
    /// On timeout, retain the previous ring and cursor; retry arm before resuming RX service.
    ///
    /// # Safety
    /// Reserve the ring for this RX channel alone until a successful abort, with no overlapping references or writers.
    pub unsafe fn arm(&mut self, ring: &'static RxRing) -> Result<(), AbortTimeout> {
        // SAFETY: this owner excludes rearming and retains both old and requested rings.
        unsafe { self.arm_with(ring, |chan| chan.abort()) }
    }

    unsafe fn arm_with(
        &mut self,
        ring: &'static RxRing,
        abort: impl FnOnce(DmaChannel) -> bool,
    ) -> Result<(), AbortTimeout> {
        self.rx_abort_pending = !abort(self.rx_chan);
        self.record_abort(!self.rx_abort_pending)?;
        // Abort first: in the other order the channel refills from bytes that predate the session.
        while !self.uart.uartfr().read().rxfe() {
            let _ = self.uart.uartdr().read();
        }
        // SAFETY: the constructor owns the channel; the caller reserves the aligned halfword ring.
        unsafe {
            self.rx_chan
                .set_addrs(self.uart.uartdr().as_ptr() as u32, ring.as_mut_ptr() as u32);
            self.rx_chan.set_count(RX_ARM_COUNT);
        }
        self.rx.reset();
        self.ring = ring.as_mut_ptr();
        // SAFETY: the caller reserves this initialized ring and the constructor fixes UART/DREQ ownership.
        unsafe { self.rx_chan.trigger(self.rx_ctrl()) };

        self.uart.uartdmacr().modify(|w| {
            w.set_rxdmae(true);
            w.set_txdmae(true);
        });
        Ok(())
    }

    /// Entries the controller has produced since it was armed.
    fn rx_produced(&self) -> u32 {
        let produced = RX_ARM_COUNT.saturating_sub(self.rx_chan.remaining());
        compiler_fence(Ordering::Acquire);
        produced
    }

    /// Entries waiting, bounded by the ring rather than the channel's unbounded production total.
    pub fn rx_available(&mut self) -> u32 {
        if self.rx_abort_pending {
            return 0;
        }
        let produced = self.rx_produced();
        self.rx.observe(produced);
        self.rx.available()
    }

    /// Drop whatever the controller has lapped, and say how many entries that was.
    pub fn rx_resync(&mut self) -> u32 {
        if self.rx_abort_pending {
            return 0;
        }
        let produced = self.rx_produced();
        self.rx.observe(produced);
        self.rx.take_lost()
    }

    /// Copy raw entries and return the observed overwrite count using one production sample.
    pub fn rx_read(&mut self, out: &mut [u16]) -> (usize, u32) {
        if self.ring.is_null() || self.rx_abort_pending {
            return (0, 0);
        }
        let produced = self.rx_produced();
        self.rx.observe(produced);
        let lost = self.rx.take_lost();
        let mut n = 0;
        while n < out.len() {
            let Some(idx) = self.rx.next_index() else {
                break;
            };
            // SAFETY: initialized DMA storage; the bounded cursor forms no references to live slots.
            out[n] = unsafe { read_volatile(self.ring.add(idx)) };
            n += 1;
        }
        (n, lost)
    }

    /// Take the next entry: a raw `UARTDR` read, data in the low byte and the error flags above it.
    pub fn rx_next(&mut self) -> Option<u16> {
        if self.ring.is_null() || self.rx_available() == 0 {
            return None;
        }
        let idx = self.rx.next_index()?;
        // SAFETY: armed, and the entry is behind the controller's write cursor.
        Some(unsafe { read_volatile(self.ring.add(idx)) })
    }

    /// BUSY false releases the source; after abort, it does not prove full transfer completion.
    pub fn tx_busy(&self) -> bool {
        self.tx_chan.busy()
    }

    /// Send `len` bytes, which [`tx_busy`](Self::tx_busy) must read false before the caller reuses.
    ///
    /// # Safety
    /// The channel must be idle; `buf` holds `len` readable bytes left untouched until BUSY clears or abort succeeds.
    pub unsafe fn tx_start(&mut self, buf: *const u8, len: usize) {
        if len == 0 {
            return;
        }
        #[cfg(feature = "rp235x")]
        assert!(len <= 0x0fff_ffff);
        #[cfg(feature = "rp2040")]
        assert!(len <= u32::MAX as usize);
        // SAFETY: the caller reserves the source until completion; the constructor owns UART and channel.
        unsafe {
            self.tx_chan
                .set_addrs(buf as u32, self.uart.uartdr().as_ptr() as u32);
            self.tx_chan.set_count(len as u32);
            self.tx_chan.trigger(self.tx_ctrl());
        }
    }

    /// Stop TX; success releases the buffer but does not prove the whole run was transmitted.
    ///
    /// # Errors
    /// Timeout retains the source lease; retry abort before retiring or reusing that run.
    pub fn tx_abort(&mut self) -> Result<(), AbortTimeout> {
        // SAFETY: this owner serializes TX starts and aborts.
        self.tx_abort_with(|chan| unsafe { chan.abort() })
    }

    fn tx_abort_with(
        &mut self,
        abort: impl FnOnce(DmaChannel) -> bool,
    ) -> Result<(), AbortTimeout> {
        self.record_abort(abort(self.tx_chan))
    }

    fn record_abort(&mut self, released: bool) -> Result<(), AbortTimeout> {
        if released {
            Ok(())
        } else {
            self.abort_timeouts = self.abort_timeouts.saturating_add(1);
            Err(AbortTimeout)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn uart() -> UartDma {
        // SAFETY: tests inject abort results and never execute any MMIO access.
        unsafe { UartDma::new(rp_pac::UART0, 0, 1, TreqSel::UART0_RX, TreqSel::UART0_TX) }
    }

    #[test]
    fn failed_rx_arm_retains_ring_cursor_and_loss_without_touching_mmio() {
        static OLD: RxRing = RxRing::new();
        static NEXT: RxRing = RxRing::new();
        let mut uart = uart();
        uart.ring = OLD.as_mut_ptr();
        uart.rx.observe(crate::RX_RING_LEN as u32 + 7);
        let available = uart.rx.available();
        for count in 1..=2 {
            // SAFETY: fake failure returns before any register or buffer access.
            assert_eq!(
                unsafe { uart.arm_with(&NEXT, |_| false) },
                Err(AbortTimeout)
            );
            assert_eq!(uart.ring, OLD.as_mut_ptr());
            assert_eq!(uart.rx.available(), available);
            assert!(uart.rx_abort_pending());
            assert_eq!(uart.abort_timeouts(), count);
            assert_eq!(uart.rx_available(), 0);
            assert_eq!(uart.rx_resync(), 0);
            assert_eq!(uart.rx_read(&mut [0; 4]), (0, 0));
            assert_eq!(uart.rx_next(), None);
        }
        assert_eq!(uart.rx.take_lost(), 7);
        assert_eq!(uart.tx_abort_with(|_| false), Err(AbortTimeout));
        assert_eq!(uart.abort_timeouts(), 3);
        assert_eq!(uart.tx_abort_with(|_| true), Ok(()));
        assert!(uart.rx_abort_pending());
    }

    #[test]
    fn tx_abort_reports_each_failure_and_allows_successful_retry() {
        let mut uart = uart();
        assert_eq!(uart.tx_abort_with(|_| false), Err(AbortTimeout));
        assert_eq!(uart.tx_abort_with(|_| false), Err(AbortTimeout));
        assert_eq!(uart.abort_timeouts(), 2);
        assert_eq!(uart.tx_abort_with(|_| true), Ok(()));
        assert_eq!(uart.abort_timeouts(), 2);
        assert!(uart.abort_stuck());
        uart.abort_timeouts = u32::MAX;
        assert_eq!(uart.tx_abort_with(|_| false), Err(AbortTimeout));
        assert_eq!(uart.abort_timeouts(), u32::MAX);
    }
}
