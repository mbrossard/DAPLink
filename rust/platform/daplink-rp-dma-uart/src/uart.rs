//! A PL011 streamed over two DMA channels: raw registers, no interrupts.

use core::ptr::read_volatile;

use rp_pac::dma::regs::CtrlTrig;
use rp_pac::dma::vals::{DataSize, TreqSel};
use rp_pac::uart::Uart;

use crate::rx::{RxRing, RxTracker, RX_RING_LOG2};
use crate::seq::DmaChannel;

/// The count a receive channel is armed with: the most the RP2350 allows, whose `TRANS_COUNT` top
/// four bits are a mode field. The channel stops there and [`arm`](UartDma::arm) must run again.
const RX_ARM_COUNT: u32 = 0x0FFF_FFFF;

/// A PL011 and the two DMA channels driving it.
pub struct UartDma {
    uart: Uart,
    rx_chan: DmaChannel,
    tx_chan: DmaChannel,
    dreq_rx: TreqSel,
    dreq_tx: TreqSel,
    rx: RxTracker,
    /// The armed ring, or null before [`arm`](Self::arm).
    ring: *mut u16,
    /// Latched when an abort wait expired with the channel still busy.
    abort_stuck: bool,
}

impl UartDma {
    /// `uart` is the PL011 to stream; the channel numbers and DREQs name its two DMA channels.
    pub const fn new(
        uart: Uart,
        rx_chan: usize,
        tx_chan: usize,
        dreq_rx: TreqSel,
        dreq_tx: TreqSel,
    ) -> Self {
        Self {
            uart,
            rx_chan: DmaChannel::new(rx_chan),
            tx_chan: DmaChannel::new(tx_chan),
            dreq_rx,
            dreq_tx,
            rx: RxTracker::new(),
            ring: core::ptr::null_mut(),
            abort_stuck: false,
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

    /// Whether an abort wait ever expired with the channel still busy; sticky since construction,
    /// and every later reconfiguration of that channel is suspect.
    pub fn abort_stuck(&self) -> bool {
        self.abort_stuck
    }

    /// Hand both directions to the controller and arm the receive ring, discarding whatever the
    /// ring holds: re-arming belongs after a divisor change and never mid-stream.
    ///
    /// # Safety
    /// `ring` must outlive every transfer and stay put.
    pub unsafe fn arm(&mut self, ring: &'static mut RxRing) {
        // Reconfiguring a busy channel leaves its counter no longer describing what was written.
        self.abort_stuck |= !self.rx_chan.abort();
        // Abort first: in the other order the channel refills from bytes that predate the session.
        while !self.uart.uartfr().read().rxfe() {
            let _ = self.uart.uartdr().read();
        }
        self.rx_chan.set_addrs(
            self.uart.uartdr().as_ptr() as u32,
            ring.buf.as_mut_ptr() as u32,
        );
        self.rx_chan.set_count(RX_ARM_COUNT);
        self.rx.reset();
        self.ring = ring.buf.as_mut_ptr();
        self.rx_chan.trigger(self.rx_ctrl());

        self.uart.uartdmacr().modify(|w| {
            w.set_rxdmae(true);
            w.set_txdmae(true);
        });
    }

    /// Entries the controller has produced since it was armed.
    fn rx_produced(&self) -> u32 {
        RX_ARM_COUNT.saturating_sub(self.rx_chan.remaining())
    }

    /// Entries waiting, bounded by the ring rather than the channel's unbounded production total.
    pub fn rx_available(&mut self) -> u32 {
        let produced = self.rx_produced();
        self.rx.observe(produced);
        self.rx.available()
    }

    /// Drop whatever the controller has lapped, and say how many entries that was.
    pub fn rx_resync(&mut self) -> u32 {
        let produced = self.rx_produced();
        self.rx.observe(produced);
        self.rx.take_lost()
    }

    /// Copy up to `out.len()` raw entries and the count a lap overwrote, taking one counter sample
    /// and one cursor fold for the batch where [`rx_next`](Self::rx_next) pays both per entry.
    pub fn rx_read(&mut self, out: &mut [u16]) -> (usize, u32) {
        if self.ring.is_null() {
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
            // SAFETY: `idx` is behind the controller's write position, so the entry is committed;
            // volatile because that write is invisible to alias analysis.
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

    /// Whether the transmit channel still owns its buffer: `CTRL.BUSY`, which outlasts the
    /// counter reaching zero, is what marks a sequence done.
    pub fn tx_busy(&self) -> bool {
        self.tx_chan.busy()
    }

    /// Send `len` bytes, which [`tx_busy`](Self::tx_busy) must read false before the caller reuses.
    ///
    /// # Safety
    /// `buf` must stay put and stay unwritten for the length of the transfer.
    pub unsafe fn tx_start(&mut self, buf: *const u8, len: usize) {
        if len == 0 {
            return;
        }
        self.tx_chan
            .set_addrs(buf as u32, self.uart.uartdr().as_ptr() as u32);
        self.tx_chan.set_count(len as u32);
        self.tx_chan.trigger(self.tx_ctrl());
    }

    /// Stop the transmit channel, dropping whatever remains of the transfer in flight.
    pub fn tx_abort(&mut self) {
        self.abort_stuck |= !self.tx_chan.abort();
    }
}
