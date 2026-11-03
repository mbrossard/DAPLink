//! VCOM receive over UART1 RX via DMA, as a [`DmaRxChannel`] the shared ring cursor reads.

use core::cell::UnsafeCell;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};

use daplink_rp_dma_uart::seq::DmaChannel;
use daplink_rs::DmaRxChannel;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_rp::Peri;
use static_cell::ConstStaticCell;

/// Receive ring entries; halfword width preserves the PL011 error flags.
pub const RX_RING_LEN: usize = 2048;
const _: () = assert!((RX_RING_LEN * 2).is_power_of_two());
/// `ring_size` field: log2 of the wrap width in bytes, derived so it cannot drift from the ring.
const RING_SIZE_LOG2: u8 = (RX_RING_LEN * 2).trailing_zeros() as u8;

/// Channel claimed for the receive ring; the handler is a free function and reads it from here.
static RX_CHAN_NUM: AtomicU32 = AtomicU32::new(0);

fn rx_chan() -> usize {
    RX_CHAN_NUM.load(Ordering::Relaxed) as usize
}

/// `INTE1`/`INTS1` index: receive completion shares `DMA_IRQ_1` with transmit.
const IRQ_LINE: usize = 1;

/// Laps completed since the channel last started, so the derived position is an absolute total.
static RX_EPOCH: AtomicU32 = AtomicU32::new(0);

/// RP2040 only: bounce-channel scratch word, claimed once because every baud change re-arms it.
#[cfg(feature = "rp2040")]
static CHAIN_SINK: AtomicU32 = AtomicU32::new(0);

/// The receive ring's memory, aligned so the controller's write-address wrap lands inside it.
#[repr(C, align(4096))]
pub(crate) struct RxRing(UnsafeCell<[u16; RX_RING_LEN]>);

impl RxRing {
    pub(crate) fn read(&self, index: usize) -> u16 {
        assert!(index < RX_RING_LEN);
        // SAFETY: static initialized storage; no references to DMA-written entries are formed.
        unsafe { self.0.get().cast::<u16>().add(index).read_volatile() }
    }
}
// The DMA wrap region must fit inside one aligned allocation.
const _: () = assert!(core::mem::align_of::<RxRing>() >= RX_RING_LEN * 2);

/// The channels driving VCOM receive: the ring channel, plus RP2040's bounce channel.
#[derive(Clone, Copy)]
pub struct DmaVcomRx {
    ch: usize,
    /// RP2040 only: the trivial channel that bounces the trigger back to `ch`.
    #[cfg(feature = "rp2040")]
    chain: usize,
    base: u32,
    src: u32,
}

impl DmaVcomRx {
    fn chan(&self) -> DmaChannel {
        // SAFETY: the constructor consumed this channel; copies coordinate within this module.
        unsafe { DmaChannel::new(self.ch) }
    }

    fn regs(&self) -> pac::dma::Channel {
        // SAFETY: only this module configures the claimed receive channel.
        unsafe { self.chan().regs() }
    }

    /// Arm the bounce channel through the non-triggering alias; a channel cannot chain to itself.
    #[cfg(feature = "rp2040")]
    fn arm_chain_b(&self) {
        let b = pac::DMA.ch(self.chain);
        let sink = CHAIN_SINK.load(Ordering::Relaxed);
        b.read_addr().write_value(sink);
        b.write_addr().write_value(sink);
        b.trans_count().write_value(1);
        let mut ctrl = pac::dma::regs::CtrlTrig::default();
        ctrl.set_en(true);
        ctrl.set_treq_sel(TreqSel::PERMANENT);
        ctrl.set_chain_to(self.ch as u8);
        ctrl.set_irq_quiet(true);
        b.al1_ctrl().write_value(ctrl.0);
    }

    /// Arm the ring channel, and RP2040's bounce partner, to fill the ring forever; epoch zeroed.
    /// What the PL011 receive FIFO holds first goes to `keep`, as `UARTDR` entries.
    pub(crate) fn start(&self, mut keep: impl FnMut(u16)) {
        // Popped while the channel is stopped, so the DMA and the CPU never share the FIFO.
        while !pac::UART1.uartfr().read().rxfe() {
            keep(pac::UART1.uartdr().read().0 as u16);
        }
        #[cfg(feature = "rp2040")]
        self.arm_chain_b();
        let ch = self.regs();
        // SAFETY: stopped claimed channel, UART1 source and static aligned halfword ring.
        unsafe { self.chan().set_addrs(self.src, self.base) };
        // Exactly one ring, so the reload is invisible; `TRIGGER_SELF` re-arms it.
        #[cfg(feature = "rp2350")]
        unsafe {
            self.chan().set_count_retriggering(RX_RING_LEN as u32)
        };
        // Exactly one ring, so the reload is invisible; the bounce channel re-arms it.
        #[cfg(feature = "rp2040")]
        unsafe {
            self.chan().set_count(RX_RING_LEN as u32)
        };
        cortex_m::interrupt::free(|_| {
            pac::DMA.ints(IRQ_LINE).write_value(1 << self.ch);
            RX_EPOCH.store(0, Ordering::Relaxed);
        });
        pac::DMA.inte(IRQ_LINE).modify(|v| *v |= 1 << self.ch);
        compiler_fence(Ordering::SeqCst);
        ch.ctrl_trig().write(|w| {
            w.set_en(true);
            // Halfwords, so the PL011's per-character flags travel with their byte.
            w.set_data_size(DataSize::SIZE_HALFWORD);
            w.set_incr_read(false);
            w.set_incr_write(true);
            w.set_ring_sel(true);
            w.set_ring_size(RING_SIZE_LOG2);
            w.set_treq_sel(TreqSel::UART1_RX);
            #[cfg(feature = "rp2350")]
            w.set_chain_to(self.ch as u8);
            #[cfg(feature = "rp2040")]
            w.set_chain_to(self.chain as u8);
            // Every reload raises the completion the epoch counts.
            w.set_irq_quiet(false);
        });
        compiler_fence(Ordering::SeqCst);
    }

    /// Entries the controller has written since [`start`](Self::start), free-running and wrapping.
    pub(crate) fn produced(&self) -> u32 {
        let mask = 1u32 << self.ch;
        let pending = || pac::DMA.ints(IRQ_LINE).read() & mask != 0;
        // A reload between the flag and counter reads puts the total a ring out, so bracket both.
        loop {
            let epoch_before = RX_EPOCH.load(Ordering::Acquire);
            let pending_before = pending();
            let remaining = self.remaining().min(RX_RING_LEN) as u32;
            if pending() != pending_before || RX_EPOCH.load(Ordering::Acquire) != epoch_before {
                continue;
            }
            let epoch = epoch_before.wrapping_add(pending_before as u32);
            return epoch
                .wrapping_mul(RX_RING_LEN as u32)
                .wrapping_add(RX_RING_LEN as u32 - remaining);
        }
    }

    /// Stop filling the ring with its counter intact; only [`halt`](Self::halt) then restarts it.
    pub(crate) fn pause(&self) {
        // SAFETY: clearing EN only pauses the claimed channel; nothing re-enables it but `start`.
        unsafe { self.chan().disable() };
    }

    /// Stop both channels; abort timeout permanently halts without releasing or rebasing the ring.
    pub(crate) fn halt(&self) {
        cortex_m::interrupt::free(|_| {
            pac::DMA.inte(IRQ_LINE).modify(|v| *v &= !(1 << self.ch));
            // SAFETY: disable both partners before either abort so chaining cannot restart one.
            unsafe {
                self.chan().disable();
                #[cfg(feature = "rp2040")]
                {
                    let chain = DmaChannel::new(self.chain);
                    chain.disable();
                    if !chain.abort() {
                        crate::dma_tx::abort_failed();
                    }
                }
                if !self.chan().abort() {
                    crate::dma_tx::abort_failed();
                }
            }
            pac::DMA.ints(IRQ_LINE).write_value(1 << self.ch);
        });
    }
}

/// Record a completed lap. Runs from `DMA_IRQ_1`, which this shares with transmit.
pub(crate) fn service() {
    let ints = pac::DMA.ints(IRQ_LINE).read();
    if ints & (1 << rx_chan() as u32) == 0 {
        return;
    }
    pac::DMA.ints(IRQ_LINE).write_value(1 << rx_chan() as u32);
    // Load-add-store: this handler is the counter's only writer, and ARMv6-M has no atomic RMW.
    let epoch = RX_EPOCH.load(Ordering::Relaxed).wrapping_add(1);
    RX_EPOCH.store(epoch, Ordering::Release);
}

impl DmaRxChannel for DmaVcomRx {
    fn remaining(&self) -> usize {
        self.chan().remaining() as usize
    }

    fn reload(&mut self, _len: usize) {
        self.halt();
        self.start(|_| {});
    }

    fn running(&self) -> bool {
        self.chan().enabled()
    }
}

/// Claim the receive channels and ring memory, and start filling from `uart_dr`. Call once.
///
/// # Safety
/// The caller owns UART1; `uart_dr` must identify its UARTDR and the channels must be idle.
pub(crate) unsafe fn new<R: ChannelInstance, C: ChannelInstance>(
    uart_dr: u32,
    _rx: Peri<'static, R>,
    _chain: Peri<'static, C>,
) -> (DmaVcomRx, &'static RxRing) {
    RX_CHAN_NUM.store(R::number() as u32, Ordering::Relaxed);
    static RING: ConstStaticCell<RxRing> =
        ConstStaticCell::new(RxRing(UnsafeCell::new([0; RX_RING_LEN])));
    #[cfg(feature = "rp2040")]
    {
        static SINK: ConstStaticCell<u32> = ConstStaticCell::new(0);
        CHAIN_SINK.store(SINK.take() as *mut u32 as u32, Ordering::Relaxed);
    }
    let ring = RING.take();
    let base = ring.0.get() as u32;
    let mem: &'static RxRing = ring;
    let chan = DmaVcomRx {
        ch: R::number() as usize,
        #[cfg(feature = "rp2040")]
        chain: C::number() as usize,
        base,
        src: uart_dr,
    };
    chan.start(|_| {});
    (chan, mem)
}
