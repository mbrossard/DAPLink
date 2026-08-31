//! SWO capture on Flexcomm3 receive over DMA0 channel 8: a chain the host starts and stops at
//! will, so each stop discards the latched state and the FIFO rather than handing them on.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use daplink_rs::{DmaRxChannel, DmaRxRing, SwoSource};
use lpc55_hal::raw;

use crate::dma;

/// Flexcomm3 receive; the request map transposes Flexcomm2 and Flexcomm3 (Flexcomm2 is 10 and 11).
const CHANNEL: usize = 8;

/// Above the VCOM channel: neither can starve the other, and this is the faster line.
const PRIORITY: u8 = 0;

static ARMED: AtomicBool = AtomicBool::new(false);

/// Blocks finished since the channel was armed, counted from the DMA0 interrupt; a lost
/// completion is a lost block of position.
static COMPLETED: AtomicUsize = AtomicUsize::new(0);
/// The absolute count the last poll reported: the floor every later report is clamped to, and
/// the reference for spotting a wrap of the whole ring.
static PREV_ABS: AtomicUsize = AtomicUsize::new(0);

/// Laps the consumer did not keep up with -- trace lost, reported through the SWO overrun flag --
/// and channel errors the controller latched.
static LAPPED: AtomicBool = AtomicBool::new(false);
pub(crate) static SWO_DMA_ERR: AtomicU32 = AtomicU32::new(0);

fn usart() -> &'static raw::usart0::RegisterBlock {
    // SAFETY: USART3 is owned by the SWO seam, which is this module's only caller.
    unsafe { &*raw::USART3::ptr() }
}

/// The receive half of Flexcomm3, as the ring's channel.
pub struct Fc3DmaRx;

impl Fc3DmaRx {
    /// Units produced since the channel was armed, across the whole chain.
    fn absolute(&self) -> usize {
        COMPLETED.load(Ordering::Relaxed) * dma::SWO_BLOCK_LEN
            + (dma::SWO_BLOCK_LEN - dma::remaining_in(CHANNEL, dma::SWO_BLOCK_LEN))
    }
}

impl DmaRxChannel for Fc3DmaRx {
    fn remaining(&self) -> usize {
        // The ring's own view: how far the controller is from the end of the whole chain.
        dma::SWO_BUF_LEN - (self.absolute() % dma::SWO_BUF_LEN)
    }

    fn sample(&mut self) -> (usize, bool) {
        let prev = PREV_ABS.load(Ordering::Relaxed);
        let (abs, errored) = dma::sample_rx_chain(CHANNEL, &COMPLETED, &PREV_ABS);
        if errored {
            SWO_DMA_ERR.fetch_add(1, Ordering::Relaxed);
            stop();
        }
        // A wrap of the whole ring, not of one block: the cursor folds modulo the ring itself.
        let wrapped = abs / dma::SWO_BUF_LEN != prev / dma::SWO_BUF_LEN;
        (dma::SWO_BUF_LEN - (abs % dma::SWO_BUF_LEN), wrapped)
    }

    fn reload(&mut self, _len: usize) {
        stop();
        arm();
    }

    fn running(&self) -> bool {
        ARMED.load(Ordering::Relaxed)
    }
}

/// Arm the channel over the SWO buffer and hand the USART3 FIFO to it.
pub(crate) fn arm() {
    let usart = usart();
    COMPLETED.store(0, Ordering::Relaxed);
    PREV_ABS.store(0, Ordering::Relaxed);
    dma::arm_rx_chain(
        CHANNEL,
        unsafe { core::ptr::addr_of_mut!((*dma::mem()).swo) },
        core::ptr::addr_of!(usart.fiford) as u32,
        PRIORITY,
    );
    // Empty the FIFO in the same write that hands it over: no byte from before the session.
    cortex_m::interrupt::free(|_| {
        usart
            .fifocfg
            .modify(|_, w| w.emptyrx().set_bit().dmarx().set_bit());
    });
    ARMED.store(true, Ordering::Relaxed);
    // Armed first: `swo_dma_service_irq` counts nothing for a chain not yet marked armed.
    dma::enable_int(CHANNEL);
}

/// Count the chain's block completions from the DMA0 interrupt.
pub fn swo_dma_service_irq() {
    if !ARMED.load(Ordering::Relaxed) {
        return;
    }
    while dma::take_lap(CHANNEL) {
        COMPLETED.fetch_add(1, Ordering::Relaxed);
    }
}

/// Stop the channel and discard everything it and the FIFO were holding.
pub(crate) fn stop() {
    let usart = usart();
    // Stop new requests at the source first, so the teardown cannot race an incoming byte.
    cortex_m::interrupt::free(|_| {
        usart.fifocfg.modify(|_, w| w.dmarx().clear_bit());
    });
    if ARMED.load(Ordering::Relaxed) {
        // Interrupt off before the flag drops, or a completion mid-teardown re-enters forever.
        dma::disable_int(CHANNEL);
        ARMED.store(false, Ordering::Relaxed);
        dma::stop(CHANNEL);
    }
    cortex_m::interrupt::free(|_| {
        usart.fifocfg.modify(|_, w| w.emptyrx().set_bit());
    });
    if usart.fifostat.read().rxerr().bit_is_set() {
        usart.fifostat.write(|w| w.rxerr().set_bit());
    }
}

/// The captured stream, as the SWO transport's byte source.
pub struct DmaSwoSource {
    ring: DmaRxRing<Fc3DmaRx>,
    mem: &'static [u8],
}

impl DmaSwoSource {
    pub(crate) fn new() -> Self {
        let mem = unsafe { &*core::ptr::addr_of!((*dma::mem()).swo.buf) };
        DmaSwoSource {
            ring: DmaRxRing::new(Fc3DmaRx, mem.len()),
            mem,
        }
    }
}

impl SwoSource for DmaSwoSource {
    fn len(&self) -> usize {
        // Advisory, and read without folding; `drain` does the accounting.
        self.ring.pending()
    }

    fn capacity(&self) -> usize {
        self.ring.capacity()
    }

    fn drain(&mut self, buf: &mut [u8]) -> usize {
        let (n, lost) = self.ring.read(self.mem, buf);
        if lost != 0 {
            LAPPED.store(true, Ordering::Relaxed);
        }
        n
    }

    fn clear(&mut self) {
        self.ring.reset_cursor();
        LAPPED.store(false, Ordering::Relaxed);
    }

    fn take_overrun(&mut self) -> bool {
        LAPPED.swap(false, Ordering::Relaxed)
    }
}
