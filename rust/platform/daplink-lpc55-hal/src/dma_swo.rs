//! SWO capture on Flexcomm3 receive over DMA0 channel 8; each stop discards chain and FIFO.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

#[path = "swo_epoch.rs"]
mod epoch;

use daplink_rs::{DmaRxChannel, DmaRxRing, SwoSource};
use lpc55_hal::raw;

use crate::dma;

/// Flexcomm3 receive; the request map transposes Flexcomm2 and Flexcomm3 (Flexcomm2 is 10 and 11).
const CHANNEL: usize = 8;

/// Above the VCOM channel: neither can starve the other, and this is the faster line.
const PRIORITY: u8 = 0;

static ARMED: AtomicBool = AtomicBool::new(false);

/// Consumer laps (lost trace, reported as SWO overrun) and latched channel errors.
static LAPPED: AtomicBool = AtomicBool::new(false);
static SWO_DMA_ERR: AtomicU32 = AtomicU32::new(0);

fn usart() -> &'static raw::usart0::RegisterBlock {
    #[cfg(test)]
    {
        dma::stop_tests::usart()
    }
    #[cfg(not(test))]
    // SAFETY: USART3 is owned by the SWO seam, which is this module's only caller.
    unsafe {
        &*raw::USART3::ptr()
    }
}

/// The receive half of Flexcomm3, as the ring's channel.
pub struct Fc3DmaRx;

impl Fc3DmaRx {
    /// Units produced since the channel was armed, across the whole chain.
    fn absolute(&self) -> u64 {
        dma::chain_position(CHANNEL, epoch::blocks, epoch::previous())
    }
}

impl DmaRxChannel for Fc3DmaRx {
    fn remaining(&self) -> usize {
        // The ring's own view: how far the controller is from the end of the whole chain.
        ring_view(self.absolute(), 0).0
    }

    fn sample(&mut self) -> (usize, bool) {
        let prev = epoch::previous();
        let (abs, errored) = dma::sample_rx_chain(CHANNEL, epoch::blocks, prev);
        epoch::publish(abs);
        if errored {
            SWO_DMA_ERR.fetch_add(1, Ordering::Relaxed);
            stop();
        }
        ring_view(abs, prev)
    }

    fn reload(&mut self, _len: usize) {
        stop();
        arm();
    }

    fn running(&self) -> bool {
        ARMED.load(Ordering::Relaxed)
    }
}

/// The ring's `(remaining, wrapped)` for an absolute count, against the last one reported.
#[inline(always)]
fn ring_view(abs: u64, prev: u64) -> (usize, bool) {
    // A wrap of the whole ring, not of one block: the cursor folds modulo the ring itself.
    let len = dma::SWO_BUF_LEN as u64;
    let wrapped = abs / len != prev / len;
    (dma::SWO_BUF_LEN - (abs % len) as usize, wrapped)
}

/// Arm the channel over the SWO buffer and hand the USART3 FIFO to it.
pub(crate) fn arm() {
    let usart = usart();
    epoch::reset();
    dma::arm_rx_chain(
        CHANNEL,
        unsafe { core::ptr::addr_of_mut!((*dma::mem()).swo) },
        core::ptr::addr_of!(usart.fiford) as u32,
        PRIORITY,
    );
    // Empty the FIFO in the same write that hands it over: no byte from before the session.
    dma::masked(|| {
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
        epoch::complete();
    }
}

/// Stop the channel and discard everything it and the FIFO were holding.
pub(crate) fn stop() {
    let usart = usart();
    // Stop new requests at the source first, so the teardown cannot race an incoming byte.
    dma::masked(|| {
        usart.fifocfg.modify(|_, w| w.dmarx().clear_bit());
    });
    if ARMED.load(Ordering::Relaxed) {
        // Interrupt off before the flag drops, or a completion mid-teardown re-enters forever.
        dma::disable_int(CHANNEL);
        dma::stop(CHANNEL);
        ARMED.store(false, Ordering::Relaxed);
    }
    dma::masked(|| {
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

#[cfg(test)]
pub(crate) fn stop_test() {
    use dma::stop_tests::{assert_unreleased, failed, script};
    arm();
    epoch::seed(7, 100);
    usart().fifocfg.modify(|_, w| w.emptyrx().clear_bit());
    script(CHANNEL, None);
    failed(|| Fc3DmaRx.reload(dma::SWO_BUF_LEN));
    assert_unreleased(CHANNEL);
    assert!(ARMED.load(Ordering::Relaxed));
    assert_eq!(epoch::blocks(), 7);
    assert_eq!(epoch::previous(), 100);
    assert!(usart().fifocfg.read().emptyrx().bit_is_clear());
    script(CHANNEL, Some(1));
    Fc3DmaRx.reload(dma::SWO_BUF_LEN);
    assert!(ARMED.load(Ordering::Relaxed));
    assert_eq!(epoch::blocks(), 0);
    assert_eq!(epoch::previous(), 0);
}

#[cfg(test)]
mod tests {
    use super::*;
    use dma::{SWO_BLOCK_LEN, SWO_BUF_LEN};

    /// The cursor folds modulo the whole ring, so only a ring boundary counts as a wrap.
    #[test]
    fn only_crossing_a_ring_boundary_reports_a_wrap() {
        let block = SWO_BLOCK_LEN as u64;
        let len = SWO_BUF_LEN as u64;
        assert_eq!(ring_view(0, 0), (SWO_BUF_LEN, false));
        assert_eq!(
            ring_view(block + 1, block - 1),
            (SWO_BUF_LEN - SWO_BLOCK_LEN - 1, false)
        );
        assert_eq!(ring_view(len, len - 1), (SWO_BUF_LEN, true));
        assert_eq!(ring_view(len + 1, len), (SWO_BUF_LEN - 1, false));
        assert_eq!(ring_view(2 * len + 5, len - 1), (SWO_BUF_LEN - 5, true));
        for boundary in [1u64 << 32, 1u64 << 41, 1u64 << 57] {
            assert_eq!(
                ring_view(boundary + 12, boundary - 16),
                (SWO_BUF_LEN - 12, true)
            );
            assert_eq!(
                ring_view(boundary + 12, boundary + 12),
                (SWO_BUF_LEN - 12, false)
            );
        }
    }

    #[test]
    fn seeded_source_reads_once_across_rollovers_and_restart_discards_the_epoch() {
        use dma::tests::{script, Then};
        extern crate std;
        let mem = std::boxed::Box::leak(std::boxed::Box::new([0u8; SWO_BUF_LEN]));
        for (index, byte) in mem.iter_mut().enumerate() {
            *byte = index as u8;
        }
        ARMED.store(true, Ordering::Relaxed);
        for blocks in [1u64 << 23, 1u64 << 32, 1u64 << 48] {
            let boundary = blocks * SWO_BLOCK_LEN as u64;
            epoch::seed(blocks - 1, boundary - 16);
            let mut source = DmaSwoSource {
                ring: DmaRxRing::new(Fc3DmaRx, SWO_BUF_LEN),
                mem,
            };
            let mut out = [0u8; SWO_BUF_LEN];
            // Establish the real ring cursor at the seeded tail before checking the crossing.
            script(&[(15, Then::Nothing)], false, false);
            assert_eq!(source.drain(&mut out), SWO_BUF_LEN - 16);
            script(&[(4, Then::SwoIrq), (499, Then::Nothing)], false, false);
            assert_eq!(source.len(), 28);
            assert_eq!(epoch::previous(), boundary - 16);
            assert_eq!(epoch::blocks(), blocks);
            assert!(!source.take_overrun());
            script(&[(499, Then::Nothing)], false, false);
            assert_eq!(source.drain(&mut out), 28);
            for (i, byte) in out[..28].iter().enumerate() {
                assert_eq!(*byte, (SWO_BUF_LEN - 16 + i) as u8);
            }
            script(&[(499, Then::Nothing)], false, false);
            assert_eq!(source.drain(&mut out), 0);
            assert!(!source.take_overrun());

            epoch::seed(blocks + 3 * dma::SWO_BLOCKS as u64, boundary + 12);
            script(&[(499, Then::Nothing)], false, false);
            assert_eq!(source.drain(&mut out), SWO_BUF_LEN);
            assert!(
                !source.take_overrun(),
                "the boolean lap interface cannot count three full laps"
            );

            ARMED.store(false, Ordering::Relaxed);
            script(&[(499, Then::Nothing)], false, false);
            source.clear();
            epoch::reset();
            ARMED.store(true, Ordering::Relaxed);
            assert_eq!((epoch::blocks(), epoch::previous()), (0, 0));
            script(&[(511, Then::Nothing)], false, false);
            assert_eq!(source.drain(&mut out), 0);
            script(&[(508, Then::Nothing)], false, false);
            assert_eq!(source.drain(&mut out), 3);
            assert_eq!(&out[..3], &[0, 1, 2]);
        }
        ARMED.store(false, Ordering::Relaxed);
    }
}
