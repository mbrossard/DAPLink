//! One DMA1 circular receive channel, the position source behind the SWO capture ring.

use core::sync::atomic::{compiler_fence, Ordering};

use daplink_rs::DmaRxChannel;
use stm32f1xx_hal::pac;

/// A byte-wide circular peripheral-to-memory transfer on one DMA1 channel; `Copy`, because the
/// SWO seam's mechanism half arms it while its source half reads the counter.
#[derive(Clone, Copy)]
pub struct BdmaRx {
    /// Zero-based DMA1 channel index (the PAC's `ch(0)` is CH1).
    ch: usize,
    /// The peripheral data register the channel reads.
    src: u32,
    /// Ring base address.
    base: u32,
    /// Ring length in bytes.
    len: usize,
}

impl BdmaRx {
    /// Describe (not program) a transfer of `buf` from `src` on channel `ch`, clocking DMA1; the
    /// caller guarantees nothing else drives that channel.
    pub(crate) fn new(ch: usize, src: u32, buf: &'static [u8]) -> Self {
        assert!(
            buf.len() > 1 && buf.len() <= 0xFFFF,
            "DMA ring length out of range"
        );
        // SAFETY: single RMW of RCC AHBENR during single-threaded bring-up.
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.ahbenr().modify(|_, w| w.dma1en().set_bit());
        Self {
            ch,
            src,
            base: buf.as_ptr() as u32,
            len: buf.len(),
        }
    }

    fn regs(&self) -> &'static pac::dma1::CH {
        // SAFETY: the index was fixed at construction, the peripheral is owned by this seam, and
        // DMA1's block address is architecturally fixed.
        unsafe { (*pac::DMA1::ptr()).ch(self.ch) }
    }

    /// (Re)program and start from the ring's base. The whole-register `CR` write lands
    /// `TCIE`/`HTIE`/`TEIE` clear, so the channel never interrupts: the ring is polled.
    pub(crate) fn start(&self) {
        let ch = self.regs();
        ch.cr().modify(|_, w| w.en().clear_bit());
        ch.par().write(|w| unsafe { w.bits(self.src) });
        ch.mar().write(|w| unsafe { w.bits(self.base) });
        ch.ndtr().write(|w| w.ndt().set(self.len as u16));
        // The configuration must be visible before the enable.
        compiler_fence(Ordering::Release);
        // DIR = 0 (from peripheral), PSIZE/MSIZE = 8-bit: the write's zero reset value.
        ch.cr().write(|w| {
            w.circ().set_bit();
            w.minc().set_bit();
            w.pl().high();
            w.en().set_bit()
        });
    }

    /// Stop the transfer, leaving the ring memory alone.
    pub(crate) fn stop(&self) {
        self.regs().cr().modify(|_, w| w.en().clear_bit());
        // The channel's ring writes must be visible before the CPU reads the backlog out.
        compiler_fence(Ordering::Acquire);
    }
}

impl DmaRxChannel for BdmaRx {
    fn remaining(&self) -> usize {
        // CNDTR counts DOWN from the ring length; the ring folds it into a write position.
        self.regs().ndtr().read().ndt().bits() as usize
    }

    fn reload(&mut self, _len: usize) {
        self.stop();
        self.start();
    }

    fn running(&self) -> bool {
        self.regs().cr().read().en().bit_is_set()
    }
}
