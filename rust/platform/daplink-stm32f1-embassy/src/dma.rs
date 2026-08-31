//! One DMA1 circular receive channel, the position source behind the VCOM and SWO rings.

use core::sync::atomic::{compiler_fence, Ordering};

use daplink_rs::DmaRxChannel;
use embassy_stm32::pac;

/// A byte-wide circular peripheral-to-memory transfer on one DMA1 channel, addressed by index.
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
    /// Whether counter reloads are reported, from the channel's transfer-complete flag.
    laps: bool,
}

impl BdmaRx {
    /// Describe a transfer of `buf` from `src` on channel index `ch`; nothing is programmed yet.
    /// The caller consumes the matching `DMA1_CHx` peripheral so nothing else drives the channel.
    pub(crate) fn new(ch: usize, src: u32, buf: &'static [u8]) -> Self {
        assert!(
            buf.len() > 1 && buf.len() <= 0xFFFF,
            "DMA ring length out of range"
        );
        pac::RCC.ahbenr().modify(|w| w.set_dma1en(true));
        Self {
            ch,
            src,
            base: buf.as_ptr() as u32,
            len: buf.len(),
            laps: false,
        }
    }

    /// Opt in to lap-reload reporting, for a consumer that can be starved for a whole ring.
    pub(crate) fn with_lap_ticks(mut self) -> Self {
        self.laps = true;
        self
    }

    fn regs(&self) -> pac::bdma::Ch {
        pac::DMA1.ch(self.ch)
    }

    /// Take the sticky transfer-complete flag; set on reload whether or not the interrupt is on.
    fn take_reload(&self) -> bool {
        let seen = pac::DMA1.isr().read().tcif(self.ch);
        if seen {
            pac::DMA1.ifcr().write(|w| w.set_tcif(self.ch, true));
        }
        seen
    }

    /// (Re)program and start the transfer from the ring's base. A whole-register `CR` write, so
    /// `TCIE`/`HTIE`/`TEIE` land clear and the channel takes no interrupt.
    pub(crate) fn start(&self) {
        let ch = self.regs();
        ch.cr().modify(|w| w.set_en(false));
        // A fresh lap count; a flag left by the previous transfer is not this one's.
        pac::DMA1.ifcr().write(|w| w.set_gif(self.ch, true));
        ch.par().write_value(self.src);
        ch.mar().write_value(self.base);
        ch.ndtr().write(|w| w.set_ndt(self.len as u16));
        // The configuration must be visible before the enable.
        compiler_fence(Ordering::Release);
        ch.cr().write(|w| {
            w.set_dir(pac::bdma::vals::Dir::FROM_PERIPHERAL);
            w.set_circ(true);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_psize(pac::bdma::vals::Size::BITS8);
            w.set_msize(pac::bdma::vals::Size::BITS8);
            w.set_pl(pac::bdma::vals::Pl::HIGH);
            w.set_en(true);
        });
    }

    /// Stop the transfer, leaving the ring memory alone.
    pub(crate) fn stop(&self) {
        self.regs().cr().modify(|w| w.set_en(false));
        // The channel's ring writes must be visible before the CPU reads the backlog out.
        compiler_fence(Ordering::Acquire);
    }
}

impl DmaRxChannel for BdmaRx {
    fn remaining(&self) -> usize {
        // CNDTR counts DOWN from the ring length; the ring folds it into a write position.
        self.regs().ndtr().read().ndt() as usize
    }

    fn sample(&mut self) -> (usize, bool) {
        let remaining = self.remaining();
        if !self.laps {
            return (remaining, false);
        }
        // Position, flag, position again: a reload between two reads would pair a lap with a stale one.
        if self.take_reload() {
            (self.remaining(), true)
        } else {
            (remaining, false)
        }
    }

    fn reload(&mut self, _len: usize) {
        self.stop();
        self.start();
    }

    fn running(&self) -> bool {
        self.regs().cr().read().en()
    }
}
