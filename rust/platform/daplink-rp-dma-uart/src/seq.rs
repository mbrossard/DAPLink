//! The per-chip DMA channel sequence: arm, abort, and the register aliases the errata require.

use rp_pac as pac;
use rp_pac::dma::regs::CtrlTrig;

/// `CTRL.EN`, bit 0 on both parts.
const CTRL_EN: u32 = 1;

/// Bound on the abort handshake spin, so a wedged channel cannot hang the caller.
const ABORT_SPINS: u32 = 100_000;

/// One DMA channel, addressed by number.
#[derive(Clone, Copy)]
pub struct DmaChannel(usize);

impl DmaChannel {
    pub const fn new(number: usize) -> Self {
        Self(number)
    }

    pub const fn number(&self) -> usize {
        self.0
    }

    pub fn regs(&self) -> pac::dma::Channel {
        pac::DMA.ch(self.0)
    }

    /// `CTRL.EN`, read through the alias that does not trigger the channel.
    pub fn enabled(&self) -> bool {
        self.regs().al1_ctrl().read() & CTRL_EN != 0
    }

    /// `CTRL.BUSY`, which outlasts the transfer count reaching zero and is what marks a run done.
    pub fn busy(&self) -> bool {
        self.regs().ctrl_trig().read().busy()
    }

    /// Clear `EN` through the non-triggering `AL1_CTRL` alias; a `CTRL_TRIG` write would start it.
    pub fn disable(&self) {
        let ch = self.regs();
        let ctrl = ch.al1_ctrl().read();
        ch.al1_ctrl().write_value(ctrl & !CTRL_EN);
    }

    /// Stop the channel and wait for the controller to let go; false if the wait expired.
    pub fn abort(&self) -> bool {
        // Clearing EN only pauses a busy channel; the abort is what ends it (RP2040-E13, RP2350-E5).
        self.disable();
        let mask = 1u16 << self.0;
        pac::DMA.chan_abort().write(|w| w.set_chan_abort(mask));
        // Both waits count: ABORT self-clears on the request, BUSY when the channel lets go.
        for _ in 0..ABORT_SPINS {
            if pac::DMA.chan_abort().read().chan_abort() & mask == 0 && !self.busy() {
                return true;
            }
        }
        false
    }

    /// Source and destination addresses, neither of which triggers the channel.
    pub fn set_addrs(&self, read: u32, write: u32) {
        let ch = self.regs();
        ch.read_addr().write_value(read);
        ch.write_addr().write_value(write);
    }

    /// Arm `count` transfers as one sequence, ending when it runs out.
    #[cfg(feature = "rp2040")]
    pub fn set_count(&self, count: u32) {
        self.regs().trans_count().write_value(count);
    }

    /// Arm `count` transfers as one sequence, ending when it runs out.
    #[cfg(feature = "rp235x")]
    pub fn set_count(&self, count: u32) {
        self.regs().trans_count().write(|w| {
            w.set_count(count);
            w.set_mode(pac::dma::vals::TransCountMode::NORMAL);
        });
    }

    /// Arm `count` transfers and reload them on completion (RP2350 12.6.2.2.1); RP2350 only,
    /// because the RP2040 counter has no self-triggering mode.
    #[cfg(feature = "rp235x")]
    pub fn set_count_retriggering(&self, count: u32) {
        self.regs().trans_count().write(|w| {
            w.set_count(count);
            w.set_mode(pac::dma::vals::TransCountMode::TRIGGER_SELF);
        });
    }

    /// Transfers left in the armed sequence.
    #[cfg(feature = "rp2040")]
    pub fn remaining(&self) -> u32 {
        self.regs().trans_count().read()
    }

    /// Transfers left in the armed sequence.
    #[cfg(feature = "rp235x")]
    pub fn remaining(&self) -> u32 {
        self.regs().trans_count().read().count()
    }

    /// Write `CTRL_TRIG`, which arms the channel and triggers it.
    pub fn trigger(&self, ctrl: CtrlTrig) {
        self.regs().ctrl_trig().write_value(ctrl);
    }
}
