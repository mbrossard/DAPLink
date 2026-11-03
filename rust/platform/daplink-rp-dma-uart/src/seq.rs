//! The per-chip DMA channel sequence: arm, abort, and the register aliases the errata require.

use core::sync::atomic::{compiler_fence, Ordering};
use rp_pac as pac;
use rp_pac::dma::regs::CtrlTrig;

const CTRL_EN: u32 = 1;

/// Bound on the abort handshake spin, so a wedged channel cannot hang the caller.
const ABORT_SPINS: u32 = 100_000;

fn wait_for_abort(mut pending: impl FnMut() -> bool, mut busy: impl FnMut() -> bool) -> bool {
    for _ in 0..ABORT_SPINS {
        if !pending() && !busy() {
            return true;
        }
    }
    false
}

/// A raw channel identity; copies share one externally coordinated ownership obligation.
#[derive(Clone, Copy)]
pub struct DmaChannel(usize);

impl DmaChannel {
    /// Construct a raw identity, not an independently owned channel lease.
    ///
    /// # Safety
    /// Reserve this channel for one subsystem; coordinate every copy, interrupt and chained channel.
    pub const unsafe fn new(number: usize) -> Self {
        #[cfg(feature = "rp2040")]
        assert!(number < 12);
        #[cfg(feature = "rp235x")]
        assert!(number < 16);
        Self(number)
    }

    pub const fn number(&self) -> usize {
        self.0
    }

    /// Access the channel's raw register block.
    ///
    /// # Safety
    /// Every write must preserve exclusive channel control and all active buffer leases.
    pub unsafe fn regs(&self) -> pac::dma::Channel {
        pac::DMA.ch(self.0)
    }

    /// `CTRL.EN`, read through the alias that does not trigger the channel.
    pub fn enabled(&self) -> bool {
        pac::DMA.ch(self.0).al1_ctrl().read() & CTRL_EN != 0
    }

    /// `CTRL.BUSY`, which outlasts the transfer count reaching zero and is what marks a run done.
    pub fn busy(&self) -> bool {
        let busy = pac::DMA.ch(self.0).ctrl_trig().read().busy();
        if !busy {
            compiler_fence(Ordering::Acquire);
        }
        busy
    }

    /// Clear `EN` through the non-triggering `AL1_CTRL` alias; a `CTRL_TRIG` write would start it.
    ///
    /// # Safety
    /// Coordinate channel mutation; disabling does not end its memory lease while BUSY remains set.
    pub unsafe fn disable(&self) {
        let ch = pac::DMA.ch(self.0);
        let ctrl = ch.al1_ctrl().read();
        ch.al1_ctrl().write_value(ctrl & !CTRL_EN);
    }

    /// Stop the channel and wait for the controller to let go; false if the wait expired.
    ///
    /// # Safety
    /// Exclude concurrent rearming, including chain partners; retain buffers on failure.
    #[must_use = "an expired abort does not release the channel's memory"]
    pub unsafe fn abort(&self) -> bool {
        // Clearing EN only pauses a busy channel; the abort is what ends it (RP2040-E13, RP2350-E5).
        // SAFETY: the caller coordinates this channel and its chain partners.
        unsafe { self.disable() };
        let mask = 1u16 << self.0;
        pac::DMA.chan_abort().write(|w| w.set_chan_abort(mask));
        // Both waits count: ABORT self-clears on the request, BUSY when the channel lets go.
        wait_for_abort(
            || pac::DMA.chan_abort().read().chan_abort() & mask != 0,
            || self.busy(),
        )
    }

    /// Source and destination addresses, neither of which triggers the channel.
    ///
    /// # Safety
    /// The channel must be idle and unable to retrigger; addresses must match the eventual transfer geometry.
    pub unsafe fn set_addrs(&self, read: u32, write: u32) {
        let ch = pac::DMA.ch(self.0);
        ch.read_addr().write_value(read);
        ch.write_addr().write_value(write);
    }

    #[cfg(feature = "rp2040")]
    /// Arm `count` transfers as one sequence, ending when it runs out.
    ///
    /// # Safety
    /// The channel must be idle and unable to retrigger; the complete count must fit its buffers.
    pub unsafe fn set_count(&self, count: u32) {
        pac::DMA.ch(self.0).trans_count().write_value(count);
    }

    #[cfg(feature = "rp235x")]
    /// Arm `count` transfers as one sequence, ending when it runs out.
    ///
    /// # Safety
    /// The channel must be idle and unable to retrigger; the complete count must fit its buffers.
    pub unsafe fn set_count(&self, count: u32) {
        assert!(count <= 0x0fff_ffff);
        pac::DMA.ch(self.0).trans_count().write(|w| {
            w.set_count(count);
            w.set_mode(pac::dma::vals::TransCountMode::NORMAL);
        });
    }

    /// Arm `count` transfers with self-retriggering on completion.
    ///
    /// # Safety
    /// The channel must be idle; geometry must remain valid on every reload until successful abort.
    #[cfg(feature = "rp235x")]
    pub unsafe fn set_count_retriggering(&self, count: u32) {
        assert!(count <= 0x0fff_ffff);
        pac::DMA.ch(self.0).trans_count().write(|w| {
            w.set_count(count);
            w.set_mode(pac::dma::vals::TransCountMode::TRIGGER_SELF);
        });
    }

    #[cfg(feature = "rp2040")]
    pub fn remaining(&self) -> u32 {
        pac::DMA.ch(self.0).trans_count().read()
    }

    #[cfg(feature = "rp235x")]
    pub fn remaining(&self) -> u32 {
        pac::DMA.ch(self.0).trans_count().read().count()
    }

    /// Write `CTRL_TRIG`, which arms the channel and triggers it.
    ///
    /// # Safety
    /// Own the idle channel and partners; `ctrl` names a valid DREQ, aligned addresses, in-bounds counts.
    /// Until BUSY clears or abort succeeds, sources stay initialized and unwritten, destinations DMA-only.
    pub unsafe fn trigger(&self, ctrl: CtrlTrig) {
        compiler_fence(Ordering::SeqCst);
        pac::DMA.ch(self.0).ctrl_trig().write_value(ctrl);
        compiler_fence(Ordering::SeqCst);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;

    #[test]
    fn abort_requires_request_clear_and_bus_idle() {
        let samples = Cell::new(0);
        assert!(wait_for_abort(
            || {
                samples.set(samples.get() + 1);
                samples.get() < 3
            },
            || samples.get() < 5,
        ));
        assert_eq!(samples.get(), 5);
    }

    #[test]
    fn expired_abort_never_reports_release() {
        let samples = Cell::new(0);
        assert!(!wait_for_abort(
            || false,
            || {
                samples.set(samples.get() + 1);
                true
            }
        ));
        assert_eq!(samples.get(), ABORT_SPINS);
        assert!(!wait_for_abort(|| true, || panic!("request still pending")));
    }

    #[test]
    fn last_poll_may_confirm_release_and_later_retry_may_succeed() {
        let polls = Cell::new(0);
        assert!(wait_for_abort(
            || false,
            || {
                polls.set(polls.get() + 1);
                polls.get() != ABORT_SPINS
            }
        ));
        assert!(!wait_for_abort(|| false, || true));
        assert!(wait_for_abort(|| false, || false));
    }
}
