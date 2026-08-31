//! The register seams a chip supplies, and the injected fast-JTAG pacing ladder.

use crate::session::Dir;

/// The per-chip JTAG seam: the four TAP lines as single register accesses plus the handover for
/// the dedicated ones. Split from [`SpiPortOps`] because [`JtagScan`](crate::JtagScan) needs only
/// this half; implementations hold pin indices as constants so each edge is one constant store.
pub trait JtagPortOps {
    /// Take the dedicated JTAG lines on mode entry: TDI a push-pull output idle low, TDO a plain
    /// input with no pull. The shared TCK/TMS pads are reclaimed by the caller, not here.
    fn jtag_pads_acquire(&mut self);

    /// Release the dedicated JTAG lines on mode exit: TDI back to an input.
    fn jtag_pads_release(&mut self);

    /// Drive TCK low (one store).
    fn tck_low(&self);

    /// Drive TCK high (one store).
    fn tck_high(&self);

    /// Drive TMS (one store); JTAG never turns TMS around, so it is always driven.
    fn tms_set(&self, high: bool);

    /// Drive TDI (one store).
    fn tdi_set(&self, high: bool);

    /// Sample TDO (one load). Returns 0 or 1.
    fn tdo_get(&self) -> u32;

    /// Read the TDI pad level back, for `DAP_SWJ_Pins` reporting.
    fn tdi_is_high(&mut self) -> bool;

    /// Read the TDO pad level back, for `DAP_SWJ_Pins` reporting.
    fn tdo_is_high(&mut self) -> bool;
}

/// The per-chip SPI register seam the session engine drives: every method a single register
/// access or small fixed group, all sequencing and framing decisions in the engine. Expects a
/// block whose frame geometry is write-protected while enabled (what forces the sessions).
pub trait SpiPortOps: JtagPortOps {
    /// Program the fixed configuration once: half-duplex master, SPI mode 3, LSB-first, software
    /// slave-select inactive, the data line on the board's SWDIO pad, AF retention across
    /// enable/disable dips. Called once, from [`SpiSwdDeps::new`](crate::SpiSwdDeps::new).
    fn configure(&mut self);

    /// Program one session's frame geometry, baud rung (raw `MBR[2:0]`), and half-duplex
    /// direction; only ever called while the block is disabled, when those fields are writable.
    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir);

    /// Enable the SPI block (frame geometry already programmed).
    fn enable(&self);

    /// Disable the SPI block, flushing the FIFOs and resetting the shifter.
    fn disable(&self);

    /// Issue the master start that begins clocking the programmed frames.
    fn start(&self);

    /// Preload one transmit frame through an 8-bit access, waiting for FIFO space first.
    fn push8(&self, byte: u8);

    /// Spin until the frame counter reaches end-of-transfer.
    fn wait_eot(&self);

    /// Drain one received frame (right-aligned) from the Rx FIFO, waiting for it first.
    fn pop32(&self) -> u32;

    /// Clear the sticky end-of-transfer / transmission-filled / overrun flags.
    fn clear_flags(&self);

    /// Return the shared SWCLK/SWDIO pads to the SPI alternate function; called before every
    /// session, because any GPIO fallback path leaves them as plain GPIOs.
    fn pads_to_spi(&mut self);

    /// Return only the SWCLK pad to the SPI alternate function. The data pad stays a GPIO input:
    /// the block still holds the previous session's direction, so handing it back could
    /// re-assert transmit on a line the target may own; the next session's
    /// [`pads_to_spi`](Self::pads_to_spi) reclaims it after programming its direction.
    fn clk_pad_to_spi(&mut self);
}

/// The injected fast-JTAG pacing ladder: a straight-line block of instructions per clock half,
/// because firmware optimization levels do not unroll a counted loop, so a counted spin cannot
/// render a sub-loop-overhead half-period. A chip without a measured ladder uses [`NoFastJtag`].
pub trait JtagPacing {
    /// Half-period spin count (CPU cycles) at or below which the fast path takes over; zero
    /// disables it entirely.
    const FAST_HALF_CYCLES: u32;

    /// Pace the low half of a fast JTAG clock: the TCK-fall to TDO-sample margin (the longer half).
    fn pace_low(&self);

    /// Pace the high half: TCK-high pulse width only, no sampling obligation.
    fn pace_high(&self);
}

/// Emit exactly `N` straight-line `nop`s as one assembler-expanded block, so a ladder's length
/// is a plain integer a bench sweep can step rather than a frozen `asm!` literal.
#[inline(always)]
pub fn nops<const N: usize>() {
    // SAFETY: `nop` has no operands or side effects; the block touches no memory, stack, or
    // flags, which is exactly what the options assert.
    unsafe {
        core::arch::asm!(
            ".rept {n}",
            "nop",
            ".endr",
            n = const N,
            options(nomem, nostack, preserves_flags)
        );
    }
}

/// The "no ladder measured" pacing: the fast path never engages and both halves are paced by
/// the injected counted delay.
#[derive(Copy, Clone, Debug, Default)]
pub struct NoFastJtag;

impl JtagPacing for NoFastJtag {
    const FAST_HALF_CYCLES: u32 = 0;

    fn pace_low(&self) {}

    fn pace_high(&self) {}
}
