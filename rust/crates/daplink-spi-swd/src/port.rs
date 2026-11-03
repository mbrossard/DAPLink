//! The register seams a chip supplies, and the injected fast-JTAG pacing ladder.

use crate::session::Dir;
use dap_rs::driver::bitbang::DelayCycles;

/// The per-chip JTAG seam: the four TAP lines as single register accesses.
pub trait JtagPortOps {
    /// Take the dedicated JTAG lines: TDI a push-pull output idle low, TDO an input with no pull.
    fn jtag_pads_acquire(&mut self);

    /// Release the dedicated JTAG lines on mode exit: TDI becomes an input or stays driven low.
    fn jtag_pads_release(&mut self);

    /// Drive TCK low (one store).
    fn tck_low(&self);

    /// Drive TCK high (one store).
    fn tck_high(&self);

    /// Drive TMS (one store); JTAG never turns TMS around, so it is always driven.
    fn tms_set(&self, high: bool);

    /// Drive TDI (one store).
    fn tdi_set(&self, high: bool);

    /// Drive TDI from bit 0 of `bits`, ignoring the rest.
    #[inline(always)]
    fn tdi_put(&self, bits: u32) {
        self.tdi_set(bits & 1 != 0);
    }

    /// Sample TDO (one load). Returns 0 or 1.
    fn tdo_get(&self) -> u32;

    /// Read the TDI pad level back, for `DAP_SWJ_Pins` reporting.
    fn tdi_is_high(&mut self) -> bool;

    /// Read the TDO pad level back, for `DAP_SWJ_Pins` reporting.
    fn tdo_is_high(&mut self) -> bool;
}

/// A register-level SPI failure; the session's FIFO contents are meaningless.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SpiFault {
    /// A status poll did not reach its flag within the port's bound.
    Timeout,
    /// The receive FIFO overran, so at least one frame was lost.
    Overrun,
}

/// The result of one register-seam operation.
pub type PortResult<T> = Result<T, SpiFault>;

/// The per-chip SPI register seam; all sequencing and framing decisions stay in the engine.
pub trait SpiPortOps: JtagPortOps {
    /// True when the board wires a separate transmit pad, so direction is a pad-ownership decision.
    const DUPLEX: bool;

    /// Use three 11-bit write frames instead of five padded bytes.
    const EXACT_WRITE_FRAMING: bool = false;

    /// Delay one SCK period after EOT before disabling (H5 ES0621 2.17.2).
    const EOT_DELAY: bool = true;

    /// Set independent GPIO pacing from the accepted host request, in Hz.
    fn set_gpio_clock(&mut self, _requested_hz: u32) {}

    /// The kernel clock source an [`SpiLadder`](crate::SpiLadder) names; `()` for a fixed kernel.
    type Source: Copy;

    /// Route the kernel mux to `source` between sessions; the default has nothing to route.
    #[inline(always)]
    fn route(&mut self, _source: Self::Source) {}

    /// Program the fixed configuration once: half-duplex master, mode 3, LSB-first, software NSS.
    fn configure(&mut self);

    /// Program one session's geometry, raw `MBR[2:0]` rung and direction, with the block disabled.
    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir);

    /// Enable the SPI block (frame geometry already programmed).
    fn enable(&self);

    /// Disable the SPI block, flushing the FIFOs and resetting the shifter.
    fn disable(&self);

    /// Issue the master start that begins clocking the programmed frames.
    fn start(&self);

    /// Preload one transmit frame through an 8-bit access, waiting for FIFO space first.
    fn push8(&self, byte: u8) -> PortResult<()>;

    /// Preload one transmit frame through a 16-bit access, for frame widths above eight bits.
    fn push16(&self, half: u16) -> PortResult<()>;

    /// Spin until end-of-transfer, reporting the overrun the same status read carries.
    fn wait_eot(&self) -> PortResult<()>;

    /// Drain one received frame (right-aligned) from the Rx FIFO, waiting for it first.
    fn pop32(&self) -> PortResult<u32>;

    /// Clear the sticky end-of-transfer, transmission-filled and overrun flags.
    fn clear_flags(&self) -> PortResult<()>;

    /// Return the shared pads to the SPI alternate function before every session.
    fn pads_to_spi(&mut self);

    /// Take the separate transmit pad for a transmit session; duplex only.
    fn drive(&mut self);

    /// Release the separate transmit pad before the target may drive the data line; duplex only.
    fn release(&mut self);

    /// Restore the clock pad to SPI, leaving data as input until the next session.
    fn clk_pad_to_spi(&mut self);
}

/// The injected fast-JTAG pacing ladder: straight-line instructions per clock half.
pub trait JtagPacing {
    /// Half-period spin count at or below which the fast path takes over; zero disables it.
    const FAST_HALF_CYCLES: u32;

    /// The fastest counted JTAG wire the shared pads carry; no counted half is prepared shorter.
    const COUNTED_MAX_HZ: u32 = 12_000_000;

    /// Prepare the common half-period passed to every counted wait until the next preparation.
    fn prepare_counted(&mut self, _cycles: u32) {}

    /// Pace a counted low half before sampling; an implementation may run its prepared spin instead.
    #[inline(always)]
    fn counted_low<D: DelayCycles>(&self, delay: &mut D, cycles: u32) {
        delay.delay_cycles(cycles);
    }

    /// Pace a counted high half.
    #[inline(always)]
    fn counted_high<D: DelayCycles>(&self, delay: &mut D, cycles: u32) {
        delay.delay_cycles(cycles);
    }

    /// Pace an unsampled data-output low half, defaulting to sampled low pacing.
    #[inline(always)]
    fn counted_output_low<D: DelayCycles>(&self, delay: &mut D, cycles: u32) {
        self.counted_low(delay, cycles);
    }

    /// Pace the low half of a fast JTAG clock: the TCK-fall to TDO-sample margin (the longer half).
    fn pace_low(&self);

    /// Pace the high half: TCK-high pulse width only, no sampling obligation.
    fn pace_high(&self);
}

macro_rules! nop_ladder {
    (@arms $n:ident [$($arms:tt)*] [$($text:literal,)*]; $next:literal $($rest:literal)*) => {
        nop_ladder!(@arms $n
            [$($arms)* $next => core::arch::asm!("", $($text,)*
                options(nomem, nostack, preserves_flags)),]
            [$($text,)* "nop",]; $($rest)*)
    };
    (@arms $n:ident [$($arms:tt)*] [$($text:literal,)*];) => {
        match $n { $($arms)* _ => unreachable!() }
    };
}

/// Emit exactly `N <= 64` straight-line `nop`s in one compiler-visible asm block.
#[inline(always)]
pub fn nops<const N: usize>() {
    const {
        assert!(N <= 64, "nop count exceeds the supported range");
    }
    // SAFETY: `nop` touches no memory, stack or flags.
    unsafe {
        nop_ladder!(@arms N [] [];
            0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
            16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
            32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47
            48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64);
    }
}

/// Spin a nonzero `iterations` through an unaligned two-instruction loop; not for paced cells.
#[cfg(target_arch = "arm")]
#[inline(always)]
pub fn spin(iterations: u32) {
    // SAFETY: register arithmetic and a local branch only; flags are clobbered.
    unsafe {
        core::arch::asm!(
            // A local label: named labels would emit relocations the smallest targets cannot take.
            "2:",
            "subs {i}, #1",
            "bne 2b",
            i = inout(reg) iterations => _,
            options(nomem, nostack),
        );
    }
}

/// Host stand-in for builds and tests off the target: the same shape, no timing guarantee.
#[cfg(not(target_arch = "arm"))]
#[inline(always)]
pub fn spin(iterations: u32) {
    for _ in 0..iterations {
        core::hint::spin_loop();
    }
}

/// The no-ladder pacing: both halves are paced by the injected counted delay.
#[derive(Copy, Clone, Debug, Default)]
pub struct NoFastJtag;

impl JtagPacing for NoFastJtag {
    const FAST_HALF_CYCLES: u32 = 0;

    fn pace_low(&self) {}

    fn pace_high(&self) {}
}
