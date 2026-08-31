//! Clock bring-up: PLL0 at 150 MHz, or 96 MHz straight from the on-chip FRO.

use lpc55_hal::drivers::clocks::Clocks;
use lpc55_hal::prelude::*;
use lpc55_hal::raw;
use lpc55_hal::{Anactrl, ClockRequirements, Pmc, Syscon};

/// The on-chip FRO frequency [`clocks`] runs the core on. FRO_HF keeps running at it when the
/// core moves to PLL0, so the USB, VCOM and SWO kernel clocks never change.
const FRO_HZ: u32 = 96_000_000;

/// Core clock the image is built for; every CPU-derived constant follows it, so it is a feature.
#[cfg(feature = "core-96mhz")]
pub const CPU_HZ: u32 = FRO_HZ;
/// Core clock the image is built for: 150 MHz from PLL0 unless the FRO feature is selected.
#[cfg(not(feature = "core-96mhz"))]
pub const CPU_HZ: u32 = 150_000_000;

/// Shared Flexcomm source clock: FRO_HF through `FROHFDIV`. Each seam divides it further in its
/// own fractional rate generator -- VCOM takes it whole, SWO takes two thirds.
pub(crate) const FLEXCOMM_SRC_HZ: u32 = FRO_HZ / 2;

/// `FROHFDIV.DIV` for [`FLEXCOMM_SRC_HZ`]: the divider output is `FRO_HF / (DIV + 1)`.
const FROHFDIV_DIV: u8 = 1;

/// `FLEXFRGnCTRL.DIV`: the fractional rate generators only work with the denominator at 0xff.
pub(crate) const FRG_DIV: u8 = 0xff;

/// Bring the clock tree up to 96 MHz, with the high-speed USB PHY and the 1 MHz/12 MHz FRO
/// outputs enabled, and the shared Flexcomm source divider running; call once per image.
pub fn clocks(anactrl: &mut Anactrl, pmc: &mut Pmc, syscon: &mut Syscon) -> Clocks {
    let clocks = ClockRequirements::default()
        .system_frequency((FRO_HZ / 1_000_000).MHz())
        .configure(anactrl, pmc, syscon)
        .expect("lpc55 clock configuration");

    // Comes out of reset halted; start it here, before any Flexcomm seam selects it.
    // SAFETY: single-threaded bring-up; the write touches only FROHFDIV.
    unsafe { &*raw::SYSCON::ptr() }
        .frohfdiv
        .write(|w| unsafe { w.div().bits(FROHFDIV_DIV).halt().run() });

    clocks
}

/// Bring the clock tree up with the core on PLL0 at 150 MHz, in UM11126's upward-transition order.
/// Returns with the core still on FRO12 when the preflight refuses or PLL0 does not lock.
#[cfg(not(feature = "core-96mhz"))]
pub(crate) fn clocks_150mhz(
    flash: &lpc55_hal::raw::FLASH,
    anactrl: &mut Anactrl,
    pmc: &mut Pmc,
    syscon: &mut Syscon,
) -> Result<Clocks, crate::power::PowerError> {
    // The HAL's FRO96 arm first: it enables the 96/12/1 MHz FRO outputs, PLL0's FRO12 included.
    let clocks = clocks(anactrl, pmc, syscon);
    crate::power::preflight_150mhz(flash, syscon)?;
    crate::power::prelock_pll0_150mhz()?;
    crate::power::latch_mux_switch();
    crate::power::switch_main_clock_to_pll0();
    crate::power::latch_readback();
    Ok(clocks)
}
