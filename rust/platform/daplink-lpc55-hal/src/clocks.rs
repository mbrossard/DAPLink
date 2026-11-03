//! Clock bring-up: PLL0 at 150 MHz, or 96 MHz straight from the on-chip FRO.

use lpc55_hal::drivers::clocks::Clocks;
use lpc55_hal::prelude::*;
use lpc55_hal::raw;
use lpc55_hal::{Anactrl, ClockRequirements, Pmc, Syscon};

/// The core FRO frequency; FRO_HF keeps it on PLL0, so kernel clocks never change.
const FRO_HZ: u32 = 96_000_000;

/// The core clock the probe boots at; `MHZ_150` panics in bring-up when its preflight refuses.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClockProfile(u32);

impl ClockProfile {
    pub const MHZ_96: Self = Self(FRO_HZ);
    pub const MHZ_150: Self = Self(150_000_000);

    pub const fn cpu_hz(self) -> u32 {
        self.0
    }

    #[inline(always)]
    pub(crate) fn configure(
        self,
        flash: &raw::FLASH,
        anactrl: &mut Anactrl,
        pmc: &mut Pmc,
        syscon: &mut Syscon,
    ) -> Clocks {
        if self == Self::MHZ_96 {
            clocks(anactrl, pmc, syscon)
        } else {
            clocks_150mhz(flash, anactrl, pmc, syscon)
                .expect("lpc55 150 MHz preflight (see CORE_150MHZ_STATUS)")
        }
    }
}

/// Shared Flexcomm source clock: FRO_HF through `FROHFDIV`, divided further by each seam's FRG.
pub(crate) const FLEXCOMM_SRC_HZ: u32 = FRO_HZ / 2;

/// `FROHFDIV.DIV` for [`FLEXCOMM_SRC_HZ`]: the divider output is `FRO_HF / (DIV + 1)`.
const FROHFDIV_DIV: u8 = 1;

/// `FLEXFRGnCTRL.DIV`: the fractional rate generators only work with the denominator at 0xff.
pub(crate) const FRG_DIV: u8 = 0xff;

/// Bring the clock tree up to 96 MHz with the HS USB PHY, FRO outputs and Flexcomm divider; once.
fn clocks(anactrl: &mut Anactrl, pmc: &mut Pmc, syscon: &mut Syscon) -> Clocks {
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

/// Bring the core up on PLL0 at 150 MHz; an error leaves unsupported silicon on FRO96, else FRO12.
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
