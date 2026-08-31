//! Flexcomm USART baud-divisor search; each seam passes its own function clock and OSR floor.

/// VCOM (Flexcomm0) function clock: the Flexcomm source clock whole, at the 48 MHz FRG ceiling.
pub(crate) const VCOM_CLOCK_HZ: u32 = crate::clocks::FLEXCOMM_SRC_HZ;

/// VCOM OSR floor. 9, above the hardware minimum of 5: NXP's own USART driver searches 16 down
/// to 9 and goes lower only when a rate is otherwise unreachable, since a smaller oversample
/// places the sample point less accurately within the bit. The rungs it withholds start at
/// 48 MHz / 8 = 6 MBd, past the ceiling of every target this bridge has been paired with.
pub(crate) const VCOM_OSR_MIN: u32 = 9;

/// SWO OSR floor: the hardware minimum (OSRVAL 0x4 = 5 clocks per bit), usable on the RX-only
/// capture path.
pub(crate) const SWO_OSR_MIN: u32 = 5;

/// Baud an `(osr, brg)` pair yields off `fclk_hz`, rounded to nearest: the realized-rate reply
/// describes the wire, not the divide that reached it. Both must be nonzero.
pub(crate) const fn realized(fclk_hz: u32, osr: u32, brg: u32) -> u32 {
    let divisor = osr * brg;
    (fclk_hz + divisor / 2) / divisor
}

/// Best 1-based `(osr, brg)` pair for `baud` (`baud = fclk / (osr * brg)`, OSR `osr_min..=16`,
/// 16-bit BRG), preferring larger OSR on ties; `None` when `baud` is 0 or no in-range BRG exists.
pub(crate) const fn flexcomm_osr_brg(fclk_hz: u32, baud: u32, osr_min: u32) -> Option<(u32, u32)> {
    if baud == 0 {
        return None;
    }
    let mut best_diff = u32::MAX;
    let mut best = None;
    let mut osr = 16u32;
    while osr >= osr_min {
        let brg = fclk_hz / (osr * baud);
        if brg != 0 && brg < 0xffff {
            // Truncating on purpose: this is the selection metric, and it chose the pairs the
            // fleet is measured on.
            let diff = baud.abs_diff(fclk_hz / (osr * brg));
            if diff < best_diff {
                best_diff = diff;
                best = Some((osr, brg));
            }
        }
        osr -= 1;
    }
    best
}

/// The rates the bench walks, pinned per build: [`VCOM_CLOCK_HZ`] comes off FRO_HF and not the
/// core clock, so the 96 MHz and 150 MHz profiles must derive the same divisors.
const _: () = {
    const fn vcom(baud: u32) -> (u32, u32, u32) {
        match flexcomm_osr_brg(VCOM_CLOCK_HZ, baud, VCOM_OSR_MIN) {
            Some((osr, brg)) => (osr, brg, realized(VCOM_CLOCK_HZ, osr, brg)),
            None => (0, 0, 0),
        }
    }
    assert!(VCOM_CLOCK_HZ == 48_000_000);
    assert!(matches!(vcom(9_600), (10, 500, 9_600)));
    assert!(matches!(vcom(115_200), (16, 26, 115_385)));
    assert!(matches!(vcom(230_400), (16, 13, 230_769)));
    assert!(matches!(vcom(460_800), (13, 8, 461_538)));
    assert!(matches!(vcom(921_600), (13, 4, 923_077)));
    assert!(matches!(vcom(2_000_000), (12, 2, 2_000_000)));
    assert!(matches!(vcom(3_000_000), (16, 1, 3_000_000)));
    // The 9-OSR floor is a policy: it, not the hardware, is what refuses the rates above it.
    assert!(matches!(vcom(5_250_000), (9, 1, 5_333_333)));
    assert!(flexcomm_osr_brg(VCOM_CLOCK_HZ, 6_000_000, VCOM_OSR_MIN).is_none());
    assert!(matches!(
        flexcomm_osr_brg(VCOM_CLOCK_HZ, 6_000_000, SWO_OSR_MIN),
        Some((8, 1))
    ));
    assert!(flexcomm_osr_brg(VCOM_CLOCK_HZ, 0, VCOM_OSR_MIN).is_none());
};
