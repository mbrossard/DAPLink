//! Flexcomm USART baud-divisor search; each seam passes its own function clock and OSR floor.

/// VCOM (Flexcomm0) function clock: the Flexcomm source clock whole, at the 48 MHz FRG ceiling.
pub(crate) const VCOM_CLOCK_HZ: u32 = crate::clocks::FLEXCOMM_SRC_HZ;

/// VCOM oversample floor; independent of the receive-only SWO policy.
pub(crate) const VCOM_OSR_MIN: u32 = 9;

/// SWO oversample floor: the hardware minimum of five clocks per bit.
pub(crate) const SWO_OSR_MIN: u32 = 5;

/// Rounded baud for a valid, nonzero `(osr, brg)` pair.
pub(crate) const fn realized(fclk_hz: u32, osr: u32, brg: u32) -> u32 {
    let divisor = osr * brg;
    (fclk_hz + divisor / 2) / divisor
}

/// Best 1-based divisors, preferring larger OSR on ties; `None` for invalid or unattainable input.
pub(crate) const fn flexcomm_osr_brg(fclk_hz: u32, baud: u32, osr_min: u32) -> Option<(u32, u32)> {
    if baud == 0 || osr_min < 5 || osr_min > 16 {
        return None;
    }
    let mut best_diff = u32::MAX;
    let mut best = None;
    let mut osr = 16u32;
    while osr >= osr_min {
        let brg = fclk_hz / osr / baud;
        if brg != 0 && brg < 0xffff {
            // Selection uses truncated baud, not the rounded rate reported to the host.
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

// Flexcomm divisors must stay independent of the core profile.
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

#[cfg(test)]
mod tests {
    use super::*;

    fn reference(clock: u32, baud: u32, floor: u32) -> Option<(u32, u32)> {
        if baud == 0 || !(5..=16).contains(&floor) {
            return None;
        }
        (floor..=16)
            .rev()
            .filter_map(|osr| {
                let brg = u64::from(clock) / (u64::from(osr) * u64::from(baud));
                (1..0xffff).contains(&brg).then(|| {
                    let rate = u64::from(clock) / (u64::from(osr) * brg);
                    (u64::from(baud).abs_diff(rate), osr, brg as u32)
                })
            })
            .min_by_key(|&(error, osr, _)| (error, 16 - osr))
            .map(|(_, osr, brg)| (osr, brg))
    }

    #[test]
    fn all_host_baud_boundaries_match_widened_reference() {
        for clock in [0, 12_000_000, VCOM_CLOCK_HZ, u32::MAX] {
            for floor in [0, 4, 5, 9, 16, 17, u32::MAX] {
                for baud in [
                    0,
                    1,
                    45,
                    46,
                    47,
                    9600,
                    115_200,
                    921_600,
                    3_000_000,
                    5_250_000,
                    6_000_000,
                    0x0fff_fffe,
                    0x0fff_ffff,
                    0x1000_0000,
                    0x1000_0001,
                    u32::MAX,
                ] {
                    assert_eq!(
                        flexcomm_osr_brg(clock, baud, floor),
                        reference(clock, baud, floor),
                        "clock={clock} baud={baud} floor={floor}"
                    );
                }
            }
        }
    }

    #[test]
    fn divisor_edges_and_ties_preserve_selection() {
        for floor in [SWO_OSR_MIN, VCOM_OSR_MIN] {
            for osr in floor..=16 {
                for brg in [1, 2, 13, 500, 0xfffe, 0xffff] {
                    let baud = VCOM_CLOCK_HZ / (osr * brg);
                    for requested in baud.saturating_sub(1)..=baud + 1 {
                        assert_eq!(
                            flexcomm_osr_brg(VCOM_CLOCK_HZ, requested, floor),
                            reference(VCOM_CLOCK_HZ, requested, floor)
                        );
                    }
                }
            }
        }
        assert_eq!(flexcomm_osr_brg(VCOM_CLOCK_HZ, 3_000_000, 9), Some((16, 1)));
    }
}
