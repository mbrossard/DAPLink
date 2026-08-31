//! PL011 baud divisor arithmetic, shared by every RP seam that programs one so its tests run
//! on the host.
#![no_std]

/// Largest programmable x64 divisor: `UARTIBRD` is 16 bits, `UARTFBRD` 6 bits.
pub const DIV_X64_MAX: u32 = (0xFFFF << 6) | 0x3F;

/// Smallest programmable x64 divisor: a zero `UARTIBRD` stops the baud generator, so the fastest
/// rate the lattice serves is [`max_baud`], at this divisor.
pub const DIV_X64_MIN: u32 = 64;

/// Highest rate the PL011 can serve from `clk`: it samples 16x and the smallest divisor is 1.
pub const fn max_baud(clk: u32) -> u32 {
    clk / 16
}

/// Baud divisor as 16.6 fixed point -- `round(64 * clk / (16 * baud))`, saturated at
/// [`DIV_X64_MAX`] so the fraction's carry lands in the integer part instead of truncating.
/// `None` for a rate the lattice cannot serve -- zero, or faster than [`max_baud`] -- which each
/// caller answers its own way. `clk` must not exceed 536 MHz.
pub fn div_x64(clk: u32, baud: u32) -> Option<u32> {
    if baud == 0 || baud > max_baud(clk) {
        return None;
    }
    Some((8 * clk / baud).div_ceil(2).min(DIV_X64_MAX))
}

/// Split a divisor into the `UARTIBRD` and `UARTFBRD` fields.
pub fn split(div_x64: u32) -> (u16, u8) {
    let d = div_x64.min(DIV_X64_MAX);
    ((d >> 6) as u16, (d & 0x3F) as u8)
}

/// Baud a programmed divisor actually yields, rounded to the nearest whole baud:
/// `round(4 * clk / div_x64)`. Zero for an unprogrammed divisor.
pub fn realized(clk: u32, div_x64: u32) -> u32 {
    (4 * clk + div_x64 / 2).checked_div(div_x64).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// clk_peri on the RP2040 DUT and on the RP2350 DUT.
    const RP2040_HZ: u32 = 125_000_000;
    const RP2350_HZ: u32 = 150_000_000;

    #[test]
    fn overflow_band_carries_into_ibrd() {
        // Requests whose fraction rounds to a full unit must land on the next integer divisor.
        assert_eq!(div_x64(RP2040_HZ, 2_604_167), Some(192));
        assert_eq!(div_x64(RP2040_HZ, 2_610_966), Some(192));
        assert_eq!(div_x64(RP2350_HZ, 3_125_001), Some(192));
        assert_eq!(div_x64(RP2350_HZ, 3_133_159), Some(192));
    }

    #[test]
    fn the_carry_leaves_fbrd_zero_rather_than_64() {
        assert_eq!(split(192), (3, 0));
        assert_eq!(split(191), (2, 63));
    }

    #[test]
    fn overflow_band_realizes_near_the_request() {
        assert_eq!(
            realized(RP2040_HZ, div_x64(RP2040_HZ, 2_604_167).unwrap()),
            2_604_167
        );
        assert_eq!(
            realized(RP2350_HZ, div_x64(RP2350_HZ, 3_125_001).unwrap()),
            3_125_000
        );
    }

    #[test]
    fn realized_rounds_to_nearest_in_both_directions() {
        // 500e6/192 = 2604166.67 and 500e6/96 = 5208333.33: the first rounds up off the
        // truncated value, the second stays put.
        assert_eq!(realized(RP2040_HZ, 192), 2_604_167);
        assert_eq!(realized(RP2040_HZ, 96), 5_208_333);
        assert_eq!(realized(RP2040_HZ, DIV_X64_MIN), 7_812_500);
    }

    #[test]
    fn an_unprogrammed_divisor_realizes_nothing() {
        assert_eq!(realized(RP2040_HZ, 0), 0);
    }

    #[test]
    fn split_never_leaves_the_register_fields() {
        for baud in [1_000, 115_200, 1_956_947, 2_604_167, max_baud(RP2040_HZ)] {
            let d = div_x64(RP2040_HZ, baud).expect("in-range baud");
            let (ibrd, fbrd) = split(d);
            assert!(ibrd >= 1, "baud={baud} ibrd={ibrd}");
            assert!(fbrd <= 63, "baud={baud} fbrd={fbrd}");
            assert_eq!(u32::from(ibrd) * 64 + u32::from(fbrd), d);
        }
    }

    #[test]
    fn rates_the_lattice_cannot_serve_are_rejected() {
        assert_eq!(div_x64(RP2040_HZ, 0), None);
        assert_eq!(div_x64(RP2040_HZ, max_baud(RP2040_HZ) + 1), None);
        assert_eq!(div_x64(RP2040_HZ, u32::MAX), None);
    }

    #[test]
    fn the_fastest_and_slowest_rates_sit_at_the_ends_of_the_lattice() {
        assert_eq!(div_x64(RP2040_HZ, max_baud(RP2040_HZ)), Some(DIV_X64_MIN));
        // Below the widest divisor every request saturates to the same lowest reachable rate.
        assert_eq!(div_x64(RP2040_HZ, 100), Some(DIV_X64_MAX));
        assert_eq!(split(DIV_X64_MAX), (0xFFFF, 0x3F));
    }

    #[test]
    fn common_rates_round_trip_within_a_tenth_of_a_percent() {
        for clk in [RP2040_HZ, RP2350_HZ] {
            for baud in [9_600, 115_200, 921_600, 1_000_000, 2_000_000, 4_000_000] {
                let got = realized(clk, div_x64(clk, baud).expect("in-range baud"));
                let err = got.abs_diff(baud) as u64 * 1000 / u64::from(baud);
                assert!(err <= 1, "clk={clk} baud={baud} realized={got}");
            }
        }
    }

    /// The DUT images computed the fields inline as a 25.7 divide before this crate existed;
    /// the extraction must not have moved a single rate.
    #[test]
    fn the_fields_match_the_inline_25_7_form() {
        for clk in [RP2040_HZ, RP2350_HZ] {
            for baud in (2_400..=max_baud(clk)).step_by(7_919) {
                let div = (8 * clk) / baud;
                let (mut ibrd, mut fbrd) = (div >> 7, ((div & 0x7f) + 1) >> 1);
                if fbrd > 63 {
                    ibrd += 1;
                    fbrd = 0;
                }
                let got = split(div_x64(clk, baud).expect("in-range baud"));
                assert_eq!(got, (ibrd as u16, fbrd as u8), "clk={clk} baud={baud}");
            }
        }
    }
}
