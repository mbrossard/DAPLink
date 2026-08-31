//! STM32 USART BRR divisor arithmetic, split out of the F401 DUT image so its tests run
//! on the host.
#![no_std]

/// BRR word and OVER8 setting for `baud` at clock `fck`: `fck/baud` rounded to nearest, taken
/// straight as the OVER16 BRR when it reaches 16, else as eighths of USARTDIV8 under OVER8.
/// `baud` must be nonzero.
pub fn usart_brr(fck: u32, baud: u32) -> (bool, u32) {
    let raw = (fck + baud / 2) / baud;
    if raw >= 16 {
        return (false, raw);
    }
    (true, ((raw << 1) & !0xF) | (raw & 0x07))
}

/// Baud a programmed `(over8, brr)` pair actually yields, rounded to the nearest whole baud.
pub fn usart_baud_realized(fck: u32, over8: bool, brr: u32) -> u32 {
    let raw = if over8 {
        (brr >> 4) * 8 + (brr & 0x7)
    } else {
        brr
    };
    (fck + raw / 2) / raw
}

#[cfg(test)]
mod tests {
    use super::*;

    const APB1_HZ: u32 = 42_000_000;

    #[test]
    fn over16_divisor_rounds_to_nearest() {
        assert_eq!(usart_brr(APB1_HZ, 115_200), (false, 365)); // 364.58
        assert_eq!(usart_brr(APB1_HZ, 921_600), (false, 46)); // 45.57
    }

    #[test]
    fn rounding_makes_2_500_000_reachable() {
        // 42e6/2.5e6 = 16.8: truncating to 16 lands 5.0% high; 17 realizes -1.2%.
        assert_eq!(usart_brr(APB1_HZ, 2_500_000), (false, 17));
        assert_eq!(usart_baud_realized(APB1_HZ, false, 17), 2_470_588);
    }

    #[test]
    fn the_lowest_in_range_baud_fits_the_16_bit_brr() {
        // 641 is the guard floor; 640's divisor would overflow BRR.
        assert_eq!(usart_brr(APB1_HZ, 641), (false, 65_523));
        assert!(usart_brr(APB1_HZ, 640).1 > 0xFFFF);
    }

    #[test]
    fn over16_boundary_at_apb1_over_16_stays_over16() {
        assert_eq!(usart_brr(APB1_HZ, 2_625_000), (false, 16));
    }

    #[test]
    fn a_mid_range_rate_uses_over8() {
        assert_eq!(usart_brr(APB1_HZ, 4_000_000), (true, 0x013));
    }

    #[test]
    fn over8_ceiling_at_apb1_over_8() {
        assert_eq!(usart_brr(APB1_HZ, 5_250_000), (true, 0x010));
    }

    #[test]
    fn over8_fraction_rounds_instead_of_truncating() {
        // 42_000_000 / 2_800_010 truncates to 14 (frac 6) but rounds to 15 (frac 7).
        assert_eq!(usart_brr(APB1_HZ, 2_800_010), (true, 0x017));
    }

    #[test]
    fn rounding_up_to_the_over16_boundary_switches_branch_at_the_same_rate() {
        // 15.99...: BRR 16 realizes 2_625_000 whichever oversampling carries it.
        assert_eq!(usart_brr(APB1_HZ, 2_625_001), (false, 16));
    }

    #[test]
    fn over8_never_sets_brr_bit3() {
        for baud in (2_720_000..=5_250_000).step_by(9973) {
            let (over8, brr) = usart_brr(APB1_HZ, baud);
            assert!(over8);
            assert_eq!(brr & 0x8, 0, "baud={baud} brr={brr:#x}");
        }
    }

    #[test]
    fn realized_baud_round_trips_the_exact_rates() {
        assert_eq!(usart_baud_realized(APB1_HZ, false, 16), 2_625_000);
        assert_eq!(usart_baud_realized(APB1_HZ, true, 0x016), 3_000_000);
        assert_eq!(usart_baud_realized(APB1_HZ, true, 0x010), 5_250_000);
    }
}
