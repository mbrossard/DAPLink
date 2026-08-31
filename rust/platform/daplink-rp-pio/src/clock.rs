//! Integer clock-divider math for `DAP_SWJ_Clock` / `Swd::set_clock`.

/// SM cycles per SWCLK bit in both program variants at the stock clock.
#[cfg(not(any(feature = "clock-180mhz", feature = "clock-200mhz")))]
pub const CYCLES_PER_BIT: u32 = 4;
/// Stretched cell used by the overclock profiles.
#[cfg(any(feature = "clock-180mhz", feature = "clock-200mhz"))]
pub const CYCLES_PER_BIT: u32 = 5;

#[cfg(all(feature = "clock-180mhz", feature = "clock-200mhz"))]
compile_error!("select only one PIO clock feature");

/// Smallest divider shipped: 2 preserves the qualified input-sampling margin.
pub const MIN_DIVIDER: u16 = 2;

/// Largest integer PIO divider; the register encodes 65,536 as zero.
pub const MAX_DIVIDER: u32 = 65_536;

/// Encoded integer divider no faster than requested; `None` means invalid or unrepresentable.
pub const fn clock_divider(
    sys_hz: u32,
    f_hz: u32,
    cycles_per_bit: u32,
    min_divider: u16,
) -> Option<u16> {
    if sys_hz == 0 || f_hz == 0 || cycles_per_bit == 0 || min_divider == 0 {
        return None;
    }
    let per_bit = div_ceil(sys_hz, f_hz);
    let div = div_ceil(per_bit, cycles_per_bit);
    let min = min_divider as u32;
    let selected = if div < min {
        min
    } else if div > MAX_DIVIDER {
        return None;
    } else {
        div
    };
    Some(if selected == MAX_DIVIDER {
        0
    } else {
        selected as u16
    })
}

/// `ceil(a / b)`; `u32::div_ceil` is not stable-const here.
const fn div_ceil(a: u32, b: u32) -> u32 {
    a / b + (!a.is_multiple_of(b)) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    const RP2040_SYS: u32 = 125_000_000;
    const RP2350_SYS: u32 = 150_000_000;

    fn decoded(divider: u16) -> u64 {
        if divider == 0 {
            MAX_DIVIDER as u64
        } else {
            divider as u64
        }
    }

    fn assert_not_faster(sys_hz: u32, requested_hz: u32, divider: u16) {
        assert!(
            sys_hz as u64 <= requested_hz as u64 * CYCLES_PER_BIT as u64 * decoded(divider),
            "sys {sys_hz}: divider {divider} exceeds request {requested_hz}"
        );
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        assert_eq!(
            clock_divider(RP2040_SYS, 0, CYCLES_PER_BIT, MIN_DIVIDER),
            None
        );
        assert_eq!(clock_divider(0, 1, CYCLES_PER_BIT, MIN_DIVIDER), None);
        assert_eq!(clock_divider(RP2040_SYS, 1, 0, MIN_DIVIDER), None);
        assert_eq!(clock_divider(RP2040_SYS, 1, CYCLES_PER_BIT, 0), None);
    }

    #[cfg(not(any(feature = "clock-180mhz", feature = "clock-200mhz")))]
    #[test]
    fn divider_table_125mhz() {
        let cases = [
            (1_000_000u32, 32u16),
            (2_000_000, 16),
            (5_000_000, 7),
            (10_000_000, 4),
            (15_625_000, 2),
            (31_250_000, 2),
            (100_000_000, 2),
        ];
        for (f, expect) in cases {
            assert_eq!(
                clock_divider(RP2040_SYS, f, CYCLES_PER_BIT, MIN_DIVIDER),
                Some(expect),
                "SWCLK {f}"
            );
        }
    }

    #[cfg(feature = "clock-180mhz")]
    #[test]
    fn divider_table_180mhz() {
        for (f, expect) in [
            (1_000_000u32, 36u16),
            (2_000_000, 18),
            (5_000_000, 8),
            (10_000_000, 4),
            (17_999_999, 3),
            (18_000_000, 2),
            (25_000_000, 2),
            (100_000_000, 2),
        ] {
            assert_eq!(
                clock_divider(180_000_000, f, CYCLES_PER_BIT, MIN_DIVIDER),
                Some(expect),
                "SWCLK {f}"
            );
        }
    }

    #[cfg(feature = "clock-200mhz")]
    #[test]
    fn divider_table_200mhz() {
        for (f, expect) in [
            (1_000_000u32, 40u16),
            (2_000_000, 20),
            (5_000_000, 8),
            (10_000_000, 4),
            (19_999_999, 3),
            (20_000_000, 2),
            (25_000_000, 2),
            (100_000_000, 2),
        ] {
            assert_eq!(
                clock_divider(200_000_000, f, CYCLES_PER_BIT, MIN_DIVIDER),
                Some(expect),
                "SWCLK {f}"
            );
        }
    }

    #[test]
    fn realized_frequency_never_exceeds_request() {
        for f in [900_000u32, 3_333_333, 7_000_000, 12_000_000, 20_000_000] {
            let div = clock_divider(RP2040_SYS, f, CYCLES_PER_BIT, 1).unwrap();
            assert_not_faster(RP2040_SYS, f, div);
        }
    }

    #[test]
    fn stock_low_frequency_ranges_are_exhaustive_and_truthful() {
        for sys in [RP2040_SYS, RP2350_SYS] {
            for f in 1..=2_000 {
                if let Some(div) = clock_divider(sys, f, CYCLES_PER_BIT, MIN_DIVIDER) {
                    assert_not_faster(sys, f, div);
                } else {
                    assert!(
                        sys as u64 > f as u64 * CYCLES_PER_BIT as u64 * MAX_DIVIDER as u64,
                        "sys {sys}: representable request {f} was rejected"
                    );
                }
            }
        }
    }

    #[test]
    fn low_frequency_boundaries() {
        for sys in [RP2040_SYS, RP2350_SYS, 200_000_000] {
            let capacity = CYCLES_PER_BIT as u64 * MAX_DIVIDER as u64;
            let first = (sys as u64).div_ceil(capacity) as u32;
            assert_eq!(
                clock_divider(sys, first - 1, CYCLES_PER_BIT, MIN_DIVIDER),
                None
            );
            let encoded = clock_divider(sys, first, CYCLES_PER_BIT, MIN_DIVIDER).unwrap();
            assert_not_faster(sys, first, encoded);
        }
    }

    #[test]
    fn maximum_divider_uses_the_zero_encoding() {
        let exact = CYCLES_PER_BIT * MAX_DIVIDER;
        assert_eq!(clock_divider(exact, 1, CYCLES_PER_BIT, 1), Some(0));
        assert_eq!(clock_divider(exact + 1, 1, CYCLES_PER_BIT, 1), None);
    }

    #[test]
    fn ceiling_math_handles_u32_max_without_overflow() {
        assert_eq!(clock_divider(u32::MAX, 1, 1, 1), None);
        assert_eq!(clock_divider(u32::MAX, 65_536, 1, 1), Some(0));
        assert_eq!(clock_divider(u32::MAX, 65_535, 1, 1), None);
    }

    #[test]
    fn min_divider_one_allows_top_bin() {
        assert_eq!(
            clock_divider(RP2040_SYS, 31_250_000, CYCLES_PER_BIT, 1),
            Some(1)
        );
    }

    #[test]
    fn minimum_divider_boundary_rounds_slower() {
        for sys in [RP2040_SYS, RP2350_SYS, 200_000_000] {
            let first = div_ceil(sys, CYCLES_PER_BIT * u32::from(MIN_DIVIDER));
            assert_eq!(
                clock_divider(sys, first - 1, CYCLES_PER_BIT, MIN_DIVIDER),
                Some(MIN_DIVIDER + 1)
            );
            assert_eq!(
                clock_divider(sys, first, CYCLES_PER_BIT, MIN_DIVIDER),
                Some(MIN_DIVIDER)
            );
        }
    }
}
