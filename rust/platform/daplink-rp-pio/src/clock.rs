//! Clock-divider math for `DAP_SWJ_Clock` / `Swd::set_clock`.

const CLOCK_125MHZ: u32 = 125_000_000;
const CLOCK_150MHZ: u32 = 150_000_000;
const CLOCK_180MHZ: u32 = 180_000_000;
const CLOCK_200MHZ: u32 = 200_000_000;

/// Return the qualified standard-program cell width for a system clock.
pub const fn swd_cycles_per_bit(sys_hz: u32) -> Option<u32> {
    match sys_hz {
        CLOCK_125MHZ | CLOCK_150MHZ => Some(4),
        CLOCK_180MHZ | CLOCK_200MHZ => Some(5),
        _ => None,
    }
}

/// A system clock every PIO program and divider is qualified at.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PioClock(u32);

impl PioClock {
    /// The qualified clock at `sys_hz`, or `None`.
    pub const fn qualified(sys_hz: u32) -> Option<Self> {
        match (swd_cycles_per_bit(sys_hz), jtag_timing(sys_hz)) {
            (Some(_), Some(_)) => Some(Self(sys_hz)),
            _ => None,
        }
    }

    /// The qualified clock at `sys_hz`; in a const, an unqualified clock fails the build.
    pub const fn expect(sys_hz: u32) -> Self {
        match Self::qualified(sys_hz) {
            Some(clock) => clock,
            None => panic!("PIO timing is not qualified at this system clock"),
        }
    }

    /// The system clock in hertz.
    pub const fn sys_hz(self) -> u32 {
        self.0
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct JtagTiming {
    low_cycles: u8,
    high_cycles: u8,
}

impl JtagTiming {
    const fn new(low_cycles: u8, high_cycles: u8) -> Self {
        assert!(low_cycles >= 3 && low_cycles <= 4);
        assert!(high_cycles >= 2 && high_cycles <= 4);
        Self {
            low_cycles,
            high_cycles,
        }
    }

    pub(crate) const fn cycles_per_bit(self) -> u32 {
        self.low_cycles as u32 + self.high_cycles as u32
    }

    #[cfg(test)]
    pub(crate) const fn low_cycles(self) -> u32 {
        self.low_cycles as u32
    }

    #[cfg(test)]
    pub(crate) const fn high_cycles(self) -> u32 {
        self.high_cycles as u32
    }

    pub(crate) const fn output_delay(self) -> u8 {
        1
    }

    pub(crate) const fn sample_delay(self) -> u8 {
        0
    }

    pub(crate) const fn loop_low_delay(self) -> u8 {
        self.low_cycles - 3
    }

    pub(crate) const fn loop_high_delay(self) -> u8 {
        self.high_cycles - 2
    }

    #[cfg(any(test, feature = "rp2040", feature = "rp235x"))]
    pub(crate) const fn full_low_delay(self) -> u8 {
        self.low_cycles - 1
    }

    #[cfg(any(test, feature = "rp2040", feature = "rp235x"))]
    pub(crate) const fn full_high_delay(self) -> u8 {
        self.high_cycles - 1
    }
}

pub(crate) const JTAG_TIMING_125MHZ: JtagTiming = JtagTiming::new(3, 2);
pub(crate) const JTAG_TIMING_150MHZ: JtagTiming = JtagTiming::new(3, 3);
pub(crate) const JTAG_TIMING_200MHZ: JtagTiming = JtagTiming::new(4, 4);

pub(crate) const fn jtag_timing(sys_hz: u32) -> Option<JtagTiming> {
    match sys_hz {
        CLOCK_125MHZ => Some(JTAG_TIMING_125MHZ),
        CLOCK_150MHZ => Some(JTAG_TIMING_150MHZ),
        CLOCK_180MHZ | CLOCK_200MHZ => Some(JTAG_TIMING_200MHZ),
        _ => None,
    }
}

/// Return the qualified late-sampled JTAG cell width for a system clock.
pub const fn jtag_cycles_per_bit(sys_hz: u32) -> Option<u32> {
    match jtag_timing(sys_hz) {
        Some(timing) => Some(timing.cycles_per_bit()),
        None => None,
    }
}

/// Generic SWD's integer-divider floor preserves its input-sampling margin.
pub const MIN_DIVIDER: u16 = 2;

/// SM cycles per SWCLK bit in the synchronizer-bypassed complete-transfer program.
pub const TRANSACTION_CYCLES_PER_BIT: u32 = 4;
/// Complete-transfer SWD divider floor outside the 125/150 MHz profiles.
pub const TRANSACTION_MIN_DIVIDER: u16 = 2;

const FRACTIONAL_SCALE: u64 = 256;
const TRANSACTION_125MHZ_MINIMUM: u64 = FRACTIONAL_SCALE + 64;
const TRANSACTION_150MHZ_MINIMUM: u64 = FRACTIONAL_SCALE + 128;
const TRANSACTION_QUALIFIED_MINIMUM: u64 = 2 * FRACTIONAL_SCALE;

/// JTAG uses divider one with a late-sampled cell.
pub const JTAG_MIN_DIVIDER: u16 = 1;

/// Largest integer PIO divider; the register encodes 65,536 as zero.
pub const MAX_DIVIDER: u32 = 65_536;

/// The slowest divider, 65,536; a request below it runs here and is reported as an error.
pub const SLOWEST_DIVIDER: PioClockDivider = PioClockDivider::integer(0);

/// PIO's 16.8 clock-divider register representation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PioClockDivider {
    /// Integer field; zero encodes 65,536.
    pub integer: u16,
    /// Fractional field in units of 1/256.
    pub fractional: u8,
}

impl PioClockDivider {
    /// Build an integer divider.
    pub const fn integer(value: u16) -> Self {
        Self {
            integer: value,
            fractional: 0,
        }
    }
}

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

/// Select a 16.8 JTAG divider no period of which is faster than requested; unsupported profiles return `None`.
pub const fn jtag_clock_divider(sys_hz: u32, f_hz: u32) -> Option<PioClockDivider> {
    let cycles_per_bit = match jtag_cycles_per_bit(sys_hz) {
        Some(value) => value,
        None => return None,
    };
    clock_divider_fractional(sys_hz, f_hz, cycles_per_bit, JTAG_MIN_DIVIDER)
}

/// Select a 16.8 SWD divider no period of which is faster than requested.
pub const fn swd_transaction_clock_divider(sys_hz: u32, f_hz: u32) -> Option<PioClockDivider> {
    let minimum = match sys_hz {
        CLOCK_125MHZ => TRANSACTION_125MHZ_MINIMUM,
        CLOCK_150MHZ => TRANSACTION_150MHZ_MINIMUM,
        // Preserve the qualified overclock profiles and treat unknown clocks conservatively.
        CLOCK_180MHZ | CLOCK_200MHZ => TRANSACTION_QUALIFIED_MINIMUM,
        _ => TRANSACTION_QUALIFIED_MINIMUM,
    };
    clock_divider_fractional_minimum(sys_hz, f_hz, TRANSACTION_CYCLES_PER_BIT, minimum)
}

const fn clock_divider_fractional(
    sys_hz: u32,
    f_hz: u32,
    cycles_per_bit: u32,
    min_divider: u16,
) -> Option<PioClockDivider> {
    clock_divider_fractional_minimum(
        sys_hz,
        f_hz,
        cycles_per_bit,
        min_divider as u64 * FRACTIONAL_SCALE,
    )
}

const fn clock_divider_fractional_minimum(
    sys_hz: u32,
    f_hz: u32,
    cycles_per_bit: u32,
    minimum: u64,
) -> Option<PioClockDivider> {
    let maximum = MAX_DIVIDER as u64 * FRACTIONAL_SCALE;
    if sys_hz == 0
        || f_hz == 0
        || cycles_per_bit == 0
        || minimum < FRACTIONAL_SCALE
        || minimum > maximum
    {
        return None;
    }
    // The fractional divider bounds only the mean; whole clocks per bit first bound every period.
    let clocks_per_bit = div_ceil(sys_hz, f_hz) as u64;
    let required = (clocks_per_bit * FRACTIONAL_SCALE).div_ceil(cycles_per_bit as u64);
    let selected = if required < minimum {
        minimum
    } else if required > maximum {
        return None;
    } else {
        required
    };
    if selected == maximum {
        Some(PioClockDivider::integer(0))
    } else {
        Some(PioClockDivider {
            integer: (selected >> 8) as u16,
            fractional: selected as u8,
        })
    }
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

    /// System clocks in the shortest run of `cycles` SM cycles under `divider`.
    fn shortest_period(divider: PioClockDivider, cycles: u32) -> u64 {
        let encoded = if divider.integer == 0 {
            MAX_DIVIDER as u64 * 256
        } else {
            u64::from(divider.integer) * 256 + u64::from(divider.fractional)
        };
        u64::from(cycles) * encoded / 256
    }

    /// Each whole-clock class's first request and the fastest one of the class after it.
    fn class_edges(sys_hz: u32) -> impl Iterator<Item = u32> {
        let (low, high) = (1_000, 25_000_000);
        (div_ceil(sys_hz, high)..=div_ceil(sys_hz, low))
            .flat_map(move |n| {
                let first = div_ceil(sys_hz, n);
                [first - 1, first]
            })
            .chain((1..=25_000).map(|k| k * 1_000))
            .filter(move |f| (low..=high).contains(f))
    }

    fn assert_no_short_period(
        sys_hz: u32,
        requested_hz: u32,
        cycles: u32,
        divider: PioClockDivider,
    ) {
        let shortest = shortest_period(divider, cycles);
        assert!(
            shortest * u64::from(requested_hz) * 100 >= u64::from(sys_hz) * 99,
            "sys {sys_hz}: {requested_hz} Hz under {divider:?} has a {shortest}-clock period"
        );
    }

    #[test]
    fn no_single_period_is_faster_than_requested() {
        for sys_hz in [CLOCK_125MHZ, CLOCK_150MHZ, CLOCK_180MHZ, CLOCK_200MHZ] {
            let swd = swd_cycles_per_bit(sys_hz).unwrap();
            let jtag = jtag_cycles_per_bit(sys_hz).unwrap();
            for f in class_edges(sys_hz) {
                let generic = clock_divider(sys_hz, f, swd, MIN_DIVIDER).unwrap();
                assert_no_short_period(sys_hz, f, swd, PioClockDivider::integer(generic));
                let transaction = swd_transaction_clock_divider(sys_hz, f).unwrap();
                assert_no_short_period(sys_hz, f, TRANSACTION_CYCLES_PER_BIT, transaction);
                assert_no_short_period(sys_hz, f, jtag, jtag_clock_divider(sys_hz, f).unwrap());
            }
        }
    }

    fn decoded(divider: u16) -> u64 {
        if divider == 0 {
            MAX_DIVIDER as u64
        } else {
            divider as u64
        }
    }

    fn assert_not_faster(sys_hz: u32, requested_hz: u32, divider: u16) {
        let cycles_per_bit = swd_cycles_per_bit(sys_hz).unwrap();
        assert!(
            sys_hz as u64 <= requested_hz as u64 * cycles_per_bit as u64 * decoded(divider),
            "sys {sys_hz}: divider {divider} exceeds request {requested_hz}"
        );
    }

    fn assert_fractional_not_faster(
        sys_hz: u32,
        requested_hz: u32,
        cycles_per_bit: u32,
        divider: PioClockDivider,
    ) {
        let encoded = if divider.integer == 0 {
            MAX_DIVIDER as u64 * 256
        } else {
            u64::from(divider.integer) * 256 + u64::from(divider.fractional)
        };
        assert!(
            u64::from(sys_hz) * 256
                <= u64::from(requested_hz) * u64::from(cycles_per_bit) * encoded,
            "sys {sys_hz}: divider {divider:?} exceeds request {requested_hz}"
        );
    }

    #[test]
    fn only_the_four_profiles_are_qualified() {
        for sys_hz in [CLOCK_125MHZ, CLOCK_150MHZ, CLOCK_180MHZ, CLOCK_200MHZ] {
            assert_eq!(
                PioClock::qualified(sys_hz).map(PioClock::sys_hz),
                Some(sys_hz)
            );
        }
        for sys_hz in [0, 48_000_000, 133_000_000, 160_000_000, 250_000_000] {
            assert_eq!(PioClock::qualified(sys_hz), None);
        }
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        let cycles = swd_cycles_per_bit(RP2040_SYS).unwrap();
        assert_eq!(clock_divider(RP2040_SYS, 0, cycles, MIN_DIVIDER), None);
        assert_eq!(clock_divider(0, 1, cycles, MIN_DIVIDER), None);
        assert_eq!(clock_divider(RP2040_SYS, 1, 0, MIN_DIVIDER), None);
        assert_eq!(clock_divider(RP2040_SYS, 1, cycles, 0), None);
        assert_eq!(swd_transaction_clock_divider(RP2040_SYS, 0), None);
        assert_eq!(swd_transaction_clock_divider(0, 1_000_000), None);
        assert_eq!(jtag_clock_divider(RP2040_SYS, 0), None);
        assert_eq!(jtag_clock_divider(0, 1_000_000), None);
    }

    #[test]
    fn transaction_stock_divider_tables_reach_25mhz_truthfully() {
        for (sys_hz, requested, expected) in [
            (
                RP2040_SYS,
                1_000_000,
                PioClockDivider {
                    integer: 31,
                    fractional: 64,
                },
            ),
            (
                RP2040_SYS,
                8_000_000,
                PioClockDivider {
                    integer: 4,
                    fractional: 0,
                },
            ),
            (
                RP2040_SYS,
                20_000_000,
                PioClockDivider {
                    integer: 1,
                    fractional: 192,
                },
            ),
            (
                RP2040_SYS,
                24_000_000,
                PioClockDivider {
                    integer: 1,
                    fractional: 128,
                },
            ),
            (
                RP2040_SYS,
                25_000_000,
                PioClockDivider {
                    integer: 1,
                    fractional: 64,
                },
            ),
            (
                RP2350_SYS,
                1_000_000,
                PioClockDivider {
                    integer: 37,
                    fractional: 128,
                },
            ),
            (
                RP2350_SYS,
                8_000_000,
                PioClockDivider {
                    integer: 4,
                    fractional: 192,
                },
            ),
            (
                RP2350_SYS,
                20_000_000,
                PioClockDivider {
                    integer: 2,
                    fractional: 0,
                },
            ),
            (
                RP2350_SYS,
                24_000_000,
                PioClockDivider {
                    integer: 1,
                    fractional: 192,
                },
            ),
            (
                RP2350_SYS,
                25_000_000,
                PioClockDivider {
                    integer: 1,
                    fractional: 128,
                },
            ),
        ] {
            let divider = swd_transaction_clock_divider(sys_hz, requested).unwrap();
            assert_eq!(divider, expected, "sys {sys_hz}: SWCLK {requested}");
            assert_fractional_not_faster(sys_hz, requested, TRANSACTION_CYCLES_PER_BIT, divider);
        }
    }

    #[test]
    fn transaction_stock_profiles_clamp_above_25mhz() {
        for (sys_hz, expected) in [
            (
                RP2040_SYS,
                PioClockDivider {
                    integer: 1,
                    fractional: 64,
                },
            ),
            (
                RP2350_SYS,
                PioClockDivider {
                    integer: 1,
                    fractional: 128,
                },
            ),
        ] {
            assert_eq!(
                swd_transaction_clock_divider(sys_hz, 25_000_000),
                Some(expected)
            );
            assert_eq!(
                swd_transaction_clock_divider(sys_hz, u32::MAX),
                Some(expected)
            );
        }
    }

    #[test]
    fn transaction_overclock_profiles_retain_the_qualified_floor() {
        for sys_hz in [CLOCK_180MHZ, CLOCK_200MHZ] {
            for requested in 1..=25_000 {
                let requested = requested * 1_000;
                let divider = swd_transaction_clock_divider(sys_hz, requested).unwrap();
                assert!(
                    divider.integer >= TRANSACTION_MIN_DIVIDER,
                    "sys {sys_hz}: SWCLK {requested} selects {divider:?}"
                );
            }
        }
        assert_eq!(
            swd_transaction_clock_divider(CLOCK_180MHZ, 25_000_000),
            Some(PioClockDivider::integer(2))
        );
        assert_eq!(
            swd_transaction_clock_divider(CLOCK_200MHZ, 25_000_000),
            Some(PioClockDivider::integer(2))
        );
    }

    #[test]
    fn transaction_divider_is_exhaustively_truthful_to_25mhz() {
        for sys_hz in [RP2040_SYS, RP2350_SYS, CLOCK_180MHZ, CLOCK_200MHZ] {
            for requested in 1..=25_000 {
                let requested = requested * 1_000;
                let divider = swd_transaction_clock_divider(sys_hz, requested).unwrap();
                assert_fractional_not_faster(
                    sys_hz,
                    requested,
                    TRANSACTION_CYCLES_PER_BIT,
                    divider,
                );
            }
        }
    }

    #[test]
    fn transaction_unknown_clock_retains_divider_two() {
        assert_eq!(
            swd_transaction_clock_divider(160_000_000, u32::MAX),
            Some(PioClockDivider::integer(2))
        );
    }

    #[test]
    fn jtag_divider_tables_match_every_timing_profile() {
        let d = |integer, fractional| PioClockDivider {
            integer,
            fractional,
        };
        for (sys_hz, timing, table) in [
            (
                125_000_000,
                JTAG_TIMING_125MHZ,
                [d(25, 0), d(1, 103), d(1, 103), d(1, 0)],
            ),
            (
                150_000_000,
                JTAG_TIMING_150MHZ,
                [d(25, 0), d(1, 86), d(1, 86), d(1, 0)],
            ),
            (
                200_000_000,
                JTAG_TIMING_200MHZ,
                [d(25, 0), d(1, 96), d(1, 64), d(1, 0)],
            ),
        ] {
            for (requested, expected) in [1_000_000, 19_000_000, 20_000_000, 25_000_000]
                .into_iter()
                .zip(table)
            {
                let divider = clock_divider_fractional(
                    sys_hz,
                    requested,
                    timing.cycles_per_bit(),
                    JTAG_MIN_DIVIDER,
                )
                .unwrap();
                assert_eq!(divider, expected, "sys {sys_hz}: TCK {requested}");
                assert_fractional_not_faster(sys_hz, requested, timing.cycles_per_bit(), divider);
            }
        }

        for (requested, expected) in [
            (
                1_000_000,
                PioClockDivider {
                    integer: 22,
                    fractional: 128,
                },
            ),
            (
                19_000_000,
                PioClockDivider {
                    integer: 1,
                    fractional: 64,
                },
            ),
            (
                20_000_000,
                PioClockDivider {
                    integer: 1,
                    fractional: 32,
                },
            ),
            (22_500_000, PioClockDivider::integer(1)),
            (25_000_000, PioClockDivider::integer(1)),
        ] {
            let divider = clock_divider_fractional(
                180_000_000,
                requested,
                JTAG_TIMING_200MHZ.cycles_per_bit(),
                JTAG_MIN_DIVIDER,
            )
            .unwrap();
            assert_eq!(divider, expected, "180 MHz: TCK {requested}");
            assert_fractional_not_faster(
                180_000_000,
                requested,
                JTAG_TIMING_200MHZ.cycles_per_bit(),
                divider,
            );
        }
    }

    #[test]
    fn jtag_divider_never_exceeds_requests_across_supported_clocks() {
        for (sys_hz, timing) in [
            (125_000_000, JTAG_TIMING_125MHZ),
            (150_000_000, JTAG_TIMING_150MHZ),
            (180_000_000, JTAG_TIMING_200MHZ),
            (200_000_000, JTAG_TIMING_200MHZ),
        ] {
            for requested in 1..=25_000 {
                let requested = requested * 1_000;
                let divider = clock_divider_fractional(
                    sys_hz,
                    requested,
                    timing.cycles_per_bit(),
                    JTAG_MIN_DIVIDER,
                )
                .unwrap();
                assert_fractional_not_faster(sys_hz, requested, timing.cycles_per_bit(), divider);
            }
        }
    }

    #[test]
    fn jtag_clock_divider_uses_the_selected_profile() {
        for (sys_hz, expected) in [
            (CLOCK_125MHZ, JTAG_TIMING_125MHZ),
            (CLOCK_150MHZ, JTAG_TIMING_150MHZ),
            (CLOCK_180MHZ, JTAG_TIMING_200MHZ),
            (CLOCK_200MHZ, JTAG_TIMING_200MHZ),
        ] {
            assert_eq!(jtag_timing(sys_hz), Some(expected));
            assert_eq!(jtag_cycles_per_bit(sys_hz), Some(expected.cycles_per_bit()));
            for requested in [1_000_000, 19_000_000, 20_000_000, 25_000_000] {
                assert_eq!(
                    jtag_clock_divider(sys_hz, requested),
                    clock_divider_fractional(
                        sys_hz,
                        requested,
                        expected.cycles_per_bit(),
                        JTAG_MIN_DIVIDER,
                    )
                );
            }
        }
        assert_eq!(jtag_timing(160_000_000), None);
        assert_eq!(jtag_clock_divider(160_000_000, 1_000_000), None);
    }

    #[test]
    fn standard_divider_tables_follow_system_clock() {
        for (sys_hz, cases) in [
            (
                CLOCK_125MHZ,
                &[(1_000_000, 32), (10_000_000, 4), (15_625_000, 2)][..],
            ),
            (
                CLOCK_180MHZ,
                &[(1_000_000, 36), (10_000_000, 4), (18_000_000, 2)][..],
            ),
            (
                CLOCK_200MHZ,
                &[(1_000_000, 40), (10_000_000, 4), (20_000_000, 2)][..],
            ),
        ] {
            let cycles = swd_cycles_per_bit(sys_hz).unwrap();
            for &(f, expected) in cases {
                assert_eq!(
                    clock_divider(sys_hz, f, cycles, MIN_DIVIDER),
                    Some(expected)
                );
            }
        }
    }

    #[test]
    fn realized_frequency_never_exceeds_request() {
        let cycles = swd_cycles_per_bit(RP2040_SYS).unwrap();
        for f in [900_000u32, 3_333_333, 7_000_000, 12_000_000, 20_000_000] {
            let div = clock_divider(RP2040_SYS, f, cycles, 1).unwrap();
            assert_not_faster(RP2040_SYS, f, div);
        }
    }

    #[test]
    fn stock_low_frequency_ranges_are_exhaustive_and_truthful() {
        for sys in [RP2040_SYS, RP2350_SYS] {
            let cycles = swd_cycles_per_bit(sys).unwrap();
            for f in 1..=2_000 {
                if let Some(div) = clock_divider(sys, f, cycles, MIN_DIVIDER) {
                    assert_not_faster(sys, f, div);
                } else {
                    assert!(
                        sys as u64 > f as u64 * cycles as u64 * MAX_DIVIDER as u64,
                        "sys {sys}: representable request {f} was rejected"
                    );
                }
            }
        }
    }

    #[test]
    fn low_frequency_boundaries() {
        for sys in [RP2040_SYS, RP2350_SYS, 200_000_000] {
            let cycles = swd_cycles_per_bit(sys).unwrap();
            let capacity = cycles as u64 * MAX_DIVIDER as u64;
            let first = (sys as u64).div_ceil(capacity) as u32;
            assert_eq!(clock_divider(sys, first - 1, cycles, MIN_DIVIDER), None);
            let encoded = clock_divider(sys, first, cycles, MIN_DIVIDER).unwrap();
            assert_not_faster(sys, first, encoded);
        }
    }

    #[test]
    fn maximum_divider_uses_the_zero_encoding() {
        let cycles = 4;
        let exact = cycles * MAX_DIVIDER;
        assert_eq!(clock_divider(exact, 1, cycles, 1), Some(0));
        assert_eq!(clock_divider(exact + 1, 1, cycles, 1), None);
    }

    #[test]
    fn ceiling_math_handles_u32_max_without_overflow() {
        assert_eq!(clock_divider(u32::MAX, 1, 1, 1), None);
        assert_eq!(clock_divider(u32::MAX, 65_536, 1, 1), Some(0));
        assert_eq!(clock_divider(u32::MAX, 65_535, 1, 1), None);
    }

    #[test]
    fn min_divider_one_allows_top_bin() {
        let cycles = swd_cycles_per_bit(RP2040_SYS).unwrap();
        assert_eq!(clock_divider(RP2040_SYS, 31_250_000, cycles, 1), Some(1));
    }

    #[test]
    fn minimum_divider_boundary_rounds_slower() {
        for sys in [RP2040_SYS, RP2350_SYS, 200_000_000] {
            let cycles = swd_cycles_per_bit(sys).unwrap();
            let first = div_ceil(sys, cycles * u32::from(MIN_DIVIDER));
            assert_eq!(
                clock_divider(sys, first - 1, cycles, MIN_DIVIDER),
                Some(MIN_DIVIDER + 1)
            );
            assert_eq!(
                clock_divider(sys, first, cycles, MIN_DIVIDER),
                Some(MIN_DIVIDER)
            );
        }
    }
}
