//! Opt-in GPIO pacing for the 72 MHz flash-resident F1 port.

use daplink_rs::PreparedSpin;

/// Prepare the flash-resident GPIO half-period for a `cycles` half-period request.
pub fn prepare_flash_gpio_half(cycles: u32) -> PreparedSpin {
    // Preserve the saturated path's one-iteration control and sampling margin.
    let scaled = if cycles <= 11 { 0 } else { cycles.div_ceil(2) };
    PreparedSpin::new(scaled)
}

#[cfg(test)]
mod tests {
    use super::prepare_flash_gpio_half;
    use daplink_rs::PreparedSpin;
    use daplink_spi_swd::session::half_period_cycles;

    #[test]
    fn saturated_half_periods_keep_one_iteration() {
        for cycles in 0..=11 {
            assert_eq!(prepare_flash_gpio_half(cycles), PreparedSpin::new(3));
        }
        assert_eq!(prepare_flash_gpio_half(12), PreparedSpin::new(6));
    }

    #[test]
    fn counted_halves_round_up_with_full_period_margin() {
        for iterations in 2..=10_000u32 {
            let shortest_period = 12 * iterations + 10;
            for cycles in (6 * iterations - 5).max(12)..=6 * iterations {
                assert_eq!(
                    prepare_flash_gpio_half(cycles),
                    PreparedSpin::new(3 * iterations)
                );
                assert!(shortest_period >= 2 * cycles + 10);
                assert!(6 * iterations - 5 >= 7);
            }
        }
    }

    #[test]
    fn clock_boundaries_are_monotonic_and_do_not_overflow() {
        for (hz, iterations) in [
            (250_000, 24),
            (500_000, 12),
            (1_000_000, 6),
            (2_000_000, 3),
            (3_000_000, 2),
            (2_999_999, 3),
            (3_272_727, 2),
            (3_272_728, 1),
            (4_000_000, 1),
            (25_000_000, 1),
            (u32::MAX, 1),
        ] {
            assert_eq!(
                prepare_flash_gpio_half(half_period_cycles(72_000_000, hz)),
                PreparedSpin::new(3 * iterations),
                "requested clock {hz}"
            );
        }
        for cycles in [u32::MAX - 2, u32::MAX - 1, u32::MAX] {
            let iterations = u64::from(cycles).div_ceil(6);
            assert_eq!(
                prepare_flash_gpio_half(cycles),
                PreparedSpin::new((3 * iterations) as u32)
            );
        }
    }
}
