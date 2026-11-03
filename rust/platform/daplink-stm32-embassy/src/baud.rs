//! Non-mutating baud validation for the H5 USART used by VCOM.

pub(crate) const PRESCALERS: [u32; 12] = [1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256];

/// Match embassy-stm32's USART prescaler and rounded BRR acceptance, including OVER8.
pub(crate) fn supported(kernel_clock: u32, baud: u32) -> bool {
    if baud == 0 {
        return false;
    }
    PRESCALERS.iter().any(|prescaler| {
        let clock = u64::from(kernel_clock / prescaler);
        let divisor = (clock + u64::from(baud / 2)) / u64::from(baud);
        (8..65536).contains(&divisor)
    })
}

#[cfg(test)]
mod tests {
    use super::supported;

    #[test]
    fn rejects_unrepresentable_requests() {
        for baud in [0, 1, 14, 33_333_334, u32::MAX] {
            assert!(!supported(250_000_000, baud), "{baud}");
        }
        assert!(!supported(0, 115_200));
    }

    #[test]
    fn accepts_prescaled_and_oversampled_boundaries() {
        for baud in [
            15, 9600, 115_200, 921_600, 3_000_000, 31_250_000, 33_333_333,
        ] {
            assert!(supported(250_000_000, baud), "{baud}");
        }
    }

    #[test]
    fn uses_runtime_clock_and_hal_rounding() {
        assert!(supported(16_000_000, 2_133_333));
        assert!(!supported(16_000_000, 2_133_334));
        assert!(supported(16_000_000, 1));
        assert!(!supported(250_000_000, 1));
        assert!(supported(u32::MAX, u32::MAX / 8));
        assert!(!supported(u32::MAX, u32::MAX));
    }
}
