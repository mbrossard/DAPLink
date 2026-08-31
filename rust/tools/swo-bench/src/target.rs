//! Per-target profiles: chip name, trace clock, and the flash window the SWD load reads.

use anyhow::{anyhow, Result};

/// A named emitting target.
#[derive(Debug)]
pub struct TargetProfile {
    /// Profile name, matched against `--target`.
    pub name: &'static str,
    /// probe-rs chip name for the DUT setup attach.
    pub chip: &'static str,
    /// Clock feeding the target's TPIU async prescaler, in Hz.
    pub trace_clock_hz: u32,
    /// Debug-domain register routing the trace pin, written before each tier.
    pub trace_route_reg: u64,
    /// Flash window base the SWD load reads from.
    pub flash_base: u32,
    /// Flash window size in bytes.
    pub flash_window_bytes: u32,
    /// ITM stimulus port the DUT streams on.
    pub stimulus_port: u8,
}

/// Known profiles. Only silicon with an ITM and a wired TRACESWO pin can appear here.
pub const PROFILES: &[TargetProfile] = &[TargetProfile {
    name: "stm32f401",
    chip: "STM32F401CEUx",
    trace_clock_hz: 84_000_000,
    // DBGMCU_CR: bit 5 TRACE_IOEN, bits 7:6 TRACE_MODE (00 = asynchronous SWO).
    trace_route_reg: 0xE004_2004,
    flash_base: 0x0800_0000,
    flash_window_bytes: 128 * 1024,
    stimulus_port: 0,
}];

/// Default `--target` when the flag is absent.
pub const DEFAULT_TARGET: &str = "stm32f401";

/// Look up a profile by name, erroring with the known-name list on a miss.
pub fn lookup(name: &str) -> Result<&'static TargetProfile> {
    PROFILES.iter().find(|p| p.name == name).ok_or_else(|| {
        let known: Vec<&str> = PROFILES.iter().map(|p| p.name).collect();
        anyhow!("unknown target '{name}' (known: {})", known.join(", "))
    })
}

impl TargetProfile {
    /// The baud the target's integer TPIU prescaler actually realizes for `requested`, or `None`
    /// when no prescaler divisor exists.
    pub fn realized_baud(&self, requested: u32) -> Option<u32> {
        if requested == 0 {
            return None;
        }
        let divisor = self.trace_clock_hz / requested;
        (divisor > 0).then(|| self.trace_clock_hz / divisor)
    }

    /// True when `[flash_base, flash_base+len)` fits inside the flash window.
    pub fn flash_contains(&self, len_bytes: usize) -> bool {
        u64::try_from(len_bytes).is_ok_and(|n| n <= u64::from(self.flash_window_bytes))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup_finds_the_known_profile_and_names_it_on_a_miss() {
        assert_eq!(lookup("stm32f401").unwrap().chip, "STM32F401CEUx");
        let err = lookup("rp2040").unwrap_err().to_string();
        assert!(err.contains("rp2040"));
        assert!(err.contains("stm32f401"));
    }

    #[test]
    fn realized_baud_follows_the_integer_prescaler() {
        let p = lookup("stm32f401").unwrap();
        // 84 MHz divides these exactly.
        assert_eq!(p.realized_baud(1_000_000), Some(1_000_000));
        assert_eq!(p.realized_baud(2_000_000), Some(2_000_000));
        // 115200 does not: 84e6/115200 = 729 -> 115226 baud.
        assert_eq!(p.realized_baud(115_200), Some(84_000_000 / 729));
        // Above the trace clock there is no divisor at all.
        assert_eq!(p.realized_baud(100_000_000), None);
        assert_eq!(p.realized_baud(0), None);
    }

    #[test]
    fn the_load_chunk_must_fit_the_flash_window() {
        let p = lookup("stm32f401").unwrap();
        assert!(p.flash_contains(4096));
        assert!(!p.flash_contains(1 << 20));
    }
}
