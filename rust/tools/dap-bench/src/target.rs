//! Per-target address profiles: every measurement address must come from, and stay within, one.

use anyhow::{anyhow, Result};

use crate::adi::{ApMode, SelectV6};

/// A named target's read/write windows on SWD.
#[derive(Debug)]
pub struct TargetProfile {
    /// Profile name, matched against `--target`.
    pub name: &'static str,
    /// Flash window base address (read direction).
    pub flash_base: u32,
    /// Flash window size in bytes.
    pub flash_window_bytes: u32,
    /// SRAM window base address (write direction; the run's end-of-sweep reset re-initializes it).
    pub sram_base: u32,
    /// SRAM window size in bytes.
    pub sram_window_bytes: u32,
    /// Multidrop `TARGETSEL` to select before attach; `None` on a single-drop DP.
    pub targetsel: Option<u32>,
    /// How this MEM-AP is addressed; the ADIv5 and ADIv6 encodings are not interchangeable.
    pub ap: ApMode,
}

/// 128 KiB, the window size shared by every profile below.
const WINDOW_128K: u32 = 128 * 1024;

/// Known target profiles.
pub const PROFILES: &[TargetProfile] = &[
    TargetProfile {
        name: "stm32",
        flash_base: 0x0800_0000,
        flash_window_bytes: WINDOW_128K,
        sram_base: 0x2000_0000,
        sram_window_bytes: WINDOW_128K,
        targetsel: None,
        ap: ApMode::V5 { apsel: 0 },
    },
    TargetProfile {
        name: "rp",
        flash_base: 0x1000_0000,
        flash_window_bytes: WINDOW_128K,
        sram_base: 0x2000_0000,
        sram_window_bytes: WINDOW_128K,
        // RP2040 core-0 debug port, selected on the multidrop SWD-DP behind TARGETSEL.
        targetsel: Some(0x0100_2927),
        ap: ApMode::V5 { apsel: 0 },
    },
    TargetProfile {
        name: "rp2350",
        flash_base: 0x1000_0000,
        flash_window_bytes: WINDOW_128K,
        sram_base: 0x2000_0000,
        sram_window_bytes: WINDOW_128K,
        // RP2350 core-0 debug port; the RP2040's TARGETSEL here NoAcks every DPIDR read.
        targetsel: Some(0x0004_0927),
        // ADIv6: the core-0 AP is addressed at 0x2000, not by an APSEL index.
        ap: ApMode::V6 {
            ap_base: 0x2000,
            sel: SelectV6::new(),
        },
    },
];

/// Default `--target` when the flag is absent.
pub const DEFAULT_TARGET: &str = "stm32";

/// Look up a profile by name, erroring with the known-name list on a miss.
pub fn lookup(name: &str) -> Result<&'static TargetProfile> {
    PROFILES.iter().find(|p| p.name == name).ok_or_else(|| {
        let known: Vec<&str> = PROFILES.iter().map(|p| p.name).collect();
        anyhow!("unknown target '{name}' (known: {})", known.join(", "))
    })
}

impl TargetProfile {
    /// The bring-up this profile asks for, which is precisely what a wrong `--target` gets wrong.
    fn attach_shape(&self) -> String {
        let dp = match self.targetsel {
            Some(v) => format!("TARGETSEL {v:#010x}"),
            None => "single-drop DP".to_string(),
        };
        let ap = match self.ap {
            ApMode::V5 { apsel } => format!("ADIv5 AP {apsel}"),
            ApMode::V6 { ap_base, .. } => format!("ADIv6 AP at {ap_base:#x}"),
        };
        format!("{dp}, {ap}")
    }

    /// Context for an attach failure. A valid but wrong profile fails the same way a dead target
    /// or a clock ceiling does -- NoAck on every DPIDR read, at every clock -- so the failure has
    /// to say which profile it drove the DP with and what else was on offer.
    pub fn attach_context(&self) -> String {
        let others: Vec<String> = PROFILES
            .iter()
            .filter(|p| p.name != self.name)
            .map(|p| format!("{} = {}", p.name, p.attach_shape()))
            .collect();
        format!(
            "attached as target profile '{}' ({}); a DPIDR NoAck at every clock is also what the wrong profile for this silicon looks like (others: {})",
            self.name,
            self.attach_shape(),
            others.join("; "),
        )
    }

    /// True when `[addr, addr+len_bytes)` fits entirely inside the flash window.
    pub fn flash_contains(&self, addr: u32, len_bytes: usize) -> bool {
        range_within(self.flash_base, self.flash_window_bytes, addr, len_bytes)
    }

    /// True when `[addr, addr+len_bytes)` fits entirely inside the SRAM window.
    pub fn sram_contains(&self, addr: u32, len_bytes: usize) -> bool {
        range_within(self.sram_base, self.sram_window_bytes, addr, len_bytes)
    }
}

/// Whether `[addr, addr+len_bytes)` fits entirely inside `[win_base, win_base+win_len)`.
fn range_within(win_base: u32, win_len: u32, addr: u32, len_bytes: usize) -> bool {
    let Ok(len) = u32::try_from(len_bytes) else {
        return false;
    };
    let Some(end) = addr.checked_add(len) else {
        return false;
    };
    let win_end = u64::from(win_base) + u64::from(win_len);
    u64::from(addr) >= u64::from(win_base) && u64::from(end) <= win_end
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup_finds_known_profiles() {
        assert_eq!(lookup("stm32").unwrap().flash_base, 0x0800_0000);
        assert_eq!(lookup("rp").unwrap().flash_base, 0x1000_0000);
    }

    #[test]
    fn lookup_rejects_unknown_name_and_lists_known_ones() {
        let err = lookup("nrf52").unwrap_err().to_string();
        assert!(err.contains("nrf52"));
        assert!(err.contains("stm32"));
        assert!(err.contains("rp"));
    }

    /// The failure this names cost an hour: an RP2350 driven as `rp` NoAcks every DPIDR read,
    /// which is indistinguishable from a dead target until the profile is on the error.
    #[test]
    fn an_attach_failure_names_the_profile_it_used_and_the_alternatives() {
        let ctx = lookup("rp").unwrap().attach_context();
        assert!(ctx.contains("'rp'"), "{ctx}");
        assert!(ctx.contains("0x01002927"), "{ctx}");
        assert!(
            ctx.contains("rp2350 = TARGETSEL 0x00040927, ADIv6 AP at 0x2000"),
            "{ctx}"
        );
        assert!(ctx.contains("stm32 = single-drop DP, ADIv5 AP 0"), "{ctx}");
    }

    #[test]
    fn range_within_bounds_checks_the_window() {
        let p = lookup("stm32").unwrap();
        assert!(p.flash_contains(p.flash_base, 4096));
        assert!(p.flash_contains(p.flash_base + p.flash_window_bytes - 4096, 4096));
        assert!(!p.flash_contains(p.flash_base + p.flash_window_bytes - 4, 4096));
        assert!(!p.flash_contains(p.flash_base - 4, 4096));
        assert!(!p.flash_contains(p.flash_base, usize::MAX));
    }
}
