//! Wired-target table: which chip, address and abilities a named target provides.

use crate::profile::Req;

/// A named wired target: its probe-rs chip name, an always-readable address for the speed
/// cases, and the abilities its silicon and wiring actually provide.
#[derive(Debug)]
pub(crate) struct Target {
    pub(crate) name: &'static str,
    pub(crate) chip: &'static str,
    pub(crate) read_addr: u64,
    /// `DBGMCU_CR` address for trace-pin routing (TRACE_IOEN bit 5, TRACE_MODE bits 7:6);
    /// `None` where the silicon has no such register.
    pub(crate) dbgmcu_cr: Option<u64>,
    pub(crate) supports: &'static [Req],
}

/// Known wired targets.
pub(crate) const TARGETS: &[Target] = &[
    Target {
        name: "stm32f401",
        chip: "STM32F401CEUx",
        read_addr: 0x0800_0000,
        dbgmcu_cr: Some(0xE004_2004),
        supports: &[Req::Jtag, Req::Swo, Req::Vcom, Req::Nreset],
    },
    Target {
        name: "rp2040",
        chip: "RP2040",
        read_addr: 0x1000_0000,
        dbgmcu_cr: None,
        // No JTAG-DP and no ITM on M0+: no jtag, no swo.
        supports: &[Req::Vcom, Req::Nreset],
    },
    Target {
        name: "rp2350",
        chip: "RP235x",
        read_addr: 0x1000_0000,
        dbgmcu_cr: None,
        // SWD-only DP, and the M33's ITM has no SWO route to a pin: no jtag, no swo.
        supports: &[Req::Vcom, Req::Nreset],
    },
];

/// Look up a target by name.
pub(crate) fn target(name: &str) -> Option<&'static Target> {
    TARGETS.iter().find(|t| t.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `swo::run` reads and writes `dbgmcu_cr` before its sweep, and the case reaches a target
    /// only through `Req::Swo`: a row claiming the capability without the register would make
    /// the whole case skip, and one carrying an address outside the debug region would poke
    /// whatever else lives there.
    #[test]
    fn a_target_claiming_swo_names_its_trace_register() {
        for t in TARGETS {
            if t.supports.contains(&Req::Swo) {
                let addr = t
                    .dbgmcu_cr
                    .unwrap_or_else(|| panic!("{}: no dbgmcu_cr", t.name));
                assert!(
                    (0xE000_0000..=0xE00F_FFFF).contains(&addr),
                    "{}: dbgmcu_cr {addr:#x} is outside the debug region",
                    t.name
                );
            }
        }
    }

    #[test]
    fn target_names_are_unique() {
        for (i, t) in TARGETS.iter().enumerate() {
            assert!(
                TARGETS[..i].iter().all(|p| p.name != t.name),
                "duplicate target `{}`",
                t.name
            );
        }
    }
}
