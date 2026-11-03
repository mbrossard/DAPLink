//! Wired-target table: which chip, address and abilities a named target provides.

use crate::profile::Req;

/// A named wired target: probe-rs chip, a readable address for the speed cases, and its abilities.
#[derive(Debug)]
pub(crate) struct Target {
    pub(crate) name: &'static str,
    pub(crate) chip: &'static str,
    pub(crate) read_addr: u64,
    /// `DBGMCU_CR` address for trace-pin routing; `None` where the silicon has none.
    pub(crate) dbgmcu_cr: Option<u64>,
    /// An address the silicon maps to nothing, for the fault case; `None` where none is known.
    pub(crate) fault_addr: Option<u64>,
    /// JTAG IR lengths in `DAP_JTAG_Configure` order, TDO first; empty for a single TAP or no JTAG-DP.
    pub(crate) jtag_ir_lens: &'static [u8],
    pub(crate) supports: &'static [Req],
}

pub(crate) const TARGETS: &[Target] = &[
    Target {
        name: "stm32f401",
        chip: "STM32F401CEUx",
        read_addr: 0x0800_0000,
        dbgmcu_cr: Some(0xE004_2004),
        // FSMC bank 1: the region exists in the family's map and this part has no FSMC.
        fault_addr: Some(0x6000_0000),
        // TDO first: the Cortex-M4 debug TAP, then the boundary-scan TAP behind it.
        jtag_ir_lens: &[4, 5],
        supports: &[Req::Jtag, Req::Swo, Req::Vcom, Req::Nreset],
    },
    Target {
        name: "rp2040",
        chip: "RP2040",
        read_addr: 0x1000_0000,
        dbgmcu_cr: None,
        // Between the AHB-lite peripherals and the SIO block, the bus fabric decodes nothing.
        fault_addr: Some(0x6000_0000),
        jtag_ir_lens: &[],
        // No JTAG-DP and no ITM on M0+: no jtag, no swo.
        supports: &[Req::Vcom, Req::Nreset],
    },
    Target {
        name: "rp2350",
        chip: "RP235x",
        read_addr: 0x1000_0000,
        dbgmcu_cr: None,
        fault_addr: Some(0x6000_0000),
        jtag_ir_lens: &[],
        // SWD-only DP, and the M33's ITM has no SWO route to a pin: no jtag, no swo.
        supports: &[Req::Vcom, Req::Nreset],
    },
];

pub(crate) fn target(name: &str) -> Option<&'static Target> {
    TARGETS.iter().find(|t| t.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `Req::Swo` target must carry a `dbgmcu_cr` inside the debug region.
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

    /// The fault address must be unmapped and distinct from the speed cases' address.
    #[test]
    fn a_fault_address_is_not_the_readable_one() {
        for t in TARGETS {
            if let Some(addr) = t.fault_addr {
                assert_ne!(
                    addr, t.read_addr,
                    "{}: fault address is the read address",
                    t.name
                );
                assert!(addr != 0, "{}: fault address is zero", t.name);
            }
        }
    }

    /// The jtag-chain case needs JTAG and at least two TAPs.
    #[test]
    fn a_declared_jtag_chain_has_more_than_one_tap() {
        for t in TARGETS {
            if !t.jtag_ir_lens.is_empty() {
                assert!(
                    t.supports.contains(&Req::Jtag),
                    "{}: chain without jtag",
                    t.name
                );
                assert!(t.jtag_ir_lens.len() >= 2, "{}: chain of one TAP", t.name);
                assert!(
                    t.jtag_ir_lens.iter().all(|&n| (2..=32).contains(&n)),
                    "{}",
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
