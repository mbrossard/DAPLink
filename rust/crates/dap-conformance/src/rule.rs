//! Named rules, each with the basis it rests on, and the per-engine policies that waive them.

use std::fmt;

use crate::session::{Profile, Wire};

/// What a rule rests on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Basis {
    /// A specification section.
    Spec(&'static str),
    /// Arm's reference firmware by release, file and function, where the specification is silent.
    Reference(&'static str),
    /// A decision of this project, in one line.
    Policy(&'static str),
    /// Nothing is owed: comparisons mask the observable.
    Unspecified,
}

/// Where a rule is checked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hook {
    /// The differential compares it, normalizes the reference to it or masks it.
    Differential,
    /// Only these literal tests, by path.
    Literal(&'static [&'static str]),
    /// Nowhere, for this reason.
    Unchecked(&'static str),
}

/// A rule's identity; [`RULES`] holds one entry per variant, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RuleId {
    SwdHeader,
    SwdContention,
    TrailingTurnaround,
    NoAckBackOff,
    IdleBetweenFrames,
    TapIdcode,
    CaptureLoad,
    ShortScan,
    ReadResultUnknown,
    JtagAckProtocol,
    NoAckStatus,
    ProtocolErrorAck,
    JtagFaultStatus,
    StopAtFailure,
    WaitAttempts,
    IdleCycles,
    ValueMatch,
    CountCompleted,
    CountFailed,
    NresetHeld,
    TdiLevel,
    SwdioLevel,
    ParkedAfterConnect,
    ParkedPinSelect,
    ClockCeiling,
    NoStrayEdge,
}

/// One rule: what it requires and on what basis.
#[derive(Clone, Copy, Debug)]
pub struct Rule {
    /// Its identity.
    pub id: RuleId,
    /// Its stable name.
    pub name: &'static str,
    /// What it rests on.
    pub basis: Basis,
    /// What it requires, in one line.
    pub summary: &'static str,
    /// Where it is checked.
    pub hook: Hook,
}

const fn rule(
    id: RuleId,
    name: &'static str,
    basis: Basis,
    summary: &'static str,
    hook: Hook,
) -> Rule {
    Rule {
        id,
        name,
        basis,
        summary,
        hook,
    }
}

const IDLE_CYCLES: &[&str] = &[
    "daplink_rs::differential_tests::known_divergence_idle_cycles_after_an_ok_ack",
    "daplink_spi_swd::differential_tests::known_divergence_idle_cycles_after_an_ok_ack",
    "daplink_stm32_embassy::block::differential_tests::differential::known_divergence_idle_cycles_after_an_ok_ack",
    "daplink_stm32_embassy::spi_block::differential_tests::known_divergence_idle_cycles_after_an_ok_ack",
];

const NRESET_HELD: &[&str] = &[
    "daplink_spi_swd::differential_tests::known_divergence_nreset_held_across_the_next_transfer",
    "daplink_stm32_embassy::block::differential_tests::differential::known_divergence_nreset_held_across_swd_entry",
    "daplink_stm32_embassy::block::differential_tests::differential::known_divergence_nreset_held_across_late_jtag_entry",
];

use RuleId::*;

/// The registry.
pub static RULES: [Rule; 26] = [
    rule(
        SwdHeader,
        "swd.header.valid",
        Basis::Spec("ADIv5 B4.2, B4.2.5"),
        "every request has a Start of 1, even parity, a Stop of 0 and a Park of 1",
        Hook::Differential,
    ),
    rule(
        SwdContention,
        "swd.drive.contention",
        Basis::Spec("ADIv5 B4.1.3"),
        "the host never drives SWDIO in a turnaround or a cell the target drives",
        Hook::Differential,
    ),
    rule(
        TrailingTurnaround,
        "swd.read.trailing-turnaround",
        Basis::Spec("ADIv5 B4.1.6"),
        "a read clocks its trailing turnaround even when its data parity fails",
        Hook::Differential,
    ),
    rule(
        NoAckBackOff,
        "swd.no-ack.back-off",
        Basis::Spec("ADIv5 B4.2.5.2"),
        "after no ACK or an invalid one the host stays off the line for any data phase",
        Hook::Unchecked(
            "the reference breaks it, and a simulated target that sent no ACK ignores the line",
        ),
    ),
    rule(
        IdleBetweenFrames,
        "swd.idle.between-frames",
        Basis::Spec("ADIv5 B4.1.1"),
        "between packets the host clocks SWDIO low, since a clocked 1 starts a request",
        Hook::Differential,
    ),
    rule(
        TapIdcode,
        "jtag.ir.tlr-idcode",
        Basis::Spec("ADIv5 B3.3.3"),
        "Test-Logic-Reset selects IDCODE, or BYPASS without one, in every TAP",
        Hook::Literal(&[
            "dap_conformance::target::tests::test_logic_reset_selects_idcode_in_every_tap",
        ]),
    ),
    rule(
        CaptureLoad,
        "jtag.ir.capture-load",
        Basis::Spec("ADIv5 B3.2.3, B3.3.3"),
        "Capture-IR loads a value whose two low bits are 01",
        Hook::Literal(&["dap_conformance::wire::jtag::tests::capture_ir_loads_01_into_every_ir"]),
    ),
    rule(
        ShortScan,
        "jtag.xpacc.short-scan-executes",
        Basis::Spec("ADIv5 B3.4.3.2, B3.4.3.3"),
        "an xPACC scan answered OK issues what its shift register holds, a short scan included",
        Hook::Literal(&[
            "dap_conformance::wire::jtag::tests::update_dr_issues_what_a_short_scan_left",
        ]),
    ),
    rule(
        ReadResultUnknown,
        "jtag.readresult.unknown",
        Basis::Spec("ADIv5 B3.4.3.2; ADIv6 B3.4.3.6"),
        "ReadResult is UNKNOWN after an IR change, a write, a FAULT or Test-Logic-Reset",
        Hook::Differential,
    ),
    rule(
        JtagAckProtocol,
        "jtag.ack.protocol",
        Basis::Spec("ADIv6 B3.1, Table B3-6"),
        "a JTAG-DP's ACK decodes by its protocol, 0 for ADIv5 and 1 for ADIv6",
        Hook::Differential,
    ),
    rule(
        NoAckStatus,
        "dap.status.no-ack-is-7",
        Basis::Spec("CMSIS-DAP 2.1.1 DAP_Transfer, Transfer Response"),
        "a transfer no target answered reports 7, a write as well as a read",
        Hook::Differential,
    ),
    rule(
        ProtocolErrorAck,
        "dap.status.protocol-error-ack-bits",
        Basis::Unspecified,
        "bits 2:0 of a protocol error, 0x08 from Arm's firmware and 0x0F from dap-rs",
        Hook::Differential,
    ),
    rule(
        JtagFaultStatus,
        "dap.status.jtag-fault-is-4",
        Basis::Policy("a probe tells FAULT from OK where CMSIS-DAP's JTAG status 1 means either"),
        "a JTAG FAULT reports 4 and no ACK 7",
        Hook::Differential,
    ),
    rule(
        StopAtFailure,
        "dap.transfer.stop-at-failure",
        Basis::Spec("CMSIS-DAP 2.1.1 DAP_Transfer"),
        "a transfer stops at its first failed item",
        Hook::Differential,
    ),
    rule(
        WaitAttempts,
        "dap.transfer.wait-attempts",
        Basis::Spec("CMSIS-DAP 2.1.1 DAP_TransferConfigure, WaitRetry"),
        "N WAIT retries make N + 1 attempts",
        Hook::Differential,
    ),
    rule(
        IdleCycles,
        "dap.transfer.swd-idle-cycles",
        Basis::Spec("CMSIS-DAP 2.1.1 DAP_TransferConfigure, IdleCycles"),
        "SWD clocks the configured idle cycles after each packet answered OK",
        Hook::Literal(IDLE_CYCLES),
    ),
    rule(
        ValueMatch,
        "dap.transfer.value-match",
        Basis::Spec("CMSIS-DAP 2.1.1 DAP_Transfer, Value Match"),
        "a value match compares each read's own value",
        Hook::Differential,
    ),
    rule(
        CountCompleted,
        "dap.count.completed",
        Basis::Reference("CMSIS-DAP 2.1.2 DAP.c DAP_SWD_TransferBlock"),
        "a failed block counts what completed; empty and mask-only requests answer as Arm's",
        Hook::Differential,
    ),
    rule(
        CountFailed,
        "dap.transfer.count-failed",
        Basis::Policy("dap-rs's count cannot be rewritten: it hides whether a posted read or its collection failed"),
        "a failed SWD DAP_Transfer counts the transfer that failed",
        Hook::Differential,
    ),
    rule(
        NresetHeld,
        "dap.reset.held-across-entry",
        Basis::Policy("connect-under-reset needs the target held in reset through entry"),
        "nRESET stays asserted across SWD and JTAG entry",
        Hook::Literal(NRESET_HELD),
    ),
    rule(
        TdiLevel,
        "dap.pins.tdi-idle-level",
        Basis::Unspecified,
        "the level TDI rests at between wire commands",
        Hook::Differential,
    ),
    rule(
        SwdioLevel,
        "dap.pins.swdio-idle-level",
        Basis::Unspecified,
        "the level SWDIO/TMS rests at between wire commands",
        Hook::Differential,
    ),
    rule(
        ParkedAfterConnect,
        "dap.pins.parked-after-connect",
        Basis::Unspecified,
        "whether DAP_Connect takes the pads or the first wire command does",
        Hook::Differential,
    ),
    rule(
        ParkedPinSelect,
        "dap.pins.parked-pin-select",
        Basis::Unspecified,
        "whether DAP_SWJ_Pins drives a parked pin it selects or only latches it",
        Hook::Differential,
    ),
    rule(
        ClockCeiling,
        "dap.clock.never-above-request",
        Basis::Policy("a probe never clocks a target faster than the host asked"),
        "no packet runs faster than the last nonzero DAP_SWJ_Clock request",
        Hook::Differential,
    ),
    rule(
        NoStrayEdge,
        "dap.wire.no-edge-outside-wire-work",
        Basis::Policy("a clock edge outside wire work can clock a target out of step"),
        "no command clocks an edge outside its own wire work",
        Hook::Differential,
    ),
];

impl RuleId {
    /// The registry entry.
    pub fn rule(self) -> &'static Rule {
        &RULES[self as usize]
    }

    /// The rule's stable name.
    pub fn name(self) -> &'static str {
        self.rule().name
    }

    #[cfg(feature = "differential")]
    pub(crate) fn bit(self) -> u64 {
        1 << self as u32
    }
}

impl fmt::Display for RuleId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// An engine's exemption from one rule, with the reason it stands.
#[derive(Clone, Copy, Debug)]
pub struct Waiver {
    /// The rule waived.
    pub rule: RuleId,
    /// Why, in one line.
    pub reason: &'static str,
    /// The profiles it is in force on; a check of one fails if it matched nothing.
    pub on: fn(&Profile) -> bool,
    /// In force only on the reports the path marks with `waive`, not on every report.
    pub marked: bool,
}

impl Waiver {
    /// Profiles with SWD in play.
    pub fn swd(profile: &Profile) -> bool {
        profile.wire != Wire::Jtag
    }
}

/// What one engine is held to: every rule with a basis, less its waivers; `Unspecified` is masked.
#[derive(Clone, Copy, Debug)]
pub struct Policy {
    /// The waivers, each with its reason.
    pub waivers: &'static [Waiver],
}

impl Policy {
    /// The waiver of `rule` in force on `profile`, if any.
    pub fn waiver(&self, rule: RuleId, profile: &Profile) -> Option<&Waiver> {
        self.waivers
            .iter()
            .find(|w| w.rule == rule && (w.on)(profile))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_rule_names_its_hook() {
        for r in &RULES {
            match r.hook {
                Hook::Literal(tests) => assert!(!tests.is_empty(), "{}", r.name),
                Hook::Unchecked(why) => assert!(!why.is_empty(), "{}", r.name),
                Hook::Differential => {}
            }
            if r.basis == Basis::Unspecified {
                assert_eq!(r.hook, Hook::Differential, "{} is masked", r.name);
            }
        }
    }

    #[test]
    fn the_registry_holds_each_rule_once_in_order() {
        for (i, r) in RULES.iter().enumerate() {
            assert_eq!(r.id as usize, i, "{}", r.name);
            assert!(RULES[..i].iter().all(|o| o.name != r.name), "{}", r.name);
        }
    }
}
