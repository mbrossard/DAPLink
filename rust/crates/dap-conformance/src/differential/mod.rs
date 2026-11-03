//! The dap-rs reference on the simulated pads, and the harness comparing a probe's paths to it.

pub mod diff;
pub mod pads;
pub mod reference;
pub mod words;

pub use diff::{
    check, run, steps, survey, verdict, waive, Check, Divergence, Failure, Invariant, Kind, Probe,
    Report, Step,
};
pub use pads::{NoLeds, SimDelay, SimPin, CPU_HZ};
pub use reference::{
    dap_rs_only, jtag_transfer, reference, scan_chain, DapRsOnly, Latch, Latched, RefDap, RefDeps,
    RefJtag, RefSwd, Reference,
};
pub use words::SimWords;

#[cfg(test)]
const fn dap_rs(rule: crate::rule::RuleId) -> crate::rule::Waiver {
    crate::rule::Waiver {
        rule,
        reason: "the path is the reference unnormalized",
        on: |_| true,
        marked: false,
    }
}

/// The reference as a path: dap-rs's answers wherever the reference is normalized.
#[cfg(test)]
pub(crate) static DAP_RS: crate::rule::Policy = crate::rule::Policy {
    waivers: &[
        dap_rs(crate::rule::RuleId::WaitAttempts),
        dap_rs(crate::rule::RuleId::ValueMatch),
        dap_rs(crate::rule::RuleId::CountCompleted),
        dap_rs(crate::rule::RuleId::NoAckStatus),
    ],
};
