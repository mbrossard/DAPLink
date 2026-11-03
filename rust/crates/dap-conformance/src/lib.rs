//! Simulated SWD/JTAG target and its observer, the CMSIS-DAP command layer, rules, seeded streams; dap-rs with `differential`.

pub mod adi;
pub mod cmsis_dap;
pub mod corpus;
#[cfg(feature = "differential")]
pub mod differential;
pub mod observe;
pub mod rule;
pub mod session;
pub mod target;
pub mod wire;

// The root re-exports every public item of `differential`, `rule`, `session` and `target`.
#[cfg(feature = "differential")]
pub use differential::{
    check, dap_rs_only, jtag_transfer, reference, run, scan_chain, steps, survey, verdict, waive,
    Check, DapRsOnly, Divergence, Failure, Invariant, Kind, Latch, Latched, NoLeds, Probe, RefDap,
    RefDeps, RefJtag, RefSwd, Reference, Report, SimDelay, SimPin, SimWords, Step, CPU_HZ,
};
pub use rule::{Basis, Hook, Policy, Rule, RuleId, Waiver, RULES};
pub use session::{
    drw_poll, jtag_select, sticky_fault, stream, swd_select, Cmd, Item, Profile, Rng, Stream, Wire,
};
pub use target::{
    shared, Cell, DpVersion, Effect, Line, LinkState, Log, Mode, Pad, Shared, Sim, Target,
    Transaction, ADI_IDCODE, ADI_IDCODE_V1, ADI_IDCODE_V6, DPIDR, DPIDR_V3, FAULT_BASE,
    OTHER_IDCODE,
};
