//! SPI-session SWD plus GPIO JTAG, and the framing, pacing and scan toolkit other engines share.

#![no_std]
#![deny(missing_docs)]

mod engine;
pub mod pace;
mod port;
mod scan;
pub mod session;
mod timing;

#[cfg(test)]
mod differential_tests;

pub use engine::{SpiJtag, SpiSwd, SpiSwdDeps};
pub use port::{nops, spin, JtagPacing, JtagPortOps, NoFastJtag, PortResult, SpiFault, SpiPortOps};
pub use scan::{
    scan_shift_ir_word, scan_transfer_word, FastScan, JtagScan, JtagScanState, KeptAck, ScaledScan,
};
pub use session::{choose_prescaler, ladder_rung, Dir, Prescaler, Session, SpiClock, SpiLadder};
pub use timing::{
    prepare_jtag_timing, prepare_swd_timing, FrequencyPacing, HalfCycleProfile, HalfCycles,
    JtagTimingPlan, PacingPlan, PacingProfile, SwdTimingPlan,
};
