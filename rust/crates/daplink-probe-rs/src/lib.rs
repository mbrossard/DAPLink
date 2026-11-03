//! Drives a real target's SWD debug port through probe-rs, as a daplink-flash transport.

#![deny(missing_docs)]
#![cfg_attr(not(test), deny(clippy::unwrap_used))]

mod adapter;

pub use adapter::{ProbeRsDeps, ProbeRsSwd};
pub use probe_rs::architecture::arm::{DapProbe, RawDapAccess};
