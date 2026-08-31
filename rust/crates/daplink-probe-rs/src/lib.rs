//! Transport shim that drives a real target's SWD debug port through probe-rs.
//!
//! Lets the daplink-rs flash stack use a CMSIS-DAP probe (via probe-rs) purely as the wire:
//! [`ProbeRsSwd`] implements the `dap-rs` [`Swd`](daplink_flash::dap_rs::swd::Swd) transport seam
//! and the [`ResetControl`](daplink_flash::reset::ResetControl) trait over probe-rs's low-level
//! [`RawDapAccess`] register interface, so flash logic treats a physical probe like any other
//! SWD backend. Register addressing mirrors the flash host: the `dap-rs` seam's 2-bit
//! [`DPRegister`](daplink_flash::dap_rs::swd::DPRegister) selector (ADIv5 `A[3:2]`) is shifted
//! left by two.

#![deny(missing_docs)]
#![cfg_attr(not(test), deny(clippy::unwrap_used))]

mod adapter;

pub use adapter::{ProbeRsDeps, ProbeRsSwd};
pub use probe_rs::architecture::arm::{DapProbe, RawDapAccess};
