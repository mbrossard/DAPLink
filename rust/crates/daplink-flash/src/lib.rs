//! Flash-algorithm runtime: drives on-target flash programming over the `dap-rs` SWD seam.
#![no_std]
#![deny(missing_docs)]
#![cfg_attr(not(test), deny(clippy::unwrap_used))]

#[cfg(any(feature = "std", test))]
extern crate alloc;

pub use dap_rs;

#[cfg(feature = "defmt")]
mod debug_status;
pub mod error;
pub mod flash_algo;
pub mod flash_intf;
pub mod flash_manager;
pub mod identity;
pub mod reset;
pub mod reset_sequence;
pub mod swd_host;
mod swd_host_core;
pub mod target;
pub mod target_flash;

#[cfg(any(feature = "std", test))]
pub mod mock;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod tests_verify;

pub use error::{Error, SwdError};
pub use flash_algo::{FlashAlgorithm, FlashRegion, RamRegion, SectorInfo, SyscallParams};
pub use flash_intf::FlashInterface;
pub use flash_manager::{page_buffer_ok, EraseMode, FlashManager};
pub use identity::{check_identity, IdentityProbe};
pub use reset::{FlashArbiter, NoArbiter, ResetControl};
pub use reset_sequence::{
    generic_set_target_state, GenericResetSequence, ResetOps, ResetSequence, UnlockOutcome,
    UnlockPolicy, GENERIC_RESET_SEQUENCE,
};
pub use swd_host::{swd_dormant_wake_preamble, swd_wake_preamble, SwdHost};
pub use target::{
    ApAddressing, DebugPort, DefaultCallbacks, DpWake, ResetType, StateOutcome, Target,
    TargetCallbacks, TargetState,
};
pub use target_flash::TargetFlash;
