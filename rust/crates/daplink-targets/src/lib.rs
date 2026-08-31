//! Flash-target registry: one record per chip, referencing shared algorithms and family sequences.

#![no_std]
#![deny(missing_docs)]
#![cfg_attr(not(test), deny(clippy::unwrap_used))]

#[cfg(test)]
extern crate std;

pub use daplink_flash::flash_algo::{
    FlashAlgorithm, FlashRegion, RamRegion, SectorInfo, SyscallParams,
};
pub use daplink_flash::identity::{check_identity, IdentityProbe};
pub use daplink_flash::reset_sequence::{ResetOps, ResetSequence};
pub use daplink_flash::target::Target;

pub mod algorithms {
    //! Shared, content-deduped flash-algorithm modules (one per flash controller).
    #[cfg(feature = "_algo_rp2040")]
    pub mod rp2040_rom;
    #[cfg(feature = "_algo_rp2350")]
    pub mod rp235x;
    #[cfg(feature = "_algo_stm32f4")]
    pub mod stm32f4;
    #[cfg(feature = "synthetic")]
    pub mod synthetic;
}

pub mod families {
    //! Hand-written per-family reset/unlock behavior and register facts.
    #[cfg(feature = "_family_rp2040")]
    pub mod rp2040;
    #[cfg(feature = "_family_rp2350")]
    pub mod rp2350;
}

pub mod targets {
    //! Per-chip target definitions referencing an algorithm and a family sequence by handle.
    #[cfg(feature = "rp2040")]
    pub mod rp2040;
    #[cfg(feature = "rp2350")]
    pub mod rp235x;
    #[cfg(feature = "stm32f401ce")]
    pub mod stm32f401ce;
    #[cfg(feature = "synthetic")]
    pub mod synthetic;
}

pub mod selector;

#[macro_use]
pub mod table;

pub use selector::{FixedTarget, ImageHint, SelectError, TargetSelector};

#[cfg(all(test, feature = "synthetic"))]
mod tests;

#[cfg(all(test, feature = "stm32f401ce"))]
mod tests_stm32f401ce;

#[cfg(all(test, feature = "rp2040"))]
mod tests_rp2040;

#[cfg(all(test, feature = "rp2350"))]
mod tests_rp235x;
