//! `dap-bench` -- a host-side CMSIS-DAP v2 block-transfer throughput benchmark.

#![deny(missing_docs)]

pub mod adi;
pub mod dap;
pub mod report;
pub mod sweep;
pub mod target;
pub mod transport;
