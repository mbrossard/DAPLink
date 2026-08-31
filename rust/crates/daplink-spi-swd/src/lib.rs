//! SPI-session SWD transport plus direct-GPIO JTAG for SPI blocks whose frame geometry is
//! write-protected while enabled: each SWD transfer phase runs as its own short session with its
//! own width, count, and half-duplex direction. HAL-free; the chip's half arrives through the
//! [`SpiPortOps`] and [`JtagPortOps`] register seams.

#![no_std]
#![deny(missing_docs)]

mod engine;
mod port;
mod proto;
mod scan;
pub mod session;
mod timing;

pub use engine::{SpiJtag, SpiSwd, SpiSwdDeps};
pub use port::{nops, JtagPacing, JtagPortOps, NoFastJtag, SpiPortOps};
pub use scan::JtagScan;
pub use session::{choose_prescaler, ladder_rung, Dir, Prescaler, Session};
pub use timing::{
    prepare_jtag_timing, prepare_swd_timing, HalfCycleProfile, HalfCycles, JtagTimingPlan,
    PacingPlan, PacingProfile, SwdTimingPlan,
};
