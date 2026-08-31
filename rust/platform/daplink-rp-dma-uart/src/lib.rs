//! PL011 over the RP-series DMA: the channel sequence both consumers share, and a polled ring.

#![no_std]

mod rx;

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub mod seq;
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
mod uart;

pub use rx::{RxRing, RxTracker, RX_RING_LEN, RX_RING_LOG2};

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use uart::UartDma;
