//! PIO SWD wire backend: a state machine shifts the bits, the CPU keeps the protocol.

#![no_std]
#![deny(missing_docs)]

#[cfg(test)]
extern crate std;

pub mod clock;
pub mod codec;
pub mod pins;
pub mod program;
pub mod proto;
pub mod shifter;
pub mod swd;
pub mod transfer;

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub mod sm;

pub use clock::{clock_divider, CYCLES_PER_BIT, MIN_DIVIDER};
pub use codec::{cmd_word, read_align, skip_word};
pub use pins::{PinMap, PioSwdPins};
pub use program::{select_program, PioProgram, ProgramEntries, ProgramLabels, Variant};
pub use shifter::{PioShifter, SwjPinsSio};
pub use swd::{PioSwdDeps, PioSwdDrv, PioSwdJtagDap, PioSwdOnlyDap};
pub use transfer::{PioSwd, XferError, DEFAULT_TURNAROUND};

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use sm::{pio_swd_deps, PioBlock, PioSm};
