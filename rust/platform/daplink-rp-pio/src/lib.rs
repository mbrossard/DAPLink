//! RP PIO SWD/JTAG sequence engines and optional complete-transfer accelerators.

#![no_std]
#![deny(missing_docs)]

#[cfg(all(feature = "rp2040", feature = "rp235x"))]
compile_error!("select exactly one RP chip: `rp2040` or `rp235x`");
#[cfg(all(target_os = "none", not(any(feature = "rp2040", feature = "rp235x"))))]
compile_error!("select an RP chip: `rp2040` or `rp235x`");

#[cfg(test)]
extern crate std;

mod asm;
#[cfg(any(test, feature = "rp2040", feature = "rp235x"))]
mod autonomous_program;
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
mod autonomous_sm;
pub mod clock;
pub mod codec;
pub mod composition;
pub mod generic_roles;
pub mod jtag;
pub mod pins;
pub mod program;
pub mod proto;
pub mod sequence;
pub mod shifter;
pub mod swd_transaction_hybrid;
pub mod transaction;
pub mod transaction_program;

#[cfg(test)]
mod autonomous_interpreter_tests;
#[cfg(test)]
mod transaction_interpreter_tests;

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
mod regs;
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
mod transaction_sm;
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
mod transfer_engine;

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub mod sm;

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use autonomous_sm::RpAutonomousJtag;
pub use clock::{
    clock_divider, jtag_clock_divider, jtag_cycles_per_bit, swd_cycles_per_bit,
    swd_transaction_clock_divider, PioClock, PioClockDivider, JTAG_MIN_DIVIDER, MIN_DIVIDER,
    TRANSACTION_CYCLES_PER_BIT, TRANSACTION_MIN_DIVIDER,
};
pub use codec::{cmd_word, read_align, skip_word};
pub use composition::{
    PioJtagAccelerator, PioTransferDap, PioTransferDeps, PioTransferJtag, PioTransferSwd,
};
pub use generic_roles::PioSwdDeps;
pub use jtag::{jtag_cmd_word, jtag_inline_word, PioJtag, PioJtagShifter};
pub use pins::{JtagPinMap, PinMap, PioJtagPins, PioSwdPins};
pub use program::{
    select_jtag_program, select_program, JtagProgram, JtagProgramEntries, PioProgram,
    ProgramEntries, ProgramLabels, Variant,
};
pub use sequence::XferError;
pub use shifter::{PioShifter, SwjPinsSio};
pub use swd_transaction_hybrid::{
    PioSwdTransactionDeps, PioSwdTransactionDrv, PioSwdTransactionOnlyDap,
    PioSwdTransferAccelerator,
};
pub use transaction::{PioSwdTransaction, PioSwdTransactionIo};
pub use transaction_program::{
    select_transaction_program, TransactionProgram, TRANSACTION_DIRECT, TRANSACTION_OEN,
};

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use transaction_sm::{RpPioSwdTransaction, RpPioSwdTransactionFence};
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use transfer_engine::{RpPioTransferEngine, RpPioTransferFence};

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use sm::pio_transfer_deps;
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use sm::{pio_swd_transaction_deps, PioBlock, PioSm};
