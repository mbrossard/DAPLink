//! What the target records when asked: one cell per clock rising edge, and the transactions.

use crate::adi::jtag_dp::Xpacc;
use crate::wire::jtag::Scan;
use crate::wire::swd::Transfer;
use crate::wire::{CellRole, Drives, Lines};

/// One clock rising edge as the target saw it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    /// The levels at the rise.
    pub lines: Lines,
    /// Who drove them.
    pub drives: Drives,
    /// What the target took the cell to be.
    pub role: CellRole,
}

/// One wire attempt, WAITs included.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Transaction {
    /// An SWD packet.
    Swd(Transfer),
    /// A TARGETSEL write: its data, and whether its parity matched.
    Targetsel(u32, bool),
    /// A JTAG-DP access, after the `Scan` that carried it.
    Xpacc(Xpacc),
    /// A JTAG IR or DR scan.
    Scan(Scan),
}

/// The cell log and the transaction log.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Log {
    /// The cells, in order.
    pub cells: Vec<Cell>,
    /// The transactions, in order.
    pub transactions: Vec<Transaction>,
}
