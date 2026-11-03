//! Wire definitions: the probe's lines, and the SWJ, SWD and JTAG framing machines.

pub mod jtag;
pub mod swd;
pub mod swj;

use jtag::{Chain, Registers};
use swd::Link;
use swj::{Detector, Mode, Swj};

/// One probe pad net.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Line {
    /// SWCLK / TCK.
    Swclk,
    /// SWDIO / TMS.
    Swdio,
    /// TDI.
    Tdi,
    /// TDO.
    Tdo,
    /// nRESET.
    Nreset,
}

impl Line {
    /// Every line, in index order.
    pub const ALL: [Line; 5] = [Line::Swclk, Line::Swdio, Line::Tdi, Line::Tdo, Line::Nreset];

    /// The line's position in [`Line::ALL`].
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// A level on every line, bit `i` for [`Line::ALL`]`[i]`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Lines(pub u8);

impl Lines {
    /// The levels in [`Line`] order.
    pub fn new(levels: [bool; 5]) -> Self {
        Self(
            levels
                .iter()
                .enumerate()
                .fold(0, |v, (i, &l)| v | u8::from(l) << i),
        )
    }

    /// The level on `line`.
    pub fn get(self, line: Line) -> bool {
        self.0 >> line.index() & 1 != 0
    }
}

/// Who drives each line in a cell.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Drives {
    /// The probe's level per line, in [`Line`] order; `None` where it does not drive.
    pub probe: [Option<bool>; 5],
    /// The target's level on its data line: SWDIO in SWD, TDO in JTAG.
    pub target: Option<bool>,
}

/// Who the protocol says drives a cell's data line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drive {
    /// The host.
    Host,
    /// The target.
    Target,
    /// Neither: a turnaround, or TDO outside a shift.
    Released,
}

/// What a cell is part of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// SWD out of step until a line reset.
    Lockout,
    /// High cells after a line reset.
    LineReset,
    /// Low cells between SWD packets.
    Idle,
    /// The SWD request header, Start first.
    Request,
    /// An SWD turnaround.
    Turnaround,
    /// The SWD ACK.
    Ack,
    /// SWD read data.
    ReadData,
    /// SWD read parity.
    ReadParity,
    /// SWD write data.
    WriteData,
    /// SWD write parity.
    WriteParity,
    /// The cell that completed an SWJ select sequence.
    Switch,
    /// A JTAG cell in this TAP state.
    Tap(jtag::TapState),
}

/// What the protocol says a cell is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CellRole {
    /// What the cell is part of.
    pub phase: Phase,
    /// Who should drive its data line.
    pub driver: Drive,
    /// The bit's index within its field or scan.
    pub bit: Option<u16>,
    /// Its class, from what the link or chain knew when it was clocked.
    pub class: Class,
}

/// A cell's class, with a stable name for graders and reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Class {
    /// An SWD request header bit.
    Request,
    /// The turnaround before an ACK.
    Turnaround,
    /// An ACK bit.
    Ack,
    /// Read data.
    ReadData,
    /// Read parity.
    ReadParity,
    /// The turnaround after a read's data phase.
    TrailingTurnaround,
    /// The turnaround before write data.
    WriteTurnaround,
    /// Write data.
    WriteData,
    /// Write parity.
    WriteParity,
    /// The turnaround after a WAIT or FAULT with no data phase.
    WaitTurnaround,
    /// The undriven data phase of a WAIT or FAULT read under ORUNDETECT.
    WaitData,
    /// Low cells after a packet with an OK, WAIT or FAULT ACK.
    Idle,
    /// Lockout, line resets, select sequences, TARGETSEL, and packets with an undefined ACK.
    Resident,
    /// No role: the framing or TAP state was not known.
    Unsynced,
    /// Shift-DR.
    ShiftDr,
    /// Shift-IR.
    ShiftIr,
    /// Test-Logic-Reset.
    TapReset,
    /// Any other TAP state.
    Control,
}

impl Class {
    /// The stable name.
    pub const fn name(self) -> &'static str {
        match self {
            Class::Request => "request",
            Class::Turnaround => "turnaround",
            Class::Ack => "ack",
            Class::ReadData => "read_data",
            Class::ReadParity => "read_parity",
            Class::TrailingTurnaround => "trailing_turnaround",
            Class::WriteTurnaround => "write_turnaround",
            Class::WriteData => "write_data",
            Class::WriteParity => "write_parity",
            Class::WaitTurnaround => "wait_turnaround",
            Class::WaitData => "wait_data",
            Class::Idle => "idle",
            Class::Resident => "resident",
            Class::Unsynced => "unsynced",
            Class::ShiftDr => "shift_dr",
            Class::ShiftIr => "shift_ir",
            Class::TapReset => "tap_reset",
            Class::Control => "control",
        }
    }

    /// The class of a cell with `role`, `Unsynced` without one.
    pub fn of(role: Option<CellRole>) -> Class {
        role.map_or(Class::Unsynced, |r| r.class)
    }
}

/// One event of an edge trace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TraceEvent {
    /// One clock rising edge.
    Cell {
        /// The host's lines at the rise.
        host: Lines,
        /// The target's lines at the trace's sample point.
        target_sample: Lines,
        /// The drivers, when known.
        drives: Option<Drives>,
        /// An opaque position, never a duration.
        stamp: u64,
    },
    /// A clock pulse too short to be a cell.
    Glitch {
        /// Its position.
        stamp: u64,
    },
    /// A known break, such as a command boundary.
    Gap {
        /// Its position.
        stamp: u64,
    },
}

/// An SWJ-DP's wire side: select sequences, the SWD link and the JTAG chain.
#[derive(Clone, Debug)]
pub struct Interface {
    /// The protocol selected.
    pub mode: Mode,
    swj: Detector,
    /// The SWD link.
    pub link: Link,
    /// The JTAG chain.
    pub chain: Chain,
}

/// What one rising edge did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clocked {
    /// What the cell was.
    pub role: CellRole,
    /// A select sequence or line reset it completed.
    pub swj: Option<Swj>,
    /// What it completed on the SWD link.
    pub event: Option<swd::Event>,
}

impl Interface {
    /// An interface in `mode` over `link` and `chain`.
    pub fn new(mode: Mode, link: Link, chain: Chain) -> Self {
        Self {
            mode,
            swj: Detector::default(),
            link,
            chain,
        }
    }

    /// One rising edge: SWDIO/TMS, TDI, and the TDO seen on the wire if known.
    pub fn clock<P: swd::Port + Registers>(
        &mut self,
        swdio: bool,
        tdi: bool,
        tdo: Option<bool>,
        port: &mut P,
    ) -> Clocked {
        let swj = self.swj.clock(swdio, self.mode);
        let (role, event) = match swj {
            Some(Swj::Switch(mode)) => {
                self.mode = mode;
                self.link.lockout();
                self.chain.reset(port);
                let role = CellRole {
                    phase: Phase::Switch,
                    driver: Drive::Host,
                    bit: None,
                    class: Class::Resident,
                };
                (role, None)
            }
            _ if self.mode == Mode::Jtag => {
                let role = self.chain.role();
                self.chain.clock(swdio, tdi, tdo, port);
                (role, None)
            }
            _ => {
                if swj == Some(Swj::LineReset) {
                    self.link.line_reset();
                }
                let role = self.link.role(swdio);
                (role, self.link.clock(swdio, port))
            }
        };
        Clocked { role, swj, event }
    }
}
