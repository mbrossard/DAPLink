//! SWD packets: the request header, ACKs, and the target side's framing state machine.

use super::{CellRole, Class, Drive, Phase};

/// OK, `ACK[0]` first on the wire.
pub const ACK_OK: u8 = 0b001;
/// WAIT.
pub const ACK_WAIT: u8 = 0b010;
/// FAULT.
pub const ACK_FAULT: u8 = 0b100;

/// A well-formed request header's fields.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Request {
    /// APnDP.
    pub ap: bool,
    /// RnW.
    pub read: bool,
    /// `A[3:2]`.
    pub a: u8,
}

impl Request {
    /// The header byte, Start in bit 0; `None` when Start, parity, Stop or Park is wrong.
    pub fn parse(v: u8) -> Option<Request> {
        let body = (v >> 1) & 0xF;
        let valid = v & 1 == 1
            && (v >> 5 & 1) == (body.count_ones() as u8 & 1)
            && v >> 6 & 1 == 0
            && v >> 7 == 1;
        valid.then_some(Request {
            ap: v >> 1 & 1 != 0,
            read: v >> 2 & 1 != 0,
            a: v >> 3 & 3,
        })
    }
}

/// The link's position within a packet; a count is the next bit's index.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Out of step until the next line reset.
    Lockout,
    /// Within a line reset's high cells.
    Reset,
    /// Between packets.
    Idle,
    /// Request bits so far, Start in bit 0.
    Header(u8, u8),
    /// The turnaround before the ACK.
    AckTrn,
    /// Driving the ACK.
    Ack(u8),
    /// Driving read data.
    Read(u8),
    /// Driving read parity.
    ReadParity,
    /// The turnaround after a read or a non-OK ACK.
    Turn,
    /// The turnaround before write data.
    WriteTrn,
    /// Write data bits so far.
    Write(u8, u32),
    /// Write data, awaiting its parity.
    WriteParity(u32),
    /// The undriven data phase of a WAIT or FAULT read under ORUNDETECT: cells so far.
    Skip(u8),
    /// The undriven response cells of a TARGETSEL write: cells so far.
    Select(u8),
}

/// What a link knows of its DP beyond the packet grammar.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LinkConfig {
    /// A DP write to `0xC` straight after a line reset is TARGETSEL.
    pub multidrop: bool,
}

/// The target behind a link.
pub trait Port {
    /// The ACK for a well-formed request.
    fn ack(&mut self, req: Request) -> u8;
    /// An OK read's data, and whether its parity goes out inverted.
    fn read(&mut self, req: Request) -> (u32, bool);
    /// An OK write's data phase, and whether its parity matched.
    fn write(&mut self, req: Request, data: u32, parity_ok: bool);
    /// CTRL/STAT.ORUNDETECT: a WAIT or FAULT still has a data phase.
    fn orundetect(&self) -> bool {
        false
    }
}

/// One packet the link finished.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transfer {
    /// The request.
    pub req: Request,
    /// The ACK the target sent.
    pub ack: u8,
    /// The data phase, when there was one.
    pub data: Option<u32>,
    /// Whether the data phase's parity was correct.
    pub parity_ok: bool,
}

/// What a cell completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// A malformed header: the link is locked out.
    Malformed,
    /// A packet.
    Done(Transfer),
    /// A TARGETSEL write: its data, and whether its parity matched.
    Targetsel(u32, bool),
}

/// The target side of an SWD link: framing, turnaround and lockout.
#[derive(Clone, Debug)]
pub struct Link {
    config: LinkConfig,
    state: State,
    req: Request,
    ack: u8,
    data: u32,
    bad: bool,
    out: Option<bool>,
    fresh: bool,
    ignored: bool,
    targetsel: bool,
}

impl Default for Link {
    fn default() -> Self {
        Self::new(LinkConfig::default())
    }
}

fn parity(v: u32) -> bool {
    v.count_ones() & 1 != 0
}

impl Link {
    /// A link out of step until its first line reset.
    pub fn new(config: LinkConfig) -> Self {
        Self {
            config,
            state: State::Lockout,
            req: Request::default(),
            ack: 0,
            data: 0,
            bad: false,
            out: None,
            fresh: false,
            ignored: false,
            targetsel: false,
        }
    }

    /// The link's position.
    pub fn state(&self) -> State {
        self.state
    }

    /// What the target drives on SWDIO.
    pub fn drive(&self) -> Option<bool> {
        self.out
    }

    /// A line reset completed.
    pub fn line_reset(&mut self) {
        self.state = State::Reset;
        self.out = None;
        self.fresh = true;
    }

    /// Between packets, as after a line reset's idle cells.
    pub fn resume(&mut self) {
        self.state = State::Idle;
        self.out = None;
        self.fresh = false;
    }

    /// A protocol switch: out of step until the next line reset.
    pub fn lockout(&mut self) {
        self.state = State::Lockout;
        self.out = None;
    }

    /// What the next cell is, clocked with SWDIO at `bit`.
    pub fn role(&self, bit: bool) -> CellRole {
        let (phase, driver, at) = match self.state {
            State::Lockout => (Phase::Lockout, Drive::Host, None),
            State::Reset if bit => (Phase::LineReset, Drive::Host, None),
            State::Idle if bit => (Phase::Request, Drive::Host, Some(0)),
            State::Reset | State::Idle => (Phase::Idle, Drive::Host, None),
            State::Header(n, _) => (Phase::Request, Drive::Host, Some(n)),
            State::AckTrn | State::Turn | State::WriteTrn | State::Select(0 | 4) => {
                (Phase::Turnaround, Drive::Released, None)
            }
            State::Select(k) => (Phase::Ack, Drive::Released, Some(k - 1)),
            State::Skip(k) if k < 32 => (Phase::ReadData, Drive::Released, Some(k)),
            State::Skip(_) => (Phase::ReadParity, Drive::Released, None),
            State::Ack(k) => (Phase::Ack, Drive::Target, Some(k - 1)),
            State::Read(k) => (Phase::ReadData, Drive::Target, Some(k - 1)),
            State::ReadParity => (Phase::ReadParity, Drive::Target, None),
            State::Write(k, _) => (Phase::WriteData, Drive::Host, Some(k)),
            State::WriteParity(_) => (Phase::WriteParity, Drive::Host, None),
        };
        CellRole {
            phase,
            driver,
            bit: at.map(u16::from),
            class: self.class(phase),
        }
    }

    fn class(&self, phase: Phase) -> Class {
        let answered = matches!(self.ack, ACK_OK | ACK_WAIT | ACK_FAULT);
        match self.state {
            _ if phase == Phase::Request => Class::Request,
            State::Idle if !self.fresh && !self.targetsel && answered => Class::Idle,
            State::AckTrn | State::Ack(_) | State::Turn if !answered => Class::Resident,
            State::AckTrn => Class::Turnaround,
            State::Ack(_) => Class::Ack,
            State::Turn if self.req.read && (self.ack == ACK_OK || self.ignored) => {
                Class::TrailingTurnaround
            }
            State::Turn => Class::WaitTurnaround,
            State::Read(_) => Class::ReadData,
            State::ReadParity => Class::ReadParity,
            State::Skip(_) => Class::WaitData,
            State::WriteTrn => Class::WriteTurnaround,
            State::Write(..) | State::WriteParity(_) if self.targetsel => Class::Resident,
            State::Write(..) => Class::WriteData,
            State::WriteParity(_) => Class::WriteParity,
            _ => Class::Resident,
        }
    }

    /// One rising edge with SWDIO at `bit`.
    pub fn clock(&mut self, bit: bool, port: &mut impl Port) -> Option<Event> {
        let mut event = None;
        let done = |s: &Self, data, parity_ok| {
            Some(Event::Done(Transfer {
                req: s.req,
                ack: s.ack,
                data,
                parity_ok,
            }))
        };
        self.state = match self.state {
            State::Lockout => State::Lockout,
            State::Reset if bit => State::Reset,
            State::Reset | State::Turn => State::Idle,
            State::Idle if bit => State::Header(1, 1),
            State::Idle => State::Idle,
            State::Header(n, v) => {
                let v = v | (u8::from(bit) << n);
                if n < 7 {
                    State::Header(n + 1, v)
                } else if let Some(req) = Request::parse(v) {
                    let fresh = std::mem::take(&mut self.fresh);
                    self.req = req;
                    self.ignored = false;
                    self.targetsel =
                        fresh && self.config.multidrop && !req.ap && !req.read && req.a == 3;
                    if self.targetsel {
                        State::Select(0)
                    } else {
                        self.ack = port.ack(req);
                        State::AckTrn
                    }
                } else {
                    event = Some(Event::Malformed);
                    State::Lockout
                }
            }
            State::AckTrn => {
                self.out = Some(self.ack & 1 != 0);
                State::Ack(1)
            }
            State::Ack(k) if k < 3 => {
                self.out = Some(self.ack >> k & 1 != 0);
                State::Ack(k + 1)
            }
            State::Ack(_) if self.ack != ACK_OK && self.overrun(port) => {
                self.out = None;
                self.ignored = true;
                if self.req.read {
                    State::Skip(0)
                } else {
                    State::WriteTrn
                }
            }
            State::Ack(_) if self.ack != ACK_OK => {
                self.out = None;
                event = done(self, None, true);
                State::Turn
            }
            State::Ack(_) if self.req.read => {
                (self.data, self.bad) = port.read(self.req);
                self.out = Some(self.data & 1 != 0);
                State::Read(1)
            }
            State::Ack(_) => {
                self.out = None;
                State::WriteTrn
            }
            State::Read(k) if k < 32 => {
                self.out = Some(self.data >> k & 1 != 0);
                State::Read(k + 1)
            }
            State::Read(_) => {
                self.out = Some(parity(self.data) ^ self.bad);
                State::ReadParity
            }
            State::ReadParity => {
                self.out = None;
                event = done(self, Some(self.data), !self.bad);
                State::Turn
            }
            State::Skip(k) if k < 32 => State::Skip(k + 1),
            State::Skip(_) => {
                event = done(self, None, true);
                State::Turn
            }
            State::Select(k) if k < 4 => State::Select(k + 1),
            State::Select(_) => State::Write(0, 0),
            State::WriteTrn => State::Write(0, 0),
            State::Write(k, v) => {
                let v = v | (u32::from(bit) << k);
                if k < 31 {
                    State::Write(k + 1, v)
                } else {
                    State::WriteParity(v)
                }
            }
            State::WriteParity(v) => {
                let ok = parity(v) == bit;
                event = if self.targetsel {
                    Some(Event::Targetsel(v, ok))
                } else {
                    if !self.ignored {
                        port.write(self.req, v, ok);
                    }
                    done(self, Some(v), ok)
                };
                State::Idle
            }
        };
        event
    }

    fn overrun(&self, port: &impl Port) -> bool {
        matches!(self.ack, ACK_WAIT | ACK_FAULT) && port.orundetect()
    }
}
