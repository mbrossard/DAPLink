//! The observer: an edge trace in, framing and transactions out, on the target's own machines.

use std::ops::Range;

use crate::adi::jtag_dp::{
    self, decode_ack, Instruction, JtagAck, Pipeline, Protocol, ReadResult, Xpacc, XPACC_LEN,
};
use crate::adi::regs::{DP_CTRL_STAT, DP_SELECT};
use crate::target::Transaction;
use crate::wire::jtag::{Chain, Registers, Scan, Tap, TapState};
use crate::wire::swd::{self, Link, LinkConfig, Request};
use crate::wire::swj::{Mode, Swj};
use crate::wire::{CellRole, Drive, Interface, Line, Lines, Phase, TraceEvent};

#[cfg(all(test, feature = "differential"))]
mod roundtrip;
#[cfg(test)]
mod tests;

/// How the observer frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Framing {
    /// As the target frames from power-up: SWD out of step until a line reset, TAPs in reset.
    Strict,
    /// From an unknown start, and again from the next cell wherever a header is malformed.
    Resync,
}

/// One TAP as the observer knows it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TapConfig {
    /// IR length.
    pub ir_len: u8,
    /// The IDCODE instruction, which Test-Logic-Reset loads; BYPASS without one.
    pub idcode: Option<u32>,
    /// The data register length behind every other instruction but BYPASS, when known.
    pub other: Option<u8>,
}

impl TapConfig {
    /// A JTAG-DP TAP of IR length `ir_len`.
    pub fn dp(ir_len: u8) -> Self {
        Self {
            ir_len,
            idcode: Some(jtag_dp::idcode_ir(ir_len)),
            other: None,
        }
    }
}

/// What the observer is told about the wire.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Config {
    /// Strict or resynchronizing.
    pub framing: Framing,
    /// The protocol at the trace's start.
    pub mode: Mode,
    /// The SWD link's DP features.
    pub link: LinkConfig,
    /// Follow CTRL/STAT.ORUNDETECT from the writes seen.
    pub orundetect: bool,
    /// The chain, the TDO end first.
    pub taps: Vec<TapConfig>,
    /// Index of the JTAG-DP in `taps`.
    pub dp: usize,
    /// The JTAG-DP protocol, or `Unknown` to learn it.
    pub protocol: Protocol,
    /// Packets that confirm an SWD header found while resynchronizing.
    pub confirm: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            framing: Framing::Resync,
            mode: Mode::Swd,
            link: LinkConfig::default(),
            orundetect: true,
            taps: vec![TapConfig::dp(4)],
            dp: 0,
            protocol: Protocol::Unknown,
            confirm: 2,
        }
    }
}

/// Why something on the wire was not observed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Why {
    /// Before the observer knew the framing or the TAP state.
    Unsynced,
    /// The trace ends inside it, or the scan was too short to carry all of the access.
    Truncated,
    /// The JTAG-DP's instructions were not known.
    Instruction,
    /// A data register of unknown length sits in the chain.
    Length,
    /// The xPACC scan ended before its ACK: whether it executed is not known.
    ShortScan,
    /// Whether the captured ReadResult is valid could not be established.
    Result,
}

/// Something the observer could not establish.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unobserved {
    /// The cells concerned.
    pub cells: Range<usize>,
    /// The transaction concerned, if one was reported.
    pub transaction: Option<usize>,
    /// Why.
    pub why: Why,
}

/// A clock pulse too short to be a cell, placed before the cell that followed it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Glitch {
    /// Its stamp in the trace.
    pub stamp: u64,
    /// The index of the next cell.
    pub cell: usize,
    /// That cell's role, when known.
    pub role: Option<CellRole>,
}

/// What the observer made of a trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Observed {
    /// One role per cell; `None` where the observer was not synchronized.
    pub roles: Vec<Option<CellRole>>,
    /// The transactions, as the target logs them.
    pub transactions: Vec<Transaction>,
    /// The cell that completed each transaction.
    pub at: Vec<usize>,
    /// What could not be established, in order.
    pub unobserved: Vec<Unobserved>,
    /// The glitches.
    pub glitches: Vec<Glitch>,
    /// The cell index after each `Gap` event.
    pub breaks: Vec<usize>,
    /// The JTAG-DP protocol at the end: given, or learned.
    pub protocol: Protocol,
}

fn parity(v: u32) -> bool {
    v.count_ones() & 1 != 0
}

/// The state the observer tracks beyond the machines'.
#[derive(Clone, Debug)]
struct Track {
    orundetect: bool,
    select: u32,
    pipeline: Pipeline,
    protocol: Protocol,
    /// The pipeline's ReadResult validity follows from what was seen.
    sure: bool,
    irs_known: bool,
    /// The scan in progress began at a capture the observer saw.
    captured: bool,
    /// Under unknown TAP state: the chain's callbacks mean nothing.
    blind: bool,
    pending: Option<Instruction>,
    irs_unknown_at_capture: bool,
    before: Option<usize>,
    after_known: bool,
    /// Cells a scan needs to fill the JTAG-DP's register from TDI alone.
    fill: usize,
    scan_len: usize,
    dp_bits: Option<(usize, u64)>,
    scan_start: usize,
}

#[derive(Default)]
struct Out {
    transactions: Vec<Transaction>,
    at: Vec<usize>,
    unobserved: Vec<Unobserved>,
}

impl Out {
    fn push(&mut self, at: usize, t: Transaction) {
        self.transactions.push(t);
        self.at.push(at);
    }

    fn gap(&mut self, cells: Range<usize>, transaction: Option<usize>, why: Why) {
        self.unobserved.push(Unobserved {
            cells,
            transaction,
            why,
        });
    }
}

struct Port<'a> {
    cells: &'a [(Lines, Lines)],
    cfg: &'a Config,
    at: usize,
    track: Track,
    out: Out,
}

impl Port<'_> {
    fn target(&self, i: usize) -> bool {
        self.cells.get(i).is_none_or(|c| c.1.get(Line::Swdio))
    }

    /// TAP `tap`'s register under `ir`, and its length if known.
    fn register(&self, tap: usize, ir: u32) -> (Instruction, Option<u8>) {
        let t = self.cfg.taps[tap];
        let ones = (1u32 << t.ir_len) - 1;
        let reg = if tap == self.cfg.dp {
            jtag_dp::instruction(ir, t.ir_len)
        } else if ir == ones {
            Instruction::Bypass
        } else if Some(ir) == t.idcode {
            Instruction::Idcode
        } else {
            Instruction::Boundary
        };
        let len = match reg {
            Instruction::Abort | Instruction::Dpacc | Instruction::Apacc => Some(XPACC_LEN),
            Instruction::Idcode => Some(32),
            Instruction::Bypass => Some(1),
            Instruction::Boundary => t.other,
        };
        (reg, len)
    }

    /// Test-Logic-Reset as the target takes it.
    fn reset_state(&mut self) {
        self.track.pipeline.forget(jtag_dp::Unknown::Reset);
        self.track.irs_known = true;
        self.track.sure = true;
    }

    fn xpacc(&mut self, access: Instruction, value: u64) {
        let (at, start) = (self.at, self.track.scan_start);
        let t = &mut self.track;
        let Some((avail, bits)) = t.dp_bits.take() else {
            t.sure = false;
            self.out.gap(start..at + 1, None, Why::Length);
            return;
        };
        if avail < 3 {
            t.sure = false;
            self.out.gap(start..at + 1, None, Why::ShortScan);
            return;
        }
        let ack_bits = (bits & 7) as u8;
        t.protocol = t.protocol.learn(ack_bits);
        let ack = decode_ack(ack_bits, t.protocol);
        t.pipeline.acked(ack);
        let result = match t.pipeline.result() {
            ReadResult::Valid(_) => ReadResult::Valid((bits >> 3) as u32),
            unknown => unknown,
        };
        let ambiguous = t.protocol == Protocol::Unknown && ack == JtagAck::OkFault;
        let short = t.scan_len < t.fill || avail < usize::from(XPACC_LEN);
        let k = self.out.transactions.len();
        if !t.sure || ambiguous || matches!(ack, JtagAck::Invalid(_)) {
            self.out.gap(start..at + 1, Some(k), Why::Result);
        }
        if short {
            self.out.gap(start..at + 1, Some(k), Why::Truncated);
        }
        let read = value & 1 != 0;
        let executed = ack != JtagAck::Wait;
        self.out.push(
            at,
            Transaction::Xpacc(Xpacc {
                tap: self.cfg.dp,
                access,
                read,
                a: (value >> 1 & 3) as u8,
                data: (value >> 3) as u32,
                ack,
                result,
                executed,
            }),
        );
        let t = &mut self.track;
        if executed {
            t.pipeline.execute(access, read);
            t.sure = !matches!(ack, JtagAck::Invalid(_)) && t.scan_len >= t.fill;
        }
    }
}

impl swd::Port for Port<'_> {
    fn ack(&mut self, _req: Request) -> u8 {
        (0..3).fold(0, |a, k| a | u8::from(self.target(self.at + 2 + k)) << k)
    }

    fn read(&mut self, _req: Request) -> (u32, bool) {
        let data = (0..32).fold(0, |v, k| v | u32::from(self.target(self.at + 1 + k)) << k);
        (data, self.target(self.at + 33) != parity(data))
    }

    fn write(&mut self, req: Request, data: u32, parity_ok: bool) {
        let t = &mut self.track;
        if req.ap || !parity_ok {
            return;
        }
        match req.a {
            DP_CTRL_STAT if t.select & 0xF == 0 => {
                t.orundetect = self.cfg.orundetect && data & 1 != 0;
            }
            DP_SELECT => t.select = data,
            _ => {}
        }
    }

    fn orundetect(&self) -> bool {
        self.track.orundetect
    }
}

impl Registers for Port<'_> {
    fn capture(&mut self, tap: usize, ir: u32) -> (u64, u8) {
        let (reg, len) = self.register(tap, ir);
        let dp = self.cfg.dp;
        let t = &mut self.track;
        if t.blind {
            return (0, len.unwrap_or(1));
        }
        if tap == 0 {
            (t.before, t.after_known, t.pending, t.fill) = (Some(0), true, None, 0);
            t.irs_unknown_at_capture = !t.irs_known;
        }
        if tap >= dp {
            t.fill += usize::from(len.unwrap_or(1));
        }
        if tap < dp {
            t.before = t.before.zip(len).map(|(b, l)| b + usize::from(l));
        } else if tap > dp {
            t.after_known &= len.is_some();
        } else if matches!(reg, Instruction::Dpacc | Instruction::Apacc) {
            t.pending = Some(reg);
        }
        (0, len.unwrap_or(1))
    }

    fn update(&mut self, tap: usize, _ir: u32, value: u64) {
        let t = &mut self.track;
        if t.blind || tap != self.cfg.dp {
            return;
        }
        if t.irs_unknown_at_capture {
            t.sure = false;
            let start = t.scan_start;
            self.out.gap(start..self.at + 1, None, Why::Instruction);
            return;
        }
        if !t.captured {
            t.sure = false;
            return;
        }
        if let Some(access) = t.pending.take() {
            if !t.after_known {
                t.dp_bits = None;
            }
            self.xpacc(access, value);
        }
    }

    fn update_ir(&mut self, irs: &[u32]) {
        if self.track.blind {
            return;
        }
        let dp = self.cfg.dp;
        let kept = irs.get(dp).is_none_or(|&ir| {
            matches!(
                self.register(dp, ir).0,
                Instruction::Dpacc | Instruction::Apacc | Instruction::Bypass
            )
        });
        let t = &mut self.track;
        t.irs_known = t.captured;
        if !t.irs_known {
            t.sure = false;
        } else if !kept {
            t.pipeline.forget(jtag_dp::Unknown::Instruction);
            t.sure = true;
        }
    }

    fn reset(&mut self) {
        if !self.track.blind {
            self.reset_state();
        }
    }

    fn scanned(&mut self, scan: Scan) {
        let t = &mut self.track;
        if t.blind {
            return;
        }
        if !t.captured {
            let start = t.scan_start;
            self.out.gap(start..self.at + 1, None, Why::Unsynced);
            return;
        }
        t.dp_bits = None;
        t.scan_len = scan.tdi.len();
        if !scan.ir && t.pending.is_some() {
            if let Some(off) = t.before {
                let avail = scan
                    .tdo
                    .len()
                    .saturating_sub(off)
                    .min(usize::from(XPACC_LEN));
                let bits = (0..avail).fold(0u64, |v, k| v | u64::from(scan.tdo[off + k]) << k);
                t.dp_bits = Some((avail, bits));
            }
        }
        self.out.push(self.at, Transaction::Scan(scan));
    }
}

/// Every TAP state, as a bit set.
const ALL_STATES: u16 = u16::MAX;

const STATES: [TapState; 16] = [
    TapState::Reset,
    TapState::Idle,
    TapState::SelectDr,
    TapState::CaptureDr,
    TapState::ShiftDr,
    TapState::Exit1Dr,
    TapState::PauseDr,
    TapState::Exit2Dr,
    TapState::UpdateDr,
    TapState::SelectIr,
    TapState::CaptureIr,
    TapState::ShiftIr,
    TapState::Exit1Ir,
    TapState::PauseIr,
    TapState::Exit2Ir,
    TapState::UpdateIr,
];

fn next_states(set: u16, tms: bool) -> u16 {
    STATES
        .iter()
        .enumerate()
        .filter(|&(i, _)| set >> i & 1 != 0)
        .fold(0, |n, (_, s)| {
            let to = STATES.iter().position(|&t| t == s.next(tms)).unwrap_or(0);
            n | 1 << to
        })
}

/// The interface and tracked state before a cell, to return to when a header proves malformed.
struct Mark {
    cell: usize,
    wire: Interface,
    track: Track,
    out: (usize, usize),
}

impl Mark {
    fn new(cell: usize, wire: &Interface, port: &Port) -> Self {
        Self {
            cell,
            wire: wire.clone(),
            track: port.track.clone(),
            out: (port.out.transactions.len(), port.out.unobserved.len()),
        }
    }
}

/// Frame `trace` as `cfg` says.
pub fn observe(trace: &[TraceEvent], cfg: &Config) -> Observed {
    let mut cells = Vec::new();
    let mut glitches = Vec::new();
    let mut breaks = Vec::new();
    for e in trace {
        match *e {
            TraceEvent::Cell {
                host,
                target_sample,
                ..
            } => cells.push((host, target_sample)),
            TraceEvent::Glitch { stamp } => glitches.push(Glitch {
                stamp,
                cell: cells.len(),
                role: None,
            }),
            TraceEvent::Gap { .. } => breaks.push(cells.len()),
        }
    }
    let taps: Vec<Tap> = cfg
        .taps
        .iter()
        .map(|t| Tap {
            ir_len: t.ir_len,
            reset_ir: t.idcode.unwrap_or((1 << t.ir_len) - 1),
        })
        .collect();
    let mut chain = Chain::new(taps);
    chain.record(true);
    let mut wire = Interface::new(cfg.mode, Link::new(cfg.link), chain);
    let resync = cfg.framing == Framing::Resync;
    let mut tap_set = (resync && cfg.mode == Mode::Jtag).then_some(ALL_STATES);
    let mut port = Port {
        cells: &cells,
        cfg,
        at: 0,
        track: Track {
            orundetect: false,
            select: 0,
            pipeline: Pipeline::default(),
            protocol: cfg.protocol,
            sure: !resync,
            irs_known: !resync,
            captured: false,
            blind: tap_set.is_some(),
            pending: None,
            irs_unknown_at_capture: false,
            before: None,
            after_known: false,
            fill: 0,
            scan_len: 0,
            dp_bits: None,
            scan_start: 0,
        },
        out: Out::default(),
    };
    if !resync {
        port.reset_state();
    }
    let mut roles: Vec<Option<CellRole>> = Vec::with_capacity(cells.len());
    let (mut start_mark, mut resume_mark): (Option<Mark>, Option<Mark>) = (None, None);
    let (mut between, mut last_fail) = (false, None);
    let mut no_trial = None;
    // Roles from before a rewind, kept for cells no resynchronized header claims.
    let mut kept: (usize, Vec<Option<CellRole>>) = (0, Vec::new());
    let mut packet_start = 0;
    let mut i = 0;
    while i < cells.len() {
        let (host, target) = cells[i];
        let swd = wire.mode == Mode::Swd;
        let mut lost = swd && wire.link.state() == swd::State::Lockout;
        let try_here = resync && lost && host.get(Line::Swdio) && no_trial != Some(i);
        if try_here && trial(&cells, cfg, i, &wire, &port.track) {
            wire.link.resume();
            lost = false;
        }
        let target_cell = swd && wire.link.role(host.get(Line::Swdio)).driver == Drive::Target;
        let swdio = if target_cell { target } else { host }.get(Line::Swdio);
        let state = wire.link.state();
        if swd && state == swd::State::Idle && swdio {
            packet_start = i;
            if resync {
                start_mark = Some(Mark::new(i, &wire, &port));
            }
        }
        if resync && swd {
            // A line reset's ones hold no Start bit to resynchronize on.
            let now = matches!(state, swd::State::Turn | swd::State::Idle);
            if now && !between {
                resume_mark = Some(Mark::new(i, &wire, &port));
            }
            between = now;
        }
        port.at = i;
        let blind = !swd && tap_set.is_some();
        port.track.blind = blind;
        let tdo = target.get(Line::Tdo);
        let c = wire.clock(swdio, host.get(Line::Tdi), Some(tdo), &mut port);
        let mut known = !(resync && lost) && !blind;
        match c.swj {
            Some(Swj::Switch(Mode::Jtag)) => {
                if blind {
                    port.track.blind = false;
                    port.reset_state();
                }
                tap_set = None;
                port.track.captured = false;
                known = true;
            }
            Some(_) => known = true,
            None => {}
        }
        if !swd && c.swj.is_none() {
            if let Some(set) = tap_set {
                let set = next_states(set, swdio);
                if set.count_ones() == 1 {
                    wire.chain.sync(STATES[set.trailing_zeros() as usize]);
                    port.track.captured = false;
                    port.track.sure = false;
                    tap_set = None;
                } else {
                    tap_set = Some(set);
                }
            } else if let Phase::Tap(TapState::CaptureDr | TapState::CaptureIr) = c.role.phase {
                port.track.captured = true;
                port.track.scan_start = i;
            }
        }
        match c.event {
            Some(swd::Event::Done(t)) => port.out.push(i, Transaction::Swd(t)),
            Some(swd::Event::Targetsel(v, ok)) => port.out.push(i, Transaction::Targetsel(v, ok)),
            Some(swd::Event::Malformed) if resync => {
                let s = packet_start;
                let back = last_fail.is_none_or(|f| s > f);
                last_fail = last_fail.max(Some(s));
                let m = if back { resume_mark.take() } else { None };
                if let Some(m) = m.or(start_mark.take()) {
                    (between, resume_mark, start_mark) = (false, None, None);
                    wire = m.wire;
                    wire.link.lockout();
                    port.track = m.track;
                    port.out.transactions.truncate(m.out.0);
                    port.out.at.truncate(m.out.0);
                    port.out.unobserved.truncate(m.out.1);
                    kept = (m.cell, roles.split_off(m.cell));
                    kept.1.truncate(s.saturating_sub(m.cell));
                    no_trial = Some(s);
                    i = m.cell;
                    continue;
                }
            }
            Some(swd::Event::Malformed) | None => {}
        }
        let earlier = i
            .checked_sub(kept.0)
            .and_then(|k| kept.1.get(k).copied().flatten());
        roles.push(known.then_some(c.role).or(earlier));
        i += 1;
    }
    let n = cells.len();
    let mid_packet = !matches!(
        wire.link.state(),
        swd::State::Idle | swd::State::Reset | swd::State::Lockout | swd::State::Turn
    );
    let mid_scan = !matches!(
        wire.chain.state(),
        TapState::Reset
            | TapState::Idle
            | TapState::SelectDr
            | TapState::SelectIr
            | TapState::UpdateDr
            | TapState::UpdateIr
    );
    if wire.mode == Mode::Swd && mid_packet && roles.get(packet_start).is_some_and(Option::is_some)
    {
        port.out.gap(packet_start..n, None, Why::Truncated);
    }
    if wire.mode == Mode::Jtag && tap_set.is_none() && mid_scan && port.track.captured {
        port.out.gap(port.track.scan_start..n, None, Why::Truncated);
    }
    let mut unobserved = port.out.unobserved;
    let mut k = 0;
    while k < n {
        if roles[k].is_none() {
            let start = k;
            while k < n && roles[k].is_none() {
                k += 1;
            }
            unobserved.push(Unobserved {
                cells: start..k,
                transaction: None,
                why: Why::Unsynced,
            });
        }
        k += 1;
    }
    unobserved.sort_by_key(|u| u.cells.start);
    for g in &mut glitches {
        g.role = roles.get(g.cell).copied().flatten();
    }
    Observed {
        roles,
        transactions: port.out.transactions,
        at: port.out.at,
        unobserved,
        glitches,
        breaks,
        protocol: port.track.protocol,
    }
}

/// Whether an SWD header starting at `start` frames `cfg.confirm` well-formed packets.
fn trial(
    cells: &[(Lines, Lines)],
    cfg: &Config,
    start: usize,
    wire: &Interface,
    track: &Track,
) -> bool {
    let mut wire = wire.clone();
    wire.link.resume();
    let mut port = Port {
        cells,
        cfg,
        at: start,
        track: track.clone(),
        out: Out::default(),
    };
    let mut good = 0;
    for (i, &(host, target)) in cells.iter().enumerate().skip(start) {
        if wire.mode != Mode::Swd {
            return good > 0;
        }
        let target_cell = wire.link.role(host.get(Line::Swdio)).driver == Drive::Target;
        let swdio = if target_cell { target } else { host }.get(Line::Swdio);
        port.at = i;
        let c = wire.clock(swdio, host.get(Line::Tdi), None, &mut port);
        if c.swj.is_some() {
            return good > 0;
        }
        match c.event {
            Some(swd::Event::Malformed) => return false,
            Some(swd::Event::Done(t)) => {
                let ack_ok = matches!(t.ack, swd::ACK_OK | swd::ACK_WAIT | swd::ACK_FAULT);
                if !ack_ok || !t.parity_ok && t.req.read {
                    return false;
                }
                good += 1;
            }
            Some(swd::Event::Targetsel(..)) => good += 1,
            None => {}
        }
        if good >= cfg.confirm {
            return true;
        }
    }
    good > 0
}
