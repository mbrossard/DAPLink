//! The simulated target: an ADIv5 or ADIv6 DP and MEM-AP over SWD or a JTAG chain, via the pads.

use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

mod log;

use crate::adi::dp::Dp;
use crate::adi::jtag_dp::{
    self, Instruction, JtagAck, Pipeline, Protocol, ReadResult, Unknown, Xpacc, XPACC_LEN,
};
use crate::adi::mem_ap::MemAp;
use crate::wire::jtag::{Chain, Registers, Scan, Tap, TapState};
use crate::wire::swd::{self, Link, Request};
use crate::wire::swj::Swj;
use crate::wire::{Drives, Interface, Lines, TraceEvent};

pub use log::{Cell, Log, Transaction};

pub use crate::adi::jtag_dp::DpVersion;
pub use crate::adi::mem_ap::FAULT_BASE;
pub use crate::wire::swj::Mode;
pub use crate::wire::Line;

/// The target's DPIDR.
pub const DPIDR: u32 = 0x2BA0_1477;
/// An ADIv6 target's DPIDR: DPv3, the CoreSight SoC-600 DP's.
pub const DPIDR_V3: u32 = 0x4C01_3477;
/// The ADI TAP's IDCODE.
pub const ADI_IDCODE: u32 = 0x4BA0_0477;
/// A DPv1 target's ADI TAP IDCODE: PARTNO `0xBA04`, the Cortex-M33's DPv1 JTAG-DP.
pub const ADI_IDCODE_V1: u32 = 0x0BA0_4477;
/// An ADIv6 target's ADI TAP IDCODE.
pub const ADI_IDCODE_V6: u32 = 0x4BA0_6477;
/// Any other TAP's IDCODE.
pub const OTHER_IDCODE: u32 = 0x0641_3041;
/// Length of the boundary-scan register an undecoded instruction selects.
const BOUNDARY_LEN: u8 = 21;

/// A simulator shared between the pads, ports and wires that drive it.
pub type Shared = Rc<RefCell<Sim>>;

/// A fresh simulator for `target`, shared.
pub fn shared(target: &Target) -> Shared {
    Rc::new(RefCell::new(Sim::new(target)))
}

/// One probe pad: its GPIO latch and direction, and whether a peripheral owns it instead.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Pad {
    /// The GPIO drives the latch onto the pad.
    pub output: bool,
    /// The GPIO output latch.
    pub latch: bool,
    /// A peripheral (the SPI block) owns the pad.
    pub peripheral: bool,
}

/// Something the target observed that changed or exposed its state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Effect {
    /// An SWJ select sequence switched the protocol.
    Mode(Mode),
    /// Fifty or more high SWDIO cycles reset the SWD interface.
    LineReset,
    /// A malformed SWD header locked the interface out until the next line reset.
    Protocol,
    /// The probe drove SWDIO in a cell the target drove.
    Contention,
    /// A write data phase arrived with bad parity and was dropped.
    WriteParity,
    /// A DP register write other than ABORT.
    Dp {
        /// Register address.
        addr: u8,
        /// Written value.
        data: u32,
    },
    /// An ABORT write.
    Abort(u32),
    /// An AP register read.
    ApRead {
        /// APSEL.
        apsel: u8,
        /// Register address.
        addr: u8,
        /// Value read.
        value: u32,
    },
    /// An AP register write.
    ApWrite {
        /// APSEL.
        apsel: u8,
        /// Register address.
        addr: u8,
        /// Value written.
        data: u32,
    },
    /// A memory access faulted and set STICKYERR.
    Sticky,
    /// The nRESET level the target sees changed.
    Reset(bool),
}

/// How a target is built: its power-up protocol, JTAG chain and scripted behaviour.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Target {
    /// Protocol at power-up.
    pub mode: Mode,
    /// IR lengths of the chain, the TDO end first.
    pub chain: Vec<u8>,
    /// Index of the ADI TAP in `chain`.
    pub adi: usize,
    /// WAIT responses owed after each AP access, cycled.
    pub waits: Vec<u8>,
    /// Read data carrying this value is sent with bad parity.
    pub poison: Option<u32>,
    /// Seeds the untouched memory contents.
    pub seed: u64,
    /// An ADIv6 DP: DPv3, and JTAG DP protocol 1.
    pub adiv6: bool,
    /// The JTAG-DP's DP version when not `adiv6`.
    pub jtag_version: DpVersion,
    /// No TAP answers: `chain` is only what the host configures, and TDO floats high.
    pub absent: bool,
}

impl Default for Target {
    fn default() -> Self {
        Self {
            mode: Mode::Swd,
            chain: vec![4],
            adi: 0,
            waits: Vec::new(),
            poison: None,
            seed: 0,
            adiv6: false,
            jtag_version: DpVersion::V0,
            absent: false,
        }
    }
}

/// The debug port, its MEM-AP and the JTAG-DP pipeline, behind one TAP of the chain.
#[derive(Clone, Debug)]
struct Adi {
    dp: Dp,
    ap: MemAp,
    pipeline: Pipeline,
    protocol: Protocol,
    poison: Option<u32>,
    /// IR lengths of the chain, and the ADI TAP's index in it.
    ir_lens: Vec<u8>,
    tap: usize,
    /// The last xPACC capture answered WAIT, so its update does nothing.
    waited: bool,
    /// The last ADI capture's ACK and ReadResult, when it was an xPACC.
    captured: Option<(JtagAck, ReadResult)>,
}

impl Adi {
    /// The register `ir` selects on `tap`; any other TAP than the ADI TAP decodes IDCODE as 1.
    fn register(&self, tap: usize, ir: u32) -> Instruction {
        let ir_len = self.ir_lens[tap];
        if tap == self.tap {
            jtag_dp::instruction(ir, ir_len)
        } else if ir == (1 << ir_len) - 1 {
            Instruction::Bypass
        } else if ir == 1 {
            Instruction::Idcode
        } else {
            Instruction::Boundary
        }
    }
}

/// The ADI model and the effect log, as the SWD link and the JTAG chain reach them.
struct Wires<'a> {
    adi: &'a mut Adi,
    fx: &'a mut Vec<Effect>,
    rates: &'a mut Vec<Option<u32>>,
    rate: Option<u32>,
    log: Option<&'a mut Vec<Transaction>>,
}

impl swd::Port for Wires<'_> {
    fn ack(&mut self, req: Request) -> u8 {
        self.rates.push(self.rate);
        self.adi.dp.swd_ack(req)
    }

    fn read(&mut self, req: Request) -> (u32, bool) {
        let adi = &mut *self.adi;
        let v = adi.dp.swd_read(req, &mut adi.ap, self.fx);
        (v, adi.poison == Some(v))
    }

    fn write(&mut self, req: Request, data: u32, parity_ok: bool) {
        let adi = &mut *self.adi;
        adi.dp.swd_write(req, data, parity_ok, &mut adi.ap, self.fx);
    }
}

impl Registers for Wires<'_> {
    fn capture(&mut self, tap: usize, ir: u32) -> (u64, u8) {
        let adi = &mut *self.adi;
        let reg = adi.register(tap, ir);
        if tap == adi.tap {
            adi.waited = false;
            adi.captured = None;
        }
        match reg {
            Instruction::Dpacc | Instruction::Apacc => {
                let busy = adi.dp.busy > 0;
                let ack = adi.pipeline.ack(adi.protocol, busy, adi.dp.stickyerr);
                if busy {
                    adi.dp.busy -= 1;
                    adi.waited = true;
                }
                let result = adi.pipeline.result();
                adi.captured = Some((ack, result));
                // UNKNOWN reads as the complement, so a path that keeps it cannot match.
                let result = match result {
                    ReadResult::Valid(v) => v,
                    ReadResult::Unknown(_) => !adi.pipeline.held(),
                };
                (u64::from(result) << 3 | ack.bits(), XPACC_LEN)
            }
            Instruction::Abort => (0, XPACC_LEN),
            Instruction::Idcode => (
                u64::from(match (tap == adi.tap, adi.protocol) {
                    (true, Protocol::One) => ADI_IDCODE_V6,
                    (true, _) if adi.dp.jtag_version == DpVersion::V1 => ADI_IDCODE_V1,
                    (true, _) => ADI_IDCODE,
                    _ => OTHER_IDCODE,
                }),
                32,
            ),
            Instruction::Bypass => (0, 1),
            Instruction::Boundary => (0xA_AAAA, BOUNDARY_LEN),
        }
    }

    fn update(&mut self, tap: usize, ir: u32, value: u64) {
        let adi = &mut *self.adi;
        if tap != adi.tap {
            return;
        }
        let reg = adi.register(tap, ir);
        let (read, a, data) = (value & 1 != 0, (value >> 1 & 3) as u8, (value >> 3) as u32);
        if let (Some(log), Some((ack, result))) = (self.log.as_deref_mut(), adi.captured) {
            log.push(Transaction::Xpacc(Xpacc {
                tap,
                access: reg,
                read,
                a,
                data,
                ack,
                result,
                executed: !adi.waited,
            }));
        }
        if adi.waited {
            return;
        }
        if matches!(reg, Instruction::Dpacc | Instruction::Apacc) {
            adi.pipeline.execute(reg, read);
        }
        let fx = &mut *self.fx;
        match reg {
            Instruction::Abort => adi.dp.jtag_abort(data, fx),
            Instruction::Dpacc if read => adi.pipeline.set(adi.dp.read(a, true)),
            Instruction::Dpacc => adi.dp.write(a, data, true, fx),
            Instruction::Apacc if adi.dp.stickyerr => adi.pipeline.set(0),
            Instruction::Apacc => {
                let v = adi.dp.ap(&mut adi.ap, read, a, data, fx);
                if read {
                    adi.pipeline.set(v);
                }
            }
            _ => {}
        }
    }

    fn update_ir(&mut self, irs: &[u32]) {
        let adi = &mut *self.adi;
        // Another instruction than DPACC, APACC or BYPASS forgets the last scan and result.
        let kept = irs.get(adi.tap).is_none_or(|&ir| {
            matches!(
                adi.register(adi.tap, ir),
                Instruction::Dpacc | Instruction::Apacc | Instruction::Bypass
            )
        });
        if !kept {
            adi.pipeline.forget(Unknown::Instruction);
        }
    }

    fn reset(&mut self) {
        self.adi.pipeline.forget(Unknown::Reset);
    }

    fn scanned(&mut self, scan: Scan) {
        if let Some(log) = self.log.as_deref_mut() {
            log.push(Transaction::Scan(scan));
        }
    }
}

/// The target's protocol position: its mode, SWD packet state and TAP state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkState {
    /// The protocol the SWJ-DP is in.
    pub mode: Mode,
    /// The SWD link's position within a packet.
    pub swd: swd::State,
    /// The TAP controllers' state.
    pub tap: TapState,
}

impl fmt::Display for LinkState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?} {:?} {:?}", self.mode, self.swd, self.tap)
    }
}

/// The pads plus the target behind them.
pub struct Sim {
    pads: [Pad; 5],
    spi_clk: bool,
    spi_data: Option<bool>,
    clk: bool,
    nreset: bool,
    wire: Interface,
    adi: Adi,
    tdo_out: Option<bool>,
    recording: bool,
    log: Log,
    /// Everything the target observed, in order.
    pub effects: Vec<Effect>,
    /// A clock label the driver stamps on the packets it sends.
    pub rate: Option<u32>,
    /// The label in force at each SWD header the target accepted.
    pub rates: Vec<Option<u32>>,
    /// Cells between SWD packets with the probe driving SWDIO low: its idle cycles.
    pub idle: u32,
}

impl Sim {
    /// A powered-up target with every pad released.
    pub fn new(target: &Target) -> Self {
        let ir_lens: Vec<u8> = if target.absent {
            Vec::new()
        } else {
            target.chain.iter().map(|&l| l.max(1)).collect()
        };
        let adi = Adi {
            dp: Dp {
                dpidr: if target.adiv6 { DPIDR_V3 } else { DPIDR },
                jtag_version: if target.adiv6 {
                    DpVersion::V3
                } else {
                    target.jtag_version
                },
                waits: target.waits.clone(),
                ..Dp::default()
            },
            ap: MemAp::new(target.seed),
            pipeline: Pipeline::default(),
            protocol: if target.adiv6 {
                Protocol::One
            } else {
                Protocol::Zero
            },
            poison: target.poison,
            ir_lens: ir_lens.clone(),
            tap: target.adi,
            waited: false,
            captured: None,
        };
        let taps = ir_lens
            .iter()
            .enumerate()
            .map(|(i, &ir_len)| Tap {
                ir_len,
                reset_ir: if i == target.adi {
                    jtag_dp::idcode_ir(ir_len)
                } else {
                    1
                },
            })
            .collect();
        Self {
            pads: [Pad::default(); 5],
            spi_clk: true,
            spi_data: None,
            clk: false,
            nreset: true,
            wire: Interface::new(target.mode, Link::default(), Chain::new(taps)),
            adi,
            tdo_out: None,
            recording: false,
            log: Log::default(),
            effects: Vec::new(),
            rate: None,
            rates: Vec::new(),
            idle: 0,
        }
    }

    fn parts(&mut self) -> (&mut Interface, Wires<'_>) {
        let wires = Wires {
            adi: &mut self.adi,
            fx: &mut self.effects,
            rates: &mut self.rates,
            rate: self.rate,
            log: self.recording.then_some(&mut self.log.transactions),
        };
        (&mut self.wire, wires)
    }

    /// Record every cell and transaction from now on, or stop; recording changes nothing else.
    pub fn record(&mut self, on: bool) {
        self.recording = on;
        self.wire.chain.record(on);
    }

    /// What has been recorded.
    pub fn log(&self) -> &Log {
        &self.log
    }

    /// Take what has been recorded, leaving the log empty.
    pub fn take_log(&mut self) -> Log {
        std::mem::take(&mut self.log)
    }

    /// The recorded cells as an edge trace: the sample point is the rise, the stamp the cell index.
    pub fn trace(&self) -> Vec<TraceEvent> {
        (0u64..)
            .zip(&self.log.cells)
            .map(|(stamp, c)| TraceEvent::Cell {
                host: c.lines,
                target_sample: c.lines,
                drives: Some(c.drives),
                stamp,
            })
            .collect()
    }

    /// The target's protocol position.
    pub fn link_state(&self) -> LinkState {
        LinkState {
            mode: self.wire.mode,
            swd: self.wire.link.state(),
            tap: self.wire.chain.state(),
        }
    }

    /// The pad on `line`.
    pub fn pad(&self, line: Line) -> Pad {
        self.pads[line.index()]
    }

    /// Set the GPIO direction of `line`, which takes the pad back from a peripheral.
    pub fn set_output(&mut self, line: Line, output: bool) {
        let pad = &mut self.pads[line.index()];
        pad.output = output;
        pad.peripheral = false;
        self.settle();
    }

    /// Set the GPIO output latch of `line`.
    pub fn set_latch(&mut self, line: Line, high: bool) {
        self.pads[line.index()].latch = high;
        self.settle();
    }

    /// Hand `line` to the SPI block or back to its GPIO.
    pub fn set_peripheral(&mut self, line: Line, on: bool) {
        self.pads[line.index()].peripheral = on;
        self.settle();
    }

    /// What the probe drives on `line`, if anything.
    pub fn probe(&self, line: Line) -> Option<bool> {
        let pad = self.pads[line.index()];
        if pad.peripheral {
            match line {
                Line::Swclk => Some(self.spi_clk),
                Line::Swdio => self.spi_data,
                _ => None,
            }
        } else if pad.output {
            Some(pad.latch)
        } else {
            None
        }
    }

    /// What the probe drives on every line, in [`Line`] order.
    pub fn drives(&self) -> [Option<bool>; 5] {
        Line::ALL.map(|l| self.probe(l))
    }

    /// The level on every line, in [`Line`] order.
    pub fn levels(&self) -> [bool; 5] {
        Line::ALL.map(|l| self.level(l))
    }

    /// The level on `line`: the driver, else the pull (SWCLK low, the rest high).
    pub fn level(&self, line: Line) -> bool {
        self.resolve(line, self.probe(line))
    }

    fn resolve(&self, line: Line, probe: Option<bool>) -> bool {
        match line {
            Line::Swclk => probe.unwrap_or(false),
            Line::Swdio => {
                let target = if self.wire.mode == Mode::Swd {
                    self.wire.link.drive()
                } else {
                    None
                };
                target.or(probe).unwrap_or(true)
            }
            Line::Tdo => {
                let target = if self.wire.mode == Mode::Jtag {
                    self.tdo_out
                } else {
                    None
                };
                target.or(probe).unwrap_or(true)
            }
            Line::Tdi | Line::Nreset => probe.unwrap_or(true),
        }
    }

    /// One SPI clock cell on the peripheral-owned pads: `tx` drives SWDIO, `None` releases it.
    pub fn spi_cell(&mut self, tx: Option<bool>) -> bool {
        self.spi_data = tx;
        self.spi_clk = false;
        self.settle();
        let bit = self.level(Line::Swdio);
        self.spi_clk = true;
        self.settle();
        bit
    }

    /// The SPI block's SWDIO drive: `Some` holds a level, `None` releases the pad.
    pub fn set_spi_data(&mut self, tx: Option<bool>) {
        self.spi_data = tx;
        self.settle();
    }

    fn settle(&mut self) {
        let clk = self.level(Line::Swclk);
        if clk != self.clk {
            self.clk = clk;
            if clk {
                self.rising();
            } else {
                self.tdo_out = self.wire.chain.tdo();
            }
        }
        let nreset = self.level(Line::Nreset);
        if nreset != self.nreset {
            self.nreset = nreset;
            self.effects.push(Effect::Reset(nreset));
        }
    }

    fn rising(&mut self) {
        let swdio = self.level(Line::Swdio);
        let probe = self.probe(Line::Swdio);
        let (mode, link) = (self.wire.mode, &self.wire.link);
        if mode == Mode::Swd {
            if link.drive().is_some() && probe.is_some() {
                self.effects.push(Effect::Contention);
            }
            let between = matches!(link.state(), swd::State::Idle | swd::State::Turn);
            if between && probe == Some(false) {
                self.idle += 1;
            }
        }
        let rise = self.recording.then(|| {
            let target = match mode {
                Mode::Swd => link.drive(),
                Mode::Jtag => self.tdo_out,
            };
            let (mut probe, mut lines) = ([None; 5], 0);
            for (i, &line) in Line::ALL.iter().enumerate() {
                probe[i] = self.probe(line);
                lines |= u8::from(self.resolve(line, probe[i])) << i;
            }
            (Lines(lines), Drives { probe, target })
        });
        let tdi = self.level(Line::Tdi);
        let (wire, mut w) = self.parts();
        let clocked = wire.clock(swdio, tdi, None, &mut w);
        match clocked.swj {
            Some(Swj::Switch(mode)) => w.fx.push(Effect::Mode(mode)),
            Some(Swj::LineReset) => w.fx.push(Effect::LineReset),
            None => {}
        }
        match clocked.event {
            Some(swd::Event::Malformed) => w.fx.push(Effect::Protocol),
            Some(swd::Event::Done(t)) => {
                if let Some(log) = w.log {
                    log.push(Transaction::Swd(t));
                }
            }
            Some(swd::Event::Targetsel(..)) | None => {}
        }
        if let Some(Swj::Switch(_)) = clocked.swj {
            self.tdo_out = None;
        }
        if let Some((lines, drives)) = rise {
            self.log.cells.push(Cell {
                lines,
                drives,
                role: clocked.role,
            });
        }
    }
}

#[cfg(all(test, feature = "differential"))]
mod tests {
    use super::*;
    use crate::adi::jtag_dp::JtagAck;
    use crate::differential::diff::{check, run, Check};
    use crate::differential::pads::{NoLeds, SimDelay, SimPin};
    use crate::differential::reference::{dap_rs_only, scan_chain};
    use crate::session::{stream, Profile};
    use crate::wire::swj::JTAG_TO_SWD;
    use crate::wire::{Drive, Phase};
    use dap_rs::dap::Dap;
    use dap_rs::driver::bitbang::{BitbangAdapter, InputOutputPin};
    use dap_rs::swo::NoSwo;
    use std::ops::Range;

    /// A pad handle; on TCK it sends every IR bit outside `keep` as 0, as a wrong bypass fill.
    struct Pin {
        pin: SimPin,
        sim: Shared,
        keep: Option<Range<usize>>,
        at: usize,
    }

    impl InputOutputPin for Pin {
        fn set_as_output(&mut self) {
            self.pin.set_as_output();
        }

        fn set_high(&mut self, high: bool) {
            if let Some(keep) = &self.keep {
                let shifting = self.sim.borrow().wire.chain.state() == TapState::ShiftIr;
                if !high && shifting {
                    if !keep.contains(&self.at) {
                        self.sim.borrow_mut().set_latch(Line::Tdi, false);
                    }
                    self.at += 1;
                }
                self.pin.set_high(high);
                if high && self.sim.borrow().wire.chain.state() != TapState::ShiftIr {
                    self.at = 0;
                }
            } else {
                self.pin.set_high(high);
            }
        }

        fn set_as_input(&mut self) {
            self.pin.set_as_input();
        }

        fn is_high(&mut self) -> bool {
            self.pin.is_high()
        }
    }

    fn zero_bypass(sim: &Shared) -> Box<dyn crate::differential::diff::Probe> {
        let (lens, adi) = {
            let s = sim.borrow();
            (s.adi.ir_lens.clone(), s.adi.tap)
        };
        let before: usize = lens[..adi].iter().map(|&l| usize::from(l)).sum();
        let keep = before..before + usize::from(lens[adi]);
        let pin = |line, keep| Pin {
            pin: if line == Line::Nreset {
                SimPin::open_drain(sim, line)
            } else {
                SimPin::new(sim, line)
            },
            sim: sim.clone(),
            keep,
            at: 0,
        };
        let deps = BitbangAdapter::new(
            pin(Line::Nreset, None),
            pin(Line::Tdi, None),
            pin(Line::Swdio, None),
            pin(Line::Swclk, Some(keep)),
            pin(Line::Tdo, None),
            SimDelay,
            scan_chain(8),
        );
        let mut dap = Dap::new(deps, NoLeds, SimDelay, NoSwo, "diff", 512);
        dap.suspend();
        Box::new(move |r: &[u8], o: &mut [u8]| dap.process_command(r, o))
    }

    #[test]
    fn a_zeroed_bypass_fill_diverges() {
        let c = Check::new(Profile::JTAG, 0..50, &crate::differential::DAP_RS);
        let d = check(&c, zero_bypass).unwrap_err();
        let crate::differential::Failure::Diverged(d) = *d else {
            panic!("{d}")
        };
        assert!(d.stream.target.chain.len() > 1, "{d}");
    }

    /// A pad handle; on TCK it runs `detour` after each xPACC read.
    struct Detour {
        pin: SimPin,
        sim: Shared,
        detour: Option<fn(&mut Sim)>,
    }

    impl InputOutputPin for Detour {
        fn set_as_output(&mut self) {
            self.pin.set_as_output();
        }

        fn set_high(&mut self, high: bool) {
            let (tap, read) = {
                let s = self.sim.borrow();
                let tap = s.adi.tap;
                let read = tap < s.adi.ir_lens.len()
                    && matches!(
                        s.adi.register(tap, s.wire.chain.ir(tap)),
                        Instruction::Dpacc | Instruction::Apacc
                    )
                    && !s.adi.waited
                    && s.wire.chain.dr(tap) & 1 != 0;
                (s.wire.chain.state(), read)
            };
            self.pin.set_high(high);
            let Some(detour) = self.detour else { return };
            let mut s = self.sim.borrow_mut();
            if high && tap == TapState::UpdateDr && s.wire.chain.state() == TapState::Idle && read {
                detour(&mut s);
            }
        }

        fn set_as_input(&mut self) {
            self.pin.set_as_input();
        }

        fn is_high(&mut self) -> bool {
            self.pin.is_high()
        }
    }

    /// The instruction in every TAP.
    fn irs(sim: &Sim) -> Vec<u32> {
        (0..sim.adi.ir_lens.len())
            .map(|i| sim.wire.chain.ir(i))
            .collect()
    }

    /// Loads `to(ir_len)` into the ADI TAP and back.
    fn via(sim: &mut Sim, to: fn(u8) -> u32) {
        let back = irs(sim);
        let mut there = back.clone();
        there[sim.adi.tap] = to(sim.adi.ir_lens[sim.adi.tap]);
        load_ir(sim, &there);
        load_ir(sim, &back);
    }

    /// Writes DP address 0, no JTAG-DP register; `forget` then clears the UNKNOWN it left.
    fn write_dp0(sim: &mut Sim, forget: bool) {
        if sim.adi.dp.busy > 0 {
            return;
        }
        let (back, previous) = (irs(sim), sim.adi.pipeline.previous);
        let (adi, lens) = (sim.adi.tap, sim.adi.ir_lens.clone());
        let mut dpacc = back.clone();
        dpacc[adi] = ((1 << lens[adi]) - 1) ^ 5;
        load_ir(sim, &dpacc);
        assert!((0..lens.len())
            .all(|i| i == adi || sim.adi.register(i, sim.wire.chain.ir(i)) == Instruction::Bypass));
        let len = (0..lens.len()).map(|i| if i == adi { 35 } else { 1 }).sum();
        scan(sim, false, &vec![false; len]);
        // The ACK after it still reports on dap-rs's own read.
        sim.adi.pipeline.previous = previous;
        if forget {
            sim.adi.pipeline.unknown = None;
        }
        load_ir(sim, &back);
    }

    /// One IR scan from Run-Test/Idle back to it, loading `irs`; TCK is left high.
    fn load_ir(sim: &mut Sim, irs: &[u32]) {
        let mut bits = Vec::new();
        for (&len, &ir) in sim.adi.ir_lens.iter().zip(irs) {
            bits.extend((0..len).map(|i| ir >> i & 1 != 0));
        }
        scan(sim, true, &bits);
    }

    /// One IR or DR scan from Run-Test/Idle back to it, shifting `bits`; TCK is left high.
    fn scan(sim: &mut Sim, ir: bool, bits: &[bool]) {
        let (tms, tdi) = (sim.pad(Line::Swdio).latch, sim.pad(Line::Tdi).latch);
        let cell = |sim: &mut Sim, tms: bool, tdi: bool| {
            sim.set_latch(Line::Swdio, tms);
            sim.set_latch(Line::Tdi, tdi);
            sim.set_latch(Line::Swclk, false);
            sim.set_latch(Line::Swclk, true);
        };
        let head: &[bool] = if ir {
            &[true, true, false, false]
        } else {
            &[true, false, false]
        };
        for &t in head {
            cell(sim, t, true);
        }
        for (i, &b) in bits.iter().enumerate() {
            cell(sim, i + 1 == bits.len(), b);
        }
        cell(sim, true, true);
        cell(sim, false, true);
        sim.set_latch(Line::Swdio, tms);
        sim.set_latch(Line::Tdi, tdi);
    }

    fn detour(sim: &Shared, to: fn(&mut Sim)) -> Box<dyn crate::differential::diff::Probe> {
        let pin = |line, detour| Detour {
            pin: if line == Line::Nreset {
                SimPin::open_drain(sim, line)
            } else {
                SimPin::new(sim, line)
            },
            sim: sim.clone(),
            detour,
        };
        let deps = BitbangAdapter::new(
            pin(Line::Nreset, None),
            pin(Line::Tdi, None),
            pin(Line::Swdio, None),
            pin(Line::Swclk, Some(to)),
            pin(Line::Tdo, None),
            SimDelay,
            scan_chain(8),
        );
        let mut dap = Dap::new(deps, NoLeds, SimDelay, NoSwo, "diff", 512);
        dap.suspend();
        Box::new(move |r: &[u8], o: &mut [u8]| dap.process_command(r, o))
    }

    /// The first seed on which a `to` detour changes what dap-rs answers.
    fn changed_by(to: fn(&mut Sim)) -> Option<u64> {
        (0..50).find(|&seed| {
            let s = stream(seed, &Profile::JTAG);
            let sim = shared(&s.target);
            let theirs = run(&s, &mut dap_rs_only(&sim), &sim);
            let sim = shared(&s.target);
            let ours = run(&s, detour(&sim, to).as_mut(), &sim);
            theirs
                .iter()
                .zip(&ours)
                .any(|(t, o)| t.response != o.response)
        })
    }

    #[test]
    fn an_idcode_detour_loses_the_posted_read() {
        assert!(changed_by(|s| via(s, jtag_dp::idcode_ir)).is_some());
    }

    #[test]
    fn a_bypass_detour_keeps_the_posted_read() {
        assert_eq!(changed_by(|s| via(s, |len| (1 << len) - 1)), None);
    }

    #[test]
    fn a_write_detour_loses_the_posted_read() {
        assert!(changed_by(|s| write_dp0(s, false)).is_some());
        assert_eq!(changed_by(|s| write_dp0(s, true)), None);
    }

    /// One xPACC scan of the lone ADI TAP, capture then update: its ACK and ReadResult.
    fn xpacc(sim: &mut Sim, ir: u32, read: bool, a: u8, data: u32) -> (u64, u32) {
        let (_, mut w) = sim.parts();
        let (sr, _) = w.capture(0, ir);
        w.update(
            0,
            ir,
            u64::from(data) << 3 | u64::from(a) << 1 | u64::from(read),
        );
        (sr & 7, (sr >> 3) as u32)
    }

    #[test]
    fn test_logic_reset_selects_idcode_in_every_tap() {
        let mut sim = Sim::new(&Target {
            mode: Mode::Jtag,
            chain: vec![5, 4, 3],
            adi: 1,
            ..Target::default()
        });
        for line in [Line::Swclk, Line::Swdio, Line::Tdi] {
            sim.set_output(line, true);
        }
        let tms = |sim: &mut Sim, high: bool| {
            sim.set_latch(Line::Swdio, high);
            sim.set_latch(Line::Swclk, false);
            sim.set_latch(Line::Swclk, true);
        };
        tms(&mut sim, false);
        load_ir(&mut sim, &[0x1F, 0xA, 0x2]);
        assert_ne!(
            sim.adi.register(1, sim.wire.chain.ir(1)),
            Instruction::Idcode
        );
        for _ in 0..5 {
            tms(&mut sim, true);
        }
        for tap in 0..3 {
            let reg = sim.adi.register(tap, sim.wire.chain.ir(tap));
            assert_eq!(reg, Instruction::Idcode, "TAP {tap}");
        }
    }

    #[test]
    fn the_adi_idcode_follows_the_dp_version() {
        for (adiv6, jtag_version, idcode) in [
            (false, DpVersion::V0, ADI_IDCODE),
            (false, DpVersion::V1, ADI_IDCODE_V1),
            (true, DpVersion::V3, ADI_IDCODE_V6),
        ] {
            let mut sim = Sim::new(&Target {
                mode: Mode::Jtag,
                adiv6,
                jtag_version,
                ..Target::default()
            });
            let (_, mut w) = sim.parts();
            let want = (u64::from(idcode), 32);
            assert_eq!(
                w.capture(0, jtag_dp::idcode_ir(4)),
                want,
                "{jtag_version:?}"
            );
        }
    }

    #[test]
    fn a_fault_leaves_the_read_result_unknown() {
        let mut sim = Sim::new(&Target {
            mode: Mode::Jtag,
            adiv6: true,
            ..Target::default()
        });
        let (dpacc, apacc) = (0xA, 0xB);
        xpacc(&mut sim, dpacc, false, 2, 0);
        xpacc(&mut sim, apacc, false, 1, FAULT_BASE);
        xpacc(&mut sim, apacc, true, 3, 0);
        assert_eq!(
            xpacc(&mut sim, dpacc, true, 1, 0),
            (JtagAck::Fault.bits(), !0)
        );
        let (ack, ctrl_stat) = xpacc(&mut sim, dpacc, true, 3, 0);
        assert_eq!(ack, JtagAck::Ok.bits());
        assert_ne!(ctrl_stat & 1 << 5, 0);
    }

    fn clock(sim: &mut Sim, swdio: Option<bool>) -> bool {
        sim.set_output(Line::Swclk, true);
        match swdio {
            Some(b) => {
                sim.set_latch(Line::Swdio, b);
                sim.set_output(Line::Swdio, true);
            }
            None => sim.set_output(Line::Swdio, false),
        }
        sim.set_latch(Line::Swclk, false);
        let bit = sim.level(Line::Swdio);
        sim.set_latch(Line::Swclk, true);
        bit
    }

    fn bits(sim: &mut Sim, v: u64, n: usize) {
        for i in 0..n {
            clock(sim, Some(v >> i & 1 != 0));
        }
    }

    fn connect(sim: &mut Sim) {
        bits(sim, u64::MAX, 56);
        bits(sim, u64::from(JTAG_TO_SWD), 16);
        bits(sim, u64::MAX, 56);
        bits(sim, 0, 8);
    }

    fn read(sim: &mut Sim, req: u8) -> (u8, u32) {
        bits(sim, u64::from(req), 8);
        clock(sim, None);
        let ack = (0..3).fold(0, |a, i| a | u8::from(clock(sim, None)) << i);
        if ack != 1 {
            clock(sim, None);
            return (ack, 0);
        }
        let v = (0..32).fold(0, |a, i| a | u32::from(clock(sim, None)) << i);
        clock(sim, None);
        clock(sim, None);
        (ack, v)
    }

    #[test]
    fn a_line_reset_and_select_answer_dpidr() {
        let mut sim = Sim::new(&Target {
            mode: Mode::Jtag,
            ..Target::default()
        });
        connect(&mut sim);
        assert_eq!(read(&mut sim, 0xA5), (1, DPIDR));
        assert!(sim.effects.contains(&Effect::Mode(Mode::Swd)));
        assert!(!sim.effects.contains(&Effect::Contention));
    }

    #[test]
    fn a_target_before_its_line_reset_does_not_answer() {
        let mut sim = Sim::new(&Target::default());
        assert_eq!(read(&mut sim, 0xA5).0, 0b111);
    }

    #[test]
    fn busy_aps_answer_wait_until_the_debt_is_paid() {
        let mut sim = Sim::new(&Target {
            waits: vec![2],
            ..Target::default()
        });
        connect(&mut sim);
        assert_eq!(read(&mut sim, 0x87).0, 1);
        assert_eq!(read(&mut sim, 0xBD).0, 2);
        assert_eq!(read(&mut sim, 0xBD).0, 2);
        assert_eq!(read(&mut sim, 0xBD).0, 1);
    }

    #[test]
    fn a_recorded_read_logs_its_cells_and_packet() {
        let mut sim = Sim::new(&Target::default());
        connect(&mut sim);
        sim.record(true);
        assert_eq!(read(&mut sim, 0xA5), (1, DPIDR));
        let log = sim.take_log();
        let req = Request {
            ap: false,
            read: true,
            a: 0,
        };
        let t = swd::Transfer {
            req,
            ack: swd::ACK_OK,
            data: Some(DPIDR),
            parity_ok: true,
        };
        assert_eq!(log.transactions, [Transaction::Swd(t)]);
        let mut want = vec![Phase::Request; 8];
        want.push(Phase::Turnaround);
        want.extend([Phase::Ack; 3]);
        want.extend([Phase::ReadData; 32]);
        want.extend([Phase::ReadParity, Phase::Turnaround]);
        let phases: Vec<Phase> = log.cells.iter().map(|c| c.role.phase).collect();
        assert_eq!(phases, want);
        for c in &log.cells {
            let target = c.role.driver == Drive::Target;
            assert_eq!(c.drives.target.is_some(), target, "{c:?}");
            assert_eq!(c.drives.probe[1].is_some(), c.role.driver == Drive::Host);
            if target {
                assert_eq!(c.drives.target, Some(c.lines.get(Line::Swdio)));
            }
        }
        assert_eq!(log.cells[8 + 1 + 3 + 31].role.bit, Some(31));
    }

    #[test]
    fn a_recorded_jtag_read_logs_its_scans_and_result() {
        let s = crate::session::Stream {
            target: Target {
                mode: Mode::Jtag,
                jtag_version: DpVersion::V1,
                ..Target::default()
            },
            cmds: vec![
                crate::session::Cmd::Connect(2),
                crate::session::jtag_select(),
                crate::session::Cmd::JtagSequence(vec![(0x46, vec![0x3F]), (0x01, vec![0x00])]),
                crate::session::Cmd::JtagConfigure(vec![4]),
                crate::session::Cmd::Transfer(
                    0,
                    vec![crate::session::Item {
                        req: 0x02,
                        word: None,
                    }],
                ),
            ],
        };
        let sim = shared(&s.target);
        sim.borrow_mut().record(true);
        run(
            &s,
            &mut crate::differential::reference::reference(&sim),
            &sim,
        );
        let sim = sim.borrow();
        let log = sim.log();
        let xpaccs: Vec<&Xpacc> = log
            .transactions
            .iter()
            .filter_map(|t| match t {
                Transaction::Xpacc(x) => Some(x),
                _ => None,
            })
            .collect();
        assert!(xpaccs
            .iter()
            .all(|x| x.access == Instruction::Dpacc && x.read));
        assert_eq!((xpaccs[0].a, xpaccs[0].ack), (0, JtagAck::OkFault));
        let last = xpaccs.last().unwrap();
        assert_eq!((last.a, last.result), (3, ReadResult::Valid(DPIDR)));
        for (i, t) in log.transactions.iter().enumerate() {
            if let Transaction::Xpacc(_) = t {
                let Transaction::Scan(scan) = &log.transactions[i - 1] else {
                    panic!("{t:?} without its scan");
                };
                assert_eq!((scan.ir, scan.tdi.len()), (false, 35));
            }
        }
        assert_eq!(sim.trace().len(), log.cells.len());
    }
}
