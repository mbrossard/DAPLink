//! JTAG: the TAP controller, and a chain of TAPs with their instruction and shift registers.

use super::{CellRole, Class, Drive, Phase};

/// IEEE 1149.1 TAP controller states.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TapState {
    /// Test-Logic-Reset.
    Reset,
    /// Run-Test/Idle.
    Idle,
    /// Select-DR-Scan.
    SelectDr,
    /// Capture-DR.
    CaptureDr,
    /// Shift-DR.
    ShiftDr,
    /// Exit1-DR.
    Exit1Dr,
    /// Pause-DR.
    PauseDr,
    /// Exit2-DR.
    Exit2Dr,
    /// Update-DR.
    UpdateDr,
    /// Select-IR-Scan.
    SelectIr,
    /// Capture-IR.
    CaptureIr,
    /// Shift-IR.
    ShiftIr,
    /// Exit1-IR.
    Exit1Ir,
    /// Pause-IR.
    PauseIr,
    /// Exit2-IR.
    Exit2Ir,
    /// Update-IR.
    UpdateIr,
}

impl TapState {
    /// The state after a TCK rising edge with TMS at `tms`.
    pub fn next(self, tms: bool) -> TapState {
        use TapState::*;
        match (self, tms) {
            (Reset, true) => Reset,
            (Reset, false) => Idle,
            (Idle, false) => Idle,
            (Idle, true) => SelectDr,
            (SelectDr, false) => CaptureDr,
            (SelectDr, true) => SelectIr,
            (CaptureDr, false) | (ShiftDr, false) | (Exit2Dr, false) => ShiftDr,
            (CaptureDr, true) | (ShiftDr, true) => Exit1Dr,
            (Exit1Dr, false) | (PauseDr, false) => PauseDr,
            (Exit1Dr, true) | (Exit2Dr, true) => UpdateDr,
            (PauseDr, true) => Exit2Dr,
            (UpdateDr, false) | (UpdateIr, false) => Idle,
            (UpdateDr, true) | (UpdateIr, true) => SelectDr,
            (SelectIr, false) => CaptureIr,
            (SelectIr, true) => Reset,
            (CaptureIr, false) | (ShiftIr, false) | (Exit2Ir, false) => ShiftIr,
            (CaptureIr, true) | (ShiftIr, true) => Exit1Ir,
            (Exit1Ir, false) | (PauseIr, false) => PauseIr,
            (Exit1Ir, true) | (Exit2Ir, true) => UpdateIr,
            (PauseIr, true) => Exit2Ir,
        }
    }
}

/// The data registers behind each TAP's instructions.
pub trait Registers {
    /// Capture-DR on `tap` under instruction `ir`: the captured value and the register's length.
    fn capture(&mut self, tap: usize, ir: u32) -> (u64, u8);
    /// Update-DR on `tap` under instruction `ir`, with the shifted-in value.
    fn update(&mut self, tap: usize, ir: u32, value: u64);
    /// Update-IR loaded `irs`, one instruction per TAP.
    fn update_ir(&mut self, irs: &[u32]);
    /// Test-Logic-Reset.
    fn reset(&mut self);
    /// A recorded scan reached Update-IR or Update-DR, before the update itself.
    fn scanned(&mut self, _scan: Scan) {}
}

/// One IR or DR scan of the whole chain, as recorded.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Scan {
    /// An IR scan, not a DR scan.
    pub ir: bool,
    /// TDI per shifted cell.
    pub tdi: Vec<bool>,
    /// TDO per shifted cell, high where no TAP drives it.
    pub tdo: Vec<bool>,
}

/// One TAP's IR length and the instruction Test-Logic-Reset loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tap {
    /// IR length, at least 1.
    pub ir_len: u8,
    /// The instruction Test-Logic-Reset loads: IDCODE, or BYPASS without one.
    pub reset_ir: u32,
}

#[derive(Clone, Copy, Debug)]
struct Shift {
    ir: u64,
    dr: u64,
    dr_len: u8,
}

/// TAPs sharing TCK and TMS, TDI entering the last and TDO leaving the first.
#[derive(Clone, Debug)]
pub struct Chain {
    state: TapState,
    taps: Vec<Tap>,
    irs: Vec<u32>,
    shift: Vec<Shift>,
    shifted: u16,
    scan: Option<(Vec<bool>, Vec<bool>)>,
}

impl Chain {
    /// A chain of `taps`, the TDO end first, in Test-Logic-Reset.
    pub fn new(taps: Vec<Tap>) -> Self {
        let n = taps.len();
        let shift = Shift {
            ir: 0,
            dr: 0,
            dr_len: 1,
        };
        Self {
            state: TapState::Reset,
            irs: taps.iter().map(|t| t.reset_ir).collect(),
            taps,
            shift: vec![shift; n],
            shifted: 0,
            scan: None,
        }
    }

    /// Record each scan's TDI and TDO for [`Registers::scanned`], or stop.
    pub fn record(&mut self, on: bool) {
        self.scan = on.then(Default::default);
    }

    /// What the next cell is.
    pub fn role(&self) -> CellRole {
        let shift = matches!(self.state, TapState::ShiftDr | TapState::ShiftIr);
        CellRole {
            phase: Phase::Tap(self.state),
            driver: if shift {
                Drive::Target
            } else {
                Drive::Released
            },
            bit: shift.then_some(self.shifted),
            class: match self.state {
                TapState::ShiftDr => Class::ShiftDr,
                TapState::ShiftIr => Class::ShiftIr,
                TapState::Reset => Class::TapReset,
                _ => Class::Control,
            },
        }
    }

    /// The TAP controllers' state.
    pub fn state(&self) -> TapState {
        self.state
    }

    /// Put the TAP controllers in `state`, as an observer does once it knows it.
    pub fn sync(&mut self, state: TapState) {
        self.state = state;
        self.shifted = 0;
        if let Some((i, o)) = &mut self.scan {
            i.clear();
            o.clear();
        }
    }

    /// The instruction in `tap`.
    pub fn ir(&self, tap: usize) -> u32 {
        self.irs[tap]
    }

    /// `tap`'s data shift register.
    #[cfg(all(test, feature = "differential"))]
    pub(crate) fn dr(&self, tap: usize) -> u64 {
        self.shift[tap].dr
    }

    /// The bit the chain presents on TDO from the next falling edge.
    pub fn tdo(&self) -> Option<bool> {
        let first = self.shift.first()?;
        match self.state {
            TapState::ShiftDr => Some(first.dr & 1 != 0),
            TapState::ShiftIr => Some(first.ir & 1 != 0),
            _ => None,
        }
    }

    /// Test-Logic-Reset, asynchronously.
    pub fn reset(&mut self, regs: &mut impl Registers) {
        self.state = TapState::Reset;
        self.reset_irs(regs);
    }

    fn reset_irs(&mut self, regs: &mut impl Registers) {
        for (ir, t) in self.irs.iter_mut().zip(&self.taps) {
            *ir = t.reset_ir;
        }
        regs.reset();
    }

    /// One TCK rising edge; a recorded scan takes `tdo` when given, else the chain's own.
    pub fn clock(&mut self, tms: bool, tdi: bool, tdo: Option<bool>, regs: &mut impl Registers) {
        match self.state {
            TapState::CaptureIr | TapState::CaptureDr => {
                self.shifted = 0;
                if let Some((i, o)) = &mut self.scan {
                    i.clear();
                    o.clear();
                }
            }
            TapState::ShiftIr | TapState::ShiftDr => {
                self.shifted = self.shifted.saturating_add(1);
                let tdo = tdo.or(self.tdo()).unwrap_or(true);
                if let Some((i, o)) = &mut self.scan {
                    i.push(tdi);
                    o.push(tdo);
                }
            }
            TapState::UpdateIr | TapState::UpdateDr => {
                if let Some((i, o)) = &mut self.scan {
                    regs.scanned(Scan {
                        ir: self.state == TapState::UpdateIr,
                        tdi: std::mem::take(i),
                        tdo: std::mem::take(o),
                    });
                }
            }
            _ => {}
        }
        match self.state {
            TapState::Reset => self.reset_irs(regs),
            // Capture-IR loads `0b01` into every IR.
            TapState::CaptureIr => self.shift.iter_mut().for_each(|s| s.ir = 1),
            TapState::ShiftIr => self.shift_bit(tdi, true),
            TapState::UpdateIr => {
                for ((ir, s), t) in self.irs.iter_mut().zip(&self.shift).zip(&self.taps) {
                    *ir = (s.ir & ((1u64 << t.ir_len) - 1)) as u32;
                }
                regs.update_ir(&self.irs);
            }
            TapState::CaptureDr => {
                for (i, s) in self.shift.iter_mut().enumerate() {
                    (s.dr, s.dr_len) = regs.capture(i, self.irs[i]);
                }
            }
            TapState::ShiftDr => self.shift_bit(tdi, false),
            TapState::UpdateDr => {
                for (i, s) in self.shift.iter().enumerate() {
                    regs.update(i, self.irs[i], s.dr);
                }
            }
            _ => {}
        }
        self.state = self.state.next(tms);
    }

    fn shift_bit(&mut self, tdi: bool, ir: bool) {
        let n = self.shift.len();
        for k in 0..n {
            let input = if k + 1 == n {
                tdi
            } else if ir {
                self.shift[k + 1].ir & 1 != 0
            } else {
                self.shift[k + 1].dr & 1 != 0
            };
            let input = u64::from(input);
            let (s, len) = (&mut self.shift[k], self.taps[k].ir_len);
            if ir {
                s.ir = s.ir >> 1 | input << (len - 1);
            } else {
                s.dr = s.dr >> 1 | input << (s.dr_len - 1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Registers that capture `captured` and record every update.
    struct Regs {
        captured: u64,
        updates: Vec<u64>,
    }

    impl Registers for Regs {
        fn capture(&mut self, _tap: usize, _ir: u32) -> (u64, u8) {
            (self.captured, 35)
        }
        fn update(&mut self, _tap: usize, _ir: u32, value: u64) {
            self.updates.push(value);
        }
        fn update_ir(&mut self, _irs: &[u32]) {}
        fn reset(&mut self) {}
    }

    /// Clock `tms` with TDI high from Run-Test/Idle, returning TDO as each rising edge saw it.
    fn walk(chain: &mut Chain, regs: &mut Regs, tms: &[bool]) -> Vec<bool> {
        let mut tdo = Vec::new();
        for &t in tms {
            tdo.push(chain.tdo().unwrap_or(true));
            chain.clock(t, true, None, regs);
        }
        tdo
    }

    fn idle(taps: &[u8]) -> (Chain, Regs) {
        let taps = taps.iter().map(|&ir_len| Tap {
            ir_len,
            reset_ir: 1,
        });
        let mut chain = Chain::new(taps.collect());
        let mut regs = Regs {
            captured: 0,
            updates: Vec::new(),
        };
        walk(&mut chain, &mut regs, &[false]);
        (chain, regs)
    }

    #[test]
    fn capture_ir_loads_01_into_every_ir() {
        let (mut chain, mut regs) = idle(&[4, 5]);
        let mut tms = vec![true, true, false, false];
        tms.extend([false; 8]);
        tms.push(true);
        let tdo = walk(&mut chain, &mut regs, &tms);
        let shifted = &tdo[4..];
        assert_eq!(
            (&shifted[..2], &shifted[4..6]),
            (&[true, false][..], &[true, false][..])
        );
    }

    #[test]
    fn update_dr_issues_what_a_short_scan_left() {
        let (mut chain, mut regs) = idle(&[4]);
        regs.captured = 0x2_1234_5672;
        let tms = [true, false, false, false, false, true, true, false];
        walk(&mut chain, &mut regs, &tms);
        assert_eq!(regs.updates, [regs.captured >> 3 | 0b111 << 32]);
    }
}
