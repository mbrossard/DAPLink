//! The autonomous JTAG programs, interpreted instruction by instruction on a simulated JTAG-DP.

use std::collections::VecDeque;
use std::vec;
use std::vec::Vec;

use dap_differential::target::{DPIDR, DPIDR_V3, FAULT_BASE};
use dap_differential::wire::jtag::TapState;
use dap_differential::{shared, DpVersion, Effect, Line, Mode, Shared, Target};

use crate::asm::{jmp, set, Cond, Dest, SideSet};
use crate::autonomous_program::{
    ir_scan_words, request_word, AutonomousProgram, ACK_SHIFT, IR_EXEC_BOTTOM, WAIT_REVERSED,
};
use crate::clock::JTAG_TIMING_200MHZ;

const SS2: SideSet = SideSet { data_bits: 2 };
const DPACC: u32 = 0x0A;
const APACC: u32 = 0x0B;

/// One PIO state machine as the programs use it: TDI is the `out`/`set` pin, TCK and TMS side-set.
struct Pio {
    sim: Shared,
    program: AutonomousProgram,
    pc: u8,
    x: u32,
    y: u32,
    isr: u32,
    osr: u32,
    exec: Option<u16>,
    tx: VecDeque<u32>,
    rx: VecDeque<u32>,
}

impl Pio {
    /// `enter`: idle levels staged, Y loaded, parked at the data program's first `pull`.
    fn new(sim: &Shared, program: AutonomousProgram) -> Self {
        let mut s = sim.borrow_mut();
        for (line, level) in [(Line::Swclk, false), (Line::Swdio, true), (Line::Tdi, true)] {
            s.set_latch(line, level);
            s.set_output(line, true);
        }
        drop(s);
        let mut pio = Self {
            sim: sim.clone(),
            program,
            pc: 0,
            x: 0,
            y: 0,
            isr: 0,
            osr: 0,
            exec: None,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
        };
        pio.execute(set(SS2, Dest::Y, WAIT_REVERSED, None, 0));
        pio
    }

    fn pin(&self, line: Line, high: bool) {
        self.sim.borrow_mut().set_latch(line, high);
    }

    /// Run one instruction; `false` when it stalls on an empty TX FIFO.
    fn step(&mut self) -> bool {
        let (word, from_exec) = match self.exec.take() {
            Some(word) => (word, true),
            None => (self.program.words[usize::from(self.pc)], false),
        };
        if word >> 13 == 0b100 && word & 0x80 != 0 && self.tx.is_empty() {
            self.exec = from_exec.then_some(word);
            return false;
        }
        if word >> 12 & 1 != 0 {
            // TMS settles before TCK moves.
            let side = word >> 10 & 3;
            self.pin(Line::Swdio, side & 2 != 0);
            self.pin(Line::Swclk, side & 1 != 0);
        }
        let jumped = self.execute(word);
        if !jumped && !from_exec {
            self.pc = if self.pc == self.program.wrap_top {
                self.program.wrap_bottom
            } else {
                self.pc + 1
            };
        }
        true
    }

    /// Execute `word`'s operation; `true` when it jumped.
    fn execute(&mut self, word: u16) -> bool {
        let (low, dest) = (u32::from(word & 0x1f), word >> 5 & 7);
        match word >> 13 {
            0b000 => {
                let taken = match dest {
                    0 => true,
                    2 => {
                        let taken = self.x != 0;
                        self.x = self.x.wrapping_sub(1);
                        taken
                    }
                    5 => self.x != self.y,
                    _ => unimplemented!("jmp condition {dest}"),
                };
                if taken {
                    self.pc = low as u8;
                }
                return taken;
            }
            0b010 => {
                assert_eq!((dest, low), (0, 1), "in pins, 1");
                let tdo = u32::from(self.sim.borrow().level(Line::Tdo));
                self.isr = self.isr >> 1 | tdo << 31;
            }
            0b011 => {
                let n = if low == 0 { 32 } else { low };
                let data = if n == 32 {
                    self.osr
                } else {
                    self.osr & ((1 << n) - 1)
                };
                self.osr = self.osr.checked_shr(n).unwrap_or(0);
                match dest {
                    0 => self.pin(Line::Tdi, data & 1 != 0),
                    1 => self.x = data,
                    7 => self.exec = Some(data as u16),
                    _ => unimplemented!("out destination {dest}"),
                }
            }
            0b100 if word & 0x80 != 0 => self.osr = self.tx.pop_front().expect("stalls first"),
            0b100 => {
                self.rx.push_back(self.isr);
                self.isr = 0;
            }
            0b101 => match word & 0xff {
                0x42 => {}
                0xf6 => self.osr = self.isr.reverse_bits(),
                other => unimplemented!("mov {other:#x}"),
            },
            0b110 => {}
            0b111 => match dest {
                0 => self.pin(Line::Tdi, low & 1 != 0),
                1 => self.x = low,
                2 => self.y = low,
                _ => unimplemented!("set destination {dest}"),
            },
            _ => unreachable!(),
        }
        false
    }

    fn run(&mut self) {
        let mut budget = 10_000;
        while self.step() {
            budget -= 1;
            assert!(budget > 0, "the program never stalled");
        }
    }

    /// `RpAutonomousJtag::transfer`: the request and data words in, the ACK and captured word out.
    fn transfer(&mut self, before: u8, request: u8, data: u32) -> (u32, u32) {
        self.tx.extend([request_word(before, request), data]);
        self.run();
        assert_eq!(
            (self.rx.len(), self.pc),
            (2, 0),
            "two words, parked at the pull"
        );
        let ack = self.rx.pop_front().unwrap() >> ACK_SHIFT;
        (ack, self.rx.pop_front().unwrap())
    }

    /// `RpAutonomousJtag::shift_ir` through the executor, for the chain `[4, 5]`.
    fn shift_ir(&mut self, ir: u32) {
        self.execute(set(SS2, Dest::Pins, 1, Some(0b10), 0));
        self.pin(Line::Swdio, true);
        self.execute(jmp(SS2, Cond::Always, IR_EXEC_BOTTOM, None, 0));
        self.tx
            .extend(ir_scan_words(ir, JTAG_TIMING_200MHZ).expect("a JTAG-DP instruction"));
        self.run();
        assert_eq!(self.pc, 0);
    }
}

/// Clock `tms` cells with TDI high and TCK back low, straight on the pads.
fn walk(sim: &Shared, tms: &[bool], tdi: impl Fn(usize) -> bool) {
    for (i, &t) in tms.iter().enumerate() {
        let mut s = sim.borrow_mut();
        s.set_latch(Line::Swdio, t);
        s.set_latch(Line::Tdi, tdi(i));
        s.set_latch(Line::Swclk, true);
        s.set_latch(Line::Swclk, false);
    }
}

/// A one-TAP IR scan from Run-Test/Idle, straight on the pads.
fn shift_ir_one(sim: &Shared, ir: u32) {
    let tms = [
        true, true, false, false, false, false, false, true, true, false,
    ];
    walk(sim, &tms, |i| {
        !(4..=7).contains(&i) || ir >> (i - 4) & 1 != 0
    });
    sim.borrow_mut().set_latch(Line::Tdi, true);
}

fn target(chain: Vec<u8>, adiv6: bool) -> Target {
    Target {
        mode: Mode::Jtag,
        chain,
        adiv6,
        ..Target::default()
    }
}

/// A chain from Test-Logic-Reset into Run-Test/Idle, under `program`.
fn start(t: &Target, after: u8) -> (Shared, Pio) {
    let sim = shared(t);
    let program = AutonomousProgram::new(0, after, JTAG_TIMING_200MHZ);
    let pio = Pio::new(&sim, program);
    walk(&sim, &[false], |_| true);
    (sim, pio)
}

fn select_ir(sim: &Shared, pio: &mut Pio, after: u8, ir: u32) {
    if after == 1 {
        pio.shift_ir(ir);
    } else {
        shift_ir_one(sim, ir);
    }
}

fn written(sim: &Shared) -> Vec<Effect> {
    std::mem::take(&mut sim.borrow_mut().effects)
}

/// OK in either protocol, a read's result one scan later, and FAULT after a faulted AP access.
#[test]
fn both_protocols_complete_their_scans_and_read_back() {
    for adiv6 in [false, true] {
        for after in [0u8, 1] {
            let chain = if after == 1 { vec![4, 5] } else { vec![4] };
            let (ok, fault, dpidr) = if adiv6 {
                (0b100, 0b010, DPIDR_V3)
            } else {
                (0b010, 0b010, DPIDR)
            };
            let t = Target {
                jtag_version: DpVersion::V1,
                ..target(chain, adiv6)
            };
            let (sim, mut pio) = start(&t, after);
            select_ir(&sim, &mut pio, after, DPACC);

            assert_eq!(pio.transfer(0, 0b010, 0x5000_0020).0, ok);
            let dp = Effect::Dp {
                addr: 4,
                data: 0x5000_0020,
            };
            assert_eq!(
                written(&sim),
                [dp],
                "the intended CTRL/STAT write, adiv6 {adiv6}"
            );
            assert_eq!(pio.transfer(0, 0b001, 0).0, ok);
            assert_eq!(pio.transfer(0, 0b111, 0), (ok, dpidr));

            assert_eq!(pio.transfer(0, 0b100, 0).0, ok);
            select_ir(&sim, &mut pio, after, APACC);
            assert_eq!(pio.transfer(0, 0b010, FAULT_BASE).0, ok);
            assert_eq!(pio.transfer(0, 0b111, 0).0, ok);
            select_ir(&sim, &mut pio, after, DPACC);
            assert_eq!(pio.transfer(0, 0b111, 0).0, fault);
            assert!(written(&sim).contains(&Effect::Sticky));
        }
    }
}

/// WAIT leaves after the ACK with nothing executed; the next scan still completes.
#[test]
fn wait_leaves_early_and_executes_nothing() {
    for adiv6 in [false, true] {
        let t = Target {
            waits: vec![1],
            ..target(vec![4], adiv6)
        };
        let (sim, mut pio) = start(&t, 0);
        shift_ir_one(&sim, APACC);
        pio.transfer(0, 0b111, 0);
        written(&sim);
        assert_eq!(pio.transfer(0, 0b010, 0x1234_5678), (0b001, 0));
        assert!(written(&sim).is_empty());
        assert_eq!(sim.borrow().link_state().tap, TapState::Idle);
        assert_ne!(pio.transfer(0, 0b010, 0x1234_5678).0, 0b001);
    }
}

/// With nothing on the chain TDO reads high: ACK `0b111`, and the scan runs to the end.
#[test]
fn no_target_reads_all_ones_through_a_complete_scan() {
    for after in [0u8, 1] {
        let t = Target {
            absent: true,
            ..target(vec![4, 5], false)
        };
        let (sim, mut pio) = start(&t, after);
        assert_eq!(pio.transfer(0, 0b111, 0), (0b111, u32::MAX));
        assert_eq!(sim.borrow().link_state().tap, TapState::Idle);
    }
}
