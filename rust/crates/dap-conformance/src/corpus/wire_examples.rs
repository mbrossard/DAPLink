//! SWD, SWJ, JTAG and JTAG-DP examples from ADIv5 and IEEE 1149.1, as cells and what they carry.

use crate::adi::jtag_dp::{Instruction, JtagAck, Protocol, ReadResult, Unknown, Xpacc};
use crate::observe::{Config, Framing, TapConfig};
use crate::target::Mode;
use crate::wire::swd::{LinkConfig, Request, Transfer, ACK_FAULT, ACK_OK, ACK_WAIT};
use crate::wire::{Lines, TraceEvent};

/// A run of cells, least significant bit first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cells {
    /// The host drives SWDIO/TMS with `n` bits of the value.
    Host(u64, u8),
    /// The target drives SWDIO with `n` bits of the value.
    Target(u64, u8),
    /// Nobody drives SWDIO, which reads high, for `n` cells.
    Released(u8),
    /// TMS for `n` cycles outside a shift; TDI and TDO high.
    Tms(u64, u8),
    /// `n` shift cycles with TDI and TDO, TMS high on the last.
    Shift(u64, u64, u8),
}

/// What an example carries, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Expect {
    /// An SWD packet.
    Swd(Transfer),
    /// A TARGETSEL write.
    Targetsel(u32, bool),
    /// An IR or DR scan: its length, TDI and TDO.
    Scan(bool, usize, u64, u64),
    /// A JTAG-DP access.
    Xpacc(Xpacc),
}

/// One worked example.
#[derive(Clone, Copy, Debug)]
pub struct Example {
    /// What it shows.
    pub name: &'static str,
    /// The protocol at its start.
    pub mode: Mode,
    /// The chain's IR lengths, the TDO end first, the JTAG-DP at index 0.
    pub chain: &'static [u8],
    /// A DP write to `0xC` after a line reset is TARGETSEL.
    pub multidrop: bool,
    /// The cells.
    pub cells: &'static [Cells],
    /// What they carry.
    pub expect: &'static [Expect],
}

impl Example {
    /// The cells as an edge trace.
    pub fn trace(&self) -> Vec<TraceEvent> {
        trace(self.cells)
    }

    /// The observer's configuration for this example.
    pub fn config(&self, framing: Framing) -> Config {
        let mut taps: Vec<TapConfig> = self
            .chain
            .iter()
            .map(|&ir_len| TapConfig {
                ir_len,
                idcode: None,
                other: None,
            })
            .collect();
        if let Some(dp) = taps.first_mut() {
            *dp = TapConfig::dp(dp.ir_len);
        }
        Config {
            framing,
            mode: self.mode,
            link: LinkConfig {
                multidrop: self.multidrop,
            },
            taps,
            dp: 0,
            protocol: Protocol::Zero,
            ..Config::default()
        }
    }
}

/// The DPIDR the SWD examples read.
pub const DPIDR: u32 = 0x2BA0_1477;

const fn swd(ap: bool, read: bool, a: u8, ack: u8, data: Option<u32>, parity_ok: bool) -> Expect {
    Expect::Swd(Transfer {
        req: Request { ap, read, a },
        ack,
        data,
        parity_ok,
    })
}

const fn xpacc(
    access: Instruction,
    read: bool,
    a: u8,
    data: u32,
    ack: JtagAck,
    result: ReadResult,
    executed: bool,
) -> Expect {
    Expect::Xpacc(Xpacc {
        tap: 0,
        access,
        read,
        a,
        data,
        ack,
        result,
        executed,
    })
}

const ONES: u64 = u64::MAX;
const LINE_RESET: [Cells; 2] = [Cells::Host(ONES, 52), Cells::Host(0, 2)];
const DPIDR_READ: [Cells; 6] = [
    Cells::Host(0xA5, 8),
    Cells::Released(1),
    Cells::Target(ACK_OK as u64, 3),
    Cells::Target(DPIDR as u64, 32),
    Cells::Target(0, 1),
    Cells::Released(1),
];
const DPIDR_OK: Expect = swd(false, true, 0, ACK_OK, Some(DPIDR), true);
const ORUNDETECT_ON: [Cells; 6] = [
    Cells::Host(0xA9, 8),
    Cells::Released(1),
    Cells::Target(ACK_OK as u64, 3),
    Cells::Released(1),
    Cells::Host(1, 32),
    Cells::Host(1, 1),
];
const ORUNDETECT_SET: Expect = swd(false, false, 1, ACK_OK, Some(1), true);
const TAP_RESET: [Cells; 2] = [Cells::Tms(0b11111, 5), Cells::Tms(0, 1)];
const IR_SCAN: Cells = Cells::Tms(0b0011, 4);
const DR_SCAN: Cells = Cells::Tms(0b001, 3);
const TO_IDLE: Cells = Cells::Tms(0b01, 2);
const DPACC: u64 = 0b1010;
const APACC: u64 = 0b1011;
const IR_CAPTURE: u64 = 0b0001;
const OK_FAULT: u64 = 0b010;
const WAIT: u64 = 0b001;
const CTRL_STAT_READ: u64 = 0b011;
const RDBUFF_READ: u64 = 0b111;
const DRW_WRITE: u64 = 0b110;
const CTRL_STAT: u64 = 0xF000_0000;
const RESET: ReadResult = ReadResult::Unknown(Unknown::Reset);
const AFTER_WRITE: ReadResult = ReadResult::Unknown(Unknown::Write);

/// Every example.
pub const EXAMPLES: &[Example] = &[
    Example {
        name: "line reset, then the DPIDR read",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            DPIDR_READ[0],
            DPIDR_READ[1],
            DPIDR_READ[2],
            DPIDR_READ[3],
            DPIDR_READ[4],
            DPIDR_READ[5],
            Cells::Host(0, 2),
        ],
        expect: &[DPIDR_OK],
    },
    Example {
        name: "AP write of DRW, OK",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            Cells::Host(0xBB, 8),
            Cells::Released(1),
            Cells::Target(ACK_OK as u64, 3),
            Cells::Released(1),
            Cells::Host(0x1234_5678, 32),
            Cells::Host(1, 1),
            Cells::Host(0, 2),
        ],
        expect: &[swd(true, false, 3, ACK_OK, Some(0x1234_5678), true)],
    },
    Example {
        name: "AP read of DRW answered WAIT, ORUNDETECT clear, then the retry",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            Cells::Host(0x9F, 8),
            Cells::Released(1),
            Cells::Target(ACK_WAIT as u64, 3),
            Cells::Released(1),
            Cells::Host(0x9F, 8),
            Cells::Released(1),
            Cells::Target(ACK_OK as u64, 3),
            Cells::Target(0, 32),
            Cells::Target(0, 1),
            Cells::Released(1),
            Cells::Host(0, 2),
        ],
        expect: &[
            swd(true, true, 3, ACK_WAIT, None, true),
            swd(true, true, 3, ACK_OK, Some(0), true),
        ],
    },
    Example {
        name: "AP read of DRW answered WAIT, ORUNDETECT set: an undriven data phase",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            ORUNDETECT_ON[0],
            ORUNDETECT_ON[1],
            ORUNDETECT_ON[2],
            ORUNDETECT_ON[3],
            ORUNDETECT_ON[4],
            ORUNDETECT_ON[5],
            Cells::Host(0, 2),
            Cells::Host(0x9F, 8),
            Cells::Released(1),
            Cells::Target(ACK_WAIT as u64, 3),
            Cells::Released(33),
            Cells::Released(1),
            Cells::Host(0, 1),
        ],
        expect: &[ORUNDETECT_SET, swd(true, true, 3, ACK_WAIT, None, true)],
    },
    Example {
        name: "AP write answered WAIT, ORUNDETECT set: a data phase the target ignores",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            ORUNDETECT_ON[0],
            ORUNDETECT_ON[1],
            ORUNDETECT_ON[2],
            ORUNDETECT_ON[3],
            ORUNDETECT_ON[4],
            ORUNDETECT_ON[5],
            Cells::Host(0, 2),
            Cells::Host(0xBB, 8),
            Cells::Released(1),
            Cells::Target(ACK_WAIT as u64, 3),
            Cells::Released(1),
            Cells::Host(0, 32),
            Cells::Host(0, 1),
            Cells::Host(0, 1),
        ],
        expect: &[ORUNDETECT_SET, swd(true, false, 3, ACK_WAIT, Some(0), true)],
    },
    Example {
        name: "DP write of SELECT answered FAULT, then the CTRL/STAT read",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            Cells::Host(0xB1, 8),
            Cells::Released(1),
            Cells::Target(ACK_FAULT as u64, 3),
            Cells::Released(1),
            Cells::Host(0x8D, 8),
            Cells::Released(1),
            Cells::Target(ACK_OK as u64, 3),
            Cells::Target(0xF000_0020, 32),
            Cells::Target(1, 1),
            Cells::Released(1),
            Cells::Host(0, 2),
        ],
        expect: &[
            swd(false, false, 2, ACK_FAULT, None, true),
            swd(false, true, 1, ACK_OK, Some(0xF000_0020), true),
        ],
    },
    Example {
        name: "no ACK to a read; the host backs off, then resets the line",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            Cells::Host(0x9F, 8),
            Cells::Released(1),
            Cells::Released(3),
            Cells::Released(34),
            LINE_RESET[0],
            LINE_RESET[1],
            DPIDR_READ[0],
            DPIDR_READ[1],
            DPIDR_READ[2],
            DPIDR_READ[3],
            DPIDR_READ[4],
            DPIDR_READ[5],
            Cells::Host(0, 2),
        ],
        expect: &[swd(true, true, 3, 0b111, None, true), DPIDR_OK],
    },
    Example {
        name: "RDBUFF read with bad parity, the trailing turnaround, then the next Start",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: false,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            Cells::Host(0xBD, 8),
            Cells::Released(1),
            Cells::Target(ACK_OK as u64, 3),
            Cells::Target(1, 32),
            Cells::Target(0, 1),
            Cells::Released(1),
            DPIDR_READ[0],
            DPIDR_READ[1],
            DPIDR_READ[2],
            DPIDR_READ[3],
            DPIDR_READ[4],
            DPIDR_READ[5],
            Cells::Host(0, 2),
        ],
        expect: &[swd(false, true, 3, ACK_OK, Some(1), false), DPIDR_OK],
    },
    Example {
        name: "TARGETSEL after a line reset, then the DPIDR read",
        mode: Mode::Swd,
        chain: &[4],
        multidrop: true,
        cells: &[
            LINE_RESET[0],
            LINE_RESET[1],
            Cells::Host(0x99, 8),
            Cells::Released(5),
            Cells::Host(0x0100_2927, 32),
            Cells::Host(0, 1),
            Cells::Host(0, 2),
            DPIDR_READ[0],
            DPIDR_READ[1],
            DPIDR_READ[2],
            DPIDR_READ[3],
            DPIDR_READ[4],
            DPIDR_READ[5],
            Cells::Host(0, 2),
        ],
        expect: &[Expect::Targetsel(0x0100_2927, true), DPIDR_OK],
    },
    Example {
        name: "JTAG-to-SWD, then the DPIDR read",
        mode: Mode::Jtag,
        chain: &[4],
        multidrop: false,
        cells: &[
            Cells::Host(ONES, 50),
            Cells::Host(0xE79E, 16),
            Cells::Host(ONES, 50),
            Cells::Host(0, 2),
            DPIDR_READ[0],
            DPIDR_READ[1],
            DPIDR_READ[2],
            DPIDR_READ[3],
            DPIDR_READ[4],
            DPIDR_READ[5],
            Cells::Host(0, 2),
        ],
        expect: &[DPIDR_OK],
    },
    Example {
        name: "Shift-DR to Run-Test/Idle through Test-Logic-Reset",
        mode: Mode::Jtag,
        chain: &[4],
        multidrop: false,
        cells: &[
            TAP_RESET[0],
            TAP_RESET[1],
            DR_SCAN,
            Cells::Tms(0b0011111, 7),
        ],
        expect: &[Expect::Scan(false, 1, 1, 1)],
    },
    Example {
        name: "IR scan loading DPACC into a 4-bit IR",
        mode: Mode::Jtag,
        chain: &[4],
        multidrop: false,
        cells: &[
            TAP_RESET[0],
            TAP_RESET[1],
            IR_SCAN,
            Cells::Shift(DPACC, IR_CAPTURE, 4),
            TO_IDLE,
        ],
        expect: &[Expect::Scan(true, 4, DPACC, IR_CAPTURE)],
    },
    Example {
        name: "two-TAP chain, the other in BYPASS: DPACC reads of CTRL/STAT and RDBUFF",
        mode: Mode::Jtag,
        chain: &[4, 5],
        multidrop: false,
        cells: &[
            TAP_RESET[0],
            TAP_RESET[1],
            IR_SCAN,
            Cells::Shift(0b11111 << 4 | DPACC, 1 << 4 | IR_CAPTURE, 9),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(1 << 35 | CTRL_STAT_READ, OK_FAULT, 36),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(1 << 35 | RDBUFF_READ, CTRL_STAT << 3 | OK_FAULT, 36),
            TO_IDLE,
        ],
        expect: &[
            Expect::Scan(true, 9, 0b11111 << 4 | DPACC, 1 << 4 | IR_CAPTURE),
            Expect::Scan(false, 36, 1 << 35 | CTRL_STAT_READ, OK_FAULT),
            xpacc(
                Instruction::Dpacc,
                true,
                1,
                0,
                JtagAck::OkFault,
                RESET,
                true,
            ),
            Expect::Scan(false, 36, 1 << 35 | RDBUFF_READ, CTRL_STAT << 3 | OK_FAULT),
            xpacc(
                Instruction::Dpacc,
                true,
                3,
                0,
                JtagAck::OkFault,
                ReadResult::Valid(CTRL_STAT as u32),
                true,
            ),
        ],
    },
    Example {
        name: "DPACC read of CTRL/STAT, then the RDBUFF read that returns it",
        mode: Mode::Jtag,
        chain: &[4],
        multidrop: false,
        cells: &[
            TAP_RESET[0],
            TAP_RESET[1],
            IR_SCAN,
            Cells::Shift(DPACC, IR_CAPTURE, 4),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(CTRL_STAT_READ, OK_FAULT, 35),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(RDBUFF_READ, CTRL_STAT << 3 | OK_FAULT, 35),
            TO_IDLE,
        ],
        expect: &[
            Expect::Scan(true, 4, DPACC, IR_CAPTURE),
            Expect::Scan(false, 35, CTRL_STAT_READ, OK_FAULT),
            xpacc(
                Instruction::Dpacc,
                true,
                1,
                0,
                JtagAck::OkFault,
                RESET,
                true,
            ),
            Expect::Scan(false, 35, RDBUFF_READ, CTRL_STAT << 3 | OK_FAULT),
            xpacc(
                Instruction::Dpacc,
                true,
                3,
                0,
                JtagAck::OkFault,
                ReadResult::Valid(CTRL_STAT as u32),
                true,
            ),
        ],
    },
    Example {
        name: "APACC write of DRW, a WAIT, the retry",
        mode: Mode::Jtag,
        chain: &[4],
        multidrop: false,
        cells: &[
            TAP_RESET[0],
            TAP_RESET[1],
            IR_SCAN,
            Cells::Shift(APACC, IR_CAPTURE, 4),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(0x1234_5678 << 3 | DRW_WRITE, OK_FAULT, 35),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(0x9ABC_DEF0 << 3 | DRW_WRITE, WAIT, 35),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(0x9ABC_DEF0 << 3 | DRW_WRITE, OK_FAULT, 35),
            TO_IDLE,
        ],
        expect: &[
            Expect::Scan(true, 4, APACC, IR_CAPTURE),
            Expect::Scan(false, 35, 0x1234_5678 << 3 | DRW_WRITE, OK_FAULT),
            xpacc(
                Instruction::Apacc,
                false,
                3,
                0x1234_5678,
                JtagAck::OkFault,
                RESET,
                true,
            ),
            Expect::Scan(false, 35, 0x9ABC_DEF0 << 3 | DRW_WRITE, WAIT),
            xpacc(
                Instruction::Apacc,
                false,
                3,
                0x9ABC_DEF0,
                JtagAck::Wait,
                AFTER_WRITE,
                false,
            ),
            Expect::Scan(false, 35, 0x9ABC_DEF0 << 3 | DRW_WRITE, OK_FAULT),
            xpacc(
                Instruction::Apacc,
                false,
                3,
                0x9ABC_DEF0,
                JtagAck::OkFault,
                AFTER_WRITE,
                true,
            ),
        ],
    },
    Example {
        name: "a WAIT scan cut short after its ACK",
        mode: Mode::Jtag,
        chain: &[4],
        multidrop: false,
        cells: &[
            TAP_RESET[0],
            TAP_RESET[1],
            IR_SCAN,
            Cells::Shift(APACC, IR_CAPTURE, 4),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(0x1234_5678 << 3 | DRW_WRITE, OK_FAULT, 35),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(0b1110, WAIT, 4),
            TO_IDLE,
            DR_SCAN,
            Cells::Shift(0x9ABC_DEF0 << 3 | DRW_WRITE, OK_FAULT, 35),
            TO_IDLE,
        ],
        expect: &[
            Expect::Scan(true, 4, APACC, IR_CAPTURE),
            Expect::Scan(false, 35, 0x1234_5678 << 3 | DRW_WRITE, OK_FAULT),
            xpacc(
                Instruction::Apacc,
                false,
                3,
                0x1234_5678,
                JtagAck::OkFault,
                RESET,
                true,
            ),
            Expect::Scan(false, 4, 0b1110, WAIT),
            xpacc(
                Instruction::Apacc,
                false,
                0,
                0,
                JtagAck::Wait,
                AFTER_WRITE,
                false,
            ),
            Expect::Scan(false, 35, 0x9ABC_DEF0 << 3 | DRW_WRITE, OK_FAULT),
            xpacc(
                Instruction::Apacc,
                false,
                3,
                0x9ABC_DEF0,
                JtagAck::OkFault,
                AFTER_WRITE,
                true,
            ),
        ],
    },
];

/// `cells` as an edge trace sampled at the rise, SWCLK and nRESET high.
pub fn trace(cells: &[Cells]) -> Vec<TraceEvent> {
    let mut out = Vec::new();
    let mut cell = |swdio: bool, tdi: bool, tdo: bool| {
        let lines = Lines::new([true, swdio, tdi, tdo, true]);
        let stamp = out.len() as u64;
        out.push(TraceEvent::Cell {
            host: lines,
            target_sample: lines,
            drives: None,
            stamp,
        });
    };
    for c in cells {
        match *c {
            Cells::Host(v, n) | Cells::Target(v, n) | Cells::Tms(v, n) => {
                (0..n).for_each(|k| cell(v >> k & 1 != 0, true, true))
            }
            Cells::Released(n) => (0..n).for_each(|_| cell(true, true, true)),
            Cells::Shift(tdi, tdo, n) => {
                (0..n).for_each(|k| cell(k + 1 == n, tdi >> k & 1 != 0, tdo >> k & 1 != 0))
            }
        }
    }
    out
}
