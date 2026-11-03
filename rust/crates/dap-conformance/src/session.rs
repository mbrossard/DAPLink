//! Seeded CMSIS-DAP command streams and the targets they run against.

use std::fmt;

use crate::target::{DpVersion, Mode, Target, FAULT_BASE};

/// One `DAP_Transfer` item: the request byte and its word, if it carries one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Item {
    /// The transfer request byte.
    pub req: u8,
    /// Write value, match value or match mask.
    pub word: Option<u32>,
}

/// One CMSIS-DAP command.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Cmd {
    /// `DAP_Connect` with a port byte.
    Connect(u8),
    /// `DAP_Disconnect`.
    Disconnect,
    /// `DAP_SWJ_Pins`: output, select, wait.
    Pins(u8, u8, u32),
    /// `DAP_SWJ_Clock`.
    Clock(u32),
    /// `DAP_SWJ_Sequence`: bit count (0 is 256) and data.
    SwjSequence(u8, Vec<u8>),
    /// `DAP_SWD_Sequence`: info bytes, each with its output data.
    SwdSequence(Vec<(u8, Vec<u8>)>),
    /// `DAP_TransferConfigure`: idle cycles, WAIT retries, match retries.
    Configure(u8, u16, u16),
    /// `DAP_Transfer`: DAP index and items.
    Transfer(u8, Vec<Item>),
    /// `DAP_TransferBlock`: DAP index, count, request byte, write data.
    Block(u8, u16, u8, Vec<u32>),
    /// `DAP_WriteABORT`: DAP index and word.
    Abort(u8, u32),
    /// `DAP_JTAG_Sequence`: info bytes, each with its TDI data.
    JtagSequence(Vec<(u8, Vec<u8>)>),
    /// `DAP_JTAG_Configure`: IR lengths.
    JtagConfigure(Vec<u8>),
    /// `DAP_JTAG_IDCODE`: TAP index.
    Idcode(u8),
}

impl Cmd {
    /// The command's report bytes.
    pub fn encode(&self) -> Vec<u8> {
        let mut r = Vec::new();
        match self {
            Cmd::Connect(port) => r.extend([0x02, *port]),
            Cmd::Disconnect => r.push(0x03),
            Cmd::Pins(out, sel, wait) => {
                r.extend([0x10, *out, *sel]);
                r.extend(wait.to_le_bytes());
            }
            Cmd::Clock(hz) => {
                r.push(0x11);
                r.extend(hz.to_le_bytes());
            }
            Cmd::SwjSequence(n, data) => {
                r.extend([0x12, *n]);
                r.extend(data);
            }
            Cmd::SwdSequence(seqs) => {
                r.extend([0x1D, seqs.len() as u8]);
                for (info, data) in seqs {
                    r.push(*info);
                    r.extend(data);
                }
            }
            Cmd::Configure(idle, wait, retries) => {
                r.extend([0x04, *idle]);
                r.extend(wait.to_le_bytes());
                r.extend(retries.to_le_bytes());
            }
            Cmd::Transfer(idx, items) => {
                r.extend([0x05, *idx, items.len() as u8]);
                for item in items {
                    r.push(item.req);
                    if let Some(w) = item.word {
                        r.extend(w.to_le_bytes());
                    }
                }
            }
            Cmd::Block(idx, count, req, data) => {
                r.extend([0x06, *idx]);
                r.extend(count.to_le_bytes());
                r.push(*req);
                for w in data {
                    r.extend(w.to_le_bytes());
                }
            }
            Cmd::Abort(idx, word) => {
                r.extend([0x08, *idx]);
                r.extend(word.to_le_bytes());
            }
            Cmd::JtagSequence(seqs) => {
                r.extend([0x14, seqs.len() as u8]);
                for (info, data) in seqs {
                    r.push(*info);
                    r.extend(data);
                }
            }
            Cmd::JtagConfigure(lengths) => {
                r.extend([0x15, lengths.len() as u8]);
                r.extend(lengths);
            }
            Cmd::Idcode(idx) => r.extend([0x16, *idx]),
        }
        r
    }

    /// The command's parts a shrink may drop one at a time, as fewer-part copies.
    pub fn smaller(&self) -> Vec<Cmd> {
        let mut out = Vec::new();
        match self {
            Cmd::Transfer(idx, items) if items.len() > 1 => {
                for i in 0..items.len() {
                    let mut v = items.clone();
                    v.remove(i);
                    out.push(Cmd::Transfer(*idx, v));
                }
            }
            Cmd::Block(idx, count, req, data) if *count > 1 => {
                let data = if data.is_empty() {
                    Vec::new()
                } else {
                    data[..data.len() - 1].to_vec()
                };
                out.push(Cmd::Block(
                    *idx,
                    1,
                    *req,
                    data.iter().take(1).copied().collect(),
                ));
                out.push(Cmd::Block(*idx, count - 1, *req, data));
            }
            Cmd::SwdSequence(seqs) | Cmd::JtagSequence(seqs) if seqs.len() > 1 => {
                for i in 0..seqs.len() {
                    let mut v = seqs.clone();
                    v.remove(i);
                    out.push(match self {
                        Cmd::SwdSequence(_) => Cmd::SwdSequence(v),
                        _ => Cmd::JtagSequence(v),
                    });
                }
            }
            _ => {}
        }
        out
    }
}

/// A target and the commands run against it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Stream {
    /// The target both engines start from.
    pub target: Target,
    /// The commands, in order.
    pub cmds: Vec<Cmd>,
}

impl fmt::Display for Stream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "target: {:?}", self.target)?;
        for (i, c) in self.cmds.iter().enumerate() {
            let hex: Vec<String> = c.encode().iter().map(|b| format!("{b:02x}")).collect();
            writeln!(f, "  [{i}] {} ({c:?})", hex.join(" "))?;
        }
        Ok(())
    }
}

/// Which protocols a stream connects with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wire {
    /// SWD only.
    Swd,
    /// JTAG only.
    Jtag,
    /// Either, per stream.
    Both,
}

/// Relative weights of the command kinds a stream draws from; zero leaves a kind out.
#[derive(Clone, Copy, Debug)]
pub struct Profile {
    /// The protocols in play.
    pub wire: Wire,
    /// Arbitrary bit sequences, pin selects and missing preambles, not only valid protocol.
    pub chaos: bool,
    /// `DAP_Connect`.
    pub connect: u32,
    /// `DAP_Disconnect`.
    pub disconnect: u32,
    /// `DAP_SWJ_Pins`.
    pub pins: u32,
    /// `DAP_SWJ_Clock`.
    pub clock: u32,
    /// `DAP_SWJ_Sequence`.
    pub swj_sequence: u32,
    /// `DAP_SWD_Sequence`.
    pub swd_sequence: u32,
    /// `DAP_TransferConfigure`.
    pub configure: u32,
    /// `DAP_Transfer`.
    pub transfer: u32,
    /// `DAP_TransferBlock`.
    pub block: u32,
    /// `DAP_WriteABORT`.
    pub abort: u32,
    /// `DAP_JTAG_Sequence`, `DAP_JTAG_Configure` and `DAP_JTAG_IDCODE`.
    pub jtag: u32,
    /// Value-match and match-mask items within transfers, per sixteen items.
    pub matches: u32,
    /// Read data the target sometimes sends with bad parity.
    pub parity: bool,
    /// The read value sent that way: DPIDR, the first read of every SWD preamble, by default.
    pub poison: u32,
    /// Every target an ADIv6 DP.
    pub adiv6: bool,
    /// The JTAG-DP's DP version when not `adiv6`.
    pub jtag_version: DpVersion,
    /// No target on the chain the host configures.
    pub absent: bool,
    /// Longest stream after the preamble.
    pub length: usize,
}

impl Profile {
    /// Valid SWD sessions: every SWD command, pins limited to nRESET.
    pub const SWD: Profile = Profile {
        wire: Wire::Swd,
        chaos: false,
        connect: 2,
        disconnect: 2,
        pins: 2,
        clock: 3,
        swj_sequence: 2,
        swd_sequence: 2,
        configure: 3,
        transfer: 12,
        block: 12,
        abort: 2,
        jtag: 0,
        matches: 2,
        parity: false,
        poison: crate::target::DPIDR,
        adiv6: false,
        jtag_version: DpVersion::V0,
        absent: false,
        length: 16,
    };

    /// Valid SWD sessions with bad parity on powered-up CTRL/STAT, which no preamble reads.
    pub const SWD_CTRL_PARITY: Profile = Profile {
        parity: true,
        poison: 0xF000_0000,
        ..Profile::SWD
    };

    /// Valid JTAG sessions: every JTAG command, pins limited to nRESET.
    pub const JTAG: Profile = Profile {
        wire: Wire::Jtag,
        swd_sequence: 0,
        jtag: 4,
        ..Profile::SWD
    };

    /// Valid JTAG sessions with an ADIv6 DP: JTAG DP protocol 1.
    pub const JTAG_ADIV6: Profile = Profile {
        adiv6: true,
        ..Profile::JTAG
    };

    /// Valid JTAG sessions with a DPv1 JTAG-DP: DPIDR at `0x0`, SELECT write-only.
    pub const JTAG_DPV1: Profile = Profile {
        jtag_version: DpVersion::V1,
        ..Profile::JTAG
    };

    /// JTAG sessions with nothing on the chain: every scan reads all ones.
    pub const JTAG_ABSENT: Profile = Profile {
        absent: true,
        ..Profile::JTAG
    };

    /// Anything goes: both protocols, arbitrary sequences and pins, streams without preambles.
    pub const CHAOS: Profile = Profile {
        wire: Wire::Both,
        chaos: true,
        pins: 3,
        swj_sequence: 3,
        jtag: 4,
        ..Profile::SWD
    };
}

/// SplitMix64, the whole generator.
pub struct Rng(u64);

impl Rng {
    /// A generator for `seed`.
    pub fn new(seed: u64) -> Self {
        Self(seed ^ 0xD1B5_4A32_D192_ED03)
    }

    /// The next 64 random bits.
    pub fn bits(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut x = self.0;
        x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        x ^ (x >> 31)
    }

    /// Uniform in `0..n`; zero when `n` is zero.
    pub fn below(&mut self, n: u64) -> u64 {
        if n == 0 {
            0
        } else {
            self.bits() % n
        }
    }

    /// `true` one time in `n`.
    pub fn one_in(&mut self, n: u64) -> bool {
        self.below(n) == 0
    }

    /// One of `xs`.
    pub fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
        xs[self.below(xs.len() as u64) as usize]
    }
}

/// The SWD select: line reset, JTAG-to-SWD, line reset, idle.
pub fn swd_select() -> Cmd {
    let mut d = vec![0xFF; 7];
    d.extend([0x9E, 0xE7]);
    d.extend([0xFF; 7]);
    d.push(0x00);
    Cmd::SwjSequence(136, d)
}

/// The JTAG select: line reset, SWD-to-JTAG, then TMS high long enough to reset the TAP.
pub fn jtag_select() -> Cmd {
    let mut d = vec![0xFF; 7];
    d.extend([0x3C, 0xE7]);
    d.push(0xFF);
    Cmd::SwjSequence(80, d)
}

/// Words 1, 2, 3 written from `0x2000_0000`, then DRW, incrementing, polled for 3 with `retries`.
pub fn drw_poll(retries: u16) -> Stream {
    let w = |req, word| Item {
        req,
        word: Some(word),
    };
    let dpidr = Item {
        req: 0x02,
        word: None,
    };
    let tar = w(0x05, 0x2000_0000);
    let setup = vec![
        dpidr,
        w(0x04, 0x5000_0000),
        w(0x08, 0),
        w(0x01, 0x2300_0012),
        tar,
        w(0x0D, 1),
        w(0x0D, 2),
        w(0x0D, 3),
        tar,
    ];
    Stream {
        target: Target::default(),
        cmds: vec![
            Cmd::Connect(1),
            swd_select(),
            Cmd::Transfer(0, setup),
            Cmd::Configure(0, 5, retries),
            Cmd::Transfer(0, vec![w(0x20, u32::MAX), w(0x1F, 3)]),
        ],
    }
}

/// A DRW read whose posted bus read faults, then transfers and blocks against the sticky FAULT.
pub fn sticky_fault() -> Stream {
    let w = |req, word| Item {
        req,
        word: Some(word),
    };
    let r = |req| Item { req, word: None };
    let setup = vec![
        r(0x02),
        w(0x04, 0x5000_0000),
        w(0x08, 0),
        w(0x01, 0x2300_0012),
        w(0x05, FAULT_BASE),
        r(0x0F),
    ];
    Stream {
        target: Target::default(),
        cmds: vec![
            Cmd::Connect(1),
            swd_select(),
            Cmd::Transfer(0, setup),
            Cmd::Transfer(0, vec![r(0x0F)]),
            Cmd::Transfer(0, vec![w(0x20, 0)]),
            Cmd::Block(0, 0, 0x0F, vec![]),
            Cmd::Block(0, 2, 0x0D, vec![1, 2]),
            Cmd::Block(0, 2, 0x0F, vec![]),
        ],
    }
}

/// A line reset and two idle bytes.
fn line_reset() -> Cmd {
    let mut d = vec![0xFF; 7];
    d.extend([0, 0]);
    Cmd::SwjSequence(72, d)
}

/// Test-Logic-Reset, then Run-Test/Idle.
fn tap_reset() -> Cmd {
    Cmd::JtagSequence(vec![(0x46, vec![0x3F]), (0x01, vec![0x00])])
}

/// Run-Test/Idle to Shift-DR, `n` captured bits the last of which exits, then back to idle.
fn dr_scan(n: u8) -> Cmd {
    Cmd::JtagSequence(vec![
        (0x41, vec![0xFF]),
        (0x02, vec![0x00]),
        (0x80 | (n - 1), vec![0xFF; usize::from(n - 1).div_ceil(8)]),
        (0xC1, vec![0xFF]),
        (0x41, vec![0xFF]),
        (0x01, vec![0x00]),
    ])
}

const CLOCKS: [u32; 10] = [
    100_000, 400_000, 1_000_000, 2_000_000, 4_000_000, 8_000_000, 12_000_000, 24_000_000,
    50_000_000, 0,
];
const TARS: [u32; 6] = [
    0x2000_0000,
    0x2000_0040,
    0x2000_03F8,
    0x2000_1000,
    FAULT_BASE + 0x10,
    FAULT_BASE,
];
fn target(rng: &mut Rng, jtag: bool) -> Target {
    let (chain, adi) = if jtag {
        match rng.below(4) {
            0 => (vec![4], 0),
            1 => (vec![5, 4], 1),
            2 => (vec![4, 5], 0),
            _ => (vec![5, 4, 7], 1),
        }
    } else {
        (vec![4], 0)
    };
    let waits = (0..rng.below(24))
        .map(|_| match rng.below(20) {
            0..=13 => 0,
            14..=16 => 1,
            17 | 18 => rng.below(4) as u8 + 2,
            _ => rng.below(8) as u8 + 5,
        })
        .collect();
    Target {
        mode: if rng.one_in(2) { Mode::Swd } else { Mode::Jtag },
        chain,
        adi,
        waits,
        poison: None,
        seed: rng.bits(),
        adiv6: false,
        jtag_version: DpVersion::V0,
        absent: false,
    }
}

fn preamble(rng: &mut Rng, t: &Target, jtag: bool) -> Vec<Cmd> {
    let w = |req, word| Item {
        req,
        word: Some(word),
    };
    if jtag {
        vec![
            Cmd::Connect(2),
            Cmd::Clock(rng.pick(&CLOCKS[..9])),
            jtag_select(),
            tap_reset(),
            Cmd::JtagConfigure(t.chain.clone()),
            Cmd::Transfer(
                t.adi as u8,
                vec![w(0x04, 0x5000_0020), w(0x08, 0), w(0x01, 0x2300_0012)],
            ),
        ]
    } else {
        vec![
            Cmd::Connect(rng.pick(&[0, 1])),
            Cmd::Clock(rng.pick(&CLOCKS[..9])),
            swd_select(),
            Cmd::Transfer(
                0,
                vec![
                    Item {
                        req: 0x02,
                        word: None,
                    },
                    w(0x00, 0x1E),
                    w(0x04, 0x5000_0000),
                    w(0x08, 0),
                    w(0x01, 0x2300_0012),
                ],
            ),
        ]
    }
}

fn item(rng: &mut Rng, profile: &Profile, jtag: bool) -> Item {
    let w = |word| Item {
        req: 0,
        word: Some(word),
    };
    let r = |req| Item { req, word: None };
    let mut it = match rng.below(16) {
        // Over JTAG the DP version decides what 0x8 reads.
        0 if jtag => r(rng.pick(&[0x02, 0x0A])),
        0 => r(0x02),
        1 => r(0x06),
        2 => r(0x0E),
        3 => Item {
            req: 0x08,
            ..w(rng.pick(&[0, 0xF0, 0x0100_0000]))
        },
        4 => Item {
            req: 0x04,
            ..w(rng.pick(&[0x5000_0000, 0, 0x5000_0020]))
        },
        5 => Item {
            req: 0x00,
            ..w(rng.pick(&[0x1E, 0x04, 0x01]))
        },
        6 => r(0x03),
        7 => r(0x07),
        8..=10 => r(0x0F),
        11 => Item {
            req: 0x01,
            ..w(rng.pick(&[0x2300_0012, 0x2300_0002]))
        },
        12 | 13 => Item {
            req: 0x05,
            ..w(rng.pick(&TARS))
        },
        14 => Item {
            req: 0x0D,
            ..w(rng.bits() as u32)
        },
        _ => r(rng.pick(&[0x03, 0x07, 0x0B, 0x0F])),
    };
    if profile.matches > 0 && rng.below(16 * 4) < u64::from(profile.matches) * 4 {
        it = if rng.one_in(2) {
            Item {
                req: 0x20,
                word: Some(rng.pick(&[u32::MAX, 0xF000_0000, 0xFF, 0])),
            }
        } else {
            Item {
                req: rng.pick(&[0x12, 0x16, 0x1F, 0x13]),
                word: Some(rng.pick(&[0, 0x5000_0000, 0xF000_0000, DPIDR_WORD])),
            }
        };
    }
    it
}

const DPIDR_WORD: u32 = crate::target::DPIDR;

fn transfer(rng: &mut Rng, profile: &Profile, jtag: bool, idx: u8) -> Cmd {
    let mut items = Vec::new();
    let n = 1 + rng.below(12) as usize;
    while items.len() < n {
        let it = item(rng, profile, jtag);
        let run = if it.req & 0x13 == 0x03 && rng.one_in(2) {
            1 + rng.below(6) as usize
        } else {
            1
        };
        items.extend(std::iter::repeat_n(it, run));
    }
    items.truncate(n.max(1));
    Cmd::Transfer(idx, items)
}

fn block(rng: &mut Rng, idx: u8) -> Cmd {
    let count = match rng.below(20) {
        0 => 0,
        1..=8 => 1 + rng.below(4) as u16,
        9..=18 => 5 + rng.below(28) as u16,
        _ => 33 + rng.below(90) as u16,
    };
    let req = rng.pick(&[
        0x0F, 0x0F, 0x0F, 0x0D, 0x0D, 0x0E, 0x06, 0x02, 0x13, 0x07, 0x05, 0x08, 0x2F, 0xFF,
    ]);
    let data = if req & 2 == 0 {
        (0..count).map(|_| rng.bits() as u32).collect()
    } else {
        Vec::new()
    };
    Cmd::Block(idx, count, req, data)
}

fn bits(rng: &mut Rng, n: usize) -> Vec<u8> {
    (0..n.div_ceil(8)).map(|_| rng.bits() as u8).collect()
}

fn command(rng: &mut Rng, p: &Profile, t: &Target, jtag: bool) -> Cmd {
    let swd_only = |w: u32| if jtag && !p.chaos { 0 } else { w };
    let jtag_only = |w: u32| if jtag || p.chaos { w } else { 0 };
    let weights = [
        p.connect,
        p.disconnect,
        p.pins,
        p.clock,
        p.swj_sequence,
        swd_only(p.swd_sequence),
        p.configure,
        p.transfer,
        p.block,
        p.abort,
        jtag_only(p.jtag),
    ];
    let mut roll = rng.below(u64::from(weights.iter().sum::<u32>()));
    let mut kind = 0;
    for (i, &w) in weights.iter().enumerate() {
        if roll < u64::from(w) {
            kind = i;
            break;
        }
        roll -= u64::from(w);
    }
    let idx = if rng.one_in(8) {
        rng.below(4) as u8
    } else {
        t.adi as u8
    };
    match kind {
        0 if p.chaos => Cmd::Connect(rng.pick(&[0, 1, 1, 2, 3])),
        0 if jtag => Cmd::Connect(2),
        0 => Cmd::Connect(rng.pick(&[0, 1])),
        1 => Cmd::Disconnect,
        2 if p.chaos => Cmd::Pins(
            rng.bits() as u8,
            rng.pick(&[0x01, 0x02, 0x03, 0x80, 0x04, 0x83, 0x8F, 0xFF, 0x00]),
            0,
        ),
        2 => Cmd::Pins(rng.pick(&[0x00, 0x7F]), 0x80, 0),
        3 => Cmd::Clock(if rng.one_in(4) {
            rng.below(60_000_000) as u32
        } else {
            rng.pick(&CLOCKS)
        }),
        4 if p.chaos => match rng.below(5) {
            0 => swd_select(),
            1 => jtag_select(),
            2 => Cmd::SwjSequence(51, vec![0xFF; 7]),
            3 => Cmd::SwjSequence(8, vec![0]),
            _ => {
                let n = 1 + rng.below(64) as usize;
                Cmd::SwjSequence(n as u8, bits(rng, n))
            }
        },
        4 if jtag => jtag_select(),
        4 => rng.pick(&[swd_select, line_reset])(),
        5 if p.chaos => Cmd::SwdSequence(
            (0..1 + rng.below(3))
                .map(|_| {
                    let n = rng.below(64) as u8;
                    let len = if n == 0 { 64 } else { n as usize };
                    if rng.one_in(2) {
                        (0x80 | n, Vec::new())
                    } else {
                        (n, bits(rng, len))
                    }
                })
                .collect(),
        ),
        5 => Cmd::SwdSequence(vec![(0, vec![0xFF; 8]), (16, vec![0, 0])]),
        6 => Cmd::Configure(
            rng.below(4) as u8,
            rng.pick(&[0, 1, 2, 3, 5, 5, 8, 100]),
            rng.pick(&[0, 1, 3, 8]),
        ),
        7 => transfer(rng, p, jtag, idx),
        8 => block(rng, idx),
        9 => Cmd::Abort(idx, rng.pick(&[0x1E, 0x04, 0x01, 0x10])),
        _ if p.chaos => match rng.below(4) {
            0 => Cmd::JtagConfigure(if rng.one_in(3) {
                vec![rng.below(8) as u8 + 1; rng.below(3) as usize + 1]
            } else {
                t.chain.clone()
            }),
            1 => Cmd::Idcode(idx),
            _ => Cmd::JtagSequence(
                (0..1 + rng.below(3))
                    .map(|_| {
                        let info = rng.bits() as u8;
                        let n = match info & 0x3F {
                            0 => 64,
                            n => n as usize,
                        };
                        (info, bits(rng, n))
                    })
                    .collect(),
            ),
        },
        _ => match rng.below(5) {
            0 => Cmd::JtagConfigure(t.chain.clone()),
            1 | 2 => Cmd::Idcode(rng.below(t.chain.len() as u64) as u8),
            3 => tap_reset(),
            _ => dr_scan(rng.pick(&[32, 33, 40])),
        },
    }
}

/// The stream for `seed`: a target, a preamble that connects to it, then commands.
pub fn stream(seed: u64, profile: &Profile) -> Stream {
    let mut rng = Rng::new(seed);
    let jtag = match profile.wire {
        Wire::Swd => false,
        Wire::Jtag => true,
        Wire::Both => rng.one_in(3),
    };
    let mut t = target(&mut rng, jtag);
    t.adiv6 = profile.adiv6;
    t.jtag_version = profile.jtag_version;
    t.absent = profile.absent;
    if profile.parity && rng.one_in(4) {
        t.poison = Some(profile.poison);
    }
    let mut cmds = if profile.chaos && rng.one_in(8) {
        Vec::new()
    } else {
        preamble(&mut rng, &t, jtag)
    };
    let n = 1 + rng.below(profile.length as u64) as usize;
    for _ in 0..n {
        let cmd = command(&mut rng, profile, &t, jtag);
        // Outside chaos a reset pulse is always released by the next command.
        let pulse = !profile.chaos && matches!(cmd, Cmd::Pins(_, 0x80, _));
        cmds.push(cmd);
        if pulse {
            cmds.push(Cmd::Pins(0x80, 0x80, 0));
        }
    }
    Stream { target: t, cmds }
}
