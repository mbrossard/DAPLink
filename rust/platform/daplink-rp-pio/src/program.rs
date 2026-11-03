//! The generic SWD and JTAG PIO programs, assembled by the const encoder in `asm`.

use crate::asm::{clear_isr, in_pins, jmp, nop, out, pull, push, Cond, Dest, SideSet};
use crate::clock::{jtag_timing, swd_cycles_per_bit, JtagTiming};

/// Which assembled variant a wiring uses.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Variant {
    /// Single (or split-readback) SWDIO, 1-bit SWCLK side-set.
    Direct,
    /// Direction-controlled level shifter, 2-bit `{OEn, SWCLK}` side-set.
    Oen,
}

/// Entry-point offsets within a program, relative to its load offset.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ProgramLabels {
    /// Released turnaround loop entry; no in-tree engine dispatches to it.
    pub turnaround_cmd: u8,
    /// Driven write loop entry.
    pub write_cmd: u8,
    /// Dispatcher / park site; the SM waits here with SWCLK low when the FIFO is empty.
    pub get_next_cmd: u8,
    /// Read loop entry.
    pub read_cmd: u8,
    /// Completion barrier entry used by an ordered release.
    pub release_cmd: u8,
}

/// Absolute entry PCs after loading at a known offset; these go in the command word.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ProgramEntries {
    /// Absolute address of the released turnaround loop entry; no in-tree engine uses it.
    pub turnaround_cmd: u8,
    /// Absolute address of the driven write loop entry.
    pub write_cmd: u8,
    /// Absolute address of the dispatcher / park site.
    pub get_next_cmd: u8,
    /// Absolute address of the read loop entry.
    pub read_cmd: u8,
    /// Absolute address of the completion barrier entry.
    pub release_cmd: u8,
}

/// Absolute entry PCs for the JTAG dispatcher and its two fixed-TMS loops.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct JtagProgramEntries {
    /// Dispatcher / park site; it leaves the last TCK/TMS levels unchanged while stalled.
    pub get_next_cmd: u8,
    /// TMS-low entry that pulls a separate data word.
    pub tms_low_data_cmd: u8,
    /// TMS-low entry that shifts data embedded in the command word.
    pub tms_low_inline_cmd: u8,
    /// TMS-high entry that pulls a separate data word.
    pub tms_high_data_cmd: u8,
    /// TMS-high entry that shifts data embedded in the command word.
    pub tms_high_inline_cmd: u8,
}

/// One assembled variant: instruction words plus the metadata the state machine set-up needs.
pub struct PioProgram {
    /// The 16-bit instruction words, in load order.
    pub instructions: &'static [u16],
    /// Wrap-target offset (execution wraps back here).
    pub wrap_target: u8,
    /// Wrap offset (the last instruction before wrapping).
    pub wrap: u8,
    /// Side-set data bits, excluding the `opt` enable bit.
    pub side_set_bits: u8,
    /// Whether side-set is optional; always true for both variants.
    pub side_set_opt: bool,
    /// Entry-point offsets relative to the load offset.
    pub labels: ProgramLabels,
    /// Which variant this is.
    pub variant: Variant,
}

/// The PIO JTAG program loaded after the direct SWD program.
pub struct JtagProgram {
    /// The 16-bit instruction words, already relocated for their fixed load offset.
    pub instructions: &'static [u16],
    /// First instruction-memory slot occupied by this program.
    pub load_offset: u8,
    /// Wrap-target address.
    pub wrap_target: u8,
    /// Wrap address.
    pub wrap: u8,
    /// Side-set data bits, excluding the optional enable bit.
    pub side_set_bits: u8,
    /// Absolute entry PCs.
    pub entries: JtagProgramEntries,
}

impl PioProgram {
    /// The absolute entry PCs; internal jumps require a zero load offset.
    pub const fn entries(&self, load_offset: u8) -> ProgramEntries {
        assert!(load_offset == 0);
        ProgramEntries {
            turnaround_cmd: load_offset + self.labels.turnaround_cmd,
            write_cmd: load_offset + self.labels.write_cmd,
            get_next_cmd: load_offset + self.labels.get_next_cmd,
            read_cmd: load_offset + self.labels.read_cmd,
            release_cmd: load_offset + self.labels.release_cmd,
        }
    }
}

const SS1: SideSet = SideSet { data_bits: 1 };

/// Single bidirectional (or split drive/readback) SWDIO, 1-bit side-set on SWCLK.
pub static DIRECT: PioProgram = PioProgram {
    instructions: &DIRECT_WORDS,
    wrap_target: 3,
    wrap: 10,
    side_set_bits: 1,
    side_set_opt: true,
    labels: ProgramLabels {
        turnaround_cmd: 0,
        write_cmd: 0,
        get_next_cmd: 3,
        read_cmd: 8,
        release_cmd: 11,
    },
    variant: Variant::Direct,
};

static DIRECT_WORDS: [u16; 13] = direct_words(1, 0);
static DIRECT_STRETCHED_WORDS: [u16; 13] = direct_words(2, 1);

const fn direct_words(low_delay: u8, read_nop_delay: u8) -> [u16; 13] {
    [
        // 0 write_cmd / turnaround_cmd: pull the data (or dummy 0 for high-Z).
        pull(SS1, None, 0),
        // 1 write_bitloop: drive the bit with SWCLK low.
        out(SS1, Dest::Pins, 1, Some(0), low_delay),
        // 2 jmp x-- write_bitloop [1] side 1  (SWCLK rises, target samples).
        jmp(SS1, Cond::XPostDec, 1, Some(1), 1),
        // 3 get_next_cmd: pull side 0  (park site, SWCLK low).
        pull(SS1, Some(0), 0),
        // 4 out x,8  (X := bit_count - 1).
        out(SS1, Dest::X, 8, None, 0),
        // 5 out pindirs,1  (SWDIO direction).
        out(SS1, Dest::PinDirs, 1, None, 0),
        // 6 out pc,5  (dispatch to the entry PC).
        out(SS1, Dest::Pc, 5, None, 0),
        // 7 read_bitloop: low-phase delay slot on the taken loop branch.
        nop(SS1, None, read_nop_delay),
        // 8 read_cmd: in pins,1 [1] side 1  (SWCLK rises + sample).
        in_pins(SS1, 1, Some(1), 1),
        // 9 jmp x-- read_bitloop side 0  (SWCLK falls).
        jmp(SS1, Cond::XPostDec, 7, Some(0), 0),
        // 10 push  (result word after the last bit).
        push(SS1, None, 0),
        // 11 release_cmd: push completion after the dispatcher applies released direction.
        push(SS1, None, 0),
        // 12 return to the dispatcher without entering the normal wrap range.
        jmp(SS1, Cond::Always, 3, None, 0),
    ]
}

static DIRECT_STRETCHED: PioProgram = PioProgram {
    instructions: &DIRECT_STRETCHED_WORDS,
    wrap_target: DIRECT.wrap_target,
    wrap: DIRECT.wrap,
    side_set_bits: DIRECT.side_set_bits,
    side_set_opt: DIRECT.side_set_opt,
    labels: DIRECT.labels,
    variant: Variant::Direct,
};

const SS2: SideSet = SideSet { data_bits: 2 };

/// Direction-controlled level shifter, 2-bit side-set on `{OEn (bit 0), SWCLK (bit 1)}`.
pub static OEN: PioProgram = PioProgram {
    instructions: &OEN_WORDS,
    wrap_target: 7,
    wrap: 14,
    side_set_bits: 2,
    side_set_opt: true,
    labels: ProgramLabels {
        turnaround_cmd: 0,
        write_cmd: 4,
        get_next_cmd: 7,
        read_cmd: 12,
        release_cmd: 15,
    },
    variant: Variant::Oen,
};

static OEN_WORDS: [u16; 17] = oen_words(1, 0);
static OEN_STRETCHED_WORDS: [u16; 17] = oen_words(2, 1);

const fn oen_words(low_delay: u8, read_nop_delay: u8) -> [u16; 17] {
    [
        // 0 turnaround_cmd: pull  (dummy data word, pairing kept uniform).
        pull(SS2, None, 0),
        // 1 turnaround_bitloop: OEn released, SWCLK low.
        nop(SS2, Some(0x1), low_delay),
        // 2 jmp x-- turnaround_bitloop [1] side 0x3  (OEn released, SWCLK high).
        jmp(SS2, Cond::XPostDec, 1, Some(0x3), 1),
        // 3 jmp get_next_cmd.
        jmp(SS2, Cond::Always, 7, None, 0),
        // 4 write_cmd: pull.
        pull(SS2, None, 0),
        // 5 write_bitloop: driving, SWCLK low.
        out(SS2, Dest::Pins, 1, Some(0x0), low_delay),
        // 6 jmp x-- write_bitloop [1] side 0x2  (driving, SWCLK high).
        jmp(SS2, Cond::XPostDec, 5, Some(0x2), 1),
        // 7 get_next_cmd: pull side 0x1  (park site, SWCLK low, OEn released).
        pull(SS2, Some(0x1), 0),
        // 8 out x,8.
        out(SS2, Dest::X, 8, None, 0),
        // 9 out pindirs,1.
        out(SS2, Dest::PinDirs, 1, None, 0),
        // 10 out pc,5.
        out(SS2, Dest::Pc, 5, None, 0),
        // 11 read_bitloop: low-phase delay slot.
        nop(SS2, None, read_nop_delay),
        // 12 read_cmd: in pins,1 [1] side 0x3  (OEn released, SWCLK rises + sample).
        in_pins(SS2, 1, Some(0x3), 1),
        // 13 jmp x-- read_bitloop side 0x1  (OEn released, SWCLK falls).
        jmp(SS2, Cond::XPostDec, 11, Some(0x1), 0),
        // 14 push.
        push(SS2, None, 0),
        // 15 release_cmd: push completion after the dispatcher applies released direction.
        push(SS2, None, 0),
        // 16 return to the released dispatcher without entering the normal wrap range.
        jmp(SS2, Cond::Always, 7, None, 0),
    ]
}

static OEN_STRETCHED: PioProgram = PioProgram {
    instructions: &OEN_STRETCHED_WORDS,
    wrap_target: OEN.wrap_target,
    wrap: OEN.wrap,
    side_set_bits: OEN.side_set_bits,
    side_set_opt: OEN.side_set_opt,
    labels: OEN.labels,
    variant: Variant::Oen,
};

const JTAG_OFFSET: u8 = DIRECT_WORDS.len() as u8;
const JTAG_GET_NEXT: u8 = JTAG_OFFSET;
const JTAG_TMS_LOW_DATA: u8 = JTAG_OFFSET + 4;
const JTAG_TMS_LOW_INLINE: u8 = JTAG_OFFSET + 5;
const JTAG_TMS_HIGH_DATA: u8 = JTAG_OFFSET + 10;
const JTAG_TMS_HIGH_INLINE: u8 = JTAG_OFFSET + 11;
const JTAG_FINISH: u8 = JTAG_OFFSET + 15;
const JTAG_DISCARD: u8 = JTAG_OFFSET + 18;
const JTAG_NOP_TMS_LOW: u8 = 0b01;
const JTAG_NOP_TMS_HIGH: u8 = 0b11;

static JTAG_125MHZ: JtagProgram = JtagProgram {
    instructions: &JTAG_125MHZ_WORDS,
    load_offset: JTAG_OFFSET,
    wrap_target: JTAG_GET_NEXT,
    wrap: JTAG_OFFSET + JTAG_125MHZ_WORDS.len() as u8 - 1,
    side_set_bits: 2,
    entries: JtagProgramEntries {
        get_next_cmd: JTAG_GET_NEXT,
        tms_low_data_cmd: JTAG_TMS_LOW_DATA,
        tms_low_inline_cmd: JTAG_TMS_LOW_INLINE,
        tms_high_data_cmd: JTAG_TMS_HIGH_DATA,
        tms_high_inline_cmd: JTAG_TMS_HIGH_INLINE,
    },
};

static JTAG_150MHZ: JtagProgram = JtagProgram {
    instructions: &JTAG_150MHZ_WORDS,
    load_offset: JTAG_125MHZ.load_offset,
    wrap_target: JTAG_125MHZ.wrap_target,
    wrap: JTAG_125MHZ.wrap,
    side_set_bits: JTAG_125MHZ.side_set_bits,
    entries: JTAG_125MHZ.entries,
};

static JTAG_200MHZ: JtagProgram = JtagProgram {
    instructions: &JTAG_200MHZ_WORDS,
    load_offset: JTAG_125MHZ.load_offset,
    wrap_target: JTAG_125MHZ.wrap_target,
    wrap: JTAG_125MHZ.wrap,
    side_set_bits: JTAG_125MHZ.side_set_bits,
    entries: JTAG_125MHZ.entries,
};

static JTAG_125MHZ_WORDS: [u16; 19] = jtag_words(crate::clock::JTAG_TIMING_125MHZ);
static JTAG_150MHZ_WORDS: [u16; 19] = jtag_words(crate::clock::JTAG_TIMING_150MHZ);
static JTAG_200MHZ_WORDS: [u16; 19] = jtag_words(crate::clock::JTAG_TIMING_200MHZ);

const fn jtag_words(timing: JtagTiming) -> [u16; 19] {
    [
        // 13: count - 1, capture, entry PC and up to 21 TDI bits share one command word.
        pull(SS2, None, 0),
        out(SS2, Dest::X, 5, None, 0),
        out(SS2, Dest::Y, 1, None, 0),
        out(SS2, Dest::Pc, 5, None, 0),
        // 17: TMS-low external-data entry and bit loop.
        pull(SS2, None, 0),
        out(SS2, Dest::Pins, 1, Some(0b00), timing.output_delay()),
        nop(SS2, Some(JTAG_NOP_TMS_LOW), timing.loop_high_delay()),
        in_pins(SS2, 1, Some(0b01), timing.sample_delay()),
        jmp(
            SS2,
            Cond::XPostDec,
            JTAG_TMS_LOW_INLINE,
            Some(0b00),
            timing.loop_low_delay(),
        ),
        jmp(SS2, Cond::Always, JTAG_FINISH, None, 0),
        // 23: TMS-high external-data entry and bit loop.
        pull(SS2, None, 0),
        out(SS2, Dest::Pins, 1, Some(0b10), timing.output_delay()),
        nop(SS2, Some(JTAG_NOP_TMS_HIGH), timing.loop_high_delay()),
        in_pins(SS2, 1, Some(0b11), timing.sample_delay()),
        jmp(
            SS2,
            Cond::XPostDec,
            JTAG_TMS_HIGH_INLINE,
            Some(0b10),
            timing.loop_low_delay(),
        ),
        // 28: Push captured data, or clear the ISR without filling the RX FIFO.
        jmp(SS2, Cond::YZero, JTAG_DISCARD, None, 0),
        push(SS2, None, 0),
        jmp(SS2, Cond::Always, JTAG_GET_NEXT, None, 0),
        // 31: wrap returns to the dispatcher.
        clear_isr(SS2),
    ]
}

const _: () = assert!(DIRECT_WORDS.len() + JTAG_125MHZ_WORDS.len() <= 32);

/// Pick the qualified standard SWD program for a wiring and system clock.
pub const fn select_program(has_oen: bool, sys_hz: u32) -> Option<&'static PioProgram> {
    match swd_cycles_per_bit(sys_hz) {
        Some(4) => Some(if has_oen { &OEN } else { &DIRECT }),
        Some(5) => Some(if has_oen {
            &OEN_STRETCHED
        } else {
            &DIRECT_STRETCHED
        }),
        _ => None,
    }
}

/// Pick the qualified late-sampled JTAG program for a system clock.
pub const fn select_jtag_program(sys_hz: u32) -> Option<&'static JtagProgram> {
    match jtag_timing(sys_hz) {
        Some(crate::clock::JTAG_TIMING_125MHZ) => Some(&JTAG_125MHZ),
        Some(crate::clock::JTAG_TIMING_150MHZ) => Some(&JTAG_150MHZ),
        Some(crate::clock::JTAG_TIMING_200MHZ) => Some(&JTAG_200MHZ),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::asm::OP_OUT;
    use crate::clock::{JTAG_TIMING_125MHZ, JTAG_TIMING_150MHZ, JTAG_TIMING_200MHZ};

    // Golden words hand-computed from the datasheet encoding, so an encoder bug cannot hide.
    #[rustfmt::skip]
    const DIRECT_GOLDEN: [u16; 13] = [
        0x80A0, // 0  pull
        0x7101, // 1  out pins,1 [1] side 0
        0x1941, // 2  jmp x-- 1 [1] side 1
        0x90A0, // 3  pull side 0
        0x6028, // 4  out x,8
        0x6081, // 5  out pindirs,1
        0x60A5, // 6  out pc,5
        0xA042, // 7  nop
        0x5901, // 8  in pins,1 [1] side 1
        0x1047, // 9  jmp x-- 7 side 0
        0x8020, // 10 push
        0x8020, // 11 release completion push
        0x0003, // 12 jmp get_next_cmd (3)
    ];

    #[rustfmt::skip]
    const OEN_GOLDEN: [u16; 17] = [
        0x80A0, // 0  pull
        0xB542, // 1  nop [1] side 0x1
        0x1D41, // 2  jmp x-- 1 [1] side 0x3
        0x0007, // 3  jmp get_next_cmd (7)
        0x80A0, // 4  pull
        0x7101, // 5  out pins,1 [1] side 0
        0x1945, // 6  jmp x-- 5 [1] side 0x2
        0x94A0, // 7  pull side 0x1
        0x6028, // 8  out x,8
        0x6081, // 9  out pindirs,1
        0x60A5, // 10 out pc,5
        0xA042, // 11 nop
        0x5D01, // 12 in pins,1 [1] side 0x3
        0x144B, // 13 jmp x-- 11 side 0x1
        0x8020, // 14 push
        0x8020, // 15 release completion push
        0x0007, // 16 jmp get_next_cmd (7)
    ];

    #[test]
    fn direct_words_match_golden() {
        assert_eq!(DIRECT_WORDS, DIRECT_GOLDEN);
    }

    #[test]
    fn oen_words_match_golden() {
        assert_eq!(OEN_WORDS, OEN_GOLDEN);
    }

    #[test]
    fn direct_metadata() {
        assert_eq!(DIRECT.instructions.len(), 13);
        assert_eq!(DIRECT.wrap_target, 3);
        assert_eq!(DIRECT.wrap, 10);
        assert_eq!(DIRECT.side_set_bits, 1);
        assert!(DIRECT.side_set_opt);
        assert_eq!(
            DIRECT.labels,
            ProgramLabels {
                turnaround_cmd: 0,
                write_cmd: 0,
                get_next_cmd: 3,
                read_cmd: 8,
                release_cmd: 11
            }
        );
    }

    #[test]
    fn oen_metadata() {
        assert_eq!(OEN.instructions.len(), 17);
        assert_eq!(OEN.wrap_target, 7);
        assert_eq!(OEN.wrap, 14);
        assert_eq!(OEN.side_set_bits, 2);
        assert_eq!(
            OEN.labels,
            ProgramLabels {
                turnaround_cmd: 0,
                write_cmd: 4,
                get_next_cmd: 7,
                read_cmd: 12,
                release_cmd: 15
            }
        );
    }

    #[test]
    fn jtag_program_fills_shared_instruction_memory() {
        let program = select_jtag_program(125_000_000).unwrap();
        assert_eq!(program.load_offset, 13);
        assert_eq!(program.instructions.len(), 19);
        assert_eq!(program.wrap_target, 13);
        assert_eq!(program.wrap, 31);
        assert_eq!(program.side_set_bits, 2);
        assert_eq!(
            program.entries,
            JtagProgramEntries {
                get_next_cmd: 13,
                tms_low_data_cmd: 17,
                tms_low_inline_cmd: 18,
                tms_high_data_cmd: 23,
                tms_high_inline_cmd: 24,
            }
        );
    }

    #[test]
    fn jtag_words_match_profile_goldens() {
        #[rustfmt::skip]
        let profiles = [
            (JTAG_TIMING_125MHZ, [
                0x80A0, 0x6025, 0x6041, 0x60A5, 0x80A0,
                0x7101, 0xB442, 0x5401, 0x1052, 0x001C, 0x80A0,
                0x7901, 0xBC42, 0x5C01, 0x1858, 0x007F, 0x8020, 0x000D, 0xA0C3,
            ]),
            (JTAG_TIMING_150MHZ, [
                0x80A0, 0x6025, 0x6041, 0x60A5, 0x80A0,
                0x7101, 0xB542, 0x5401, 0x1052, 0x001C, 0x80A0,
                0x7901, 0xBD42, 0x5C01, 0x1858, 0x007F, 0x8020, 0x000D, 0xA0C3,
            ]),
            (JTAG_TIMING_200MHZ, [
                0x80A0, 0x6025, 0x6041, 0x60A5, 0x80A0,
                0x7101, 0xB642, 0x5401, 0x1152, 0x001C, 0x80A0,
                0x7901, 0xBE42, 0x5C01, 0x1958, 0x007F, 0x8020, 0x000D, 0xA0C3,
            ]),
        ];
        for (timing, golden) in profiles {
            assert_eq!(jtag_words(timing), golden);
        }
        assert_eq!(JTAG_125MHZ_WORDS, jtag_words(JTAG_TIMING_125MHZ));
        assert_eq!(JTAG_150MHZ_WORDS, jtag_words(JTAG_TIMING_150MHZ));
        assert_eq!(JTAG_200MHZ_WORDS, jtag_words(JTAG_TIMING_200MHZ));
    }

    #[test]
    fn jtag_shift_loops_match_every_profile_geometry() {
        let duration = |word: u16| u32::from((word >> 8) & 0x03) + 1;
        for timing in [JTAG_TIMING_125MHZ, JTAG_TIMING_150MHZ, JTAG_TIMING_200MHZ] {
            let words = jtag_words(timing);
            for start in [5, 11] {
                let low = duration(words[start + 3]) + duration(words[start]);
                let high = duration(words[start + 1]) + duration(words[start + 2]);
                assert_eq!((low, high), (timing.low_cycles(), timing.high_cycles()));
                assert_eq!(low + high, timing.cycles_per_bit());
            }
        }
    }

    #[test]
    fn entries_match_absolute_program_addresses() {
        let direct = DIRECT.entries(0);
        assert_eq!(direct.turnaround_cmd, 0);
        assert_eq!(direct.write_cmd, 0);
        assert_eq!(direct.get_next_cmd, 3);
        assert_eq!(direct.read_cmd, 8);
        assert_eq!(direct.release_cmd, 11);

        let oen = OEN.entries(0);
        assert_eq!(oen.turnaround_cmd, 0);
        assert_eq!(oen.write_cmd, 4);
        assert_eq!(oen.get_next_cmd, 7);
        assert_eq!(oen.read_cmd, 12);
        assert_eq!(oen.release_cmd, 15);
    }

    #[test]
    #[should_panic]
    fn nonzero_load_offset_is_rejected() {
        let _ = DIRECT.entries(1);
    }

    #[test]
    fn select_program_picks_variant() {
        assert_eq!(
            select_program(false, 125_000_000).unwrap().variant,
            Variant::Direct
        );
        assert_eq!(
            select_program(true, 200_000_000).unwrap().variant,
            Variant::Oen
        );
        assert!(select_program(false, 160_000_000).is_none());
        assert!(select_jtag_program(160_000_000).is_none());
    }

    #[test]
    fn decode_direct_dispatch_ops() {
        let w = DIRECT_WORDS[4];
        assert_eq!(w >> 13, OP_OUT);
        assert_eq!((w >> 5) & 0x7, Dest::X as u16);
        assert_eq!(w & 0x1f, 8);

        let w = DIRECT_WORDS[6];
        assert_eq!((w >> 5) & 0x7, Dest::Pc as u16);
        assert_eq!(w & 0x1f, 5);
    }

    #[test]
    fn decode_direct_sideset_and_delay() {
        // One data bit plus opt: bit 4 enable, bit 3 side value, bits 2:0 delay.
        let field = (DIRECT_WORDS[1] >> 8) & 0x1f;
        assert_eq!(field >> 4, 1, "opt enable set");
        assert_eq!((field >> 3) & 1, 0, "SWCLK low");
        assert_eq!(field & 0x7, 1, "low-phase delay");

        let field = (DIRECT_WORDS[2] >> 8) & 0x1f;
        assert_eq!(field >> 4, 1);
        assert_eq!((field >> 3) & 1, 1, "SWCLK high");
        assert_eq!(field & 0x7, 1);
    }

    #[test]
    fn decode_oen_sideset_two_bits() {
        // Two data bits plus opt: bit 4 enable, bits 3:2 side value, bits 1:0 delay.
        let field = (OEN_WORDS[12] >> 8) & 0x1f;
        assert_eq!(field >> 4, 1, "opt enable set");
        assert_eq!((field >> 2) & 0x3, 0x3, "OEn released, SWCLK high");
        assert_eq!(field & 0x3, 1, "delay 1");
    }

    #[test]
    fn oen_entries_apply_the_required_enable_polarity() {
        fn side(word: u16) -> Option<u16> {
            let field = (word >> 8) & 0x1f;
            (field >> 4 != 0).then_some((field >> 2) & 0x3)
        }

        assert_eq!(OEN.labels.turnaround_cmd, 0);
        for index in [1usize, 2, 7, 12, 13] {
            assert_eq!(side(OEN_WORDS[index]).unwrap() & 1, 1, "word {index}");
        }
        for index in [5usize, 6] {
            assert_eq!(side(OEN_WORDS[index]).unwrap() & 1, 0, "word {index}");
        }
    }

    #[test]
    fn oen_level_transitions_release_around_every_driven_phase() {
        fn oen_level(word: u16, previous: bool) -> bool {
            let field = (word >> 8) & 0x1f;
            if field >> 4 == 0 {
                previous
            } else {
                ((field >> 2) & 1) != 0
            }
        }

        let mut released = false;
        for index in [7usize, 5, 6, 7, 1, 2, 7, 12, 13, 7, 15, 16, 7] {
            released = oen_level(OEN_WORDS[index], released);
            let expected = !matches!(index, 5 | 6);
            assert_eq!(released, expected, "word {index}");
        }
    }
}
