//! The two PIO program variants, assembled by a const encoder so the crate needs no assembler.

/// A PIO instruction destination (the `out`/`set` target field, bits 7:5).
#[derive(Copy, Clone)]
enum Dest {
    Pins = 0,
    X = 1,
    PinDirs = 4,
    Pc = 5,
}

/// A PIO `jmp` condition (bits 7:5 of a JMP instruction).
#[derive(Copy, Clone)]
enum Cond {
    Always = 0,
    XPostDec = 2,
}

/// How the 5-bit delay/side-set field splits for a variant.
#[derive(Copy, Clone)]
struct SideSet {
    /// Side-set data bits; an enable bit is always present because both variants use `opt`.
    data_bits: u8,
}

impl SideSet {
    /// Total side-set field width, including the `opt` enable bit.
    const fn total_bits(self) -> u8 {
        self.data_bits + 1
    }

    /// Encode the 5-bit `[12:8]` delay/side-set field; `side` is `None` to leave the pins alone.
    const fn field(self, side: Option<u8>, delay: u8) -> u16 {
        let total = self.total_bits();
        let delay_bits = 5 - total;
        let (enable, value) = match side {
            Some(v) => (1u16, v as u16),
            None => (0, 0),
        };
        // Side-set data sits just below the enable bit; delay fills the low bits.
        let mut field = (delay as u16) & ((1 << delay_bits) - 1);
        field |= value << delay_bits;
        field |= enable << (total - 1) << delay_bits;
        field
    }
}

/// Assemble one instruction: opcode in 15:13, delay/side-set in 12:8, opcode-specific low byte.
const fn instr(opcode: u16, field: u16, low: u16) -> u16 {
    (opcode << 13) | (field << 8) | low
}

const OP_JMP: u16 = 0b000;
const OP_IN: u16 = 0b010;
const OP_OUT: u16 = 0b011;
const OP_PUSHPULL: u16 = 0b100;
const OP_MOV: u16 = 0b101;

const fn jmp(ss: SideSet, cond: Cond, addr: u8, side: Option<u8>, delay: u8) -> u16 {
    instr(
        OP_JMP,
        ss.field(side, delay),
        ((cond as u16) << 5) | (addr as u16),
    )
}
const fn out(ss: SideSet, dest: Dest, count: u8, side: Option<u8>, delay: u8) -> u16 {
    // A count of 32 encodes as 0; every count here is below 32.
    instr(
        OP_OUT,
        ss.field(side, delay),
        ((dest as u16) << 5) | (count as u16),
    )
}
const fn in_pins(ss: SideSet, count: u8, side: Option<u8>, delay: u8) -> u16 {
    // Source = PINS (0).
    instr(OP_IN, ss.field(side, delay), count as u16)
}
const fn pull(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    // PULL, blocking: bit 7 = 1 (pull), bit 5 = 1 (block).
    instr(OP_PUSHPULL, ss.field(side, delay), 0b1010_0000)
}
const fn push(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    // PUSH, blocking: bit 7 = 0 (push), bit 5 = 1 (block).
    instr(OP_PUSHPULL, ss.field(side, delay), 0b0010_0000)
}
const fn nop(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    // Canonical NOP = `mov y, y`.
    instr(OP_MOV, ss.field(side, delay), (2 << 5) | 2)
}

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
    /// Released turnaround loop entry.
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
    /// Absolute address of the released turnaround loop entry.
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

const LOW_DELAY: u8 = if cfg!(any(feature = "clock-180mhz", feature = "clock-200mhz")) {
    2
} else {
    1
};
const READ_NOP_DELAY: u8 = if cfg!(any(feature = "clock-180mhz", feature = "clock-200mhz")) {
    1
} else {
    0
};

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

static DIRECT_WORDS: [u16; 13] = [
    // 0 write_cmd / turnaround_cmd: pull the data (or dummy 0 for high-Z).
    pull(SS1, None, 0),
    // 1 write_bitloop: drive the bit with SWCLK low.
    out(SS1, Dest::Pins, 1, Some(0), LOW_DELAY),
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
    nop(SS1, None, READ_NOP_DELAY),
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
];

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

static OEN_WORDS: [u16; 17] = [
    // 0 turnaround_cmd: pull  (dummy data word, pairing kept uniform).
    pull(SS2, None, 0),
    // 1 turnaround_bitloop: OEn released, SWCLK low.
    nop(SS2, Some(0x1), LOW_DELAY),
    // 2 jmp x-- turnaround_bitloop [1] side 0x3  (OEn released, SWCLK high).
    jmp(SS2, Cond::XPostDec, 1, Some(0x3), 1),
    // 3 jmp get_next_cmd.
    jmp(SS2, Cond::Always, 7, None, 0),
    // 4 write_cmd: pull.
    pull(SS2, None, 0),
    // 5 write_bitloop: driving, SWCLK low.
    out(SS2, Dest::Pins, 1, Some(0x0), LOW_DELAY),
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
    nop(SS2, None, READ_NOP_DELAY),
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
];

/// Pick the variant for a wiring: [`OEN`] with an output-enable pin, [`DIRECT`] otherwise.
pub const fn select_program(has_oen: bool) -> &'static PioProgram {
    if has_oen {
        &OEN
    } else {
        &DIRECT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Golden words hand-computed from the datasheet encoding, so an encoder bug cannot hide.
    #[rustfmt::skip]
    const DIRECT_GOLDEN: [u16; 13] = [
        0x80A0, // 0  pull
        if cfg!(any(feature = "clock-180mhz", feature = "clock-200mhz")) { 0x7201 } else { 0x7101 },
        0x1941, // 2  jmp x-- 1 [1] side 1
        0x90A0, // 3  pull side 0
        0x6028, // 4  out x,8
        0x6081, // 5  out pindirs,1
        0x60A5, // 6  out pc,5
        if cfg!(any(feature = "clock-180mhz", feature = "clock-200mhz")) { 0xA142 } else { 0xA042 },
        0x5901, // 8  in pins,1 [1] side 1
        0x1047, // 9  jmp x-- 7 side 0
        0x8020, // 10 push
        0x8020, // 11 release completion push
        0x0003, // 12 jmp get_next_cmd (3)
    ];

    #[rustfmt::skip]
    const OEN_GOLDEN: [u16; 17] = [
        0x80A0, // 0  pull
        if cfg!(any(feature = "clock-180mhz", feature = "clock-200mhz")) { 0xB642 } else { 0xB542 },
        0x1D41, // 2  jmp x-- 1 [1] side 0x3
        0x0007, // 3  jmp get_next_cmd (7)
        0x80A0, // 4  pull
        if cfg!(any(feature = "clock-180mhz", feature = "clock-200mhz")) { 0x7201 } else { 0x7101 },
        0x1945, // 6  jmp x-- 5 [1] side 0x2
        0x94A0, // 7  pull side 0x1
        0x6028, // 8  out x,8
        0x6081, // 9  out pindirs,1
        0x60A5, // 10 out pc,5
        if cfg!(any(feature = "clock-180mhz", feature = "clock-200mhz")) { 0xA142 } else { 0xA042 },
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
        assert_eq!(select_program(false).variant, Variant::Direct);
        assert_eq!(select_program(true).variant, Variant::Oen);
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
        assert_eq!(field & 0x7, u16::from(LOW_DELAY), "low-phase delay");

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
