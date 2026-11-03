//! Complete SWD transfer suffixes selected by one ACK-dependent continuation word.

use crate::asm::{
    in_pins, jmp, mov_isr_null, mov_y_to_pins, nop, out, pull, push, set, Cond, Dest, SideSet,
};
use crate::program::Variant;

const FULL_PHASE_DELAY: u8 = 1;

/// Successful-read continuation address.
pub(crate) const READ_CONTINUATION: u8 = 14;
/// Released trailing-turnaround continuation address.
pub(crate) const RELEASE_CONTINUATION: u8 = 19;
/// Successful-write continuation address.
pub(crate) const WRITE_CONTINUATION: u8 = 21;

/// One complete 32-slot ACK-continuation program.
pub struct TransactionProgram {
    /// Instruction words loaded at offset zero.
    pub instructions: &'static [u16; 32],
    /// Side-set data width, excluding the optional enable bit.
    pub side_set_bits: u8,
    /// Electrical wiring selected by the program.
    pub variant: Variant,
}

const fn side(variant: Variant, drive: bool, high: bool) -> u8 {
    match variant {
        Variant::Direct => high as u8,
        Variant::Oen => ((high as u8) << 1) | ((!drive) as u8),
    }
}

const fn words(variant: Variant) -> [u16; 32] {
    let ss = SideSet {
        data_bits: match variant {
            Variant::Direct => 1,
            Variant::Oen => 2,
        },
    };
    let r0 = Some(side(variant, false, false));
    let r1 = Some(side(variant, false, true));
    let d0 = Some(side(variant, true, false));
    let d1 = Some(side(variant, true, true));

    [
        // Request and raw ACK. Every blocking FIFO instruction holds a released low clock.
        pull(ss, r0, 0),
        // Keep an external OEN released until SWDIO is configured as an output.
        set(ss, Dest::PinDirs, 1, r0, 0),
        set(ss, Dest::X, 7, r0, 0),
        out(ss, Dest::Pins, 1, d0, FULL_PHASE_DELAY),
        jmp(ss, Cond::XPostDec, 3, d1, FULL_PHASE_DELAY),
        set(ss, Dest::PinDirs, 0, r0, FULL_PHASE_DELAY),
        set(ss, Dest::X, 2, r1, FULL_PHASE_DELAY),
        mov_isr_null(ss, r0, 0),
        nop(ss, r0, 0),
        in_pins(ss, 1, r1, FULL_PHASE_DELAY),
        jmp(ss, Cond::XPostDec, 8, r0, 0),
        push(ss, r0, 0),
        pull(ss, r0, 0),
        out(ss, Dest::Pc, 5, r0, 0),
        // Successful read: data autopushes at 32 bits, then parity is pushed separately.
        set(ss, Dest::X, 31, r0, FULL_PHASE_DELAY),
        in_pins(ss, 1, r1, FULL_PHASE_DELAY),
        jmp(ss, Cond::XPostDec, 15, r0, FULL_PHASE_DELAY),
        in_pins(ss, 1, r1, FULL_PHASE_DELAY),
        push(ss, r0, FULL_PHASE_DELAY),
        nop(ss, r1, FULL_PHASE_DELAY),
        jmp(ss, Cond::Always, 0, r0, 0),
        // Successful write: parity follows the continuation PC in the OSR.
        out(ss, Dest::Y, 1, r1, FULL_PHASE_DELAY),
        pull(ss, r0, FULL_PHASE_DELAY),
        // Direction precedes OEN assertion, as it did for the request phase.
        set(ss, Dest::PinDirs, 1, r0, 0),
        set(ss, Dest::X, 31, r0, 0),
        out(ss, Dest::Pins, 1, d0, FULL_PHASE_DELAY),
        jmp(ss, Cond::XPostDec, 25, d1, FULL_PHASE_DELAY),
        mov_y_to_pins(ss, d0, FULL_PHASE_DELAY),
        set(ss, Dest::X, 7, d1, FULL_PHASE_DELAY),
        // Eight low cells clock the write through the DP before SWCLK stops (ADIv5 B4.1.1).
        set(ss, Dest::Pins, 0, d0, FULL_PHASE_DELAY),
        jmp(ss, Cond::XPostDec, 29, d1, FULL_PHASE_DELAY),
        set(ss, Dest::PinDirs, 0, r0, 0),
    ]
}

static DIRECT_WORDS: [u16; 32] = words(Variant::Direct);
static OEN_WORDS: [u16; 32] = words(Variant::Oen);

/// Direct or split-readback program, with SWCLK as its one side-set pin.
pub static TRANSACTION_DIRECT: TransactionProgram = TransactionProgram {
    instructions: &DIRECT_WORDS,
    side_set_bits: 1,
    variant: Variant::Direct,
};

/// External-shifter program, with contiguous `{OEn, SWCLK}` side-set pins.
pub static TRANSACTION_OEN: TransactionProgram = TransactionProgram {
    instructions: &OEN_WORDS,
    side_set_bits: 2,
    variant: Variant::Oen,
};

/// Select the program matching one SWD wiring.
pub const fn select_transaction_program(has_oen: bool) -> &'static TransactionProgram {
    if has_oen {
        &TRANSACTION_OEN
    } else {
        &TRANSACTION_DIRECT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field(word: u16) -> u8 {
        ((word >> 8) & 0x1f) as u8
    }

    fn side_value(word: u16, data_bits: u8) -> Option<u8> {
        let delay_bits = 5 - (data_bits + 1);
        let encoded = field(word);
        if encoded & (1 << (delay_bits + data_bits)) == 0 {
            None
        } else {
            Some((encoded >> delay_bits) & ((1 << data_bits) - 1))
        }
    }

    fn delay(word: u16, data_bits: u8) -> u8 {
        let delay_bits = 5 - (data_bits + 1);
        field(word) & ((1 << delay_bits) - 1)
    }

    #[test]
    fn both_programs_match_independently_encoded_golden_words() {
        #[rustfmt::skip]
        const DIRECT_GOLDEN: [u16; 32] = [
            0x90a0, 0xf081, 0xf027, 0x7101, 0x1943, 0xf180, 0xf922, 0xb0c3,
            0xb042, 0x5901, 0x1048, 0x9020, 0x90a0, 0x70a5, 0xf13f, 0x5901,
            0x114f, 0x5901, 0x9120, 0xb942, 0x1000, 0x7941, 0x91a0, 0xf081,
            0xf03f, 0x7101, 0x1959, 0xb102, 0xf927, 0xf100, 0x195d, 0xf080,
        ];
        #[rustfmt::skip]
        const OEN_GOLDEN: [u16; 32] = [
            0x94a0, 0xf481, 0xf427, 0x7101, 0x1943, 0xf580, 0xfd22, 0xb4c3,
            0xb442, 0x5d01, 0x1448, 0x9420, 0x94a0, 0x74a5, 0xf53f, 0x5d01,
            0x154f, 0x5d01, 0x9520, 0xbd42, 0x1400, 0x7d41, 0x95a0, 0xf481,
            0xf43f, 0x7101, 0x1959, 0xb102, 0xf927, 0xf100, 0x195d, 0xf480,
        ];
        assert_eq!(DIRECT_WORDS, DIRECT_GOLDEN);
        assert_eq!(OEN_WORDS, OEN_GOLDEN);
    }

    #[test]
    fn both_variants_fill_one_instruction_memory_at_offset_zero() {
        for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
            assert_eq!(program.instructions.len(), 32);
            assert_eq!(program.instructions[13] & 0x1f, 5, "OUT PC width");
            assert_eq!(READ_CONTINUATION, 14);
            assert_eq!(RELEASE_CONTINUATION, 19);
            assert_eq!(WRITE_CONTINUATION, 21);
        }
    }

    #[test]
    fn direct_and_oen_have_the_same_control_flow() {
        for pc in 0..32 {
            // Side-set and delay occupy bits 12:8; all other instruction bits must match.
            assert_eq!(DIRECT_WORDS[pc] & !0x1f00, OEN_WORDS[pc] & !0x1f00);
        }
    }

    #[test]
    fn input_cells_sample_on_the_rising_instruction() {
        for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
            let bits = program.side_set_bits;
            for &(pre, sample, falling) in &[(8, 9, 10), (14, 15, 16), (16, 17, 18)] {
                assert_eq!(delay(program.instructions[sample], bits), 1);
                assert_eq!(
                    side_value(program.instructions[pre], bits),
                    Some(side(program.variant, false, false))
                );
                assert_eq!(
                    side_value(program.instructions[sample], bits),
                    Some(side(program.variant, false, true))
                );
                assert_eq!(
                    side_value(program.instructions[falling], bits),
                    Some(side(program.variant, false, false))
                );
            }
        }
    }

    #[test]
    fn every_target_driven_rising_edge_is_four_cycles_apart() {
        let duration = |word, bits| 1 + delay(word, bits);
        for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
            let words = program.instructions;
            let bits = program.side_set_bits;
            assert_eq!(
                duration(words[6], bits) + duration(words[7], bits) + duration(words[8], bits),
                4
            );
            assert_eq!(
                duration(words[9], bits) + duration(words[10], bits) + duration(words[8], bits),
                4
            );
            assert_eq!(duration(words[15], bits) + duration(words[16], bits), 4);
            assert_eq!(duration(words[17], bits) + duration(words[18], bits), 4);
        }
    }

    #[test]
    fn every_probe_driven_rising_edge_is_four_cycles_apart() {
        let duration = |word, bits| 1 + delay(word, bits);
        for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
            let words = program.instructions;
            let bits = program.side_set_bits;
            assert_eq!(duration(words[4], bits) + duration(words[3], bits), 4);
            assert_eq!(duration(words[4], bits) + duration(words[5], bits), 4);
            assert_eq!(duration(words[26], bits) + duration(words[25], bits), 4);
            assert_eq!(duration(words[26], bits) + duration(words[27], bits), 4);
            assert_eq!(duration(words[28], bits) + duration(words[29], bits), 4);
            assert_eq!(duration(words[30], bits) + duration(words[29], bits), 4);
        }
    }

    fn clock_high(program: &TransactionProgram, word: u16) -> bool {
        let value = side_value(word, program.side_set_bits).expect("mandatory side-set");
        value & (1 << (program.side_set_bits - 1)) != 0
    }

    fn edge_count(program: &TransactionProgram, path: &[usize]) -> usize {
        let mut previous = false;
        let mut edges = 0;
        for &pc in path {
            let high = clock_high(program, program.instructions[pc]);
            edges += (!previous && high) as usize;
            previous = high;
        }
        edges
    }

    fn request_ack_path() -> std::vec::Vec<usize> {
        let mut path = std::vec![0, 1, 2];
        for _ in 0..8 {
            path.extend([3, 4]);
        }
        path.extend([5, 6, 7]);
        for _ in 0..3 {
            path.extend([8, 9, 10]);
        }
        path.extend([11, 12, 13]);
        path
    }

    #[test]
    fn every_continuation_has_the_exact_protocol_clock_count() {
        for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
            let common = request_ack_path();

            let mut read = common.clone();
            read.push(14);
            for _ in 0..32 {
                read.extend([15, 16]);
            }
            read.extend([17, 18, 19, 20]);
            assert_eq!(edge_count(program, &read), 46, "read");

            let mut write = common.clone();
            write.extend([21, 22, 23, 24]);
            for _ in 0..32 {
                write.extend([25, 26]);
            }
            write.extend([27, 28]);
            for _ in 0..8 {
                write.extend([29, 30]);
            }
            write.extend([31, 0]);
            assert_eq!(edge_count(program, &write), 54, "write");

            let mut release = common;
            release.extend([19, 20]);
            assert_eq!(edge_count(program, &release), 13, "failed ACK");
        }
    }

    #[test]
    fn every_fifo_stall_and_parity_push_holds_released_low() {
        for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
            let released_low = side(program.variant, false, false);
            for pc in [0, 11, 12, 18, 22] {
                assert_eq!(
                    side_value(program.instructions[pc], program.side_set_bits),
                    Some(released_low),
                    "PC {pc}"
                );
            }
        }
    }

    #[test]
    fn oen_is_released_before_each_direction_change() {
        for pc in [1, 2, 23, 24] {
            assert_eq!(side_value(OEN_WORDS[pc], 2), Some(0b01), "PC {pc}");
        }
        for pc in [3, 25] {
            assert_eq!(side_value(OEN_WORDS[pc], 2), Some(0b00), "PC {pc}");
        }
        for pc in [5, 31] {
            assert_eq!(side_value(OEN_WORDS[pc], 2), Some(0b01), "PC {pc}");
        }
    }

    #[test]
    fn every_protocol_outcome_finishes_released() {
        for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
            let r0 = side(program.variant, false, false);
            for pc in [0, 5, 7, 10, 12, 18, 20, 22, 31] {
                assert_eq!(
                    side_value(program.instructions[pc], program.side_set_bits),
                    Some(r0)
                );
            }
        }
    }
}
