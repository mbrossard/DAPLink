//! Const-encoded RP PIO program for complete JTAG-DP data scans.

use crate::asm::{
    in_pins, irq, jmp, mov_reverse_isr_to_osr, nop, out, pull, push, set, Cond, Dest, SideSet,
};
use crate::clock::JtagTiming;
use crate::composition::MAX_BYPASS_BEFORE;

const SS2: SideSet = SideSet { data_bits: 2 };

pub(crate) const IR_DONE_IRQ: u8 = 1;
/// The WAIT ACK `0b001` as the program compares it, bit-reversed; every other ACK shifts data.
pub(crate) const WAIT_REVERSED: u8 = 0b100;
/// Where the request cells leave the ACK in the first word a transfer pushes.
pub(crate) const ACK_SHIFT: u32 = 29;
pub(crate) const IR_EXEC_BOTTOM: u8 = 29;
pub(crate) const IR_EXEC_TOP: u8 = 31;
const IR_SCAN_CYCLES: usize = 15;
const IR_SCAN_WORDS: usize = IR_SCAN_CYCLES + 1;

pub(crate) const fn request_word(bypass_before: u8, request: u8) -> u32 {
    assert!(bypass_before <= MAX_BYPASS_BEFORE);
    let prefix_bits = 2 + bypass_before;
    let prefix = (1u32 << prefix_bits) - 1;
    prefix | (((request & 0x07) as u32) << prefix_bits)
}

/// Encode a fixed-chain scan for the JTAG-DP instructions used by dap-rs.
pub(crate) fn ir_scan_words(ir: u32, timing: JtagTiming) -> Option<[u32; IR_SCAN_WORDS]> {
    match ir {
        0x08 | 0x0a | 0x0b | 0x0e => Some(encode_ir_scan_words(ir, timing)),
        _ => None,
    }
}

const fn encode_ir_scan_words(ir: u32, timing: JtagTiming) -> [u32; IR_SCAN_WORDS] {
    let mut words = [0; IR_SCAN_WORDS];
    let mut cycle = 0;
    while cycle < IR_SCAN_CYCLES {
        let high = nop(
            SS2,
            Some((ir_tms(cycle) << 1) | 1),
            timing.loop_high_delay(),
        );
        let next = cycle + 1;
        let low = set(
            SS2,
            Dest::Pins,
            ir_tdi(next, ir),
            Some(ir_tms(next) << 1),
            timing.loop_low_delay(),
        );
        words[cycle] = (high as u32) | ((low as u32) << 16);
        cycle += 1;
    }
    // Mark completion, then enter the data program, whose first PULL takes the request queued behind this scan.
    words[IR_SCAN_CYCLES] =
        (irq(SS2, IR_DONE_IRQ) as u32) | ((jmp(SS2, Cond::Always, 0, None, 0) as u32) << 16);
    words
}

const fn ir_tms(cycle: usize) -> u8 {
    ((cycle < 2) || (cycle >= 12 && cycle < 14)) as u8
}

const fn ir_tdi(cycle: usize, ir: u32) -> u8 {
    if cycle >= 2 && cycle < 12 {
        let low_data = !0x3c | ((ir & 0x0f) << 2);
        ((low_data >> (cycle - 2)) & 1) as u8
    } else {
        1
    }
}

/// A complete program image and its active instruction count.
#[derive(Clone, Copy)]
pub(crate) struct AutonomousProgram {
    pub(crate) words: [u16; 32],
    pub(crate) len: u8,
    pub(crate) wrap_bottom: u8,
    pub(crate) wrap_top: u8,
}

impl AutonomousProgram {
    pub(crate) const fn new(bypass_before: u8, bypass_after: u8, timing: JtagTiming) -> Self {
        assert!(bypass_before <= MAX_BYPASS_BEFORE);
        assert!(bypass_after <= 1);
        if bypass_after == 0 {
            after_zero(bypass_before, timing)
        } else {
            after_one(bypass_before, timing)
        }
    }
}

const fn request_prefix(bypass_before: u8) -> u8 {
    // Capture-DR and entry into Shift-DR precede the bypass and three request bits.
    4 + bypass_before
}

const fn after_one(bypass_before: u8, timing: JtagTiming) -> AutonomousProgram {
    let data = 14;
    let tail = 20;
    let words = [
        pull(SS2, None, 0),
        set(SS2, Dest::Pins, 1, Some(0b10), timing.full_low_delay()),
        nop(SS2, Some(0b11), timing.full_high_delay()),
        set(
            SS2,
            Dest::X,
            request_prefix(bypass_before),
            Some(0b00),
            timing.loop_low_delay(),
        ),
        out(SS2, Dest::Pins, 1, Some(0b00), timing.output_delay()),
        nop(SS2, Some(0b01), timing.loop_high_delay()),
        in_pins(SS2, 1, Some(0b01), timing.sample_delay()),
        jmp(SS2, Cond::XPostDec, 4, Some(0b00), timing.loop_low_delay()),
        mov_reverse_isr_to_osr(SS2),
        push(SS2, None, 0),
        out(SS2, Dest::X, 3, None, 0),
        jmp(SS2, Cond::XNotY, data, None, 0),
        pull(SS2, None, 0),
        jmp(SS2, Cond::Always, tail, None, 0),
        pull(SS2, None, 0),
        set(SS2, Dest::X, 31, None, 0),
        out(SS2, Dest::Pins, 1, Some(0b00), timing.output_delay()),
        nop(SS2, Some(0b01), timing.loop_high_delay()),
        in_pins(SS2, 1, Some(0b01), timing.sample_delay()),
        jmp(SS2, Cond::XPostDec, 16, Some(0b00), timing.loop_low_delay()),
        set(SS2, Dest::Pins, 1, Some(0b10), timing.output_delay()),
        nop(SS2, Some(0b11), timing.full_high_delay()),
        nop(SS2, Some(0b10), timing.full_low_delay()),
        nop(SS2, Some(0b11), timing.full_high_delay()),
        nop(SS2, Some(0b00), timing.full_low_delay()),
        nop(SS2, Some(0b01), timing.full_high_delay()),
        nop(SS2, Some(0b00), timing.full_low_delay()),
        push(SS2, None, 0),
        jmp(SS2, Cond::Always, 0, None, 0),
        pull(SS2, None, 0),
        out(SS2, Dest::Exec, 16, None, 0),
        out(SS2, Dest::Exec, 16, None, 0),
    ];
    AutonomousProgram {
        words,
        len: 32,
        wrap_bottom: IR_EXEC_BOTTOM,
        wrap_top: IR_EXEC_TOP,
    }
}

const fn after_zero(bypass_before: u8, timing: JtagTiming) -> AutonomousProgram {
    let data = 16;
    let tail = 25;
    let words = [
        pull(SS2, None, 0),
        set(SS2, Dest::Pins, 1, Some(0b10), timing.full_low_delay()),
        nop(SS2, Some(0b11), timing.full_high_delay()),
        set(
            SS2,
            Dest::X,
            request_prefix(bypass_before),
            Some(0b00),
            timing.loop_low_delay(),
        ),
        out(SS2, Dest::Pins, 1, Some(0b00), timing.output_delay()),
        nop(SS2, Some(0b01), timing.loop_high_delay()),
        in_pins(SS2, 1, Some(0b01), timing.sample_delay()),
        jmp(SS2, Cond::XPostDec, 4, Some(0b00), timing.loop_low_delay()),
        mov_reverse_isr_to_osr(SS2),
        push(SS2, None, 0),
        out(SS2, Dest::X, 3, None, 0),
        set(SS2, Dest::Y, WAIT_REVERSED, None, 0),
        jmp(SS2, Cond::XNotY, data, None, 0),
        pull(SS2, None, 0),
        set(SS2, Dest::Pins, 1, Some(0b10), timing.full_low_delay()),
        jmp(
            SS2,
            Cond::Always,
            tail,
            Some(0b11),
            timing.full_high_delay(),
        ),
        pull(SS2, None, 0),
        set(SS2, Dest::X, 30, None, 0),
        out(SS2, Dest::Pins, 1, Some(0b00), timing.output_delay()),
        nop(SS2, Some(0b01), timing.loop_high_delay()),
        in_pins(SS2, 1, Some(0b01), timing.sample_delay()),
        jmp(SS2, Cond::XPostDec, 18, Some(0b00), timing.loop_low_delay()),
        out(SS2, Dest::Pins, 1, Some(0b10), timing.output_delay()),
        nop(SS2, Some(0b11), timing.loop_high_delay()),
        in_pins(SS2, 1, Some(0b11), timing.sample_delay()),
        jmp(SS2, Cond::Always, 26, Some(0b10), timing.full_low_delay()),
        nop(SS2, Some(0b11), timing.full_high_delay()),
        nop(SS2, Some(0b00), timing.full_low_delay()),
        nop(SS2, Some(0b01), timing.full_high_delay()),
        nop(SS2, Some(0b00), timing.full_low_delay()),
        push(SS2, None, 0),
        0,
    ];
    AutonomousProgram {
        words,
        len: 31,
        wrap_bottom: 0,
        wrap_top: 30,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::{JTAG_TIMING_125MHZ, JTAG_TIMING_150MHZ, JTAG_TIMING_200MHZ};

    const TIMINGS: [JtagTiming; 3] = [JTAG_TIMING_125MHZ, JTAG_TIMING_150MHZ, JTAG_TIMING_200MHZ];

    #[rustfmt::skip]
    const AFTER_ONE_200MHZ: [u16; 32] = [
        0x80a0, 0xfb01, 0xbf42, 0xf125, 0x7101, 0xb642, 0x5401, 0x1144,
        0xa0f6, 0x8020, 0x6023, 0x00ae, 0x80a0, 0x0014, 0x80a0, 0xe03f,
        0x7101, 0xb642, 0x5401, 0x1150, 0xf901, 0xbf42, 0xbb42, 0xbf42,
        0xb342, 0xb742, 0xb342, 0x8020, 0x0000, 0x80a0, 0x60f0, 0x60f0,
    ];

    #[rustfmt::skip]
    const AFTER_ZERO_200MHZ: [u16; 31] = [
        0x80a0, 0xfb01, 0xbf42, 0xf124, 0x7101, 0xb642, 0x5401, 0x1144,
        0xa0f6, 0x8020, 0x6023, 0xe044, 0x00b0, 0x80a0, 0xfb01, 0x1f19,
        0x80a0, 0xe03e, 0x7101, 0xb642, 0x5401, 0x1152, 0x7901, 0xbe42,
        0x5c01, 0x1b1a, 0xbf42, 0xb342, 0xb742, 0xb342, 0x8020,
    ];

    const AFTER_ONE_TIMING_WORDS: &[(usize, [u16; 3])] = &[
        (1, [0xfa01, 0xfa01, 0xfb01]),
        (2, [0xbd42, 0xbe42, 0xbf42]),
        (3, [0xf025, 0xf025, 0xf125]),
        (5, [0xb442, 0xb542, 0xb642]),
        (7, [0x1044, 0x1044, 0x1144]),
        (17, [0xb442, 0xb542, 0xb642]),
        (19, [0x1050, 0x1050, 0x1150]),
        (21, [0xbd42, 0xbe42, 0xbf42]),
        (22, [0xba42, 0xba42, 0xbb42]),
        (23, [0xbd42, 0xbe42, 0xbf42]),
        (24, [0xb242, 0xb242, 0xb342]),
        (25, [0xb542, 0xb642, 0xb742]),
        (26, [0xb242, 0xb242, 0xb342]),
    ];

    const AFTER_ZERO_TIMING_WORDS: &[(usize, [u16; 3])] = &[
        (1, [0xfa01, 0xfa01, 0xfb01]),
        (2, [0xbd42, 0xbe42, 0xbf42]),
        (3, [0xf024, 0xf024, 0xf124]),
        (5, [0xb442, 0xb542, 0xb642]),
        (7, [0x1044, 0x1044, 0x1144]),
        (14, [0xfa01, 0xfa01, 0xfb01]),
        (15, [0x1d19, 0x1e19, 0x1f19]),
        (19, [0xb442, 0xb542, 0xb642]),
        (21, [0x1052, 0x1052, 0x1152]),
        (23, [0xbc42, 0xbd42, 0xbe42]),
        (25, [0x1a1a, 0x1a1a, 0x1b1a]),
        (26, [0xbd42, 0xbe42, 0xbf42]),
        (27, [0xb242, 0xb242, 0xb342]),
        (28, [0xb542, 0xb642, 0xb742]),
        (29, [0xb242, 0xb242, 0xb342]),
    ];

    #[rustfmt::skip]
    const IR_DPACC_GOLDENS: [[u32; IR_SCAN_WORDS]; 3] = [
        [
            0xf801bc42, 0xf001bc42, 0xf001b442, 0xf000b442,
            0xf001b442, 0xf000b442, 0xf001b442, 0xf001b442,
            0xf001b442, 0xf001b442, 0xf001b442, 0xf801b442,
            0xf801bc42, 0xf001bc42, 0xf001b442, 0x0000c001,
        ],
        [
            0xf801bd42, 0xf001bd42, 0xf001b542, 0xf000b542,
            0xf001b542, 0xf000b542, 0xf001b542, 0xf001b542,
            0xf001b542, 0xf001b542, 0xf001b542, 0xf801b542,
            0xf801bd42, 0xf001bd42, 0xf001b542, 0x0000c001,
        ],
        [
            0xf901be42, 0xf101be42, 0xf101b642, 0xf100b642,
            0xf101b642, 0xf100b642, 0xf101b642, 0xf101b642,
            0xf101b642, 0xf101b642, 0xf101b642, 0xf901b642,
            0xf901be42, 0xf101be42, 0xf101b642, 0x0000c001,
        ],
    ];

    fn duration(word: u16) -> u32 {
        u32::from((word >> 8) & 0x03) + 1
    }

    fn patch_timing_words<const N: usize>(
        mut words: [u16; N],
        profile: usize,
        patches: &[(usize, [u16; 3])],
    ) -> [u16; N] {
        for (index, values) in patches {
            words[*index] = values[profile];
        }
        words
    }

    #[test]
    fn after_one_program_matches_every_profile_golden() {
        for (profile, timing) in TIMINGS.into_iter().enumerate() {
            let program = AutonomousProgram::new(1, 1, timing);
            let golden = patch_timing_words(AFTER_ONE_200MHZ, profile, AFTER_ONE_TIMING_WORDS);
            assert_eq!(program.len, 32);
            assert_eq!(program.wrap_bottom, IR_EXEC_BOTTOM);
            assert_eq!(program.wrap_top, IR_EXEC_TOP);
            assert_eq!(program.words, golden);
        }
    }

    #[test]
    fn after_zero_program_matches_every_profile_golden() {
        for (profile, timing) in TIMINGS.into_iter().enumerate() {
            let program = AutonomousProgram::new(0, 0, timing);
            let golden = patch_timing_words(AFTER_ZERO_200MHZ, profile, AFTER_ZERO_TIMING_WORDS);
            assert_eq!(program.len, 31);
            assert_eq!(program.wrap_bottom, 0);
            assert_eq!(program.wrap_top, 30);
            assert_eq!(&program.words[..31], &golden);
        }
    }

    #[test]
    fn geometry_patch_covers_every_safe_request_length() {
        for timing in TIMINGS {
            for before in 0..=MAX_BYPASS_BEFORE {
                for after in 0..=1 {
                    let program = AutonomousProgram::new(before, after, timing);
                    let expected = set(
                        SS2,
                        Dest::X,
                        request_prefix(before),
                        Some(0b00),
                        timing.loop_low_delay(),
                    );
                    assert_eq!(program.words[3], expected);
                    assert!(program.len <= 32);
                    let ack_branch = if after == 0 {
                        program.words[12]
                    } else {
                        program.words[11]
                    };
                    assert_eq!(ack_branch & 0x1f, if after == 0 { 16 } else { 14 });
                }
            }
        }
    }

    #[test]
    fn request_word_places_prefix_and_request_lsb_first() {
        for before in 0..=MAX_BYPASS_BEFORE {
            let prefix_bits = 2 + before;
            let prefix_mask = (1u32 << prefix_bits) - 1;
            for request in 0..=7 {
                let word = request_word(before, request);
                assert_eq!(word & prefix_mask, prefix_mask);
                assert_eq!((word >> prefix_bits) & 7, u32::from(request));
                assert_eq!(word >> (prefix_bits + 3), 0);
            }
        }
    }

    #[test]
    fn reversed_ack_decode_leaves_early_only_on_wait() {
        for before in 0..=MAX_BYPASS_BEFORE {
            for ack in 0u32..=7 {
                let mut isr = 0u32;
                for _ in 0..2 + before {
                    isr >>= 1;
                }
                for bit in 0..3 {
                    isr = (isr >> 1) | (((ack >> bit) & 1) << 31);
                }
                let decoded = isr.reverse_bits() & 7;
                assert_eq!(decoded == u32::from(WAIT_REVERSED), ack == 0b001);
                assert_eq!(isr >> ACK_SHIFT, ack);
            }
        }
    }

    #[test]
    fn modeled_paths_end_idle_with_one_result_word() {
        for before in 0..=MAX_BYPASS_BEFORE {
            // Select-DR, request, and the three clocks returning from Shift-DR to Idle.
            assert_eq!(1 + (5 + before) + 3, 9 + before);
            // A successful after-zero or after-one scan adds data and optional bypass clocks.
            assert_eq!(1 + (5 + before) + 32 + 2, 40 + before);
            assert_eq!(1 + (5 + before) + 32 + 1 + 2, 41 + before);
        }
    }

    #[test]
    fn data_scan_loops_match_every_profile_geometry() {
        for timing in TIMINGS {
            for (program, starts) in [
                (AutonomousProgram::new(0, 1, timing), [4, 16]),
                (AutonomousProgram::new(0, 0, timing), [4, 18]),
            ] {
                for start in starts {
                    let low = duration(program.words[start + 3]) + duration(program.words[start]);
                    let high =
                        duration(program.words[start + 1]) + duration(program.words[start + 2]);
                    assert_eq!((low, high), (timing.low_cycles(), timing.high_cycles()));
                    assert_eq!(low + high, timing.cycles_per_bit());
                }
            }
        }
    }

    #[test]
    fn data_scan_tails_match_every_profile_geometry() {
        for timing in TIMINGS {
            let after_one = AutonomousProgram::new(0, 1, timing);
            assert_eq!(
                duration(after_one.words[19]) + duration(after_one.words[20]),
                timing.low_cycles()
            );
            assert_eq!(duration(after_one.words[21]), timing.high_cycles());
            for (low, high) in [(22, 23), (24, 25)] {
                assert_eq!(duration(after_one.words[low]), timing.low_cycles());
                assert_eq!(duration(after_one.words[high]), timing.high_cycles());
            }
            assert_eq!(duration(after_one.words[26]), timing.low_cycles());

            let after_zero = AutonomousProgram::new(0, 0, timing);
            assert_eq!(
                duration(after_zero.words[21]) + duration(after_zero.words[22]),
                timing.low_cycles()
            );
            assert_eq!(
                duration(after_zero.words[23]) + duration(after_zero.words[24]),
                timing.high_cycles()
            );
            for (low, high) in [(25, 26), (27, 28), (14, 15)] {
                assert_eq!(duration(after_zero.words[low]), timing.low_cycles());
                assert_eq!(duration(after_zero.words[high]), timing.high_cycles());
            }
            assert_eq!(duration(after_zero.words[29]), timing.low_cycles());
        }
    }

    #[test]
    fn after_one_paths_return_explicitly_before_the_executor() {
        let program = AutonomousProgram::new(0, 1, JTAG_TIMING_200MHZ);
        assert_eq!(program.words[9], push(SS2, None, 0));
        assert_eq!(program.words[12], pull(SS2, None, 0));
        assert_eq!(program.words[13], jmp(SS2, Cond::Always, 20, None, 0));
        assert_eq!(program.words[27], push(SS2, None, 0));
        assert_eq!(program.words[28], jmp(SS2, Cond::Always, 0, None, 0));
        assert_eq!((program.wrap_bottom, program.wrap_top), (29, 31));
    }

    #[test]
    fn ir_executor_packs_the_expected_tms_and_tdi_cells() {
        assert_eq!((IR_EXEC_BOTTOM, IR_EXEC_TOP), (29, 31));
        let expected_tms = [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0];
        let expected_tdi = [1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1];
        for (profile, timing) in TIMINGS.into_iter().enumerate() {
            let words = encode_ir_scan_words(0x0a, timing);
            assert_eq!(words, IR_DPACC_GOLDENS[profile]);
            let mut observed_tdi = [0; IR_SCAN_CYCLES];
            observed_tdi[0] = 1;
            for (cycle, word) in words[..IR_SCAN_CYCLES].iter().enumerate() {
                let high = *word as u16;
                let low = (*word >> 16) as u16;
                assert_eq!((high >> 10) & 0x03, (expected_tms[cycle] << 1 | 1));
                let next_tms = expected_tms.get(cycle + 1).copied().unwrap_or(0);
                assert_eq!((low >> 10) & 0x03, next_tms << 1);
                assert_eq!(duration(high) + 1, timing.high_cycles());
                assert_eq!(duration(low) + 2, timing.low_cycles());
                assert_eq!(
                    duration(high) + 1 + duration(low) + 2,
                    timing.cycles_per_bit()
                );
                if cycle + 1 < IR_SCAN_CYCLES {
                    observed_tdi[cycle + 1] = (low & 0x1f) as u8;
                } else {
                    assert_eq!(low & 0x1f, 1);
                }
            }
            assert_eq!(observed_tdi, expected_tdi);
            let final_word = words[IR_SCAN_CYCLES];
            assert_eq!(final_word as u16, irq(SS2, IR_DONE_IRQ));
            assert_eq!(
                (final_word >> 16) as u16,
                jmp(SS2, Cond::Always, 0, None, 0)
            );
        }
        assert_eq!(
            ir_scan_words(0x0a, JTAG_TIMING_200MHZ).unwrap(),
            encode_ir_scan_words(0x0a, JTAG_TIMING_200MHZ)
        );
        assert!(ir_scan_words(0x09, JTAG_TIMING_200MHZ).is_none());
    }
}
