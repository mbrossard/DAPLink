//! Host-side packing and state-machine operations for PIO JTAG sequences.

use daplink_rs::dap_rs::{dap::TransferConfig, jtag::SequenceInfo, swd::RnW};
use daplink_rs::daplink_core::jtag_dp::ACK_WAIT;

use crate::clock::PioClockDivider;
use crate::codec::read_align;
use crate::program::JtagProgramEntries;

/// Raw operations provided by the second PIO state machine.
pub trait PioJtagShifter {
    /// Push one JTAG command or data word.
    fn jtag_put(&mut self, word: u32);

    /// Pop one captured JTAG word.
    fn jtag_get(&mut self) -> u32;

    /// Set the JTAG state machine's clock divider.
    fn set_jtag_clock_divider(&mut self, divider: PioClockDivider);

    /// Quiesce SWD, stage the JTAG pin roles, and start at `dispatcher`.
    fn enter_jtag(&mut self, dispatcher: u8);

    /// Stop JTAG and release its dedicated pins before SWD is restored.
    fn exit_jtag(&mut self);

    /// Stop JTAG while retaining TCK-low idle output levels for a peripheral handoff.
    fn park_jtag(&mut self) {
        self.exit_jtag();
    }
}

const INLINE_BITS: u8 = 21;

/// The selected TAP's position within the configured scan chain.
#[derive(Copy, Clone)]
pub struct JtagScan {
    /// Number of bypass registers shifted before the selected TAP.
    pub device_index: u8,
    /// Total number of TAPs in the scan chain.
    pub device_count: u8,
}

/// Encode `bits` in `1..=32`; `entry` must name the loaded fixed-TMS entry in `0..32`.
pub const fn jtag_cmd_word(entry: u8, bits: u8, capture: bool) -> u32 {
    debug_assert!(bits >= 1 && bits <= 32);
    (bits - 1) as u32 | ((capture as u32) << 5) | ((entry as u32) << 6)
}

/// Encode `bits` in `1..=21`; `entry` must name the loaded inline entry in `0..32`.
pub const fn jtag_inline_word(entry: u8, bits: u8, capture: bool, tdi: u32) -> u32 {
    debug_assert!(bits >= 1 && bits <= INLINE_BITS);
    let mask = (1u32 << bits) - 1;
    jtag_cmd_word(entry, bits, capture) | ((tdi & mask) << 11)
}

/// Stateless JTAG sequence packing over the loaded PIO entry points.
#[derive(Copy, Clone)]
pub struct PioJtag {
    entries: JtagProgramEntries,
}

impl PioJtag {
    /// Build the sequence engine from its absolute PIO entry points.
    pub const fn new(entries: JtagProgramEntries) -> Self {
        Self { entries }
    }

    /// The dispatcher address used for the SWD-to-JTAG state-machine handoff.
    pub const fn dispatcher(&self) -> u8 {
        self.entries.get_next_cmd
    }

    fn queue_word<S: PioJtagShifter>(
        &self,
        shifter: &mut S,
        tms: bool,
        bits: u8,
        tdi: u32,
        capture: bool,
    ) {
        let (data_entry, inline_entry) = if tms {
            (
                self.entries.tms_high_data_cmd,
                self.entries.tms_high_inline_cmd,
            )
        } else {
            (
                self.entries.tms_low_data_cmd,
                self.entries.tms_low_inline_cmd,
            )
        };
        if bits <= INLINE_BITS {
            shifter.jtag_put(jtag_inline_word(inline_entry, bits, capture, tdi));
        } else {
            shifter.jtag_put(jtag_cmd_word(data_entry, bits, capture));
            shifter.jtag_put(tdi);
        }
    }

    fn capture_word<S: PioJtagShifter>(&self, shifter: &mut S, bits: u8) -> u32 {
        read_align(shifter.jtag_get(), bits)
    }

    fn shift_repeated<S: PioJtagShifter>(
        &self,
        shifter: &mut S,
        tms: bool,
        mut bits: u16,
        tdi: u32,
    ) {
        while bits != 0 {
            let chunk = bits.min(32) as u8;
            self.queue_word(shifter, tms, chunk, tdi, false);
            bits -= u16::from(chunk);
        }
    }

    /// Shift one instruction-register scan, fusing the common short chain into four commands.
    pub fn shift_ir<S: PioJtagShifter>(
        &self,
        shifter: &mut S,
        ir: u32,
        ir_before: u16,
        ir_length: u8,
        ir_after: u16,
    ) {
        let scan_bits = ir_before
            .saturating_add(u16::from(ir_length))
            .saturating_add(ir_after);
        let low_bits = scan_bits.saturating_add(1);
        if ir_length != 0 && low_bits <= 32 {
            let selected_bits = if ir_after == 0 {
                ir_length - 1
            } else {
                ir_length
            };
            let selected_mask = if selected_bits == 32 {
                u32::MAX
            } else {
                (1u32 << selected_bits) - 1
            };
            let selected_offset = 2 + u32::from(ir_before);
            let shifted_mask = selected_mask << selected_offset;
            let low_data = !shifted_mask | ((ir & selected_mask) << selected_offset);
            let exit_bit = if ir_after == 0 {
                (ir >> (ir_length - 1)) & 1
            } else {
                1
            };

            self.queue_word(shifter, true, 2, u32::MAX, false);
            self.queue_word(shifter, false, low_bits as u8, low_data, false);
            self.queue_word(shifter, true, 2, exit_bit | 0x02, false);
            self.queue_word(shifter, false, 1, u32::MAX, false);
            return;
        }

        self.tms_sequence(shifter, &[true, true, false, false]);
        self.shift_repeated(shifter, false, ir_before, u32::MAX);
        let mut remaining = ir_length;
        let mut value = ir;
        while remaining > 1 {
            let bits = (remaining - 1).min(32);
            self.queue_word(shifter, false, bits, value, false);
            value = if bits == 32 { 0 } else { value >> bits };
            remaining -= bits;
        }
        self.queue_word(shifter, ir_after == 0, 1, value, false);
        if ir_after != 0 {
            self.shift_repeated(shifter, false, ir_after - 1, u32::MAX);
            self.queue_word(shifter, true, 1, u32::MAX, false);
        }
        self.tms_sequence(shifter, &[true, false]);
    }

    /// One DPACC/APACC scan, leaving early on WAIT only: the ACK and word, `None` off-geometry.
    pub fn transfer<S: PioJtagShifter>(
        &self,
        shifter: &mut S,
        scan: JtagScan,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> Option<(u32, u32)> {
        let JtagScan {
            device_index,
            device_count,
        } = scan;
        if device_count == 0 || device_index >= device_count {
            return None;
        }
        let bypass_before = u16::from(device_index);
        let bypass_after = u16::from(device_count) - bypass_before - 1;
        if bypass_before > 27 || bypass_after > 33 {
            return None;
        }
        let request_offset = 2 + bypass_before;
        let request_bits = request_offset + 3;
        let request = u32::from(a2a3 << 1) | u32::from(r_nw as u8);
        let prefix = (1u32 << request_offset) - 1;

        // Enter Shift-DR and clock the bypass/request prefix in two uninterrupted commands.
        self.queue_word(shifter, true, 1, u32::MAX, false);
        self.queue_word(
            shifter,
            false,
            request_bits as u8,
            prefix | (request << request_offset),
            true,
        );
        let request_rx = self.capture_word(shifter, request_bits as u8);
        let ack = (request_rx >> request_offset) & 0x07;

        let idle_and_entry = u16::from(transfer_config.idle_cycles) + 1;
        let first_idle = idle_and_entry.min(32) as u8;
        let remaining_idle = idle_and_entry - u16::from(first_idle);
        let captured = if ack != ACK_WAIT {
            if bypass_after == 0 {
                self.queue_word(shifter, false, 31, data, true);
                self.queue_word(shifter, true, 2, (data >> 31) | 0x02, true);
                self.queue_word(shifter, false, first_idle, u32::MAX, false);
                let low = self.capture_word(shifter, 31);
                let high = self.capture_word(shifter, 2);
                low | ((high & 1) << 31)
            } else {
                self.queue_word(shifter, false, 32, data, true);
                if bypass_after > 1 {
                    self.queue_word(shifter, false, (bypass_after - 1) as u8, u32::MAX, false);
                }
                // The last bypass bit exits Shift-DR; the second high clock enters Update-DR.
                self.queue_word(shifter, true, 2, u32::MAX, false);
                // The first low clock enters Idle and the rest are requested idle clocks.
                self.queue_word(shifter, false, first_idle, u32::MAX, false);
                self.capture_word(shifter, 32)
            }
        } else {
            // From Shift-DR, two high clocks enter Update-DR and one low clock enters Idle.
            self.queue_word(shifter, true, 2, u32::MAX, false);
            self.queue_word(shifter, false, first_idle, u32::MAX, false);
            0
        };
        self.shift_repeated(shifter, false, remaining_idle, u32::MAX);
        Some((ack, captured))
    }

    /// Shift one fixed-TMS CMSIS-DAP sequence, packing at most 32 bits per FIFO command.
    pub fn sequence<S: PioJtagShifter>(
        &self,
        shifter: &mut S,
        info: SequenceInfo,
        tdi: &[u8],
        rxbuf: &mut [u8],
    ) {
        if info.capture {
            rxbuf.fill(0);
        }
        let mut remaining = info.n_bits as usize;
        let mut byte_index = 0;
        while remaining != 0 {
            let mut queued = 0;
            // Bound result-producing commands by the four-entry RX FIFO.
            let mut queued_bits = [0u8; 4];
            let mut queued_bytes = [0usize; 4];
            let mut queued_starts = [0usize; 4];
            while remaining != 0 && (!info.capture || queued < queued_bits.len()) {
                let bits = remaining.min(32);
                let bytes = bits.div_ceil(8);
                let mut tx = 0u32;
                for offset in 0..bytes {
                    if let Some(byte) = tdi.get(byte_index + offset) {
                        tx |= u32::from(*byte) << (offset * 8);
                    }
                }
                self.queue_word(shifter, info.tms, bits as u8, tx, info.capture);
                if info.capture {
                    queued_bits[queued] = bits as u8;
                    queued_bytes[queued] = bytes;
                    queued_starts[queued] = byte_index;
                    queued += 1;
                }
                byte_index += bytes;
                remaining -= bits;
            }

            for index in 0..queued {
                let rx = self.capture_word(shifter, queued_bits[index]);
                if info.capture {
                    for offset in 0..queued_bytes[index] {
                        if let Some(slot) = rxbuf.get_mut(queued_starts[index] + offset) {
                            *slot = (rx >> (offset * 8)) as u8;
                        }
                    }
                }
            }
        }
    }

    /// Shift a TAP walk with TDI high, grouping equal TMS runs into FIFO commands.
    pub fn tms_sequence<S: PioJtagShifter>(&self, shifter: &mut S, tms: &[bool]) {
        let mut index = 0;
        while index < tms.len() {
            let level = tms[index];
            let mut bits = 1usize;
            while index + bits < tms.len() && tms[index + bits] == level && bits < 32 {
                bits += 1;
            }
            self.queue_word(shifter, level, bits as u8, u32::MAX, false);
            index += bits;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::vec::Vec;

    #[derive(Debug, PartialEq, Eq)]
    enum Event {
        Put(u32),
        Get,
    }

    struct Mock {
        tx: Vec<u32>,
        rx: VecDeque<u32>,
        events: Vec<Event>,
    }

    impl Mock {
        fn new(rx: &[u32]) -> Self {
            Self {
                tx: Vec::new(),
                rx: rx.iter().copied().collect(),
                events: Vec::new(),
            }
        }
    }

    impl PioJtagShifter for Mock {
        fn jtag_put(&mut self, word: u32) {
            self.tx.push(word);
            self.events.push(Event::Put(word));
        }
        fn jtag_get(&mut self) -> u32 {
            self.events.push(Event::Get);
            self.rx.pop_front().expect("mock JTAG RX underflow")
        }
        fn set_jtag_clock_divider(&mut self, _divider: PioClockDivider) {}
        fn enter_jtag(&mut self, _dispatcher: u8) {}
        fn exit_jtag(&mut self) {}
    }

    fn engine() -> PioJtag {
        PioJtag::new(
            crate::program::select_jtag_program(125_000_000)
                .unwrap()
                .entries,
        )
    }

    fn raw(value: u32, bits: u8) -> u32 {
        if bits == 32 {
            value
        } else {
            value << (32 - bits)
        }
    }

    #[test]
    fn command_word_packs_count_capture_entry_and_inline_data() {
        assert_eq!(jtag_cmd_word(17, 32, true), 31 | (1 << 5) | (17 << 6));
        assert_eq!(
            jtag_inline_word(24, 10, true, 0x03a5),
            9 | (1 << 5) | (24 << 6) | (0x03a5 << 11)
        );
    }

    #[test]
    fn sixty_four_bit_capture_queues_both_chunks_before_draining() {
        let first = 0x7654_3210;
        let second = 0xFEDC_BA98;
        let tx = [0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE];
        let mut rx = [0u8; 8];
        let mut shifter = Mock::new(&[first, second]);
        engine().sequence(
            &mut shifter,
            SequenceInfo {
                n_bits: 64,
                capture: true,
                tms: false,
            },
            &tx,
            &mut rx,
        );
        assert_eq!(rx, tx);
        assert_eq!(
            shifter.tx,
            [
                jtag_cmd_word(17, 32, true),
                first,
                jtag_cmd_word(17, 32, true),
                second,
            ]
        );
        assert_eq!(
            shifter.events,
            [
                Event::Put(jtag_cmd_word(17, 32, true)),
                Event::Put(first),
                Event::Put(jtag_cmd_word(17, 32, true)),
                Event::Put(second),
                Event::Get,
                Event::Get,
            ]
        );
        assert!(shifter.rx.is_empty());
    }

    #[test]
    fn partial_word_capture_is_lsb_first_and_zero_extending() {
        let mut rx = [0xFFu8; 2];
        let mut shifter = Mock::new(&[raw(0x2D5, 10)]);
        engine().sequence(
            &mut shifter,
            SequenceInfo {
                n_bits: 10,
                capture: true,
                tms: true,
            },
            &[0xA5, 0x03],
            &mut rx,
        );
        assert_eq!(rx, [0xD5, 0x02]);
        assert_eq!(shifter.tx, [jtag_inline_word(24, 10, true, 0x03a5)]);
    }

    #[test]
    fn noncapture_sequences_suppress_rx_fifo_words() {
        let mut untouched = [0xA5];
        let mut shifter = Mock::new(&[]);
        engine().sequence(
            &mut shifter,
            SequenceInfo {
                n_bits: 8,
                capture: false,
                tms: false,
            },
            &[0xC3],
            &mut untouched,
        );
        assert_eq!(untouched, [0xA5]);
        assert_eq!(shifter.tx, [jtag_inline_word(18, 8, false, 0xc3)]);
        assert!(shifter.rx.is_empty());
    }

    #[test]
    fn tms_walk_keeps_the_final_transition_and_groups_runs() {
        let mut shifter = Mock::new(&[]);
        engine().tms_sequence(&mut shifter, &[true, true, false, false, false, true]);
        assert_eq!(
            shifter.tx,
            [
                jtag_inline_word(24, 2, false, u32::MAX),
                jtag_inline_word(18, 3, false, u32::MAX),
                jtag_inline_word(24, 1, false, u32::MAX),
            ]
        );
        assert!(shifter.rx.is_empty());
    }

    #[test]
    fn short_instruction_scan_fuses_tap_walk_and_chain_bits() {
        let mut shifter = Mock::new(&[]);
        engine().shift_ir(&mut shifter, 0x0a, 0, 4, 5);
        let selected_mask = 0x0f << 2;
        assert_eq!(
            shifter.tx,
            [
                jtag_inline_word(24, 2, false, u32::MAX),
                jtag_inline_word(18, 10, false, !selected_mask | (0x0a << 2)),
                jtag_inline_word(24, 2, false, 0x03),
                jtag_inline_word(18, 1, false, u32::MAX),
            ]
        );
        assert!(shifter.rx.is_empty());
    }

    #[test]
    fn one_tap_instruction_scan_exits_on_the_final_ir_bit() {
        let mut shifter = Mock::new(&[]);
        engine().shift_ir(&mut shifter, 0x0a, 0, 4, 0);
        assert_eq!(shifter.tx.len(), 4);
        assert_eq!(shifter.tx[1], jtag_inline_word(18, 5, false, 0x0b));
        assert_eq!(shifter.tx[2], jtag_inline_word(24, 2, false, 0x03));
        assert!(shifter.rx.is_empty());
    }

    #[test]
    fn twenty_one_bits_inline_and_twenty_two_use_a_data_word() {
        let mut inline = Mock::new(&[]);
        engine().queue_word(&mut inline, false, 21, 0x001f_ffff, false);
        assert_eq!(inline.tx, [jtag_inline_word(18, 21, false, 0x001f_ffff)]);

        let mut external = Mock::new(&[]);
        engine().queue_word(&mut external, false, 22, 0x002f_ffff, false);
        assert_eq!(external.tx, [jtag_cmd_word(17, 22, false), 0x002f_ffff]);
    }

    fn transfer_config(idle_cycles: u8) -> TransferConfig {
        TransferConfig {
            idle_cycles,
            wait_retries: 0,
            match_retries: 0,
            match_mask: 0,
        }
    }

    #[test]
    fn complete_transfer_batches_the_data_and_bypass_exit() {
        let captured = 0x7654_3210;
        let mut shifter = Mock::new(&[raw(0x02 << 2, 5), captured]);
        let result = engine().transfer(
            &mut shifter,
            JtagScan {
                device_index: 0,
                device_count: 2,
            },
            RnW::R,
            1,
            &transfer_config(0),
            0xA5A5_5A5A,
        );
        assert_eq!(result, Some((0b010, captured)));
        assert_eq!(
            shifter.tx,
            [
                jtag_inline_word(24, 1, false, u32::MAX),
                jtag_inline_word(18, 5, true, 15),
                jtag_cmd_word(17, 32, true),
                0xA5A5_5A5A,
                jtag_inline_word(24, 2, false, u32::MAX),
                jtag_inline_word(18, 1, false, u32::MAX),
            ]
        );
        assert_eq!(
            shifter.events,
            [
                Event::Put(jtag_inline_word(24, 1, false, u32::MAX)),
                Event::Put(jtag_inline_word(18, 5, true, 15)),
                Event::Get,
                Event::Put(jtag_cmd_word(17, 32, true)),
                Event::Put(0xA5A5_5A5A),
                Event::Put(jtag_inline_word(24, 2, false, u32::MAX)),
                Event::Put(jtag_inline_word(18, 1, false, u32::MAX)),
                Event::Get,
            ]
        );
    }

    #[test]
    fn complete_transfer_exits_on_the_final_data_bit_for_one_tap() {
        let captured = 0xFEDC_BA98;
        let mut shifter = Mock::new(&[
            raw(0x02 << 2, 5),
            raw(captured & 0x7fff_ffff, 31),
            raw(captured >> 31, 2),
        ]);
        let result = engine().transfer(
            &mut shifter,
            JtagScan {
                device_index: 0,
                device_count: 1,
            },
            RnW::W,
            0,
            &transfer_config(0),
            0x0123_4567,
        );
        assert_eq!(result, Some((0b010, captured)));
        assert_eq!(shifter.tx[2], jtag_cmd_word(17, 31, true));
        assert_eq!(shifter.tx[3], 0x0123_4567);
        assert_eq!(shifter.tx[4], jtag_inline_word(24, 2, true, 2));
        assert_eq!(shifter.tx.len(), 6);
    }

    /// A DP executes every ACK but WAIT at Update-DR, so those scans shift the whole request.
    #[test]
    fn every_ack_but_wait_shifts_the_data() {
        for ack in [0b100u32, 0b000, 0b111] {
            let mut shifter = Mock::new(&[raw(ack << 2, 5), raw(0, 31), raw(0, 2)]);
            let one = JtagScan {
                device_index: 0,
                device_count: 1,
            };
            let result = engine().transfer(&mut shifter, one, RnW::W, 0, &transfer_config(0), 7);
            assert_eq!(result, Some((ack, 0)));
            assert_eq!(shifter.tx[2], jtag_cmd_word(17, 31, true));
            assert_eq!(shifter.tx[3], 7);
        }
    }

    #[test]
    fn wait_ack_exits_without_shifting_data() {
        let mut shifter = Mock::new(&[raw(0x01 << 2, 5)]);
        let result = engine().transfer(
            &mut shifter,
            JtagScan {
                device_index: 0,
                device_count: 1,
            },
            RnW::R,
            0,
            &transfer_config(0),
            0,
        );
        assert_eq!(result, Some((0b001, 0)));
        assert_eq!(shifter.tx.len(), 4);
        assert_eq!(shifter.tx[2], jtag_inline_word(24, 2, false, u32::MAX));
        assert_eq!(shifter.tx[3], jtag_inline_word(18, 1, false, u32::MAX));
    }

    #[test]
    fn production_scan_geometries_preserve_ack_and_data_capture() {
        for count in 1..=7 {
            for index in 0..count {
                let request_offset = 2 + u32::from(index);
                let request_bits = request_offset as u8 + 3;
                let captured = 0x7654_3210;
                let mut responses = Vec::from([raw(0x02 << request_offset, request_bits)]);
                if index + 1 == count {
                    responses.push(raw(captured & 0x7fff_ffff, 31));
                    responses.push(raw(captured >> 31, 2));
                } else {
                    responses.push(captured);
                }
                let mut shifter = Mock::new(&responses);
                let result = engine().transfer(
                    &mut shifter,
                    JtagScan {
                        device_index: index,
                        device_count: count,
                    },
                    RnW::R,
                    0,
                    &transfer_config(0),
                    0,
                );
                assert_eq!(result, Some((0b010, captured)));
                assert!(shifter.rx.is_empty());
            }
        }
    }

    #[test]
    fn unsupported_scan_geometry_fails_before_emitting_clocks() {
        for (index, count) in [(0, 0), (1, 1), (28, 29), (0, 35)] {
            let mut shifter = Mock::new(&[]);
            let result = engine().transfer(
                &mut shifter,
                JtagScan {
                    device_index: index,
                    device_count: count,
                },
                RnW::R,
                0,
                &transfer_config(0),
                0,
            );
            assert_eq!(result, None);
            assert!(shifter.events.is_empty());
        }
    }
}
