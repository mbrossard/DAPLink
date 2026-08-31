//! The SWD protocol sequencing, driven over a [`PioShifter`].

use crate::codec::{cmd_word, read_align, skip_word};
use crate::program::ProgramEntries;
use crate::proto::{ack_from_frame, parity32, request_byte, Ack};
use crate::shifter::PioShifter;

/// The turnaround length the engine starts at; dap-rs fixes it at one clock.
pub const DEFAULT_TURNAROUND: u8 = 1;

/// The largest bit count one command word encodes.
const MAX_CMD_BITS: usize = 256;

/// A single SWD transfer error, mapped by the adapter onto the dap-rs error type.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum XferError {
    /// ACK was WAIT (target busy).
    AckWait,
    /// ACK was FAULT (sticky error).
    AckFault,
    /// ACK was NoAck (no target responded).
    NoAck,
    /// Read-back parity did not match the data.
    BadParity,
    /// ACK held an undefined value.
    Unknown(u8),
}

/// The SWD protocol engine over a PIO shifter seam.
pub struct PioSwd<S> {
    sh: S,
    entries: ProgramEntries,
    trn: u8,
}

impl<S: PioShifter> PioSwd<S> {
    /// Build the engine over a shifter and the loaded program's absolute entry PCs.
    pub fn new(shifter: S, entries: ProgramEntries) -> Self {
        Self {
            sh: shifter,
            entries,
            trn: DEFAULT_TURNAROUND,
        }
    }

    /// Borrow the shifter, e.g. for the platform's `DAP_SWJ_Pins` handling.
    pub fn shifter_mut(&mut self) -> &mut S {
        &mut self.sh
    }

    /// The absolute program entry PCs this engine was built with.
    pub fn entries(&self) -> ProgramEntries {
        self.entries
    }

    /// Set the turnaround length in SWCLK cycles; SWD permits 1 to 4.
    pub fn set_turnaround(&mut self, cycles: u8) {
        self.trn = cycles.clamp(1, 4);
    }

    /// The current turnaround length.
    pub fn turnaround(&self) -> u8 {
        self.trn
    }

    /// Set the encoded integer divider directly; zero means 65,536.
    pub fn set_clock_divider(&mut self, divider: u16) {
        self.sh.set_clock_divider(divider);
    }

    /// Drive `bits` clocks with SWDIO driving `data`, LSB first.
    fn write_bits(&mut self, bits: u16, data: u32) {
        self.sh.put(cmd_word(self.entries.write_cmd, true, bits));
        self.sh.put(data);
    }

    /// Drive `bits` clocks with SWDIO released; no result.
    fn hiz(&mut self, bits: u16) {
        self.sh
            .put(cmd_word(self.entries.turnaround_cmd, false, bits));
        self.sh.put(0);
    }

    /// Clock `bits` (`1..=32`) with SWDIO released, returning the right-aligned sample.
    fn read_bits(&mut self, bits: u16) -> u32 {
        self.sh.put(cmd_word(self.entries.read_cmd, false, bits));
        read_align(self.sh.get(), bits as u8)
    }

    /// Clock `bits` released and discard the result; the width may exceed 32 bits.
    fn read_discard(&mut self, bits: u16) {
        self.sh.put(cmd_word(self.entries.read_cmd, false, bits));
        let _ = self.sh.get();
    }

    /// Set SWDIO direction without clocking a data phase.
    pub fn set_direction(&mut self, drive: bool) {
        self.sh.put(skip_word(self.entries.get_next_cmd, drive));
    }

    /// Complete prior TX traffic, release SWDIO in FIFO order, and wait for confirmation.
    pub fn drain_and_release(&mut self) {
        self.sh.put(cmd_word(self.entries.release_cmd, false, 1));
        let _ = self.sh.get();
    }

    /// Perform an SWD read transfer of `addr` (`AP` when `ap`).
    pub fn read_transfer(&mut self, ap: bool, addr: u8) -> Result<u32, XferError> {
        let req = request_byte(ap, true, addr) as u32;
        self.write_bits(8, req);
        let raw = self.read_bits((self.trn + 3) as u16);
        match Ack::from_bits(ack_from_frame(raw, self.trn)) {
            Ack::Ok => {
                let val = self.read_bits(32);
                let par = (self.read_bits(1) & 1) as u8;
                // Trailing turnaround hands the bus back with SWDIO still released.
                self.hiz(self.trn as u16);
                if parity32(val) != par {
                    return Err(XferError::BadParity);
                }
                Ok(val)
            }
            Ack::Wait => {
                self.hiz(self.trn as u16);
                Err(XferError::AckWait)
            }
            Ack::Fault => {
                self.hiz(self.trn as u16);
                Err(XferError::AckFault)
            }
            Ack::NoAck => {
                self.protocol_backoff();
                Err(XferError::NoAck)
            }
            Ack::Unknown(v) => {
                self.protocol_backoff();
                Err(XferError::Unknown(v))
            }
        }
    }

    /// Perform an SWD write transfer of `val` to `addr` (`AP` when `ap`).
    pub fn write_transfer(&mut self, ap: bool, addr: u8, val: u32) -> Result<(), XferError> {
        let req = request_byte(ap, false, addr) as u32;
        self.write_bits(8, req);
        let raw = self.read_bits((self.trn + 3) as u16);
        match Ack::from_bits(ack_from_frame(raw, self.trn)) {
            Ack::Ok => {
                // Turnaround back to the host, then the data phase and its parity.
                self.hiz(self.trn as u16);
                self.write_bits(32, val);
                self.write_bits(1, parity32(val) as u32);
                Ok(())
            }
            Ack::Wait => {
                self.hiz(self.trn as u16);
                Err(XferError::AckWait)
            }
            Ack::Fault => {
                self.hiz(self.trn as u16);
                Err(XferError::AckFault)
            }
            Ack::NoAck => {
                self.protocol_backoff();
                Err(XferError::NoAck)
            }
            Ack::Unknown(v) => {
                self.protocol_backoff();
                Err(XferError::Unknown(v))
            }
        }
    }

    /// After a protocol error, clock a full data phase released so the line is left clean.
    fn protocol_backoff(&mut self) {
        self.read_discard((self.trn as u16) + 32 + 1);
    }

    /// Drive an arbitrary TX bit sequence, chunked to the 32-bit command limit, LSB first.
    pub fn write_sequence(&mut self, num_bits: usize, data: &[u8]) {
        let mut remaining = num_bits;
        let mut idx = 0;
        while remaining > 0 {
            let chunk = remaining.min(32);
            let nbytes = chunk.div_ceil(8);
            let mut word = 0u32;
            let mut b = 0;
            while b < nbytes {
                if let Some(byte) = data.get(idx + b) {
                    word |= (*byte as u32) << (8 * b);
                }
                b += 1;
            }
            self.write_bits(chunk as u16, word);
            idx += nbytes;
            remaining -= chunk;
        }
    }

    /// Read an arbitrary RX bit sequence, chunked to the 32-bit command limit, LSB first.
    pub fn read_sequence(&mut self, num_bits: usize, out: &mut [u8]) {
        let mut remaining = num_bits;
        let mut idx = 0;
        while remaining > 0 {
            let chunk = remaining.min(32);
            let raw = self.read_bits(chunk as u16);
            let nbytes = chunk.div_ceil(8);
            let mut b = 0;
            while b < nbytes {
                if let Some(slot) = out.get_mut(idx + b) {
                    *slot = (raw >> (8 * b)) as u8;
                }
                b += 1;
            }
            idx += nbytes;
            remaining -= chunk;
        }
    }

    /// Clock `n` idle cycles driving zero; the all-zero tail may use the full 256-bit form.
    pub fn idle_cycles(&mut self, n: usize) {
        let mut remaining = n;
        while remaining > 0 {
            let chunk = remaining.min(MAX_CMD_BITS);
            self.write_bits(chunk as u16, 0);
            remaining -= chunk;
        }
    }

    /// Quiesce the SM and hand the shared pads to SIO; the caller stages levels beforehand.
    pub fn lend_pads(&mut self) {
        self.drain_and_release();
        self.sh.set_enabled(false);
        self.sh.pads_to_sio();
    }

    /// Hand the pads back and re-seat the SM, whatever state it was parked in.
    pub fn reclaim_pads(&mut self) {
        self.sh.pads_to_pio();
        self.recover();
    }

    /// Destructive stuck-SM recovery for callers that explicitly choose to discard queued work.
    pub fn recover(&mut self) {
        self.sh.set_enabled(false);
        self.sh.clear_fifos();
        self.sh.restart();
        self.sh.force_oen_released();
        self.sh.force_swclk_low();
        self.sh.exec_jmp(self.entries.get_next_cmd);
        self.sh.force_pindirs_idle();
        self.sh.set_enabled(true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use daplink_rs::dap_rs::swj::Pins;
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;

    /// Recorded shifter ops beyond FIFO traffic, for asserting the recovery order.
    #[derive(Debug, PartialEq, Eq)]
    enum Op {
        Enable(bool),
        OenReleased,
        SwclkLow,
        ClearFifos,
        Restart,
        Jmp(u8),
        PindirsIdle,
        Divider(u16),
        PadsToSio,
        PadsToPio,
    }

    /// A recording mock: captures every TX word, replays queued RX words, logs control ops.
    struct MockShifter {
        tx: Vec<u32>,
        rx: VecDeque<u32>,
        ops: Vec<Op>,
    }

    impl MockShifter {
        fn new(rx: &[u32]) -> Self {
            Self {
                tx: Vec::new(),
                rx: rx.iter().copied().collect(),
                ops: Vec::new(),
            }
        }
    }

    impl PioShifter for MockShifter {
        fn put(&mut self, word: u32) {
            self.tx.push(word);
        }
        fn get(&mut self) -> u32 {
            self.rx.pop_front().expect("mock RX underflow")
        }
        fn set_clock_divider(&mut self, divider: u16) {
            self.ops.push(Op::Divider(divider));
        }
        fn set_enabled(&mut self, enabled: bool) {
            self.ops.push(Op::Enable(enabled));
        }
        fn clear_fifos(&mut self) {
            self.ops.push(Op::ClearFifos);
        }
        fn restart(&mut self) {
            self.ops.push(Op::Restart);
        }
        fn exec_jmp(&mut self, addr: u8) {
            self.ops.push(Op::Jmp(addr));
        }
        fn force_oen_released(&mut self) {
            self.ops.push(Op::OenReleased);
        }
        fn force_swclk_low(&mut self) {
            self.ops.push(Op::SwclkLow);
        }
        fn force_pindirs_idle(&mut self) {
            self.ops.push(Op::PindirsIdle);
        }
        fn pads_to_sio(&mut self) {
            self.ops.push(Op::PadsToSio);
        }
        fn pads_to_pio(&mut self) {
            self.ops.push(Op::PadsToPio);
        }
    }

    impl crate::shifter::SwjPinsSio for MockShifter {
        fn swj_pins(&mut self, _output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
            Pins::empty()
        }
    }

    /// FIFO-order model: an RX read completes every TX word queued before its response.
    struct OrderedShifter {
        pending: Vec<u32>,
        completed: Vec<u32>,
        rx: VecDeque<u32>,
        ops: Vec<Op>,
    }

    impl OrderedShifter {
        fn new(rx: &[u32]) -> Self {
            Self {
                pending: Vec::new(),
                completed: Vec::new(),
                rx: rx.iter().copied().collect(),
                ops: Vec::new(),
            }
        }
    }

    impl PioShifter for OrderedShifter {
        fn put(&mut self, word: u32) {
            self.pending.push(word);
        }
        fn get(&mut self) -> u32 {
            self.completed.append(&mut self.pending);
            self.rx.pop_front().unwrap_or(0)
        }
        fn set_clock_divider(&mut self, divider: u16) {
            self.ops.push(Op::Divider(divider));
        }
        fn set_enabled(&mut self, enabled: bool) {
            self.ops.push(Op::Enable(enabled));
        }
        fn clear_fifos(&mut self) {
            self.pending.clear();
            self.rx.clear();
            self.ops.push(Op::ClearFifos);
        }
        fn restart(&mut self) {
            self.ops.push(Op::Restart);
        }
        fn exec_jmp(&mut self, addr: u8) {
            self.ops.push(Op::Jmp(addr));
        }
        fn force_oen_released(&mut self) {
            self.ops.push(Op::OenReleased);
        }
        fn force_swclk_low(&mut self) {
            self.ops.push(Op::SwclkLow);
        }
        fn force_pindirs_idle(&mut self) {
            self.ops.push(Op::PindirsIdle);
        }
        fn pads_to_sio(&mut self) {
            self.ops.push(Op::PadsToSio);
        }
        fn pads_to_pio(&mut self) {
            self.ops.push(Op::PadsToPio);
        }
    }

    fn entries() -> ProgramEntries {
        crate::program::DIRECT.entries(0)
    }

    fn variants() -> [ProgramEntries; 2] {
        [
            crate::program::DIRECT.entries(0),
            crate::program::OEN.entries(0),
        ]
    }

    /// Place a logical value in the top `nbits`, the inverse of `read_align`.
    fn raw(value: u32, nbits: u8) -> u32 {
        if nbits >= 32 {
            value
        } else {
            value << (32 - nbits as u32)
        }
    }

    #[test]
    fn read_transfer_ok_returns_data_and_composes_words() {
        let trn = 1u8;
        let data = 0x1234_5678u32;
        let ack_raw = raw(0b001 << trn, trn + 3);
        let data_raw = raw(data, 32);
        let par_raw = raw(parity32(data) as u32, 1);
        let mut swd = PioSwd::new(MockShifter::new(&[ack_raw, data_raw, par_raw]), entries());

        let got = swd.read_transfer(false, 0).unwrap();
        assert_eq!(got, data);

        let e = entries();
        let tx = &swd.sh.tx;
        assert_eq!(tx[0], cmd_word(e.write_cmd, true, 8));
        assert_eq!(tx[1], request_byte(false, true, 0) as u32);
        assert_eq!(tx[2], cmd_word(e.read_cmd, false, (trn + 3) as u16));
        assert_eq!(tx[3], cmd_word(e.read_cmd, false, 32));
        assert_eq!(tx[4], cmd_word(e.read_cmd, false, 1));
        assert_eq!(tx[5], cmd_word(e.turnaround_cmd, false, trn as u16));
        assert_eq!(tx[6], 0);
        assert_eq!(tx.len(), 7);
    }

    #[test]
    fn read_transfer_bad_parity() {
        let trn = 1u8;
        let data = 0x0000_0001u32;
        let ack_raw = raw(0b001 << trn, trn + 3);
        let data_raw = raw(data, 32);
        let wrong_par = raw(0, 1);
        let mut swd = PioSwd::new(MockShifter::new(&[ack_raw, data_raw, wrong_par]), entries());
        assert_eq!(swd.read_transfer(false, 0), Err(XferError::BadParity));
    }

    #[test]
    fn read_transfer_wait_and_fault() {
        let trn = 1u8;
        for (bits, err) in [(0b010u32, XferError::AckWait), (0b100, XferError::AckFault)] {
            let ack_raw = raw(bits << trn, trn + 3);
            let mut swd = PioSwd::new(MockShifter::new(&[ack_raw]), entries());
            assert_eq!(swd.read_transfer(false, 0), Err(err));
            // WAIT/FAULT still clock the trailing turnaround: 2 header + 1 ack + 2 hiz.
            assert_eq!(swd.sh.tx.len(), 5);
        }
    }

    #[test]
    fn read_transfer_noack_backs_off() {
        let trn = 1u8;
        let ack_raw = raw(0b111 << trn, trn + 3);
        let mut swd = PioSwd::new(MockShifter::new(&[ack_raw, 0]), entries());
        assert_eq!(swd.read_transfer(false, 0), Err(XferError::NoAck));
        let e = entries();
        let last = *swd.sh.tx.last().unwrap();
        assert_eq!(last, cmd_word(e.read_cmd, false, (trn as u16) + 32 + 1));
    }

    #[test]
    fn write_transfer_ok_drives_data_and_parity() {
        let trn = 1u8;
        let val = 0xDEAD_BEEFu32;
        let ack_raw = raw(0b001 << trn, trn + 3);
        let mut swd = PioSwd::new(MockShifter::new(&[ack_raw]), entries());
        assert!(swd.write_transfer(false, 2, val).is_ok());

        let e = entries();
        let tx = &swd.sh.tx;
        assert_eq!(tx[0], cmd_word(e.write_cmd, true, 8));
        assert_eq!(tx[1], request_byte(false, false, 2) as u32);
        assert_eq!(tx[2], cmd_word(e.read_cmd, false, (trn + 3) as u16));
        assert_eq!(tx[3], cmd_word(e.turnaround_cmd, false, trn as u16));
        assert_eq!(tx[4], 0);
        assert_eq!(tx[5], cmd_word(e.write_cmd, true, 32));
        assert_eq!(tx[6], val);
        assert_eq!(tx[7], cmd_word(e.write_cmd, true, 1));
        assert_eq!(tx[8], parity32(val) as u32);
    }

    #[test]
    fn write_sequence_line_reset_chunks() {
        // A line reset is at least 50 clocks of SWDIO high; 51 ones fit in 7 bytes.
        let bytes = [0xFFu8; 7];
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.write_sequence(51, &bytes);
        let e = entries();
        let tx = &swd.sh.tx;
        assert_eq!(tx.len(), 4);
        assert_eq!(tx[0], cmd_word(e.write_cmd, true, 32));
        assert_eq!(tx[1], 0xFFFF_FFFF);
        assert_eq!(tx[2], cmd_word(e.write_cmd, true, 19));
        assert_eq!(tx[3] & 0x7_FFFF, 0x7_FFFF);
    }

    #[test]
    fn every_ack_transcript_is_exact_for_both_programs() {
        let value = 0xA5C3_691Eu32;
        for entries in variants() {
            for trn in 1..=4u8 {
                for ack in 0..=7u8 {
                    for ap in [false, true] {
                        for addr in 0..=3u8 {
                            let ack_raw = raw((ack as u32) << trn, trn + 3);
                            let mut read_rx = vec![ack_raw];
                            if ack == 1 {
                                read_rx.push(value);
                                read_rx.push(raw(parity32(value) as u32, 1));
                            } else if !matches!(ack, 2 | 4) {
                                read_rx.push(0);
                            }
                            let mut read = PioSwd::new(MockShifter::new(&read_rx), entries);
                            read.set_turnaround(trn);
                            let read_result = read.read_transfer(ap, addr);

                            let mut read_tx = vec![
                                cmd_word(entries.write_cmd, true, 8),
                                request_byte(ap, true, addr) as u32,
                                cmd_word(entries.read_cmd, false, (trn + 3) as u16),
                            ];
                            let expected_read = match ack {
                                1 => {
                                    read_tx.extend([
                                        cmd_word(entries.read_cmd, false, 32),
                                        cmd_word(entries.read_cmd, false, 1),
                                        cmd_word(entries.turnaround_cmd, false, trn as u16),
                                        0,
                                    ]);
                                    Ok(value)
                                }
                                2 => {
                                    read_tx.extend([
                                        cmd_word(entries.turnaround_cmd, false, trn as u16),
                                        0,
                                    ]);
                                    Err(XferError::AckWait)
                                }
                                4 => {
                                    read_tx.extend([
                                        cmd_word(entries.turnaround_cmd, false, trn as u16),
                                        0,
                                    ]);
                                    Err(XferError::AckFault)
                                }
                                7 => {
                                    read_tx.push(cmd_word(
                                        entries.read_cmd,
                                        false,
                                        trn as u16 + 33,
                                    ));
                                    Err(XferError::NoAck)
                                }
                                other => {
                                    read_tx.push(cmd_word(
                                        entries.read_cmd,
                                        false,
                                        trn as u16 + 33,
                                    ));
                                    Err(XferError::Unknown(other))
                                }
                            };
                            assert_eq!(read_result, expected_read);
                            assert_eq!(read.sh.tx, read_tx);
                            assert!(read.sh.rx.is_empty());

                            let mut write_rx = vec![ack_raw];
                            if !matches!(ack, 1 | 2 | 4) {
                                write_rx.push(0);
                            }
                            let mut write = PioSwd::new(MockShifter::new(&write_rx), entries);
                            write.set_turnaround(trn);
                            let write_result = write.write_transfer(ap, addr, value);

                            let mut write_tx = vec![
                                cmd_word(entries.write_cmd, true, 8),
                                request_byte(ap, false, addr) as u32,
                                cmd_word(entries.read_cmd, false, (trn + 3) as u16),
                            ];
                            let expected_write = match ack {
                                1 => {
                                    write_tx.extend([
                                        cmd_word(entries.turnaround_cmd, false, trn as u16),
                                        0,
                                        cmd_word(entries.write_cmd, true, 32),
                                        value,
                                        cmd_word(entries.write_cmd, true, 1),
                                        parity32(value) as u32,
                                    ]);
                                    Ok(())
                                }
                                2 => {
                                    write_tx.extend([
                                        cmd_word(entries.turnaround_cmd, false, trn as u16),
                                        0,
                                    ]);
                                    Err(XferError::AckWait)
                                }
                                4 => {
                                    write_tx.extend([
                                        cmd_word(entries.turnaround_cmd, false, trn as u16),
                                        0,
                                    ]);
                                    Err(XferError::AckFault)
                                }
                                7 => {
                                    write_tx.push(cmd_word(
                                        entries.read_cmd,
                                        false,
                                        trn as u16 + 33,
                                    ));
                                    Err(XferError::NoAck)
                                }
                                other => {
                                    write_tx.push(cmd_word(
                                        entries.read_cmd,
                                        false,
                                        trn as u16 + 33,
                                    ));
                                    Err(XferError::Unknown(other))
                                }
                            };
                            assert_eq!(write_result, expected_write);
                            assert_eq!(write.sh.tx, write_tx);
                            assert!(write.sh.rx.is_empty());
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn bad_parity_keeps_the_success_transcript_for_both_programs() {
        let value = 0x1234_5678u32;
        for entries in variants() {
            for trn in 1..=4u8 {
                let ack_raw = raw(0b001 << trn, trn + 3);
                let wrong_parity = raw((parity32(value) ^ 1) as u32, 1);
                let mut swd =
                    PioSwd::new(MockShifter::new(&[ack_raw, value, wrong_parity]), entries);
                swd.set_turnaround(trn);
                assert_eq!(swd.read_transfer(false, 0), Err(XferError::BadParity));
                assert_eq!(
                    swd.sh.tx,
                    vec![
                        cmd_word(entries.write_cmd, true, 8),
                        request_byte(false, true, 0) as u32,
                        cmd_word(entries.read_cmd, false, (trn + 3) as u16),
                        cmd_word(entries.read_cmd, false, 32),
                        cmd_word(entries.read_cmd, false, 1),
                        cmd_word(entries.turnaround_cmd, false, trn as u16),
                        0,
                    ]
                );
            }
        }
    }

    #[test]
    fn write_sequence_jtag_to_swd_select_word() {
        // The JTAG-to-SWD select sequence is 0xE79E, LSB first on the wire.
        let bytes = 0xE79Eu16.to_le_bytes();
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.write_sequence(16, &bytes);
        let e = entries();
        assert_eq!(swd.sh.tx[0], cmd_word(e.write_cmd, true, 16));
        assert_eq!(swd.sh.tx[1] & 0xFFFF, 0xE79E);
    }

    #[test]
    fn read_sequence_stores_lsb_first() {
        let mut swd = PioSwd::new(MockShifter::new(&[raw(0xA5, 8), raw(0x0B, 4)]), entries());
        let mut out = [0u8; 2];
        swd.read_sequence(8, &mut out[..1]);
        swd.read_sequence(4, &mut out[1..]);
        assert_eq!(out, [0xA5, 0x0B]);
    }

    #[test]
    fn idle_cycles_use_zero_tail_chunks() {
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.idle_cycles(300);
        let e = entries();
        let tx = &swd.sh.tx;
        assert_eq!(tx.len(), 4);
        assert_eq!(tx[0], cmd_word(e.write_cmd, true, 256));
        assert_eq!(tx[1], 0);
        assert_eq!(tx[2], cmd_word(e.write_cmd, true, 44));
        assert_eq!(tx[3], 0);
    }

    #[test]
    fn set_direction_emits_skip_word() {
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.set_direction(true);
        swd.set_direction(false);
        let e = entries();
        assert_eq!(swd.sh.tx[0], skip_word(e.get_next_cmd, true));
        assert_eq!(swd.sh.tx[1], skip_word(e.get_next_cmd, false));
    }

    #[test]
    fn clean_release_completes_successful_write_tail_for_both_programs() {
        let value = 0xC3A5_9669;
        for entries in variants() {
            let ack = raw(0b001 << DEFAULT_TURNAROUND, DEFAULT_TURNAROUND + 3);
            let mut swd = PioSwd::new(OrderedShifter::new(&[ack]), entries);
            assert_eq!(swd.write_transfer(false, 2, value), Ok(()));

            let tail = swd.sh.pending.clone();
            assert_eq!(
                tail,
                vec![
                    cmd_word(entries.turnaround_cmd, false, 1),
                    0,
                    cmd_word(entries.write_cmd, true, 32),
                    value,
                    cmd_word(entries.write_cmd, true, 1),
                    parity32(value) as u32,
                ]
            );

            swd.drain_and_release();
            assert!(swd.sh.pending.is_empty());
            assert_eq!(
                &swd.sh.completed[swd.sh.completed.len() - tail.len() - 1..],
                &[tail.as_slice(), &[cmd_word(entries.release_cmd, false, 1)],].concat()
            );
            assert!(!swd.sh.ops.contains(&Op::ClearFifos));
        }
    }

    #[test]
    fn clean_release_completes_pending_sequence_for_both_programs() {
        for entries in variants() {
            let mut swd = PioSwd::new(OrderedShifter::new(&[]), entries);
            swd.write_sequence(40, &[0xA5, 0x5A, 0xC3, 0x3C, 0x69]);
            let sequence = swd.sh.pending.clone();
            assert!(!sequence.is_empty());

            swd.drain_and_release();
            assert!(swd.sh.pending.is_empty());
            assert_eq!(&swd.sh.completed[..sequence.len()], sequence.as_slice());
            assert_eq!(
                swd.sh.completed.last(),
                Some(&cmd_word(entries.release_cmd, false, 1))
            );
            assert!(!swd.sh.ops.contains(&Op::ClearFifos));
        }
    }

    #[test]
    fn forced_recovery_discards_pending_traffic_after_staging_idle_levels() {
        let mut swd = PioSwd::new(OrderedShifter::new(&[0xDEAD_BEEF]), entries());
        swd.write_sequence(8, &[0xA5]);
        assert!(!swd.sh.pending.is_empty());
        swd.recover();
        assert!(swd.sh.pending.is_empty());
        assert!(swd.sh.rx.is_empty());
        assert_eq!(
            swd.sh.ops,
            vec![
                Op::Enable(false),
                Op::ClearFifos,
                Op::Restart,
                Op::OenReleased,
                Op::SwclkLow,
                Op::Jmp(entries().get_next_cmd),
                Op::PindirsIdle,
                Op::Enable(true),
            ]
        );
    }

    #[test]
    fn initialization_stages_inactive_oen_before_exposing_output_directions() {
        let mut shifter = MockShifter::new(&[]);
        crate::shifter::start_idle(&mut shifter, entries().get_next_cmd);
        assert_eq!(
            shifter.ops,
            vec![
                Op::OenReleased,
                Op::SwclkLow,
                Op::PindirsIdle,
                Op::Jmp(entries().get_next_cmd),
                Op::Enable(true),
            ]
        );
    }

    #[test]
    fn recover_runs_steps_in_order() {
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.recover();
        let e = entries();
        assert_eq!(
            swd.sh.ops,
            vec![
                Op::Enable(false),
                Op::ClearFifos,
                Op::Restart,
                Op::OenReleased,
                Op::SwclkLow,
                Op::Jmp(e.get_next_cmd),
                Op::PindirsIdle,
                Op::Enable(true),
            ]
        );
    }

    #[test]
    fn lend_pads_quiesces_the_sm_before_moving_the_function_select() {
        let mut swd = PioSwd::new(MockShifter::new(&[0]), entries());
        swd.lend_pads();
        // A running SM whose pad function is switched out from under it glitches the wire.
        assert_eq!(swd.sh.ops, vec![Op::Enable(false), Op::PadsToSio]);
        assert_eq!(swd.sh.tx, vec![cmd_word(entries().release_cmd, false, 1)]);
    }

    #[test]
    fn reclaim_pads_moves_the_function_select_before_re_seating_the_sm() {
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.reclaim_pads();
        let e = entries();
        assert_eq!(
            swd.sh.ops,
            vec![
                Op::PadsToPio,
                Op::Enable(false),
                Op::ClearFifos,
                Op::Restart,
                Op::OenReleased,
                Op::SwclkLow,
                Op::Jmp(e.get_next_cmd),
                Op::PindirsIdle,
                Op::Enable(true),
            ]
        );
    }

    #[test]
    fn a_full_loan_leaves_the_sm_enabled_and_the_pads_on_pio() {
        let mut swd = PioSwd::new(MockShifter::new(&[0]), entries());
        swd.lend_pads();
        swd.reclaim_pads();
        assert_eq!(swd.sh.ops.last(), Some(&Op::Enable(true)));
        let last_pad = swd
            .sh
            .ops
            .iter()
            .rposition(|o| matches!(o, Op::PadsToSio | Op::PadsToPio));
        assert_eq!(swd.sh.ops[last_pad.expect("pads moved")], Op::PadsToPio);
    }

    #[test]
    fn turnaround_two_widens_ack_and_hiz() {
        let mut swd = PioSwd::new(MockShifter::new(&[raw(0b001 << 2, 2 + 3)]), entries());
        swd.set_turnaround(2);
        assert_eq!(swd.turnaround(), 2);
        assert_eq!(swd.write_transfer(false, 0, 0), Ok(()));
        let e = entries();
        assert_eq!(swd.sh.tx[2], cmd_word(e.read_cmd, false, 5));
        assert_eq!(swd.sh.tx[3], cmd_word(e.turnaround_cmd, false, 2));
    }

    #[test]
    fn set_turnaround_clamps() {
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.set_turnaround(0);
        assert_eq!(swd.turnaround(), 1);
        swd.set_turnaround(9);
        assert_eq!(swd.turnaround(), 4);
    }
}
