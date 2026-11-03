//! SWD/SWJ sequence spans over the PIO0 [`PioShifter`]; register transfers run on PIO1.

use crate::codec::{cmd_word, read_align, skip_word};
use crate::jtag::PioJtagShifter;
use crate::program::ProgramEntries;
use crate::shifter::PioShifter;

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

/// The PIO0 sequence engine over a PIO shifter seam.
pub struct PioSwd<S> {
    sh: S,
    entries: ProgramEntries,
}

impl<S: PioShifter> PioSwd<S> {
    /// Build the engine over a shifter and the loaded program's absolute entry PCs.
    pub fn new(shifter: S, entries: ProgramEntries) -> Self {
        Self {
            sh: shifter,
            entries,
        }
    }

    /// Borrow the shifter, e.g. for the platform's `DAP_SWJ_Pins` handling.
    pub fn shifter_mut(&mut self) -> &mut S {
        &mut self.sh
    }

    #[cfg(test)]
    pub(crate) fn entries(&self) -> ProgramEntries {
        self.entries
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

    /// Clock `bits` (`1..=32`) with SWDIO released, returning the right-aligned sample.
    fn read_bits(&mut self, bits: u16) -> u32 {
        self.sh.put(cmd_word(self.entries.read_cmd, false, bits));
        read_align(self.sh.get(), bits as u8)
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

    /// Queue LSB-first TX bits, zero-padding short input; return does not imply wire completion.
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

    /// Clock all requested RX bits LSB-first; a short destination discards excess captured bytes.
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

    /// Complete queued SWD traffic and hand the PIO-owned pads to the JTAG state machine.
    pub fn lend_to_jtag(&mut self, dispatcher: u8)
    where
        S: PioJtagShifter,
    {
        self.drain_and_release();
        self.sh.enter_jtag(dispatcher);
    }

    /// Stop JTAG and restore the SWD state machine at its released dispatcher.
    pub fn reclaim_from_jtag(&mut self)
    where
        S: PioJtagShifter,
    {
        self.sh.exit_jtag();
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
        fn release_nreset(&mut self) {}
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
    fn set_direction_emits_skip_word() {
        let mut swd = PioSwd::new(MockShifter::new(&[]), entries());
        swd.set_direction(true);
        swd.set_direction(false);
        let e = entries();
        assert_eq!(swd.sh.tx[0], skip_word(e.get_next_cmd, true));
        assert_eq!(swd.sh.tx[1], skip_word(e.get_next_cmd, false));
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
}
