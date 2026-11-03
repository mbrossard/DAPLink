//! Host-side ACK decision for the complete-suffix SWD state machine.

use crate::codec::read_align;
use crate::proto::{parity32, request_byte, Ack};
use crate::sequence::XferError;
use crate::transaction_program::{READ_CONTINUATION, RELEASE_CONTINUATION, WRITE_CONTINUATION};

/// FIFO operations required by [`PioSwdTransaction`].
pub trait PioSwdTransactionIo {
    /// Push one word to the state machine.
    fn put(&mut self, word: u32);

    /// Pop one word from the state machine.
    fn get(&mut self) -> u32;
}

/// Fixed-request SWD transfers over an ACK-continuation PIO program.
pub struct PioSwdTransaction<S> {
    io: S,
}

impl<S: PioSwdTransactionIo> PioSwdTransaction<S> {
    /// Build a transaction engine over one configured state machine.
    pub const fn new(io: S) -> Self {
        Self { io }
    }

    /// Borrow the state-machine interface.
    pub fn io_mut(&mut self) -> &mut S {
        &mut self.io
    }

    /// Recover the state-machine interface.
    pub fn into_inner(self) -> S {
        self.io
    }

    /// Read data/parity; the release suffix may still run until the composition's command fence.
    pub fn read_transfer(&mut self, ap: bool, addr: u8) -> Result<u32, XferError> {
        self.io.put(u32::from(request_byte(ap, true, addr)));
        let ack = self.read_ack();
        match ack {
            Ack::Ok => {
                self.io.put(u32::from(READ_CONTINUATION));
                let data = self.io.get();
                let parity = read_align(self.io.get(), 1) as u8;
                if parity32(data) == parity {
                    Ok(data)
                } else {
                    Err(XferError::BadParity)
                }
            }
            // Every failed ACK gets only the released turnaround, as dap-rs clocks.
            Ack::Wait => {
                self.release_after_ack();
                Err(XferError::AckWait)
            }
            Ack::Fault => {
                self.release_after_ack();
                Err(XferError::AckFault)
            }
            Ack::NoAck => {
                self.release_after_ack();
                Err(XferError::NoAck)
            }
            Ack::Unknown(value) => {
                self.release_after_ack();
                Err(XferError::Unknown(value))
            }
        }
    }

    /// Accept ACK and queue the write suffix; the composition's command fence waits for completion.
    pub fn write_transfer(&mut self, ap: bool, addr: u8, data: u32) -> Result<(), XferError> {
        self.io.put(u32::from(request_byte(ap, false, addr)));
        let ack = self.read_ack();
        match ack {
            Ack::Ok => {
                let continuation = u32::from(WRITE_CONTINUATION) | (u32::from(parity32(data)) << 5);
                self.io.put(continuation);
                self.io.put(data);
                Ok(())
            }
            // As for reads, dap-rs clocks one turnaround after any failed write ACK.
            Ack::Wait => {
                self.release_after_ack();
                Err(XferError::AckWait)
            }
            Ack::Fault => {
                self.release_after_ack();
                Err(XferError::AckFault)
            }
            Ack::NoAck => {
                self.release_after_ack();
                Err(XferError::NoAck)
            }
            Ack::Unknown(value) => {
                self.release_after_ack();
                Err(XferError::Unknown(value))
            }
        }
    }

    fn read_ack(&mut self) -> Ack {
        Ack::from_bits(read_align(self.io.get(), 3) as u8)
    }

    fn release_after_ack(&mut self) {
        self.io.put(u32::from(RELEASE_CONTINUATION));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;

    #[derive(Debug, PartialEq, Eq)]
    enum Event {
        Put(u32),
        Get(u32),
    }

    struct MockIo {
        rx: VecDeque<u32>,
        events: Vec<Event>,
    }

    impl MockIo {
        fn new(rx: &[u32]) -> Self {
            Self {
                rx: rx.iter().copied().collect(),
                events: Vec::new(),
            }
        }
    }

    impl PioSwdTransactionIo for MockIo {
        fn put(&mut self, word: u32) {
            self.events.push(Event::Put(word));
        }

        fn get(&mut self) -> u32 {
            let word = self.rx.pop_front().expect("RX underflow");
            self.events.push(Event::Get(word));
            word
        }
    }

    fn ack_word(ack: u8) -> u32 {
        u32::from(ack) << 29
    }

    fn expected_error(ack: u8) -> Option<XferError> {
        match Ack::from_bits(ack) {
            Ack::Ok => None,
            Ack::Wait => Some(XferError::AckWait),
            Ack::Fault => Some(XferError::AckFault),
            Ack::NoAck => Some(XferError::NoAck),
            Ack::Unknown(value) => Some(XferError::Unknown(value)),
        }
    }

    #[test]
    fn all_read_ack_values_drain_the_expected_fifo_shape() {
        const DATA: u32 = 0xa5c3_78f0;
        for ack in 0..=7 {
            let mut rx = vec![ack_word(ack)];
            if matches!(Ack::from_bits(ack), Ack::Ok) {
                rx.push(DATA);
                rx.push(u32::from(parity32(DATA)) << 31);
            }
            let mut engine = PioSwdTransaction::new(MockIo::new(&rx));
            let result = engine.read_transfer(true, 2);
            assert_eq!(
                result,
                expected_error(ack).map_or(Ok(DATA), Err),
                "ACK {ack:03b}"
            );
            assert!(engine.io.rx.is_empty(), "ACK {ack:03b} left RX data");
        }
    }

    #[test]
    fn all_write_ack_values_drain_the_expected_fifo_shape() {
        const DATA: u32 = 0x0123_4567;
        for ack in 0..=7 {
            let rx = [ack_word(ack)];
            let mut engine = PioSwdTransaction::new(MockIo::new(&rx));
            let result = engine.write_transfer(false, 1, DATA);
            assert_eq!(
                result,
                expected_error(ack).map_or(Ok(()), Err),
                "ACK {ack:03b}"
            );
            assert!(engine.io.rx.is_empty(), "ACK {ack:03b} left RX data");
        }
    }

    #[test]
    fn ack_uses_only_the_three_top_aligned_bits() {
        let mut engine =
            PioSwdTransaction::new(MockIo::new(&[ack_word(0b001) | 0x1fff_ffff, 0, 0]));
        assert_eq!(engine.read_transfer(false, 0), Ok(0));
    }

    #[test]
    fn read_parity_comes_from_bit_31() {
        let data = 1;
        let mut good = PioSwdTransaction::new(MockIo::new(&[ack_word(0b001), data, 1 << 31]));
        assert_eq!(good.read_transfer(false, 0), Ok(data));

        let mut low_bit_only = PioSwdTransaction::new(MockIo::new(&[ack_word(0b001), data, 1]));
        assert_eq!(
            low_bit_only.read_transfer(false, 0),
            Err(XferError::BadParity)
        );
    }

    #[test]
    fn ok_transcripts_use_two_read_and_three_write_words() {
        let read_data = 0x1357_9bdf;
        let mut read = PioSwdTransaction::new(MockIo::new(&[
            ack_word(0b001),
            read_data,
            u32::from(parity32(read_data)) << 31,
        ]));
        assert_eq!(read.read_transfer(true, 3), Ok(read_data));
        let read_tx: Vec<_> = read
            .io
            .events
            .iter()
            .filter_map(|event| match event {
                Event::Put(word) => Some(*word),
                _ => None,
            })
            .collect();
        assert_eq!(
            read_tx,
            [
                u32::from(request_byte(true, true, 3)),
                u32::from(READ_CONTINUATION)
            ]
        );

        let write_data = 1;
        let mut write = PioSwdTransaction::new(MockIo::new(&[ack_word(0b001)]));
        assert_eq!(write.write_transfer(false, 2, write_data), Ok(()));
        let write_tx: Vec<_> = write
            .io
            .events
            .iter()
            .filter_map(|event| match event {
                Event::Put(word) => Some(*word),
                _ => None,
            })
            .collect();
        assert_eq!(
            write_tx,
            [
                u32::from(request_byte(false, false, 2)),
                u32::from(WRITE_CONTINUATION) | (1 << 5),
                write_data,
            ]
        );
    }
}
