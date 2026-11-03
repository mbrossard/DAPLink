//! RP2040/RP2350 USB buffer-control words (single-buffered) for an endpoint halt and its clear.

const LENGTH: u32 = 0x3ff;
const AVAILABLE: u32 = 1 << 10;
const STALL: u32 = 1 << 11;
const PID: u32 = 1 << 13;
#[cfg(test)]
const FULL: u32 = 1 << 15;

/// The halt, written in order: STALL answers every token, AVAILABLE keeps a parked transfer parked.
pub fn halt_words(dir_in: bool, out_mps: u16) -> (u32, u32) {
    let len = if dir_in {
        0
    } else {
        u32::from(out_mps) & LENGTH
    };
    (STALL | len, STALL | AVAILABLE | len)
}

/// The IN clear: software owns the word and the next packet is DATA0, as PID flips before use.
pub fn clear_in_word() -> u32 {
    PID
}

/// The OUT clear, written in order: an empty buffer expecting DATA0, then its hand-over.
pub fn clear_out_words(out_mps: u16) -> (u32, u32) {
    let arm = u32::from(out_mps) & LENGTH;
    (arm, arm | AVAILABLE)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, PartialEq, Clone, Copy)]
    enum Reply {
        Stall,
        Nak,
        /// OUT data at the wrong PID: acknowledged, not stored.
        Discarded,
        /// DATA sent (IN) or accepted (OUT) at this PID.
        Data(bool),
    }

    /// One single-buffered buffer-control word with the controller's token-phase rules.
    struct Model {
        word: u32,
        /// Software wrote a word the controller owned (AVAILABLE set, STALL clear).
        owned_write: bool,
        /// AVAILABLE was set in the same write as another change.
        unstaged: bool,
    }

    impl Model {
        fn new(word: u32) -> Self {
            Self {
                word,
                owned_write: false,
                unstaged: false,
            }
        }

        fn write(&mut self, w: u32) {
            if self.word & AVAILABLE != 0 && self.word & STALL == 0 {
                self.owned_write = true;
            }
            if w & AVAILABLE != 0 && self.word & !AVAILABLE != w & !AVAILABLE {
                self.unstaged = true;
            }
            self.word = w;
        }

        /// embassy-rp 0.10.0 `endpoint_set_stalled`: `modify(|w| w.set_stall(stalled))`.
        fn modify_stall(&mut self, stalled: bool) {
            let w = if stalled {
                self.word | STALL
            } else {
                self.word & !STALL
            };
            self.write(w);
        }

        /// Status phase: length and PID are set, everything else is written to zero.
        fn complete(&mut self, pid: bool, len: u32) {
            self.word = (len & LENGTH) | if pid { PID } else { 0 };
        }

        fn in_token(&mut self) -> Reply {
            if self.word & STALL != 0 {
                return Reply::Stall;
            }
            if self.word & AVAILABLE != 0 && self.word & FULL != 0 {
                let pid = self.word & PID != 0;
                self.complete(pid, self.word & LENGTH);
                return Reply::Data(pid);
            }
            Reply::Nak
        }

        fn out_token(&mut self, pid: bool, len: u32) -> Reply {
            if self.word & AVAILABLE != 0 && self.word & FULL == 0 {
                if self.word & STALL != 0 {
                    return Reply::Stall;
                }
                if (self.word & PID != 0) != pid {
                    return Reply::Discarded;
                }
                self.word = (len & LENGTH) | FULL | if pid { PID } else { 0 };
                return Reply::Data(pid);
            }
            Reply::Nak
        }

        /// embassy-rp 0.10.0 `EndpointIn::write`: parked while AVAILABLE, else arms at `!pid`.
        fn driver_write(&mut self, len: u32) -> bool {
            if self.word & AVAILABLE != 0 {
                return false;
            }
            let pid = self.word & PID == 0;
            let w = (len & LENGTH) | FULL | if pid { PID } else { 0 };
            self.write(w);
            self.write(w | AVAILABLE);
            true
        }

        /// embassy-rp 0.10.0 `EndpointOut::read`: parked while AVAILABLE, else re-arms at `!pid`.
        fn driver_read(&mut self, mps: u32) -> Option<u32> {
            if self.word & AVAILABLE != 0 {
                return None;
            }
            let len = self.word & LENGTH;
            let pid = self.word & PID == 0;
            let w = (mps & LENGTH) | if pid { PID } else { 0 };
            self.write(w);
            self.write(w | AVAILABLE);
            Some(len)
        }
    }

    /// The wrapper's halt; the inner `modify` runs between the writes, for its wake.
    fn halt(m: &mut Model, dir_in: bool) {
        let (stall, park) = halt_words(dir_in, 64);
        m.write(stall);
        m.modify_stall(true);
        m.write(park);
    }

    fn clear_in(m: &mut Model) {
        m.write(clear_in_word());
        m.modify_stall(false);
    }

    fn clear_out(m: &mut Model) {
        let (arm, go) = clear_out_words(64);
        m.write(arm);
        m.modify_stall(false);
        m.write(go);
    }

    /// Idle IN word after `packets` packets; enable leaves PID set so the first is DATA0.
    fn idle_in(packets: u32) -> Model {
        Model::new(if packets.is_multiple_of(2) { PID } else { 0 })
    }

    /// Idle OUT word expecting the PID of packet `packets`.
    fn idle_out(packets: u32) -> Model {
        Model::new(64 | AVAILABLE | if packets.is_multiple_of(2) { 0 } else { PID })
    }

    #[test]
    fn a_halted_in_endpoint_stalls_and_keeps_the_writer_parked() {
        for armed in [false, true] {
            let mut m = idle_in(0);
            if armed {
                assert!(m.driver_write(3));
            }
            halt(&mut m, true);
            assert_eq!(m.in_token(), Reply::Stall);
            assert!(!m.driver_write(3), "a write went through the halt");
            assert_eq!(m.in_token(), Reply::Stall);
            assert!(!m.unstaged);
        }
    }

    #[test]
    fn the_in_clear_sends_data0_next_from_either_parity() {
        for halted in [false, true] {
            for packets in 0..4 {
                let mut m = idle_in(packets);
                if halted {
                    halt(&mut m, true);
                }
                clear_in(&mut m);
                assert!(m.driver_write(3));
                assert_eq!(m.in_token(), Reply::Data(false), "after {packets} packets");
                assert!(!m.owned_write && !m.unstaged);
            }
        }
    }

    #[test]
    fn a_reply_armed_before_the_halt_is_dropped_at_either_parity() {
        for packets in 0..2 {
            let mut m = idle_in(packets);
            assert!(m.driver_write(3));
            halt(&mut m, true);
            assert_eq!(m.in_token(), Reply::Stall);
            clear_in(&mut m);
            assert_eq!(
                m.in_token(),
                Reply::Nak,
                "the armed reply survived the halt"
            );
        }
    }

    #[test]
    fn a_halted_out_endpoint_stalls_and_keeps_the_reader_parked() {
        for packets in 0..2 {
            let mut m = idle_out(packets);
            halt(&mut m, false);
            assert_eq!(m.out_token(false, 8), Reply::Stall);
            assert_eq!(m.driver_read(64), None);
            assert_eq!(m.out_token(true, 8), Reply::Stall);
            assert!(!m.unstaged);
        }
    }

    #[test]
    fn the_out_clear_accepts_data0_next_from_either_parity() {
        for halted in [false, true] {
            for packets in 0..2 {
                let mut m = idle_out(packets);
                if halted {
                    halt(&mut m, false);
                }
                clear_out(&mut m);
                assert_eq!(
                    m.out_token(true, 8),
                    Reply::Discarded,
                    "after {packets} packets"
                );
                assert_eq!(m.out_token(false, 8), Reply::Data(false));
                assert_eq!(m.driver_read(64), Some(8));
                assert_eq!(m.out_token(true, 8), Reply::Data(true));
                assert!(!m.unstaged);
            }
        }
    }

    #[test]
    fn each_pair_hands_the_word_over_only_in_its_second_write() {
        let mut m = idle_out(0);
        let (arm, go) = clear_out_words(64);
        m.write(arm);
        assert_eq!(m.word & AVAILABLE, 0);
        m.write(go);
        let mut m = idle_out(1);
        let (stall, park) = halt_words(false, 64);
        m.write(stall);
        assert_eq!(m.word & AVAILABLE, 0);
        m.write(park);
    }

    /// embassy-rp 0.10.0 alone: the clear keeps the toggle and lets an armed reply out.
    #[test]
    fn the_driver_alone_keeps_the_toggle_across_a_clear() {
        let mut m = idle_in(1);
        m.modify_stall(false);
        assert!(m.driver_write(3));
        assert_eq!(m.in_token(), Reply::Data(true), "DATA1 after a clear");
        let mut m = idle_out(1);
        m.modify_stall(false);
        assert_eq!(
            m.out_token(false, 8),
            Reply::Discarded,
            "DATA0 after a clear"
        );
        let mut m = idle_in(1);
        assert!(m.driver_write(3));
        m.modify_stall(true);
        assert_eq!(m.in_token(), Reply::Stall);
        m.modify_stall(false);
        assert_eq!(
            m.in_token(),
            Reply::Data(true),
            "the armed reply went out after a clear"
        );
    }

    /// A halt or an unhalted clear over an armed buffer, as an idle OUT always is, revokes it.
    #[test]
    fn writes_over_an_armed_buffer_revoke_it() {
        let mut m = idle_out(0);
        halt(&mut m, false);
        assert!(m.owned_write);
        let mut m = idle_in(0);
        assert!(m.driver_write(3));
        halt(&mut m, true);
        assert!(m.owned_write);
        let mut m = idle_in(0);
        assert!(m.driver_write(3));
        clear_in(&mut m);
        assert!(m.owned_write);
        assert_eq!(m.in_token(), Reply::Nak, "the armed reply is dropped");
        let mut m = idle_out(0);
        clear_out(&mut m);
        assert!(m.owned_write);
    }
}
