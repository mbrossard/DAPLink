use crate::MAX_LINE;

/// LF-delimited command receiver; an overlong frame is discarded through its LF.
pub struct CommandLine {
    bytes: [u8; MAX_LINE],
    len: usize,
    discarding: bool,
}

impl Default for CommandLine {
    fn default() -> Self {
        Self::new()
    }
}

impl CommandLine {
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_LINE],
            len: 0,
            discarding: false,
        }
    }

    /// Return a complete payload on LF; CR is buffered and counts toward capacity.
    pub fn push(&mut self, byte: u8) -> Option<&[u8]> {
        if byte == b'\n' {
            let len = self.len;
            self.len = 0;
            let discarded = self.discarding;
            self.discarding = false;
            (!discarded).then_some(&self.bytes[..len])
        } else if self.discarding {
            None
        } else if self.len < self.bytes.len() {
            self.bytes[self.len] = byte;
            self.len += 1;
            None
        } else {
            self.len = 0;
            self.discarding = true;
            None
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{parse, v1, Command};
    use std::vec::Vec;

    #[derive(Debug, PartialEq)]
    enum Action {
        Command(Command),
        Stream(v1::V1Command),
        Invalid,
    }

    fn receive(chunks: impl IntoIterator<Item = impl AsRef<[u8]>>) -> (Vec<Vec<u8>>, Vec<Action>) {
        let mut receiver = CommandLine::new();
        let mut frames = Vec::new();
        let mut actions = Vec::new();
        for chunk in chunks {
            for &byte in chunk.as_ref() {
                if let Some(line) = receiver.push(byte) {
                    frames.push(line.to_vec());
                    let line = core::str::from_utf8(line).unwrap_or("");
                    actions.push(if let Some(command) = v1::parse(line) {
                        Action::Stream(command)
                    } else if let Some(command) = parse(line) {
                        Action::Command(command)
                    } else {
                        Action::Invalid
                    });
                }
            }
        }
        (frames, actions)
    }

    #[test]
    fn overflow_suffix_never_dispatches_and_next_frame_recovers() {
        for suffix in [
            b"STATS RESET".as_slice(),
            b"V1 RESET",
            b"BAUD 115200",
            b"SWO STREAM 0",
        ] {
            for ending in [b"\n".as_slice(), b"\r\n"] {
                let mut input = std::vec![b'X'; MAX_LINE + 1];
                input.extend_from_slice(suffix);
                input.extend_from_slice(ending);
                input.extend_from_slice(b"PING\n");
                let expected = (
                    std::vec![b"PING".to_vec()],
                    std::vec![Action::Command(Command::Ping)],
                );
                for chunk_size in [1, 16, 32, input.len()] {
                    assert_eq!(receive(input.chunks(chunk_size)), expected);
                }
                for split in 0..=input.len() {
                    assert_eq!(receive([&input[..split], &input[split..]]), expected);
                }
            }
        }
    }

    #[test]
    fn capacity_counts_pre_lf_bytes_including_cr() {
        for (payload_len, ending, accepted) in [
            (128, b"\n".as_slice(), true),
            (129, b"\n", false),
            (127, b"\r\n", true),
            (128, b"\r\n", false),
        ] {
            let mut input = std::vec![b' '; payload_len];
            input[..4].copy_from_slice(b"PING");
            input.extend_from_slice(ending);
            input.extend_from_slice(b"ID\n");
            let actions = if accepted {
                std::vec![Action::Command(Command::Ping), Action::Command(Command::Id)]
            } else {
                std::vec![Action::Command(Command::Id)]
            };
            for chunk_size in [1, 16, 32, input.len()] {
                assert_eq!(receive(input.chunks(chunk_size)).1, actions);
            }
            for split in 0..=input.len() {
                assert_eq!(receive([&input[..split], &input[split..]]).1, actions);
            }
        }
    }

    #[test]
    fn discard_persists_across_reads_and_multiple_overlong_frames() {
        let mut input = std::vec![b'X'; 3 * MAX_LINE];
        input.extend_from_slice(b"PING\r");
        assert_eq!(receive(input.chunks(16)), (Vec::new(), Vec::new()));
        input.extend_from_slice(b"\n");
        input.extend_from_slice(&[b'X'; MAX_LINE + 1]);
        input.extend_from_slice(b"STATS RESET\nPING\n\n");
        let expected = (
            std::vec![b"PING".to_vec(), Vec::new()],
            std::vec![Action::Command(Command::Ping), Action::Invalid],
        );
        for chunk_size in [1, 16, 32, input.len()] {
            assert_eq!(receive(input.chunks(chunk_size)), expected);
        }
    }

    #[test]
    fn ordinary_lines_empty_frames_utf8_and_cr_only_are_unchanged() {
        assert_eq!(receive([b"PING\r".as_slice()]), (Vec::new(), Vec::new()));
        let input = b"PING\r\nID\nSTAMP 7\nBAUD 115200\nSTATS\nV1 RESET\n\n\xff\nPING\rID\n";
        let expected = std::vec![
            Action::Command(Command::Ping),
            Action::Command(Command::Id),
            Action::Command(Command::Stamp(7)),
            Action::Command(Command::Baud(115200)),
            Action::Command(Command::Stats),
            Action::Stream(v1::V1Command::Reset),
            Action::Invalid,
            Action::Invalid,
            Action::Invalid
        ];
        for chunk_size in [1, 16, 32, input.len()] {
            assert_eq!(receive(input.chunks(chunk_size)).1, expected);
        }
    }
}
