//! Command framing: spec lengths (`command_len`) and SWD intercepts; `daplink_rs::guard` sizes engine reads.

/// A nonempty block whose request payload and response capacity have been checked.
#[derive(Clone, Copy)]
pub struct BlockRequest {
    /// Requested word count, from 1 through 65535.
    pub count: usize,
    /// Low four transfer-request bits; the protocol ignores the upper bits for blocks.
    pub request: u8,
}

impl BlockRequest {
    /// Parse a TransferBlock header, rejecting short blocks or responses; the id is not checked.
    pub fn parse(report: &[u8], response_len: usize) -> Option<Self> {
        if report.len() < 5 || response_len < 4 {
            return None;
        }
        let count = usize::from(u16::from_le_bytes([report[2], report[3]]));
        if count == 0 {
            return None;
        }
        let request = report[4] & 15;
        if request & 2 != 0 {
            if response_len < 4 + 4 * count {
                return None;
            }
        } else if report.len() < 5 + 4 * count {
            return None;
        }
        Some(Self { count, request })
    }
}

/// Attempts per transfer a complete TransferConfigure report allows: the first plus its WAIT retries.
pub fn transfer_wait_attempts(report: &[u8]) -> Option<usize> {
    if report.len() < 6 || report[0] != 4 {
        return None;
    }
    Some(usize::from(u16::from_le_bytes([report[2], report[3]])) + 1)
}

/// SWD idle cycles a complete TransferConfigure report asks for after each OK transfer.
pub fn transfer_idle_cycles(report: &[u8]) -> Option<u8> {
    if report.len() < 6 || report[0] != 4 {
        return None;
    }
    Some(report[1])
}

/// A sequence of up to 255 low SWDIO bits, for the idle cycles.
pub const IDLE_LOW: [u8; 32] = [0; 32];

/// Write a block header and return the response length including the `words` already stored.
///
/// `ran` counts a failed last transfer, which the count leaves out, as the Arm firmware's does.
///
/// # Panics
///
/// If `response` is shorter than four bytes.
pub fn encode_block_response(response: &mut [u8], ran: u16, status: u8, words: usize) -> usize {
    let count = ran.saturating_sub(u16::from(status != 1));
    response[0] = 6;
    response[1..3].copy_from_slice(&count.to_le_bytes());
    response[3] = status;
    4 + 4 * words
}

/// Length of the command starting `report`, or `None` until enough bytes are present.
pub fn command_len(report: &[u8]) -> Option<usize> {
    let bits_bytes = |bits: usize| bits.div_ceil(8);
    match *report.first()? {
        // DAP_Transfer: a request byte per item, plus a word for a write, match value or mask.
        0x05 => {
            let count = usize::from(*report.get(2)?);
            let mut at = 3;
            for _ in 0..count {
                let req = *report.get(at)?;
                at += 1;
                let read = req & (1 << 1) != 0;
                let vmatch = req & (1 << 4) != 0;
                if !read || vmatch {
                    at += 4;
                }
            }
            Some(at)
        }
        // DAP_TransferBlock: five header bytes, then a word per transfer for a write.
        0x06 => {
            let count = usize::from(u16::from_le_bytes([*report.get(2)?, *report.get(3)?]));
            Some(
                5 + if *report.get(4)? & (1 << 1) == 0 {
                    4 * count
                } else {
                    0
                },
            )
        }
        // DAP_SWJ_Sequence: a bit count (0 = 256), then the bits.
        0x12 => {
            let bits = match *report.get(1)? {
                0 => 256,
                n => usize::from(n),
            };
            Some(2 + bits_bytes(bits))
        }
        // DAP_JTAG_Sequence: per sequence an info byte (bits 0-5, 0 = 64) and its TDI bytes.
        0x14 => {
            let count = usize::from(*report.get(1)?);
            let mut at = 2;
            for _ in 0..count {
                let info = *report.get(at)?;
                let bits = match info & 0x3f {
                    0 => 64,
                    n => usize::from(n),
                };
                at += 1 + bits_bytes(bits);
            }
            Some(at)
        }
        // DAP_SWD_Sequence: per sequence an info byte; output sequences carry their bits.
        0x1d => {
            let count = usize::from(*report.get(1)?);
            let mut at = 2;
            for _ in 0..count {
                let info = *report.get(at)?;
                let bits = match info & 0x3f {
                    0 => 64,
                    n => usize::from(n),
                };
                at += 1 + if info & 0x80 == 0 {
                    bits_bytes(bits)
                } else {
                    0
                };
            }
            Some(at)
        }
        // JTAG_Configure and the vendor flash stream: a count, then that many bytes.
        0x15 | 0x8c => Some(2 + usize::from(*report.get(1)?)),
        // DAP_UART_Transfer: RxCount and TxCount shorts, then TxCount bytes.
        0x21 => Some(5 + usize::from(u16::from_le_bytes([*report.get(3)?, *report.get(4)?]))),
        // DAP_QueueCommands / DAP_ExecuteCommands: a count, then that many commands.
        0x7e | 0x7f => {
            let count = usize::from(*report.get(1)?);
            let mut at = 2;
            for _ in 0..count {
                let inner = report.get(at..)?;
                at += match *inner.first()? {
                    0x05 | 0x06 | 0x12 | 0x14 | 0x15 | 0x1d | 0x21 => command_len(inner)?,
                    // Unsizable: the batch is complete at the bytes received.
                    id => match fixed_len(id) {
                        Some(len) => len,
                        None => return Some(report.len()),
                    },
                };
            }
            Some(at)
        }
        _ => Some(1),
    }
}

/// Length of a fixed-shape command inside a queued batch; `None` for an unknown id.
fn fixed_len(id: u8) -> Option<usize> {
    Some(match id {
        0x03 | 0x07 | 0x0a | 0x1b | 0x23 => 1,
        0x00 | 0x02 | 0x13 | 0x16 | 0x17 | 0x18 | 0x1a | 0x1e | 0x1f | 0x22 => 2,
        0x01 | 0x09 | 0x1c => 3,
        0x11 | 0x19 => 5,
        0x04 | 0x08 | 0x20 => 6,
        0x10 => 7,
        _ => return None,
    })
}

#[cfg(test)]
mod command_len_tests {
    use super::command_len;

    #[test]
    fn a_block_write_needs_its_words_and_a_read_only_its_header() {
        let mut w = [0u8; 5 + 4 * 126];
        w[0] = 0x06;
        w[2] = 126;
        assert_eq!(command_len(&w[..64]), Some(509));
        assert_eq!(command_len(&[0x06, 0, 127, 0, 0x0f]), Some(5));
        assert_eq!(command_len(&[0x06, 0, 1]), None);
    }

    #[test]
    fn a_transfer_counts_write_and_match_words() {
        // Two items: a write (4 bytes) and a read with value match (4 bytes), then a plain read.
        let t = [0x05, 0, 3, 0x00, 1, 2, 3, 4, 0x12, 5, 6, 7, 8, 0x02];
        assert_eq!(command_len(&t), Some(14));
        assert_eq!(command_len(&t[..9]), None);
    }

    #[test]
    fn sequences_count_their_bit_bytes() {
        assert_eq!(command_len(&[0x12, 0]), Some(34));
        assert_eq!(command_len(&[0x12, 9]), Some(4));
        // JTAG: one 64-bit sequence (8 TDI bytes), one 3-bit (1 byte).
        let mut j = [0u8; 2 + 9 + 2];
        j[0] = 0x14;
        j[1] = 2;
        j[2] = 0;
        j[11] = 3;
        assert_eq!(command_len(&j), Some(13));
        // SWD: an output of 8 bits, an input of 32.
        assert_eq!(command_len(&[0x1d, 2, 8, 0xff, 0x80 | 32]), Some(5));
    }

    #[test]
    fn a_vendor_flash_write_needs_its_payload() {
        let mut w = [0u8; 2 + 255];
        w[0] = 0x8c;
        w[1] = 255;
        assert_eq!(command_len(&w[..64]), Some(257));
        assert_eq!(command_len(&[0x8c]), None);
    }

    #[test]
    fn queued_commands_add_up_and_other_ids_are_complete() {
        assert_eq!(
            command_len(&[0x7f, 2, 0x00, 0xfe, 0x06, 0, 1, 0, 0x02]),
            Some(9)
        );
        assert_eq!(command_len(&[0x00, 0xfe]), Some(1));
        assert_eq!(command_len(&[]), None);
    }

    #[test]
    fn swo_extended_status_takes_its_control_byte() {
        assert_eq!(command_len(&[0x7f, 2, 0x1e, 0x07, 0x00, 0xfe]), Some(6));
    }

    #[test]
    fn an_unknown_queued_id_ends_the_batch_at_the_bytes_received() {
        assert_eq!(command_len(&[0x7f, 1, 0x42]), Some(3));
        assert_eq!(command_len(&[0x7e, 3, 0x00, 0xfe, 0x42, 9, 9]), Some(7));
    }
}
