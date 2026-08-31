//! ITM packet decode: one stimulus port's payload bytes out of a captured SWO byte stream.

/// What one pass over a captured byte stream found.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Decoded {
    /// Payload bytes of the requested software stimulus port, in capture order.
    pub payload: Vec<u8>,
    /// ITM overflow packets seen: the target's own trace FIFO dropped data.
    pub overflow: usize,
    /// Synchronization/idle bytes skipped.
    pub sync: usize,
    /// Packets belonging to another port or to a hardware source.
    pub other_source: usize,
    /// Protocol packets skipped (timestamps, extensions).
    pub protocol: usize,
    /// Header bytes that decode to no known packet: direct evidence of byte loss on the line.
    pub errors: usize,
    /// Bytes of a final packet the capture window cut short.
    pub truncated: usize,
}

/// Decode `bytes`, collecting the payload written to software stimulus `port`.
pub fn decode_port(bytes: &[u8], port: u8) -> Decoded {
    let mut out = Decoded::default();
    let mut i = 0;
    while i < bytes.len() {
        let header = bytes[i];
        let size = header & 0x03;
        if size == 0 {
            i += protocol_packet(bytes, i, &mut out);
            continue;
        }
        // Source packet: 1, 2 or 4 payload bytes follow the header.
        let len = 1usize << (size - 1);
        let Some(payload) = bytes.get(i + 1..i + 1 + len) else {
            out.truncated = bytes.len() - i;
            break;
        };
        let is_hardware = header & 0x04 != 0;
        if !is_hardware && header >> 3 == port {
            out.payload.extend_from_slice(payload);
        } else {
            out.other_source += 1;
        }
        i += 1 + len;
    }
    out
}

/// Consume the protocol packet at `i`, returning how many bytes it spanned (never zero).
fn protocol_packet(bytes: &[u8], i: usize, out: &mut Decoded) -> usize {
    let header = bytes[i];
    match header {
        // Sync stream: a run of zero bytes closed by 0x80. Both are counted and skipped.
        0x00 | 0x80 => {
            out.sync += 1;
            1
        }
        0x70 => {
            out.overflow += 1;
            1
        }
        // Timestamp and extension packets continue while the preceding byte's bit 7 is set.
        _ if header & 0x0F == 0x00 || header & 0x0B == 0x08 || header == 0x94 || header == 0xB4 => {
            out.protocol += 1;
            1 + continuation_len(bytes, i, header)
        }
        _ => {
            out.errors += 1;
            1
        }
    }
}

/// Length of the continuation payload following the protocol header at `i`, capped at 4 bytes.
fn continuation_len(bytes: &[u8], i: usize, header: u8) -> usize {
    if header & 0x80 == 0 {
        return 0;
    }
    let mut n = 0;
    while n < 4 {
        match bytes.get(i + 1 + n) {
            Some(&b) => {
                n += 1;
                if b & 0x80 == 0 {
                    break;
                }
            }
            None => break,
        }
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Frame `payload` as one-byte writes to software stimulus `port`.
    fn framed_u8(port: u8, payload: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        for &b in payload {
            v.push((port << 3) | 0x01);
            v.push(b);
        }
        v
    }

    /// Frame `words` as 32-bit writes to software stimulus `port`.
    fn framed_u32(port: u8, words: &[u32]) -> Vec<u8> {
        let mut v = Vec::new();
        for w in words {
            v.push((port << 3) | 0x03);
            v.extend_from_slice(&w.to_le_bytes());
        }
        v
    }

    #[test]
    fn byte_writes_on_the_requested_port_decode_to_their_payload() {
        let d = decode_port(&framed_u8(0, &[1, 2, 3]), 0);
        assert_eq!(d.payload, vec![1, 2, 3]);
        assert_eq!(d.errors, 0);
    }

    #[test]
    fn word_writes_decode_lsb_first() {
        let d = decode_port(&framed_u32(0, &[0x3CC3_A55A, 7]), 0);
        assert_eq!(d.payload, vec![0x5A, 0xA5, 0xC3, 0x3C, 7, 0, 0, 0]);
        assert_eq!(d.other_source, 0);
    }

    #[test]
    fn other_ports_and_hardware_sources_are_counted_not_collected() {
        let mut stream = framed_u32(0, &[1]);
        stream.extend_from_slice(&framed_u8(1, &[0xEE])); // another stimulus port
        stream.extend_from_slice(&[0x0E, 0xAA, 0xBB]); // DWT hardware source, 2 bytes
        stream.extend_from_slice(&framed_u32(0, &[2]));
        let d = decode_port(&stream, 0);
        assert_eq!(d.payload, vec![1, 0, 0, 0, 2, 0, 0, 0]);
        assert_eq!(d.other_source, 2);
        assert_eq!(d.errors, 0);
    }

    #[test]
    fn overflow_and_sync_are_counted_and_do_not_break_framing() {
        let mut stream = vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x80]; // sync sequence
        stream.push(0x70); // ITM overflow
        stream.extend_from_slice(&framed_u32(0, &[9]));
        let d = decode_port(&stream, 0);
        assert_eq!(d.payload, vec![9, 0, 0, 0]);
        assert_eq!(d.overflow, 1);
        assert_eq!(d.sync, 6);
    }

    #[test]
    fn timestamp_packets_are_skipped_with_their_continuation() {
        let mut stream = vec![0xC0, 0x81, 0x02]; // local timestamp, two continuation bytes
        stream.extend_from_slice(&framed_u8(0, &[0x42]));
        let d = decode_port(&stream, 0);
        assert_eq!(d.payload, vec![0x42]);
        assert_eq!(d.protocol, 1);
        assert_eq!(d.errors, 0);
    }

    #[test]
    fn a_cut_packet_is_reported_as_truncated_not_as_payload() {
        let mut stream = framed_u32(0, &[1]);
        stream.extend_from_slice(&[0x03, 0xAA, 0xBB]); // 4-byte write, only 2 bytes captured
        let d = decode_port(&stream, 0);
        assert_eq!(d.payload, vec![1, 0, 0, 0]);
        assert_eq!(d.truncated, 3);
    }

    #[test]
    fn an_unknown_header_costs_one_byte_and_the_stream_resyncs() {
        let mut stream = vec![0x74]; // reserved protocol header
        stream.extend_from_slice(&framed_u8(0, &[0x11, 0x22]));
        let d = decode_port(&stream, 0);
        assert_eq!(d.errors, 1);
        assert_eq!(d.payload, vec![0x11, 0x22]);
    }

    #[test]
    fn an_empty_stream_decodes_to_nothing() {
        assert_eq!(decode_port(&[], 0), Decoded::default());
    }
}
