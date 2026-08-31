//! v1 measurement stream: `interval` payload bytes of the shared generator, then one frame.

mod crc;
mod encode;
mod verb;
mod verify;

pub use crc::{crc16, crc32, Crc32};
pub use encode::{Encoder, TxCounters};
pub use verb::{
    parse, report_reported, report_response, DirMask, V1Command, V1Report, V1Start, OK_V1_RESET,
    OK_V1_START, OK_V1_STOPPED, V1_LINE,
};
pub use verify::{RxCounters, Verifier};

/// Bytes in one frame.
pub const FRAME_LEN: usize = 24;

/// Magic word heading every frame.
pub const MAGIC: [u8; 4] = *b"V1MK";

/// Payload bytes between frames when the host does not choose otherwise.
pub const DEFAULT_INTERVAL: u32 = 2048;

/// Budget of a run that ends only on abort; no end-of-stream frame is emitted.
pub const UNBOUNDED: u64 = u64::MAX;

/// Turn a verb's budget field into an engine budget; 0 on the wire means [`UNBOUNDED`].
pub const fn budget(spec: u32) -> u64 {
    if spec == 0 {
        UNBOUNDED
    } else {
        spec as u64
    }
}

/// Which end transmits the stream a frame belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    HostToDut,
    DutToHost,
}

impl Direction {
    /// The wire encoding of the frame's direction field.
    pub const fn as_byte(self) -> u8 {
        match self {
            Self::HostToDut => 0,
            Self::DutToHost => 1,
        }
    }

    /// Read a direction field; an undefined value is not a frame.
    pub const fn from_byte(b: u8) -> Option<Self> {
        match b {
            0 => Some(Self::HostToDut),
            1 => Some(Self::DutToHost),
            _ => None,
        }
    }
}

/// What a frame says about the stream around it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameKind {
    /// Ends a full payload span.
    Marker,
    /// Carries a cumulative credit grant; not part of the payload.
    Credit,
    /// Ends the run, carrying its final payload total.
    EndOfStream,
}

impl FrameKind {
    /// The wire encoding of the frame's kind field.
    pub const fn as_byte(self) -> u8 {
        match self {
            Self::Marker => 0,
            Self::Credit => 1,
            Self::EndOfStream => 2,
        }
    }

    /// Read a kind field; an undefined value is not a frame.
    pub const fn from_byte(b: u8) -> Option<Self> {
        match b {
            0 => Some(Self::Marker),
            1 => Some(Self::Credit),
            2 => Some(Self::EndOfStream),
            _ => None,
        }
    }
}

/// Transmitter shaping: `bytes` of wire traffic, then `idle_ms` of silence. `bytes` of 0 shapes
/// nothing.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BurstSpec {
    pub bytes: u32,
    pub idle_ms: u32,
}

/// One decoded frame; on the wire, little-endian: magic `V1MK` (4 bytes), kind (1), direction
/// (1), sequence u32, payload offset u64, payload-span CRC32, header CRC16 over the first 22.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    pub kind: FrameKind,
    pub dir: Direction,
    /// Frame sequence for markers and end-of-stream, grant sequence for credit frames.
    pub seq: u32,
    /// Payload offset for markers and end-of-stream, cumulative grant for credit frames.
    pub offset: u64,
    /// CRC32 of the payload span this frame ends; 0 in a credit frame.
    pub crc: u32,
}

impl Frame {
    /// Serialize the frame, header CRC16 included.
    pub fn encode(&self) -> [u8; FRAME_LEN] {
        let mut out = [0u8; FRAME_LEN];
        out[..4].copy_from_slice(&MAGIC);
        out[4] = self.kind.as_byte();
        out[5] = self.dir.as_byte();
        out[6..10].copy_from_slice(&self.seq.to_le_bytes());
        out[10..18].copy_from_slice(&self.offset.to_le_bytes());
        out[18..22].copy_from_slice(&self.crc.to_le_bytes());
        let header = crc16(&out[..22]);
        out[22..].copy_from_slice(&header.to_le_bytes());
        out
    }

    /// Read a frame, rejecting anything whose magic, header CRC16, kind, or direction disagrees.
    pub fn decode(bytes: &[u8; FRAME_LEN]) -> Option<Self> {
        if bytes[..4] != MAGIC {
            return None;
        }
        if u16::from_le_bytes([bytes[22], bytes[23]]) != crc16(&bytes[..22]) {
            return None;
        }
        Some(Self {
            kind: FrameKind::from_byte(bytes[4])?,
            dir: Direction::from_byte(bytes[5])?,
            seq: u32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]),
            offset: u64::from_le_bytes([
                bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15], bytes[16],
                bytes[17],
            ]),
            crc: u32::from_le_bytes([bytes[18], bytes[19], bytes[20], bytes[21]]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_round_trips_through_the_wire_form() {
        let frame = Frame {
            kind: FrameKind::Marker,
            dir: Direction::DutToHost,
            seq: 3,
            offset: 8192,
            crc: 0xDEAD_BEEF,
        };
        let bytes = frame.encode();
        assert_eq!(&bytes[..4], b"V1MK");
        assert_eq!(bytes[4], 0);
        assert_eq!(bytes[5], 1);
        assert_eq!(Frame::decode(&bytes), Some(frame));
    }

    #[test]
    fn decode_rejects_a_frame_whose_header_was_touched() {
        let frame = Frame {
            kind: FrameKind::EndOfStream,
            dir: Direction::HostToDut,
            seq: 9,
            offset: 12345,
            crc: 7,
        };
        let good = frame.encode();
        assert_eq!(Frame::decode(&good), Some(frame));
        // Every header byte is under the CRC16, so a single flip anywhere invalidates the frame.
        for i in 0..22 {
            let mut bad = good;
            bad[i] ^= 0x01;
            assert_eq!(Frame::decode(&bad), None, "byte {i} was not protected");
        }
        let mut bad = good;
        bad[23] ^= 0x80;
        assert_eq!(Frame::decode(&bad), None);
    }

    #[test]
    fn decode_rejects_undefined_kinds_and_directions() {
        let mut bytes = Frame {
            kind: FrameKind::Marker,
            dir: Direction::HostToDut,
            seq: 0,
            offset: 2048,
            crc: 0,
        }
        .encode();
        bytes[4] = 3;
        let header = crc16(&bytes[..22]);
        bytes[22..].copy_from_slice(&header.to_le_bytes());
        assert_eq!(Frame::decode(&bytes), None);
        bytes[4] = 0;
        bytes[5] = 2;
        let header = crc16(&bytes[..22]);
        bytes[22..].copy_from_slice(&header.to_le_bytes());
        assert_eq!(Frame::decode(&bytes), None);
    }

    #[test]
    fn budget_reads_zero_as_unbounded() {
        assert_eq!(budget(0), UNBOUNDED);
        assert_eq!(budget(4096), 4096);
    }
}
