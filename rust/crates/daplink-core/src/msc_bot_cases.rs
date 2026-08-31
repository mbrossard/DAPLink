//! The Bulk-Only Transport "thirteen cases" as const expectation rows, for a transport's host tests.

use crate::msc_proto::{
    Sense, CBW_LEN, CBW_SIGNATURE, CSW_FAILED, CSW_PASSED, CSW_PHASE_ERROR, SENSE_ILLEGAL,
    SENSE_LBA_RANGE, SENSE_LUN_NOT_SUPPORTED, SENSE_MEDIUM_CHANGED, SENSE_NONE,
};
use crate::{MSC_SECTOR_SIZE, VFS_TOTAL_SECTORS};

/// What a transport that can stall must have stalled before sending the CSW.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StallIntent {
    /// Neither bulk endpoint is stalled.
    None,
    /// The device-to-host endpoint is stalled: a declared data-IN phase that sends no bytes.
    In,
    /// The host-to-device endpoint is stalled: a declared data-OUT phase that reads none. Only a
    /// stalling transport enforces this variant; an OUT row yields a non-stalling one no packet.
    Out,
}

/// One Bulk-Only Transport row: a CBW as a host would send it, and the required answer.
pub struct BotCase {
    /// Case label, quoted in assertion messages.
    pub name: &'static str,
    /// The 31 CBW bytes.
    pub cbw: [u8; CBW_LEN],
    /// Zero-filled 64-byte host-to-device packets the harness supplies, and expects consumed.
    pub out_packets: usize,
    /// Whether a UNIT ATTENTION is pending when the CBW arrives.
    pub ua_before: bool,
    /// Whether it must still be pending once the command has been answered.
    pub ua_after: bool,
    /// Device-to-host data bytes that must precede the CSW.
    pub data_len: usize,
    /// Required `bCSWStatus`.
    pub status: u8,
    /// Required `dCSWDataResidue`.
    pub residue: u32,
    /// The sense the transport must have latched afterwards.
    pub sense: Sense,
    /// What a stalling transport must have stalled.
    pub stall: StallIntent,
}

impl BotCase {
    /// The tag the CSW must echo, read back from the CBW bytes so the two cannot drift apart.
    pub const fn tag(&self) -> u32 {
        u32::from_le_bytes([self.cbw[4], self.cbw[5], self.cbw[6], self.cbw[7]])
    }

    /// The `(IN stalled, OUT stalled)` pair a stalling transport must show.
    pub const fn stalled_pair(&self) -> (bool, bool) {
        match self.stall {
            StallIntent::None => (false, false),
            StallIntent::In => (true, false),
            StallIntent::Out => (false, true),
        }
    }

    /// Whether a transport that cannot stall must end the data phase with a zero-length packet:
    /// exactly the rows where a stalling one stalls IN, a declared data-IN phase with no bytes.
    pub const fn ends_with_zlp(&self) -> bool {
        matches!(self.stall, StallIntent::In)
    }
}

/// Build the 31 CBW bytes. Taking the command block by value keeps the table free of references
/// to temporaries, which a `const` initializer cannot hold.
const fn cbw<const N: usize>(
    tag: u32,
    data_len: u32,
    dir_in: bool,
    lun: u8,
    cb: [u8; N],
) -> [u8; CBW_LEN] {
    assert!(N >= 1 && N <= 16, "bCBWCBLength must be 1..=16");
    let mut pkt = [0u8; CBW_LEN];
    let sig = CBW_SIGNATURE.to_le_bytes();
    let tag = tag.to_le_bytes();
    let len = data_len.to_le_bytes();
    let mut i = 0;
    while i < 4 {
        pkt[i] = sig[i];
        pkt[4 + i] = tag[i];
        pkt[8 + i] = len[i];
        i += 1;
    }
    pkt[12] = if dir_in { 0x80 } else { 0x00 };
    pkt[13] = lun;
    pkt[14] = N as u8;
    let mut j = 0;
    while j < N {
        pkt[15 + j] = cb[j];
        j += 1;
    }
    pkt
}

/// INQUIRY with a 16-bit allocation length at `cb[3..5]`.
const fn inquiry(alloc: u16) -> [u8; 6] {
    let a = alloc.to_be_bytes();
    [0x12, 0x00, 0x00, a[0], a[1], 0x00]
}

/// READ(10) / WRITE(10): LBA at `cb[2..6]`, sector count at `cb[7..9]`.
const fn rw10(op: u8, lba: u32, count: u16) -> [u8; 10] {
    let l = lba.to_be_bytes();
    let c = count.to_be_bytes();
    [op, 0x00, l[0], l[1], l[2], l[3], 0x00, c[0], c[1], 0x00]
}

/// REQUEST SENSE with an 8-bit allocation length at `cb[4]`.
const fn request_sense(alloc: u8) -> [u8; 6] {
    [0x03, 0x00, 0x00, 0x00, alloc, 0x00]
}

/// MODE SENSE(6) for all pages, with an 8-bit allocation length at `cb[4]`.
const fn mode_sense6(alloc: u8) -> [u8; 6] {
    [0x1A, 0x00, 0x3F, 0x00, alloc, 0x00]
}

/// READ FORMAT CAPACITIES with a 16-bit allocation length at `cb[7..9]`.
const fn read_format_capacities(alloc: u16) -> [u8; 10] {
    let a = alloc.to_be_bytes();
    [0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, a[0], a[1], 0x00]
}

/// TEST UNIT READY: no fields, no data phase of its own.
const TUR: [u8; 6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
/// An opcode the dispatch table does not answer, so it fails with ILLEGAL REQUEST.
const UNKNOWN_OP: [u8; 6] = [0xFD, 0x00, 0x00, 0x00, 0x00, 0x00];
/// READ(10) opcode.
const OP_READ: u8 = 0x28;
/// WRITE(10) opcode.
const OP_WRITE: u8 = 0x2A;
/// READ CAPACITY(10): every field reserved, and the reply ignores any allocation length.
const READ_CAPACITY: [u8; 10] = [0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
/// SYNCHRONIZE CACHE(10), which a host issues before it lets go of the volume.
const SYNC_CACHE: [u8; 10] = [0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
/// START STOP UNIT with LOEJ set: the eject a host sends when unmounting.
const START_STOP_EJECT: [u8; 6] = [0x1B, 0x00, 0x00, 0x00, 0x02, 0x00];
/// One sector's worth of declared transfer length.
const SECTOR: u32 = MSC_SECTOR_SIZE as u32;
/// Full-speed bulk packets in one sector, which is how many OUT packets a write row supplies.
const SECTOR_PACKETS: usize = MSC_SECTOR_SIZE / 64;

/// The case table, driven row by row from a transport's own harness. Rows 4, 8, 9, 10 and the
/// UNIT ATTENTION shapes are the ones whose failure desynchronizes or wedges the stream.
pub static BOT_CASES: &[BotCase] = &[
    BotCase {
        name: "1 Hn=Dn: TEST UNIT READY, no data phase",
        cbw: cbw(0x01, 0, false, 0, TUR),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "2 Hn<Di: INQUIRY under a CBW declaring no data phase",
        cbw: cbw(0x02, 0, true, 0, inquiry(36)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PHASE_ERROR,
        residue: 0,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::None,
    },
    BotCase {
        name: "2b Hn<Di: READ(10) under a CBW declaring no data phase",
        cbw: cbw(0x03, 0, true, 0, rw10(OP_READ, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 0,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::None,
    },
    BotCase {
        name: "3 Hn<Do: WRITE(10) under a CBW declaring no data phase",
        cbw: cbw(0x04, 0, false, 0, rw10(OP_WRITE, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 0,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::None,
    },
    BotCase {
        name: "4 Hi>Dn: TEST UNIT READY declaring a data-IN phase",
        cbw: cbw(0x05, 36, true, 0, TUR),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 36,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::In,
    },
    BotCase {
        name: "5 Hi>Di: INQUIRY replying short of the declared length",
        cbw: cbw(0x06, 64, true, 0, inquiry(36)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 36,
        status: CSW_PASSED,
        residue: 28,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "6 Hi=Di: READ(10) of one sector",
        cbw: cbw(0x07, SECTOR, true, 0, rw10(OP_READ, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: MSC_SECTOR_SIZE,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "6b Hi=Di: INQUIRY matching the declared length",
        cbw: cbw(0x08, 36, true, 0, inquiry(36)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 36,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "7 Hi<Di: INQUIRY clamped to the shorter declared length",
        cbw: cbw(0x09, 16, true, 0, inquiry(36)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 16,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "7b Hi<Di: READ(10) of two sectors declaring one",
        cbw: cbw(0x0A, SECTOR, true, 0, rw10(OP_READ, 5, 2)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::In,
    },
    BotCase {
        name: "8 Hi<>Do: WRITE(10) under a data-IN direction bit",
        cbw: cbw(0x0B, SECTOR, true, 0, rw10(OP_WRITE, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PHASE_ERROR,
        residue: SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::In,
    },
    BotCase {
        name: "9 Ho>Dn: TEST UNIT READY declaring a data-OUT phase",
        cbw: cbw(0x0C, SECTOR, false, 0, TUR),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "10 Ho<>Di: READ(10) under a data-OUT direction bit",
        cbw: cbw(0x0D, SECTOR, false, 0, rw10(OP_READ, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PHASE_ERROR,
        residue: SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "11 Ho>Do: WRITE(10) of one sector declaring two",
        cbw: cbw(0x0E, 2 * SECTOR, false, 0, rw10(OP_WRITE, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 2 * SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "12 Ho=Do: WRITE(10) of one sector",
        cbw: cbw(0x0F, SECTOR, false, 0, rw10(OP_WRITE, 5, 1)),
        out_packets: SECTOR_PACKETS,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "13 Ho<Do: WRITE(10) of two sectors declaring one",
        cbw: cbw(0x10, SECTOR, false, 0, rw10(OP_WRITE, 5, 2)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "UA clean: TEST UNIT READY reports the pending attention",
        cbw: cbw(0x11, 0, false, 0, TUR),
        out_packets: 0,
        ua_before: true,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 0,
        sense: SENSE_MEDIUM_CHANGED,
        stall: StallIntent::None,
    },
    BotCase {
        name: "UA violation: TEST UNIT READY declaring a data-IN phase keeps the attention",
        cbw: cbw(0x12, 36, true, 0, TUR),
        out_packets: 0,
        ua_before: true,
        ua_after: true,
        data_len: 0,
        status: CSW_FAILED,
        residue: 36,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::In,
    },
    BotCase {
        name: "Reply clamped to nothing: INQUIRY with a zero allocation length",
        cbw: cbw(0x13, 36, true, 0, inquiry(0)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PASSED,
        residue: 36,
        sense: SENSE_NONE,
        stall: StallIntent::In,
    },
    BotCase {
        name: "Absent LUN: a CBW addressed to LUN 1 declaring a data-IN phase",
        cbw: cbw(0x14, 36, true, 1, inquiry(36)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 36,
        sense: SENSE_LUN_NOT_SUPPORTED,
        stall: StallIntent::In,
    },
    BotCase {
        name: "Dispatch failure: an unknown opcode declaring a data-IN phase",
        cbw: cbw(0x15, 36, true, 0, UNKNOWN_OP),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 36,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::In,
    },
    BotCase {
        name: "Absent LUN: a CBW addressed to LUN 1 declaring a data-OUT phase",
        cbw: cbw(0x16, SECTOR, false, 1, rw10(OP_WRITE, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: SECTOR,
        sense: SENSE_LUN_NOT_SUPPORTED,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "Dispatch failure: an unknown opcode declaring a data-OUT phase",
        cbw: cbw(0x17, SECTOR, false, 0, UNKNOWN_OP),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "Dispatch failure: an unknown opcode with no data phase declared",
        cbw: cbw(0x18, 0, false, 0, UNKNOWN_OP),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 0,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::None,
    },
    BotCase {
        name: "7c Hi>Di: READ(10) of one sector declaring two",
        cbw: cbw(0x19, 2 * SECTOR, true, 0, rw10(OP_READ, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 2 * SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::In,
    },
    BotCase {
        name: "7d Hi<Di: READ(10) of one sector declaring a non-multiple of the sector size",
        cbw: cbw(0x1A, 64, true, 0, rw10(OP_READ, 5, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 64,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::In,
    },
    BotCase {
        name: "6c Hi=Di: READ(10) of two sectors",
        cbw: cbw(0x1B, 2 * SECTOR, true, 0, rw10(OP_READ, 5, 2)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 2 * MSC_SECTOR_SIZE,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "12b Ho=Do: WRITE(10) of two sectors",
        cbw: cbw(0x1C, 2 * SECTOR, false, 0, rw10(OP_WRITE, 5, 2)),
        out_packets: 2 * SECTOR_PACKETS,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Zero sector count: READ(10) of no sectors under no data phase",
        cbw: cbw(0x1D, 0, false, 0, rw10(OP_READ, 5, 0)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Zero sector count: WRITE(10) of no sectors declaring a data-OUT phase",
        cbw: cbw(0x1E, SECTOR, false, 0, rw10(OP_WRITE, 5, 0)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: SECTOR,
        sense: SENSE_ILLEGAL,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "Out of range: READ(10) straddling the last block",
        cbw: cbw(
            0x1F,
            2 * SECTOR,
            true,
            0,
            rw10(OP_READ, VFS_TOTAL_SECTORS - 1, 2),
        ),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 2 * SECTOR,
        sense: SENSE_LBA_RANGE,
        stall: StallIntent::In,
    },
    BotCase {
        name: "Out of range: WRITE(10) one block past the end",
        cbw: cbw(0x20, SECTOR, false, 0, rw10(OP_WRITE, VFS_TOTAL_SECTORS, 1)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: SECTOR,
        sense: SENSE_LBA_RANGE,
        stall: StallIntent::Out,
    },
    BotCase {
        name: "Residue: REQUEST SENSE matching the 18-byte descriptor",
        cbw: cbw(0x21, 18, true, 0, request_sense(18)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 18,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Residue: REQUEST SENSE allocating more than the descriptor holds",
        cbw: cbw(0x22, 64, true, 0, request_sense(64)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 18,
        status: CSW_PASSED,
        residue: 46,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Residue: MODE SENSE(6) write-protect probe allocating 192 bytes",
        cbw: cbw(0x23, 0xC0, true, 0, mode_sense6(0xC0)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 4,
        status: CSW_PASSED,
        residue: 0xC0 - 4,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Residue: READ FORMAT CAPACITIES allocating 252 bytes",
        cbw: cbw(0x24, 0xFC, true, 0, read_format_capacities(0xFC)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 12,
        status: CSW_PASSED,
        residue: 0xFC - 12,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "6d Hi=Di: READ CAPACITY(10) matching its eight-byte reply",
        cbw: cbw(0x25, 8, true, 0, READ_CAPACITY),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 8,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "7e Hi<Di: READ CAPACITY(10) clamped to a shorter declared length",
        cbw: cbw(0x26, 4, true, 0, READ_CAPACITY),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 4,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Over-long transfer: INQUIRY declaring the whole 32-bit length",
        cbw: cbw(0x27, u32::MAX, true, 0, inquiry(36)),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 36,
        status: CSW_PASSED,
        residue: u32::MAX - 36,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "UA held: INQUIRY does not spend the pending attention",
        cbw: cbw(0x28, 36, true, 0, inquiry(36)),
        out_packets: 0,
        ua_before: true,
        ua_after: true,
        data_len: 36,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Absent LUN: reserved LUN bits must not alias LUN 0",
        cbw: cbw(0x29, 0, false, 0x10, TUR),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_FAILED,
        residue: 0,
        sense: SENSE_LUN_NOT_SUPPORTED,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Unmount path: SYNCHRONIZE CACHE(10), no data phase",
        cbw: cbw(0x2A, 0, false, 0, SYNC_CACHE),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
    BotCase {
        name: "Unmount path: START STOP UNIT with LOEJ, no data phase",
        cbw: cbw(0x2B, 0, false, 0, START_STOP_EJECT),
        out_packets: 0,
        ua_before: false,
        ua_after: false,
        data_len: 0,
        status: CSW_PASSED,
        residue: 0,
        sense: SENSE_NONE,
        stall: StallIntent::None,
    },
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msc_proto::parse_cbw;

    #[test]
    fn every_case_carries_a_well_formed_cbw() {
        // The rows are raw wire bytes, so the parser is what says they are CBWs at all.
        for case in BOT_CASES {
            let parsed = parse_cbw(&case.cbw).unwrap_or_else(|| panic!("{}: not a CBW", case.name));
            assert_eq!(parsed.tag, case.tag(), "{}", case.name);
            assert!(parsed.data_len >= case.residue, "{}", case.name);
        }
    }

    #[test]
    fn case_names_and_tags_are_unique() {
        // A duplicate name or tag would let one row's failure be read as another's.
        for (i, a) in BOT_CASES.iter().enumerate() {
            for b in &BOT_CASES[i + 1..] {
                assert_ne!(a.name, b.name);
                assert_ne!(a.tag(), b.tag(), "{} / {}", a.name, b.name);
            }
        }
    }

    #[test]
    fn a_passing_row_accounts_for_every_declared_byte() {
        // Residue is what the CBW declared minus what moved, so a row that does not add up is a
        // wrong expectation rather than a transport bug, and would be read as the latter.
        for case in BOT_CASES {
            if case.status != CSW_PASSED {
                continue;
            }
            let parsed = parse_cbw(&case.cbw).expect("a CBW");
            let moved = if parsed.dir_in {
                case.data_len as u32
            } else {
                (case.out_packets * 64) as u32
            };
            assert_eq!(moved + case.residue, parsed.data_len, "{}", case.name);
        }
    }

    #[test]
    fn a_failing_row_moves_nothing_and_returns_the_whole_declared_length() {
        // A rejected command must not consume host packets: doing so desynchronizes the stream.
        for case in BOT_CASES {
            if case.status == CSW_PASSED {
                continue;
            }
            let parsed = parse_cbw(&case.cbw).expect("a CBW");
            assert_eq!(case.data_len, 0, "{}", case.name);
            assert_eq!(case.out_packets, 0, "{}", case.name);
            assert_eq!(case.residue, parsed.data_len, "{}", case.name);
        }
    }

    #[test]
    fn a_stall_intent_implies_a_declared_phase_in_that_direction() {
        // A stall needs a declared data phase, in the direction the host itself declared.
        for case in BOT_CASES {
            let parsed = parse_cbw(&case.cbw).expect("a CBW");
            match case.stall {
                StallIntent::None => {}
                StallIntent::In => assert!(parsed.dir_in && parsed.data_len > 0, "{}", case.name),
                StallIntent::Out => assert!(!parsed.dir_in && parsed.data_len > 0, "{}", case.name),
            }
            assert_eq!(case.ends_with_zlp(), case.stall == StallIntent::In);
        }
    }
}
