//! Bulk-Only-Transport (BOT) + SCSI command layer for the MSC class. Pure, no I/O.

use crate::{
    MSC_SECTOR_SIZE, SCSI_PRODUCT_ID, SCSI_REVISION_ID, SCSI_VENDOR_ID, VFS_TOTAL_SECTORS,
};

/// CBW signature ("USBC").
pub const CBW_SIGNATURE: u32 = 0x4342_5355;
/// CSW signature ("USBS").
pub const CSW_SIGNATURE: u32 = 0x5342_5355;
/// CSW status: command passed.
pub const CSW_PASSED: u8 = 0;
/// CSW status: command failed.
pub const CSW_FAILED: u8 = 1;
/// CSW status: phase error, for a CBW whose declared direction or data-phase presence contradicts
/// the command; the host recovers with a Bulk-Only reset. Length mismatches use [`CSW_FAILED`].
pub const CSW_PHASE_ERROR: u8 = 2;

/// A parsed Command Block Wrapper.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cbw {
    /// Command tag echoed back in the CSW.
    pub tag: u32,
    /// Expected data-transfer length.
    pub data_len: u32,
    /// Direction: `true` = device-to-host (IN).
    pub dir_in: bool,
    /// Logical unit number.
    pub lun: u8,
    /// The SCSI command block bytes.
    pub cb: [u8; 16],
    /// Valid length of `cb`.
    pub cb_len: u8,
}

/// A SCSI sense triple (key / additional-sense-code / qualifier).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sense {
    /// Sense key.
    pub key: u8,
    /// Additional sense code.
    pub asc: u8,
    /// Additional sense code qualifier.
    pub ascq: u8,
}

/// No error.
pub const SENSE_NONE: Sense = Sense {
    key: 0x00,
    asc: 0x00,
    ascq: 0x00,
};
/// Invalid command / field.
pub const SENSE_ILLEGAL: Sense = Sense {
    key: 0x05,
    asc: 0x20,
    ascq: 0x00,
};
/// Logical unit not supported: the CBW named a unit this device does not have.
pub const SENSE_LUN_NOT_SUPPORTED: Sense = Sense {
    key: 0x05,
    asc: 0x25,
    ascq: 0x00,
};
/// Medium not present.
pub const SENSE_NO_MEDIUM: Sense = Sense {
    key: 0x02,
    asc: 0x3A,
    ascq: 0x00,
};
/// LBA out of range.
pub const SENSE_LBA_RANGE: Sense = Sense {
    key: 0x05,
    asc: 0x21,
    ascq: 0x00,
};
/// Medium may have changed (remount lever).
pub const SENSE_MEDIUM_CHANGED: Sense = Sense {
    key: 0x06,
    asc: 0x28,
    ascq: 0x00,
};

/// What the transport should do in response to a dispatched SCSI command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScsiAction {
    /// Return `data[..len]` to the host, then a passing CSW.
    Reply {
        /// Reply bytes buffer.
        data: [u8; 36],
        /// Valid length of `data`.
        len: usize,
    },
    /// Host wants to read `count` sectors starting at `lba`.
    ReadSectors {
        /// Starting logical block address.
        lba: u32,
        /// Sector count.
        count: u32,
    },
    /// Host wants to write `count` sectors starting at `lba`.
    WriteSectors {
        /// Starting logical block address.
        lba: u32,
        /// Sector count.
        count: u32,
    },
    /// No data phase; passing CSW.
    Ok,
    /// Latch `sense` and return a failing CSW.
    Fail(Sense),
}

/// Clamp a reply's byte length to the host's declared CBW data-transfer length; a device that
/// sends more than the CBW declared reports a wrong residue.
pub fn clamp_reply_len(reply_len: usize, cbw_data_len: u32) -> usize {
    core::cmp::min(reply_len, cbw_data_len as usize)
}

/// Whether a block command's declared CBW length is exactly `count * MSC_SECTOR_SIZE`. A CBW that
/// disagrees must be rejected before the data phase, or the Bulk-Only stream desynchronizes.
pub fn block_data_len_matches(cbw_data_len: u32, count: u32) -> bool {
    cbw_data_len as u64 == count as u64 * MSC_SECTOR_SIZE as u64
}

/// What the transport should do with a dispatched command once the CBW's declared data phase has
/// been reconciled against it -- the result of [`reconcile_cbw`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BotVerdict {
    /// The CBW and the command agree; run the data phase (if any) and report as usual.
    Proceed,
    /// The CBW contradicts the command; skip the data phase and report a failing CSW.
    Reject {
        /// The CSW status to report: [`CSW_FAILED`] or [`CSW_PHASE_ERROR`].
        status: u8,
        /// The sense to latch for the following REQUEST SENSE.
        sense: Sense,
        /// Which endpoint a stalling transport stalls before the CSW: `Some(true)` = IN,
        /// `Some(false)` = OUT, `None` = nothing declared. A non-stalling one uses the residue.
        stall_dir_in: Option<bool>,
    },
}

/// Reconcile a CBW's declared data phase with the dispatched command -- the "thirteen cases" gate.
/// Direction or presence disagreements are [`CSW_PHASE_ERROR`], length ones [`CSW_FAILED`].
pub fn reconcile_cbw(dir_in: bool, data_len: u32, action: &ScsiAction) -> BotVerdict {
    // A reject stalls the host's declared direction, and only when a data phase was declared.
    let stall_host_dir = if data_len > 0 { Some(dir_in) } else { None };
    let reject = |status: u8, stall_dir_in: Option<bool>| BotVerdict::Reject {
        status,
        sense: SENSE_ILLEGAL,
        stall_dir_in,
    };
    match *action {
        // A data-IN reply with no declared data phase: phase error, nothing to stall.
        ScsiAction::Reply { len, .. } if len > 0 && data_len == 0 => reject(CSW_PHASE_ERROR, None),
        // A data-IN reply against a host-to-device data phase: direction mismatch.
        ScsiAction::Reply { .. } if !dir_in && data_len > 0 => {
            reject(CSW_PHASE_ERROR, stall_host_dir)
        }
        // READ(10) against an OUT phase, or WRITE(10) against an IN one: direction mismatch.
        ScsiAction::ReadSectors { .. } if data_len > 0 && !dir_in => {
            reject(CSW_PHASE_ERROR, stall_host_dir)
        }
        ScsiAction::WriteSectors { .. } if data_len > 0 && dir_in => {
            reject(CSW_PHASE_ERROR, stall_host_dir)
        }
        // A block command whose declared length is not `count * MSC_SECTOR_SIZE`.
        ScsiAction::ReadSectors { count, .. } | ScsiAction::WriteSectors { count, .. }
            if !block_data_len_matches(data_len, count) =>
        {
            reject(CSW_FAILED, stall_host_dir)
        }
        // A no-data command with a declared data phase: an unstalled CSW would be read as data.
        ScsiAction::Ok if data_len > 0 => reject(CSW_FAILED, stall_host_dir),
        _ => BotVerdict::Proceed,
    }
}

/// Whether processing `opcode` should clear the latched sense after replying: REQUEST SENSE
/// reports the latched condition once and clears it, every other command leaves it untouched.
pub fn should_clear_sense(opcode: u8) -> bool {
    opcode == OP_REQUEST_SENSE
}

/// Whether `opcode` is TEST UNIT READY, the command a pending UNIT ATTENTION is reported on.
pub fn is_test_unit_ready(opcode: u8) -> bool {
    opcode == OP_TEST_UNIT_READY
}

/// Length of a Command Block Wrapper, in bytes.
pub const CBW_LEN: usize = 31;
/// Length of a Command Status Wrapper, in bytes.
pub const CSW_LEN: usize = 13;

/// Parse a [`CBW_LEN`]-byte Command Block Wrapper; signature, packet length and `bCBWCBLength`
/// (`1..=16`) are validated, and `bCBWLUN` is returned whole so a reserved-bit LUN cannot alias 0.
pub fn parse_cbw(pkt: &[u8]) -> Option<Cbw> {
    if pkt.len() != CBW_LEN {
        return None;
    }
    let signature = u32::from_le_bytes([pkt[0], pkt[1], pkt[2], pkt[3]]);
    if signature != CBW_SIGNATURE {
        return None;
    }
    let tag = u32::from_le_bytes([pkt[4], pkt[5], pkt[6], pkt[7]]);
    // Unbounded on purpose: replies clamp, block phases must match exactly, residues saturate.
    let data_len = u32::from_le_bytes([pkt[8], pkt[9], pkt[10], pkt[11]]);
    let dir_in = (pkt[12] & 0x80) != 0;
    // Reserved bits included: masking would fold LUN 0x10 onto LUN 0 and reach the real volume.
    let lun = pkt[13];
    let cb_len = pkt[14] & 0x1F;
    if !(1..=16).contains(&cb_len) {
        return None;
    }
    let mut cb = [0u8; 16];
    cb.copy_from_slice(&pkt[15..CBW_LEN]);
    Some(Cbw {
        tag,
        data_len,
        dir_in,
        lun,
        cb,
        cb_len,
    })
}

/// Build a [`CSW_LEN`]-byte Command Status Wrapper.
pub fn build_csw(tag: u32, residue: u32, status: u8) -> [u8; CSW_LEN] {
    let mut csw = [0u8; CSW_LEN];
    csw[0..4].copy_from_slice(&CSW_SIGNATURE.to_le_bytes());
    csw[4..8].copy_from_slice(&tag.to_le_bytes());
    csw[8..12].copy_from_slice(&residue.to_le_bytes());
    csw[12] = status;
    csw
}

// SCSI opcodes.
const OP_TEST_UNIT_READY: u8 = 0x00;
const OP_REQUEST_SENSE: u8 = 0x03;
const OP_INQUIRY: u8 = 0x12;
const OP_MODE_SENSE_6: u8 = 0x1A;
const OP_START_STOP_UNIT: u8 = 0x1B;
const OP_PREVENT_ALLOW_REMOVAL: u8 = 0x1E;
const OP_READ_FORMAT_CAPACITIES: u8 = 0x23;
const OP_READ_CAPACITY_10: u8 = 0x25;
const OP_READ_10: u8 = 0x28;
const OP_WRITE_10: u8 = 0x2A;
const OP_VERIFY_10: u8 = 0x2F;
const OP_SYNCHRONIZE_CACHE_10: u8 = 0x35;
const OP_SYNCHRONIZE_CACHE_16: u8 = 0x91;

fn be16(b: &[u8]) -> u16 {
    u16::from_be_bytes([b[0], b[1]])
}

fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

fn reply(bytes: &[u8], alloc_len: usize) -> ScsiAction {
    let mut data = [0u8; 36];
    data[..bytes.len()].copy_from_slice(bytes);
    let len = core::cmp::min(bytes.len(), alloc_len);
    ScsiAction::Reply { data, len }
}

fn in_range(lba: u32, count: u32) -> bool {
    lba.checked_add(count)
        .is_some_and(|end| end <= VFS_TOTAL_SECTORS)
}

/// Dispatch a SCSI command block.
pub fn dispatch(cb: &[u8; 16], media_ready: bool, sense: Sense) -> ScsiAction {
    let op = cb[0];
    match op {
        OP_TEST_UNIT_READY => {
            if media_ready {
                ScsiAction::Ok
            } else {
                ScsiAction::Fail(SENSE_NO_MEDIUM)
            }
        }
        OP_REQUEST_SENSE => {
            let mut buf = [0u8; 18];
            buf[0] = 0x70;
            buf[2] = sense.key;
            buf[7] = 10;
            buf[12] = sense.asc;
            buf[13] = sense.ascq;
            reply(&buf, cb[4] as usize)
        }
        OP_INQUIRY => {
            let mut buf = [0u8; 36];
            buf[0] = 0x00;
            buf[1] = 0x80;
            buf[2] = 0x04;
            buf[3] = 0x02;
            buf[4] = 31;
            buf[8..16].copy_from_slice(SCSI_VENDOR_ID);
            buf[16..32].copy_from_slice(SCSI_PRODUCT_ID);
            buf[32..36].copy_from_slice(SCSI_REVISION_ID);
            // INQUIRY's allocation length is the 16-bit big-endian field at cb[3..5].
            reply(&buf, be16(&cb[3..5]) as usize)
        }
        OP_MODE_SENSE_6 => {
            let mut buf = [0u8; 4];
            buf[0] = 3;
            reply(&buf, cb[4] as usize)
        }
        OP_START_STOP_UNIT | OP_PREVENT_ALLOW_REMOVAL | OP_VERIFY_10 => ScsiAction::Ok,
        // SYNCHRONIZE CACHE(10)/(16): the sink writes through, so success is truthful.
        OP_SYNCHRONIZE_CACHE_10 | OP_SYNCHRONIZE_CACHE_16 => ScsiAction::Ok,
        OP_READ_FORMAT_CAPACITIES => {
            // One capacity descriptor: 4-byte header, block COUNT, code, 24-bit block length.
            let mut buf = [0u8; 12];
            buf[3] = 8;
            buf[4..8].copy_from_slice(&VFS_TOTAL_SECTORS.to_be_bytes());
            buf[8] = 0x02; // formatted media
            buf[9..12].copy_from_slice(&(MSC_SECTOR_SIZE as u32).to_be_bytes()[1..]);
            // The allocation length is the 16-bit big-endian field at cb[7..9].
            reply(&buf, be16(&cb[7..9]) as usize)
        }
        OP_READ_CAPACITY_10 => {
            let mut buf = [0u8; 8];
            buf[0..4].copy_from_slice(&(VFS_TOTAL_SECTORS - 1).to_be_bytes());
            buf[4..8].copy_from_slice(&(MSC_SECTOR_SIZE as u32).to_be_bytes());
            let mut data = [0u8; 36];
            data[..buf.len()].copy_from_slice(&buf);
            ScsiAction::Reply { data, len: 8 }
        }
        OP_READ_10 => {
            let lba = be32(&cb[2..6]);
            let count = be16(&cb[7..9]) as u32;
            // Readiness precedes the zero-count no-op: a not-ready unit must report NOT READY.
            if !media_ready {
                ScsiAction::Fail(SENSE_NO_MEDIUM)
            } else if count == 0 {
                ScsiAction::Ok
            } else if !in_range(lba, count) {
                ScsiAction::Fail(SENSE_LBA_RANGE)
            } else {
                ScsiAction::ReadSectors { lba, count }
            }
        }
        OP_WRITE_10 => {
            let lba = be32(&cb[2..6]);
            let count = be16(&cb[7..9]) as u32;
            if !media_ready {
                ScsiAction::Fail(SENSE_NO_MEDIUM)
            } else if count == 0 {
                ScsiAction::Ok
            } else if !in_range(lba, count) {
                ScsiAction::Fail(SENSE_LBA_RANGE)
            } else {
                ScsiAction::WriteSectors { lba, count }
            }
        }
        // Deliberately minimal: the set above is what a host needs to mount and stream an image.
        _ => ScsiAction::Fail(SENSE_ILLEGAL),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cb_with(op: u8, rest: &[u8]) -> [u8; 16] {
        let mut cb = [0u8; 16];
        cb[0] = op;
        cb[1..1 + rest.len()].copy_from_slice(rest);
        cb
    }

    fn make_cbw(tag: u32, data_len: u32, dir_in: bool, cb: &[u8]) -> [u8; 31] {
        let mut pkt = [0u8; 31];
        pkt[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
        pkt[4..8].copy_from_slice(&tag.to_le_bytes());
        pkt[8..12].copy_from_slice(&data_len.to_le_bytes());
        pkt[12] = if dir_in { 0x80 } else { 0x00 };
        pkt[13] = 0;
        pkt[14] = cb.len() as u8;
        pkt[15..15 + cb.len()].copy_from_slice(cb);
        pkt
    }

    #[test]
    fn cbw_round_trip() {
        let cb = cb_with(
            OP_READ_10,
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00, 0x08],
        );
        let pkt = make_cbw(0x1122_3344, 4096, true, &cb);
        let parsed = parse_cbw(&pkt).unwrap();
        assert_eq!(parsed.tag, 0x1122_3344);
        assert_eq!(parsed.data_len, 4096);
        assert!(parsed.dir_in);
        assert_eq!(parsed.lun, 0);
        assert_eq!(parsed.cb_len, 16);
        assert_eq!(parsed.cb, cb);
    }

    #[test]
    fn cbw_lun_reserved_bits_are_not_masked() {
        // LUN 0x10 must be reported whole, so the transports' single-LUN check rejects it.
        let mut pkt = make_cbw(1, 0, false, &[0x00]);
        pkt[13] = 0x10;
        assert_eq!(parse_cbw(&pkt).unwrap().lun, 0x10);
    }

    #[test]
    fn cbw_bad_signature_rejected() {
        let mut pkt = make_cbw(1, 0, false, &[0x00]);
        pkt[0] = 0xFF;
        assert_eq!(parse_cbw(&pkt), None);
    }

    #[test]
    fn cbw_cb_len_zero_rejected() {
        let mut pkt = make_cbw(1, 0, false, &[0x00]);
        pkt[14] = 0;
        assert_eq!(parse_cbw(&pkt), None);
    }

    #[test]
    fn cbw_cb_len_over_16_rejected() {
        let mut pkt = make_cbw(1, 0, false, &[0x00]);
        pkt[14] = 17;
        assert_eq!(parse_cbw(&pkt), None);
    }

    #[test]
    fn cbw_short_packet_rejected() {
        let pkt = make_cbw(1, 0, false, &[0x00]);
        assert_eq!(parse_cbw(&pkt[..30]), None);
        let mut long = [0u8; 32];
        long[..31].copy_from_slice(&pkt);
        assert_eq!(parse_cbw(&long), None);
    }

    #[test]
    fn csw_bytes() {
        let csw = build_csw(0xDEAD_BEEF, 5, CSW_FAILED);
        assert_eq!(
            csw,
            [
                0x55, 0x53, 0x42, 0x53, // signature LE
                0xEF, 0xBE, 0xAD, 0xDE, // tag LE
                0x05, 0x00, 0x00, 0x00, // residue LE
                0x01, // status
            ]
        );
    }

    #[test]
    fn inquiry_exact_bytes() {
        let cb = cb_with(OP_INQUIRY, &[0x00, 0x00, 0x00, 36]);
        match dispatch(&cb, true, SENSE_NONE) {
            ScsiAction::Reply { data, len } => {
                assert_eq!(len, 36);
                assert_eq!(data[0], 0x00);
                assert_eq!(data[1], 0x80);
                assert_eq!(data[2], 0x04);
                assert_eq!(data[3], 0x02);
                assert_eq!(data[4], 31);
                assert_eq!(&data[8..16], SCSI_VENDOR_ID);
                assert_eq!(&data[16..32], SCSI_PRODUCT_ID);
                assert_eq!(&data[32..36], SCSI_REVISION_ID);
            }
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    #[test]
    fn inquiry_clamps_to_alloc_len() {
        let cb = cb_with(OP_INQUIRY, &[0x00, 0x00, 0x00, 5]);
        match dispatch(&cb, true, SENSE_NONE) {
            ScsiAction::Reply { len, .. } => assert_eq!(len, 5),
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    #[test]
    fn inquiry_16bit_alloc_length() {
        // cb[3] = 0x00, cb[4] = 0x24 (36) -> full 36-byte reply.
        let cb = cb_with(OP_INQUIRY, &[0x00, 0x00, 0x00, 0x24]);
        match dispatch(&cb, true, SENSE_NONE) {
            ScsiAction::Reply { len, .. } => assert_eq!(len, 36),
            other => panic!("expected Reply, got {other:?}"),
        }

        // cb[3] = 0x01, cb[4] = 0x00 (256) -> min(36, 256) = 36; a byte read of cb[4] gives 0.
        let cb = cb_with(OP_INQUIRY, &[0x00, 0x00, 0x01, 0x00]);
        match dispatch(&cb, true, SENSE_NONE) {
            ScsiAction::Reply { len, .. } => assert_eq!(len, 36),
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    #[test]
    fn read_capacity_be_math() {
        let cb = cb_with(OP_READ_CAPACITY_10, &[]);
        match dispatch(&cb, true, SENSE_NONE) {
            ScsiAction::Reply { data, len } => {
                assert_eq!(len, 8);
                assert_eq!(&data[0..4], &(VFS_TOTAL_SECTORS - 1).to_be_bytes());
                assert_eq!(&data[4..8], &512u32.to_be_bytes());
            }
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    /// Build the CDB tail for a READ(10)/WRITE(10): lba at cb[2..6], count at cb[7..9].
    fn rw10_rest(lba: u32, count: u16) -> [u8; 9] {
        // `rest` maps to cb[1..]; so cb[2..6] = rest[1..5], cb[7..9] = rest[6..8].
        let mut rest = [0u8; 9];
        rest[1..5].copy_from_slice(&lba.to_be_bytes());
        rest[6..8].copy_from_slice(&count.to_be_bytes());
        rest
    }

    #[test]
    fn read10_field_extraction() {
        let cb = cb_with(OP_READ_10, &rw10_rest(0x10, 4));
        assert_eq!(
            dispatch(&cb, true, SENSE_NONE),
            ScsiAction::ReadSectors {
                lba: 0x10,
                count: 4
            }
        );
    }

    #[test]
    fn write10_field_extraction() {
        let cb = cb_with(OP_WRITE_10, &rw10_rest(0x10, 4));
        assert_eq!(
            dispatch(&cb, true, SENSE_NONE),
            ScsiAction::WriteSectors {
                lba: 0x10,
                count: 4
            }
        );
    }

    #[test]
    fn read10_at_last_sector_ok() {
        let count: u32 = 8;
        let lba = VFS_TOTAL_SECTORS - count;
        let cb = cb_with(OP_READ_10, &rw10_rest(lba, count as u16));
        assert_eq!(
            dispatch(&cb, true, SENSE_NONE),
            ScsiAction::ReadSectors { lba, count }
        );
    }

    #[test]
    fn read10_past_end_lba_range() {
        let count: u32 = 8;
        let lba = VFS_TOTAL_SECTORS - count + 1;
        let cb = cb_with(OP_READ_10, &rw10_rest(lba, count as u16));
        assert_eq!(
            dispatch(&cb, true, SENSE_NONE),
            ScsiAction::Fail(SENSE_LBA_RANGE)
        );
    }

    #[test]
    fn write10_past_end_lba_range() {
        let count: u32 = 8;
        let lba = VFS_TOTAL_SECTORS - count + 1;
        let cb = cb_with(OP_WRITE_10, &rw10_rest(lba, count as u16));
        assert_eq!(
            dispatch(&cb, true, SENSE_NONE),
            ScsiAction::Fail(SENSE_LBA_RANGE)
        );
    }

    #[test]
    fn read10_zero_count_is_ok() {
        let cb = cb_with(OP_READ_10, &rw10_rest(0x10, 0));
        assert_eq!(dispatch(&cb, true, SENSE_NONE), ScsiAction::Ok);
    }

    #[test]
    fn read10_zero_count_no_medium_is_not_ready() {
        let cb = cb_with(OP_READ_10, &rw10_rest(0x10, 0));
        assert_eq!(
            dispatch(&cb, false, SENSE_NONE),
            ScsiAction::Fail(SENSE_NO_MEDIUM)
        );
    }

    #[test]
    fn write10_zero_count_no_medium_is_not_ready() {
        let cb = cb_with(OP_WRITE_10, &rw10_rest(0x10, 0));
        assert_eq!(
            dispatch(&cb, false, SENSE_NONE),
            ScsiAction::Fail(SENSE_NO_MEDIUM)
        );
    }

    #[test]
    fn write10_zero_count_is_ok() {
        let cb = cb_with(OP_WRITE_10, &rw10_rest(0x10, 0));
        assert_eq!(dispatch(&cb, true, SENSE_NONE), ScsiAction::Ok);
    }

    #[test]
    fn read_write_no_medium() {
        let read = cb_with(OP_READ_10, &rw10_rest(0, 1));
        assert_eq!(
            dispatch(&read, false, SENSE_NONE),
            ScsiAction::Fail(SENSE_NO_MEDIUM)
        );
        let write = cb_with(OP_WRITE_10, &rw10_rest(0, 1));
        assert_eq!(
            dispatch(&write, false, SENSE_NONE),
            ScsiAction::Fail(SENSE_NO_MEDIUM)
        );
    }

    #[test]
    fn test_unit_ready_both_states() {
        let cb = cb_with(OP_TEST_UNIT_READY, &[]);
        assert_eq!(dispatch(&cb, true, SENSE_NONE), ScsiAction::Ok);
        assert_eq!(
            dispatch(&cb, false, SENSE_NONE),
            ScsiAction::Fail(SENSE_NO_MEDIUM)
        );
    }

    #[test]
    fn unknown_opcode_illegal() {
        let cb = cb_with(0xFD, &[]);
        assert_eq!(
            dispatch(&cb, true, SENSE_NONE),
            ScsiAction::Fail(SENSE_ILLEGAL)
        );
    }

    #[test]
    fn request_sense_renders_latched() {
        let latched = Sense {
            key: 0x05,
            asc: 0x21,
            ascq: 0x00,
        };
        let cb = cb_with(OP_REQUEST_SENSE, &[0x00, 0x00, 0x00, 18]);
        match dispatch(&cb, true, latched) {
            ScsiAction::Reply { data, len } => {
                assert_eq!(len, 18);
                assert_eq!(data[0], 0x70);
                assert_eq!(data[2], latched.key);
                assert_eq!(data[7], 10);
                assert_eq!(data[12], latched.asc);
                assert_eq!(data[13], latched.ascq);
            }
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    #[test]
    fn request_sense_clamps_to_alloc_len() {
        let cb = cb_with(OP_REQUEST_SENSE, &[0x00, 0x00, 0x00, 8]);
        match dispatch(&cb, true, SENSE_NONE) {
            ScsiAction::Reply { len, .. } => assert_eq!(len, 8),
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    #[test]
    fn nop_commands_ok() {
        assert_eq!(
            dispatch(&cb_with(OP_START_STOP_UNIT, &[]), true, SENSE_NONE),
            ScsiAction::Ok
        );
        assert_eq!(
            dispatch(&cb_with(OP_PREVENT_ALLOW_REMOVAL, &[]), true, SENSE_NONE),
            ScsiAction::Ok
        );
        assert_eq!(
            dispatch(&cb_with(OP_VERIFY_10, &[]), true, SENSE_NONE),
            ScsiAction::Ok
        );
        match dispatch(
            &cb_with(OP_MODE_SENSE_6, &[0x00, 0x00, 0x00, 4]),
            true,
            SENSE_NONE,
        ) {
            ScsiAction::Reply { data, .. } => assert_eq!(data[0], 3),
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    #[test]
    fn clamp_reply_len_never_exceeds_cbw_data_len() {
        // Reply shorter than the declared data phase is returned as-is...
        assert_eq!(clamp_reply_len(8, 36), 8);
        // ...and a reply longer than the CBW declared is clamped down (residue stays correct).
        assert_eq!(clamp_reply_len(36, 16), 16);
        assert_eq!(clamp_reply_len(36, 0), 0);
    }

    #[test]
    fn should_clear_sense_only_for_request_sense() {
        assert!(should_clear_sense(OP_REQUEST_SENSE));
        assert!(should_clear_sense(0x03));
        assert!(!should_clear_sense(OP_INQUIRY));
        assert!(!should_clear_sense(OP_TEST_UNIT_READY));
    }

    /// Shorthand for the expected reject verdicts in the `reconcile_cbw` rule tests.
    fn reject(status: u8, stall_dir_in: Option<bool>) -> BotVerdict {
        BotVerdict::Reject {
            status,
            sense: SENSE_ILLEGAL,
            stall_dir_in,
        }
    }

    #[test]
    fn reconcile_reply_with_zero_data_len_is_phase_error() {
        // Case 2 (Hn < Di, non-block): a data-IN reply against a CBW declaring no data phase.
        let action = ScsiAction::Reply {
            data: [0u8; 36],
            len: 36,
        };
        assert_eq!(
            reconcile_cbw(true, 0, &action),
            reject(CSW_PHASE_ERROR, None)
        );
        // A zero-length reply against a zero data phase is fine (nothing to send).
        let empty = ScsiAction::Reply {
            data: [0u8; 36],
            len: 0,
        };
        assert_eq!(reconcile_cbw(true, 0, &empty), BotVerdict::Proceed);
    }

    #[test]
    fn reconcile_reply_against_out_data_phase_is_phase_error() {
        // A data-IN reply against a declared OUT phase: direction mismatch, stall OUT.
        let action = ScsiAction::Reply {
            data: [0u8; 36],
            len: 36,
        };
        assert_eq!(
            reconcile_cbw(false, 36, &action),
            reject(CSW_PHASE_ERROR, Some(false))
        );
        // The agreeing IN direction proceeds; case 7 non-block is clamped, not rejected.
        assert_eq!(reconcile_cbw(true, 16, &action), BotVerdict::Proceed);
    }

    #[test]
    fn reconcile_read_against_out_data_phase_is_phase_error() {
        // Case 10 (Ho <> Di): READ(10) with the CBW direction bit clear; stall OUT.
        let action = ScsiAction::ReadSectors { lba: 0, count: 1 };
        assert_eq!(
            reconcile_cbw(false, 512, &action),
            reject(CSW_PHASE_ERROR, Some(false))
        );
        assert_eq!(reconcile_cbw(true, 512, &action), BotVerdict::Proceed);
    }

    #[test]
    fn reconcile_write_against_in_data_phase_is_phase_error() {
        // Case 8 (Hi <> Do): WRITE(10) with the CBW direction bit set; stall IN.
        let action = ScsiAction::WriteSectors { lba: 0, count: 1 };
        assert_eq!(
            reconcile_cbw(true, 512, &action),
            reject(CSW_PHASE_ERROR, Some(true))
        );
        assert_eq!(reconcile_cbw(false, 512, &action), BotVerdict::Proceed);
    }

    #[test]
    fn reconcile_block_length_mismatch_is_failed() {
        // Cases 3/7/11/13 block: a declared length disagreeing with the sector count fails.
        let read = ScsiAction::ReadSectors { lba: 0, count: 1 };
        assert_eq!(
            reconcile_cbw(true, 256, &read),
            reject(CSW_FAILED, Some(true))
        );
        // With no declared data phase: still a length mismatch, but nothing to stall.
        assert_eq!(reconcile_cbw(false, 0, &read), reject(CSW_FAILED, None));
        let write = ScsiAction::WriteSectors { lba: 0, count: 2 };
        assert_eq!(
            reconcile_cbw(false, 512, &write),
            reject(CSW_FAILED, Some(false))
        );
    }

    #[test]
    fn reconcile_ok_with_data_phase_is_failed() {
        // Cases 4/9: a no-data command with a declared data phase fails and stalls that direction.
        assert_eq!(
            reconcile_cbw(true, 36, &ScsiAction::Ok),
            reject(CSW_FAILED, Some(true))
        );
        assert_eq!(
            reconcile_cbw(false, 512, &ScsiAction::Ok),
            reject(CSW_FAILED, Some(false))
        );
        // Case 1 (Hn = Dn) proceeds.
        assert_eq!(
            reconcile_cbw(false, 0, &ScsiAction::Ok),
            BotVerdict::Proceed
        );
    }

    #[test]
    fn reconcile_matching_and_failing_actions_proceed() {
        // Matching block transfers proceed...
        let read = ScsiAction::ReadSectors { lba: 0, count: 2 };
        assert_eq!(reconcile_cbw(true, 1024, &read), BotVerdict::Proceed);
        let write = ScsiAction::WriteSectors { lba: 0, count: 2 };
        assert_eq!(reconcile_cbw(false, 1024, &write), BotVerdict::Proceed);
        // ...and a dispatch-level failure is not second-guessed (the Fail arm handles it).
        let fail = ScsiAction::Fail(SENSE_NO_MEDIUM);
        assert_eq!(reconcile_cbw(true, 512, &fail), BotVerdict::Proceed);
        assert_eq!(reconcile_cbw(false, 0, &fail), BotVerdict::Proceed);
    }

    #[test]
    fn read_format_capacities_reports_formatted_media() {
        // One formatted-media descriptor: the block COUNT, then the 24-bit block length.
        let mut rest = [0u8; 9];
        rest[6..8].copy_from_slice(&0x00FCu16.to_be_bytes()); // allocation length at cb[7..9]
        let cb = cb_with(OP_READ_FORMAT_CAPACITIES, &rest);
        match dispatch(&cb, true, SENSE_NONE) {
            ScsiAction::Reply { data, len } => {
                assert_eq!(len, 12);
                assert_eq!(&data[0..4], &[0, 0, 0, 8], "8-byte capacity list");
                assert_eq!(&data[4..8], &VFS_TOTAL_SECTORS.to_be_bytes());
                assert_eq!(data[8], 0x02, "formatted media");
                assert_eq!(
                    &data[9..12],
                    &(MSC_SECTOR_SIZE as u32).to_be_bytes()[1..],
                    "24-bit block length"
                );
            }
            other => panic!("expected Reply, got {other:?}"),
        }
    }

    #[test]
    fn sync_cache_is_ok() {
        // Nothing is cached, so both widths report success rather than an illegal request.
        assert_eq!(
            dispatch(&cb_with(OP_SYNCHRONIZE_CACHE_10, &[]), true, SENSE_NONE),
            ScsiAction::Ok
        );
        assert_eq!(
            dispatch(&cb_with(OP_SYNCHRONIZE_CACHE_16, &[]), true, SENSE_NONE),
            ScsiAction::Ok
        );
    }
}
