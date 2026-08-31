//! Pre-dispatch bounds guard: answers a truncated request or a response overrun with `0xFF`.

/// Result of guarding one report: dispatch it, or reply with the invalid-command byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Verdict {
    /// The request is safe for the engine to consume.
    Dispatch,
    /// Reply `0xFF` (invalid command) without dispatching.
    Reject,
}

/// Validate `report` (command id + payload) against `resp_capacity`, the response buffer length.
/// Unknown and vendor ids dispatch untouched; the engine answers those without unchecked reads.
pub(crate) fn check(report: &[u8], resp_capacity: usize) -> Verdict {
    let Some(&cmd) = report.first() else {
        // Empty report: the engine's request parser answers it with no response.
        return Verdict::Dispatch;
    };
    let ok = match cmd {
        0x00 => report.len() >= 2,  // DAP_Info: info id
        0x01 => report.len() >= 3,  // DAP_HostStatus: type + status
        0x02 => report.len() >= 2,  // DAP_Connect: port
        0x03 | 0x07 | 0x0A => true, // Disconnect / TransferAbort / ResetTarget
        0x04 => report.len() >= 6,  // DAP_TransferConfigure: idle + 2 x u16
        0x05 => check_transfer(report, resp_capacity),
        0x06 => check_transfer_block(report, resp_capacity),
        0x08 => report.len() >= 6, // DAP_WriteABORT: index + word
        0x09 => report.len() >= 3, // DAP_Delay: u16
        0x10 => report.len() >= 7, // DAP_SWJ_Pins: out + mask + u32 wait
        0x11 => report.len() >= 5, // DAP_SWJ_Clock: u32
        0x12 => report.len() >= 2, // DAP_SWJ_Sequence: bit count (payload length-checked by the engine itself)
        0x13 => report.len() >= 2, // DAP_SWD_Configure: config byte
        0x14 => check_jtag_sequence(report, resp_capacity),
        0x15 => report.len() >= 2 && report.len() >= 2 + usize::from(report[1]), // JTAG_Configure
        0x16 => report.len() >= 2,               // DAP_JTAG_IDCODE: index
        0x17 | 0x18 | 0x1A => report.len() >= 2, // SWO_Transport / Mode / Control
        0x19 => report.len() >= 5,               // DAP_SWO_Baudrate: u32
        // SWO_Status / ExtendedStatus (0x1E) read no request bytes at the pinned dap-rs rev.
        0x1B | 0x1E => true,
        0x1C => report.len() >= 3, // DAP_SWO_Data: u16 count (response side clamped by the engine)
        0x1D => check_swd_sequence(report),
        // Vendor ids, atomic-command stubs and unknown ids are answered without unchecked reads.
        _ => true,
    };
    if ok {
        Verdict::Dispatch
    } else {
        Verdict::Reject
    }
}

/// `DAP_Transfer` (0x05): 1 request byte per entry plus a u32 for a write/match value or mask.
/// Response: a 3-byte fixed part, +4 per read value, +4 per requested timestamp.
fn check_transfer(report: &[u8], resp_capacity: usize) -> bool {
    if report.len() < 3 {
        return false;
    }
    let count = usize::from(report[2]);
    let mut off = 3usize;
    let mut resp = 3usize;
    for _ in 0..count {
        let Some(&req) = report.get(off) else {
            return false;
        };
        off += 1;
        let read = req & (1 << 1) != 0;
        let vmatch = req & (1 << 4) != 0;
        let mmask = req & (1 << 5) != 0;
        let timestamp = req & (1 << 7) != 0;
        if read {
            if vmatch {
                off += 4; // match value
            } else {
                resp += 4; // read data
            }
            if timestamp {
                resp += 4;
            }
        } else {
            off += 4; // write value or new match mask
            if !mmask && timestamp {
                resp += 4;
            }
        }
        if off > report.len() {
            return false;
        }
    }
    resp <= resp_capacity
}

/// `DAP_TransferBlock` (0x06): fixed 5-byte header; a write needs `count` u32 request values,
/// a read produces `count` u32 response values after the 4-byte fixed response part.
fn check_transfer_block(report: &[u8], resp_capacity: usize) -> bool {
    if report.len() < 5 {
        return false;
    }
    let count = usize::from(u16::from_le_bytes([report[2], report[3]]));
    let read = report[4] & (1 << 1) != 0;
    if read {
        4 + 4 * count <= resp_capacity
    } else {
        report.len() >= 5 + 4 * count
    }
}

/// `DAP_JTAG_Sequence` (0x14): per sequence, 1 info byte plus `ceil(cycles/8)` TDI bytes (0 = 64).
/// A capture sequence produces the same byte count after the 2-byte fixed response part.
fn check_jtag_sequence(report: &[u8], resp_capacity: usize) -> bool {
    if report.len() < 2 {
        return false;
    }
    let count = usize::from(report[1]);
    let mut off = 2usize;
    let mut resp = 2usize;
    for _ in 0..count {
        let Some(&info) = report.get(off) else {
            return false;
        };
        let cycles = match info & 0x3F {
            0 => 64,
            n => usize::from(n),
        };
        let nbytes = cycles.div_ceil(8);
        off += 1 + nbytes;
        if off > report.len() {
            return false;
        }
        if info & (1 << 7) != 0 {
            resp += nbytes;
        }
    }
    resp <= resp_capacity
}

/// `DAP_SWD_Sequence` (0x1D): 1 info byte per sequence; an output carries `ceil(cycles/8)` (0 = 64).
/// The engine bounds-checks the data and response sides, so only the info positions matter here.
fn check_swd_sequence(report: &[u8]) -> bool {
    if report.len() < 2 {
        return false;
    }
    let count = usize::from(report[1]);
    let mut off = 2usize;
    for _ in 0..count {
        let Some(&info) = report.get(off) else {
            return false;
        };
        off += 1;
        let output = info & (1 << 7) == 0;
        if output {
            let cycles = match info & 0x3F {
                0 => 64,
                n => usize::from(n),
            };
            off += cycles.div_ceil(8);
            if off > report.len() {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAP: usize = 64;

    fn dispatches(report: &[u8]) -> bool {
        check(report, CAP) == Verdict::Dispatch
    }

    #[test]
    fn empty_report_dispatches() {
        assert!(dispatches(&[]));
    }

    #[test]
    fn id_only_commands_dispatch() {
        for cmd in [0x03u8, 0x07, 0x0A, 0x1B, 0x1E] {
            assert!(dispatches(&[cmd]));
        }
    }

    #[test]
    fn truncated_fixed_commands_reject() {
        assert!(!dispatches(&[0x11])); // SWJ_Clock without the u32
        assert!(!dispatches(&[0x11, 1, 2, 3])); // one byte short
        assert!(dispatches(&[0x11, 1, 2, 3, 4]));
        assert!(!dispatches(&[0x19])); // SWO_Baudrate without the u32
        assert!(!dispatches(&[0x00])); // Info without the id byte
        assert!(!dispatches(&[0x10, 0, 0])); // SWJ_Pins short of the wait u32
        assert!(!dispatches(&[0x08, 0, 1, 2, 3])); // WriteABORT short of the word
        assert!(!dispatches(&[0x04, 0, 0, 0, 0])); // TransferConfigure one short
        assert!(dispatches(&[0x04, 0, 0, 0, 0, 0]));
        assert!(!dispatches(&[0x01])); // HostStatus without type+status
        assert!(!dispatches(&[0x01, 0])); // HostStatus one byte short
        assert!(dispatches(&[0x01, 0, 0]));
        assert!(!dispatches(&[0x02])); // Connect without the port byte
        assert!(!dispatches(&[0x09])); // Delay without the u16
        assert!(!dispatches(&[0x09, 0])); // Delay one byte short
        assert!(dispatches(&[0x09, 0, 0]));
        assert!(!dispatches(&[0x13])); // SWD_Configure without the config byte
        assert!(!dispatches(&[0x16])); // JTAG_IDCODE without the index byte
        assert!(!dispatches(&[0x17])); // SWO_Transport without the transport byte
        assert!(!dispatches(&[0x18])); // SWO_Mode without the mode byte
        assert!(!dispatches(&[0x1A])); // SWO_Control without the control byte
        assert!(!dispatches(&[0x1C])); // SWO_Data without the u16 count
        assert!(!dispatches(&[0x1C, 1])); // SWO_Data one byte short of the u16 count
        assert!(dispatches(&[0x1C, 0, 0]));
    }

    #[test]
    fn jtag_configure_needs_count_bytes() {
        assert!(!dispatches(&[0x15]));
        assert!(!dispatches(&[0x15, 2, 4])); // count 2, one IR length
        assert!(dispatches(&[0x15, 2, 4, 5]));
    }

    #[test]
    fn transfer_reads_clamp_to_response_capacity() {
        // 15 plain reads: 3 + 60 = 63 fits a 64-byte response.
        let mut ok = [0u8; 18];
        ok[0] = 0x05;
        ok[2] = 15;
        for e in ok[3..18].iter_mut() {
            *e = 1 << 1;
        }
        assert!(dispatches(&ok));
        // 16 plain reads: 3 + 64 = 67 does not.
        let mut over = [0u8; 19];
        over[0] = 0x05;
        over[2] = 16;
        for e in over[3..19].iter_mut() {
            *e = 1 << 1;
        }
        assert!(!dispatches(&over));
        // 16 timestamped reads reject even harder; 7 fit (3 + 56).
        let mut ts = [0u8; 10];
        ts[0] = 0x05;
        ts[2] = 7;
        for e in ts[3..10].iter_mut() {
            *e = (1 << 1) | (1 << 7);
        }
        assert!(dispatches(&ts));
    }

    #[test]
    fn transfer_truncated_at_count_byte() {
        assert!(!dispatches(&[0x05])); // missing DAP index byte
        assert!(!dispatches(&[0x05, 0])); // missing count byte
        assert!(dispatches(&[0x05, 0, 0])); // count 0 dispatches with nothing to walk
    }

    #[test]
    fn transfer_write_payloads_must_be_present() {
        // One write with its word present.
        assert!(dispatches(&[0x05, 0, 1, 0x00, 1, 2, 3, 4]));
        // One write missing its word.
        assert!(!dispatches(&[0x05, 0, 1, 0x00]));
        // Value-match read consumes a word from the request, produces none.
        assert!(dispatches(&[0x05, 0, 1, (1 << 1) | (1 << 4), 1, 2, 3, 4]));
        assert!(!dispatches(&[0x05, 0, 1, (1 << 1) | (1 << 4), 1, 2]));
        // Match-mask write consumes a word, no wire write, no timestamp bytes.
        assert!(dispatches(&[0x05, 0, 1, 1 << 5, 1, 2, 3, 4]));
        // Writes do not count against the response budget: many writes are fine.
        let mut wr = [0u8; 3 + 30 * 5];
        wr[0] = 0x05;
        wr[2] = 30;
        for c in wr[3..].chunks_mut(5) {
            c[0] = 0x00;
        }
        assert!(dispatches(&wr));
    }

    #[test]
    fn transfer_block_read_count_clamps() {
        // Read block: 15 words fit (4 + 60), 16 do not (4 + 64 > 64).
        assert!(dispatches(&[0x06, 0, 15, 0, 1 << 1]));
        assert!(!dispatches(&[0x06, 0, 16, 0, 1 << 1]));
        // A 64-read block: valid length, rejected on the response budget.
        assert!(!dispatches(&[0x06, 0, 64, 0, 1 << 1]));
    }

    #[test]
    fn transfer_block_write_needs_words() {
        let mut wr = [0u8; 5 + 8];
        wr[0] = 0x06;
        wr[2] = 2; // two write words = 8 bytes
        assert!(dispatches(&wr));
        let mut short = [0u8; 5 + 7];
        short[0] = 0x06;
        short[2] = 2;
        assert!(!dispatches(&short));
        assert!(!dispatches(&[0x06, 0])); // truncated header
    }

    #[test]
    fn jtag_sequence_walks_payload_and_capture_budget() {
        // One 8-cycle output sequence: info + 1 TDI byte.
        assert!(dispatches(&[0x14, 1, 8, 0xAA]));
        assert!(!dispatches(&[0x14, 1, 8]));
        // TDI byte missing above; cycles = 0 means 64 -> 8 TDI bytes below.
        assert!(!dispatches(&[0x14, 1, 0x00, 1, 2, 3, 4, 5, 6, 7]));
        assert!(dispatches(&[0x14, 1, 0x00, 1, 2, 3, 4, 5, 6, 7, 8]));
        // Capture sequences count against the response budget: 8 x 64 cycles is 2 + 64 > 64.
        let mut cap = [0u8; 2 + 8 * 9];
        cap[0] = 0x14;
        cap[1] = 8;
        for c in cap[2..].chunks_mut(9) {
            c[0] = 0x80; // capture, 64 cycles
        }
        assert!(!dispatches(&cap));
        // 7 of them fit (2 + 56).
        let mut fit = [0u8; 2 + 7 * 9];
        fit[0] = 0x14;
        fit[1] = 7;
        for c in fit[2..].chunks_mut(9) {
            c[0] = 0x80;
        }
        assert!(dispatches(&fit));
    }

    #[test]
    fn jtag_sequence_truncated_at_count_byte() {
        assert!(!dispatches(&[0x14])); // missing sequence count
        assert!(dispatches(&[0x14, 0])); // count 0 dispatches with nothing to walk
    }

    #[test]
    fn swd_sequence_info_bytes_must_exist() {
        // One 8-cycle output sequence with its data byte.
        assert!(dispatches(&[0x1D, 1, 8, 0xAA]));
        // Output data missing.
        assert!(!dispatches(&[0x1D, 1, 8]));
        // Input sequence carries no request data.
        assert!(dispatches(&[0x1D, 1, 0x88]));
        // Second info byte missing.
        assert!(!dispatches(&[0x1D, 2, 0x88]));
    }

    #[test]
    fn swd_sequence_truncated_at_count_byte() {
        assert!(!dispatches(&[0x1D])); // missing sequence count
        assert!(dispatches(&[0x1D, 0])); // count 0 dispatches with nothing to walk
    }

    #[test]
    fn vendor_and_unknown_ids_dispatch() {
        assert!(dispatches(&[0x80]));
        assert!(dispatches(&[0x9F, 1, 2, 3]));
        assert!(dispatches(&[0x7F]));
        assert!(dispatches(&[0x42]));
    }
}
