//! The DPACC/APACC scan over `Jtag::sequence`, and the ACK decode every JTAG engine shares.

use dap_rs::jtag::{Jtag, SequenceInfo, TransferResult};
use dap_rs::swd::RnW;
use daplink_core::jtag_dp::{ACK_WAIT, STATUS_OK, STATUS_WAIT};
use daplink_core::JtagDp;

/// A completed scan's result for TAP `index`; FAULT and no ACK both stop dap-rs without a retry.
pub fn jtag_result(dp: &JtagDp, index: u8, ack: u32, data: u32) -> TransferResult {
    match dp.status(index, ack) {
        STATUS_OK => TransferResult::Ok(data),
        STATUS_WAIT => TransferResult::Wait,
        _ => TransferResult::Fault,
    }
}

/// dap-rs's per-chunk scan, leaving early on WAIT only; returns the ACK and the captured word.
pub(crate) fn chunked_scan<DEPS, J: Jtag<DEPS>>(
    jtag: &mut J,
    r_nw: RnW,
    a2a3: u8,
    idle_cycles: u8,
    data: u32,
) -> (u32, u32) {
    jtag.tms_sequence(&[true, false, false]);
    let config = jtag.config();
    let before = u16::from(config.index);
    let after = u16::from(config.device_count).saturating_sub(before + 1);
    repeated(jtag, before, false);
    let ack = register(jtag, (u32::from(a2a3) << 1) | r_nw as u32, 3, false);
    let captured = if ack != ACK_WAIT {
        let captured = register(jtag, data, 32, after == 0);
        if after > 0 {
            repeated(jtag, after - 1, false);
            repeated(jtag, 1, true);
        }
        jtag.tms_sequence(&[true, false]);
        captured
    } else {
        jtag.tms_sequence(&[true, true, false]);
        0
    };
    repeated(jtag, u16::from(idle_cycles), false);
    (ack, captured)
}

/// `bits` in Shift-DR, eight per call and the last one alone, TMS high on it if `exit`.
fn register<DEPS, J: Jtag<DEPS>>(jtag: &mut J, mut data: u32, bits: u8, exit: bool) -> u32 {
    let mut captured = 0;
    let mut left = bits;
    while left > 1 {
        let n = (left - 1).min(8);
        captured >>= n;
        captured |= u32::from(chunk(jtag, data as u8, n, false)) << (bits - n);
        data >>= n;
        left -= n;
    }
    captured >>= 1;
    captured | u32::from(chunk(jtag, data as u8, 1, exit)) << (bits - 1)
}

/// `bits` of TDI high, eight per call.
fn repeated<DEPS, J: Jtag<DEPS>>(jtag: &mut J, mut bits: u16, tms: bool) {
    while bits > 0 {
        let n = bits.min(8);
        bits -= n;
        chunk(jtag, 0xFF, n as u8, tms);
    }
}

fn chunk<DEPS, J: Jtag<DEPS>>(jtag: &mut J, tdi: u8, n_bits: u8, tms: bool) -> u8 {
    let mut tdo = 0;
    let info = SequenceInfo {
        n_bits,
        capture: true,
        tms,
    };
    jtag.sequence(info, &[tdi], core::slice::from_mut(&mut tdo));
    tdo
}
