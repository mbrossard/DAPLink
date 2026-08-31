//! The vendor-flash case: stream `--dut-uf2` through the probe's vendor flash commands.

use crate::dap::RawDap;
use crate::uf2::{first_block, verify};
use crate::{Ctx, Failure};

/// `MSD_Open`: open the image stream. `[id, stream_type]` -> `[id, error_t]`.
const MSD_OPEN: u8 = 0x8A;
/// `MSD_Close`: finish the image stream. `[id]` -> `[id, error_t]`.
const MSD_CLOSE: u8 = 0x8B;
/// `MSD_Write`: one chunk of the image stream. `[id, len, data...]` -> `[id, error_t]`.
const MSD_WRITE: u8 = 0x8C;
/// `SelectEraseMode`: 0 selects chip erase, nonzero selects sector erase. `[id, mode]` ->
/// `[id, DAP_OK]`.
const SELECT_ERASE_MODE: u8 = 0x8D;
/// uf2 stream type.
const STREAM_TYPE_UF2: u8 = 2;
/// Shared success status for every reply in this set (`ERROR_SUCCESS` and `DAP_OK` are both 0).
const STATUS_OK: u8 = 0x00;

/// Reflash `--dut-uf2` over raw bulk and verify the readback; skips when the firmware answers
/// the first vendor command as unimplemented.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    let uf2_path = cx
        .dut_uf2
        .clone()
        .ok_or_else(|| Failure::Fatal("needs --dut-uf2".into()))?;
    let image =
        std::fs::read(&uf2_path).map_err(|e| Failure::Fatal(format!("read {uf2_path}: {e}")))?;
    let (addr, payload) = first_block(&image)
        .ok_or_else(|| Failure::Fatal(format!("{uf2_path} has no valid uf2 block")))?;

    // Raw bulk claims the USB interface, so the probe-rs session goes first.
    cx.session = None;
    let mut dap = crate::dap::open(&cx.selector).map_err(Failure::Fatal)?;
    let chunk = chunk_len(packet_size(&mut dap).map_err(Failure::Fatal)?);
    if chunk == 0 {
        return Err(Failure::Fatal(
            "packet size too small to carry any MSD_Write payload".into(),
        ));
    }

    // The first vendor command is the capability probe: no handler answers a bare 0xFF.
    let erase = dap
        .command(&[SELECT_ERASE_MODE, 0x01])
        .map_err(Failure::Fatal)?;
    if is_unsupported(&erase) {
        drop(dap);
        crate::connect::run(cx, full_run)?;
        return Err(Failure::Skip("vendor-flash: firmware".into()));
    }

    let flow = expect_ok(&erase, SELECT_ERASE_MODE, "SelectEraseMode")
        .and_then(|()| stream_image(&mut dap, chunk, &image));
    drop(dap);

    // Later cases depend on the fresh SWD session, not on this case's success.
    crate::connect::run(cx, full_run)?;
    flow.map_err(Failure::Failed)?;
    verify(cx, addr, payload)
}

/// `MSD_Open`, every chunk of `image` through `MSD_Write`, then `MSD_Close`, requiring status 0
/// throughout.
fn stream_image(dap: &mut RawDap, chunk: usize, image: &[u8]) -> Result<(), String> {
    let open = dap.command(&[MSD_OPEN, STREAM_TYPE_UF2])?;
    expect_ok(&open, MSD_OPEN, "MSD_Open")?;

    for piece in image.chunks(chunk) {
        let resp = dap.command(&write_report(piece))?;
        expect_ok(&resp, MSD_WRITE, "MSD_Write")?;
    }

    let close = dap.command(&[MSD_CLOSE])?;
    expect_ok(&close, MSD_CLOSE, "MSD_Close")
}

/// Frame one `MSD_Write` chunk: `[id, len, data...]`, `len` a single byte.
fn write_report(chunk: &[u8]) -> Vec<u8> {
    let mut report = Vec::with_capacity(2 + chunk.len());
    report.push(MSD_WRITE);
    report.push(chunk.len() as u8);
    report.extend_from_slice(chunk);
    report
}

/// `MSD_Write` payload per report: the packet less the id and length bytes, capped at 255.
fn chunk_len(packet_size: usize) -> usize {
    packet_size.saturating_sub(2).min(255)
}

/// The probe's `DAP_Info` packet size (`0xFF`), as a byte count.
fn packet_size(dap: &mut RawDap) -> Result<usize, String> {
    let size = dap.dap_info(0xFF)?;
    let lo = *size.first().ok_or("packet size reply too short")?;
    let hi = *size.get(1).ok_or("packet size reply too short")?;
    Ok(u16::from_le_bytes([lo, hi]) as usize)
}

/// The engine's fallthrough answer to an unrecognized command id: a bare `[0xFF]`.
fn is_unsupported(resp: &[u8]) -> bool {
    resp.first() == Some(&0xFF)
}

/// Every reply in the vendor flash set is `[id, status]`; require both.
fn expect_ok(resp: &[u8], id: u8, what: &str) -> Result<(), String> {
    match (resp.first(), resp.get(1)) {
        (Some(&i), Some(&s)) if i == id && s == STATUS_OK => Ok(()),
        _ => Err(format!("{what} reply {resp:02x?} is not [{id:#04x}, 0x00]")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chunk_len_reserves_the_id_and_length_bytes() {
        assert_eq!(chunk_len(64), 62);
        assert_eq!(chunk_len(3), 1);
    }

    #[test]
    fn chunk_len_floors_at_zero_for_undersized_packets() {
        assert_eq!(chunk_len(2), 0);
        assert_eq!(chunk_len(0), 0);
    }

    #[test]
    fn chunk_len_caps_at_255_regardless_of_a_larger_packet() {
        assert_eq!(chunk_len(300), 255);
    }

    #[test]
    fn write_report_frames_id_length_then_data() {
        let report = write_report(&[0xAA, 0xBB, 0xCC]);
        assert_eq!(report, vec![MSD_WRITE, 3, 0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn write_report_handles_an_empty_chunk() {
        assert_eq!(write_report(&[]), vec![MSD_WRITE, 0]);
    }

    #[test]
    fn is_unsupported_detects_the_unimplemented_echo() {
        assert!(is_unsupported(&[0xFF]));
        assert!(!is_unsupported(&[SELECT_ERASE_MODE, STATUS_OK]));
        assert!(!is_unsupported(&[]));
    }

    #[test]
    fn expect_ok_requires_the_echoed_id_and_a_clean_status() {
        assert!(expect_ok(&[MSD_OPEN, 0x00], MSD_OPEN, "MSD_Open").is_ok());
        assert!(expect_ok(&[MSD_OPEN, 0x02], MSD_OPEN, "MSD_Open").is_err());
        assert!(expect_ok(&[MSD_CLOSE, 0x00], MSD_OPEN, "MSD_Open").is_err());
        assert!(expect_ok(&[0xFF], MSD_OPEN, "MSD_Open").is_err());
    }
}
