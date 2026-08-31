//! `DAP_SWO_*` command encode/decode: pure byte-buffer transforms, independent of a probe.

use dap_bench::dap::DecodeError;

/// `DAP_SWO_Transport`: select where captured trace is handed to the host.
pub const DAP_SWO_TRANSPORT: u8 = 0x17;
/// `DAP_SWO_Mode`: select the trace line encoding.
pub const DAP_SWO_MODE: u8 = 0x18;
/// `DAP_SWO_Baudrate`: request a capture baud; the probe answers with the realized one.
pub const DAP_SWO_BAUDRATE: u8 = 0x19;
/// `DAP_SWO_Control`: start or stop capture.
pub const DAP_SWO_CONTROL: u8 = 0x1A;
/// `DAP_SWO_Status`: capture-active flag plus buffered byte count.
pub const DAP_SWO_STATUS: u8 = 0x1B;
/// `DAP_SWO_Data`: drain up to N buffered trace bytes.
pub const DAP_SWO_DATA: u8 = 0x1C;

/// `DAP_Info` id for the probe's SWO trace buffer size (4 bytes LE).
pub const INFO_SWO_TRACE_BUFFER_SIZE: u8 = 0xFD;
/// `DAP_Info` id for the capability bitfield.
pub const INFO_CAPABILITIES: u8 = 0xF0;
/// Capability bit 2: SWO UART capture.
pub const CAP_SWO_UART: u8 = 0x04;

/// Generic single-byte OK status.
const DAP_OK: u8 = 0x00;

/// Bytes of framing ahead of the payload in a `DAP_SWO_Data` response (command, status, count).
const DATA_HEADER_LEN: usize = 4;

/// Where the probe hands captured trace to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Transport {
    /// No transport selected.
    None = 0,
    /// Polled through `DAP_SWO_Data` on the command endpoint.
    DapCommand = 1,
    /// Pushed on a dedicated USB trace endpoint.
    UsbEndpoint = 2,
}

/// Trace line encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    /// Capture disabled.
    Off = 0,
    /// Asynchronous NRZ (UART) SWO.
    Uart = 1,
    /// Manchester-encoded SWO.
    Manchester = 2,
}

/// Capture run state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Control {
    /// Stop capturing.
    Stop = 0,
    /// Start capturing.
    Start = 1,
}

/// A decoded `DAP_SWO_Status` response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// Capture is running.
    pub active: bool,
    /// Bytes buffered on the probe right now.
    pub bytes_available: u32,
}

/// A decoded `DAP_SWO_Data` response.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Data {
    /// Capture was running when the probe served this request.
    pub active: bool,
    /// The trace bytes drained, in capture order.
    pub bytes: Vec<u8>,
}

/// Checks that `resp` starts with `command` and is at least `min_len` long, returning the rest.
fn expect_command(resp: &[u8], command: u8, min_len: usize) -> Result<&[u8], DecodeError> {
    let (&found, rest) = resp.split_first().ok_or(DecodeError::Truncated)?;
    if found != command {
        return Err(DecodeError::UnexpectedCommand {
            expected: command,
            found,
        });
    }
    if resp.len() < min_len {
        return Err(DecodeError::Truncated);
    }
    Ok(rest)
}

/// Decodes a `[command, status]` response, rejecting any status other than OK.
fn decode_ok(resp: &[u8], command: u8) -> Result<(), DecodeError> {
    let rest = expect_command(resp, command, 2)?;
    match rest[0] {
        DAP_OK => Ok(()),
        other => Err(DecodeError::InvalidStatus(other)),
    }
}

/// Encodes `DAP_SWO_Transport`: `[0x17, transport]`.
pub fn encode_transport(transport: Transport) -> Vec<u8> {
    vec![DAP_SWO_TRANSPORT, transport as u8]
}

/// Decodes a `DAP_SWO_Transport` response.
pub fn decode_transport(resp: &[u8]) -> Result<(), DecodeError> {
    decode_ok(resp, DAP_SWO_TRANSPORT)
}

/// Encodes `DAP_SWO_Mode`: `[0x18, mode]`.
pub fn encode_mode(mode: Mode) -> Vec<u8> {
    vec![DAP_SWO_MODE, mode as u8]
}

/// Decodes a `DAP_SWO_Mode` response.
pub fn decode_mode(resp: &[u8]) -> Result<(), DecodeError> {
    decode_ok(resp, DAP_SWO_MODE)
}

/// Encodes `DAP_SWO_Baudrate`: `[0x19, baud(4 LE)]`.
pub fn encode_baudrate(baud: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(5);
    buf.push(DAP_SWO_BAUDRATE);
    buf.extend_from_slice(&baud.to_le_bytes());
    buf
}

/// Decodes a `DAP_SWO_Baudrate` response: the realized baud, `0` meaning the request was refused.
pub fn decode_baudrate(resp: &[u8]) -> Result<u32, DecodeError> {
    let rest = expect_command(resp, DAP_SWO_BAUDRATE, 5)?;
    Ok(u32::from_le_bytes(rest[0..4].try_into().unwrap()))
}

/// Encodes `DAP_SWO_Control`: `[0x1A, control]`.
pub fn encode_control(control: Control) -> Vec<u8> {
    vec![DAP_SWO_CONTROL, control as u8]
}

/// Decodes a `DAP_SWO_Control` response.
pub fn decode_control(resp: &[u8]) -> Result<(), DecodeError> {
    decode_ok(resp, DAP_SWO_CONTROL)
}

/// Encodes `DAP_SWO_Status`: `[0x1B]`.
pub fn encode_status() -> Vec<u8> {
    vec![DAP_SWO_STATUS]
}

/// Decodes a `DAP_SWO_Status` response: `[0x1B, status, count(4 LE)]`.
pub fn decode_status(resp: &[u8]) -> Result<Status, DecodeError> {
    let rest = expect_command(resp, DAP_SWO_STATUS, 6)?;
    Ok(Status {
        active: rest[0] & 0x01 != 0,
        bytes_available: u32::from_le_bytes(rest[1..5].try_into().unwrap()),
    })
}

/// Encodes `DAP_SWO_Data`: `[0x1C, count(2 LE)]`.
pub fn encode_data(count: u16) -> Vec<u8> {
    let mut buf = Vec::with_capacity(3);
    buf.push(DAP_SWO_DATA);
    buf.extend_from_slice(&count.to_le_bytes());
    buf
}

/// Decodes a `DAP_SWO_Data` response: `[0x1C, status, count(2 LE), data...]`.
pub fn decode_data(resp: &[u8]) -> Result<Data, DecodeError> {
    let rest = expect_command(resp, DAP_SWO_DATA, DATA_HEADER_LEN)?;
    let count = usize::from(u16::from_le_bytes(rest[1..3].try_into().unwrap()));
    let bytes = rest.get(3..3 + count).ok_or(DecodeError::Truncated)?;
    Ok(Data {
        active: rest[0] & 0x01 != 0,
        bytes: bytes.to_vec(),
    })
}

/// Trace bytes a single `DAP_SWO_Data` exchange can return within `packet_size`.
pub const fn data_capacity(packet_size: u16) -> u16 {
    packet_size.saturating_sub(DATA_HEADER_LEN as u16)
}

/// Response length of a `DAP_SWO_Data` request for `count` bytes.
pub const fn data_response_len(count: u16) -> usize {
    DATA_HEADER_LEN + count as usize
}

/// Encodes a `DAP_Info` request: `[0x00, info_id]`.
pub fn encode_info(info_id: u8) -> Vec<u8> {
    vec![0x00, info_id]
}

/// Decodes a `DAP_Info` response carrying a 4-byte LE word (the SWO trace buffer size).
pub fn decode_info_u32(resp: &[u8]) -> Result<u32, DecodeError> {
    let rest = expect_command(resp, 0x00, 6)?;
    if usize::from(rest[0]) < 4 {
        return Err(DecodeError::Truncated);
    }
    Ok(u32::from_le_bytes(rest[1..5].try_into().unwrap()))
}

/// Decodes a `DAP_Info` response carrying a single capability byte.
pub fn decode_info_u8(resp: &[u8]) -> Result<u8, DecodeError> {
    let rest = expect_command(resp, 0x00, 3)?;
    if rest[0] == 0 {
        return Err(DecodeError::Truncated);
    }
    Ok(rest[1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transport_mode_control_round_trip() {
        assert_eq!(encode_transport(Transport::DapCommand), [0x17, 0x01]);
        assert_eq!(encode_mode(Mode::Uart), [0x18, 0x01]);
        assert_eq!(encode_control(Control::Start), [0x1A, 0x01]);
        assert!(decode_transport(&[0x17, 0x00]).is_ok());
        assert!(decode_mode(&[0x18, 0x00]).is_ok());
        assert!(decode_control(&[0x1A, 0x00]).is_ok());
    }

    #[test]
    fn a_refused_command_decodes_as_an_invalid_status() {
        assert_eq!(
            decode_mode(&[0x18, 0xFF]),
            Err(DecodeError::InvalidStatus(0xFF))
        );
        assert_eq!(
            decode_control(&[0x1B, 0x00]),
            Err(DecodeError::UnexpectedCommand {
                expected: 0x1A,
                found: 0x1B
            })
        );
        assert_eq!(decode_transport(&[0x17]), Err(DecodeError::Truncated));
    }

    #[test]
    fn baudrate_reports_the_realized_rate() {
        assert_eq!(encode_baudrate(2_000_000), [0x19, 0x80, 0x84, 0x1E, 0x00]);
        let resp = [0x19, 0x40, 0x42, 0x0F, 0x00];
        assert_eq!(decode_baudrate(&resp).unwrap(), 1_000_000);
        // A probe that cannot realize the rate answers zero rather than an error status.
        assert_eq!(decode_baudrate(&[0x19, 0, 0, 0, 0]).unwrap(), 0);
        assert_eq!(
            decode_baudrate(&[0x19, 0, 0, 0]),
            Err(DecodeError::Truncated)
        );
    }

    #[test]
    fn status_splits_the_active_bit_from_the_count() {
        let resp = [0x1B, 0xC1, 0x10, 0x00, 0x00, 0x00];
        let status = decode_status(&resp).unwrap();
        assert_eq!(
            status,
            Status {
                active: true,
                bytes_available: 16,
            }
        );
        let idle = decode_status(&[0x1B, 0x00, 0, 0, 0, 0]).unwrap();
        assert!(!idle.active);
        assert_eq!(
            decode_status(&[0x1B, 0x01, 0, 0]),
            Err(DecodeError::Truncated)
        );
    }

    #[test]
    fn data_carries_a_counted_payload() {
        assert_eq!(encode_data(508), [0x1C, 0xFC, 0x01]);
        let resp = [0x1C, 0x01, 0x03, 0x00, 0xAA, 0xBB, 0xCC];
        let data = decode_data(&resp).unwrap();
        assert!(data.active);
        assert_eq!(data.bytes, vec![0xAA, 0xBB, 0xCC]);
        // An empty drain is the common polling answer, not an error.
        assert!(decode_data(&[0x1C, 0x01, 0x00, 0x00])
            .unwrap()
            .bytes
            .is_empty());
        // A count longer than the buffer is a truncated response, never a silent short read.
        assert_eq!(
            decode_data(&[0x1C, 0x01, 0x04, 0x00, 0xAA]),
            Err(DecodeError::Truncated)
        );
    }

    #[test]
    fn data_capacity_leaves_room_for_the_framing() {
        assert_eq!(data_capacity(512), 508);
        assert_eq!(data_response_len(508), 512);
        assert_eq!(data_capacity(2), 0);
    }

    #[test]
    fn info_decoders_read_the_length_prefixed_payload() {
        assert_eq!(encode_info(INFO_SWO_TRACE_BUFFER_SIZE), [0x00, 0xFD]);
        let resp = [0x00, 0x04, 0xFF, 0x03, 0x00, 0x00];
        assert_eq!(decode_info_u32(&resp).unwrap(), 1023);
        assert_eq!(decode_info_u8(&[0x00, 0x01, 0x17]).unwrap(), 0x17);
        assert_eq!(
            decode_info_u8(&[0x00, 0x01, 0x17]).unwrap() & CAP_SWO_UART,
            CAP_SWO_UART
        );
        assert_eq!(
            decode_info_u32(&[0x00, 0x02, 0x01, 0x02]),
            Err(DecodeError::Truncated)
        );
    }
}
