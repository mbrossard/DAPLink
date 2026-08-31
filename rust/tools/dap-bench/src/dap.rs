//! CMSIS-DAP encode/decode for the SWD and JTAG block-transfer subset: pure byte transforms.

use std::fmt;

/// `DAP_Info`: query probe capabilities and identity strings.
pub const DAP_INFO: u8 = 0x00;
/// `DAP_Connect`: select the wire protocol (SWD or JTAG) and connect.
pub const DAP_CONNECT: u8 = 0x02;
/// `DAP_TransferConfigure`: set idle cycles and the WAIT/match retry counts.
pub const DAP_TRANSFER_CONFIGURE: u8 = 0x04;
/// `DAP_Transfer`: one or more packed register transfers against an auto-incrementing TAR.
pub const DAP_TRANSFER: u8 = 0x05;
/// `DAP_TransferBlock`: a single block transfer at the per-packet ceiling.
pub const DAP_TRANSFER_BLOCK: u8 = 0x06;
/// `DAP_WriteABORT`: write the ABORT register directly.
pub const DAP_WRITE_ABORT: u8 = 0x08;
/// `DAP_SWJ_Clock`: set the SWJ (SWD/JTAG) clock frequency.
pub const DAP_SWJ_CLOCK: u8 = 0x11;
/// `DAP_SWJ_Sequence`: shift an arbitrary bit sequence out on SWDIO/TMS (line reset, SWD/JTAG
/// select, idle cycles, TAP reset).
pub const DAP_SWJ_SEQUENCE: u8 = 0x12;
/// `DAP_JTAG_Configure`: describe the JTAG scan chain (per-TAP IR lengths).
pub const DAP_JTAG_CONFIGURE: u8 = 0x15;
/// `DAP_JTAG_IDCODE`: read a JTAG TAP's IDCODE through the configured chain.
pub const DAP_JTAG_IDCODE: u8 = 0x16;

/// Generic single-byte status a probe writes back for a response with no other payload.
const DAP_OK: u8 = 0x00;

/// Errors returned while decoding a CMSIS-DAP response buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The buffer was shorter than the layout requires.
    Truncated,
    /// The buffer's command-id byte did not match the command being decoded.
    UnexpectedCommand {
        /// The command id the decoder expected.
        expected: u8,
        /// The command id actually found in the buffer.
        found: u8,
    },
    /// A status/ack byte carried a value outside the set the layout defines.
    InvalidStatus(u8),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Truncated => write!(f, "response buffer too short"),
            DecodeError::UnexpectedCommand { expected, found } => {
                write!(
                    f,
                    "expected command id 0x{expected:02X}, found 0x{found:02X}"
                )
            }
            DecodeError::InvalidStatus(v) => write!(f, "invalid status byte 0x{v:02X}"),
        }
    }
}

impl std::error::Error for DecodeError {}

/// Check `resp` against `command` and `min_len`, returning the bytes after the command id.
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

// --- DAP_Info ----------------------------------------------------------------------------------

/// `DAP_Info` info-id selector (the second request byte).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InfoId {
    /// Vendor name string (0-length means "read it from the USB device descriptor instead").
    VendorId = 0x01,
    /// Product name string.
    ProductId = 0x02,
    /// Serial number string.
    SerialNumber = 0x03,
    /// Firmware version string.
    FirmwareVersion = 0x04,
    /// Maximum number of packets the probe can queue at once (1 byte), i.e. `DAP_PACKET_COUNT`.
    PacketCount = 0xFE,
    /// Maximum packet size in bytes (2 bytes LE), i.e. `DAP_PACKET_SIZE`.
    PacketSize = 0xFF,
}

/// Encodes a `DAP_Info` request for `info_id`: `[0x00, info_id]`.
pub fn encode_info(info_id: InfoId) -> Vec<u8> {
    vec![DAP_INFO, info_id as u8]
}

/// Extracts the length-prefixed payload out of a `DAP_Info` response: `[0x00, len, data...]`.
fn info_payload(resp: &[u8]) -> Result<&[u8], DecodeError> {
    let rest = expect_command(resp, DAP_INFO, 2)?;
    let len = rest[0] as usize;
    rest.get(1..1 + len).ok_or(DecodeError::Truncated)
}

/// Decode a length-prefixed ASCII `DAP_Info` string; the spec's trailing NUL is stripped.
pub fn decode_info_string(resp: &[u8]) -> Result<String, DecodeError> {
    let mut data = info_payload(resp)?;
    if data.last() == Some(&0) {
        data = &data[..data.len() - 1];
    }
    Ok(data.iter().map(|&b| b as char).collect())
}

/// Decodes a `DAP_Info(PacketCount)` response: one byte, `DAP_PACKET_COUNT`.
pub fn decode_info_packet_count(resp: &[u8]) -> Result<u8, DecodeError> {
    let data = info_payload(resp)?;
    data.first().copied().ok_or(DecodeError::Truncated)
}

/// Decodes a `DAP_Info(PacketSize)` response: two bytes LE, `DAP_PACKET_SIZE`.
pub fn decode_info_packet_size(resp: &[u8]) -> Result<u16, DecodeError> {
    let data = info_payload(resp)?;
    let bytes: [u8; 2] = data
        .get(0..2)
        .ok_or(DecodeError::Truncated)?
        .try_into()
        .unwrap();
    Ok(u16::from_le_bytes(bytes))
}

// --- DAP_Connect -------------------------------------------------------------------------------

/// The wire protocol requested by `DAP_Connect`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Port {
    /// Serial Wire Debug.
    Swd = 1,
    /// JTAG.
    Jtag = 2,
}

/// Encodes a `DAP_Connect` request: `[0x02, port]`.
pub fn encode_connect(port: Port) -> Vec<u8> {
    vec![DAP_CONNECT, port as u8]
}

/// Decode a `DAP_Connect` response: `[0x02, port]`, `port == 0` meaning the connect failed.
pub fn decode_connect(resp: &[u8]) -> Result<Option<Port>, DecodeError> {
    let rest = expect_command(resp, DAP_CONNECT, 2)?;
    match rest[0] {
        0 => Ok(None),
        1 => Ok(Some(Port::Swd)),
        2 => Ok(Some(Port::Jtag)),
        other => Err(DecodeError::InvalidStatus(other)),
    }
}

// --- DAP_TransferConfigure ---------------------------------------------------------------------

/// `DAP_TransferConfigure` parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransferConfigureRequest {
    /// Number of extra idle cycles after each transfer.
    pub idle_cycles: u8,
    /// Number of retries after a WAIT response.
    pub wait_retry: u16,
    /// Number of retries while a value-match transfer doesn't match.
    pub match_retry: u16,
}

/// Encodes a `DAP_TransferConfigure` request:
/// `[0x04, idle_cycles, wait_retry(2 LE), match_retry(2 LE)]`.
pub fn encode_transfer_configure(cfg: TransferConfigureRequest) -> Vec<u8> {
    let mut buf = Vec::with_capacity(6);
    buf.push(DAP_TRANSFER_CONFIGURE);
    buf.push(cfg.idle_cycles);
    buf.extend_from_slice(&cfg.wait_retry.to_le_bytes());
    buf.extend_from_slice(&cfg.match_retry.to_le_bytes());
    buf
}

/// Decodes a `DAP_TransferConfigure` response: `[0x04, status]`.
pub fn decode_transfer_configure(resp: &[u8]) -> Result<(), DecodeError> {
    let rest = expect_command(resp, DAP_TRANSFER_CONFIGURE, 2)?;
    if rest[0] == DAP_OK {
        Ok(())
    } else {
        Err(DecodeError::InvalidStatus(rest[0]))
    }
}

// --- DAP_Transfer / DAP_TransferBlock request-byte and ack modeling ----------------------------

const TRANSFER_APNDP: u8 = 1 << 0;
const TRANSFER_RNW: u8 = 1 << 1;
const TRANSFER_A_SHIFT: u8 = 2;
const TRANSFER_A_MASK: u8 = 0b11 << TRANSFER_A_SHIFT;

/// One `DAP_Transfer`/`DAP_TransferBlock` request byte: which register and which direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransferRequest {
    /// `true` selects an Access Port (AP) register, `false` a Debug Port (DP) register.
    pub ap: bool,
    /// `true` is a read, `false` is a write.
    pub read: bool,
    /// The two-bit register address, `A[3:2]` -- register byte offset `address * 4`.
    pub address: u8,
}

impl TransferRequest {
    /// A plain transfer request.
    pub const fn new(ap: bool, read: bool, address: u8) -> TransferRequest {
        TransferRequest { ap, read, address }
    }

    /// Packs this request into its wire byte.
    pub const fn to_byte(self) -> u8 {
        let mut b = 0u8;
        if self.ap {
            b |= TRANSFER_APNDP;
        }
        if self.read {
            b |= TRANSFER_RNW;
        }
        b |= (self.address << TRANSFER_A_SHIFT) & TRANSFER_A_MASK;
        b
    }
}

/// The SWD ACK phase, as reported in bits `[2:0]` of a transfer status byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ack {
    /// `OK`.
    Ok,
    /// `WAIT`: the target asked for a retry.
    Wait,
    /// `FAULT`.
    Fault,
    /// No response on the wire at all.
    NoAck,
    /// An ack value the spec does not define.
    Unknown(u8),
}

impl Ack {
    fn from_bits(bits: u8) -> Ack {
        match bits {
            0b001 => Ack::Ok,
            0b010 => Ack::Wait,
            0b100 => Ack::Fault,
            0b111 => Ack::NoAck,
            other => Ack::Unknown(other),
        }
    }
}

/// A `DAP_Transfer`/`DAP_TransferBlock` status byte: the ACK phase plus the protocol-error and
/// value-mismatch flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransferStatus {
    bits: u8,
}

impl TransferStatus {
    /// Wraps a raw status byte.
    pub const fn from_byte(bits: u8) -> TransferStatus {
        TransferStatus { bits }
    }

    /// The SWD ACK phase (bits `[2:0]`).
    pub fn ack(self) -> Ack {
        Ack::from_bits(self.bits & 0b111)
    }

    /// Bit 3: a protocol error (parity error on the data phase) occurred.
    pub const fn protocol_error(self) -> bool {
        self.bits & (1 << 3) != 0
    }

    /// True when the ACK phase was `OK` and no error flag is set.
    pub fn is_ok(self) -> bool {
        matches!(self.ack(), Ack::Ok) && !self.protocol_error()
    }
}

/// One transfer to append to a `DAP_Transfer` request: the request byte, plus the data word for
/// a write (`None` for a plain read).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransferItem {
    /// The packed request byte.
    pub request: TransferRequest,
    /// The write value that follows the request byte on the wire; `None` for a plain read.
    pub data: Option<u32>,
}

/// Encodes a `DAP_Transfer` request:
/// `[0x05, dap_index, transfer_count, (request_byte, [data(4 LE)])...]`.
pub fn encode_transfer(dap_index: u8, items: &[TransferItem]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(3 + items.len() * 5);
    buf.push(DAP_TRANSFER);
    buf.push(dap_index);
    buf.push(items.len() as u8);
    for item in items {
        buf.push(item.request.to_byte());
        if let Some(value) = item.data {
            buf.extend_from_slice(&value.to_le_bytes());
        }
    }
    buf
}

/// A decoded `DAP_Transfer` response.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransferResponse {
    /// Number of transfers actually executed (may be less than requested if one failed).
    pub transfer_count: u8,
    /// The status of the last-executed transfer.
    pub status: TransferStatus,
    /// Data words returned by successful reads, in request order.
    pub read_data: Vec<u32>,
}

/// Decodes a `DAP_Transfer` response: `[0x05, transfer_count, status, read_data(4 LE)...]`.
pub fn decode_transfer(resp: &[u8]) -> Result<TransferResponse, DecodeError> {
    let rest = expect_command(resp, DAP_TRANSFER, 3)?;
    let transfer_count = rest[0];
    let status = TransferStatus::from_byte(rest[1]);
    let data = &rest[2..];
    if data.len() % 4 != 0 {
        return Err(DecodeError::Truncated);
    }
    let read_data = data
        .chunks_exact(4)
        .map(|c| u32::from_le_bytes(c.try_into().unwrap()))
        .collect();
    Ok(TransferResponse {
        transfer_count,
        status,
        read_data,
    })
}

/// Encodes a `DAP_TransferBlock` request: `[0x06, dap_index, transfer_count(2 LE),
/// request_byte, write_data(4*N LE)...]`, `write_data` empty for a block read.
pub fn encode_transfer_block(
    dap_index: u8,
    transfer_count: u16,
    request: TransferRequest,
    write_data: &[u32],
) -> Vec<u8> {
    let mut buf = Vec::with_capacity(5 + write_data.len() * 4);
    buf.push(DAP_TRANSFER_BLOCK);
    buf.push(dap_index);
    buf.extend_from_slice(&transfer_count.to_le_bytes());
    buf.push(request.to_byte());
    for value in write_data {
        buf.extend_from_slice(&value.to_le_bytes());
    }
    buf
}

/// A decoded `DAP_TransferBlock` response.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransferBlockResponse {
    /// Number of transfers actually executed.
    pub transfer_count: u16,
    /// The status of the last-executed transfer.
    pub status: TransferStatus,
    /// Data words returned by a block read, in order.
    pub read_data: Vec<u32>,
}

/// Decodes a `DAP_TransferBlock` response:
/// `[0x06, transfer_count(2 LE), status, read_data(4*N LE)...]`.
pub fn decode_transfer_block(resp: &[u8]) -> Result<TransferBlockResponse, DecodeError> {
    let rest = expect_command(resp, DAP_TRANSFER_BLOCK, 4)?;
    let transfer_count = u16::from_le_bytes(rest[0..2].try_into().unwrap());
    let status = TransferStatus::from_byte(rest[2]);
    let data = &rest[3..];
    if data.len() % 4 != 0 {
        return Err(DecodeError::Truncated);
    }
    let read_data = data
        .chunks_exact(4)
        .map(|c| u32::from_le_bytes(c.try_into().unwrap()))
        .collect();
    Ok(TransferBlockResponse {
        transfer_count,
        status,
        read_data,
    })
}

// --- DAP_WriteABORT ----------------------------------------------------------------------------

/// Encodes a `DAP_WriteABORT` request: `[0x08, dap_index, abort_word(4 LE)]`.
pub fn encode_write_abort(dap_index: u8, abort_word: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(6);
    buf.push(DAP_WRITE_ABORT);
    buf.push(dap_index);
    buf.extend_from_slice(&abort_word.to_le_bytes());
    buf
}

/// Decodes a `DAP_WriteABORT` response: `[0x08, status]`.
pub fn decode_write_abort(resp: &[u8]) -> Result<(), DecodeError> {
    let rest = expect_command(resp, DAP_WRITE_ABORT, 2)?;
    if rest[0] == DAP_OK {
        Ok(())
    } else {
        Err(DecodeError::InvalidStatus(rest[0]))
    }
}

// --- DAP_SWJ_Clock / DAP_SWJ_Sequence ----------------------------------------------------------

/// Encodes a `DAP_SWJ_Clock` request: `[0x11, clock_hz(4 LE)]`.
pub fn encode_swj_clock(clock_hz: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(5);
    buf.push(DAP_SWJ_CLOCK);
    buf.extend_from_slice(&clock_hz.to_le_bytes());
    buf
}

/// Decodes a `DAP_SWJ_Clock` response: `[0x11, status]`.
pub fn decode_swj_clock(resp: &[u8]) -> Result<(), DecodeError> {
    let rest = expect_command(resp, DAP_SWJ_CLOCK, 2)?;
    if rest[0] == DAP_OK {
        Ok(())
    } else {
        Err(DecodeError::InvalidStatus(rest[0]))
    }
}

/// Encodes a `DAP_SWJ_Sequence` request: `[0x12, bit_count, data(ceil(bits/8))...]`.
/// `bit_count == 0` means 256 bits per spec; `data.len()` must match.
pub fn encode_swj_sequence(bit_count: u8, data: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(2 + data.len());
    buf.push(DAP_SWJ_SEQUENCE);
    buf.push(bit_count);
    buf.extend_from_slice(data);
    buf
}

/// Decodes a `DAP_SWJ_Sequence` response: `[0x12, status]`.
pub fn decode_swj_sequence(resp: &[u8]) -> Result<(), DecodeError> {
    let rest = expect_command(resp, DAP_SWJ_SEQUENCE, 2)?;
    if rest[0] == DAP_OK {
        Ok(())
    } else {
        Err(DecodeError::InvalidStatus(rest[0]))
    }
}

// --- DAP_JTAG_Configure ------------------------------------------------------------------------

/// Encodes a `DAP_JTAG_Configure` request: `[0x15, count, ir_length...]`, one IR bit length per
/// TAP with index 0 closest to TDO -- the index space every later command selects TAPs in.
pub fn encode_jtag_configure(ir_lengths: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(2 + ir_lengths.len());
    buf.push(DAP_JTAG_CONFIGURE);
    buf.push(ir_lengths.len() as u8);
    buf.extend_from_slice(ir_lengths);
    buf
}

/// Decodes a `DAP_JTAG_Configure` response: `[0x15, status]`.
pub fn decode_jtag_configure(resp: &[u8]) -> Result<(), DecodeError> {
    let rest = expect_command(resp, DAP_JTAG_CONFIGURE, 2)?;
    if rest[0] == DAP_OK {
        Ok(())
    } else {
        Err(DecodeError::InvalidStatus(rest[0]))
    }
}

/// Encodes a `DAP_JTAG_IDCODE` request: `[0x16, index]` into the configured chain.
pub fn encode_jtag_idcode(index: u8) -> Vec<u8> {
    vec![DAP_JTAG_IDCODE, index]
}

/// Decodes a `DAP_JTAG_IDCODE` response: `[0x16, status, idcode(4 LE)]`.
pub fn decode_jtag_idcode(resp: &[u8]) -> Result<u32, DecodeError> {
    let rest = expect_command(resp, DAP_JTAG_IDCODE, 2)?;
    if rest[0] != DAP_OK {
        return Err(DecodeError::InvalidStatus(rest[0]));
    }
    let bytes: [u8; 4] = rest
        .get(1..5)
        .ok_or(DecodeError::Truncated)?
        .try_into()
        .unwrap();
    Ok(u32::from_le_bytes(bytes))
}

// --- Per-packet transfer capacity --------------------------------------------------------------

/// Max words that fit a `DAP_TransferBlock` read response in one `packet_size`-byte packet (4
/// bytes of framing: command, transfer_count(2), status; 4 bytes per word).
pub const fn transfer_block_read_capacity(packet_size: u16) -> usize {
    (packet_size as usize).saturating_sub(4) / 4
}

/// Max words that fit a `DAP_TransferBlock` write request in one `packet_size`-byte packet (5
/// bytes of framing: command, dap_index, transfer_count(2), request_byte; 4 bytes per word).
pub const fn transfer_block_write_capacity(packet_size: u16) -> usize {
    (packet_size as usize).saturating_sub(5) / 4
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_ids_match_the_cmsis_dap_spec() {
        assert_eq!(DAP_INFO, 0x00);
        assert_eq!(DAP_CONNECT, 0x02);
        assert_eq!(DAP_TRANSFER_CONFIGURE, 0x04);
        assert_eq!(DAP_TRANSFER, 0x05);
        assert_eq!(DAP_TRANSFER_BLOCK, 0x06);
        assert_eq!(DAP_WRITE_ABORT, 0x08);
        assert_eq!(DAP_SWJ_CLOCK, 0x11);
        assert_eq!(DAP_SWJ_SEQUENCE, 0x12);
        assert_eq!(DAP_JTAG_CONFIGURE, 0x15);
        assert_eq!(DAP_JTAG_IDCODE, 0x16);
    }

    #[test]
    fn encode_info_packs_command_and_info_id() {
        assert_eq!(encode_info(InfoId::PacketSize), vec![0x00, 0xFF]);
        assert_eq!(encode_info(InfoId::PacketCount), vec![0x00, 0xFE]);
        assert_eq!(encode_info(InfoId::VendorId), vec![0x00, 0x01]);
        assert_eq!(encode_info(InfoId::ProductId), vec![0x00, 0x02]);
        assert_eq!(encode_info(InfoId::SerialNumber), vec![0x00, 0x03]);
        assert_eq!(encode_info(InfoId::FirmwareVersion), vec![0x00, 0x04]);
    }

    #[test]
    fn decode_info_packet_size_reads_le_u16() {
        let resp = [0x00, 0x02, 0x00, 0x02]; // 0x0200 = 512
        assert_eq!(decode_info_packet_size(&resp).unwrap(), 512);
    }

    #[test]
    fn decode_info_packet_count_reads_single_byte() {
        let resp = [0x00, 0x01, 0x04];
        assert_eq!(decode_info_packet_count(&resp).unwrap(), 4);
    }

    #[test]
    fn decode_info_string_strips_trailing_nul() {
        let resp = [0x00u8, 0x06, b'1', b'.', b'0', b'.', b'0', 0x00];
        assert_eq!(decode_info_string(&resp).unwrap(), "1.0.0");
    }

    #[test]
    fn decode_info_string_zero_length_is_empty() {
        let resp = [0x00u8, 0x00];
        assert_eq!(decode_info_string(&resp).unwrap(), "");
    }

    #[test]
    fn decode_info_rejects_wrong_command_byte() {
        let resp = [0x05, 0x02, 0x00, 0x02];
        assert_eq!(
            decode_info_packet_size(&resp),
            Err(DecodeError::UnexpectedCommand {
                expected: 0x00,
                found: 0x05
            })
        );
    }

    #[test]
    fn decode_info_rejects_truncated_buffer() {
        let resp = [0x00, 0x02, 0x00]; // claims 2 bytes, only 1 present
        assert_eq!(decode_info_packet_size(&resp), Err(DecodeError::Truncated));
    }

    #[test]
    fn encode_connect_packs_port() {
        assert_eq!(encode_connect(Port::Swd), vec![0x02, 0x01]);
        assert_eq!(encode_connect(Port::Jtag), vec![0x02, 0x02]);
    }

    #[test]
    fn decode_connect_reports_selected_port() {
        assert_eq!(decode_connect(&[0x02, 0x01]).unwrap(), Some(Port::Swd));
        assert_eq!(decode_connect(&[0x02, 0x02]).unwrap(), Some(Port::Jtag));
    }

    #[test]
    fn decode_connect_zero_means_failed() {
        assert_eq!(decode_connect(&[0x02, 0x00]).unwrap(), None);
    }

    #[test]
    fn encode_transfer_configure_packs_fields_le() {
        let cfg = TransferConfigureRequest {
            idle_cycles: 1,
            wait_retry: 5,
            match_retry: 8,
        };
        assert_eq!(
            encode_transfer_configure(cfg),
            vec![0x04, 0x01, 0x05, 0x00, 0x08, 0x00]
        );
    }

    #[test]
    fn decode_transfer_configure_ok() {
        assert_eq!(decode_transfer_configure(&[0x04, 0x00]), Ok(()));
    }

    #[test]
    fn transfer_request_to_byte_matches_spec_bit_layout() {
        // AP read of register 0x4 (address == 1): bit0=1 (AP), bit1=1 (R), bits[3:2]=01.
        let req = TransferRequest::new(true, true, 1);
        assert_eq!(req.to_byte(), 0b0000_0111);

        // DP write of register 0xC (address == 3): bit0=0, bit1=0, bits[3:2]=11.
        let req = TransferRequest::new(false, false, 3);
        assert_eq!(req.to_byte(), 0b0000_1100);
    }

    #[test]
    fn encode_transfer_single_write() {
        let items = [TransferItem {
            request: TransferRequest::new(true, false, 0),
            data: Some(0xDEAD_BEEF),
        }];
        let bytes = encode_transfer(0, &items);
        // [cmd, dap_index, count, request_byte, data(4 LE)]
        assert_eq!(bytes, vec![0x05, 0x00, 0x01, 0x01, 0xEF, 0xBE, 0xAD, 0xDE]);
    }

    #[test]
    fn encode_transfer_read_carries_no_data() {
        let items = [TransferItem {
            request: TransferRequest::new(false, true, 1),
            data: None,
        }];
        let bytes = encode_transfer(3, &items);
        assert_eq!(bytes, vec![0x05, 0x03, 0x01, 0b0000_0110]);
    }

    #[test]
    fn decode_transfer_reads_status_and_data() {
        // Two successful AP reads: 0x1111_1111 then 0x2222_2222.
        let resp = [
            0x05, 0x02, 0x01, //
            0x11, 0x11, 0x11, 0x11, //
            0x22, 0x22, 0x22, 0x22,
        ];
        let decoded = decode_transfer(&resp).unwrap();
        assert_eq!(decoded.transfer_count, 2);
        assert_eq!(decoded.status.ack(), Ack::Ok);
        assert_eq!(decoded.read_data, vec![0x1111_1111, 0x2222_2222]);
    }

    #[test]
    fn decode_transfer_reports_wait_ack() {
        let resp = [0x05, 0x00, 0b010];
        let decoded = decode_transfer(&resp).unwrap();
        assert_eq!(decoded.status.ack(), Ack::Wait);
        assert!(decoded.read_data.is_empty());
    }

    #[test]
    fn transfer_status_is_ok_only_for_clean_ok() {
        assert!(TransferStatus::from_byte(0b001).is_ok());
        assert!(!TransferStatus::from_byte(0b010).is_ok());
    }

    #[test]
    fn encode_transfer_block_read_carries_no_write_data() {
        let req = TransferRequest::new(true, true, 0);
        let bytes = encode_transfer_block(0, 4, req, &[]);
        assert_eq!(bytes, vec![0x06, 0x00, 0x04, 0x00, 0x03]);
    }

    #[test]
    fn encode_transfer_block_write_carries_data_le() {
        let req = TransferRequest::new(true, false, 0);
        let bytes = encode_transfer_block(0, 2, req, &[0x0000_0001, 0x0000_0002]);
        assert_eq!(
            bytes,
            vec![
                0x06, 0x00, 0x02, 0x00, 0x01, //
                0x01, 0x00, 0x00, 0x00, //
                0x02, 0x00, 0x00, 0x00,
            ]
        );
    }

    #[test]
    fn decode_transfer_block_reads_count_status_and_data() {
        let resp = [
            0x06, 0x02, 0x00, // cmd, transfer_count LE = 2
            0x01, // status: OK
            0xAA, 0xAA, 0xAA, 0xAA, //
            0xBB, 0xBB, 0xBB, 0xBB,
        ];
        let decoded = decode_transfer_block(&resp).unwrap();
        assert_eq!(decoded.transfer_count, 2);
        assert_eq!(decoded.status.ack(), Ack::Ok);
        assert_eq!(decoded.read_data, vec![0xAAAA_AAAA, 0xBBBB_BBBB]);
    }

    #[test]
    fn encode_write_abort_packs_index_and_word() {
        let bytes = encode_write_abort(0, 0x0000_0001);
        assert_eq!(bytes, vec![0x08, 0x00, 0x01, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn decode_write_abort_ok() {
        assert_eq!(decode_write_abort(&[0x08, 0x00]), Ok(()));
        assert_eq!(
            decode_write_abort(&[0x08, 0xFF]),
            Err(DecodeError::InvalidStatus(0xFF))
        );
    }

    #[test]
    fn encode_swj_clock_packs_hz_le() {
        // 2 MHz.
        assert_eq!(
            encode_swj_clock(2_000_000),
            vec![0x11, 0x80, 0x84, 0x1E, 0x00]
        );
    }

    #[test]
    fn decode_swj_clock_ok() {
        assert_eq!(decode_swj_clock(&[0x11, 0x00]), Ok(()));
    }

    #[test]
    fn encode_swj_sequence_line_reset() {
        // 50+ ones (JTAG-to-SWD line reset uses 56 SWDIO=1 clocks in the CMSIS-DAP examples).
        let data = [0xFFu8; 7]; // 56 bits
        let bytes = encode_swj_sequence(56, &data);
        let mut expected = vec![0x12, 56];
        expected.extend_from_slice(&data);
        assert_eq!(bytes, expected);
    }

    #[test]
    fn decode_swj_sequence_ok() {
        assert_eq!(decode_swj_sequence(&[0x12, 0x00]), Ok(()));
    }

    #[test]
    fn encode_jtag_configure_packs_count_and_ir_lengths() {
        // Two-TAP chain: Cortex-M debug TAP (IR 4) at index 0, boundary-scan TAP (IR 5) at 1.
        assert_eq!(encode_jtag_configure(&[4, 5]), vec![0x15, 0x02, 4, 5]);
        assert_eq!(encode_jtag_configure(&[4]), vec![0x15, 0x01, 4]);
    }

    #[test]
    fn decode_jtag_configure_ok_and_error() {
        assert_eq!(decode_jtag_configure(&[0x15, 0x00]), Ok(()));
        assert_eq!(
            decode_jtag_configure(&[0x15, 0xFF]),
            Err(DecodeError::InvalidStatus(0xFF))
        );
    }

    #[test]
    fn encode_jtag_idcode_packs_index() {
        assert_eq!(encode_jtag_idcode(0), vec![0x16, 0x00]);
        assert_eq!(encode_jtag_idcode(1), vec![0x16, 0x01]);
    }

    #[test]
    fn decode_jtag_idcode_reads_le_u32() {
        let resp = [0x16, 0x00, 0x77, 0x04, 0xA0, 0x3B]; // 0x3BA00477
        assert_eq!(decode_jtag_idcode(&resp), Ok(0x3BA0_0477));
    }

    #[test]
    fn decode_jtag_idcode_rejects_error_status_and_truncation() {
        assert_eq!(
            decode_jtag_idcode(&[0x16, 0xFF]),
            Err(DecodeError::InvalidStatus(0xFF))
        );
        assert_eq!(
            decode_jtag_idcode(&[0x16, 0x00, 0x77, 0x04]),
            Err(DecodeError::Truncated)
        );
    }

    #[test]
    fn transfer_block_read_capacity_matches_spec_formula() {
        assert_eq!(transfer_block_read_capacity(64), 15);
        assert_eq!(transfer_block_read_capacity(512), 127);
        assert_eq!(transfer_block_read_capacity(1024), 255);
    }

    #[test]
    fn transfer_block_write_capacity_matches_spec_formula() {
        assert_eq!(transfer_block_write_capacity(64), 14);
        assert_eq!(transfer_block_write_capacity(512), 126);
        assert_eq!(transfer_block_write_capacity(1024), 254);
    }
}
