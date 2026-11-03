//! ADIv5 SWD/JTAG attach and MEM-AP block read/write, owning all transfer chunking.

use std::fmt;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};

use crate::dap::{
    check_reply, decode_connect, decode_info_packet_size, decode_jtag_configure,
    decode_jtag_idcode, decode_swj_clock, decode_swj_sequence, decode_transfer,
    decode_transfer_block, decode_transfer_configure, decode_write_abort, encode_connect,
    encode_info, encode_jtag_configure, encode_jtag_idcode, encode_swj_clock, encode_swj_sequence,
    encode_transfer, encode_transfer_block, encode_transfer_configure, encode_write_abort,
    mixed_read_capacity, mixed_write_capacity, transfer_block_read_capacity,
    transfer_block_write_capacity, Ack, InfoId, Port, TransferConfigureRequest, TransferItem,
    TransferRequest,
};
use crate::transport::Probe;

/// ADIv5 DP CTRL/STAT bit requesting debug power-up.
pub const CDBGPWRUPREQ: u32 = 1 << 28;
/// `CTRL/STAT` bit acknowledging debug power-up.
pub const CDBGPWRUPACK: u32 = 1 << 29;
/// `CTRL/STAT` bit requesting system power-up.
pub const CSYSPWRUPREQ: u32 = 1 << 30;
/// `CTRL/STAT` bit acknowledging system power-up.
pub const CSYSPWRUPACK: u32 = 1 << 31;

/// The select value (`0xE79E`) that switches an SWJ port into SWD mode.
pub const JTAG_TO_SWD_SELECT: u16 = 0xE79E;

/// The two halves of the 128-bit CoreSight selection alert, LSB-first.
const SELECTION_ALERT_LO: u64 = 0x8685_2D95_6209_F392;
/// Upper half of the selection alert.
const SELECTION_ALERT_HI: u64 = 0x19BC_0EA2_E3DD_AFE9;
/// SWD-to-dormant sequence, sent before the alert so a DP already in SWD joins the dormant path.
const SWD_TO_DORMANT: [u8; 4] = [0xBA, 0xBB, 0xBB, 0x33];
/// 12-bit activation code selecting SWD out of dormant.
const SWD_ACTIVATION: u16 = 0x1A0;
/// The select value (`0xE73C`) that switches an SWJ port from SWD back into JTAG mode.
pub const SWD_TO_JTAG_SELECT: u16 = 0xE73C;
/// JTAG TAP reset (eight TMS-high) then four low into Run-Test/Idle, where DAP JTAG scans start.
const JTAG_TAP_RESET_IDLE_BITS: u8 = 12;
/// Data bytes for the JTAG TAP reset + idle (eight high then four low, LSB-first).
const JTAG_TAP_RESET_IDLE: [u8; 2] = [0xFF, 0x00];
/// IR bit length of the STM32F401 Cortex-M4 debug TAP (chain index 0, closest to TDO).
pub const STM32F401_CM_TAP_IR_LEN: u8 = 4;
/// IR bit length of the STM32F401 boundary-scan TAP (chain index 1, closest to TDI).
pub const STM32F401_BS_TAP_IR_LEN: u8 = 5;

/// SWD request byte for a DP `TARGETSEL` write, LSB-first.
const TARGETSEL_REQUEST: u8 = 0x99;
/// The turnaround and ACK bits of a `TARGETSEL` write, host-driven high: `TARGETSEL` has no ACK.
const TARGETSEL_ACK_WINDOW_HIGH: u64 = 0x1F;
/// A 51-cycle line reset terminated by three low cycles, so the DP frames the next packet.
const LINE_RESET_TERMINATED_BITS: u8 = 54;
/// Data bytes for the terminated line reset (51 high then 3 low, LSB-first).
const LINE_RESET_TERMINATED: [u8; 7] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];

/// DPIDR (read) / ABORT (write) live at `A[3:2] = 0b00`.
const DP_A_DPIDR: u8 = 0;
/// CTRL/STAT lives at `A[3:2] = 0b01` (with `DPBANKSEL = 0`).
const DP_A_CTRL_STAT: u8 = 1;
/// SELECT (write) lives at `A[3:2] = 0b10`.
const DP_A_SELECT: u8 = 2;

/// CSW lives at AP offset `0x00` -> `A[3:2] = 0b00`, `APBANKSEL = 0`.
const AP_A_CSW: u8 = 0;
/// TAR lives at AP offset `0x04` -> `A[3:2] = 0b01`, `APBANKSEL = 0`.
const AP_A_TAR: u8 = 1;
/// DRW lives at AP offset `0x0C` -> `A[3:2] = 0b11`, `APBANKSEL = 0`.
const AP_A_DRW: u8 = 3;
/// IDR lives at AP offset `0xFC` -> `A[3:2] = 0b11`, `APBANKSEL = 0xF`.
const AP_A_IDR: u8 = 3;
/// `APBANKSEL` for the MEM-AP core registers (CSW/TAR/DRW).
const AP_BANK_CORE: u8 = 0x0;
/// `APBANKSEL` for the MEM-AP IDR register.
const AP_BANK_IDR: u8 = 0xF;

/// SWD ABORT sticky-error clear; leaves `DAPABORT` and STKCMPCLR (UNPREDICTABLE on MINDP) at 0.
const ABORT_CLEAR_ALL: u32 = 0x1C;

/// `CTRL/STAT` sticky error flags: write-one-to-clear over JTAG, read-only over SWD.
const CTRL_STAT_STICKY_W1C: u32 = (1 << 1) | (1 << 4) | (1 << 5);

/// MEM-AP CSW: Cortex-M debug `Prot`/`DbgSwEnable`, single auto-increment, word size.
const CSW_WORD_ADDRINC: u32 = 0x2300_0002 | (1 << 4);

/// MEM-AP auto-increment wrap boundary, in bytes; crossing it needs a fresh TAR write.
pub const WRAP_BYTES: usize = 1024;

/// Default SWJ clock when the caller does not specify one.
pub const DEFAULT_CLOCK_HZ: u32 = 2_000_000;

/// Fallback `DAP_PACKET_SIZE` if the probe reports an unusable (zero) value.
pub const FALLBACK_PACKET_SIZE: u16 = 64;

/// Per-transfer USB round-trip timeout.
const TRANSFER_TIMEOUT: Duration = Duration::from_millis(1000);

/// Host-side retries of a whole `DAP_Transfer` / `DAP_TransferBlock` on a `WAIT` acknowledge.
const WAIT_RETRIES: u32 = 8;

/// Attempts at the first post-reset DPIDR read before giving up (the read that enables SWD).
const DPIDR_READ_ATTEMPTS: u32 = 4;

/// Poll iterations for the CTRL/STAT power-up acknowledge.
const POWERUP_POLLS: u32 = 100;

/// Cortex-M Debug Halting Control and Status Register.
pub const DHCSR: u32 = 0xE000_EDF0;
/// `DHCSR` debug key, required in the upper 16 bits of any write to unlock the register.
pub const DHCSR_DBGKEY: u32 = 0xA05F_0000;
/// `DHCSR` bit enabling halting debug.
pub const DHCSR_C_DEBUGEN: u32 = 1 << 0;
/// `DHCSR` bit requesting the core halt.
pub const DHCSR_C_HALT: u32 = 1 << 1;
/// `DHCSR` status bit: the core reports halted.
pub const DHCSR_S_HALT: u32 = 1 << 17;
/// DHCSR status: the core has been reset since the last DHCSR read (sticky, cleared by the read).
pub const DHCSR_S_RESET_ST: u32 = 1 << 25;
/// Bound on how long a halt request may take to be acknowledged by `S_HALT`.
const HALT_TIMEOUT: Duration = Duration::from_millis(100);

/// Cortex-M Application Interrupt and Reset Control Register.
pub const AIRCR: u32 = 0xE000_ED0C;
/// `AIRCR` write key (`VECTKEY`), required in the upper 16 bits of any write to unlock it.
pub const AIRCR_VECTKEY: u32 = 0x05FA_0000;
/// `AIRCR` bit requesting a system reset (`SYSRESETREQ`).
pub const AIRCR_SYSRESETREQ: u32 = 1 << 2;

/// Parameters for an [`Adiv5::attach`] bring-up.
#[derive(Debug, Clone, Copy)]
pub struct AttachConfig {
    /// SWJ clock in Hz for `DAP_SWJ_Clock`.
    pub clock_hz: u32,
    /// How to address the MEM-AP: ADIv5 `APSEL`, or an ADIv6 AP base address.
    pub ap: ApMode,
    /// `DAP_TransferConfigure` WAIT-retry count (probe-internal retries on a `WAIT` ack).
    pub wait_retry: u16,
    /// `DAP_TransferConfigure` match-retry count (probe-internal retries for value-match reads).
    pub match_retry: u16,
    /// Multidrop `TARGETSEL` to select before the DPIDR read; `None` for single-drop.
    pub targetsel: Option<u32>,
}

impl Default for AttachConfig {
    fn default() -> Self {
        AttachConfig {
            clock_hz: DEFAULT_CLOCK_HZ,
            ap: ApMode::V5 { apsel: 0 },
            wait_retry: 64,
            match_retry: 16,
            targetsel: None,
        }
    }
}

/// Parameters for an [`Adiv5::attach_jtag`] bring-up.
#[derive(Debug, Clone)]
pub struct JtagAttachConfig {
    /// SWJ clock in Hz for `DAP_SWJ_Clock`.
    pub clock_hz: u32,
    /// The MEM-AP `APSEL`; JTAG attach is ADIv5-only.
    pub apsel: u8,
    /// Scan-chain IR lengths, one per TAP, index 0 closest to TDO, for `DAP_JTAG_Configure`.
    pub ir_lengths: Vec<u8>,
    /// Chain index of the Arm debug TAP; carried as every command's DAP index byte.
    pub tap_index: u8,
    /// `DAP_TransferConfigure` WAIT-retry count.
    pub wait_retry: u16,
    /// `DAP_TransferConfigure` match-retry count.
    pub match_retry: u16,
}

impl Default for JtagAttachConfig {
    fn default() -> Self {
        JtagAttachConfig {
            clock_hz: DEFAULT_CLOCK_HZ,
            apsel: 0,
            ir_lengths: vec![STM32F401_CM_TAP_IR_LEN, STM32F401_BS_TAP_IR_LEN],
            tap_index: 0,
            wait_retry: 64,
            match_retry: 16,
        }
    }
}

/// A decoded ADIv5 Debug Port identification register (DPIDR).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Dpidr {
    /// The raw 32-bit register value.
    pub raw: u32,
}

impl Dpidr {
    /// JEDEC JEP106 designer code, `DPIDR[11:1]` (`0x23B` for Arm).
    pub const fn designer(self) -> u16 {
        ((self.raw >> 1) & 0x7FF) as u16
    }

    /// DP architecture version, `DPIDR[15:12]`: 2 and up carry the multidrop identity registers.
    pub const fn version(self) -> u8 {
        ((self.raw >> 12) & 0xF) as u8
    }
}

impl fmt::Display for Dpidr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let arm = if self.designer() == 0x23B {
            " (Arm)"
        } else {
            ""
        };
        write!(f, "0x{:08X}{arm}", self.raw)
    }
}

/// A decoded IEEE 1149.1 JTAG TAP IDCODE; a JTAG-DP implements no DPIDR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Idcode {
    /// The raw 32-bit register value.
    pub raw: u32,
}

impl Idcode {
    /// JEDEC JEP106 designer code, `IDCODE[11:1]` (`0x23B` for an Arm debug TAP).
    pub const fn designer(self) -> u16 {
        ((self.raw >> 1) & 0x7FF) as u16
    }

    /// Bit 0, reads as one on a valid IDCODE (distinguishes it from a BYPASS zero).
    pub const fn marker(self) -> bool {
        self.raw & 1 != 0
    }

    /// True when this looks like an Arm debug TAP: valid marker bit and the Arm JEP106 designer.
    pub const fn looks_like_arm(self) -> bool {
        self.marker() && self.designer() == 0x23B
    }
}

impl fmt::Display for Idcode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let arm = if self.looks_like_arm() { " (Arm)" } else { "" };
        write!(f, "0x{:08X}{arm}", self.raw)
    }
}

/// A decoded ADIv5 Access Port identification register (IDR).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApIdr {
    /// The raw 32-bit register value.
    pub raw: u32,
}

impl fmt::Display for ApIdr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "0x{:08X}", self.raw)
    }
}

/// Compose an ADIv5 DP `SELECT` word from `apsel` and `apbank`, `DPBANKSEL = 0`.
const fn select_word(apsel: u8, apbank: u8) -> u32 {
    ((apsel as u32) << 24) | (((apbank & 0xF) as u32) << 4)
}

/// Words from word-aligned `addr` to the next wrap boundary, in `1..=WRAP_BYTES/4`.
const fn words_to_wrap(addr: u32) -> usize {
    let into = (addr as usize) & (WRAP_BYTES - 1);
    (WRAP_BYTES - into) / 4
}

/// Next chunk length in words, bounded by the wrap boundary and `max_words`.
const fn block_chunk_words(cur: u32, remaining: usize, max_words: usize) -> usize {
    let wrap = words_to_wrap(cur);
    let cap = if max_words == 0 { 1 } else { max_words };
    let mut n = remaining;
    if n > wrap {
        n = wrap;
    }
    if n > cap {
        n = cap;
    }
    n
}

/// USB round-trip timeout for the pipelined block packets.
const PIPELINE_TIMEOUT: Duration = Duration::from_millis(1000);

/// Where a transfer re-arms the MEM-AP TAR.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TarPolicy {
    /// One TAR write before every block.
    #[default]
    PerChunk,
    /// One TAR write per 1 KiB window; blocks inside it ride the AP's own increment.
    PerWindow,
}

impl TarPolicy {
    /// Short label used in the JSON and in error text.
    pub const fn label(self) -> &'static str {
        match self {
            TarPolicy::PerChunk => "per-chunk",
            TarPolicy::PerWindow => "per-window",
        }
    }
}

/// The command shape a planned transfer is issued as.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TransferShape {
    /// A `DAP_TransferBlock` per chunk, with its TAR write as a separate `DAP_Transfer`.
    #[default]
    Block,
    /// One `DAP_Transfer` per chunk carrying the TAR write and the chunk's DRW items.
    Mixed,
}

impl TransferShape {
    /// Short label used in the JSON and in error text.
    pub const fn label(self) -> &'static str {
        match self {
            TransferShape::Block => "block",
            TransferShape::Mixed => "mixed",
        }
    }

    /// Words one command carries in this shape at `packet_size`, per direction.
    pub const fn capacity(self, packet_size: u16, read: bool) -> usize {
        match (self, read) {
            (TransferShape::Block, true) => transfer_block_read_capacity(packet_size),
            (TransferShape::Block, false) => transfer_block_write_capacity(packet_size),
            (TransferShape::Mixed, true) => mixed_read_capacity(packet_size),
            (TransferShape::Mixed, false) => mixed_write_capacity(packet_size),
        }
    }
}

/// One planned block transfer: its address, its word count, and whether TAR is written first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockOp {
    /// First word address of the block.
    pub addr: u32,
    /// Words the block moves.
    pub words: usize,
    /// Whether a TAR write precedes this block.
    pub tar: bool,
}

/// Split `words` at `addr` into `(chunk_addr, chunk_words)` pieces bounded by the wrap and `cap`.
fn plan_chunks(addr: u32, words: usize, cap: usize) -> Vec<(u32, usize)> {
    let mut chunks = Vec::new();
    let mut cur = addr;
    let mut remaining = words;
    while remaining > 0 {
        let n = block_chunk_words(cur, remaining, cap);
        chunks.push((cur, n));
        cur = cur.wrapping_add((n * 4) as u32);
        remaining -= n;
    }
    chunks
}

/// `plan_chunks` plus `policy`'s TAR placement; both policies touch the same addresses in order.
pub fn plan_ops(addr: u32, words: usize, cap: usize, policy: TarPolicy) -> Vec<BlockOp> {
    plan_chunks(addr, words, cap)
        .into_iter()
        .enumerate()
        .map(|(i, (chunk_addr, n))| BlockOp {
            addr: chunk_addr,
            words: n,
            tar: match policy {
                TarPolicy::PerChunk => true,
                TarPolicy::PerWindow => i == 0 || (chunk_addr as usize).is_multiple_of(WRAP_BYTES),
            },
        })
        .collect()
}

/// Commands a `words`-word transfer at `addr` costs under `policy`.
pub fn command_count(addr: u32, words: usize, cap: usize, policy: TarPolicy) -> usize {
    plan_ops(addr, words, cap, policy)
        .iter()
        .map(|op| 1 + usize::from(op.tar))
        .sum()
}

/// Commands a `words`-word transfer at `addr` costs under `policy` in `shape`.
pub fn shaped_command_count(
    addr: u32,
    words: usize,
    cap: usize,
    policy: TarPolicy,
    shape: TransferShape,
) -> usize {
    match shape {
        TransferShape::Block => command_count(addr, words, cap, policy),
        TransferShape::Mixed => plan_ops(addr, words, cap, policy).len(),
    }
}

/// `DAP_Transfer` response length for a single-item TAR write.
const TAR_WRITE_RESPONSE_LEN: usize = 3;

/// `DAP_TransferBlock` read response length for `words` data words.
const fn block_read_response_len(words: usize) -> usize {
    4 + 4 * words
}

/// The `DAP_TransferBlock` write response length in bytes: header only, no data words.
const fn block_write_response_len() -> usize {
    4
}

/// The pipelined packets over `ops`, with exact response lengths; `data` is `None` for a read.
fn build_block_requests(
    ops: &[BlockOp],
    data: Option<&[u32]>,
    dap_index: u8,
) -> (Vec<Vec<u8>>, Vec<usize>) {
    let mut requests = Vec::with_capacity(ops.len() * 2);
    let mut expected_lens = Vec::with_capacity(ops.len() * 2);
    let mut done = 0;
    for op in ops {
        if op.tar {
            requests.push(encode_transfer(
                dap_index,
                &[TransferItem {
                    request: TransferRequest::new(true, false, AP_A_TAR),
                    data: Some(op.addr),
                }],
            ));
            expected_lens.push(TAR_WRITE_RESPONSE_LEN);
        }

        match data {
            None => {
                requests.push(encode_transfer_block(
                    dap_index,
                    op.words as u16,
                    TransferRequest::new(true, true, AP_A_DRW),
                    &[],
                ));
                expected_lens.push(block_read_response_len(op.words));
            }
            Some(data) => {
                requests.push(encode_transfer_block(
                    dap_index,
                    op.words as u16,
                    TransferRequest::new(true, false, AP_A_DRW),
                    &data[done..done + op.words],
                ));
                expected_lens.push(block_write_response_len());
                done += op.words;
            }
        }
    }
    (requests, expected_lens)
}

/// `DAP_Transfer` response length for a mixed chunk of `words` reads.
const fn mixed_response_len(words: usize) -> usize {
    3 + 4 * words
}

/// The mixed packets over `ops`, one `DAP_Transfer` per chunk; `data` is `None` for a read.
fn build_mixed_requests(
    ops: &[BlockOp],
    data: Option<&[u32]>,
    dap_index: u8,
) -> (Vec<Vec<u8>>, Vec<usize>) {
    let mut requests = Vec::with_capacity(ops.len());
    let mut expected_lens = Vec::with_capacity(ops.len());
    let mut done = 0;
    for op in ops {
        let mut items = Vec::with_capacity(1 + op.words);
        if op.tar {
            items.push(TransferItem {
                request: TransferRequest::new(true, false, AP_A_TAR),
                data: Some(op.addr),
            });
        }
        let drw = TransferRequest::new(true, data.is_none(), AP_A_DRW);
        for i in 0..op.words {
            items.push(TransferItem {
                request: drw,
                data: data.map(|d| d[done + i]),
            });
        }
        requests.push(encode_transfer(dap_index, &items));
        expected_lens.push(mixed_response_len(if data.is_none() {
            op.words
        } else {
            0
        }));
        done += op.words;
    }
    (requests, expected_lens)
}

/// Transfer items one mixed chunk asks for: the DRW items plus the TAR write where present.
const fn mixed_items(op: &BlockOp) -> usize {
    op.words + if op.tar { 1 } else { 0 }
}

/// Decode and check one mixed chunk response, returning its read words (empty for a write).
fn check_mixed(resp: &[u8], op: &BlockOp, read: bool) -> Result<Vec<u32>> {
    let items = mixed_items(op);
    let t = decode_transfer(resp).context("decoding mixed DAP_Transfer response failed")?;
    let words = if read { op.words } else { 0 };
    if !t.status.is_ok() || t.transfer_count as usize != items || t.read_data.len() != words {
        bail!(
            "mixed DAP_Transfer failed after {} of {items} transfers ({})",
            t.transfer_count,
            t.status
        );
    }
    Ok(t.read_data)
}

/// Check one pipelined TAR-write response.
fn check_pipelined_tar(tar_resp: &[u8]) -> Result<()> {
    let tar = decode_transfer(tar_resp).context("decoding pipelined TAR write response failed")?;
    if !tar.status.is_ok() || tar.transfer_count != 1 {
        bail!("pipelined TAR write failed ({})", tar.status);
    }
    Ok(())
}

/// Decode and check one pipelined block response, returning its read words (empty for a write).
fn check_pipelined_block(block_resp: &[u8], words: usize) -> Result<Vec<u32>> {
    let block = decode_transfer_block(block_resp)
        .context("decoding pipelined DAP_TransferBlock response failed")?;
    if !block.status.is_ok() || block.transfer_count as usize != words {
        bail!(
            "pipelined DAP_TransferBlock failed after {} of {words} words ({})",
            block.transfer_count,
            block.status
        );
    }
    Ok(block.read_data)
}

/// `failure`, unless `resp` does not even fit `request`, the deeper fault.
fn diagnose(request: &[u8], resp: &[u8], failure: anyhow::Error) -> anyhow::Error {
    match check_reply(request, resp) {
        Ok(()) => failure,
        Err(e) => anyhow!(e).context("CMSIS-DAP reply does not fit its request"),
    }
}

/// How an attach addresses the AP; an ADIv6 AP addressed the ADIv5 way reads rubbish, not an error.
#[derive(Debug, Clone, Copy)]
pub enum ApMode {
    /// ADIv5: `APSEL` in `SELECT[31:24]`, `APBANKSEL` in `SELECT[7:4]`.
    V5 {
        /// The MEM-AP index.
        apsel: u8,
    },
    /// ADIv6: a memory-mapped 4 KiB component frame reached by base address.
    V6 {
        /// The AP's base address.
        ap_base: u32,
        /// Cached `SELECT`/`SELECT1` pair.
        sel: SelectV6,
    },
}

impl Default for ApMode {
    fn default() -> Self {
        ApMode::V5 { apsel: 0 }
    }
}

/// ADIv6 MEM-AP core page holding CSW/TAR/DRW; `A[3:2]` picks one.
const AP_V6_OFF_CORE: u32 = 0xD00;
/// ADIv6 MEM-AP IDR byte offset within the AP's 4 KiB component frame (ADIv5 keeps `0xFC`).
const AP_V6_OFF_IDR: u32 = 0xDFC;
/// `DPBANKSEL` value exposing `SELECT1` at DP address `0x4` (the DPv3 register map's bank 5).
const DP_BANKSEL_SELECT1: u32 = 0x5;
/// `DPBANKSEL` exposing `TARGETID` at the CTRL/STAT transfer slot (ADIv5.2 DPv2 bank 2).
const DP_BANKSEL_TARGETID: u32 = 0x2;
/// `DPBANKSEL` exposing `DLPIDR` at the CTRL/STAT transfer slot (ADIv5.2 DPv2 bank 3).
const DP_BANKSEL_DLPIDR: u32 = 0x3;

/// `TARGETID` fields carried into `TARGETSEL`; TREVISION is excluded.
const TARGETID_TARGETSEL_MASK: u32 = 0x0FFF_FFFF;
/// `DLPIDR` field contributing TINSTANCE `[31:28]`, the top nibble of `TARGETSEL`.
const DLPIDR_TINSTANCE_MASK: u32 = 0xF000_0000;

/// `TARGETSEL` from `TARGETID` with TREVISION replaced by `DLPIDR`'s TINSTANCE.
pub const fn derive_targetsel(targetid: u32, dlpidr: u32) -> u32 {
    (targetid & TARGETID_TARGETSEL_MASK) | (dlpidr & DLPIDR_TINSTANCE_MASK)
}

/// The multidrop identification registers of a DPv2+ Debug Port, read at attach.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DpIdent {
    /// Raw `TARGETID` (DP bank 2).
    pub targetid: u32,
    /// Raw `DLPIDR` (DP bank 3).
    pub dlpidr: u32,
}

impl DpIdent {
    /// `TARGETID` TDESIGNER (JEP106 designer), `[11:1]`.
    pub const fn designer(self) -> u16 {
        ((self.targetid >> 1) & 0x7FF) as u16
    }

    /// `TARGETID` TPARTNO, `[27:12]`.
    pub const fn partno(self) -> u16 {
        ((self.targetid >> 12) & 0xFFFF) as u16
    }

    /// `TARGETID` TREVISION, `[31:28]`.
    pub const fn revision(self) -> u8 {
        ((self.targetid >> 28) & 0xF) as u8
    }

    /// `DLPIDR` PROTVSN (implemented SWD protocol version), `[3:0]`.
    pub const fn protvsn(self) -> u8 {
        (self.dlpidr & 0xF) as u8
    }

    /// `DLPIDR` TINSTANCE (device instance), `[31:28]`.
    pub const fn tinstance(self) -> u8 {
        ((self.dlpidr >> 28) & 0xF) as u8
    }

    /// The `TARGETSEL` this port answers to (see [`derive_targetsel`]).
    pub const fn targetsel(self) -> u32 {
        derive_targetsel(self.targetid, self.dlpidr)
    }
}

impl fmt::Display for DpIdent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "TARGETID 0x{:08X} DLPIDR 0x{:08X} -> TARGETSEL 0x{:08X}",
            self.targetid,
            self.dlpidr,
            self.targetsel()
        )
    }
}

/// The ADIv6 frame byte offset of the SELECT window holding MEM-AP register bank `apbank`.
const fn ap_v6_frame_offset(apbank: u8) -> u32 {
    if apbank == AP_BANK_IDR {
        AP_V6_OFF_IDR
    } else {
        AP_V6_OFF_CORE | ((apbank as u32) << 4)
    }
}

/// Split an ADIv6 AP register address into `SELECT`, `SELECT1` and the transfer address `A[3:2]`.
pub const fn split_ap_addr_v6(addr: u64) -> (u32, u32, u8) {
    (
        (addr as u32) & 0xFFFF_FFF0,
        (addr >> 32) as u32,
        ((addr >> 2) & 0x3) as u8,
    )
}

/// Cached ADIv6 `SELECT`/`SELECT1` pair; planning is pure, so a failed write cannot poison it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SelectV6 {
    select: Option<u32>,
    select1: Option<u32>,
}

/// The DP writes [`SelectV6::plan`] found necessary to address one AP register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SelectV6Plan {
    /// `SELECT1` value to write, if changed; the `DPBANKSEL = 5` detour clobbers `SELECT`.
    pub write_select1: Option<u32>,
    /// `SELECT` value to write, if it changed -- always present after a `SELECT1` write.
    pub write_select: Option<u32>,
    /// Transfer address `A[3:2]` of the target register.
    pub a: u8,
}

impl SelectV6 {
    /// A cache with both halves invalid, so the first use writes both.
    pub const fn new() -> SelectV6 {
        SelectV6 {
            select: None,
            select1: None,
        }
    }

    /// Plan the DP writes that make the AP register at full address `addr` reachable.
    pub fn plan(&self, addr: u64) -> SelectV6Plan {
        let (select, select1, a) = split_ap_addr_v6(addr);
        let write_select1 = if self.select1 != Some(select1) {
            Some(select1)
        } else {
            None
        };
        // The `SELECT1` bank detour overwrote `SELECT`, so rewrite it.
        let write_select = if write_select1.is_some() || self.select != Some(select) {
            Some(select)
        } else {
            None
        };
        SelectV6Plan {
            write_select1,
            write_select,
            a,
        }
    }

    /// Record a plan whose DP writes all succeeded.
    pub fn commit(&mut self, plan: &SelectV6Plan) {
        if let Some(select1) = plan.write_select1 {
            self.select1 = Some(select1);
        }
        if let Some(select) = plan.write_select {
            self.select = Some(select);
        }
    }
}

/// An attach over a [`Probe`], owning the MEM-AP transfer state and its `SELECT`/`CSW` caches.
pub struct Adiv5<'a> {
    probe: &'a mut Probe,
    ap: ApMode,
    /// Which wire this attach is running over, since sticky-error clearing differs by wire.
    wire: Port,
    /// DAP index byte: 0 over SWD, the selected TAP's chain index over JTAG.
    dap_index: u8,
    /// Cached DP `SELECT` (`None` forces a rewrite on the next AP access).
    select: Option<u32>,
    /// Cached MEM-AP `CSW` (`None` forces a rewrite on the next memory access).
    csw: Option<u32>,
    /// Negotiated `DAP_PACKET_SIZE`, used to bound block transfers.
    packet_size: u16,
    /// DPIDR read during an SWD attach; zero on a JTAG attach.
    dpidr: Dpidr,
    /// AP IDR read during the attach.
    ap_idr: ApIdr,
    /// TAP IDCODE read during [`Adiv5::attach_jtag`]; `None` on an SWD attach.
    idcode: Option<Idcode>,
    /// Multidrop identity read during the attach; `None` on a DPv1 DP or a JTAG attach.
    dp_ident: Option<DpIdent>,
    /// Where block transfers re-arm TAR.
    tar_policy: TarPolicy,
    /// The command shape block transfers are issued as.
    shape: TransferShape,
}

impl<'a> Adiv5<'a> {
    /// Clear SELECT/CSW caches for entry, retaining the supplied AP mode and its selection state.
    fn new_handle(probe: &'a mut Probe, ap: ApMode, packet_size: u16) -> Adiv5<'a> {
        Adiv5 {
            probe,
            ap,
            wire: Port::Swd,
            dap_index: 0,
            select: None,
            csw: None,
            packet_size,
            dpidr: Dpidr { raw: 0 },
            ap_idr: ApIdr { raw: 0 },
            idcode: None,
            dp_ident: None,
            tar_policy: TarPolicy::default(),
            shape: TransferShape::default(),
        }
    }

    /// Choose where block transfers re-arm TAR; the default is [`TarPolicy::PerChunk`].
    pub fn set_tar_policy(&mut self, policy: TarPolicy) {
        self.tar_policy = policy;
    }

    /// The TAR policy this handle's block transfers plan against.
    pub fn tar_policy(&self) -> TarPolicy {
        self.tar_policy
    }

    /// Choose the command shape block transfers take; the default is [`TransferShape::Block`].
    pub fn set_shape(&mut self, shape: TransferShape) {
        self.shape = shape;
    }

    /// The command shape this handle's block transfers take.
    pub fn shape(&self) -> TransferShape {
        self.shape
    }

    /// The chunk plan this handle's shape, packet size and TAR policy imply.
    fn plan(&self, addr: u32, words: usize, read: bool) -> Vec<BlockOp> {
        let cap = self.shape.capacity(self.packet_size, read);
        plan_ops(addr, words, cap, self.tar_policy)
    }

    /// One mixed `DAP_Transfer` per chunk; a refused chunk fails, since a retry re-reads a moved TAR.
    fn serial_mixed(
        &mut self,
        ops: &[BlockOp],
        mut out: Option<&mut [u32]>,
        data: Option<&[u32]>,
    ) -> Result<()> {
        let (requests, _) = build_mixed_requests(ops, data, self.dap_index);
        let mut done = 0;
        for (op, request) in ops.iter().zip(requests) {
            let resp = self
                .probe
                .transfer(&request, TRANSFER_TIMEOUT)
                .context("mixed DAP_Transfer USB round-trip failed")?;
            let words =
                check_mixed(&resp, op, out.is_some()).map_err(|e| diagnose(&request, &resp, e))?;
            if let Some(buf) = out.as_deref_mut() {
                buf[done..done + op.words].copy_from_slice(&words);
            }
            done += op.words;
        }
        Ok(())
    }

    /// [`Adiv5::serial_mixed`] with up to `depth` chunks outstanding.
    fn pipelined_mixed(
        &mut self,
        ops: &[BlockOp],
        mut out: Option<&mut [u32]>,
        data: Option<&[u32]>,
        depth: usize,
    ) -> Result<()> {
        let (requests, expected_lens) = build_mixed_requests(ops, data, self.dap_index);
        let responses =
            self.probe
                .submit_pipelined(&requests, &expected_lens, depth, PIPELINE_TIMEOUT)?;
        let mut done = 0;
        for ((op, resp), request) in ops.iter().zip(&responses).zip(&requests) {
            let words =
                check_mixed(resp, op, out.is_some()).map_err(|e| diagnose(request, resp, e))?;
            if let Some(buf) = out.as_deref_mut() {
                buf[done..done + op.words].copy_from_slice(&words);
            }
            done += op.words;
        }
        Ok(())
    }

    /// SWD bring-up, leaving the MEM-AP programmed for 32-bit auto-incrementing transfers.
    pub fn attach(probe: &'a mut Probe, config: AttachConfig) -> Result<Adiv5<'a>> {
        let packet_size = connect_and_configure(
            probe,
            Port::Swd,
            config.clock_hz,
            config.wait_retry,
            config.match_retry,
        )?;

        let mut adi = Adiv5::new_handle(probe, config.ap, packet_size);

        adi.dpidr = Dpidr {
            raw: match (config.ap, config.targetsel) {
                // An ADIv6 DP powers up dormant, where line reset and JTAG-to-SWD are inert.
                (ApMode::V6 { .. }, targetsel) => adi.read_dpidr_dormant(targetsel)?,
                (_, Some(targetsel)) => adi.read_dpidr_targetsel(targetsel)?,
                (_, None) => adi.read_dpidr()?,
            },
        };

        adi.finish_attach()?;

        Ok(adi)
    }

    /// ADIv5 JTAG-DP bring-up, reading `DAP_JTAG_IDCODE` in place of DPIDR.
    pub fn attach_jtag(probe: &'a mut Probe, config: JtagAttachConfig) -> Result<Adiv5<'a>> {
        if config.ir_lengths.is_empty() {
            bail!("JTAG attach needs at least one scan-chain IR length");
        }
        if usize::from(config.tap_index) >= config.ir_lengths.len() {
            bail!(
                "JTAG TAP index {} is out of range for a {}-TAP chain",
                config.tap_index,
                config.ir_lengths.len()
            );
        }

        let packet_size = connect_and_configure(
            probe,
            Port::Jtag,
            config.clock_hz,
            config.wait_retry,
            config.match_retry,
        )?;

        let mut adi = Adiv5::new_handle(
            probe,
            ApMode::V5 {
                apsel: config.apsel,
            },
            packet_size,
        );
        adi.wire = Port::Jtag;
        adi.dap_index = config.tap_index;

        probe_command(
            adi.probe,
            encode_jtag_configure(&config.ir_lengths),
            decode_jtag_configure,
        )
        .context("DAP_JTAG_Configure failed")?;

        // Retried like the SWD DPIDR read, re-issuing the whole entry on a failure.
        let mut idcode = None;
        let mut last: Option<anyhow::Error> = None;
        for attempt in 0..DPIDR_READ_ATTEMPTS {
            adi.jtag_entry()?;
            match probe_command(
                adi.probe,
                encode_jtag_idcode(config.tap_index),
                decode_jtag_idcode,
            ) {
                Ok(raw) => {
                    idcode = Some(Idcode { raw });
                    break;
                }
                Err(e) => {
                    last = Some(e.context(format!(
                        "DAP_JTAG_IDCODE through TAP index {} (attempt {})",
                        config.tap_index,
                        attempt + 1
                    )));
                }
            }
        }
        let idcode = match idcode {
            Some(idcode) => idcode,
            None => return Err(last.unwrap_or_else(|| anyhow!("IDCODE read failed"))),
        };
        if idcode.raw == 0 || idcode.raw == 0xFFFF_FFFF {
            bail!(
                "TAP index {} returned IDCODE 0x{:08X}: the scan chain looks dead or \
                 misdescribed",
                config.tap_index,
                idcode.raw
            );
        }
        if !idcode.looks_like_arm() {
            eprintln!(
                "warning: IDCODE {idcode} does not look like an Arm debug TAP (expected \
                 designer 0x23B)"
            );
        }
        adi.idcode = Some(idcode);

        adi.finish_attach()?;

        Ok(adi)
    }

    /// Bring-up after DPIDR/IDCODE: clear sticky errors, power up, read the MEM-AP IDR, program CSW.
    fn finish_attach(&mut self) -> Result<()> {
        self.clear_sticky_errors()
            .context("clearing sticky errors after the entry read failed")?;
        self.power_up()?;
        self.ap_idr = self.read_idr()?;
        self.ensure_csw()?;
        // Only a DPv2+ DP implements TARGETID/DLPIDR, and only over SWD.
        if self.wire == Port::Swd && self.dpidr.version() >= 2 {
            self.dp_ident = self.read_dp_ident()?;
        }
        Ok(())
    }

    /// Read `TARGETID` and `DLPIDR`, `None` where refused; re-asserts the AP window after.
    fn read_dp_ident(&mut self) -> Result<Option<DpIdent>> {
        let ident = self.read_targetid_dlpidr();
        if ident.is_err() {
            let _ = self.clear_sticky_errors();
        }
        self.select = None;
        if let ApMode::V6 { sel, .. } = &mut self.ap {
            *sel = SelectV6::new();
        }
        self.ensure_select(AP_BANK_CORE)
            .context("restoring the AP core window after the DP-bank reads failed")?;
        Ok(ident.ok())
    }

    /// The two bank-selected reads themselves; the caller owns the cache restore.
    fn read_targetid_dlpidr(&mut self) -> Result<DpIdent> {
        self.write_dp(DP_A_SELECT, DP_BANKSEL_TARGETID)
            .context("selecting DP bank 2 (TARGETID) failed")?;
        let targetid = self
            .read_dp(DP_A_CTRL_STAT)
            .context("TARGETID read failed")?;
        self.write_dp(DP_A_SELECT, DP_BANKSEL_DLPIDR)
            .context("selecting DP bank 3 (DLPIDR) failed")?;
        let dlpidr = self.read_dp(DP_A_CTRL_STAT).context("DLPIDR read failed")?;
        Ok(DpIdent { targetid, dlpidr })
    }

    /// The DPIDR read during [`Adiv5::attach`]; zero after [`Adiv5::attach_jtag`].
    pub fn dpidr(&self) -> Dpidr {
        self.dpidr
    }

    /// The TAP IDCODE read during [`Adiv5::attach_jtag`], or `None` on an SWD attach.
    pub fn idcode(&self) -> Option<Idcode> {
        self.idcode
    }

    /// The AP IDR read during the attach.
    pub fn ap_idr(&self) -> ApIdr {
        self.ap_idr
    }

    /// The multidrop identity read during the attach; `None` on a DPv1 DP or over JTAG.
    pub fn dp_ident(&self) -> Option<DpIdent> {
        self.dp_ident
    }

    /// The negotiated `DAP_PACKET_SIZE` bounding block transfers.
    pub fn packet_size(&self) -> u16 {
        self.packet_size
    }

    /// Read a single 32-bit word from `addr` (write TAR, then read DRW).
    pub fn read_mem32(&mut self, addr: u32) -> Result<u32> {
        self.ensure_csw()?;
        self.write_tar(addr)?;
        self.read_ap(AP_BANK_CORE, AP_A_DRW)
    }

    /// Write a single 32-bit word to `addr` (write TAR, then write DRW).
    pub fn write_mem32(&mut self, addr: u32, value: u32) -> Result<()> {
        self.ensure_csw()?;
        self.write_tar(addr)?;
        self.write_ap(AP_BANK_CORE, AP_A_DRW, value)
    }

    /// Halt the core and wait for `DHCSR.S_HALT`; an ACKed halt request can still be ignored.
    pub fn halt_core(&mut self) -> Result<()> {
        let started = Instant::now();
        let mut request = true;
        loop {
            if request {
                self.write_mem32(DHCSR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT)
                    .context("DHCSR halt write failed")?;
            }
            let status = self.read_mem32(DHCSR).context("DHCSR status read failed")?;
            if status & DHCSR_S_HALT != 0 {
                return Ok(());
            }
            request = status & DHCSR_S_RESET_ST != 0;
            if started.elapsed() >= HALT_TIMEOUT {
                bail!("core halt not acknowledged (DHCSR S_HALT) within {HALT_TIMEOUT:?}");
            }
        }
    }

    /// Request a system reset; the reboot usually drops the link, so a transfer error is expected.
    pub fn system_reset(&mut self) -> Result<()> {
        self.write_mem32(AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ)?;
        // A posted SWD write commits on later clock edges; this read clocks it out before exit.
        let _ = self.read_dp(DP_A_CTRL_STAT);
        Ok(())
    }

    /// Read `out.len()` words from word-aligned `addr`, split at wrap and packet boundaries.
    /// Each block retries a WAIT up to eight times before the call fails.
    pub fn read_block(&mut self, addr: u32, out: &mut [u32]) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("read_block address 0x{addr:08X} is not word-aligned");
        }
        self.ensure_csw()?;
        let ops = self.plan(addr, out.len(), true);
        if self.shape == TransferShape::Mixed {
            return self.serial_mixed(&ops, Some(out), None);
        }
        let mut done = 0;
        for op in ops {
            if op.tar {
                self.write_tar(op.addr)?;
            }
            // A WAIT retry re-reads from the AP's own TAR, so per-window must re-arm it first.
            let retry_tar = (!op.tar).then_some(op.addr);
            self.read_block_raw(&mut out[done..done + op.words], retry_tar)?;
            done += op.words;
        }
        Ok(())
    }

    /// Write `data` to word-aligned `addr`, chunked exactly as in [`Adiv5::read_block`].
    pub fn write_block(&mut self, addr: u32, data: &[u32]) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("write_block address 0x{addr:08X} is not word-aligned");
        }
        self.ensure_csw()?;
        let ops = self.plan(addr, data.len(), false);
        if self.shape == TransferShape::Mixed {
            return self.serial_mixed(&ops, None, Some(data));
        }
        let mut done = 0;
        for op in ops {
            if op.tar {
                self.write_tar(op.addr)?;
            }
            let retry_tar = (!op.tar).then_some(op.addr);
            self.write_block_raw(&data[done..done + op.words], retry_tar)?;
            done += op.words;
        }
        Ok(())
    }

    /// Read with up to `depth` pairs outstanding; a `WAIT` fails the call, since a retry would desync.
    pub fn read_block_depth(&mut self, addr: u32, out: &mut [u32], depth: usize) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("read_block address 0x{addr:08X} is not word-aligned");
        }
        if depth <= 1 {
            return self.read_block(addr, out);
        }
        self.ensure_csw()?;
        let ops = self.plan(addr, out.len(), true);
        if self.shape == TransferShape::Mixed {
            return self.pipelined_mixed(&ops, Some(out), None, depth);
        }
        let (requests, expected_lens) = build_block_requests(&ops, None, self.dap_index);
        let responses =
            self.probe
                .submit_pipelined(&requests, &expected_lens, depth, PIPELINE_TIMEOUT)?;

        let explain = |pos: usize, e| diagnose(&requests[pos], &responses[pos], e);
        let (mut done, mut pos) = (0, 0);
        for op in &ops {
            if op.tar {
                check_pipelined_tar(&responses[pos]).map_err(|e| explain(pos, e))?;
                pos += 1;
            }
            let words =
                check_pipelined_block(&responses[pos], op.words).map_err(|e| explain(pos, e))?;
            if words.len() != op.words {
                return Err(explain(pos, anyhow!("DAP_TransferBlock read lost words")));
            }
            pos += 1;
            out[done..done + op.words].copy_from_slice(&words);
            done += op.words;
        }
        Ok(())
    }

    /// Write with up to `depth` pairs outstanding, as [`Adiv5::read_block_depth`].
    pub fn write_block_depth(&mut self, addr: u32, data: &[u32], depth: usize) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("write_block address 0x{addr:08X} is not word-aligned");
        }
        if depth <= 1 {
            return self.write_block(addr, data);
        }
        self.ensure_csw()?;
        let ops = self.plan(addr, data.len(), false);
        if self.shape == TransferShape::Mixed {
            return self.pipelined_mixed(&ops, None, Some(data), depth);
        }
        let (requests, expected_lens) = build_block_requests(&ops, Some(data), self.dap_index);
        let responses =
            self.probe
                .submit_pipelined(&requests, &expected_lens, depth, PIPELINE_TIMEOUT)?;

        let explain = |pos: usize, e| diagnose(&requests[pos], &responses[pos], e);
        let mut pos = 0;
        for op in &ops {
            if op.tar {
                check_pipelined_tar(&responses[pos]).map_err(|e| explain(pos, e))?;
                pos += 1;
            }
            check_pipelined_block(&responses[pos], op.words).map_err(|e| explain(pos, e))?;
            pos += 1;
        }
        Ok(())
    }

    /// Wake a DP that powers up in the ADIv6 dormant state.
    fn dormant_wake(&mut self) -> Result<()> {
        let ones = [0xFFu8; 8];
        self.swj_sequence(51, &ones)?;
        self.swj_sequence(31, &SWD_TO_DORMANT)?;
        self.swj_sequence(8, &ones[..1])?;
        self.swj_sequence(64, &SELECTION_ALERT_LO.to_le_bytes())?;
        self.swj_sequence(64, &SELECTION_ALERT_HI.to_le_bytes())?;
        self.swj_sequence(12, &SWD_ACTIVATION.to_le_bytes())?;
        Ok(())
    }

    /// Single-drop SWD entry: line reset, JTAG-to-SWD select, line reset, idle clocks.
    fn line_reset_switch(&mut self) -> Result<()> {
        self.swj_sequence(56, &[0xFF; 7])?;
        self.swj_sequence(16, &JTAG_TO_SWD_SELECT.to_le_bytes())?;
        self.swj_sequence(56, &[0xFF; 7])?;
        self.swj_sequence(8, &[0x00])?;
        Ok(())
    }

    /// JTAG entry: line reset, SWD-to-JTAG select, then TAP reset into Run-Test/Idle.
    fn jtag_entry(&mut self) -> Result<()> {
        self.swj_sequence(56, &[0xFF; 7])?;
        self.swj_sequence(16, &SWD_TO_JTAG_SELECT.to_le_bytes())?;
        self.swj_sequence(JTAG_TAP_RESET_IDLE_BITS, &JTAG_TAP_RESET_IDLE)?;
        Ok(())
    }

    /// Entry sequence plus the DPIDR read, retried as a pair on a non-`OK` ack.
    fn read_dpidr(&mut self) -> Result<u32> {
        let mut last: Option<anyhow::Error> = None;
        for attempt in 0..DPIDR_READ_ATTEMPTS {
            self.line_reset_switch()?;
            match self.read_dp(DP_A_DPIDR) {
                Ok(v) => return Ok(v),
                Err(e) => {
                    last = Some(e.context(format!("DPIDR read attempt {}", attempt + 1)));
                }
            }
        }
        Err(last.unwrap_or_else(|| anyhow!("DPIDR read failed")))
    }

    /// `TARGETSEL` plus the confirming DPIDR read, retried as a pair.
    fn read_dpidr_targetsel(&mut self, targetsel: u32) -> Result<u32> {
        let mut last: Option<anyhow::Error> = None;
        for attempt in 0..DPIDR_READ_ATTEMPTS {
            self.select_target(targetsel)?;
            match self.read_dp(DP_A_DPIDR) {
                Ok(v) => return Ok(v),
                Err(e) => {
                    last = Some(e.context(format!("DPIDR read attempt {}", attempt + 1)));
                }
            }
        }
        Err(last.unwrap_or_else(|| anyhow!("DPIDR read failed")))
    }

    /// Dormant wake, optional `TARGETSEL`, then the DPIDR read; every attempt re-issues the wake.
    fn read_dpidr_dormant(&mut self, targetsel: Option<u32>) -> Result<u32> {
        let mut last: Option<anyhow::Error> = None;
        for attempt in 0..DPIDR_READ_ATTEMPTS {
            self.dormant_wake()?;
            match targetsel {
                Some(value) => self.select_target(value)?,
                // The DPIDR packet still needs the framing line reset `select_target` embeds.
                None => {
                    self.swj_sequence(LINE_RESET_TERMINATED_BITS, &LINE_RESET_TERMINATED)?;
                }
            }
            match self.read_dp(DP_A_DPIDR) {
                Ok(v) => return Ok(v),
                Err(e) => {
                    last = Some(e.context(format!("ADIv6 DPIDR read attempt {}", attempt + 1)));
                }
            }
        }
        Err(last.unwrap_or_else(|| anyhow!("DPIDR read failed")))
    }

    /// Select one multidrop DP: terminated line reset and `TARGETSEL` as one host-driven sequence.
    fn select_target(&mut self, targetsel: u32) -> Result<()> {
        self.swj_sequence(LINE_RESET_TERMINATED_BITS, &LINE_RESET_TERMINATED)?;
        self.swj_sequence(48, &targetsel_sequence(targetsel))?;
        Ok(())
    }

    /// Request debug + system power-up via CTRL/STAT and poll until both acknowledges are set.
    fn power_up(&mut self) -> Result<()> {
        self.ensure_select(AP_BANK_CORE)?;
        self.write_dp(DP_A_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)
            .context("CTRL/STAT power-up request failed")?;
        for _ in 0..POWERUP_POLLS {
            let stat = self.read_dp(DP_A_CTRL_STAT)?;
            if stat & (CSYSPWRUPACK | CDBGPWRUPACK) == (CSYSPWRUPACK | CDBGPWRUPACK) {
                return Ok(());
            }
        }
        bail!("debug power-up not acknowledged within {POWERUP_POLLS} polls")
    }

    /// Clear every sticky error flag; the JTAG arm bypasses [`Adiv5::write_dp`] to avoid recursion.
    fn clear_sticky_errors(&mut self) -> Result<()> {
        match self.wire {
            Port::Jtag => {
                let req = encode_transfer(
                    self.dap_index,
                    &[TransferItem {
                        request: TransferRequest::new(false, false, DP_A_CTRL_STAT),
                        data: Some(CSYSPWRUPREQ | CDBGPWRUPREQ | CTRL_STAT_STICKY_W1C),
                    }],
                );
                let resp = self
                    .probe
                    .transfer(&req, TRANSFER_TIMEOUT)
                    .context("CTRL/STAT sticky-clear USB round-trip failed")?;
                let decoded = decode_transfer(&resp)
                    .context("decoding CTRL/STAT sticky-clear response failed")?;
                if !decoded.status.is_ok() {
                    bail!(
                        "CTRL/STAT sticky-clear write not acknowledged ({})",
                        decoded.status
                    );
                }
                Ok(())
            }
            Port::Swd => probe_command(
                self.probe,
                encode_write_abort(self.dap_index, ABORT_CLEAR_ALL),
                decode_write_abort,
            ),
        }
    }

    /// Read the MEM-AP IDR (bank `0xF`, offset `0xFC`) for the report header.
    fn read_idr(&mut self) -> Result<ApIdr> {
        let raw = self.read_ap(AP_BANK_IDR, AP_A_IDR)?;
        Ok(ApIdr { raw })
    }

    /// Point DP `SELECT` at the AP register window `apbank`, rewriting only on a change.
    fn ensure_select(&mut self, apbank: u8) -> Result<()> {
        match self.ap {
            ApMode::V5 { apsel } => {
                let want = select_word(apsel, apbank);
                if self.select != Some(want) {
                    self.write_dp(DP_A_SELECT, want)?;
                    self.select = Some(want);
                }
                Ok(())
            }
            ApMode::V6 { .. } => self.ensure_select_v6(ap_v6_frame_offset(apbank)),
        }
    }

    /// Point `SELECT`/`SELECT1` at the ADIv6 AP register at frame offset `off`.
    fn ensure_select_v6(&mut self, off: u32) -> Result<()> {
        let plan = match &self.ap {
            ApMode::V6 { ap_base, sel } => sel.plan(u64::from(*ap_base) + u64::from(off)),
            ApMode::V5 { .. } => bail!("ADIv6 SELECT requested on an ADIv5 attach"),
        };
        if let Some(select1) = plan.write_select1 {
            self.write_dp(DP_A_SELECT, DP_BANKSEL_SELECT1)?;
            self.write_dp(DP_A_CTRL_STAT, select1)?;
        }
        if let Some(select) = plan.write_select {
            self.write_dp(DP_A_SELECT, select)?;
        }
        if let ApMode::V6 { sel, .. } = &mut self.ap {
            sel.commit(&plan);
        }
        Ok(())
    }

    /// Ensure the MEM-AP CSW is programmed for 32-bit auto-incrementing transfers.
    fn ensure_csw(&mut self) -> Result<()> {
        if self.csw != Some(CSW_WORD_ADDRINC) {
            self.write_ap(AP_BANK_CORE, AP_A_CSW, CSW_WORD_ADDRINC)?;
            self.csw = Some(CSW_WORD_ADDRINC);
        }
        Ok(())
    }

    /// Write the MEM-AP TAR (transfer address register).
    fn write_tar(&mut self, addr: u32) -> Result<()> {
        self.write_ap(AP_BANK_CORE, AP_A_TAR, addr)
    }

    /// Read a DP register at transfer address `a`; DP reads are not posted.
    fn read_dp(&mut self, a: u8) -> Result<u32> {
        let data = self.raw_transfer(&[TransferItem {
            request: TransferRequest::new(false, true, a),
            data: None,
        }])?;
        data.first()
            .copied()
            .ok_or_else(|| anyhow!("DP read returned no data word"))
    }

    /// Write `val` to a DP register at transfer address `a`.
    fn write_dp(&mut self, a: u8, val: u32) -> Result<()> {
        self.raw_transfer(&[TransferItem {
            request: TransferRequest::new(false, false, a),
            data: Some(val),
        }])?;
        Ok(())
    }

    /// Read a MEM-AP register at bank `apbank` / transfer address `a`.
    fn read_ap(&mut self, apbank: u8, a: u8) -> Result<u32> {
        self.ensure_select(apbank)?;
        let data = self.raw_transfer(&[TransferItem {
            request: TransferRequest::new(true, true, a),
            data: None,
        }])?;
        data.first()
            .copied()
            .ok_or_else(|| anyhow!("AP read returned no data word"))
    }

    /// Write `val` to a MEM-AP register at bank `apbank` / transfer address `a`.
    fn write_ap(&mut self, apbank: u8, a: u8, val: u32) -> Result<()> {
        self.ensure_select(apbank)?;
        self.raw_transfer(&[TransferItem {
            request: TransferRequest::new(true, false, a),
            data: Some(val),
        }])?;
        Ok(())
    }

    /// Shift a raw bit sequence out through `DAP_SWJ_Sequence`.
    fn swj_sequence(&mut self, bit_count: u8, data: &[u8]) -> Result<()> {
        probe_command(
            self.probe,
            encode_swj_sequence(bit_count, data),
            decode_swj_sequence,
        )
    }

    /// One `DAP_Transfer`, retried whole on `WAIT` and clearing sticky errors on `FAULT`.
    fn raw_transfer(&mut self, items: &[TransferItem]) -> Result<Vec<u32>> {
        for _ in 0..=WAIT_RETRIES {
            let req = encode_transfer(self.dap_index, items);
            let resp = self
                .probe
                .transfer(&req, TRANSFER_TIMEOUT)
                .context("DAP_Transfer USB round-trip failed")?;
            let decoded =
                decode_transfer(&resp).context("decoding DAP_Transfer response failed")?;
            let status = decoded.status;
            if status.is_ok() && decoded.transfer_count as usize == items.len() {
                return Ok(decoded.read_data);
            }
            check_reply(&req, &resp).context("DAP_Transfer reply does not fit its request")?;
            match status.ack() {
                Ack::Wait => continue,
                Ack::Fault => {
                    let _ = self.clear_sticky_errors();
                    bail!(
                        "DAP_Transfer faulted after {} of {} transfers",
                        decoded.transfer_count,
                        items.len()
                    );
                }
                _ => bail!(
                    "DAP_Transfer failed: {status} after {} of {} transfers",
                    decoded.transfer_count,
                    items.len()
                ),
            }
        }
        bail!("DAP_Transfer WAIT retries exhausted")
    }

    /// One `DAP_TransferBlock` read of DRW (TAR pre-armed); `out.len()` must fit one packet.
    fn read_block_raw(&mut self, out: &mut [u32], retry_tar: Option<u32>) -> Result<()> {
        let request = TransferRequest::new(true, true, AP_A_DRW);
        for attempt in 0..=WAIT_RETRIES {
            if attempt > 0 {
                if let Some(addr) = retry_tar {
                    self.write_tar(addr)?;
                }
            }
            let req = encode_transfer_block(self.dap_index, out.len() as u16, request, &[]);
            let resp = self
                .probe
                .transfer_expect(&req, 4 + 4 * out.len(), TRANSFER_TIMEOUT)
                .context("DAP_TransferBlock (read) USB round-trip failed")?;
            let decoded = decode_transfer_block(&resp)
                .context("decoding DAP_TransferBlock response failed")?;
            let complete = decoded.transfer_count as usize == out.len();
            if decoded.status.is_ok() && complete && decoded.read_data.len() == out.len() {
                out.copy_from_slice(&decoded.read_data);
                return Ok(());
            }
            check_reply(&req, &resp).context("DAP_TransferBlock reply does not fit its request")?;
            let status = decoded.status;
            match status.ack() {
                Ack::Wait => continue,
                Ack::Fault => {
                    let _ = self.clear_sticky_errors();
                    bail!(
                        "DAP_TransferBlock (read) faulted after {} words",
                        decoded.transfer_count
                    );
                }
                _ => bail!("DAP_TransferBlock (read) failed: {status}"),
            }
        }
        bail!("DAP_TransferBlock (read) WAIT retries exhausted")
    }

    /// One `DAP_TransferBlock` write of DRW (TAR pre-armed); `data.len()` must fit one packet.
    fn write_block_raw(&mut self, data: &[u32], retry_tar: Option<u32>) -> Result<()> {
        let request = TransferRequest::new(true, false, AP_A_DRW);
        for attempt in 0..=WAIT_RETRIES {
            if attempt > 0 {
                if let Some(addr) = retry_tar {
                    self.write_tar(addr)?;
                }
            }
            let req = encode_transfer_block(self.dap_index, data.len() as u16, request, data);
            let resp = self
                .probe
                .transfer_expect(&req, 4, TRANSFER_TIMEOUT)
                .context("DAP_TransferBlock (write) USB round-trip failed")?;
            let decoded = decode_transfer_block(&resp)
                .context("decoding DAP_TransferBlock response failed")?;
            if decoded.status.is_ok() && decoded.transfer_count as usize == data.len() {
                return Ok(());
            }
            check_reply(&req, &resp).context("DAP_TransferBlock reply does not fit its request")?;
            let status = decoded.status;
            match status.ack() {
                Ack::Wait => continue,
                Ack::Fault => {
                    let _ = self.clear_sticky_errors();
                    bail!(
                        "DAP_TransferBlock (write) faulted after {} words",
                        decoded.transfer_count
                    );
                }
                _ => bail!("DAP_TransferBlock (write) failed: {status}"),
            }
        }
        bail!("DAP_TransferBlock (write) WAIT retries exhausted")
    }
}

/// Connect, set clock and retry budget; returns the packet size, or the minimum when zero.
fn connect_and_configure(
    probe: &mut Probe,
    want_port: Port,
    clock_hz: u32,
    wait_retry: u16,
    match_retry: u16,
) -> Result<u16> {
    let port = probe_command(probe, encode_connect(want_port), decode_connect)
        .with_context(|| format!("DAP_Connect({want_port:?}) failed"))?;
    if port != Some(want_port) {
        bail!(
            "probe did not select {want_port:?} (DAP_Connect returned {port:?}; a probe that \
             does not advertise the port in its capabilities refuses it)"
        );
    }
    probe_command(probe, encode_swj_clock(clock_hz), decode_swj_clock)
        .context("DAP_SWJ_Clock failed")?;
    probe_command(
        probe,
        encode_transfer_configure(TransferConfigureRequest {
            idle_cycles: 0,
            wait_retry,
            match_retry,
        }),
        decode_transfer_configure,
    )
    .context("DAP_TransferConfigure failed")?;

    let packet_size = probe_command(
        probe,
        encode_info(InfoId::PacketSize),
        decode_info_packet_size,
    )
    .context("DAP_Info(PacketSize) failed")?;
    Ok(if packet_size == 0 {
        FALLBACK_PACKET_SIZE
    } else {
        packet_size
    })
}

/// The `DAP_SWJ_Sequence` payload for a multidrop `TARGETSEL` write, 46 bits padded to 48.
fn targetsel_sequence(targetsel: u32) -> [u8; 6] {
    let parity = u64::from(targetsel.count_ones() & 1);
    let seq = (parity << 45)
        | (u64::from(targetsel) << 13)
        | (TARGETSEL_ACK_WINDOW_HIGH << 8)
        | u64::from(TARGETSEL_REQUEST);
    seq.to_le_bytes()[..6].try_into().unwrap()
}

/// Send one framed CMSIS-DAP command and decode its response.
fn probe_command<T, E>(
    probe: &mut Probe,
    request: Vec<u8>,
    decode: impl Fn(&[u8]) -> Result<T, E>,
) -> Result<T>
where
    E: std::error::Error + Send + Sync + 'static,
{
    let resp = probe
        .transfer(&request, TRANSFER_TIMEOUT)
        .context("CMSIS-DAP command USB round-trip failed")?;
    let value = decode(&resp).context("decoding CMSIS-DAP response failed")?;
    check_reply(&request, &resp).context("CMSIS-DAP response does not fit its request")?;
    Ok(value)
}

#[cfg(test)]
mod tests {

    #[test]
    fn sticky_clear_leaves_dapabort_and_stkcmpclr_clear() {
        assert_eq!(ABORT_CLEAR_ALL, 0x1C);
    }

    /// ADIv6 relocates CSW/TAR/DRW to one `0xD00`-page SELECT window, IDR to its own.
    #[test]
    fn split_ap_addr_v6_places_select_select1_and_a() {
        assert_eq!(split_ap_addr_v6(0x2D00), (0x2D00, 0, 0)); // CSW
        assert_eq!(split_ap_addr_v6(0x2D04), (0x2D00, 0, 1)); // TAR
        assert_eq!(split_ap_addr_v6(0x2D0C), (0x2D00, 0, 3)); // DRW
        assert_eq!(split_ap_addr_v6(0x2DFC), (0x2DF0, 0, 3)); // IDR
        assert_eq!(split_ap_addr_v6(0x0001_2345_6788), (0x2345_6780, 0x1, 2));
    }

    /// The ADIv6 frame offsets and the ADIv5 encoding of the same AP must differ.
    #[test]
    fn adiv6_frame_offsets_and_their_disagreement_with_adiv5() {
        assert_eq!(ap_v6_frame_offset(AP_BANK_CORE), 0xD00);
        assert_eq!(ap_v6_frame_offset(AP_BANK_IDR), 0xDFC);
        assert_ne!(select_word(0x20, AP_BANK_CORE), 0x2D00);
    }

    /// The RP `TARGETSEL` literals in `target.rs` follow this derivation.
    #[test]
    fn derive_targetsel_matches_the_rp2040_core0_vector() {
        // TARGETID's TREVISION and DLPIDR's PROTVSN are seeded nonzero to prove they are masked.
        let targetid = 0xE100_2927;
        assert_eq!(derive_targetsel(targetid, 0x0000_000F), 0x0100_2927);
        // A nonzero TINSTANCE lands in the top nibble.
        assert_eq!(derive_targetsel(targetid, 0x3000_0001), 0x3100_2927);
    }

    #[test]
    fn dp_ident_decodes_targetid_and_dlpidr_fields() {
        let ident = DpIdent {
            targetid: 0x0100_2927,
            dlpidr: 0x0000_0001,
        };
        assert_eq!(ident.designer(), 0x493, "JEP106 Raspberry Pi");
        assert_eq!(ident.partno(), 0x1002);
        assert_eq!(ident.revision(), 0x0);
        assert_eq!(ident.protvsn(), 0x1);
        assert_eq!(ident.tinstance(), 0x0);
        assert_eq!(ident.targetsel(), 0x0100_2927);

        let ident = DpIdent {
            targetid: 0x5ABC_D235,
            dlpidr: 0x7000_0003,
        };
        assert_eq!(ident.revision(), 0x5);
        assert_eq!(ident.partno(), 0xABCD);
        assert_eq!(ident.designer(), (0xD235 >> 1) & 0x7FF);
        assert_eq!(ident.tinstance(), 0x7);
        assert_eq!(ident.targetsel(), 0x7ABC_D235);
    }

    /// The RP2350 values the tool's own page quotes, and the DPv2 gate that reads them.
    #[test]
    fn dp_ident_reproduces_the_rp2350_targetsel_and_is_gated_on_dpv2() {
        let ident = DpIdent {
            targetid: 0x0004_0927,
            dlpidr: 0x0000_0001,
        };
        assert_eq!(ident.targetsel(), 0x0004_0927);
        assert_eq!(Dpidr { raw: 0x0BE1_2477 }.version(), 2);
        assert_eq!(Dpidr { raw: 0x0BC1_1477 }.version(), 1);
    }

    #[test]
    fn dp_ident_bank_and_mask_constants_partition_the_word() {
        // Both registers sit at the CTRL/STAT slot and differ only by DPBANKSEL.
        assert_eq!(DP_BANKSEL_TARGETID, 0x2);
        assert_eq!(DP_BANKSEL_DLPIDR, 0x3);
        assert_eq!(DP_BANKSEL_TARGETID & !0xF, 0);
        assert_eq!(DP_BANKSEL_DLPIDR & !0xF, 0);
        assert_eq!(TARGETID_TARGETSEL_MASK | DLPIDR_TINSTANCE_MASK, 0xFFFF_FFFF);
        assert_eq!(TARGETID_TARGETSEL_MASK & DLPIDR_TINSTANCE_MASK, 0);
    }

    /// Hardware check of the ADIv6 constants: `DAPBENCH_PROBE=vid:pid:serial cargo test -- --ignored`,
    /// with the target power-cycled so the DP starts dormant.
    #[test]
    #[ignore = "requires a CMSIS-DAP v2 probe wired to a power-cycled RP2350 / Pico 2"]
    fn attach_v6_against_hardware() {
        let profile = crate::target::lookup("rp2350").expect("the rp2350 profile");
        let probes = crate::transport::enumerate();
        let info = crate::transport::select_from_env(&probes)
            .expect("a probe named by DAPBENCH_PROBE, or exactly one attached")
            .clone();
        let mut probe = Probe::open(info).expect("open probe");
        let mut adi = Adiv5::attach(
            &mut probe,
            AttachConfig {
                targetsel: profile.targetsel,
                ap: profile.ap,
                ..AttachConfig::default()
            },
        )
        .expect("ADIv6 attach");
        println!("DPIDR  {}", adi.dpidr());
        println!("AP IDR {}", adi.ap_idr());
        assert_ne!(adi.ap_idr().raw, 0, "an AP IDR of zero is not an AP");
        let ident = adi.dp_ident().expect("a DPv2+ DP carries TARGETID/DLPIDR");
        println!("DP     {ident}");
        assert_eq!(
            Some(ident.targetsel()),
            profile.targetsel,
            "the profile's hardcoded TARGETSEL must be what this silicon answers to"
        );
        let addr = profile.sram_base;
        let before = adi.read_mem32(addr).expect("SRAM read");
        adi.write_mem32(addr, before ^ 0xA5A5_5A5A)
            .expect("SRAM write");
        assert_eq!(
            adi.read_mem32(addr).expect("read-back"),
            before ^ 0xA5A5_5A5A
        );
        adi.write_mem32(addr, before).expect("restore");
    }

    /// `SELECT1` is written once per attach, and its change forces the `SELECT` rewrite.
    #[test]
    fn select_v6_plans_both_halves_first_then_caches() {
        let mut sel = SelectV6::new();
        let first = sel.plan(0x2D00);
        assert_eq!(first.write_select1, Some(0));
        assert_eq!(first.write_select, Some(0x2D00));
        sel.commit(&first);
        let same = sel.plan(0x2D04);
        assert_eq!(same.write_select1, None);
        assert_eq!(same.write_select, None);
        assert_eq!(same.a, 1);
        let upper = sel.plan(0x0001_0000_2D00);
        assert_eq!(upper.write_select1, Some(1));
        assert_eq!(upper.write_select, Some(0x2D00));
    }
    use super::*;

    #[test]
    fn select_word_packs_apsel_and_apbank() {
        assert_eq!(select_word(0x01, 0xF), 0x0100_00F0);
        assert_eq!(select_word(0x00, 0x0), 0x0000_0000);
    }

    #[test]
    fn words_to_wrap_stops_at_1kib_boundary() {
        assert_eq!(words_to_wrap(0x0800_0000), 256);
        assert_eq!(words_to_wrap(0x0800_0000 + 4), 255);
        assert_eq!(words_to_wrap(0x0800_0000 + 1020), 1);
    }

    #[test]
    fn block_chunk_words_is_bounded_by_wrap_and_capacity() {
        // Aligned start, plenty remaining, small packet cap: capped by capacity.
        assert_eq!(block_chunk_words(0x0800_0000, 1000, 15), 15);
        // Near the wrap boundary: capped by the wrap, even under a larger capacity.
        assert_eq!(block_chunk_words(0x0800_0000 + 1016, 1000, 255), 2);
    }

    #[test]
    fn targetsel_sequence_matches_the_bench_captured_bytes() {
        // RP2040 core-0 TARGETSEL.
        assert_eq!(
            targetsel_sequence(0x0100_2927),
            [0x99, 0xFF, 0x24, 0x05, 0x20, 0x00]
        );
    }

    #[test]
    fn plan_chunks_splits_at_wrap_and_capacity() {
        // 40 words at 15 words/packet: capacity, then a short chunk at the wrap, then capacity.
        let chunks = plan_chunks(0x2000_0390, 40, 15);
        assert_eq!(
            chunks,
            vec![
                (0x2000_0390, 15),
                (0x2000_03CC, 13), // cut short at the 1 KiB boundary (0x2000_0400)
                (0x2000_0400, 12),
            ]
        );
        assert_eq!(chunks.iter().map(|&(_, n)| n).sum::<usize>(), 40);
    }

    #[test]
    fn plan_chunks_empty_range_yields_no_chunks() {
        assert!(plan_chunks(0x2000_0000, 0, 15).is_empty());
    }

    #[test]
    fn plan_chunks_matches_read_block_lockstep_chunking() {
        // Pipelined and lockstep runs must touch the same addresses in the same order.
        for &(addr, words, cap) in &[
            (0x0800_0000u32, 1024usize, 15usize),
            (0x0800_0104, 1000, 127),
            (0x0800_03FC, 3, 254),
        ] {
            let chunks = plan_chunks(addr, words, cap);
            let mut cur = addr;
            let mut total = 0;
            for &(chunk_addr, n) in &chunks {
                assert_eq!(chunk_addr, cur, "chunks must be contiguous");
                assert!(n >= 1 && n <= cap);
                assert!(n <= words_to_wrap(chunk_addr));
                cur = cur.wrapping_add((n * 4) as u32);
                total += n;
            }
            assert_eq!(total, words);
        }
    }

    #[test]
    fn per_window_reads_cost_76_commands_per_aligned_4_kib() {
        // 4 KiB read at 15 words/block: 18 blocks in each of 4 windows, one TAR each.
        let ops = plan_ops(0x0800_0000, 1024, 15, TarPolicy::PerWindow);
        assert_eq!(ops.len(), 72);
        assert_eq!(ops.iter().filter(|o| o.tar).count(), 4);
        assert_eq!(
            command_count(0x0800_0000, 1024, 15, TarPolicy::PerWindow),
            76
        );
        assert_eq!(
            command_count(0x0800_0000, 1024, 15, TarPolicy::PerChunk),
            144
        );
    }

    #[test]
    fn per_window_writes_cost_80_commands_per_aligned_4_kib() {
        // 14 words/block: 19 blocks per window (18 full plus a 4-word tail), one TAR each.
        let ops = plan_ops(0x2000_0000, 1024, 14, TarPolicy::PerWindow);
        assert_eq!(ops.len(), 76);
        assert_eq!(ops.iter().filter(|o| o.tar).count(), 4);
        assert_eq!(ops.iter().map(|o| o.words).sum::<usize>(), 1024);
        assert_eq!(
            command_count(0x2000_0000, 1024, 14, TarPolicy::PerWindow),
            80
        );
        assert_eq!(
            command_count(0x2000_0000, 1024, 14, TarPolicy::PerChunk),
            152
        );
    }

    #[test]
    fn per_window_tar_marks_exactly_the_window_starts() {
        // Unaligned start: the first block re-arms TAR because it is first, the rest only at a
        // 1 KiB boundary, and no block spans one.
        let ops = plan_ops(0x0800_0390, 1024, 15, TarPolicy::PerWindow);
        assert!(ops[0].tar);
        let mut cur = 0x0800_0390u32;
        for (i, op) in ops.iter().enumerate() {
            assert_eq!(op.addr, cur, "op {i} must be contiguous");
            let starts_window = (op.addr as usize).is_multiple_of(WRAP_BYTES);
            assert_eq!(op.tar, i == 0 || starts_window, "op {i} TAR placement");
            assert!(
                op.words <= words_to_wrap(op.addr),
                "op {i} crosses a window"
            );
            cur = cur.wrapping_add((op.words * 4) as u32);
        }
        // Five windows are touched from an unaligned start, so five TAR writes.
        assert_eq!(ops.iter().filter(|o| o.tar).count(), 5);
    }

    #[test]
    fn per_window_and_per_chunk_plan_the_same_blocks() {
        for &(addr, words, cap) in &[
            (0x0800_0000u32, 1024usize, 15usize),
            (0x0800_0390, 1024, 15),
            (0x2000_0004, 7, 14),
            (0x2000_03FC, 1, 15),
        ] {
            let chunked = plan_ops(addr, words, cap, TarPolicy::PerChunk);
            let windowed = plan_ops(addr, words, cap, TarPolicy::PerWindow);
            assert_eq!(chunked.len(), windowed.len());
            for (a, b) in chunked.iter().zip(&windowed) {
                assert_eq!((a.addr, a.words), (b.addr, b.words));
            }
            assert!(chunked.iter().all(|o| o.tar));
        }
    }

    #[test]
    fn a_short_per_window_transfer_writes_tar_once() {
        // A tail that fits one window: one TAR, however many blocks it takes.
        let ops = plan_ops(0x2000_0000, 40, 15, TarPolicy::PerWindow);
        assert_eq!(
            ops.iter().map(|o| o.words).collect::<Vec<_>>(),
            vec![15, 15, 10]
        );
        assert_eq!(ops.iter().filter(|o| o.tar).count(), 1);
        assert_eq!(command_count(0x2000_0000, 40, 15, TarPolicy::PerWindow), 4);
    }

    #[test]
    fn a_mixed_read_chunk_is_the_23_byte_request_and_63_byte_response() {
        let ops = plan_ops(
            0x0800_0000,
            15,
            mixed_read_capacity(64),
            TarPolicy::PerChunk,
        );
        assert_eq!(ops.len(), 1);
        let (requests, lens) = build_mixed_requests(&ops, None, 0);
        // Command id, dap index, item count, the five-byte TAR write, then one byte per read.
        assert_eq!(requests[0].len(), 23);
        assert_eq!(requests[0][2], 16);
        assert_eq!(lens[0], 63);
        // A write chunk is eleven items of five bytes behind the same TAR write.
        let data: Vec<u32> = (0..11).collect();
        let ops = plan_ops(
            0x2000_0000,
            11,
            mixed_write_capacity(64),
            TarPolicy::PerChunk,
        );
        let (requests, lens) = build_mixed_requests(&ops, Some(&data), 0);
        assert_eq!(requests[0].len(), 63);
        assert_eq!(requests[0][2], 12);
        assert_eq!(lens[0], 3);
    }

    #[test]
    fn a_mixed_4_kib_transfer_costs_one_command_per_chunk() {
        // Reads: 72 chunks either way, since the TAR write rides inside the chunk's command.
        for policy in [TarPolicy::PerChunk, TarPolicy::PerWindow] {
            assert_eq!(
                shaped_command_count(
                    0x0800_0000,
                    1024,
                    mixed_read_capacity(64),
                    policy,
                    TransferShape::Mixed
                ),
                72
            );
            // Writes carry eleven words a chunk, so 24 chunks per 1 KiB window.
            assert_eq!(
                shaped_command_count(
                    0x2000_0000,
                    1024,
                    mixed_write_capacity(64),
                    policy,
                    TransferShape::Mixed
                ),
                96
            );
        }
        assert_eq!(
            shaped_command_count(
                0x0800_0000,
                1024,
                TransferShape::Block.capacity(64, true),
                TarPolicy::PerChunk,
                TransferShape::Block
            ),
            144
        );
    }

    #[test]
    fn a_per_window_mixed_chunk_drops_only_its_tar_item() {
        let ops = plan_ops(
            0x2000_0000,
            30,
            mixed_read_capacity(64),
            TarPolicy::PerWindow,
        );
        let (requests, lens) = build_mixed_requests(&ops, None, 0);
        // Two chunks, both one command; only the first carries the TAR write item.
        assert_eq!(requests.len(), 2);
        assert_eq!((requests[0][2], requests[1][2]), (16, 15));
        assert_eq!(requests[1].len(), 18);
        assert_eq!(lens, vec![63, 63]);
    }

    #[test]
    fn per_window_requests_omit_the_repeated_tar_packets() {
        let ops = plan_ops(0x2000_0000, 30, 15, TarPolicy::PerWindow);
        let (requests, lens) = build_block_requests(&ops, None, 0);
        // TAR, block, block -- the second block rides the AP's auto-increment.
        assert_eq!(requests.len(), 3);
        assert_eq!(lens, vec![3, 4 + 4 * 15, 4 + 4 * 15]);
        assert_eq!(requests[1], vec![0x06, 0x00, 15, 0x00, 0b0000_1111]);
        assert_eq!(requests[2], vec![0x06, 0x00, 15, 0x00, 0b0000_1111]);
    }

    #[test]
    fn build_block_requests_read_interleaves_tar_and_block_with_exact_lens() {
        let ops = plan_ops(0x2000_0000, 30, 15, TarPolicy::PerChunk);
        assert_eq!(
            ops.iter().map(|o| o.addr).collect::<Vec<_>>(),
            vec![0x2000_0000, 0x2000_003C]
        );
        let (requests, lens) = build_block_requests(&ops, None, 0);
        assert_eq!(requests.len(), 4);
        assert_eq!(lens, vec![3, 4 + 4 * 15, 3, 4 + 4 * 15]);

        // Packet 0: DAP_Transfer, one AP write of TAR (A[3:2] = 0b01) carrying the address.
        assert_eq!(
            requests[0],
            vec![0x05, 0x00, 0x01, 0b0000_0101, 0x00, 0x00, 0x00, 0x20]
        );
        // Packet 1: DAP_TransferBlock, 15-word AP read of DRW (A[3:2] = 0b11).
        assert_eq!(requests[1], vec![0x06, 0x00, 15, 0x00, 0b0000_1111]);
        // Packet 2 re-arms TAR at the second chunk's address.
        assert_eq!(
            requests[2],
            vec![0x05, 0x00, 0x01, 0b0000_0101, 0x3C, 0x00, 0x00, 0x20]
        );
    }

    #[test]
    fn build_block_requests_write_slices_the_data_per_chunk() {
        let data: Vec<u32> = (0..20).collect();
        let ops = plan_ops(0x2000_0000, data.len(), 14, TarPolicy::PerChunk);
        assert_eq!(ops.iter().map(|o| o.words).collect::<Vec<_>>(), vec![14, 6]);
        let (requests, lens) = build_block_requests(&ops, Some(&data), 0);
        assert_eq!(requests.len(), 4);
        assert_eq!(lens, vec![3, 4, 3, 4]);

        // Block packets carry the right slice: 5 framing bytes + 4 per word, first word first.
        assert_eq!(requests[1].len(), 5 + 4 * 14);
        assert_eq!(&requests[1][5..9], &0u32.to_le_bytes());
        assert_eq!(requests[3].len(), 5 + 4 * 6);
        assert_eq!(&requests[3][5..9], &14u32.to_le_bytes());
        // Both block packets are AP writes of DRW.
        assert_eq!(requests[1][4], 0b0000_1101);
        assert_eq!(requests[3][4], 0b0000_1101);
    }

    #[test]
    fn build_block_requests_carries_the_dap_index() {
        // Over JTAG the DAP index byte selects the TAP, so every packet carries it (byte 1).
        let ops = plan_ops(0x2000_0000, 20, 15, TarPolicy::PerChunk);
        let (requests, _) = build_block_requests(&ops, None, 1);
        for (i, req) in requests.iter().enumerate() {
            assert_eq!(req[1], 1, "packet {i} DAP index");
        }
    }

    #[test]
    fn check_pipelined_responses_accept_a_clean_pair_and_reject_failures() {
        // Clean TAR write + clean 2-word block read.
        let tar_ok = [0x05, 0x01, 0x01];
        let block_ok = [
            0x06, 0x02, 0x00, 0x01, //
            0xAA, 0xAA, 0xAA, 0xAA, //
            0xBB, 0xBB, 0xBB, 0xBB,
        ];
        assert!(check_pipelined_tar(&tar_ok).is_ok());
        assert_eq!(
            check_pipelined_block(&block_ok, 2).unwrap(),
            vec![0xAAAA_AAAA, 0xBBBB_BBBB]
        );
        // A WAIT on the TAR write fails the chunk -- no retry inside a pipelined window.
        let tar_wait = [0x05, 0x00, 0b010];
        assert!(check_pipelined_tar(&tar_wait).is_err());
        // A short block (probe stopped early) fails the chunk.
        let block_short = [0x06, 0x01, 0x00, 0x01, 0xAA, 0xAA, 0xAA, 0xAA];
        assert!(check_pipelined_block(&block_short, 2).is_err());
    }
}
