//! ADIv5 SWD/JTAG attach and MEM-AP block read/write, owning all transfer chunking.

use std::fmt;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};

use crate::dap::{
    decode_connect, decode_info_packet_size, decode_jtag_configure, decode_jtag_idcode,
    decode_swj_clock, decode_swj_sequence, decode_transfer, decode_transfer_block,
    decode_transfer_configure, decode_write_abort, encode_connect, encode_info,
    encode_jtag_configure, encode_jtag_idcode, encode_swj_clock, encode_swj_sequence,
    encode_transfer, encode_transfer_block, encode_transfer_configure, encode_write_abort,
    transfer_block_read_capacity, transfer_block_write_capacity, Ack, InfoId, Port,
    TransferConfigureRequest, TransferItem, TransferRequest,
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

/// The SWD line-reset-and-select bit sequence value (`0xE79E`) that switches a JTAG-or-SWD
/// (SWJ) port into SWD mode.
pub const JTAG_TO_SWD_SELECT: u16 = 0xE79E;

// --- Dormant-state wake (ADIv6 parts such as the RP2350) ---
/// The two halves of the 128-bit CoreSight selection alert, LSB-first.
const SELECTION_ALERT_LO: u64 = 0x8685_2D95_6209_F392;
/// Upper half of the selection alert.
const SELECTION_ALERT_HI: u64 = 0x19BC_0EA2_E3DD_AFE9;
/// SWD-to-dormant sequence, sent before the alert so a DP already in SWD joins the dormant path.
const SWD_TO_DORMANT: [u8; 4] = [0xBA, 0xBB, 0xBB, 0x33];
/// 12-bit activation code selecting SWD out of dormant.
const SWD_ACTIVATION: u16 = 0x1A0;
/// The 16-bit select value (`0xE73C`) that switches an SWJ port from SWD back into JTAG mode,
/// the JTAG-direction counterpart of [`JTAG_TO_SWD_SELECT`].
pub const SWD_TO_JTAG_SELECT: u16 = 0xE73C;
/// JTAG TAP-reset-and-idle bit count: eight TMS-high into Test-Logic-Reset, four low into
/// Run-Test/Idle, where the CMSIS-DAP JTAG engine expects every scan to start.
const JTAG_TAP_RESET_IDLE_BITS: u8 = 12;
/// Data bytes for the JTAG TAP reset + idle (eight high then four low, LSB-first).
const JTAG_TAP_RESET_IDLE: [u8; 2] = [0xFF, 0x00];
/// IR bit length of the STM32F401 Cortex-M4 debug TAP (chain index 0, closest to TDO).
pub const STM32F401_CM_TAP_IR_LEN: u8 = 4;
/// IR bit length of the STM32F401 boundary-scan TAP (chain index 1, closest to TDI).
pub const STM32F401_BS_TAP_IR_LEN: u8 = 5;

// --- Multidrop TARGETSEL (Arm Debug Interface selection protocol), shifted out LSB-first
// through DAP_SWJ_Sequence.

/// SWD request byte for a DP `TARGETSEL` write: start=1, APnDP=0, RnW=0, A[3:2]=0b11, even
/// address parity=0, stop=0, park=1 -- LSB-first `0x99`.
const TARGETSEL_REQUEST: u8 = 0x99;
/// The 5 turnaround + ACK + turnaround bits of a `TARGETSEL` write, driven high: `TARGETSEL` has
/// no acknowledge, so the host never releases SWDIO for them.
const TARGETSEL_ACK_WINDOW_HIGH: u64 = 0x1F;
/// A line reset (51 SWDIO-high cycles, >= the 50 the protocol requires) terminated by three low
/// cycles, leaving the DP out of the reset state so it frames the next packet.
const LINE_RESET_TERMINATED_BITS: u8 = 54;
/// Data bytes for the terminated line reset (51 high then 3 low, LSB-first).
const LINE_RESET_TERMINATED: [u8; 7] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];

// --- DP register transfer addresses (the two-bit A[3:2] selector) ---
/// DPIDR (read) / ABORT (write) live at `A[3:2] = 0b00`.
const DP_A_DPIDR: u8 = 0;
/// CTRL/STAT lives at `A[3:2] = 0b01` (with `DPBANKSEL = 0`).
const DP_A_CTRL_STAT: u8 = 1;
/// SELECT (write) lives at `A[3:2] = 0b10`.
const DP_A_SELECT: u8 = 2;

// --- MEM-AP register transfer addresses (A[3:2]) and their banks ---
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

/// SWD ABORT write clearing every sticky error flag
/// (`ORUNERRCLR | WDERRCLR | STKERRCLR | STKCMPCLR | DAPABORT`).
const ABORT_CLEAR_ALL: u32 = 0x1E;

/// `CTRL/STAT` sticky error flags (`STICKYORUN | STICKYCMP | STICKYERR`); write-one-to-clear
/// over JTAG, whose ABORT scan implements only `DAPABORT`. Read-only over SWD.
const CTRL_STAT_STICKY_W1C: u32 = (1 << 1) | (1 << 4) | (1 << 5);

/// MEM-AP CSW value for the measured transfers: the standard Cortex-M debug `Prot`/`DbgSwEnable`
/// field (`0x23` in `[31:24]`) + `AddrInc = single(1 << 4)` + `Size = word(0b010)`.
const CSW_WORD_ADDRINC: u32 = 0x2300_0002 | (1 << 4);

/// MEM-AP auto-increment wrap boundary, in bytes. The AP TAR only guarantees auto-increment
/// within a 10-bit (1 KiB) window; crossing it requires a fresh TAR write.
pub const WRAP_BYTES: usize = 1024;

/// Default SWJ clock when the caller does not specify one.
pub const DEFAULT_CLOCK_HZ: u32 = 2_000_000;

/// Fallback `DAP_PACKET_SIZE` if the probe reports an unusable (zero) value.
const FALLBACK_PACKET_SIZE: u16 = 64;

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
    /// Multidrop `TARGETSEL` to select before the DPIDR read; `None` keeps the single-drop
    /// entry unchanged.
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
    /// The MEM-AP to select. JTAG attach is ADIv5-only, so this is a bare `APSEL`, not an
    /// [`ApMode`].
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

/// Compose an ADIv5 DP `SELECT` word from `apsel` (`SELECT[31:24]`) and `apbank`
/// (`APBANKSEL[7:4]`), leaving `DPBANKSEL[3:0] = 0`.
const fn select_word(apsel: u8, apbank: u8) -> u32 {
    ((apsel as u32) << 24) | (((apbank & 0xF) as u32) << 4)
}

/// Words from word-aligned `addr` to the next wrap boundary, in `1..=WRAP_BYTES/4`.
const fn words_to_wrap(addr: u32) -> usize {
    let into = (addr as usize) & (WRAP_BYTES - 1);
    (WRAP_BYTES - into) / 4
}

/// The next chunk length (in words) for a block transfer starting at `cur` with `remaining`
/// words left, bounded by the wrap boundary and the per-packet `max_words` capacity.
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

/// USB round-trip timeout for the packets [`Adiv5::read_block_depth`]/[`Adiv5::write_block_depth`]
/// submit directly.
const PIPELINE_TIMEOUT: Duration = Duration::from_millis(1000);

/// Split `words` at `addr` into the `(chunk_addr, chunk_words)` pieces [`Adiv5::read_block`]
/// walks, bounded by the wrap boundary and the per-packet capacity `cap`.
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

/// The `DAP_Transfer` response length in bytes for a single-item TAR write: command id,
/// transfer count, status (no read data).
const TAR_WRITE_RESPONSE_LEN: usize = 3;

/// The `DAP_TransferBlock` read response length in bytes for `words` data words (command id,
/// 2-byte count, status header, then `4*words` of payload).
const fn block_read_response_len(words: usize) -> usize {
    4 + 4 * words
}

/// The `DAP_TransferBlock` write response length in bytes: header only, no data words.
const fn block_write_response_len() -> usize {
    4
}

/// The pipelined packets over `chunks` -- a TAR write then a `DAP_TransferBlock` each, with
/// exact response lengths. Every chunk re-arms TAR; `data` is `None` for a read pass.
fn build_block_requests(
    chunks: &[(u32, usize)],
    data: Option<&[u32]>,
    dap_index: u8,
) -> (Vec<Vec<u8>>, Vec<usize>) {
    let mut requests = Vec::with_capacity(chunks.len() * 2);
    let mut expected_lens = Vec::with_capacity(chunks.len() * 2);
    let mut done = 0;
    for &(chunk_addr, n) in chunks {
        requests.push(encode_transfer(
            dap_index,
            &[TransferItem {
                request: TransferRequest::new(true, false, AP_A_TAR),
                data: Some(chunk_addr),
            }],
        ));
        expected_lens.push(TAR_WRITE_RESPONSE_LEN);

        match data {
            None => {
                requests.push(encode_transfer_block(
                    dap_index,
                    n as u16,
                    TransferRequest::new(true, true, AP_A_DRW),
                    &[],
                ));
                expected_lens.push(block_read_response_len(n));
            }
            Some(data) => {
                requests.push(encode_transfer_block(
                    dap_index,
                    n as u16,
                    TransferRequest::new(true, false, AP_A_DRW),
                    &data[done..done + n],
                ));
                expected_lens.push(block_write_response_len());
                done += n;
            }
        }
    }
    (requests, expected_lens)
}

/// Decode and check one pipelined chunk's response pair (TAR write, then block), returning the
/// block's read words (empty for a write).
fn check_pipelined_chunk(tar_resp: &[u8], block_resp: &[u8], words: usize) -> Result<Vec<u32>> {
    let tar = decode_transfer(tar_resp).context("decoding pipelined TAR write response failed")?;
    if !tar.status.is_ok() || tar.transfer_count != 1 {
        bail!("pipelined TAR write failed (ack {:?})", tar.status.ack());
    }
    let block = decode_transfer_block(block_resp)
        .context("decoding pipelined DAP_TransferBlock response failed")?;
    if !block.status.is_ok() || block.transfer_count as usize != words {
        bail!(
            "pipelined DAP_TransferBlock failed after {} of {words} words (ack {:?})",
            block.transfer_count,
            block.status.ack()
        );
    }
    Ok(block.read_data)
}

/// How an attach addresses the target access port. Encoding an ADIv6 AP the ADIv5 way reaches a
/// different register and reads plausible rubbish rather than failing.
#[derive(Debug, Clone, Copy)]
pub enum ApMode {
    /// ADIv5: `APSEL` in `SELECT[31:24]`, `APBANKSEL` in `SELECT[7:4]`.
    V5 {
        /// The MEM-AP index.
        apsel: u8,
    },
    /// ADIv6: a memory-mapped 4 KiB component frame reached by base address, its MEM-AP
    /// programmer's model relocated to the `0xD00` page.
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

/// ADIv6 MEM-AP core page: CSW/TAR/DRW at frame offsets `0xD00/0xD04/0xD0C`, so this is the
/// 16-byte SELECT window holding all three and `A[3:2]` picks one.
const AP_V6_OFF_CORE: u32 = 0xD00;
/// ADIv6 MEM-AP IDR byte offset within the AP's 4 KiB component frame (ADIv5 keeps `0xFC`).
const AP_V6_OFF_IDR: u32 = 0xDFC;
/// `DPBANKSEL` value exposing `SELECT1` at DP address `0x4` (the DPv3 register map's bank 5).
const DP_BANKSEL_SELECT1: u32 = 0x5;
/// `DPBANKSEL` exposing `TARGETID` at the CTRL/STAT transfer slot (ADIv5.2 DPv2 bank 2).
const DP_BANKSEL_TARGETID: u32 = 0x2;
/// `DPBANKSEL` exposing `DLPIDR` at the CTRL/STAT transfer slot (ADIv5.2 DPv2 bank 3).
const DP_BANKSEL_DLPIDR: u32 = 0x3;

/// `TARGETID` fields carried unchanged into `TARGETSEL`: bit 0 (RAO), TDESIGNER `[11:1]`,
/// TPARTNO `[27:12]`. TREVISION `[31:28]` is excluded.
const TARGETID_TARGETSEL_MASK: u32 = 0x0FFF_FFFF;
/// `DLPIDR` field contributing TINSTANCE `[31:28]`, the top nibble of `TARGETSEL`.
const DLPIDR_TINSTANCE_MASK: u32 = 0xF000_0000;

/// Compose the multidrop `TARGETSEL` selecting a debug port from its `TARGETID` and `DLPIDR`:
/// `TARGETID` with its TREVISION field replaced by `DLPIDR`'s TINSTANCE. That is the comparison
/// the DP itself makes, so it is also how a profile's hardcoded literal is checked against
/// silicon.
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

/// Split a full ADIv6 AP register address into `SELECT` (bits `[31:4]`, `DPBANKSEL` left zero),
/// `SELECT1` (bits `[63:32]`), and the transfer address `A[3:2]`.
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
    /// `SELECT1` value to write, if it changed. Writing it needs the `DPBANKSEL = 5` detour,
    /// which clobbers `SELECT`.
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
        // A SELECT1 change forces the SELECT rewrite even when the low word matches: the bank
        // detour overwrote it.
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
    /// DAP index byte carried in every `DAP_Transfer`/`DAP_TransferBlock`/`DAP_WriteABORT`; 0
    /// over SWD, the selected TAP's chain index over JTAG.
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
}

impl<'a> Adiv5<'a> {
    /// A fresh handle with every cache empty, ready for a wire-specific entry sequence.
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
        }
    }

    /// SWD bring-up, leaving the MEM-AP programmed for 32-bit auto-incrementing transfers.
    pub fn attach(probe: &'a mut Probe, config: AttachConfig) -> Result<Adiv5<'a>> {
        // 1. Connect, set the clock, configure the retry budget, query the packet size.
        let packet_size = connect_and_configure(
            probe,
            Port::Swd,
            config.clock_hz,
            config.wait_retry,
            config.match_retry,
        )?;

        let mut adi = Adiv5::new_handle(probe, config.ap, packet_size);

        // 2 + 3. Line reset / JTAG-to-SWD / line reset / idle (single-drop), or line reset +
        // terminated line reset + TARGETSEL (multidrop), then the SWD-enabling DPIDR read.
        adi.dpidr = Dpidr {
            raw: match (config.ap, config.targetsel) {
                // An ADIv6 DP powers up dormant: the line reset and JTAG-to-SWD select the other
                // paths use are inert against it, so the wake replaces them.
                (ApMode::V6 { .. }, targetsel) => adi.read_dpidr_dormant(targetsel)?,
                (_, Some(targetsel)) => adi.read_dpidr_targetsel(targetsel)?,
                (_, None) => adi.read_dpidr()?,
            },
        };

        // 4 + 5. Power up the debug domains, read the MEM-AP IDR, program CSW.
        adi.finish_attach()?;

        Ok(adi)
    }

    /// ADIv5 JTAG-DP bring-up: JTAG entry, `DAP_JTAG_Configure`, then `DAP_JTAG_IDCODE` in place
    /// of the DPIDR a JTAG-DP does not implement. The IDCODE read also validates the chain.
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

        // 1. Connect over JTAG, set the clock, configure the retry budget, query the packet
        // size.
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

        // Describe the chain once; every later command selects TAPs against this description.
        probe_command(
            adi.probe,
            encode_jtag_configure(&config.ir_lengths),
            decode_jtag_configure,
        )
        .context("DAP_JTAG_Configure failed")?;

        // 3. JTAG entry + IDCODE through the configured index, retried like the SWD path retries
        // its DPIDR read (re-issuing the whole entry on a failure).
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

        // 4 + 5. The same power-up / IDR / CSW bring-up as the SWD path.
        adi.finish_attach()?;

        Ok(adi)
    }

    /// The bring-up shared once the DPIDR/IDCODE read lands: clear sticky errors, power up the
    /// debug and system domains, read the MEM-AP IDR, program CSW.
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

    /// Read `TARGETID` (DP bank 2) and `DLPIDR` (DP bank 3), both at the CTRL/STAT transfer slot,
    /// returning `None` where the DP refused either. The identity is a diagnostic, so losing it
    /// must not fail an attach that otherwise reached memory.
    ///
    /// Selecting a DP bank means writing `SELECT`, which clobbers the AP addressing bits under
    /// both ADIv5 and ADIv6 whether or not the reads landed. So the caches are dropped and the AP
    /// core window re-asserted on both paths, leaving the CSW just programmed addressed.
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

    /// The multidrop identity read during the attach, or `None` on a DPv1 DP or over JTAG. Its
    /// [`DpIdent::targetsel`] is what a profile's hardcoded `TARGETSEL` literal is checked against.
    pub fn dp_ident(&self) -> Option<DpIdent> {
        self.dp_ident
    }

    /// The negotiated `DAP_PACKET_SIZE` bounding block transfers.
    pub fn packet_size(&self) -> u16 {
        self.packet_size
    }

    // --- Memory primitives ---------------------------------------------------------------------

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

    /// Halt the core and wait for `DHCSR.S_HALT`: an ACKed-but-ignored halt request is a real
    /// failure mode, so the write alone is not trusted.
    pub fn halt_core(&mut self) -> Result<()> {
        self.write_mem32(DHCSR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT)
            .context("DHCSR halt write failed")?;
        let started = Instant::now();
        loop {
            let status = self.read_mem32(DHCSR).context("DHCSR status read failed")?;
            if status & DHCSR_S_HALT != 0 {
                return Ok(());
            }
            if started.elapsed() >= HALT_TIMEOUT {
                bail!("core halt not acknowledged (DHCSR S_HALT) within {HALT_TIMEOUT:?}");
            }
        }
    }

    /// Request a system reset (`AIRCR.SYSRESETREQ`). The reboot typically drops the debug
    /// connection, so a transfer error here is expected and is the caller's to downgrade.
    pub fn system_reset(&mut self) -> Result<()> {
        self.write_mem32(AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ)
    }

    /// Read `out.len()` words from word-aligned `addr`, splitting at each wrap boundary and at
    /// the per-packet block capacity, so any length is accepted.
    pub fn read_block(&mut self, addr: u32, out: &mut [u32]) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("read_block address 0x{addr:08X} is not word-aligned");
        }
        self.ensure_csw()?;
        let max = transfer_block_read_capacity(self.packet_size);
        let mut done = 0;
        let mut cur = addr;
        while done < out.len() {
            let n = block_chunk_words(cur, out.len() - done, max);
            self.write_tar(cur)?;
            self.read_block_raw(&mut out[done..done + n])?;
            done += n;
            cur = cur.wrapping_add((n * 4) as u32);
        }
        Ok(())
    }

    /// Write `data` to word-aligned `addr`, chunked exactly as in [`Adiv5::read_block`].
    pub fn write_block(&mut self, addr: u32, data: &[u32]) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("write_block address 0x{addr:08X} is not word-aligned");
        }
        self.ensure_csw()?;
        let max = transfer_block_write_capacity(self.packet_size);
        let mut done = 0;
        let mut cur = addr;
        while done < data.len() {
            let n = block_chunk_words(cur, data.len() - done, max);
            self.write_tar(cur)?;
            self.write_block_raw(&data[done..done + n])?;
            done += n;
            cur = cur.wrapping_add((n * 4) as u32);
        }
        Ok(())
    }

    /// Read `out.len()` words from `addr` with up to `depth` pairs outstanding; `depth <= 1` is
    /// [`Adiv5::read_block`]. A `WAIT` fails the call, since a retry would desync the FIFO.
    pub fn read_block_depth(&mut self, addr: u32, out: &mut [u32], depth: usize) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("read_block address 0x{addr:08X} is not word-aligned");
        }
        if depth <= 1 {
            return self.read_block(addr, out);
        }
        self.ensure_csw()?;
        let max = transfer_block_read_capacity(self.packet_size);
        let chunks = plan_chunks(addr, out.len(), max);
        let (requests, expected_lens) = build_block_requests(&chunks, None, self.dap_index);
        let responses =
            self.probe
                .submit_pipelined(&requests, &expected_lens, depth, PIPELINE_TIMEOUT)?;

        let mut done = 0;
        for (i, &(_, n)) in chunks.iter().enumerate() {
            let words = check_pipelined_chunk(&responses[2 * i], &responses[2 * i + 1], n)?;
            out[done..done + n].copy_from_slice(&words);
            done += n;
        }
        Ok(())
    }

    /// Write `data` at `addr` with up to `depth` command pairs outstanding; chunking, pipelining
    /// and WAIT handling are [`Adiv5::read_block_depth`]'s.
    pub fn write_block_depth(&mut self, addr: u32, data: &[u32], depth: usize) -> Result<()> {
        if !addr.is_multiple_of(4) {
            bail!("write_block address 0x{addr:08X} is not word-aligned");
        }
        if depth <= 1 {
            return self.write_block(addr, data);
        }
        self.ensure_csw()?;
        let max = transfer_block_write_capacity(self.packet_size);
        let chunks = plan_chunks(addr, data.len(), max);
        let (requests, expected_lens) = build_block_requests(&chunks, Some(data), self.dap_index);
        let responses =
            self.probe
                .submit_pipelined(&requests, &expected_lens, depth, PIPELINE_TIMEOUT)?;

        for (i, &(_, n)) in chunks.iter().enumerate() {
            check_pipelined_chunk(&responses[2 * i], &responses[2 * i + 1], n)?;
        }
        Ok(())
    }

    // --- Bring-up steps ------------------------------------------------------------------------

    /// Wake a DP that powers up in the ADIv6 dormant state, where a line reset and JTAG-to-SWD
    /// select achieve nothing and every DPIDR read comes back `NoAck`.
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

    /// Single-drop SWD entry: a >=50-clock line reset, the `0xE79E` JTAG-to-SWD select word,
    /// a second line reset, then 8 idle clocks (ADIv5 wants >= 2 before the first packet).
    fn line_reset_switch(&mut self) -> Result<()> {
        self.swj_sequence(56, &[0xFF; 7])?;
        self.swj_sequence(16, &JTAG_TO_SWD_SELECT.to_le_bytes())?;
        self.swj_sequence(56, &[0xFF; 7])?;
        self.swj_sequence(8, &[0x00])?;
        Ok(())
    }

    /// JTAG entry over `DAP_SWJ_Sequence` (which drives TMS): a >=50-clock line reset, the
    /// `0xE73C` SWD-to-JTAG select word, then the TAP reset into Run-Test/Idle.
    fn jtag_entry(&mut self) -> Result<()> {
        self.swj_sequence(56, &[0xFF; 7])?;
        self.swj_sequence(16, &SWD_TO_JTAG_SELECT.to_le_bytes())?;
        self.swj_sequence(JTAG_TAP_RESET_IDLE_BITS, &JTAG_TAP_RESET_IDLE)?;
        Ok(())
    }

    /// Entry sequence plus the DPIDR read that leaves the reset state and enables SWD; the whole
    /// pair is retried on a non-`OK` ack.
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

    /// `TARGETSEL` plus the DPIDR read that confirms the selection. A retry re-issues both: a
    /// reset after `TARGETSEL` would deselect the DP again.
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

    /// Dormant wake, optional `TARGETSEL`, then the DPIDR read. Every attempt re-issues the wake:
    /// waking once outside the loop leaves every retry after the first inert.
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

    /// Select one multidrop debug port: terminated line reset, then `TARGETSEL` as one
    /// host-driven sequence, since `TARGETSEL` has no acknowledge phase.
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

    /// Clear every sticky error flag: ABORT over SWD, write-one-to-clear CTRL/STAT over JTAG.
    /// The JTAG arm bypasses [`Adiv5::write_dp`], whose FAULT recovery calls back here.
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
                        "CTRL/STAT sticky-clear write not acknowledged (ack {:?})",
                        decoded.status.ack()
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

    // --- Register access -----------------------------------------------------------------------

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

    /// Point `SELECT`/`SELECT1` at the ADIv6 AP register at frame byte offset `off`. A `SELECT1`
    /// change takes the `DPBANKSEL = 5` detour and forces the `SELECT` rewrite it clobbers.
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

    // --- Transfer plumbing ---------------------------------------------------------------------

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
                other => bail!(
                    "DAP_Transfer failed: ack {other:?} after {} of {} transfers",
                    decoded.transfer_count,
                    items.len()
                ),
            }
        }
        bail!("DAP_Transfer WAIT retries exhausted")
    }

    /// One `DAP_TransferBlock` read of DRW (TAR pre-armed); `out.len()` must fit one packet.
    fn read_block_raw(&mut self, out: &mut [u32]) -> Result<()> {
        let request = TransferRequest::new(true, true, AP_A_DRW);
        for _ in 0..=WAIT_RETRIES {
            let req = encode_transfer_block(self.dap_index, out.len() as u16, request, &[]);
            // A full-capacity response is an exact packet multiple: no short packet ends it.
            let resp = self
                .probe
                .transfer_expect(&req, 4 + 4 * out.len(), TRANSFER_TIMEOUT)
                .context("DAP_TransferBlock (read) USB round-trip failed")?;
            let decoded = decode_transfer_block(&resp)
                .context("decoding DAP_TransferBlock response failed")?;
            if decoded.status.is_ok() && decoded.transfer_count as usize == out.len() {
                out.copy_from_slice(&decoded.read_data);
                return Ok(());
            }
            match decoded.status.ack() {
                Ack::Wait => continue,
                Ack::Fault => {
                    let _ = self.clear_sticky_errors();
                    bail!(
                        "DAP_TransferBlock (read) faulted after {} words",
                        decoded.transfer_count
                    );
                }
                other => bail!("DAP_TransferBlock (read) failed: ack {other:?}"),
            }
        }
        bail!("DAP_TransferBlock (read) WAIT retries exhausted")
    }

    /// One `DAP_TransferBlock` write of DRW (TAR pre-armed); `data.len()` must fit one packet.
    fn write_block_raw(&mut self, data: &[u32]) -> Result<()> {
        let request = TransferRequest::new(true, false, AP_A_DRW);
        for _ in 0..=WAIT_RETRIES {
            let req = encode_transfer_block(self.dap_index, data.len() as u16, request, data);
            // A block-write response is a fixed 4-byte header with no payload.
            let resp = self
                .probe
                .transfer_expect(&req, 4, TRANSFER_TIMEOUT)
                .context("DAP_TransferBlock (write) USB round-trip failed")?;
            let decoded = decode_transfer_block(&resp)
                .context("decoding DAP_TransferBlock response failed")?;
            if decoded.status.is_ok() && decoded.transfer_count as usize == data.len() {
                return Ok(());
            }
            match decoded.status.ack() {
                Ack::Wait => continue,
                Ack::Fault => {
                    let _ = self.clear_sticky_errors();
                    bail!(
                        "DAP_TransferBlock (write) faulted after {} words",
                        decoded.transfer_count
                    );
                }
                other => bail!("DAP_TransferBlock (write) failed: ack {other:?}"),
            }
        }
        bail!("DAP_TransferBlock (write) WAIT retries exhausted")
    }
}

/// `DAP_Connect`, `DAP_SWJ_Clock`, `DAP_TransferConfigure`, `DAP_Info(PacketSize)`; returns the
/// negotiated packet size, or the CMSIS-DAP minimum when the probe reports none.
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

    // The packet size bounds every DAP_TransferBlock, so a zero report is unusable.
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

/// The host-driven `DAP_SWJ_Sequence` payload for a multidrop `TARGETSEL` write: request byte,
/// the ACK window driven high, the value LSB-first, its even parity -- 46 bits padded to 48.
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
    decode(&resp).context("decoding CMSIS-DAP response failed")
}

#[cfg(test)]
mod tests {

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

    /// The two RP `TARGETSEL` literals `target.rs` hardcodes are derived values; these pin the
    /// derivation so a profile's literal can be checked against what silicon reports.
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
        // RP2040-shaped: TARGETID 0x0100_2927 (rev 0), DLPIDR TINSTANCE 0, PROTVSN 1.
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

    /// The only executable check that the ADIv6 constants -- the dormant wake, the `0x2000` AP
    /// base, the `0xDFC` IDR offset -- are right. Run with
    /// `DAPBENCH_PROBE=vid:pid:serial cargo test -- --ignored attach_v6_against_hardware`; the
    /// selector may be omitted only when one probe is attached. The target must be power-cycled
    /// rather than freshly flashed, since a flash leaves the DP awake and hides the wake.
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
        // A word round-trip through SRAM proves the SELECT path reaches memory.
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
        // RP2040 core-0 TARGETSEL; captured host trace `99 ff 24 05 20 00`.
        assert_eq!(
            targetsel_sequence(0x0100_2927),
            [0x99, 0xFF, 0x24, 0x05, 0x20, 0x00]
        );
    }

    // -- Pipelined chunk planning and request construction -------------------------------------

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
    fn build_block_requests_read_interleaves_tar_and_block_with_exact_lens() {
        let chunks = plan_chunks(0x2000_0000, 30, 15);
        assert_eq!(chunks, vec![(0x2000_0000, 15), (0x2000_003C, 15)]);
        let (requests, lens) = build_block_requests(&chunks, None, 0);
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
        let chunks = plan_chunks(0x2000_0000, data.len(), 14);
        assert_eq!(chunks, vec![(0x2000_0000, 14), (0x2000_0038, 6)]);
        let (requests, lens) = build_block_requests(&chunks, Some(&data), 0);
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
        let chunks = plan_chunks(0x2000_0000, 20, 15);
        let (requests, _) = build_block_requests(&chunks, None, 1);
        for (i, req) in requests.iter().enumerate() {
            assert_eq!(req[1], 1, "packet {i} DAP index");
        }
    }

    #[test]
    fn check_pipelined_chunk_accepts_a_clean_pair_and_rejects_failures() {
        // Clean TAR write + clean 2-word block read.
        let tar_ok = [0x05, 0x01, 0x01];
        let block_ok = [
            0x06, 0x02, 0x00, 0x01, //
            0xAA, 0xAA, 0xAA, 0xAA, //
            0xBB, 0xBB, 0xBB, 0xBB,
        ];
        assert_eq!(
            check_pipelined_chunk(&tar_ok, &block_ok, 2).unwrap(),
            vec![0xAAAA_AAAA, 0xBBBB_BBBB]
        );
        // A WAIT on the TAR write fails the chunk -- no retry inside a pipelined window.
        let tar_wait = [0x05, 0x00, 0b010];
        assert!(check_pipelined_chunk(&tar_wait, &block_ok, 2).is_err());
        // A short block (probe stopped early) fails the chunk.
        let block_short = [0x06, 0x01, 0x00, 0x01, 0xAA, 0xAA, 0xAA, 0xAA];
        assert!(check_pipelined_chunk(&tar_ok, &block_short, 2).is_err());
    }
}
