//! Delay-free transaction core: DP/AP register access, memory access, and debug bring-up.

use core::marker::PhantomData;

use dap_rs::swd::{APnDP, DPRegister, Swd};

use crate::error::Error;
use crate::reset::ResetControl;
use crate::target::{ApAddressing, DebugPort, DpWake};

/// Number of WAIT retries handed to the `dap-rs` transfer helpers.
const WAIT_RETRIES: usize = 100;

/// Debug-port CTRL/STAT power-up handshake bits.
pub(crate) const CSYSPWRUPREQ: u32 = 1 << 30;
pub(crate) const CDBGPWRUPREQ: u32 = 1 << 28;
pub(crate) const CSYSPWRUPACK: u32 = 1 << 31;
pub(crate) const CDBGPWRUPACK: u32 = 1 << 29;
/// ABORT register: clear all sticky error flags.
pub(crate) const ABORT_CLEAR_ALL: u32 = 0x1E;
/// ABORT register: `DAPABORT`; an AP transaction that never completed answers WAIT until this runs.
pub(crate) const ABORT_DAP: u32 = 0x01;

// ADIv6 relocates the MEM-AP registers to the 0xD00 page: ADIv5 offset `x` lives at `0xD00 | x`.
const ADIV6_CORE_PAGE: u32 = 0xD00;

// MEM-AP CSW: DbgSwEnable(31) | Prot bits | AddrInc=single(bit4) | Size=word(0b010).
pub(crate) const CSW_WORD_ADDRINC: u32 = 0x2300_0002 | (1 << 4);

/// Memory-mapped core debug registers.
pub(crate) const DHCSR: u32 = 0xE000_EDF0;
pub(crate) const DCRSR: u32 = 0xE000_EDF4;
pub(crate) const DCRDR: u32 = 0xE000_EDF8;

/// Write key required for every DHCSR write.
pub(crate) const DBGKEY: u32 = 0xA05F_0000;
pub(crate) const C_DEBUGEN: u32 = 1 << 0;
pub(crate) const C_HALT: u32 = 1 << 1;
pub(crate) const S_REGRDY: u32 = 1 << 16;
pub(crate) const S_HALT: u32 = 1 << 17;

pub(crate) const DEMCR: u32 = 0xE000_EDFC;
/// Halt the core on reset (reset vector catch).
pub(crate) const VC_CORERESET: u32 = 1 << 0;
/// Trace/debug enable; left set while a session is active.
pub(crate) const TRCENA: u32 = 1 << 24;

/// Poll budget for register-ready waits.
pub(crate) const REGRDY_POLLS: u32 = 10_000;

// 128-bit ADIv6 dormant-state selection alert, transmitted LSB-first as two 64-bit halves.
pub(crate) const SELECTION_ALERT_LO: u64 = 0x8685_2D95_6209_F392;
pub(crate) const SELECTION_ALERT_HI: u64 = 0x19BC_0EA2_E3DD_AFE9;
/// 31-bit SWD-to-dormant select sequence; a dormant DP ignores it, so it is safe before the alert.
pub(crate) const SWD_TO_DORMANT: [u8; 4] = [0xBA, 0xBB, 0xBB, 0x33];
/// 4 idle cycles + 8-bit SWD activation code `0x1A`, sent after the selection alert.
pub(crate) const SWD_ACTIVATION: u16 = 0x1A0;

/// Multidrop selection verify masks: `TARGETID` carries the low 28 bits of `TARGETSEL`,
/// `DLPIDR` carries the top nibble (target instance).
pub(crate) const TARGETID_MASK: u32 = 0x0FFF_FFFF;
pub(crate) const DLPIDR_MASK: u32 = 0xF000_0000;

/// Dormant-state exit preamble for a DP that powers up dormant (ADIv6): line reset, forced
/// dormant entry, selection alert, then SWD activation.
pub fn swd_dormant_wake_preamble<S, DEPS>(swd: &mut S) -> dap_rs::swd::Result<()>
where
    S: Swd<DEPS>,
{
    let ones = [0xFFu8; 8];
    swd.write_sequence(51, &ones)?;
    swd.write_sequence(31, &SWD_TO_DORMANT)?;
    swd.write_sequence(8, &ones[..1])?;
    swd.write_sequence(64, &SELECTION_ALERT_LO.to_le_bytes())?;
    swd.write_sequence(64, &SELECTION_ALERT_HI.to_le_bytes())?;
    // Caller issues the final line reset before its first packet / TARGETSEL.
    swd.write_sequence(12, &SWD_ACTIVATION.to_le_bytes())?;
    Ok(())
}

/// SWD wake preamble: line reset, JTAG-to-SWD switch, second line reset, idle cycles.
pub fn swd_wake_preamble<S, DEPS>(swd: &mut S) -> dap_rs::swd::Result<()>
where
    S: Swd<DEPS>,
{
    let ones = [0xFFu8; 8];
    swd.write_sequence(51, &ones)?;
    let switch = 0xE79Eu16.to_le_bytes();
    swd.write_sequence(16, &switch)?;
    swd.write_sequence(51, &ones)?;
    let idle = [0x00u8; 1];
    swd.write_sequence(8, &idle)?;
    Ok(())
}

/// Map an AP-register byte offset to the 2-bit transfer address `A[3:2]` encoded as a
/// [`DPRegister`]: `0x00 -> DPIDR`, `0x04 -> CTRLSTAT`, `0x08 -> SELECT`, `0x0C -> RDBUFF`.
fn ap_transfer_reg(addr: u8) -> DPRegister {
    match (addr >> 2) & 0x3 {
        0 => DPRegister::DPIDR,
        1 => DPRegister::CTRLSTAT,
        2 => DPRegister::SELECT,
        _ => DPRegister::RDBUFF,
    }
}

/// Whether a failed transfer is worth one sticky-error recovery attempt (FAULT/NoAck/protocol
/// error, not a parity error or exhausted WAIT budget).
fn recoverable(err: dap_rs::swd::Error) -> bool {
    use dap_rs::swd::Error as E;
    matches!(err, E::AckFault | E::NoAck | E::AckProtocol)
}

/// The debug and access port transactions that need no delay source.
pub(crate) struct SwdHostCore<S, DEPS> {
    pub(crate) swd: S,
    apsel: u32,
    /// Debug-port connection policy (wake strategy, multidrop selection, AP addressing).
    dp: DebugPort,
    /// Cached SELECT value (`None` forces a rewrite on next AP access).
    select: Option<u32>,
    /// Cached CSW value (`None` forces a rewrite on next memory access).
    csw: Option<u32>,
    /// DPIDR read by the most recent [`init_debug`](Self::init_debug); `None` before first connect.
    dpidr: Option<u32>,
    /// WAIT-retry budget handed to every `dap-rs` transfer helper.
    wait_retries: usize,
    /// Poll budget for the debug power-up handshake in [`init_debug`](Self::init_debug).
    powerup_polls: u32,
    _deps: PhantomData<fn() -> DEPS>,
}

impl<S, DEPS> SwdHostCore<S, DEPS>
where
    S: Swd<DEPS>,
{
    /// Wrap an SWD driver.
    pub(crate) fn new(swd: S) -> Self {
        Self {
            swd,
            apsel: 0,
            dp: DebugPort::default(),
            select: None,
            csw: None,
            dpidr: None,
            wait_retries: WAIT_RETRIES,
            powerup_polls: REGRDY_POLLS,
            _deps: PhantomData,
        }
    }

    /// The DPIDR read by the most recent [`init_debug`](Self::init_debug), if connected.
    pub(crate) fn dpidr(&self) -> Option<u32> {
        self.dpidr
    }

    /// Tighten the transfer-retry and power-up poll budgets for transient target detection.
    pub(crate) fn set_detect_budget(&mut self, wait_retries: usize, powerup_polls: u32) {
        self.wait_retries = wait_retries;
        self.powerup_polls = powerup_polls;
    }

    /// Clear the debug port's sticky error flags via ABORT.
    pub(crate) fn clear_sticky_errors(&mut self) -> Result<(), Error> {
        self.write_dp(DPRegister::DPIDR, ABORT_CLEAR_ALL)
    }

    /// Detection spelling of [`abort_transaction`](Self::abort_transaction).
    pub(crate) fn detect_abort(&mut self) {
        self.abort_transaction();
    }

    /// Cancel an outstanding access-port transaction and acknowledge the flags a failed one
    /// latched: `DAPABORT`, then `ABORT_CLEAR_ALL`. Neither substitutes for the other.
    pub(crate) fn abort_transaction(&mut self) {
        let _ = self
            .swd
            .write_dp(self.wait_retries, DPRegister::DPIDR, ABORT_DAP);
        let _ = self
            .swd
            .write_dp(self.wait_retries, DPRegister::DPIDR, ABORT_CLEAR_ALL);
        self.csw = None;
    }

    /// Read a word during target detection, where a failure is ordinary traffic: one recovered
    /// retry via [`detect_abort`](Self::detect_abort), and the port is left usable either way.
    pub(crate) fn detect_read_word(&mut self, addr: u32) -> Result<u32, Error> {
        match self.read_word(addr) {
            Ok(value) => Ok(value),
            Err(err) => {
                self.detect_abort();
                let retried = self.read_word(addr);
                if retried.is_err() {
                    self.detect_abort();
                }
                retried.map_err(|_| err)
            }
        }
    }

    /// Select which access port subsequent memory operations target.
    pub(crate) fn set_apsel(&mut self, apsel: u32) {
        if self.apsel != apsel {
            self.apsel = apsel;
            self.select = None;
            self.csw = None;
        }
    }

    /// Set the debug-port connection policy applied by [`init_debug`](Self::init_debug).
    pub(crate) fn set_dp_config(&mut self, dp: DebugPort) {
        self.dp = dp;
        self.select = None;
        self.csw = None;
    }

    fn read_dp(&mut self, reg: DPRegister) -> Result<u32, Error> {
        self.transfer_with_recovery(false, |swd, retries| swd.read_dp(retries, reg))
    }

    fn write_dp(&mut self, reg: DPRegister, value: u32) -> Result<(), Error> {
        self.transfer_with_recovery(false, |swd, retries| swd.write_dp(retries, reg, value))
    }

    /// Run one transfer, retrying it once after sticky-error recovery if it faults. A failed
    /// recovery or retry surfaces the original error.
    fn transfer_with_recovery<T>(
        &mut self,
        ap_mem: bool,
        mut op: impl FnMut(&mut S, usize) -> dap_rs::swd::Result<T>,
    ) -> Result<T, Error> {
        match op(&mut self.swd, self.wait_retries) {
            Ok(v) => Ok(v),
            Err(err) => {
                if recoverable(err) && self.recover_sticky(ap_mem).is_ok() {
                    if let Ok(v) = op(&mut self.swd, self.wait_retries) {
                        return Ok(v);
                    }
                }
                Err(err.into())
            }
        }
    }

    /// Clear the latched sticky flags and re-arm the transfer context: ABORT, re-issue cached
    /// SELECT, and -- for an AP memory access -- the cached CSW.
    fn recover_sticky(&mut self, ap_mem: bool) -> dap_rs::swd::Result<()> {
        self.swd
            .write_dp(self.wait_retries, DPRegister::DPIDR, ABORT_CLEAR_ALL)?;
        if let Some(select) = self.select {
            self.swd
                .write_dp(self.wait_retries, DPRegister::SELECT, select)?;
        }
        if ap_mem {
            if let Some(csw) = self.csw {
                // CSW at AP offset 0x00 -> DPRegister::DPIDR.
                self.swd
                    .write(self.wait_retries, APnDP::AP, DPRegister::DPIDR, csw)?;
            }
        }
        Ok(())
    }

    /// Ensure SELECT addresses the target AP's CSW/TAR/DRW register window (ADIv5: `APSEL` in
    /// `SELECT[31:24]`; ADIv6: AP base in `SELECT[31:4]` relocated to the 0xD00 page).
    fn ensure_select(&mut self) -> Result<(), Error> {
        let want = match self.dp.ap {
            ApAddressing::V5 => self.apsel << 24,
            ApAddressing::V6 { ap_base } => ap_base | ADIV6_CORE_PAGE,
        };
        if self.select != Some(want) {
            self.write_dp(DPRegister::SELECT, want)?;
            self.select = Some(want);
        }
        Ok(())
    }

    fn write_ap(&mut self, reg: DPRegister, value: u32) -> Result<(), Error> {
        self.ensure_select()?;
        self.transfer_with_recovery(true, |swd, retries| {
            swd.write(retries, APnDP::AP, reg, value)
        })
    }

    /// Post an AP read then fetch the pipelined result from RDBUFF.
    fn read_ap(&mut self, reg: DPRegister) -> Result<u32, Error> {
        self.ensure_select()?;
        self.transfer_with_recovery(true, |swd, retries| {
            let _ = swd.read(retries, APnDP::AP, reg)?;
            swd.read_dp(retries, DPRegister::RDBUFF)
        })
    }

    /// Read a raw AP register at `apsel`/byte-offset `addr`, bypassing the memory-AP `SELECT`
    /// caching so a vendor access port (e.g. Nordic CTRL-AP) can be reached directly.
    pub(crate) fn ap_read_raw(&mut self, apsel: u8, addr: u8) -> Result<u32, Error> {
        let select = (u32::from(apsel) << 24) | (u32::from(addr) & 0xF0);
        self.write_dp(DPRegister::SELECT, select)?;
        self.select = None;
        self.csw = None;
        let reg = ap_transfer_reg(addr);
        self.transfer_with_recovery(false, |swd, retries| {
            let _ = swd.read(retries, APnDP::AP, reg)?;
            swd.read_dp(retries, DPRegister::RDBUFF)
        })
    }

    /// Write a raw AP register (see [`ap_read_raw`](Self::ap_read_raw) for the addressing model).
    pub(crate) fn ap_write_raw(&mut self, apsel: u8, addr: u8, value: u32) -> Result<(), Error> {
        let select = (u32::from(apsel) << 24) | (u32::from(addr) & 0xF0);
        self.write_dp(DPRegister::SELECT, select)?;
        self.select = None;
        self.csw = None;
        let reg = ap_transfer_reg(addr);
        self.transfer_with_recovery(false, |swd, retries| {
            swd.write(retries, APnDP::AP, reg, value)
        })
    }

    fn ensure_csw(&mut self) -> Result<(), Error> {
        if self.csw != Some(CSW_WORD_ADDRINC) {
            // CSW at AP offset 0x00 -> DPRegister::DPIDR.
            self.write_ap(DPRegister::DPIDR, CSW_WORD_ADDRINC)?;
            self.csw = Some(CSW_WORD_ADDRINC);
        }
        Ok(())
    }

    fn write_tar(&mut self, addr: u32) -> Result<(), Error> {
        // TAR at AP offset 0x04 -> DPRegister::CTRLSTAT.
        self.write_ap(DPRegister::CTRLSTAT, addr)
    }

    fn read_drw(&mut self) -> Result<u32, Error> {
        // DRW at AP offset 0x0C -> DPRegister::RDBUFF.
        self.read_ap(DPRegister::RDBUFF)
    }

    fn write_drw(&mut self, value: u32) -> Result<(), Error> {
        self.write_ap(DPRegister::RDBUFF, value)
    }

    /// Read a single 32-bit word.
    pub(crate) fn read_word(&mut self, addr: u32) -> Result<u32, Error> {
        self.ensure_csw()?;
        self.write_tar(addr)?;
        self.read_drw()
    }

    /// Write a single 32-bit word.
    pub(crate) fn write_word(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        self.ensure_csw()?;
        self.write_tar(addr)?;
        self.write_drw(value)
    }

    /// Read `data.len()` bytes starting at `addr`; word-aligned, word-multiple only.
    pub(crate) fn read_memory(&mut self, addr: u32, data: &mut [u8]) -> Result<(), Error> {
        if !data.len().is_multiple_of(4) {
            return Err(Error::AddressNotAligned { addr, alignment: 4 });
        }
        let words = data.len() / 4;
        self.read_words(addr, words, &mut |a, word| {
            let off = (a - addr) as usize;
            data[off..off + 4].copy_from_slice(&word.to_le_bytes());
            true
        })
    }

    /// Read `words` words from `addr`, handing each `(address, value)` to `sink` as it arrives;
    /// `sink` returns `false` to stop early.
    pub(crate) fn read_words(
        &mut self,
        addr: u32,
        words: usize,
        sink: &mut dyn FnMut(u32, u32) -> bool,
    ) -> Result<(), Error> {
        if !addr.is_multiple_of(4) {
            return Err(Error::AddressNotAligned { addr, alignment: 4 });
        }
        if self.read_words_posted(addr, words, sink).is_ok() {
            return Ok(());
        }
        // A faulted block latches sticky flags; clear them so the retry reports the real failure.
        let _ = self.clear_sticky_errors();
        for i in 0..words {
            let a = addr + (i as u32) * 4;
            let word = self.read_word(a)?;
            if !sink(a, word) {
                break;
            }
        }
        Ok(())
    }

    /// The pipelined block read behind [`read_words`](Self::read_words); failure is left for
    /// that caller to recover from.
    fn read_words_posted(
        &mut self,
        addr: u32,
        words: usize,
        sink: &mut dyn FnMut(u32, u32) -> bool,
    ) -> Result<(), Error> {
        self.ensure_csw()?;
        let mut cur = addr;
        let mut done = 0;
        while done < words {
            // TAR auto-increment wraps at a 1KiB boundary; re-arm TAR on each crossing.
            let to_wrap = ((1024 - cur % 1024) / 4) as usize;
            let n = to_wrap.min(words - done);
            self.write_tar(cur)?;
            // An AP read returns the PREVIOUS read's data; a final RDBUFF read collects the last.
            self.posted_read()?;
            for i in 0..n {
                let word = if i + 1 < n {
                    self.posted_read()?
                } else {
                    self.rdbuff_read()?
                };
                if !sink(cur + (i as u32) * 4, word) {
                    return Ok(());
                }
            }
            done += n;
            cur = cur.wrapping_add((n as u32) * 4);
        }
        Ok(())
    }

    /// Post one DRW read; no single-transfer sticky recovery, since retrying inside a pipeline
    /// would deliver a different word and leave TAR ahead of the caller's count.
    fn posted_read(&mut self) -> Result<u32, Error> {
        self.swd
            .read(self.wait_retries, APnDP::AP, DPRegister::RDBUFF)
            .map_err(Into::into)
    }

    /// Collect the outstanding posted read from the DP's RDBUFF, closing a pipeline.
    fn rdbuff_read(&mut self) -> Result<u32, Error> {
        self.swd
            .read_dp(self.wait_retries, DPRegister::RDBUFF)
            .map_err(Into::into)
    }

    /// Write `data` starting at `addr` using TAR auto-increment within the 1KiB wrap window.
    pub(crate) fn write_memory(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        if !addr.is_multiple_of(4) || !data.len().is_multiple_of(4) {
            return Err(Error::AddressNotAligned { addr, alignment: 4 });
        }
        self.ensure_csw()?;
        let mut cur = addr;
        // TAR auto-increment wraps at a 1KiB boundary; re-arm TAR on each crossing.
        let mut need_tar = true;
        for chunk in data.chunks(4) {
            if need_tar || cur.is_multiple_of(1024) {
                self.write_tar(cur)?;
                need_tar = false;
            }
            let mut buf = [0u8; 4];
            buf.copy_from_slice(chunk);
            self.write_drw(u32::from_le_bytes(buf))?;
            cur = cur.wrapping_add(4);
        }
        Ok(())
    }

    fn wait_regrdy(&mut self) -> Result<(), Error> {
        for _ in 0..REGRDY_POLLS {
            if self.read_word(DHCSR)? & S_REGRDY != 0 {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Read core register `reg` (0-15 GP, 16 xPSR, etc.).
    pub(crate) fn read_core_reg(&mut self, reg: u16) -> Result<u32, Error> {
        // DCRSR write with bit16 clear requests a read into DCRDR.
        self.write_word(DCRSR, u32::from(reg) & 0x1F)?;
        self.wait_regrdy()?;
        self.read_word(DCRDR)
    }

    /// Write core register `reg`.
    pub(crate) fn write_core_reg(&mut self, reg: u16, value: u32) -> Result<(), Error> {
        self.write_word(DCRDR, value)?;
        // DCRSR write with bit16 set requests a write from DCRDR.
        self.write_word(DCRSR, (u32::from(reg) & 0x1F) | (1 << 16))?;
        self.wait_regrdy()
    }

    /// Emit one wire sequence, mapping a driver error to [`Error::DebugInitFailed`].
    fn wire_seq(&mut self, num_bits: usize, data: &[u8]) -> Result<(), Error> {
        self.swd
            .write_sequence(num_bits, data)
            .map_err(|_| Error::DebugInitFailed)
    }

    /// Dormant-to-SWD wake: line reset, SWD-to-dormant select, selection alert, activation.
    fn dormant_wake(&mut self) -> Result<(), Error> {
        let ones = [0xFFu8; 8];
        self.wire_seq(51, &ones)?;
        self.wire_seq(31, &SWD_TO_DORMANT)?;
        self.wire_seq(8, &ones[..1])?;
        self.wire_seq(64, &SELECTION_ALERT_LO.to_le_bytes())?;
        self.wire_seq(64, &SELECTION_ALERT_HI.to_le_bytes())?;
        self.wire_seq(12, &SWD_ACTIVATION.to_le_bytes())?;
        Ok(())
    }

    /// Emit a line reset then the `TARGETSEL` write for `targetsel`; no target drives the
    /// acknowledge phase, but SWDIO must still be released across that window.
    fn write_targetsel(&mut self, targetsel: u32) -> Result<(), Error> {
        // 51 cycles high then 3 low: without the low cycles the DP stays in the reset state.
        self.wire_seq(54, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07])?;
        // One driven 48-bit sequence: request 0x99, 5 high bits, the value LSB first, parity at 45.
        let parity = u64::from(targetsel.count_ones() & 1);
        let seq = (parity << 45) | (u64::from(targetsel) << 13) | 0x1F99;
        self.wire_seq(48, &seq.to_le_bytes()[..6])?;
        Ok(())
    }

    /// Confirm the selected DP matches `targetsel` by reading `TARGETID` (DP bank 2) and
    /// `DLPIDR` (DP bank 3). Leaves `SELECT` invalidated.
    fn verify_targetsel(&mut self, targetsel: u32) -> Result<(), Error> {
        self.write_dp(DPRegister::SELECT, 2)?;
        let target_id = self.read_dp(DPRegister::CTRLSTAT)?;
        self.write_dp(DPRegister::SELECT, 3)?;
        let dlpidr = self.read_dp(DPRegister::CTRLSTAT)?;
        self.select = None;
        if (target_id & TARGETID_MASK) != (targetsel & TARGETID_MASK)
            || (dlpidr & DLPIDR_MASK) != (targetsel & DLPIDR_MASK)
        {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "init_debug: TARGETSEL verify mismatch: want={=u32:08x} TARGETID={=u32:08x} DLPIDR={=u32:08x}",
                targetsel,
                target_id,
                dlpidr
            );
            return Err(Error::DebugInitFailed);
        }
        Ok(())
    }

    /// Perform the SWD line reset / JTAG-to-SWD switch, read DPIDR, power up the debug
    /// domains and clear sticky errors.
    pub(crate) fn init_debug(&mut self) -> Result<(), Error> {
        self.select = None;
        self.csw = None;

        // Fixed rate for bring-up keeps multidrop TARGETSEL direction flips predictable.
        self.swd.set_clock(10_000_000);

        #[cfg(feature = "defmt")]
        defmt::info!(
            "init_debug: wake={=str} targetsel={:08x}",
            match self.dp.wake {
                DpWake::JtagToSwd => "jtag-to-swd",
                DpWake::DormantToSwd => "dormant-to-swd",
            },
            self.dp.targetsel
        );
        match self.dp.wake {
            DpWake::JtagToSwd => {
                swd_wake_preamble(&mut self.swd).map_err(|_| Error::DebugInitFailed)?;
            }
            DpWake::DormantToSwd => self.dormant_wake()?,
        }

        // TARGETSEL write is unacknowledged by design, so it is a raw bit sequence.
        if let Some(targetsel) = self.dp.targetsel {
            self.write_targetsel(targetsel)?;
        }

        // Reading DPIDR is required to leave the reset state.
        let dpidr = match self.read_dp(DPRegister::DPIDR) {
            Ok(v) => v,
            Err(_e) => {
                #[cfg(feature = "defmt")]
                defmt::warn!("init_debug: DPIDR read failed: {}", _e);
                return Err(Error::DebugInitFailed);
            }
        };
        #[cfg(feature = "defmt")]
        defmt::info!("init_debug: DPIDR={=u32:08x}", dpidr);
        self.dpidr = Some(dpidr);

        self.write_dp(DPRegister::DPIDR, ABORT_CLEAR_ALL)
            .map_err(|_| Error::DebugInitFailed)?;

        if let Some(targetsel) = self.dp.targetsel {
            self.verify_targetsel(targetsel)?;
            #[cfg(feature = "defmt")]
            defmt::info!("init_debug: TARGETSEL verified");
        }

        self.ensure_select()?;

        self.write_dp(DPRegister::CTRLSTAT, CSYSPWRUPREQ | CDBGPWRUPREQ)
            .map_err(|_| Error::DebugInitFailed)?;
        let mut powered = false;
        for _ in 0..self.powerup_polls {
            let stat = self.read_dp(DPRegister::CTRLSTAT)?;
            if stat & (CSYSPWRUPACK | CDBGPWRUPACK) == (CSYSPWRUPACK | CDBGPWRUPACK) {
                powered = true;
                break;
            }
        }
        if !powered {
            return Err(Error::PowerUpTimeout);
        }

        Ok(())
    }

    /// Read the debug port's own CTRL/STAT register, for diagnostics.
    #[cfg(feature = "defmt")]
    pub(crate) fn ctrl_stat(&mut self) -> Result<u32, Error> {
        self.ensure_select()?;
        self.read_dp(DPRegister::CTRLSTAT)
    }

    /// Ask the core to halt (set C_HALT | C_DEBUGEN via DHCSR) and return without waiting; the
    /// caller must poll DHCSR for `S_HALT` (needs a delay source, so belongs to `SwdHost`).
    pub(crate) fn request_halt(&mut self) -> Result<(), Error> {
        self.write_word(DHCSR, DBGKEY | C_HALT | C_DEBUGEN)
    }

    /// Read DHCSR once, for a caller polling the core's halt state.
    pub(crate) fn read_dhcsr(&mut self) -> Result<u32, Error> {
        self.read_word(DHCSR)
    }
}

impl<S, DEPS> SwdHostCore<S, DEPS>
where
    S: Swd<DEPS> + ResetControl,
{
    /// Assert or release the physical nRESET line via the driver's reset control.
    pub(crate) fn set_reset(&mut self, asserted: bool) -> Result<(), Error> {
        self.swd.set_reset(asserted).map_err(|_| Error::ResetFailed)
    }
}
