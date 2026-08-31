//! Debug host: the transaction core plus a delay source, the flash syscalls, and the reset paths.

use dap_rs::swd::Swd;
use embedded_hal::delay::DelayNs;

use crate::error::Error;
use crate::flash_algo::SyscallParams;
use crate::reset::ResetControl;
use crate::reset_sequence::{ResetOps, ResetSequence, GENERIC_RESET_SEQUENCE};
use crate::swd_host_core::{SwdHostCore, C_DEBUGEN, DBGKEY, DHCSR, S_HALT};
use crate::target::{DebugPort, ResetType, TargetState};

pub use crate::swd_host_core::{swd_dormant_wake_preamble, swd_wake_preamble};

/// Application Interrupt and Reset Control Register.
const AIRCR: u32 = 0xE000_ED0C;
/// Write key required for every AIRCR write; a mismatched key is ignored.
const AIRCR_KEY: u32 = 0x05FA_0000;
/// Request a full system reset.
const SYSRESETREQ: u32 = 1 << 2;
/// Request a local core (vector) reset only.
const VECTRESET: u32 = 1 << 0;

/// nRESET assertion time for a hardware reset pulse, in microseconds.
const HW_RESET_ASSERT_US: u32 = 50_000;
/// Settling time after releasing nRESET before reconnecting debug, in microseconds.
const HW_RESET_RECOVER_US: u32 = 10_000;

/// Syscall halt-wait poll start delay; backs off exponentially, bounded by a wall-clock budget.
const SYSCALL_POLL_START_US: u32 = 100;
/// Cap on the per-poll backoff delay.
const SYSCALL_POLL_MAX_US: u32 = 100_000;
/// Total budget for one syscall to return before `AlgoTimeout`, in microseconds (~30 s).
const SYSCALL_TIMEOUT_US: u64 = 30_000_000;

/// Halt-poll backoff: delay before the first DHCSR read, in microseconds.
const HALT_POLL_START_US: u32 = 100;
/// Cap on the halt poll's per-attempt backoff delay, in microseconds.
const HALT_POLL_MAX_US: u32 = 10_000;
/// Total budget for the core to report halted, in microseconds (1 s).
const HALT_POLL_BUDGET_US: u64 = 1_000_000;

/// What a halt poll is waiting on.
#[derive(Clone, Copy, PartialEq, Eq)]
enum HaltSource {
    /// A halt this host has to ask for: DHCSR's `C_HALT` is written until the port accepts it.
    Requested,
    /// A halt the target produces on its own via the armed reset vector catch.
    VectorCatch,
}

/// A host that drives a target's debug and access ports over SWD.
pub struct SwdHost<S, D, DEPS> {
    core: SwdHostCore<S, DEPS>,
    delay: D,
}

impl<S, D, DEPS> SwdHost<S, D, DEPS>
where
    S: Swd<DEPS>,
    D: DelayNs,
{
    /// Wrap an SWD driver and a delay source.
    pub fn new(swd: S, delay: D) -> Self {
        Self {
            core: SwdHostCore::new(swd),
            delay,
        }
    }

    /// The DPIDR read by the most recent [`init_debug`](Self::init_debug), if connected.
    pub fn dpidr(&self) -> Option<u32> {
        self.core.dpidr()
    }

    /// Tighten the transfer-retry and power-up poll budgets for transient target detection.
    pub fn set_detect_budget(&mut self, wait_retries: usize, powerup_polls: u32) {
        self.core.set_detect_budget(wait_retries, powerup_polls);
    }

    /// Clear the debug port's sticky error flags via ABORT.
    pub fn clear_sticky_errors(&mut self) -> Result<(), Error> {
        self.core.clear_sticky_errors()
    }

    /// Cancel any outstanding access-port transaction and clear latched error flags, returning a
    /// detection port to a usable state without re-waking it.
    pub fn detect_abort(&mut self) {
        self.core.detect_abort()
    }

    /// Read a word during target detection: a failed read is recovered and retried once, and
    /// the port is left usable either way.
    pub fn detect_read_word(&mut self, addr: u32) -> Result<u32, Error> {
        self.core.detect_read_word(addr)
    }

    /// Select which access port subsequent memory operations target.
    pub fn set_apsel(&mut self, apsel: u32) {
        self.core.set_apsel(apsel);
    }

    /// Set the debug-port connection policy applied by [`init_debug`](Self::init_debug).
    pub fn set_dp_config(&mut self, dp: DebugPort) {
        self.core.set_dp_config(dp);
    }

    /// Consume the host, returning the wrapped driver and delay.
    pub fn into_parts(self) -> (S, D) {
        (self.core.swd, self.delay)
    }

    /// Borrow both the SWD driver and the delay source (for lifecycle callbacks).
    pub fn parts_mut(&mut self) -> (&mut S, &mut D) {
        (&mut self.core.swd, &mut self.delay)
    }

    /// Read a single 32-bit word.
    pub fn read_word(&mut self, addr: u32) -> Result<u32, Error> {
        self.core.read_word(addr)
    }

    /// Write a single 32-bit word.
    pub fn write_word(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        self.core.write_word(addr, value)
    }

    /// Read `data.len()` bytes starting at `addr`; word-aligned, word-multiple only.
    pub fn read_memory(&mut self, addr: u32, data: &mut [u8]) -> Result<(), Error> {
        self.core.read_memory(addr, data)
    }

    /// Read `words` words from `addr`, handing each `(address, value)` to `sink` as it arrives;
    /// `sink` returns `false` to stop early. `addr` must be word-aligned.
    pub fn read_words(
        &mut self,
        addr: u32,
        words: usize,
        sink: &mut dyn FnMut(u32, u32) -> bool,
    ) -> Result<(), Error> {
        self.core.read_words(addr, words, sink)
    }

    /// Write `data` starting at `addr` using TAR auto-increment within the 1KiB wrap window.
    pub fn write_memory(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.core.write_memory(addr, data)
    }

    /// Read core register `reg` (0-15 GP, 16 xPSR, etc.).
    pub fn read_core_reg(&mut self, reg: u16) -> Result<u32, Error> {
        self.core.read_core_reg(reg)
    }

    /// Write core register `reg`.
    pub fn write_core_reg(&mut self, reg: u16, value: u32) -> Result<(), Error> {
        self.core.write_core_reg(reg, value)
    }

    /// Perform the SWD line reset / JTAG-to-SWD switch, read DPIDR, power up the debug
    /// domains and clear sticky errors.
    pub fn init_debug(&mut self) -> Result<(), Error> {
        self.core.init_debug()
    }

    /// Halt the core (set C_HALT | C_DEBUGEN via DHCSR) and wait for the halt to take effect.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] if the core does not report halted within the poll budget, or the
    /// transfer error the last poll failed with.
    pub fn halt(&mut self) -> Result<(), Error> {
        self.wait_for_halt(HaltSource::Requested)
    }

    /// Poll DHCSR until the core reports halted, bounded by [`HALT_POLL_BUDGET_US`]. Its `defmt`
    /// twin below is a separate body deliberately, and the two must stay in step.
    #[cfg(not(feature = "defmt"))]
    fn wait_for_halt(&mut self, source: HaltSource) -> Result<(), Error> {
        let mut requested = source == HaltSource::VectorCatch;
        let mut may_reinit = true;
        let mut backoff_us = HALT_POLL_START_US;
        let mut elapsed_us: u64 = 0;
        let mut last_err = None;
        while elapsed_us < HALT_POLL_BUDGET_US {
            self.delay.delay_us(backoff_us);
            elapsed_us += u64::from(backoff_us);
            if !requested {
                match self.core.request_halt() {
                    Ok(()) => requested = true,
                    Err(e) => {
                        last_err = Some(e);
                        self.help_a_refusing_port(&mut may_reinit);
                    }
                }
            } else {
                match self.core.read_dhcsr() {
                    Ok(dhcsr) => {
                        last_err = None;
                        if dhcsr & S_HALT != 0 {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        last_err = Some(e);
                        self.help_a_refusing_port(&mut may_reinit);
                    }
                }
            }
            backoff_us = backoff_us.saturating_mul(2).min(HALT_POLL_MAX_US);
        }
        Err(last_err.unwrap_or(Error::Timeout))
    }

    /// Recover a port that refused a transfer: cancel the outstanding transaction, reconnect once.
    fn help_a_refusing_port(&mut self, may_reinit: &mut bool) {
        self.core.abort_transaction();
        if *may_reinit {
            *may_reinit = false;
            let _ = self.core.init_debug();
        }
    }

    /// The instrumented twin of [`wait_for_halt`](Self::wait_for_halt), reporting how it ended.
    // `first` is kept alongside `last` because S_RESET_ST/S_RETIRE_ST are clear-on-read.
    #[cfg(feature = "defmt")]
    fn wait_for_halt(&mut self, source: HaltSource) -> Result<(), Error> {
        let mut requested = source == HaltSource::VectorCatch;
        let mut may_reinit = true;
        let mut backoff_us = HALT_POLL_START_US;
        let mut elapsed_us: u64 = 0;
        let mut last_err = None;
        let mut answered = 0u32;
        let mut first = None;
        let mut last = None;
        let mut reconnected = false;
        while elapsed_us < HALT_POLL_BUDGET_US {
            self.delay.delay_us(backoff_us);
            elapsed_us += u64::from(backoff_us);
            let outcome = if !requested {
                self.core.request_halt().map(|()| {
                    requested = true;
                    None
                })
            } else {
                self.core.read_dhcsr().map(Some)
            };
            match outcome {
                Ok(None) => {}
                Ok(Some(dhcsr)) => {
                    last_err = None;
                    answered += 1;
                    if first.is_none() {
                        first = Some(dhcsr);
                    }
                    last = Some(dhcsr);
                    if dhcsr & S_HALT != 0 {
                        crate::debug_status::log_poll_halt(
                            "halted",
                            answered,
                            elapsed_us,
                            first,
                            last,
                            reconnected,
                        );
                        return Ok(());
                    }
                }
                Err(e) => {
                    last_err = Some(e);
                    if !may_reinit {
                        self.core.abort_transaction();
                    } else {
                        reconnected = true;
                        defmt::warn!(
                            "wait_for_halt: transfer failed ({}) after {=u32} answered polls and {=u64} us; cancelling the outstanding transaction and spending the one tolerated reconnect",
                            e,
                            answered,
                            elapsed_us
                        );
                        self.help_a_refusing_port(&mut may_reinit);
                    }
                }
            }
            backoff_us = backoff_us.saturating_mul(2).min(HALT_POLL_MAX_US);
        }
        let outcome = if last_err.is_some() {
            "budget exhausted, last transfer failed"
        } else {
            "budget exhausted, core never halted"
        };
        crate::debug_status::log_poll_halt(outcome, answered, elapsed_us, first, last, reconnected);
        Err(last_err.unwrap_or(Error::Timeout))
    }

    /// Invoke a flash-algorithm routine using the standard calling convention; returns r0.
    pub fn flash_syscall(
        &mut self,
        params: &SyscallParams,
        entry: u32,
        args: [u32; 4],
    ) -> Result<u32, Error> {
        // r0..r3 = arguments.
        for (i, arg) in args.iter().enumerate() {
            self.core.write_core_reg(i as u16, *arg)?;
        }
        self.core.write_core_reg(9, params.static_base)?;
        self.core.write_core_reg(13, params.stack_pointer)?;
        // LR = breakpoint with the Thumb bit set so the return traps in Thumb state.
        self.core.write_core_reg(14, params.breakpoint | 1)?;
        self.core.write_core_reg(15, entry)?;
        // xPSR: set Thumb bit + thread mode; Cortex-M only executes Thumb code.
        self.core.write_core_reg(16, 0x0100_0000)?;

        #[cfg(feature = "log")]
        log::debug!(
            "flash_syscall: entry={:#010x} r0={:#x} r1={:#x} r2={:#x} r3={:#x}",
            entry,
            args[0],
            args[1],
            args[2],
            args[3]
        );

        // Resume: clear C_HALT, keep debug enabled.
        self.core.write_word(DHCSR, DBGKEY | C_DEBUGEN)?;

        // Delay before the first read: S_HALT still set from before the resume returns a stale r0.
        let mut halted = false;
        let mut backoff_us = SYSCALL_POLL_START_US;
        let mut elapsed_us: u64 = 0;
        while elapsed_us < SYSCALL_TIMEOUT_US {
            self.delay.delay_us(backoff_us);
            elapsed_us += u64::from(backoff_us);
            if self.core.read_word(DHCSR)? & S_HALT != 0 {
                halted = true;
                break;
            }
            backoff_us = backoff_us.saturating_mul(2).min(SYSCALL_POLL_MAX_US);
        }
        if !halted {
            #[cfg(feature = "log")]
            log::warn!(
                "flash_syscall: AlgoTimeout at entry={:#010x} after ~{} ms",
                entry,
                elapsed_us / 1000
            );
            return Err(Error::AlgoTimeout);
        }

        let ret = self.core.read_core_reg(0)?;
        #[cfg(feature = "log")]
        log::debug!(
            "flash_syscall: entry={:#010x} -> r0={:#010x} (~{} ms)",
            entry,
            ret,
            elapsed_us / 1000
        );
        Ok(ret)
    }
}

impl<S, D, DEPS> SwdHost<S, D, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
{
    /// Trigger a reset of the selected type; a hardware reset re-initializes debug afterward.
    fn trigger_reset(&mut self, reset_type: ResetType) -> Result<(), Error> {
        match reset_type {
            ResetType::Hardware => {
                self.core.set_reset(true)?;
                self.delay.delay_us(HW_RESET_ASSERT_US);
                self.core.set_reset(false)?;
                self.delay.delay_us(HW_RESET_RECOVER_US);
                self.core.init_debug()?;
            }
            ResetType::SoftwareSysReset => self.request_software_reset(AIRCR_KEY | SYSRESETREQ),
            ResetType::SoftwareVectReset => self.request_software_reset(AIRCR_KEY | VECTRESET),
        }
        Ok(())
    }

    /// Write `value` to AIRCR to request a software reset, tolerating the write's own failure and
    /// aborting any latched fault so it cannot consume the following halt poll's budget.
    fn request_software_reset(&mut self, value: u32) {
        if self.core.write_word(AIRCR, value).is_err() {
            self.core.abort_transaction();
        }
    }

    /// Drive the target into `state` using [`GENERIC_RESET_SEQUENCE`], the default per-device
    /// [`ResetSequence`].
    pub fn set_target_state(
        &mut self,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        GENERIC_RESET_SEQUENCE.set_target_state(self, state, reset_type)
    }

    /// Drive the target into `state` using the supplied [`ResetSequence`], dispatching any
    /// reset it entails on `reset_type`.
    pub fn run_reset_sequence(
        &mut self,
        sequence: &dyn ResetSequence,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        sequence.set_target_state(self, state, reset_type)
    }

    /// Run the supplied sequence's family unlock step against this host, threading `policy`.
    pub fn run_unlock_sequence(
        &mut self,
        sequence: &dyn ResetSequence,
        policy: crate::reset_sequence::UnlockPolicy,
    ) -> Result<crate::reset_sequence::UnlockOutcome, Error> {
        sequence.unlock(self, policy)
    }
}

/// Expose this host's debug primitives to the reset-sequence seam.
impl<S, D, DEPS> ResetOps for SwdHost<S, D, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
{
    fn write_word(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        self.core.write_word(addr, value)
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Error> {
        self.core.read_word(addr)
    }

    fn trigger_reset(&mut self, reset_type: ResetType) -> Result<(), Error> {
        SwdHost::trigger_reset(self, reset_type)
    }

    fn poll_halt(&mut self) -> Result<(), Error> {
        self.wait_for_halt(HaltSource::VectorCatch)
    }

    fn set_reset(&mut self, asserted: bool) -> Result<(), Error> {
        self.core.set_reset(asserted)
    }

    fn halt(&mut self) -> Result<(), Error> {
        SwdHost::halt(self)
    }

    fn ap_read_raw(&mut self, apsel: u8, addr: u8) -> Result<u32, Error> {
        self.core.ap_read_raw(apsel, addr)
    }

    fn ap_write_raw(&mut self, apsel: u8, addr: u8, value: u32) -> Result<(), Error> {
        self.core.ap_write_raw(apsel, addr, value)
    }

    fn delay_us(&mut self, us: u32) -> Result<(), Error> {
        self.delay.delay_us(us);
        Ok(())
    }

    fn reconnect(&mut self) -> Result<(), Error> {
        self.core.init_debug()
    }

    #[cfg(feature = "defmt")]
    fn dp_ctrl_stat(&mut self) -> Result<u32, Error> {
        self.core.ctrl_stat()
    }
}

#[cfg(test)]
mod reset_sequence_tests {
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult};
    use std::vec::Vec;

    /// One thing the mock saw, in order (ordering, not just counts, is under test).
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum Ev {
        MemWrite(u32, u32),
        DhcsrPoll(u32),
        /// A DHCSR read the port refused (`AckWait`, from a stalled access port).
        DhcsrRefused,
        DapAbort,
        StickyClear,
    }

    /// Recording SWD mock: logs word writes, DHCSR polls, nRESET assertions and line resets, and
    /// is scriptable for the reset window through the fields below (inert at their defaults).
    struct RecMock {
        cfg: Config,
        last_tar: u32,
        writes: Vec<(u32, u32)>,
        resets: Vec<bool>,
        dhcsr_polls: u32,
        seq_count: u32,
        events: Vec<Ev>,
        /// An access-port transaction that never completed: only `DAPABORT` ends it.
        stalled: bool,
        /// The access port answers WAIT to everything and `DAPABORT` does not clear it.
        wait_always: bool,
        /// The AIRCR write faults and leaves the port stalled.
        stall_on_reset_write: bool,
        /// Answered DHCSR reads still to report as not-yet-halted before reporting `S_HALT`.
        polls_before_halt: u32,
        /// DHCSR reads to refuse (`AckWait`) once the reset has been requested.
        refuse_after_reset: u32,
        /// Refusals still owed, counted down one per refused read.
        refuse_polls: u32,
        /// Whether the read in progress is one of the refused ones.
        refusing: bool,
        /// The core never reports halted, however long it is polled.
        never_halts: bool,
        dap_aborts: u32,
    }

    impl RecMock {
        fn new() -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                writes: Vec::new(),
                resets: Vec::new(),
                dhcsr_polls: 0,
                seq_count: 0,
                events: Vec::new(),
                stalled: false,
                wait_always: false,
                stall_on_reset_write: false,
                polls_before_halt: 0,
                refuse_after_reset: 0,
                refuse_polls: 0,
                refusing: false,
                never_halts: false,
                dap_aborts: 0,
            }
        }

        fn wrote(&self, addr: u32, value: u32) -> bool {
            self.writes.iter().any(|&(a, v)| a == addr && v == value)
        }

        /// Index of the first recorded event matching `pred`, for ordering assertions.
        fn first(&self, pred: impl Fn(&Ev) -> bool) -> Option<usize> {
            self.events.iter().position(pred)
        }

        /// Whether the access port refuses everything.
        fn port_stalled(&self) -> bool {
            self.stalled || self.wait_always
        }

        /// Whether the read in progress is refused.
        fn ap_blocked(&self) -> bool {
            self.port_stalled() || self.refusing
        }
    }

    impl Swd<RecMock> for RecMock {
        fn available(_deps: &RecMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if self.ap_blocked() {
                        return Err(SwdErr::AckWait);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => {
                    let polling_dhcsr = self.last_tar == DHCSR;
                    if self.ap_blocked() {
                        if polling_dhcsr && !self.refusing {
                            self.events.push(Ev::DhcsrRefused);
                        }
                        return Err(SwdErr::AckWait);
                    }
                    if polling_dhcsr {
                        self.dhcsr_polls += 1;
                        let value = if self.never_halts || self.polls_before_halt > 0 {
                            self.polls_before_halt = self.polls_before_halt.saturating_sub(1);
                            C_DEBUGEN
                        } else {
                            S_HALT
                        };
                        self.events.push(Ev::DhcsrPoll(value));
                        Ok(value)
                    } else {
                        Ok(0)
                    }
                }
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_DAP => {
                    self.stalled = false;
                    self.dap_aborts += 1;
                    self.events.push(Ev::DapAbort);
                }
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.events.push(Ev::StickyClear);
                }
                (APnDP::AP, _) if self.port_stalled() => return Err(SwdErr::AckWait),
                // TAR: a refused-DHCSR-read is decided once per read, not in the retried read itself.
                (APnDP::AP, DPRegister::CTRLSTAT) => {
                    self.last_tar = data;
                    if data == DHCSR {
                        self.refusing = self.refuse_polls > 0;
                        if self.refusing {
                            self.refuse_polls -= 1;
                            self.events.push(Ev::DhcsrRefused);
                        }
                    }
                }
                (APnDP::AP, DPRegister::RDBUFF) => {
                    if self.last_tar == AIRCR {
                        self.refuse_polls = self.refuse_after_reset;
                    }
                    if self.stall_on_reset_write && self.last_tar == AIRCR {
                        self.stalled = true;
                        self.events.push(Ev::MemWrite(self.last_tar, data));
                        self.writes.push((self.last_tar, data));
                        return Err(SwdErr::AckFault);
                    }
                    self.writes.push((self.last_tar, data));
                    self.events.push(Ev::MemWrite(self.last_tar, data));
                }
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            self.seq_count += 1;
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    impl ResetControl for RecMock {
        type Error = SwdErr;
        fn set_reset(&mut self, asserted: bool) -> Result<(), SwdErr> {
            self.resets.push(asserted);
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// Delay source that records every wait instead of sleeping.
    #[derive(Default)]
    struct RecDelay {
        waits: Vec<u32>,
    }

    impl RecDelay {
        fn total_us(&self) -> u64 {
            self.waits.iter().map(|&us| u64::from(us)).sum()
        }
    }

    impl DelayNs for RecDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.waits.push(ns / 1000);
        }
        fn delay_us(&mut self, us: u32) {
            self.waits.push(us);
        }
    }

    fn run(state: TargetState, reset: ResetType) -> RecMock {
        let mut host = SwdHost::new(RecMock::new(), NoDelay);
        host.set_target_state(state, reset)
            .expect("set_target_state");
        host.into_parts().0
    }

    /// Drive a scripted mock through the software-system-reset reset-and-halt.
    fn run_scripted(m: RecMock, state: TargetState) -> (Result<(), Error>, RecMock, RecDelay) {
        let mut host = SwdHost::new(m, RecDelay::default());
        let outcome = host.set_target_state(state, ResetType::SoftwareSysReset);
        let (m, d) = host.into_parts();
        (outcome, m, d)
    }

    #[test]
    fn sysreset_program_writes_aircr_and_polls_without_reset_line() {
        let m = run(TargetState::ResetProgram, ResetType::SoftwareSysReset);
        assert!(m.wrote(DEMCR, TRCENA | VC_CORERESET), "arm vector catch");
        assert!(m.wrote(DHCSR, DBGKEY | C_DEBUGEN), "keep debug enabled");
        assert!(
            m.wrote(AIRCR, AIRCR_KEY | SYSRESETREQ),
            "system reset request"
        );
        assert!(m.wrote(DEMCR, TRCENA), "disarm vector catch");
        assert!(m.dhcsr_polls >= 1, "polled S_HALT");
        assert!(m.resets.is_empty(), "software reset must not toggle nRESET");
    }

    #[test]
    fn vectreset_program_writes_vectreset() {
        let m = run(TargetState::ResetProgram, ResetType::SoftwareVectReset);
        assert!(m.wrote(DEMCR, TRCENA | VC_CORERESET));
        assert!(
            m.wrote(AIRCR, AIRCR_KEY | VECTRESET),
            "vector reset request"
        );
        assert!(m.wrote(DEMCR, TRCENA));
        assert!(m.dhcsr_polls >= 1);
        assert!(m.resets.is_empty());
    }

    #[test]
    fn hardware_program_toggles_reset_reinits_and_halts() {
        let m = run(TargetState::ResetProgram, ResetType::Hardware);
        assert!(m.wrote(DEMCR, TRCENA | VC_CORERESET));
        assert!(m.wrote(DHCSR, DBGKEY | C_DEBUGEN));
        assert_eq!(
            m.resets,
            std::vec![true, false],
            "assert then release nRESET"
        );
        assert!(m.seq_count > 0, "re-init debug after the reset pulse");
        assert!(
            !m.writes.iter().any(|&(a, _)| a == AIRCR),
            "hardware reset must not write AIRCR"
        );
        assert!(m.dhcsr_polls >= 1, "polled S_HALT");
        assert!(m.wrote(DEMCR, TRCENA), "disarm vector catch");
    }

    #[test]
    fn sysreset_post_flash_releases_and_resets_without_polling() {
        let m = run(TargetState::PostFlashReset, ResetType::SoftwareSysReset);
        assert!(m.wrote(DEMCR, TRCENA), "clear any vector catch");
        assert!(m.wrote(DHCSR, DBGKEY), "release debug (C_DEBUGEN clear)");
        assert!(m.wrote(AIRCR, AIRCR_KEY | SYSRESETREQ));
        assert_eq!(m.dhcsr_polls, 0, "reset-to-run must not poll S_HALT");
        assert!(m.resets.is_empty());
    }

    #[test]
    fn hardware_post_flash_pulses_reset_without_polling() {
        let m = run(TargetState::PostFlashReset, ResetType::Hardware);
        assert!(m.wrote(DEMCR, TRCENA));
        assert!(m.wrote(DHCSR, DBGKEY));
        assert_eq!(m.resets, std::vec![true, false]);
        assert_eq!(m.dhcsr_polls, 0, "reset-to-run must not poll S_HALT");
    }

    /// The halt poll waits between reads, starting short and doubling up to a cap.
    #[test]
    fn the_halt_poll_gives_the_target_time_between_reads() {
        let mut m = RecMock::new();
        m.polls_before_halt = 5;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(outcome, Ok(()), "a core that halts late still halts");
        assert!(m.dhcsr_polls > 5, "the poll kept asking");

        // Skip the pre-reset poll's own backoff; only the reset window's waits are checked.
        let settle = d
            .waits
            .iter()
            .position(|&us| us == crate::reset_sequence::RESET_SETTLE_US)
            .expect("the sequence settled after the reset");
        let polls: Vec<u32> = d.waits[settle + 1..].to_vec();
        assert!(!polls.is_empty(), "the poll waited between reads");
        assert_eq!(
            polls[0], HALT_POLL_START_US,
            "the first read comes after the starting delay, not immediately"
        );
        for pair in polls.windows(2) {
            let (prev, next) = (pair[0], pair[1]);
            assert!(
                next == HALT_POLL_MAX_US || next == prev * 2,
                "each wait doubles until the cap: {prev} -> {next}"
            );
            assert!(next <= HALT_POLL_MAX_US, "no wait exceeds the cap");
        }
    }

    /// The reset write's fault stalls the access port; the cancel must land between the reset
    /// write and the first halt poll, or every access after it answers WAIT forever.
    #[test]
    fn the_reset_write_cancels_the_transaction_its_tolerated_fault_left() {
        let mut m = RecMock::new();
        m.stall_on_reset_write = true;
        let (outcome, m, _) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(outcome, Ok(()), "the reset-and-halt completes");
        let reset_write = m
            .first(|e| matches!(e, Ev::MemWrite(AIRCR, _)))
            .expect("the AIRCR write was attempted");
        let cancel = m
            .first(|e| matches!(e, Ev::DapAbort))
            .expect("the stalled transaction was cancelled");
        let first_poll = m
            .events
            .iter()
            .skip(reset_write)
            .position(|e| matches!(e, Ev::DhcsrPoll(_) | Ev::DhcsrRefused))
            .map(|i| i + reset_write)
            .expect("the halt poll ran after the reset");
        assert!(
            reset_write < cancel && cancel < first_poll,
            "the cancel falls between the reset write and the first halt poll \
             (events: {:?})",
            m.events
        );
    }

    /// The poll must survive refused (WAIT) reads after a reset rather than fail on the first one.
    #[test]
    fn the_halt_poll_waits_out_an_access_port_that_answers_wait() {
        let mut m = RecMock::new();
        m.refuse_after_reset = 3;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(outcome, Ok(()), "a port that comes back is not a failure");
        assert!(
            m.first(|e| matches!(e, Ev::DhcsrRefused)).is_some(),
            "the poll met the WAIT it is supposed to survive"
        );
        assert!(
            m.first(|e| matches!(e, Ev::DhcsrPoll(_))).is_some(),
            "and went on to read DHCSR once the port answered"
        );
        assert!(d.total_us() > 0, "time was actually given to the target");
    }

    /// A core that never halts is polled for the whole budget and then fails with a timeout.
    #[test]
    fn a_core_that_never_halts_exhausts_the_budget_and_fails() {
        let mut m = RecMock::new();
        m.never_halts = true;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(
            outcome,
            Err(Error::Timeout),
            "the poll fails rather than waiting forever"
        );
        assert!(
            d.total_us() >= HALT_POLL_BUDGET_US,
            "the whole budget was given to the target first: {} us",
            d.total_us()
        );
        assert!(
            m.dhcsr_polls < 1_000,
            "the budget is spent in time, not in transfers: {} polls",
            m.dhcsr_polls
        );
    }

    /// A port that never answers reports the WAIT as a transfer error, not silent success.
    #[test]
    fn a_port_that_only_ever_waits_fails_with_the_transfer_error() {
        let mut m = RecMock::new();
        m.wait_always = true;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(
            outcome,
            Err(Error::Swd(crate::error::SwdError::Wait)),
            "the exhausted WAIT is what the caller is told about"
        );
        assert!(
            d.total_us() >= HALT_POLL_BUDGET_US,
            "the port was given the whole budget: {} us",
            d.total_us()
        );
        assert!(
            m.dap_aborts > 0,
            "and the one recovery a WAIT-stalled port responds to was attempted"
        );
    }
}

#[cfg(test)]
mod connect_tests {
    //! Debug-port connection tests: wake preambles, multidrop TARGETSEL, ADIv6 addressing.
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use crate::target::{ApAddressing, DebugPort, DpWake};
    use dap_rs::swd::{APnDP, Config, DPRegister, Result as SwdResult};
    use std::vec::Vec;

    struct WireMock {
        cfg: Config,
        /// Every emitted raw sequence, as (num_bits, little-endian bytes).
        seqs: Vec<(usize, Vec<u8>)>,
        select_writes: Vec<u32>,
        /// Last SELECT value (its low nibble is the DP bank the next read observes).
        select: u32,
        dpidr: u32,
        targetid: u32,
        dlpidr: u32,
    }

    impl WireMock {
        fn new() -> Self {
            Self {
                cfg: Config::default(),
                seqs: Vec::new(),
                select_writes: Vec::new(),
                select: 0,
                dpidr: 0x0BC1_1477,
                targetid: 0,
                dlpidr: 0,
            }
        }
        /// Whether a raw sequence of exactly `num_bits` carrying `bytes` was emitted.
        fn emitted(&self, num_bits: usize, bytes: &[u8]) -> bool {
            self.seqs
                .iter()
                .any(|(n, b)| *n == num_bits && b.as_slice() == bytes)
        }
    }

    impl Swd<WireMock> for WireMock {
        fn available(_deps: &WireMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => Ok(0),
                (APnDP::DP, DPRegister::DPIDR) => Ok(self.dpidr),
                (APnDP::DP, DPRegister::CTRLSTAT) => Ok(match self.select & 0xF {
                    2 => self.targetid,
                    3 => self.dlpidr,
                    _ => CSYSPWRUPACK | CDBGPWRUPACK,
                }),
                (APnDP::DP, _) => Ok(0),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            if apndp == APnDP::DP && a == DPRegister::SELECT {
                self.select = data;
                self.select_writes.push(data);
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> SwdResult<()> {
            self.seqs.push((num_bits, data.to_vec()));
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    fn host(m: WireMock) -> SwdHost<WireMock, NoDelay, WireMock> {
        SwdHost::new(m, NoDelay)
    }

    #[test]
    fn classic_wake_emits_jtag_to_swd_switch_not_dormant_alert() {
        let mut h = host(WireMock::new());
        h.set_dp_config(DebugPort::default());
        h.init_debug().expect("classic connect");
        let (m, _) = h.into_parts();
        assert!(
            m.emitted(16, &0xE79Eu16.to_le_bytes()),
            "classic switch word present"
        );
        assert!(
            !m.emitted(64, &SELECTION_ALERT_LO.to_le_bytes()),
            "no dormant selection alert on the classic path"
        );
    }

    #[test]
    fn dormant_wake_emits_selection_alert_and_activation() {
        let mut h = host(WireMock::new());
        h.set_dp_config(DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: None,
            ap: ApAddressing::V6 { ap_base: 0x2000 },
        });
        h.init_debug().expect("dormant connect");
        let (m, _) = h.into_parts();
        assert!(
            m.emitted(31, &SWD_TO_DORMANT),
            "swd->dormant (31-bit select)"
        );
        assert!(m.emitted(64, &SELECTION_ALERT_LO.to_le_bytes()), "alert lo");
        assert!(m.emitted(64, &SELECTION_ALERT_HI.to_le_bytes()), "alert hi");
        assert!(m.emitted(12, &SWD_ACTIVATION.to_le_bytes()), "activation");
        assert!(
            !m.emitted(16, &0xE79Eu16.to_le_bytes()),
            "no classic switch on the dormant path"
        );
    }

    /// Blind 48-bit TARGETSEL matching the requested value, followed by a passing verify.
    #[test]
    fn multidrop_emits_blind_targetsel_and_verifies() {
        const TARGETSEL: u32 = 0x0100_2927;
        let mut m = WireMock::new();
        m.targetid = TARGETSEL & TARGETID_MASK;
        m.dlpidr = TARGETSEL & DLPIDR_MASK;
        let mut h = host(m);
        h.set_dp_config(DebugPort {
            wake: DpWake::JtagToSwd,
            targetsel: Some(TARGETSEL),
            ap: ApAddressing::V5,
        });
        h.init_debug().expect("multidrop connect");
        let (m, _) = h.into_parts();

        let parity = u64::from(TARGETSEL.count_ones() & 1);
        let seq = (parity << 45) | (u64::from(TARGETSEL) << 13) | 0x1F99;
        assert!(
            m.emitted(48, &seq.to_le_bytes()[..6]),
            "blind 48-bit TARGETSEL emitted"
        );
        assert!(
            m.select_writes.contains(&2),
            "selected DP bank 2 (TARGETID)"
        );
        assert!(m.select_writes.contains(&3), "selected DP bank 3 (DLPIDR)");
    }

    #[test]
    fn multidrop_verify_mismatch_fails() {
        const TARGETSEL: u32 = 0x0100_2927;
        let mut m = WireMock::new();
        m.targetid = 0x0BAD_BEEF & TARGETID_MASK;
        m.dlpidr = TARGETSEL & DLPIDR_MASK;
        let mut h = host(m);
        h.set_dp_config(DebugPort {
            wake: DpWake::JtagToSwd,
            targetsel: Some(TARGETSEL),
            ap: ApAddressing::V5,
        });
        assert_eq!(h.init_debug().unwrap_err(), Error::DebugInitFailed);
    }

    /// ADIv6 programs SELECT with the AP base ORed with the 0xD00 register page.
    #[test]
    fn adiv6_programs_ap_base_page_into_select() {
        let mut h = host(WireMock::new());
        h.set_dp_config(DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: None,
            ap: ApAddressing::V6 { ap_base: 0x2000 },
        });
        h.init_debug().expect("adiv6 connect");
        let _ = h.read_word(0x4000_0000).expect("mem read over adiv6");
        let (m, _) = h.into_parts();
        assert!(
            m.select_writes.contains(&0x2D00),
            "AP base 0x2000 | 0xD00 register page written to SELECT"
        );
        assert!(
            !m.select_writes.contains(&0x2000),
            "ADIv6 must target the 0xD00 register page, not the bare AP base 0x2000"
        );
        assert!(
            !m.select_writes.contains(&0),
            "ADIv6 must not fall back to the ADIv5 apsel<<24 = 0 SELECT"
        );
    }

    /// A dormant-wake ADIv6 attach with a multidrop `TARGETSEL` emits the terminated line reset
    /// and blind TARGETSEL before reading DPIDR.
    #[test]
    fn dormant_wake_with_targetsel_emits_line_reset_and_blind_targetsel() {
        const TARGETSEL: u32 = 0x0004_0927;
        let mut m = WireMock::new();
        m.targetid = TARGETSEL & TARGETID_MASK;
        m.dlpidr = TARGETSEL & DLPIDR_MASK;
        let mut h = host(m);
        h.set_dp_config(DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: Some(TARGETSEL),
            ap: ApAddressing::V6 { ap_base: 0x2000 },
        });
        h.init_debug().expect("dormant + multidrop connect");
        let (m, _) = h.into_parts();

        assert!(m.emitted(64, &SELECTION_ALERT_LO.to_le_bytes()), "alert lo");
        assert!(
            m.emitted(54, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]),
            "terminated line reset before TARGETSEL / DPIDR"
        );
        let parity = u64::from(TARGETSEL.count_ones() & 1);
        let seq = (parity << 45) | (u64::from(TARGETSEL) << 13) | 0x1F99;
        assert!(
            m.emitted(48, &seq.to_le_bytes()[..6]),
            "blind 48-bit TARGETSEL emitted on the dormant path"
        );
        assert!(m.select_writes.contains(&2), "read TARGETID (DP bank 2)");
        assert!(m.select_writes.contains(&3), "read DLPIDR (DP bank 3)");
    }
}

#[cfg(test)]
mod detect_tests {
    //! Detection-read tests: a faulted read recovers and leaves the port usable.
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult};

    /// SWD mock with a poisoned address: reading it faults and latches a sticky error.
    struct FaultMock {
        cfg: Config,
        last_tar: u32,
        poison: u32,
        mem_value: u32,
        sticky: bool,
        abort_writes: u32,
        dap_aborts: u32,
    }

    impl FaultMock {
        fn new(poison: u32, mem_value: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                poison,
                mem_value,
                sticky: false,
                abort_writes: 0,
                dap_aborts: 0,
            }
        }
    }

    impl Swd<FaultMock> for FaultMock {
        fn available(_deps: &FaultMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if self.sticky {
                        return Err(SwdErr::AckFault);
                    }
                    if self.last_tar == self.poison {
                        self.sticky = true;
                        return Err(SwdErr::AckFault);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => {
                    if self.sticky {
                        Err(SwdErr::AckFault)
                    } else {
                        Ok(self.mem_value)
                    }
                }
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::AP, DPRegister::CTRLSTAT) => self.last_tar = data,
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.sticky = false;
                    self.abort_writes += 1;
                }
                // DAPABORT: nothing is stalled in this model, so it only records the cancel.
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_DAP => self.dap_aborts += 1,
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// A faulted detection read recovers the port; a following good-address read succeeds.
    #[test]
    fn detect_read_word_clears_sticky_and_recovers() {
        const POISON: u32 = 0x1000_0100;
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(FaultMock::new(POISON, 0x0000_0410), NoDelay);
        host.set_detect_budget(8, 64);

        assert!(
            host.detect_read_word(POISON).is_err(),
            "foreign read faults"
        );
        assert_eq!(
            host.core.swd.dap_aborts, 2,
            "each failed attempt ends by cancelling the transaction"
        );
        assert_eq!(
            host.core.swd.abort_writes, 4,
            "per attempt: one clear from the transfer retry, one from the detection recovery"
        );
        assert!(
            !host.core.swd.sticky,
            "sticky error cleared after the faulted read"
        );

        assert_eq!(
            host.detect_read_word(GOOD),
            Ok(0x0000_0410),
            "a good read succeeds after recovery"
        );
    }

    /// A persistently faulting address fails a plain `read_word`, but the retry's own recovery
    /// re-arms the port so the next, good address succeeds without an explicit clear.
    #[test]
    fn plain_read_self_heals_on_the_next_transfer() {
        const POISON: u32 = 0x1000_0100;
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(FaultMock::new(POISON, 0x0000_0410), NoDelay);

        assert!(host.read_word(POISON).is_err(), "poison read faults");
        assert!(
            host.core.swd.sticky,
            "the failed retry left the fault latched"
        );
        assert_eq!(host.read_word(GOOD), Ok(0x0000_0410));
        assert_eq!(
            host.core.swd.abort_writes, 2,
            "one ABORT per recovering transfer"
        );
    }

    /// SWD mock modeling a real MEM-AP: an unmapped read latches sticky flags AND leaves the
    /// access port with a transaction it never completed (only `DAPABORT` clears that).
    struct StallMock {
        cfg: Config,
        last_tar: u32,
        poison: u32,
        mem_value: u32,
        /// Latched error flags: acknowledged by an `ABORT_CLEAR_ALL` write.
        sticky: bool,
        /// An access-port transaction that never completed: cancelled only by `ABORT_DAP`.
        stalled: bool,
        sticky_clears: u32,
        dap_aborts: u32,
    }

    impl StallMock {
        fn new(poison: u32, mem_value: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                poison,
                mem_value,
                sticky: false,
                stalled: false,
                sticky_clears: 0,
                dap_aborts: 0,
            }
        }

        /// The ack an access port in this state gives: WAIT while a transaction is outstanding,
        /// FAULT while errors are merely latched.
        fn blocked(&self) -> Option<SwdErr> {
            if self.stalled {
                Some(SwdErr::AckWait)
            } else if self.sticky {
                Some(SwdErr::AckFault)
            } else {
                None
            }
        }
    }

    impl Swd<StallMock> for StallMock {
        fn available(_deps: &StallMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if let Some(e) = self.blocked() {
                        return Err(e);
                    }
                    if self.last_tar == self.poison {
                        self.sticky = true;
                        self.stalled = true;
                        return Err(SwdErr::AckFault);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => match self.blocked() {
                    Some(e) => Err(e),
                    None => Ok(self.mem_value),
                },
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_DAP => {
                    self.stalled = false;
                    self.dap_aborts += 1;
                }
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.sticky = false;
                    self.sticky_clears += 1;
                }
                (APnDP::AP, _) => {
                    if let Some(e) = self.blocked() {
                        return Err(e);
                    }
                    if a == DPRegister::CTRLSTAT {
                        self.last_tar = data;
                    }
                }
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    /// A stalled access port must be cancelled, not merely acknowledged, so the next
    /// candidate's identity read is still answered.
    #[test]
    fn a_stalled_access_port_is_cancelled_so_the_next_candidate_reads() {
        const POISON: u32 = 0x1000_0100;
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(StallMock::new(POISON, 0x0000_0413), NoDelay);
        host.set_detect_budget(8, 64);

        assert!(
            host.detect_read_word(POISON).is_err(),
            "the unmapped identity address faults"
        );
        assert!(
            !host.core.swd.stalled,
            "the outstanding transaction was cancelled, not just acknowledged"
        );
        assert!(host.core.swd.dap_aborts > 0, "the cancel was actually sent");

        assert_eq!(
            host.detect_read_word(GOOD),
            Ok(0x0000_0413),
            "the following candidate's identity read is answered"
        );
    }

    /// A detection read beginning on a port already stalled by an earlier access recovers it.
    #[test]
    fn detect_read_recovers_a_port_left_stalled_by_an_earlier_access() {
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(StallMock::new(0xDEAD_0000, 0x0000_0413), NoDelay);
        host.set_detect_budget(8, 64);
        host.core.swd.stalled = true;

        assert_eq!(
            host.detect_read_word(GOOD),
            Ok(0x0000_0413),
            "a mapped address reads back despite the inherited stall"
        );
        assert_eq!(
            host.core.swd.dap_aborts, 1,
            "one cancel, then the read is retried over a usable port"
        );
    }
}

#[cfg(test)]
mod recovery_tests {
    //! Sticky-error recovery tests: one ABORT and re-arm, and no retry loop on a persistent fault.
    extern crate std;

    use super::*;
    use crate::error::SwdError;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult};
    use std::vec::Vec;

    /// SWD mock whose AP reads fault while `faults_left` is non-zero, latching a sticky error.
    struct TransientMock {
        cfg: Config,
        last_tar: u32,
        mem_value: u32,
        faults_left: u32,
        sticky: bool,
        abort_writes: u32,
        select_writes: Vec<u32>,
        csw_writes: Vec<u32>,
    }

    impl TransientMock {
        fn new(faults_left: u32, mem_value: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                mem_value,
                faults_left,
                sticky: false,
                abort_writes: 0,
                select_writes: Vec::new(),
                csw_writes: Vec::new(),
            }
        }
    }

    impl Swd<TransientMock> for TransientMock {
        fn available(_deps: &TransientMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if self.sticky {
                        return Err(SwdErr::AckFault);
                    }
                    if self.faults_left > 0 {
                        self.faults_left -= 1;
                        self.sticky = true;
                        return Err(SwdErr::AckFault);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => {
                    if self.sticky {
                        Err(SwdErr::AckFault)
                    } else {
                        Ok(self.mem_value)
                    }
                }
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.sticky = false;
                    self.abort_writes += 1;
                }
                (APnDP::DP, DPRegister::SELECT) => self.select_writes.push(data),
                (APnDP::AP, _) if self.sticky => return Err(SwdErr::AckFault),
                (APnDP::AP, DPRegister::DPIDR) => self.csw_writes.push(data),
                (APnDP::AP, DPRegister::CTRLSTAT) => self.last_tar = data,
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// One transient fault recovers with a single ABORT, re-issues cached SELECT/CSW, and the
    /// retried read returns the memory value.
    #[test]
    fn transient_fault_recovers_with_abort_select_and_retry() {
        let mut host = SwdHost::new(TransientMock::new(1, 0xDEAD_BEEF), NoDelay);
        assert_eq!(
            host.read_word(0x2000_0000),
            Ok(0xDEAD_BEEF),
            "retried read succeeds"
        );
        let m = host.into_parts().0;
        assert_eq!(m.abort_writes, 1, "recovery wrote ABORT once");
        assert_eq!(
            m.select_writes,
            std::vec![0, 0],
            "SELECT re-issued during recovery"
        );
        assert_eq!(m.csw_writes, std::vec![CSW_WORD_ADDRINC, CSW_WORD_ADDRINC]);
    }

    /// A persistent fault gets exactly one recovery attempt, then surfaces the original error.
    #[test]
    fn persistent_fault_returns_original_error_after_one_retry() {
        let mut host = SwdHost::new(TransientMock::new(u32::MAX, 0), NoDelay);
        assert_eq!(
            host.read_word(0x2000_0000),
            Err(Error::Swd(SwdError::Fault))
        );
        let m = host.into_parts().0;
        assert_eq!(m.abort_writes, 1, "exactly one recovery attempt, no loop");
    }
}

#[cfg(test)]
mod syscall_tests {
    //! Flash-syscall calling-convention tests: register setup, resume, S_HALT poll, timeout.
    extern crate std;

    use super::*;
    use crate::flash_algo::SyscallParams;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Result as SwdResult};
    use std::vec::Vec;

    /// Recording core-debug mock: logs word writes, reports a fixed DHCSR value, and hands
    /// back a fixed DCRDR (the algorithm's r0 status).
    struct CoreMock {
        cfg: Config,
        last_tar: u32,
        writes: Vec<(u32, u32)>,
        /// Value every DHCSR read reports (S_REGRDY, with or without S_HALT).
        dhcsr: u32,
        dcrdr: u32,
        dhcsr_polls: u32,
    }

    impl CoreMock {
        fn new(dhcsr: u32, dcrdr: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                writes: Vec::new(),
                dhcsr,
                dcrdr,
                dhcsr_polls: 0,
            }
        }
    }

    impl Swd<CoreMock> for CoreMock {
        fn available(_deps: &CoreMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => Ok(0),
                (APnDP::DP, DPRegister::RDBUFF) => match self.last_tar {
                    DHCSR => {
                        self.dhcsr_polls += 1;
                        Ok(self.dhcsr)
                    }
                    DCRDR => Ok(self.dcrdr),
                    _ => Ok(0),
                },
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::AP, DPRegister::CTRLSTAT) => self.last_tar = data,
                (APnDP::AP, DPRegister::RDBUFF) => self.writes.push((self.last_tar, data)),
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    const PARAMS: SyscallParams = SyscallParams {
        breakpoint: 0x2000_0004,
        static_base: 0x2000_0800,
        stack_pointer: 0x2000_1000,
    };
    const ENTRY: u32 = 0x2000_0040;

    /// The full calling convention, pinned as an ordered op-stream over the debug registers.
    #[test]
    fn flash_syscall_sets_registers_resumes_and_returns_r0() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY | S_HALT, 0x2A), NoDelay);
        let ret = host
            .flash_syscall(&PARAMS, ENTRY, [1, 2, 3, 4])
            .expect("syscall");
        assert_eq!(ret, 0x2A, "r0 read back as the return value");
        let m = host.into_parts().0;

        let stream: Vec<(u32, u32)> = m
            .writes
            .iter()
            .copied()
            .filter(|&(a, _)| a == DCRDR || a == DCRSR || a == DHCSR)
            .collect();
        let mut expect: Vec<(u32, u32)> = Vec::new();
        for (reg, val) in [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 4),
            (9, PARAMS.static_base),
            (13, PARAMS.stack_pointer),
            (14, PARAMS.breakpoint | 1),
            (15, ENTRY),
            (16, 0x0100_0000),
        ] {
            expect.push((DCRDR, val));
            expect.push((DCRSR, reg | (1 << 16)));
        }
        expect.push((DHCSR, DBGKEY | C_DEBUGEN));
        expect.push((DCRSR, 0));
        assert_eq!(stream, expect);
        assert!(m.dhcsr_polls >= 1, "polled S_HALT after the resume");
    }

    /// A core that never reports S_HALT after resume exhausts the budget and surfaces `AlgoTimeout`.
    #[test]
    fn flash_syscall_times_out_when_core_never_halts() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY, 0), NoDelay);
        assert_eq!(
            host.flash_syscall(&PARAMS, ENTRY, [0; 4]),
            Err(Error::AlgoTimeout)
        );
        let m = host.into_parts().0;
        assert!(
            m.dhcsr_polls > 100,
            "backoff kept polling until the budget elapsed"
        );
    }
}

#[cfg(test)]
mod write_memory_tests {
    //! `write_memory` must explicitly re-arm TAR when a transfer crosses a 1KiB boundary.
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Result as SwdResult};
    use std::vec::Vec;

    /// One recorded AP register write, tagged by which CSW/TAR/DRW role it played.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ApWrite {
        Csw(u32),
        Tar(u32),
        Drw(u32),
    }

    /// SWD mock that records every AP register write in order.
    struct RecordingMock {
        cfg: Config,
        writes: Vec<ApWrite>,
    }

    impl RecordingMock {
        fn new() -> Self {
            Self {
                cfg: Config::default(),
                writes: Vec::new(),
            }
        }
    }

    impl Swd<RecordingMock> for RecordingMock {
        fn available(_deps: &RecordingMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, _apndp: APnDP, _a: DPRegister) -> SwdResult<u32> {
            Ok(0)
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::AP, DPRegister::DPIDR) => self.writes.push(ApWrite::Csw(data)),
                (APnDP::AP, DPRegister::CTRLSTAT) => self.writes.push(ApWrite::Tar(data)),
                (APnDP::AP, DPRegister::RDBUFF) => self.writes.push(ApWrite::Drw(data)),
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// A write spanning a 1KiB boundary re-arms TAR right at the crossing, not just at the start.
    #[test]
    fn write_memory_rearms_tar_at_1kib_boundary() {
        const BASE: u32 = 0x2000_0000;
        const BOUNDARY: u32 = BASE + 0x400;
        let start = BOUNDARY - 8;
        let data: [u8; 12] = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
        ];

        let mut host = SwdHost::new(RecordingMock::new(), NoDelay);
        host.write_memory(start, &data).expect("write_memory");
        let m = host.into_parts().0;

        assert_eq!(
            m.writes,
            std::vec![
                ApWrite::Csw(CSW_WORD_ADDRINC),
                ApWrite::Tar(start),
                ApWrite::Drw(0x0403_0201),
                ApWrite::Drw(0x0807_0605),
                // TAR explicitly re-armed at the boundary rather than trusting auto-increment.
                ApWrite::Tar(BOUNDARY),
                ApWrite::Drw(0x0C0B_0A09),
            ]
        );
    }
}
