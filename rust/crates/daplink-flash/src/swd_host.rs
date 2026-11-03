//! Debug host: the transaction core plus a delay source, the flash syscalls, and the reset paths.

use dap_rs::swd::Swd;
use embedded_hal::delay::DelayNs;

use crate::error::Error;
use crate::flash_algo::SyscallParams;
use crate::reset_sequence::{
    ResetOps, ResetSequence, StepPoll, GENERIC_RESET_SEQUENCE, HALT_POLL_BUDGET_US,
    HALT_POLL_MAX_US, HALT_POLL_START_US,
};
use crate::swd_host_core::{SwdHostCore, C_DEBUGEN, DBGKEY, DHCSR, S_HALT, S_LOCKUP};
use crate::target::{DebugPort, ResetType, TargetState};
use crate::ResetControl;

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
const SYSCALL_POLL_MAX_US: u32 = 2_000;
/// Total budget for one syscall to return before `AlgoTimeout`, in microseconds (~30 s).
const SYSCALL_TIMEOUT_US: u64 = 30_000_000;

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
    /// Whether the resumable halt poll still has its one tolerated reconnect in hand.
    halt_probe_reinit: bool,
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
            halt_probe_reinit: true,
        }
    }

    /// The DPIDR read by the most recent [`init_debug`](Self::init_debug), if connected.
    pub fn dpidr(&self) -> Option<u32> {
        self.core.dpidr()
    }

    /// Tighten the transfer-retry and power-up poll budgets for transient target detection.
    /// They apply to every later transfer until set again.
    pub fn set_detect_budget(&mut self, wait_retries: usize, powerup_polls: u32) {
        self.core.set_detect_budget(wait_retries, powerup_polls);
    }

    /// Clear the debug port's sticky error flags via ABORT.
    pub fn clear_sticky_errors(&mut self) -> Result<(), Error> {
        self.core.clear_sticky_errors()
    }

    /// Attempt AP abort and sticky-error clearing without re-waking; recovery errors are ignored.
    pub fn detect_abort(&mut self) {
        self.core.detect_abort()
    }

    /// Read a detection word, retrying after best-effort recovery; failure does not prove recovery.
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

    /// Read aligned words; `sink` returns false to stop. Recovery may replay earlier addresses.
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

    /// Wake the DP, read DPIDR, power up the debug domains and clear sticky errors.
    pub fn init_debug(&mut self) -> Result<(), Error> {
        self.core.init_debug()?;
        Ok(())
    }

    /// Halt the core and wait; [`Error::Timeout`] past the poll budget.
    pub fn halt(&mut self) -> Result<(), Error> {
        self.wait_for_halt(HaltSource::Requested)
    }

    /// Poll halt with a delay budget; transfer/recovery time is additional.
    fn wait_for_halt(&mut self, source: HaltSource) -> Result<(), Error> {
        let mut requested = source == HaltSource::VectorCatch;
        let mut may_reinit = true;
        let mut backoff_us = HALT_POLL_START_US;
        let mut elapsed_us: u64 = 0;
        let mut last_err = None;
        // `first` is kept alongside `last` because S_RESET_ST/S_RETIRE_ST are clear-on-read.
        #[cfg(feature = "defmt")]
        let (mut answered, mut first, mut last, mut reconnected) = (0u32, None, None, false);
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
                    #[cfg(feature = "defmt")]
                    {
                        answered += 1;
                        first = first.or(Some(dhcsr));
                        last = Some(dhcsr);
                    }
                    if dhcsr & S_HALT != 0 {
                        #[cfg(feature = "defmt")]
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
                    #[cfg(feature = "defmt")]
                    if may_reinit {
                        reconnected = true;
                        defmt::warn!(
                            "wait_for_halt: transfer failed ({}) after {=u32} answered polls and {=u64} us; reconnecting once",
                            e,
                            answered,
                            elapsed_us
                        );
                    }
                    last_err = Some(e);
                    self.help_a_refusing_port(&mut may_reinit);
                }
            }
            backoff_us = backoff_us.saturating_mul(2).min(HALT_POLL_MAX_US);
        }
        #[cfg(feature = "defmt")]
        crate::debug_status::log_poll_halt(
            if last_err.is_some() {
                "budget exhausted, last transfer failed"
            } else {
                "budget exhausted, core never halted"
            },
            answered,
            elapsed_us,
            first,
            last,
            reconnected,
        );
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

    /// Set up the algorithm calling convention and resume the core; the caller drives the poll.
    pub fn flash_syscall_start(
        &mut self,
        params: &SyscallParams,
        entry: u32,
        args: [u32; 4],
    ) -> Result<SyscallRun, Error> {
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
        self.resume_syscall()?;
        Ok(SyscallRun::new(entry, params.breakpoint & !1))
    }

    /// Invoke a flash-algorithm routine and return r0, blocking on a virtual clock.
    pub fn flash_syscall(
        &mut self,
        params: &SyscallParams,
        entry: u32,
        args: [u32; 4],
    ) -> Result<u32, Error> {
        let mut run = self.flash_syscall_start(params, entry, args)?;
        let mut now_us = 0u64;
        loop {
            match run.advance(self, now_us) {
                StepPoll::Done => break,
                StepPoll::Err(e) => return Err(e),
                StepPoll::Pending { wake_at_us } => {
                    if wake_at_us > now_us {
                        let wait = (wake_at_us - now_us).min(u64::from(u32::MAX)) as u32;
                        self.delay.delay_us(wait);
                        now_us = wake_at_us;
                    }
                }
            }
        }
        run.finish(self)
    }
}

/// One DHCSR probe plus the r0 read a [`SyscallRun`] needs, object-safe for its poll loop.
pub trait SyscallOps {
    /// One bounded DHCSR probe: `Ok(true)` once the core reports halted (the algorithm returned).
    fn syscall_halted(&mut self) -> Result<bool, Error>;
    /// Halt a core a syscall gave up on, best-effort, so the teardown finds it in Debug state.
    fn syscall_abandon(&mut self);
    /// Read r15, where the core halted; a return to the trap reads as the trap's address.
    fn syscall_pc(&mut self) -> Result<u32, Error>;
    /// Read r0, the algorithm's status word, after the core has halted.
    fn syscall_status(&mut self) -> Result<u32, Error>;
}

impl<S, D, DEPS> SyscallOps for SwdHost<S, D, DEPS>
where
    S: Swd<DEPS>,
{
    fn syscall_halted(&mut self) -> Result<bool, Error> {
        let dhcsr = self.core.read_word(DHCSR)?;
        if dhcsr & S_HALT == 0 && dhcsr & S_LOCKUP != 0 {
            return Err(Error::AlgoLockup);
        }
        Ok(dhcsr & S_HALT != 0)
    }

    fn syscall_abandon(&mut self) {
        let _ = self.abandon_syscall();
    }

    fn syscall_pc(&mut self) -> Result<u32, Error> {
        self.core.read_core_reg(15)
    }

    fn syscall_status(&mut self) -> Result<u32, Error> {
        self.syscall_r0()
    }
}

/// How a completed syscall's r0 status word maps onto a result.
#[derive(Clone, Copy)]
pub enum SyscallCheck {
    /// Success on status `0`, else the given error.
    Zero(Error),
    /// Verify: success on status `0`, else [`Error::VerifyFailed`] naming `addr`.
    Verify {
        /// Address reported on a mismatch.
        addr: u32,
    },
    /// Address-returning verify: success on status `end`, else [`Error::VerifyFailed`] at status.
    VerifyAddr {
        /// The address the routine returns on success (`addr + len`).
        end: u32,
    },
}

impl SyscallCheck {
    /// Map a raw status word onto a result.
    pub fn apply(self, status: u32) -> Result<(), Error> {
        match self {
            SyscallCheck::Zero(e) => (status == 0).then_some(()).ok_or(e),
            SyscallCheck::Verify { addr } => (status == 0)
                .then_some(())
                .ok_or(Error::VerifyFailed { addr }),
            SyscallCheck::VerifyAddr { end } => (status == end)
                .then_some(())
                .ok_or(Error::VerifyFailed { addr: status }),
        }
    }
}

/// A resumable syscall completion poll, one DHCSR probe per [`advance`](Self::advance).
pub struct SyscallRun {
    /// Algorithm entry, for diagnostics only.
    #[cfg_attr(not(feature = "log"), allow(dead_code))]
    entry: u32,
    /// The return trap's address, the only pc a completed routine halts at.
    trap_pc: u32,
    /// Whether the first advance has set the deadline off the caller's clock.
    started: bool,
    /// Absolute timestamp the syscall budget expires at.
    deadline_us: u64,
    /// Absolute timestamp of the next DHCSR probe.
    next_poll_us: u64,
    /// Per-attempt backoff, doubling to [`SYSCALL_POLL_MAX_US`].
    backoff_us: u32,
    /// Set once the core has reported halted.
    done: bool,
    /// Set once the run has failed; every later advance repeats it and touches nothing.
    failed: Option<Error>,
    /// Status word an inert run reports without a device read (device work done inline).
    inert_status: Option<u32>,
    /// How [`finish_checked`](Self::finish_checked) maps the status word.
    check: SyscallCheck,
}

impl SyscallRun {
    /// A run polling for the completion of the syscall at `entry`.
    fn new(entry: u32, trap_pc: u32) -> Self {
        Self {
            entry,
            trap_pc,
            started: false,
            deadline_us: 0,
            next_poll_us: 0,
            backoff_us: SYSCALL_POLL_START_US,
            done: false,
            failed: None,
            inert_status: None,
            check: SyscallCheck::Zero(Error::AlgoFailed { code: 0 }),
        }
    }

    /// A run already complete with `status`, for callers whose device work finished inline.
    pub fn done(status: u32) -> Self {
        Self {
            entry: 0,
            trap_pc: 0,
            started: true,
            deadline_us: 0,
            next_poll_us: 0,
            backoff_us: SYSCALL_POLL_START_US,
            done: true,
            failed: None,
            inert_status: Some(status),
            check: SyscallCheck::Zero(Error::AlgoFailed { code: 0 }),
        }
    }

    /// Attach the status-word interpretation used by [`finish_checked`](Self::finish_checked).
    pub fn with_check(mut self, check: SyscallCheck) -> Self {
        self.check = check;
        self
    }

    /// Perform at most one bounded DHCSR probe, at wall-clock `now_us`.
    pub fn advance(&mut self, ops: &mut dyn SyscallOps, now_us: u64) -> StepPoll {
        if let Some(e) = self.failed {
            return StepPoll::Err(e);
        }
        if self.done {
            return StepPoll::Done;
        }
        if !self.started {
            // The first probe keeps the delay the blocking poll spent before its first read.
            self.started = true;
            self.deadline_us = now_us + SYSCALL_TIMEOUT_US;
            self.next_poll_us = now_us + u64::from(SYSCALL_POLL_START_US);
            self.backoff_us = SYSCALL_POLL_START_US;
            return StepPoll::Pending {
                wake_at_us: self.next_poll_us,
            };
        }
        if now_us < self.next_poll_us {
            return StepPoll::Pending {
                wake_at_us: self.next_poll_us,
            };
        }
        // A refused transfer is propagated, not retried: the blocking poll used `?` here too.
        match ops.syscall_halted() {
            Ok(true) => {
                self.done = true;
                return StepPoll::Done;
            }
            Ok(false) => {}
            Err(e) => {
                ops.syscall_abandon();
                self.failed = Some(e);
                return StepPoll::Err(e);
            }
        }
        if now_us >= self.deadline_us {
            #[cfg(feature = "log")]
            log::warn!("flash_syscall: AlgoTimeout at entry={:#010x}", self.entry);
            ops.syscall_abandon();
            self.failed = Some(Error::AlgoTimeout);
            return StepPoll::Err(Error::AlgoTimeout);
        }
        self.backoff_us = self.backoff_us.saturating_mul(2).min(SYSCALL_POLL_MAX_US);
        self.next_poll_us = now_us + u64::from(self.backoff_us);
        StepPoll::Pending {
            wake_at_us: self.next_poll_us,
        }
    }

    /// Report the terminal state of a [`done`](Self::done) run without a probe.
    pub fn advance_inert(&mut self, _now_us: u64) -> StepPoll {
        match self.failed {
            Some(e) => StepPoll::Err(e),
            None => StepPoll::Done,
        }
    }

    /// Read the algorithm's raw r0 status; call it only once the poll has reported
    /// [`StepPoll::Done`], which this does not check.
    pub fn finish(&mut self, ops: &mut dyn SyscallOps) -> Result<u32, Error> {
        if let Some(status) = self.inert_status {
            return Ok(status);
        }
        let pc = ops.syscall_pc()?;
        // Read even after a foreign halt: on a masked row this also clears `C_MASKINTS`.
        let ret = ops.syscall_status()?;
        if pc != self.trap_pc {
            return Err(Error::AlgoHalted { pc });
        }
        #[cfg(feature = "log")]
        log::debug!(
            "flash_syscall: entry={:#010x} -> r0={:#010x}",
            self.entry,
            ret
        );
        Ok(ret)
    }

    /// Read r0 and map it onto a result through the attached [`SyscallCheck`].
    pub fn finish_checked(&mut self, ops: &mut dyn SyscallOps) -> Result<(), Error> {
        let status = self.finish(ops)?;
        self.check.apply(status)
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

    /// Request a software reset via AIRCR, aborting any latched fault so the halt poll keeps its budget.
    fn request_software_reset(&mut self, value: u32) {
        if self.core.write_word(AIRCR, value).is_err() {
            self.core.abort_transaction();
        }
    }

    /// Drive the target into `state` using [`GENERIC_RESET_SEQUENCE`].
    pub fn set_target_state(
        &mut self,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        GENERIC_RESET_SEQUENCE.set_target_state(self, state, reset_type)
    }

    /// Drive the target into `state` using `sequence`, resetting with `reset_type`.
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

    fn poll_halt_once(&mut self, restart: bool) -> Result<bool, Error> {
        if restart {
            self.halt_probe_reinit = true;
        }
        match self.core.read_dhcsr() {
            Ok(dhcsr) => Ok(dhcsr & S_HALT != 0),
            Err(e) => {
                let mut may_reinit = self.halt_probe_reinit;
                self.help_a_refusing_port(&mut may_reinit);
                self.halt_probe_reinit = may_reinit;
                Err(e)
            }
        }
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
#[path = "swd_host_tests.rs"]
mod tests;

/// Whether flash algorithms run with `DHCSR.C_MASKINTS` set (the `mask-interrupts` feature).
pub const MASK_INTERRUPTS: bool = cfg!(feature = "mask-interrupts");

/// `DHCSR.C_MASKINTS`: PendSV, SysTick and external interrupts masked while the core runs.
const C_MASKINTS: u32 = 1 << 3;

impl<S, D, DEPS> SwdHost<S, D, DEPS>
where
    S: Swd<DEPS>,
{
    /// Resume into the algorithm; `C_MASKINTS` may change only in a write that keeps `C_HALT`.
    #[inline(always)]
    fn resume_syscall(&mut self) -> Result<(), Error> {
        if !MASK_INTERRUPTS {
            return self.core.write_word(DHCSR, DBGKEY | C_DEBUGEN);
        }
        let halted = DBGKEY | C_DEBUGEN | crate::swd_host_core::C_HALT;
        self.core.write_word(DHCSR, halted | C_MASKINTS)?;
        self.core.write_word(DHCSR, DBGKEY | C_DEBUGEN | C_MASKINTS)
    }

    /// Halt a core still running a call; `C_MASKINTS` may change only once `C_HALT` reads set.
    #[inline(always)]
    fn abandon_syscall(&mut self) -> Result<(), Error> {
        let halted = DBGKEY | C_DEBUGEN | crate::swd_host_core::C_HALT;
        if !MASK_INTERRUPTS {
            return self.core.write_word(DHCSR, halted);
        }
        self.core.write_word(DHCSR, halted | C_MASKINTS)?;
        self.core.write_word(DHCSR, halted)
    }

    /// Read the halted algorithm's r0, then clear `C_MASKINTS` while the core stays halted.
    #[inline(always)]
    fn syscall_r0(&mut self) -> Result<u32, Error> {
        if !MASK_INTERRUPTS {
            return self.core.read_core_reg(0);
        }
        let r0 = self.core.read_core_reg(0)?;
        let halted = DBGKEY | C_DEBUGEN | crate::swd_host_core::C_HALT;
        self.core.write_word(DHCSR, halted)?;
        Ok(r0)
    }
}
