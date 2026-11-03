//! The per-target reset/halt sequence seam.

use crate::error::Error;
use crate::swd_host_core::{C_DEBUGEN, DBGKEY, DEMCR, DHCSR, VC_CORERESET, VC_HARDERR};
use crate::target::{ResetType, TargetState};

/// Object-safe debug primitives a [`ResetSequence`] drives the target through.
pub trait ResetOps {
    /// Write a single 32-bit word to the target's memory map.
    fn write_word(&mut self, addr: u32, value: u32) -> Result<(), Error>;
    /// Read a single 32-bit word from the target's memory map.
    fn read_word(&mut self, addr: u32) -> Result<u32, Error>;
    /// Trigger a reset of the given type (nRESET pulse or an AIRCR software reset).
    fn trigger_reset(&mut self, reset_type: ResetType) -> Result<(), Error>;
    /// Poll DHCSR until halted, re-establishing debug if a transfer faults while leaving reset.
    fn poll_halt(&mut self) -> Result<(), Error>;
    /// Probe halt state; `restart` begins a new poll. The default blocks in `poll_halt` on restart.
    fn poll_halt_once(&mut self, restart: bool) -> Result<bool, Error> {
        if restart {
            self.poll_halt()?;
        }
        Ok(true)
    }

    /// Assert (`true` = hold in reset) or release the physical nRESET line.
    fn set_reset(&mut self, asserted: bool) -> Result<(), Error>;
    /// Halt the core via DHCSR and wait for the halt to take effect.
    fn halt(&mut self) -> Result<(), Error>;

    /// Read a raw access-port register at `apsel`/`addr`; defaults to `Err(Unsupported)`.
    fn ap_read_raw(&mut self, _apsel: u8, _addr: u8) -> Result<u32, Error> {
        Err(Error::Unsupported)
    }

    /// Write a raw access-port register. Defaults to `Err(Unsupported)`.
    fn ap_write_raw(&mut self, _apsel: u8, _addr: u8, _value: u32) -> Result<(), Error> {
        Err(Error::Unsupported)
    }

    /// Busy-wait for at least `us` microseconds. Defaults to a no-op.
    fn delay_us(&mut self, _us: u32) -> Result<(), Error> {
        Ok(())
    }

    /// Read the debug port's own CTRL/STAT register, for post-mortem diagnostics. Defmt only.
    #[cfg(feature = "defmt")]
    fn dp_ctrl_stat(&mut self) -> Result<u32, Error> {
        Err(Error::Unsupported)
    }

    /// Re-establish debug after a sequence-triggered chip reset; defaults to `Err(Unsupported)`.
    fn reconnect(&mut self) -> Result<(), Error> {
        Err(Error::Unsupported)
    }
}

/// Result of a family [`unlock`](ResetSequence::unlock) step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UnlockOutcome {
    /// Debug access was already open (or the family has no lock concept); nothing was done.
    NotLocked,
    /// A destructive unlock ran: the part was mass-erased, so flash is now blank.
    UnlockedByErase,
}

/// Caller-supplied consent for destructive access-port unlocks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UnlockPolicy {
    /// Never perform a destructive unlock; a locked part fails with [`Error::TargetLocked`].
    #[default]
    Refuse,
    /// Permit a full-chip mass erase to drop access-port protection on a locked part.
    EraseAll,
}

/// A target's reset/halt strategy into a [`TargetState`].
pub trait ResetSequence {
    /// Drive the target into `state`; defaults to [`generic_set_target_state`].
    fn set_target_state(
        &self,
        ops: &mut dyn ResetOps,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        generic_set_target_state(ops, state, reset_type)
    }

    /// A resumable run for `state`, or `None` (the default) for the blocking path only.
    fn resumable(&self, _state: TargetState, _reset_type: ResetType) -> Option<ResetRun> {
        None
    }

    /// The family's access-port unlock step, once per session after DP power-up; no-op by default.
    fn unlock(
        &self,
        _ops: &mut dyn ResetOps,
        _policy: UnlockPolicy,
    ) -> Result<UnlockOutcome, Error> {
        Ok(UnlockOutcome::NotLocked)
    }
}

/// Report DP CTRL/STAT after a reset-and-halt primitive failed; no core access, so no state moves.
#[cfg(feature = "defmt")]
fn report_debug_state(ops: &mut dyn ResetOps) {
    match ops.dp_ctrl_stat() {
        Ok(value) => crate::debug_status::log_ctrl_stat(value),
        Err(e) => defmt::warn!("debug state: DP CTRL/STAT read failed: {}", e),
    }
}

/// Run one primitive of the reset-and-halt, discarding the step's name.
#[cfg(not(feature = "defmt"))]
macro_rules! reset_try {
    ($ops:ident, $label:literal, $op:expr) => {
        $op
    };
}

/// Run one reset-and-halt primitive, naming it and reporting debug state on failure.
#[cfg(feature = "defmt")]
macro_rules! reset_try {
    ($ops:ident, $label:literal, $op:expr) => {{
        let outcome = $op;
        if let Err(e) = &outcome {
            defmt::warn!("reset-and-halt: {=str} failed: {}", $label, e);
            report_debug_state($ops);
        }
        outcome
    }};
}

/// Settling time between the reset request and the first halt poll, in microseconds.
pub(crate) const RESET_SETTLE_US: u32 = 2_000;

/// Halt-poll backoff: delay before the second DHCSR probe, in microseconds.
pub(crate) const HALT_POLL_START_US: u32 = 100;
/// Cap on the halt poll's per-attempt backoff delay, in microseconds.
pub(crate) const HALT_POLL_MAX_US: u32 = 10_000;
/// Total budget for the core to report halted, in microseconds (1 s).
pub(crate) const HALT_POLL_BUDGET_US: u64 = 1_000_000;

/// What one [`ResetRun::advance`] achieved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StepPoll {
    /// More work remains; advance again at `wake_at_us` (immediately if already past).
    Pending {
        /// Absolute microsecond timestamp the caller should advance the run at.
        wake_at_us: u64,
    },
    /// The sequence completed.
    Done,
    /// The sequence failed; the run stays failed and performs no further operations.
    Err(Error),
}

/// One step of the generic reset-and-halt sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetStep {
    /// Halt the core, before anything is armed. Also the entry step of the wait-free states.
    Halt,
    /// Arm the reset vector catch and keep debug enabled across the reset.
    ArmCatch,
    /// Request the reset itself.
    TriggerReset,
    /// Let the part settle after the reset request.
    Settle {
        /// Absolute timestamp the settle ends at.
        deadline_us: u64,
    },
    /// Probe DHCSR until the core reports halted, or the budget runs out.
    PollHalt {
        /// Absolute timestamp the halt budget expires at.
        deadline_us: u64,
        /// Absolute timestamp of the next probe.
        next_poll_us: u64,
    },
    /// Disarm the vector catch, leaving trace enabled.
    Disarm,
    /// Nothing left to do.
    Done,
}

/// Resumable reset states with explicit poll deadlines; backend operations may still block.
pub struct ResetRun {
    step: ResetStep,
    state: TargetState,
    reset_type: ResetType,
    /// Halt-poll backoff in force, doubling to [`HALT_POLL_MAX_US`].
    backoff_us: u32,
    /// The last transfer error a halt probe saw, reported if the budget expires.
    last_err: Option<Error>,
    /// Set once the run has failed; every later advance repeats it and touches nothing.
    failed: Option<Error>,
    /// Whether the next halt probe is the first of this poll.
    restart_poll: bool,
}

impl ResetRun {
    /// Start a run driving the target into `state`, dispatching any reset on `reset_type`.
    pub fn new(state: TargetState, reset_type: ResetType) -> Self {
        Self {
            step: ResetStep::Halt,
            state,
            reset_type,
            backoff_us: HALT_POLL_START_US,
            last_err: None,
            failed: None,
            restart_poll: true,
        }
    }

    /// The step the run is about to perform.
    pub fn step(&self) -> ResetStep {
        self.step
    }

    /// Latch `err` and report it.
    fn fail(&mut self, err: Error) -> StepPoll {
        self.failed = Some(err);
        self.step = ResetStep::Done;
        StepPoll::Err(err)
    }

    /// Advance one state at wall-clock `now_us`; backend calls can block or perform several writes.
    pub fn advance(&mut self, ops: &mut dyn ResetOps, now_us: u64) -> StepPoll {
        if let Some(err) = self.failed {
            return StepPoll::Err(err);
        }
        match self.step {
            ResetStep::Halt => self.advance_entry(ops, now_us),
            ResetStep::ArmCatch => {
                if let Err(e) = self.arm_catch(ops) {
                    return self.fail(e);
                }
                self.step = ResetStep::TriggerReset;
                StepPoll::Pending { wake_at_us: now_us }
            }
            ResetStep::TriggerReset => {
                if let Err(e) = reset_try!(ops, "trigger reset", ops.trigger_reset(self.reset_type))
                {
                    return self.fail(e);
                }
                self.step = ResetStep::Settle {
                    deadline_us: now_us + u64::from(RESET_SETTLE_US),
                };
                StepPoll::Pending {
                    wake_at_us: now_us + u64::from(RESET_SETTLE_US),
                }
            }
            ResetStep::Settle { deadline_us } => {
                if now_us < deadline_us {
                    return StepPoll::Pending {
                        wake_at_us: deadline_us,
                    };
                }
                // The first probe keeps the blocking poll's starting backoff.
                let next_poll_us = now_us + u64::from(HALT_POLL_START_US);
                self.step = ResetStep::PollHalt {
                    deadline_us: now_us + HALT_POLL_BUDGET_US,
                    next_poll_us,
                };
                self.backoff_us = HALT_POLL_START_US;
                self.restart_poll = true;
                StepPoll::Pending {
                    wake_at_us: next_poll_us,
                }
            }
            ResetStep::PollHalt {
                deadline_us,
                next_poll_us,
            } => self.advance_poll(ops, now_us, deadline_us, next_poll_us),
            ResetStep::Disarm => {
                if let Err(e) = reset_try!(
                    ops,
                    "disarm vector catch (DEMCR)",
                    ops.write_word(DEMCR, VC_HARDERR)
                ) {
                    return self.fail(e);
                }
                self.step = ResetStep::Done;
                StepPoll::Done
            }
            ResetStep::Done => StepPoll::Done,
        }
    }

    /// The halt for `ResetProgram`, or the whole wait-free body of every other state.
    fn advance_entry(&mut self, ops: &mut dyn ResetOps, now_us: u64) -> StepPoll {
        match self.state {
            TargetState::ResetProgram => {
                if let Err(e) = reset_try!(ops, "halt before arming the catch", ops.halt()) {
                    return self.fail(e);
                }
                self.step = ResetStep::ArmCatch;
                StepPoll::Pending { wake_at_us: now_us }
            }
            TargetState::ResetRun | TargetState::PostFlashReset => {
                // DEMCR survives a system reset: clear it, release debug, then reset into the image.
                let r = ops
                    .write_word(DEMCR, 0)
                    .and_then(|()| ops.write_word(DHCSR, DBGKEY))
                    .and_then(|()| ops.trigger_reset(self.reset_type));
                self.finish(r)
            }
            TargetState::ResetHold => {
                let r = match self.reset_type {
                    ResetType::Hardware => ops.set_reset(true),
                    // A software reset cannot latch a hold; halt as the closest approximation.
                    ResetType::SoftwareSysReset | ResetType::SoftwareVectReset => ops.halt(),
                };
                self.finish(r)
            }
            TargetState::Halt => {
                let r = ops.halt();
                self.finish(r)
            }
            TargetState::Run | TargetState::NoDebug => {
                // Release the core to run with debug disabled.
                let r = ops.write_word(DHCSR, DBGKEY);
                self.finish(r)
            }
            TargetState::Debug => {
                // Enable debug without halting.
                let r = ops.write_word(DHCSR, DBGKEY | C_DEBUGEN);
                self.finish(r)
            }
        }
    }

    /// Arm the vector catch and keep debug enabled across the reset.
    fn arm_catch(&mut self, ops: &mut dyn ResetOps) -> Result<(), Error> {
        reset_try!(
            ops,
            "arm vector catch (DEMCR)",
            ops.write_word(DEMCR, VC_CORERESET | VC_HARDERR)
        )?;
        reset_try!(
            ops,
            "enable debug (DHCSR)",
            ops.write_word(DHCSR, DBGKEY | C_DEBUGEN)
        )
    }

    /// One halt probe, or the wait before the next one.
    fn advance_poll(
        &mut self,
        ops: &mut dyn ResetOps,
        now_us: u64,
        deadline_us: u64,
        next_poll_us: u64,
    ) -> StepPoll {
        if now_us < next_poll_us {
            return StepPoll::Pending {
                wake_at_us: next_poll_us,
            };
        }
        let restart = core::mem::replace(&mut self.restart_poll, false);
        match ops.poll_halt_once(restart) {
            Ok(true) => {
                self.step = ResetStep::Disarm;
                return StepPoll::Pending { wake_at_us: now_us };
            }
            Ok(false) => self.last_err = None,
            Err(e) => self.last_err = Some(e),
        }
        if now_us >= deadline_us {
            let err = self.last_err.unwrap_or(Error::Timeout);
            #[cfg(feature = "defmt")]
            {
                defmt::warn!("reset-and-halt: halt poll failed: {}", err);
                report_debug_state(ops);
            }
            return self.fail(err);
        }
        self.backoff_us = self.backoff_us.saturating_mul(2).min(HALT_POLL_MAX_US);
        let wake_at_us = now_us + u64::from(self.backoff_us);
        self.step = ResetStep::PollHalt {
            deadline_us,
            next_poll_us: wake_at_us,
        };
        StepPoll::Pending { wake_at_us }
    }

    /// Terminate a wait-free state on `r`.
    fn finish(&mut self, r: Result<(), Error>) -> StepPoll {
        match r {
            Ok(()) => {
                self.step = ResetStep::Done;
                StepPoll::Done
            }
            Err(e) => self.fail(e),
        }
    }
}

/// The generic DHCSR/DEMCR/AIRCR reset sequence, blocking in [`ResetOps::delay_us`].
pub fn generic_set_target_state(
    ops: &mut dyn ResetOps,
    state: TargetState,
    reset_type: ResetType,
) -> Result<(), Error> {
    let mut run = ResetRun::new(state, reset_type);
    // A virtual clock: deadlines are compared only against time already spent.
    let mut now_us = 0u64;
    loop {
        match run.advance(ops, now_us) {
            StepPoll::Done => return Ok(()),
            StepPoll::Err(e) => return Err(e),
            StepPoll::Pending { wake_at_us } => {
                if wake_at_us > now_us {
                    let wait = (wake_at_us - now_us).min(u64::from(u32::MAX)) as u32;
                    ops.delay_us(wait)?;
                    now_us = wake_at_us;
                }
            }
        }
    }
}

/// The generic (default) reset sequence, equivalent to [`generic_set_target_state`].
#[derive(Clone, Copy, Debug, Default)]
pub struct GenericResetSequence;

impl ResetSequence for GenericResetSequence {
    fn resumable(&self, state: TargetState, reset_type: ResetType) -> Option<ResetRun> {
        Some(ResetRun::new(state, reset_type))
    }
}

/// A shared instance of the generic sequence, for handing out a `&'static dyn ResetSequence`.
pub static GENERIC_RESET_SEQUENCE: GenericResetSequence = GenericResetSequence;

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    /// Recording [`ResetOps`] mock: logs the ordered stream of operations.
    #[derive(Default)]
    struct RecOps {
        ops: Vec<Op>,
        /// When set, `trigger_reset` records its op then fails.
        fail_trigger: bool,
        /// Halt probes that report the core still running before one reports it halted.
        polls_before_halt: u32,
        /// Index of the operation to fail, or `None` to fail none.
        fail_at: Option<usize>,
    }

    impl RecOps {
        /// Record `op` and report whether it is the one scripted to fail.
        fn record(&mut self, op: Op) -> Result<(), Error> {
            let index = self.ops.len();
            self.ops.push(op);
            if self.fail_at == Some(index) {
                return Err(Error::Timeout);
            }
            Ok(())
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum Op {
        Write(u32, u32),
        Read(u32),
        Reset(ResetType),
        PollHalt,
        SetReset(bool),
        Halt,
        /// A settling wait, in microseconds.
        Delay(u32),
    }

    impl ResetOps for RecOps {
        fn write_word(&mut self, addr: u32, value: u32) -> Result<(), Error> {
            self.record(Op::Write(addr, value))
        }

        fn poll_halt_once(&mut self, _restart: bool) -> Result<bool, Error> {
            self.record(Op::PollHalt)?;
            if self.polls_before_halt > 0 {
                self.polls_before_halt -= 1;
                return Ok(false);
            }
            Ok(true)
        }
        fn read_word(&mut self, addr: u32) -> Result<u32, Error> {
            self.ops.push(Op::Read(addr));
            Ok(0)
        }
        fn trigger_reset(&mut self, reset_type: ResetType) -> Result<(), Error> {
            self.record(Op::Reset(reset_type))?;
            if self.fail_trigger {
                return Err(Error::ResetFailed);
            }
            Ok(())
        }
        fn poll_halt(&mut self) -> Result<(), Error> {
            self.record(Op::PollHalt)
        }
        fn set_reset(&mut self, asserted: bool) -> Result<(), Error> {
            self.record(Op::SetReset(asserted))
        }
        fn halt(&mut self) -> Result<(), Error> {
            self.record(Op::Halt)
        }
        fn delay_us(&mut self, us: u32) -> Result<(), Error> {
            self.ops.push(Op::Delay(us));
            Ok(())
        }
    }

    /// Drive `run` on a virtual clock, returning the outcome and each requested wait.
    fn drive(run: &mut ResetRun, ops: &mut RecOps) -> (Result<(), Error>, Vec<u64>) {
        let mut now_us = 0u64;
        let mut waits = Vec::new();
        for _ in 0..10_000 {
            match run.advance(ops, now_us) {
                StepPoll::Done => return (Ok(()), waits),
                StepPoll::Err(e) => return (Err(e), waits),
                StepPoll::Pending { wake_at_us } => {
                    if wake_at_us > now_us {
                        waits.push(wake_at_us - now_us);
                        now_us = wake_at_us;
                    }
                }
            }
        }
        panic!("the run never terminated");
    }

    /// The step machine issues exactly the operations the blocking sequence does, in order.
    #[test]
    fn the_step_machine_matches_the_blocking_op_stream() {
        for state in [
            TargetState::ResetProgram,
            TargetState::ResetRun,
            TargetState::PostFlashReset,
            TargetState::ResetHold,
            TargetState::Halt,
            TargetState::Run,
            TargetState::NoDebug,
            TargetState::Debug,
        ] {
            let mut blocking = RecOps::default();
            generic_set_target_state(&mut blocking, state, ResetType::Hardware)
                .expect("blocking sequence");

            let mut stepped = RecOps::default();
            let mut run = ResetRun::new(state, ResetType::Hardware);
            let (outcome, _) = drive(&mut run, &mut stepped);
            assert_eq!(outcome, Ok(()), "{state:?} steps to completion");

            // The stepped path hands back the delays the blocking path spends; nothing else differs.
            let expected: Vec<Op> = blocking
                .ops
                .into_iter()
                .filter(|op| !matches!(op, Op::Delay(_)))
                .collect();
            assert_eq!(stepped.ops, expected, "{state:?} op stream");
            assert_eq!(run.step(), ResetStep::Done);
        }
    }

    /// The settle and doubling halt-poll backoff come back as deadlines.
    #[test]
    fn the_waits_come_back_as_deadlines() {
        let mut ops = RecOps {
            polls_before_halt: 9,
            ..RecOps::default()
        };
        let mut run = ResetRun::new(TargetState::ResetProgram, ResetType::SoftwareSysReset);
        let (outcome, waits) = drive(&mut run, &mut ops);
        assert_eq!(outcome, Ok(()));
        assert_eq!(
            waits[0],
            u64::from(RESET_SETTLE_US),
            "the settle is the first deadline"
        );
        assert_eq!(
            waits[1],
            u64::from(HALT_POLL_START_US),
            "the first probe keeps the blocking path's starting backoff"
        );
        for pair in waits[1..].windows(2) {
            let (prev, next) = (pair[0], pair[1]);
            assert!(
                next == u64::from(HALT_POLL_MAX_US) || next == prev * 2,
                "each backoff doubles until the cap: {prev} -> {next}"
            );
        }
        assert_eq!(
            ops.ops.iter().filter(|op| **op == Op::PollHalt).count(),
            10,
            "one bounded probe per advance, and no more"
        );
    }

    /// A halt that never arrives fails on the budget, not on the first unhelpful probe.
    #[test]
    fn a_core_that_never_halts_fails_on_the_budget() {
        let mut ops = RecOps {
            polls_before_halt: u32::MAX,
            ..RecOps::default()
        };
        let mut run = ResetRun::new(TargetState::ResetProgram, ResetType::Hardware);
        let (outcome, waits) = drive(&mut run, &mut ops);
        assert_eq!(outcome, Err(Error::Timeout));
        let polled: u64 = waits[1..].iter().sum();
        assert!(
            polled >= HALT_POLL_BUDGET_US,
            "the poll spent its whole budget: {polled}"
        );
    }

    /// A failure at any step surfaces once, and the run stays failed without touching the target.
    #[test]
    fn a_failure_at_any_step_surfaces_once_and_sticks() {
        let clean = {
            let mut ops = RecOps::default();
            let mut run = ResetRun::new(TargetState::ResetProgram, ResetType::Hardware);
            drive(&mut run, &mut ops).0.expect("clean run");
            ops.ops
        };
        for (fail_at, failing) in clean.iter().enumerate() {
            let mut ops = RecOps {
                fail_at: Some(fail_at),
                ..RecOps::default()
            };
            let mut run = ResetRun::new(TargetState::ResetProgram, ResetType::Hardware);
            let (outcome, _) = drive(&mut run, &mut ops);
            if *failing == Op::PollHalt {
                // A failed probe is retried within the budget, as on the blocking path.
                assert_eq!(outcome, Ok(()), "a failed halt probe is retried");
                continue;
            }
            assert_eq!(outcome, Err(Error::Timeout), "step {fail_at} propagates");
            assert_eq!(
                ops.ops.len(),
                fail_at + 1,
                "step {fail_at} is the last operation attempted"
            );
            let before = ops.ops.len();
            assert_eq!(run.advance(&mut ops, 0), StepPoll::Err(Error::Timeout));
            assert_eq!(run.advance(&mut ops, 0), StepPoll::Err(Error::Timeout));
            assert_eq!(
                ops.ops.len(),
                before,
                "a failed run touches nothing further"
            );
        }
    }

    /// The default sequence produces exactly the generic reset-and-halt op stream.
    #[test]
    fn default_reset_program_matches_generic() {
        let mut ops = RecOps::default();
        GENERIC_RESET_SEQUENCE
            .set_target_state(&mut ops, TargetState::ResetProgram, ResetType::Hardware)
            .expect("generic sequence");
        assert_eq!(
            ops.ops,
            std::vec![
                Op::Halt,
                Op::Write(DEMCR, VC_CORERESET | VC_HARDERR),
                Op::Write(DHCSR, DBGKEY | C_DEBUGEN),
                Op::Reset(ResetType::Hardware),
                Op::Delay(RESET_SETTLE_US),
                Op::Delay(HALT_POLL_START_US),
                Op::PollHalt,
                Op::Write(DEMCR, VC_HARDERR),
            ]
        );
    }

    /// The halt is confirmed before arming, and polling waits [`RESET_SETTLE_US`] after reset.
    #[test]
    fn reset_program_halts_first_and_settles_between_the_reset_and_the_poll() {
        let mut ops = RecOps::default();
        generic_set_target_state(
            &mut ops,
            TargetState::ResetProgram,
            ResetType::SoftwareSysReset,
        )
        .expect("generic sequence");

        let at = |op: Op| {
            ops.ops
                .iter()
                .position(|o| *o == op)
                .unwrap_or_else(|| panic!("{op:?} missing from the reset-and-halt stream"))
        };
        assert!(
            at(Op::Halt) < at(Op::Write(DEMCR, VC_CORERESET | VC_HARDERR)),
            "the halt is confirmed before the vector catch is armed"
        );
        assert!(
            at(Op::Reset(ResetType::SoftwareSysReset)) < at(Op::Delay(RESET_SETTLE_US)),
            "the settle follows the reset request"
        );
        assert!(
            at(Op::Delay(RESET_SETTLE_US)) < at(Op::PollHalt),
            "the settle precedes the first halt poll"
        );
    }

    /// `ResetProgram` arms the catch before the reset regardless of `reset_type`.
    #[test]
    fn reset_program_software_sysreset_arms_catch_before_reset() {
        let mut ops = RecOps::default();
        generic_set_target_state(
            &mut ops,
            TargetState::ResetProgram,
            ResetType::SoftwareSysReset,
        )
        .expect("generic sequence");
        assert_eq!(
            ops.ops,
            std::vec![
                Op::Halt,
                Op::Write(DEMCR, VC_CORERESET | VC_HARDERR),
                Op::Write(DHCSR, DBGKEY | C_DEBUGEN),
                Op::Reset(ResetType::SoftwareSysReset),
                Op::Delay(RESET_SETTLE_US),
                Op::Delay(HALT_POLL_START_US),
                Op::PollHalt,
                Op::Write(DEMCR, VC_HARDERR),
            ]
        );
    }

    /// A custom sequence's `set_target_state` runs instead of the generic default.
    #[test]
    fn custom_sequence_overrides_default() {
        /// Marker address the custom sequence reads to prove it ran.
        const CUSTOM_MARK: u32 = 0xDEAD_BEEF;

        struct CustomSequence;
        impl ResetSequence for CustomSequence {
            fn set_target_state(
                &self,
                ops: &mut dyn ResetOps,
                _state: TargetState,
                _reset_type: ResetType,
            ) -> Result<(), Error> {
                ops.read_word(CUSTOM_MARK)?;
                Ok(())
            }
        }

        let mut ops = RecOps::default();
        CustomSequence
            .set_target_state(&mut ops, TargetState::ResetProgram, ResetType::Hardware)
            .expect("custom sequence");
        assert_eq!(ops.ops, std::vec![Op::Read(CUSTOM_MARK)]);
    }

    /// `ResetRun` clears the catch, releases debug, and triggers the reset without polling.
    #[test]
    fn reset_run_releases_and_resets_without_polling() {
        let mut ops = RecOps::default();
        generic_set_target_state(&mut ops, TargetState::ResetRun, ResetType::SoftwareSysReset)
            .expect("generic sequence");
        assert_eq!(
            ops.ops,
            std::vec![
                Op::Write(DEMCR, 0),
                Op::Write(DHCSR, DBGKEY),
                Op::Reset(ResetType::SoftwareSysReset),
            ]
        );
    }

    /// `PostFlashReset` produces the identical op stream as `ResetRun`.
    #[test]
    fn post_flash_reset_matches_reset_run_stream() {
        let mut ops = RecOps::default();
        generic_set_target_state(&mut ops, TargetState::PostFlashReset, ResetType::Hardware)
            .expect("generic sequence");
        assert_eq!(
            ops.ops,
            std::vec![
                Op::Write(DEMCR, 0),
                Op::Write(DHCSR, DBGKEY),
                Op::Reset(ResetType::Hardware),
            ]
        );
    }

    /// `ResetHold` with a hardware reset latches nRESET asserted and does nothing else.
    #[test]
    fn reset_hold_hardware_asserts_reset_only() {
        let mut ops = RecOps::default();
        generic_set_target_state(&mut ops, TargetState::ResetHold, ResetType::Hardware)
            .expect("generic sequence");
        assert_eq!(ops.ops, std::vec![Op::SetReset(true)]);
    }

    /// `ResetHold` with a software reset cannot latch a hold, so it falls back to a plain halt.
    #[test]
    fn reset_hold_software_falls_back_to_halt() {
        let mut ops = RecOps::default();
        generic_set_target_state(
            &mut ops,
            TargetState::ResetHold,
            ResetType::SoftwareVectReset,
        )
        .expect("generic sequence");
        assert_eq!(ops.ops, std::vec![Op::Halt]);
    }

    /// A failing `trigger_reset` stops the sequence, leaving the catch armed.
    #[test]
    fn reset_program_trigger_failure_propagates_and_stops() {
        let mut ops = RecOps {
            fail_trigger: true,
            ..RecOps::default()
        };
        let err = generic_set_target_state(
            &mut ops,
            TargetState::ResetProgram,
            ResetType::SoftwareSysReset,
        )
        .expect_err("trigger failure must propagate");
        assert_eq!(err, Error::ResetFailed);
        assert_eq!(
            ops.ops,
            std::vec![
                Op::Halt,
                Op::Write(DEMCR, VC_CORERESET | VC_HARDERR),
                Op::Write(DHCSR, DBGKEY | C_DEBUGEN),
                Op::Reset(ResetType::SoftwareSysReset),
            ],
            "no settle, no poll_halt and no disarm write after the failed trigger (catch left armed)"
        );
    }

    /// A port implementing only the six required [`ResetOps`] methods, to exercise the defaults.
    struct BareOps;

    impl ResetOps for BareOps {
        fn write_word(&mut self, _addr: u32, _value: u32) -> Result<(), Error> {
            Ok(())
        }
        fn read_word(&mut self, _addr: u32) -> Result<u32, Error> {
            Ok(0)
        }
        fn trigger_reset(&mut self, _reset_type: ResetType) -> Result<(), Error> {
            Ok(())
        }
        fn poll_halt(&mut self) -> Result<(), Error> {
            Ok(())
        }
        fn set_reset(&mut self, _asserted: bool) -> Result<(), Error> {
            Ok(())
        }
        fn halt(&mut self) -> Result<(), Error> {
            Ok(())
        }
    }

    /// The widened `ResetOps` primitives default so a bare mock stays valid.
    #[test]
    fn widened_ops_defaults_are_unsupported_or_noop() {
        let mut ops = BareOps;
        assert_eq!(ops.ap_read_raw(1, 0x0C), Err(Error::Unsupported));
        assert_eq!(ops.ap_write_raw(1, 0x00, 1), Err(Error::Unsupported));
        assert_eq!(ops.reconnect(), Err(Error::Unsupported));
        assert_eq!(ops.delay_us(20_000), Ok(()));
        assert_eq!(
            generic_set_target_state(
                &mut ops,
                TargetState::ResetProgram,
                ResetType::SoftwareSysReset
            ),
            Ok(())
        );
    }

    /// The default `unlock` is a no-op: reports `NotLocked` under every policy.
    #[test]
    fn default_unlock_is_notlocked_under_every_policy() {
        for policy in [UnlockPolicy::Refuse, UnlockPolicy::EraseAll] {
            let mut ops = RecOps::default();
            let out = GENERIC_RESET_SEQUENCE
                .unlock(&mut ops, policy)
                .expect("default unlock");
            assert_eq!(out, UnlockOutcome::NotLocked);
            assert!(ops.ops.is_empty(), "default unlock performs no operations");
        }
    }

    /// Dispatch through a `&dyn ResetSequence` reaches the generic default.
    #[test]
    fn dyn_dispatch_reaches_default() {
        let seq: &dyn ResetSequence = &GENERIC_RESET_SEQUENCE;
        let mut ops = RecOps::default();
        seq.set_target_state(&mut ops, TargetState::Halt, ResetType::SoftwareSysReset)
            .expect("dyn dispatch");
        assert_eq!(ops.ops, std::vec![Op::Halt]);
    }
}
