//! The per-target reset/halt sequence seam.

use crate::error::Error;
use crate::swd_host_core::{C_DEBUGEN, DBGKEY, DEMCR, DHCSR, TRCENA, VC_CORERESET};
use crate::target::{ResetType, TargetState};

/// Object-safe debug primitives a [`ResetSequence`] drives the target through.
pub trait ResetOps {
    /// Write a single 32-bit word to the target's memory map.
    fn write_word(&mut self, addr: u32, value: u32) -> Result<(), Error>;
    /// Read a single 32-bit word from the target's memory map.
    fn read_word(&mut self, addr: u32) -> Result<u32, Error>;
    /// Trigger a reset of the given type (nRESET pulse or an AIRCR software reset).
    fn trigger_reset(&mut self, reset_type: ResetType) -> Result<(), Error>;
    /// Poll DHCSR until the core reports halted, re-establishing debug if a transfer faults
    /// while the core is still leaving reset.
    fn poll_halt(&mut self) -> Result<(), Error>;
    /// Assert (`true` = hold in reset) or release the physical nRESET line.
    fn set_reset(&mut self, asserted: bool) -> Result<(), Error>;
    /// Halt the core via DHCSR and wait for the halt to take effect.
    fn halt(&mut self) -> Result<(), Error>;

    /// Read a raw access-port register: `apsel` selects the AP, `addr` the byte offset in its
    /// register space. Defaults to `Err(Unsupported)`.
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

    /// Re-establish the debug connection after a sequence-triggered chip reset. Defaults to
    /// `Err(Unsupported)`.
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

/// A target's reset/halt strategy: drives the target into a [`TargetState`], using
/// `reset_type` for any reset the transition entails.
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

    /// Run the family's access-port unlock step, once per session after DP power-up. Default:
    /// no-op returning [`UnlockOutcome::NotLocked`].
    fn unlock(
        &self,
        _ops: &mut dyn ResetOps,
        _policy: UnlockPolicy,
    ) -> Result<UnlockOutcome, Error> {
        Ok(UnlockOutcome::NotLocked)
    }
}

/// Report the target's debug state after a reset-and-halt primitive has already failed: DP
/// CTRL/STAT, then DHCSR, then DEMCR.
#[cfg(feature = "defmt")]
fn report_debug_state(ops: &mut dyn ResetOps) {
    match ops.dp_ctrl_stat() {
        Ok(value) => crate::debug_status::log_ctrl_stat(value),
        Err(e) => defmt::warn!("debug state: DP CTRL/STAT read failed: {}", e),
    }
    match ops.read_word(DHCSR) {
        Ok(value) => crate::debug_status::log_dhcsr("post-mortem", value),
        Err(e) => defmt::warn!("debug state: DHCSR read failed: {}", e),
    }
    match ops.read_word(DEMCR) {
        Ok(value) => crate::debug_status::log_demcr("post-mortem", value),
        Err(e) => defmt::warn!("debug state: DEMCR read failed: {}", e),
    }
}

/// Run one primitive of the reset-and-halt, discarding the step's name.
#[cfg(not(feature = "defmt"))]
macro_rules! reset_step {
    ($ops:ident, $label:literal, $op:expr) => {
        $op?
    };
}

/// Run one primitive of the reset-and-halt and, if it fails, name it and read the target's debug
/// state back before propagating.
#[cfg(feature = "defmt")]
macro_rules! reset_step {
    ($ops:ident, $label:literal, $op:expr) => {{
        let outcome = $op;
        if let Err(e) = &outcome {
            defmt::warn!("reset-and-halt: {=str} failed: {}", $label, e);
            report_debug_state($ops);
        }
        outcome?
    }};
}

/// Settling time between the reset request and the first halt poll, in microseconds.
pub(crate) const RESET_SETTLE_US: u32 = 2_000;

/// The generic reset sequence built on the memory-mapped CoreSight debug registers
/// (DHCSR/DEMCR/AIRCR), with no per-device knowledge.
pub fn generic_set_target_state(
    ops: &mut dyn ResetOps,
    state: TargetState,
    reset_type: ResetType,
) -> Result<(), Error> {
    match state {
        TargetState::ResetProgram => {
            // Halt and confirm the halt before anything is armed.
            reset_step!(ops, "halt before arming the catch", ops.halt());
            // Arm the reset vector catch and keep debug enabled across the reset.
            reset_step!(
                ops,
                "arm vector catch (DEMCR)",
                ops.write_word(DEMCR, TRCENA | VC_CORERESET)
            );
            reset_step!(
                ops,
                "enable debug (DHCSR)",
                ops.write_word(DHCSR, DBGKEY | C_DEBUGEN)
            );
            reset_step!(ops, "trigger reset", ops.trigger_reset(reset_type));
            reset_step!(ops, "post-reset settle", ops.delay_us(RESET_SETTLE_US));
            reset_step!(ops, "halt poll", ops.poll_halt());
            // Disarm the catch, leaving trace enabled.
            reset_step!(
                ops,
                "disarm vector catch (DEMCR)",
                ops.write_word(DEMCR, TRCENA)
            );
            Ok(())
        }
        TargetState::ResetRun | TargetState::PostFlashReset => {
            // Clear any catch and release debug so the core runs after the reset.
            ops.write_word(DEMCR, TRCENA)?;
            ops.write_word(DHCSR, DBGKEY)?;
            ops.trigger_reset(reset_type)
        }
        TargetState::ResetHold => match reset_type {
            ResetType::Hardware => ops.set_reset(true),
            // A software reset cannot latch a hold; halt as the closest approximation.
            ResetType::SoftwareSysReset | ResetType::SoftwareVectReset => ops.halt(),
        },
        TargetState::Halt => ops.halt(),
        TargetState::Run | TargetState::NoDebug => {
            // Release the core to run with debug disabled.
            ops.write_word(DHCSR, DBGKEY)
        }
        TargetState::Debug => {
            // Enable debug without halting.
            ops.write_word(DHCSR, DBGKEY | C_DEBUGEN)
        }
    }
}

/// The generic (default) reset sequence: zero per-device knowledge, behaves exactly like
/// [`generic_set_target_state`].
#[derive(Clone, Copy, Debug, Default)]
pub struct GenericResetSequence;

impl ResetSequence for GenericResetSequence {}

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
            self.ops.push(Op::Write(addr, value));
            Ok(())
        }
        fn read_word(&mut self, addr: u32) -> Result<u32, Error> {
            self.ops.push(Op::Read(addr));
            Ok(0)
        }
        fn trigger_reset(&mut self, reset_type: ResetType) -> Result<(), Error> {
            self.ops.push(Op::Reset(reset_type));
            if self.fail_trigger {
                return Err(Error::ResetFailed);
            }
            Ok(())
        }
        fn poll_halt(&mut self) -> Result<(), Error> {
            self.ops.push(Op::PollHalt);
            Ok(())
        }
        fn set_reset(&mut self, asserted: bool) -> Result<(), Error> {
            self.ops.push(Op::SetReset(asserted));
            Ok(())
        }
        fn halt(&mut self) -> Result<(), Error> {
            self.ops.push(Op::Halt);
            Ok(())
        }
        fn delay_us(&mut self, us: u32) -> Result<(), Error> {
            self.ops.push(Op::Delay(us));
            Ok(())
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
                Op::Write(DEMCR, TRCENA | VC_CORERESET),
                Op::Write(DHCSR, DBGKEY | C_DEBUGEN),
                Op::Reset(ResetType::Hardware),
                Op::Delay(RESET_SETTLE_US),
                Op::PollHalt,
                Op::Write(DEMCR, TRCENA),
            ]
        );
    }

    /// The core is halted and the halt confirmed before anything is armed, and the halt poll is
    /// not issued until the part has had [`RESET_SETTLE_US`] to settle.
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
            at(Op::Halt) < at(Op::Write(DEMCR, TRCENA | VC_CORERESET)),
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
                Op::Write(DEMCR, TRCENA | VC_CORERESET),
                Op::Write(DHCSR, DBGKEY | C_DEBUGEN),
                Op::Reset(ResetType::SoftwareSysReset),
                Op::Delay(RESET_SETTLE_US),
                Op::PollHalt,
                Op::Write(DEMCR, TRCENA),
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
                Op::Write(DEMCR, TRCENA),
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
                Op::Write(DEMCR, TRCENA),
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

    /// A failing `trigger_reset` propagates its error and performs no further operations; the
    /// disarm write never runs, so the catch is left armed.
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
                Op::Write(DEMCR, TRCENA | VC_CORERESET),
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
