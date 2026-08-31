//! Hand-written reset/halt behavior for the RP2040 family.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{generic_set_target_state, ResetOps, ResetSequence};
use daplink_flash::target::{ResetType, TargetState};

/// The RP2040's reset/halt strategy: `ResetProgram` halts without resetting, and any reset-to-run
/// is forced to `SYSRESETREQ`, because every RP2040 reset drops the multidrop debug port.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rp2040ResetSequence;

impl Rp2040ResetSequence {
    /// Multidrop core-select hook; a no-op, since attach already selects core 0 by `TARGETSEL`.
    fn select_core(&self, _ops: &mut dyn ResetOps) -> Result<(), Error> {
        Ok(())
    }
}

impl ResetSequence for Rp2040ResetSequence {
    fn set_target_state(
        &self,
        ops: &mut dyn ResetOps,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        self.select_core(ops)?;
        match state {
            // Halt-only: any reset drops the multidrop DP, and the RAM-resident algorithm sets its own PC.
            TargetState::ResetProgram => ops.halt(),
            // Forced `SYSRESETREQ`: nRESET may not be wired, and the session ends here anyway.
            TargetState::PostFlashReset | TargetState::ResetRun => {
                generic_set_target_state(ops, state, ResetType::SoftwareSysReset)
            }
            // Other transitions take the generic path, dispatched on the target's `reset_type`.
            _ => generic_set_target_state(ops, state, reset_type),
        }
    }
}

/// The shared RP2040 reset-sequence instance handed out by a target's `reset_sequence()`.
pub static RP2040_RESET_SEQUENCE: Rp2040ResetSequence = Rp2040ResetSequence;
