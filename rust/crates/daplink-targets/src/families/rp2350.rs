//! Hand-written reset/halt behavior for the RP2350 (Pico 2) family.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{generic_set_target_state, ResetOps, ResetSequence};
use daplink_flash::target::{ResetType, TargetState};

/// The RP2350's reset/halt strategy: `ResetProgram` halts without resetting, and any reset-to-run
/// is forced to `SYSRESETREQ`, because every RP2350 reset drops the multidrop debug port.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rp2350ResetSequence;

impl ResetSequence for Rp2350ResetSequence {
    fn set_target_state(
        &self,
        ops: &mut dyn ResetOps,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        match state {
            // Halt-only: a reset would drop the DP and clear the catch, so no halt could be confirmed.
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

/// The shared RP2350 reset-sequence instance handed out by a target's `reset_sequence()`.
pub static RP2350_RESET_SEQUENCE: Rp2350ResetSequence = Rp2350ResetSequence;
