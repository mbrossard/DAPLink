//! Hand-written reset/halt behavior for the RP2040 family.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{generic_set_target_state, ResetOps, ResetSequence};
use daplink_flash::target::{ResetType, TargetState};

use super::rp::{normalise, quiesce, Blocks, VC_HARDERR};

/// RP2040 `WATCHDOG`, `PSM` and `RESETS` bases, and `PSM.PROC1`.
const BLOCKS: Blocks = Blocks {
    watchdog: 0x4005_8000,
    psm: 0x4001_0000,
    psm_proc1: 1 << 16,
    resets: 0x4000_C000,
    demcr: VC_HARDERR,
};

/// RP2040 reset/halt: `ResetProgram` halts and quiesces the app, and reset-to-run forces `SYSRESETREQ`.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rp2040ResetSequence;

impl ResetSequence for Rp2040ResetSequence {
    fn set_target_state(
        &self,
        ops: &mut dyn ResetOps,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        match state {
            // No reset: an nRESET drops the multidrop DP, and the RAM-resident algorithm sets its own PC.
            TargetState::ResetProgram => {
                ops.halt()?;
                quiesce(ops, &BLOCKS)?;
                normalise(ops)
            }
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
