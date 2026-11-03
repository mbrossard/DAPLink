//! Hand-written reset/halt behavior for the RP2350 (Pico 2) family.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{generic_set_target_state, ResetOps, ResetSequence};
use daplink_flash::target::{ResetType, TargetState};

use super::rp::{clear_core_reg, normalise, quiesce, Blocks, VC_HARDERR};

/// RP2350 `WATCHDOG`, `PSM` and `RESETS` bases, and `PSM.PROC1`.
const BLOCKS: Blocks = Blocks {
    watchdog: 0x400D_8000,
    psm: 0x4001_8000,
    psm_proc1: 1 << 24,
    resets: 0x4002_0000,
    demcr: VC_HARDERR | VC_OTHER_FAULTS,
};

/// `VC_SFERR`, `VC_INTERR`, `VC_BUSERR`, `VC_STATERR`, `VC_CHKERR`, `VC_NOCPERR` and `VC_MMERR`.
const VC_OTHER_FAULTS: u32 = 0xBF0;

/// `ACCESSCTRL.CFGRESET` and its write: the password and the bit restoring boot-time permissions.
const ACCESSCTRL_CFGRESET: u32 = 0x4006_0008;
const CFGRESET: u32 = 0xACCE_0001;
/// `DSCSR`, its Secure-state bit `CDS`, and `CDSKEY`, which a write changing `CDS` must clear.
const DSCSR: u32 = 0xE000_EE08;
const DSCSR_CDS: u32 = 1 << 16;
const DSCSR_CDSKEY: u32 = 1 << 17;
/// `DCRSR` selectors of `MSPLIM_S` and `PSPLIM_S`.
const STACK_LIMITS: [u32; 2] = [0x1C, 0x1D];

/// RP2350 reset/halt: `ResetProgram` halts and quiesces the app, and reset-to-run forces `SYSRESETREQ`.
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
            // No reset: it would drop the DP and clear the catch, so no halt could be confirmed.
            TargetState::ResetProgram => {
                ops.halt()?;
                // First, so an app's bus permissions cannot refuse the quiesce writes.
                ops.write_word(ACCESSCTRL_CFGRESET, CFGRESET)?;
                quiesce(ops, &BLOCKS)?;
                // The algorithm calls the Secure bootrom, so a core halted Non-secure switches.
                let dscsr = ops.read_word(DSCSR)?;
                if dscsr & DSCSR_CDS == 0 {
                    ops.write_word(DSCSR, (dscsr & !DSCSR_CDSKEY) | DSCSR_CDS)?;
                }
                // No reset clears an app's stack limits, and the algorithm's stack may sit below them.
                for reg in STACK_LIMITS {
                    clear_core_reg(ops, reg)?;
                }
                // After the switch: both are banked, and a debugger reaches the current state's copy.
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

/// The shared RP2350 reset-sequence instance handed out by a target's `reset_sequence()`.
pub static RP2350_RESET_SEQUENCE: Rp2350ResetSequence = Rp2350ResetSequence;
