//! Hand-written reset/unlock behavior and FICR facts for the Nordic nRF52 family.

use daplink_flash::error::Error;
use daplink_flash::identity::IdentityProbe;
use daplink_flash::reset_sequence::{
    generic_set_target_state, ResetOps, ResetSequence, UnlockOutcome, UnlockPolicy,
};
use daplink_flash::target::{ResetType, TargetState};

/// APSEL of the CTRL-AP, reachable even while APPROTECT locks the memory AP.
pub const CTRL_AP: u8 = 1;
/// CTRL-AP `RESET`: 1 holds a full-chip soft reset, 0 releases it.
pub const CTRL_AP_RESET: u8 = 0x00;
/// CTRL-AP `APPROTECTSTATUS`: 0 when access-port protection is enabled.
pub const CTRL_AP_APPROTECTSTATUS: u8 = 0x0C;

/// `FICR.INFO.PART`, the part number (e.g. `0x0005_2840`).
pub const FICR_INFO_PART: u32 = 0x1000_0100;
/// `FICR.CODEPAGESIZE`, the code-flash page size in bytes.
pub const FICR_CODEPAGESIZE: u32 = 0x1000_0010;
/// The family's uniform 4 KiB code-flash page.
pub const CODEPAGESIZE_4K: u32 = 0x0000_1000;

/// An exact `FICR.INFO.PART` identity probe for one nRF52 variant.
pub const fn info_part_probe(part: u32) -> IdentityProbe {
    IdentityProbe::new(FICR_INFO_PART, 0xFFFF_FFFF, part)
}

/// Pins the 4 KiB `FICR.CODEPAGESIZE` the records assume.
pub const CODEPAGESIZE_PROBE: IdentityProbe =
    IdentityProbe::new(FICR_CODEPAGESIZE, 0xFFFF_FFFF, CODEPAGESIZE_4K);

/// nRF52 reset/unlock: `ResetHold` latches CTRL-AP `RESET`; a locked part is refused, never erased.
#[derive(Clone, Copy, Debug, Default)]
pub struct Nrf52ResetSequence;

impl ResetSequence for Nrf52ResetSequence {
    fn set_target_state(
        &self,
        ops: &mut dyn ResetOps,
        state: TargetState,
        reset_type: ResetType,
    ) -> Result<(), Error> {
        match state {
            TargetState::ResetHold => ops.ap_write_raw(CTRL_AP, CTRL_AP_RESET, 1),
            _ => generic_set_target_state(ops, state, reset_type),
        }
    }

    fn unlock(
        &self,
        ops: &mut dyn ResetOps,
        _policy: UnlockPolicy,
    ) -> Result<UnlockOutcome, Error> {
        if ops.ap_read_raw(CTRL_AP, CTRL_AP_APPROTECTSTATUS)? != 0 {
            return Ok(UnlockOutcome::NotLocked);
        }
        // Recovery needs a destructive ERASEALL, which no policy performs here.
        Err(Error::TargetLocked)
    }
}

/// The shared nRF52 reset-sequence instance handed out by a target's `reset_sequence()`.
pub static NRF52_RESET_SEQUENCE: Nrf52ResetSequence = Nrf52ResetSequence;
