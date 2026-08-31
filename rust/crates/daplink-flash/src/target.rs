//! Target description and lifecycle callbacks.

use crate::flash_algo::{FlashRegion, RamRegion, SectorInfo};
use crate::identity::IdentityProbe;
use crate::reset_sequence::{ResetSequence, GENERIC_RESET_SEQUENCE};

/// How a target is reset during a flashing session.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetType {
    /// Toggle the physical nRESET line.
    Hardware,
    /// Software reset via the vector-catch reset (VECTRESET).
    SoftwareVectReset,
    /// Software reset via a full system reset request (SYSRESETREQ).
    SoftwareSysReset,
}

/// How a target's debug port is woken out of its power-on state before the first DPIDR read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DpWake {
    /// Classic JTAG-to-SWD switch (`0xE79E`), bracketed by line resets.
    JtagToSwd,
    /// ADIv6 dormant-state exit: 128-bit selection alert plus SWD activation code.
    DormantToSwd,
}

/// How access ports are addressed on the debug port: ADIv5 (8-bit `APSEL`) or ADIv6 (a full AP
/// base address programmed into `SELECT`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ApAddressing {
    /// ADIv5: the access port is selected by an 8-bit `APSEL` placed in `SELECT[31:24]`.
    V5,
    /// ADIv6: the access port is selected by its base address placed in `SELECT[31:4]`.
    V6 {
        /// The access port's base address, programmed into `SELECT[31:4]`.
        ap_base: u32,
    },
}

/// Data-driven debug-port connection policy: how to wake the DP, whether a multidrop target
/// selection is required, and how access ports are addressed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DebugPort {
    /// The wake strategy applied before the first DPIDR read.
    pub wake: DpWake,
    /// Multidrop `TARGETSEL`; `None` for a single-drop port.
    pub targetsel: Option<u32>,
    /// Access-port addressing model (ADIv5 vs ADIv6).
    pub ap: ApAddressing,
}

impl Default for DebugPort {
    fn default() -> Self {
        Self {
            wake: DpWake::JtagToSwd,
            targetsel: None,
            ap: ApAddressing::V5,
        }
    }
}

/// Debug lifecycle states a target can be driven into.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TargetState {
    /// Held in reset.
    ResetHold,
    /// Reset then halted, ready to run an algorithm.
    ResetProgram,
    /// Reset then released to run.
    ResetRun,
    /// Debug disabled.
    NoDebug,
    /// Debug enabled.
    Debug,
    /// Halted.
    Halt,
    /// Running.
    Run,
    /// Reset applied after flashing completed.
    PostFlashReset,
}

/// Static description of a debuggable target: its flash, RAM and sector layout. Object-safe, so a
/// `&dyn Target` can be stored generically.
pub trait Target {
    /// A short, human-readable name for this target (e.g. `"RP2040"`); defaults to `""`.
    fn name(&self) -> &'static str {
        ""
    }

    /// Programmable flash regions.
    fn flash_regions(&self) -> &[FlashRegion<'_>];
    /// RAM regions available for the algorithm and its buffers.
    fn ram_regions(&self) -> &[RamRegion];
    /// Sector layout runs across the flash.
    fn sectors(&self) -> &[SectorInfo];

    /// Reset mechanism to use (defaults to [`ResetType::Hardware`]).
    fn reset_type(&self) -> ResetType {
        ResetType::Hardware
    }

    /// The reset/halt strategy this target uses; defaults to the generic sequence dispatched on
    /// [`reset_type`](Self::reset_type).
    fn reset_sequence(&self) -> &dyn ResetSequence {
        &GENERIC_RESET_SEQUENCE
    }

    /// Access-port selector for the debug AP (defaults to 0). Used only under ADIv5 addressing;
    /// under ADIv6 the AP is chosen by [`DebugPort::ap`]'s base address instead.
    fn apsel(&self) -> u32 {
        0
    }

    /// The debug-port connection policy; defaults to [`DebugPort::default`].
    fn debug_port(&self) -> DebugPort {
        DebugPort::default()
    }

    /// Whether a flash algorithm is available for this target; defaults to `true`.
    fn flash_algorithm_available(&self) -> bool {
        true
    }

    /// Whether the target must be reset after erase (defaults to `false`).
    fn erase_reset(&self) -> bool {
        false
    }

    /// TAR auto-increment page size in bytes (defaults to 1024).
    fn auto_increment_size(&self) -> u32 {
        1024
    }

    /// Preferred SWD clock in Hz, or 0 for the probe default.
    fn flash_clock(&self) -> u32 {
        0
    }

    /// Target-identity probes run after connect/halt, before any erase/program. Default empty.
    fn identity_probes(&self) -> &[IdentityProbe] {
        &[]
    }

    /// The Microsoft UF2 family id for this device, or `None` if it has none.
    fn uf2_family(&self) -> Option<u32> {
        None
    }
}

/// Whether a callback handled a [`TargetState`] transition itself, or wants the caller to
/// fall through to the generic reset sequence.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StateOutcome {
    /// The callback drove the transition; the caller must not run the generic sequence.
    Handled,
    /// The callback did nothing; the caller should run the generic reset sequence.
    NotHandled,
}

/// Hooks invoked at various points of the flashing lifecycle; all default to no-ops.
pub trait TargetCallbacks<S, D> {
    /// Called just before debug initialization.
    fn before_init_debug(&self, _swd: &mut S, _delay: &mut D) -> Result<(), crate::Error> {
        Ok(())
    }

    /// Called before running the algorithm, to apply any pre-run configuration.
    fn prerun_config(&self, _swd: &mut S, _delay: &mut D) -> Result<(), crate::Error> {
        Ok(())
    }

    /// Whether writing `data` at `addr` would set device security bits.
    fn security_bits_set(&self, _addr: u32, _data: &[u8]) -> bool {
        false
    }

    /// Drive the target into `state`; return [`StateOutcome::Handled`] to claim the transition,
    /// otherwise the caller applies the generic reset sequence.
    fn set_target_state(
        &self,
        _swd: &mut S,
        _delay: &mut D,
        _state: TargetState,
    ) -> Result<StateOutcome, crate::Error> {
        Ok(StateOutcome::NotHandled)
    }
}

/// Default callbacks: every hook is the trait's default no-op.
#[derive(Clone, Copy, Debug, Default)]
pub struct DefaultCallbacks;

impl<S, D> TargetCallbacks<S, D> for DefaultCallbacks {}
