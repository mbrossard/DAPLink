//! Decoded `defmt` reports of the debug registers, each naming when the value was observed.

use crate::swd_host_core::{
    CDBGPWRUPACK, CSYSPWRUPACK, C_DEBUGEN, C_HALT, S_HALT, S_REGRDY, TRCENA, VC_CORERESET,
};

/// The core is locked up (an unrecoverable fault).
const S_LOCKUP: u32 = 1 << 19;
/// The core is sleeping (WFI/WFE).
const S_SLEEP: u32 = 1 << 18;
/// An instruction retired since this bit was last read. Clear-on-read.
const S_RETIRE_ST: u32 = 1 << 24;
/// The core has been reset, or is being reset, since this bit was last read. Clear-on-read.
const S_RESET_ST: u32 = 1 << 25;

/// Catch a HardFault as a debug event.
const VC_HARDERR: u32 = 1 << 10;

/// A transaction returned FAULT and latched an error.
const STICKYERR: u32 = 1 << 5;
/// A pushed-compare/verify operation mismatched.
const STICKYCMP: u32 = 1 << 4;
/// An overrun occurred: a transfer was issued before the previous one completed.
const STICKYORUN: u32 = 1 << 1;
/// A write-data error was detected.
const WDATAERR: u32 = 1 << 7;
/// The most recent read or RDBUFF access returned OK.
const READOK: u32 = 1 << 6;

/// Report a DHCSR value, decoded. `when` names the observation.
pub(crate) fn log_dhcsr(when: &str, value: u32) {
    defmt::warn!(
        "debug state: DHCSR ({=str}) = {=u32:#010x} S_HALT={=bool} S_LOCKUP={=bool} S_SLEEP={=bool} S_RESET_ST={=bool} S_RETIRE_ST={=bool} S_REGRDY={=bool} C_HALT={=bool} C_DEBUGEN={=bool}",
        when,
        value,
        value & S_HALT != 0,
        value & S_LOCKUP != 0,
        value & S_SLEEP != 0,
        value & S_RESET_ST != 0,
        value & S_RETIRE_ST != 0,
        value & S_REGRDY != 0,
        value & C_HALT != 0,
        value & C_DEBUGEN != 0,
    );
}

/// Report a DEMCR value, decoded.
pub(crate) fn log_demcr(when: &str, value: u32) {
    defmt::warn!(
        "debug state: DEMCR ({=str}) = {=u32:#010x} VC_CORERESET={=bool} VC_HARDERR={=bool} TRCENA={=bool}",
        when,
        value,
        value & VC_CORERESET != 0,
        value & VC_HARDERR != 0,
        value & TRCENA != 0,
    );
}

/// Report a debug-port CTRL/STAT value, decoded.
pub(crate) fn log_ctrl_stat(value: u32) {
    defmt::warn!(
        "debug state: DP CTRL/STAT = {=u32:#010x} CDBGPWRUPACK={=bool} CSYSPWRUPACK={=bool} STICKYERR={=bool} STICKYCMP={=bool} STICKYORUN={=bool} WDATAERR={=bool} READOK={=bool}",
        value,
        value & CDBGPWRUPACK != 0,
        value & CSYSPWRUPACK != 0,
        value & STICKYERR != 0,
        value & STICKYCMP != 0,
        value & STICKYORUN != 0,
        value & WDATAERR != 0,
        value & READOK != 0,
    );
}

/// Report how a reset-window halt poll ended.
pub(crate) fn log_poll_halt(
    outcome: &str,
    answered: u32,
    elapsed_us: u64,
    first: Option<u32>,
    last: Option<u32>,
    reconnected: bool,
) {
    defmt::info!(
        "wait_for_halt: {=str} after {=u32} answered DHCSR reads in {=u64} us (reconnect spent: {=bool})",
        outcome,
        answered,
        elapsed_us,
        reconnected,
    );
    match (first, last) {
        (Some(f), Some(l)) => {
            log_dhcsr("first answered poll", f);
            if l != f {
                log_dhcsr("last answered poll", l);
            }
        }
        // No DHCSR read was answered inside the window.
        _ => defmt::warn!("wait_for_halt: no DHCSR read was answered inside the reset window"),
    }
}
