//! Probe-control traits shared across the flash stack.

/// Drive the target reset line.
pub trait ResetControl {
    /// Reset-control error type.
    type Error;
    /// Assert (`true` = hold in reset) or release the target reset line.
    fn set_reset(&mut self, asserted: bool) -> Result<(), Self::Error>;
}

/// Read-only probe presence gate: report the attached debug port's identity.
pub trait ProbeInfo {
    /// Read the debug-port IDCODE/DPIDR; `None` on failure. Emits the classic wake preamble.
    fn read_dpidr(&mut self) -> Option<u32>;
}

/// Exclusive-access arbiter for the shared SWD bus (flash-wins).
pub trait FlashArbiter {
    /// RAII session guard; dropping it releases exclusive access.
    type Session;
    /// Begin an exclusive session, or `None` if one is already live.
    fn begin_session(&self) -> Option<Self::Session>;
}

/// Zero-cost no-op arbiter for dedicated (non-shared) adapters: always grants.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoArbiter;

impl FlashArbiter for NoArbiter {
    type Session = ();
    fn begin_session(&self) -> Option<()> {
        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_arbiter_grants() {
        let a = NoArbiter;
        assert!(a.begin_session().is_some());
    }
}
