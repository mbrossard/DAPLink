//! Error types for the flash runtime.

/// SWD-layer error, mirroring the subset of `dap_rs::swd::Error` this crate acts on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SwdError {
    /// Parity error on a transfer.
    Parity,
    /// Target responded WAIT (retries exhausted).
    Wait,
    /// Target responded FAULT.
    Fault,
    /// Protocol error / no acknowledge.
    Protocol,
    /// Unknown acknowledge value.
    Unknown(u8),
}

impl From<dap_rs::swd::Error> for SwdError {
    fn from(e: dap_rs::swd::Error) -> Self {
        use dap_rs::swd::Error as E;
        match e {
            E::BadParity => SwdError::Parity,
            E::AckWait => SwdError::Wait,
            E::AckFault => SwdError::Fault,
            E::NoAck | E::AckProtocol => SwdError::Protocol,
            E::AckUnknown(v) => SwdError::Unknown(v),
        }
    }
}

/// Errors produced by the flash runtime.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// A low-level SWD transfer failed.
    Swd(SwdError),
    /// A polled operation timed out.
    Timeout,
    /// Flash-algorithm `init` returned a non-zero status.
    InitFailed,
    /// Flash-algorithm `uninit` returned a non-zero status.
    UninitFailed,
    /// Target reset sequence failed.
    ResetFailed,
    /// A [`ResetOps`](crate::reset_sequence::ResetOps) primitive the port in use does not implement.
    Unsupported,
    /// The debug access port is locked and the active
    /// [`UnlockPolicy`](crate::reset_sequence::UnlockPolicy) refuses to unlock it.
    TargetLocked,
    /// Debug power-up handshake did not complete in time.
    PowerUpTimeout,
    /// Debug init (line reset / DPIDR / power-up) failed.
    DebugInitFailed,
    /// No flash algorithm is available for the region.
    AlgoMissing,
    /// The target is recognized but no flash algorithm has been produced for it yet.
    AlgorithmUnavailable,
    /// Downloading the algorithm blob into target RAM failed.
    AlgoDownloadFailed,
    /// A flash-algorithm syscall did not return in time.
    AlgoTimeout,
    /// A flash-algorithm syscall returned a non-zero status code.
    AlgoFailed {
        /// The status code returned in r0.
        code: u32,
    },
    /// A sector or chip erase failed.
    EraseFailed,
    /// The requested erase mode is not supported by the algorithm.
    EraseNotSupported,
    /// A page write failed.
    WriteFailed,
    /// Post-program verify mismatched at `addr`.
    VerifyFailed {
        /// First mismatching address.
        addr: u32,
    },
    /// The operation would set target security bits (refused).
    SecurityBitsWouldBeSet,
    /// Payload larger than the target's staging buffer.
    DataTooLarge {
        /// Requested size.
        size: u32,
        /// Maximum supported size.
        max: u32,
    },
    /// A caller-provided buffer was too small.
    BufferTooSmall {
        /// Provided size.
        size: usize,
        /// Minimum required size.
        min: usize,
    },
    /// An operation was attempted in an invalid state.
    InvalidState,
    /// An address was not aligned to the required boundary.
    AddressNotAligned {
        /// The offending address.
        addr: u32,
        /// The required alignment.
        alignment: u32,
    },
    /// An address fell outside every known region.
    AddressOutOfRange {
        /// The offending address.
        addr: u32,
    },
    /// No flash region covers `addr`.
    NoFlashRegion {
        /// The unmatched address.
        addr: u32,
    },
    /// A target-identity probe read a value that did not match the expected signature.
    IdentityMismatch {
        /// Address of the identity register/word that was read.
        addr: u32,
        /// Expected value after masking (`read & mask == expected` was required).
        expected: u32,
        /// The value actually read (already masked with the probe's mask), for diagnostics.
        read: u32,
    },
}

impl From<dap_rs::swd::Error> for Error {
    fn from(e: dap_rs::swd::Error) -> Self {
        Error::Swd(SwdError::from(e))
    }
}

impl From<SwdError> for Error {
    fn from(e: SwdError) -> Self {
        Error::Swd(e)
    }
}
