//! Decoder error type.

/// Errors produced by the image decoders.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A uf2 block was malformed: bad magic, oversized payload, or a trailing partial block.
    Uf2,
    /// An internal invariant failed, e.g. the caller's scratchpad is too small for a record/block.
    Internal,
}
