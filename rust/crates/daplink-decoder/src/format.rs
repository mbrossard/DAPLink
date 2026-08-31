//! The compile-time per-format descriptor seam: a consumer composes its accepted-format set out
//! of unit descriptor types ([`Uf2Format`](crate::Uf2Format)) rather than setting a build flag.

use crate::decoder::Decoder;
use crate::detect::Format;

/// A compile-time descriptor for one image format.
///
/// Implementors are zero-sized unit types used only as type-level tags; a consumer composes them
/// into an accepted-format set and never constructs a value of the descriptor itself.
pub trait FormatDecoder {
    /// The streaming decoder this format constructs.
    type Running: Decoder;

    /// The [`Format`] tag this descriptor represents.
    const FORMAT: Format;

    /// The uppercase 8.3 extension field (space-padded) that names this format on a volume.
    const EXT: [u8; 3];

    /// Construct a fresh streaming decoder for this format.
    fn make() -> Self::Running;
}
