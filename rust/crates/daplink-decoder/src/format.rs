//! Compile-time per-format descriptors a consumer composes into its accepted-format set.

use crate::decoder::Decoder;
use crate::detect::Format;

/// A zero-sized, type-level descriptor for one image format.
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
