//! The streaming decoder seam.

use crate::error::Error;

/// Receives decoded image fragments at absolute target addresses.
pub trait DecoderContext {
    /// Place `data` at `addr`; an error stops the decoder, leaving earlier fragments placed.
    fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error>;

    /// Report each parsed UF2 block's family, `None` when undeclared; `false` disregards the block.
    fn set_uf2_family(&mut self, _family: Option<u32>) -> bool {
        true
    }
}

/// A streaming image decoder; preserve its scratchpad contents across input calls.
pub trait Decoder {
    /// Consume the next `data` chunk.
    fn input(
        &mut self,
        data: &[u8],
        scratchpad: &mut [u8],
        context: &mut dyn DecoderContext,
    ) -> Result<(), Error>;

    /// Finalize; errors if the stream ended mid-record/block or lacked a required terminator.
    fn finish(
        &mut self,
        scratchpad: &mut [u8],
        context: &mut dyn DecoderContext,
    ) -> Result<(), Error>;
}
