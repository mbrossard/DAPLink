//! The streaming decoder seam.

use crate::error::Error;

/// Sink for decoded image fragments. The consumer (a virtual-FS transfer engine, tests)
/// implements this; a decoder calls `write_fragment` with an absolute target address and the
/// bytes to place there.
pub trait DecoderContext {
    /// Place `data` at absolute address `addr`.
    fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error>;

    /// Report the UF2 family id of the block about to be written, or `None` when the block
    /// declares no family. The UF2 decoder calls this once per block, immediately before the
    /// block's [`write_fragment`](Self::write_fragment), so a consumer can cross-check the
    /// family against a target before accepting the bytes. The default is a no-op; formats
    /// without a family concept (raw binary, Intel-hex) never call it.
    fn set_uf2_family(&mut self, _family: Option<u32>) {}
}

/// A streaming image decoder. The caller feeds arbitrary `data` chunks and supplies a
/// caller-owned `scratchpad` used to re-assemble records/blocks that span chunk boundaries; the
/// scratchpad's contents MUST persist across `input` calls for a given decoder instance. Decoded
/// output is delivered through `context`.
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
