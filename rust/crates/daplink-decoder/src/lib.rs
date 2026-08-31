#![no_std]
#![deny(missing_docs)]
#![cfg_attr(not(test), deny(clippy::unwrap_used))]
//! Zero-dependency streaming decoders for firmware images plus content detection.
//!
//! A consumer expresses the set of formats it accepts by composing the per-format descriptor
//! types ([`Uf2Format`]) it wires together; [`FormatDecoder`] ties each descriptor to its
//! streaming decoder, canonical extension, and [`Format`] tag.

#[cfg(feature = "std")]
extern crate std;

pub mod decoder;
pub mod detect;
#[cfg(feature = "std")]
pub mod encode;
pub mod error;
pub mod format;
pub mod uf2;

pub use decoder::{Decoder, DecoderContext};
pub use detect::{detect_format, Format};
pub use error::Error;
pub use format::FormatDecoder;
pub use uf2::{Uf2Decoder, Uf2Format};

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    struct Rec {
        writes: std::vec::Vec<(u32, std::vec::Vec<u8>)>,
    }

    impl DecoderContext for Rec {
        fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
            self.writes.push((addr, data.to_vec()));
            Ok(())
        }
    }

    /// End-to-end through the public facade: a consumer picks a format by composing its
    /// [`FormatDecoder`] descriptor, feeds a malformed block, and gets the guard-rejection
    /// `Error` back rather than a panic or a silently accepted write.
    #[test]
    fn uf2_facade_rejects_malformed_block() {
        let mut dec = <Uf2Format as FormatDecoder>::make();
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
        };
        let err = dec.input(&[0xAAu8; 512], &mut sp, &mut ctx);
        assert_eq!(err, Err(Error::Uf2));
        assert!(ctx.writes.is_empty());
    }
}
