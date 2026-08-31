//! The accepted-format set the transfer engine is generic over, composed as a descriptor tuple.

use daplink_decoder::{
    detect_format, Decoder, DecoderContext, Error, Format, FormatDecoder, Uf2Format,
};

/// The full accepted-format set: uf2 is the only image format in tree.
pub type AllDecoders = (Uf2Format,);

/// A running decoder from a composed set: nesting `Either` builds a static-dispatch union sized
/// to the set's largest decoder, naming only composed types.
pub enum Either<L, R> {
    /// The head format's running decoder.
    Left(L),
    /// A running decoder from the tail of the set.
    Right(R),
}

impl<L: Decoder, R: Decoder> Decoder for Either<L, R> {
    fn input(
        &mut self,
        data: &[u8],
        scratchpad: &mut [u8],
        context: &mut dyn DecoderContext,
    ) -> Result<(), Error> {
        match self {
            Either::Left(l) => l.input(data, scratchpad, context),
            Either::Right(r) => r.input(data, scratchpad, context),
        }
    }

    fn finish(
        &mut self,
        scratchpad: &mut [u8],
        context: &mut dyn DecoderContext,
    ) -> Result<(), Error> {
        match self {
            Either::Left(l) => l.finish(scratchpad, context),
            Either::Right(r) => r.finish(scratchpad, context),
        }
    }
}

/// A firmware-composed set of accepted image formats, implemented for tuples of
/// [`FormatDecoder`] descriptors and resolved without dynamic dispatch or a heap.
pub trait DecoderSet {
    /// The running-decoder union for this set.
    type Running: Decoder;

    /// Whether `fmt` is a member of this set.
    fn supports(fmt: Format) -> bool;

    /// The member format named by an uppercase 8.3 extension field, if any.
    fn ext_format(ext: &[u8; 3]) -> Option<Format>;

    /// Construct the running decoder for a member `fmt`.
    fn make(fmt: Format) -> Option<Self::Running>;

    /// Sniff a fresh first `sector` for a member format, by that format's own magic.
    fn detect(sector: &[u8]) -> Option<Format> {
        let f = detect_format(None, sector);
        if f != Format::Unknown && Self::supports(f) {
            return Some(f);
        }
        None
    }
}

impl<A: FormatDecoder> DecoderSet for (A,) {
    type Running = A::Running;

    fn supports(fmt: Format) -> bool {
        fmt == A::FORMAT
    }

    fn ext_format(ext: &[u8; 3]) -> Option<Format> {
        if *ext == A::EXT {
            Some(A::FORMAT)
        } else {
            None
        }
    }

    fn make(fmt: Format) -> Option<Self::Running> {
        if fmt == A::FORMAT {
            Some(A::make())
        } else {
            None
        }
    }
}

impl<A: FormatDecoder, B: FormatDecoder> DecoderSet for (A, B) {
    type Running = Either<A::Running, B::Running>;

    fn supports(fmt: Format) -> bool {
        fmt == A::FORMAT || fmt == B::FORMAT
    }

    fn ext_format(ext: &[u8; 3]) -> Option<Format> {
        if *ext == A::EXT {
            Some(A::FORMAT)
        } else if *ext == B::EXT {
            Some(B::FORMAT)
        } else {
            None
        }
    }

    fn make(fmt: Format) -> Option<Self::Running> {
        if fmt == A::FORMAT {
            Some(Either::Left(A::make()))
        } else if fmt == B::FORMAT {
            Some(Either::Right(B::make()))
        } else {
            None
        }
    }
}

impl<A: FormatDecoder, B: FormatDecoder, C: FormatDecoder> DecoderSet for (A, B, C) {
    type Running = Either<A::Running, Either<B::Running, C::Running>>;

    fn supports(fmt: Format) -> bool {
        fmt == A::FORMAT || fmt == B::FORMAT || fmt == C::FORMAT
    }

    fn ext_format(ext: &[u8; 3]) -> Option<Format> {
        if *ext == A::EXT {
            Some(A::FORMAT)
        } else if *ext == B::EXT {
            Some(B::FORMAT)
        } else if *ext == C::EXT {
            Some(C::FORMAT)
        } else {
            None
        }
    }

    fn make(fmt: Format) -> Option<Self::Running> {
        if fmt == A::FORMAT {
            Some(Either::Left(A::make()))
        } else if fmt == B::FORMAT {
            Some(Either::Right(Either::Left(B::make())))
        } else if fmt == C::FORMAT {
            Some(Either::Right(Either::Right(C::make())))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use daplink_decoder::DecoderContext;
    use std::vec::Vec;

    struct Rec {
        writes: Vec<(u32, Vec<u8>)>,
    }
    impl DecoderContext for Rec {
        fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
            self.writes.push((addr, data.to_vec()));
            Ok(())
        }
    }

    /// A second descriptor: the same uf2 decoder behind an extension no host uses.
    struct AltFormat;
    impl FormatDecoder for AltFormat {
        type Running = daplink_decoder::Uf2Decoder;
        const FORMAT: Format = Format::Unknown;
        const EXT: [u8; 3] = *b"ALT";
        fn make() -> Self::Running {
            daplink_decoder::Uf2Decoder::new(None)
        }
    }

    fn uf2_sector() -> [u8; 512] {
        let mut b = [0u8; 512];
        b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        b[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        b[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        b[32..36].copy_from_slice(&[1, 2, 3, 4]);
        b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        b
    }

    #[test]
    fn single_member_set_decodes_its_own_format() {
        let mut d = <AllDecoders as DecoderSet>::make(Format::Uf2).unwrap();
        let mut scratch = [0u8; 1024];
        let mut rec = Rec { writes: Vec::new() };
        d.input(&uf2_sector(), &mut scratch, &mut rec).unwrap();
        d.finish(&mut scratch, &mut rec).unwrap();
        assert_eq!(rec.writes[0], (0x0800_0000, std::vec![1, 2, 3, 4]));
    }

    #[test]
    fn tuple2_positional_wiring_and_decode() {
        // Each member maps to its own side of the Either; a non-member has no constructor.
        type Set = (AltFormat, Uf2Format);
        let mut scratch = [0u8; 1024];
        let d = <Set as DecoderSet>::make(Format::Unknown).unwrap();
        assert!(matches!(d, Either::Left(_)));
        let mut d = <Set as DecoderSet>::make(Format::Uf2).unwrap();
        assert!(matches!(d, Either::Right(_)));
        let mut rec = Rec { writes: Vec::new() };
        d.input(&uf2_sector(), &mut scratch, &mut rec).unwrap();
        d.finish(&mut scratch, &mut rec).unwrap();
        assert_eq!(rec.writes[0], (0x0800_0000, std::vec![1, 2, 3, 4]));
    }

    #[test]
    fn tuple3_positional_wiring() {
        type Set = (AltFormat, AltFormat, Uf2Format);
        let d = <Set as DecoderSet>::make(Format::Uf2).unwrap();
        assert!(matches!(d, Either::Right(Either::Right(_))));
    }

    #[test]
    fn ext_format_maps_each_position() {
        type Set = (AltFormat, Uf2Format);
        assert_eq!(
            <Set as DecoderSet>::ext_format(b"ALT"),
            Some(Format::Unknown)
        );
        assert_eq!(<Set as DecoderSet>::ext_format(b"UF2"), Some(Format::Uf2));
        assert_eq!(<Set as DecoderSet>::ext_format(b"TXT"), None);
        // A single-format set only names its own extension.
        assert_eq!(<(Uf2Format,) as DecoderSet>::ext_format(b"ALT"), None);
    }

    #[test]
    fn detect_restricted_to_member_formats() {
        assert_eq!(<(AltFormat,) as DecoderSet>::detect(&uf2_sector()), None);
        assert_eq!(
            <AllDecoders as DecoderSet>::detect(&uf2_sector()),
            Some(Format::Uf2)
        );
        // Content that carries no magic is never detected as anything.
        assert_eq!(<AllDecoders as DecoderSet>::detect(&[0xA5u8; 512]), None);
    }
}
