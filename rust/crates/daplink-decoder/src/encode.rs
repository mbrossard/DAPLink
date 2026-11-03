//! Host-side uf2 encoder, the inverse of [`uf2`](crate::uf2).

use crate::uf2::{
    UF2_BLOCK, UF2_DATA_REGION, UF2_FLAG_FAMILY_ID_PRESENT, UF2_MAGIC_END, UF2_MAGIC_START0,
    UF2_MAGIC_START1,
};
use std::vec::Vec;

/// Payload bytes per block; 256 is what the RP2040 ROM bootloader expects.
const UF2_PAYLOAD: usize = 256;

fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

/// Encode `bin` as uf2 at `base`, tagged `family`; the RP2040 ROM needs a zero-padded final block.
pub fn bin_to_uf2(bin: &[u8], base: u32, family: u32) -> Vec<u8> {
    let num_blocks = bin.len().div_ceil(UF2_PAYLOAD) as u32;
    let mut out = Vec::with_capacity(num_blocks as usize * UF2_BLOCK);

    for (i, chunk) in bin.chunks(UF2_PAYLOAD).enumerate() {
        let addr = base + (i as u32) * UF2_PAYLOAD as u32;
        put_u32(&mut out, UF2_MAGIC_START0);
        put_u32(&mut out, UF2_MAGIC_START1);
        put_u32(&mut out, UF2_FLAG_FAMILY_ID_PRESENT);
        put_u32(&mut out, addr);
        put_u32(&mut out, UF2_PAYLOAD as u32);
        put_u32(&mut out, i as u32);
        put_u32(&mut out, num_blocks);
        put_u32(&mut out, family);

        out.extend_from_slice(chunk);
        out.resize(out.len() + (UF2_DATA_REGION - chunk.len()), 0);

        put_u32(&mut out, UF2_MAGIC_END);
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::{Decoder, DecoderContext};
    use crate::error::Error;
    use crate::uf2::Uf2Decoder;

    struct Rebuild {
        base: u32,
        image: Vec<u8>,
    }

    impl Rebuild {
        fn new(base: u32) -> Self {
            Self {
                base,
                image: Vec::new(),
            }
        }
    }

    impl DecoderContext for Rebuild {
        fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
            let off = (addr - self.base) as usize;
            if self.image.len() < off + data.len() {
                self.image.resize(off + data.len(), 0);
            }
            self.image[off..off + data.len()].copy_from_slice(data);
            Ok(())
        }
    }

    #[test]
    fn uf2_empty_input_is_empty() {
        assert_eq!(bin_to_uf2(&[], 0x1000_0000, 0xE48B_FF56), Vec::<u8>::new());
    }

    #[test]
    fn uf2_block_is_512_and_padded() {
        let out = bin_to_uf2(&[0xAB; 10], 0x1000_0000, 0xE48B_FF56);
        assert_eq!(out.len(), 512);
        // payloadSize field is always the full 256, not the 10 real bytes.
        assert_eq!(
            u32::from_le_bytes([out[16], out[17], out[18], out[19]]),
            256
        );
    }

    /// Frozen byte layout of a 2-block uf2 file; hosts parse these fixed offsets.
    #[test]
    fn uf2_golden_vector_two_blocks() {
        const BASE: u32 = 0x1000_0000;
        const FAMILY: u32 = 0xE48B_FF56;
        // One full payload plus a 44-byte tail: two blocks, the second zero-padded.
        let bin: Vec<u8> = (0..300u32).map(|i| i as u8).collect();
        let out = bin_to_uf2(&bin, BASE, FAMILY);
        assert_eq!(out.len(), 2 * UF2_BLOCK);

        let mut expected_block0 = std::vec::Vec::with_capacity(UF2_BLOCK);
        expected_block0.extend_from_slice(&[0x55, 0x46, 0x32, 0x0A]); // UF2_MAGIC_START0
        expected_block0.extend_from_slice(&[0x57, 0x51, 0x5D, 0x9E]); // UF2_MAGIC_START1
        expected_block0.extend_from_slice(&[0x00, 0x20, 0x00, 0x00]); // flags: family id present
        expected_block0.extend_from_slice(&[0x00, 0x00, 0x00, 0x10]); // addr = 0x1000_0000
        expected_block0.extend_from_slice(&[0x00, 0x01, 0x00, 0x00]); // payload size = 256
        expected_block0.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]); // block index 0
        expected_block0.extend_from_slice(&[0x02, 0x00, 0x00, 0x00]); // block count = 2
        expected_block0.extend_from_slice(&[0x56, 0xFF, 0x8B, 0xE4]); // family id
        expected_block0.extend_from_slice(&bin[0..256]);
        expected_block0.resize(expected_block0.len() + (476 - 256), 0);
        expected_block0.extend_from_slice(&[0x30, 0x6F, 0xB1, 0x0A]); // UF2_MAGIC_END
        assert_eq!(expected_block0.len(), UF2_BLOCK);
        assert_eq!(&out[..UF2_BLOCK], &expected_block0[..]);

        let mut expected_block1 = std::vec::Vec::with_capacity(UF2_BLOCK);
        expected_block1.extend_from_slice(&[0x55, 0x46, 0x32, 0x0A]);
        expected_block1.extend_from_slice(&[0x57, 0x51, 0x5D, 0x9E]);
        expected_block1.extend_from_slice(&[0x00, 0x20, 0x00, 0x00]);
        expected_block1.extend_from_slice(&[0x00, 0x01, 0x00, 0x10]); // addr = 0x1000_0100
        expected_block1.extend_from_slice(&[0x00, 0x01, 0x00, 0x00]); // payload size = 256, not 44
        expected_block1.extend_from_slice(&[0x01, 0x00, 0x00, 0x00]); // block index 1
        expected_block1.extend_from_slice(&[0x02, 0x00, 0x00, 0x00]); // block count = 2
        expected_block1.extend_from_slice(&[0x56, 0xFF, 0x8B, 0xE4]);
        expected_block1.extend_from_slice(&bin[256..300]);
        expected_block1.resize(expected_block1.len() + (476 - 44), 0);
        expected_block1.extend_from_slice(&[0x30, 0x6F, 0xB1, 0x0A]);
        assert_eq!(expected_block1.len(), UF2_BLOCK);
        assert_eq!(&out[UF2_BLOCK..], &expected_block1[..]);

        let mut expected = expected_block0;
        expected.extend_from_slice(&expected_block1);
        assert_eq!(out, expected);
    }

    #[test]
    fn uf2_round_trip_through_decoder() {
        let base = 0x1000_0000u32;
        let bin: Vec<u8> = (0..UF2_PAYLOAD * 2 + 37).map(|i| (i * 7) as u8).collect();
        let out = bin_to_uf2(&bin, base, 0xE48B_FF56);

        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; UF2_BLOCK];
        let mut ctx = Rebuild::new(base);
        dec.input(&out, &mut sp, &mut ctx).unwrap();
        dec.finish(&mut sp, &mut ctx).unwrap();

        // The rebuilt image is padded to a block boundary; its prefix must equal the input.
        assert!(ctx.image.len() >= bin.len());
        assert_eq!(&ctx.image[..bin.len()], &bin[..]);
    }
}
