//! uf2 decoder.

use crate::decoder::{Decoder, DecoderContext};
use crate::detect::Format;
use crate::error::Error;
use crate::format::FormatDecoder;

/// Format descriptor for Microsoft uf2 images.
pub struct Uf2Format;

impl FormatDecoder for Uf2Format {
    type Running = Uf2Decoder;
    const FORMAT: Format = Format::Uf2;
    const EXT: [u8; 3] = *b"UF2";

    fn make() -> Uf2Decoder {
        Uf2Decoder::new(None)
    }
}

pub(crate) const UF2_MAGIC_START0: u32 = 0x0A32_4655;
pub(crate) const UF2_MAGIC_START1: u32 = 0x9E5D_5157;
pub(crate) const UF2_MAGIC_END: u32 = 0x0AB1_6F30;
pub(crate) const UF2_FLAG_FAMILY_ID_PRESENT: u32 = 0x0000_2000;
/// "Not main flash" (flag bit 0): the block carries metadata or container data that must NOT be
/// programmed to flash, even though its address and payload look ordinary.
pub(crate) const UF2_FLAG_NOT_MAIN_FLASH: u32 = 0x0000_0001;
pub(crate) const UF2_BLOCK: usize = 512;

/// Read a little-endian `u32` from a 512-byte block at `off` (infallible: `off + 4 <= 512`).
fn le32(block: &[u8; UF2_BLOCK], off: usize) -> u32 {
    u32::from_le_bytes([block[off], block[off + 1], block[off + 2], block[off + 3]])
}

/// Decoder for Microsoft uf2 images, optionally scoped to a family id.
pub struct Uf2Decoder {
    family_id: Option<u32>,
    partial_len: usize,
}

impl Uf2Decoder {
    /// Create a decoder for the given optional `family_id`.
    pub fn new(family_id: Option<u32>) -> Self {
        Self {
            family_id,
            partial_len: 0,
        }
    }

    fn process_block(
        &mut self,
        block: &[u8; UF2_BLOCK],
        context: &mut dyn DecoderContext,
    ) -> Result<(), Error> {
        let magic_start0 = le32(block, 0);
        let magic_start1 = le32(block, 4);
        let flags = le32(block, 8);
        let target_addr = le32(block, 12);
        let payload_size = le32(block, 16);
        let file_size_or_family_id = le32(block, 28);
        let magic_end = le32(block, 508);

        if magic_start0 != UF2_MAGIC_START0
            || magic_start1 != UF2_MAGIC_START1
            || magic_end != UF2_MAGIC_END
        {
            return Err(Error::Uf2);
        }

        // Surface this block's family id (or `None` when the block does not declare one) so a
        // consumer can cross-check it against a target before accepting the payload.
        let block_family = if flags & UF2_FLAG_FAMILY_ID_PRESENT != 0 {
            Some(file_size_or_family_id)
        } else {
            None
        };
        context.set_uf2_family(block_family);

        // A "not main flash" block carries metadata or container data, not flash payload; skip its
        // write entirely (the family id above is still surfaced for cross-checking). Without this a
        // valid UF2 with such blocks would program data explicitly marked non-flash, or fail when
        // its address falls outside the target's accepted range.
        if flags & UF2_FLAG_NOT_MAIN_FLASH != 0 {
            return Ok(());
        }

        if let Some(want) = self.family_id {
            if block_family != Some(want) {
                return Ok(());
            }
        }

        let payload_size = payload_size as usize;
        if payload_size > 476 {
            return Err(Error::Uf2);
        }

        if payload_size > 0 {
            context.write_fragment(target_addr, &block[32..32 + payload_size])?;
        }

        Ok(())
    }
}

impl Decoder for Uf2Decoder {
    fn input(
        &mut self,
        data: &[u8],
        scratchpad: &mut [u8],
        context: &mut dyn DecoderContext,
    ) -> Result<(), Error> {
        if scratchpad.len() < UF2_BLOCK {
            return Err(Error::Internal);
        }

        let mut offset = 0;

        if self.partial_len > 0 {
            let need = UF2_BLOCK - self.partial_len;
            let take = need.min(data.len());
            scratchpad[self.partial_len..self.partial_len + take].copy_from_slice(&data[..take]);
            self.partial_len += take;
            offset += take;

            if self.partial_len == UF2_BLOCK {
                let block: [u8; UF2_BLOCK] = scratchpad[..UF2_BLOCK]
                    .try_into()
                    .expect("slice of exactly UF2_BLOCK bytes");
                self.partial_len = 0;
                self.process_block(&block, context)?;
            } else {
                return Ok(());
            }
        }

        while offset + UF2_BLOCK <= data.len() {
            let block: [u8; UF2_BLOCK] = data[offset..offset + UF2_BLOCK]
                .try_into()
                .expect("slice of exactly UF2_BLOCK bytes");
            self.process_block(&block, context)?;
            offset += UF2_BLOCK;
        }

        let rem = data.len() - offset;
        if rem > 0 {
            scratchpad[..rem].copy_from_slice(&data[offset..]);
            self.partial_len = rem;
        }

        Ok(())
    }

    fn finish(
        &mut self,
        _scratchpad: &mut [u8],
        _context: &mut dyn DecoderContext,
    ) -> Result<(), Error> {
        if self.partial_len > 0 {
            return Err(Error::Uf2);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Rec {
        writes: std::vec::Vec<(u32, std::vec::Vec<u8>)>,
        families: std::vec::Vec<Option<u32>>,
    }

    impl DecoderContext for Rec {
        fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
            self.writes.push((addr, data.to_vec()));
            Ok(())
        }

        fn set_uf2_family(&mut self, family: Option<u32>) {
            self.families.push(family);
        }
    }

    fn block(addr: u32, payload: &[u8], flags: u32, family: u32) -> [u8; 512] {
        let mut b = [0u8; 512];
        b[0..4].copy_from_slice(&UF2_MAGIC_START0.to_le_bytes());
        b[4..8].copy_from_slice(&UF2_MAGIC_START1.to_le_bytes());
        b[8..12].copy_from_slice(&flags.to_le_bytes());
        b[12..16].copy_from_slice(&addr.to_le_bytes());
        b[16..20].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        b[20..24].copy_from_slice(&0u32.to_le_bytes());
        b[24..28].copy_from_slice(&1u32.to_le_bytes());
        b[28..32].copy_from_slice(&family.to_le_bytes());
        b[32..32 + payload.len()].copy_from_slice(payload);
        b[508..512].copy_from_slice(&UF2_MAGIC_END.to_le_bytes());
        b
    }

    #[test]
    fn uf2_single_valid_block() {
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let b = block(0x0800_0000, &[1, 2, 3, 4], 0, 0);
        dec.input(&b, &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.writes.len(), 1);
        assert_eq!(ctx.writes[0].0, 0x0800_0000);
        assert_eq!(ctx.writes[0].1, std::vec![1, 2, 3, 4]);
    }

    #[test]
    fn uf2_not_main_flash_block_is_not_written() {
        // A "not main flash" block with valid magics and a nonzero payload must be skipped: its
        // data is metadata/container, not flash content, so write_fragment is never called.
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let b = block(0x0800_0000, &[1, 2, 3, 4], UF2_FLAG_NOT_MAIN_FLASH, 0);
        dec.input(&b, &mut sp, &mut ctx).unwrap();
        assert!(
            ctx.writes.is_empty(),
            "a not-main-flash block must not be programmed"
        );
    }

    #[test]
    fn uf2_block_split_across_chunks() {
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let b = block(0x0800_1000, &[9, 8, 7], 0, 0);
        dec.input(&b[..300], &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.writes.len(), 0);
        dec.input(&b[300..], &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.writes.len(), 1);
        assert_eq!(ctx.writes[0].0, 0x0800_1000);
        assert_eq!(ctx.writes[0].1, std::vec![9, 8, 7]);
        dec.finish(&mut sp, &mut ctx).unwrap();
    }

    #[test]
    fn uf2_bad_magic_errs() {
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let mut b = block(0x0800_0000, &[1, 2, 3, 4], 0, 0);
        b[0..4].copy_from_slice(&0u32.to_le_bytes());
        let err = dec.input(&b, &mut sp, &mut ctx).unwrap_err();
        assert_eq!(err, Error::Uf2);
    }

    #[test]
    fn uf2_family_id_filter_skips() {
        let mut dec = Uf2Decoder::new(Some(0x1234));
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };

        let b1 = block(0x0800_0000, &[1, 2], UF2_FLAG_FAMILY_ID_PRESENT, 0x9999);
        dec.input(&b1, &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.writes.len(), 0);

        let b2 = block(0x0800_0000, &[1, 2], 0, 0x1234);
        dec.input(&b2, &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.writes.len(), 0);
    }

    #[test]
    fn uf2_family_id_filter_matches_and_programs() {
        // Family present and matching the scoped decoder's id -> the block IS programmed.
        let mut dec = Uf2Decoder::new(Some(0x1234));
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };

        let b = block(0x0800_0000, &[5, 6, 7], UF2_FLAG_FAMILY_ID_PRESENT, 0x1234);
        dec.input(&b, &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.writes.len(), 1);
        assert_eq!(ctx.writes[0].0, 0x0800_0000);
        assert_eq!(ctx.writes[0].1, std::vec![5, 6, 7]);
    }

    #[test]
    fn uf2_set_uf2_family_reports_presence_and_absence() {
        // The family callback is the load-bearing cross-check path for the whole VFS
        // integration: it must fire with `Some(id)` when a block declares a family, and
        // `None` when it does not -- regardless of whether the decoder itself is
        // family-scoped.
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };

        let with_family = block(0x0800_0000, &[1, 2], UF2_FLAG_FAMILY_ID_PRESENT, 0xABCD);
        dec.input(&with_family, &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.families, std::vec![Some(0xABCD)]);

        let without_family = block(0x0800_0200, &[3, 4], 0, 0xABCD);
        dec.input(&without_family, &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.families, std::vec![Some(0xABCD), None]);
    }

    #[test]
    fn uf2_payload_too_large_errs() {
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let mut b = block(0x0800_0000, &[], 0, 0);
        b[16..20].copy_from_slice(&500u32.to_le_bytes());
        let err = dec.input(&b, &mut sp, &mut ctx).unwrap_err();
        assert_eq!(err, Error::Uf2);
    }

    #[test]
    fn uf2_multi_block() {
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let b1 = block(0x0800_0000, &[1, 2], 0, 0);
        let b2 = block(0x0800_0200, &[3, 4], 0, 0);
        let mut data = std::vec::Vec::new();
        data.extend_from_slice(&b1);
        data.extend_from_slice(&b2);
        dec.input(&data, &mut sp, &mut ctx).unwrap();
        assert_eq!(ctx.writes.len(), 2);
        assert_eq!(ctx.writes[0].0, 0x0800_0000);
        assert_eq!(ctx.writes[1].0, 0x0800_0200);
    }

    #[test]
    fn uf2_trailing_partial_finish_errs() {
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let b = block(0x0800_0000, &[1, 2, 3, 4], 0, 0);
        dec.input(&b[..200], &mut sp, &mut ctx).unwrap();
        let err = dec.finish(&mut sp, &mut ctx).unwrap_err();
        assert_eq!(err, Error::Uf2);
    }

    #[test]
    fn uf2_scratchpad_too_small_errs() {
        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 256];
        let mut ctx = Rec {
            writes: std::vec::Vec::new(),
            families: std::vec::Vec::new(),
        };
        let b = block(0x0800_0000, &[1, 2, 3, 4], 0, 0);
        let err = dec.input(&b, &mut sp, &mut ctx).unwrap_err();
        assert_eq!(err, Error::Internal);
    }
}
