//! Shared UF2 parsing and readback verification for the drop-style reflash cases.

use daplink_test_protocol as proto;
use probe_rs::MemoryInterface;

use crate::util::chain;
use crate::{Ctx, Failure};

pub(crate) const UF2_MAGIC_START0: u32 = 0x0A32_4655;
pub(crate) const UF2_MAGIC_START1: u32 = 0x9E5D_5157;
pub(crate) const UF2_MAGIC_END: u32 = 0x0AB1_6F30;
pub(crate) const UF2_BLOCK: usize = 512;
pub(crate) const UF2_PAYLOAD: usize = 256;

/// The first block's target address and 256-byte payload; a spot check, not a decode.
pub(crate) fn first_block(uf2: &[u8]) -> Option<(u32, &[u8])> {
    let block = uf2.get(..UF2_BLOCK)?;
    let le32 = |off: usize| u32::from_le_bytes(block[off..off + 4].try_into().unwrap());
    if le32(0) != UF2_MAGIC_START0 || le32(4) != UF2_MAGIC_START1 || le32(508) != UF2_MAGIC_END {
        return None;
    }
    Some((le32(12), &block[32..32 + UF2_PAYLOAD]))
}

/// The widest span an image may claim before its readback is refused as implausible.
const MAX_SPAN: u64 = 1 << 20;

/// The address range every block of `uf2` writes, as `(start, len)`; `None` if malformed or wider than [`MAX_SPAN`].
pub(crate) fn image_span(uf2: &[u8]) -> Option<(u32, usize)> {
    let (mut lo, mut hi) = (u64::MAX, 0u64);
    for block in uf2.chunks(UF2_BLOCK) {
        let (addr, payload) = first_block(block)?;
        lo = lo.min(u64::from(addr));
        hi = hi.max(u64::from(addr) + payload.len() as u64);
    }
    if lo == u64::MAX || hi.checked_sub(lo)? > MAX_SPAN {
        return None;
    }
    Some((lo as u32, (hi - lo) as usize))
}

/// Replace block `index`'s start magic, leaving every other block intact; `None` if there is no such block.
pub(crate) fn corrupt_block(uf2: &[u8], index: usize) -> Option<Vec<u8>> {
    if uf2.is_empty() || !uf2.len().is_multiple_of(UF2_BLOCK) || index >= uf2.len() / UF2_BLOCK {
        return None;
    }
    let mut out = uf2.to_vec();
    let at = index * UF2_BLOCK;
    out[at..at + 4].copy_from_slice(&(!UF2_MAGIC_START0).to_le_bytes());
    Some(out)
}

/// Block `index`'s target address, read whatever its magic says.
pub(crate) fn block_addr(uf2: &[u8], index: usize) -> Option<u32> {
    let at = index.checked_mul(UF2_BLOCK)? + 12;
    Some(u32::from_le_bytes(uf2.get(at..at + 4)?.try_into().ok()?))
}

/// The block the corruption case damages: neither the first nor the last.
pub(crate) fn middle_block(uf2: &[u8]) -> Option<usize> {
    let blocks = uf2.len() / UF2_BLOCK;
    (blocks >= 2).then_some(blocks / 2)
}

/// Read back `payload.len()` bytes at `addr`, plus an RTT boot event when `--dut-elf` allows.
pub(crate) fn verify(cx: &mut Ctx, addr: u32, payload: &[u8]) -> Result<(), Failure> {
    let dut_elf = cx.dut_elf.clone();
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("reattach lost the session".into()))?;
    let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;

    let mut back = vec![0u8; payload.len()];
    core.read_8(u64::from(addr), &mut back)
        .map_err(|e| Failure::Failed(format!("readback at {addr:#010x}: {}", chain(e))))?;
    if back != payload {
        return Err(Failure::Failed(format!(
            "readback at {addr:#010x} does not match the flashed image"
        )));
    }

    if let Some(elf) = dut_elf {
        let mut rtt = crate::rtt::attach(&mut core, Some(&elf)).map_err(Failure::Failed)?;
        crate::rtt::expect(&mut rtt, &mut core, proto::EVT_BOOT)
            .map_err(|e| Failure::Failed(format!("boot event: {e}")))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal one-block uf2 image, matching `daplink_decoder::encode::bin_to_uf2`'s layout.
    fn one_block_uf2(addr: u32, payload: &[u8; UF2_PAYLOAD]) -> Vec<u8> {
        let mut block = vec![0u8; UF2_BLOCK];
        block[0..4].copy_from_slice(&UF2_MAGIC_START0.to_le_bytes());
        block[4..8].copy_from_slice(&UF2_MAGIC_START1.to_le_bytes());
        block[8..12].copy_from_slice(&0u32.to_le_bytes()); // flags
        block[12..16].copy_from_slice(&addr.to_le_bytes());
        block[16..20].copy_from_slice(&(UF2_PAYLOAD as u32).to_le_bytes());
        block[32..32 + UF2_PAYLOAD].copy_from_slice(payload);
        block[508..512].copy_from_slice(&UF2_MAGIC_END.to_le_bytes());
        block
    }

    #[test]
    fn first_block_reads_address_and_payload() {
        let payload = [0x5Au8; UF2_PAYLOAD];
        let uf2 = one_block_uf2(0x0800_0000, &payload);
        let (addr, data) = first_block(&uf2).expect("valid block parses");
        assert_eq!(addr, 0x0800_0000);
        assert_eq!(data, &payload[..]);
    }

    #[test]
    fn first_block_rejects_bad_magic() {
        let mut uf2 = one_block_uf2(0x0800_0000, &[0u8; UF2_PAYLOAD]);
        uf2[0] = 0x00;
        assert!(first_block(&uf2).is_none());
    }

    /// The span covers every block, not just the first.
    #[test]
    fn image_span_covers_every_block() {
        let mut image = one_block_uf2(0x0800_0000, &[0x11; UF2_PAYLOAD]);
        image.extend_from_slice(&one_block_uf2(0x0800_0100, &[0x22; UF2_PAYLOAD]));
        image.extend_from_slice(&one_block_uf2(0x0800_0200, &[0x33; UF2_PAYLOAD]));
        assert_eq!(image_span(&image), Some((0x0800_0000, 3 * UF2_PAYLOAD)));
    }

    /// Blocks need not arrive in address order, and the span is still the whole range.
    #[test]
    fn image_span_does_not_assume_ascending_blocks() {
        let mut image = one_block_uf2(0x0800_0200, &[0u8; UF2_PAYLOAD]);
        image.extend_from_slice(&one_block_uf2(0x0800_0000, &[0u8; UF2_PAYLOAD]));
        assert_eq!(image_span(&image), Some((0x0800_0000, 3 * UF2_PAYLOAD)));
    }

    /// A malformed or wildly addressed image is refused rather than turned into a huge read.
    #[test]
    fn image_span_refuses_what_it_cannot_bound() {
        let mut image = one_block_uf2(0x0800_0000, &[0u8; UF2_PAYLOAD]);
        image.extend_from_slice(&one_block_uf2(0x9000_0000, &[0u8; UF2_PAYLOAD]));
        assert_eq!(image_span(&image), None);
        assert_eq!(image_span(&[]), None);
        let bad = corrupt_block(&one_block_uf2(0, &[0u8; UF2_PAYLOAD]), 0).unwrap();
        assert_eq!(image_span(&bad), None);
    }

    /// The corrupted copy differs from the good one only in the named block's magic.
    #[test]
    fn corrupt_block_damages_only_the_named_block() {
        let mut image = one_block_uf2(0x0800_0000, &[0x11; UF2_PAYLOAD]);
        image.extend_from_slice(&one_block_uf2(0x0800_0100, &[0x22; UF2_PAYLOAD]));
        let bad = corrupt_block(&image, 1).expect("block 1 exists");
        assert_eq!(bad.len(), image.len());
        assert_eq!(bad[..UF2_BLOCK], image[..UF2_BLOCK]);
        assert_ne!(
            bad[UF2_BLOCK..UF2_BLOCK + 4],
            image[UF2_BLOCK..UF2_BLOCK + 4]
        );
        assert_eq!(bad[UF2_BLOCK + 4..], image[UF2_BLOCK + 4..]);
        // The first block still parses, so the readback check still knows what to expect.
        assert!(first_block(&bad).is_some());
    }

    #[test]
    fn corrupt_block_refuses_what_it_cannot_corrupt() {
        let image = one_block_uf2(0x0800_0000, &[0u8; UF2_PAYLOAD]);
        assert!(corrupt_block(&image, 1).is_none());
        assert!(corrupt_block(&[], 0).is_none());
        assert!(corrupt_block(&image[..UF2_BLOCK - 1], 0).is_none());
    }

    /// A damaged block still names its address.
    #[test]
    fn block_addr_reads_a_corrupted_block() {
        let mut image = one_block_uf2(0x0800_0000, &[0u8; UF2_PAYLOAD]);
        image.extend_from_slice(&one_block_uf2(0x0800_0100, &[0u8; UF2_PAYLOAD]));
        let bad = corrupt_block(&image, 1).unwrap();
        assert_eq!(block_addr(&bad, 1), Some(0x0800_0100));
        assert_eq!(block_addr(&bad, 2), None);
    }

    /// A one-block image has no middle, so the case skips.
    #[test]
    fn middle_block_needs_more_than_one_block() {
        let one = one_block_uf2(0, &[0u8; UF2_PAYLOAD]);
        assert_eq!(middle_block(&one), None);
        let mut four = Vec::new();
        for _ in 0..4 {
            four.extend_from_slice(&one);
        }
        assert_eq!(middle_block(&four), Some(2));
    }

    #[test]
    fn first_block_rejects_a_truncated_file() {
        assert!(first_block(&[0u8; UF2_BLOCK - 1]).is_none());
        assert!(first_block(&[]).is_none());
    }
}
