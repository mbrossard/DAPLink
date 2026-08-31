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

/// The first block's target address and 256-byte payload: magic at 0/4/508, address at 12,
/// payload at 32. A spot check, not a decode.
pub(crate) fn first_block(uf2: &[u8]) -> Option<(u32, &[u8])> {
    let block = uf2.get(..UF2_BLOCK)?;
    let le32 = |off: usize| u32::from_le_bytes(block[off..off + 4].try_into().unwrap());
    if le32(0) != UF2_MAGIC_START0 || le32(4) != UF2_MAGIC_START1 || le32(508) != UF2_MAGIC_END {
        return None;
    }
    Some((le32(12), &block[32..32 + UF2_PAYLOAD]))
}

/// Read back `payload.len()` bytes at `addr` and require a match, plus an RTT boot event when
/// `--dut-elf` allows a symbol-scoped attach.
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

    #[test]
    fn first_block_rejects_a_truncated_file() {
        assert!(first_block(&[0u8; UF2_BLOCK - 1]).is_none());
        assert!(first_block(&[]).is_none());
    }
}
