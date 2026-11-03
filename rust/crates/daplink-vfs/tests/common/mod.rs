//! Sector builders shared by the unit and integration tests.
#![allow(dead_code)]

/// A root-directory entry: 8.3 `name`, attributes, first cluster and size.
pub fn dir_entry(name: &[u8; 11], attr: u8, first_cluster: u16, size: u32) -> [u8; 32] {
    let mut e = [0u8; 32];
    e[0..11].copy_from_slice(name);
    e[11] = attr;
    e[26..28].copy_from_slice(&first_cluster.to_le_bytes());
    e[28..32].copy_from_slice(&size.to_le_bytes());
    e
}

/// An unnumbered UF2 block without a family, carrying `payload` at `addr`.
pub fn uf2_block(addr: u32, payload: &[u8]) -> [u8; 512] {
    uf2_numbered(0, 0, addr, payload)
}

/// UF2 block `block_no` of `num_blocks`, without a family, carrying `payload` at `addr`.
pub fn uf2_numbered(block_no: u32, num_blocks: u32, addr: u32, payload: &[u8]) -> [u8; 512] {
    let mut b = [0u8; 512];
    b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
    b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
    b[12..16].copy_from_slice(&addr.to_le_bytes());
    b[16..20].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    b[20..24].copy_from_slice(&block_no.to_le_bytes());
    b[24..28].copy_from_slice(&num_blocks.to_le_bytes());
    b[32..32 + payload.len()].copy_from_slice(payload);
    b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
    b
}
