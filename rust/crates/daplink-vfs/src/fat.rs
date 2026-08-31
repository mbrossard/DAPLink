//! FAT16 volume geometry and on-disk sector rendering: boot sector, FAT, and cluster mapping.

/// Bytes per logical sector.
pub const SECTOR_SIZE: usize = 512;
/// Logical sectors per allocation cluster.
pub const SECTORS_PER_CLUSTER: u32 = 8;
/// Reserved sectors before the first FAT (the boot sector).
pub const RESERVED_SECTORS: u32 = 1;
/// Number of FAT copies.
pub const NUM_FATS: u32 = 2;
/// Number of 32-byte root-directory entries.
pub const ROOT_ENTRIES: u32 = 32;
/// Total logical sectors advertised by the volume.
pub const TOTAL_SECTORS: u32 = 131_200;
/// Sectors occupied by a single FAT copy.
pub const FAT_SECTORS: u32 = 65;

/// Ceiling division for `u32` values (`b` must be non-zero).
pub(crate) const fn div_ceil_u32(a: u32, b: u32) -> u32 {
    a.div_ceil(b)
}

/// Root-directory sectors (`ROOT_ENTRIES` 32-byte entries rounded up to a sector).
pub const ROOT_DIR_SECTORS: u32 = div_ceil_u32(ROOT_ENTRIES * 32, SECTOR_SIZE as u32);
/// First data-region LBA (after reserved, both FATs, and the root directory).
pub const DATA_START_LBA: u32 = RESERVED_SECTORS + NUM_FATS * FAT_SECTORS + ROOT_DIR_SECTORS;
/// Number of allocatable data clusters.
pub const CLUSTER_COUNT: u32 = (TOTAL_SECTORS - DATA_START_LBA) / SECTORS_PER_CLUSTER;

// Cross-checks: geometry must agree with the foundation constants and FAT16 constraints.
const _: () = assert!(TOTAL_SECTORS == daplink_core::VFS_TOTAL_SECTORS);
const _: () = assert!(SECTOR_SIZE == daplink_core::MSC_SECTOR_SIZE);
const _: () = assert!(div_ceil_u32((CLUSTER_COUNT + 2) * 2, SECTOR_SIZE as u32) == FAT_SECTORS);
const _: () = assert!(CLUSTER_COUNT >= 4085 && CLUSTER_COUNT < 65525);

/// The first LBA of cluster `n` (`n >= 2`).
pub const fn cluster_to_lba(n: u32) -> u32 {
    DATA_START_LBA + (n - 2) * SECTORS_PER_CLUSTER
}

/// Constant boot-sector header, offsets `0..62`: every field except the volume id (`39..43`),
/// the volume label (`43..54`) and the boot signature (`510..512`), which are patched at runtime.
const BOOT_HEADER: [u8; 62] = {
    let mut h = [0u8; 62];
    // Jump instruction.
    h[0] = 0xEB;
    h[1] = 0x3C;
    h[2] = 0x90;
    // OEM name.
    let oem = b"MSDOS5.0";
    let mut i = 0;
    while i < oem.len() {
        h[3 + i] = oem[i];
        i += 1;
    }
    // Bytes per sector.
    let bps = (SECTOR_SIZE as u16).to_le_bytes();
    h[11] = bps[0];
    h[12] = bps[1];
    // Sectors per cluster.
    h[13] = SECTORS_PER_CLUSTER as u8;
    // Reserved sectors.
    let rs = (RESERVED_SECTORS as u16).to_le_bytes();
    h[14] = rs[0];
    h[15] = rs[1];
    // Number of FATs.
    h[16] = NUM_FATS as u8;
    // Root-directory entries.
    let re = (ROOT_ENTRIES as u16).to_le_bytes();
    h[17] = re[0];
    h[18] = re[1];
    // Total sectors (16-bit) stays zero: the 32-bit field below carries it.
    // Media descriptor (fixed disk).
    h[21] = 0xF8;
    // FAT size (16-bit), in sectors.
    let fs = (FAT_SECTORS as u16).to_le_bytes();
    h[22] = fs[0];
    h[23] = fs[1];
    // Sectors per track / number of heads (arbitrary but plausible).
    let spt = 63u16.to_le_bytes();
    h[24] = spt[0];
    h[25] = spt[1];
    let heads = 255u16.to_le_bytes();
    h[26] = heads[0];
    h[27] = heads[1];
    // Total sectors (32-bit).
    let ts = TOTAL_SECTORS.to_le_bytes();
    h[32] = ts[0];
    h[33] = ts[1];
    h[34] = ts[2];
    h[35] = ts[3];
    // Physical drive number.
    h[36] = 0x80;
    // Extended boot signature.
    h[38] = 0x29;
    // Volume id (39..43) and label (43..54) stay zeroed: `boot_sector` patches them per probe.
    // File-system type.
    let fst = b"FAT16   ";
    let mut j = 0;
    while j < fst.len() {
        h[54 + j] = fst[j];
        j += 1;
    }
    h
};

/// The volume id a probe uses when it has no per-probe one to offer: "DAPL" as a little-endian
/// word. Two probes sharing it present two volumes hosts may confuse, so prefer a per-probe id.
pub const DEFAULT_VOLUME_ID: u32 = 0x4441_504C;

/// Render the FAT16 boot sector into `buf`, patching the 11-byte `label` and 32-bit `volume_id`
/// over the constant header.
pub fn boot_sector(buf: &mut [u8; 512], label: &[u8; 11], volume_id: u32) {
    *buf = [0u8; 512];
    buf[0..62].copy_from_slice(&BOOT_HEADER);
    // Volume id.
    buf[39..43].copy_from_slice(&volume_id.to_le_bytes());
    // Volume label.
    buf[43..54].copy_from_slice(label);
    // Boot signature.
    buf[510] = 0x55;
    buf[511] = 0xAA;
}

/// The FAT entry for `cluster` given the allocated `chains`. Each `(first, n)` is a contiguous
/// run: cluster `k` points to `k + 1`, the run's last cluster is `0xFFFF`, and the rest is free.
fn fat_entry(cluster: u32, chains: &[(u32, u32)]) -> u16 {
    for &(first, n) in chains {
        if cluster >= first && cluster < first + n {
            if cluster == first + n - 1 {
                return 0xFFFF;
            }
            return (cluster + 1) as u16;
        }
    }
    0x0000
}

/// Render FAT sector `idx` (0-based within a single FAT copy) into `buf`.
pub fn fat_sector(buf: &mut [u8; 512], idx: u32, chains: &[(u32, u32)]) {
    *buf = [0u8; 512];
    for i in 0..256usize {
        let cluster = idx * 256 + i as u32;
        let entry = match cluster {
            0 => 0xFFF8,
            1 => 0xFFFF,
            _ => fat_entry(cluster, chains),
        };
        buf[2 * i..2 * i + 2].copy_from_slice(&entry.to_le_bytes());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derived_geometry() {
        assert_eq!(ROOT_DIR_SECTORS, 2);
        assert_eq!(DATA_START_LBA, 133);
        assert_eq!(CLUSTER_COUNT, 16383);
        assert_eq!(cluster_to_lba(2), 133);
        assert_eq!(cluster_to_lba(3), 141);
    }

    /// frozen: a host's FAT16 driver parses these exact fields at these exact offsets to mount.
    #[test]
    fn boot_sector_exact_bytes() {
        let mut b = [0u8; 512];
        boot_sector(&mut b, b"DAPLINK    ", DEFAULT_VOLUME_ID);
        assert_eq!(&b[0..3], &[0xEB, 0x3C, 0x90]);
        assert_eq!(&b[3..11], b"MSDOS5.0");
        assert_eq!(u16::from_le_bytes([b[11], b[12]]), 512);
        assert_eq!(b[13], 8);
        assert_eq!(u16::from_le_bytes([b[14], b[15]]), 1);
        assert_eq!(b[16], 2);
        assert_eq!(u16::from_le_bytes([b[17], b[18]]), 32);
        assert_eq!(u16::from_le_bytes([b[19], b[20]]), 0);
        assert_eq!(b[21], 0xF8);
        assert_eq!(u16::from_le_bytes([b[22], b[23]]), 65);
        assert_eq!(u32::from_le_bytes([b[32], b[33], b[34], b[35]]), 131_200);
        assert_eq!(b[36], 0x80);
        assert_eq!(b[38], 0x29);
        assert_eq!(
            u32::from_le_bytes([b[39], b[40], b[41], b[42]]),
            DEFAULT_VOLUME_ID
        );
        assert_eq!(&b[43..54], b"DAPLINK    ");
        assert_eq!(&b[54..62], b"FAT16   ");
        assert_eq!(b[510], 0x55);
        assert_eq!(b[511], 0xAA);
    }

    /// frozen: bytes `62..510` are unused and must stay zero, not stray bytes of a future field.
    #[test]
    fn boot_sector_padding_between_header_and_signature_is_zero() {
        let mut b = [0u8; 512];
        boot_sector(&mut b, b"DAPLINK    ", DEFAULT_VOLUME_ID);
        assert!(
            b[62..510].iter().all(|&byte| byte == 0),
            "boot sector padding must be all zero"
        );
    }

    /// The volume id is patched per probe over the constant header, exactly as the label is.
    #[test]
    fn boot_sector_carries_the_per_probe_volume_id() {
        let mut b = [0u8; 512];
        boot_sector(&mut b, b"DAPLINK    ", 0x1BCA_0157);
        assert_eq!(
            u32::from_le_bytes([b[39], b[40], b[41], b[42]]),
            0x1BCA_0157
        );
        // Neither neighbour is disturbed: the boot signature before, the label after.
        assert_eq!(b[38], 0x29);
        assert_eq!(&b[43..54], b"DAPLINK    ");
    }

    #[test]
    fn fat_reserved_entries() {
        let mut b = [0u8; 512];
        fat_sector(&mut b, 0, &[]);
        assert_eq!(u16::from_le_bytes([b[0], b[1]]), 0xFFF8);
        assert_eq!(u16::from_le_bytes([b[2], b[3]]), 0xFFFF);
        // Cluster 2 is free with no chains.
        assert_eq!(u16::from_le_bytes([b[4], b[5]]), 0x0000);
    }

    #[test]
    fn fat_single_chain_and_free_space() {
        // A file occupying clusters 2..=4 (3 clusters).
        let mut b = [0u8; 512];
        fat_sector(&mut b, 0, &[(2, 3)]);
        assert_eq!(u16::from_le_bytes([b[4], b[5]]), 3); // cluster 2 -> 3
        assert_eq!(u16::from_le_bytes([b[6], b[7]]), 4); // cluster 3 -> 4
        assert_eq!(u16::from_le_bytes([b[8], b[9]]), 0xFFFF); // cluster 4 -> EOC
        assert_eq!(u16::from_le_bytes([b[10], b[11]]), 0x0000); // cluster 5 free
    }

    #[test]
    fn fat_two_disjoint_chains() {
        let mut b = [0u8; 512];
        fat_sector(&mut b, 0, &[(2, 2), (4, 1)]);
        assert_eq!(u16::from_le_bytes([b[4], b[5]]), 3); // 2 -> 3
        assert_eq!(u16::from_le_bytes([b[6], b[7]]), 0xFFFF); // 3 EOC
        assert_eq!(u16::from_le_bytes([b[8], b[9]]), 0xFFFF); // 4 single-cluster EOC
    }
}
