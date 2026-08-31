//! Platform-independent halves of the msc case: BOT/SCSI framing, FAT16 geometry, write plans.
#![cfg_attr(target_os = "macos", allow(dead_code))]

pub(crate) const SECTOR_SIZE: usize = 512;

pub(crate) const CBW_SIGNATURE: u32 = 0x4342_5355;
pub(crate) const CSW_SIGNATURE: u32 = 0x5342_5355;
pub(crate) const CBW_LEN: usize = 31;
pub(crate) const CSW_LEN: usize = 13;
pub(crate) const CSW_PASSED: u8 = 0;

pub(crate) const OP_READ_10: u8 = 0x28;
pub(crate) const OP_WRITE_10: u8 = 0x2A;

/// The class triple marking an interface as the probe's drag-and-drop volume.
pub(crate) const CLASS_MSC: u8 = 0x08;
pub(crate) const SUBCLASS_SCSI: u8 = 0x06;
pub(crate) const PROTOCOL_BOT: u8 = 0x50;

/// The dropped image's 8.3 name on the volume.
pub(crate) const IMAGE_NAME: [u8; 11] = *b"IMAGE   UF2";
/// `DETAILS.TXT`'s 8.3 name, as `daplink-vfs`'s `files.rs` renders it.
pub(crate) const DETAILS_NAME: [u8; 11] = *b"DETAILS TXT";

/// Whether an interface's class triple marks it as an MSC/SCSI/BOT volume.
pub(crate) fn is_msc_bot_class(class: u8, subclass: u8, protocol: u8) -> bool {
    class == CLASS_MSC && subclass == SUBCLASS_SCSI && protocol == PROTOCOL_BOT
}

/// Sector-at-a-time access, so [`program_image`] drives a USB handle or a test recorder alike.
pub(crate) trait SectorIo {
    fn read_sector(&mut self, lba: u32) -> Result<[u8; SECTOR_SIZE], String>;
    fn write_sector(&mut self, lba: u32, data: &[u8; SECTOR_SIZE]) -> Result<(), String>;
}

/// Build a READ(10)/WRITE(10) CDB: opcode, LBA at `[2..6]`, count at `[7..9]`.
pub(crate) fn rw10_cb(op: u8, lba: u32, count: u16) -> [u8; 10] {
    let mut cb = [0u8; 10];
    cb[0] = op;
    cb[2..6].copy_from_slice(&lba.to_be_bytes());
    cb[7..9].copy_from_slice(&count.to_be_bytes());
    cb
}

/// Build a [`CBW_LEN`]-byte Command Block Wrapper.
pub(crate) fn build_cbw(tag: u32, data_len: u32, dir_in: bool, cb: &[u8]) -> [u8; CBW_LEN] {
    let mut pkt = [0u8; CBW_LEN];
    pkt[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    pkt[4..8].copy_from_slice(&tag.to_le_bytes());
    pkt[8..12].copy_from_slice(&data_len.to_le_bytes());
    pkt[12] = if dir_in { 0x80 } else { 0x00 };
    pkt[13] = 0; // LUN 0, the volume's only unit
    pkt[14] = cb.len() as u8;
    pkt[15..15 + cb.len()].copy_from_slice(cb);
    pkt
}

/// Parse a [`CSW_LEN`]-byte Command Status Wrapper, requiring the signature and echoed tag.
pub(crate) fn parse_csw(bytes: &[u8], expected_tag: u32) -> Result<(u8, u32), String> {
    if bytes.len() != CSW_LEN {
        return Err(format!("CSW length {} != {CSW_LEN}", bytes.len()));
    }
    let sig = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
    if sig != CSW_SIGNATURE {
        return Err(format!("CSW signature {sig:#010x} is wrong"));
    }
    let tag = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
    if tag != expected_tag {
        return Err(format!(
            "CSW tag {tag:#010x} != expected {expected_tag:#010x}"
        ));
    }
    let residue = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
    Ok((bytes[12], residue))
}

/// FAT16 geometry, parsed from the boot sector the volume actually rendered; nothing is
/// hardcoded, since probes differ.
pub(crate) struct BootLayout {
    bytes_per_sector: u16,
    sectors_per_cluster: u8,
    reserved_sectors: u16,
    num_fats: u8,
    root_entries: u16,
    fat_size_sectors: u16,
}

impl BootLayout {
    pub(crate) fn parse(sector: &[u8; SECTOR_SIZE]) -> Result<Self, String> {
        if sector[510] != 0x55 || sector[511] != 0xAA {
            return Err("boot sector missing the 0x55AA signature".into());
        }
        let bytes_per_sector = u16::from_le_bytes([sector[11], sector[12]]);
        let sectors_per_cluster = sector[13];
        let reserved_sectors = u16::from_le_bytes([sector[14], sector[15]]);
        let num_fats = sector[16];
        let root_entries = u16::from_le_bytes([sector[17], sector[18]]);
        let fat_size_sectors = u16::from_le_bytes([sector[22], sector[23]]);
        if bytes_per_sector as usize != SECTOR_SIZE
            || sectors_per_cluster == 0
            || reserved_sectors == 0
            || num_fats == 0
            || root_entries == 0
            || fat_size_sectors == 0
        {
            return Err(format!(
                "implausible FAT16 boot sector geometry: {sector:02x?}"
            ));
        }
        Ok(Self {
            bytes_per_sector,
            sectors_per_cluster,
            reserved_sectors,
            num_fats,
            root_entries,
            fat_size_sectors,
        })
    }

    pub(crate) fn root_dir_lba(&self) -> u32 {
        self.reserved_sectors as u32 + self.num_fats as u32 * self.fat_size_sectors as u32
    }

    pub(crate) fn root_dir_sectors(&self) -> u32 {
        (self.root_entries as u32 * 32).div_ceil(self.bytes_per_sector as u32)
    }

    pub(crate) fn cluster_to_lba(&self, cluster: u32) -> u32 {
        self.root_dir_lba() + self.root_dir_sectors() + (cluster - 2) * self.sectors_per_cluster()
    }

    pub(crate) fn sectors_per_cluster(&self) -> u32 {
        self.sectors_per_cluster as u32
    }

    pub(crate) fn fat_copy_lba(&self, copy: u32) -> u32 {
        self.reserved_sectors as u32 + copy * self.fat_size_sectors as u32
    }

    pub(crate) fn num_fats(&self) -> u32 {
        self.num_fats as u32
    }
}

/// A parsed 32-byte root-directory entry.
pub(crate) struct DirEntry {
    pub(crate) first_cluster: u32,
    pub(crate) size: u32,
}

pub(crate) fn parse_dirent(bytes: &[u8]) -> DirEntry {
    DirEntry {
        first_cluster: u16::from_le_bytes([bytes[26], bytes[27]]) as u32,
        size: u32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]),
    }
}

/// Build a 32-byte root-directory entry.
pub(crate) fn build_dirent(name: &[u8; 11], attr: u8, first_cluster: u32, size: u32) -> [u8; 32] {
    let mut e = [0u8; 32];
    e[0..11].copy_from_slice(name);
    e[11] = attr;
    e[26..28].copy_from_slice(&(first_cluster as u16).to_le_bytes());
    e[28..32].copy_from_slice(&size.to_le_bytes());
    e
}

/// Find `name` among `root`'s slots, stopping at the end-of-directory marker.
pub(crate) fn find_entry(root: &[u8], name: &[u8; 11]) -> Option<DirEntry> {
    for entry in root.chunks(32) {
        if entry.len() < 32 || entry[0] == 0x00 {
            break;
        }
        if entry[0] == 0xE5 || entry[11] & 0x08 != 0 {
            continue;
        }
        if &entry[0..11] == name.as_slice() {
            return Some(parse_dirent(entry));
        }
    }
    None
}

/// The byte offset of the first free (end-of-directory or deleted) 32-byte slot in `root`.
pub(crate) fn find_free_slot(root: &[u8]) -> Option<usize> {
    for (i, entry) in root.chunks(32).enumerate() {
        if entry.len() < 32 {
            break;
        }
        if entry[0] == 0x00 || entry[0] == 0xE5 {
            return Some(i * 32);
        }
    }
    None
}

/// The lowest cluster past every entry already in `root`, so a drop never collides with the
/// generated files however many the firmware carries.
pub(crate) fn next_free_cluster(root: &[u8], layout: &BootLayout) -> u32 {
    let cluster_bytes = layout.sectors_per_cluster() * SECTOR_SIZE as u32;
    let mut end = 2u32;
    for entry in root.chunks(32) {
        if entry.len() < 32 || entry[0] == 0x00 {
            break;
        }
        if entry[0] == 0xE5 || entry[11] & 0x08 != 0 {
            continue;
        }
        let d = parse_dirent(entry);
        if d.first_cluster < 2 {
            continue;
        }
        let n_clusters = d.size.div_ceil(cluster_bytes).max(1);
        end = end.max(d.first_cluster + n_clusters);
    }
    end
}

/// Patch FAT sector `sector_idx` (256 entries per 512-byte FAT16 sector) with what a contiguous
/// chain from `first_cluster` contributes; everything else is left as the caller supplied it.
pub(crate) fn apply_fat_chain(
    sector: &mut [u8; SECTOR_SIZE],
    sector_idx: u32,
    first_cluster: u32,
    n_clusters: u32,
) {
    let base = sector_idx * 256;
    for i in 0..256u32 {
        let cluster = base + i;
        if cluster < first_cluster || cluster >= first_cluster + n_clusters {
            continue;
        }
        let entry: u16 = if cluster == first_cluster + n_clusters - 1 {
            0xFFFF
        } else {
            (cluster + 1) as u16
        };
        let off = (i * 2) as usize;
        sector[off..off + 2].copy_from_slice(&entry.to_le_bytes());
    }
}

/// A well-behaved host's drop order: data sectors ascending, then every FAT copy's chain
/// entries, then the directory entry -- whose landing last is what completes the transfer.
pub(crate) fn program_image<W: SectorIo>(
    w: &mut W,
    layout: &BootLayout,
    first_cluster: u32,
    clusters_needed: u32,
    image: &[u8],
    dirent_lba: u32,
    dirent_sector: &[u8; SECTOR_SIZE],
) -> Result<(), String> {
    let data_lba = layout.cluster_to_lba(first_cluster);
    for (lba, chunk) in (data_lba..).zip(image.chunks(SECTOR_SIZE)) {
        let mut sector = [0u8; SECTOR_SIZE];
        sector[..chunk.len()].copy_from_slice(chunk);
        w.write_sector(lba, &sector)?;
    }

    let first_fat_sector = first_cluster / 256;
    let last_fat_sector = (first_cluster + clusters_needed - 1) / 256;
    for fat_sector_idx in first_fat_sector..=last_fat_sector {
        for copy in 0..layout.num_fats() {
            let fat_lba = layout.fat_copy_lba(copy) + fat_sector_idx;
            let mut sector = w.read_sector(fat_lba)?;
            apply_fat_chain(&mut sector, fat_sector_idx, first_cluster, clusters_needed);
            w.write_sector(fat_lba, &sector)?;
        }
    }

    w.write_sector(dirent_lba, dirent_sector)
}

/// The `Status: ` line's value from a rendered `DETAILS.TXT` body, trimmed of its padding.
pub(crate) fn status_line(body: &[u8]) -> Option<String> {
    let marker = b"Status: ";
    let start = body.windows(marker.len()).position(|w| w == marker)? + marker.len();
    let rest = &body[start..];
    let end = rest
        .windows(2)
        .position(|w| w == b"\r\n")
        .unwrap_or(rest.len());
    Some(String::from_utf8_lossy(&rest[..end]).trim_end().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- CBW/CSW framing ---

    #[test]
    fn cbw_round_trip_fields() {
        let cb = rw10_cb(OP_READ_10, 0x10, 4);
        let pkt = build_cbw(0x1122_3344, 2048, true, &cb);
        assert_eq!(&pkt[0..4], &CBW_SIGNATURE.to_le_bytes());
        assert_eq!(&pkt[4..8], &0x1122_3344u32.to_le_bytes());
        assert_eq!(&pkt[8..12], &2048u32.to_le_bytes());
        assert_eq!(pkt[12], 0x80);
        assert_eq!(pkt[13], 0);
        assert_eq!(pkt[14], 10);
        assert_eq!(&pkt[15..25], &cb);
    }

    #[test]
    fn cbw_out_direction_clears_the_bit() {
        let pkt = build_cbw(1, 512, false, &rw10_cb(OP_WRITE_10, 0, 1));
        assert_eq!(pkt[12], 0x00);
    }

    #[test]
    fn csw_round_trip() {
        let mut bytes = [0u8; CSW_LEN];
        bytes[0..4].copy_from_slice(&CSW_SIGNATURE.to_le_bytes());
        bytes[4..8].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
        bytes[8..12].copy_from_slice(&5u32.to_le_bytes());
        bytes[12] = CSW_PASSED;
        let (status, residue) = parse_csw(&bytes, 0xDEAD_BEEF).expect("valid CSW parses");
        assert_eq!(status, CSW_PASSED);
        assert_eq!(residue, 5);
    }

    #[test]
    fn csw_rejects_wrong_signature() {
        let mut bytes = [0u8; CSW_LEN];
        bytes[0..4].copy_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
        assert!(parse_csw(&bytes, 0).is_err());
    }

    #[test]
    fn csw_rejects_mismatched_tag() {
        let mut bytes = [0u8; CSW_LEN];
        bytes[0..4].copy_from_slice(&CSW_SIGNATURE.to_le_bytes());
        bytes[4..8].copy_from_slice(&1u32.to_le_bytes());
        assert!(parse_csw(&bytes, 2).is_err());
    }

    #[test]
    fn csw_rejects_wrong_length() {
        assert!(parse_csw(&[0u8; CSW_LEN - 1], 0).is_err());
    }

    // --- Skip-detection ---

    #[test]
    fn is_msc_bot_class_requires_the_full_triple() {
        assert!(is_msc_bot_class(0x08, 0x06, 0x50));
        assert!(!is_msc_bot_class(0xFF, 0x06, 0x50)); // the CMSIS-DAP v2 vendor interface
        assert!(!is_msc_bot_class(0x08, 0x05, 0x50)); // wrong subclass
        assert!(!is_msc_bot_class(0x08, 0x06, 0x00)); // wrong protocol (CBI, not BOT)
    }

    #[test]
    fn a_non_flash_descriptor_set_matches_no_interface() {
        // A firmware built without flash support advertises no mass-storage interface at all.
        let interfaces = [(0xFFu8, 0x00u8, 0x00u8)];
        assert!(!interfaces
            .iter()
            .any(|&(c, s, p)| is_msc_bot_class(c, s, p)));
    }

    #[test]
    fn a_flash_descriptor_set_matches_its_msc_interface() {
        let interfaces = [(0xFFu8, 0x00u8, 0x00u8), (0x08u8, 0x06u8, 0x50u8)];
        assert!(interfaces
            .iter()
            .any(|&(c, s, p)| is_msc_bot_class(c, s, p)));
    }

    // --- FAT math from a synthetic boot sector ---

    /// A boot sector rendering the real volume geometry, so the expectations below cross-check
    /// the firmware side.
    fn synthetic_boot_sector() -> [u8; SECTOR_SIZE] {
        let mut b = [0u8; SECTOR_SIZE];
        b[11..13].copy_from_slice(&512u16.to_le_bytes());
        b[13] = 8; // sectors per cluster
        b[14..16].copy_from_slice(&1u16.to_le_bytes()); // reserved sectors
        b[16] = 2; // number of FATs
        b[17..19].copy_from_slice(&32u16.to_le_bytes()); // root entries
        b[22..24].copy_from_slice(&65u16.to_le_bytes()); // FAT size (sectors)
        b[510] = 0x55;
        b[511] = 0xAA;
        b
    }

    #[test]
    fn boot_layout_parses_derived_geometry() {
        let layout = BootLayout::parse(&synthetic_boot_sector()).expect("valid boot sector");
        assert_eq!(layout.root_dir_lba(), 131);
        assert_eq!(layout.root_dir_sectors(), 2);
        assert_eq!(layout.cluster_to_lba(2), 133);
        assert_eq!(layout.cluster_to_lba(3), 141);
        assert_eq!(layout.fat_copy_lba(0), 1);
        assert_eq!(layout.fat_copy_lba(1), 66);
    }

    #[test]
    fn boot_layout_rejects_a_missing_signature() {
        let mut b = synthetic_boot_sector();
        b[511] = 0x00;
        assert!(BootLayout::parse(&b).is_err());
    }

    #[test]
    fn boot_layout_rejects_implausible_geometry() {
        let mut b = synthetic_boot_sector();
        b[13] = 0; // zero sectors per cluster
        assert!(BootLayout::parse(&b).is_err());
    }

    #[test]
    fn apply_fat_chain_writes_a_contiguous_run_terminated_by_eoc() {
        let mut sector = [0u8; SECTOR_SIZE];
        apply_fat_chain(&mut sector, 0, 2, 3);
        assert_eq!(u16::from_le_bytes([sector[4], sector[5]]), 3); // cluster 2 -> 3
        assert_eq!(u16::from_le_bytes([sector[6], sector[7]]), 4); // cluster 3 -> 4
        assert_eq!(u16::from_le_bytes([sector[8], sector[9]]), 0xFFFF); // cluster 4 EOC
        assert_eq!(u16::from_le_bytes([sector[10], sector[11]]), 0x0000); // cluster 5 untouched
    }

    #[test]
    fn apply_fat_chain_ignores_entries_outside_the_sector() {
        // A chain starting at cluster 300 has nothing in FAT sector 0 (entries 0..256).
        let mut sector = [0xAAu8; SECTOR_SIZE];
        apply_fat_chain(&mut sector, 0, 300, 5);
        assert_eq!(sector, [0xAAu8; SECTOR_SIZE]);
    }

    // --- Dirent construction ---

    #[test]
    fn build_dirent_lays_out_name_attr_cluster_and_size() {
        let e = build_dirent(&IMAGE_NAME, 0x20, 10, 4096);
        assert_eq!(&e[0..11], &IMAGE_NAME);
        assert_eq!(e[11], 0x20);
        assert_eq!(u16::from_le_bytes([e[26], e[27]]), 10);
        assert_eq!(u32::from_le_bytes([e[28], e[29], e[30], e[31]]), 4096);
    }

    /// A one-entry root directory carrying `DETAILS.TXT`, as seen on a fresh mount.
    fn root_with_details() -> [u8; SECTOR_SIZE] {
        let mut root = [0u8; SECTOR_SIZE];
        root[0..32].copy_from_slice(&build_dirent(&DETAILS_NAME, 0x21, 2, 512));
        root
    }

    #[test]
    fn find_entry_locates_details_txt() {
        let root = root_with_details();
        let d = find_entry(&root, &DETAILS_NAME).expect("DETAILS.TXT present");
        assert_eq!(d.first_cluster, 2);
        assert_eq!(d.size, 512);
    }

    #[test]
    fn find_entry_misses_an_absent_name() {
        let root = root_with_details();
        assert!(find_entry(&root, &IMAGE_NAME).is_none());
    }

    #[test]
    fn find_free_slot_skips_the_occupied_entry() {
        let root = root_with_details();
        assert_eq!(find_free_slot(&root), Some(32));
    }

    #[test]
    fn next_free_cluster_lands_past_details_txt() {
        let layout = BootLayout::parse(&synthetic_boot_sector()).unwrap();
        let root = root_with_details();
        // DETAILS.TXT (512 bytes) fits one 4096-byte cluster, so cluster 3 is free.
        assert_eq!(next_free_cluster(&root, &layout), 3);
    }

    #[test]
    fn next_free_cluster_is_2_on_an_empty_volume() {
        let layout = BootLayout::parse(&synthetic_boot_sector()).unwrap();
        assert_eq!(next_free_cluster(&[0u8; SECTOR_SIZE], &layout), 2);
    }

    // --- write-plan ordering ---

    /// A sector store that logs every LBA written, so `program_image`'s order is observable.
    struct Recorder {
        disk: std::collections::HashMap<u32, [u8; SECTOR_SIZE]>,
        writes: Vec<u32>,
    }

    impl SectorIo for Recorder {
        fn read_sector(&mut self, lba: u32) -> Result<[u8; SECTOR_SIZE], String> {
            Ok(self.disk.get(&lba).copied().unwrap_or([0u8; SECTOR_SIZE]))
        }
        fn write_sector(&mut self, lba: u32, data: &[u8; SECTOR_SIZE]) -> Result<(), String> {
            self.disk.insert(lba, *data);
            self.writes.push(lba);
            Ok(())
        }
    }

    #[test]
    fn program_image_writes_data_ascending_then_fat_then_dirent_last() {
        let layout = BootLayout::parse(&synthetic_boot_sector()).unwrap();
        let mut rec = Recorder {
            disk: std::collections::HashMap::new(),
            writes: Vec::new(),
        };
        // Three sectors' worth of image, one cluster (8 sectors/cluster covers it).
        let image = vec![0xAAu8; SECTOR_SIZE * 3];
        let first_cluster = 3;
        let clusters_needed = 1;
        let dirent_lba = layout.root_dir_lba();
        let dirent_sector = [0u8; SECTOR_SIZE];

        program_image(
            &mut rec,
            &layout,
            first_cluster,
            clusters_needed,
            &image,
            dirent_lba,
            &dirent_sector,
        )
        .expect("write plan succeeds");

        let data_lba = layout.cluster_to_lba(first_cluster);
        let expected_data: Vec<u32> = (0..3).map(|i| data_lba + i).collect();
        let fat_lba_0 = layout.fat_copy_lba(0) + first_cluster / 256;
        let fat_lba_1 = layout.fat_copy_lba(1) + first_cluster / 256;

        assert_eq!(
            &rec.writes[..3],
            expected_data.as_slice(),
            "data ascending, first"
        );
        assert_eq!(
            &rec.writes[3..5],
            &[fat_lba_0, fat_lba_1],
            "both FAT copies follow the data"
        );
        assert_eq!(*rec.writes.last().unwrap(), dirent_lba, "dirent lands last");
    }

    #[test]
    fn program_image_fat_entries_chain_the_written_clusters() {
        let layout = BootLayout::parse(&synthetic_boot_sector()).unwrap();
        let mut rec = Recorder {
            disk: std::collections::HashMap::new(),
            writes: Vec::new(),
        };
        let image = vec![0xBBu8; SECTOR_SIZE];
        program_image(
            &mut rec,
            &layout,
            3,
            1,
            &image,
            layout.root_dir_lba(),
            &[0u8; SECTOR_SIZE],
        )
        .unwrap();
        let fat_sector = rec.disk[&layout.fat_copy_lba(0)];
        assert_eq!(u16::from_le_bytes([fat_sector[6], fat_sector[7]]), 0xFFFF); // cluster 3 EOC
    }

    // --- status parsing ---

    #[test]
    fn status_line_extracts_a_clean_done_status() {
        let mut body = vec![b' '; 512];
        let text =
            b"...\r\nStatus: Flash: 4096 bytes -> Done                        \r\nFault: \r\n";
        body[..text.len()].copy_from_slice(text);
        assert_eq!(
            status_line(&body).as_deref(),
            Some("Flash: 4096 bytes -> Done")
        );
    }

    #[test]
    fn status_line_absent_is_none() {
        assert_eq!(status_line(&[b' '; 512]), None);
    }
}
