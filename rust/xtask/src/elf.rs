//! Minimal ELF32 reader: enough to pull the flash image out of a built Cortex-M binary.

const PT_LOAD: u32 = 1;

/// Read a little-endian `u16`/`u32` at `off`, or `None` if that would run past `data`.
fn le16(data: &[u8], off: usize) -> Option<u16> {
    data.get(off..off + 2)
        .map(|b| u16::from_le_bytes([b[0], b[1]]))
}

fn le32(data: &[u8], off: usize) -> Option<u32> {
    data.get(off..off + 4)
        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// Reject anything but a 32-bit little-endian ELF, up front, for both the program-header and
/// section-header readers below.
fn check_header(elf: &[u8]) -> Result<(), String> {
    if elf.len() < 52 || &elf[0..4] != b"\x7fELF" {
        return Err("not an ELF file".into());
    }
    if elf[4] != 1 {
        return Err("not a 32-bit ELF file".into());
    }
    if elf[5] != 1 {
        return Err("not a little-endian ELF file".into());
    }
    Ok(())
}

/// `sh_type` for a section with no file-backed bytes -- true of every `.uninit`/`.bss`-style
/// section, which by construction can only back RAM, never flash.
const SHT_NOBITS: u32 = 8;
/// `sh_flags` bit: the section occupies memory during execution.
const SHF_ALLOC: u32 = 0x2;

/// The fields of one ELF32 section header this reader cares about.
struct Section {
    sh_type: u32,
    sh_flags: u32,
    sh_addr: u32,
    sh_size: u32,
}

/// Find a section by name in the ELF's section-header table. `None` when the table is absent
/// (stripped, or the linker emitted none) or no section carries that name.
///
/// # Errors
/// A malformed header, section-header table, or section name string table.
fn find_section(elf: &[u8], name: &str) -> Result<Option<Section>, String> {
    check_header(elf)?;

    let shoff = le32(elf, 32).ok_or("truncated ELF header (e_shoff)")? as usize;
    let shentsize = le16(elf, 46).ok_or("truncated ELF header (e_shentsize)")? as usize;
    let shnum = le16(elf, 48).ok_or("truncated ELF header (e_shnum)")? as usize;
    let shstrndx = le16(elf, 50).ok_or("truncated ELF header (e_shstrndx)")? as usize;
    if shoff == 0 || shnum == 0 {
        return Ok(None);
    }
    if shentsize < 40 {
        return Err(format!("implausible ELF32 shentsize {shentsize}"));
    }
    if shstrndx >= shnum {
        return Err(format!(
            "e_shstrndx {shstrndx} outside the {shnum}-entry section table"
        ));
    }

    let strtab_sh = shoff + shstrndx * shentsize;
    let strtab_off =
        le32(elf, strtab_sh + 16).ok_or("truncated section header (sh_offset)")? as usize;
    let strtab_size =
        le32(elf, strtab_sh + 20).ok_or("truncated section header (sh_size)")? as usize;
    let strtab = elf
        .get(strtab_off..strtab_off + strtab_size)
        .ok_or("section name string table runs past end of ELF")?;

    for i in 0..shnum {
        let sh = shoff + i * shentsize;
        let sh_name = le32(elf, sh).ok_or("truncated section header (sh_name)")? as usize;
        let this_name = strtab
            .get(sh_name..)
            .and_then(|rest| rest.split(|&b| b == 0).next())
            .ok_or("sh_name offset runs past the string table")?;
        if this_name != name.as_bytes() {
            continue;
        }
        return Ok(Some(Section {
            sh_type: le32(elf, sh + 4).ok_or("truncated section header (sh_type)")?,
            sh_flags: le32(elf, sh + 8).ok_or("truncated section header (sh_flags)")?,
            sh_addr: le32(elf, sh + 12).ok_or("truncated section header (sh_addr)")?,
            sh_size: le32(elf, sh + 20).ok_or("truncated section header (sh_size)")?,
        }));
    }
    Ok(None)
}

/// The `.uninit` section's `(address, size)` when it is allocated, `SHT_NOBITS` and non-empty;
/// `None` for any other shape, there being nothing to zero either way.
pub fn uninit_range(elf: &[u8]) -> Result<Option<(u32, u32)>, String> {
    let Some(section) = find_section(elf, ".uninit")? else {
        return Ok(None);
    };
    if section.sh_type != SHT_NOBITS || section.sh_flags & SHF_ALLOC == 0 || section.sh_size == 0 {
        return Ok(None);
    }
    Ok(Some((section.sh_addr, section.sh_size)))
}

/// Flatten the `PT_LOAD` segments into `(base_address, image)` by physical address, zero-filling
/// gaps and BSS tails; segments may appear in any order, and an overlap is rejected.
pub fn flash_image(elf: &[u8]) -> Result<(u32, Vec<u8>), String> {
    check_header(elf)?;

    let phoff = le32(elf, 28).ok_or("truncated ELF header (e_phoff)")? as usize;
    let phentsize = le16(elf, 42).ok_or("truncated ELF header (e_phentsize)")? as usize;
    let phnum = le16(elf, 44).ok_or("truncated ELF header (e_phnum)")? as usize;
    if phentsize < 32 {
        return Err(format!("implausible ELF32 phentsize {phentsize}"));
    }

    // Collect every segment first, so the image is sized once its full span is known.
    let mut segments: Vec<(u32, &[u8], u32)> = Vec::new();
    for i in 0..phnum {
        let ph = phoff + i * phentsize;
        let p_type = le32(elf, ph).ok_or("truncated program header (p_type)")?;
        if p_type != PT_LOAD {
            continue;
        }
        let p_offset = le32(elf, ph + 4).ok_or("truncated program header (p_offset)")? as usize;
        let p_paddr = le32(elf, ph + 12).ok_or("truncated program header (p_paddr)")?;
        let p_filesz = le32(elf, ph + 16).ok_or("truncated program header (p_filesz)")? as usize;
        let p_memsz = le32(elf, ph + 20).ok_or("truncated program header (p_memsz)")?;
        if p_filesz == 0 {
            continue;
        }
        let bytes = elf
            .get(p_offset..p_offset + p_filesz)
            .ok_or("PT_LOAD segment file range runs past end of ELF")?;
        let span = p_memsz.max(p_filesz as u32);
        segments.push((p_paddr, bytes, span));
    }

    if segments.is_empty() {
        return Err("no PT_LOAD segments with file content".into());
    }

    let base = segments.iter().map(|(addr, _, _)| *addr).min().unwrap();
    let end = segments
        .iter()
        .map(|(addr, _, span)| addr + span)
        .max()
        .unwrap();

    // Sorted by address, so overlap is an adjacent-pair check over each segment's full span.
    let mut by_addr = segments.clone();
    by_addr.sort_by_key(|(addr, _, _)| *addr);
    for pair in by_addr.windows(2) {
        let (addr_a, _, span_a) = pair[0];
        let (addr_b, _, _) = pair[1];
        if addr_a + span_a > addr_b {
            return Err(format!(
                "overlapping PT_LOAD segments at {addr_a:#010x} (span {span_a:#x}) and {addr_b:#010x}"
            ));
        }
    }

    let mut image = vec![0u8; (end - base) as usize];
    for (addr, bytes, _) in segments {
        let off = (addr - base) as usize;
        image[off..off + bytes.len()].copy_from_slice(bytes);
    }

    Ok((base, image))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal well-formed ELF32 LE with the given `PT_LOAD` segments
    /// (`paddr`, `bytes`), laid out contiguously after a single program header table.
    fn make_elf(segments: &[(u32, &[u8])]) -> Vec<u8> {
        let ehsize = 52;
        let phentsize = 32;
        let phnum = segments.len();
        let phoff = ehsize;
        let mut data_off = phoff + phnum * phentsize;

        let mut out = vec![0u8; data_off];
        out[0..4].copy_from_slice(b"\x7fELF");
        out[4] = 1; // ELFCLASS32
        out[5] = 1; // ELFDATA2LSB
        out[6] = 1; // EI_VERSION
        out[16..18].copy_from_slice(&2u16.to_le_bytes()); // e_type = ET_EXEC
        out[18..20].copy_from_slice(&40u16.to_le_bytes()); // e_machine = EM_ARM
        out[28..32].copy_from_slice(&(phoff as u32).to_le_bytes());
        out[40..42].copy_from_slice(&(ehsize as u16).to_le_bytes());
        out[42..44].copy_from_slice(&(phentsize as u16).to_le_bytes());
        out[44..46].copy_from_slice(&(phnum as u16).to_le_bytes());

        for (i, (paddr, bytes)) in segments.iter().enumerate() {
            let ph = phoff + i * phentsize;
            out[ph..ph + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
            out[ph + 4..ph + 8].copy_from_slice(&(data_off as u32).to_le_bytes()); // p_offset
            out[ph + 8..ph + 12].copy_from_slice(&paddr.to_le_bytes()); // p_vaddr
            out[ph + 12..ph + 16].copy_from_slice(&paddr.to_le_bytes()); // p_paddr
            out[ph + 16..ph + 20].copy_from_slice(&(bytes.len() as u32).to_le_bytes()); // p_filesz
            out[ph + 20..ph + 24].copy_from_slice(&(bytes.len() as u32).to_le_bytes()); // p_memsz

            out.extend_from_slice(bytes);
            data_off += bytes.len();
        }

        out
    }

    /// Like [`make_elf`], but each segment also names its own `p_memsz` (`paddr`, `bytes`,
    /// `memsz`), so a BSS tail (`memsz > bytes.len()`) can be constructed.
    fn make_elf_with_memsz(segments: &[(u32, &[u8], u32)]) -> Vec<u8> {
        let ehsize = 52;
        let phentsize = 32;
        let phnum = segments.len();
        let phoff = ehsize;
        let mut data_off = phoff + phnum * phentsize;

        let mut out = vec![0u8; data_off];
        out[0..4].copy_from_slice(b"\x7fELF");
        out[4] = 1; // ELFCLASS32
        out[5] = 1; // ELFDATA2LSB
        out[6] = 1; // EI_VERSION
        out[16..18].copy_from_slice(&2u16.to_le_bytes()); // e_type = ET_EXEC
        out[18..20].copy_from_slice(&40u16.to_le_bytes()); // e_machine = EM_ARM
        out[28..32].copy_from_slice(&(phoff as u32).to_le_bytes());
        out[40..42].copy_from_slice(&(ehsize as u16).to_le_bytes());
        out[42..44].copy_from_slice(&(phentsize as u16).to_le_bytes());
        out[44..46].copy_from_slice(&(phnum as u16).to_le_bytes());

        for (i, (paddr, bytes, memsz)) in segments.iter().enumerate() {
            let ph = phoff + i * phentsize;
            out[ph..ph + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
            out[ph + 4..ph + 8].copy_from_slice(&(data_off as u32).to_le_bytes()); // p_offset
            out[ph + 8..ph + 12].copy_from_slice(&paddr.to_le_bytes()); // p_vaddr
            out[ph + 12..ph + 16].copy_from_slice(&paddr.to_le_bytes()); // p_paddr
            out[ph + 16..ph + 20].copy_from_slice(&(bytes.len() as u32).to_le_bytes()); // p_filesz
            out[ph + 20..ph + 24].copy_from_slice(&memsz.to_le_bytes()); // p_memsz

            out.extend_from_slice(bytes);
            data_off += bytes.len();
        }

        out
    }

    #[test]
    fn single_segment_round_trips() {
        let elf = make_elf(&[(0x1000_0000, &[1, 2, 3, 4])]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        assert_eq!(image, vec![1, 2, 3, 4]);
    }

    #[test]
    fn base_is_the_lowest_segment_and_gaps_are_zero_filled() {
        let elf = make_elf(&[(0x1000_0100, &[0xAA, 0xBB]), (0x1000_0000, &[1, 2, 3, 4])]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        // 4 bytes of first segment, 0xFC bytes of zero gap, then the second segment.
        assert_eq!(image.len(), 0x102);
        assert_eq!(&image[0..4], &[1, 2, 3, 4]);
        assert!(image[4..0x100].iter().all(|&b| b == 0));
        assert_eq!(&image[0x100..0x102], &[0xAA, 0xBB]);
    }

    #[test]
    fn zero_size_segments_are_skipped() {
        let elf = make_elf(&[(0x1000_0000, &[]), (0x1000_0010, &[9])]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0010);
        assert_eq!(image, vec![9]);
    }

    #[test]
    fn rejects_non_elf_input() {
        assert!(flash_image(&[0u8; 64]).is_err());
    }

    // --- out-of-file-order segments ---

    #[test]
    fn out_of_order_segments_still_place_by_address_not_header_order() {
        // Three segments, deliberately listed high-to-low in the program header table.
        let elf = make_elf(&[
            (0x1000_0200, &[7, 7]),
            (0x1000_0000, &[1, 2, 3, 4]),
            (0x1000_0100, &[0xAA, 0xBB]),
        ]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        assert_eq!(image.len(), 0x202);
        assert_eq!(&image[0..4], &[1, 2, 3, 4]);
        assert_eq!(&image[0x100..0x102], &[0xAA, 0xBB]);
        assert_eq!(&image[0x200..0x202], &[7, 7]);
    }

    // --- gap zero-fill boundaries ---

    #[test]
    fn adjacent_segments_leave_no_gap() {
        let elf = make_elf(&[(0x1000_0000, &[1, 2, 3, 4]), (0x1000_0004, &[5, 6])]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        assert_eq!(image, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn a_one_byte_gap_is_zero_filled() {
        let elf = make_elf(&[(0x1000_0000, &[1, 2, 3, 4]), (0x1000_0005, &[5, 6])]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        assert_eq!(image, vec![1, 2, 3, 4, 0, 5, 6]);
    }

    #[test]
    fn a_larger_gap_is_zero_filled() {
        let elf = make_elf(&[(0x1000_0000, &[1, 2]), (0x1000_0010, &[9])]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        assert_eq!(image.len(), 0x11);
        assert_eq!(&image[0..2], &[1, 2]);
        assert!(image[2..0x10].iter().all(|&b| b == 0));
        assert_eq!(&image[0x10..0x11], &[9]);
    }

    // --- BSS tail: p_filesz < p_memsz ---

    #[test]
    fn bss_tail_past_filesz_is_zero_filled_not_read_from_file() {
        // The tail memsz adds past the file bytes must be zero, not whatever follows in the file.
        let elf = make_elf_with_memsz(&[(0x1000_0000, &[1, 2, 3, 4], 8)]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        assert_eq!(image.len(), 8);
        assert_eq!(&image[0..4], &[1, 2, 3, 4]);
        assert_eq!(&image[4..8], &[0, 0, 0, 0]);
    }

    #[test]
    fn a_trailing_bss_segment_extends_the_image_past_the_last_byte_written() {
        // The image must cover the last segment's full memsz span, not its file bytes alone.
        let elf = make_elf_with_memsz(&[(0x1000_0000, &[1, 2], 2), (0x1000_0010, &[3, 4], 16)]);
        let (base, image) = flash_image(&elf).unwrap();
        assert_eq!(base, 0x1000_0000);
        assert_eq!(image.len(), 0x20);
        assert_eq!(&image[0..2], &[1, 2]);
        assert_eq!(&image[0x10..0x12], &[3, 4]);
        assert!(image[0x12..0x20].iter().all(|&b| b == 0));
    }

    // --- overlapping segments ---

    #[test]
    fn overlapping_segments_are_rejected() {
        let elf = make_elf(&[(0x1000_0000, &[1, 2, 3, 4]), (0x1000_0002, &[9, 9, 9, 9])]);
        assert!(flash_image(&elf).is_err());
    }

    #[test]
    fn a_bss_tail_overlapping_the_next_segment_is_rejected() {
        // A BSS tail reaching into the next segment's file bytes is still an overlap.
        let elf = make_elf_with_memsz(&[(0x1000_0000, &[1, 2], 8), (0x1000_0004, &[9, 9], 2)]);
        assert!(flash_image(&elf).is_err());
    }

    // --- section-header table ---

    /// A well-formed ELF32 LE with only a section-header table: the null section, `.shstrtab`,
    /// then one entry per `(name, sh_type, sh_flags, sh_addr, sh_size)` row, none file-backed.
    fn make_elf_with_sections(sections: &[(&str, u32, u32, u32, u32)]) -> Vec<u8> {
        let ehsize = 52;
        let mut out = vec![0u8; ehsize];
        out[0..4].copy_from_slice(b"\x7fELF");
        out[4] = 1; // ELFCLASS32
        out[5] = 1; // ELFDATA2LSB
        out[6] = 1; // EI_VERSION
        out[16..18].copy_from_slice(&2u16.to_le_bytes()); // e_type = ET_EXEC
        out[18..20].copy_from_slice(&40u16.to_le_bytes()); // e_machine = EM_ARM
        out[40..42].copy_from_slice(&(ehsize as u16).to_le_bytes());

        // Section name string table: offset 0 is the conventional empty name for index 0.
        let mut shstrtab = vec![0u8];
        let mut name_offsets = Vec::new();
        for (name, ..) in sections {
            name_offsets.push(shstrtab.len() as u32);
            shstrtab.extend_from_slice(name.as_bytes());
            shstrtab.push(0);
        }
        let shstrtab_name_off = shstrtab.len() as u32;
        shstrtab.extend_from_slice(b".shstrtab\0");

        let shstrtab_data_off = out.len() as u32;
        out.extend_from_slice(&shstrtab);

        let shentsize = 40usize;
        let shnum = sections.len() + 2; // null section + .shstrtab + the given rows
        let shoff = out.len();
        out.resize(shoff + shnum * shentsize, 0);

        // Index 1: .shstrtab itself (SHT_STRTAB = 3).
        let sh1 = shoff + shentsize;
        out[sh1..sh1 + 4].copy_from_slice(&shstrtab_name_off.to_le_bytes());
        out[sh1 + 4..sh1 + 8].copy_from_slice(&3u32.to_le_bytes());
        out[sh1 + 16..sh1 + 20].copy_from_slice(&shstrtab_data_off.to_le_bytes());
        out[sh1 + 20..sh1 + 24].copy_from_slice(&(shstrtab.len() as u32).to_le_bytes());

        for (i, (_, sh_type, sh_flags, sh_addr, sh_size)) in sections.iter().enumerate() {
            let sh = shoff + (i + 2) * shentsize;
            out[sh..sh + 4].copy_from_slice(&name_offsets[i].to_le_bytes());
            out[sh + 4..sh + 8].copy_from_slice(&sh_type.to_le_bytes());
            out[sh + 8..sh + 12].copy_from_slice(&sh_flags.to_le_bytes());
            out[sh + 12..sh + 16].copy_from_slice(&sh_addr.to_le_bytes());
            out[sh + 20..sh + 24].copy_from_slice(&sh_size.to_le_bytes());
        }

        out[32..36].copy_from_slice(&(shoff as u32).to_le_bytes()); // e_shoff
        out[46..48].copy_from_slice(&(shentsize as u16).to_le_bytes()); // e_shentsize
        out[48..50].copy_from_slice(&(shnum as u16).to_le_bytes()); // e_shnum
        out[50..52].copy_from_slice(&1u16.to_le_bytes()); // e_shstrndx

        out
    }

    #[test]
    fn uninit_range_reads_a_nobits_alloc_section() {
        let elf = make_elf_with_sections(&[(".uninit", SHT_NOBITS, SHF_ALLOC, 0x2000_0100, 64)]);
        assert_eq!(uninit_range(&elf).unwrap(), Some((0x2000_0100, 64)));
    }

    #[test]
    fn uninit_range_is_none_without_the_section() {
        let elf = make_elf_with_sections(&[(".bss", SHT_NOBITS, SHF_ALLOC, 0x2000_0000, 32)]);
        assert_eq!(uninit_range(&elf).unwrap(), None);
    }

    #[test]
    fn uninit_range_is_none_for_an_empty_section() {
        let elf = make_elf_with_sections(&[(".uninit", SHT_NOBITS, SHF_ALLOC, 0x2000_0100, 0)]);
        assert_eq!(uninit_range(&elf).unwrap(), None);
    }

    #[test]
    fn uninit_range_is_none_for_a_file_backed_section() {
        // A `.uninit` name that carries file bytes is not RAM this reader may zero.
        let elf = make_elf_with_sections(&[(".uninit", 1, SHF_ALLOC, 0x2000_0100, 64)]);
        assert_eq!(uninit_range(&elf).unwrap(), None);
    }

    #[test]
    fn uninit_range_is_none_without_alloc() {
        let elf = make_elf_with_sections(&[(".uninit", SHT_NOBITS, 0, 0x2000_0100, 64)]);
        assert_eq!(uninit_range(&elf).unwrap(), None);
    }

    #[test]
    fn uninit_range_is_none_without_a_section_table() {
        // The plain PT_LOAD-only builder emits no section-header table at all.
        let elf = make_elf(&[(0x1000_0000, &[1, 2, 3, 4])]);
        assert_eq!(uninit_range(&elf).unwrap(), None);
    }

    #[test]
    fn uninit_range_rejects_non_elf_input() {
        assert!(uninit_range(&[0u8; 64]).is_err());
    }
}
