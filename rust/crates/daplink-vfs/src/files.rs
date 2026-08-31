//! Virtual files exposed on the volume: the [`VfsFile`] trait plus the built-in status files.

/// One logical sector, in bytes.
const SECTOR: usize = 512;

/// A file rendered on demand into the virtual volume.
pub trait VfsFile {
    /// The 8.3 short name shown in the root directory.
    fn name(&self) -> &FatShortName;
    /// The file size, in bytes.
    fn size(&self) -> u32;
    /// Whether the file currently exists on the volume; `false` gets no directory entry and no
    /// cluster chain. Sampled per (re)mount, so the answer must be stable between remounts.
    fn present(&self) -> bool {
        true
    }
    /// The FAT attribute byte (defaults to read-only).
    fn attrs(&self) -> FileAttrs {
        FileAttrs::READ_ONLY
    }
    /// Render sector `sector_offset` of this file into `buf`, returning the number of valid bytes.
    fn read(&mut self, sector_offset: u32, buf: &mut [u8]) -> usize;
    /// Called at remount with the last transfer's rendered status text.
    fn on_remount(&mut self, _status: &[u8]) {}
    /// Called at remount with whether the last transfer failed.
    fn on_outcome(&mut self, _failed: bool) {}
}

/// An 8.3 short name, stored as the on-disk 11-byte padded field.
pub struct FatShortName([u8; 11]);

impl FatShortName {
    /// Parse `"NAME.EXT"` into a padded, uppercased 8.3 name, validating the characters.
    pub fn new(s: &str) -> Option<Self> {
        let bytes = s.as_bytes();
        let (name, ext) = match s.find('.') {
            Some(dot) => (&bytes[..dot], &bytes[dot + 1..]),
            None => (bytes, &[][..]),
        };
        if name.is_empty() || name.len() > 8 || ext.len() > 3 {
            return None;
        }
        let mut out = [b' '; 11];
        for (i, &c) in name.iter().enumerate() {
            out[i] = valid_char(c)?;
        }
        for (i, &c) in ext.iter().enumerate() {
            out[8 + i] = valid_char(c)?;
        }
        Some(FatShortName(out))
    }

    /// The raw 11-byte 8.3 field.
    pub fn as_bytes(&self) -> &[u8; 11] {
        &self.0
    }
}

/// Accept a legal 8.3 character (uppercased), or `None`.
fn valid_char(c: u8) -> Option<u8> {
    let u = c.to_ascii_uppercase();
    if u.is_ascii_uppercase()
        || u.is_ascii_digit()
        || matches!(
            u,
            b'_' | b'-'
                | b'~'
                | b'$'
                | b'%'
                | b'!'
                | b'#'
                | b'&'
                | b'@'
                | b'^'
                | b'('
                | b')'
                | b'{'
                | b'}'
                | b'\''
        )
    {
        Some(u)
    } else {
        None
    }
}

/// A FAT directory attribute byte.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FileAttrs(pub u8);

impl FileAttrs {
    /// Read-only + archive (`0x21`).
    pub const READ_ONLY: FileAttrs = FileAttrs(0x21);
    /// Volume-label attribute bit (`0x08`).
    pub const VOLUME_ID: FileAttrs = FileAttrs(0x08);
    /// Hidden attribute bit (`0x02`).
    pub const HIDDEN: FileAttrs = FileAttrs(0x02);
}

/// Copy up to a sector's worth of `body` starting at `sector_offset` into `buf`.
fn read_from(body: &[u8], sector_offset: u32, buf: &mut [u8]) -> usize {
    let start = sector_offset as usize * SECTOR;
    if start >= body.len() {
        return 0;
    }
    let n = core::cmp::min(buf.len(), body.len() - start);
    buf[..n].copy_from_slice(&body[start..start + n]);
    n
}

/// Device identity shown in `DETAILS.TXT`.
#[derive(Clone, Copy)]
pub struct DeviceInfo<'a> {
    /// Firmware version string.
    pub version: &'a str,
    /// Device serial string.
    pub serial: &'a str,
    /// Probe resets since power-on, or 0 when the firmware does not track it. Sampled once at
    /// boot, so a jump between two reads of `DETAILS.TXT` proves a reboot in between.
    pub reset_count: u32,
}

/// Width of the fixed `Status:` field in `DETAILS.TXT`; longer status text is truncated.
pub const STATUS_MAX: usize = 64;

/// Width of the fixed `Fault:` field in `DETAILS.TXT`, and of the reason a
/// [`ResetStamp`](crate::stamp::ResetStamp) latches for it.
pub const FAULT_MAX: usize = 48;

/// Width of the fixed zero-padded counter fields in `DETAILS.TXT`.
const COUNTER_WIDTH: usize = 8;

/// A fixed 512-byte `DETAILS.TXT`, baked once at construction. Only the `Remount count:`,
/// `Status:` and `Fault:` fields are patched in place, so no size or offset ever moves.
pub struct DetailsFile<'a> {
    body: [u8; SECTOR],
    /// Byte offset of the `Remount count:` numeric field.
    remount_off: usize,
    /// Byte offset of the `Status:` placeholder field.
    status_off: usize,
    /// Byte offset of the `Fault:` placeholder field.
    fault_off: usize,
    remount_count: u32,
    _info: core::marker::PhantomData<&'a ()>,
}

/// Write `v` into `field` as zero-padded decimal (saturating at all-nines on overflow).
fn write_counter(field: &mut [u8], mut v: u32) {
    field.fill(b'0');
    let mut i = field.len();
    while v > 0 {
        if i == 0 {
            field.fill(b'9');
            return;
        }
        i -= 1;
        field[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
}

impl<'a> DetailsFile<'a> {
    /// An unbaked details file: an all-zero body and no field offsets. Every field is a plain
    /// value, so a `static` holding it lands in `.bss` and can be [`bake`](Self::bake)d in place.
    pub const BLANK: Self = Self {
        body: [0u8; SECTOR],
        remount_off: 0,
        status_off: 0,
        fault_off: 0,
        remount_count: 0,
        _info: core::marker::PhantomData,
    };

    /// Create a details file for the given device `info`, baking the whole body up front.
    pub fn new(info: DeviceInfo<'a>) -> Self {
        let mut this = Self::BLANK;
        this.bake(info);
        this
    }

    /// Render the whole body for `info` in place, with no sector-sized temporary in the caller's
    /// frame. Resets the remount counter, so call it once.
    pub fn bake(&mut self, info: DeviceInfo<'a>) {
        self.remount_count = 0;
        let body = &mut self.body;
        body.fill(b' ');
        let (remount_off, status_off, fault_off);
        {
            let mut w = crate::render::StatusWriter::new(body);
            // One literal per run of constant text: each `bytes()` call costs a pointer and a call.
            w.bytes(b"# DAPLink Firmware - see https://daplink.io\r\nProduct: daplink-rs\r\nUnique ID: ");
            w.bytes(info.serial.as_bytes());
            w.bytes(b"\r\nInterface Version: ");
            w.bytes(info.version.as_bytes());
            w.bytes(b"\r\nReset count: ");
            let reset_off = w.len();
            w.bytes(&[b'0'; COUNTER_WIDTH]);
            w.bytes(b"\r\nRemount count: ");
            remount_off = w.len();
            w.bytes(&[b'0'; COUNTER_WIDTH]);
            w.bytes(b"\r\nStatus: ");
            status_off = w.len();
            w.bytes(&[b' '; STATUS_MAX]);
            w.bytes(b"\r\nFault: ");
            fault_off = w.len();
            w.bytes(&[b' '; FAULT_MAX]);
            w.bytes(b"\r\n");
            write_counter(
                &mut body[reset_off..reset_off + COUNTER_WIDTH],
                info.reset_count,
            );
        }
        self.remount_off = remount_off;
        self.status_off = status_off;
        self.fault_off = fault_off;
    }
}

impl DetailsFile<'_> {
    /// Seed the `Status:` field with a status that survived a probe reset (from a
    /// [`ResetStamp`](crate::stamp::ResetStamp)), without counting a remount.
    pub fn seed_status(&mut self, status: &[u8]) {
        let field = &mut self.body[self.status_off..self.status_off + STATUS_MAX];
        field.fill(b' ');
        let n = core::cmp::min(status.len(), STATUS_MAX);
        field[..n].copy_from_slice(&status[..n]);
    }

    /// Seed the `Fault:` field with the reason a fault latched before the reset that ended the
    /// previous boot; blank when the probe has not faulted.
    pub fn seed_fault(&mut self, reason: &[u8]) {
        let field = &mut self.body[self.fault_off..self.fault_off + FAULT_MAX];
        field.fill(b' ');
        let n = core::cmp::min(reason.len(), FAULT_MAX);
        field[..n].copy_from_slice(&reason[..n]);
    }
}

impl VfsFile for DetailsFile<'_> {
    fn name(&self) -> &FatShortName {
        &DETAILS_NAME
    }

    fn size(&self) -> u32 {
        SECTOR as u32
    }

    fn read(&mut self, sector_offset: u32, buf: &mut [u8]) -> usize {
        read_from(&self.body, sector_offset, buf)
    }

    fn on_remount(&mut self, status: &[u8]) {
        self.remount_count = self.remount_count.saturating_add(1);
        let off = self.remount_off;
        write_counter(&mut self.body[off..off + COUNTER_WIDTH], self.remount_count);
        let field = &mut self.body[self.status_off..self.status_off + STATUS_MAX];
        field.fill(b' ');
        let n = core::cmp::min(status.len(), STATUS_MAX);
        field[..n].copy_from_slice(&status[..n]);
    }
}

static DETAILS_NAME: FatShortName = FatShortName(*b"DETAILS TXT");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_name_padding() {
        let n = FatShortName::new("DETAILS.TXT").unwrap();
        assert_eq!(n.as_bytes(), b"DETAILS TXT");
        let n = FatShortName::new("readme").unwrap();
        assert_eq!(n.as_bytes(), b"README     ");
    }

    #[test]
    fn short_name_rejects_bad() {
        assert!(FatShortName::new("").is_none());
        assert!(FatShortName::new("TOOLONGNAME.BIN").is_none());
        assert!(FatShortName::new("A.TOOLONG").is_none());
    }

    #[test]
    fn details_body_is_512_and_padded() {
        let mut f = DetailsFile::new(DeviceInfo {
            version: "1.2.3",
            serial: "ABCD",
            reset_count: 7,
        });
        assert_eq!(f.size(), 512);
        assert_eq!(f.name().as_bytes(), b"DETAILS TXT");
        let mut buf = [0u8; 512];
        let n = f.read(0, &mut buf);
        assert_eq!(n, 512);
        let s = std::str::from_utf8(&buf).unwrap();
        assert!(s.starts_with("# DAPLink Firmware - see https://daplink.io\r\n"));
        assert!(s.contains("Interface Version: 1.2.3"));
        assert!(s.contains("Unique ID: ABCD"));
        assert!(s.contains("Reset count: 00000007"));
        assert!(s.contains("Remount count: 00000000"));
        assert!(s.contains("Status: "));
        assert!(s.contains("Fault: "));
        assert!(s.ends_with(' '));
    }

    /// A latched reason patches the `Fault:` field and nothing else.
    #[test]
    fn details_fault_field_is_blank_until_seeded() {
        let mut f = DetailsFile::new(DeviceInfo {
            version: "0.1.0",
            serial: "S",
            reset_count: 1,
        });
        let mut before = [0u8; 512];
        f.read(0, &mut before);
        let sb = std::str::from_utf8(&before).unwrap();
        let fault_off = sb.find("Fault: ").unwrap() + "Fault: ".len();
        assert!(before[fault_off..fault_off + FAULT_MAX]
            .iter()
            .all(|&b| b == b' '));

        f.seed_fault(b"panic src/main.rs:412");
        let mut after = [0u8; 512];
        assert_eq!(f.read(0, &mut after), 512);
        let sa = std::str::from_utf8(&after).unwrap();
        assert!(sa.contains("Fault: panic src/main.rs:412"));
        for (i, (a, b)) in before.iter().zip(after.iter()).enumerate() {
            assert!(
                a == b || (fault_off..fault_off + FAULT_MAX).contains(&i),
                "unexpected byte change at offset {i}"
            );
        }
    }

    /// A reason longer than the field is truncated, never shifting a later offset.
    #[test]
    fn details_fault_truncates_at_field_width() {
        let mut f = DetailsFile::new(DeviceInfo {
            version: "0",
            serial: "S",
            reset_count: 0,
        });
        f.seed_fault(&[b'A'; 200]);
        let mut buf = [0u8; 512];
        f.read(0, &mut buf);
        let s = std::str::from_utf8(&buf).unwrap();
        let off = s.find("Fault: ").unwrap() + "Fault: ".len();
        assert!(buf[off..off + FAULT_MAX].iter().all(|&b| b == b'A'));
        assert_eq!(buf[off + FAULT_MAX], b'\r');
    }

    /// frozen: a host that scrapes `Status:`/`Fault:` at fixed columns, or caches the 512-byte
    /// size across remounts, depends on every one of these bytes and offsets staying put.
    #[test]
    fn details_txt_golden_vector_known_device() {
        const SERIAL: &str = "1234567890ABCDEF";
        const VERSION: &str = "1.2.3";
        // Offsets into the rendered body for this exact serial/version pair.
        const REMOUNT_OFF: usize = 159;
        const STATUS_OFF: usize = 177;
        const FAULT_OFF: usize = 250;
        const CONTENT_LEN: usize = 300;

        let mut f = DetailsFile::new(DeviceInfo {
            version: VERSION,
            serial: SERIAL,
            reset_count: 0,
        });
        let mut buf = [0u8; 512];
        assert_eq!(f.read(0, &mut buf), 512);

        let status_pad = " ".repeat(STATUS_MAX);
        let fault_pad = " ".repeat(FAULT_MAX);
        let expected_text = std::format!(
            "# DAPLink Firmware - see https://daplink.io\r\n\
             Product: daplink-rs\r\n\
             Unique ID: {SERIAL}\r\n\
             Interface Version: {VERSION}\r\n\
             Reset count: 00000000\r\n\
             Remount count: 00000000\r\n\
             Status: {status_pad}\r\n\
             Fault: {fault_pad}\r\n"
        );
        assert_eq!(expected_text.len(), CONTENT_LEN);

        let mut expected = std::vec![b' '; 512];
        expected[..CONTENT_LEN].copy_from_slice(expected_text.as_bytes());
        assert_eq!(&buf[..], &expected[..]);

        // The offsets above must land exactly on the placeholder fields the golden text encodes.
        assert_eq!(&buf[REMOUNT_OFF..REMOUNT_OFF + 8], b"00000000");
        assert_eq!(
            &buf[STATUS_OFF..STATUS_OFF + STATUS_MAX],
            status_pad.as_bytes()
        );
        assert_eq!(&buf[FAULT_OFF..FAULT_OFF + FAULT_MAX], fault_pad.as_bytes());

        // The size and these offsets must not move when a remount patches the fields.
        f.on_remount(b"Flash: 42 bytes -> Done [RP2040]");
        let mut after = [0u8; 512];
        assert_eq!(f.read(0, &mut after), 512);
        assert_eq!(&after[REMOUNT_OFF..REMOUNT_OFF + 8], b"00000001");
        assert_eq!(
            &after[STATUS_OFF..STATUS_OFF + "Flash: 42 bytes -> Done [RP2040]".len()],
            b"Flash: 42 bytes -> Done [RP2040]"
        );
        // Bytes outside the two patched fields are unchanged from the golden baseline.
        for (i, (a, b)) in buf.iter().zip(after.iter()).enumerate() {
            let in_remount = (REMOUNT_OFF..REMOUNT_OFF + 8).contains(&i);
            let in_status = (STATUS_OFF..STATUS_OFF + STATUS_MAX).contains(&i);
            assert!(a == b || in_remount || in_status, "byte {i} moved");
        }
    }

    #[test]
    fn details_size_and_offsets_never_change() {
        let mut f = DetailsFile::new(DeviceInfo {
            version: "0.1.0",
            serial: "E6614C30934B622C",
            reset_count: 0,
        });
        let mut before = [0u8; 512];
        f.read(0, &mut before);

        // Two remounts with different-length statuses leave every other byte identical.
        f.on_remount(b"Flash: 9984 bytes -> Done [RP2040]");
        f.on_remount(b"x");
        let mut after = [0u8; 512];
        let n = f.read(0, &mut after);
        assert_eq!(n, 512);
        let s = std::str::from_utf8(&after).unwrap();
        assert!(s.contains("Remount count: 00000002"));
        // The second (shorter) status fully overwrites the first: no residue of "Done".
        assert!(s.contains("Status: x"));
        assert!(!s.contains("Done"));
        // Outside the counter + status fields the body is byte-identical to the baked original.
        let sb = std::str::from_utf8(&before).unwrap();
        let remount_off = sb.find("Remount count: ").unwrap() + "Remount count: ".len();
        let status_off = sb.find("Status: ").unwrap() + "Status: ".len();
        for (i, (a, b)) in before.iter().zip(after.iter()).enumerate() {
            if a == b {
                continue;
            }
            let in_counter = (remount_off..remount_off + 8).contains(&i);
            let in_status = (status_off..status_off + STATUS_MAX).contains(&i);
            assert!(
                in_counter || in_status,
                "unexpected byte change at offset {i}"
            );
        }
    }

    #[test]
    fn details_status_truncates_at_field_width() {
        let mut f = DetailsFile::new(DeviceInfo {
            version: "0",
            serial: "S",
            reset_count: 0,
        });
        let long = [b'A'; 100];
        f.on_remount(&long);
        let mut buf = [0u8; 512];
        f.read(0, &mut buf);
        let s = std::str::from_utf8(&buf).unwrap();
        let status_off = s.find("Status: ").unwrap() + "Status: ".len();
        let field = &buf[status_off..status_off + STATUS_MAX];
        assert!(
            field.iter().all(|&b| b == b'A'),
            "field filled to its width"
        );
        assert_eq!(
            buf[status_off + STATUS_MAX],
            b'\r',
            "truncated exactly at the field width"
        );
    }

    #[test]
    fn counter_field_saturates() {
        let mut field = [0u8; 8];
        write_counter(&mut field, 12345);
        assert_eq!(&field, b"00012345");
        write_counter(&mut field, 4_294_967_295);
        assert_eq!(&field, b"99999999");
    }
}
