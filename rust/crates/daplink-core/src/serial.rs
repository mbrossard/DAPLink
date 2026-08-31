//! The probe's USB serial number: `0000` plus the low 48 bits of an FNV-1a 64 hash of the unique id.

/// FNV-1a 64 offset basis.
pub const FNV1A64_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
/// FNV-1a 64 prime.
pub const FNV1A64_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Length of a daplink-rs serial number, in characters (and, being ASCII, in bytes).
pub const SERIAL_LEN: usize = 16;

/// The fixed leading characters of every daplink-rs serial: an unregistered DAPLink board id,
/// so hosts that parse that namespace resolve nothing rather than the wrong board.
pub const SERIAL_PREFIX: &[u8; 4] = b"0000";

/// Uppercase hexadecimal digits -- the only alphabet the serial may use.
const HEX: &[u8; 16] = b"0123456789ABCDEF";

/// Fold one byte into a running FNV-1a 64 state: XOR, then multiply by the prime.
const fn fnv1a64_byte(hash: u64, byte: u8) -> u64 {
    (hash ^ byte as u64).wrapping_mul(FNV1A64_PRIME)
}

/// The FNV-1a 64 hash of `bytes`.
pub const fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash = FNV1A64_OFFSET_BASIS;
    let mut i = 0;
    while i < bytes.len() {
        hash = fnv1a64_byte(hash, bytes[i]);
        i += 1;
    }
    hash
}

/// The FNV-1a 64 hash of `words` as their little-endian bytes, in ascending word order. Identical
/// to a byte-wise read of the same region, so a part read a word at a time cannot drift from one.
pub const fn fnv1a64_words_le(words: &[u32]) -> u64 {
    let mut hash = FNV1A64_OFFSET_BASIS;
    let mut w = 0;
    while w < words.len() {
        let word = words[w];
        let mut b = 0;
        while b < 4 {
            hash = fnv1a64_byte(hash, (word >> (8 * b)) as u8);
            b += 1;
        }
        w += 1;
    }
    hash
}

/// Render `hash` as a serial number into `out`: the fixed prefix followed by the low 48 bits as
/// 12 uppercase hexadecimal digits, most significant first.
pub fn format_hash(hash: u64, out: &mut [u8; SERIAL_LEN]) -> &str {
    out[..4].copy_from_slice(SERIAL_PREFIX);
    for (i, slot) in out[4..].iter_mut().enumerate() {
        // Digit 0 is the most significant of the 12, i.e. bits 47..44.
        let shift = 44 - 4 * i;
        *slot = HEX[((hash >> shift) & 0xf) as usize];
    }
    // The prefix and `HEX` are ASCII by construction, so this cannot fail.
    core::str::from_utf8(out).unwrap_or("")
}

/// Format the serial number for a part whose raw unique id is the byte sequence `uid`. The caller
/// owns `out`: this crate has an empty dependency list, so it can hold no static of its own.
pub fn format_serial<'a>(uid: &[u8], out: &'a mut [u8; SERIAL_LEN]) -> &'a str {
    format_hash(fnv1a64(uid), out)
}

/// Format the serial number for a part whose raw unique id is read as 32-bit `words`.
/// Equivalent to [`format_serial`] over each word's little-endian bytes; see [`fnv1a64_words_le`].
pub fn format_serial_words<'a>(words: &[u32], out: &'a mut [u8; SERIAL_LEN]) -> &'a str {
    format_hash(fnv1a64_words_le(words), out)
}

/// The FAT volume id for the probe carrying `serial`: its last eight hexadecimal digits, i.e. the
/// low 32 bits of the same hash. `None` unless `serial` ends in eight hex digits.
pub fn volume_id(serial: &str) -> Option<u32> {
    let b = serial.as_bytes();
    if b.len() < 8 {
        return None;
    }
    let mut v = 0u32;
    for &c in &b[b.len() - 8..] {
        let digit = match c {
            b'0'..=b'9' => c - b'0',
            b'A'..=b'F' => c - b'A' + 10,
            b'a'..=b'f' => c - b'a' + 10,
            _ => return None,
        };
        v = (v << 4) | digit as u32;
    }
    Some(v)
}

/// Golden vectors pinning the probe's public identity, one per family: both the hash function and
/// the byte sequence each family feeds it. A failure here is a design decision, not a test to fix.
#[cfg(test)]
mod tests {
    use super::*;

    fn serial_of(uid: &[u8]) -> [u8; SERIAL_LEN] {
        let mut out = [0u8; SERIAL_LEN];
        format_serial(uid, &mut out);
        out
    }

    /// The published FNV-1a 64 test vector, plus the degenerate anchors. `""` must hash to the
    /// offset basis unchanged; a single zero byte must not.
    #[test]
    fn fnv1a64_matches_published_vectors() {
        assert_eq!(fnv1a64(b""), FNV1A64_OFFSET_BASIS);
        assert_eq!(fnv1a64(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv1a64(&[0x00]), 0xaf63_bd4c_8601_b7df);
        assert_eq!(fnv1a64(b"abc"), 0xe71f_a219_0541_574b);
    }

    /// The full 16-character rendering of a known hash: fixed prefix, then the low 48 bits.
    #[test]
    fn format_hash_is_prefix_plus_low_48_bits() {
        let mut out = [0u8; SERIAL_LEN];
        assert_eq!(
            format_hash(0x0123_4567_89AB_CDEF, &mut out),
            "0000456789ABCDEF"
        );
        // The top 16 bits of the hash are dropped, so they cannot affect the serial.
        let mut other = [0u8; SERIAL_LEN];
        assert_eq!(
            format_hash(0xFFFF_4567_89AB_CDEF, &mut other),
            format_hash(0x0123_4567_89AB_CDEF, &mut out)
        );
    }

    /// Every serial is exactly 16 characters drawn from `0-9A-F`, as mass storage requires.
    #[test]
    fn serials_are_16_uppercase_hex_characters() {
        for uid in [&b""[..], &[0u8][..], &[0xff; 16][..], &[0x5a; 12][..]] {
            let s = serial_of(uid);
            assert_eq!(s.len(), 16);
            assert!(
                s.iter()
                    .all(|c| c.is_ascii_digit() || (b'A'..=b'F').contains(c)),
                "serial must be uppercase hex: {:?}",
                core::str::from_utf8(&s)
            );
        }
    }

    /// The fixed prefix is not derived from the hash and is not a registered DAPLink board id.
    #[test]
    fn prefix_is_constant() {
        assert_eq!(SERIAL_PREFIX, b"0000");
        assert_eq!(&serial_of(b"one")[..4], SERIAL_PREFIX);
        assert_eq!(&serial_of(b"another")[..4], SERIAL_PREFIX);
    }

    /// The whole unique id feeds the hash, so parts differing only at either end differ in serial.
    #[test]
    fn every_input_byte_changes_the_serial() {
        let base = [
            0x38, 0x00, 0x45, 0x00, 0x32, 0x31, 0x33, 0x51, 0x35, 0x35, 0x39, 0x33,
        ];
        for i in 0..base.len() {
            let mut other = base;
            other[i] ^= 0x01;
            assert_ne!(
                serial_of(&base),
                serial_of(&other),
                "flipping byte {i} must change the serial"
            );
        }
    }

    /// Word-wise and byte-wise hashing of the same region agree.
    #[test]
    fn word_and_byte_hashing_agree() {
        let words = [0x0045_0038u32, 0x5133_3132, 0x3339_3535];
        let bytes = [
            0x38, 0x00, 0x45, 0x00, 0x32, 0x31, 0x33, 0x51, 0x35, 0x35, 0x39, 0x33,
        ];
        assert_eq!(fnv1a64_words_le(&words), fnv1a64(&bytes));
        let (mut a, mut b) = ([0u8; SERIAL_LEN], [0u8; SERIAL_LEN]);
        assert_eq!(
            format_serial_words(&words, &mut a),
            format_serial(&bytes, &mut b)
        );
    }

    /// Per-family golden vectors: the byte sequence each family feeds the hash, and the serial it
    /// must produce. The input sequence is as frozen as the hash function itself.
    #[test]
    fn per_family_input_sequences_are_frozen() {
        // STM32 via embassy: the 96-bit id as three words, little-endian bytes in address order.
        let words = [0x0045_0038u32, 0x5133_3132, 0x3339_3535];
        let mut out = [0u8; SERIAL_LEN];
        assert_eq!(format_serial_words(&words, &mut out), "000037F21BCA0157");

        // STM32F1 via stm32f1xx-hal: the 96-bit id at 0x1FFF_F7E8 read as 12 ascending bytes.
        let f1_uid = [0u8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
        assert_eq!(serial_of(&f1_uid), *b"00004819416D7FB9");

        // RP2040/RP235x: the 8-byte id exactly as its source hands it over.
        let rp_uid = [0u8, 1, 2, 3, 4, 5, 6, 7];
        assert_eq!(serial_of(&rp_uid), *b"000049E2B28ECB7D");

        // LPC55: the 128-bit factory UUID at 0x0009_FC70 as four words, little-endian bytes.
        let lpc_uuid = [0x0011_2233u32, 0x4455_6677, 0x8899_AABB, 0xCCDD_EEFF];
        let mut lpc = [0u8; SERIAL_LEN];
        assert_eq!(format_serial_words(&lpc_uuid, &mut lpc), "0000F34696224BC5");
    }

    /// The two LPC55 UUIDs meaning "unavailable" are used as read, so their serials are fixed and
    /// a probe showing one is recognisable on sight rather than merely odd.
    #[test]
    fn lpc55_unavailable_uuid_serials_are_documented() {
        let mut out = [0u8; SERIAL_LEN];
        assert_eq!(
            format_serial_words(&[0x0000_0000u32; 4], &mut out),
            "00001FB960FF6465"
        );
        assert_eq!(
            format_serial_words(&[0xFFFF_FFFFu32; 4], &mut out),
            "00007508F5A1E855"
        );
    }

    /// Frozen RP2040 and RP2350 vectors: the serials each RP platform crate must reproduce.
    #[test]
    fn rp2040_and_rp2350_frozen_vectors() {
        // RP2040: `rp2040_flash::flash::flash_unique_id` returns 8 bytes, fed unmodified.
        let rp2040_uid: [u8; 8] = [0x38, 0x00, 0x45, 0x00, 0x32, 0x31, 0x33, 0x51];
        assert_eq!(serial_of(&rp2040_uid), *b"0000F132445D5C49");

        // RP2350: `embassy_rp::otp::get_chipid` returns a `u64`, fed as its big-endian bytes.
        let rp2350_id: u64 = 0x0035_3231_3300_4500;
        assert_eq!(serial_of(&rp2350_id.to_be_bytes()), *b"0000C885D2512F01");
    }

    /// The FAT volume id is the low 32 bits of the same hash, read straight off the serial.
    #[test]
    fn volume_id_is_the_low_32_bits_of_the_hash() {
        let mut out = [0u8; SERIAL_LEN];
        let serial = format_serial_words(&[0x0045_0038u32, 0x5133_3132, 0x3339_3535], &mut out);
        assert_eq!(serial, "000037F21BCA0157");
        assert_eq!(volume_id(serial), Some(0x1BCA_0157));
    }

    /// A frozen volume-id vector against an RP golden serial, so the two cannot drift apart.
    #[test]
    fn volume_id_frozen_vector() {
        assert_eq!(volume_id("0000F132445D5C49"), Some(0x445D_5C49));
    }

    /// A non-hex serial yields no volume id, leaving the fallback to the caller.
    #[test]
    fn volume_id_rejects_non_hex_and_short_serials() {
        assert_eq!(volume_id("RP-UID-UNKNOWN"), None);
        assert_eq!(volume_id("ABC"), None);
        assert_eq!(volume_id(""), None);
    }
}
