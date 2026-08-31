//! The two checksums the frames carry: CRC32 over payload spans, CRC16 over frame headers.

/// Nibble table for the reflected CRC32 polynomial 0xEDB88320.
const TABLE: [u32; 16] = {
    let mut table = [0u32; 16];
    let mut i = 0;
    while i < 16 {
        let mut c = i as u32;
        let mut bit = 0;
        while bit < 4 {
            c = if c & 1 != 0 {
                0xEDB8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
            bit += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
};

/// Running CRC32 of a payload span, four bits at a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Crc32(u32);

impl Crc32 {
    /// An empty span.
    pub const fn new() -> Self {
        Self(0xFFFF_FFFF)
    }

    /// Extend the span by one byte.
    pub fn push(&mut self, b: u8) {
        let mut c = self.0;
        c = TABLE[((c ^ u32::from(b)) & 0x0F) as usize] ^ (c >> 4);
        c = TABLE[((c ^ (u32::from(b) >> 4)) & 0x0F) as usize] ^ (c >> 4);
        self.0 = c;
    }

    /// Extend the span by a slice.
    pub fn extend(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.push(b);
        }
    }

    /// The checksum of the span so far.
    pub const fn value(self) -> u32 {
        self.0 ^ 0xFFFF_FFFF
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC32 of one slice.
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = Crc32::new();
    crc.extend(bytes);
    crc.value()
}

/// CRC-16/CCITT-FALSE, which protects a frame header.
pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in bytes {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- checksum vectors, pinned: host and DUT must agree with each other and with the spec ---

    #[test]
    fn crc32_matches_the_standard_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
        assert_eq!(crc32(&[0u8; 4]), 0x2144_DF1C);
    }

    #[test]
    fn crc16_matches_the_ccitt_false_check_value() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
        assert_eq!(crc16(b""), 0xFFFF);
    }

    #[test]
    fn incremental_and_one_shot_crc32_agree() {
        let data: [u8; 64] = core::array::from_fn(|i| (i * 7) as u8);
        let mut crc = Crc32::new();
        for (i, &b) in data.iter().enumerate() {
            crc.push(b);
            assert_eq!(crc.value(), crc32(&data[..=i]));
        }
    }
}
