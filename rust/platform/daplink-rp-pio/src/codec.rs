//! The FIFO command-word codec: `[entry PC | direction | bit count - 1]`.

/// Maximum bit count one command word encodes; the field is `bit_count - 1` over 8 bits.
pub const MAX_BITS: u16 = 256;

/// Encode a FIFO command word; `bits` must be in `1..=256`, a zero-bit command would deadlock.
pub const fn cmd_word(entry_abs: u8, dir_drive: bool, bits: u16) -> u32 {
    debug_assert!(bits >= 1 && bits <= MAX_BITS);
    ((bits - 1) as u32 & 0xff) | ((dir_drive as u32) << 8) | ((entry_abs as u32) << 9)
}

/// Encode a direction-only command: dispatch to the park site with no data phase.
pub const fn skip_word(get_next_cmd_abs: u8, dir_drive: bool) -> u32 {
    // The SM ignores the count here, so it carries the harmless all-ones value.
    0xff | ((dir_drive as u32) << 8) | ((get_next_cmd_abs as u32) << 9)
}

/// Right-align an `n`-bit read result; the SM shifts input right, so it arrives in the top bits.
pub const fn read_align(raw: u32, n: u8) -> u32 {
    debug_assert!(n >= 1 && n <= 32);
    if n >= 32 {
        raw
    } else {
        raw >> (32 - n as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cmd_word_packs_fields() {
        let w = cmd_word(8, true, 8);
        assert_eq!(w & 0xff, 7);
        assert_eq!((w >> 8) & 1, 1);
        assert_eq!((w >> 9) & 0x1f, 8);

        let w = cmd_word(24, false, 1);
        assert_eq!(w & 0xff, 0);
        assert_eq!((w >> 8) & 1, 0);
        assert_eq!((w >> 9) & 0x1f, 24);

        assert_eq!(cmd_word(3, true, 256) & 0xff, 0xff);
    }

    #[test]
    fn skip_word_ignores_count() {
        let w = skip_word(3, true);
        assert_eq!(w & 0xff, 0xff);
        assert_eq!((w >> 8) & 1, 1);
        assert_eq!((w >> 9) & 0x1f, 3);
    }

    #[test]
    fn read_align_shifts_by_width() {
        assert_eq!(read_align(0b101 << 29, 3), 0b101);
        assert_eq!(read_align(1 << 31, 1), 1);
        assert_eq!(read_align(0xDEAD_BEEF, 32), 0xDEAD_BEEF);
    }
}
