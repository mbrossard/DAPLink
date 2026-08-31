//! Half-open address-range containment, shared by every layer that reasons about target memory.

/// Whether `addr` lies within the half-open range `[start, end)`.
pub const fn contains(start: u32, end: u32, addr: u32) -> bool {
    start <= addr && addr < end
}

#[cfg(test)]
mod tests {
    use super::contains;

    #[test]
    fn half_open_semantics() {
        assert!(contains(0x1000, 0x2000, 0x1000));
        assert!(contains(0x1000, 0x2000, 0x1FFF));
        assert!(!contains(0x1000, 0x2000, 0x2000));
        assert!(!contains(0x1000, 0x2000, 0x0FFF));
    }

    #[test]
    fn empty_range_contains_nothing() {
        assert!(!contains(0x1000, 0x1000, 0x1000));
        assert!(!contains(0x2000, 0x1000, 0x1800));
    }

    #[test]
    fn full_address_space_edges() {
        assert!(contains(0, u32::MAX, 0));
        assert!(contains(0, u32::MAX, u32::MAX - 1));
        assert!(!contains(0, u32::MAX, u32::MAX));
    }
}
