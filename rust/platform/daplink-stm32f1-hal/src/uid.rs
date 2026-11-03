//! The die's unique ID and the USB serial number derived from it.

/// Base of the 96-bit unique device id in system memory (RM0008).
const UID_BASE: *const u32 = 0x1FFF_F7E8 as *const u32;

/// Read the 96-bit unique ID as three words in ascending address order.
pub fn uid_words() -> [u32; 3] {
    // SAFETY: the fixed, word-aligned 12-byte UID region; each read is one aligned volatile load.
    unsafe {
        [
            UID_BASE.read_volatile(),
            UID_BASE.add(1).read_volatile(),
            UID_BASE.add(2).read_volatile(),
        ]
    }
}

/// The USB serial, hashed from the UID words in the order pinned by `daplink-core`.
pub fn serial() -> &'static str {
    daplink_rs::daplink_core::cached_serial_words!(uid_words())
}
