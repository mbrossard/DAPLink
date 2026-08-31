//! The factory UUID and the USB serial number derived from it.

use daplink_rs::daplink_core::serial::{format_serial_words, SERIAL_LEN};

/// Base of the 128-bit factory UUID in the Protected Flash Region (UM11126 section 47.8).
const UUID_BASE: *const u32 = 0x0009_FC70 as *const u32;

/// Read the factory UUID as four 32-bit words in ascending address order.
pub fn uuid_words() -> [u32; 4] {
    // SAFETY: the documented, word-aligned PFR UUID location; each read is one aligned 32-bit
    // volatile load, so the width survives optimization (the PFR may reject narrower accesses).
    unsafe {
        [
            UUID_BASE.read_volatile(),
            UUID_BASE.add(1).read_volatile(),
            UUID_BASE.add(2).read_volatile(),
            UUID_BASE.add(3).read_volatile(),
        ]
    }
}

/// The probe's USB serial number, hashed from the UUID words' little-endian bytes in ascending
/// word order -- a frozen input sequence, pinned by a golden vector in `daplink-core`.
pub fn serial(buf: &mut [u8; SERIAL_LEN]) -> &str {
    format_serial_words(&uuid_words(), buf)
}
