//! The factory UUID and the USB serial number derived from it.

use daplink_rs::daplink_core::serial::{format_serial_words, SERIAL_LEN};

/// Base of the 128-bit factory UUID in the Protected Flash Region (UM11126 section 47.8).
const UUID_BASE: *const u32 = 0x0009_FC70 as *const u32;

fn uuid_words() -> [u32; 4] {
    // SAFETY: the documented PFR UUID, read as aligned volatile words (narrower may fault).
    unsafe {
        [
            UUID_BASE.read_volatile(),
            UUID_BASE.add(1).read_volatile(),
            UUID_BASE.add(2).read_volatile(),
            UUID_BASE.add(3).read_volatile(),
        ]
    }
}

/// USB serial hashed from the UUID words' little-endian bytes; the input order is frozen.
pub(crate) fn serial(buf: &mut [u8; SERIAL_LEN]) -> &str {
    format_serial_words(&uuid_words(), buf)
}
