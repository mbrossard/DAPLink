//! The die's unique ID and the USB serial number derived from it.

use core::sync::atomic::{compiler_fence, Ordering};

use daplink_rs::daplink_core::serial::{format_serial_words, SERIAL_LEN};

/// Base of the 96-bit unique device id in system memory (RM0008).
const UID_BASE: *const u32 = 0x1FFF_F7E8 as *const u32;

/// Read the 96-bit unique ID as three words in ascending address order.
pub fn uid_words() -> [u32; 3] {
    // SAFETY: the architecturally fixed UID address, 12 bytes wide and word-aligned; each read
    // is one aligned 32-bit volatile load within it.
    unsafe {
        [
            UID_BASE.read_volatile(),
            UID_BASE.add(1).read_volatile(),
            UID_BASE.add(2).read_volatile(),
        ]
    }
}

/// The USB serial, hashed from the UID words' little-endian bytes in ascending word order -- a
/// frozen input sequence, pinned by a golden vector in `daplink-core`.
pub fn serial() -> &'static str {
    static mut SERIAL: [u8; SERIAL_LEN] = [0; SERIAL_LEN];
    static mut LOADED: bool = false;

    critical_section::with(|_| {
        // SAFETY: the statics are touched only here; `LOADED` makes the encoding idempotent.
        unsafe {
            if !LOADED {
                format_serial_words(&uid_words(), &mut *core::ptr::addr_of_mut!(SERIAL));
                // Publish the buffer before the flag.
                compiler_fence(Ordering::Release);
                LOADED = true;
            }
        }
    });

    // SAFETY: initialized above, never written again once `LOADED` is set, and ASCII hex only.
    unsafe { core::str::from_utf8_unchecked(&*core::ptr::addr_of!(SERIAL)) }
}
