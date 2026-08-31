//! The die's unique ID and the USB serial number derived from it.

use core::sync::atomic::{compiler_fence, Ordering};

use daplink_rs::daplink_core::serial::{format_serial_words, SERIAL_LEN};

/// Read the 96-bit unique ID as three words in ascending address order.
pub fn uid_words() -> [u32; 3] {
    let base = embassy_stm32::pac::UID.uid(0).as_ptr().cast::<u32>();
    // SAFETY: the PAC's own UID address, 12 bytes wide and word-aligned; each read is one
    // aligned 32-bit volatile load within it.
    unsafe {
        [
            base.read_volatile(),
            base.add(1).read_volatile(),
            base.add(2).read_volatile(),
        ]
    }
}

/// The probe's USB serial number, hashed from the UID words' little-endian bytes in ascending
/// word order -- a frozen input sequence, pinned by a golden vector in `daplink-core`.
pub fn serial() -> &'static str {
    static mut SERIAL: [u8; SERIAL_LEN] = [0; SERIAL_LEN];
    static mut LOADED: bool = false;

    critical_section::with(|_| {
        // SAFETY: the statics are only touched inside this critical section; `LOADED` makes the
        // encoding idempotent.
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
