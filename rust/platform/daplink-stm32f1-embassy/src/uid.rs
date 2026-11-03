//! The die's unique ID and the USB serial number derived from it.

/// Read the 96-bit unique ID as three words in ascending address order.
pub fn uid_words() -> [u32; 3] {
    let base = embassy_stm32::pac::UID.uid(0).as_ptr().cast::<u32>();
    // SAFETY: the PAC's word-aligned 12-byte UID region; each read is one aligned volatile load.
    unsafe {
        [
            base.read_volatile(),
            base.add(1).read_volatile(),
            base.add(2).read_volatile(),
        ]
    }
}

/// The USB serial, hashed from the UID words in the order pinned by `daplink-core`.
pub fn serial() -> &'static str {
    daplink_rs::daplink_core::cached_serial_words!(uid_words())
}
