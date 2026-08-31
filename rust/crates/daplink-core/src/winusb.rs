//! WinUSB (MS OS 2.0) descriptor bytes shared by the polled and async USB adapters.

/// Vendor request code the BOS capability points the host at for the MS OS 2.0 descriptor set.
pub const MSOS_VENDOR_CODE: u8 = 0x41;

/// BOS device-capability type for a platform capability.
pub const BOS_CAPABILITY_PLATFORM: u8 = 0x05;

const MSOS_LEN: u16 = 170;

const fn lo(v: u16) -> u8 {
    v.to_le_bytes()[0]
}
const fn hi(v: u16) -> u8 {
    v.to_le_bytes()[1]
}

/// BOS MS OS 2.0 platform capability payload; points Windows at [`MSOS_VENDOR_CODE`].
#[rustfmt::skip]
pub const BOS_PLATFORM: [u8; 25] = [
    0x00, // reserved
    // MS OS 2.0 platform capability UUID {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
    0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,
    0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f,
    0x00, 0x00, 0x03, 0x06,     // dwWindowsVersion = Windows 8.1
    lo(MSOS_LEN), hi(MSOS_LEN), // wMSOSDescriptorSetTotalLength
    MSOS_VENDOR_CODE,           // bMS_VendorCode
    0x00,                       // bAltEnumCode = none
];

/// MS OS 2.0 descriptor set: interface 0 declares the `WINUSB` compatible id and the CMSIS-DAP GUID.
#[rustfmt::skip]
pub const MS_OS_DESCRIPTOR: [u8; MSOS_LEN as usize] = [
    // Set header (10 bytes)
    0x0a, 0x00,                 // wLength = 10
    0x00, 0x00,                 // wDescriptorType = MS_OS_20_SET_HEADER_DESCRIPTOR
    0x00, 0x00, 0x03, 0x06,     // dwWindowsVersion = Windows 8.1
    lo(MSOS_LEN), hi(MSOS_LEN), // wTotalLength
    // Function subset header (8 bytes)
    0x08, 0x00,                 // wLength = 8
    0x02, 0x00,                 // wDescriptorType = MS_OS_20_SUBSET_HEADER_FUNCTION
    0x00, 0x00,                 // bFirstInterface = 0, bReserved
    lo(160), hi(160),           // wSubsetLength = 160
    // Compatible ID descriptor (20 bytes)
    0x14, 0x00,                 // wLength = 20
    0x03, 0x00,                 // wDescriptorType = MS_OS_20_FEATURE_COMPATIBLE_ID
    b'W', b'I', b'N', b'U', b'S', b'B', 0x00, 0x00, // CompatibleID = "WINUSB"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // SubCompatibleID
    // Registry property descriptor (132 bytes)
    lo(132), hi(132),           // wLength = 132
    0x04, 0x00,                 // wDescriptorType = MS_OS_20_FEATURE_REG_PROPERTY
    0x07, 0x00,                 // wPropertyDataType = REG_MULTI_SZ
    lo(42), hi(42),             // wPropertyNameLength = 42
    // "DeviceInterfaceGUIDs\0" (UTF-16LE, 42 bytes)
    b'D', 0, b'e', 0, b'v', 0, b'i', 0, b'c', 0, b'e', 0,
    b'I', 0, b'n', 0, b't', 0, b'e', 0, b'r', 0, b'f', 0,
    b'a', 0, b'c', 0, b'e', 0, b'G', 0, b'U', 0, b'I', 0,
    b'D', 0, b's', 0, 0, 0,
    lo(80), hi(80),             // wPropertyDataLength = 80
    // "{CDB3B5AD-293B-4663-AA36-1AAE46463776}\0\0" (UTF-16LE, 80 bytes)
    b'{', 0, b'C', 0, b'D', 0, b'B', 0, b'3', 0, b'B', 0, b'5', 0, b'A', 0,
    b'D', 0, b'-', 0, b'2', 0, b'9', 0, b'3', 0, b'B', 0, b'-', 0, b'4', 0,
    b'6', 0, b'6', 0, b'3', 0, b'-', 0, b'A', 0, b'A', 0, b'3', 0, b'6', 0,
    b'-', 0, b'1', 0, b'A', 0, b'A', 0, b'E', 0, b'4', 0, b'6', 0, b'4', 0,
    b'6', 0, b'3', 0, b'7', 0, b'7', 0, b'6', 0, b'}', 0, 0, 0, 0, 0,
];

#[cfg(test)]
mod tests {
    use super::*;

    /// frozen: the exact BOS platform capability bytes a Windows host receives. Windows caches the
    /// capability by content, so changing a byte breaks binding on hosts already paired.
    #[test]
    fn bos_platform_bytes_are_pinned() {
        #[rustfmt::skip]
        const EXPECTED: [u8; 25] = [
            0x00,
            0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,
            0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f,
            0x00, 0x00, 0x03, 0x06,
            0xaa, 0x00,
            0x41,
            0x00,
        ];
        assert_eq!(BOS_PLATFORM, EXPECTED);
    }

    /// frozen: the exact MS OS 2.0 descriptor set returned for `MSOS_VENDOR_CODE`. WinUSB binding
    /// depends on every byte of the blob, not just the fields this crate computes.
    #[test]
    fn ms_os_descriptor_bytes_are_pinned() {
        #[rustfmt::skip]
        const EXPECTED: [u8; 170] = [
            // Set header (10 bytes)
            0x0a, 0x00,
            0x00, 0x00,
            0x00, 0x00, 0x03, 0x06,
            0xaa, 0x00,
            // Function subset header (8 bytes)
            0x08, 0x00,
            0x02, 0x00,
            0x00, 0x00,
            0xa0, 0x00,
            // Compatible ID descriptor (20 bytes)
            0x14, 0x00,
            0x03, 0x00,
            b'W', b'I', b'N', b'U', b'S', b'B', 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            // Registry property descriptor (132 bytes)
            0x84, 0x00,
            0x04, 0x00,
            0x07, 0x00,
            0x2a, 0x00,
            b'D', 0, b'e', 0, b'v', 0, b'i', 0, b'c', 0, b'e', 0,
            b'I', 0, b'n', 0, b't', 0, b'e', 0, b'r', 0, b'f', 0,
            b'a', 0, b'c', 0, b'e', 0, b'G', 0, b'U', 0, b'I', 0,
            b'D', 0, b's', 0, 0, 0,
            0x50, 0x00,
            b'{', 0, b'C', 0, b'D', 0, b'B', 0, b'3', 0, b'B', 0, b'5', 0, b'A', 0,
            b'D', 0, b'-', 0, b'2', 0, b'9', 0, b'3', 0, b'B', 0, b'-', 0, b'4', 0,
            b'6', 0, b'6', 0, b'3', 0, b'-', 0, b'A', 0, b'A', 0, b'3', 0, b'6', 0,
            b'-', 0, b'1', 0, b'A', 0, b'A', 0, b'E', 0, b'4', 0, b'6', 0, b'4', 0,
            b'6', 0, b'3', 0, b'7', 0, b'7', 0, b'6', 0, b'}', 0, 0, 0, 0, 0,
        ];
        assert_eq!(MS_OS_DESCRIPTOR, EXPECTED);
    }

    /// frozen: `bFirstInterface` stays 0, the interface DAP v2 is composed on. Moving DAP v2 without
    /// moving this byte makes WinUSB bind the wrong interface.
    #[test]
    fn ms_os_descriptor_first_interface_is_zero() {
        assert_eq!(MS_OS_DESCRIPTOR[14], 0, "bFirstInterface");
    }

    /// frozen: `wTotalLength` and `wSubsetLength` self-check against the real blob length; a host
    /// walks the set by those fields, so a disagreement is a silent parse failure, not a build error.
    #[test]
    fn ms_os_descriptor_length_fields_self_check() {
        let total = u16::from_le_bytes([MS_OS_DESCRIPTOR[8], MS_OS_DESCRIPTOR[9]]);
        assert_eq!(total as usize, MS_OS_DESCRIPTOR.len());

        let subset_len = u16::from_le_bytes([MS_OS_DESCRIPTOR[16], MS_OS_DESCRIPTOR[17]]);
        // Set header (10 bytes) + subset (header-inclusive) length must exhaust the blob.
        assert_eq!(10 + subset_len as usize, MS_OS_DESCRIPTOR.len());

        // BOS_PLATFORM's advertised length must match the blob, or a host under-reads it.
        let bos_total = u16::from_le_bytes([BOS_PLATFORM[21], BOS_PLATFORM[22]]);
        assert_eq!(bos_total as usize, MS_OS_DESCRIPTOR.len());
    }
}
