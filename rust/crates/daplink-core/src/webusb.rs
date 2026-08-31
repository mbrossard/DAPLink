//! WebUSB BOS capability and landing-page URL descriptor shared by the polled and async USB adapters.

/// Vendor request code a browser issues to fetch a WebUSB URL descriptor.
pub const VENDOR_CODE: u8 = 0x42;

/// `wIndex` of GET_URL, the only WebUSB request there is.
pub const REQUEST_GET_URL: u16 = 0x02;

/// URL descriptor index of the landing page; 0 would mean the device has no landing page at all.
pub const LANDING_URL_INDEX: u8 = 1;

/// Landing page without its scheme -- the URL descriptor carries that as its own `bScheme` byte.
pub const LANDING_URL: &str = "daplink.io";

/// URL descriptor for [`LANDING_URL`]: `bLength`, `bDescriptorType` (`0x03`), `bScheme`, bare ASCII.
pub const LANDING_URL_DESCRIPTOR: [u8; 3 + LANDING_URL.len()] = landing_url_descriptor(0x01);

/// Builds [`LANDING_URL_DESCRIPTOR`]; a parameter cannot appear in a `const fn`'s own return type.
const fn landing_url_descriptor(scheme: u8) -> [u8; 3 + LANDING_URL.len()] {
    const LEN: usize = 3 + LANDING_URL.len();
    assert!(LEN <= u8::MAX as usize, "URL descriptor exceeds bLength");
    let url = LANDING_URL.as_bytes();
    let mut out = [0u8; LEN];
    out[0] = LEN as u8;
    out[1] = 0x03;
    out[2] = scheme;
    let mut i = 0;
    while i < url.len() {
        out[3 + i] = url[i];
        i += 1;
    }
    out
}

/// BOS WebUSB platform capability payload; points a browser at [`VENDOR_CODE`] and the landing page.
#[rustfmt::skip]
pub const BOS_PLATFORM: [u8; 21] = [
    0x00, // reserved
    // WebUSB platform capability UUID {3408B638-09A9-47A0-8BFD-A0768815B665}
    0x38, 0xb6, 0x08, 0x34, 0xa9, 0x09, 0xa0, 0x47,
    0x8b, 0xfd, 0xa0, 0x76, 0x88, 0x15, 0xb6, 0x65,
    0x00, 0x01,        // bcdVersion = 1.00
    VENDOR_CODE,       // bVendorCode
    LANDING_URL_INDEX, // iLandingPage
];

/// Whether these control-request fields name the landing page's GET_URL; callers check the recipient.
pub const fn is_get_url_request(request: u8, value: u16, index: u16) -> bool {
    request == VENDOR_CODE && index == REQUEST_GET_URL && value == LANDING_URL_INDEX as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::winusb::{BOS_CAPABILITY_PLATFORM, MSOS_VENDOR_CODE};

    /// frozen: the exact BOS WebUSB capability bytes a browser reads before asking for the URL.
    #[test]
    fn bos_platform_bytes_are_pinned() {
        #[rustfmt::skip]
        const EXPECTED: [u8; 21] = [
            0x00,
            0x38, 0xb6, 0x08, 0x34, 0xa9, 0x09, 0xa0, 0x47,
            0x8b, 0xfd, 0xa0, 0x76, 0x88, 0x15, 0xb6, 0x65,
            0x00, 0x01,
            0x42,
            0x01,
        ];
        assert_eq!(BOS_PLATFORM, EXPECTED);
    }

    /// frozen: the exact URL descriptor returned for [`VENDOR_CODE`], scheme byte included.
    #[test]
    fn landing_url_descriptor_bytes_are_pinned() {
        #[rustfmt::skip]
        const EXPECTED: [u8; 13] = [
            0x0d, 0x03, 0x01,
            b'd', b'a', b'p', b'l', b'i', b'n', b'k', b'.', b'i', b'o',
        ];
        assert_eq!(LANDING_URL_DESCRIPTOR, EXPECTED);
    }

    /// frozen: the capability sequence a host reads off the BOS, WebUSB first, then MS OS 2.0. Both
    /// adapters register in this order and each side caches the capability it read.
    #[test]
    fn bos_capability_sequence_is_pinned() {
        let mut bos = [0u8; 52];
        let mut at = 0;
        for payload in [&BOS_PLATFORM[..], &crate::winusb::BOS_PLATFORM[..]] {
            bos[at] = (payload.len() + 3) as u8;
            bos[at + 1] = 0x10; // bDescriptorType = DEVICE_CAPABILITY
            bos[at + 2] = BOS_CAPABILITY_PLATFORM;
            bos[at + 3..at + 3 + payload.len()].copy_from_slice(payload);
            at += 3 + payload.len();
        }
        assert_eq!(at, bos.len());

        #[rustfmt::skip]
        const EXPECTED: [u8; 52] = [
            0x18, 0x10, 0x05,
            0x00,
            0x38, 0xb6, 0x08, 0x34, 0xa9, 0x09, 0xa0, 0x47,
            0x8b, 0xfd, 0xa0, 0x76, 0x88, 0x15, 0xb6, 0x65,
            0x00, 0x01,
            0x42,
            0x01,
            0x1c, 0x10, 0x05,
            0x00,
            0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,
            0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f,
            0x00, 0x00, 0x03, 0x06,
            0xaa, 0x00,
            0x41,
            0x00,
        ];
        assert_eq!(bos, EXPECTED);
    }

    /// The capability must name a URL index this device serves, and a vendor code MS OS 2.0 does not
    /// use -- a shared code makes both requests ambiguous, and the descriptor still parses.
    #[test]
    fn the_capability_points_at_the_url_this_device_serves() {
        assert_ne!(VENDOR_CODE, MSOS_VENDOR_CODE);
        assert_eq!(BOS_PLATFORM[19], VENDOR_CODE, "bVendorCode");
        assert_eq!(BOS_PLATFORM[20], LANDING_URL_INDEX, "iLandingPage");
        assert_ne!(LANDING_URL_INDEX, 0, "0 would mean: no landing page");

        assert_eq!(
            LANDING_URL_DESCRIPTOR[0] as usize,
            LANDING_URL_DESCRIPTOR.len()
        );
        assert_eq!(
            LANDING_URL_DESCRIPTOR[1], 0x03,
            "bDescriptorType = WEBUSB_URL"
        );
        assert_eq!(LANDING_URL_DESCRIPTOR[2], 0x01, "bScheme = https");
        assert_eq!(
            core::str::from_utf8(&LANDING_URL_DESCRIPTOR[3..]).unwrap(),
            LANDING_URL
        );
    }

    /// The matcher answers only the landing page, and never the MS OS 2.0 descriptor-set request.
    #[test]
    fn matcher_accepts_only_the_landing_page_request() {
        assert!(is_get_url_request(
            VENDOR_CODE,
            LANDING_URL_INDEX as u16,
            REQUEST_GET_URL
        ));
        assert!(!is_get_url_request(MSOS_VENDOR_CODE, 0, 0x07));
        assert!(!is_get_url_request(VENDOR_CODE, 0, REQUEST_GET_URL));
        assert!(!is_get_url_request(VENDOR_CODE, 2, REQUEST_GET_URL));
        assert!(!is_get_url_request(VENDOR_CODE, 1, 0x07));
    }
}
