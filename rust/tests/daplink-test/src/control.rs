//! Control-transfer path: device-recipient requests and BOS walking, for descriptor-level
//! assertions the harness's bulk paths cannot reach.

use std::time::Duration;

use nusb::transfer::{ControlIn, ControlType, Recipient};
use nusb::{Device, MaybeFuture};
use probe_rs::probe::DebugProbeSelector;

/// Every request here is answered from a constant blob, so a slow answer is a fault, not load.
const TIMEOUT: Duration = Duration::from_secs(1);

/// `bDescriptorType` of the Binary Object Store and of a device capability within it.
const DESC_BOS: u8 = 0x0f;
const DESC_DEVICE_CAPABILITY: u8 = 0x10;

/// `bDevCapabilityType` of a platform capability, the form both vendor capabilities take.
pub(crate) const CAP_PLATFORM: u8 = 0x05;

/// Open the probe named by `selector`; claims no interface, so this does not disturb a case
/// that later opens the probe for bulk transfers.
pub(crate) fn open(selector: &DebugProbeSelector) -> Result<Device, String> {
    nusb::list_devices()
        .wait()
        .map_err(|e| e.to_string())?
        .find(|d| {
            d.vendor_id() == selector.vendor_id
                && d.product_id() == selector.product_id
                && selector
                    .serial_number
                    .as_deref()
                    .is_none_or(|s| d.serial_number() == Some(s))
        })
        .ok_or_else(|| "probe not found on the bus".to_string())?
        .open()
        .wait()
        .map_err(|e| e.to_string())
}

/// Read the device's BOS descriptor.
pub(crate) fn bos(device: &Device) -> Result<Vec<u8>, String> {
    device
        .get_descriptor(DESC_BOS, 0, 0, TIMEOUT)
        .wait()
        .map_err(|e| e.to_string())
}

/// A vendor IN request at `recipient`; the recipient is a parameter because a device-recipient
/// request answered at another recipient is a defect a happy-path read cannot see.
pub(crate) fn vendor_in(
    device: &Device,
    recipient: Recipient,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
) -> Result<Vec<u8>, String> {
    device
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient,
                request,
                value,
                index,
                length,
            },
            TIMEOUT,
        )
        .wait()
        .map_err(|e| e.to_string())
}

/// One capability walked out of a BOS: its type byte and the bytes after it.
#[derive(Debug)]
pub(crate) struct Capability<'a> {
    pub(crate) ty: u8,
    pub(crate) payload: &'a [u8],
}

impl Capability<'_> {
    /// The UUID of a platform capability, whose payload is `bReserved`, a 16-byte UUID, then
    /// capability-specific data.
    pub(crate) fn platform_uuid(&self) -> Option<&[u8]> {
        (self.ty == CAP_PLATFORM && self.payload.len() >= 17).then(|| &self.payload[1..17])
    }
}

/// Walk a BOS into its capabilities, checking that the header's length and capability count
/// agree with what is actually there -- a host walks by those fields, so a disagreement is a
/// silent truncation rather than a parse error.
pub(crate) fn capabilities(bos: &[u8]) -> Result<Vec<Capability<'_>>, String> {
    if bos.len() < 5 {
        return Err(format!(
            "BOS is {} bytes, shorter than its header",
            bos.len()
        ));
    }
    if bos[0] != 5 || bos[1] != DESC_BOS {
        return Err(format!(
            "BOS header is bLength {} type 0x{:02x}, expected 5 and 0x{DESC_BOS:02x}",
            bos[0], bos[1]
        ));
    }
    let total = u16::from_le_bytes([bos[2], bos[3]]) as usize;
    if total != bos.len() {
        return Err(format!(
            "BOS wTotalLength {total}, but the device returned {} bytes",
            bos.len()
        ));
    }

    let mut caps = Vec::new();
    let mut at = 5;
    while at < total {
        let len = bos[at] as usize;
        if len < 3 || at + len > total {
            return Err(format!(
                "capability {} at offset {at} has bLength {len}, past the {total}-byte BOS",
                caps.len()
            ));
        }
        if bos[at + 1] != DESC_DEVICE_CAPABILITY {
            return Err(format!(
                "capability {} at offset {at} is type 0x{:02x}, expected \
                 0x{DESC_DEVICE_CAPABILITY:02x}",
                caps.len(),
                bos[at + 1]
            ));
        }
        caps.push(Capability {
            ty: bos[at + 2],
            payload: &bos[at + 3..at + len],
        });
        at += len;
    }

    let declared = bos[4] as usize;
    if declared != caps.len() {
        return Err(format!(
            "BOS bNumDeviceCaps {declared}, but {} capabilities fill it",
            caps.len()
        ));
    }
    Ok(caps)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A BOS holding one platform capability with `uuid` and `data`.
    fn bos_with(uuid: [u8; 16], data: &[u8]) -> Vec<u8> {
        let cap_len = 3 + 1 + 16 + data.len();
        let total = 5 + cap_len;
        let mut out = vec![5, DESC_BOS, total as u8, 0, 1];
        out.extend_from_slice(&[cap_len as u8, DESC_DEVICE_CAPABILITY, CAP_PLATFORM, 0x00]);
        out.extend_from_slice(&uuid);
        out.extend_from_slice(data);
        out
    }

    #[test]
    fn walks_a_platform_capability() {
        let bos = bos_with([0xaa; 16], &[0x01, 0x02]);
        let caps = capabilities(&bos).expect("well-formed BOS walks");
        assert_eq!(caps.len(), 1);
        assert_eq!(caps[0].ty, CAP_PLATFORM);
        assert_eq!(caps[0].platform_uuid(), Some(&[0xaa; 16][..]));
        assert_eq!(&caps[0].payload[17..], &[0x01, 0x02]);
    }

    #[test]
    fn walks_several_capabilities_in_order() {
        let mut bos = bos_with([0x11; 16], &[]);
        let second = bos_with([0x22; 16], &[]);
        bos.extend_from_slice(&second[5..]);
        bos[2] = bos.len() as u8;
        bos[4] = 2;
        let caps = capabilities(&bos).expect("two capabilities walk");
        assert_eq!(caps[0].platform_uuid(), Some(&[0x11; 16][..]));
        assert_eq!(caps[1].platform_uuid(), Some(&[0x22; 16][..]));
    }

    #[test]
    fn rejects_a_total_length_the_read_disagrees_with() {
        let mut bos = bos_with([0xaa; 16], &[]);
        bos[2] += 2;
        assert!(capabilities(&bos).unwrap_err().contains("wTotalLength"));
    }

    #[test]
    fn rejects_a_capability_count_the_contents_disagree_with() {
        let mut bos = bos_with([0xaa; 16], &[]);
        bos[4] = 3;
        assert!(capabilities(&bos).unwrap_err().contains("bNumDeviceCaps"));
    }

    #[test]
    fn rejects_a_capability_running_past_the_bos() {
        let mut bos = bos_with([0xaa; 16], &[]);
        bos[5] += 4;
        assert!(capabilities(&bos).unwrap_err().contains("bLength"));
    }

    #[test]
    fn rejects_a_non_capability_descriptor_inside_the_bos() {
        let mut bos = bos_with([0xaa; 16], &[]);
        bos[6] = 0x0b;
        assert!(capabilities(&bos).unwrap_err().contains("expected"));
    }

    #[test]
    fn rejects_a_truncated_header() {
        assert!(capabilities(&[5, DESC_BOS, 5])
            .unwrap_err()
            .contains("header"));
    }

    #[test]
    fn a_non_platform_capability_has_no_uuid() {
        let cap = Capability {
            ty: 0x02,
            payload: &[0; 20],
        };
        assert_eq!(cap.platform_uuid(), None);
    }
}
