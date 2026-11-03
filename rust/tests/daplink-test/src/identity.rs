//! The identity case: USB enumeration identity from the bus listing, then the MS OS 2.0 vendor descriptors.

use daplink_core::serial::SERIAL_LEN;
use daplink_core::{winusb, DAPLINK_PID, DAPLINK_VID};
use nusb::transfer::Recipient;

use crate::util::hex;
use crate::{control, Ctx, Failure};

/// `wIndex` of the MS OS 2.0 descriptor-set request; both adapters match on this literal.
const MSOS_DESCRIPTOR_INDEX: u16 = 0x07;

/// Expected product-string prefix; firmwares append their board name.
const PRODUCT_PREFIX: &str =
    daplink_core::DapConfig::daplink("", daplink_core::DapQueue::<1, 64>::endpoint_size::<64>())
        .product;

pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let device = crate::dap::find_device(&cx.selector).map_err(Failure::Fatal)?;

    if (device.vendor_id(), device.product_id()) != (DAPLINK_VID, DAPLINK_PID) {
        return Err(Failure::Failed(format!(
            "VID:PID {:04x}:{:04x}, expected {DAPLINK_VID:04x}:{DAPLINK_PID:04x}",
            device.vendor_id(),
            device.product_id()
        )));
    }

    match device.product_string() {
        Some(p) if p.starts_with(PRODUCT_PREFIX) => {}
        Some(p) => {
            return Err(Failure::Failed(format!(
                "product string {p:?}, expected one starting {PRODUCT_PREFIX:?}"
            )))
        }
        None => return Err(Failure::Failed("no product string on the device".into())),
    }

    // The value is per unit; the BOT format (uppercase hex, at least 12 digits) is asserted.
    match device.serial_number() {
        Some(s) if s.len() != SERIAL_LEN => {
            return Err(Failure::Failed(format!(
                "serial {s:?} is {} characters, expected {SERIAL_LEN}",
                s.len()
            )))
        }
        Some(s)
            if !s
                .bytes()
                .all(|c| c.is_ascii_digit() || (b'A'..=b'F').contains(&c)) =>
        {
            return Err(Failure::Failed(format!(
                "serial {s:?} is not uppercase hexadecimal (mass-storage BOT 4.1.2)"
            )))
        }
        Some(_) => {}
        None => return Err(Failure::Failed("probe reports no serial number".into())),
    }

    descriptors(cx)
}

/// The vendor descriptors, checked against the `daplink-core` constants; silence is a failure.
fn descriptors(cx: &Ctx) -> Result<(), Failure> {
    let device = crate::dap::open_device(&cx.selector)
        .map_err(|e| Failure::Failed(format!("opening the probe for control transfers: {e}")))?;
    let bos =
        control::bos(&device).map_err(|e| Failure::Failed(format!("BOS read failed: {e}")))?;
    let caps = control::capabilities(&bos).map_err(Failure::Failed)?;

    // Located by UUID, never by position: USB stacks prepend capabilities of their own.
    let locate = |name: &str, expected: &[u8]| {
        caps.iter()
            .position(|c| c.platform_uuid() == Some(&expected[1..17]))
            .ok_or_else(|| {
                Failure::Failed(format!(
                    "no {name} platform capability in the BOS: {}",
                    hex(&bos)
                ))
            })
    };
    let msos = locate("MS OS 2.0", &winusb::BOS_PLATFORM)?;

    // The vendor code first, so a mismatch names the field rather than a byte offset.
    let msos_code = caps[msos].payload.get(23).copied();
    if msos_code != Some(winusb::MSOS_VENDOR_CODE) {
        return Err(Failure::Failed(format!(
            "MS OS 2.0 bMS_VendorCode {msos_code:?}, expected {:#04x}",
            winusb::MSOS_VENDOR_CODE
        )));
    }

    if caps[msos].payload != winusb::BOS_PLATFORM {
        return Err(Failure::Failed(format!(
            "MS OS 2.0 platform capability is {}, expected {}",
            hex(caps[msos].payload),
            hex(&winusb::BOS_PLATFORM)
        )));
    }

    // Negative control: a request at another recipient must not be answered.
    if let Ok(answer) = control::vendor_in(
        &device,
        Recipient::Interface,
        winusb::MSOS_VENDOR_CODE,
        0,
        MSOS_DESCRIPTOR_INDEX,
        winusb::MS_OS_DESCRIPTOR.len() as u16,
    ) {
        return Err(Failure::Failed(format!(
            "the MS OS 2.0 descriptor-set request at the interface recipient answered {}, expected a stall",
            hex(&answer)
        )));
    }

    let set = control::vendor_in(
        &device,
        Recipient::Device,
        winusb::MSOS_VENDOR_CODE,
        0,
        MSOS_DESCRIPTOR_INDEX,
        winusb::MS_OS_DESCRIPTOR.len() as u16,
    )
    .map_err(|e| Failure::Failed(format!("MS OS 2.0 descriptor-set read failed: {e}")))?;
    if set != winusb::MS_OS_DESCRIPTOR {
        return Err(Failure::Failed(format!(
            "MS OS 2.0 descriptor set is {}, expected {}",
            hex(&set),
            hex(&winusb::MS_OS_DESCRIPTOR)
        )));
    }

    // Parse the set structurally against the interface found on the bus, so a wrong field fails named.
    if let Some(dap_interface) = vendor_interface_number(&device) {
        parse_msos_set(&set, dap_interface).map_err(Failure::Failed)?;
    }
    Ok(())
}

/// The CMSIS-DAP v2 vendor interface's number, or `None` for a product with no vendor interface.
fn vendor_interface_number(device: &nusb::Device) -> Option<u8> {
    /// Vendor-specific interface class, as CMSIS-DAP v2 advertises.
    const CLASS_VENDOR: u8 = 0xFF;
    device
        .active_configuration()
        .ok()?
        .interfaces()
        .map(|group| group.first_alt_setting())
        .find(|alt| alt.class() == CLASS_VENDOR)
        .map(|alt| alt.interface_number())
}

/// One length-prefixed descriptor inside an MS OS 2.0 set.
struct Descriptor<'a> {
    kind: u16,
    body: &'a [u8],
}

/// Read one descriptor at `at`, returning it and the offset just past it.
fn next_descriptor<'a>(
    set: &'a [u8],
    at: usize,
    name: &str,
) -> Result<(Descriptor<'a>, usize), String> {
    let len = read_u16(set, at, &format!("{name} wLength"))? as usize;
    if len < 4 || at + len > set.len() {
        return Err(format!(
            "{name} wLength {len} runs past the {}-byte descriptor set",
            set.len()
        ));
    }
    let kind = read_u16(set, at + 2, &format!("{name} wDescriptorType"))?;
    Ok((
        Descriptor {
            kind,
            body: &set[at + 4..at + len],
        },
        at + len,
    ))
}

/// A little-endian `u16` at `at`, named for the error it produces when `bytes` is too short.
fn read_u16(bytes: &[u8], at: usize, field: &str) -> Result<u16, String> {
    bytes
        .get(at..at + 2)
        .map(|s| u16::from_le_bytes([s[0], s[1]]))
        .ok_or_else(|| format!("{field} runs past the end of its descriptor"))
}

/// Decode a UTF-16LE byte string, the encoding every MS OS 2.0 string field uses.
fn utf16le(bytes: &[u8]) -> Result<String, String> {
    if !bytes.len().is_multiple_of(2) {
        return Err(format!(
            "UTF-16LE field is {} bytes, an odd length",
            bytes.len()
        ));
    }
    let units: Vec<u16> = bytes
        .chunks_exact(2)
        .map(|c| u16::from_le_bytes([c[0], c[1]]))
        .collect();
    String::from_utf16(&units).map_err(|_| "UTF-16LE field is not valid UTF-16".to_string())
}

/// Whether `s` is a `{8-4-4-4-12}` GUID string, the shape a `DeviceInterfaceGUIDs` value needs.
fn is_guid_string(s: &str) -> bool {
    const GROUP_LENS: [usize; 5] = [8, 4, 4, 4, 12];
    let Some(inner) = s.strip_prefix('{').and_then(|s| s.strip_suffix('}')) else {
        return false;
    };
    let groups: Vec<&str> = inner.split('-').collect();
    groups.len() == GROUP_LENS.len()
        && groups
            .iter()
            .zip(GROUP_LENS)
            .all(|(g, len)| g.len() == len && g.bytes().all(|c| c.is_ascii_hexdigit()))
}

/// A structural parse of the MS OS 2.0 set against `dap_interface`, the bus's CMSIS-DAP v2 interface.
fn parse_msos_set(set: &[u8], dap_interface: u8) -> Result<(), String> {
    let (header, at) = next_descriptor(set, 0, "set header")?;
    if header.kind != 0x0000 {
        return Err(format!(
            "set header wDescriptorType {:#06x}, expected 0x0000",
            header.kind
        ));
    }
    let total_len = read_u16(header.body, 4, "set header wTotalLength")?;
    if total_len as usize != set.len() {
        return Err(format!(
            "set header wTotalLength {total_len}, but the descriptor set is {} bytes",
            set.len()
        ));
    }

    let (subset, at) = next_descriptor(set, at, "function subset header")?;
    if subset.kind != 0x0002 {
        return Err(format!(
            "function subset header wDescriptorType {:#06x}, expected 0x0002",
            subset.kind
        ));
    }
    let first_interface = *subset
        .body
        .first()
        .ok_or("function subset header has no bFirstInterface byte")?;
    if first_interface != dap_interface {
        return Err(format!(
            "function subset header bFirstInterface {first_interface}, expected {dap_interface} \
             (the CMSIS-DAP v2 interface found on the bus)"
        ));
    }

    let (compat, at) = next_descriptor(set, at, "compatible-id descriptor")?;
    if compat.kind != 0x0003 {
        return Err(format!(
            "compatible-id descriptor wDescriptorType {:#06x}, expected 0x0003",
            compat.kind
        ));
    }
    if compat.body.get(..6) != Some(b"WINUSB") {
        return Err(format!(
            "compatible-id descriptor CompatibleID {:?}, expected \"WINUSB\"",
            String::from_utf8_lossy(compat.body.get(..8).unwrap_or(compat.body))
        ));
    }

    let (prop, _) = next_descriptor(set, at, "registry property descriptor")?;
    if prop.kind != 0x0004 {
        return Err(format!(
            "registry property descriptor wDescriptorType {:#06x}, expected 0x0004",
            prop.kind
        ));
    }
    let data_type = read_u16(prop.body, 0, "registry property wPropertyDataType")?;
    if data_type != 7 {
        return Err(format!(
            "registry property wPropertyDataType {data_type}, expected 7 (REG_MULTI_SZ)"
        ));
    }
    let name_len = read_u16(prop.body, 2, "registry property wPropertyNameLength")? as usize;
    let name_bytes = prop
        .body
        .get(4..4 + name_len)
        .ok_or("registry property name runs past the descriptor")?;
    let name = utf16le(name_bytes)?;
    if name.trim_end_matches('\0') != "DeviceInterfaceGUIDs" {
        return Err(format!(
            "registry property name {name:?}, expected \"DeviceInterfaceGUIDs\""
        ));
    }
    let data_len_at = 4 + name_len;
    let data_len = read_u16(
        prop.body,
        data_len_at,
        "registry property wPropertyDataLength",
    )? as usize;
    let data_bytes = prop
        .body
        .get(data_len_at + 2..data_len_at + 2 + data_len)
        .ok_or("registry property value runs past the descriptor")?;
    let guid = utf16le(data_bytes)?;
    if !is_guid_string(guid.trim_end_matches('\0')) {
        return Err(format!(
            "registry property value {guid:?} is not a GUID string"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipping constant parses structurally, not only byte for byte.
    #[test]
    fn the_frozen_constant_parses() {
        parse_msos_set(&winusb::MS_OS_DESCRIPTOR, 0).expect("the shipping descriptor set parses");
    }

    #[test]
    fn a_wrong_first_interface_fails_named() {
        let mut set = winusb::MS_OS_DESCRIPTOR;
        set[14] = 1; // bFirstInterface
        let err = parse_msos_set(&set, 0).unwrap_err();
        assert!(err.contains("bFirstInterface"), "{err}");
    }

    #[test]
    fn a_wrong_compatible_id_fails_named() {
        let mut set = winusb::MS_OS_DESCRIPTOR;
        set[22] = b'X'; // first byte of CompatibleID "WINUSB"
        let err = parse_msos_set(&set, 0).unwrap_err();
        assert!(err.contains("CompatibleID"), "{err}");
    }

    #[test]
    fn a_wrong_registry_data_type_fails_named() {
        let mut set = winusb::MS_OS_DESCRIPTOR;
        set[42] = 1; // wPropertyDataType low byte, REG_MULTI_SZ (7) otherwise
        let err = parse_msos_set(&set, 0).unwrap_err();
        assert!(err.contains("wPropertyDataType"), "{err}");
    }

    #[test]
    fn a_wrong_registry_name_fails_named() {
        let mut set = winusb::MS_OS_DESCRIPTOR;
        set[46] = b'X'; // first byte of the "DeviceInterfaceGUIDs" UTF-16LE name
        let err = parse_msos_set(&set, 0).unwrap_err();
        assert!(err.contains("registry property name"), "{err}");
    }

    #[test]
    fn a_wrong_guid_fails_named() {
        let mut set = winusb::MS_OS_DESCRIPTOR;
        set[90] = b'X'; // opening brace of the GUID string
        let err = parse_msos_set(&set, 0).unwrap_err();
        assert!(err.contains("GUID"), "{err}");
    }
}
