//! The identity case: the probe's USB enumeration identity -- VID:PID, product string, serial --
//! asserted from the bus listing before anything opens the probe, then the vendor descriptors a
//! host reads off it: the BOS platform capabilities, the WebUSB landing page, and the MS OS 2.0
//! descriptor set.

use daplink_core::serial::{SERIAL_LEN, SERIAL_PREFIX};
use daplink_core::{webusb, winusb, DAPLINK_PID, DAPLINK_VID};
use nusb::transfer::Recipient;
use nusb::MaybeFuture;

use crate::util::hex;
use crate::{control, Ctx, Failure};

/// `wIndex` of the MS OS 2.0 descriptor-set request; both adapters match on this literal.
const MSOS_DESCRIPTOR_INDEX: u16 = 0x07;

/// Expected leading text of the product string, from the shipping identity so this case tracks
/// it; firmwares append their board name, so this is a prefix check, not an equality one.
const PRODUCT_PREFIX: &str = daplink_core::DapConfig::daplink("").product;

pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let device = nusb::list_devices()
        .wait()
        .map_err(|e| Failure::Failed(e.to_string()))?
        .find(|d| {
            d.vendor_id() == cx.selector.vendor_id
                && d.product_id() == cx.selector.product_id
                && cx
                    .selector
                    .serial_number
                    .as_deref()
                    .is_none_or(|s| d.serial_number() == Some(s))
        })
        .ok_or_else(|| Failure::Fatal("probe not found on the bus".into()))?;

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

    // The value is per unit; the format is asserted because the mass-storage Bulk-Only
    // Transport rules bind it (uppercase hex, at least 12 digits).
    let prefix = core::str::from_utf8(SERIAL_PREFIX).expect("prefix is ASCII");
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
        Some(s) if !s.starts_with(prefix) => {
            return Err(Failure::Failed(format!(
                "serial {s:?} does not start with the fixed prefix {prefix:?}"
            )))
        }
        Some(_) => {}
        None => return Err(Failure::Failed("probe reports no serial number".into())),
    }

    descriptors(cx)
}

/// The vendor descriptors, expected against the `daplink-core` constants both USB adapters
/// compose. Neither adapter gates them, so an image that stays silent is defective and this
/// fails rather than skipping.
fn descriptors(cx: &Ctx) -> Result<(), Failure> {
    let device = control::open(&cx.selector)
        .map_err(|e| Failure::Failed(format!("opening the probe for control transfers: {e}")))?;
    let bos =
        control::bos(&device).map_err(|e| Failure::Failed(format!("BOS read failed: {e}")))?;
    let caps = control::capabilities(&bos).map_err(Failure::Failed)?;

    // Located by UUID, never by position: the USB stack prepends capabilities of its own, and
    // the two adapters need not prepend the same ones.
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
    let web = locate("WebUSB", &webusb::BOS_PLATFORM)?;
    let msos = locate("MS OS 2.0", &winusb::BOS_PLATFORM)?;

    // The vendor codes first, so a mismatch names the field rather than a byte offset. Sharing
    // one code makes both vendor requests ambiguous while every descriptor still parses.
    let code = |at: usize, payload: &[u8]| payload.get(at).copied();
    let web_code = code(19, caps[web].payload);
    let msos_code = code(23, caps[msos].payload);
    if web_code != Some(webusb::VENDOR_CODE) {
        return Err(Failure::Failed(format!(
            "WebUSB bVendorCode {web_code:?}, expected {:#04x}",
            webusb::VENDOR_CODE
        )));
    }
    if msos_code != Some(winusb::MSOS_VENDOR_CODE) {
        return Err(Failure::Failed(format!(
            "MS OS 2.0 bMS_VendorCode {msos_code:?}, expected {:#04x}",
            winusb::MSOS_VENDOR_CODE
        )));
    }

    for (name, payload, expected) in [
        ("WebUSB", caps[web].payload, &webusb::BOS_PLATFORM[..]),
        ("MS OS 2.0", caps[msos].payload, &winusb::BOS_PLATFORM[..]),
    ] {
        if payload != expected {
            return Err(Failure::Failed(format!(
                "{name} platform capability is {}, expected {}",
                hex(payload),
                hex(expected)
            )));
        }
    }

    // Both adapters register WebUSB first, and a host caches the capability it read; the wire
    // order is the guarantee, not the offsets, which the stack's own capabilities shift.
    if web > msos {
        return Err(Failure::Failed(format!(
            "BOS lists MS OS 2.0 at capability {msos} before WebUSB at {web}"
        )));
    }

    let url = control::vendor_in(
        &device,
        Recipient::Device,
        webusb::VENDOR_CODE,
        webusb::LANDING_URL_INDEX as u16,
        webusb::REQUEST_GET_URL,
        u8::MAX as u16,
    )
    .map_err(|e| Failure::Failed(format!("GET_URL failed: {e}")))?;
    if url != webusb::LANDING_URL_DESCRIPTOR {
        return Err(Failure::Failed(format!(
            "GET_URL returned {}, expected {}",
            hex(&url),
            hex(&webusb::LANDING_URL_DESCRIPTOR)
        )));
    }

    // Negative control: both adapters guard on the device recipient, and a browser issues the
    // request there. Answering it at another recipient passes the happy path unnoticed.
    if let Ok(answer) = control::vendor_in(
        &device,
        Recipient::Interface,
        webusb::VENDOR_CODE,
        webusb::LANDING_URL_INDEX as u16,
        webusb::REQUEST_GET_URL,
        u8::MAX as u16,
    ) {
        return Err(Failure::Failed(format!(
            "GET_URL at the interface recipient answered {}, expected a stall",
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
    Ok(())
}
