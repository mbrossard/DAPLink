//! WinUSB (MS OS 2.0) and WebUSB descriptors, so a host claims the CMSIS-DAP v2 interface
//! without a driver and a browser can reach the landing page.

use daplink_rs::daplink_core::webusb;
use daplink_rs::daplink_core::winusb::{
    BOS_CAPABILITY_PLATFORM, BOS_PLATFORM, MSOS_VENDOR_CODE, MS_OS_DESCRIPTOR,
};
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, Request, RequestType};

/// The device's vendor-descriptor surface as one endpoint-less USB class.
pub struct VendorDescriptors;

impl<B: UsbBus> UsbClass<B> for VendorDescriptors {
    fn get_bos_descriptors(&self, writer: &mut BosWriter) -> usb_device::Result<()> {
        // WebUSB before MS OS 2.0, the order the async adapter registers them in.
        writer.capability(BOS_CAPABILITY_PLATFORM, &webusb::BOS_PLATFORM)?;
        writer.capability(BOS_CAPABILITY_PLATFORM, &BOS_PLATFORM)
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        if is_ms_os_descriptor_request(req) {
            xfer.accept_with_static(&MS_OS_DESCRIPTOR).ok();
        } else if is_webusb_url_request(req) {
            xfer.accept_with_static(&webusb::LANDING_URL_DESCRIPTOR)
                .ok();
        }
    }
}

/// Whether `req` is the MS OS 2.0 descriptor-set request; its code and index need the recipient
/// to be unique.
fn is_ms_os_descriptor_request(req: &Request) -> bool {
    req.request_type == RequestType::Vendor
        && req.recipient == Recipient::Device
        && req.request == MSOS_VENDOR_CODE
        && req.index == 0x07
}

/// Whether `req` is a browser's WebUSB GET_URL request for the landing page.
fn is_webusb_url_request(req: &Request) -> bool {
    req.request_type == RequestType::Vendor
        && req.recipient == Recipient::Device
        && webusb::is_get_url_request(req.request, req.value, req.index)
}
