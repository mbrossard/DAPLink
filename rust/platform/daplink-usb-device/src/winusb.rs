//! WinUSB (MS OS 2.0) descriptors, so a host claims the CMSIS-DAP v2 interface without a driver.

use daplink_rs::daplink_core::winusb::{
    BOS_CAPABILITY_PLATFORM, BOS_PLATFORM, MSOS_VENDOR_CODE, MS_OS_DESCRIPTOR,
};
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, Request, RequestType};

/// Endpoint-register repairs for a bus driver that keeps the data toggle across a halt clear.
pub trait EndpointRepair: Sync {
    /// Whether the controller has a register for `ep`; requests naming any other are refused.
    fn has(&self, ep: EndpointAddress) -> bool;
    /// Clear any halt on `ep` and set its data toggle to DATA0.
    fn clear_halt(&self, ep: EndpointAddress);
}

/// A controller with endpoint registers `0..N` whose bus already resets the toggle on a clear.
pub struct RegisterCount<const N: usize>;

impl<const N: usize> EndpointRepair for RegisterCount<N> {
    fn has(&self, ep: EndpointAddress) -> bool {
        ep.index() < N
    }

    fn clear_halt(&self, _ep: EndpointAddress) {}
}

/// The device's vendor-descriptor surface as one endpoint-less USB class.
#[derive(Default)]
pub struct VendorDescriptors {
    repair: Option<&'static dyn EndpointRepair>,
}

impl VendorDescriptors {
    /// Run `repair` ahead of usb-device's own endpoint-halt and configuration handling.
    pub const fn with_repair(repair: &'static dyn EndpointRepair) -> Self {
        Self {
            repair: Some(repair),
        }
    }
}

impl<B: UsbBus> UsbClass<B> for VendorDescriptors {
    fn get_bos_descriptors(&self, writer: &mut BosWriter) -> usb_device::Result<()> {
        writer.capability(BOS_CAPABILITY_PLATFORM, &BOS_PLATFORM)
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = *xfer.request();
        if is_ms_os_descriptor_request(&req) {
            xfer.accept_with_static(&MS_OS_DESCRIPTOR).ok();
        } else if let (Some(repair), Some(ep)) = (self.repair, endpoint_request(&req)) {
            if req.request == Request::GET_STATUS && !ep.is_some_and(|ep| repair.has(ep)) {
                xfer.reject().ok();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let Some(repair) = self.repair else {
            return;
        };
        let req = *xfer.request();
        if let Some(ep) = endpoint_request(&req) {
            let halt = req.value == Request::FEATURE_ENDPOINT_HALT
                && matches!(req.request, Request::CLEAR_FEATURE | Request::SET_FEATURE);
            match ep.filter(|&ep| repair.has(ep)) {
                None if halt => {
                    xfer.reject().ok();
                }
                Some(ep) if halt && req.request == Request::CLEAR_FEATURE && ep.index() != 0 => {
                    repair.clear_halt(ep);
                }
                _ => {}
            }
        } else if req.request_type == RequestType::Standard
            && req.recipient == Recipient::Device
            && req.request == Request::SET_CONFIGURATION
            && req.value == 1
        {
            // A new configuration starts every endpoint at DATA0.
            for index in 1..16u8 {
                for ep in [
                    EndpointAddress::from(index),
                    EndpointAddress::from(index | 0x80),
                ] {
                    if repair.has(ep) {
                        repair.clear_halt(ep);
                    }
                }
            }
        }
    }
}

/// A standard endpoint-recipient request's endpoint: `Some(None)` when wIndex names none.
fn endpoint_request(req: &Request) -> Option<Option<EndpointAddress>> {
    (req.request_type == RequestType::Standard && req.recipient == Recipient::Endpoint)
        .then(|| (req.index & !0x8f == 0).then(|| EndpointAddress::from(req.index as u8)))
}

/// Whether `req` is the MS OS 2.0 descriptor-set request (recipient included, for uniqueness).
fn is_ms_os_descriptor_request(req: &Request) -> bool {
    req.request_type == RequestType::Vendor
        && req.recipient == Recipient::Device
        && req.request == MSOS_VENDOR_CODE
        && req.index == 0x07
}
