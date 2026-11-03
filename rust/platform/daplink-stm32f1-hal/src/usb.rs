//! The endpoint-register repair stm32-usbd's unstall lacks.

use daplink_rs::daplink_core::stm32_epr::clear_halt_step;
use daplink_usb_device::{EndpointAddress, EndpointRepair};
use stm32f1xx_hal::pac;
use usb_device::UsbDirection;

/// The F1's eight EPnR registers; stm32-usbd indexes its endpoint array with any wIndex.
pub struct F1EndpointRepair;

/// The repair to hand [`daplink_usb_device::DapUsbNoVcom::with_endpoint_repair`].
pub static ENDPOINT_REPAIR: F1EndpointRepair = F1EndpointRepair;

impl EndpointRepair for F1EndpointRepair {
    fn has(&self, ep: EndpointAddress) -> bool {
        ep.index() < 8
    }

    fn clear_halt(&self, ep: EndpointAddress) {
        critical_section::with(|_| {
            // SAFETY: EPnR are word-spaced from the USB base, and stm32-usbd writes them only with interrupts off.
            let reg = unsafe { (pac::USB::ptr() as *mut u32).add(ep.index()) };
            let dir_in = ep.direction() == UsbDirection::In;
            // SAFETY: an aligned volatile access to the register above.
            while let Some(w) = clear_halt_step(unsafe { reg.read_volatile() }, dir_in) {
                // SAFETY: as for the read.
                unsafe { reg.write_volatile(w) };
            }
        });
    }
}
