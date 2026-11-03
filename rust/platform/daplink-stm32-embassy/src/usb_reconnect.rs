//! STM32 USB detach/reattach through `BCDR.DPPU`, the controller's own D+ pull-up.

use daplink_rs::daplink_core::UsbReconnect;

/// Re-enumerates by toggling the controller's internal D+ pull-up.
pub struct StmUsbReconnect;

impl UsbReconnect for StmUsbReconnect {
    fn detach(&mut self) {
        embassy_stm32::pac::USB.bcdr().modify(|w| w.set_dppu(false));
    }

    fn attach(&mut self) {
        embassy_stm32::pac::USB.bcdr().modify(|w| w.set_dppu(true));
    }
}
