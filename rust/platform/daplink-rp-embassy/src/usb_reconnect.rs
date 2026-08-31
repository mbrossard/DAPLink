//! RP-family USB detach/reattach: toggling `SIE_CTRL.PULLUP_EN` is a bus-level disconnect.

use daplink_rs::daplink_core::UsbReconnect;

/// Forces a USB re-enumeration by toggling the RP USB controller's D+ pull-up.
pub struct RpUsbReconnect;

impl UsbReconnect for RpUsbReconnect {
    fn detach(&mut self) {
        embassy_rp::pac::USB
            .sie_ctrl()
            .modify(|w| w.set_pullup_en(false));
    }

    fn attach(&mut self) {
        embassy_rp::pac::USB
            .sie_ctrl()
            .modify(|w| w.set_pullup_en(true));
    }
}
