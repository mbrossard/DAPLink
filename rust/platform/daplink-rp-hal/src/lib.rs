//! rp2040-hal platform support for the daplink-rs debug probe.

#![no_std]

mod clocks;
mod delay;
mod init;
mod io_pin;
mod leds;
mod reboot;
mod swo;
mod usb;
mod vcom;

pub use clocks::{clocks, usb_bus, ClockProfile, XOSC_HZ};
pub use delay::{CycleDelay, MIN_HALF_PERIOD_CYCLES, SWD_FIXED_WORK_CYCLES};
pub use init::{probe_parts_pico, probe_parts_pico_accel, PicoAccelParts, PicoParts, ProbeDeps};
pub use io_pin::IoPin;
pub use leds::{
    carrier_leds, status_led, CarrierBankLed, CarrierLeds, CarrierPins, StatusLedPin,
    STATUS_LED_PIN,
};
pub use reboot::{reboot_armed, reboot_watch, RpUsbBoot};
pub use swo::{swo_drain, SwoProducer, Uart0Swo};
pub use usb::UsbBus;
pub use vcom::{
    vcom_account_delivery, vcom_account_discard, vcom_account_drop, vcom_account_host_traffic,
    vcom_baud_state, vcom_counters, vcom_enable_tx_interrupt, vcom_irq_service, vcom_realized_baud,
    vcom_request_baud, VcomIrq, VcomUsb,
};
