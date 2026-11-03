//! embassy-rp platform support for the daplink-rs debug probe.

#![no_std]

#[cfg(not(any(feature = "rp2040", feature = "rp2350")))]
compile_error!("enable exactly one chip feature: `rp2040` or `rp2350`");
#[cfg(all(feature = "rp2040", feature = "rp2350"))]
compile_error!("enable exactly one chip feature: `rp2040` or `rp2350`");

mod app;
mod clock;
mod delay;
mod dma_rx;
mod dma_tx;
mod init;
mod io_pin;
mod leds;
mod reboot;
mod swo;
mod usb;
mod usb_reconnect;
mod vcom;

pub use app::{dap_sessions, spawn_probe_services, AppState, DapShape, Shapes};
pub use clock::{init, ClockProfile};
pub use delay::{CycleDelay, MIN_HALF_PERIOD_CYCLES, SWD_FIXED_WORK_CYCLES};
pub use dma_tx::VcomDmaInterruptHandler;
pub use init::{
    probe_parts_debugprobe, probe_parts_debugprobe_accel, probe_parts_pico, probe_parts_pico2,
    probe_parts_pico2_accel, probe_parts_pico_accel, DebugProbeAccelParts, DebugProbeDeps,
    DebugProbeParts, HostIo, Pico2AccelParts, Pico2Deps, Pico2Parts, PicoAccelParts, PicoParts,
    ProbeDeps, VcomIo, DEBUGPROBE_SWCLK_PIN, DEBUGPROBE_SWDIO_PIN, DEBUGPROBE_SWDI_PIN,
};
pub use io_pin::{sio_pin, IoPin, SplitIoPin};
pub use leds::{
    carrier_leds, status_led, CarrierBankLed, CarrierLeds, CarrierPins, STATUS_LED_PIN,
};
pub use reboot::{reboot_armed, reboot_watch, RpUsbBoot};
pub use swo::{DmaSwoRx, Uart0Swo, UartSwoHw};
pub use usb::RpUsbDriver;
pub use usb_reconnect::RpUsbReconnect;
pub use vcom::{usb_task, vcom_baud_state, vcom_realized_baud, vcom_task, RpVcom, UsbDrv};
