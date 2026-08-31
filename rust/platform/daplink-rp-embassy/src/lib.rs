//! embassy-rp platform support for the daplink-rs debug probe.

#![no_std]

#[cfg(not(any(feature = "rp2040", feature = "rp2350")))]
compile_error!("enable exactly one chip feature: `rp2040` or `rp2350`");
#[cfg(all(feature = "rp2040", feature = "rp2350"))]
compile_error!("enable exactly one chip feature: `rp2040` or `rp2350`");
#[cfg(all(feature = "rp2350", feature = "clock-180mhz"))]
compile_error!("`clock-180mhz` is only valid for RP2040");
#[cfg(all(feature = "clock-180mhz", feature = "clock-200mhz"))]
compile_error!("select only one core-clock feature");

mod clock;
mod delay;
mod dma_rx;
mod dma_tx;
mod init;
mod io_pin;
mod leds;
mod reboot;
mod swo;
mod usb_reconnect;
mod vcom;

pub use clock::init;
pub use delay::{CycleDelay, CPU_HZ, MIN_HALF_PERIOD_CYCLES, SWD_FIXED_WORK_CYCLES};
pub use dma_tx::VcomDmaInterruptHandler;
pub use init::{
    probe_parts_debugprobe, probe_parts_debugprobe_accel, probe_parts_pico, probe_parts_pico2,
    probe_parts_pico2_accel, probe_parts_pico_accel, DebugProbeAccelParts, DebugProbeDeps,
    DebugProbeParts, HostIo, Pico2AccelParts, Pico2Deps, Pico2Parts, PicoAccelParts, PicoParts,
    ProbeDeps, DEBUGPROBE_SWCLK_PIN, DEBUGPROBE_SWDIO_PIN, DEBUGPROBE_SWDI_PIN,
};
pub use io_pin::{sio_pin, IoPin, SplitIoPin};
#[cfg(feature = "carrier-leds")]
pub use leds::{
    carrier_leds, CarrierBankLed, CarrierLeds, CarrierPins, DAP_LED_PIN, FLASH_LED_PIN,
    HEARTBEAT_LED_PIN, SWO_LED_PIN, VCOM_LED_PIN,
};
pub use leds::{status_led, STATUS_LED_PIN};
pub use reboot::{reboot_armed, reboot_watch, RpUsbBoot};
pub use swo::{DmaSwoRx, Uart0Swo, UartSwoHw};
pub use usb_reconnect::RpUsbReconnect;
pub use vcom::{usb_task, vcom_baud_state, vcom_realized_baud, vcom_task, RpVcom, UsbDrv};
