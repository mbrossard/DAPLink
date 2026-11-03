//! embassy-stm32 platform support for STM32F1-family daplink-rs probes.

#![no_std]

pub mod clocks;
mod dma;
mod init;
mod interrupts;
mod io_pin;
mod led;
mod swo;
mod uid;
mod usb;
mod usb_connect;
mod vcom;

pub use daplink_rs::DynCycleDelay;
pub use init::{swd_jtag_deps, swd_only_deps, ProbeDeps, SwdDeps};
pub use interrupts::mask_unhandled_interrupts;
pub use io_pin::IoPin;
pub use led::BicolorLed;
pub use swo::{usart1_swo, UartSwo};
pub use uid::{serial, uid_words};
pub use usb::F1UsbDriver;
pub use usb_connect::{usb_connect_pa15, usb_connect_pb15, UsbConnect};
pub use vcom::{vcom_baud_state, vcom_realized_baud, DmaVcom};
