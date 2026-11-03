//! embassy-stm32 platform support for daplink-rs probes on STM32H5-class parts.

#![no_std]

mod baud;
#[cfg(feature = "block-swd")]
mod block;
#[cfg(any(feature = "block-swd", feature = "spi-block"))]
mod block_common;
#[cfg(any(feature = "block-swd", feature = "spi-block"))]
mod dap_mode;
#[cfg(feature = "spi-swd")]
mod duplex;
mod gpdma;
mod icache;
mod init;
mod interrupts;
mod io_pin;
#[cfg(any(feature = "block-swd", feature = "spi-swd"))]
pub mod pace;
#[cfg(feature = "spi-swd")]
mod pads;
mod reboot;
#[cfg(feature = "spi-block")]
mod spi_block;
#[cfg(feature = "spi-swd")]
mod spi_swd;
mod swj;
mod swo;
mod uid;
mod usb;
mod usb_reconnect;
mod vcom;
#[cfg(feature = "wire-probe")]
mod wire_probe;

#[cfg(feature = "block-swd")]
pub use block::{BlockPacing, H5Block, H5Port, Wire};
#[cfg(any(feature = "block-swd", feature = "spi-block"))]
pub use dap_mode::DapMode;
pub use daplink_rs::DynCycleDelay;
#[cfg(feature = "spi-swd")]
pub use duplex::{DuplexBoard, StmDuplexPort};
pub use icache::{configure_factory_ro_region, enable_instruction_cache, IcacheEnable};
pub use init::{swd_jtag_deps_at, ProbeDeps};
pub use interrupts::mask_unhandled_interrupts;
pub use io_pin::{park_inputs, IoPin};
#[cfg(feature = "spi-swd")]
pub use pads::{JtagPadMap, StmJtagPads};
pub use reboot::{check_and_enter_bootloader, reboot_armed, StmSystemDfu};
#[cfg(feature = "spi-block")]
pub use spi_block::SpiBlock;
#[cfg(feature = "spi-swd")]
pub use spi_swd::{SpiSwdBoard, SpiSwdJtagDap, StmSpiPort};
pub use swj::SwjPads;
pub use swo::{dma_faults, uart_swo, DmaSwoRx, UartSwo, UartSwoHw};
pub use uid::{serial, uid_words};
pub use usb::H5UsbDriver;
pub use usb_reconnect::StmUsbReconnect;
pub use vcom::{vcom_baud_state, vcom_realized_baud, StmVcom, VcomInterruptHandler};
#[cfg(feature = "wire-probe")]
pub use wire_probe::run_wire_probe;
