//! embassy-stm32 platform support for the daplink-rs debug probe.

#![no_std]

mod delay;
mod icache;
mod init;
mod interrupts;
mod io_pin;
mod reboot;
#[cfg(feature = "spi-swd")]
mod spi_swd;
mod swo;
mod uid;
mod usb_reconnect;
mod vcom;

pub use delay::DynCycleDelay;
pub use icache::{configure_factory_ro_region, enable_instruction_cache, IcacheEnable};
pub use init::{swd_jtag_deps, ProbeDeps};
pub use interrupts::mask_unhandled_interrupts;
pub use io_pin::IoPin;
pub use reboot::{check_and_enter_bootloader, reboot_armed, StmSystemDfu};
#[cfg(feature = "spi-swd")]
pub use spi_swd::{SpiSwdBoard, SpiSwdJtagDap, StmSpiPort};
pub use swo::{dma_faults, uart_swo, DmaSwoRx, UartSwo, UartSwoHw};
pub use uid::{serial, uid_words};
pub use usb_reconnect::StmUsbReconnect;
pub use vcom::{vcom_baud_state, vcom_realized_baud, StmVcom, VcomInterruptHandler};
