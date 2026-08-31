//! embassy-stm32 platform support for STM32F1-family daplink-rs probes.

#![no_std]

pub mod clocks;
mod delay;
mod dma;
mod init;
mod interrupts;
mod io_pin;
mod led;
#[cfg(feature = "spi-swd")]
mod spi_swd;
mod swo;
mod uid;
mod vcom;

pub use delay::DynCycleDelay;
pub use init::{swd_jtag_deps, swd_only_deps, ProbeDeps, SwdDeps};
pub use interrupts::mask_unhandled_interrupts;
pub use io_pin::IoPin;
pub use led::BicolorLed;
#[cfg(feature = "spi-swd")]
pub use spi_swd::{F1SpiPort, SpiSwdBoard, SpiSwdJtagDap};
pub use swo::{usart1_swo, UartSwo};
pub use uid::{serial, uid_words};
pub use vcom::{vcom_baud_state, vcom_realized_baud, DmaVcom};
