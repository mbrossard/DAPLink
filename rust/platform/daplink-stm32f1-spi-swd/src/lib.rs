//! STM32F1 register seam for the SPI-session SWD engine.

#![no_std]

mod port;
mod timing;

pub use port::{F1SpiPort, SpiSwdBoard, SpiSwdJtagDap};
pub use stm32f1::stm32f103 as pac;
pub use timing::prepare_flash_gpio_half;
