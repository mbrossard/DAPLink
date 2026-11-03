//! This board's probe pads: SWCLK/TCK = PB10, SWDIO/TMS = PB15, TDI = PA7, TDO = PA6.

use daplink_stm32_embassy::SwjPads;
use embassy_stm32::pac;

pub struct H5Pads;

impl SwjPads for H5Pads {
    const SWCLK: usize = 10;
    const SWDIO: usize = 15;

    fn swj_port() -> pac::gpio::Gpio {
        pac::GPIOB
    }
}

#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
impl daplink_stm32_embassy::JtagPadMap for H5Pads {
    const TCK_PIN: usize = Self::SWCLK;
    const TMS_PIN: usize = Self::SWDIO;
    const TAP_SPEED: pac::gpio::vals::Ospeedr = pac::gpio::vals::Ospeedr::MEDIUM_SPEED;
    const TDI_PIN: usize = 7;
    const TDO_PIN: usize = 6;

    fn tck_port() -> pac::gpio::Gpio {
        Self::swj_port()
    }
    fn tdi_port() -> pac::gpio::Gpio {
        pac::GPIOA
    }
}
