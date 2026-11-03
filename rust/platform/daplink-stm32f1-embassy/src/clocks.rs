//! STM32F103 clock configuration.

use embassy_stm32::time::Hertz;
use embassy_stm32::Config;

/// Core clock produced by [`clocks_72m_hse8m`].
pub const CPU_HZ: u32 = 72_000_000;

/// 72 MHz off an 8 MHz HSE; USB's fixed /1.5 PLL tap makes 72 MHz the only legal sysclk.
pub fn clocks_72m_hse8m() -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    config
}
