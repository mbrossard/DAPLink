//! STM32H523 clock configuration: 250 MHz from the internal HSI, crystal-less USB.

use embassy_stm32::Config;

/// Core clock produced by [`clocks_250m_hsi`]; the cycle-counting delay counts against it.
pub const CPU_HZ: u32 = 250_000_000;

/// SPI2 kernel clock seen by the `accel` SWD backend: `per_ck`, decoupled from [`CPU_HZ`] so a
/// core-clock change cannot shift the SCK rung ladder.
#[cfg(feature = "accel")]
pub const SPI2_KERNEL_HZ: u32 = 64_000_000;

// The PLL dividers, mirrored as consts so the RM0481 constraints below stay pinned.
#[allow(dead_code)]
const HSI_HZ: u32 = 64_000_000;
#[allow(dead_code)]
const PLL_PREDIV: u32 = 16;
#[allow(dead_code)]
const PLL_MUL: u32 = 125;
#[allow(dead_code)]
const PLL_DIVP: u32 = 2;

/// PLL reference after the prediv; must be 2-16 MHz for the wide VCO range.
#[allow(dead_code)]
const PLL_REF_HZ: u32 = HSI_HZ / PLL_PREDIV;
/// VCO output before the P divider; the wide range is 128-560 MHz.
#[allow(dead_code)]
const PLL_VCO_HZ: u32 = PLL_REF_HZ * PLL_MUL;

const _: () = assert!(PLL_REF_HZ >= 2_000_000 && PLL_REF_HZ <= 16_000_000);
const _: () = assert!(PLL_VCO_HZ >= 128_000_000 && PLL_VCO_HZ <= 560_000_000);
const _: () = assert!(
    PLL_DIVP.is_multiple_of(2),
    "PLL1 P divider must be even on the H5"
);
const _: () = assert!(PLL_VCO_HZ.is_multiple_of(PLL_DIVP) && PLL_VCO_HZ / PLL_DIVP == CPU_HZ);
const _: () = assert!(
    CPU_HZ <= 250_000_000,
    "H5 sysclk maximum is 250 MHz at VOS0"
);

/// 250 MHz (VOS0) off the internal HSI via PLL1, buses undivided, USB on CRS-trimmed HSI48.
pub fn clocks_250m_hsi() -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV16, // 64 MHz / 16 = 4 MHz PLL reference
            mul: PllMul::MUL125,      // 4 MHz * 125 = 500 MHz VCO
            divp: Some(PllDiv::DIV2), // 500 MHz / 2 = 250 MHz sysclk
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.voltage_scale = VoltageScale::Scale0;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.apb3_pre = APBPrescaler::DIV1;
        // Pin the USART kernel-clock muxes, so a HAL default change cannot retune a link.
        config.rcc.mux.usart1sel = mux::Usart1sel::PCLK2;
        config.rcc.mux.usart2sel = mux::Usartsel::PCLK1;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }
    config
}
