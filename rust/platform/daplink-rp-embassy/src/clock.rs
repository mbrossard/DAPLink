//! Chip-specific Embassy clock initialization.

use embassy_rp::{config::Config, Peripherals};

#[cfg(all(not(feature = "rp2040"), feature = "rp2350", feature = "clock-200mhz"))]
use embassy_rp::clocks::ClockConfig;
#[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
use embassy_rp::clocks::CoreVoltage;
#[cfg(all(
    feature = "rp2040",
    any(feature = "clock-180mhz", feature = "clock-200mhz")
))]
use embassy_rp::clocks::{ClockConfig, CoreVoltage, PllConfig};

/// Initialize Embassy with the selected chip and clock profile.
pub fn init() -> Peripherals {
    #[cfg(all(
        feature = "rp2040",
        any(feature = "clock-180mhz", feature = "clock-200mhz")
    ))]
    prepare_rp2040_overclock_voltage();
    embassy_rp::init(clock_config())
}

#[cfg(feature = "rp2040")]
fn clock_config() -> Config {
    #[cfg(not(any(feature = "clock-180mhz", feature = "clock-200mhz")))]
    return Config::default();

    #[cfg(feature = "clock-180mhz")]
    let fbdiv = 90;
    #[cfg(all(not(feature = "clock-180mhz"), feature = "clock-200mhz"))]
    let fbdiv = 100;

    #[cfg(any(feature = "clock-180mhz", feature = "clock-200mhz"))]
    Config::new(ClockConfig::manual_pll(
        12_000_000,
        PllConfig {
            refdiv: 1,
            fbdiv,
            post_div1: 3,
            post_div2: 2,
        },
        CoreVoltage::V1_15,
    ))
}

#[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
fn clock_config() -> Config {
    let mut config = Config::default();
    #[cfg(feature = "clock-200mhz")]
    {
        config.clocks = ClockConfig::system_freq(crate::CPU_HZ).unwrap();
    }
    config.clocks.core_voltage = CoreVoltage::V1_15;
    config
}

#[cfg(all(
    feature = "rp2040",
    any(feature = "clock-180mhz", feature = "clock-200mhz")
))]
fn prepare_rp2040_overclock_voltage() {
    let vreg = embassy_rp::pac::VREG_AND_CHIP_RESET;
    vreg.vreg().modify(|w| w.set_vsel(CoreVoltage::V1_15 as u8));
    cortex_m::asm::delay(200_000);
    while !vreg.vreg().read().rok() {}
    vreg.bod().write(|w| {
        w.set_vsel(0b1010);
        w.set_en(true);
    });
}
