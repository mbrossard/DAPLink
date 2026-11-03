//! Chip-specific Embassy clock initialization.

#[cfg(feature = "rp2040")]
use embassy_rp::clocks::PllConfig;
use embassy_rp::clocks::{ClockConfig, CoreVoltage};
use embassy_rp::{config::Config, Peripherals};

/// A supported system-clock composition.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClockProfile {
    cpu_hz: u32,
}

impl ClockProfile {
    /// The chip's normal system clock.
    #[cfg(feature = "rp2040")]
    pub const DEFAULT: Self = Self::new(125_000_000);
    /// The chip's normal system clock.
    #[cfg(all(not(feature = "rp2040"), feature = "rp2350"))]
    pub const DEFAULT: Self = Self::new(150_000_000);

    /// The validated 180 MHz RP2040 profile.
    #[cfg(feature = "rp2040")]
    pub const MHZ_180: Self = Self::new(180_000_000);

    /// The validated 200 MHz profile.
    pub const MHZ_200: Self = Self::new(200_000_000);

    const fn new(cpu_hz: u32) -> Self {
        Self { cpu_hz }
    }

    /// System-clock frequency selected by this profile.
    pub const fn cpu_hz(self) -> u32 {
        self.cpu_hz
    }

    #[cfg(feature = "rp2040")]
    const fn is_overclocked(self) -> bool {
        self.cpu_hz != Self::DEFAULT.cpu_hz
    }
}

/// Initialize Embassy with an explicitly selected clock profile.
#[inline(always)]
pub fn init(profile: ClockProfile) -> Peripherals {
    #[cfg(feature = "rp2040")]
    if profile.is_overclocked() {
        prepare_rp2040_overclock_voltage();
    }
    embassy_rp::init(clock_config(profile))
}

#[cfg(feature = "rp2040")]
#[inline(always)]
fn clock_config(profile: ClockProfile) -> Config {
    if profile == ClockProfile::DEFAULT {
        return Config::default();
    }

    let fbdiv = match profile.cpu_hz {
        180_000_000 => 90,
        200_000_000 => 100,
        _ => unreachable!(),
    };
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
#[inline(always)]
fn clock_config(profile: ClockProfile) -> Config {
    let mut config = Config::default();
    if profile != ClockProfile::DEFAULT {
        config.clocks = ClockConfig::system_freq(profile.cpu_hz).unwrap();
    }
    config.clocks.core_voltage = CoreVoltage::V1_15;
    config
}

#[cfg(feature = "rp2040")]
fn prepare_rp2040_overclock_voltage() {
    let vreg = embassy_rp::pac::VREG_AND_CHIP_RESET;
    // Raise and verify core voltage before switching to the faster system PLL.
    vreg.vreg().modify(|w| w.set_vsel(CoreVoltage::V1_15 as u8));
    cortex_m::asm::delay(200_000);
    while !vreg.vreg().read().rok() {}
    vreg.bod().write(|w| {
        w.set_vsel(0b1010);
        w.set_en(true);
    });
}

#[cfg(test)]
mod tests {
    use super::ClockProfile;

    #[test]
    fn supported_profiles_report_their_system_clock() {
        #[cfg(feature = "rp2040")]
        {
            assert_eq!(ClockProfile::DEFAULT.cpu_hz(), 125_000_000);
            assert_eq!(ClockProfile::MHZ_180.cpu_hz(), 180_000_000);
        }
        #[cfg(feature = "rp2350")]
        assert_eq!(ClockProfile::DEFAULT.cpu_hz(), 150_000_000);
        assert_eq!(ClockProfile::MHZ_200.cpu_hz(), 200_000_000);
    }
}
