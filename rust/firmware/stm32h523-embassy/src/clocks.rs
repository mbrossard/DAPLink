//! STM32H523 clock configuration: 250 MHz from the internal HSI, crystal-less USB.

use daplink_spi_swd::SpiLadder;
use embassy_stm32::pac::rcc::vals::Spi2sel;
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource};
use embassy_stm32::Config;

/// Core clock produced by every [`ClockPlan`].
pub const CPU_HZ: u32 = 250_000_000;

const HSI_HZ: u32 = 64_000_000;

/// The PLLs of one image: PLL1 P is sysclk; PLL1 Q and PLL2 are SPI2 kernels or off.
pub struct ClockPlan {
    pll1: Pll,
    pll2: Option<Pll>,
}

const PLL1_SYSCLK: Pll = Pll {
    source: PllSource::HSI,
    prediv: PllPreDiv::DIV16, // 64 MHz / 16 = 4 MHz PLL reference
    mul: PllMul::MUL125,      // 4 MHz * 125 = 500 MHz VCO
    divp: Some(PllDiv::DIV2), // 500 MHz / 2 = 250 MHz sysclk
    divq: None,
    divr: None,
};

/// PLL1 alone, for the bit-bang engines.
pub const DEFAULT: ClockPlan = ClockPlan {
    pll1: PLL1_SYSCLK,
    pll2: None,
};

/// PLL1 Q and PLL2 P as the SPI2 kernels of [`SPI2`], for `accel`.
pub const ACCEL: ClockPlan = ClockPlan {
    pll1: Pll {
        divq: Some(PllDiv::DIV5), // 500 MHz / 5 = 100 MHz SPI2 kernel
        ..PLL1_SYSCLK
    },
    pll2: Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV16, // 4 MHz reference, as PLL1
        mul: PllMul::MUL100,      // 400 MHz VCO
        divp: Some(PllDiv::DIV5), // 80 MHz SPI2 kernel
        divq: None,
        divr: None,
    }),
};

/// SPI2's kernels under [`ACCEL`], starting on per_ck; SCK is capped at PLL1 Q / 2.
pub const SPI2: SpiLadder<Spi2sel, 3> = SpiLadder {
    kernels: [
        (Spi2sel::PER, HSI_HZ),
        (Spi2sel::PLL1_Q, 100_000_000),
        (Spi2sel::PLL2_P, 80_000_000),
    ],
    max_sck_hz: 50_000_000,
};

const fn vco_hz(pll: &Pll) -> u32 {
    HSI_HZ / pll.prediv.to_bits() as u32 * (pll.mul.to_bits() as u32 + 1)
}

/// One PLL output's rate, 0 when it is off.
const fn out_hz(pll: &Pll, div: Option<PllDiv>) -> u32 {
    match div {
        Some(div) => vco_hz(pll) / (div.to_bits() as u32 + 1),
        None => 0,
    }
}

const fn exact(pll: &Pll, div: Option<PllDiv>) -> bool {
    match div {
        Some(div) => vco_hz(pll).is_multiple_of(div.to_bits() as u32 + 1),
        None => true,
    }
}

/// HSI source, a 2-16 MHz reference, the wide 128-560 MHz VCO range, and exact outputs.
const fn valid(pll: &Pll) -> bool {
    let prediv = pll.prediv.to_bits() as u32;
    let vco = vco_hz(pll);
    matches!(pll.source, PllSource::HSI)
        && prediv != 0
        && HSI_HZ.is_multiple_of(prediv)
        && HSI_HZ / prediv >= 2_000_000
        && HSI_HZ / prediv <= 16_000_000
        && vco >= 128_000_000
        && vco <= 560_000_000
        && exact(pll, pll.divp)
        && exact(pll, pll.divq)
        && exact(pll, pll.divr)
}

/// `pll` rebuilt on the HSI [`valid`] requires; unlike a copy, it leaves the padding unwritten.
fn fields(pll: &Pll) -> Pll {
    Pll {
        source: PllSource::HSI,
        prediv: pll.prediv,
        mul: pll.mul,
        divp: pll.divp,
        divq: pll.divq,
        divr: pll.divr,
    }
}

impl ClockPlan {
    /// SPI2's kernel rate from `source` under this plan, 0 when it is off.
    const fn spi2_kernel_hz(&self, source: Spi2sel) -> u32 {
        match (source, &self.pll2) {
            (Spi2sel::PER, _) => HSI_HZ,
            (Spi2sel::PLL1_Q, _) => out_hz(&self.pll1, self.pll1.divq),
            (Spi2sel::PLL2_P, Some(pll2)) => out_hz(pll2, pll2.divp),
            _ => 0,
        }
    }

    /// Whether this plan runs every kernel of `ladder` at the ladder's rate.
    pub const fn feeds<const K: usize>(&self, ladder: &SpiLadder<Spi2sel, K>) -> bool {
        let mut i = 0;
        while i < K {
            let (source, hz) = ladder.kernels[i];
            if self.spi2_kernel_hz(source) != hz {
                return false;
            }
            i += 1;
        }
        true
    }

    const fn check(&self) {
        assert!(valid(&self.pll1));
        if let Some(pll2) = &self.pll2 {
            assert!(valid(pll2));
        }
        assert!(
            matches!(self.pll1.divp, Some(div) if div.to_bits() % 2 == 1),
            "PLL1 P divider must be even on the H5"
        );
        assert!(out_hz(&self.pll1, self.pll1.divp) == CPU_HZ);
        // DS14540 Table 20: SPI1-3 kernel clock at most 125 MHz in VOS0.
        assert!(self.spi2_kernel_hz(Spi2sel::PLL1_Q) <= 125_000_000);
        assert!(self.spi2_kernel_hz(Spi2sel::PLL2_P) <= 125_000_000);
    }

    /// 250 MHz (VOS0) off HSI via PLL1; USB on CRS-trimmed HSI48.
    pub fn config(&self) -> Config {
        let mut config = Config::default();
        {
            use embassy_stm32::rcc::*;
            config.rcc.hsi = Some(HSIPrescaler::DIV1);
            config.rcc.pll1 = Some(fields(&self.pll1));
            if let Some(pll2) = &self.pll2 {
                config.rcc.pll2 = Some(fields(pll2));
            }
            config.rcc.sys = Sysclk::PLL1_P;
            config.rcc.voltage_scale = VoltageScale::Scale0;
            config.rcc.ahb_pre = AHBPrescaler::DIV1;
            config.rcc.apb1_pre = APBPrescaler::DIV1;
            config.rcc.apb2_pre = APBPrescaler::DIV1;
            config.rcc.apb3_pre = APBPrescaler::DIV1;
            // Pin the USART kernel-clock muxes against HAL default changes.
            config.rcc.mux.usart1sel = mux::Usart1sel::PCLK2;
            config.rcc.mux.usart2sel = mux::Usartsel::PCLK1;
            config.rcc.hsi48 = Some(Hsi48Config {
                sync_from_usb: true,
            });
            config.rcc.mux.usbsel = mux::Usbsel::HSI48;
        }
        config
    }
}

const _: () = assert!(
    CPU_HZ <= 250_000_000,
    "H5 sysclk maximum is 250 MHz at VOS0"
);
const _: () = DEFAULT.check();
const _: () = ACCEL.check();
const _: () = assert!(ACCEL.feeds(&SPI2));
const _: () = assert!(DEFAULT.spi2_kernel_hz(Spi2sel::PLL1_Q) == 0 && DEFAULT.pll2.is_none());
