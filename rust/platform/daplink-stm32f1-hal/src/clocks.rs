//! STM32F103 clock bring-up and D+-pulse USB re-enumeration.

use stm32f1xx_hal::flash::Parts as FlashParts;
use stm32f1xx_hal::gpio::gpioa::PA12;
use stm32f1xx_hal::gpio::{Active, Cr, Floating, Input};
use stm32f1xx_hal::pac::RCC;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::rcc::{Config, Rcc};

/// Control-register handle for the upper half (pins 8..15) of GPIO port A.
type CrhA = Cr<'A', true>;

/// External crystal frequency on this board class (8 MHz HSE).
pub const HSE_HZ: u32 = 8_000_000;

/// Core clock produced by [`clocks`] (HSE x9 via the PLL), and the delay's cycle base.
pub const CPU_HZ: u32 = 72_000_000;

/// Bring up the clock tree from the 8 MHz HSE: SYSCLK 72, PCLK1 36 (its cap), PCLK2 72 MHz, and
/// USB 48 MHz off the PLL's fixed /1.5 tap.
///
/// # Panics
/// Panics if the USB clock is invalid, which for this fixed configuration is a hardware fault.
pub fn clocks(flash: &mut FlashParts, rcc: RCC) -> Rcc {
    let rcc = rcc.freeze(
        Config::hse(HSE_HZ.Hz()).sysclk(CPU_HZ.Hz()).pclk1(36.MHz()),
        &mut flash.acr,
    );
    assert!(rcc.clocks.usbclk_valid());
    rcc
}

/// Force a re-enumeration where the D+ pull-up is tied to 3V3: hold D+ (PA12) low for ~10 ms so
/// the host sees a detach, then release it as a floating input.
pub fn usb_renum<MODE: Active>(
    pa12: PA12<MODE>,
    crh: &mut CrhA,
    sysclk_hz: u32,
) -> PA12<Input<Floating>> {
    let mut dp = pa12.into_push_pull_output(crh);
    dp.set_low();
    cortex_m::asm::delay(sysclk_hz / 100);
    dp.into_floating_input(crh)
}
