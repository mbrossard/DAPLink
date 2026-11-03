//! Clock and USB-bus construction helpers for RP2040 firmware.

use crate::UsbBus;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::pac::{
    CLOCKS, PLL_SYS, PLL_USB, RESETS, USBCTRL_DPRAM, USBCTRL_REGS, VREG_AND_CHIP_RESET, XOSC,
};
use rp2040_hal::pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig};
use rp2040_hal::watchdog::Watchdog;
use rp2040_hal::xosc::setup_xosc_blocking;
use usb_device::bus::UsbBusAllocator;

pub const XOSC_HZ: u32 = 12_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClockProfile(u32);

impl ClockProfile {
    pub const DEFAULT: Self = Self(125_000_000);
    pub const MHZ_200: Self = Self(200_000_000);

    pub const fn cpu_hz(self) -> u32 {
        self.0
    }
}

/// Bring the clock tree up (system PLL 125 MHz, USB PLL 48 MHz) from the crystal.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
pub fn clocks(
    profile: ClockProfile,
    xosc: XOSC,
    clocks: CLOCKS,
    pll_sys: PLL_SYS,
    pll_usb: PLL_USB,
    vreg: VREG_AND_CHIP_RESET,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> ClocksManager {
    if profile == ClockProfile::DEFAULT {
        return init_clocks_and_plls(XOSC_HZ, xosc, clocks, pll_sys, pll_usb, resets, watchdog)
            .expect("the RP2040 clock/PLL configuration is valid");
    }
    let xosc = setup_xosc_blocking(xosc, XOSC_HZ.Hz()).expect("the crystal configuration is valid");
    watchdog.enable_tick_generation((XOSC_HZ / 1_000_000) as u8);

    // Raise and verify core voltage before switching to the faster system PLL.
    vreg.vreg().modify(|_, w| w.vsel().voltage1_15());
    cortex_m::asm::delay(200_000);
    while vreg.vreg().read().rok().bit_is_clear() {}
    vreg.bod()
        .write(|w| unsafe { w.vsel().bits(0b1010) }.en().set_bit());

    let mut clocks = ClocksManager::new(clocks);
    let pll_sys = setup_pll_blocking(
        pll_sys,
        xosc.operating_frequency(),
        PLLConfig {
            vco_freq: 1_200_000_000u32.Hz(),
            refdiv: 1,
            post_div1: 3,
            post_div2: 2,
        },
        &mut clocks,
        resets,
    )
    .expect("the system PLL configuration is valid");
    let pll_usb = setup_pll_blocking(
        pll_usb,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .expect("the USB PLL configuration is valid");
    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .expect("the RP2040 clock tree is valid");
    clocks
}

/// Construct the `usb-device` bus allocator over the RP2040 USB controller.
pub fn usb_bus(
    regs: USBCTRL_REGS,
    dpram: USBCTRL_DPRAM,
    usb_clock: rp2040_hal::clocks::UsbClock,
    resets: &mut RESETS,
) -> UsbBusAllocator<UsbBus> {
    UsbBusAllocator::new(UsbBus::new(rp2040_hal::usb::UsbBus::new(
        regs, dpram, usb_clock, true, resets,
    )))
}
