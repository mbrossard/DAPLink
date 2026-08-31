//! Clock and USB-bus construction helpers for RP2040 firmware.

#[cfg(not(feature = "clock-200mhz"))]
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::clocks::ClocksManager;
#[cfg(feature = "clock-200mhz")]
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::pac::{
    CLOCKS, PLL_SYS, PLL_USB, RESETS, USBCTRL_DPRAM, USBCTRL_REGS, VREG_AND_CHIP_RESET, XOSC,
};
#[cfg(feature = "clock-200mhz")]
use rp2040_hal::pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig};
use rp2040_hal::usb::UsbBus;
use rp2040_hal::watchdog::Watchdog;
#[cfg(feature = "clock-200mhz")]
use rp2040_hal::xosc::setup_xosc_blocking;
use usb_device::bus::UsbBusAllocator;

/// Crystal oscillator frequency on RP2040 boards (12 MHz).
pub const XOSC_HZ: u32 = 12_000_000;

/// Bring the clock tree up (system PLL 125 MHz, USB PLL 48 MHz) from the crystal.
///
/// # Panics
/// Panics if the PLL configuration fails; it is fixed and valid, so that is a hardware fault.
#[cfg(not(feature = "clock-200mhz"))]
pub fn clocks(
    xosc: XOSC,
    clocks: CLOCKS,
    pll_sys: PLL_SYS,
    pll_usb: PLL_USB,
    _vreg: VREG_AND_CHIP_RESET,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> ClocksManager {
    init_clocks_and_plls(XOSC_HZ, xosc, clocks, pll_sys, pll_usb, resets, watchdog)
        .expect("the RP2040 clock/PLL configuration is valid")
}

#[cfg(feature = "clock-200mhz")]
pub fn clocks(
    xosc: XOSC,
    clocks: CLOCKS,
    pll_sys: PLL_SYS,
    pll_usb: PLL_USB,
    vreg: VREG_AND_CHIP_RESET,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> ClocksManager {
    let xosc = setup_xosc_blocking(xosc, XOSC_HZ.Hz()).expect("the crystal configuration is valid");
    watchdog.enable_tick_generation((XOSC_HZ / 1_000_000) as u8);

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
    UsbBusAllocator::new(UsbBus::new(regs, dpram, usb_clock, true, resets))
}
