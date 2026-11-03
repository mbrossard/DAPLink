//! Probe bring-up for the original ST-Link/V2 wiring.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::{
    wired_swd_deps, InputOutputPin, LateJtagDeps, NoSwo, ScanChainStorage, SwdOnlyDeps,
};
use daplink_usb_device::UsbBusAllocator;
use static_cell::StaticCell;
use stm32f1xx_hal::gpio::gpioa::PA9;
use stm32f1xx_hal::gpio::{Output, PinState, PushPull};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus};

use crate::clocks::{clocks, usb_renum};
use crate::io_pin::{IoPin, Port};
use crate::swo::{usart1_swo, Usart1Swo};
use crate::DynCycleDelay;

/// The wired SWD+JTAG bit-bang dependencies the DAP engine is assembled from.
pub type ProbeDeps = LateJtagDeps<IoPin, DynCycleDelay>;

/// PA9 activity LED, push-pull and active low.
pub type LedPin = PA9<Output<PushPull>>;

/// The built probe pieces for the original ST-Link/V2 layout.
pub struct ProbeParts<D = ProbeDeps, S = Usart1Swo> {
    pub deps: D,
    pub serial: &'static str,
    /// The `usb-device` bus allocator.
    pub usb_alloc: &'static UsbBusAllocator<UsbBus<Peripheral>>,
    /// SWO capture or an inert replacement.
    pub swo: S,
    /// The PA9 activity LED pin, off (high).
    pub led: LedPin,
}

/// Original ST-Link/V2 wiring; call once per image, leaving PA15 USB_RENUMn untouched.
///
/// # Panics
/// Panics if the clock configuration fails.
pub fn probe_parts_stlinkv2(dp: pac::Peripherals, delay: DynCycleDelay) -> ProbeParts {
    parts(
        dp,
        || {
            static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
            LateJtagDeps::wired(
                IoPin::open_drain(Port::B, 0),
                IoPin::new(Port::A, 7),
                IoPin::new(Port::B, 14),
                IoPin::new(Port::B, 13),
                IoPin::new(Port::A, 6),
                delay,
                &SCAN,
            )
        },
        |usart, clock| {
            IoPin::new(Port::A, 10).set_as_input();
            usart1_swo(usart, clock)
        },
    )
}

/// SWD-only composition leaves PA6, PA7 and PA10 entirely to the caller.
pub fn probe_parts_stlinkv2_swd(
    dp: pac::Peripherals,
    delay: DynCycleDelay,
) -> ProbeParts<SwdOnlyDeps<IoPin, DynCycleDelay>, NoSwo> {
    parts(
        dp,
        || {
            static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
            wired_swd_deps(
                IoPin::open_drain(Port::B, 0),
                IoPin::nc(),
                IoPin::new(Port::B, 14),
                IoPin::new(Port::B, 13),
                IoPin::nc(),
                delay,
                &SCAN,
            )
        },
        |_, _| NoSwo,
    )
}

fn parts<D, S>(
    dp: pac::Peripherals,
    deps: impl FnOnce() -> D,
    swo: impl FnOnce(pac::USART1, u32) -> S,
) -> ProbeParts<D, S> {
    let mut flash = dp.FLASH.constrain();
    let mut rcc = clocks(&mut flash, dp.RCC);
    let mut gpioa = dp.GPIOA.split(&mut rcc);

    // D+ pulse before the peripheral takes the pin, so a warm reflash reads as a fresh attach.
    let usb_dp = usb_renum(gpioa.pa12, &mut gpioa.crh, rcc.clocks.sysclk().raw());

    let led = gpioa
        .pa9
        .into_push_pull_output_with_state(&mut gpioa.crh, PinState::High);

    let deps = deps();
    let swo = swo(dp.USART1, rcc.clocks.pclk2().raw());

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: usb_dp,
    };
    static USB_ALLOC: StaticCell<UsbBusAllocator<UsbBus<Peripheral>>> = StaticCell::new();
    let usb_alloc = USB_ALLOC.init(UsbBus::new(usb));

    ProbeParts {
        deps,
        serial: crate::uid::serial(),
        usb_alloc,
        swo,
        led,
    }
}
