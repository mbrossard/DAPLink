//! Probe bring-up for the original ST-Link/V2 wiring.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::{LateJtagDeps, ScanChainStorage};
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

/// The activity LED: PA9 (`LED_STLINK`), push-pull, **active low** (its die sits between PA9 and
/// +3V3). PA9 is USART1_TX, free because this image takes USART1's RX alone and composes no VCOM.
pub type LedPin = PA9<Output<PushPull>>;

/// The built probe pieces for the original ST-Link/V2 layout.
pub struct ProbeParts {
    /// The wired bit-bang dependencies.
    pub deps: ProbeDeps,
    /// USB serial string derived from the die's unique id.
    pub serial: &'static str,
    /// The `usb-device` bus allocator, ready for `DapUsb::new`.
    pub usb_alloc: &'static UsbBusAllocator<UsbBus<Peripheral>>,
    /// The USART1-RX SWO capture slot (PA10, DMA1 CH5), assembled into the DAP engine.
    pub swo: Usart1Swo,
    /// The PA9 activity LED pin, off (high), for `StatusLed::new_active_low`.
    pub led: LedPin,
}

/// Bring the probe up in the original ST-Link/V2 wiring (SWCLK=PB13, SWDIO=PB14, nRESET=PB0,
/// TDI=PA7, TDO=PA6, SWO=PA10, LED=PA9); call once per image.
///
/// PA15 (`USB_RENUMn`) is left alone: its reset state already enables the D+ pull-up, and driving
/// the net has kept this board off the bus. Re-enumeration is the PA12 D+ pulse instead.
///
/// # Panics
/// Panics if the clock configuration fails (fixed and valid, so that is a hardware fault).
pub fn probe_parts_stlinkv2(dp: pac::Peripherals, delay: DynCycleDelay) -> ProbeParts {
    let mut flash = dp.FLASH.constrain();
    let mut rcc = clocks(&mut flash, dp.RCC);
    let mut gpioa = dp.GPIOA.split(&mut rcc);

    // D+ pulse before the peripheral takes the pin, so a warm reflash reads as a fresh attach.
    let usb_dp = usb_renum(gpioa.pa12, &mut gpioa.crh, rcc.clocks.sysclk().raw());

    // LED off (high) until the engine drives it.
    let led = gpioa
        .pa9
        .into_push_pull_output_with_state(&mut gpioa.crh, PinState::High);

    // nRESET is open-drain: a wired-AND net shared with the target's reset supervisor.
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    let deps = LateJtagDeps::wired(
        IoPin::open_drain(Port::B, 0), // nRESET
        IoPin::new(Port::A, 7),        // TDI
        IoPin::new(Port::B, 14),       // SWDIO / TMS
        IoPin::new(Port::B, 13),       // SWCLK / TCK
        IoPin::new(Port::A, 6),        // TDO
        delay,
        &SCAN,
    );

    // PA10 parks pulled up to the line's idle mark; a receive-only pad needs no AF selection.
    let _pa10 = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
    let swo = usart1_swo(dp.USART1, rcc.clocks.pclk2().raw());

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
