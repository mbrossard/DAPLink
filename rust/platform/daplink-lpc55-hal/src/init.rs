//! MCU-Link probe bring-up: clocks, the high-speed USB1 device and identity, the wired SWD+JTAG
//! bit-bang dependencies, the SWO capture, and the VCOM UART.

use lpc55_hal as hal;
use lpc55_hal::drivers::clocks::Clocks;
use lpc55_hal::drivers::pins::Level;
use lpc55_hal::drivers::usbd::Usb0VbusPin;
use lpc55_hal::drivers::{Timer, UsbBus};
use lpc55_hal::peripherals::usbhs::EnabledUsbhsDevice;
use static_cell::StaticCell;
use usb_device::bus::UsbBusAllocator;

use daplink_rs::daplink_core::serial::SERIAL_LEN;
use daplink_rs::daplink_core::{
    DapConfig, DAP_HS_PACKET_SIZE, DEFAULT_VCOM_BAUD, SCAN_CHAIN_SLOTS,
};
use daplink_rs::{LateJtagDeps, ScanChainStorage};

#[cfg(feature = "core-96mhz")]
use crate::clocks::clocks;
#[cfg(not(feature = "core-96mhz"))]
use crate::clocks::clocks_150mhz;
use crate::delay::DynCycleDelay;
use crate::pin::{self, IoPin};
use crate::swo::{fc3_swo, Fc3Swo};
use crate::vcom::Vcom;

/// The high-speed USB1 device bus type for the MCU-Link probe.
pub type Bus = UsbBus<EnabledUsbhsDevice>;

/// The wired SWD+JTAG bit-bang dependencies for the MCU-Link's SWD header.
pub type ProbeDeps = LateJtagDeps<IoPin, DynCycleDelay>;

/// Serial storage: computed once in [`probe_parts`], before any USB descriptor reads it.
static DEV_SERIAL: StaticCell<[u8; SERIAL_LEN]> = StaticCell::new();

/// The constructed probe pieces returned by [`probe_parts`].
pub struct ProbeParts {
    /// The wired SWD+JTAG bit-bang dependencies, ready for the firmware's DAP engine.
    pub deps: ProbeDeps,
    /// The Flexcomm3 SWO capture, ready for the engine's SWO slot.
    pub swo: Fc3Swo,
    /// The high-speed USB bus allocator, ready for `DapUsb::new`.
    pub usb_alloc: &'static UsbBusAllocator<Bus>,
    /// The USB device identity, sized for the 512-byte high-speed bulk endpoints.
    pub cfg: DapConfig<'static>,
    /// USB serial string derived from the factory UUID.
    pub serial: &'static str,
    /// The VCOM UART (Flexcomm0), bridged to the CDC-ACM serial port by the firmware.
    pub uart: Vcom,
}

/// Bring the MCU-Link probe up: the core clock tree ([`crate::CPU_HZ`]), the high-speed USB1
/// device, the SWD pads, the SWO capture and the VCOM UART. Owns `StaticCell`s: call once.
pub fn probe_parts(dp: hal::raw::Peripherals, delay: DynCycleDelay) -> ProbeParts {
    let mut anactrl = hal::Anactrl::from(dp.ANACTRL);
    let mut pmc = hal::Pmc::from(dp.PMC);
    let mut syscon = hal::Syscon::from(dp.SYSCON);

    let mut gpio = hal::Gpio::from(dp.GPIO).enabled(&mut syscon);
    let mut iocon = hal::Iocon::from(dp.IOCON).enabled(&mut syscon);
    let pins = hal::Pins::take().expect("the lpc55 pins are taken exactly once");

    // Each pad is set to GPIO mode once; `IoPin` then flips direction through the GPIO registers.

    // SWCLK: plain output, idle low.
    let _ = pins
        .pio0_0
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(Level::Low);
    // SWDIO data pad: input, its buffer tri-stated by default.
    let _ = pins
        .pio0_2
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_input();
    // SWDIO buffer enable (TXEN): output, low = buffer tri-stated.
    let _ = pins
        .pio0_28
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(Level::Low);
    // nRESET data pad: output, high = de-asserted.
    let _ = pins
        .pio0_19
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(Level::High);
    // nRESET buffer enable (TXEN): output, low = buffer tri-stated.
    let _ = pins
        .pio0_13
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(Level::Low);
    // JTAG TDI: plain output, idle low.
    let _ = pins
        .pio0_1
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(Level::Low);
    // JTAG TDO is PIO0_3: `fc3_swo` below brings that pad up and leaves it in GPIO input mode.

    // `UsbBus::new` is typed over a `Usb0VbusPin` even on the HS path, which never reads the value.
    let usb0_vbus_pin = pins.pio0_22.into_usb0_vbus_pin(&mut iocon);

    // Full SWJ wiring over the SWD header's buffered pads plus the two JTAG lines.
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    let deps = LateJtagDeps::wired(
        pin::nreset(),
        IoPin::output(0, 1),
        IoPin::buffered(0, 2, 0, 28),
        IoPin::output(0, 0),
        IoPin::input(0, 3),
        delay,
        &SCAN,
    );

    #[cfg(feature = "core-96mhz")]
    let clocks = clocks(&mut anactrl, &mut pmc, &mut syscon);
    // Refusal halts the boot: every CPU-derived constant is built for 150 MHz.
    #[cfg(not(feature = "core-96mhz"))]
    let clocks = clocks_150mhz(&dp.FLASH, &mut anactrl, &mut pmc, &mut syscon)
        .expect("lpc55 150 MHz preflight (see CORE_150MHZ_STATUS)");

    let flexcomm0 = hal::peripherals::flexcomm::Flexcomm0::from((
        dp.FLEXCOMM0,
        dp.I2C0,
        dp.I2S0,
        dp.SPI0,
        dp.USART0,
    ));
    let fc_token = clocks
        .support_flexcomm_token()
        .expect("the Flexcomm clock token is available");
    let uart = Vcom::new(
        flexcomm0,
        pins.pio0_25,
        pins.pio0_24,
        &mut iocon,
        &mut syscon,
        &fc_token,
        DEFAULT_VCOM_BAUD,
    );

    // SWO capture on Flexcomm3 USART3 RX; this bring-up owns the shared PIO0_3 pad.
    let flexcomm3 = hal::peripherals::flexcomm::Flexcomm3::from((
        dp.FLEXCOMM3,
        dp.I2C3,
        dp.I2S3,
        dp.SPI3,
        dp.USART3,
    ));
    let swo = fc3_swo(flexcomm3, pins.pio0_3, &mut iocon, &mut syscon, &fc_token);
    iocon.disabled(&mut syscon).release();

    let usb_alloc = bringup_usb_device(
        dp.CTIMER0,
        dp.USBPHY,
        dp.USB1,
        dp.USBHSH,
        &mut anactrl,
        &mut pmc,
        &mut syscon,
        &clocks,
        usb0_vbus_pin,
    );

    let serial = crate::uid::serial(DEV_SERIAL.init([0u8; SERIAL_LEN]));
    // High-speed bulk endpoints carry 512-byte packets; EP0 stays at the required 64.
    let cfg = DapConfig::daplink(serial).with_packet_size(DAP_HS_PACKET_SIZE);

    ProbeParts {
        deps,
        swo,
        usb_alloc,
        cfg,
        serial,
        uart,
    }
}

/// Bring up the high-speed USB1 controller as a device and return the `'static` bus allocator.
/// Owns the USB allocator `StaticCell`: callable only once per image.
#[allow(clippy::too_many_arguments)]
fn bringup_usb_device<P: Usb0VbusPin + Sync>(
    ctimer0: hal::raw::CTIMER0,
    usbphy: hal::raw::USBPHY,
    usb1: hal::raw::USB1,
    usbhsh: hal::raw::USBHSH,
    anactrl: &mut hal::Anactrl,
    pmc: &mut hal::Pmc,
    syscon: &mut hal::Syscon,
    clocks: &Clocks,
    usb0_vbus_pin: P,
) -> &'static UsbBusAllocator<Bus> {
    // CTIMER0 (1 MHz FRO): the USBHS driver's HS PHY power-up delays run on it.
    let mut delay_timer = Timer::new(
        hal::peripherals::ctimer::Ctimer0::from(ctimer0).enabled(
            syscon,
            clocks
                .support_1mhz_fro_token()
                .expect("the 1 MHz FRO clock token is available"),
        ),
    );

    let usbhs = hal::Usbhs::from((usbphy, usb1, usbhsh));
    let usb_peripheral = usbhs.enabled_as_device(
        anactrl,
        pmc,
        syscon,
        &mut delay_timer,
        clocks
            .support_usbhs_token()
            .expect("the high-speed USB clock token is available"),
    );

    static USB_ALLOC: StaticCell<UsbBusAllocator<Bus>> = StaticCell::new();
    let alloc = USB_ALLOC.init(UsbBus::new(usb_peripheral, usb0_vbus_pin));

    // The PHY can come out of bring-up wedged; check and recover at the end of the sequence.
    crate::usb_phy::verify_and_recover();
    alloc
}
