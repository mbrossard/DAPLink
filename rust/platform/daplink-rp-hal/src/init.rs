//! Probe bring-up for the bare "Pico as a probe" wiring.

use crate::delay::CycleDelay;
use crate::swo::{uart0_swo, SwoProducer, Uart0Swo};
use crate::UsbBus;
use crate::{clocks, usb_bus, IoPin};
use daplink_rs::daplink_core::serial::{format_serial, SERIAL_LEN};
use daplink_rs::daplink_core::DEFAULT_VCOM_BAUD;
use daplink_rs::{LateJtagDeps, ScanChainStorage};
use daplink_usb_device::UsbBusAllocator;
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::pac::{self, UART1};
use rp2040_hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use rp2040_hal::{gpio::Pins, watchdog::Watchdog, Clock, Sio};
use static_cell::StaticCell;

/// The chip's flash unique id, the serial number's source.
#[cfg(all(target_arch = "arm", target_os = "none"))]
fn flash_uid() -> [u8; 8] {
    let mut uid = [0u8; 8];
    // SAFETY: interrupts are still off this early in bring-up, as `flash_unique_id` requires.
    unsafe { rp2040_flash::flash::flash_unique_id(&mut uid, true) };
    uid
}

/// Off-target there is no boot ROM to read.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn flash_uid() -> [u8; 8] {
    [0u8; 8]
}

/// The wired bit-bang dependencies the DAP engine is assembled from.
pub type ProbeDeps<const CPU_HZ: u32> = LateJtagDeps<IoPin, CycleDelay<CPU_HZ>>;

/// The built probe pieces for the Pico layout.
pub struct PicoParts<const CPU_HZ: u32> {
    pub deps: ProbeDeps<CPU_HZ>,
    pub serial: &'static str,
    pub usb_alloc: &'static mut UsbBusAllocator<UsbBus>,
    /// The interrupt and USB halves of the VCOM transport.
    pub vcom: (crate::VcomIrq, crate::VcomUsb),
    /// SWO capture slot (UART0 RX on GP13).
    pub swo: Uart0Swo,
    /// Producer half of the SWO ring, held by the UART0 interrupt.
    pub swo_producer: SwoProducer,
    /// Onboard status LED pin (GP25).
    pub status_pin: crate::StatusLedPin,
    pub carrier_pins: crate::CarrierPins,
}

/// Bring the probe up in the Pico wiring (SWCLK=GP2, SWDIO=GP3, nRESET=GP1, TDI=GP6, TDO=GP7).
#[inline(always)]
pub fn probe_parts_pico<const CPU_HZ: u32>(
    mut pac: pac::Peripherals,
    profile: crate::ClockProfile,
    delay: CycleDelay<CPU_HZ>,
) -> PicoParts<CPU_HZ> {
    assert_eq!(CPU_HZ, profile.cpu_hz());
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = clocks(
        profile,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        pac.VREG_AND_CHIP_RESET,
        &mut pac.RESETS,
        &mut watchdog,
    );
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Pulled up so SWCLK/SWDIO/nRESET idle high until driven; `IoPin` drives the SIO registers.
    let _ = pins.gpio1.into_pull_up_input();
    let _ = pins.gpio2.into_pull_up_input();
    let _ = pins.gpio3.into_pull_up_input();
    let _ = pins.gpio6.into_pull_up_input();
    let _ = pins.gpio7.into_pull_up_input();

    static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
        ScanChainStorage::new();
    let deps = LateJtagDeps::wired(
        IoPin::open_drain(1),
        IoPin::new(6),
        IoPin::new(3),
        IoPin::new(2),
        IoPin::new(7),
        delay,
        &SCAN,
    );

    static USB_ALLOC: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();
    let usb_alloc = USB_ALLOC.init(usb_bus(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        &mut pac.RESETS,
    ));

    static SERIAL: StaticCell<[u8; SERIAL_LEN]> = StaticCell::new();
    let uid = flash_uid();
    let serial = format_serial(&uid, SERIAL.init([0u8; SERIAL_LEN]));

    let uart_pins = (
        pins.gpio4.into_function::<FunctionUart>(),
        pins.gpio5.into_function::<FunctionUart>(),
    );
    let mut uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(DEFAULT_VCOM_BAUD.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .expect("UART1 VCOM configuration is valid");
    uart.enable_rx_interrupt();
    // SAFETY: UART1 is fully configured and interrupts are not enabled globally yet.
    unsafe { &*UART1::ptr() }.uartimsc().modify(|_, w| {
        w.feim().set_bit();
        w.peim().set_bit();
        w.beim().set_bit();
        w.oeim().set_bit()
    });
    let vcom = crate::vcom::vcom_split();

    let (swo, swo_producer) = uart0_swo(
        pac.UART0,
        &mut pac.RESETS,
        pins.gpio13,
        clocks.peripheral_clock.freq().to_Hz(),
    );

    PicoParts {
        deps,
        serial,
        usb_alloc,
        vcom,
        swo,
        swo_producer,
        status_pin: pins.gpio25.into_push_pull_output(),
        carrier_pins: crate::CarrierPins {
            gpio10: pins.gpio10,
            gpio11: pins.gpio11,
            gpio12: pins.gpio12,
            gpio14: pins.gpio14,
        },
    }
}

/// The built probe pieces for the Pico layout with the wire driven by a PIO state machine.
pub struct PicoAccelParts {
    pub serial: &'static str,
    pub usb_alloc: &'static mut UsbBusAllocator<UsbBus>,
    /// The interrupt and USB halves of the VCOM transport.
    pub vcom: (crate::VcomIrq, crate::VcomUsb),
    /// SWO capture slot (UART0 RX on GP13).
    pub swo: Uart0Swo,
    /// Producer half of the SWO ring, held by the UART0 interrupt.
    pub swo_producer: SwoProducer,
    /// GPIO numbers the PIO bring-up claims.
    pub swclk: u8,
    pub swdio: u8,
    pub tdi: u8,
    pub tdo: u8,
    pub nreset: u8,
    /// Onboard status LED pin (GP25).
    pub status_pin: crate::StatusLedPin,
    pub carrier_pins: crate::CarrierPins,
}

/// Bring up the Pico wiring with the SWJ pads and both PIO blocks reserved for the firmware.
#[inline(always)]
pub fn probe_parts_pico_accel(
    mut pac: pac::Peripherals,
    profile: crate::ClockProfile,
) -> PicoAccelParts {
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = clocks(
        profile,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        pac.VREG_AND_CHIP_RESET,
        &mut pac.RESETS,
        &mut watchdog,
    );
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Pulled up so the lines idle high; the pull survives the later move to the PIO function.
    let _ = pins.gpio1.into_pull_up_input();
    let _ = pins.gpio2.into_pull_up_input();
    let _ = pins.gpio3.into_pull_up_input();
    let _ = pins.gpio6.into_pull_up_input();
    let _ = pins.gpio7.into_pull_up_input();

    // Both whole blocks are handed to raw-register bring-up; optimized engines also use PIO1.
    let _ = (pac.PIO0, pac.PIO1);

    static USB_ALLOC: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();
    let usb_alloc = USB_ALLOC.init(usb_bus(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        &mut pac.RESETS,
    ));

    static SERIAL: StaticCell<[u8; SERIAL_LEN]> = StaticCell::new();
    let uid = flash_uid();
    let serial = format_serial(&uid, SERIAL.init([0u8; SERIAL_LEN]));

    let uart_pins = (
        pins.gpio4.into_function::<FunctionUart>(),
        pins.gpio5.into_function::<FunctionUart>(),
    );
    let mut uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(DEFAULT_VCOM_BAUD.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .expect("UART1 VCOM configuration is valid");
    uart.enable_rx_interrupt();
    // SAFETY: UART1 is fully configured and interrupts are not enabled globally yet.
    unsafe { &*UART1::ptr() }.uartimsc().modify(|_, w| {
        w.feim().set_bit();
        w.peim().set_bit();
        w.beim().set_bit();
        w.oeim().set_bit()
    });
    let vcom = crate::vcom::vcom_split();

    let (swo, swo_producer) = uart0_swo(
        pac.UART0,
        &mut pac.RESETS,
        pins.gpio13,
        clocks.peripheral_clock.freq().to_Hz(),
    );

    PicoAccelParts {
        serial,
        usb_alloc,
        vcom,
        swo,
        swo_producer,
        swclk: 2,
        swdio: 3,
        tdi: 6,
        tdo: 7,
        nreset: 1,
        status_pin: pins.gpio25.into_push_pull_output(),
        carrier_pins: crate::CarrierPins {
            gpio10: pins.gpio10,
            gpio11: pins.gpio11,
            gpio12: pins.gpio12,
            gpio14: pins.gpio14,
        },
    }
}
