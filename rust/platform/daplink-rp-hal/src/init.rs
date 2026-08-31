//! Probe bring-up for the bare "Pico as a probe" wiring.

use crate::delay::CycleDelay;
use crate::swo::{uart0_swo, SwoProducer, Uart0Swo};
use crate::{clocks, usb_bus, IoPin};
use daplink_rs::daplink_core::serial::{format_serial, SERIAL_LEN};
use daplink_rs::daplink_core::DEFAULT_VCOM_BAUD;
use daplink_rs::{JtagLines, LateJtagDeps, ScanChainStorage};
use daplink_usb_device::UsbBusAllocator;
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::pac::{self, UART1};
use rp2040_hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use rp2040_hal::usb::UsbBus;
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

/// Off-target there is no boot ROM to read, and only the host gate builds this arm.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn flash_uid() -> [u8; 8] {
    [0u8; 8]
}

/// The wired bit-bang dependencies the DAP engine is assembled from.
pub type ProbeDeps = LateJtagDeps<IoPin, CycleDelay>;

/// The built probe pieces for the Pico layout.
pub struct PicoParts {
    pub deps: ProbeDeps,
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
    #[cfg(feature = "carrier-leds")]
    pub carrier_pins: crate::CarrierPins,
}

/// Bring the probe up in the Pico wiring (SWCLK=GP2, SWDIO=GP3, nRESET=GP1, TDI=GP6, TDO=GP7);
/// call once per image.
///
/// # Panics
/// Panics if the clock/PLL configuration fails; it is fixed and valid, so that is a hardware fault.
pub fn probe_parts_pico(mut pac: pac::Peripherals, delay: CycleDelay) -> PicoParts {
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = clocks(
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
        #[cfg(feature = "carrier-leds")]
        carrier_pins: crate::CarrierPins {
            heartbeat: pins.gpio10.into_push_pull_output(),
            dap: pins.gpio11.into_push_pull_output(),
            flash: pins.gpio12.into_push_pull_output(),
            vcom: pins.gpio14.into_push_pull_output(),
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
    /// Second handles to the four JTAG pads; TCK/TMS ride SWCLK/SWDIO over the pad loan.
    pub jtag_lines: JtagLines<IoPin>,
    /// GPIO numbers the PIO bring-up claims.
    pub swclk: u8,
    pub swdio: u8,
    pub nreset: u8,
    /// Onboard status LED pin (GP25).
    pub status_pin: crate::StatusLedPin,
    #[cfg(feature = "carrier-leds")]
    pub carrier_pins: crate::CarrierPins,
}

/// Bring the probe up in the Pico wiring with SWCLK/SWDIO left for a PIO state machine; the
/// firmware owns PIO0 and those pads from here, so call once per image.
///
/// # Panics
/// Panics if the clock/PLL configuration fails; it is fixed and valid, so that is a hardware fault.
pub fn probe_parts_pico_accel(mut pac: pac::Peripherals) -> PicoAccelParts {
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = clocks(
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

    // PIO0 is handed to the firmware's raw-register bring-up, which addresses it by block.
    let _ = pac.PIO0;

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
        jtag_lines: JtagLines::new(IoPin::new(2), IoPin::new(3), IoPin::new(6), IoPin::new(7)),
        swclk: 2,
        swdio: 3,
        nreset: 1,
        status_pin: pins.gpio25.into_push_pull_output(),
        #[cfg(feature = "carrier-leds")]
        carrier_pins: crate::CarrierPins {
            heartbeat: pins.gpio10.into_push_pull_output(),
            dap: pins.gpio11.into_push_pull_output(),
            flash: pins.gpio12.into_push_pull_output(),
            vcom: pins.gpio14.into_push_pull_output(),
        },
    }
}
