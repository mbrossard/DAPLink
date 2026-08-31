//! Probe bring-up for the bare "Pico as a probe" wiring.

use crate::delay::CycleDelay;
use crate::io_pin::{sio_pin, IoPin, SplitIoPin};
use crate::swo::{uart0_swo, Uart0Swo};
use daplink_rs::daplink_core::serial::{format_serial, SERIAL_LEN};
use daplink_rs::{wired_swd_deps, JtagLines, LateJtagDeps, ScanChainStorage, SwdOnlyDeps};
use embassy_rp::gpio::Pull;
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, DMA_CH3, FLASH, PIN_25, PIN_4, PIN_5, UART1, USB};
use embassy_rp::{Peri, Peripherals};
use static_cell::StaticCell;

/// The wired bit-bang dependencies the DAP engine is assembled from.
pub type ProbeDeps = LateJtagDeps<IoPin, CycleDelay>;

/// Peripherals the firmware wires with its own interrupt bindings.
pub struct HostIo {
    pub usb: Peri<'static, USB>,
    pub uart: Peri<'static, UART1>,
    /// VCOM UART TX pin (GP4).
    pub uart_tx: Peri<'static, PIN_4>,
    /// VCOM UART RX pin (GP5).
    pub uart_rx: Peri<'static, PIN_5>,
    /// VCOM receive ring channel.
    pub vcom_rx_dma: Peri<'static, DMA_CH1>,
    /// RP2040's receive bounce channel, reserved on RP2350 so both parts allocate alike.
    pub vcom_chain_dma: Peri<'static, DMA_CH2>,
    /// VCOM transmit channel.
    pub vcom_tx_dma: Peri<'static, DMA_CH3>,
}

/// The built probe pieces for the Pico layout.
pub struct PicoParts {
    pub deps: ProbeDeps,
    pub serial: &'static str,
    pub io: HostIo,
    /// SWO capture slot (UART0 RX on GP13).
    pub swo: Uart0Swo,
    /// Onboard status LED pin (GP25).
    pub status_pin: Peri<'static, PIN_25>,
    #[cfg(feature = "carrier-leds")]
    pub carrier_pins: crate::CarrierPins,
}

/// Bring the probe up in the Pico wiring (SWCLK=GP2, SWDIO=GP3, nRESET=GP1, TDI=GP6, TDO=GP7);
/// call once per image.
pub fn probe_parts_pico(p: Peripherals, delay: CycleDelay) -> PicoParts {
    // Pulled up so the target lines idle high until driven.
    let nreset = {
        let _ = sio_pin(p.PIN_1, Pull::Up);
        IoPin::open_drain(1)
    };
    let swdio = sio_pin(p.PIN_3, Pull::Up);
    let swclk = sio_pin(p.PIN_2, Pull::Up);
    let tdi = sio_pin(p.PIN_6, Pull::Up);
    let tdo = sio_pin(p.PIN_7, Pull::Up);

    static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
        ScanChainStorage::new();
    let deps = LateJtagDeps::wired(nreset, tdi, swdio, swclk, tdo, delay, &SCAN);

    let serial = serial_string(p.FLASH);

    // The raw SWO backend takes the register block, so the typed `p.UART0` is dropped here.
    let _ = p.UART0;
    let swo = uart0_swo(pac::UART0, p.PIN_13, p.DMA_CH0);

    PicoParts {
        deps,
        serial,
        io: HostIo {
            usb: p.USB,
            uart: p.UART1,
            uart_tx: p.PIN_4,
            uart_rx: p.PIN_5,
            vcom_rx_dma: p.DMA_CH1,
            vcom_chain_dma: p.DMA_CH2,
            vcom_tx_dma: p.DMA_CH3,
        },
        swo,
        status_pin: p.PIN_25,
        #[cfg(feature = "carrier-leds")]
        carrier_pins: crate::CarrierPins {
            heartbeat: p.PIN_10,
            dap: p.PIN_11,
            flash: p.PIN_12,
            vcom: p.PIN_14,
            swo: p.PIN_15,
        },
    }
}

/// The built probe pieces for the Pico layout with the wire driven by a PIO state machine.
pub struct PicoAccelParts {
    pub serial: &'static str,
    pub io: HostIo,
    /// SWO capture slot (UART0 RX on GP13).
    pub swo: Uart0Swo,
    /// Second handles to the four JTAG pads; TCK/TMS ride SWCLK/SWDIO over the pad loan.
    pub jtag_lines: JtagLines<IoPin>,
    /// GPIO numbers the PIO bring-up claims.
    pub swclk: u8,
    pub swdio: u8,
    pub nreset: u8,
    /// Onboard status LED pin (GP25).
    pub status_pin: Peri<'static, PIN_25>,
    #[cfg(feature = "carrier-leds")]
    pub carrier_pins: crate::CarrierPins,
}

/// Bring the probe up in the Pico wiring with SWCLK/SWDIO left for a PIO state machine; the
/// firmware owns PIO0 and those pads from here, so call once per image.
pub fn probe_parts_pico_accel(p: Peripherals) -> PicoAccelParts {
    // Pulled up so the lines idle high; the pull survives the later move to the PIO function.
    let _ = sio_pin(p.PIN_1, Pull::Up);
    let _ = sio_pin(p.PIN_3, Pull::Up);
    let _ = sio_pin(p.PIN_2, Pull::Up);
    let tdi = sio_pin(p.PIN_6, Pull::Up);
    let tdo = sio_pin(p.PIN_7, Pull::Up);

    // PIO0 is handed to the firmware's raw-register bring-up, which addresses it by block.
    let _ = p.PIO0;

    // The raw SWO backend takes the register block, so the typed `p.UART0` is dropped here.
    let _ = p.UART0;
    let swo = uart0_swo(pac::UART0, p.PIN_13, p.DMA_CH0);

    PicoAccelParts {
        serial: serial_string(p.FLASH),
        io: HostIo {
            usb: p.USB,
            uart: p.UART1,
            uart_tx: p.PIN_4,
            uart_rx: p.PIN_5,
            vcom_rx_dma: p.DMA_CH1,
            vcom_chain_dma: p.DMA_CH2,
            vcom_tx_dma: p.DMA_CH3,
        },
        swo,
        jtag_lines: JtagLines::new(IoPin::new(2), IoPin::new(3), tdi, tdo),
        swclk: 2,
        swdio: 3,
        nreset: 1,
        status_pin: p.PIN_25,
        #[cfg(feature = "carrier-leds")]
        carrier_pins: crate::CarrierPins {
            heartbeat: p.PIN_10,
            dap: p.PIN_11,
            flash: p.PIN_12,
            vcom: p.PIN_14,
            swo: p.PIN_15,
        },
    }
}

/// The wired bit-bang dependencies for the Pico 2 layout.
pub type Pico2Deps = ProbeDeps;

/// The built probe pieces for the Pico 2 layout.
pub type Pico2Parts = PicoParts;

/// Bring the probe up in the Pico 2 wiring.
#[inline(always)]
pub fn probe_parts_pico2(p: Peripherals, delay: CycleDelay) -> Pico2Parts {
    probe_parts_pico(p, delay)
}

/// The built probe pieces for the accelerated Pico 2 layout.
pub type Pico2AccelParts = PicoAccelParts;

/// Bring the probe up in the accelerated Pico 2 wiring.
#[inline(always)]
pub fn probe_parts_pico2_accel(p: Peripherals) -> Pico2AccelParts {
    probe_parts_pico_accel(p)
}

/// SWCLK, driven through the board's level shifter.
pub const DEBUGPROBE_SWCLK_PIN: u8 = 12;
/// SWDIO readback: the shifted input, readable whether or not the probe is driving the wire.
pub const DEBUGPROBE_SWDI_PIN: u8 = 13;
/// SWDIO drive; the read turnaround tristates it, there being no output-enable pin.
pub const DEBUGPROBE_SWDIO_PIN: u8 = 14;

/// The wired SWD-only dependencies for the Debug Probe layout.
pub type DebugProbeDeps = SwdOnlyDeps<SplitIoPin, CycleDelay>;

/// The built probe pieces for the Debug Probe layout.
pub struct DebugProbeParts {
    pub deps: DebugProbeDeps,
    pub serial: &'static str,
    pub io: HostIo,
}

/// Bring the probe up in the Debug Probe wiring (SWCLK=GP12, SWDI=GP13, SWDIO=GP14, VCOM
/// UART1 TX=GP4/RX=GP5); no reset line and no JTAG pins are wired out, so those are no-connects.
/// Call once per image.
pub fn probe_parts_debugprobe(p: Peripherals, delay: CycleDelay) -> DebugProbeParts {
    // Pulled up so the driven lines idle high; the shifted input is left to the shifter.
    let _ = sio_pin(p.PIN_12, Pull::Up);
    let _ = sio_pin(p.PIN_13, Pull::None);
    let _ = sio_pin(p.PIN_14, Pull::Up);

    let swclk = SplitIoPin::new(DEBUGPROBE_SWCLK_PIN);
    let swdio = SplitIoPin::split(DEBUGPROBE_SWDIO_PIN, DEBUGPROBE_SWDI_PIN);
    let nc = SplitIoPin::nc();

    static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
        ScanChainStorage::new();
    let deps = wired_swd_deps(nc, nc, swdio, swclk, nc, delay, &SCAN);

    DebugProbeParts {
        deps,
        serial: serial_string(p.FLASH),
        io: HostIo {
            usb: p.USB,
            uart: p.UART1,
            uart_tx: p.PIN_4,
            uart_rx: p.PIN_5,
            vcom_rx_dma: p.DMA_CH1,
            vcom_chain_dma: p.DMA_CH2,
            vcom_tx_dma: p.DMA_CH3,
        },
    }
}

/// The built probe pieces for the Debug Probe layout with the wire driven by a PIO state machine.
pub struct DebugProbeAccelParts {
    pub serial: &'static str,
    pub io: HostIo,
    /// GPIO numbers the PIO bring-up claims; SWDIO drives on one pad and reads back on another.
    pub swclk: u8,
    pub swdio_out: u8,
    pub swdio_in: u8,
}

/// Bring the probe up in the Debug Probe wiring with the SWD pads left for a PIO state machine;
/// the firmware owns PIO0 and those pads from here, so call once per image.
pub fn probe_parts_debugprobe_accel(p: Peripherals) -> DebugProbeAccelParts {
    // Pulled up so the driven lines idle high; the pull survives the move to the PIO function.
    let _ = sio_pin(p.PIN_12, Pull::Up);
    let _ = sio_pin(p.PIN_13, Pull::None);
    let _ = sio_pin(p.PIN_14, Pull::Up);

    // PIO0 is handed to the firmware's raw-register bring-up, which addresses it by block.
    let _ = p.PIO0;

    DebugProbeAccelParts {
        serial: serial_string(p.FLASH),
        io: HostIo {
            usb: p.USB,
            uart: p.UART1,
            uart_tx: p.PIN_4,
            uart_rx: p.PIN_5,
            vcom_rx_dma: p.DMA_CH1,
            vcom_chain_dma: p.DMA_CH2,
            vcom_tx_dma: p.DMA_CH3,
        },
        swclk: DEBUGPROBE_SWCLK_PIN,
        swdio_out: DEBUGPROBE_SWDIO_PIN,
        swdio_in: DEBUGPROBE_SWDI_PIN,
    }
}

/// Derive the USB serial string from the QSPI flash unique id, as hex.
#[cfg(feature = "rp2040")]
fn serial_string(flash: Peri<'static, FLASH>) -> &'static str {
    // Used only to read the unique id (a fixed command independent of capacity).
    const FLASH_SIZE: usize = 2 * 1024 * 1024;
    static SERIAL: StaticCell<[u8; SERIAL_LEN]> = StaticCell::new();
    let mut uid = [0u8; 8];
    let uid_ok =
        embassy_rp::flash::Flash::<_, embassy_rp::flash::Blocking, FLASH_SIZE>::new_blocking(flash)
            .blocking_unique_id(&mut uid)
            .is_ok();
    if uid_ok {
        format_serial(&uid, SERIAL.init([0u8; SERIAL_LEN]))
    } else {
        // A distinct marker, not an all-zero hash, so a failed read stays visible.
        "RP-UID-UNKNOWN"
    }
}

/// Derive the USB serial string from the OTP chip id, as hex: `embassy-rp` exposes no QSPI
/// unique-id command on RP235x, so `flash` goes unused here.
#[cfg(feature = "rp2350")]
fn serial_string(_flash: Peri<'static, FLASH>) -> &'static str {
    static SERIAL: StaticCell<[u8; SERIAL_LEN]> = StaticCell::new();
    match embassy_rp::otp::get_chipid() {
        Ok(id) => format_serial(&id.to_be_bytes(), SERIAL.init([0u8; SERIAL_LEN])),
        // A distinct marker, not an all-zero hash, so a failed read stays visible.
        Err(_) => "RP-UID-UNKNOWN",
    }
}
