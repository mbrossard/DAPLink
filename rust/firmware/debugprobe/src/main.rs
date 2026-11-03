//! CMSIS-DAP v2 probe for the Raspberry Pi Debug Probe (RP2040) on embassy: SWD only, plus VCOM.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use panic_halt as _;
use static_cell::ConstStaticCell;

use daplink_rp_embassy::{
    dap_sessions, init as init_rp, spawn_probe_services, vcom_baud_state, ClockProfile, CycleDelay,
    RpUsbBoot, RpUsbDriver, VcomDmaInterruptHandler,
};
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, EndpointSize, PacketSize, DAP_FW_VERSION, DAP_PACKET_SIZE,
};
use daplink_rs::{NoSwo, NoopLeds, RebootVendor, VcomBaudVendor, VcomStatsVendor};

// `--no-default-features` bit-bangs SWD; default `accel` adds PIO1 complete-transfer SWD.
#[cfg(not(feature = "accel"))]
use daplink_rp_embassy::{probe_parts_debugprobe, SplitIoPin};
#[cfg(not(feature = "accel"))]
use daplink_rs::SwdOnlyDap;

#[cfg(feature = "accel")]
use daplink_rp_embassy::probe_parts_debugprobe_accel;
#[cfg(feature = "accel")]
use daplink_rp_pio::{
    pio_swd_transaction_deps, PioSm, PioSwdPins, PioSwdTransactionOnlyDap, RpPioSwdTransaction,
    RpPioSwdTransactionFence,
};
#[cfg(feature = "accel")]
use daplink_rs::ScanChainStorage;

// embassy-rp owns __pre_init (SIO reset); interpose and tail-call the real one.
daplink_rs::ram_text_pre_init!(wrapping);

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    DMA_IRQ_1 => VcomDmaInterruptHandler;
});

#[cfg(not(feature = "overclock-180mhz"))]
const CLOCK_PROFILE: ClockProfile = ClockProfile::DEFAULT;
#[cfg(feature = "overclock-180mhz")]
const CLOCK_PROFILE: ClockProfile = ClockProfile::MHZ_180;
#[cfg(feature = "accel")]
const CPU_HZ: u32 = daplink_rp_pio::PioClock::expect(CLOCK_PROFILE.cpu_hz()).sys_hz();

/// Pipelined CMSIS-DAP command-queue depth and advertised packet count.
const DAP_PACKET_COUNT: u8 = 8;
/// The DAP packet size the command queue holds.
#[cfg(not(feature = "dap-packet-512"))]
const DAP_PACKET_BYTES: u16 = DAP_PACKET_SIZE;
#[cfg(feature = "dap-packet-512")]
const DAP_PACKET_BYTES: u16 = 512;
const DAP_QUEUE_DEPTH: usize = DAP_PACKET_COUNT as usize;
const DAP_QUEUE_SLOT: usize = DAP_PACKET_BYTES as usize;
type AppState = daplink_rp_embassy::AppState<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

const DAP_PACKET: PacketSize =
    DapQueue::<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>::packet_size::<DAP_PACKET_BYTES>();
const DAP_ENDPOINT: EndpointSize =
    DapQueue::<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>::endpoint_size::<DAP_PACKET_SIZE>();

type Vendor = (RebootVendor<RpUsbBoot>, VcomBaudVendor, VcomStatsVendor);

/// PIO0 sequences and fenced PIO1 SWD on split pads.
#[cfg(feature = "accel")]
type Engine = PioSwdTransactionOnlyDap<
    'static,
    PioSm,
    RpPioSwdTransaction,
    CycleDelay,
    NoSwo,
    Vendor,
    NoopLeds,
    RpPioSwdTransactionFence,
>;

/// The CPU bit-banged wire.
#[cfg(not(feature = "accel"))]
type Engine = SwdOnlyDap<'static, SplitIoPin, CycleDelay, NoSwo, Vendor, NoopLeds>;

#[cfg(not(feature = "accel"))]
macro_rules! probe_deps {
    ($parts:expr, $delay:expr) => {
        $parts.deps
    };
}

/// PIO0 sequences and PIO1 transfers; the turnaround releases the driven pad.
#[cfg(feature = "accel")]
macro_rules! probe_deps {
    ($parts:expr, $delay:expr) => {{
        static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        let pins = PioSwdPins {
            swclk: $parts.swclk,
            swdio_out: $parts.swdio_out,
            // The shifted readback pad, live whether or not the probe drives the wire.
            swdio_in: Some($parts.swdio_in),
            oen: None,
            nreset: None,
        };
        // SAFETY: bring-up reserved both PIO blocks and these pads; this runs once.
        unsafe { pio_swd_transaction_deps(pins, CPU_HZ, &SCAN) }
    }};
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init_rp(CLOCK_PROFILE);
    let delay = CycleDelay::new(CLOCK_PROFILE);
    #[cfg(not(feature = "accel"))]
    let parts = probe_parts_debugprobe(p, delay);
    #[cfg(feature = "accel")]
    let parts = probe_parts_debugprobe_accel(p);
    let cfg =
        DapConfig::daplink(parts.serial, DAP_ENDPOINT).with_product(product_string!("debugprobe"));
    let identity = daplink_rs::daplink_core::Identity {
        vendor: cfg.manufacturer,
        product: cfg.product,
        serial: cfg.serial,
        board_vendor: "daplink-rs",
        board_name: "debugprobe",
    };

    #[cfg(not(feature = "accel"))]
    let deps = probe_deps!(parts, delay);
    #[cfg(feature = "accel")]
    let (deps, fence) = probe_deps!(parts, delay);
    let engine = daplink_rs::DapProbe::with_vendor(
        deps,
        NoopLeds,
        delay,
        NoSwo,
        DAP_FW_VERSION,
        DAP_PACKET,
        (
            RebootVendor::new(RpUsbBoot),
            VcomBaudVendor::new(vcom_baud_state),
            VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
        ),
    );
    #[cfg(feature = "accel")]
    let engine = engine.with_command_fence(fence);
    let mut engine: Engine = engine
        .with_packet_count(DAP_PACKET_COUNT)
        .with_identity(identity);

    let (usb_io, vcom_io) = parts.io.split();
    let (usb, class, cdc_tx, cdc_rx, control) =
        daplink_embassy_usb::usb_device(RpUsbDriver::new(Driver::new(usb_io, Irqs)), &cfg);
    spawn_probe_services(
        spawner,
        usb,
        cdc_tx,
        cdc_rx,
        control,
        vcom_io,
        daplink_rs::noop_activity(),
    );

    static APP_STATE: ConstStaticCell<AppState> = ConstStaticCell::new(AppState::new());
    dap_sessions::<false, _, _, _, _, _, _>(
        class,
        APP_STATE.take(),
        &mut engine,
        |dap, request, response| dap.process_command(request, response),
        |dap| dap.suspend(),
        |_dap, _processed| {},
    )
    .await
}
