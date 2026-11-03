//! CMSIS-DAP v2 probe for the Raspberry Pi Pico 2 (RP2350) on embassy.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
#[cfg(not(feature = "flash"))]
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use static_cell::ConstStaticCell;

// A flash build latches the fault into `RESET_STAMP` and resets; every other build halts.
#[cfg(all(feature = "flash", not(feature = "defmt")))]
daplink_vfs::fault_reset_handlers!();
#[cfg(all(not(feature = "flash"), not(feature = "defmt")))]
use panic_halt as _;
// `defmt` halts in every build, so the RTT-attached probe sees the panic message.
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[cfg(not(feature = "accel"))]
use daplink_rp_embassy::IoPin;
#[cfg(not(feature = "flash"))]
use daplink_rp_embassy::Uart0Swo;
#[cfg(feature = "flash")]
use daplink_rp_embassy::UsbDrv;
use daplink_rp_embassy::{
    dap_sessions, init as init_rp, spawn_probe_services, status_led, vcom_baud_state, ClockProfile,
    CycleDelay, RpUsbBoot, RpUsbDriver, VcomDmaInterruptHandler,
};
#[cfg(not(feature = "flash"))]
use daplink_rs::daplink_core::DAP_FW_VERSION;
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, EndpointSize, PacketSize, DAP_PACKET_SIZE,
};
#[cfg(not(feature = "flash"))]
use daplink_rs::StatusLed;
use daplink_rs::{Activity, RebootVendor, VcomBaudVendor, VcomStatsVendor};

#[cfg(feature = "carrier-leds")]
use daplink_rp_embassy::{carrier_leds, CarrierBankLed};

// `--no-default-features` bit-bangs; default `accel` adds PIO1 complete-transfer SWD and JTAG.
#[cfg(not(feature = "accel"))]
use daplink_rp_embassy::probe_parts_pico2;
#[cfg(all(not(feature = "accel"), not(feature = "flash")))]
use daplink_rs::BitbangDap;

#[cfg(feature = "accel")]
use daplink_rp_embassy::probe_parts_pico2_accel;
#[cfg(all(feature = "accel", not(feature = "flash")))]
use daplink_rp_pio::PioTransferDap;
#[cfg(feature = "accel")]
use daplink_rp_pio::{
    pio_transfer_deps, PioJtagPins, PioSm, PioSwdPins, RpPioTransferEngine, RpPioTransferFence,
};
#[cfg(all(feature = "accel", feature = "flash"))]
use daplink_rp_pio::{PioTransferDeps, PioTransferJtag, PioTransferSwd};
#[cfg(any(feature = "accel", feature = "flash"))]
use daplink_rs::ScanChainStorage;

#[cfg(feature = "flash")]
use daplink_embassy_usb::msc::MscEndpoints;
#[cfg(feature = "flash")]
use daplink_rp_embassy::RpUsbReconnect;
#[cfg(feature = "flash")]
use daplink_rs::flash_mux::{mux_dap_engine, mux_flash_backend, MuxFlashBackend};
#[cfg(all(feature = "flash", feature = "accel"))]
use daplink_rs::FencedFlashBackend;
#[cfg(feature = "flash")]
use daplink_rs::SwdMux;
#[cfg(all(feature = "flash", not(feature = "accel")))]
use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv};
#[cfg(feature = "flash-nrf52833")]
use daplink_targets::targets::nrf52833::NRF52833;
#[cfg(feature = "flash-nrf52840")]
use daplink_targets::targets::nrf52840::NRF52840;
#[cfg(feature = "flash-rp2040")]
use daplink_targets::targets::rp2040::RP2040;
#[cfg(feature = "flash-rp2350")]
use daplink_targets::targets::rp2350::RP2350;
#[cfg(feature = "flash-stm32f401")]
use daplink_targets::targets::stm32f401ce::STM32F401CE;
#[cfg(feature = "flash")]
use daplink_vfs::{AllDecoders, FlashSink, FlashVolume, VendorFlash, VolumeStorage};

// embassy-rp owns __pre_init (SIO reset); interpose and tail-call the real one.
daplink_rs::ram_text_pre_init!(wrapping);

#[cfg(feature = "flash-rp2040")]
type PrimaryTarget = RP2040;
#[cfg(feature = "flash-rp2040")]
static PRIMARY: &PrimaryTarget = &RP2040;
#[cfg(all(not(feature = "flash-rp2040"), feature = "flash-stm32f401"))]
type PrimaryTarget = STM32F401CE;
#[cfg(all(not(feature = "flash-rp2040"), feature = "flash-stm32f401"))]
static PRIMARY: &PrimaryTarget = &STM32F401CE;
#[cfg(all(
    not(feature = "flash-rp2040"),
    not(feature = "flash-stm32f401"),
    feature = "flash-rp2350"
))]
type PrimaryTarget = RP2350;
#[cfg(all(
    not(feature = "flash-rp2040"),
    not(feature = "flash-stm32f401"),
    feature = "flash-rp2350"
))]
static PRIMARY: &PrimaryTarget = &RP2350;
#[cfg(all(
    feature = "flash-nrf52840",
    not(any(
        feature = "flash-rp2040",
        feature = "flash-stm32f401",
        feature = "flash-rp2350"
    ))
))]
type PrimaryTarget = NRF52840;
#[cfg(all(
    feature = "flash-nrf52840",
    not(any(
        feature = "flash-rp2040",
        feature = "flash-stm32f401",
        feature = "flash-rp2350"
    ))
))]
static PRIMARY: &PrimaryTarget = &NRF52840;
#[cfg(all(
    feature = "flash-nrf52833",
    not(any(
        feature = "flash-rp2040",
        feature = "flash-stm32f401",
        feature = "flash-rp2350",
        feature = "flash-nrf52840"
    ))
))]
type PrimaryTarget = NRF52833;
#[cfg(all(
    feature = "flash-nrf52833",
    not(any(
        feature = "flash-rp2040",
        feature = "flash-stm32f401",
        feature = "flash-rp2350",
        feature = "flash-nrf52840"
    ))
))]
static PRIMARY: &PrimaryTarget = &NRF52833;

#[cfg(feature = "flash")]
daplink_targets::target_table! {
    umbrella = "flash",
    count = TARGET_COUNT,
    selector = Selector,
    scratch = SCRATCH,
    table = flash_table,
    buffer = flash_scratch,
    rows = [
        ("flash-rp2040", RP2040, daplink_targets::targets::rp2040::PROGRAM_BUFFER_SIZE),
        ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
        ("flash-rp2350", RP2350, daplink_targets::targets::rp2350::PROGRAM_BUFFER_SIZE),
        ("flash-nrf52840", NRF52840, daplink_targets::targets::nrf52840::PROGRAM_BUFFER_SIZE),
        ("flash-nrf52833", NRF52833, daplink_targets::targets::nrf52833::PROGRAM_BUFFER_SIZE),
    ],
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    DMA_IRQ_1 => VcomDmaInterruptHandler;
});

#[cfg(not(feature = "overclock-200mhz"))]
const CLOCK_PROFILE: ClockProfile = ClockProfile::DEFAULT;
#[cfg(feature = "overclock-200mhz")]
const CLOCK_PROFILE: ClockProfile = ClockProfile::MHZ_200;
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

/// The onboard status LED: GP25, active-high.
#[cfg(not(feature = "flash"))]
type StatusLedType = StatusLed<Output<'static>>;

#[cfg(not(feature = "flash"))]
type RebootOnly = (RebootVendor<RpUsbBoot>, VcomBaudVendor, VcomStatsVendor);

#[cfg(all(not(feature = "accel"), not(feature = "flash")))]
type Engine = BitbangDap<'static, IoPin, CycleDelay, Uart0Swo, RebootOnly, StatusLedType>;

#[cfg(all(not(feature = "flash"), feature = "accel"))]
type Engine = PioTransferDap<
    'static,
    PioSm,
    RpPioTransferEngine,
    CycleDelay,
    Uart0Swo,
    RebootOnly,
    StatusLedType,
    RpPioTransferFence,
>;

#[cfg(all(feature = "flash", not(feature = "accel")))]
type MuxDeps = LateJtagDeps<IoPin, CycleDelay>;
#[cfg(all(feature = "flash", not(feature = "accel")))]
type MuxSwd = LateSwdDrv<IoPin, CycleDelay>;
#[cfg(all(feature = "flash", not(feature = "accel")))]
type MuxJtag = LateJtag<IoPin, CycleDelay>;

#[cfg(all(feature = "flash", feature = "accel"))]
type MuxDeps = PioTransferDeps<PioSm, RpPioTransferEngine>;
#[cfg(all(feature = "flash", feature = "accel"))]
type MuxSwd = PioTransferSwd<PioSm, RpPioTransferEngine>;
#[cfg(all(feature = "flash", feature = "accel"))]
type MuxJtag = PioTransferJtag<PioSm, RpPioTransferEngine>;

#[cfg(feature = "flash")]
type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
#[cfg(feature = "flash")]
type RawBackend = MuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, CycleDelay, PrimaryTarget, Selector>;
#[cfg(all(feature = "flash", not(feature = "accel")))]
type Backend = RawBackend;
#[cfg(all(feature = "flash", feature = "accel"))]
type Backend = FencedFlashBackend<RawBackend, &'static RpPioTransferFence>;
#[cfg(feature = "flash")]
type Sink = FlashSink<Backend>;
#[cfg(feature = "flash")]
type RebootBusy = &'static Mux;
/// Reboot, declined with `STATUS_BUSY` during a live flash session.
#[cfg(feature = "flash")]
type Vendor = (
    RebootVendor<RpUsbBoot, RebootBusy>,
    VendorFlash<AllDecoders, Sink>,
    VcomBaudVendor,
    VcomStatsVendor,
);
#[cfg(not(feature = "accel"))]
macro_rules! probe_deps {
    ($parts:expr) => {
        $parts.deps
    };
}

#[cfg(feature = "accel")]
macro_rules! probe_deps {
    ($parts:expr) => {{
        static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        let pins = PioSwdPins {
            swclk: $parts.swclk,
            swdio_out: $parts.swdio,
            swdio_in: None,
            oen: None,
            nreset: Some($parts.nreset),
        };
        let jtag = PioJtagPins {
            tck: $parts.swclk,
            tms: $parts.swdio,
            tdi: $parts.tdi,
            tdo: $parts.tdo,
        };
        // SAFETY: bring-up reserved both PIO blocks and these pads; this is PIO1's sole owner.
        unsafe { pio_transfer_deps(pins, jtag, CPU_HZ, &SCAN) }
    }};
}

#[cfg(feature = "carrier-leds")]
const HEARTBEAT_PERIOD_MS: u64 = 500;

#[cfg(feature = "carrier-leds")]
#[embassy_executor::task]
async fn heartbeat_task(mut led: CarrierBankLed) -> ! {
    loop {
        embassy_time::Timer::after_millis(HEARTBEAT_PERIOD_MS).await;
        led.pulse();
    }
}

#[cfg(feature = "flash")]
#[embassy_executor::task]
async fn msc_task(
    mut msc: MscEndpoints<'static, UsbDrv>,
    vfs: &'static mut FlashVolume<Sink>,
) -> ! {
    daplink_embassy_usb::msc::msc_task_with_reconnect(&mut msc, vfs, RpUsbReconnect).await
}

#[cfg(feature = "flash")]
daplink_vfs::retained_stamp!();

#[cfg(feature = "flash")]
type Engine = daplink_rs::flash_mux::MuxDapEngine<
    'static,
    MuxDeps,
    MuxSwd,
    MuxJtag,
    CycleDelay,
    daplink_rp_embassy::Uart0Swo,
    Vendor,
    daplink_rs::StatusLed<embassy_rp::gpio::Output<'static>>,
    EngineFence,
>;
#[cfg(all(feature = "flash", feature = "accel"))]
type EngineFence = &'static RpPioTransferFence;
#[cfg(all(feature = "flash", not(feature = "accel")))]
type EngineFence = daplink_rs::NoCommandFence;

#[cfg(feature = "carrier-leds")]
type FlowLed = CarrierBankLed;
#[cfg(not(feature = "carrier-leds"))]
type FlowLed = ();

/// What the DAP loop keeps from the boot-time composition.
struct Composed {
    engine: &'static mut Engine,
    class: daplink_embassy_usb::DapV2Class<daplink_rp_embassy::UsbDrv>,
    #[cfg(feature = "flash")]
    mux: &'static Mux,
    dap_led: FlowLed,
    flash_led: FlowLed,
}

/// Boot-time composition, kept out of the main future so its temporaries leave the poll frame.
#[inline(never)]
fn setup(spawner: Spawner) -> Composed {
    #[cfg(feature = "flash")]
    let stamp = unsafe {
        // SAFETY: dedicated NOLOAD SRAM is readable at entry; no DMA or other core accesses it.
        daplink_vfs::stamp::boot_retained(core::ptr::addr_of_mut!(__daplink_retained_stamp).cast())
    };
    #[cfg(all(feature = "flash", not(feature = "defmt")))]
    // SAFETY: only this core runs terminal, non-returning fault handlers.
    unsafe {
        daplink_vfs::stamp::arm_terminal_faults(stamp.fault)
    };

    #[cfg(feature = "defmt")]
    defmt::info!("rp2350-pico2-embassy: boot");

    let p = init_rp(CLOCK_PROFILE);
    let delay = CycleDelay::new(CLOCK_PROFILE);
    #[cfg(not(feature = "accel"))]
    let parts = probe_parts_pico2(p, delay);
    #[cfg(feature = "accel")]
    let parts = probe_parts_pico2_accel(p);
    let cfg = DapConfig::daplink(parts.serial, DAP_ENDPOINT)
        .with_product(product_string!("rp2350-pico2"));
    let identity = daplink_rs::daplink_core::Identity {
        vendor: cfg.manufacturer,
        product: cfg.product,
        serial: cfg.serial,
        board_vendor: "daplink-rs",
        board_name: "rp2350-pico2",
    };
    #[cfg(feature = "defmt")]
    defmt::info!("rp2350-pico2-embassy: serial {=str}", parts.serial);
    let status_led = status_led(parts.status_pin);
    #[cfg(feature = "carrier-leds")]
    let carrier = carrier_leds(parts.carrier_pins);

    // SWO: the UART0 RX DMA ring stays halted until DAP_SWO_Control(Start).
    #[cfg(not(feature = "carrier-leds"))]
    let swo = parts.swo;
    #[cfg(feature = "carrier-leds")]
    let swo = {
        static SWO_LED: static_cell::StaticCell<CarrierBankLed> = static_cell::StaticCell::new();
        let mut swo = parts.swo;
        swo.source_mut().attach_led(SWO_LED.init(carrier.swo));
        swo
    };

    #[cfg(feature = "carrier-leds")]
    spawner.spawn(heartbeat_task(carrier.heartbeat).expect("heartbeat_task spawns once"));

    #[cfg(not(feature = "flash"))]
    let engine: Engine = {
        #[cfg(not(feature = "accel"))]
        let deps = probe_deps!(parts);
        #[cfg(feature = "accel")]
        let (deps, fence) = probe_deps!(parts);
        let engine = daplink_rs::DapProbe::with_vendor(
            deps,
            status_led,
            delay,
            swo,
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
        engine
            .with_packet_count(DAP_PACKET_COUNT)
            .with_identity(identity)
    };
    #[cfg(feature = "flash")]
    let (engine, vfs, mux) = {
        static MUX: static_cell::StaticCell<Mux> = static_cell::StaticCell::new();
        static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> = ConstStaticCell::new([0; SCRATCH]);
        static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

        #[cfg(not(feature = "accel"))]
        let deps = probe_deps!(parts);
        #[cfg(feature = "accel")]
        let (deps, fence) = probe_deps!(parts);
        #[cfg(feature = "accel")]
        let fence: &'static RpPioTransferFence = {
            static FENCE: static_cell::StaticCell<RpPioTransferFence> =
                static_cell::StaticCell::new();
            FENCE.init(fence)
        };
        let mux: &'static Mux = MUX.init(SwdMux::new(deps));
        let table = flash_table();
        let vendor_backend = mux_flash_backend(mux, delay, PRIMARY, table, VENDOR_SCRATCH.take());
        #[cfg(feature = "accel")]
        let vendor_backend = FencedFlashBackend::new(vendor_backend, fence);
        let vendor: Vendor = (
            RebootVendor::with_busy_check(RpUsbBoot, mux),
            VendorFlash::new(FlashSink::new(vendor_backend)),
            VcomBaudVendor::new(vcom_baud_state),
            VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
        );
        let engine = mux_dap_engine(mux, delay, swo, DAP_PACKET, &MUX_SCAN, vendor, status_led);
        #[cfg(feature = "accel")]
        let engine = engine.with_command_fence(fence);
        let engine = engine
            .with_packet_count(DAP_PACKET_COUNT)
            .with_identity(identity);
        let vfs_backend = mux_flash_backend(mux, delay, PRIMARY, table, flash_scratch());
        #[cfg(feature = "accel")]
        let vfs_backend = FencedFlashBackend::new(vfs_backend, fence);
        let vfs = daplink_vfs::assemble_info(
            &VOLUME,
            FlashSink::new(vfs_backend),
            daplink_vfs::DeviceInfo {
                version: env!("CARGO_PKG_VERSION"),
                serial: parts.serial,
                reset_count: stamp.snapshot.reset_count(),
            },
            Some((stamp.snapshot, stamp.status, stamp.steady)),
            "DAPLINK",
        );
        (engine, vfs, mux)
    };

    // embassy-rp brings USB up for the RP2040 only: fix RESETS, `PHY_ISO` and `INTE` first.
    embassy_rp::pac::RESETS
        .reset()
        .modify(|w| w.set_usbctrl(false));
    while !embassy_rp::pac::RESETS.reset_done().read().usbctrl() {}
    embassy_rp::pac::USB
        .main_ctrl()
        .modify(|w| w.set_phy_iso(false));
    embassy_rp::pac::USB.inte().write(|_w| {});

    let (usb_io, vcom_io) = parts.io.split();
    let driver = RpUsbDriver::new(Driver::new(usb_io, Irqs));
    // `Driver::new` zeroes `LINESTATE_TUNING`; the datasheet says to keep its reset value.
    embassy_rp::pac::USB
        .linestate_tuning()
        .write_value(embassy_rp::pac::usb::regs::LinestateTuning(0x0f8));
    #[cfg(not(feature = "flash"))]
    let (usb, class, cdc_tx, cdc_rx, control) = daplink_embassy_usb::usb_device(driver, &cfg);
    #[cfg(feature = "flash")]
    let (usb, class, cdc_tx, cdc_rx, control, msc) =
        daplink_embassy_usb::usb_device_msc(driver, &cfg);
    #[cfg(feature = "flash")]
    spawner.spawn(msc_task(msc, vfs).expect("msc_task spawns once"));

    #[cfg(feature = "carrier-leds")]
    let vcom_led: &'static mut dyn Activity = {
        static VCOM_LED: static_cell::StaticCell<CarrierBankLed> = static_cell::StaticCell::new();
        VCOM_LED.init(carrier.vcom)
    };
    #[cfg(not(feature = "carrier-leds"))]
    let vcom_led: &'static mut dyn Activity = daplink_rs::noop_activity();
    spawn_probe_services(spawner, usb, cdc_tx, cdc_rx, control, vcom_io, vcom_led);

    #[cfg(feature = "carrier-leds")]
    let (dap_led, flash_led) = (carrier.dap, carrier.flash);
    #[cfg(not(feature = "carrier-leds"))]
    let (dap_led, flash_led) = ((), ());

    static ENGINE: static_cell::StaticCell<Engine> = static_cell::StaticCell::new();
    Composed {
        engine: ENGINE.init(engine),
        class,
        #[cfg(feature = "flash")]
        mux,
        dap_led,
        flash_led,
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let Composed {
        engine,
        class,
        #[cfg(feature = "flash")]
        mux,
        mut dap_led,
        mut flash_led,
    } = setup(spawner);
    static APP_STATE: ConstStaticCell<AppState> = ConstStaticCell::new(AppState::new());
    dap_sessions::<{ cfg!(feature = "defmt") }, _, _, _, _, _, _>(
        class,
        APP_STATE.take(),
        engine,
        |dap, request, response| {
            let len = dap.process_command(request, response);
            dap_led.pulse();
            #[cfg(feature = "flash")]
            let flashing = mux.is_flashing();
            #[cfg(not(feature = "flash"))]
            let flashing = false;
            flash_led.level(flashing);
            len
        },
        |dap| dap.suspend(),
        |_dap, _processed| {
            #[cfg(feature = "defmt")]
            defmt::info!("dap session: {=u32} commands processed", _processed);
        },
    )
    .await
}
