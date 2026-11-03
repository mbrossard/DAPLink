//! CMSIS-DAP v2 probe for STM32F103CB boards on embassy; `nanodap` selects that board's wiring.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
#[cfg(feature = "nanodap")]
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{ControlChanged, Receiver, Sender};
use embassy_usb::UsbDevice;
use static_cell::ConstStaticCell;

// A flash build latches the fault into `RESET_STAMP` and resets; every other build halts.
// `defmt` halts in every build, so the RTT-attached probe sees the panic message.
#[cfg(all(feature = "flash", not(feature = "defmt")))]
daplink_vfs::fault_reset_handlers!();
#[cfg(all(not(feature = "flash"), not(feature = "defmt")))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use daplink_embassy_usb::{dap_sessions, vcom_bridge, DapPipeline, DapV2Class, PipelineSession};
#[cfg(not(feature = "flash"))]
use daplink_rs::daplink_core::DAP_FW_VERSION;
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, EndpointSize, PacketSize, DAP_PACKET_SIZE,
    DEFAULT_VCOM_BAUD,
};
#[cfg(all(feature = "nanodap", not(feature = "flash"), not(feature = "accel")))]
use daplink_rs::BitbangDap;
#[cfg(feature = "nanodap")]
use daplink_rs::NoSwo;
#[cfg(feature = "nanodap")]
use daplink_rs::StatusLed;
#[cfg(not(feature = "nanodap"))]
use daplink_rs::SwdOnlyDap;
use daplink_rs::{NoBootloader, RebootVendor, VcomBaudVendor, VcomStatsVendor};
#[cfg(all(feature = "accel", not(feature = "flash")))]
use daplink_spi_swd::NoFastJtag;
#[cfg(all(feature = "nanodap", not(feature = "accel")))]
use daplink_stm32f1_embassy::swd_jtag_deps;
#[cfg(not(all(feature = "accel", feature = "flash")))]
use daplink_stm32f1_embassy::IoPin;
use daplink_stm32f1_embassy::{
    clocks, mask_unhandled_interrupts, serial, vcom_baud_state, DmaVcom, DynCycleDelay, F1UsbDriver,
};
#[cfg(not(feature = "nanodap"))]
use daplink_stm32f1_embassy::{swd_only_deps, usart1_swo, BicolorLed, UartSwo};
#[cfg(all(feature = "accel", not(feature = "flash")))]
use daplink_stm32f1_spi_swd::SpiSwdJtagDap;

#[cfg(feature = "flash")]
use daplink_embassy_usb::msc::MscEndpoints;
#[cfg(feature = "flash")]
use daplink_rs::flash_mux::{mux_dap_engine, mux_flash_backend, MuxDapEngine, MuxFlashBackend};
#[cfg(all(feature = "flash", not(feature = "accel")))]
use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv};
#[cfg(feature = "flash")]
use daplink_rs::{PooledFlashBackend, StagingPool};
#[cfg(feature = "flash")]
use daplink_rs::{ScanChainStorage, SwdMux};
#[cfg(feature = "flash")]
use daplink_stm32f1_embassy::UsbConnect;
#[cfg(feature = "flash-nrf52833")]
use daplink_targets::targets::nrf52833::NRF52833;
#[cfg(feature = "flash-nrf52840")]
use daplink_targets::targets::nrf52840::NRF52840;
#[cfg(feature = "flash-stm32f401")]
use daplink_targets::targets::stm32f401ce::STM32F401CE;
#[cfg(feature = "flash")]
use daplink_vfs::{AllDecoders, FlashSink, FlashVolume, VendorFlash, VolumeStorage};

#[cfg(all(feature = "flash", not(feature = "nanodap")))]
compile_error!("`flash` composes only on the `nanodap` board wiring");

#[cfg(feature = "accel")]
mod spi_swd;

// The seam's pin map (TDO on PA5, the D+ switch off PB15) is the nanoDAP's.
#[cfg(all(feature = "accel", not(feature = "nanodap")))]
compile_error!("`accel` composes only on the `nanodap` board wiring");

// Two board mappings: the plain board (PA9 bicolor LED, PA15 D+ switch, USART1 SWO, SWD only)
// and the nanoDAP (PC13 LED, PB15 D+ switch, JTAG on PA7/PA5, no SWO); `flash` and `accel` need it.

type UsbDrv = F1UsbDriver<'static>;

/// The PA9 bicolor COM LED.
#[cfg(not(feature = "nanodap"))]
type StatusLedType = BicolorLed;
/// The nanoDAP status LED on PC13, active-low.
#[cfg(feature = "nanodap")]
type StatusLedType = StatusLed<Output<'static>>;

/// The F103 ROM bootloader is USART-only, so reboot is declined.
#[cfg(not(feature = "flash"))]
type Vendor = (RebootVendor<NoBootloader>, VcomBaudVendor, VcomStatsVendor);

/// Plain board: SWD only, with USART1 SWO.
#[cfg(not(feature = "nanodap"))]
type Engine = SwdOnlyDap<'static, IoPin, DynCycleDelay, UartSwo, Vendor, StatusLedType>;

/// nanoDAP: bit-bang SWD and JTAG; the board routes no SWO.
#[cfg(all(feature = "nanodap", not(feature = "flash"), not(feature = "accel")))]
type Engine = BitbangDap<'static, IoPin, DynCycleDelay, NoSwo, Vendor, StatusLedType>;

/// nanoDAP `accel`: SPI-assisted SWD, bit-bang JTAG.
#[cfg(all(feature = "accel", not(feature = "flash")))]
type Engine = SpiSwdJtagDap<
    'static,
    spi_swd::Board,
    IoPin,
    DynCycleDelay,
    NoFastJtag,
    NoSwo,
    Vendor,
    StatusLedType,
>;

/// nanoDAP `flash`: the wire engine the DAP and flash roles share, SPI-assisted under `accel`.
#[cfg(all(feature = "flash", feature = "accel"))]
type MuxDeps = spi_swd::AccelDeps;
#[cfg(all(feature = "flash", not(feature = "accel")))]
type MuxDeps = LateJtagDeps<IoPin, DynCycleDelay>;
#[cfg(all(feature = "flash", feature = "accel"))]
type MuxSwd = spi_swd::AccelSwd;
#[cfg(all(feature = "flash", not(feature = "accel")))]
type MuxSwd = LateSwdDrv<IoPin, DynCycleDelay>;
#[cfg(all(feature = "flash", feature = "accel"))]
type MuxJtag = spi_swd::AccelJtag;
#[cfg(all(feature = "flash", not(feature = "accel")))]
type MuxJtag = LateJtag<IoPin, DynCycleDelay>;

#[cfg(feature = "flash")]
type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
#[cfg(feature = "flash")]
type Backend = PooledFlashBackend<
    'static,
    MuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, PrimaryTarget, Selector>,
>;
#[cfg(feature = "flash")]
type Sink = FlashSink<Backend>;
#[cfg(feature = "flash")]
type RebootBusy = &'static Mux;
#[cfg(feature = "flash")]
type Vendor = (
    RebootVendor<NoBootloader, RebootBusy>,
    &'static mut VendorFlash<AllDecoders, Sink>,
    VcomBaudVendor,
    VcomStatsVendor,
);

/// nanoDAP `flash`: the DAP role of the shared engine.
#[cfg(feature = "flash")]
type Engine =
    MuxDapEngine<'static, MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, NoSwo, Vendor, StatusLedType>;

#[cfg(feature = "flash-stm32f401")]
type PrimaryTarget = STM32F401CE;

#[cfg(feature = "flash-stm32f401")]
static PRIMARY: &PrimaryTarget = &STM32F401CE;
#[cfg(all(feature = "flash-nrf52840", not(feature = "flash-stm32f401")))]
type PrimaryTarget = NRF52840;
#[cfg(all(feature = "flash-nrf52840", not(feature = "flash-stm32f401")))]
static PRIMARY: &PrimaryTarget = &NRF52840;
#[cfg(all(
    feature = "flash-nrf52833",
    not(any(feature = "flash-stm32f401", feature = "flash-nrf52840"))
))]
type PrimaryTarget = NRF52833;
#[cfg(all(
    feature = "flash-nrf52833",
    not(any(feature = "flash-stm32f401", feature = "flash-nrf52840"))
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
    // The scratch must fit 20 KiB of RAM; the nRF52's 4 KiB pages program in chunks.
    cap = 0x400,
    rows = [
        ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
        ("flash-nrf52840", NRF52840, daplink_targets::targets::nrf52840::PROGRAM_BUFFER_SIZE),
        ("flash-nrf52833", NRF52833, daplink_targets::targets::nrf52833::PROGRAM_BUFFER_SIZE),
    ],
}

// Both receive paths are polled DMA rings, so only the VCOM transmit channel is bound.
bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => InterruptHandler<peripherals::USB>;
    // USART2_TX is DMA1 channel 7.
    DMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH7>;
});

/// No fixed work: a write's ACK-to-turnaround cell is two halves of bare store and spin.
const SWD_FIXED_WORK_CYCLES: u32 = 0;

/// Floor on the residual spin, in core cycles; zero runs flat out.
const MIN_HALF_PERIOD_CYCLES: u32 = 0;

/// Floor on the accel JTAG residual spin; below it TDO is sampled too close to the TCK fall.
#[cfg(feature = "accel")]
const JTAG_MIN_HALF_PERIOD_CYCLES: u32 = 8;

/// Command-queue depth, also the advertised `DAP_PACKET_COUNT`.
const DAP_QUEUE_DEPTH: usize = 4;

const DAP_QUEUE_SLOT: usize = DAP_PACKET_SIZE as usize;

type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

const DAP_PACKET: PacketSize = Queue::packet_size::<DAP_PACKET_SIZE>();
const DAP_ENDPOINT: EndpointSize = Queue::endpoint_size::<DAP_PACKET_SIZE>();

type Pipeline = DapPipeline<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

const DAP_PACKET_COUNT: u8 = DAP_QUEUE_DEPTH as u8;

// Advertised count, ring depth and slot size are one fact.
const _: () = {
    assert!(DAP_QUEUE_DEPTH <= u8::MAX as usize);
    assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
    assert!(Queue::new().packet_capacity() == DAP_PACKET_SIZE as usize);
};

/// EP0 size: 32 bytes, freeing packet memory for the class endpoints.
const EP0_SIZE: u8 = 32;

/// MSC bulk packet size: what the 512-byte packet memory leaves after the other endpoints.
#[cfg(feature = "flash")]
const MSC_EP: usize = 32;

/// VCOM receive ring: must drain within 22 ms at 921600 baud.
const VCOM_RX_RING: usize = 2048;

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDrv>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn vcom_task(
    mut cdc_tx: Sender<'static, UsbDrv>,
    mut cdc_rx: Receiver<'static, UsbDrv>,
    control: ControlChanged<'static>,
    mut uart: DmaVcom,
    applied_baud: u32,
) -> ! {
    vcom_bridge(
        &mut cdc_tx,
        &mut cdc_rx,
        &control,
        &mut uart,
        applied_baud,
        daplink_rs::noop_activity(),
    )
    .await
}

/// Mass-storage transport, bouncing the PB15 D+ switch on each remount.
#[cfg(feature = "flash")]
#[embassy_executor::task]
async fn msc_task(
    mut msc: MscEndpoints<'static, UsbDrv, MSC_EP>,
    vfs: &'static mut FlashVolume<Sink>,
    connect: UsbConnect<'static>,
) -> ! {
    daplink_embassy_usb::msc::msc_task_with_reconnect(&mut msc, vfs, connect).await
}

#[cfg(feature = "flash")]
daplink_vfs::retained_stamp!();

async fn dap_loop_pipelined(
    class: DapV2Class<UsbDrv>,
    dap: &mut Engine,
    pipeline: &mut Pipeline,
) -> ! {
    let (reader, writer) = class.split();
    let mut session = PipelineSession::new(reader, writer, pipeline);
    dap_sessions::<{ cfg!(feature = "defmt") }, _, _, _, _, _>(
        &mut session,
        dap,
        |dap, request, response| dap.process_command(request, response),
        |dap| dap.suspend(),
        |_dap, _processed| {
            #[cfg(feature = "defmt")]
            defmt::info!("dap session: {=u32} commands processed", _processed);
        },
    )
    .await
}

// Each builder's temporaries die before the next runs, so the boot stack holds one at a time.
#[cfg(feature = "flash")]
#[inline(never)]
fn build_engine(
    mux: &'static Mux,
    table: &'static [&'static dyn daplink_targets::Target],
    delay: DynCycleDelay,
    swo: NoSwo,
    led: StatusLedType,
    identity: daplink_rs::daplink_core::Identity,
) -> &'static mut Engine {
    static ENGINE: static_cell::StaticCell<Engine> = static_cell::StaticCell::new();
    static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
        ScanChainStorage::new();
    ENGINE.init_with(|| {
        mux_dap_engine(
            mux,
            delay,
            swo,
            DAP_PACKET,
            &MUX_SCAN,
            (
                RebootVendor::with_busy_check(NoBootloader, mux),
                build_vendor_flash(mux, table, delay),
                VcomBaudVendor::new(vcom_baud_state),
                VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
            ),
            led,
        )
        .with_packet_count(DAP_PACKET_COUNT)
        .with_identity(identity)
    })
}

#[cfg(feature = "flash")]
#[inline(never)]
fn build_vendor_flash(
    mux: &'static Mux,
    table: &'static [&'static dyn daplink_targets::Target],
    delay: DynCycleDelay,
) -> &'static mut VendorFlash<AllDecoders, Sink> {
    static VENDOR_FLASH: static_cell::StaticCell<VendorFlash<AllDecoders, Sink>> =
        static_cell::StaticCell::new();
    VENDOR_FLASH.init_with(|| VendorFlash::new(FlashSink::new(pooled_backend(mux, table, delay))))
}

/// The vendor and mass-storage backends share one scratch; the mux keeps their sessions apart.
#[cfg(feature = "flash")]
static STAGING: StagingPool<'static> = StagingPool::new();

#[cfg(feature = "flash")]
fn pooled_backend(
    mux: &'static Mux,
    table: &'static [&'static dyn daplink_targets::Target],
    delay: DynCycleDelay,
) -> Backend {
    PooledFlashBackend::new(
        mux_flash_backend(mux, delay, PRIMARY, table, &mut []),
        &STAGING,
    )
}

#[cfg(feature = "flash")]
#[inline(never)]
fn build_volume(
    volume: &'static VolumeStorage<Sink>,
    mux: &'static Mux,
    table: &'static [&'static dyn daplink_targets::Target],
    delay: DynCycleDelay,
    serial: &'static str,
    stamp: (
        daplink_vfs::BootSnapshot,
        daplink_vfs::stamp::StatusWriter<'static>,
        daplink_vfs::stamp::SteadyToken<'static>,
    ),
) -> &'static mut FlashVolume<Sink> {
    daplink_vfs::assemble_info(
        volume,
        FlashSink::new(pooled_backend(mux, table, delay)),
        daplink_vfs::DeviceInfo {
            version: env!("CARGO_PKG_VERSION"),
            serial,
            reset_count: stamp.0.reset_count(),
        },
        Some(stamp),
        "DAPLINK",
    )
}

/// The USB, VCOM and mass-storage peripherals `start_usb` consumes.
struct UsbParts {
    usb: embassy_stm32::Peri<'static, peripherals::USB>,
    dp: embassy_stm32::Peri<'static, peripherals::PA12>,
    dm: embassy_stm32::Peri<'static, peripherals::PA11>,
    usart: embassy_stm32::Peri<'static, peripherals::USART2>,
    tx: embassy_stm32::Peri<'static, peripherals::PA2>,
    rx: embassy_stm32::Peri<'static, peripherals::PA3>,
    tx_dma: embassy_stm32::Peri<'static, peripherals::DMA1_CH7>,
    rx_dma: embassy_stm32::Peri<'static, peripherals::DMA1_CH6>,
    #[cfg(not(feature = "nanodap"))]
    connect: embassy_stm32::Peri<'static, peripherals::PA15>,
    #[cfg(feature = "nanodap")]
    connect: embassy_stm32::Peri<'static, peripherals::PB15>,
}

/// Build the USB device, spawn its tasks and attach; returns the DAP class.
#[inline(never)]
fn start_usb(
    spawner: Spawner,
    p: UsbParts,
    cfg: &DapConfig<'static>,
    #[cfg(feature = "flash")] vfs: &'static mut FlashVolume<Sink>,
) -> DapV2Class<UsbDrv> {
    // A switched D+ pull-up (PA15, or PB15 on the nanoDAP), detached until the device exists.
    #[cfg(not(feature = "nanodap"))]
    let mut usb_connect = daplink_stm32f1_embassy::usb_connect_pa15(p.connect);
    #[cfg(feature = "nanodap")]
    let mut usb_connect = daplink_stm32f1_embassy::usb_connect_pb15(p.connect);

    #[cfg(not(feature = "flash"))]
    let (usb, class, cdc_tx, cdc_rx, control) = daplink_embassy_usb::usb_device(
        F1UsbDriver::new(Driver::new(p.usb, Irqs, p.dp, p.dm)),
        cfg,
    );
    #[cfg(feature = "flash")]
    let (usb, class, cdc_tx, cdc_rx, control, msc) = daplink_embassy_usb::usb_device_msc::<_, MSC_EP>(
        F1UsbDriver::new(Driver::new(p.usb, Irqs, p.dp, p.dm)),
        cfg,
    );
    spawner.spawn(usb_task(usb).expect("usb_task spawns once"));

    // VCOM: USART2 (TX=PA2, RX=PA3), both legs on DMA -- the seam never reads the data register.
    static RX_RING: ConstStaticCell<[u8; VCOM_RX_RING]> = ConstStaticCell::new([0; VCOM_RX_RING]);
    let uart = DmaVcom::new_usart2(
        p.usart,
        p.tx,
        p.rx,
        p.tx_dma,
        p.rx_dma,
        Irqs,
        RX_RING.take(),
    );
    spawner.spawn(
        vcom_task(cdc_tx, cdc_rx, control, uart, DEFAULT_VCOM_BAUD).expect("vcom_task spawns once"),
    );

    usb_connect.attach();
    // The mass-storage task now owns the D+ switch.
    #[cfg(feature = "flash")]
    spawner.spawn(msc_task(msc, vfs, usb_connect).expect("msc_task spawns once"));

    // The switch must stay driven for as long as the device runs.
    #[cfg(not(feature = "flash"))]
    core::mem::forget(usb_connect);
    class
}

/// Boot-time composition, kept out of the main future so its temporaries leave the poll frame.
#[inline(never)]
fn setup(spawner: Spawner) -> (&'static mut Engine, DapV2Class<UsbDrv>) {
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
    defmt::info!("stm32f103-embassy: boot");

    #[cfg_attr(feature = "nanodap", allow(unused_mut))]
    let mut config = clocks::clocks_72m_hse8m();
    // PA15 carries the USB-connect switch and is JTDI at reset: keep SW-DP only, so it is GPIO.
    #[cfg(not(feature = "nanodap"))]
    {
        config.swj = embassy_stm32::gpio::SwjCfg::SwdOnly;
    }
    let p = embassy_stm32::init(config);

    // `init` NVIC-enables every DMA line, bound or not, and an unbound one vectors into a loop.
    // SAFETY: nothing here services an interrupt other than through its vector-table entry.
    let _unhandled = unsafe { mask_unhandled_interrupts() };
    #[cfg(feature = "defmt")]
    defmt::info!("masked {=u32} unhandled NVIC lines", _unhandled);

    // Pin map: SWCLK=PB13, SWDIO=PB14, nRESET=PB0, plus nanoDAP JTAG on TDI=PA7, TDO=PA5.
    let delay = DynCycleDelay::with_fixed_work(
        clocks::CPU_HZ,
        SWD_FIXED_WORK_CYCLES,
        MIN_HALF_PERIOD_CYCLES,
    );
    #[cfg(not(feature = "nanodap"))]
    let deps = swd_only_deps(p.PB0, p.PB14, p.PB13, delay);
    #[cfg(all(feature = "nanodap", not(feature = "accel")))]
    let deps = swd_jtag_deps(p.PB0, p.PA7, p.PB14, p.PB13, p.PA5, delay);

    #[cfg(feature = "accel")]
    let deps = spi_swd::accel_deps(
        p.SPI2,
        p.PB13,
        p.PB14,
        p.PB0,
        p.PA7,
        p.PA5,
        delay,
        DynCycleDelay::with_fixed_work(clocks::CPU_HZ, 0, JTAG_MIN_HALF_PERIOD_CYCLES),
        embassy_stm32::rcc::frequency::<peripherals::SPI2>().0,
    );

    let serial = serial();
    let mut cfg =
        DapConfig::daplink(serial, DAP_ENDPOINT).with_product(product_string!("stm32f103"));
    let identity = daplink_rs::daplink_core::Identity {
        vendor: cfg.manufacturer,
        product: cfg.product,
        serial: cfg.serial,
        board_vendor: "daplink-rs",
        board_name: "stm32f103",
    };
    cfg.max_packet_size_0 = EP0_SIZE;
    #[cfg(feature = "defmt")]
    defmt::info!(
        "usb identity: {=str} serial {=str}",
        cfg.product,
        cfg.serial
    );

    // SWO on PA10 into a DMA1 CH5 ring; no embassy driver, so the clock is enabled here.
    #[cfg(not(feature = "nanodap"))]
    let swo = {
        embassy_stm32::pac::RCC
            .apb2enr()
            .modify(|w| w.set_usart1en(true));
        let _ = embassy_stm32::pac::RCC.apb2enr().read();
        usart1_swo(
            p.USART1,
            embassy_stm32::rcc::frequency::<peripherals::USART1>().0,
            p.PA10,
            p.DMA1_CH5,
        )
    };
    // nanodap: the board routes no trace signal.
    #[cfg(feature = "nanodap")]
    let swo = NoSwo;

    // PA9 bicolor pair: high = red (activity), low = green (idle).
    #[cfg(not(feature = "nanodap"))]
    let led = {
        let mut led = BicolorLed::new(p.PA9);
        led.idle();
        led
    };
    // nanodap: LED on PC13, active-low.
    #[cfg(feature = "nanodap")]
    let led = StatusLed::new_active_low(Output::new(p.PC13, Level::High, Speed::Low));

    #[cfg(not(feature = "flash"))]
    let engine: &'static mut Engine = {
        static ENGINE: static_cell::StaticCell<Engine> = static_cell::StaticCell::new();
        ENGINE.init_with(|| {
            Engine::with_vendor(
                deps,
                led,
                delay,
                swo,
                DAP_FW_VERSION,
                DAP_PACKET,
                (
                    RebootVendor::new(NoBootloader),
                    VcomBaudVendor::new(vcom_baud_state),
                    VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
                ),
            )
            .with_packet_count(DAP_PACKET_COUNT)
            .with_identity(identity)
        })
    };
    #[cfg(feature = "flash")]
    let (engine, vfs) = {
        static MUX: static_cell::StaticCell<Mux> = static_cell::StaticCell::new();
        static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

        let mux: &'static Mux = MUX.init(SwdMux::new(deps));
        let table = flash_table();
        STAGING.put(flash_scratch());
        let engine = build_engine(mux, table, delay, swo, led, identity);
        let vfs = build_volume(
            &VOLUME,
            mux,
            table,
            delay,
            serial,
            (stamp.snapshot, stamp.status, stamp.steady),
        );
        (engine, vfs)
    };

    #[cfg(not(feature = "nanodap"))]
    let connect = p.PA15;
    #[cfg(feature = "nanodap")]
    let connect = p.PB15;
    let class = start_usb(
        spawner,
        UsbParts {
            usb: p.USB,
            dp: p.PA12,
            dm: p.PA11,
            usart: p.USART2,
            tx: p.PA2,
            rx: p.PA3,
            tx_dma: p.DMA1_CH7,
            rx_dma: p.DMA1_CH6,
            connect,
        },
        &cfg,
        #[cfg(feature = "flash")]
        vfs,
    );

    (engine, class)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (engine, class) = setup(spawner);
    static DAP_PIPELINE: ConstStaticCell<Pipeline> = ConstStaticCell::new(Pipeline::new());
    let pipeline = DAP_PIPELINE.take();

    dap_loop_pipelined(class, engine, pipeline).await
}
