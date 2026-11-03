//! CMSIS-DAP v2 probe for the WeAct STM32H523 core board on embassy: bit-bang, or `accel` on SPI2.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{Interrupt, InterruptExt, Priority};
use embassy_stm32::pac;
use embassy_stm32::peripherals;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{ControlChanged, Receiver, Sender};
use embassy_usb::UsbDevice;
use static_cell::ConstStaticCell;

// A flash build latches the fault into `RESET_STAMP` and resets; every other build halts.
#[cfg(all(feature = "flash", not(feature = "defmt")))]
daplink_vfs::fault_reset_handlers!();
#[cfg(all(not(feature = "flash"), not(feature = "defmt")))]
use panic_halt as _;
// `defmt` halts in every build, so the RTT-attached probe sees the panic message.
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use daplink_embassy_usb::{
    dap_reader_task, dap_writer_task, DapReadHalf, DapV2Reader, DapV2Writer, DapWriteHalf,
};
use daplink_embassy_usb::{dap_sessions, vcom_bridge, DapPipeline, DapV2Class};
#[cfg(not(feature = "flash"))]
use daplink_rs::daplink_core::DAP_FW_VERSION;
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, EndpointSize, PacketSize, DAP_PACKET_SIZE,
    DEFAULT_VCOM_BAUD,
};
#[cfg(all(not(feature = "flash"), feature = "plain-bitbang"))]
use daplink_rs::BitbangDap;
#[cfg(all(
    not(feature = "flash"),
    not(any(feature = "accel", feature = "plain-bitbang"))
))]
use daplink_rs::DapProbe;
use daplink_rs::{BootloaderEntry, RebootVendor, StatusLed, VcomBaudVendor, VcomStatsVendor};
#[cfg(not(feature = "accel"))]
use daplink_stm32_embassy::swd_jtag_deps_at;
#[cfg(any(
    all(feature = "accel", not(feature = "flash")),
    feature = "plain-bitbang"
))]
use daplink_stm32_embassy::IoPin;
#[cfg(all(feature = "accel", not(feature = "flash")))]
use daplink_stm32_embassy::SpiSwdJtagDap;
use daplink_stm32_embassy::{
    check_and_enter_bootloader, configure_factory_ro_region, enable_instruction_cache,
    mask_unhandled_interrupts, reboot_armed, serial, uart_swo, vcom_baud_state, DynCycleDelay,
    H5UsbDriver, StmSystemDfu, StmVcom, UartSwo, VcomInterruptHandler,
};

#[cfg(feature = "flash")]
use daplink_embassy_usb::msc::MscEndpoints;
#[cfg(feature = "flash")]
use daplink_rs::flash_mux::{mux_dap_engine, mux_flash_backend, MuxDapEngine, MuxFlashBackend};
#[cfg(all(feature = "flash", feature = "plain-bitbang"))]
use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv};
#[cfg(feature = "flash")]
use daplink_rs::{ScanChainStorage, SwdMux};
#[cfg(not(feature = "accel"))]
use daplink_spi_swd::pace::{self, ClassFit};
#[cfg(feature = "wire-probe")]
use daplink_stm32_embassy::run_wire_probe;
#[cfg(feature = "flash")]
use daplink_stm32_embassy::StmUsbReconnect;
#[cfg(not(feature = "accel"))]
use daplink_stm32_embassy::{BlockPacing, H5Block};
#[cfg(feature = "flash-nrf52833")]
use daplink_targets::targets::nrf52833::NRF52833;
#[cfg(feature = "flash-nrf52840")]
use daplink_targets::targets::nrf52840::NRF52840;
#[cfg(feature = "flash-stm32f401")]
use daplink_targets::targets::stm32f401ce::STM32F401CE;
#[cfg(feature = "flash")]
use daplink_vfs::{AllDecoders, FlashSink, FlashVolume, VendorFlash, VolumeStorage};

mod clocks;
mod fits;
#[cfg(any(not(feature = "accel"), feature = "wire-probe"))]
mod pads;

#[cfg(all(feature = "accel", feature = "plain-bitbang"))]
compile_error!("plain-bitbang replaces the fast bit-bang engine; accel is a third engine");

#[cfg(all(
    feature = "spi-block",
    any(feature = "flash", feature = "accel-duplex")
))]
compile_error!("spi-block runs half-duplex and holds the bus without the flash mux");

#[cfg(not(feature = "plain-bitbang"))]
mod jtag;

#[cfg(feature = "accel")]
mod spi_swd;
#[cfg(feature = "dap-trace")]
mod trace;

#[cfg(not(feature = "accel"))]
const CLOCKS: clocks::ClockPlan = clocks::DEFAULT;
#[cfg(feature = "accel")]
const CLOCKS: clocks::ClockPlan = clocks::ACCEL;
// SPI2's ladder rates must be the ones the selected plan's PLLs make.
#[cfg(feature = "accel")]
const _: () = assert!(CLOCKS.feeds(&clocks::SPI2));

/// The bit-bang line's scan chain and JTAG-DP state.
#[cfg(not(feature = "accel"))]
static LINE_SCAN: daplink_rs::ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
    daplink_rs::ScanChainStorage::new();

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
    rows = [
        ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
        ("flash-nrf52840", NRF52840, daplink_targets::targets::nrf52840::PROGRAM_BUFFER_SIZE),
        ("flash-nrf52833", NRF52833, daplink_targets::targets::nrf52833::PROGRAM_BUFFER_SIZE),
    ],
}

bind_interrupts!(struct Irqs {
    // The trace handler runs first: the driver clears the CTR bits ISTR mirrors.
    USB_DRD_FS => #[cfg(feature = "dap-trace")] trace::UsbTraceHandler, InterruptHandler<peripherals::USB>, #[cfg(feature = "dap-trace")] trace::UsbTraceExit;
    USART2 => VcomInterruptHandler;
});

/// System-memory DFU re-entry from the pre-init slot while the no-init magic word is intact.
#[no_mangle]
unsafe extern "C" fn __pre_init() {
    // SAFETY: pre-init context, and the address is the system-memory bootloader's vector table.
    unsafe { check_and_enter_bootloader(0x0BF9_7000) }
}

type UsbDrv = H5UsbDriver<'static>;

/// The onboard status LED: PC13, active-low.
type StatusLedType = StatusLed<Output<'static>>;

#[cfg(not(feature = "flash"))]
type Vendor = (RebootVendor<StmSystemDfu>, VcomBaudVendor, VcomStatsVendor);

#[cfg(all(not(feature = "flash"), feature = "plain-bitbang"))]
type Engine = BitbangDap<'static, IoPin, DynCycleDelay, UartSwo, Vendor, StatusLedType>;

/// The default engine: bit-bang SWD, with lean JTAG scans off the pads seam.
#[cfg(all(
    not(feature = "flash"),
    not(any(feature = "accel", feature = "plain-bitbang"))
))]
type Engine = DapProbe<
    'static,
    jtag::FastJtagDeps,
    StatusLedType,
    DynCycleDelay,
    jtag::FastJtagDrv,
    jtag::FastSwd,
    UartSwo,
    Vendor,
>;

#[cfg(all(not(feature = "flash"), feature = "accel"))]
type Engine = SpiSwdJtagDap<
    'static,
    spi_swd::Port,
    IoPin,
    DynCycleDelay,
    jtag::H5FastJtag,
    UartSwo,
    Vendor,
    StatusLedType,
    3,
>;

#[cfg(all(feature = "flash", feature = "accel"))]
type MuxDeps = spi_swd::AccelDeps;
#[cfg(all(feature = "flash", feature = "plain-bitbang"))]
type MuxDeps = LateJtagDeps<IoPin, DynCycleDelay>;
#[cfg(all(
    feature = "flash",
    not(any(feature = "accel", feature = "plain-bitbang"))
))]
type MuxDeps = jtag::FastJtagDeps;
#[cfg(all(feature = "flash", feature = "accel"))]
type MuxSwd = spi_swd::AccelSwd;
#[cfg(all(feature = "flash", feature = "plain-bitbang"))]
type MuxSwd = LateSwdDrv<IoPin, DynCycleDelay>;
#[cfg(all(
    feature = "flash",
    not(any(feature = "accel", feature = "plain-bitbang"))
))]
type MuxSwd = jtag::FastSwd;
#[cfg(all(feature = "flash", feature = "accel"))]
type MuxJtag = spi_swd::AccelJtag;
#[cfg(all(feature = "flash", feature = "plain-bitbang"))]
type MuxJtag = LateJtag<IoPin, DynCycleDelay>;
#[cfg(all(
    feature = "flash",
    not(any(feature = "accel", feature = "plain-bitbang"))
))]
type MuxJtag = jtag::FastJtagDrv;

#[cfg(feature = "flash")]
type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
#[cfg(feature = "flash")]
type Backend = MuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, PrimaryTarget, Selector>;
#[cfg(feature = "flash")]
type Sink = FlashSink<Backend>;
#[cfg(feature = "flash")]
type RebootBusy = &'static Mux;
/// Reboot, declined with `STATUS_BUSY` during a live flash session.
#[cfg(feature = "flash")]
type Vendor = (
    RebootVendor<StmSystemDfu, RebootBusy>,
    VendorFlash<AllDecoders, Sink>,
    VcomBaudVendor,
    VcomStatsVendor,
);
#[cfg(feature = "flash")]
type Engine =
    MuxDapEngine<'static, MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, UartSwo, Vendor, StatusLedType>;

/// USART1_RX alternate function on PA10 (RM0481 AF tables).
const SWO_RX_AF: u8 = 7;

/// GPDMA hardware request line for USART1_RX (RM0481 request mapping).
const SWO_DMA_REQUEST: u8 = 21;

/// GPDMA1 channel for the SWO ring; `p.GPDMA1_CH0` in `main` names the same one.
const SWO_DMA_CHANNEL: usize = 0;

/// GPDMA request line for USART2 RX.
const VCOM_DMA_REQUEST: u8 = 23;

/// GPDMA1 channel for the VCOM receive ring.
const VCOM_DMA_CHANNEL: usize = 1;

/// GPDMA request line for USART2 TX.
const VCOM_TX_DMA_REQUEST: u8 = 24;

/// GPDMA1 channel for VCOM transmit.
const VCOM_TX_DMA_CHANNEL: usize = 2;

/// `OSPEEDR` class 01 for PB10 and PB15: less ringing, edges still inside 20 ns halves.
#[cfg(not(feature = "accel"))]
const SWD_PAD_SPEED: u32 = 0b01;

#[cfg(not(feature = "accel"))]
const MIN_HALF_PERIOD_CYCLES: u32 = pace::ps_cycles(fits::RESIDENT.floor_ps, clocks::CPU_HZ);

#[cfg(not(feature = "accel"))]
#[derive(Clone, Copy)]
struct BlockWork;

#[cfg(not(feature = "accel"))]
impl BlockPacing for BlockWork {
    type Pads = pads::H5Pads;
    const MAX_WIRE_HZ: u32 = fits::BLOCK_MAX_HZ;
    const CONTROL: ClassFit = fits::BLOCK_CONTROL;
    const ACK: ClassFit = fits::BLOCK_ACK;
    const READ: ClassFit = fits::BLOCK_READ;
    const WRITE: ClassFit = fits::BLOCK_WRITE;
}

/// Command-queue depth, also advertised to the host.
const DAP_QUEUE_DEPTH: usize = 8;

/// The DAP packet size advertised in `DAP_Info`.
#[cfg(not(feature = "dap-packet-512"))]
const DAP_PACKET_BYTES: u16 = DAP_PACKET_SIZE;
#[cfg(feature = "dap-packet-512")]
const DAP_PACKET_BYTES: u16 = 512;

const DAP_QUEUE_SLOT: usize = DAP_PACKET_BYTES as usize;

type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

const DAP_PACKET: PacketSize = Queue::packet_size::<DAP_PACKET_BYTES>();
const DAP_ENDPOINT: EndpointSize = Queue::endpoint_size::<DAP_PACKET_SIZE>();

type Pipeline = DapPipeline<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

const DAP_PACKET_COUNT: u8 = DAP_QUEUE_DEPTH as u8;

// Advertised count, ring depth and slot size are one fact.
const _: () = {
    assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
    assert!(Queue::new().packet_capacity() == DAP_PACKET_BYTES as usize);
};

/// The DAP executor's line: DTS, never enabled here, so only the pender raises it.
const DAP_IRQ: Interrupt = interrupt::DTS;

/// Below USB, USART2 and GPDMA (all P0), above the thread-mode wire loop.
const DAP_IRQ_PRIORITY: Priority = Priority::P2;

static DAP_EXECUTOR: embassy_executor::InterruptExecutor =
    embassy_executor::InterruptExecutor::new();

#[interrupt]
unsafe fn DTS() {
    #[cfg(feature = "dap-trace")]
    trace::dts_enter();
    // SAFETY: the executor's own vector, and `main` starts it before the line is enabled.
    unsafe { DAP_EXECUTOR.on_interrupt() }
    #[cfg(feature = "dap-trace")]
    trace::dts_exit();
}

#[embassy_executor::task]
async fn dap_read_task(
    half: DapReadHalf<'static, DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>,
    reader: DapV2Reader<UsbDrv>,
) -> ! {
    dap_reader_task(half, reader).await
}

#[embassy_executor::task]
async fn dap_write_task(
    half: DapWriteHalf<'static, DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>,
    writer: DapV2Writer<UsbDrv>,
) -> ! {
    dap_writer_task(half, writer).await
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDrv>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn vcom_task(
    mut cdc_tx: Sender<'static, UsbDrv>,
    mut cdc_rx: Receiver<'static, UsbDrv>,
    control: ControlChanged<'static>,
    mut uart: StmVcom,
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

/// Thread-mode reboot: waits for the DAP acknowledgement to drain, then enters the armed DFU.
#[embassy_executor::task]
async fn reboot_task() -> ! {
    loop {
        if reboot_armed() {
            Timer::after_millis(50).await;
            StmSystemDfu.enter();
        }
        Timer::after_millis(25).await;
    }
}

/// Mass-storage transport, bouncing D+ on each remount.
#[cfg(feature = "flash")]
#[embassy_executor::task]
async fn msc_task(
    mut msc: MscEndpoints<'static, UsbDrv>,
    vfs: &'static mut FlashVolume<Sink>,
) -> ! {
    daplink_embassy_usb::msc::msc_task_with_reconnect(&mut msc, vfs, StmUsbReconnect).await
}

#[cfg(feature = "flash")]
daplink_vfs::retained_stamp!();

/// DAP loop: reader and writer run on [`DAP_EXECUTOR`]; the executor half runs here.
async fn dap_loop_pipelined(
    class: DapV2Class<UsbDrv>,
    dap: &mut Engine,
    pipeline: &'static mut Pipeline,
    spawner: embassy_executor::SendSpawner,
    #[cfg(not(feature = "accel"))] block: &mut H5Block<BlockWork>,
    #[cfg(not(any(feature = "accel", feature = "plain-bitbang")))] jblock: &mut jtag::JtagBlock,
    #[cfg(feature = "spi-block")] sblock: &mut daplink_stm32_embassy::SpiBlock<spi_swd::Board, 3>,
    #[cfg(all(feature = "flash", not(feature = "accel")))] mux: &'static Mux,
) -> ! {
    let (usb_reader, usb_writer) = class.split();

    let mut session = {
        let (read_half, exec_half, write_half) = pipeline.split();
        spawner.spawn(dap_read_task(read_half, usb_reader).expect("dap_read_task spawns once"));
        spawner.spawn(dap_write_task(write_half, usb_writer).expect("dap_write_task spawns once"));
        exec_half
    };
    // Shared with the suspend hook, which must disarm them too.
    #[cfg(not(feature = "accel"))]
    let block = core::cell::RefCell::new(block);
    #[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
    let jblock = core::cell::RefCell::new(jblock);
    #[cfg(feature = "spi-block")]
    let sblock = core::cell::RefCell::new(sblock);

    #[cfg(all(feature = "flash", not(feature = "accel")))]
    let mut sessions_seen = 0;

    dap_sessions::<{ cfg!(feature = "defmt") }, _, _, _, _, _>(
        &mut session,
        dap,
        |dap, request, response| {
            #[cfg(all(not(feature = "accel"), feature = "flash"))]
            let flashing = {
                let (flashing, returned) = mux.flash_state(&mut sessions_seen);
                // The session moved the pads, so dap-rs restores them before a block runs again.
                if returned {
                    block.borrow_mut().suspend();
                    #[cfg(not(feature = "plain-bitbang"))]
                    jblock.borrow_mut().suspend();
                }
                flashing
            };
            // The block owns JTAG DAP_TransferBlock unless a flash session holds the wire.
            #[cfg(all(not(feature = "accel"), not(feature = "flash")))]
            let handled = block.borrow_mut().process(request, response);
            #[cfg(all(not(feature = "accel"), feature = "flash"))]
            let handled = if flashing {
                None
            } else {
                block.borrow_mut().process(request, response)
            };
            #[cfg(all(feature = "accel", not(feature = "spi-block")))]
            let handled: Option<usize> = None;
            #[cfg(feature = "spi-block")]
            let handled = sblock.borrow_mut().process(request, response);
            #[cfg(all(
                not(any(feature = "accel", feature = "plain-bitbang")),
                not(feature = "flash")
            ))]
            let handled = handled.or_else(|| jblock.borrow_mut().process(request, response));
            #[cfg(all(
                not(any(feature = "accel", feature = "plain-bitbang")),
                feature = "flash"
            ))]
            let handled = handled.or_else(|| {
                if flashing {
                    None
                } else {
                    jblock.borrow_mut().process(request, response)
                }
            });
            match handled {
                Some(len) => len,
                None => {
                    let len = dap.process_command(request, response);
                    let _answered = &response[..len.min(response.len())];
                    #[cfg(not(feature = "accel"))]
                    block.borrow_mut().observe(request, _answered);
                    #[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
                    jblock.borrow_mut().observe(request, _answered);
                    #[cfg(feature = "spi-block")]
                    sblock.borrow_mut().observe(request, _answered);
                    len
                }
            }
        },
        |dap| {
            dap.suspend();
            #[cfg(not(feature = "accel"))]
            block.borrow_mut().suspend();
            #[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
            jblock.borrow_mut().suspend();
            #[cfg(feature = "spi-block")]
            sblock.borrow_mut().suspend();
        },
        |_dap, _processed| {
            #[cfg(feature = "defmt")]
            defmt::info!("dap session: {=u32} commands processed", _processed);
        },
    )
    .await
}

/// What the DAP loop keeps from the boot-time composition.
struct Composed {
    engine: &'static mut Engine,
    class: DapV2Class<UsbDrv>,
    #[cfg(all(feature = "flash", not(feature = "accel")))]
    mux: &'static Mux,
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
    defmt::info!("stm32h523-embassy: boot");

    let p = embassy_stm32::init(CLOCKS.config());

    // After the clock tree settles, so every stamp counts at 250 MHz.
    #[cfg(feature = "dap-trace")]
    daplink_embassy_usb::trace::enable_cyccnt();

    // `init` NVIC-enables every GPDMA line, bound or not, and an unbound one vectors into a loop.
    // SAFETY: nothing here services an interrupt other than through its vector-table entry.
    let _unhandled = unsafe { mask_unhandled_interrupts() };
    #[cfg(feature = "defmt")]
    defmt::info!("masked {=u32} unhandled NVIC lines", _unhandled);

    // MPU region first, then cache: both before anything reads the factory read-only window.
    configure_factory_ro_region();
    let status = enable_instruction_cache();
    #[cfg(feature = "dap-trace")]
    trace::enable_miss_monitor();
    #[cfg(feature = "defmt")]
    defmt::info!(
        "icache: {=str} (status {=u32:#010x})",
        status.describe(),
        status.detail()
    );
    let _ = status;

    // PB12-PB14 share the SWCLK and SWDIO nets: unpulled inputs before anything drives PB10/PB15.
    daplink_stm32_embassy::park_inputs(pac::GPIOB, 1 << 12 | 1 << 13 | 1 << 14);

    // Diagnostic pad timing sequence, before anything else claims PB10/PB15.
    #[cfg(feature = "wire-probe")]
    run_wire_probe::<pads::H5Pads>(clocks::CPU_HZ);

    // Pin map: nRESET=PB0, TDI=PA7, SWDIO=PB15, SWCLK=PB10, TDO=PA6, clear of USB and debug pads.
    #[cfg(not(feature = "accel"))]
    let delay = DynCycleDelay::with_fixed_work(
        clocks::CPU_HZ,
        fits::RESIDENT.fixed_cycles,
        MIN_HALF_PERIOD_CYCLES,
    );
    #[cfg(not(feature = "accel"))]
    let line = swd_jtag_deps_at(
        p.PB0,
        p.PA7,
        p.PB15,
        p.PB10,
        p.PA6,
        delay,
        SWD_PAD_SPEED,
        &LINE_SCAN,
    );
    #[cfg(feature = "plain-bitbang")]
    let deps = line;
    #[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
    let deps = jtag::fast_jtag_deps(line, delay);

    // Accel: SPI2 sessions replace bit-bang SWD on the same pins; JTAG stays bit-bang.
    #[cfg(feature = "accel")]
    let delay = DynCycleDelay::new(clocks::CPU_HZ);
    #[cfg(feature = "accel")]
    let deps = spi_swd::accel_deps(
        p.SPI2,
        p.PB10,
        p.PB15,
        p.PB14,
        p.PB0,
        p.PA7,
        p.PA6,
        delay,
        clocks::SPI2,
    );

    let serial = serial();
    let cfg = DapConfig::daplink(serial, DAP_ENDPOINT).with_product(product_string!("stm32h523"));
    let identity = daplink_rs::daplink_core::Identity {
        vendor: cfg.manufacturer,
        product: cfg.product,
        serial: cfg.serial,
        board_vendor: "daplink-rs",
        board_name: "stm32h523",
    };
    #[cfg(feature = "defmt")]
    defmt::info!(
        "usb identity: {=str} serial {=str}",
        cfg.product,
        cfg.serial
    );

    // SWO on PA10 into a GPDMA1 ring; no embassy driver, so the clocks are enabled here.
    pac::RCC.apb2enr().modify(|w| w.set_usart1en(true));
    pac::RCC.ahb1enr().modify(|w| w.set_gpdma1en(true));
    let _ = pac::RCC.apb2enr().read();
    let swo = uart_swo(
        pac::USART1,
        embassy_stm32::rcc::frequency::<peripherals::USART1>().0,
        p.PA10,
        SWO_RX_AF,
        pac::GPDMA1,
        SWO_DMA_CHANNEL,
        SWO_DMA_REQUEST,
        p.GPDMA1_CH0,
    );

    // Onboard LED on PC13, active-low.
    let led = StatusLed::new_active_low(Output::new(p.PC13, Level::High, Speed::Low));

    #[cfg(not(feature = "flash"))]
    let engine: Engine = Engine::with_vendor(
        deps,
        led,
        delay,
        swo,
        DAP_FW_VERSION,
        DAP_PACKET,
        (
            RebootVendor::new(StmSystemDfu),
            VcomBaudVendor::new(vcom_baud_state),
            VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
        ),
    )
    .with_packet_count(DAP_PACKET_COUNT)
    .with_identity(identity);
    #[cfg(feature = "flash")]
    let (engine, vfs, mux) = {
        static MUX: static_cell::StaticCell<Mux> = static_cell::StaticCell::new();
        static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> = ConstStaticCell::new([0; SCRATCH]);
        static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

        let mux: &'static Mux = MUX.init(SwdMux::new(deps));
        let table = flash_table();
        let vendor_backend: Backend =
            mux_flash_backend(mux, delay, PRIMARY, table, VENDOR_SCRATCH.take());
        let engine = mux_dap_engine(
            mux,
            delay,
            swo,
            DAP_PACKET,
            &MUX_SCAN,
            (
                RebootVendor::with_busy_check(StmSystemDfu, mux),
                VendorFlash::new(FlashSink::new(vendor_backend)),
                VcomBaudVendor::new(vcom_baud_state),
                VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
            ),
            led,
        )
        .with_packet_count(DAP_PACKET_COUNT)
        .with_identity(identity);
        let vfs_backend: Backend = mux_flash_backend(mux, delay, PRIMARY, table, flash_scratch());
        let vfs = daplink_vfs::assemble_info(
            &VOLUME,
            FlashSink::new(vfs_backend),
            daplink_vfs::DeviceInfo {
                version: env!("CARGO_PKG_VERSION"),
                serial,
                reset_count: stamp.snapshot.reset_count(),
            },
            Some((stamp.snapshot, stamp.status, stamp.steady)),
            "DAPLINK",
        );
        (engine, vfs, mux)
    };
    // The SPI engine owns the pads, so the block hook is absent and the mux stays engine-side.
    #[cfg(all(feature = "flash", feature = "accel"))]
    let _ = mux;

    // Full-speed device controller: D+=PA12, D-=PA11, with its own pull-up.
    #[cfg(not(feature = "flash"))]
    let (usb, class, cdc_tx, cdc_rx, control) = daplink_embassy_usb::usb_device(
        H5UsbDriver::new(Driver::new(p.USB, Irqs, p.PA12, p.PA11)),
        &cfg,
    );
    #[cfg(feature = "flash")]
    let (usb, class, cdc_tx, cdc_rx, control, msc) = daplink_embassy_usb::usb_device_msc(
        H5UsbDriver::new(Driver::new(p.USB, Irqs, p.PA12, p.PA11)),
        &cfg,
    );
    spawner.spawn(usb_task(usb).expect("usb_task spawns once"));
    spawner.spawn(reboot_task().expect("reboot_task spawns once"));
    #[cfg(feature = "flash")]
    spawner.spawn(msc_task(msc, vfs).expect("msc_task spawns once"));

    // VCOM UART: USART2, TX=PA2, RX=PA3, ring-buffered on the seam's own interrupt.
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = DEFAULT_VCOM_BAUD;
    // Configuration only: the seam takes over the USART2 line and both rings from here.
    let uart = Uart::new_blocking(p.USART2, p.PA3, p.PA2, uart_cfg)
        .expect("the VCOM USART configuration is valid");
    spawner.spawn(
        vcom_task(
            cdc_tx,
            cdc_rx,
            control,
            StmVcom::new(
                uart,
                pac::USART2,
                embassy_stm32::interrupt::Interrupt::USART2,
                embassy_stm32::rcc::frequency::<peripherals::USART2>().0,
                pac::GPDMA1,
                VCOM_DMA_CHANNEL,
                VCOM_DMA_REQUEST,
                p.GPDMA1_CH1,
                pac::GPDMA1,
                VCOM_TX_DMA_CHANNEL,
                VCOM_TX_DMA_REQUEST,
                p.GPDMA1_CH2,
            ),
            DEFAULT_VCOM_BAUD,
        )
        .expect("vcom_task spawns once"),
    );

    static ENGINE: static_cell::StaticCell<Engine> = static_cell::StaticCell::new();
    Composed {
        engine: ENGINE.init(engine),
        class,
        #[cfg(all(feature = "flash", not(feature = "accel")))]
        mux,
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let Composed {
        engine,
        class,
        #[cfg(all(feature = "flash", not(feature = "accel")))]
        mux,
    } = setup(spawner);
    static DAP_PIPELINE: ConstStaticCell<Pipeline> = ConstStaticCell::new(Pipeline::new());
    let pipeline = DAP_PIPELINE.take();

    // The priority must be set before the executor enables the line.
    let dap_spawner = {
        DAP_IRQ.set_priority(DAP_IRQ_PRIORITY);
        DAP_EXECUTOR.start(DAP_IRQ)
    };

    #[cfg(not(feature = "accel"))]
    let mut block: H5Block<BlockWork> = H5Block::new(clocks::CPU_HZ);
    #[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
    let mut jblock = jtag::JtagBlock::new(clocks::CPU_HZ);
    #[cfg(feature = "spi-block")]
    let mut sblock =
        daplink_stm32_embassy::SpiBlock::<spi_swd::Board, 3>::new(clocks::CPU_HZ, clocks::SPI2);

    dap_loop_pipelined(
        class,
        engine,
        pipeline,
        dap_spawner,
        #[cfg(not(feature = "accel"))]
        &mut block,
        #[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
        &mut jblock,
        #[cfg(feature = "spi-block")]
        &mut sblock,
        #[cfg(all(feature = "flash", not(feature = "accel")))]
        mux,
    )
    .await
}
