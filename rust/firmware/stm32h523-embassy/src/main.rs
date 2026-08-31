//! Minimal CMSIS-DAP v2 debug probe for the WeAct STM32H523 core board on embassy.

#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::pac;
use embassy_stm32::peripherals;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
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

use daplink_embassy_usb::{session_ended, session_epoch, vcom_bridge, DapV2Class};
#[cfg(not(feature = "flash"))]
use daplink_rs::daplink_core::DAP_FW_VERSION;
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, QueueError, Submit, DAP_PACKET_SIZE, DEFAULT_VCOM_BAUD,
};
#[cfg(all(not(feature = "flash"), not(feature = "accel")))]
use daplink_rs::BitbangDap;
use daplink_rs::{
    BootloaderEntry, RebootVendor, StatusLed, VcomBaudVendor, VcomStatsVendor, VendorChain,
};
#[cfg(not(feature = "accel"))]
use daplink_stm32_embassy::swd_jtag_deps;
#[cfg(feature = "accel")]
use daplink_stm32_embassy::SpiSwdJtagDap;
use daplink_stm32_embassy::{
    check_and_enter_bootloader, configure_factory_ro_region, enable_instruction_cache,
    mask_unhandled_interrupts, reboot_armed, serial, uart_swo, vcom_baud_state, DynCycleDelay,
    IoPin, StmSystemDfu, StmVcom, UartSwo, VcomInterruptHandler,
};

// The mass-storage drive and the vendor flash commands, both over the flash-wins mux.
#[cfg(feature = "flash")]
use daplink_embassy_usb::msc::MscEndpoints;
#[cfg(feature = "flash")]
use daplink_rs::flash_mux::{
    role_build_flash_backend, role_mux_dap_engine_with_vendor, RoleMuxDapEngine,
    RoleMuxFlashBackend,
};
#[cfg(feature = "flash")]
use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv, ScanChainStorage, SwdMux};
#[cfg(feature = "flash")]
use daplink_stm32_embassy::StmUsbReconnect;
#[cfg(feature = "flash-stm32f401")]
use daplink_targets::targets::stm32f401ce::STM32F401CE;
#[cfg(feature = "flash")]
use daplink_vfs::{AllDecoders, FlashSink, FlashVolume, VendorFlash, VolumeStorage};

mod clocks;

// The accelerated SWD backend's board seam, composed in place of bit-bang SWD under `accel`.
#[cfg(feature = "accel")]
mod spi_swd;

// The accel engine and the flash-wins mux each claim the one wire adapter; they do not compose.
#[cfg(all(feature = "accel", feature = "flash"))]
compile_error!("`accel` and `flash` are mutually exclusive in this firmware");

// The registry the flash spine selects from, derived from the rows below.
#[cfg(feature = "flash")]
daplink_targets::target_table! {
    umbrella = "flash",
    count = TARGET_COUNT,
    primary = PrimaryTarget,
    selector = Selector,
    scratch = SCRATCH,
    primary_ref = PRIMARY,
    table = flash_table,
    buffer = flash_scratch,
    rows = [
        ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
    ],
}

bind_interrupts!(struct Irqs {
    USB_DRD_FS => InterruptHandler<peripherals::USB>;
    USART2 => VcomInterruptHandler;
});

/// System-memory DFU re-entry from `cortex-m-rt`'s pre-init slot, while the no-init magic word
/// is still intact; `0x0BF9_7000` is this part's bootloader vector table.
#[no_mangle]
unsafe extern "C" fn __pre_init() {
    // SAFETY: pre-init context, and the address is the system-memory bootloader's vector table.
    unsafe { check_and_enter_bootloader(0x0BF9_7000) }
}

/// USB driver type for the H5 full-speed device controller.
type UsbDrv = Driver<'static, peripherals::USB>;

/// The onboard status LED's concrete `DapLeds` type: PC13, active-low.
type StatusLedType = StatusLed<Output<'static>>;

#[cfg(not(feature = "flash"))]
type Vendor = VendorChain<RebootVendor<StmSystemDfu>, VendorChain<VcomBaudVendor, VcomStatsVendor>>;

/// The bit-bang SWD+JTAG engine: USART1-RX SWO capture, status LED, reboot in the vendor slot.
#[cfg(all(not(feature = "flash"), not(feature = "accel")))]
type Engine = BitbangDap<'static, IoPin, DynCycleDelay, UartSwo, Vendor, StatusLedType>;

/// The accelerated engine: SPI2-session SWD with GPIO bit-bang JTAG, same SWO/LED/vendor slots.
#[cfg(all(not(feature = "flash"), feature = "accel"))]
type Engine = SpiSwdJtagDap<
    'static,
    spi_swd::Board,
    IoPin,
    DynCycleDelay,
    spi_swd::H5FastJtag,
    UartSwo,
    Vendor,
    StatusLedType,
>;

/// The dap-rs dependency role of the one adapter the flash build arbitrates.
#[cfg(feature = "flash")]
type MuxDeps = LateJtagDeps<IoPin, DynCycleDelay>;
/// The SWD driver role of [`MuxDeps`].
#[cfg(feature = "flash")]
type MuxSwd = LateSwdDrv<IoPin, DynCycleDelay>;
/// The JTAG driver role of [`MuxDeps`].
#[cfg(feature = "flash")]
type MuxJtag = LateJtag<IoPin, DynCycleDelay>;

/// The flash-wins arbiter owning the one physical adapter.
#[cfg(feature = "flash")]
type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
/// The flash-algorithm runtime over an ungated flash-side handle on the mux.
#[cfg(feature = "flash")]
type Backend =
    RoleMuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, PrimaryTarget, Selector>;
/// The image-fragment sink one flash front-end owns.
#[cfg(feature = "flash")]
type Sink = FlashSink<Backend>;
/// The vendor slot's flash-busy gate: the mux's own live-session flag.
#[cfg(feature = "flash")]
type RebootBusy = &'static Mux;
/// The reboot command, declined with `STATUS_BUSY` while [`RebootBusy`] reports a live flash
/// session, chained ahead of the vendor flash-stream commands.
#[cfg(feature = "flash")]
type Vendor = VendorChain<
    RebootVendor<StmSystemDfu, RebootBusy>,
    VendorChain<VendorFlash<AllDecoders, Sink>, VendorChain<VcomBaudVendor, VcomStatsVendor>>,
>;
/// The flash build's engine: gated by the mux, with the vendor flash commands composed in.
#[cfg(feature = "flash")]
type Engine = RoleMuxDapEngine<
    'static,
    MuxDeps,
    MuxSwd,
    MuxJtag,
    DynCycleDelay,
    UartSwo,
    Vendor,
    StatusLedType,
>;

/// USART1_RX alternate function on PA10 (RM0481 AF tables).
const SWO_RX_AF: u8 = 7;

/// GPDMA hardware request line for USART1_RX (RM0481 request mapping).
const SWO_DMA_REQUEST: u8 = 21;

/// GPDMA1 channel index driving the SWO capture ring; `main`'s `p.GPDMA1_CH0` names the same one.
const SWO_DMA_CHANNEL: usize = 0;

/// GPDMA request line for USART2 RX, feeding the VCOM receive ring.
const VCOM_DMA_REQUEST: u8 = 23;

/// GPDMA1 channel index driving the VCOM receive ring; SWO holds channel 0.
const VCOM_DMA_CHANNEL: usize = 1;

/// GPDMA request line for USART2 TX, draining the VCOM transmit path.
const VCOM_TX_DMA_REQUEST: u8 = 24;

/// GPDMA1 channel index draining VCOM transmit bursts; SWO holds 0, VCOM receive 1.
const VCOM_TX_DMA_CHANNEL: usize = 2;

/// Core cycles of fixed bit-cell work subtracted from every requested half-period delay; swept,
/// not calculated, and bit-bang images only -- the `accel` image's SWD wire is SPI-paced.
#[cfg(not(feature = "accel"))]
const SWD_FIXED_WORK_CYCLES: u32 = 16;

/// Floor on the residual spin, in core cycles: below this the target stops acking at 250 MHz.
#[cfg(not(feature = "accel"))]
const MIN_HALF_PERIOD_CYCLES: u32 = 6;

/// Pipelined CMSIS-DAP command-queue depth, also the advertised `DAP_PACKET_COUNT`.
const DAP_QUEUE_DEPTH: usize = 4;

/// Per-slot byte capacity of the command queue: one full-speed DAP v2 bulk packet.
const DAP_QUEUE_SLOT: usize = DAP_PACKET_SIZE as usize;

/// This image's command queue: the depth the host is told about and the storage behind it.
type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

/// The `DAP_Info(MaxPacketCount)` this image advertises; fed to the engine below.
const DAP_PACKET_COUNT: u8 = 4;

// Advertised count, ring depth and slot size are one fact; widening one alone fails the build.
const _: () = {
    assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
    assert!(Queue::new().packet_capacity() == DAP_PACKET_SIZE as usize);
};

/// Drive the USB device: enumeration, control transfers, and endpoint FIFOs.
#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDrv>) -> ! {
    usb.run().await
}

/// Bridge the CDC-ACM VCOM port to USART2, following the host's line-coding baud.
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

/// Thread-mode reboot trigger: waits out a grace so the DAP acknowledgement drains, then enters
/// the DFU the vendor handler armed.
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

/// Mass-storage bulk transport: the CBW/CSW cycle, bouncing D+ on each post-transfer remount.
#[cfg(feature = "flash")]
#[embassy_executor::task]
async fn msc_task(
    mut msc: MscEndpoints<'static, UsbDrv>,
    vfs: &'static mut FlashVolume<Sink>,
) -> ! {
    daplink_embassy_usb::msc::msc_task_with_reconnect(&mut msc, vfs, StmUsbReconnect).await
}

/// Reset-surviving probe state (boot counter, last transfer status) in the `.uninit` section
/// `link.x` already emits; on this part only a power cycle clears it.
#[cfg(feature = "flash")]
#[link_section = ".uninit.RESET_STAMP"]
static mut RESET_STAMP: core::mem::MaybeUninit<daplink_vfs::ResetStamp> =
    core::mem::MaybeUninit::uninit();

/// Pipelined CMSIS-DAP loop: a reader fills the queue, an executor runs and answers the oldest.
async fn dap_loop_pipelined(
    class: &mut DapV2Class<UsbDrv>,
    dap: &mut Engine,
    queue: &RefCell<Queue>,
) -> ! {
    loop {
        class.wait_connection().await;
        // Snapshot after the connection is up, so this session's own boundary cannot end it.
        let session = session_epoch();
        // Start clean: drop anything a prior session left queued, and clear a stale abort.
        queue.borrow_mut().reset();

        // Edge-triggered coordination, fresh per connection; one task, so `NoopRawMutex`.
        let request_available: Signal<NoopRawMutex, ()> = Signal::new();
        let slot_freed: Signal<NoopRawMutex, ()> = Signal::new();
        // Set by whichever future first sees the host leave, so the other stops and the join ends.
        let disconnected: Cell<bool> = Cell::new(false);
        // Commands run this connection; a one-line summary at disconnect, not a line per packet.
        #[cfg(feature = "defmt")]
        let processed: Cell<u32> = Cell::new(0);

        let (mut reader, mut writer) = class.split();
        let dap = &mut *dap;

        // Reader: pull OUT packets into the request ring; no borrow is held across an await.
        let reader_fut = async {
            let mut req = [0u8; DAP_QUEUE_SLOT];
            loop {
                let Ok(n) = reader.read_command(&mut req).await else {
                    disconnected.set(true);
                    request_available.signal(());
                    return;
                };
                loop {
                    let outcome = queue.borrow_mut().submit(&req[..n]);
                    match outcome {
                        Ok(Submit::Enqueued) => {
                            request_available.signal(());
                            break;
                        }
                        // Neither enqueues, so neither wakes the executor.
                        Ok(Submit::AbortIntercepted | Submit::Dropped) => break,
                        // Ring full: wait for a freed slot, unless the executor has stopped.
                        Err(QueueError::Full) => {
                            if disconnected.get() {
                                return;
                            }
                            slot_freed.wait().await;
                        }
                        // Unreachable: the slot capacity is the endpoint packet size.
                        Err(QueueError::Oversized) => break,
                    }
                }
            }
        };

        // Executor: run the oldest queued command, then drain every ready response to the host.
        let executor_fut = async {
            let mut cmd = [0u8; DAP_QUEUE_SLOT];
            let mut resp = [0u8; DAP_QUEUE_SLOT];
            loop {
                let taken = queue.borrow_mut().take_request(&mut cmd);
                let Some(n) = taken else {
                    // Nothing to run: stop if the host is gone, else park until a request lands.
                    if disconnected.get() {
                        return;
                    }
                    request_available.wait().await;
                    continue;
                };
                // A request slot just opened; let a backpressured reader proceed.
                slot_freed.signal(());

                let len = dap.process_command(&cmd[..n], &mut resp);
                #[cfg(feature = "defmt")]
                processed.set(processed.get() + 1);

                if queue.borrow_mut().post_response(&resp[..len]).is_err() {
                    // The drain below empties the ring before the next command is taken.
                    debug_assert!(false, "response ring full after a single command");
                }
                // Retire under the peek's generation, never the next session's packet.
                loop {
                    let peeked = {
                        let q = queue.borrow();
                        (q.generation(), q.peek_response(&mut resp))
                    };
                    let (generation, Some(l)) = peeked else { break };
                    if writer.write_response(&resp[..l]).await.is_err() {
                        disconnected.set(true);
                        // Release a reader parked on a full ring, so the join can complete.
                        slot_freed.signal(());
                        return;
                    }
                    if !queue.borrow_mut().discard_response_for(generation) {
                        break;
                    }
                }
            }
        };

        // The boundary must be the first arm, so a write parked on a dead session is dropped.
        select(session_ended(session), join(reader_fut, executor_fut)).await;
        #[cfg(feature = "defmt")]
        defmt::info!("dap session: {=u32} commands processed", processed.get());

        // Host gone: release the wire so protocol state cannot leak into the next session.
        dap.suspend();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Boot the stamp first: it arms the fault latch, and everything below can panic.
    // SAFETY: `RESET_STAMP` is only touched here, once, before anything concurrent runs.
    #[cfg(feature = "flash")]
    let stamp = daplink_vfs::ResetStamp::boot_in_place(unsafe {
        &mut *core::ptr::addr_of_mut!(RESET_STAMP)
    });

    #[cfg(feature = "defmt")]
    defmt::info!("stm32h523-embassy: boot");

    let p = embassy_stm32::init(clocks::clocks_250m_hsi());

    // `init` NVIC-enables every GPDMA line, bound or not, and an unbound one vectors into a loop.
    // SAFETY: nothing here services an interrupt other than through its vector-table entry.
    let _unhandled = unsafe { mask_unhandled_interrupts() };
    #[cfg(feature = "defmt")]
    defmt::info!("masked {=u32} unhandled NVIC lines", _unhandled);

    // MPU region first, then cache: both before anything reads the factory read-only window.
    configure_factory_ro_region();
    let status = enable_instruction_cache();
    #[cfg(feature = "defmt")]
    defmt::info!(
        "icache: {=str} (status {=u32:#010x})",
        status.describe(),
        status.detail()
    );
    let _ = status;

    // Pin map: nRESET=PB0, TDI=PA7, SWDIO=PB14, SWCLK=PB13, TDO=PA6, clear of USB and debug pads.
    #[cfg(not(feature = "accel"))]
    let delay = DynCycleDelay::with_fixed_work(
        clocks::CPU_HZ,
        SWD_FIXED_WORK_CYCLES,
        MIN_HALF_PERIOD_CYCLES,
    );
    #[cfg(not(feature = "accel"))]
    let deps = swd_jtag_deps(p.PB0, p.PA7, p.PB14, p.PB13, p.PA6, delay);

    // Accel: the SPI2 session backend replaces bit-bang SWD on the same pins, JTAG stays bitbang.
    #[cfg(feature = "accel")]
    let delay = DynCycleDelay::new(clocks::CPU_HZ);
    #[cfg(feature = "accel")]
    let deps = spi_swd::accel_deps(
        p.SPI2,
        p.PB13,
        p.PB14,
        p.PB0,
        p.PA7,
        p.PA6,
        delay,
        clocks::SPI2_KERNEL_HZ,
    );

    let serial = serial();
    let cfg = DapConfig::daplink(serial).with_product(product_string!("stm32h523"));
    #[cfg(feature = "defmt")]
    defmt::info!(
        "usb identity: {=str} serial {=str}",
        cfg.product,
        cfg.serial
    );

    // SWO capture on PA10 into a GPDMA1 ring; no embassy driver, so the clocks are enabled here.
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

    // Onboard LED on PC13, active-low (the pin sinks current).
    let led = StatusLed::new_active_low(Output::new(p.PC13, Level::High, Speed::Low));

    #[cfg(not(feature = "flash"))]
    let mut engine: Engine = Engine::with_vendor(
        deps,
        led,
        delay,
        swo,
        DAP_FW_VERSION,
        cfg.max_packet_size,
        VendorChain::new(
            RebootVendor::new(StmSystemDfu),
            VendorChain::new(
                VcomBaudVendor::new(vcom_baud_state),
                VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
            ),
        ),
    )
    .with_packet_count(DAP_PACKET_COUNT);
    #[cfg(feature = "flash")]
    let (mut engine, vfs) = {
        static MUX: static_cell::StaticCell<Mux> = static_cell::StaticCell::new();
        static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        // Two front-ends, two backends; the mux grants one session and refuses the other.
        static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> = ConstStaticCell::new([0; SCRATCH]);
        static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

        let mux: &'static Mux = MUX.init(SwdMux::new(deps));
        let table = flash_table();
        let vendor_backend: Backend =
            role_build_flash_backend(mux, delay, PRIMARY, table, VENDOR_SCRATCH.take());
        let engine = role_mux_dap_engine_with_vendor(
            mux,
            delay,
            swo,
            cfg.max_packet_size,
            &MUX_SCAN,
            VendorChain::new(
                RebootVendor::with_busy_check(StmSystemDfu, mux),
                VendorChain::new(
                    VendorFlash::new(FlashSink::new(vendor_backend)),
                    VendorChain::new(
                        VcomBaudVendor::new(vcom_baud_state),
                        VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
                    ),
                ),
            ),
            led,
        )
        .with_packet_count(DAP_PACKET_COUNT);
        let vfs_backend: Backend =
            role_build_flash_backend(mux, delay, PRIMARY, table, flash_scratch());
        let vfs = daplink_vfs::assemble_info(
            &VOLUME,
            FlashSink::new(vfs_backend),
            daplink_vfs::DeviceInfo {
                version: env!("CARGO_PKG_VERSION"),
                serial,
                reset_count: stamp.reset_count(),
            },
            Some(stamp),
            "DAPLINK",
        );
        (engine, vfs)
    };

    // Full-speed device controller: D+=PA12, D-=PA11, with its own pull-up.
    #[cfg(not(feature = "flash"))]
    let (usb, mut class, cdc_tx, cdc_rx, control) =
        daplink_embassy_usb::usb_device(Driver::new(p.USB, Irqs, p.PA12, p.PA11), &cfg);
    #[cfg(feature = "flash")]
    let (usb, mut class, cdc_tx, cdc_rx, control, msc) =
        daplink_embassy_usb::usb_device_msc(Driver::new(p.USB, Irqs, p.PA12, p.PA11), &cfg);
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

    // Composition finished, so this boot is not a reset loop: forgive the fault streak.
    #[cfg(feature = "flash")]
    daplink_vfs::stamp::clear_fault_streak();

    // Caller-owned queue storage; `DAP_PACKET_COUNT` above names the same depth.
    static DAP_QUEUE: ConstStaticCell<RefCell<Queue>> =
        ConstStaticCell::new(RefCell::new(Queue::new()));
    let queue = DAP_QUEUE.take();

    dap_loop_pipelined(&mut class, &mut engine, queue).await
}
