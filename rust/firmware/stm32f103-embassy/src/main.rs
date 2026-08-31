//! CMSIS-DAP v2 probe for STM32F103CB boards on embassy; `nanodap` selects that board's wiring.

#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_stm32::bind_interrupts;
#[cfg(feature = "nanodap")]
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
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

use daplink_embassy_usb::{session_ended, session_epoch, vcom_bridge, DapV2Class};
#[cfg(not(feature = "flash"))]
use daplink_rs::daplink_core::DAP_FW_VERSION;
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, QueueError, Submit, DAP_PACKET_SIZE, DEFAULT_VCOM_BAUD,
};
#[cfg(all(feature = "nanodap", not(feature = "flash"), not(feature = "accel")))]
use daplink_rs::BitbangDap;
#[cfg(feature = "nanodap")]
use daplink_rs::NoSwo;
#[cfg(feature = "nanodap")]
use daplink_rs::StatusLed;
#[cfg(not(feature = "nanodap"))]
use daplink_rs::SwdOnlyDap;
use daplink_rs::{NoBootloader, RebootVendor, VcomBaudVendor, VcomStatsVendor, VendorChain};
#[cfg(feature = "accel")]
use daplink_spi_swd::NoFastJtag;
#[cfg(all(feature = "nanodap", not(feature = "accel")))]
use daplink_stm32f1_embassy::swd_jtag_deps;
#[cfg(feature = "accel")]
use daplink_stm32f1_embassy::SpiSwdJtagDap;
use daplink_stm32f1_embassy::{
    clocks, mask_unhandled_interrupts, serial, vcom_baud_state, DmaVcom, DynCycleDelay, IoPin,
};
#[cfg(not(feature = "nanodap"))]
use daplink_stm32f1_embassy::{swd_only_deps, usart1_swo, BicolorLed, UartSwo};

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
use daplink_stm32f1_embassy::clocks::UsbConnect;
#[cfg(feature = "flash-stm32f401")]
use daplink_targets::targets::stm32f401ce::STM32F401CE;
#[cfg(feature = "flash")]
use daplink_vfs::{AllDecoders, FlashSink, FlashVolume, VendorFlash, VolumeStorage};

// The flash spine stays scoped to the nanoDAP board it is composed and benched on.
#[cfg(all(feature = "flash", not(feature = "nanodap")))]
compile_error!("`flash` composes only on the `nanodap` board wiring");

// The accelerated SWD backend's board seam, composed in place of bit-bang SWD under `accel`.
#[cfg(feature = "accel")]
mod spi_swd;

// The seam's pin map (TDO on PA5, the D+ switch off PB15) is the nanoDAP's.
#[cfg(all(feature = "accel", not(feature = "nanodap")))]
compile_error!("`accel` composes only on the `nanodap` board wiring");

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

// Both receive paths are polled DMA rings, so only the VCOM transmit channel is bound.
bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => InterruptHandler<peripherals::USB>;
    // VCOM transmit (USART2_TX is DMA1 channel 7, RM0008 table 78).
    DMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH7>;
});

/// USB driver type for the F1 full-speed device controller.
type UsbDrv = Driver<'static, peripherals::USB>;

/// The engine's `DapLeds` slot: the PA9 bicolor COM LED (tri-state, see below).
#[cfg(not(feature = "nanodap"))]
type StatusLedType = BicolorLed;
/// The engine's `DapLeds` slot: the nanoDAP's single status LED on PC13, active-low.
#[cfg(feature = "nanodap")]
type StatusLedType = StatusLed<Output<'static>>;

/// The reboot-to-bootloader vendor command, honestly declined: the F103's ROM bootloader is
/// USART-only, so there is no software-reachable USB update path to reboot into.
#[cfg(not(feature = "flash"))]
type Vendor = VendorChain<RebootVendor<NoBootloader>, VendorChain<VcomBaudVendor, VcomStatsVendor>>;

/// The default-map engine: bit-bang SWD only, with USART1-RX SWO capture in its SWO slot.
#[cfg(not(feature = "nanodap"))]
type Engine = SwdOnlyDap<'static, IoPin, DynCycleDelay, UartSwo, Vendor, StatusLedType>;

/// The nanoDAP engine: bit-bang SWD+JTAG (TDO on PA5), no SWO -- the board routes no trace pin.
#[cfg(all(feature = "nanodap", not(feature = "flash"), not(feature = "accel")))]
type Engine = BitbangDap<'static, IoPin, DynCycleDelay, NoSwo, Vendor, StatusLedType>;

/// The accelerated engine: SPI-session SWD with bit-bang JTAG, same LED/vendor slots, no SWO.
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
/// The declined reboot command, chained ahead of the vendor flash-stream commands.
#[cfg(feature = "flash")]
type Vendor = VendorChain<
    RebootVendor<NoBootloader, RebootBusy>,
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
    NoSwo,
    Vendor,
    StatusLedType,
>;

/// Core cycles of fixed bit-cell work subtracted from every requested half-period delay;
/// swept for this pin path at 72 MHz, so re-sweep after an `opt-level` change.
const SWD_FIXED_WORK_CYCLES: u32 = 8;

/// Floor on the residual spin, in core cycles; zero lets a saturated request run flat out.
const MIN_HALF_PERIOD_CYCLES: u32 = 0;

/// Half-period floor for the accel image's JTAG pacing, in core cycles: its leaner loop needs
/// an unsubtracted, floored delay or the TDO sample lands on the TCK falling edge.
#[cfg(feature = "accel")]
const JTAG_MIN_HALF_PERIOD_CYCLES: u32 = 8;

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

/// Control-endpoint (EP0) size: 32 bytes, freeing packet memory for the class endpoints.
const EP0_SIZE: u8 = 32;

/// Mass-storage bulk packet size: 32 bytes, all this part's 512-byte packet memory leaves once
/// EP0, DAP v2 and the CDC pair are allocated.
#[cfg(feature = "flash")]
const MSC_EP: usize = 32;

/// VCOM receive ring: must drain before the DMA cursor laps, 22 ms at 921600 baud.
const VCOM_RX_RING: usize = 2048;

/// Drive the USB device: enumeration, control transfers, and endpoint FIFOs.
#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDrv>) -> ! {
    usb.run().await
}

/// Bridge the CDC-ACM VCOM port to its USART, following the host's line-coding baud.
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

/// Mass-storage bulk transport: the CBW/CSW cycle, bouncing the board's PB15 D+ switch on each
/// post-transfer remount so the host re-reads `DETAILS.TXT`.
#[cfg(feature = "flash")]
#[embassy_executor::task]
async fn msc_task(
    mut msc: MscEndpoints<'static, UsbDrv, MSC_EP>,
    vfs: &'static mut FlashVolume<Sink>,
    connect: UsbConnect<'static>,
) -> ! {
    daplink_embassy_usb::msc::msc_task_with_reconnect(&mut msc, vfs, connect).await
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

    // Accel: the session backend replaces bit-bang SWD on the same pins, JTAG stays bit-bang.
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
    let mut cfg = DapConfig::daplink(serial).with_product(product_string!("stm32f103"));
    cfg.max_packet_size_0 = EP0_SIZE;
    #[cfg(feature = "defmt")]
    defmt::info!(
        "usb identity: {=str} serial {=str}",
        cfg.product,
        cfg.serial
    );

    // SWO capture on PA10 into a DMA1 CH5 ring; no embassy driver, so the clock is enabled here.
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

    // COM LED on PA9, a bicolor series pair: high = red (session activity), low = green (idle).
    #[cfg(not(feature = "nanodap"))]
    let led = {
        let mut led = BicolorLed::new(p.PA9);
        led.idle();
        led
    };
    // nanodap: onboard single LED on PC13, active-low (the pin sinks current).
    #[cfg(feature = "nanodap")]
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
            RebootVendor::new(NoBootloader),
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
                RebootVendor::with_busy_check(NoBootloader, mux),
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

    // A switched D+ pull-up (PA15, or PB15 on the nanoDAP), detached until the device exists.
    #[cfg(not(feature = "nanodap"))]
    let mut usb_connect = clocks::usb_connect_pa15(p.PA15);
    #[cfg(feature = "nanodap")]
    let mut usb_connect = clocks::usb_connect_pb15(p.PB15);

    // Full-speed device controller: D+=PA12, D-=PA11.
    #[cfg(not(feature = "flash"))]
    let (usb, mut class, cdc_tx, cdc_rx, control) =
        daplink_embassy_usb::usb_device(Driver::new(p.USB, Irqs, p.PA12, p.PA11), &cfg);
    #[cfg(feature = "flash")]
    let (usb, mut class, cdc_tx, cdc_rx, control, msc) =
        daplink_embassy_usb::usb_device_msc::<_, MSC_EP>(
            Driver::new(p.USB, Irqs, p.PA12, p.PA11),
            &cfg,
        );
    spawner.spawn(usb_task(usb).expect("usb_task spawns once"));

    // VCOM: USART2 (TX=PA2, RX=PA3), both legs on DMA -- the seam never reads the data register.
    static RX_RING: ConstStaticCell<[u8; VCOM_RX_RING]> = ConstStaticCell::new([0; VCOM_RX_RING]);
    let uart = DmaVcom::new_usart2(
        p.USART2,
        p.PA2,
        p.PA3,
        p.DMA1_CH7,
        p.DMA1_CH6,
        Irqs,
        RX_RING.take(),
    );
    spawner.spawn(
        vcom_task(cdc_tx, cdc_rx, control, uart, DEFAULT_VCOM_BAUD).expect("vcom_task spawns once"),
    );

    // The device exists and `usb.run()` is scheduled, so present the D+ pull-up.
    usb_connect.attach();
    // Presented, so the mass-storage task takes the switch and bounces it once per remount.
    #[cfg(feature = "flash")]
    spawner.spawn(msc_task(msc, vfs, usb_connect).expect("msc_task spawns once"));

    // Composition finished, so this boot is not a reset loop: forgive the fault streak.
    #[cfg(feature = "flash")]
    daplink_vfs::stamp::clear_fault_streak();

    // Caller-owned queue storage; `DAP_PACKET_COUNT` above names the same depth.
    static DAP_QUEUE: ConstStaticCell<RefCell<Queue>> =
        ConstStaticCell::new(RefCell::new(Queue::new()));
    let queue = DAP_QUEUE.take();

    dap_loop_pipelined(&mut class, &mut engine, queue).await
}
