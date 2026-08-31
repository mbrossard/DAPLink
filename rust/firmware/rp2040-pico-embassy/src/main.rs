//! Minimal CMSIS-DAP v2 debug probe for the Raspberry Pi Pico (RP2040) on embassy.

#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{UART1, USB};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use static_cell::ConstStaticCell;

// A flash build latches the fault into `RESET_STAMP` and resets; every other build halts.
#[cfg(all(feature = "flash", not(feature = "defmt")))]
daplink_vfs::fault_reset_handlers!();
#[cfg(all(not(feature = "flash"), not(feature = "defmt")))]
use panic_halt as _;
// `defmt` halts in every build, so the RTT-attached probe sees the panic message.
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use daplink_embassy_usb::{session_ended, session_epoch, DapV2Class};
use daplink_rp_embassy::{
    init as init_rp, reboot_watch, status_led, usb_task, vcom_baud_state, vcom_task, CycleDelay,
    IoPin, RpUsbBoot, RpVcom, Uart0Swo, UsbDrv, VcomDmaInterruptHandler,
};
#[cfg(not(feature = "flash"))]
use daplink_rs::daplink_core::DAP_FW_VERSION;
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, QueueError, Submit, DAP_PACKET_SIZE, DEFAULT_VCOM_BAUD,
};
use daplink_rs::{Activity, RebootVendor, StatusLed, VcomBaudVendor, VcomStatsVendor, VendorChain};

// One wire engine per image: `bitbang` wins where both are on.
#[cfg(not(any(feature = "accel", feature = "bitbang")))]
compile_error!("enable `accel` (the default wire engine) or `bitbang`");

#[cfg(feature = "carrier-leds")]
use daplink_rp_embassy::{carrier_leds, CarrierBankLed};

#[cfg(not(rp_accel))]
use daplink_rp_embassy::probe_parts_pico;
#[cfg(all(not(rp_accel), not(feature = "flash")))]
use daplink_rs::BitbangDap;

#[cfg(rp_accel)]
use daplink_rp_embassy::probe_parts_pico_accel;
#[cfg(all(rp_accel, not(feature = "flash")))]
use daplink_rp_pio::PioSwdJtagDap;
#[cfg(rp_accel)]
use daplink_rp_pio::{pio_swd_deps, PioBlock, PioSm, PioSwdPins};
#[cfg(rp_accel)]
use daplink_rs::ComposedJtagDeps;
#[cfg(any(rp_accel, feature = "flash"))]
use daplink_rs::ScanChainStorage;

// The mass-storage drive and the vendor flash commands, both over the flash-wins mux.
#[cfg(feature = "flash")]
use daplink_embassy_usb::msc::MscEndpoints;
#[cfg(feature = "flash")]
use daplink_rp_embassy::RpUsbReconnect;
#[cfg(all(feature = "flash", rp_accel))]
use daplink_rp_pio::{PioSwdDeps, PioSwdDrv};
#[cfg(feature = "flash")]
use daplink_rs::flash_mux::{
    role_build_flash_backend, role_mux_dap_engine_with_vendor, RoleMuxDapEngine,
    RoleMuxFlashBackend,
};
#[cfg(feature = "flash")]
use daplink_rs::SwdMux;
#[cfg(all(feature = "flash", rp_accel))]
use daplink_rs::{ComposedJtagDrv, ComposedJtagSwd};
#[cfg(all(feature = "flash", not(rp_accel)))]
use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv};
#[cfg(feature = "flash-rp2040")]
use daplink_targets::targets::rp2040::RP2040;
#[cfg(feature = "flash-stm32f401")]
use daplink_targets::targets::stm32f401ce::STM32F401CE;
#[cfg(feature = "flash")]
use daplink_vfs::{AllDecoders, FlashSink, FlashVolume, VendorFlash, VolumeStorage};

// embassy-rp owns __pre_init (SIO reset); interpose and tail-call the real one.
daplink_rs::ram_text_pre_init!(wrapping);

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
        ("flash-rp2040", RP2040, daplink_targets::targets::rp2040::PROGRAM_BUFFER_SIZE),
        ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
    ],
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
    DMA_IRQ_1 => VcomDmaInterruptHandler;
});

#[cfg(not(feature = "overclock-200mhz"))]
const _: () = assert!(daplink_rp_embassy::CPU_HZ == 125_000_000);
#[cfg(feature = "overclock-200mhz")]
const _: () = assert!(daplink_rp_embassy::CPU_HZ == 200_000_000);

#[cfg(all(rp_accel, not(feature = "overclock-200mhz")))]
const _: () = assert!(daplink_rp_pio::CYCLES_PER_BIT == 4);
#[cfg(all(rp_accel, feature = "overclock-200mhz"))]
const _: () = assert!(daplink_rp_pio::CYCLES_PER_BIT == 5);

/// The onboard status LED's concrete `DapLeds` type: GP25, active-high.
type StatusLedType = StatusLed<Output<'static>>;

/// The reboot-to-bootloader vendor handler; a non-flash build has no session to gate it behind.
#[cfg(not(feature = "flash"))]
type RebootOnly =
    VendorChain<RebootVendor<RpUsbBoot>, VendorChain<VcomBaudVendor, VcomStatsVendor>>;

/// The bit-bang SWD+JTAG engine: UART0-RX SWO capture, status LED, reboot in the vendor slot.
#[cfg(all(not(rp_accel), not(feature = "flash")))]
type Engine = BitbangDap<'static, IoPin, CycleDelay, Uart0Swo, RebootOnly, StatusLedType>;

/// The accelerated engine: PIO SWD with bit-bang JTAG over the pad loan, same slots as above.
#[cfg(all(rp_accel, not(feature = "flash")))]
type Engine = PioSwdJtagDap<'static, PioSm, IoPin, CycleDelay, Uart0Swo, RebootOnly, StatusLedType>;

/// The dap-rs dependency role of the one adapter the flash build arbitrates.
#[cfg(all(feature = "flash", not(rp_accel)))]
type MuxDeps = LateJtagDeps<IoPin, CycleDelay>;
/// The SWD driver role of [`MuxDeps`].
#[cfg(all(feature = "flash", not(rp_accel)))]
type MuxSwd = LateSwdDrv<IoPin, CycleDelay>;
/// The JTAG driver role of [`MuxDeps`].
#[cfg(all(feature = "flash", not(rp_accel)))]
type MuxJtag = LateJtag<IoPin, CycleDelay>;

/// The accelerated spelling of [`MuxDeps`]: PIO SWD with bit-bang JTAG composed on.
#[cfg(all(feature = "flash", rp_accel))]
type MuxDeps = ComposedJtagDeps<PioSwdDeps<PioSm>, IoPin, CycleDelay>;
/// The SWD driver role of [`MuxDeps`].
#[cfg(all(feature = "flash", rp_accel))]
type MuxSwd = ComposedJtagSwd<PioSwdDrv<PioSm>, IoPin, CycleDelay>;
/// The JTAG driver role of [`MuxDeps`].
#[cfg(all(feature = "flash", rp_accel))]
type MuxJtag = ComposedJtagDrv<PioSwdDeps<PioSm>, IoPin, CycleDelay>;

/// The flash-wins arbiter owning the one physical adapter.
#[cfg(feature = "flash")]
type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
/// The flash-algorithm runtime over an ungated flash-side handle on the mux.
#[cfg(feature = "flash")]
type Backend = RoleMuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, CycleDelay, PrimaryTarget, Selector>;
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
    RebootVendor<RpUsbBoot, RebootBusy>,
    VendorChain<VendorFlash<AllDecoders, Sink>, VendorChain<VcomBaudVendor, VcomStatsVendor>>,
>;
/// The flash build's engine: gated by the mux, with the vendor flash commands composed in.
#[cfg(feature = "flash")]
type Engine = RoleMuxDapEngine<
    'static,
    MuxDeps,
    MuxSwd,
    MuxJtag,
    CycleDelay,
    Uart0Swo,
    Vendor,
    StatusLedType,
>;

/// The one physical adapter's dependency value, inlined at its single use site in each build.
#[cfg(not(rp_accel))]
macro_rules! probe_deps {
    ($parts:expr) => {
        $parts.deps
    };
}

/// The accelerated spelling of [`probe_deps`]: PIO0 on SWCLK/SWDIO, bit-bang JTAG over the loan.
#[cfg(rp_accel)]
macro_rules! probe_deps {
    ($parts:expr) => {{
        static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        let pins = PioSwdPins {
            swclk: $parts.swclk,
            swdio_out: $parts.swdio,
            // One bidirectional wire and no level shifter, so no separate readback or OE pad.
            swdio_in: None,
            oen: None,
            nreset: Some($parts.nreset),
        };
        // SAFETY: the bring-up above hands over PIO0 and these pads, and this runs once.
        let deps = unsafe { pio_swd_deps(PioBlock::Pio0, pins, daplink_rp_embassy::CPU_HZ, &SCAN) };
        ComposedJtagDeps::new(deps, $parts.jtag_lines, CycleDelay)
    }};
}

/// Pipelined CMSIS-DAP command-queue depth, also the advertised `DAP_PACKET_COUNT`.
const DAP_QUEUE_DEPTH: usize = 8;

/// Per-slot byte capacity of the command queue: one full-speed DAP v2 bulk packet.
const DAP_QUEUE_SLOT: usize = DAP_PACKET_SIZE as usize;

/// This image's command queue: the depth the host is told about and the storage behind it.
type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

/// The `DAP_Info(MaxPacketCount)` this image advertises; fed to the engine below.
const DAP_PACKET_COUNT: u8 = 8;

// Advertised count, ring depth and slot size are one fact; widening one alone fails the build.
const _: () = {
    assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
    assert!(Queue::new().packet_capacity() == DAP_PACKET_SIZE as usize);
};

/// Pipelined CMSIS-DAP loop: a reader fills the queue, an executor runs and answers the oldest.
/// `dap_led` pulses once per command; `flash_led` tracks whether the flash mux holds the wire.
async fn dap_loop_pipelined<D: Activity, F: Activity>(
    class: &mut DapV2Class<UsbDrv>,
    dap: &mut Engine,
    queue: &RefCell<Queue>,
    mut dap_led: D,
    mut flash_led: F,
    #[cfg(feature = "flash")] flash_mux: &'static Mux,
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
        let (dap_led, flash_led) = (&mut dap_led, &mut flash_led);

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
                dap_led.pulse();
                #[cfg(feature = "flash")]
                let flashing = flash_mux.is_flashing();
                #[cfg(not(feature = "flash"))]
                let flashing = false;
                flash_led.level(flashing);

                if queue.borrow_mut().post_response(&resp[..len]).is_err() {
                    // The drain below empties the ring before the next command is taken.
                    debug_assert!(false, "response ring full after a single command");
                    #[cfg(feature = "defmt")]
                    defmt::warn!("dap: response ring full, response dropped");
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

/// Carrier heartbeat period: the one "firmware is alive" signal independent of host traffic.
#[cfg(feature = "carrier-leds")]
const HEARTBEAT_PERIOD_MS: u64 = 500;

/// Periodic heartbeat blink, spawned once when the carrier debug bank is wired.
#[cfg(feature = "carrier-leds")]
#[embassy_executor::task]
async fn heartbeat_task(mut led: CarrierBankLed) -> ! {
    loop {
        embassy_time::Timer::after_millis(HEARTBEAT_PERIOD_MS).await;
        led.pulse();
    }
}

/// Mass-storage bulk transport: the CBW/CSW cycle, bouncing D+ on each post-transfer remount.
#[cfg(feature = "flash")]
#[embassy_executor::task]
async fn msc_task(
    mut msc: MscEndpoints<'static, UsbDrv>,
    vfs: &'static mut FlashVolume<Sink>,
) -> ! {
    daplink_embassy_usb::msc::msc_task_with_reconnect(&mut msc, vfs, RpUsbReconnect).await
}

/// Reset-surviving probe state (boot counter, last transfer status) in `.uninit` RAM; on this
/// part only a power cycle clears it.
#[cfg(feature = "flash")]
#[link_section = ".uninit.RESET_STAMP"]
static mut RESET_STAMP: core::mem::MaybeUninit<daplink_vfs::ResetStamp> =
    core::mem::MaybeUninit::uninit();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Boot the stamp first: it arms the fault latch, and everything below can panic.
    // SAFETY: `RESET_STAMP` is only touched here, once, before anything concurrent runs.
    #[cfg(feature = "flash")]
    let stamp = daplink_vfs::ResetStamp::boot_in_place(unsafe {
        &mut *core::ptr::addr_of_mut!(RESET_STAMP)
    });

    #[cfg(feature = "defmt")]
    defmt::info!("rp2040-pico-embassy: boot");

    let p = init_rp();
    #[cfg(not(rp_accel))]
    let parts = probe_parts_pico(p, CycleDelay);
    #[cfg(rp_accel)]
    let parts = probe_parts_pico_accel(p);
    let cfg = DapConfig::daplink(parts.serial).with_product(product_string!("rp2040"));
    #[cfg(feature = "defmt")]
    defmt::info!("rp2040-pico-embassy: serial {=str}", parts.serial);
    let status_led = status_led(parts.status_pin);
    #[cfg(feature = "carrier-leds")]
    let carrier = carrier_leds(parts.carrier_pins);

    // SWO capture: the DMA ring fills from UART0 RX, halted until DAP_SWO_Control(Start).
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
    let mut engine: Engine = Engine::with_vendor(
        probe_deps!(parts),
        status_led,
        CycleDelay,
        swo,
        DAP_FW_VERSION,
        cfg.max_packet_size,
        VendorChain::new(
            RebootVendor::new(RpUsbBoot),
            VendorChain::new(
                VcomBaudVendor::new(vcom_baud_state),
                VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
            ),
        ),
    )
    .with_packet_count(DAP_PACKET_COUNT);
    #[cfg(feature = "flash")]
    let (mut engine, vfs, mux) = {
        static MUX: static_cell::StaticCell<Mux> = static_cell::StaticCell::new();
        static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        // Two front-ends, two backends; the mux grants one session and refuses the other.
        static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> = ConstStaticCell::new([0; SCRATCH]);
        static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

        let mux: &'static Mux = MUX.init(SwdMux::new(probe_deps!(parts)));
        let table = flash_table();
        let vendor_backend: Backend =
            role_build_flash_backend(mux, CycleDelay, PRIMARY, table, VENDOR_SCRATCH.take());
        let engine = role_mux_dap_engine_with_vendor(
            mux,
            CycleDelay,
            swo,
            cfg.max_packet_size,
            &MUX_SCAN,
            VendorChain::new(
                RebootVendor::with_busy_check(RpUsbBoot, mux),
                VendorChain::new(
                    VendorFlash::new(FlashSink::new(vendor_backend)),
                    VendorChain::new(
                        VcomBaudVendor::new(vcom_baud_state),
                        VcomStatsVendor::new(daplink_embassy_usb::vcom_counters),
                    ),
                ),
            ),
            status_led,
        )
        .with_packet_count(DAP_PACKET_COUNT);
        let vfs_backend: Backend =
            role_build_flash_backend(mux, CycleDelay, PRIMARY, table, flash_scratch());
        let vfs = daplink_vfs::assemble_info(
            &VOLUME,
            FlashSink::new(vfs_backend),
            daplink_vfs::DeviceInfo {
                version: env!("CARGO_PKG_VERSION"),
                serial: parts.serial,
                reset_count: stamp.reset_count(),
            },
            Some(stamp),
            "DAPLINK",
        );
        (engine, vfs, mux)
    };

    #[cfg(not(feature = "flash"))]
    let (usb, mut class, cdc_tx, cdc_rx, control) =
        daplink_embassy_usb::usb_device(Driver::new(parts.io.usb, Irqs), &cfg);
    #[cfg(feature = "flash")]
    let (usb, mut class, cdc_tx, cdc_rx, control, msc) =
        daplink_embassy_usb::usb_device_msc(Driver::new(parts.io.usb, Irqs), &cfg);
    spawner.spawn(usb_task(usb).expect("usb_task spawns once"));
    spawner.spawn(reboot_watch().expect("reboot_watch spawns once"));
    #[cfg(feature = "flash")]
    spawner.spawn(msc_task(msc, vfs).expect("msc_task spawns once"));

    // VCOM UART1: TX=GP4, RX=GP5, wired to the target's serial port on the carrier.
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = DEFAULT_VCOM_BAUD;
    // RX ring covers 22 ms at the top baud, so a service gap that long costs no byte.
    static TX_BUF: ConstStaticCell<[u8; 128]> = ConstStaticCell::new([0; 128]);
    static RX_BUF: ConstStaticCell<[u8; 2048]> = ConstStaticCell::new([0; 2048]);
    let uart = BufferedUart::new(
        parts.io.uart,
        parts.io.uart_tx,
        parts.io.uart_rx,
        Irqs,
        TX_BUF.take(),
        RX_BUF.take(),
        uart_cfg,
    );
    #[cfg(feature = "carrier-leds")]
    let vcom_led: &'static mut dyn Activity = {
        static VCOM_LED: static_cell::StaticCell<CarrierBankLed> = static_cell::StaticCell::new();
        VCOM_LED.init(carrier.vcom)
    };
    #[cfg(not(feature = "carrier-leds"))]
    let vcom_led: &'static mut dyn Activity = daplink_rs::noop_activity();
    spawner.spawn(
        vcom_task(
            cdc_tx,
            cdc_rx,
            control,
            RpVcom::new(
                uart,
                parts.io.vcom_rx_dma,
                parts.io.vcom_chain_dma,
                parts.io.vcom_tx_dma,
            ),
            DEFAULT_VCOM_BAUD,
            vcom_led,
        )
        .expect("vcom_task spawns once"),
    );

    // Composition finished, so this boot is not a reset loop: forgive the fault streak.
    #[cfg(feature = "flash")]
    daplink_vfs::stamp::clear_fault_streak();

    #[cfg(feature = "carrier-leds")]
    let (dap_led, flash_led) = (carrier.dap, carrier.flash);
    #[cfg(not(feature = "carrier-leds"))]
    let (dap_led, flash_led) = ((), ());

    // Caller-owned queue storage; `DAP_PACKET_COUNT` above names the same depth.
    static DAP_QUEUE: ConstStaticCell<RefCell<Queue>> =
        ConstStaticCell::new(RefCell::new(Queue::new()));
    let queue = DAP_QUEUE.take();

    #[cfg(feature = "flash")]
    {
        dap_loop_pipelined(&mut class, &mut engine, queue, dap_led, flash_led, mux).await
    }
    #[cfg(not(feature = "flash"))]
    {
        dap_loop_pipelined(&mut class, &mut engine, queue, dap_led, flash_led).await
    }
}
