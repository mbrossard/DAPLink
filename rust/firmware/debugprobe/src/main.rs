//! CMSIS-DAP v2 probe for the Raspberry Pi Debug Probe (RP2040) on embassy: SWD only, plus VCOM.

#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{UART1, USB};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use panic_halt as _;
use static_cell::ConstStaticCell;

use daplink_embassy_usb::{session_ended, session_epoch, DapV2Class};
use daplink_rp_embassy::{
    init as init_rp, reboot_watch, usb_task, vcom_baud_state, vcom_task, CycleDelay, RpUsbBoot,
    RpVcom, UsbDrv, VcomDmaInterruptHandler,
};
use daplink_rs::daplink_core::{
    product_string, DapConfig, DapQueue, QueueError, Submit, DAP_FW_VERSION, DAP_PACKET_SIZE,
    DEFAULT_VCOM_BAUD,
};
use daplink_rs::{NoSwo, NoopLeds, RebootVendor, VcomBaudVendor, VcomStatsVendor, VendorChain};

#[cfg(not(rp_accel))]
use daplink_rp_embassy::{probe_parts_debugprobe, SplitIoPin};
#[cfg(not(rp_accel))]
use daplink_rs::SwdOnlyDap;

#[cfg(rp_accel)]
use daplink_rp_embassy::probe_parts_debugprobe_accel;
#[cfg(rp_accel)]
use daplink_rp_pio::{pio_swd_deps, PioBlock, PioSm, PioSwdOnlyDap, PioSwdPins};
#[cfg(rp_accel)]
use daplink_rs::ScanChainStorage;

// One wire engine per image: `bitbang` wins where both are on.
#[cfg(not(any(feature = "accel", feature = "bitbang")))]
compile_error!("enable `accel` (the default wire engine) or `bitbang`");

// embassy-rp owns __pre_init (SIO reset); interpose and tail-call the real one.
daplink_rs::ram_text_pre_init!(wrapping);

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
    DMA_IRQ_1 => VcomDmaInterruptHandler;
});

#[cfg(not(feature = "overclock-180mhz"))]
const _: () = assert!(daplink_rp_embassy::CPU_HZ == 125_000_000);
#[cfg(feature = "overclock-180mhz")]
const _: () = assert!(daplink_rp_embassy::CPU_HZ == 180_000_000);

#[cfg(all(rp_accel, not(feature = "overclock-180mhz")))]
const _: () = assert!(daplink_rp_pio::CYCLES_PER_BIT == 4);
#[cfg(all(rp_accel, feature = "overclock-180mhz"))]
const _: () = assert!(daplink_rp_pio::CYCLES_PER_BIT == 5);

/// The reboot-to-bootloader vendor handler; no flash session exists here to gate it behind.
type Vendor = VendorChain<RebootVendor<RpUsbBoot>, VendorChain<VcomBaudVendor, VcomStatsVendor>>;

/// The engine: PIO SWD over the split-pad wire, no JTAG, no SWO, no LEDs, reboot in the vendor slot.
#[cfg(rp_accel)]
type Engine = PioSwdOnlyDap<'static, PioSm, CycleDelay, NoSwo, Vendor, NoopLeds>;

/// The same engine with the wire bit-banged by the CPU, for A/B against the row above.
#[cfg(not(rp_accel))]
type Engine = SwdOnlyDap<'static, SplitIoPin, CycleDelay, NoSwo, Vendor, NoopLeds>;

/// The one physical adapter's dependency value, inlined at its single use site in each build.
#[cfg(not(rp_accel))]
macro_rules! probe_deps {
    ($parts:expr) => {
        $parts.deps
    };
}

/// The accelerated spelling of [`probe_deps`]: PIO0 driving SWCLK and SWDIO, the read turnaround
/// releasing the driven pad while the shifted input stays sampled.
#[cfg(rp_accel)]
macro_rules! probe_deps {
    ($parts:expr) => {{
        static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
            ScanChainStorage::new();
        let pins = PioSwdPins {
            swclk: $parts.swclk,
            swdio_out: $parts.swdio_out,
            // The board's shifted readback pad: live whether or not the probe drives the wire.
            swdio_in: Some($parts.swdio_in),
            // No output-enable pad on this board; the turnaround tristates the driven one.
            oen: None,
            // No reset line is wired out.
            nreset: None,
        };
        // SAFETY: the bring-up above hands over PIO0 and these pads, and this runs once.
        unsafe { pio_swd_deps(PioBlock::Pio0, pins, daplink_rp_embassy::CPU_HZ, &SCAN) }
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

        // Host gone: release the wire so protocol state cannot leak into the next session.
        dap.suspend();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init_rp();
    #[cfg(not(rp_accel))]
    let parts = probe_parts_debugprobe(p, CycleDelay);
    #[cfg(rp_accel)]
    let parts = probe_parts_debugprobe_accel(p);
    let cfg = DapConfig::daplink(parts.serial).with_product(product_string!("debugprobe"));

    let mut engine: Engine = Engine::with_vendor(
        probe_deps!(parts),
        NoopLeds,
        CycleDelay,
        NoSwo,
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

    let (usb, mut class, cdc_tx, cdc_rx, control) =
        daplink_embassy_usb::usb_device(Driver::new(parts.io.usb, Irqs), &cfg);
    spawner.spawn(usb_task(usb).expect("usb_task spawns once"));
    spawner.spawn(reboot_watch().expect("reboot_watch spawns once"));

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
            daplink_rs::noop_activity(),
        )
        .expect("vcom_task spawns once"),
    );

    // Caller-owned queue storage; `DAP_PACKET_COUNT` above names the same depth.
    static DAP_QUEUE: ConstStaticCell<RefCell<Queue>> =
        ConstStaticCell::new(RefCell::new(Queue::new()));
    let queue = DAP_QUEUE.take();

    dap_loop_pipelined(&mut class, &mut engine, queue).await
}
