//! embassy-usb adapter: the CMSIS-DAP v2 bulk class, the composite device, and the VCOM bridge.

#![no_std]

#[cfg(test)]
extern crate std;

#[cfg(feature = "flash")]
pub mod msc;

use core::cell::RefCell;
use core::sync::atomic::Ordering;

use embassy_futures::join::join;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex};
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender, State};
use embassy_usb::control::{InResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Config, Handler, UsbDevice};
use portable_atomic::AtomicU32;
use static_cell::{ConstStaticCell, StaticCell};

use daplink_rs::daplink_core::webusb;
use daplink_rs::daplink_core::winusb::{
    BOS_CAPABILITY_PLATFORM, BOS_PLATFORM, MSOS_VENDOR_CODE, MS_OS_DESCRIPTOR,
};
use daplink_rs::daplink_core::{
    DapConfig, CDC_PACKET_SIZE, DAP_V2_INTERFACE_CLASS, USB_MAX_POWER_MA,
};

/// CMSIS-DAP v2 hosts discover the probe by this interface string.
const INTERFACE_STRING: &str = "CMSIS-DAP v2 Interface";

/// Counts host sessions of the DAP v2 interface, bumped once per session boundary.
static SESSION_EPOCH: AtomicU32 = AtomicU32::new(0);
/// Wakes the one loop parked in [`session_ended`]; the epoch, not this edge, is the truth.
static BOUNDARY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The session counter, to be snapshotted once a session has started.
pub fn session_epoch() -> u32 {
    SESSION_EPOCH.load(Ordering::Relaxed)
}

/// Resolve once the session `epoch` was snapshotted under has ended; a boundary is never missed.
pub async fn session_ended(epoch: u32) {
    while session_epoch() == epoch {
        BOUNDARY.wait().await;
    }
}

/// Bump first, wake second: a waiter woken by this edge must already see the new count.
fn session_boundary() {
    SESSION_EPOCH.fetch_add(1, Ordering::Relaxed);
    BOUNDARY.signal(());
}

struct DapV2Handler {
    iface_string: StringIndex,
}

impl Handler for DapV2Handler {
    // Suspend and resume do not end a session; only these three boundaries do.
    fn reset(&mut self) {
        session_boundary();
    }

    fn configured(&mut self, configured: bool) {
        if !configured {
            session_boundary();
        }
    }

    fn enabled(&mut self, enabled: bool) {
        if !enabled {
            session_boundary();
        }
    }

    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        (index == self.iface_string).then_some(INTERFACE_STRING)
    }

    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        // Device recipient required: the code and index alone are not unique across recipients.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Device {
            return None;
        }
        if req.request == MSOS_VENDOR_CODE && req.index == 0x07 {
            return Some(InResponse::Accepted(&MS_OS_DESCRIPTOR));
        }
        webusb::is_get_url_request(req.request, req.value, req.index)
            .then_some(InResponse::Accepted(&webusb::LANDING_URL_DESCRIPTOR))
    }
}

/// CMSIS-DAP v2 bulk interface; each bulk transfer is exactly one command or response, never reassembled.
pub struct DapV2Class<D: Driver<'static>> {
    read_ep: D::EndpointOut,
    write_ep: D::EndpointIn,
}

impl<D: Driver<'static>> DapV2Class<D> {
    fn new(builder: &mut Builder<'static, D>, max_packet_size: u16) -> Self {
        static HANDLER: StaticCell<DapV2Handler> = StaticCell::new();
        let iface_string = builder.string();
        let (class, subclass, protocol) = DAP_V2_INTERFACE_CLASS;
        let mut function = builder.function(class, subclass, protocol);
        let mut interface = function.interface();
        let mut alt = interface.alt_setting(class, subclass, protocol, Some(iface_string));
        // WebUSB before MS OS 2.0, the order the polled adapter registers them in.
        alt.bos_capability(BOS_CAPABILITY_PLATFORM, &webusb::BOS_PLATFORM);
        alt.bos_capability(BOS_CAPABILITY_PLATFORM, &BOS_PLATFORM);
        let read_ep = alt.endpoint_bulk_out(None, max_packet_size);
        let write_ep = alt.endpoint_bulk_in(None, max_packet_size);
        drop(function);
        builder.handler(HANDLER.init(DapV2Handler { iface_string }));
        Self { read_ep, write_ep }
    }

    /// Wait for the host to enable the interface.
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    /// Read one command packet.
    pub async fn read_command(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(buf).await
    }

    /// Write one response packet.
    pub async fn write_response(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }

    /// Borrow the endpoints as disjoint halves, so reader and writer futures share one task.
    pub fn split(&mut self) -> (DapV2Reader<'_, D>, DapV2Writer<'_, D>) {
        (
            DapV2Reader {
                read_ep: &mut self.read_ep,
            },
            DapV2Writer {
                write_ep: &mut self.write_ep,
            },
        )
    }
}

/// The command-reading half of a [`split`](DapV2Class::split) class.
pub struct DapV2Reader<'a, D: Driver<'static>> {
    read_ep: &'a mut D::EndpointOut,
}

impl<D: Driver<'static>> DapV2Reader<'_, D> {
    /// Read one command packet; an `Err` means the host went away.
    pub async fn read_command(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(buf).await
    }
}

/// The response-writing half of a [`split`](DapV2Class::split) class.
pub struct DapV2Writer<'a, D: Driver<'static>> {
    write_ep: &'a mut D::EndpointIn,
}

impl<D: Driver<'static>> DapV2Writer<'_, D> {
    /// Write one response packet; an `Err` means the host went away.
    pub async fn write_response(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }
}

/// The shared composition: DAP v2 on interface 0, then the CDC-ACM VCOM function on 1 and 2.
// A macro, not a helper: returning the half-built `Builder` copies it and changes every image.
macro_rules! compose_classes {
    ($driver:expr, $cfg:expr, $builder:ident, $class:ident, $tx:ident, $rx:ident, $ctl:ident) => {
        let mut config = Config::new($cfg.vid, $cfg.pid);
        config.manufacturer = Some($cfg.manufacturer);
        config.product = Some($cfg.product);
        config.serial_number = Some($cfg.serial);
        config.max_power = USB_MAX_POWER_MA;
        config.max_packet_size_0 = $cfg.max_packet_size_0;
        // The VCOM function needs an IAD, which Windows honors only on an EF/02/01 device.
        config.composite_with_iads = true;
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;

        static CONFIG_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
        static BOS_DESC: ConstStaticCell<[u8; 96]> = ConstStaticCell::new([0; 96]);
        static MSOS_DESC: ConstStaticCell<[u8; 0]> = ConstStaticCell::new([]);
        // Must fit the longest STRING descriptor; embassy-usb panics mid-enumeration otherwise.
        static CONTROL_BUF: ConstStaticCell<[u8; 128]> = ConstStaticCell::new([0; 128]);

        let mut $builder = Builder::new(
            $driver,
            config,
            CONFIG_DESC.take(),
            BOS_DESC.take(),
            MSOS_DESC.take(),
            CONTROL_BUF.take(),
        );
        let $class = DapV2Class::new(&mut $builder, $cfg.max_packet_size);
        static CDC_STATE: StaticCell<State> = StaticCell::new();
        let cdc = CdcAcmClass::new(&mut $builder, CDC_STATE.init(State::new()), CDC_PACKET_SIZE);
        let ($tx, $rx, $ctl) = cdc.split_with_control();
    };
}

/// Build the composite USB device: the CMSIS-DAP v2 bulk interface plus a CDC-ACM VCOM function.
#[cfg(not(feature = "flash"))]
#[allow(clippy::type_complexity)]
pub fn usb_device<D: Driver<'static>>(
    driver: D,
    cfg: &DapConfig<'static>,
) -> (
    UsbDevice<'static, D>,
    DapV2Class<D>,
    Sender<'static, D>,
    Receiver<'static, D>,
    ControlChanged<'static>,
) {
    compose_classes!(driver, cfg, builder, class, cdc_tx, cdc_rx, control);
    (builder.build(), class, cdc_tx, cdc_rx, control)
}

/// Build the DAP-only device, for a board with no VCOM wiring: one interface, so no IAD and no
/// composite device class. Its own body, so the shared path's image stays byte-identical.
pub fn usb_device_dap_only<D: Driver<'static>>(
    driver: D,
    cfg: &DapConfig<'static>,
) -> (UsbDevice<'static, D>, DapV2Class<D>) {
    let mut config = Config::new(cfg.vid, cfg.pid);
    config.manufacturer = Some(cfg.manufacturer);
    config.product = Some(cfg.product);
    config.serial_number = Some(cfg.serial);
    config.max_power = USB_MAX_POWER_MA;
    config.max_packet_size_0 = cfg.max_packet_size_0;

    static CONFIG_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
    static BOS_DESC: ConstStaticCell<[u8; 96]> = ConstStaticCell::new([0; 96]);
    static MSOS_DESC: ConstStaticCell<[u8; 0]> = ConstStaticCell::new([]);
    // Must fit the longest STRING descriptor; embassy-usb panics mid-enumeration otherwise.
    static CONTROL_BUF: ConstStaticCell<[u8; 128]> = ConstStaticCell::new([0; 128]);

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.take(),
        BOS_DESC.take(),
        MSOS_DESC.take(),
        CONTROL_BUF.take(),
    );
    let class = DapV2Class::new(&mut builder, cfg.max_packet_size);
    (builder.build(), class)
}

/// [`usb_device`] with mass storage appended, so the drive lands on interface 3 and the interfaces
/// below it keep their numbers. `EP` is its bulk packet size, inferred from the caller's binding.
#[cfg(feature = "flash")]
#[allow(clippy::type_complexity)]
pub fn usb_device_msc<D: Driver<'static>, const EP: usize>(
    driver: D,
    cfg: &DapConfig<'static>,
) -> (
    UsbDevice<'static, D>,
    DapV2Class<D>,
    Sender<'static, D>,
    Receiver<'static, D>,
    ControlChanged<'static>,
    msc::MscEndpoints<'static, D, EP>,
) {
    static MSC_HANDLER: StaticCell<msc::MscControlHandler<'static>> = StaticCell::new();
    static MSC_RESET: msc::BotReset = msc::BotReset::new();
    compose_classes!(driver, cfg, builder, class, cdc_tx, cdc_rx, control);
    let eps = msc::add_msc_function(
        &mut builder,
        MSC_HANDLER.init(msc::MscControlHandler::new(&MSC_RESET)),
    );
    (builder.build(), class, cdc_tx, cdc_rx, control, eps)
}

/// The counters a seam sources itself, free-running since boot; a slot outside the seam's
/// `SEAM_VALID` mask stays zero.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct SeamStats {
    /// Bytes dropped because the seam's receive ring was full.
    pub ring_full: u32,
    /// Error-flagged reception, in the unit the seam's `SEAM_UNITS` declares.
    pub frame_err: u32,
    /// Receive overrun, in the unit the seam's `SEAM_UNITS` declares.
    pub overrun: u32,
    /// Bytes the transmit path put on the wire.
    pub uart_tx: u32,
    /// Receive-DMA faults the seam recovered from.
    pub rx_dma_err: u32,
}

/// Hardware UART seam behind [`vcom_bridge`]; a platform crate wraps its HAL UART in this.
// Single-executor firmware: the futures never cross threads, so no `Send` bound is wanted.
#[allow(async_fn_in_trait)]
pub trait VcomUart {
    /// Which [`SeamStats`] counters this seam genuinely observes, as `VCOM_VALID_*` bits. A
    /// counter left out of this mask is reported as zero and the host is told not to read it.
    const SEAM_VALID: u32;
    /// Evidence quality per sourced slot, as `vcom_grade` bits; overstating one is the defect
    /// the grade word exists to prevent.
    const SEAM_GRADE: u32;
    /// `vcom_episodes` bits for the slots this seam counts in episodes rather than bytes.
    const SEAM_UNITS: u32;

    /// Read into `buf`, waiting for at least one byte; a hard UART error is the impl's to absorb.
    /// Dropped mid-poll routinely, so only a poll returning `Ready` may consume bytes.
    async fn read(&mut self, buf: &mut [u8]) -> usize;
    /// Take what the seam holds without waiting; a busy host starves the return leg otherwise.
    fn read_ready(&mut self, buf: &mut [u8]) -> usize;
    /// The seam's counter ledger, covering exactly the slots [`Self::SEAM_VALID`] names.
    fn seam_stats(&self) -> SeamStats;
    /// Write the whole buffer to the UART TX line.
    async fn write_all(&mut self, data: &[u8]);
    /// Resolve once every accepted TX byte has left the wire, the shift register included.
    async fn drain_tx(&mut self);
    /// Reprogram the UART divisor; a rate the UART cannot realize is the impl's to reject.
    fn set_baudrate(&mut self, baud: u32);
}

/// Cap on one return-leg drain pass, so it stays bounded rather than exhaustive.
const DRAIN_BYTES: usize = 4 * CDC_PACKET_SIZE as usize;

/// Handover buffer between the bridge's two halves; a full pipe parks the UART half, losing no byte.
const RETURN_PIPE: usize = 512;

/// How long the UART half waits before looking again when the handover pipe is full.
const PIPE_FULL_POLL: Duration = Duration::from_micros(200);

/// How long a part-filled return packet waits for company before it is sent short.
const COALESCE: Duration = Duration::from_millis(1);

/// The deadline arm when nothing is staged: long enough that only real events drive the select.
const IDLE_DEADLINE: Duration = Duration::from_secs(3600);

static UART_RX: AtomicU32 = AtomicU32::new(0);
static CDC_IN: AtomicU32 = AtomicU32::new(0);
static HOST_DROPPED: AtomicU32 = AtomicU32::new(0);
static CDC_OUT: AtomicU32 = AtomicU32::new(0);
static RING_FULL: AtomicU32 = AtomicU32::new(0);
static FRAME_ERR: AtomicU32 = AtomicU32::new(0);
static OVERRUN: AtomicU32 = AtomicU32::new(0);
static UART_TX: AtomicU32 = AtomicU32::new(0);
static RX_DMA_ERR: AtomicU32 = AtomicU32::new(0);
/// Nominal rate the seam's divisor was last programmed for.
static APPLIED_BAUD: AtomicU32 = AtomicU32::new(daplink_rs::daplink_core::DEFAULT_VCOM_BAUD);
/// Nominal rate the host asked for that has yet to reach the divisor, `0` when none is in flight.
/// [`apply_line_change`] can hold the wire for a whole transmit drain, so this is not a formality.
static PENDING_BAUD: AtomicU32 = AtomicU32::new(0);

/// Coverage, grade and unit words the running seam published; the bridge's own four slots are
/// always observed, exact, and in bytes.
static SEAM_VALID: AtomicU32 = AtomicU32::new(0);
static SEAM_GRADE: AtomicU32 = AtomicU32::new(0);
static SEAM_UNITS: AtomicU32 = AtomicU32::new(0);

/// The slots counted at the USB endpoints, which no seam can decline.
const BRIDGE_VALID: u32 = daplink_rs::VCOM_VALID_IDENTITY | daplink_rs::VCOM_VALID_CDC_OUT;
const BRIDGE_GRADE: u32 =
    daplink_rs::vcom_grade(daplink_rs::VCOM_SLOT_UART_RX, daplink_rs::VCOM_GRADE_EXACT)
        | daplink_rs::vcom_grade(daplink_rs::VCOM_SLOT_CDC_IN, daplink_rs::VCOM_GRADE_EXACT)
        | daplink_rs::vcom_grade(
            daplink_rs::VCOM_SLOT_HOST_DROPPED,
            daplink_rs::VCOM_GRADE_EXACT,
        )
        | daplink_rs::vcom_grade(daplink_rs::VCOM_SLOT_CDC_OUT, daplink_rs::VCOM_GRADE_EXACT);

/// CDC packets each way, and how many were shorter than the endpoint's maximum.
#[cfg(feature = "usb-counters")]
#[no_mangle]
pub static PROBE_CDC_IN_PKTS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-counters")]
#[no_mangle]
pub static PROBE_CDC_IN_SHORT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-counters")]
#[no_mangle]
pub static PROBE_CDC_IN_BYTES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-counters")]
#[no_mangle]
pub static PROBE_CDC_OUT_PKTS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-counters")]
#[no_mangle]
pub static PROBE_CDC_OUT_SHORT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-counters")]
#[no_mangle]
pub static PROBE_CDC_OUT_BYTES: AtomicU32 = AtomicU32::new(0);
/// Zero-length packets sent to terminate a full-size transfer; each still costs a frame slot.
#[cfg(feature = "usb-counters")]
#[no_mangle]
pub static PROBE_CDC_IN_ZLP: AtomicU32 = AtomicU32::new(0);

/// The VCOM bridge's conservation ledger, for the [`0x9D`](daplink_rs::VCOM_STATS_CMD_ID) query.
pub fn vcom_counters() -> daplink_rs::VcomCounters {
    daplink_rs::VcomCounters {
        uart_rx: UART_RX.load(Ordering::Relaxed),
        cdc_in: CDC_IN.load(Ordering::Relaxed),
        host_dropped: HOST_DROPPED.load(Ordering::Relaxed),
        ring_full: RING_FULL.load(Ordering::Relaxed),
        frame_err: FRAME_ERR.load(Ordering::Relaxed),
        overrun: OVERRUN.load(Ordering::Relaxed),
        cdc_out: CDC_OUT.load(Ordering::Relaxed),
        uart_tx: UART_TX.load(Ordering::Relaxed),
        rx_dma_err: RX_DMA_ERR.load(Ordering::Relaxed),
        valid: BRIDGE_VALID | SEAM_VALID.load(Ordering::Relaxed),
        grade: BRIDGE_GRADE | SEAM_GRADE.load(Ordering::Relaxed),
        units: SEAM_UNITS.load(Ordering::Relaxed),
    }
}

/// The bridge's answer to the [`0x9E`](daplink_rs::VCOM_BAUD_CMD_ID) query, over the `realized`
/// rate the platform crate reads back from its divisor. Pending is loaded first: a change that
/// starts between the two loads then reads unsettled, never settled at the rate it is leaving.
pub fn vcom_baud_state(realized: u32) -> daplink_rs::VcomBaudState {
    let pending = PENDING_BAUD.load(Ordering::Acquire);
    daplink_rs::VcomBaudState::deferred(
        realized,
        APPLIED_BAUD.load(Ordering::Relaxed),
        pending == 0,
    )
}

/// One UART-half event, resolved before the UART is touched so no select borrow is held.
enum VcomEvent {
    FromHost(usize),
    HostGone,
    FromUart(usize),
    LineCoding(u32),
    /// The handover pipe was full, so the receive arm was not armed at all this pass.
    PipeFull,
}

/// Bridge the CDC-ACM VCOM port to the target UART, following the host's requested baud; `led`
/// pulses once per packet moved either way ([`daplink_rs::noop_activity`] where none is wired).
pub async fn vcom_bridge<D: Driver<'static>, U: VcomUart>(
    cdc_tx: &mut Sender<'static, D>,
    cdc_rx: &mut Receiver<'static, D>,
    control: &ControlChanged<'static>,
    uart: &mut U,
    applied_baud: u32,
    led: &mut dyn daplink_rs::Activity,
) -> ! {
    // Single executor, and no borrow is held across an await: the cheap mutex is sound.
    let pipe: Pipe<NoopRawMutex, RETURN_PIPE> = Pipe::new();
    let led = RefCell::new(led);
    // Published before the first byte can move, so an idle bridge already reports its coverage.
    APPLIED_BAUD.store(applied_baud, Ordering::Relaxed);
    SEAM_VALID.store(U::SEAM_VALID, Ordering::Relaxed);
    SEAM_GRADE.store(U::SEAM_GRADE, Ordering::Relaxed);
    SEAM_UNITS.store(U::SEAM_UNITS, Ordering::Relaxed);

    let uart_half = async {
        let mut from_host = [0u8; CDC_PACKET_SIZE as usize];
        let mut from_uart = [0u8; DRAIN_BYTES];
        let mut applied_baud = applied_baud;
        loop {
            // Sized to the room the pipe has now, so staging can never block: this is its only writer.
            let room = pipe.free_capacity().min(from_uart.len());
            // The block ends the select's borrow of the UART, so the handling below reaches the seam.
            let event = if room == 0 {
                match select3(
                    cdc_rx.read_packet(&mut from_host),
                    control.control_changed(),
                    Timer::after(PIPE_FULL_POLL),
                )
                .await
                {
                    Either3::First(Ok(n)) => VcomEvent::FromHost(n),
                    Either3::First(Err(_)) => VcomEvent::HostGone,
                    Either3::Second(()) => VcomEvent::LineCoding(cdc_rx.line_coding().data_rate()),
                    Either3::Third(()) => VcomEvent::PipeFull,
                }
            } else {
                match select3(
                    cdc_rx.read_packet(&mut from_host),
                    uart.read(&mut from_uart[..room]),
                    control.control_changed(),
                )
                .await
                {
                    Either3::First(Ok(n)) => VcomEvent::FromHost(n),
                    Either3::First(Err(_)) => VcomEvent::HostGone,
                    Either3::Second(n) => VcomEvent::FromUart(n),
                    Either3::Third(()) => VcomEvent::LineCoding(cdc_rx.line_coding().data_rate()),
                }
            };
            let stats = uart.seam_stats();
            RING_FULL.store(stats.ring_full, Ordering::Relaxed);
            FRAME_ERR.store(stats.frame_err, Ordering::Relaxed);
            OVERRUN.store(stats.overrun, Ordering::Relaxed);
            UART_TX.store(stats.uart_tx, Ordering::Relaxed);
            RX_DMA_ERR.store(stats.rx_dma_err, Ordering::Relaxed);
            match event {
                VcomEvent::FromHost(n) => {
                    CDC_OUT.fetch_add(n as u32, Ordering::Relaxed);
                    #[cfg(feature = "usb-counters")]
                    {
                        PROBE_CDC_OUT_PKTS.fetch_add(1, Ordering::Relaxed);
                        PROBE_CDC_OUT_BYTES.fetch_add(n as u32, Ordering::Relaxed);
                        if n < CDC_PACKET_SIZE as usize {
                            PROBE_CDC_OUT_SHORT.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    // The select polls in order, so a busy host would starve the receive arm.
                    let pending = uart.read_ready(&mut from_uart[..room]);
                    pipe.write_all(&from_uart[..pending]).await;
                    uart.write_all(&from_host[..n]).await;
                    led.borrow_mut().pulse();
                }
                // Host gone: drop the dead session's return bytes, then wait for the next one.
                VcomEvent::HostGone => {
                    discard_stale(&pipe);
                    cdc_rx.wait_connection().await;
                }
                VcomEvent::FromUart(n) if n > 0 => pipe.write_all(&from_uart[..n]).await,
                // A zero-length ready read is the seam absorbing a UART error; nothing to forward.
                VcomEvent::FromUart(_) => {}
                // No read was armed, so observe the cursor by hand or a lapped ring goes uncharged.
                VcomEvent::PipeFull => {
                    uart.read_ready(&mut []);
                }
                VcomEvent::LineCoding(baud) => {
                    if baud != 0 && baud != applied_baud {
                        apply_line_change(uart, &pipe, &mut from_uart, baud).await;
                        applied_baud = baud;
                    }
                }
            }
        }
    };

    let host_half = async {
        let mut from_pipe = [0u8; DRAIN_BYTES];
        let mut staged = Staged::new();
        loop {
            let wait = if staged.is_idle() {
                IDLE_DEADLINE
            } else {
                COALESCE
            };
            match select(pipe.read(&mut from_pipe), Timer::after(wait)).await {
                Either::First(n) => {
                    if staged.push(cdc_tx, &from_pipe[..n]).await.is_err() {
                        cdc_tx.wait_connection().await;
                    }
                    led.borrow_mut().pulse();
                }
                Either::Second(()) => staged.flush(cdc_tx).await,
            }
        }
    };

    // Two halves, not one loop: neither leg's progress may wait on the other's.
    let (never, _) = join(uart_half, host_half).await;
    never
}

/// The line-change transaction: CDC OUT is quiesced by parking here, TX drains to wire-idle at
/// the old rate, unread old-rate RX leaves for the pipe, and only then does the divisor move.
async fn apply_line_change<U: VcomUart, M: RawMutex, const N: usize>(
    uart: &mut U,
    pipe: &Pipe<M, N>,
    scratch: &mut [u8],
    baud: u32,
) {
    // Published before the drain, which holds the wire for as long as the old rate needs: a host
    // reading 0x9E across the change is told a change is in flight, not the rate it is leaving.
    PENDING_BAUD.store(baud, Ordering::Release);
    uart.drain_tx().await;
    loop {
        let n = uart.read_ready(scratch);
        if n == 0 {
            break;
        }
        pipe.write_all(&scratch[..n]).await;
    }
    uart.set_baudrate(baud);
    APPLIED_BAUD.store(baud, Ordering::Relaxed);
    PENDING_BAUD.store(0, Ordering::Release);
}

/// Drop what the pipe still holds, charged as taken-then-dropped: its session is over and the
/// next one must not read it.
fn discard_stale<M: RawMutex, const N: usize>(pipe: &Pipe<M, N>) {
    let stale = pipe.len() as u32;
    if stale > 0 {
        UART_RX.fetch_add(stale, Ordering::Relaxed);
        HOST_DROPPED.fetch_add(stale, Ordering::Relaxed);
        pipe.clear();
    }
}

/// The one endpoint operation [`Staged`] performs, seamed so its ledger is host-testable.
#[allow(async_fn_in_trait)]
trait PacketSink {
    async fn send(&mut self, packet: &[u8]) -> Result<(), EndpointError>;
}

impl<D: Driver<'static>> PacketSink for Sender<'static, D> {
    async fn send(&mut self, packet: &[u8]) -> Result<(), EndpointError> {
        self.write_packet(packet).await
    }
}

/// The return leg's staging: full packets go out as they form, a remainder waits for [`COALESCE`].
struct Staged {
    buf: [u8; CDC_PACKET_SIZE as usize],
    len: usize,
    /// Whether the host is still holding an open transfer, which a full-size packet leaves and
    /// only a short packet or a zero-length one closes.
    unterminated: bool,
}

impl Staged {
    const fn new() -> Self {
        Self {
            buf: [0; CDC_PACKET_SIZE as usize],
            len: 0,
            unterminated: false,
        }
    }

    /// Nothing owed to the host, so the deadline arm can be parked.
    fn is_idle(&self) -> bool {
        self.len == 0 && !self.unterminated
    }

    /// Take return-leg bytes, sending every whole packet they complete. `CDC_IN` counts a byte
    /// only once the endpoint accepts its packet; until then it sits staged, in `UART_RX` alone.
    /// `Err` means the host went away mid-write and everything unaccepted was charged as dropped.
    async fn push<S: PacketSink>(&mut self, cdc_tx: &mut S, data: &[u8]) -> Result<(), ()> {
        if data.is_empty() {
            return Ok(());
        }
        UART_RX.fetch_add(data.len() as u32, Ordering::Relaxed);
        let packet = CDC_PACKET_SIZE as usize;
        let mut rest = data;
        while self.len + rest.len() >= packet {
            let take = packet - self.len;
            self.buf[self.len..packet].copy_from_slice(&rest[..take]);
            if cdc_tx.send(&self.buf[..packet]).await.is_err() {
                // The failed packet, its staged bytes included, and the rest of `data` die here.
                HOST_DROPPED.fetch_add((self.len + rest.len()) as u32, Ordering::Relaxed);
                self.len = 0;
                self.unterminated = false;
                return Err(());
            }
            CDC_IN.fetch_add(packet as u32, Ordering::Relaxed);
            #[cfg(feature = "usb-counters")]
            {
                PROBE_CDC_IN_PKTS.fetch_add(1, Ordering::Relaxed);
                PROBE_CDC_IN_BYTES.fetch_add(packet as u32, Ordering::Relaxed);
            }
            self.len = 0;
            self.unterminated = true;
            rest = &rest[take..];
        }
        if !rest.is_empty() {
            self.buf[self.len..self.len + rest.len()].copy_from_slice(rest);
            self.len += rest.len();
        }
        Ok(())
    }

    /// Release whatever is owed: a part packet, or the zero-length packet that closes a transfer
    /// the last full-size packet left open.
    async fn flush<S: PacketSink>(&mut self, cdc_tx: &mut S) {
        if self.len > 0 {
            let len = self.len;
            if cdc_tx.send(&self.buf[..len]).await.is_ok() {
                CDC_IN.fetch_add(len as u32, Ordering::Relaxed);
                #[cfg(feature = "usb-counters")]
                {
                    PROBE_CDC_IN_PKTS.fetch_add(1, Ordering::Relaxed);
                    PROBE_CDC_IN_BYTES.fetch_add(len as u32, Ordering::Relaxed);
                    PROBE_CDC_IN_SHORT.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                HOST_DROPPED.fetch_add(len as u32, Ordering::Relaxed);
            }
            self.len = 0;
            // A short packet closes the transfer by itself.
            self.unterminated = false;
        } else if self.unterminated {
            let _ = cdc_tx.send(&[]).await;
            #[cfg(feature = "usb-counters")]
            PROBE_CDC_IN_ZLP.fetch_add(1, Ordering::Relaxed);
            self.unterminated = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};

    const PACKET: usize = CDC_PACKET_SIZE as usize;

    /// Accepts `accept` packets, then refuses every further one.
    struct MockSink {
        accept: usize,
        accepted_bytes: usize,
        packets: usize,
    }

    impl MockSink {
        fn up_to(accept: usize) -> Self {
            Self {
                accept,
                accepted_bytes: 0,
                packets: 0,
            }
        }
    }

    impl PacketSink for MockSink {
        async fn send(&mut self, packet: &[u8]) -> Result<(), EndpointError> {
            if self.accept == 0 {
                return Err(EndpointError::Disabled);
            }
            self.accept -= 1;
            self.accepted_bytes += packet.len();
            self.packets += 1;
            Ok(())
        }
    }

    /// Seam mock for the line-change transaction: every call stamps a shared step counter.
    struct MockUart {
        pending: usize,
        step: u32,
        drained_at: Option<u32>,
        last_read_at: Option<u32>,
        applied_at: Option<u32>,
        applied_baud: u32,
        /// What `0x9E` would have answered while the transaction held the wire.
        state_while_draining: Option<daplink_rs::VcomBaudState>,
    }

    impl MockUart {
        fn with_pending(pending: usize) -> Self {
            Self {
                pending,
                step: 0,
                drained_at: None,
                last_read_at: None,
                applied_at: None,
                applied_baud: 0,
                state_while_draining: None,
            }
        }
    }

    impl VcomUart for MockUart {
        const SEAM_VALID: u32 = 0;
        const SEAM_GRADE: u32 = 0;
        const SEAM_UNITS: u32 = 0;

        async fn read(&mut self, _buf: &mut [u8]) -> usize {
            unreachable!("the transaction never blocks on RX")
        }

        fn read_ready(&mut self, buf: &mut [u8]) -> usize {
            self.step += 1;
            self.last_read_at = Some(self.step);
            let n = self.pending.min(buf.len());
            self.pending -= n;
            buf[..n].fill(0xa5);
            n
        }

        fn seam_stats(&self) -> SeamStats {
            SeamStats::default()
        }

        async fn write_all(&mut self, _data: &[u8]) {}

        async fn drain_tx(&mut self) {
            self.step += 1;
            self.drained_at = Some(self.step);
            self.state_while_draining = Some(vcom_baud_state(9_600));
        }

        fn set_baudrate(&mut self, baud: u32) {
            self.step += 1;
            self.applied_at = Some(self.step);
            self.applied_baud = baud;
        }
    }

    /// The mock never yields, so one poll completes any staging future.
    fn now<F: Future>(fut: F) -> F::Output {
        match pin!(fut).poll(&mut Context::from_waker(Waker::noop())) {
            Poll::Ready(v) => v,
            Poll::Pending => panic!("staging future did not complete in one poll"),
        }
    }

    fn counters() -> (u32, u32, u32) {
        (
            UART_RX.load(Ordering::Relaxed),
            CDC_IN.load(Ordering::Relaxed),
            HOST_DROPPED.load(Ordering::Relaxed),
        )
    }

    /// `uart_rx == cdc_in + host_dropped + staged`, checked as deltas from `base`.
    fn assert_reconciles(base: (u32, u32, u32), staged: &Staged) {
        let (uart_rx, cdc_in, dropped) = counters();
        let d_rx = uart_rx.wrapping_sub(base.0);
        let d_in = cdc_in.wrapping_sub(base.1);
        let d_drop = dropped.wrapping_sub(base.2);
        assert_eq!(d_rx, d_in + d_drop + staged.len as u32);
    }

    /// One test body: the ledger statics are process-global, so the scenarios must not race.
    #[test]
    fn ledger_reconciles_at_every_boundary() {
        let data = [0x5au8; 3 * PACKET];

        // A short push only stages; nothing is accepted yet.
        let base = counters();
        let mut staged = Staged::new();
        let mut sink = MockSink::up_to(usize::MAX);
        assert!(now(staged.push(&mut sink, &data[..5])).is_ok());
        assert_eq!(sink.packets, 0);
        assert_eq!(staged.len, 5);
        let after = counters();
        assert_eq!(after.0 - base.0, 5);
        assert_eq!(after.1, base.1);
        assert_eq!(after.2, base.2);
        assert_reconciles(base, &staged);

        // A successful flush moves the stage into CDC_IN, exactly once.
        now(staged.flush(&mut sink));
        assert_eq!(sink.accepted_bytes, 5);
        let after = counters();
        assert_eq!(after.1 - base.1, 5);
        assert_reconciles(base, &staged);

        // Full packets are charged as the endpoint accepts them; the remainder stays staged.
        let base = counters();
        assert!(now(staged.push(&mut sink, &data[..2 * PACKET + 7])).is_ok());
        assert_eq!(sink.packets, 3);
        let after = counters();
        assert_eq!(after.0 - base.0, (2 * PACKET + 7) as u32);
        assert_eq!(after.1 - base.1, (2 * PACKET) as u32);
        assert_eq!(staged.len, 7);
        assert_reconciles(base, &staged);
        now(staged.flush(&mut sink));

        // A mid-push disconnect drops the accepted packet's successors and the failed packet.
        let base = counters();
        let mut sink = MockSink::up_to(1);
        assert!(now(staged.push(&mut sink, &data[..3 * PACKET])).is_err());
        assert_eq!(staged.len, 0);
        let after = counters();
        assert_eq!(after.0 - base.0, (3 * PACKET) as u32);
        assert_eq!(after.1 - base.1, PACKET as u32);
        assert_eq!(after.2 - base.2, (2 * PACKET) as u32);
        assert_reconciles(base, &staged);

        // Disconnect at every staged length: everything ever pushed lands in exactly one bin.
        for stage in 0..PACKET {
            let base = counters();
            let mut staged = Staged::new();
            let mut sink = MockSink::up_to(usize::MAX);
            assert!(now(staged.push(&mut sink, &data[..stage])).is_ok());
            let mut sink = MockSink::up_to(0);
            assert!(now(staged.push(&mut sink, &data[..PACKET])).is_err());
            assert_eq!(staged.len, 0);
            let after = counters();
            assert_eq!(after.1, base.1);
            assert_eq!(after.2 - base.2, (stage + PACKET) as u32);
            assert_reconciles(base, &staged);
        }

        // A flush the host refuses charges the stage as dropped, at every length.
        for stage in 1..PACKET {
            let base = counters();
            let mut staged = Staged::new();
            let mut sink = MockSink::up_to(usize::MAX);
            assert!(now(staged.push(&mut sink, &data[..stage])).is_ok());
            let mut sink = MockSink::up_to(0);
            now(staged.flush(&mut sink));
            assert_eq!(staged.len, 0);
            let after = counters();
            assert_eq!(after.1, base.1);
            assert_eq!(after.2 - base.2, stage as u32);
            assert_reconciles(base, &staged);
        }

        // The terminating zero-length packet moves no bytes through the ledger.
        let base = counters();
        let mut staged = Staged::new();
        let mut sink = MockSink::up_to(usize::MAX);
        assert!(now(staged.push(&mut sink, &data[..PACKET])).is_ok());
        assert!(staged.unterminated);
        now(staged.flush(&mut sink));
        assert!(!staged.unterminated);
        assert_eq!(sink.packets, 2);
        assert_eq!(sink.accepted_bytes, PACKET);
        let after = counters();
        assert_eq!(after.1 - base.1, PACKET as u32);
        assert_eq!(after.2, base.2);
        assert_reconciles(base, &staged);

        // A dead session's pipe residue is charged as taken-then-dropped, and the pipe empties.
        let base = counters();
        let pipe: Pipe<NoopRawMutex, RETURN_PIPE> = Pipe::new();
        now(pipe.write_all(&data[..17]));
        discard_stale(&pipe);
        assert_eq!(pipe.len(), 0);
        let after = counters();
        assert_eq!(after.0 - base.0, 17);
        assert_eq!(after.1, base.1);
        assert_eq!(after.2 - base.2, 17);
        // An empty pipe charges nothing.
        discard_stale(&pipe);
        assert_eq!(counters(), after);
    }

    /// The bridge's own four slots are always claimed, exact, and counted in bytes.
    #[test]
    fn the_query_composes_the_bridge_slots_over_the_seam_words() {
        let c = vcom_counters();
        assert_eq!(c.valid & BRIDGE_VALID, BRIDGE_VALID);
        for slot in [
            daplink_rs::VCOM_SLOT_UART_RX,
            daplink_rs::VCOM_SLOT_CDC_IN,
            daplink_rs::VCOM_SLOT_HOST_DROPPED,
            daplink_rs::VCOM_SLOT_CDC_OUT,
        ] {
            assert_eq!((c.grade >> (2 * slot)) & 3, daplink_rs::VCOM_GRADE_EXACT);
            assert_eq!(c.units & (1 << slot), 0);
        }
    }

    /// Drain to wire-idle, hand every unread old-rate byte to the pipe, then apply -- in order,
    /// with the query reporting the change in flight throughout.
    #[test]
    fn line_change_drains_wire_and_rx_before_applying() {
        let mut uart = MockUart::with_pending(DRAIN_BYTES + 44);
        let pipe: Pipe<NoopRawMutex, RETURN_PIPE> = Pipe::new();
        let mut scratch = [0u8; DRAIN_BYTES];
        now(apply_line_change(&mut uart, &pipe, &mut scratch, 250_000));
        assert_eq!(uart.applied_baud, 250_000);
        assert_eq!(uart.pending, 0);
        assert_eq!(pipe.len(), DRAIN_BYTES + 44);
        assert!(uart.drained_at.unwrap() < uart.last_read_at.unwrap());
        assert!(uart.last_read_at.unwrap() < uart.applied_at.unwrap());

        // 0x9E reports the transaction while it runs, so a host never reads the rate being left
        // as though it were settled.
        let during = uart.state_while_draining.unwrap();
        assert_ne!(during.flags & daplink_rs::VCOM_BAUD_APPLIED_VALID, 0);
        assert_eq!(during.flags & daplink_rs::VCOM_BAUD_SETTLED, 0);

        let settled = vcom_baud_state(249_999);
        assert_eq!(settled.realized, 249_999);
        assert_eq!(settled.applied, 250_000);
        assert_eq!(
            settled.flags,
            daplink_rs::VCOM_BAUD_APPLIED_VALID | daplink_rs::VCOM_BAUD_SETTLED
        );
    }
}
