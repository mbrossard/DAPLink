//! embassy-usb adapter: the CMSIS-DAP v2 bulk class and pipeline, the composite device, VCOM and MSC.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod msc;
pub mod trace;

#[cfg(test)]
mod descriptor_tests;
#[cfg(all(test, feature = "multi-packet"))]
mod multi_packet_tests;

#[cfg(test)]
mod vcom_bridge_tests;

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::join::{join, join3};
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex};
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender, State};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Config, Handler, UsbDevice};
use portable_atomic::AtomicU32;
use static_cell::{ConstStaticCell, StaticCell};

use daplink_rs::daplink_core::winusb::{
    BOS_CAPABILITY_PLATFORM, BOS_PLATFORM, MSOS_VENDOR_CODE, MS_OS_DESCRIPTOR,
};
use daplink_rs::daplink_core::{
    CommandExecutor, DapConfig, DapQueue, QueueError, RequestProducer, ResponseConsumer, Submit,
    CDC_PACKET_SIZE, DAP_V2_INTERFACE_CLASS, USB_MAX_POWER_MA,
};

/// CMSIS-DAP v2 hosts discover the probe by this interface string.
const INTERFACE_STRING: &str = "CMSIS-DAP v2 Interface";

/// Counts host sessions of the DAP v2 interface, bumped once per session boundary.
static SESSION_EPOCH: AtomicU32 = AtomicU32::new(0);
/// Wakes the one loop parked in [`session_ended`]; the epoch, not this edge, is the truth.
static BOUNDARY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The session counter, to be snapshotted once a session has started.
fn session_epoch() -> u32 {
    SESSION_EPOCH.load(Ordering::Relaxed)
}

/// Resolve once the session `epoch` was snapshotted under has ended, never missing it; one waiter.
async fn session_ended(epoch: u32) {
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
        (req.request == MSOS_VENDOR_CODE && req.index == 0x07)
            .then_some(InResponse::Accepted(&MS_OS_DESCRIPTOR))
    }
}

/// CMSIS-DAP v2 bulk interface; each transfer is exactly one command or response.
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
        alt.bos_capability(BOS_CAPABILITY_PLATFORM, &BOS_PLATFORM);
        let read_ep = alt.endpoint_bulk_out(None, max_packet_size);
        let write_ep = alt.endpoint_bulk_in(None, max_packet_size);
        drop(function);
        builder.handler(HANDLER.init(DapV2Handler { iface_string }));
        Self { read_ep, write_ep }
    }

    /// Take the endpoints apart, so each half can move into its own task.
    pub fn split(self) -> (DapV2Reader<D>, DapV2Writer<D>) {
        (
            DapV2Reader {
                read_ep: self.read_ep,
            },
            DapV2Writer {
                write_ep: self.write_ep,
            },
        )
    }
}

/// The command-reading half of a [`split`](DapV2Class::split) class.
pub struct DapV2Reader<D: Driver<'static>> {
    read_ep: D::EndpointOut,
}

/// The response-writing half of a [`split`](DapV2Class::split) class.
pub struct DapV2Writer<D: Driver<'static>> {
    write_ep: D::EndpointIn,
}

/// The OUT side the pipeline's reader future drives.
#[allow(async_fn_in_trait)]
pub trait DapReader {
    /// Wait for the host to enable the interface.
    async fn wait_connection(&mut self);
    /// Read one command; completion or cancellation must end all access to `buf`.
    async fn read_command(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError>;
}

/// The IN side the pipeline's writer future drives.
#[allow(async_fn_in_trait)]
pub trait DapWriter {
    /// Write one response to a host reading `P`-byte packets; cancellation ends access to `data`.
    async fn write_response<const P: usize>(&mut self, data: &[u8]) -> Result<(), EndpointError>;
}

impl<D: Driver<'static>> DapReader for DapV2Reader<D> {
    async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    #[cfg(not(feature = "multi-packet"))]
    async fn read_command(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(buf).await
    }

    // A command ends on a short packet, a full slot, or its parsed length; hosts send no ZLP.
    #[cfg(feature = "multi-packet")]
    async fn read_command(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let mps = usize::from(self.read_ep.info().max_packet_size);
        let mut total = 0;
        loop {
            let n = self.read_ep.read(&mut buf[total..]).await?;
            total += n;
            if n < mps || total >= buf.len() {
                return Ok(total);
            }
            if daplink_rs::daplink_core::dap_wire::command_len(&buf[..total])
                .is_some_and(|len| len <= total)
            {
                return Ok(total);
            }
        }
    }
}

impl<D: Driver<'static>> DapWriter for DapV2Writer<D> {
    #[cfg(not(feature = "multi-packet"))]
    async fn write_response<const P: usize>(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }

    // Packet-sized pieces, then a zero-length packet if the host's DAP-packet read is still open.
    #[cfg(feature = "multi-packet")]
    async fn write_response<const P: usize>(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let mps = usize::from(self.write_ep.info().max_packet_size);
        if data.is_empty() {
            return self.write_ep.write(data).await;
        }
        for piece in data.chunks(mps) {
            self.write_ep.write(piece).await?;
        }
        if needs_zlp(data.len(), mps, P) {
            self.write_ep.write(&[]).await?;
        }
        Ok(())
    }
}

/// A response that fills whole packets but not the host's DAP-packet buffer ends on a ZLP.
#[cfg(any(feature = "multi-packet", test))]
fn needs_zlp(len: usize, mps: usize, dap_packet: usize) -> bool {
    len.is_multiple_of(mps) && len < dap_packet
}

/// One endpoint task's session control: `start` releases, `stop` winds up, `parked` acknowledges.
struct HalfControl {
    start: Signal<CriticalSectionRawMutex, ()>,
    stop: Signal<CriticalSectionRawMutex, ()>,
    parked: Signal<CriticalSectionRawMutex, ()>,
}

impl HalfControl {
    const fn new() -> Self {
        Self {
            start: Signal::new(),
            stop: Signal::new(),
            parked: Signal::new(),
        }
    }

    /// Release only an initial or acknowledged-parked task.
    fn release(&self) {
        self.stop.reset();
        self.parked.reset();
        self.start.signal(());
    }
}

/// What the three pipeline futures hand each other; signals may cross interrupt priorities.
struct PipelineState {
    /// A request was queued, or the reader gave up.
    request_available: Signal<CriticalSectionRawMutex, ()>,
    /// The executor freed a request slot.
    request_slot_freed: Signal<CriticalSectionRawMutex, ()>,
    /// A response was posted, or the executor gave up.
    response_available: Signal<CriticalSectionRawMutex, ()>,
    /// The writer freed a response slot.
    response_slot_freed: Signal<CriticalSectionRawMutex, ()>,
    /// The host went away; each future returns once it is done draining.
    disconnected: AtomicBool,
    /// The reader task saw the interface enabled, so the session owner may start a session.
    connected: Signal<CriticalSectionRawMutex, ()>,
    reader: HalfControl,
    writer: HalfControl,
}

impl PipelineState {
    const fn new() -> Self {
        Self {
            request_available: Signal::new(),
            request_slot_freed: Signal::new(),
            response_available: Signal::new(),
            response_slot_freed: Signal::new(),
            disconnected: AtomicBool::new(false),
            connected: Signal::new(),
            reader: HalfControl::new(),
            writer: HalfControl::new(),
        }
    }

    fn disconnected(&self) -> bool {
        self.disconnected.load(Ordering::Acquire)
    }

    /// Record the loss and wake every parked future, whichever context it is parked in.
    fn disconnect(&self) {
        self.disconnected.store(true, Ordering::Release);
        self.request_available.signal(());
        self.request_slot_freed.signal(());
        self.response_available.signal(());
        self.response_slot_freed.signal(());
    }

    /// Clear the handover so a new session cannot inherit the last one's latched edges.
    fn clear(&self) {
        self.disconnected.store(false, Ordering::Release);
        self.request_available.reset();
        self.request_slot_freed.reset();
        self.response_available.reset();
        self.response_slot_freed.reset();
    }
}

/// The queue and handover state behind the DAP pipeline futures; `P` is the host's DAP packet size.
pub struct DapPipeline<const N: usize, const S: usize, const P: usize = S> {
    queue: DapQueue<N, S>,
    state: PipelineState,
}

impl<const N: usize, const S: usize, const P: usize> DapPipeline<N, S, P> {
    /// An empty pipeline; `const`, so it can back a `static` cell.
    pub const fn new() -> Self {
        const { assert!(P <= S, "DAP packet size exceeds the queue slot") };
        Self {
            queue: DapQueue::new(),
            state: PipelineState::new(),
        }
    }

    /// The queue itself, for the session owner that holds the pipeline exclusively.
    pub fn queue(&mut self) -> &mut DapQueue<N, S> {
        &mut self.queue
    }

    /// Drop the prior session's queue and bump its generation; the next `split` clears handover state.
    pub fn reset(&mut self) {
        self.queue.reset();
    }

    /// Unique handles for the endpoint tasks and their session controller, at independent priorities.
    pub fn split(
        &mut self,
    ) -> (
        DapReadHalf<'_, N, S>,
        DapExecHalf<'_, N, S>,
        DapWriteHalf<'_, N, S, P>,
    ) {
        self.state = PipelineState::new();
        let state = &self.state;
        let (requests, commands, responses) = self.queue.split();
        (
            DapReadHalf { requests, state },
            DapExecHalf {
                commands,
                state,
                session: SessionState::Initial,
            },
            DapWriteHalf { responses, state },
        )
    }
}

impl<const N: usize, const S: usize, const P: usize> Default for DapPipeline<N, S, P> {
    fn default() -> Self {
        Self::new()
    }
}

/// The reader's half: the OUT endpoint's producer end of the request ring.
pub struct DapReadHalf<'a, const N: usize, const S: usize> {
    requests: RequestProducer<'a, N, S>,
    state: &'a PipelineState,
}

/// The executor's half: it drains requests and posts responses, and it owns the session.
pub struct DapExecHalf<'a, const N: usize, const S: usize> {
    commands: CommandExecutor<'a, N, S>,
    state: &'a PipelineState,
    session: SessionState,
}

#[derive(Debug, PartialEq, Eq)]
enum SessionState {
    /// Tasks never released; no acknowledgement is owed.
    Initial,
    /// Both halves released.
    Running,
    /// Stop signalled; each flag records an acknowledged park.
    Stopping {
        reader_parked: bool,
        writer_parked: bool,
    },
    /// Both halves acknowledged; the queue may be reset.
    Parked,
}

/// The writer's half: the IN endpoint's consumer end of the response ring.
pub struct DapWriteHalf<'a, const N: usize, const S: usize, const P: usize = S> {
    responses: ResponseConsumer<'a, N, S>,
    state: &'a PipelineState,
}

impl<const N: usize, const S: usize> DapExecHalf<'_, N, S> {
    /// Finish any previous teardown, then wait for the reader to report the interface enabled.
    pub async fn wait_connection(&mut self) {
        self.end_session().await;
        self.state.connected.wait().await;
    }

    /// Stop any previous session, reset its queue, then release the endpoint tasks.
    pub async fn begin_session(&mut self) {
        self.end_session().await;
        // SAFETY: initial tasks cannot access slots; parked tasks acknowledged after dropping operations.
        unsafe { self.commands.reset() };
        self.state.clear();
        self.session = SessionState::Running;
        self.state.reader.release();
        self.state.writer.release();
    }

    /// Wait for both endpoint operations to end; cancellation preserves teardown progress.
    pub async fn end_session(&mut self) {
        match self.session {
            SessionState::Initial | SessionState::Parked => return,
            SessionState::Running => {
                self.session = SessionState::Stopping {
                    reader_parked: false,
                    writer_parked: false,
                };
                self.state.reader.stop.signal(());
                self.state.writer.stop.signal(());
            }
            SessionState::Stopping { .. } => {}
        }
        if let SessionState::Stopping {
            reader_parked,
            writer_parked,
        } = &mut self.session
        {
            if !*reader_parked {
                self.state.reader.parked.wait().await;
                *reader_parked = true;
            }
            if !*writer_parked {
                self.state.writer.parked.wait().await;
                *writer_parked = true;
            }
        }
        self.session = SessionState::Parked;
    }
}

/// Fill the request ring from the OUT endpoint; reads land straight in the ring slot.
async fn dap_reader<R, const N: usize, const S: usize>(
    half: &mut DapReadHalf<'_, N, S>,
    reader: &mut R,
) where
    R: DapReader,
{
    loop {
        let len = loop {
            let Some(slot) = half.requests.staging() else {
                if half.state.disconnected() {
                    return;
                }
                let aux = trace::aux_ready(|| half.state.request_slot_freed.signaled());
                trace::mark_aux(trace::EV_WAIT_SLOT, aux, 0);
                half.state.request_slot_freed.wait().await;
                trace::mark(trace::EV_RESUME_SLOT, 0);
                continue;
            };
            match reader.read_command(slot).await {
                Ok(len) => break len,
                Err(_) => {
                    trace::mark(trace::EV_READ_ERR, 0);
                    half.state.disconnect();
                    return;
                }
            }
        };
        trace::mark(trace::EV_READ_RET, len as u16);
        match half.requests.commit(len) {
            Ok(Submit::Enqueued) => {
                let aux = trace::aux_full(trace::next_submit_ordinal(), || {
                    half.requests.requests_full()
                });
                trace::mark_aux(trace::EV_SUBMIT_ENQ, aux, len as u16);
                half.state.request_available.signal(());
            }
            Ok(Submit::AbortIntercepted | Submit::Dropped) => {
                trace::mark(trace::EV_SUBMIT_DROP, len as u16)
            }
            Err(_) => panic!("DAP command exceeds queue slot"),
        }
    }
}

/// Execute the oldest queued command in place and publish its response; the wire runs here.
pub async fn dap_executor<E, const N: usize, const S: usize>(
    half: &mut DapExecHalf<'_, N, S>,
    mut execute: E,
) where
    E: FnMut(&[u8], &mut [u8]) -> usize,
{
    loop {
        let outcome = half.commands.execute_next_with(|request, response| {
            let cmd = request.first().copied().unwrap_or(0);
            let ordinal = trace::next_exec_ordinal();
            trace::mark_aux(trace::EV_EXEC_ENTER, ordinal, cmd as u16);
            let len = execute(request, response);
            trace::mark_aux(trace::EV_EXEC_EXIT, ordinal, len as u16);
            len
        });
        match outcome {
            Ok(Some(_)) => {
                half.state.request_slot_freed.signal(());
                half.state.response_available.signal(());
            }
            Ok(None) => {
                if half.state.disconnected() {
                    return;
                }
                trace::mark(trace::EV_EXEC_IDLE, 0);
                let aux = trace::aux_ready(|| half.state.request_available.signaled());
                trace::mark_aux(trace::EV_WAIT_REQ, aux, 0);
                half.state.request_available.wait().await;
                trace::mark(trace::EV_RESUME_REQ, 0);
            }
            Err(QueueError::Full) => {
                if half.state.disconnected() {
                    return;
                }
                let aux = trace::aux_ready(|| half.state.response_slot_freed.signaled());
                trace::mark_aux(trace::EV_WAIT_RESP_SLOT, aux, 0);
                half.state.response_slot_freed.wait().await;
                trace::mark(trace::EV_RESUME_RESP_SLOT, 0);
            }
            Err(QueueError::Oversized) => panic!("DAP response exceeds queue slot"),
        }
    }
}

/// Drain the response ring into the IN endpoint, so the executor never waits on a previous packet.
async fn dap_writer<W, const N: usize, const S: usize, const P: usize>(
    half: &mut DapWriteHalf<'_, N, S, P>,
    writer: &mut W,
) where
    W: DapWriter,
{
    let mut response = [0u8; S];
    loop {
        let generation = half.responses.generation();
        let Some(len) = half.responses.peek_response(&mut response) else {
            if half.state.disconnected() {
                return;
            }
            half.state.response_available.wait().await;
            continue;
        };
        let ordinal = trace::aux_full(trace::next_stage_ordinal(), || {
            half.responses.responses_full()
        });
        trace::mark_aux(trace::EV_WRITE_ENTER, ordinal, len as u16);
        if writer.write_response::<P>(&response[..len]).await.is_err() {
            trace::mark(trace::EV_WRITE_ERR, 0);
            half.state.disconnect();
            return;
        }
        trace::mark_aux(trace::EV_WRITE_RET, ordinal, len as u16);
        // A reset since the peek retires nothing: that response belonged to a dead session.
        if half.responses.discard_response_for(generation) {
            half.state.response_slot_freed.signal(());
        }
    }
}

/// Serve OUT sessions under the controller; dropping this task can leave teardown waiting forever.
pub async fn dap_reader_task<R, const N: usize, const S: usize>(
    mut half: DapReadHalf<'static, N, S>,
    mut reader: R,
) -> !
where
    R: DapReader,
{
    let state = half.state;
    loop {
        reader.wait_connection().await;
        state.connected.signal(());
        state.reader.start.wait().await;
        select(state.reader.stop.wait(), dap_reader(&mut half, &mut reader)).await;
        // The select and its endpoint future are dropped before publishing quiescence.
        state.reader.parked.signal(());
    }
}

/// Serve IN sessions under the controller; dropping this task can leave teardown waiting forever.
pub async fn dap_writer_task<W, const N: usize, const S: usize, const P: usize>(
    mut half: DapWriteHalf<'static, N, S, P>,
    mut writer: W,
) -> !
where
    W: DapWriter,
{
    let state = half.state;
    loop {
        state.writer.start.wait().await;
        select(state.writer.stop.wait(), dap_writer(&mut half, &mut writer)).await;
        // The select and its endpoint future are dropped before publishing quiescence.
        state.writer.parked.signal(());
    }
}

/// All three futures as one task; the caller has already reset the pipeline for this session.
pub async fn run_dap_pipeline<const N: usize, const S: usize, const P: usize, R, W, E>(
    reader: &mut R,
    writer: &mut W,
    pipeline: &mut DapPipeline<N, S, P>,
    execute: E,
) where
    R: DapReader,
    W: DapWriter,
    E: FnMut(&[u8], &mut [u8]) -> usize,
{
    let (mut read_half, mut exec_half, mut write_half) = pipeline.split();
    exec_half.state.clear();
    join3(
        dap_reader(&mut read_half, reader),
        dap_executor(&mut exec_half, execute),
        dap_writer(&mut write_half, writer),
    )
    .await;
}

/// One session composition: what a shape does at each session boundary, and what it runs between.
#[allow(async_fn_in_trait)]
pub trait DapSession {
    /// Wait until the composition reports the interface enabled.
    async fn wait_connection(&mut self);

    /// Finish teardown and reset before starting; cancellation must preserve teardown progress.
    async fn begin_session(&mut self);

    /// Serve the session until the composition's own futures finish.
    async fn run<E: FnMut(&[u8], &mut [u8]) -> usize>(&mut self, execute: E);

    /// Wind the session down so the next [`begin_session`](Self::begin_session) has the queue.
    async fn end_session(&mut self);
}

/// The cooperative composition as a session: both endpoint halves and the queue in one task.
pub struct PipelineSession<'a, R, W, const N: usize, const S: usize, const P: usize = S> {
    reader: R,
    writer: W,
    pipeline: &'a mut DapPipeline<N, S, P>,
}

impl<'a, R, W, const N: usize, const S: usize, const P: usize> PipelineSession<'a, R, W, N, S, P> {
    /// Take the halves of a split class and the queue they meet on.
    pub fn new(reader: R, writer: W, pipeline: &'a mut DapPipeline<N, S, P>) -> Self {
        Self {
            reader,
            writer,
            pipeline,
        }
    }
}

impl<R: DapReader, W: DapWriter, const N: usize, const S: usize, const P: usize> DapSession
    for PipelineSession<'_, R, W, N, S, P>
{
    async fn wait_connection(&mut self) {
        self.reader.wait_connection().await;
    }

    async fn begin_session(&mut self) {
        self.pipeline.reset();
    }

    async fn run<E: FnMut(&[u8], &mut [u8]) -> usize>(&mut self, execute: E) {
        run_dap_pipeline(&mut self.reader, &mut self.writer, self.pipeline, execute).await;
    }

    /// Nothing outlives the session: dropping the three futures is the teardown.
    async fn end_session(&mut self) {}
}

impl<const N: usize, const S: usize> DapSession for DapExecHalf<'_, N, S> {
    async fn wait_connection(&mut self) {
        DapExecHalf::wait_connection(self).await;
    }

    async fn begin_session(&mut self) {
        DapExecHalf::begin_session(self).await;
    }

    async fn run<E: FnMut(&[u8], &mut [u8]) -> usize>(&mut self, execute: E) {
        dap_executor(self, execute).await;
    }

    async fn end_session(&mut self) {
        DapExecHalf::end_session(self).await;
    }
}

/// Serve DAP sessions around a board engine; `COUNT` tallies the commands `finished` reports.
pub async fn dap_sessions<const COUNT: bool, T, C, E, S, F>(
    session: &mut T,
    context: &mut C,
    mut execute: E,
    mut suspend: S,
    mut finished: F,
) -> !
where
    T: DapSession,
    E: FnMut(&mut C, &[u8], &mut [u8]) -> usize,
    S: FnMut(&mut C),
    F: FnMut(&mut C, u32),
{
    loop {
        session.wait_connection().await;
        // Snapshot after the connection is up, so this session's own boundary cannot end it.
        let epoch = session_epoch();
        session.begin_session().await;
        let processed = Cell::new(0u32);

        // The boundary must be the first arm, so a write parked on a dead session is dropped.
        select(
            session_ended(epoch),
            session.run(|request, response| {
                let len = execute(context, request, response);
                if COUNT {
                    processed.set(processed.get() + 1);
                }
                len
            }),
        )
        .await;
        // Whatever the composition parked elsewhere is wound up before the next session.
        session.end_session().await;

        finished(context, processed.get());
        suspend(context);
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
        let $class = DapV2Class::new(&mut $builder, $cfg.max_packet_size.get());
        static LINE_EVENTS: StaticCell<LineEvents> = StaticCell::new();
        // Ahead of the CDC class, whose handler answers its requests and ends the walk.
        $builder.handler(LINE_EVENTS.init(LineEvents { dtr: false }));
        static CDC_STATE: StaticCell<State> = StaticCell::new();
        let cdc = CdcAcmClass::new(&mut $builder, CDC_STATE.init(State::new()), CDC_PACKET_SIZE);
        let ($tx, $rx, $ctl) = cdc.split_with_control();
    };
}

/// Build the composite USB device: the CMSIS-DAP v2 bulk interface plus a CDC-ACM VCOM function.
/// One `usb_device*` call per image: a second panics on the static class storage.
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

/// Build the DAP-only device, for a board with no VCOM: one interface, no IAD.
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
    config.composite_with_iads = false;
    config.device_class = 0x00;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;

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
    let class = DapV2Class::new(&mut builder, cfg.max_packet_size.get());
    (builder.build(), class)
}

/// [`usb_device`] with mass storage appended on interface 3; `EP` is its bulk packet size.
/// One `usb_device*` call per image: a second panics on the static class storage.
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

/// The counters a seam sources itself; slots outside `SEAM_VALID` stay zero.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct SeamStats {
    /// Bytes dropped because the seam's receive ring was full.
    pub ring_full: u32,
    /// Error-flagged reception, in the unit the seam's `SEAM_UNITS` declares.
    pub frame_err: u32,
    /// Receive overrun, in the unit the seam's `SEAM_UNITS` declares.
    pub overrun: u32,
    /// Transmit credit in the seam's declared evidence grade; not universally wire-idle completion.
    pub uart_tx: u32,
    /// Receive-DMA faults the seam recovered from.
    pub rx_dma_err: u32,
}

/// Whether a UART accepted a requested nominal rate, independently of its realized divisor rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use = "only accepted requests may update the nominal baud cache"]
pub enum BaudApply {
    /// The nominal request was accepted; divisor rounding or clamping may still apply.
    Applied,
    /// The nominal request was rejected; peripheral restoration may still have been necessary.
    Rejected,
}

/// Hardware UART seam behind [`vcom_bridge`]; a platform crate wraps its HAL UART in this.
// Single-executor firmware: the futures never cross threads, so no `Send` bound is wanted.
#[allow(async_fn_in_trait)]
pub trait VcomUart {
    /// Which [`SeamStats`] counters this seam observes, as `VCOM_VALID_*` bits.
    const SEAM_VALID: u32;
    /// Evidence quality per sourced slot, as `vcom_grade` bits; never overstate one.
    const SEAM_GRADE: u32;
    /// `vcom_episodes` bits for the slots this seam counts in episodes rather than bytes.
    const SEAM_UNITS: u32;

    /// Read into `buf`, awaiting a byte; only a poll returning `Ready` may consume bytes.
    async fn read(&mut self, buf: &mut [u8]) -> usize;
    /// Take what the seam holds without waiting; a busy host starves the return leg otherwise.
    fn read_ready(&mut self, buf: &mut [u8]) -> usize;
    /// The seam's counter ledger, covering exactly the slots [`Self::SEAM_VALID`] names.
    fn seam_stats(&self) -> SeamStats;
    /// Accept the whole buffer for transmission; use `drain_tx` for wire-idle completion.
    async fn write_all(&mut self, data: &[u8]);
    /// Resolve once every accepted TX byte has left the wire, the shift register included.
    async fn drain_tx(&mut self);
    /// Stop filling the receive ring, leaving what it holds for `read_ready`.
    fn halt_rx(&mut self);
    /// After `halt_rx` and TX drain: apply nominal `baud` and restart receive; zero is rejected.
    fn set_baudrate(&mut self, baud: u32) -> BaudApply;
}

/// Cap on one return-leg drain pass, so it stays bounded rather than exhaustive.
const DRAIN_BYTES: usize = 4 * CDC_PACKET_SIZE as usize;

/// Handover capacity; a full pipe backpressures the UART half.
const RETURN_PIPE: usize = 512;

/// How long the UART half waits before looking again when the handover pipe is full.
const PIPE_FULL_POLL: Duration = Duration::from_micros(200);

/// Most old-rate bytes a line change hands on: about a seam ring plus the pipe.
const LINE_CHANGE_CAP: usize = 2048 + RETURN_PIPE;

/// How long a line change waits on a full pipe before charging the backlog instead.
const LINE_CHANGE_STALL: Duration = Duration::from_millis(50);

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
/// Last nominal rate accepted by the seam, not necessarily its realized divisor rate.
static APPLIED_BAUD: AtomicU32 = AtomicU32::new(daplink_rs::daplink_core::DEFAULT_VCOM_BAUD);
/// Requested nominal awaiting acceptance or rejection; zero when no change is in flight.
static PENDING_BAUD: AtomicU32 = AtomicU32::new(0);

/// Falling DTR edges, counted as the request arrives so none can hide inside one bridge await.
static DTR_FALLS: AtomicU32 = AtomicU32::new(0);

/// Bus resets; a line change held across one belongs to a session that no longer exists.
static BUS_RESETS: AtomicU32 = AtomicU32::new(0);

/// The class's line coding came from the host, not from its 8000-baud reset default.
static HOST_CODING: AtomicBool = AtomicBool::new(false);
/// The rate of the last host coding, taken even when the rest of the record was refused.
static HOST_BAUD: AtomicU32 = AtomicU32::new(0);
/// A refused coding never reaches the class, so its wake comes from here instead.
static LINE_REFUSED: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
static LINE_WAKER: AtomicWaker = AtomicWaker::new();

/// No other function on these devices has a class request with any of these codes.
const SET_LINE_CODING: u8 = 0x20;
const GET_LINE_CODING: u8 = 0x21;
const SET_CONTROL_LINE_STATE: u8 = 0x22;

/// Watches the CDC class's control requests and declines each, so the class still answers it.
struct LineEvents {
    dtr: bool,
}

impl LineEvents {
    fn set_dtr(&mut self, dtr: bool) {
        if self.dtr && !dtr {
            DTR_FALLS.fetch_add(1, Ordering::Relaxed);
        }
        self.dtr = dtr;
    }
}

impl Handler for LineEvents {
    fn reset(&mut self) {
        self.set_dtr(false);
        BUS_RESETS.fetch_add(1, Ordering::Relaxed);
        HOST_CODING.store(false, Ordering::Relaxed);
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return None;
        }
        match req.request {
            SET_CONTROL_LINE_STATE => self.set_dtr(req.value & 1 != 0),
            // The class ignores a short coding, so it stays as it was.
            SET_LINE_CODING if data.len() >= 7 => {
                let baud = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                HOST_BAUD.store(baud, Ordering::Relaxed);
                HOST_CODING.store(true, Ordering::Relaxed);
                // The rate is followed whatever the framing; only 8N1 runs, so the rest is refused.
                if data[4..7] != [0, 0, 8] {
                    LINE_REFUSED.store(true, Ordering::Relaxed);
                    LINE_WAKER.wake();
                    return Some(OutResponse::Rejected);
                }
            }
            _ => {}
        }
        None
    }

    /// Answers with what runs: the followed rate at 8N1, whatever the host last asked.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Class
            || req.recipient != Recipient::Interface
            || req.request != GET_LINE_CODING
            || req.length != 7
            || !HOST_CODING.load(Ordering::Relaxed)
        {
            return None;
        }
        buf[0..4].copy_from_slice(&HOST_BAUD.load(Ordering::Relaxed).to_le_bytes());
        buf[4..7].copy_from_slice(&[0, 0, 8]);
        Some(InResponse::Accepted(&buf[..7]))
    }
}

/// A line event from either channel: the class saw a request, or the handler refused one.
async fn line_event(control: &ControlChanged<'static>) {
    let refused = core::future::poll_fn(|cx| {
        LINE_WAKER.register(cx.waker());
        if LINE_REFUSED.swap(false, Ordering::Relaxed) {
            core::task::Poll::Ready(())
        } else {
            core::task::Poll::Pending
        }
    });
    select(control.control_changed(), refused).await;
}

struct PendingBaud;

impl PendingBaud {
    fn new(baud: u32) -> Self {
        PENDING_BAUD.store(baud, Ordering::Release);
        Self
    }
}

impl Drop for PendingBaud {
    fn drop(&mut self) {
        PENDING_BAUD.store(0, Ordering::Release);
    }
}

/// Coverage, grade and unit words the running seam published.
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

/// Combine the accepted nominal and the platform's `realized` rate for the baud query.
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
    Control,
    /// The handover pipe was full, so the receive arm was not armed at all this pass.
    PipeFull,
}

/// Bridge the CDC-ACM VCOM port to the target UART; `led` pulses once per packet moved.
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
    let closed: Signal<NoopRawMutex, ()> = Signal::new();
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
        let mut falls = DTR_FALLS.load(Ordering::Relaxed);
        let mut coding_changed = false;
        loop {
            // A falling DTR is the host closing the port: its leftovers are not the next opener's.
            if dtr_fell(&mut falls) {
                close_session(uart, &pipe, &mut from_uart, &closed);
            }
            if core::mem::take(&mut coding_changed) && HOST_CODING.load(Ordering::Relaxed) {
                let baud = HOST_BAUD.load(Ordering::Relaxed);
                let still_open = || DTR_FALLS.load(Ordering::Relaxed) == falls;
                apply_line_change(
                    uart,
                    &pipe,
                    &mut from_uart,
                    baud,
                    &mut applied_baud,
                    still_open,
                )
                .await;
                continue;
            }
            // Sized to the room the pipe has now, so staging can never block: this is its only writer.
            let room = pipe.free_capacity().min(from_uart.len());
            // The block ends the select's borrow of the UART, so the handling below reaches the seam.
            let event = if room == 0 {
                match select3(
                    cdc_rx.read_packet(&mut from_host),
                    line_event(control),
                    Timer::after(PIPE_FULL_POLL),
                )
                .await
                {
                    Either3::First(Ok(n)) => VcomEvent::FromHost(n),
                    Either3::First(Err(_)) => VcomEvent::HostGone,
                    Either3::Second(()) => VcomEvent::Control,
                    Either3::Third(()) => VcomEvent::PipeFull,
                }
            } else {
                match select3(
                    cdc_rx.read_packet(&mut from_host),
                    uart.read(&mut from_uart[..room]),
                    line_event(control),
                )
                .await
                {
                    Either3::First(Ok(n)) => VcomEvent::FromHost(n),
                    Either3::First(Err(_)) => VcomEvent::HostGone,
                    Either3::Second(n) => VcomEvent::FromUart(n),
                    Either3::Third(()) => VcomEvent::Control,
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
                // Host gone: the pipe is stale either way; a close latched before it ends the session.
                VcomEvent::HostGone => {
                    if dtr_fell(&mut falls) {
                        close_session(uart, &pipe, &mut from_uart, &closed);
                    } else {
                        discard_stale(&pipe);
                    }
                    cdc_rx.wait_connection().await;
                }
                VcomEvent::FromUart(n) if n > 0 => pipe.write_all(&from_uart[..n]).await,
                // A zero-length ready read is the seam absorbing a UART error; nothing to forward.
                VcomEvent::FromUart(_) => {}
                // No read was armed, so observe the cursor by hand or a lapped ring goes uncharged.
                VcomEvent::PipeFull => {
                    uart.read_ready(&mut []);
                }
                // Handled at the top, after the DTR check, so a close never waits out a stall.
                VcomEvent::Control => coding_changed = true,
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
            let step = async {
                match select(pipe.read(&mut from_pipe), Timer::after(wait)).await {
                    Either::First(n) => {
                        if staged.push(cdc_tx, &from_pipe[..n]).await.is_err() {
                            cdc_tx.wait_connection().await;
                        }
                        led.borrow_mut().pulse();
                    }
                    Either::Second(()) => staged.flush(cdc_tx).await,
                }
            };
            // A closed session's stage and chunk in hand are dropped; an armed IN packet cannot be.
            if let Either::First(()) = select(closed.wait(), step).await {
                staged.abandon();
            }
        }
    };

    // Two halves, not one loop: neither leg's progress may wait on the other's.
    let (never, _) = join(uart_half, host_half).await;
    never
}

/// Whether DTR fell since `seen` was taken, which this advances.
fn dtr_fell(seen: &mut u32) -> bool {
    let now = DTR_FALLS.load(Ordering::Relaxed);
    core::mem::replace(seen, now) != now
}

/// Hand old-rate RX to the pipe as it has room, charge what the bounds leave, then reconfigure.
async fn apply_line_change<U: VcomUart, M: RawMutex, const N: usize>(
    uart: &mut U,
    pipe: &Pipe<M, N>,
    scratch: &mut [u8],
    baud: u32,
    applied_baud: &mut u32,
    still_open: impl Fn() -> bool,
) {
    if baud == 0 || baud == *applied_baud {
        return;
    }
    let _pending = PendingBaud::new(baud);
    let resets = BUS_RESETS.load(Ordering::Relaxed);
    let reset = || BUS_RESETS.load(Ordering::Relaxed) != resets;
    uart.drain_tx().await;
    let mut moved = 0;
    let mut since = Instant::now();
    // Bounded: neither a stalled reader nor a source outpacing it may hold the new rate off.
    loop {
        let (n, room) = forward_ready(uart, pipe, scratch);
        if room > 0 {
            moved += n;
            since = Instant::now();
            if n < room || moved >= LINE_CHANGE_CAP {
                break;
            }
        } else if since.elapsed() >= LINE_CHANGE_STALL || !still_open() || reset() {
            break;
        } else {
            Timer::after(PIPE_FULL_POLL).await;
        }
    }
    // The next session sends its own coding; the receive ring was never halted.
    if reset() {
        return;
    }
    // No await from here on: a halted ring must reach `set_baudrate`, which restarts it.
    uart.halt_rx();
    loop {
        let (n, room) = forward_ready(uart, pipe, scratch);
        if room == 0 {
            drain_dropped(uart, scratch);
            break;
        }
        if n < room {
            break;
        }
    }
    if uart.set_baudrate(baud) == BaudApply::Applied {
        APPLIED_BAUD.store(baud, Ordering::Relaxed);
        *applied_baud = baud;
    }
}

/// Move what the seam holds into the pipe's free room, synchronously; returns (moved, room).
fn forward_ready<U: VcomUart, M: RawMutex, const N: usize>(
    uart: &mut U,
    pipe: &Pipe<M, N>,
    scratch: &mut [u8],
) -> (usize, usize) {
    let room = pipe.free_capacity().min(scratch.len());
    let n = uart.read_ready(&mut scratch[..room]);
    if n > 0 {
        // Fits whole: `room` was measured on a pipe this caller alone writes.
        let _ = pipe.try_write(&scratch[..n]);
    }
    (n, room)
}

/// End a session: drop the pipe, the seam's backlog and the host half's stage, charged.
fn close_session<U: VcomUart, M: RawMutex, const N: usize>(
    uart: &mut U,
    pipe: &Pipe<M, N>,
    scratch: &mut [u8],
    closed: &Signal<M, ()>,
) {
    discard_stale(pipe);
    drain_dropped(uart, scratch);
    closed.signal(());
}

/// Take everything the seam holds now, charged as taken-then-dropped.
fn drain_dropped<U: VcomUart>(uart: &mut U, scratch: &mut [u8]) {
    loop {
        let n = uart.read_ready(scratch);
        charge_dropped(n);
        if n < scratch.len() {
            break;
        }
    }
}

/// Drop what the pipe still holds, charged as taken-then-dropped.
fn discard_stale<M: RawMutex, const N: usize>(pipe: &Pipe<M, N>) {
    charge_dropped(pipe.len());
    pipe.clear();
}

fn charge_dropped(n: usize) {
    if n > 0 {
        UART_RX.fetch_add(n as u32, Ordering::Relaxed);
        HOST_DROPPED.fetch_add(n as u32, Ordering::Relaxed);
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
    /// Counted into `UART_RX` and not yet out: the stage, plus the chunk in hand during a push.
    owed: usize,
    /// Whether a full-size packet left the host's transfer open.
    unterminated: bool,
}

impl Staged {
    const fn new() -> Self {
        Self {
            buf: [0; CDC_PACKET_SIZE as usize],
            len: 0,
            owed: 0,
            unterminated: false,
        }
    }

    /// Nothing owed to the host, so the deadline arm can be parked.
    fn is_idle(&self) -> bool {
        self.len == 0 && !self.unterminated
    }

    /// Take return-leg bytes, sending each whole packet; `Err` means the host left mid-write.
    async fn push<S: PacketSink>(&mut self, cdc_tx: &mut S, data: &[u8]) -> Result<(), ()> {
        if data.is_empty() {
            return Ok(());
        }
        UART_RX.fetch_add(data.len() as u32, Ordering::Relaxed);
        self.owed = self.len + data.len();
        let packet = CDC_PACKET_SIZE as usize;
        let mut rest = data;
        while self.len + rest.len() >= packet {
            let take = packet - self.len;
            self.buf[self.len..packet].copy_from_slice(&rest[..take]);
            if cdc_tx.send(&self.buf[..packet]).await.is_err() {
                // The failed packet, its staged bytes included, and the rest of `data` die here.
                HOST_DROPPED.fetch_add(self.owed as u32, Ordering::Relaxed);
                self.owed = 0;
                self.len = 0;
                self.unterminated = false;
                return Err(());
            }
            CDC_IN.fetch_add(packet as u32, Ordering::Relaxed);
            self.owed -= packet;
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

    /// Release a part packet, or the zero-length packet closing an open transfer.
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
            self.owed = 0;
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

    /// Drop the stage and any chunk a cancelled push held, charged as dropped.
    fn abandon(&mut self) {
        HOST_DROPPED.fetch_add(self.owed as u32, Ordering::Relaxed);
        self.owed = 0;
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::{pin, Pin};
    use core::task::{Context, Poll, Waker};
    use std::boxed::Box;
    use std::collections::VecDeque;
    use std::rc::Rc;
    use std::vec;
    use std::vec::Vec;

    const PACKET: usize = CDC_PACKET_SIZE as usize;

    #[derive(Default)]
    struct ReaderState {
        packets: VecDeque<Vec<u8>>,
        stalled: bool,
        disabled: bool,
        connection_stalled: bool,
        active: Rc<Cell<usize>>,
    }

    struct MockReader(Rc<RefCell<ReaderState>>);

    impl DapReader for MockReader {
        async fn wait_connection(&mut self) {
            core::future::poll_fn(|_| {
                if self.0.borrow().connection_stalled {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
        }

        async fn read_command(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
            let _active = ActiveOperation::new(self.0.borrow().active.clone());
            core::future::poll_fn(|_| {
                let mut state = self.0.borrow_mut();
                if state.stalled {
                    return Poll::Pending;
                }
                if let Some(packet) = state.packets.pop_front() {
                    let copied = packet.len().min(buf.len());
                    buf[..copied].copy_from_slice(&packet[..copied]);
                    return Poll::Ready(Ok(packet.len()));
                }
                if state.disabled {
                    Poll::Ready(Err(EndpointError::Disabled))
                } else {
                    Poll::Pending
                }
            })
            .await
        }
    }

    #[derive(Default)]
    struct WriterState {
        sent: Vec<Vec<u8>>,
        /// Writes refused while stalled: the only sign, from outside, that a response is staged.
        refused: usize,
        stalled: bool,
        disabled: bool,
        active: Rc<Cell<usize>>,
    }

    struct MockWriter(Rc<RefCell<WriterState>>);

    impl DapWriter for MockWriter {
        async fn write_response<const P: usize>(
            &mut self,
            data: &[u8],
        ) -> Result<(), EndpointError> {
            let _active = ActiveOperation::new(self.0.borrow().active.clone());
            core::future::poll_fn(|_| {
                let mut state = self.0.borrow_mut();
                if state.stalled {
                    state.refused += 1;
                    return Poll::Pending;
                }
                if state.disabled {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                state.sent.push(data.to_vec());
                Poll::Ready(Ok(()))
            })
            .await
        }
    }

    struct ActiveOperation(Rc<Cell<usize>>);

    impl ActiveOperation {
        fn new(active: Rc<Cell<usize>>) -> Self {
            active.set(active.get() + 1);
            Self(active)
        }
    }

    impl Drop for ActiveOperation {
        fn drop(&mut self) {
            self.0.set(self.0.get() - 1);
        }
    }

    fn poll_once<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
        future.poll(&mut Context::from_waker(Waker::noop()))
    }

    fn completes(future: impl Future<Output = ()>) {
        assert!(poll_once(pin!(future)).is_ready());
    }

    fn mock_reader(packets: &[&[u8]]) -> (MockReader, Rc<RefCell<ReaderState>>) {
        let state = Rc::new(RefCell::new(ReaderState {
            packets: packets.iter().map(|packet| packet.to_vec()).collect(),
            ..ReaderState::default()
        }));
        (MockReader(state.clone()), state)
    }

    fn mock_writer() -> (MockWriter, Rc<RefCell<WriterState>>) {
        let state = Rc::new(RefCell::new(WriterState::default()));
        (MockWriter(state.clone()), state)
    }

    fn echo(request: &[u8], response: &mut [u8]) -> usize {
        response[..request.len()].copy_from_slice(request);
        request.len()
    }

    fn drive<F: Future>(mut future: Pin<&mut F>, done: impl Fn() -> bool) {
        let mut context = Context::from_waker(Waker::noop());
        for _ in 0..1_000 {
            if done() {
                return;
            }
            assert!(future.as_mut().poll(&mut context).is_pending());
        }
        panic!("pipeline did not reach the expected state");
    }

    #[test]
    fn a_zlp_follows_only_whole_packet_responses_shorter_than_the_dap_packet() {
        assert!(needs_zlp(64, 64, 512));
        assert!(needs_zlp(448, 64, 512));
        assert!(!needs_zlp(512, 64, 512));
        assert!(!needs_zlp(65, 64, 512));
        assert!(!needs_zlp(64, 64, 64));
    }

    #[test]
    fn read_stall_does_not_block_queued_execution() {
        let (mut reader, reader_state) = mock_reader(&[]);
        reader_state.borrow_mut().stalled = true;
        let (mut writer, writer_state) = mock_writer();
        let mut pipeline: DapPipeline<1, 64> = DapPipeline::new();
        pipeline.queue().submit(&[0x11]).expect("queue has room");

        let mut future = pin!(run_dap_pipeline(
            &mut reader,
            &mut writer,
            &mut pipeline,
            echo
        ));
        drive(future.as_mut(), || !writer_state.borrow().sent.is_empty());

        assert_eq!(writer_state.borrow().sent, vec![vec![0x11]]);
    }

    #[test]
    fn write_stall_backpressures_without_dropping_requests() {
        let (mut reader, reader_state) = mock_reader(&[&[0x21], &[0x22], &[0x23]]);
        let (mut writer, writer_state) = mock_writer();
        writer_state.borrow_mut().stalled = true;
        let mut pipeline: DapPipeline<2, 64> = DapPipeline::new();

        let mut future = pin!(run_dap_pipeline(
            &mut reader,
            &mut writer,
            &mut pipeline,
            echo
        ));
        // Both rings fill behind the stalled writer, and the reader empties the endpoint anyway.
        drive(future.as_mut(), || {
            reader_state.borrow().packets.is_empty() && writer_state.borrow().refused > 0
        });

        assert!(writer_state.borrow().sent.is_empty());
    }

    #[test]
    fn full_queue_preserves_response_order() {
        let (mut reader, _) = mock_reader(&[&[0x31], &[0x32], &[0x33]]);
        let (mut writer, writer_state) = mock_writer();
        let mut pipeline: DapPipeline<1, 64> = DapPipeline::new();

        let mut future = pin!(run_dap_pipeline(
            &mut reader,
            &mut writer,
            &mut pipeline,
            echo
        ));
        drive(future.as_mut(), || writer_state.borrow().sent.len() == 3);

        assert_eq!(
            writer_state.borrow().sent,
            vec![vec![0x31], vec![0x32], vec![0x33]]
        );
    }

    /// The pipeline drains `2 * N` packets in FIFO order at each tested queue depth.
    fn pipeline_preserves_order_at_depth<const N: usize>() {
        let packets: Vec<Vec<u8>> = (0..2 * N).map(|i| vec![0x50 + i as u8]).collect();
        let refs: Vec<&[u8]> = packets.iter().map(Vec::as_slice).collect();
        let (mut reader, _) = mock_reader(&refs);
        let (mut writer, writer_state) = mock_writer();
        let mut pipeline: DapPipeline<N, 64> = DapPipeline::new();

        let mut future = pin!(run_dap_pipeline(
            &mut reader,
            &mut writer,
            &mut pipeline,
            echo
        ));
        drive(future.as_mut(), || {
            writer_state.borrow().sent.len() == packets.len()
        });

        assert_eq!(writer_state.borrow().sent, packets);
    }

    #[test]
    fn pipeline_order_at_depth_four() {
        pipeline_preserves_order_at_depth::<4>();
    }

    #[test]
    fn pipeline_order_at_depth_eight() {
        pipeline_preserves_order_at_depth::<8>();
    }

    #[test]
    fn transfer_abort_is_consumed_without_a_response() {
        let (mut reader, _) = mock_reader(&[&[0x41], &[0x07], &[0x42]]);
        let (mut writer, writer_state) = mock_writer();
        // Depth one: the abort would deadlock the reader if it took a request slot.
        let mut pipeline: DapPipeline<1, 64> = DapPipeline::new();

        {
            let mut future = pin!(run_dap_pipeline(
                &mut reader,
                &mut writer,
                &mut pipeline,
                echo
            ));
            drive(future.as_mut(), || writer_state.borrow().sent.len() == 2);
        }

        // Two packets in, two responses out, in order: the abort is answered by nothing.
        assert_eq!(writer_state.borrow().sent, vec![vec![0x41], vec![0x42]]);
        assert!(pipeline.queue().pending_response().is_none());
    }

    #[test]
    fn caller_reset_discards_stale_session_state() {
        let mut pipeline: DapPipeline<2, 64> = DapPipeline::new();
        pipeline.queue().submit(&[0x51]).expect("queue has room");
        pipeline
            .queue()
            .post_response(&[0x52])
            .expect("queue has room");
        pipeline.queue().submit(&[0x07]).expect("abort is accepted");
        let generation = pipeline.queue().generation();
        pipeline.reset();

        let (mut reader, _) = mock_reader(&[&[0x53]]);
        let (mut writer, writer_state) = mock_writer();
        {
            let mut future = pin!(run_dap_pipeline(
                &mut reader,
                &mut writer,
                &mut pipeline,
                echo
            ));
            drive(future.as_mut(), || !writer_state.borrow().sent.is_empty());
        }

        assert_eq!(writer_state.borrow().sent, vec![vec![0x53]]);
        assert_ne!(pipeline.queue().generation(), generation);
    }

    #[test]
    fn short_command_and_zero_length_packet_keep_framing() {
        let (mut reader, _) = mock_reader(&[&[], &[0x61]]);
        let (mut writer, writer_state) = mock_writer();
        let mut pipeline: DapPipeline<1, 64> = DapPipeline::new();

        let mut future = pin!(run_dap_pipeline(
            &mut reader,
            &mut writer,
            &mut pipeline,
            echo
        ));
        drive(future.as_mut(), || !writer_state.borrow().sent.is_empty());

        assert_eq!(writer_state.borrow().sent, vec![vec![0x61]]);
    }

    #[test]
    #[should_panic(expected = "DAP response exceeds queue slot")]
    fn oversized_response_is_a_visible_failure() {
        let (mut reader, _) = mock_reader(&[&[0x71]]);
        let (mut writer, _) = mock_writer();
        let mut pipeline: DapPipeline<1, 64> = DapPipeline::new();
        let mut future = pin!(run_dap_pipeline(
            &mut reader,
            &mut writer,
            &mut pipeline,
            |_, _| 65,
        ));
        let mut context = Context::from_waker(Waker::noop());

        let _ = future.as_mut().poll(&mut context);
    }

    #[test]
    fn writer_cancellation_does_not_retire_the_response() {
        let (mut reader, _) = mock_reader(&[&[0x81]]);
        let (mut writer, writer_state) = mock_writer();
        writer_state.borrow_mut().stalled = true;
        let mut pipeline: DapPipeline<1, 64> = DapPipeline::new();

        {
            let mut future = Box::pin(run_dap_pipeline(
                &mut reader,
                &mut writer,
                &mut pipeline,
                echo,
            ));
            drive(future.as_mut(), || writer_state.borrow().refused > 0);
        }
        assert_eq!(pipeline.queue().pending_response(), Some(&[0x81][..]));
        assert!(writer_state.borrow().sent.is_empty());
    }

    #[test]
    fn initial_tasks_are_not_acknowledged_as_parked() {
        let pipeline: &'static mut DapPipeline<1, 64> = Box::leak(Box::new(DapPipeline::new()));
        let (read_half, mut owner, write_half) = pipeline.split();
        let (reader, reader_state) = mock_reader(&[]);
        reader_state.borrow_mut().connection_stalled = true;
        let (writer, _) = mock_writer();
        let mut reader_task = Box::pin(dap_reader_task(read_half, reader));
        let mut writer_task = Box::pin(dap_writer_task(write_half, writer));

        completes(owner.end_session());
        assert_eq!(owner.session, SessionState::Initial);
        assert!(!owner.state.reader.parked.signaled());
        assert!(!owner.state.writer.parked.signaled());
        completes(owner.begin_session());
        assert!(poll_once(pin!(owner.begin_session())).is_pending());
        assert!(poll_once(reader_task.as_mut()).is_pending());
        assert!(poll_once(writer_task.as_mut()).is_pending());
        assert!(poll_once(pin!(owner.end_session())).is_pending());
        assert_eq!(
            owner.session,
            SessionState::Stopping {
                reader_parked: false,
                writer_parked: false,
            }
        );
        assert!(!owner.state.reader.parked.signaled());
        reader_state.borrow_mut().connection_stalled = false;
        assert!(poll_once(reader_task.as_mut()).is_pending());
        completes(owner.end_session());
        assert_eq!(owner.session, SessionState::Parked);
        completes(owner.end_session());
    }

    #[test]
    fn cancelled_stop_and_restart_preserve_each_parking_acknowledgement() {
        for reader_first in [false, true] {
            let pipeline: &'static mut DapPipeline<1, 64> = Box::leak(Box::new(DapPipeline::new()));
            let (read_half, mut owner, write_half) = pipeline.split();
            let (reader, reader_state) = mock_reader(&[&[0x91]]);
            let (writer, writer_state) = mock_writer();
            writer_state.borrow_mut().stalled = true;
            let mut reader_task = Box::pin(dap_reader_task(read_half, reader));
            let mut writer_task = Box::pin(dap_writer_task(write_half, writer));
            completes(owner.begin_session());
            assert!(poll_once(reader_task.as_mut()).is_pending());
            assert!(poll_once(pin!(dap_executor(&mut owner, echo))).is_pending());
            assert!(poll_once(reader_task.as_mut()).is_pending());
            assert!(poll_once(writer_task.as_mut()).is_pending());
            assert_eq!(reader_state.borrow().active.get(), 1);
            assert_eq!(writer_state.borrow().active.get(), 1);
            let generation = owner.commands.generation();

            assert!(poll_once(pin!(owner.end_session())).is_pending());
            assert!(poll_once(pin!(owner.end_session())).is_pending());
            if reader_first {
                assert!(poll_once(reader_task.as_mut()).is_pending());
                assert_eq!(reader_state.borrow().active.get(), 0);
            } else {
                assert!(poll_once(writer_task.as_mut()).is_pending());
                assert_eq!(writer_state.borrow().active.get(), 0);
            }
            assert!(poll_once(pin!(owner.end_session())).is_pending());
            assert_eq!(
                owner.session,
                SessionState::Stopping {
                    reader_parked: reader_first,
                    writer_parked: false,
                }
            );
            assert!(poll_once(pin!(DapSession::begin_session(&mut owner))).is_pending());
            assert_eq!(owner.commands.generation(), generation);
            if reader_first {
                assert!(poll_once(writer_task.as_mut()).is_pending());
            } else {
                assert!(poll_once(reader_task.as_mut()).is_pending());
            }
            assert_eq!(reader_state.borrow().active.get(), 0);
            assert_eq!(writer_state.borrow().active.get(), 0);
            completes(owner.begin_session());
            assert_eq!(owner.session, SessionState::Running);
            assert_eq!(owner.commands.generation(), generation + 1);
            assert!(!owner.commands.responses_full());
            writer_state.borrow_mut().stalled = false;
            reader_state.borrow_mut().packets.push_back(vec![0x92]);
            assert!(poll_once(reader_task.as_mut()).is_pending());
            assert!(poll_once(pin!(dap_executor(&mut owner, echo))).is_pending());
            assert!(poll_once(writer_task.as_mut()).is_pending());
            assert_eq!(writer_state.borrow().sent, vec![vec![0x92]]);
            assert!(poll_once(pin!(owner.end_session())).is_pending());
            assert!(poll_once(reader_task.as_mut()).is_pending());
            assert!(poll_once(writer_task.as_mut()).is_pending());
            completes(owner.end_session());
            completes(owner.end_session());
            assert_eq!(owner.commands.generation(), generation + 1);
        }
    }

    #[test]
    fn endpoint_errors_park_only_after_operations_are_dropped() {
        for reader_error in [false, true] {
            let pipeline: &'static mut DapPipeline<1, 64> = Box::leak(Box::new(DapPipeline::new()));
            let (read_half, mut owner, write_half) = pipeline.split();
            let (reader, reader_state) = mock_reader(&[]);
            let (writer, writer_state) = mock_writer();
            reader_state.borrow_mut().disabled = reader_error;
            writer_state.borrow_mut().disabled = !reader_error;
            let mut reader_task = Box::pin(dap_reader_task(read_half, reader));
            let mut writer_task = Box::pin(dap_writer_task(write_half, writer));
            completes(owner.begin_session());
            owner.commands.post_response(&[0x91]).unwrap();
            owner.state.response_available.signal(());
            assert!(poll_once(reader_task.as_mut()).is_pending());
            assert!(poll_once(writer_task.as_mut()).is_pending());
            assert!(owner.state.disconnected());
            if reader_error {
                assert_eq!(reader_state.borrow().active.get(), 0);
                assert!(owner.state.reader.parked.signaled());
            } else {
                assert_eq!(writer_state.borrow().active.get(), 0);
                assert!(owner.state.writer.parked.signaled());
            }
            let _ = poll_once(pin!(owner.end_session()));
            assert!(poll_once(reader_task.as_mut()).is_pending());
            assert!(poll_once(writer_task.as_mut()).is_pending());
            completes(owner.end_session());
            assert_eq!(owner.session, SessionState::Parked);
            assert_eq!(reader_state.borrow().active.get(), 0);
            assert_eq!(writer_state.borrow().active.get(), 0);
        }
    }

    #[test]
    fn cancelling_an_endpoint_task_cannot_forge_a_parking_acknowledgement() {
        let pipeline: &'static mut DapPipeline<1, 64> = Box::leak(Box::new(DapPipeline::new()));
        let (read_half, mut owner, write_half) = pipeline.split();
        let (reader, reader_state) = mock_reader(&[]);
        let (writer, _) = mock_writer();
        let mut reader_task = Box::pin(dap_reader_task(read_half, reader));
        let mut writer_task = Box::pin(dap_writer_task(write_half, writer));
        completes(owner.begin_session());
        assert!(poll_once(reader_task.as_mut()).is_pending());
        assert_eq!(reader_state.borrow().active.get(), 1);
        drop(reader_task);
        assert_eq!(reader_state.borrow().active.get(), 0);
        assert!(!owner.state.reader.parked.signaled());
        let generation = owner.commands.generation();
        assert!(poll_once(pin!(owner.begin_session())).is_pending());
        assert!(poll_once(writer_task.as_mut()).is_pending());
        assert!(poll_once(pin!(owner.wait_connection())).is_pending());
        assert_eq!(owner.commands.generation(), generation);
    }

    #[test]
    fn resplitting_exclusive_pipeline_clears_old_task_permissions() {
        let mut pipeline: DapPipeline<1, 64> = DapPipeline::new();
        {
            let (_, mut owner, _) = pipeline.split();
            completes(owner.begin_session());
            assert!(owner.state.reader.start.signaled());
            assert!(owner.state.writer.start.signaled());
        }
        let (_, owner, _) = pipeline.split();
        assert_eq!(owner.session, SessionState::Initial);
        assert!(!owner.state.reader.start.signaled());
        assert!(!owner.state.writer.start.signaled());
        assert!(!owner.state.reader.parked.signaled());
        assert!(!owner.state.writer.parked.signaled());
    }

    #[test]
    fn suspend_resume_keeps_pending_response_and_generation() {
        let _guard = SESSION_TESTS.lock().expect("session tests serialize");
        let pipeline: &'static mut DapPipeline<1, 64> = Box::leak(Box::new(DapPipeline::new()));
        let (_, mut owner, write_half) = pipeline.split();
        let (writer, writer_state) = mock_writer();
        writer_state.borrow_mut().stalled = true;
        let mut writer_task = Box::pin(dap_writer_task(write_half, writer));
        completes(owner.begin_session());
        owner.commands.post_response(&[0x91]).unwrap();
        let generation = owner.commands.generation();
        let epoch = session_epoch();
        assert!(poll_once(writer_task.as_mut()).is_pending());
        let mut handler = DapV2Handler {
            iface_string: StringIndex(4),
        };
        handler.suspended(true);
        assert_eq!(session_epoch(), epoch);
        assert_eq!(owner.commands.generation(), generation);
        handler.suspended(false);
        writer_state.borrow_mut().stalled = false;
        assert!(poll_once(writer_task.as_mut()).is_pending());
        assert_eq!(session_epoch(), epoch);
        assert_eq!(owner.commands.generation(), generation);
        assert_eq!(writer_state.borrow().sent, vec![vec![0x91]]);
        handler.reset();
        assert_ne!(session_epoch(), epoch);
    }

    /// The interrupt composition: reader, writer and executor polled independently.
    #[test]
    fn split_tasks_serve_a_session_and_park_at_its_end() {
        let pipeline: &'static mut DapPipeline<2, 64> = Box::leak(Box::new(DapPipeline::new()));
        let (read_half, mut exec_half, write_half) = pipeline.split();

        let (reader, reader_state) = mock_reader(&[&[0x91], &[0x92]]);
        // The host goes away once both commands are in: the session must then wind up cleanly.
        reader_state.borrow_mut().disabled = true;
        let (writer, writer_state) = mock_writer();

        let mut reader_task = Box::pin(dap_reader_task(read_half, reader));
        let mut writer_task = Box::pin(dap_writer_task(write_half, writer));
        let mut owner = Box::pin(async {
            exec_half.wait_connection().await;
            exec_half.begin_session().await;
            dap_executor(&mut exec_half, echo).await;
            exec_half.end_session().await;
        });

        let mut context = Context::from_waker(Waker::noop());
        let mut finished = false;
        for _ in 0..1_000 {
            assert!(reader_task.as_mut().poll(&mut context).is_pending());
            assert!(writer_task.as_mut().poll(&mut context).is_pending());
            if owner.as_mut().poll(&mut context).is_ready() {
                finished = true;
                break;
            }
        }
        assert!(finished, "the session never wound up");
        assert_eq!(writer_state.borrow().sent, vec![vec![0x91], vec![0x92]]);
    }

    /// Accepts `accept` packets, then refuses every further one.
    struct MockSink {
        accept: usize,
        accepted_bytes: usize,
        packets: usize,
        /// Past `accept`, wait forever instead of refusing.
        stall: bool,
    }

    impl MockSink {
        fn up_to(accept: usize) -> Self {
            Self {
                accept,
                accepted_bytes: 0,
                packets: 0,
                stall: false,
            }
        }
    }

    impl PacketSink for MockSink {
        async fn send(&mut self, packet: &[u8]) -> Result<(), EndpointError> {
            if self.accept == 0 {
                if self.stall {
                    core::future::pending::<()>().await;
                }
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
        halted_at: Option<u32>,
        last_read_at: Option<u32>,
        applied_at: Option<u32>,
        applied_baud: u32,
        divisor: u32,
        attempts: Vec<u32>,
        tx: VecDeque<u8>,
        sent: Vec<(u8, u32)>,
        drain_blocked: Rc<Cell<bool>>,
        productive_rx: bool,
        rx_taken: usize,
        /// What `0x9E` would have answered while the transaction held the wire.
        state_while_draining: Option<daplink_rs::VcomBaudState>,
    }

    impl MockUart {
        fn with_pending(pending: usize) -> Self {
            Self {
                pending,
                step: 0,
                drained_at: None,
                halted_at: None,
                last_read_at: None,
                applied_at: None,
                applied_baud: 9_600,
                divisor: 48_000_000 / 9_600,
                attempts: Vec::new(),
                tx: VecDeque::new(),
                sent: Vec::new(),
                drain_blocked: Rc::new(Cell::new(false)),
                productive_rx: false,
                rx_taken: 0,
                state_while_draining: None,
            }
        }

        fn realized_baud(&self) -> u32 {
            48_000_000 / self.divisor
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
            let n = if self.productive_rx {
                buf.len()
            } else {
                let n = self.pending.min(buf.len());
                self.pending -= n;
                n
            };
            self.rx_taken += n;
            buf[..n].fill(0xa5);
            n
        }

        fn seam_stats(&self) -> SeamStats {
            SeamStats::default()
        }

        async fn write_all(&mut self, data: &[u8]) {
            self.tx.extend(data);
        }

        async fn drain_tx(&mut self) {
            self.state_while_draining = Some(vcom_baud_state(self.realized_baud()));
            core::future::poll_fn(|_| {
                if self.drain_blocked.get() {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
            while let Some(byte) = self.tx.pop_front() {
                self.sent.push((byte, self.realized_baud()));
            }
            self.step += 1;
            self.drained_at = Some(self.step);
        }

        fn halt_rx(&mut self) {
            self.step += 1;
            self.halted_at = Some(self.step);
            self.productive_rx = false;
        }

        fn set_baudrate(&mut self, baud: u32) -> BaudApply {
            self.attempts.push(baud);
            let Some(divisor) = 48_000_000_u32.checked_div(baud) else {
                return BaudApply::Rejected;
            };
            if !(16..65_536).contains(&divisor) {
                return BaudApply::Rejected;
            }
            self.step += 1;
            self.applied_at = Some(self.step);
            self.applied_baud = baud;
            self.divisor = divisor;
            BaudApply::Applied
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
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
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

        // A push cancelled mid-send leaves its chunk owed; abandoning charges every byte once.
        let base = counters();
        let mut staged = Staged::new();
        let mut sink = MockSink::up_to(usize::MAX);
        assert!(now(staged.push(&mut sink, &data[..7])).is_ok());
        let mut sink = MockSink::up_to(1);
        sink.stall = true;
        assert!(poll_once(pin!(staged.push(&mut sink, &data[..2 * PACKET]))).is_pending());
        staged.abandon();
        assert_eq!(staged.len, 0);
        let after = counters();
        assert_eq!(after.1 - base.1, PACKET as u32);
        assert_eq!(after.2 - base.2, (7 + PACKET) as u32);
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
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
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

    /// Drain, hand old-rate bytes to the pipe, then apply, the query showing the change in flight.
    #[test]
    fn line_change_drains_wire_and_rx_before_applying() {
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
        let mut uart = MockUart::with_pending(DRAIN_BYTES + 44);
        let pipe: Pipe<NoopRawMutex, RETURN_PIPE> = Pipe::new();
        let mut scratch = [0u8; DRAIN_BYTES];
        let mut applied_baud = uart.applied_baud;
        APPLIED_BAUD.store(applied_baud, Ordering::Relaxed);
        now(apply_line_change(
            &mut uart,
            &pipe,
            &mut scratch,
            250_000,
            &mut applied_baud,
            || true,
        ));
        assert_eq!(uart.applied_baud, 250_000);
        assert_eq!(applied_baud, 250_000);
        assert_eq!(uart.pending, 0);
        assert_eq!(pipe.len(), DRAIN_BYTES + 44);
        assert!(uart.drained_at.unwrap() < uart.halted_at.unwrap());
        assert!(uart.halted_at.unwrap() < uart.last_read_at.unwrap());
        assert!(uart.last_read_at.unwrap() < uart.applied_at.unwrap());

        // 0x9E reports the transaction while it runs, never the rate being left as settled.
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

    #[test]
    fn line_changes_keep_accepted_nominal_and_divisor_state_distinct() {
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
        let mut uart = MockUart::with_pending(0);
        let pipe: Pipe<NoopRawMutex, RETURN_PIPE> = Pipe::new();
        let mut scratch = [0u8; DRAIN_BYTES];
        let mut applied = uart.applied_baud;
        APPLIED_BAUD.store(applied, Ordering::Relaxed);
        for (request, expected, attempted) in [
            (0, 9_600, false),
            (9_600, 9_600, false),
            (230_400, 230_400, true),
            (96_000_000, 230_400, true),
            (96_000_000, 230_400, true),
            (230_400, 230_400, false),
            (9_600, 9_600, true),
            (1, 9_600, true),
            (1, 9_600, true),
            (0, 9_600, false),
        ] {
            let old_divisor = uart.divisor;
            let attempts = uart.attempts.len();
            now(apply_line_change(
                &mut uart,
                &pipe,
                &mut scratch,
                request,
                &mut applied,
                || true,
            ));
            assert_eq!(uart.attempts.len(), attempts + usize::from(attempted));
            if request != expected {
                assert_eq!(uart.divisor, old_divisor);
            }
            assert_eq!(applied, expected);
            assert_eq!(uart.applied_baud, expected);
            let reported = vcom_baud_state(uart.realized_baud());
            assert_eq!(reported.applied, expected);
            assert_eq!(reported.realized, 48_000_000 / (48_000_000 / expected));
            assert_ne!(reported.flags & daplink_rs::VCOM_BAUD_SETTLED, 0);
            assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 0);
        }
        assert_eq!(
            uart.attempts,
            vec![230_400, 96_000_000, 96_000_000, 9_600, 1, 1]
        );
    }

    #[test]
    fn queued_transmit_bytes_leave_at_the_pre_change_divisor() {
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
        let mut uart = MockUart::with_pending(0);
        let pipe: Pipe<NoopRawMutex, RETURN_PIPE> = Pipe::new();
        let mut scratch = [0u8; DRAIN_BYTES];
        let mut applied = uart.applied_baud;
        APPLIED_BAUD.store(applied, Ordering::Relaxed);
        now(uart.write_all(&[1, 2, 3]));
        let blocked = uart.drain_blocked.clone();
        blocked.set(true);
        {
            let mut change = Box::pin(apply_line_change(
                &mut uart,
                &pipe,
                &mut scratch,
                230_400,
                &mut applied,
                || true,
            ));
            assert!(poll_once(change.as_mut()).is_pending());
            assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 230_400);
            let reported = vcom_baud_state(9_600);
            assert_eq!(reported.applied, 9_600);
            assert_eq!(reported.flags & daplink_rs::VCOM_BAUD_SETTLED, 0);
            blocked.set(false);
            assert!(poll_once(change.as_mut()).is_ready());
        }
        assert_eq!(uart.sent, vec![(1, 9_600), (2, 9_600), (3, 9_600)]);
        assert_eq!(applied, 230_400);
        assert_eq!(uart.realized_baud(), 230_769);
        now(uart.write_all(&[4]));
        now(uart.drain_tx());
        assert_eq!(uart.sent.last(), Some(&(4, 230_769)));
    }

    #[test]
    fn cancelling_transmit_drain_clears_pending_without_applying() {
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
        let mut uart = MockUart::with_pending(3);
        let pipe: Pipe<NoopRawMutex, RETURN_PIPE> = Pipe::new();
        let mut scratch = [0u8; DRAIN_BYTES];
        let mut applied = uart.applied_baud;
        APPLIED_BAUD.store(applied, Ordering::Relaxed);
        uart.drain_blocked.set(true);
        now(uart.write_all(&[1, 2]));
        assert!(poll_once(pin!(apply_line_change(
            &mut uart,
            &pipe,
            &mut scratch,
            250_000,
            &mut applied,
            || true,
        )))
        .is_pending());
        assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 0);
        assert_eq!(applied, 9_600);
        assert_eq!(vcom_baud_state(uart.realized_baud()).applied, 9_600);
        assert!(uart.attempts.is_empty());
        assert_eq!(uart.pending, 3);
        assert_eq!(uart.tx, VecDeque::from([1, 2]));
        uart.drain_blocked.set(false);
        now(apply_line_change(
            &mut uart,
            &pipe,
            &mut scratch,
            250_000,
            &mut applied,
            || true,
        ));
        assert_eq!(applied, 250_000);
        assert_eq!(uart.attempts, vec![250_000]);
    }

    /// Only bytes the pipe has room for leave the seam, so a cancelled change strands none.
    #[test]
    fn cancelling_a_line_change_on_a_full_pipe_strands_no_bytes() {
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
        let _clock = crate::msc::tests::clock_guard();
        for occupied in [0, 2, 4] {
            let mut uart = MockUart::with_pending(8);
            let pipe: Pipe<NoopRawMutex, 4> = Pipe::new();
            now(pipe.write_all(&[0x11; 4][..occupied]));
            let mut scratch = [0u8; 8];
            let mut applied = uart.applied_baud;
            APPLIED_BAUD.store(applied, Ordering::Relaxed);
            let base = counters();
            {
                let mut change = Box::pin(apply_line_change(
                    &mut uart,
                    &pipe,
                    &mut scratch,
                    250_000,
                    &mut applied,
                    || true,
                ));
                for _ in 0..4 {
                    assert!(poll_once(change.as_mut()).is_pending());
                    assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 250_000);
                    assert_eq!(APPLIED_BAUD.load(Ordering::Relaxed), 9_600);
                }
            }
            assert_eq!(pipe.len(), 4);
            assert_eq!(uart.rx_taken, 4 - occupied);
            assert_eq!(uart.pending, 8 - uart.rx_taken);
            assert_eq!(counters(), base);
            assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 0);
            assert_eq!(applied, 9_600);
            assert!(uart.attempts.is_empty());
        }
    }

    /// A source that never runs dry holds a line change no longer than the byte cap.
    #[test]
    fn productive_old_rate_rx_holds_a_line_change_only_to_the_cap() {
        let _guard = VCOM_TESTS.lock().expect("VCOM tests serialize");
        let _clock = crate::msc::tests::clock_guard();
        let mut uart = MockUart::with_pending(0);
        uart.productive_rx = true;
        let pipe: Pipe<NoopRawMutex, 4> = Pipe::new();
        let mut scratch = [0u8; 4];
        let mut applied = uart.applied_baud;
        APPLIED_BAUD.store(applied, Ordering::Relaxed);
        let base = counters();
        let mut forwarded = 0;
        {
            let mut change = Box::pin(apply_line_change(
                &mut uart,
                &pipe,
                &mut scratch,
                250_000,
                &mut applied,
                || true,
            ));
            let mut polls = 0;
            while poll_once(change.as_mut()).is_pending() {
                assert_eq!(APPLIED_BAUD.load(Ordering::Relaxed), 9_600);
                forwarded += pipe.try_read(&mut [0u8; 4]).unwrap();
                embassy_time::MockDriver::get().advance(PIPE_FULL_POLL);
                polls += 1;
                assert!(polls <= LINE_CHANGE_CAP);
            }
        }
        assert_eq!(uart.rx_taken, LINE_CHANGE_CAP);
        assert_eq!(forwarded + pipe.len(), LINE_CHANGE_CAP);
        assert_eq!(counters(), base);
        assert_eq!(uart.attempts, vec![250_000]);
        assert_eq!(applied, 250_000);
        assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 0);
    }

    pub(crate) static VCOM_TESTS: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// The session loop drives one global epoch counter, so its tests take turns.
    static SESSION_TESTS: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// What the three board closures did, in the order they were called.
    #[derive(Default)]
    struct SessionLog {
        events: Vec<&'static str>,
        counts: Vec<u32>,
    }

    type LogHandle = Rc<RefCell<SessionLog>>;

    fn session_loop<'a, 'p>(
        session: &'a mut PipelineSession<'p, MockReader, MockWriter, 2, 64>,
        context: &'a mut LogHandle,
    ) -> impl Future + use<'a, 'p> {
        dap_sessions::<true, _, _, _, _, _>(
            session,
            context,
            |log, request, response| {
                log.borrow_mut().events.push("execute");
                echo(request, response)
            },
            |log| log.borrow_mut().events.push("suspend"),
            |log, processed| {
                let mut log = log.borrow_mut();
                log.events.push("finished");
                log.counts.push(processed);
            },
        )
    }

    #[test]
    fn each_session_starts_clean_and_is_summarized_once() {
        let _turn = SESSION_TESTS
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let (reader, reader_state) = mock_reader(&[&[0xa1]]);
        let (writer, writer_state) = mock_writer();
        let mut pipeline: DapPipeline<2, 64> = DapPipeline::new();
        // A prior session's leftovers: the boundary reset must drop them.
        pipeline.queue().submit(&[0xee]).expect("queue has room");
        let mut session = PipelineSession::new(reader, writer, &mut pipeline);
        let log: LogHandle = Rc::new(RefCell::new(SessionLog::default()));
        let mut context = log.clone();

        let mut future = Box::pin(session_loop(&mut session, &mut context));
        drive(future.as_mut(), || !writer_state.borrow().sent.is_empty());
        // Nothing after the select has run while the session is up.
        assert_eq!(log.borrow().events, vec!["execute"]);

        session_boundary();
        drive(future.as_mut(), || log.borrow().counts.len() == 1);
        assert_eq!(log.borrow().events, vec!["execute", "finished", "suspend"]);

        reader_state.borrow_mut().packets.push_back(vec![0xa2]);
        drive(future.as_mut(), || writer_state.borrow().sent.len() == 2);
        session_boundary();
        drive(future.as_mut(), || log.borrow().counts.len() == 2);

        // One command each session, one summary each session, and the stale packet never ran.
        assert_eq!(log.borrow().counts, vec![1, 1]);
        assert_eq!(writer_state.borrow().sent, vec![vec![0xa1], vec![0xa2]]);
    }

    #[test]
    fn a_write_parked_on_a_dead_session_is_dropped() {
        let _turn = SESSION_TESTS
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let (reader, reader_state) = mock_reader(&[&[0xb1]]);
        let (writer, writer_state) = mock_writer();
        writer_state.borrow_mut().stalled = true;
        let mut pipeline: DapPipeline<2, 64> = DapPipeline::new();
        let mut session = PipelineSession::new(reader, writer, &mut pipeline);
        let log: LogHandle = Rc::new(RefCell::new(SessionLog::default()));
        let mut context = log.clone();

        let mut future = Box::pin(session_loop(&mut session, &mut context));
        drive(future.as_mut(), || writer_state.borrow().refused > 0);

        session_boundary();
        drive(future.as_mut(), || log.borrow().counts.len() == 1);
        assert!(writer_state.borrow().sent.is_empty());

        // The next session opens on an empty queue: the parked response is gone, not delivered.
        writer_state.borrow_mut().stalled = false;
        reader_state.borrow_mut().packets.push_back(vec![0xb2]);
        drive(future.as_mut(), || !writer_state.borrow().sent.is_empty());
        assert_eq!(writer_state.borrow().sent, vec![vec![0xb2]]);
    }

    #[test]
    fn deconfiguring_or_disabling_ends_the_session() {
        let _turn = SESSION_TESTS
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let mut handler = DapV2Handler {
            iface_string: StringIndex(0),
        };
        let start = session_epoch();
        handler.configured(true);
        handler.enabled(true);
        assert_eq!(session_epoch(), start);
        handler.configured(false);
        assert_eq!(session_epoch(), start + 1);
        handler.enabled(false);
        assert_eq!(session_epoch(), start + 2);
    }

    /// A boundary from coming up belongs to the last session; the next one winds up what it ran.
    #[derive(Default)]
    struct BoundaryOnConnect {
        connected: bool,
        ran: Rc<Cell<u32>>,
        ended: Rc<Cell<u32>>,
    }

    impl DapSession for BoundaryOnConnect {
        async fn wait_connection(&mut self) {
            if !core::mem::replace(&mut self.connected, true) {
                session_boundary();
            }
        }
        async fn begin_session(&mut self) {}
        async fn run<E: FnMut(&[u8], &mut [u8]) -> usize>(&mut self, _execute: E) {
            self.ran.set(self.ran.get() + 1);
            core::future::pending::<()>().await;
        }
        async fn end_session(&mut self) {
            self.ended.set(self.ended.get() + 1);
        }
    }

    #[test]
    fn a_session_survives_its_own_connection_boundary_and_is_wound_up_at_the_next() {
        let _turn = SESSION_TESTS
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let mut session = BoundaryOnConnect::default();
        let (ran, ended) = (session.ran.clone(), session.ended.clone());
        let mut context = ();
        let mut future = Box::pin(dap_sessions::<false, _, _, _, _, _>(
            &mut session,
            &mut context,
            |_, _, _| 0,
            |_| {},
            |_, _| {},
        ));
        drive(future.as_mut(), || ran.get() == 1);
        let waker_context = &mut Context::from_waker(Waker::noop());
        assert!(future.as_mut().poll(waker_context).is_pending());
        assert_eq!((ran.get(), ended.get()), (1, 0));
        session_boundary();
        drive(future.as_mut(), || ended.get() == 1);
    }
}
