//! USB mass-storage (Bulk-Only Transport) class over the virtual volume.

use daplink_rs::daplink_core::msc_proto::{parse_cbw, Sense, CBW_LEN, SENSE_NONE};
use daplink_rs::daplink_core::{
    NoUsbReconnect, UsbReconnect, MSC_BULK_ONLY_RESET, MSC_GET_MAX_LUN, MSC_INTERFACE_CLASS,
    MSC_SECTOR_SIZE,
};
use daplink_vfs::msc_bot::{run_command, BulkIn, BulkOut, MsClock};
use daplink_vfs::{DirectIo, SectorIo, TransferSink, Vfs, VfsObserver};
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn, EndpointOut};
use embassy_usb::{Builder, Handler};

/// A host-issued Bulk-Only Reset, raised on the control endpoint and consumed by the class task.
pub type BotReset = Signal<CriticalSectionRawMutex, ()>;

/// Greatest common divisor, reducing the tick-rate fraction below at compile time.
const fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// Milliseconds per embassy tick, as the reduced fraction `MS_NUM`/`MS_DEN` of `1000 / TICK_HZ`:
/// a ticks-per-millisecond integer cannot express every tick rate (32_768 gives 32.768).
const MS_NUM: u32 = (1000 / gcd(1000, embassy_time::TICK_HZ)) as u32;
/// Denominator of the [`MS_NUM`] fraction, and the unit of the clock's carried remainder.
const MS_DEN: u32 = (embassy_time::TICK_HZ / gcd(1000, embassy_time::TICK_HZ)) as u32;

// The clock's arithmetic is 32-bit, so the reduced denominator has to fit a u32.
const _: () = assert!(embassy_time::TICK_HZ / gcd(1000, embassy_time::TICK_HZ) <= u32::MAX as u64);
// The fraction must be exactly 1000/TICK_HZ, cross-multiplied so no division rounds here.
const _: () = assert!(MS_NUM as u64 * embassy_time::TICK_HZ == MS_DEN as u64 * 1000);

/// Time off the bus for a forced re-enumeration: long enough that every host unmounts the volume.
const USB_RECONNECT_DETACH_MS: u64 = 500;

/// Newtype so the bulk-transport traits can be implemented over the driver's endpoints. A pending
/// reset resolves the transfer as an error, which ends the command at that packet boundary without
/// a CSW; it is polled before the endpoint, so a packet already buffered cannot outrun it.
struct In<'e, E: EndpointIn> {
    ep: &'e mut E,
    reset: &'e BotReset,
}

impl<E: EndpointIn> BulkIn for In<'_, E> {
    async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        match select(self.reset.wait(), EndpointIn::write(self.ep, data)).await {
            Either::First(()) => Err(()),
            Either::Second(r) => r.map_err(|_| ()),
        }
    }
}

/// The host-to-device twin of [`In`].
struct Out<'e, E: EndpointOut> {
    ep: &'e mut E,
    reset: &'e BotReset,
}

impl<E: EndpointOut> BulkOut for Out<'_, E> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        match select(self.reset.wait(), EndpointOut::read(self.ep, buf)).await {
            Either::First(()) => Err(()),
            Either::Second(r) => r.map_err(|_| ()),
        }
    }
}

/// The bulk endpoint pair backing the mass-storage interface; `EP` is the bulk packet size.
pub struct MscEndpoints<'d, D: Driver<'d>, const EP: usize = 64> {
    ep_in: D::EndpointIn,
    ep_out: D::EndpointOut,
    reset: &'d BotReset,
}

/// Answers the two Bulk-Only class control requests once the interface number is known.
pub struct MscControlHandler<'d> {
    iface: Option<u8>,
    reset: &'d BotReset,
}

impl<'d> MscControlHandler<'d> {
    /// Create the handler over the signal the class task observes; the interface number is filled
    /// in at registration time.
    pub const fn new(reset: &'d BotReset) -> Self {
        Self { iface: None, reset }
    }

    /// Whether `req` targets this interface as a class request.
    fn is_for_us(&self, req: &Request) -> bool {
        req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && Some(req.index as u8) == self.iface
    }
}

impl Handler for MscControlHandler<'_> {
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if self.is_for_us(&req) && req.request == MSC_GET_MAX_LUN {
            // Single LUN: the maximum LUN number is 0.
            buf[0] = 0;
            Some(InResponse::Accepted(&buf[..1]))
        } else {
            None
        }
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if self.is_for_us(&req) && req.request == MSC_BULK_ONLY_RESET {
            // Every byte of transport state lives in the class task, so the reset is signalled to
            // it rather than applied here. Endpoint toggles stay the host's to clear.
            self.reset.signal(());
            Some(OutResponse::Accepted)
        } else {
            None
        }
    }
}

/// Register the function, interface, and bulk pair, wiring the handler to the interface number.
pub(crate) fn add_msc_function<'d, D: Driver<'d>, const EP: usize>(
    builder: &mut Builder<'d, D>,
    handler: &'d mut MscControlHandler<'d>,
) -> MscEndpoints<'d, D, EP> {
    let reset = handler.reset;
    let (class, subclass, protocol) = MSC_INTERFACE_CLASS;
    let mut function = builder.function(class, subclass, protocol);
    let mut interface = function.interface();
    handler.iface = Some(interface.interface_number().0);
    let mut alt = interface.alt_setting(class, subclass, protocol, None);
    let ep_out = alt.endpoint_bulk_out(None, EP as u16);
    let ep_in = alt.endpoint_bulk_in(None, EP as u16);
    drop(function);

    builder.handler(handler);

    MscEndpoints {
        ep_in,
        ep_out,
        reset,
    }
}

/// Drive the class forever over `vfs`; a remount republishes through SCSI UNIT ATTENTION alone.
pub async fn msc_task<'d, D, S, O, const EP: usize>(
    eps: &mut MscEndpoints<'d, D, EP>,
    vfs: &mut Vfs<'_, S, O>,
) -> !
where
    D: Driver<'d>,
    S: TransferSink,
    O: VfsObserver,
{
    msc_task_with_reconnect(eps, vfs, NoUsbReconnect).await
}

/// [`msc_task`] with a hardware [`UsbReconnect`]: each remount also forces a detach/reattach, for
/// hosts that never poll TEST UNIT READY on a mounted volume. This owns the detach timing.
pub async fn msc_task_with_reconnect<'d, D, S, O, R, const EP: usize>(
    eps: &mut MscEndpoints<'d, D, EP>,
    vfs: &mut Vfs<'_, S, O>,
    mut reconnect: R,
) -> !
where
    D: Driver<'d>,
    S: TransferSink,
    O: VfsObserver,
    R: UsbReconnect,
{
    let reset = eps.reset;
    let mut clock = MsClock::new(embassy_time::Instant::now().as_ticks());
    let mut sense: Sense = SENSE_NONE;
    let mut ua_pending = false;
    let mut sector = [0u8; MSC_SECTOR_SIZE];
    let mut cbw_buf = [0u8; EP];
    loop {
        vfs.tick(clock.advance_to::<MS_NUM, MS_DEN>(embassy_time::Instant::now().as_ticks()));
        // Latch a media-changed remount edge; it persists until a TEST UNIT READY reports it.
        ua_pending |= DirectIo::new(vfs).media_changed();
        if vfs.take_needs_reconnect() {
            reconnect.detach();
            embassy_time::Timer::after_millis(USB_RECONNECT_DETACH_MS).await;
            reconnect.attach();
        }

        let n = match select3(
            reset.wait(),
            EndpointOut::read(&mut eps.ep_out, &mut cbw_buf),
            embassy_time::Timer::after_millis(100),
        )
        .await
        {
            // Between commands a reset has nothing to abandon, but it must still be consumed here
            // or the next command would be cut short by it.
            Either3::First(()) => continue,
            Either3::Second(Ok(n)) => n,
            // The endpoint errors whenever the interface is down: park until the host re-enables it.
            Either3::Second(Err(_)) => {
                eps.ep_out.wait_enabled().await;
                continue;
            }
            // Idle tick: loop back so `vfs.tick` advances the remount/timeout cycle.
            Either3::Third(()) => continue,
        };

        let parsed = if n == CBW_LEN {
            parse_cbw(&cbw_buf[..CBW_LEN])
        } else {
            None
        };
        // This class cannot stall, so a non-CBW packet is met with silence: the host resets us.
        let Some(cbw) = parsed else {
            continue;
        };
        let mut io = DirectIo::new(vfs);
        run_command::<_, _, _, EP>(
            &mut In {
                ep: &mut eps.ep_in,
                reset,
            },
            &mut Out {
                ep: &mut eps.ep_out,
                reset,
            },
            &mut io,
            &mut sector,
            &mut sense,
            &mut ua_pending,
            &cbw,
        )
        .await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::RefCell;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use daplink_decoder::Format;
    use daplink_rs::daplink_core::msc_proto::{
        CBW_SIGNATURE, CSW_FAILED, CSW_LEN, CSW_PASSED, CSW_SIGNATURE,
    };
    use daplink_vfs::{
        fat, DecodeError, MediaState, NoObserver, NullSink, SectorPoll, TransferOutcome, VfsConfig,
        VfsFile,
    };
    use embassy_time::{Duration, Instant, MockDriver};
    use embassy_usb::driver::{
        Bus, ControlPipe, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo,
        EndpointType, Event, Unsupported,
    };
    use embassy_usb::Config;
    use std::sync::{Mutex, MutexGuard};
    use std::vec;
    use std::vec::Vec;

    /// The bulk packet size the tests drive the transport at.
    const EP_CHUNK: usize = 64;

    /// The mock time driver is one process-wide clock, so the tests that advance it take turns.
    fn clock_guard() -> MutexGuard<'static, ()> {
        static LOCK: Mutex<()> = Mutex::new(());
        LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Poll to completion; every mock resolves inline, so a noop waker suffices.
    fn block_on<F: Future>(f: F) -> F::Output {
        let mut cx = Context::from_waker(Waker::noop());
        let mut f = pin!(f);
        loop {
            if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// Poll the never-returning class task `polls` times, advancing the clock between polls. One
    /// poll runs the loop until something is genuinely pending, so a parked read ends it.
    fn drive<F: Future>(f: F, polls: usize, step_ms: u64) {
        let mut cx = Context::from_waker(Waker::noop());
        let mut f = pin!(f);
        for _ in 0..polls {
            let _ = f.as_mut().poll(&mut cx);
            if step_ms > 0 {
                MockDriver::get().advance(Duration::from_millis(step_ms));
            }
        }
    }

    /// One scripted host-to-device event on the bulk OUT endpoint.
    enum OutStep {
        /// A packet the host sends.
        Packet(Vec<u8>),
        /// A read that fails the way a disabled interface fails one.
        Disabled,
        /// The host sends nothing this turn: the read never completes, so the idle timer wins.
        Idle,
        /// The host is gone for good; unlike [`OutStep::Idle`] this step is never consumed.
        Park,
    }

    /// Shorthand for a scripted host packet.
    fn pkt(bytes: &[u8]) -> OutStep {
        OutStep::Packet(bytes.to_vec())
    }

    /// Delivers scripted host-to-device packets, raising `reset` once `reset_after` are consumed.
    /// A script that runs out reads as a torn-down interface, the way a disabled endpoint does.
    struct MockOut<'a> {
        info: EndpointInfo,
        steps: Vec<OutStep>,
        idx: usize,
        /// How many times the class task waited for the interface to come back up.
        enables: usize,
        reset_after: Option<usize>,
        reset: &'a BotReset,
    }

    impl<'a> MockOut<'a> {
        fn new(reset: &'a BotReset, steps: Vec<OutStep>) -> Self {
            Self {
                info: ep_info(0x01, EP_CHUNK as u16),
                steps,
                idx: 0,
                enables: 0,
                reset_after: None,
                reset,
            }
        }
    }

    impl Endpoint for MockOut<'_> {
        fn info(&self) -> &EndpointInfo {
            &self.info
        }
        async fn wait_enabled(&mut self) {
            self.enables += 1;
        }
    }

    impl EndpointOut for MockOut<'_> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
            let n = match self.steps.get(self.idx) {
                None => return Err(EndpointError::Disabled),
                Some(OutStep::Park) => core::future::pending().await,
                Some(OutStep::Disabled) => {
                    self.idx += 1;
                    return Err(EndpointError::Disabled);
                }
                Some(OutStep::Idle) => {
                    self.idx += 1;
                    core::future::pending().await
                }
                Some(OutStep::Packet(p)) => {
                    buf[..p.len()].copy_from_slice(p);
                    p.len()
                }
            };
            self.idx += 1;
            if self.reset_after == Some(self.idx) {
                self.reset.signal(());
            }
            Ok(n)
        }
    }

    /// Records every device-to-host packet, raising `reset` once `reset_after` have gone out.
    struct MockIn<'a> {
        info: EndpointInfo,
        written: Vec<u8>,
        /// Length of each write in order: a zero-length packet is only visible as framing.
        packets: Vec<usize>,
        reset_after: Option<usize>,
        reset: &'a BotReset,
    }

    impl<'a> MockIn<'a> {
        fn new(reset: &'a BotReset, reset_after: Option<usize>) -> Self {
            Self {
                info: ep_info(0x81, EP_CHUNK as u16),
                written: Vec::new(),
                packets: Vec::new(),
                reset_after,
                reset,
            }
        }

        /// Every Command Status Wrapper in the stream, as `(tag, residue, status)`.
        fn csws(&self) -> Vec<(u32, u32, u8)> {
            let mut out = Vec::new();
            let mut off = 0;
            for &n in &self.packets {
                let p = &self.written[off..off + n];
                off += n;
                if n != CSW_LEN || u32::from_le_bytes([p[0], p[1], p[2], p[3]]) != CSW_SIGNATURE {
                    continue;
                }
                out.push((
                    u32::from_le_bytes([p[4], p[5], p[6], p[7]]),
                    u32::from_le_bytes([p[8], p[9], p[10], p[11]]),
                    p[12],
                ));
            }
            out
        }
    }

    impl Endpoint for MockIn<'_> {
        fn info(&self) -> &EndpointInfo {
            &self.info
        }
        async fn wait_enabled(&mut self) {
            unreachable!("only the OUT endpoint parks on the interface coming back")
        }
    }

    impl EndpointIn for MockIn<'_> {
        async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
            self.written.extend_from_slice(data);
            self.packets.push(data.len());
            if self.reset_after == Some(self.packets.len()) {
                self.reset.signal(());
            }
            Ok(())
        }
    }

    /// A bulk endpoint descriptor for a mock; the builder reads one back per allocation.
    fn ep_info(addr: u8, max_packet_size: u16) -> EndpointInfo {
        EndpointInfo {
            addr: EndpointAddress::from(addr),
            ep_type: EndpointType::Bulk,
            max_packet_size,
            interval_ms: 0,
        }
    }

    /// A driver whose endpoints are the mocks above, so the class task runs on the host. Only
    /// endpoint allocation is exercised; the bus and control pipe belong to the USB stack task.
    struct TestDriver<'a> {
        reset: &'a BotReset,
    }

    impl<'a> Driver<'a> for TestDriver<'a> {
        type EndpointOut = MockOut<'a>;
        type EndpointIn = MockIn<'a>;
        type ControlPipe = TestPipe;
        type Bus = TestBus;

        fn alloc_endpoint_out(
            &mut self,
            ep_type: EndpointType,
            _ep_addr: Option<EndpointAddress>,
            max_packet_size: u16,
            _interval_ms: u8,
        ) -> Result<Self::EndpointOut, EndpointAllocError> {
            let mut ep = MockOut::new(self.reset, Vec::new());
            ep.info.ep_type = ep_type;
            ep.info.max_packet_size = max_packet_size;
            Ok(ep)
        }

        fn alloc_endpoint_in(
            &mut self,
            ep_type: EndpointType,
            _ep_addr: Option<EndpointAddress>,
            max_packet_size: u16,
            _interval_ms: u8,
        ) -> Result<Self::EndpointIn, EndpointAllocError> {
            let mut ep = MockIn::new(self.reset, None);
            ep.info.ep_type = ep_type;
            ep.info.max_packet_size = max_packet_size;
            Ok(ep)
        }

        fn start(self, _control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
            (TestBus, TestPipe)
        }
    }

    /// Never reached: the tests drive the class task, never the USB stack that owns the bus.
    struct TestBus;

    impl Bus for TestBus {
        async fn enable(&mut self) {
            unreachable!("the USB stack task is not run")
        }
        async fn disable(&mut self) {
            unreachable!("the USB stack task is not run")
        }
        async fn poll(&mut self) -> Event {
            unreachable!("the USB stack task is not run")
        }
        fn endpoint_set_enabled(&mut self, _ep_addr: EndpointAddress, _enabled: bool) {
            unreachable!("the USB stack task is not run")
        }
        fn endpoint_set_stalled(&mut self, _ep_addr: EndpointAddress, _stalled: bool) {
            unreachable!("the USB stack task is not run")
        }
        fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
            unreachable!("the USB stack task is not run")
        }
        async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
            unreachable!("the USB stack task is not run")
        }
    }

    /// The twin of [`TestBus`] for the control pipe: the handler is called directly instead.
    struct TestPipe;

    impl ControlPipe for TestPipe {
        fn max_packet_size(&self) -> usize {
            unreachable!("the USB stack task is not run")
        }
        async fn setup(&mut self) -> [u8; 8] {
            unreachable!("the USB stack task is not run")
        }
        async fn data_out(
            &mut self,
            _buf: &mut [u8],
            _first: bool,
            _last: bool,
        ) -> Result<usize, EndpointError> {
            unreachable!("the USB stack task is not run")
        }
        async fn data_in(
            &mut self,
            _data: &[u8],
            _first: bool,
            _last: bool,
        ) -> Result<(), EndpointError> {
            unreachable!("the USB stack task is not run")
        }
        async fn accept(&mut self) {
            unreachable!("the USB stack task is not run")
        }
        async fn reject(&mut self) {
            unreachable!("the USB stack task is not run")
        }
        async fn accept_set_address(&mut self, _addr: u8) {
            unreachable!("the USB stack task is not run")
        }
    }

    /// A one-sector RAM disk recording every data-phase edge the transport publishes.
    struct MockIo {
        phases: [bool; 4],
        edges: usize,
        sector: [u8; MSC_SECTOR_SIZE],
    }

    impl MockIo {
        fn new() -> Self {
            Self {
                phases: [false; 4],
                edges: 0,
                sector: [0u8; MSC_SECTOR_SIZE],
            }
        }
    }

    impl SectorIo for MockIo {
        fn media_ready(&self) -> bool {
            true
        }
        fn read(&mut self, _lba: u32, buf: &mut [u8; 512]) -> SectorPoll {
            *buf = self.sector;
            SectorPoll::Ready
        }
        fn write(&mut self, _lba: u32, buf: &[u8; 512]) -> SectorPoll {
            self.sector = *buf;
            SectorPoll::Ready
        }
        fn set_data_phase(&mut self, open: bool) {
            self.phases[self.edges] = open;
            self.edges += 1;
        }
    }

    /// A sink that asks for a bus-level re-enumeration on every remount.
    struct ReconnectSink;

    impl TransferSink for ReconnectSink {
        fn begin(&mut self, _fmt: Format, _sniffed: bool) {}
        fn fragment(&mut self, _addr: u32, _data: &[u8]) -> Result<(), DecodeError> {
            Ok(())
        }
        fn end(&mut self, _outcome: TransferOutcome) {}
        fn wants_reconnect(&self) -> bool {
            true
        }
    }

    /// Records each bus-level detach and attach edge with the tick it happened at.
    struct RecordingReconnect<'a> {
        edges: &'a RefCell<Vec<(&'static str, u64)>>,
    }

    impl UsbReconnect for RecordingReconnect<'_> {
        fn detach(&mut self) {
            self.edges
                .borrow_mut()
                .push(("detach", Instant::now().as_ticks()));
        }
        fn attach(&mut self) {
            self.edges
                .borrow_mut()
                .push(("attach", Instant::now().as_ticks()));
        }
    }

    /// Serialise a Command Block Wrapper the way a host puts one on the wire.
    fn cbw_packet(tag: u32, data_len: u32, dir_in: bool, cb: &[u8]) -> [u8; CBW_LEN] {
        let mut p = [0u8; CBW_LEN];
        p[..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
        p[4..8].copy_from_slice(&tag.to_le_bytes());
        p[8..12].copy_from_slice(&data_len.to_le_bytes());
        p[12] = if dir_in { 0x80 } else { 0 };
        p[14] = cb.len() as u8;
        p[15..15 + cb.len()].copy_from_slice(cb);
        p
    }

    /// CDB for READ(10) or WRITE(10) of `count` sectors from `lba`.
    fn rw10(op: u8, lba: u32, count: u16) -> [u8; 10] {
        let mut cb = [0u8; 10];
        cb[0] = op;
        cb[2..6].copy_from_slice(&lba.to_be_bytes());
        cb[7..9].copy_from_slice(&count.to_be_bytes());
        cb
    }

    /// A TEST UNIT READY command wrapper: no data phase, so the CSW is the whole answer.
    fn tur(tag: u32) -> [u8; CBW_LEN] {
        cbw_packet(tag, 0, false, &[0x00])
    }

    /// The endpoint pair the class task drives, over the scripted host `steps`.
    fn endpoints(
        reset: &BotReset,
        steps: Vec<OutStep>,
    ) -> MscEndpoints<'_, TestDriver<'_>, EP_CHUNK> {
        MscEndpoints {
            ep_in: MockIn::new(reset, None),
            ep_out: MockOut::new(reset, steps),
            reset,
        }
    }

    /// Run `steps` past a freshly mounted volume and report the Command Status Wrappers it earned.
    fn csws_for(steps: Vec<OutStep>) -> Vec<(u32, u32, u8)> {
        let reset = BotReset::new();
        let mut eps = endpoints(&reset, steps);
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drive(msc_task(&mut eps, &mut vfs), 1, 0);
        eps.ep_in.csws()
    }

    /// One well-formed 512-byte UF2 block carrying `payload` at `addr`.
    fn uf2_block(addr: u32, payload: &[u8]) -> [u8; 512] {
        let mut b = [0u8; 512];
        b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        b[12..16].copy_from_slice(&addr.to_le_bytes());
        b[16..20].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        b[32..32 + payload.len()].copy_from_slice(payload);
        b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        b
    }

    /// Drop a sized image on the volume so it detaches, with the remount countdown armed.
    fn drop_image_and_detach<S: TransferSink, O: VfsObserver>(vfs: &mut Vfs<'_, S, O>) {
        let mut root = [0u8; 512];
        // One visible 512-byte APP.UF2 root-directory entry, allocated at cluster 2.
        root[0..11].copy_from_slice(b"APP     UF2");
        root[11] = 0x20;
        root[26..28].copy_from_slice(&2u16.to_le_bytes());
        root[28..32].copy_from_slice(&512u32.to_le_bytes());
        vfs.write_sector(131, &root);
        vfs.write_sector(fat::DATA_START_LBA, &uf2_block(0x0800_0000, &[1, 2, 3, 4]));
        assert_eq!(vfs.tick(1), MediaState::Detached);
    }

    /// A class request aimed at the mass-storage interface.
    fn class_request(request: u8, index: u16) -> Request {
        Request {
            direction: embassy_usb::driver::Direction::In,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request,
            value: 0,
            index,
            length: 1,
        }
    }

    /// The handler as the builder leaves it, registered on interface `iface`.
    fn handler_on(reset: &BotReset, iface: u8) -> MscControlHandler<'_> {
        let mut h = MscControlHandler::new(reset);
        h.iface = Some(iface);
        h
    }

    #[test]
    fn a_reset_abandons_the_write_data_phase_before_the_next_cbw() {
        let reset = BotReset::new();
        // The host resets one packet into the payload, then sends the command it expects served.
        let payload = [0xAAu8; EP_CHUNK];
        let mut ep_out = MockOut::new(&reset, vec![pkt(&payload), pkt(&tur(2))]);
        ep_out.reset_after = Some(1);
        let mut ep_in = MockIn::new(&reset, None);
        let mut io = MockIo::new();
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        let mut sense = SENSE_NONE;
        let mut ua_pending = true;
        let cbw = parse_cbw(&cbw_packet(1, 512, false, &rw10(0x2A, 0, 1))).expect("valid CBW");

        block_on(run_command::<_, _, _, EP_CHUNK>(
            &mut In {
                ep: &mut ep_in,
                reset: &reset,
            },
            &mut Out {
                ep: &mut ep_out,
                reset: &reset,
            },
            &mut io,
            &mut sector,
            &mut sense,
            &mut ua_pending,
            &cbw,
        ));

        assert_eq!(
            ep_out.idx, 1,
            "the next CBW stays on the endpoint, unread as data"
        );
        assert!(
            ep_in.written.is_empty(),
            "an abandoned command answers no CSW"
        );
        assert_eq!(
            &io.phases[..io.edges],
            &[true, false],
            "the data phase closes"
        );
        assert_eq!(sense, SENSE_NONE, "a reset leaves sense standing");
        assert!(
            ua_pending,
            "a reset is not a remount: UNIT ATTENTION stands"
        );
    }

    #[test]
    fn without_a_reset_the_next_cbw_is_swallowed_as_write_data() {
        let reset = BotReset::new();
        let payload = [0xAAu8; EP_CHUNK];
        let mut ep_out = MockOut::new(&reset, vec![pkt(&payload), pkt(&tur(2))]);
        let mut ep_in = MockIn::new(&reset, None);
        let mut io = MockIo::new();
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        let mut sense = SENSE_NONE;
        let mut ua_pending = false;
        let cbw = parse_cbw(&cbw_packet(1, 512, false, &rw10(0x2A, 0, 1))).expect("valid CBW");

        block_on(run_command::<_, _, _, EP_CHUNK>(
            &mut In {
                ep: &mut ep_in,
                reset: &reset,
            },
            &mut Out {
                ep: &mut ep_out,
                reset: &reset,
            },
            &mut io,
            &mut sector,
            &mut sense,
            &mut ua_pending,
            &cbw,
        ));

        // The declared payload still owes 448 bytes, so the CBW is consumed as part of it.
        assert_eq!(ep_out.idx, 2, "the CBW went into the sector buffer");
    }

    #[test]
    fn a_reset_abandons_the_read_data_phase() {
        let reset = BotReset::new();
        let mut ep_out = MockOut::new(&reset, Vec::new());
        let mut ep_in = MockIn::new(&reset, Some(1));
        let mut io = MockIo::new();
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        let mut sense = SENSE_NONE;
        let mut ua_pending = false;
        let cbw = parse_cbw(&cbw_packet(1, 512, true, &rw10(0x28, 0, 1))).expect("valid CBW");

        block_on(run_command::<_, _, _, EP_CHUNK>(
            &mut In {
                ep: &mut ep_in,
                reset: &reset,
            },
            &mut Out {
                ep: &mut ep_out,
                reset: &reset,
            },
            &mut io,
            &mut sector,
            &mut sense,
            &mut ua_pending,
            &cbw,
        ));

        assert_eq!(
            ep_in.written.len(),
            EP_CHUNK,
            "one packet went out before the reset, and no CSW after it"
        );
    }

    #[test]
    fn get_max_lun_reports_a_single_logical_unit() {
        // BOT 1.0 section 3.2: one byte, the highest LUN number the device supports.
        let reset = BotReset::new();
        let mut handler = handler_on(&reset, 3);
        let mut buf = [0xFFu8; 8];
        match handler.control_in(class_request(MSC_GET_MAX_LUN, 3), &mut buf) {
            Some(InResponse::Accepted(data)) => assert_eq!(data, &[0]),
            other => panic!("expected one accepted byte, got {other:?}"),
        }
    }

    #[test]
    fn the_class_requests_are_answered_only_for_this_interface() {
        // BOT 1.0 sections 3.1 and 3.2: class requests to this interface, and nothing else.
        let reset = BotReset::new();
        let mut handler = handler_on(&reset, 3);
        let mut buf = [0u8; 8];
        assert!(
            handler
                .control_in(class_request(MSC_GET_MAX_LUN, 4), &mut buf)
                .is_none(),
            "another interface's request is not ours to answer"
        );

        let mut standard = class_request(MSC_GET_MAX_LUN, 3);
        standard.request_type = RequestType::Standard;
        assert!(handler.control_in(standard, &mut buf).is_none());

        let mut to_device = class_request(MSC_GET_MAX_LUN, 3);
        to_device.recipient = Recipient::Device;
        assert!(handler.control_in(to_device, &mut buf).is_none());

        assert!(handler
            .control_in(class_request(0xFD, 3), &mut buf)
            .is_none());

        // Before registration the interface number is unknown, so nothing is claimed.
        let mut unregistered = MscControlHandler::new(&reset);
        assert!(unregistered
            .control_in(class_request(MSC_GET_MAX_LUN, 0), &mut buf)
            .is_none());
    }

    #[test]
    fn a_bulk_only_reset_is_raised_for_the_class_task_and_accepted() {
        // BOT 1.0 section 3.1: the device readies itself for the next CBW and returns success.
        let reset = BotReset::new();
        let mut handler = handler_on(&reset, 3);
        let mut req = class_request(MSC_BULK_ONLY_RESET, 3);
        req.length = 0;
        assert!(matches!(
            handler.control_out(req, &[]),
            Some(OutResponse::Accepted)
        ));
        assert!(reset.signaled());
    }

    #[test]
    fn a_reset_for_another_interface_is_not_raised() {
        let reset = BotReset::new();
        let mut handler = handler_on(&reset, 3);
        assert!(handler
            .control_out(class_request(MSC_BULK_ONLY_RESET, 4), &[])
            .is_none());
        assert!(handler
            .control_out(class_request(MSC_GET_MAX_LUN, 3), &[])
            .is_none());
        assert!(!reset.signaled(), "no class task reset was raised");
    }

    #[test]
    fn the_class_registers_the_bulk_only_pipe_pair() {
        // BOT 1.0 section 3: one bulk IN and one bulk OUT, on an 08h/06h/50h interface.
        let reset = BotReset::new();
        let mut handler = MscControlHandler::new(&reset);
        let mut config_desc = [0u8; 256];
        let mut bos_desc = [0u8; 96];
        let mut msos_desc = [0u8; 0];
        let mut control_buf = [0u8; 128];
        {
            let mut builder = Builder::new(
                TestDriver { reset: &reset },
                Config::new(0x1234, 0x5678),
                &mut config_desc,
                &mut bos_desc,
                &mut msos_desc,
                &mut control_buf,
            );
            let _eps = add_msc_function::<_, EP_CHUNK>(&mut builder, &mut handler);
        }
        let at = config_desc
            .windows(9)
            .position(|w| w[0] == 9 && w[1] == 0x04)
            .expect("an interface descriptor was written");
        let iface = &config_desc[at..];
        assert_eq!((iface[5], iface[6], iface[7]), MSC_INTERFACE_CLASS);
        assert_eq!(iface[4], 2, "one bulk pipe pair and nothing else");
        for (ep, dir_in) in [(&iface[9..16], false), (&iface[16..23], true)] {
            assert_eq!((ep[0], ep[1]), (7, 0x05), "an endpoint descriptor");
            assert_eq!(ep[3] & 0x03, 0x02, "bulk");
            assert_eq!(ep[2] & 0x80 != 0, dir_in, "OUT is declared before IN");
            assert_eq!(u16::from_le_bytes([ep[4], ep[5]]), EP_CHUNK as u16);
        }
    }

    #[test]
    fn a_valid_cbw_is_answered_with_a_csw_carrying_its_tag() {
        assert_eq!(
            csws_for(vec![pkt(&tur(0x1234_5678)), OutStep::Park]),
            [(0x1234_5678, 0, CSW_PASSED)]
        );
    }

    #[test]
    fn a_packet_that_is_not_31_bytes_long_is_not_a_cbw() {
        let mut short = tur(1).to_vec();
        short.truncate(CBW_LEN - 1);
        assert!(csws_for(vec![OutStep::Packet(short), OutStep::Park]).is_empty());

        let mut long = tur(1).to_vec();
        long.resize(EP_CHUNK, 0);
        assert!(csws_for(vec![OutStep::Packet(long), OutStep::Park]).is_empty());
    }

    #[test]
    fn a_cbw_with_a_foreign_signature_is_not_a_cbw() {
        let mut bad = tur(1);
        bad[0] ^= 0xFF;
        assert!(csws_for(vec![pkt(&bad), OutStep::Park]).is_empty());
    }

    #[test]
    fn an_invalid_cbw_is_met_with_silence_where_the_spec_asks_for_a_stall() {
        // BOT 1.0 section 6.6.1 wants both bulk pipes stalled until a Bulk-Only Reset. Stalling
        // belongs to the USB stack task, not this one, so the command after it is served instead.
        let mut bad = tur(1);
        bad[0] ^= 0xFF;
        assert_eq!(
            csws_for(vec![pkt(&bad), pkt(&tur(9)), OutStep::Park]),
            [(9, 0, CSW_PASSED)]
        );
    }

    #[test]
    fn a_cbw_for_a_non_zero_lun_is_refused_rather_than_served_as_lun_0() {
        // GET MAX LUN reports 0, so no other unit exists; 0x10 must not be masked down to it.
        assert_eq!(
            csws_for(vec![pkt(&tur(1)), OutStep::Park]),
            [(1, 0, CSW_PASSED)]
        );
        for lun in [1u8, 0x10] {
            let mut c = tur(2);
            c[13] = lun;
            assert_eq!(
                csws_for(vec![pkt(&c), OutStep::Park]),
                [(2, 0, CSW_FAILED)],
                "LUN {lun:#04x}"
            );
        }
    }

    #[test]
    fn a_reset_raised_between_commands_does_not_truncate_the_next_one() {
        // BOT 1.0 section 3.1: after the reset the device is ready for the next CBW.
        let reset = BotReset::new();
        reset.signal(());
        let mut eps = endpoints(&reset, vec![pkt(&tur(7)), OutStep::Park]);
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drive(msc_task(&mut eps, &mut vfs), 2, 0);
        assert_eq!(eps.ep_in.csws(), [(7, 0, CSW_PASSED)]);
    }

    #[test]
    fn a_disabled_endpoint_parks_the_task_until_the_interface_is_enabled_again() {
        let reset = BotReset::new();
        let mut eps = endpoints(&reset, vec![OutStep::Disabled, pkt(&tur(3)), OutStep::Park]);
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drive(msc_task(&mut eps, &mut vfs), 2, 0);
        assert_eq!(eps.ep_out.enables, 1, "the task waited for the interface");
        assert_eq!(eps.ep_in.csws(), [(3, 0, CSW_PASSED)]);
    }

    #[test]
    fn turns_of_the_loop_alone_never_advance_the_remount() {
        let _clock = clock_guard();
        let reset = BotReset::new();
        let mut steps: Vec<OutStep> = (1..=200).map(|tag| pkt(&tur(tag))).collect();
        steps.push(OutStep::Park);
        let mut eps = endpoints(&reset, steps);
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drop_image_and_detach(&mut vfs);
        // Two hundred commands with the clock stopped: the volume is still off the air for all
        // of them, so no turn of the loop can have credited the remount countdown any time.
        drive(msc_task(&mut eps, &mut vfs), 1, 0);
        let csws = eps.ep_in.csws();
        assert_eq!(csws.len(), 200);
        assert!(csws.iter().all(|&(_, _, status)| status == CSW_FAILED));
    }

    #[test]
    fn the_volume_stays_detached_short_of_the_remount_delay() {
        let _clock = clock_guard();
        let reset = BotReset::new();
        // Twenty idle turns of 100 ms, short of the 2500 ms the volume asked to stay down for.
        let mut steps: Vec<OutStep> = (0..20).map(|_| OutStep::Idle).collect();
        steps.push(pkt(&tur(1)));
        steps.push(pkt(&tur(2)));
        steps.push(OutStep::Park);
        let mut eps = endpoints(&reset, steps);
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drop_image_and_detach(&mut vfs);
        drive(msc_task(&mut eps, &mut vfs), 21, 100);
        // Both fail: a remount would have spent its one UNIT ATTENTION on the first and passed
        // the second, so a pair of NOT READY answers is what still-detached looks like.
        assert_eq!(eps.ep_in.csws(), [(1, 0, CSW_FAILED), (2, 0, CSW_FAILED)]);
    }

    #[test]
    fn the_volume_remounts_once_the_remount_delay_of_wall_time_has_passed() {
        let _clock = clock_guard();
        let reset = BotReset::new();
        // Thirty idle turns of 100 ms carry the volume past the 2500 ms remount countdown.
        let mut steps: Vec<OutStep> = (0..30).map(|_| OutStep::Idle).collect();
        steps.push(pkt(&tur(1)));
        steps.push(pkt(&tur(2)));
        steps.push(OutStep::Park);
        let mut eps = endpoints(&reset, steps);
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drop_image_and_detach(&mut vfs);
        drive(msc_task(&mut eps, &mut vfs), 32, 100);
        // The remount's UNIT ATTENTION is reported once, and the medium is back behind it.
        assert_eq!(eps.ep_in.csws(), [(1, 0, CSW_FAILED), (2, 0, CSW_PASSED)]);
    }

    #[test]
    fn a_remount_that_wants_a_reconnect_holds_the_bus_down_before_reattaching() {
        let _clock = clock_guard();
        let reset = BotReset::new();
        let edges = RefCell::new(Vec::new());
        let steps: Vec<OutStep> = (0..60).map(|_| OutStep::Idle).collect();
        let mut eps = endpoints(&reset, steps);
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            ReconnectSink,
            NoObserver,
        );
        drop_image_and_detach(&mut vfs);
        drive(
            msc_task_with_reconnect(&mut eps, &mut vfs, RecordingReconnect { edges: &edges }),
            60,
            100,
        );
        let edges = edges.into_inner();
        assert_eq!(
            edges.iter().map(|e| e.0).collect::<Vec<_>>(),
            ["detach", "attach"],
            "one detach, one attach, in that order"
        );
        let down_ms = Duration::from_ticks(edges[1].1 - edges[0].1).as_millis();
        assert!(
            down_ms >= USB_RECONNECT_DETACH_MS,
            "the bus stayed down {down_ms} ms"
        );
    }
}
