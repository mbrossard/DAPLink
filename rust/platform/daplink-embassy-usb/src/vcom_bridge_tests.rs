//! The whole VCOM bridge over mock endpoints and a mock seam: port closes and line changes.

use super::*;
use core::future::{poll_fn, Future};
use core::pin::Pin;
use core::task::{Context, Poll, Waker};
use embassy_usb::driver::{
    Bus, ControlPipe, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event,
    Unsupported,
};
use std::boxed::Box;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::MutexGuard;
use std::vec;
use std::vec::Vec;

/// The receive ring of the RP and H5 seams.
const RING: usize = 2048;
const PACKET: usize = CDC_PACKET_SIZE as usize;
/// The packet a device controller holds armed for the next IN token; no class can recall it.
const ARMED: usize = PACKET;
/// What `host_half` holds once a send parks: the rest of the chunk it took from the pipe.
const IN_HAND: usize = DRAIN_BYTES - ARMED;
const POLLS_PER_MS: usize = 64;

const REQ_SET_LINE_CODING: u8 = 0x20;
const REQ_GET_LINE_CODING: u8 = 0x21;
const REQ_SET_CONTROL_LINE_STATE: u8 = 0x22;

/// The host side of the wire.
struct Host {
    /// Cleared, the endpoints fail as a reset bus fails them.
    enabled: bool,
    /// Whether IN transfers are submitted: an open port whose reader keeps up.
    reading: bool,
    in_per_ms: usize,
    in_budget: usize,
    /// Return-leg bytes, one entry per opener.
    sessions: Vec<Vec<u8>>,
    /// A packet the device armed while no IN transfer was submitted; the next reader takes it.
    armed: Option<Vec<u8>>,
    out: VecDeque<Vec<u8>>,
    setups: VecDeque<([u8; 8], Vec<u8>)>,
    data: Vec<u8>,
    bus_reset: bool,
    /// The status stage of each control OUT: accepted or refused.
    statuses: Vec<bool>,
    /// The data stage of each control IN.
    replies: Vec<Vec<u8>>,
}

type HostRef = Rc<RefCell<Host>>;

struct WireEp {
    info: EndpointInfo,
    host: HostRef,
}

impl Endpoint for WireEp {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        poll_fn(|_| match self.host.borrow().enabled {
            true => Poll::Ready(()),
            false => Poll::Pending,
        })
        .await
    }
}

impl EndpointIn for WireEp {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        poll_fn(|_| {
            let mut host = self.host.borrow_mut();
            if !host.enabled {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if host.armed.is_some() {
                return Poll::Pending;
            }
            if !host.reading {
                host.armed = Some(buf.to_vec());
                return Poll::Ready(Ok(()));
            }
            if host.in_budget == 0 {
                return Poll::Pending;
            }
            host.in_budget -= 1;
            host.sessions
                .last_mut()
                .expect("opened")
                .extend_from_slice(buf);
            Poll::Ready(Ok(()))
        })
        .await
    }
}

impl EndpointOut for WireEp {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        poll_fn(|_| {
            let mut host = self.host.borrow_mut();
            if !host.enabled {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            match host.out.pop_front() {
                Some(packet) => {
                    buf[..packet.len()].copy_from_slice(&packet);
                    Poll::Ready(Ok(packet.len()))
                }
                None => Poll::Pending,
            }
        })
        .await
    }
}

struct WireBus(HostRef);

impl Bus for WireBus {
    async fn enable(&mut self) {}
    async fn disable(&mut self) {}
    async fn poll(&mut self) -> Event {
        poll_fn(
            |_| match core::mem::take(&mut self.0.borrow_mut().bus_reset) {
                true => Poll::Ready(Event::Reset),
                false => Poll::Pending,
            },
        )
        .await
    }
    fn endpoint_set_enabled(&mut self, _: EndpointAddress, _: bool) {}
    fn endpoint_set_stalled(&mut self, _: EndpointAddress, _: bool) {}
    fn endpoint_is_stalled(&mut self, _: EndpointAddress) -> bool {
        false
    }
    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

struct WirePipe(HostRef);

impl ControlPipe for WirePipe {
    fn max_packet_size(&self) -> usize {
        64
    }
    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|_| {
            let mut host = self.0.borrow_mut();
            match host.setups.pop_front() {
                Some((setup, data)) => {
                    host.data = data;
                    Poll::Ready(setup)
                }
                None => Poll::Pending,
            }
        })
        .await
    }
    async fn data_out(&mut self, buf: &mut [u8], _: bool, _: bool) -> Result<usize, EndpointError> {
        let host = self.0.borrow();
        buf[..host.data.len()].copy_from_slice(&host.data);
        Ok(host.data.len())
    }
    async fn data_in(&mut self, data: &[u8], _: bool, _: bool) -> Result<(), EndpointError> {
        self.0.borrow_mut().replies.push(data.to_vec());
        Ok(())
    }
    async fn accept(&mut self) {
        self.0.borrow_mut().statuses.push(true);
    }
    async fn reject(&mut self) {
        self.0.borrow_mut().statuses.push(false);
    }
    async fn accept_set_address(&mut self, _: u8) {}
}

struct WireDriver {
    host: HostRef,
    next: u8,
}

impl WireDriver {
    fn ep(&mut self, ep_type: EndpointType, dir: u8, max_packet_size: u16) -> WireEp {
        self.next += 1;
        WireEp {
            info: EndpointInfo {
                addr: EndpointAddress::from(dir | self.next),
                ep_type,
                max_packet_size,
                interval_ms: 0,
            },
            host: self.host.clone(),
        }
    }
}

impl Driver<'static> for WireDriver {
    type EndpointOut = WireEp;
    type EndpointIn = WireEp;
    type ControlPipe = WirePipe;
    type Bus = WireBus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        _: Option<EndpointAddress>,
        max_packet_size: u16,
        _: u8,
    ) -> Result<WireEp, EndpointAllocError> {
        Ok(self.ep(ep_type, 0x00, max_packet_size))
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        _: Option<EndpointAddress>,
        max_packet_size: u16,
        _: u8,
    ) -> Result<WireEp, EndpointAllocError> {
        Ok(self.ep(ep_type, 0x80, max_packet_size))
    }

    fn start(self, _: u16) -> (WireBus, WirePipe) {
        (WireBus(self.host.clone()), WirePipe(self.host))
    }
}

/// The seam: a ring of old-rate bytes that a divisor change restarts, as every embassy seam does.
#[derive(Default)]
struct Seam {
    ring: VecDeque<u8>,
    ring_full: usize,
    /// Bytes a divisor change's ring restart threw away.
    restarted: usize,
    per_ms: usize,
    tag: u8,
    applied: Vec<u32>,
    tx: Vec<u8>,
}

impl Seam {
    fn produce(&mut self, n: usize) {
        for _ in 0..n {
            if self.ring.len() < RING {
                self.ring.push_back(self.tag);
            } else {
                self.ring_full += 1;
            }
        }
    }

    fn take(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.ring.len());
        for (slot, byte) in buf[..n].iter_mut().zip(self.ring.drain(..n)) {
            *slot = byte;
        }
        n
    }
}

struct SeamUart(Rc<RefCell<Seam>>);

impl VcomUart for SeamUart {
    const SEAM_VALID: u32 = 0;
    const SEAM_GRADE: u32 = 0;
    const SEAM_UNITS: u32 = 0;

    async fn read(&mut self, buf: &mut [u8]) -> usize {
        poll_fn(|_| match self.0.borrow_mut().take(buf) {
            0 => Poll::Pending,
            n => Poll::Ready(n),
        })
        .await
    }

    fn read_ready(&mut self, buf: &mut [u8]) -> usize {
        self.0.borrow_mut().take(buf)
    }

    fn seam_stats(&self) -> SeamStats {
        SeamStats::default()
    }

    async fn write_all(&mut self, data: &[u8]) {
        self.0.borrow_mut().tx.extend_from_slice(data);
    }

    async fn drain_tx(&mut self) {}

    fn halt_rx(&mut self) {}

    fn set_baudrate(&mut self, baud: u32) -> BaudApply {
        if baud == 0 {
            return BaudApply::Rejected;
        }
        let mut seam = self.0.borrow_mut();
        seam.restarted += seam.ring.len();
        seam.ring.clear();
        seam.applied.push(baud);
        BaudApply::Applied
    }
}

/// One bridge and its USB device, stepped a millisecond at a time on the mock clock.
struct Rig {
    host: HostRef,
    seam: Rc<RefCell<Seam>>,
    run: Pin<Box<dyn Future<Output = ()>>>,
    _locks: (MutexGuard<'static, ()>, MutexGuard<'static, ()>),
}

impl Rig {
    fn new() -> Self {
        let vcom = crate::tests::VCOM_TESTS
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let clock = crate::msc::tests::clock_guard();
        let host = Rc::new(RefCell::new(Host {
            enabled: true,
            reading: false,
            in_per_ms: usize::MAX,
            in_budget: 0,
            sessions: Vec::new(),
            armed: None,
            out: VecDeque::new(),
            setups: VecDeque::new(),
            data: Vec::new(),
            bus_reset: false,
            statuses: Vec::new(),
            replies: Vec::new(),
        }));
        let seam = Rc::new(RefCell::new(Seam::default()));
        let driver = WireDriver {
            host: host.clone(),
            next: 0,
        };
        let mut builder = Builder::new(
            driver,
            Config::new(0x1209, 0x0001),
            Box::leak(Box::new([0u8; 256])),
            Box::leak(Box::new([0u8; 64])),
            Box::leak(Box::new([0u8; 0])),
            Box::leak(Box::new([0u8; 64])),
        );
        builder.handler(Box::leak(Box::new(LineEvents { dtr: false })));
        let state = Box::leak(Box::new(State::new()));
        let cdc = CdcAcmClass::new(&mut builder, state, CDC_PACKET_SIZE);
        let (mut tx, mut rx, control) = cdc.split_with_control();
        let mut usb = builder.build();
        let mut uart = SeamUart(seam.clone());
        let run = Box::pin(async move {
            let mut led = ();
            let bridge = vcom_bridge(&mut tx, &mut rx, &control, &mut uart, 9_600, &mut led);
            let _ = join(usb.run(), bridge).await;
        });
        let rig = Self {
            host,
            seam,
            run,
            _locks: (vcom, clock),
        };
        rig.enumerate();
        rig
    }

    /// The line coding a Linux host sends at probe, before any open; it replaces the class sentinel.
    fn enumerate(&self) {
        self.set_baud(9_600);
    }

    fn step(&mut self, ms: usize) {
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..ms {
            {
                let mut host = self.host.borrow_mut();
                host.in_budget = host.in_per_ms;
                if host.reading {
                    if let Some(packet) = host.armed.take() {
                        host.in_budget = host.in_budget.saturating_sub(1);
                        host.sessions.last_mut().expect("opened").extend(packet);
                    }
                }
            }
            {
                let mut seam = self.seam.borrow_mut();
                let n = seam.per_ms;
                seam.produce(n);
            }
            for _ in 0..POLLS_PER_MS {
                let _ = self.run.as_mut().poll(&mut cx);
            }
            embassy_time::MockDriver::get().advance(Duration::from_millis(1));
        }
    }

    fn class_request(&self, request: u8, value: u16, data: Vec<u8>) {
        let [v0, v1] = value.to_le_bytes();
        let setup = [0x21, request, v0, v1, 0, 0, data.len() as u8, 0];
        self.host.borrow_mut().setups.push_back((setup, data));
    }

    /// Open as a tty does: DTR and RTS up, IN transfers submitted.
    fn open(&self) {
        self.class_request(REQ_SET_CONTROL_LINE_STATE, 0x3, Vec::new());
        let mut host = self.host.borrow_mut();
        host.sessions.push(Vec::new());
        host.reading = true;
    }

    /// Close as a tty does: DTR drops, then the IN transfers are killed.
    fn close(&self) {
        self.class_request(REQ_SET_CONTROL_LINE_STATE, 0, Vec::new());
        self.host.borrow_mut().reading = false;
    }

    fn set_baud(&self, baud: u32) {
        self.set_line_coding(baud, [0, 0, 8]);
    }

    fn set_line_coding(&self, baud: u32, format: [u8; 3]) {
        let mut coding = baud.to_le_bytes().to_vec();
        coding.extend_from_slice(&format);
        self.class_request(REQ_SET_LINE_CODING, 0, coding);
    }

    /// GET_LINE_CODING, a class IN request of seven bytes.
    fn get_line_coding(&self) {
        let setup = [0xA1, REQ_GET_LINE_CODING, 0, 0, 0, 0, 7, 0];
        self.host.borrow_mut().setups.push_back((setup, Vec::new()));
    }

    fn last_status(&self) -> Option<bool> {
        self.host.borrow().statuses.last().copied()
    }

    fn last_reply(&self) -> Vec<u8> {
        self.host
            .borrow()
            .replies
            .last()
            .cloned()
            .unwrap_or_default()
    }

    /// Produce `total` bytes at `per_ms`, as a target writing at that rate would.
    fn trickle(&mut self, tag: u8, total: usize, per_ms: usize) {
        {
            let mut seam = self.seam.borrow_mut();
            seam.tag = tag;
            seam.per_ms = per_ms;
        }
        self.step(total / per_ms);
        self.seam.borrow_mut().per_ms = 0;
    }

    fn produce(&self, tag: u8, n: usize) {
        let mut seam = self.seam.borrow_mut();
        seam.tag = tag;
        seam.produce(n);
    }

    fn session(&self, i: usize) -> Vec<u8> {
        self.host.borrow().sessions[i].clone()
    }

    fn applied(&self) -> Vec<u32> {
        self.seam.borrow().applied.clone()
    }

    /// The bus resets: endpoints fail until re-enabled, and the armed packet is lost.
    fn bus_reset(&self) {
        let mut host = self.host.borrow_mut();
        host.enabled = false;
        host.bus_reset = true;
        host.armed = None;
    }

    fn ring_full(&self) -> usize {
        self.seam.borrow().ring_full
    }
}

fn dropped() -> u32 {
    HOST_DROPPED.load(Ordering::Relaxed)
}

fn ledger() -> [u32; 3] {
    [
        UART_RX.load(Ordering::Relaxed),
        CDC_IN.load(Ordering::Relaxed),
        HOST_DROPPED.load(Ordering::Relaxed),
    ]
}

/// Everything the bridge took off the seam since `base` reached the host or was charged.
fn reconciled(base: [u32; 3]) -> bool {
    let [rx, cdc_in, dropped] = ledger();
    rx.wrapping_sub(base[0]) == cdc_in.wrapping_sub(base[1]) + dropped.wrapping_sub(base[2])
}

/// A close drops and charges what the bridge held; only the armed packet reaches the next opener.
#[test]
fn a_dtr_fall_drops_what_the_closing_session_left() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    // The reader stops keeping up, then the port closes.
    rig.host.borrow_mut().reading = false;
    let base = ledger();
    rig.trickle(b'A', 3000, 500);
    rig.step(20);
    rig.close();
    rig.step(1000);
    rig.open();
    rig.step(20);

    assert!(rig.session(0).is_empty());
    assert_eq!(rig.session(1), vec![b'A'; ARMED]);
    assert_eq!(dropped() - base[2], (RING + RETURN_PIPE + IN_HAND) as u32);
    assert_eq!(
        rig.ring_full(),
        3000 - (ARMED + IN_HAND + RETURN_PIPE + RING)
    );
    assert!(reconciled(base));
}

/// A close and a reopen inside one held line change still end the old session.
#[test]
fn a_close_and_reopen_inside_one_await_drops_the_old_session() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    rig.host.borrow_mut().reading = false;
    let base = ledger();
    rig.trickle(b'A', 3000, 500);
    rig.step(20);
    rig.set_baud(115_200);
    rig.step(5);
    assert!(rig.applied().is_empty());
    rig.close();
    rig.open();
    rig.step(60);

    assert_eq!(rig.applied(), vec![115_200]);
    assert_eq!(rig.session(1).len(), ARMED);
    assert_eq!(dropped() - base[2], (RING + RETURN_PIPE + IN_HAND) as u32);
    assert!(reconciled(base));
}

/// DTR low gates nothing: what the target sends after a close waits for whoever opens next.
#[test]
fn bytes_arriving_after_a_close_reach_the_next_opener() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    let base = ledger();
    rig.produce(b'A', 300);
    rig.step(5);
    assert_eq!(rig.session(0), vec![b'A'; 300]);
    rig.close();
    rig.step(5);
    rig.produce(b'B', 1000);
    rig.step(1000);
    rig.open();
    rig.step(20);

    assert_eq!(rig.session(1), vec![b'B'; 1000]);
    assert_eq!(dropped(), base[2]);
    assert!(reconciled(base));
}

/// A bus reset drops DTR, so it ends the session as a close does, the seam ring included.
#[test]
fn a_bus_reset_drops_everything_the_session_left() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    rig.host.borrow_mut().reading = false;
    let base = ledger();
    rig.trickle(b'A', 3000, 500);
    rig.step(20);
    let held = dropped();
    rig.bus_reset();
    rig.step(20);
    rig.host.borrow_mut().enabled = true;
    rig.enumerate();
    rig.open();
    rig.step(20);

    assert_eq!(dropped() - held, (RING + RETURN_PIPE + IN_HAND) as u32);
    assert!(rig.session(1).is_empty());
    assert!(reconciled(base));
}

/// A bus reset abandons a waiting line change: the new session's rate is the first applied.
#[test]
fn a_bus_reset_during_a_line_change_drops_everything_held() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    rig.host.borrow_mut().reading = false;
    let base = ledger();
    rig.trickle(b'A', 3000, 500);
    rig.step(20);
    rig.set_baud(115_200);
    rig.step(20);
    assert!(rig.applied().is_empty());
    let held = dropped();
    rig.bus_reset();
    rig.step(20);
    rig.host.borrow_mut().enabled = true;
    rig.step(20);
    // Neither the held rate nor the class's reset default reaches the divisor.
    assert!(rig.applied().is_empty());
    assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 0);
    rig.enumerate();
    rig.open();
    rig.set_baud(230_400);
    rig.step(20);

    assert_eq!(dropped() - held, (RING + RETURN_PIPE + IN_HAND) as u32);
    assert!(rig.session(1).is_empty());
    assert!(reconciled(base));
    assert_eq!(rig.applied(), vec![230_400]);
}

/// A host that stops reading holds a line change for the stall bound, then the backlog is charged.
#[test]
fn a_line_change_is_bounded_for_a_host_that_stopped_reading() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    rig.host.borrow_mut().reading = false;
    let base = ledger();
    rig.produce(b'A', 1000);
    rig.step(10);

    // Return leg stalled, no change: host-to-target bytes still flow.
    rig.host.borrow_mut().out.extend([vec![1; 64], vec![2; 64]]);
    rig.step(10);
    assert_eq!(rig.seam.borrow().tx.len(), 128);

    rig.set_baud(115_200);
    rig.step(10);
    rig.host.borrow_mut().out.extend([vec![3; 64], vec![4; 64]]);
    rig.step(30);
    assert!(rig.applied().is_empty());
    assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 115_200);
    let reported = vcom_baud_state(9_600);
    assert_eq!(reported.flags & daplink_rs::VCOM_BAUD_SETTLED, 0);
    assert_eq!(rig.seam.borrow().tx.len(), 128);

    // Past the bound the change lands, the ring residue is charged, and the forward leg moves.
    rig.step(20);
    assert_eq!(rig.applied(), vec![115_200]);
    assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 0);
    assert_eq!(rig.seam.borrow().tx.len(), 256);
    let charged = 1000 - (ARMED + IN_HAND + RETURN_PIPE);
    assert_eq!(dropped() - base[2], charged as u32);
    assert_eq!(rig.seam.borrow().restarted, 0);

    // What the bridge kept reaches the reader once it returns.
    rig.host.borrow_mut().reading = true;
    rig.step(20);
    assert_eq!(rig.session(0), vec![b'A'; 1000 - charged]);
    assert!(reconciled(base));
}

/// `stty -F` shape: a close during a line change lands it at once and drops the old backlog.
#[test]
fn a_close_during_a_line_change_settles_it_for_the_next_opener() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    rig.close();
    rig.step(5);
    let base = ledger();
    rig.trickle(b'A', 3000, 500);
    rig.step(20);

    rig.host.borrow_mut().in_per_ms = 4;
    rig.open();
    rig.set_baud(115_200);
    rig.step(1);
    assert!(rig.applied().is_empty());
    rig.close();
    rig.step(2);
    assert_eq!(rig.applied(), vec![115_200]);
    assert_eq!(PENDING_BAUD.load(Ordering::Relaxed), 0);
    rig.step(1000);

    rig.host.borrow_mut().in_per_ms = usize::MAX;
    rig.open();
    rig.step(20);
    assert_eq!(rig.applied(), vec![115_200]);
    let during = rig.session(1);
    assert_eq!(during, vec![b'A'; ARMED + IN_HAND]);
    // The reader had taken every armed packet, so nothing of the old session is left to deliver.
    assert!(rig.session(2).is_empty());
    let charged = (dropped() - base[2]) as usize;
    assert_eq!(3000, rig.ring_full() + during.len() + charged);
    assert!(reconciled(base));
}

/// A reading host faster than the source drains the old-rate backlog, so the change completes.
#[test]
fn a_line_change_completes_while_the_sink_outpaces_the_source() {
    let mut rig = Rig::new();
    rig.open();
    rig.host.borrow_mut().in_per_ms = 8;
    rig.seam.borrow_mut().per_ms = 100;
    rig.seam.borrow_mut().tag = b'A';
    rig.step(5);
    let base = dropped();
    rig.produce(b'A', 2000);
    rig.set_baud(115_200);
    let mut waited = 0;
    while rig.applied().is_empty() && waited < 1000 {
        rig.step(1);
        waited += 1;
    }
    assert_eq!(rig.applied(), vec![115_200]);
    // About backlog / (sink - source): 2100 B at 412 B/ms.
    assert!(waited <= 10, "took {waited} ms");
    assert_eq!(dropped(), base);
}

/// A source faster than the host reads hits the byte cap, so the change still lands promptly.
#[test]
fn a_line_change_is_bounded_while_the_source_outpaces_the_sink() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(5);
    let base = ledger();
    rig.host.borrow_mut().in_per_ms = 2;
    rig.seam.borrow_mut().per_ms = 200;
    rig.seam.borrow_mut().tag = b'A';
    rig.step(500);
    let overflowed = rig.ring_full();
    rig.set_baud(115_200);
    rig.host.borrow_mut().out.push_back(vec![9; 64]);
    let mut waited = 0;
    while rig.applied().is_empty() && waited < 1000 {
        rig.step(1);
        waited += 1;
    }
    assert_eq!(rig.applied(), vec![115_200]);
    // About the cap over the sink's 128 B/ms.
    assert!(waited <= 30, "took {waited} ms");
    assert!(dropped() > base[2]);
    rig.step(5);
    assert_eq!(rig.seam.borrow().tx, vec![9; 64]);
    // The overload is still counted while the change waits.
    assert!(rig.ring_full() > overflowed);

    rig.seam.borrow_mut().per_ms = 0;
    rig.step(100);
    assert!(reconciled(base));
}

/// 7E1 at a new rate: the framing is refused, the rate followed, and 8N1 reported.
#[test]
fn a_refused_framing_still_moves_the_baud() {
    let mut rig = Rig::new();
    rig.open();
    rig.step(1);
    rig.set_line_coding(230_400, [0, 2, 7]);
    rig.step(2);
    assert_eq!(rig.applied(), vec![230_400]);
    assert_eq!(
        rig.last_status(),
        Some(false),
        "a framing other than 8N1 is refused"
    );
    rig.get_line_coding();
    rig.step(1);
    let mut expected = 230_400u32.to_le_bytes().to_vec();
    expected.extend_from_slice(&[0, 0, 8]);
    assert_eq!(
        rig.last_reply(),
        expected,
        "the record echoes a framing that never ran"
    );
}
