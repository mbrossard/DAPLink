//! Multi-packet framing against mock endpoints: command reassembly, response piecing and ZLPs.

use super::*;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use embassy_futures::block_on;
use embassy_usb::driver::{
    Bus, ControlPipe, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event,
    Unsupported,
};
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec;
use std::vec::Vec;

/// Deterministic xorshift64*; fixed seeds keep every failure reproducible.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }

    fn bytes(&mut self, out: &mut Vec<u8>, n: usize) {
        for _ in 0..n {
            out.push(self.next() as u8);
        }
    }
}

type Packets = Rc<RefCell<VecDeque<Vec<u8>>>>;
type Sent = Rc<RefCell<Vec<Vec<u8>>>>;

struct MockOut {
    info: EndpointInfo,
    packets: Packets,
    /// Park instead of failing once the host has nothing more to send.
    park_when_empty: bool,
}

struct MockIn {
    info: EndpointInfo,
    sent: Sent,
}

fn info(addr: u8, mps: usize) -> EndpointInfo {
    EndpointInfo {
        addr: EndpointAddress::from(addr),
        ep_type: EndpointType::Bulk,
        max_packet_size: mps as u16,
        interval_ms: 0,
    }
}

impl Endpoint for MockOut {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {}
}

impl EndpointOut for MockOut {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        core::future::poll_fn(|_| {
            let Some(packet) = self.packets.borrow_mut().pop_front() else {
                return match self.park_when_empty {
                    true => Poll::Pending,
                    false => Poll::Ready(Err(EndpointError::Disabled)),
                };
            };
            assert!(packet.len() <= usize::from(self.info.max_packet_size));
            if packet.len() > buf.len() {
                return Poll::Ready(Err(EndpointError::BufferOverflow));
            }
            buf[..packet.len()].copy_from_slice(&packet);
            Poll::Ready(Ok(packet.len()))
        })
        .await
    }
}

impl Endpoint for MockIn {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {}
}

impl EndpointIn for MockIn {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        assert!(buf.len() <= usize::from(self.info.max_packet_size));
        self.sent.borrow_mut().push(buf.to_vec());
        Ok(())
    }
}

/// Only the endpoint types are used; the bus and control pipe are never started.
struct MockDriver;
struct NoBus;
struct NoControl;

impl Bus for NoBus {
    async fn enable(&mut self) {}
    async fn disable(&mut self) {}
    async fn poll(&mut self) -> Event {
        unreachable!()
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

impl ControlPipe for NoControl {
    fn max_packet_size(&self) -> usize {
        64
    }
    async fn setup(&mut self) -> [u8; 8] {
        unreachable!()
    }
    async fn data_out(&mut self, _: &mut [u8], _: bool, _: bool) -> Result<usize, EndpointError> {
        unreachable!()
    }
    async fn data_in(&mut self, _: &[u8], _: bool, _: bool) -> Result<(), EndpointError> {
        unreachable!()
    }
    async fn accept(&mut self) {}
    async fn reject(&mut self) {}
    async fn accept_set_address(&mut self, _: u8) {}
}

impl Driver<'static> for MockDriver {
    type EndpointOut = MockOut;
    type EndpointIn = MockIn;
    type ControlPipe = NoControl;
    type Bus = NoBus;

    fn alloc_endpoint_out(
        &mut self,
        _: EndpointType,
        _: Option<EndpointAddress>,
        _: u16,
        _: u8,
    ) -> Result<MockOut, EndpointAllocError> {
        Err(EndpointAllocError)
    }

    fn alloc_endpoint_in(
        &mut self,
        _: EndpointType,
        _: Option<EndpointAddress>,
        _: u16,
        _: u8,
    ) -> Result<MockIn, EndpointAllocError> {
        Err(EndpointAllocError)
    }

    fn start(self, _: u16) -> (NoBus, NoControl) {
        (NoBus, NoControl)
    }
}

fn reader(mps: usize, park_when_empty: bool) -> (DapV2Reader<MockDriver>, Packets) {
    let packets = Packets::default();
    let read_ep = MockOut {
        info: info(0x01, mps),
        packets: packets.clone(),
        park_when_empty,
    };
    (DapV2Reader { read_ep }, packets)
}

fn writer(mps: usize) -> (DapV2Writer<MockDriver>, Sent) {
    let sent = Sent::default();
    let write_ep = MockIn {
        info: info(0x81, mps),
        sent: sent.clone(),
    };
    (DapV2Writer { write_ep }, sent)
}

/// How a host splits one bulk transfer: full packets, then any short remainder, and no ZLP.
fn host_packets(cmd: &[u8], mps: usize) -> impl Iterator<Item = Vec<u8>> + '_ {
    cmd.chunks(mps).map(<[u8]>::to_vec)
}

/// A host reading `p`-byte DAP packets: each read ends on a short packet or a full buffer.
fn host_reads(packets: &[Vec<u8>], mps: usize, p: usize) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut cur: Option<Vec<u8>> = None;
    for packet in packets {
        let buf = cur.get_or_insert_with(Vec::new);
        buf.extend_from_slice(packet);
        assert!(buf.len() <= p, "a response overran the host's DAP packet");
        if packet.len() < mps || buf.len() == p {
            out.push(cur.take().unwrap());
        }
    }
    assert!(
        cur.is_none(),
        "the host is still waiting for the end of a response"
    );
    out
}

/// A valid request of at most `s` bytes, never `DAP_TransferAbort`, over the shapes the reader sizes.
fn command(rng: &mut Rng, s: usize) -> Vec<u8> {
    let mut c = Vec::new();
    match rng.below(8) {
        // DAP_TransferBlock write: header then words.
        0 => {
            let count = rng.below((s - 5) / 4 + 1);
            c.extend_from_slice(&[0x06, 0]);
            c.extend_from_slice(&(count as u16).to_le_bytes());
            c.push(0x0c);
            rng.bytes(&mut c, 4 * count);
        }
        // DAP_TransferBlock read: header only.
        1 => c.extend_from_slice(&[0x06, 0, rng.below(127) as u8, 0, 0x0f]),
        // DAP_Transfer: writes and plain reads.
        2 => {
            c.extend_from_slice(&[0x05, 0, 0]);
            while c.len() + 5 <= s && c[2] < 255 && rng.below(40) != 0 {
                if rng.below(2) == 0 {
                    c.push(0x02);
                } else {
                    c.push(0x01);
                    rng.bytes(&mut c, 4);
                }
                c[2] += 1;
            }
        }
        // DAP_SWJ_Sequence.
        3 => {
            let bits = 1 + rng.below(256);
            c.extend_from_slice(&[0x12, bits as u8]);
            rng.bytes(&mut c, bits.div_ceil(8));
        }
        // The vendor flash stream, including exact packet multiples.
        4 => {
            let n = match rng.below(2) {
                0 => rng.below(256.min(s - 1)),
                _ => [62, 126, 254][rng.below(3)].min(s - 2),
            };
            c.extend_from_slice(&[0x8c, n as u8]);
            rng.bytes(&mut c, n);
        }
        // DAP_ExecuteCommands: fixed commands padded with block writes.
        5 => {
            c.extend_from_slice(&[0x7f, 0]);
            loop {
                let words = rng.below(20);
                if c.len() + 2 + 5 + 4 * words > s || rng.below(6) == 0 {
                    break;
                }
                c.extend_from_slice(&[0x00, 0xfe, 0x06, 0]);
                c.extend_from_slice(&(words as u16).to_le_bytes());
                c.push(0x0c);
                rng.bytes(&mut c, 4 * words);
                c[1] += 2;
            }
        }
        // DAP_JTAG_Sequence.
        6 => {
            c.extend_from_slice(&[0x14, 0]);
            while rng.below(8) != 0 && c.len() + 9 <= s {
                let info = rng.next() as u8;
                c.push(info);
                let bits = match info & 0x3f {
                    0 => 64,
                    n => usize::from(n),
                };
                rng.bytes(&mut c, bits.div_ceil(8));
                c[1] += 1;
            }
        }
        // A short fixed command.
        _ => c.extend_from_slice(&[0x00, 0xff]),
    }
    assert!(c.len() <= s);
    c
}

fn check_reassembly<const S: usize>(mps: usize, seed: u64) {
    let mut rng = Rng(seed);
    for _ in 0..200 {
        let (mut r, packets) = reader(mps, false);
        let mut want = Vec::new();
        for _ in 0..1 + rng.below(12) {
            let c = command(&mut rng, S);
            packets.borrow_mut().extend(host_packets(&c, mps));
            // Some hosts close an exact multiple with a ZLP; it must read as an empty command.
            let zlp = c.len().is_multiple_of(mps) && rng.below(2) == 0;
            want.push(c);
            if zlp {
                packets.borrow_mut().push_back(Vec::new());
                want.push(Vec::new());
            }
        }
        for c in &want {
            let mut buf = [0u8; S];
            let n = block_on(r.read_command(&mut buf)).unwrap();
            assert_eq!(&buf[..n], &c[..], "mps {mps}");
        }
        assert!(packets.borrow().is_empty());
    }
}

#[test]
fn commands_reassemble_across_bulk_packets() {
    check_reassembly::<512>(64, 0x5eed_0000_0000_0001);
    check_reassembly::<512>(512, 0x5eed_0000_0000_0002);
    check_reassembly::<64>(64, 0x5eed_0000_0000_0003);
}

#[test]
fn a_long_jtag_configure_or_uart_transfer_spans_two_packets() {
    for header in [&[0x15, 63][..], &[0x21, 0x3e, 0, 60, 0]] {
        let (mut r, packets) = reader(64, false);
        let mut c = header.to_vec();
        c.resize(65, 4);
        packets.borrow_mut().extend(host_packets(&c, 64));
        let mut buf = [0u8; 512];
        assert_eq!(block_on(r.read_command(&mut buf)), Ok(65), "{:#04x}", c[0]);
        assert_eq!(&buf[..65], &c[..]);
        assert!(packets.borrow().is_empty());
    }
}

fn check_responses<const P: usize>(mps: usize) {
    for len in 0..=P {
        let data: Vec<u8> = (0..len).map(|i| i as u8).collect();
        let (mut w, sent) = writer(mps);
        block_on(w.write_response::<P>(&data)).unwrap();
        let sent = sent.take();
        let zlp = len != 0 && len.is_multiple_of(mps) && len < P;
        let pieces = data.chunks(mps).count() + usize::from(zlp) + usize::from(len == 0);
        assert_eq!(sent.len(), pieces, "len {len}, mps {mps}, P {P}");
        assert_eq!(sent.concat(), data);
        assert!(sent.iter().rev().skip(1).all(|p| p.len() == mps));
        // The host reads exactly this response, with no packet left for the next one.
        assert_eq!(host_reads(&sent, mps, P), vec![data]);
    }
}

#[test]
fn responses_split_into_packets_and_end_on_a_zlp_only_when_needed() {
    check_responses::<512>(64);
    check_responses::<512>(512);
    check_responses::<64>(64);
    for len in 0..=1024 {
        assert_eq!(
            needs_zlp(len, 64, 512),
            len.is_multiple_of(64) && len < 512,
            "{len}"
        );
    }
}

fn check_pipeline<const N: usize, const S: usize>(mps: usize, seed: u64) {
    let mut rng = Rng(seed);
    let (mut r, packets) = reader(mps, true);
    let (mut w, sent) = writer(mps);
    let mut want = Vec::new();
    for _ in 0..64 {
        let c = command(&mut rng, S);
        packets.borrow_mut().extend(host_packets(&c, mps));
        want.push(c);
    }
    let mut pipeline: DapPipeline<N, S> = DapPipeline::new();
    let echo = |req: &[u8], resp: &mut [u8]| {
        resp[..req.len()].copy_from_slice(req);
        req.len()
    };
    let mut future = pin!(run_dap_pipeline(&mut r, &mut w, &mut pipeline, echo));
    let mut cx = Context::from_waker(Waker::noop());
    for _ in 0..100_000 {
        if packets.borrow().is_empty() && host_reads_done(&sent.borrow(), mps, S, want.len()) {
            break;
        }
        assert!(future.as_mut().poll(&mut cx).is_pending());
    }
    assert_eq!(host_reads(&sent.borrow(), mps, S), want);
}

fn host_reads_done(sent: &[Vec<u8>], mps: usize, p: usize, n: usize) -> bool {
    let mut done = 0;
    let mut cur = 0;
    for packet in sent {
        cur += packet.len();
        if packet.len() < mps || cur == p {
            done += 1;
            cur = 0;
        }
    }
    done == n
}

#[test]
fn a_pipeline_echoes_every_command_through_its_slots() {
    check_pipeline::<1, 512>(64, 0x9199_0000_0000_0001);
    check_pipeline::<4, 512>(64, 0x9199_0000_0000_0002);
    check_pipeline::<8, 512>(512, 0x9199_0000_0000_0003);
    check_pipeline::<4, 64>(64, 0x9199_0000_0000_0004);
}

#[test]
fn a_slot_larger_than_the_dap_packet_ends_a_full_packet_without_a_zlp() {
    const P: usize = 512;
    let mps = 64;
    let (mut r, packets) = reader(mps, true);
    let (mut w, sent) = writer(mps);
    packets
        .borrow_mut()
        .extend([vec![0x00, 0xff], vec![0x00, 0xfe]]);
    let mut pipeline: DapPipeline<1, 1024, P> = DapPipeline::new();
    let respond = |req: &[u8], resp: &mut [u8]| {
        let len = if req[1] == 0xff { P } else { 1 };
        resp[..len].fill(req[1]);
        len
    };
    let mut future = pin!(run_dap_pipeline(&mut r, &mut w, &mut pipeline, respond));
    let mut cx = Context::from_waker(Waker::noop());
    for _ in 0..100_000 {
        if packets.borrow().is_empty() && host_reads_done(&sent.borrow(), mps, P, 2) {
            break;
        }
        assert!(future.as_mut().poll(&mut cx).is_pending());
    }
    assert_eq!(
        host_reads(&sent.borrow(), mps, P),
        vec![vec![0xff; P], vec![0xfe]]
    );
}
