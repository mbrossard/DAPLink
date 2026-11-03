extern crate std;

use super::*;
use std::{cell::RefCell, collections::BTreeMap, string::String, vec::Vec};

#[derive(Default)]
struct Stream {
    clear_at: Option<u32>,
    reads: u32,
    requested: bool,
}

#[derive(Default)]
struct Fake {
    regs: BTreeMap<usize, u32>,
    streams: BTreeMap<usize, Stream>,
    writes: Vec<(usize, u32)>,
}

std::thread_local! { static MMIO: RefCell<Fake> = RefCell::new(Fake::default()); }

pub(super) fn read(address: usize) -> u32 {
    MMIO.with(|m| {
        let mut m = m.borrow_mut();
        let value = *m.regs.get(&address).unwrap_or(&0);
        if let Some(s) = m.streams.get_mut(&address) {
            if !s.requested {
                return value | 1;
            }
            s.reads += 1;
            if !s.clear_at.is_some_and(|at| s.reads >= at) {
                return value | 1;
            }
        }
        value
    })
}

pub(super) fn write(address: usize, value: u32) {
    MMIO.with(|m| {
        let mut m = m.borrow_mut();
        m.writes.push((address, value));
        m.regs.insert(address, value);
        if value & 1 == 0 {
            if let Some(s) = m.streams.get_mut(&address) {
                s.requested = true;
                s.reads = 0;
            }
        }
    });
}

fn stream(cr: Reg, clear_at: Option<u32>) {
    MMIO.with(|m| {
        let mut m = m.borrow_mut();
        m.streams.insert(
            cr.0,
            Stream {
                clear_at,
                ..Stream::default()
            },
        );
        m.regs.insert(cr.0, 0x0801_0401);
        m.writes.clear();
    });
}

fn reset() {
    MMIO.with(|m| *m.borrow_mut() = Fake::default());
    TX_PENDING.store(false, Ordering::Relaxed);
    RX_FAILED.store(false, Ordering::Relaxed);
    TX_DMA_STUCK.store(0, Ordering::Relaxed);
    RX_DMA_STUCK.store(0, Ordering::Relaxed);
    echo_stats_reset();
}

#[derive(Default)]
pub(super) struct UpChannel(Vec<u8>);
impl UpChannel {
    pub(super) fn write(&mut self, bytes: &[u8]) {
        self.0.extend_from_slice(bytes);
    }
}

#[derive(Default)]
pub(super) struct SwoOut {
    pub(super) stats: proto::SwoStats,
}
impl SwoOut {
    pub(super) fn enable(&mut self) {}
    pub(super) fn disable(&mut self) {}
    pub(super) fn emit(&mut self, _: &[u8]) {}
    pub(super) fn set_baud(&mut self, baud: u32) -> Option<u32> {
        Some(baud)
    }
}

fn command(run: &mut Option<Run>, rx: &mut Rx, line: &str) -> String {
    let mut up = UpChannel::default();
    respond(
        &mut up,
        &mut SwoOut::default(),
        &mut false,
        run,
        rx,
        line.as_bytes(),
    );
    String::from_utf8(up.0).unwrap()
}

const START: &str = "V1 START 3 1 2 2048 100000 200000";

fn old_run() -> Option<Run> {
    let V1Command::Start(start) = v1::parse(START).unwrap() else {
        unreachable!()
    };
    let mut run = Run::new(&start);
    let mut bytes = [0u8; 4096];
    run.enc.as_mut().unwrap().fill(&mut bytes, 7);
    let mut host = Encoder::new(
        Direction::HostToDut,
        start.seed_h2d,
        start.interval,
        v1::budget(start.budget_h2d),
    );
    let n = host.fill(&mut bytes, 7);
    let ver = run.ver.as_mut().unwrap();
    ver.arm(5);
    ver.feed(&bytes[..n], 9);
    assert!(run.report(0, 0, 0).tx > 0);
    assert!(run.report(0, 0, 0).verified > 0);
    run.tx.fill(0xa5);
    run.tx_head = 5;
    run.tx_tail = 19;
    run.tx_dma = 7;
    Some(run)
}

fn snapshot(
    run: &Option<Run>,
    rx: &Rx,
) -> (
    [u8; V1_TX_RING],
    usize,
    usize,
    usize,
    bool,
    u64,
    u64,
    usize,
    V1Report,
) {
    let r = run.as_ref().unwrap();
    (
        r.tx,
        r.tx_head,
        r.tx_tail,
        r.tx_dma,
        r.active,
        rx.written,
        rx.read,
        rx.at,
        r.report(0, 0, 0),
    )
}

#[test]
fn production_uart_retains_owners_rejects_mutations_and_recovers_only_explicitly() {
    for at in [1, DMA_ABORT_SPIN_LIMIT / 2, DMA_ABORT_SPIN_LIMIT] {
        reset();
        stream(DMA1_S6CR, Some(at));
        assert_eq!(unsafe { tx_dma_stop() }, Ok(()));
        MMIO.with(|m| {
            let m = m.borrow();
            assert_eq!(m.streams[&DMA1_S6CR.0].reads, at);
            assert_eq!(
                m.writes,
                [(DMA1_S6CR.0, 0x0801_0400), (DMA1_HIFCR.0, 0b11_1101 << 16)]
            );
        });
    }
    for cmd in [START, "V1 RESET"] {
        reset();
        let mut rx = Rx::new();
        rx.written = 19;
        rx.read = 3;
        rx.at = 19;
        let mut run = old_run();
        let before = snapshot(&run, &rx);
        ECHO_RX.store(7, Ordering::Relaxed);
        stream(DMA1_S6CR, Some(DMA_ABORT_SPIN_LIMIT + 1));
        assert_eq!(command(&mut run, &mut rx, cmd), "ERR dma_abort_timeout\n");
        assert_eq!(snapshot(&run, &rx), before);
        assert_eq!(ECHO_RX.load(Ordering::Relaxed), 7);
        MMIO.with(|m| assert_eq!(m.borrow().writes, [(DMA1_S6CR.0, 0x0801_0400)]));
        MMIO.with(|m| {
            m.borrow_mut().writes.clear();
            m.borrow_mut().regs.insert(DMA1_HISR.0, 1 << 21);
        });
        service_uart(&mut run, &mut rx);
        for rejected in ["BAUD 115200", "V1 STOP", "STATS RESET"] {
            assert_eq!(
                command(&mut run, &mut rx, rejected),
                "ERR dma_abort_timeout\n"
            );
        }
        assert_eq!(snapshot(&run, &rx), before);
        assert_eq!(ECHO_TX.load(Ordering::Relaxed), 0);
        MMIO.with(|m| assert!(m.borrow().writes.is_empty()));
        assert!(command(&mut run, &mut rx, "PING").starts_with("OK"));
        assert!(command(&mut run, &mut rx, "STATS").contains("stuck=1"));
        assert!(command(&mut run, &mut rx, "V1 REPORT").starts_with("OK V1"));
        stream(DMA1_S6CR, None);
        assert_eq!(command(&mut run, &mut rx, cmd), "ERR dma_abort_timeout\n");
        assert_eq!(snapshot(&run, &rx), before);
        stream(DMA1_S6CR, Some(1));
        MMIO.with(|m| {
            m.borrow_mut().regs.insert(DMA1_S5NDTR.0, RX_RING as u32);
        });
        assert!(command(&mut run, &mut rx, cmd).starts_with("OK V1"));
        assert!(uart_ready());
        assert_eq!(echo_stats().stuck, 2);
        assert_eq!(run.as_ref().unwrap().tx_dma, 0);
    }

    reset();
    let mut rx = Rx::new();
    let mut run = old_run();
    stream(DMA1_S6CR, None);
    MMIO.with(|m| {
        m.borrow_mut().regs.insert(DMA1_HISR.0, 1 << 21);
    });
    let before = snapshot(&run, &rx);
    assert!(!unsafe { run.as_mut().unwrap().pump_tx() });
    assert_eq!(snapshot(&run, &rx), before);
    assert_eq!(echo_stats().tx, 0);
    service_uart(&mut run, &mut rx);
    assert_eq!(snapshot(&run, &rx), before);
    MMIO.with(|m| assert_eq!(m.borrow().writes, [(DMA1_S6CR.0, 0x0801_0400)]));

    reset();
    let mut rx = Rx::new();
    rx.written = 11;
    let mut run = None;
    stream(DMA1_S5CR, None);
    assert_eq!(unsafe { rx.start() }, Err(StopTimeout));
    assert_eq!(rx.written, 11);
    assert_eq!(echo_stats().stuck, 1);
    MMIO.with(|m| {
        assert_eq!(
            m.borrow().writes,
            [(RCC_AHB1ENR.0, 1 << 21), (DMA1_S5CR.0, 0x0801_0400)]
        )
    });
    stream(DMA1_S5CR, Some(1));
    assert_eq!(unsafe { rx.start() }, Err(StopTimeout));
    service_uart(&mut run, &mut rx);
    for cmd in [START, "V1 RESET", "BAUD 115200", "STATS RESET", "V1 STOP"] {
        assert_eq!(command(&mut run, &mut rx, cmd), "ERR dma_abort_timeout\n");
    }
    MMIO.with(|m| assert!(m.borrow().writes.is_empty()));
    TX_DMA_STUCK.store(u32::MAX, Ordering::Relaxed);
    assert_eq!(echo_stats().stuck, u32::MAX);
    echo_stats_reset();
    assert_eq!(echo_stats().stuck, u32::MAX);
    assert_eq!(RX_DMA_STUCK.load(Ordering::Relaxed), 1);

    reset();
    let mut rx = Rx::new();
    stream(DMA1_S5CR, Some(1));
    assert_eq!(unsafe { rx.start() }, Ok(()));
    let mut run = old_run().unwrap();
    run.tx_dma = 0;
    stream(DMA1_S6CR, Some(1));
    assert!(unsafe { run.pump_tx() });
    assert_eq!(run.tx_dma, 14);
    assert_eq!(read(DMA1_S6NDTR.0), 14);
    assert_eq!(read(DMA1_S6M0AR.0), unsafe { run.tx.as_ptr().add(5) }
        as u32);
}
