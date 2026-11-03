use std::collections::VecDeque;
use std::vec;
use std::vec::Vec;

use crate::program::Variant;
use crate::proto::{parity32, request_byte, Ack};
use crate::sequence::XferError;
use crate::transaction::{PioSwdTransaction, PioSwdTransactionIo};
use crate::transaction_program::{
    TransactionProgram, RELEASE_CONTINUATION, TRANSACTION_DIRECT, TRANSACTION_OEN,
    WRITE_CONTINUATION,
};

const FIFO_DEPTH: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Owner {
    Probe,
    Target,
    Released,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Edge {
    pc: u8,
    rising: bool,
    owner: Owner,
    oen_released: bool,
}

struct Target {
    response: Vec<bool>,
    response_pos: usize,
    driving: bool,
    value: bool,
    host_bits: Vec<bool>,
}

impl Target {
    fn new(read: bool, ack: u8, data: u32, bad_parity: bool) -> Self {
        let mut response = Vec::new();
        if ack != 0b111 {
            for bit in 0..3 {
                response.push(ack & (1 << bit) != 0);
            }
            if ack == 0b001 && read {
                for bit in 0..32 {
                    response.push(data & (1 << bit) != 0);
                }
                response.push((parity32(data) != 0) ^ bad_parity);
            }
        }
        Self {
            response,
            response_pos: 0,
            driving: false,
            value: true,
            host_bits: Vec::new(),
        }
    }

    fn after_rising_edge(&mut self, probe_drives: bool, line: bool) {
        if probe_drives {
            self.host_bits.push(line);
            return;
        }
        if self.host_bits.len() < 8 {
            return;
        }
        if let Some(&value) = self.response.get(self.response_pos) {
            self.value = value;
            self.driving = true;
            self.response_pos += 1;
        } else {
            self.value = true;
            self.driving = false;
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Blocked {
    Tx,
    Rx,
}

struct PendingPush {
    word: u32,
    next_pc: u8,
    duration: u64,
}

struct PioV0 {
    program: &'static TransactionProgram,
    pc: u8,
    x: u32,
    y: u32,
    osr: u32,
    isr: u32,
    in_count: u8,
    pin_out: bool,
    pin_dir: bool,
    clock: bool,
    oen_released: bool,
    cycles: u64,
    tx: VecDeque<u32>,
    rx: VecDeque<u32>,
    pending_push: Option<PendingPush>,
    edges: Vec<Edge>,
    collision: bool,
    target: Target,
}

impl PioV0 {
    fn new(
        program: &'static TransactionProgram,
        read: bool,
        ack: u8,
        data: u32,
        bad_parity: bool,
    ) -> Self {
        Self {
            program,
            pc: 0,
            x: 0,
            y: 0,
            osr: 0,
            isr: 0,
            in_count: 0,
            pin_out: false,
            pin_dir: false,
            clock: false,
            oen_released: true,
            cycles: 0,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            pending_push: None,
            edges: Vec::new(),
            collision: false,
            target: Target::new(read, ack, data, bad_parity),
        }
    }

    fn probe_drives(&self) -> bool {
        self.pin_dir
            && match self.program.variant {
                Variant::Direct => true,
                Variant::Oen => !self.oen_released,
            }
    }

    fn owner(&self) -> Owner {
        match (self.probe_drives(), self.target.driving) {
            (true, false) => Owner::Probe,
            (false, true) => Owner::Target,
            (false, false) => Owner::Released,
            (true, true) => Owner::Probe,
        }
    }

    fn line(&self) -> bool {
        if self.probe_drives() {
            self.pin_out
        } else if self.target.driving {
            self.target.value
        } else {
            true
        }
    }

    fn put(&mut self, word: u32) -> bool {
        if self.tx.len() == FIFO_DEPTH {
            false
        } else {
            self.tx.push_back(word);
            true
        }
    }

    fn side_and_delay(&self, word: u16) -> (Option<u8>, u8) {
        let data_bits = self.program.side_set_bits;
        let delay_bits = 5 - (data_bits + 1);
        let field = ((word >> 8) & 0x1f) as u8;
        let enabled = field & (1 << (delay_bits + data_bits)) != 0;
        let side = enabled.then_some((field >> delay_bits) & ((1 << data_bits) - 1));
        (side, field & ((1 << delay_bits) - 1))
    }

    fn apply_side_set(&mut self, word: u16) -> Option<(bool, bool)> {
        let (side, _) = self.side_and_delay(word);
        let side = side?;
        let old_clock = self.clock;
        self.clock = side & (1 << (self.program.side_set_bits - 1)) != 0;
        if self.program.variant == Variant::Oen {
            self.oen_released = side & 1 != 0;
        }
        if self.clock == old_clock {
            return None;
        }
        self.collision |= self.probe_drives() && self.target.driving;
        let line = self.line();
        let owner = self.owner();
        let rising = self.clock;
        self.edges.push(Edge {
            pc: self.pc,
            rising,
            owner,
            oen_released: self.oen_released,
        });
        Some((rising, line))
    }

    fn shift_out(&mut self, count: u8) -> u32 {
        let count = if count == 0 { 32 } else { count };
        let value = if count == 32 {
            self.osr
        } else {
            self.osr & ((1 << count) - 1)
        };
        self.osr = if count == 32 { 0 } else { self.osr >> count };
        value
    }

    fn queue_push(&mut self, word: u32, next_pc: u8, duration: u64) -> Result<(), Blocked> {
        if self.rx.len() == FIFO_DEPTH {
            self.pending_push = Some(PendingPush {
                word,
                next_pc,
                duration,
            });
            Err(Blocked::Rx)
        } else {
            self.rx.push_back(word);
            self.isr = 0;
            self.in_count = 0;
            self.pc = next_pc;
            self.cycles += duration;
            Ok(())
        }
    }

    fn step(&mut self) -> Result<(), Blocked> {
        if let Some(pending) = self.pending_push.take() {
            if self.rx.len() == FIFO_DEPTH {
                self.pending_push = Some(pending);
                return Err(Blocked::Rx);
            }
            self.rx.push_back(pending.word);
            self.isr = 0;
            self.in_count = 0;
            self.pc = pending.next_pc;
            self.cycles += pending.duration;
            return Ok(());
        }

        let word = self.program.instructions[self.pc as usize];
        let (_, delay) = self.side_and_delay(word);
        let edge = self.apply_side_set(word);
        let duration = u64::from(delay) + 1;
        let next_pc = (self.pc + 1) & 31;
        let opcode = word >> 13;
        let mut selected_pc = next_pc;
        let mut pushed = None;

        match opcode {
            0 => {
                let condition = ((word >> 5) & 7) as u8;
                let take = match condition {
                    0 => true,
                    2 => {
                        let nonzero = self.x != 0;
                        self.x = self.x.wrapping_sub(1);
                        nonzero
                    }
                    _ => panic!("unsupported JMP condition {condition}"),
                };
                if take {
                    selected_pc = (word & 0x1f) as u8;
                }
            }
            2 => {
                assert_eq!((word >> 5) & 7, 0, "only IN PINS is expected");
                let count = (word & 0x1f) as u8;
                let count = if count == 0 { 32 } else { count };
                let sample = edge
                    .filter(|(rising, _)| *rising)
                    .map(|(_, line)| line)
                    .unwrap_or_else(|| self.line());
                self.isr >>= count;
                if sample {
                    self.isr |= 1 << (32 - count);
                }
                self.in_count += count;
                if self.in_count == 32 {
                    pushed = Some(self.isr);
                }
            }
            3 => {
                let destination = ((word >> 5) & 7) as u8;
                let value = self.shift_out((word & 0x1f) as u8);
                match destination {
                    0 => self.pin_out = value & 1 != 0,
                    2 => self.y = value,
                    5 => selected_pc = (value & 0x1f) as u8,
                    _ => panic!("unsupported OUT destination {destination}"),
                }
            }
            4 => {
                if word & 0x80 != 0 {
                    let Some(value) = self.tx.pop_front() else {
                        return Err(Blocked::Tx);
                    };
                    self.osr = value;
                } else {
                    pushed = Some(self.isr);
                }
            }
            5 => {
                let destination = ((word >> 5) & 7) as u8;
                let source = (word & 7) as u8;
                match (destination, source) {
                    (0, 2) => self.pin_out = self.y & 1 != 0,
                    (2, 2) => {}
                    (6, 3) => {
                        self.isr = 0;
                        self.in_count = 0;
                    }
                    _ => panic!("unsupported MOV {destination},{source}"),
                }
            }
            7 => {
                let destination = ((word >> 5) & 7) as u8;
                let value = u32::from(word & 0x1f);
                match destination {
                    0 => self.pin_out = value & 1 != 0,
                    1 => self.x = value,
                    4 => self.pin_dir = value & 1 != 0,
                    _ => panic!("unsupported SET destination {destination}"),
                }
            }
            _ => panic!("unsupported opcode {opcode}"),
        }

        if let Some((true, line)) = edge {
            self.target.after_rising_edge(self.probe_drives(), line);
        }
        self.collision |= self.probe_drives() && self.target.driving;

        if let Some(value) = pushed {
            self.queue_push(value, selected_pc, duration)
        } else {
            self.pc = selected_pc;
            self.cycles += duration;
            Ok(())
        }
    }

    fn run_until_rx(&mut self) {
        for _ in 0..10_000 {
            if !self.rx.is_empty() {
                return;
            }
            self.step()
                .expect("program blocked before producing RX data");
        }
        panic!("program did not produce RX data");
    }

    fn run_until_tx_stall(&mut self, pc: u8) {
        for _ in 0..10_000 {
            match self.step() {
                Err(Blocked::Tx) if self.pc == pc => return,
                Err(blocked) => panic!("unexpected {blocked:?} stall at PC{}", self.pc),
                Ok(()) => {}
            }
        }
        panic!("program did not stall at PC{pc}");
    }

    fn parked_tx_empty(&self) -> bool {
        self.pc == 0 && self.tx.is_empty() && self.pending_push.is_none()
    }

    fn released_low(&self) -> bool {
        !self.clock && !self.probe_drives()
    }

    fn elapse(&mut self, cycles: u64) {
        self.cycles += cycles;
    }
}

#[derive(Clone, Copy, Default)]
struct CpuStalls {
    continuation: u64,
    write_data: u64,
    rx_drain: u64,
}

struct SimIo {
    pio: PioV0,
    stalls: CpuStalls,
    puts: usize,
    gets: usize,
    write_data_pending: bool,
    rx_stalled: bool,
    in_flight: bool,
}

impl SimIo {
    fn new(pio: PioV0, stalls: CpuStalls) -> Self {
        Self {
            pio,
            stalls,
            puts: 0,
            gets: 0,
            write_data_pending: false,
            rx_stalled: false,
            in_flight: false,
        }
    }

    fn stall_at_pull(&mut self, pc: u8, cycles: u64) {
        self.pio.run_until_tx_stall(pc);
        assert!(self.pio.released_low(), "PC{pc} stall must be released low");
        let edges = self.pio.edges.len();
        self.pio.elapse(cycles);
        assert_eq!(self.pio.edges.len(), edges, "CPU stall generated an edge");
    }

    fn finish(&mut self) {
        if !self.in_flight {
            return;
        }
        for _ in 0..10_000 {
            if self.pio.parked_tx_empty() {
                self.in_flight = false;
                return;
            }
            match self.pio.step() {
                Ok(()) => {}
                Err(Blocked::Tx) if self.pio.parked_tx_empty() => {
                    self.in_flight = false;
                    return;
                }
                Err(blocked) => panic!("finish blocked on {blocked:?} at PC{}", self.pio.pc),
            }
        }
        panic!("program did not finish");
    }

    fn put_blocking(&mut self, word: u32) {
        while !self.pio.put(word) {
            self.pio.step().expect("TX-full recovery blocked");
        }
        self.in_flight = true;
    }
}

impl PioSwdTransactionIo for SimIo {
    fn put(&mut self, word: u32) {
        if self.puts == 1 {
            self.stall_at_pull(12, self.stalls.continuation);
            self.write_data_pending = word & 0x1f == u32::from(WRITE_CONTINUATION);
        } else if self.write_data_pending {
            self.stall_at_pull(WRITE_CONTINUATION + 1, self.stalls.write_data);
            self.write_data_pending = false;
        }
        self.put_blocking(word);
        self.puts += 1;
    }

    fn get(&mut self) -> u32 {
        self.pio.run_until_rx();
        if self.gets != 0 && self.stalls.rx_drain != 0 && !self.rx_stalled {
            for _ in 0..10_000 {
                if self.pio.parked_tx_empty() {
                    break;
                }
                self.pio
                    .step()
                    .expect("RX-drain delay blocked the state machine");
            }
            assert!(self.pio.parked_tx_empty());
            assert!(self.pio.released_low());
            let edges = self.pio.edges.len();
            self.pio.elapse(self.stalls.rx_drain);
            assert_eq!(self.pio.edges.len(), edges);
            self.rx_stalled = true;
        }
        self.gets += 1;
        self.pio.rx.pop_front().expect("RX FIFO became empty")
    }
}

fn expected_error(ack: u8) -> Option<XferError> {
    match Ack::from_bits(ack) {
        Ack::Ok => None,
        Ack::Wait => Some(XferError::AckWait),
        Ack::Fault => Some(XferError::AckFault),
        Ack::NoAck => Some(XferError::NoAck),
        Ack::Unknown(value) => Some(XferError::Unknown(value)),
    }
}

fn bits(value: u32, count: usize) -> Vec<bool> {
    (0..count).map(|bit| value & (1 << bit) != 0).collect()
}

fn expected_edge_pcs(read: bool, ack: u8) -> Vec<u8> {
    let mut pcs = vec![4; 8];
    pcs.push(6);
    pcs.extend([9; 3]);
    match Ack::from_bits(ack) {
        Ack::Ok if read => {
            pcs.extend([15; 32]);
            pcs.extend([17, 19]);
        }
        Ack::Ok => {
            pcs.push(21);
            pcs.extend([26; 32]);
            pcs.push(28);
            pcs.extend([30; 8]);
        }
        _ => pcs.push(19),
    }
    pcs
}

fn expected_owners(read: bool, ack: u8) -> Vec<Owner> {
    let mut owners = vec![Owner::Probe; 8];
    owners.push(Owner::Released);
    let ack_owner = if ack == 0b111 {
        Owner::Released
    } else {
        Owner::Target
    };
    owners.extend([ack_owner; 3]);
    match Ack::from_bits(ack) {
        Ack::Ok if read => {
            owners.extend([Owner::Target; 33]);
            owners.push(Owner::Released);
        }
        Ack::Ok => {
            owners.push(Owner::Released);
            owners.extend([Owner::Probe; 41]);
        }
        _ => owners.push(Owner::Released),
    }
    owners
}

fn assert_transcript(pio: &PioV0, read: bool, ack: u8, request: u8, data: u32) {
    assert!(!pio.collision, "probe and target drove SWDIO together");
    assert!(pio.released_low());
    assert!(pio.parked_tx_empty());

    let rising: Vec<_> = pio.edges.iter().filter(|edge| edge.rising).collect();
    assert_eq!(
        rising.iter().map(|edge| edge.pc).collect::<Vec<_>>(),
        expected_edge_pcs(read, ack)
    );
    assert_eq!(
        rising.iter().map(|edge| edge.owner).collect::<Vec<_>>(),
        expected_owners(read, ack)
    );
    assert_eq!(pio.edges.len(), rising.len() * 2);
    for (index, edge) in pio.edges.iter().enumerate() {
        assert_eq!(edge.rising, index % 2 == 0, "edge {index}");
    }
    if pio.program.variant == Variant::Oen {
        for edge in rising {
            assert_eq!(edge.oen_released, edge.owner != Owner::Probe);
        }
    }

    let mut expected_host = bits(u32::from(request), 8);
    if !read && ack == 0b001 {
        expected_host.extend(bits(data, 32));
        expected_host.push(parity32(data) != 0);
        expected_host.extend([false; 8]);
    }
    assert_eq!(pio.target.host_bits, expected_host);
}

fn run_read(
    program: &'static TransactionProgram,
    ack: u8,
    data: u32,
    bad_parity: bool,
    stalls: CpuStalls,
) -> (Result<u32, XferError>, PioV0) {
    let pio = PioV0::new(program, true, ack, data, bad_parity);
    let mut engine = PioSwdTransaction::new(SimIo::new(pio, stalls));
    let result = engine.read_transfer(true, 2);
    engine.io_mut().finish();
    let pio = engine.into_inner().pio;
    assert_transcript(&pio, true, ack, request_byte(true, true, 2), data);
    (result, pio)
}

fn run_write(
    program: &'static TransactionProgram,
    ack: u8,
    data: u32,
    stalls: CpuStalls,
) -> (Result<(), XferError>, PioV0) {
    let pio = PioV0::new(program, false, ack, data, false);
    let mut engine = PioSwdTransaction::new(SimIo::new(pio, stalls));
    let result = engine.write_transfer(false, 1, data);
    engine.io_mut().finish();
    let pio = engine.into_inner().pio;
    assert_transcript(&pio, false, ack, request_byte(false, false, 1), data);
    (result, pio)
}

#[test]
fn every_ack_selects_the_exact_suffix_for_both_wirings() {
    const DATA: u32 = 0xa5c3_78f0;
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        for ack in 0..=7 {
            let (read, _) = run_read(program, ack, DATA, false, CpuStalls::default());
            assert_eq!(
                read,
                expected_error(ack).map_or(Ok(DATA), Err),
                "read {:?} ACK {ack:03b}",
                program.variant
            );
            let (write, _) = run_write(program, ack, DATA, CpuStalls::default());
            assert_eq!(
                write,
                expected_error(ack).map_or(Ok(()), Err),
                "write {:?} ACK {ack:03b}",
                program.variant
            );
        }
    }
}

#[test]
fn a_failed_write_ack_clocks_only_the_turnaround() {
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        for ack in (0..=7).filter(|&ack| ack != 0b001) {
            let (_, pio) = run_write(program, ack, 0x0123_4567, CpuStalls::default());
            let rising: Vec<_> = pio.edges.iter().filter(|edge| edge.rising).collect();
            assert_eq!(rising.len(), 13, "{:?} ACK {ack:03b}", program.variant);
            assert_eq!(
                rising.last().map(|edge| edge.pc),
                Some(RELEASE_CONTINUATION)
            );
        }
    }
}

#[test]
fn a_no_ack_write_reports_no_ack() {
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        let (write, _) = run_write(program, 0b111, 0x0123_4567, CpuStalls::default());
        assert_eq!(write, Err(XferError::NoAck), "{:?}", program.variant);
    }
}

#[test]
fn a_bad_parity_read_still_clocks_the_trailing_turnaround() {
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        let (read, pio) = run_read(program, 0b001, 1, true, CpuStalls::default());
        assert_eq!(read, Err(XferError::BadParity), "{:?}", program.variant);
        let rising: Vec<_> = pio.edges.iter().filter(|edge| edge.rising).collect();
        // dap-rs stops at 45, before the turnaround.
        assert_eq!(rising.len(), 46, "{:?}", program.variant);
        assert_eq!(rising.last().map(|edge| edge.owner), Some(Owner::Released));
    }
}

#[test]
fn data_and_parity_patterns_cross_the_encoded_program() {
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        for data in [0, 1, u32::MAX, 0x8000_0001, 0x5aa5_c33c] {
            assert_eq!(
                run_read(program, 1, data, false, CpuStalls::default()).0,
                Ok(data)
            );
            assert_eq!(
                run_read(program, 1, data, true, CpuStalls::default()).0,
                Err(XferError::BadParity)
            );
            assert_eq!(run_write(program, 1, data, CpuStalls::default()).0, Ok(()));
        }
    }
}

#[test]
fn arbitrary_cpu_stalls_only_extend_released_low_phases() {
    let stalls = CpuStalls {
        continuation: 17_003,
        write_data: 29_011,
        rx_drain: 41_009,
    };
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        let (_, unstalled_read) = run_read(program, 1, 0x1357_9bdf, false, CpuStalls::default());
        let (_, stalled_read) = run_read(program, 1, 0x1357_9bdf, false, stalls);
        assert_eq!(unstalled_read.edges, stalled_read.edges);
        assert_eq!(
            stalled_read.cycles - unstalled_read.cycles,
            stalls.continuation + stalls.rx_drain
        );

        let (_, unstalled_write) = run_write(program, 1, 0x2468_ace0, CpuStalls::default());
        let (_, stalled_write) = run_write(program, 1, 0x2468_ace0, stalls);
        assert_eq!(unstalled_write.edges, stalled_write.edges);
        assert_eq!(
            stalled_write.cycles - unstalled_write.cycles,
            stalls.continuation + stalls.write_data
        );
    }
}

#[test]
fn four_word_fifos_block_without_losing_program_state() {
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        let mut tx = PioV0::new(program, true, 1, 0, false);
        for word in 0..FIFO_DEPTH as u32 {
            assert!(tx.put(word));
        }
        assert!(!tx.put(99));
        tx.step().unwrap();
        assert_eq!(tx.tx.len(), FIFO_DEPTH - 1);
        assert!(tx.put(99));
        assert_eq!(tx.tx.len(), FIFO_DEPTH);

        let mut rx = PioV0::new(program, true, 1, 0, false);
        rx.rx.extend([10, 11, 12, 13]);
        assert!(rx.put(u32::from(request_byte(false, true, 0))));
        for _ in 0..1_000 {
            match rx.step() {
                Err(Blocked::Rx) => break,
                Err(blocked) => panic!("unexpected {blocked:?}"),
                Ok(()) => {}
            }
        }
        assert_eq!(rx.pc, 11);
        assert_eq!(rx.rx.len(), FIFO_DEPTH);
        assert!(rx.pending_push.is_some());
        assert!(rx.released_low());
        assert_eq!(rx.rx.pop_front(), Some(10));
        rx.step().unwrap();
        assert_eq!(rx.pc, 12);
        assert_eq!(rx.rx.len(), FIFO_DEPTH);
        assert_eq!(rx.rx.back(), Some(&(1 << 29)));
    }
}

#[test]
fn in_flight_and_pc0_txempty_form_the_finish_predicate() {
    let mut io = SimIo::new(
        PioV0::new(&TRANSACTION_DIRECT, true, 2, 0, false),
        CpuStalls::default(),
    );
    assert!(io.pio.parked_tx_empty());
    assert!(!io.in_flight, "initial PC0 is not a completed transfer");

    io.put(u32::from(request_byte(false, true, 0)));
    assert_eq!(io.pio.pc, 0);
    assert!(!io.pio.tx.is_empty());
    assert!(!io.pio.parked_tx_empty());
    assert!(io.in_flight);

    let ack = io.get();
    assert_eq!(ack >> 29, 0b010);
    io.put(u32::from(RELEASE_CONTINUATION));
    assert!(!io.pio.parked_tx_empty());
    io.finish();
    assert!(io.pio.parked_tx_empty());
    assert!(!io.in_flight);
}

#[test]
fn a_wait_read_hands_the_next_request_the_cell_after_its_turnaround() {
    const DATA: u32 = 0x2000_8000;
    let stalls = CpuStalls {
        continuation: 17_003,
        ..CpuStalls::default()
    };
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        let (waited, mut pio) = run_read(program, 0b010, 0, false, stalls);
        assert_eq!(waited, Err(XferError::AckWait));
        pio.target = Target::new(true, 0b001, DATA, false);
        let mut engine = PioSwdTransaction::new(SimIo::new(pio, stalls));
        assert_eq!(engine.read_transfer(true, 3), Ok(DATA));
        engine.io_mut().finish();
        let pio = engine.into_inner().pio;
        assert!(!pio.collision, "{:?}", program.variant);

        let rising: Vec<_> = pio.edges.iter().filter(|edge| edge.rising).collect();
        assert_eq!(rising.len(), 13 + 46, "{:?}", program.variant);
        assert_eq!(rising[12].pc, RELEASE_CONTINUATION);
        assert_eq!(rising[12].owner, Owner::Released);
        assert_eq!(rising[13].owner, Owner::Probe);
        assert_eq!(
            pio.target.host_bits,
            bits(u32::from(request_byte(true, true, 3)), 8)
        );
    }
}

#[test]
fn a_write_data_phase_gets_eight_low_cells_before_the_clock_stops() {
    // ADIv5 and the DAP-Lite SW-DP need eight rising edges after the parity before SWCLK stops.
    const DATA: u32 = 0x2000_8000;
    for program in [&TRANSACTION_DIRECT, &TRANSACTION_OEN] {
        let pio = PioV0::new(program, false, 0b001, DATA, false);
        let mut engine = PioSwdTransaction::new(SimIo::new(pio, CpuStalls::default()));
        assert_eq!(engine.write_transfer(false, 1, DATA), Ok(()));
        engine.io_mut().finish();
        let pio = engine.into_inner().pio;
        assert!(pio.released_low() && pio.parked_tx_empty());

        let after_parity = &pio.target.host_bits[8 + 33..];
        assert!(
            after_parity.len() >= 8 && after_parity.iter().all(|&bit| !bit),
            "{:?}: {} cells after the parity",
            program.variant,
            after_parity.len()
        );
    }
}
