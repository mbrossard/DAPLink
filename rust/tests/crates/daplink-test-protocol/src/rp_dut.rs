//! RP DUT controller: command dispatch, v1 runs, stable transmit slots and DMA-abort retention.

use crate::v1::{self, Direction, Encoder, V1Command, V1Report, V1Start, Verifier};
use crate::{self as proto, Stats};
use core::cell::UnsafeCell;

const V1_CHUNK: usize = 32;
const TX_CHUNK: usize = 256;
const NO_BUFFER: usize = 2;

const DR_FE: u16 = 1 << 8;
const DR_PE: u16 = 1 << 9;
const DR_BE: u16 = 1 << 10;
const DR_OE: u16 = 1 << 11;

/// Two stable transmit slots, kept separate from the routinely reborrowed protocol state.
pub struct TxBuffers([UnsafeCell<[u8; TX_CHUNK]>; 2]);

impl TxBuffers {
    pub const fn new() -> Self {
        Self([
            UnsafeCell::new([0; TX_CHUNK]),
            UnsafeCell::new([0; TX_CHUNK]),
        ])
    }
}

impl Default for TxBuffers {
    fn default() -> Self {
        Self::new()
    }
}

/// RP runtime operations the controller drives; abort ownership is stated per method.
pub trait Hardware {
    fn now_us(&self) -> u32;
    fn set_baud(&mut self, baud: u32) -> Option<u32>;
    /// True only after TX releases its buffer; false retains the old run for a later retry.
    fn tx_abort(&mut self) -> bool;
    fn rx_abort_pending(&self) -> bool;
    fn abort_stuck(&self) -> u32;
}

struct V1Run {
    armed: bool,
    credit: bool,
    encoder: Option<Encoder>,
    verifier: Option<Verifier>,
    tx_len: [usize; 2],
    tx_inflight: usize,
}

impl V1Run {
    const fn new() -> Self {
        Self {
            armed: false,
            credit: false,
            encoder: None,
            verifier: None,
            tx_len: [0; 2],
            tx_inflight: NO_BUFFER,
        }
    }

    fn start(&mut self, start: &V1Start, now_us: u32) {
        *self = Self {
            credit: start.credit,
            ..Self::new()
        };
        if start.dirs.tx() {
            let mut encoder = Encoder::new(
                Direction::DutToHost,
                start.seed_d2h,
                start.interval,
                v1::budget(start.budget_d2h),
            );
            if let Some(burst) = start.burst {
                encoder = encoder.with_burst(burst);
            }
            if start.credit {
                encoder = encoder.with_credit();
            }
            self.encoder = Some(encoder);
        }
        if start.dirs.rx() {
            let mut verifier = Verifier::new(
                Direction::HostToDut,
                start.seed_h2d,
                start.interval,
                v1::budget(start.budget_h2d),
            );
            if start.credit {
                verifier = verifier.with_credit();
            }
            verifier.arm(now_us);
            self.verifier = Some(verifier);
        }
        self.armed = true;
    }

    fn close(&mut self) {
        if let Some(encoder) = self.encoder.as_mut() {
            encoder.close();
        }
    }

    fn reset(&mut self) {
        if let Some(encoder) = self.encoder.as_mut() {
            encoder.reset();
        }
        if let Some(verifier) = self.verifier.as_mut() {
            verifier.reset();
        }
        self.armed = false;
        self.tx_len = [0; 2];
        self.tx_inflight = NO_BUFFER;
    }

    fn feed(&mut self, bytes: &[u8], now_us: u32) {
        if let Some(verifier) = self.verifier.as_mut() {
            verifier.feed(bytes, now_us);
        }
    }

    fn report(&self, stats: &Stats) -> V1Report {
        let tx = self.encoder.as_ref().map(Encoder::counters);
        let rx = self.verifier.as_ref().map(Verifier::counters);
        V1Report::new(
            &tx.unwrap_or_default(),
            &rx.unwrap_or_default(),
            stats.ferr,
            stats.ovr,
            0,
        )
    }
}

/// Protocol and measurement state shared by the RP DUT firmwares.
pub struct Dut<'a> {
    name: &'static str,
    baud_max: u32,
    stamp: u32,
    stats: Stats,
    run: V1Run,
    tx: &'a TxBuffers,
    abort_pending: bool,
}

impl<'storage> Dut<'storage> {
    pub const fn new(name: &'static str, baud_max: u32, tx: &'storage mut TxBuffers) -> Self {
        Self {
            name,
            baud_max,
            stamp: 0,
            stats: Stats {
                rx: 0,
                tx: 0,
                ferr: 0,
                ovr: 0,
                drop: 0,
                stuck: 0,
            },
            run: V1Run::new(),
            tx,
            abort_pending: false,
        }
    }

    pub fn armed(&self) -> bool {
        self.run.armed
    }

    /// Pause data service after a failed abort until Start or Reset successfully retries it.
    pub fn abort_pending(&self) -> bool {
        self.abort_pending
    }

    pub fn record_overruns(&mut self, count: u32) {
        self.stats.ovr = self.stats.ovr.wrapping_add(count);
    }

    pub fn record_rx(&mut self, entry: u16) {
        self.stats.rx = self.stats.rx.wrapping_add(1);
        if entry & (DR_FE | DR_PE | DR_BE) != 0 {
            self.stats.ferr = self.stats.ferr.wrapping_add(1);
        }
        if entry & DR_OE != 0 {
            self.stats.ovr = self.stats.ovr.wrapping_add(1);
        }
    }

    pub fn record_tx(&mut self, count: u32) {
        self.stats.tx = self.stats.tx.wrapping_add(count);
    }

    pub fn sync_credit(&mut self) {
        if self.run.credit {
            let grant = self.run.verifier.as_ref().map_or(0, Verifier::credit);
            if let Some(encoder) = self.run.encoder.as_mut() {
                encoder.set_grant(grant);
            }
        }
    }

    pub fn feed(&mut self, bytes: &[u8], now_us: u32) {
        for chunk in bytes.chunks(V1_CHUNK) {
            self.run.feed(chunk, now_us);
        }
    }

    /// Charge the in-flight slot as sent and free it; call once its DMA has completed.
    pub fn retire_tx(&mut self) {
        if self.abort_pending {
            return;
        }
        if self.run.tx_inflight != NO_BUFFER {
            let slot = self.run.tx_inflight;
            self.record_tx(self.run.tx_len[slot] as u32);
            self.run.tx_len[slot] = 0;
            self.run.tx_inflight = NO_BUFFER;
        }
    }

    /// Reserve a staged slot; a raw DMA caller must retain its storage and prevent retirement until completion.
    pub fn take_pending_tx(&mut self) -> Option<&[u8]> {
        if self.abort_pending || self.run.tx_inflight != NO_BUFFER {
            return None;
        }
        let slot = (0..2).find(|&slot| self.run.tx_len[slot] != 0)?;
        self.run.tx_inflight = slot;
        // SAFETY: the slot is initialized; filling skips it until retirement, and this borrow excludes mutation.
        Some(&unsafe { &*self.tx.0[slot].get() }[..self.run.tx_len[slot]])
    }

    pub fn fill_tx(&mut self, now_us: u32) {
        if self.abort_pending {
            return;
        }
        if let Some(encoder) = self.run.encoder.as_mut() {
            for slot in 0..2 {
                if slot != self.run.tx_inflight && self.run.tx_len[slot] == 0 {
                    // SAFETY: only this controller can access storage, and this slot is not in flight.
                    let bytes = unsafe { &mut *self.tx.0[slot].get() };
                    self.run.tx_len[slot] = encoder.fill(bytes, now_us);
                }
            }
        }
    }

    pub fn respond<'a>(
        &mut self,
        hardware: &mut dyn Hardware,
        request: &[u8],
        reply: &'a mut [u8; v1::V1_LINE],
    ) -> &'a [u8] {
        let request = core::str::from_utf8(request).unwrap_or("");
        if let Some(command) = v1::parse(request) {
            return self.v1_respond(hardware, command, reply);
        }
        match proto::parse(request) {
            Some(proto::Command::Ping) => line(reply, proto::OK_PONG),
            Some(proto::Command::Id) => {
                proto::id_response(small(reply), self.name, self.stamp).as_bytes()
            }
            Some(proto::Command::Stamp(n)) => {
                self.stamp = n;
                proto::stamp_response(small(reply), n).as_bytes()
            }
            Some(proto::Command::Baud(n)) => match hardware.set_baud(n) {
                Some(realized) => proto::baud_response(small(reply), realized).as_bytes(),
                None if hardware.rx_abort_pending() => line(reply, proto::ERR_DMA_ABORT_TIMEOUT),
                None => line(reply, proto::ERR_UNIMPLEMENTED),
            },
            Some(proto::Command::BaudMax) => {
                proto::baud_max_response(small(reply), self.baud_max).as_bytes()
            }
            Some(proto::Command::Stats) => {
                let stats = Stats {
                    stuck: hardware.abort_stuck(),
                    ..self.stats
                };
                proto::stats_response(small(reply), &stats).as_bytes()
            }
            Some(proto::Command::StatsReset) => {
                if self.abort_pending || hardware.rx_abort_pending() {
                    return line(reply, proto::ERR_DMA_ABORT_TIMEOUT);
                }
                self.reset_stats();
                line(reply, proto::OK_STATS_RESET)
            }
            Some(
                proto::Command::SwoEmit(_)
                | proto::Command::SwoCfg(_)
                | proto::Command::SwoStream(_)
                | proto::Command::SwoStats,
            )
            | None => line(reply, proto::ERR_UNIMPLEMENTED),
        }
    }

    fn v1_respond<'a>(
        &mut self,
        hardware: &mut dyn Hardware,
        command: V1Command,
        reply: &'a mut [u8; v1::V1_LINE],
    ) -> &'a [u8] {
        match command {
            V1Command::Start(start) => {
                if !self.abort_tx(hardware) {
                    return line(reply, proto::ERR_DMA_ABORT_TIMEOUT);
                }
                self.reset_stats();
                self.run.start(&start, hardware.now_us());
                line(reply, v1::OK_V1_START)
            }
            V1Command::Stop => {
                if self.abort_pending || hardware.rx_abort_pending() {
                    return line(reply, proto::ERR_DMA_ABORT_TIMEOUT);
                }
                self.run.close();
                line(reply, v1::OK_V1_STOPPED)
            }
            V1Command::Report => {
                v1::report_response(reply, &self.run.report(&self.stats)).as_bytes()
            }
            V1Command::Reset => {
                if !self.abort_tx(hardware) {
                    return line(reply, proto::ERR_DMA_ABORT_TIMEOUT);
                }
                self.run.reset();
                self.reset_stats();
                line(reply, v1::OK_V1_RESET)
            }
        }
    }

    fn reset_stats(&mut self) {
        self.stats = Stats::default();
    }

    fn abort_tx(&mut self, hardware: &mut dyn Hardware) -> bool {
        if hardware.rx_abort_pending() {
            return false;
        }
        self.abort_pending = !hardware.tx_abort();
        !self.abort_pending
    }
}

fn small(reply: &mut [u8; v1::V1_LINE]) -> &mut [u8; proto::MAX_LINE] {
    (&mut reply[..proto::MAX_LINE]).try_into().unwrap()
}

fn line<'a>(reply: &'a mut [u8; v1::V1_LINE], text: &str) -> &'a [u8] {
    crate::line(reply, format_args!("{text}")).as_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct FakeHardware {
        now: u32,
        realized: Option<u32>,
        aborts: u32,
        stuck: u32,
        fail_abort: bool,
        rx_pending: bool,
    }

    impl Hardware for FakeHardware {
        fn now_us(&self) -> u32 {
            self.now
        }

        fn set_baud(&mut self, _baud: u32) -> Option<u32> {
            self.realized
        }

        fn tx_abort(&mut self) -> bool {
            self.aborts += 1;
            self.stuck += self.fail_abort as u32;
            !self.fail_abort
        }

        fn rx_abort_pending(&self) -> bool {
            self.rx_pending
        }

        fn abort_stuck(&self) -> u32 {
            self.stuck
        }
    }

    fn text(bytes: &[u8]) -> &str {
        core::str::from_utf8(bytes).unwrap()
    }

    #[test]
    fn failed_start_and_reset_preserve_run_slots_and_stats_until_explicit_retry() {
        for command in [b"V1 START 1 9 8 3 0 2048".as_slice(), b"V1 RESET"] {
            let mut tx = TxBuffers::new();
            let mut dut = Dut::new("rp", 1_000_000, &mut tx);
            let mut hardware = FakeHardware::default();
            let mut reply = [0; v1::V1_LINE];
            dut.respond(&mut hardware, b"V1 START 3 1 2 3 1024 4096", &mut reply);
            dut.fill_tx(0);
            let first = dut.take_pending_tx().unwrap();
            let mut saved = [0; TX_CHUNK];
            saved[..first.len()].copy_from_slice(first);
            let ptr = first.as_ptr();
            let len = first.len();
            let lengths = dut.run.tx_len;
            let inflight = dut.run.tx_inflight;
            dut.record_rx(DR_FE | DR_OE | 1);
            dut.record_tx(13);
            let before_stats = dut.stats;
            let before_report = dut.run.report(&dut.stats);
            hardware.fail_abort = true;
            for attempts in 1..=2 {
                assert_eq!(
                    text(dut.respond(&mut hardware, command, &mut reply)),
                    "ERR dma_abort_timeout\n"
                );
                assert!(dut.abort_pending());
                assert!(dut.armed());
                dut.retire_tx();
                dut.fill_tx(100);
                assert!(dut.take_pending_tx().is_none());
                assert_eq!(dut.run.tx_len, lengths);
                assert_eq!(dut.run.tx_inflight, inflight);
                assert_eq!(dut.stats, before_stats);
                assert_eq!(dut.run.report(&dut.stats), before_report);
                assert_eq!(hardware.stuck, attempts);
                // SAFETY: the old slot remains reserved, and no live slice overlaps a mutation.
                assert_eq!(
                    unsafe { core::slice::from_raw_parts(ptr, len) },
                    &saved[..len]
                );
            }
            for rejected in [b"STATS RESET".as_slice(), b"V1 STOP"] {
                assert_eq!(
                    text(dut.respond(&mut hardware, rejected, &mut reply)),
                    "ERR dma_abort_timeout\n"
                );
                assert_eq!(dut.stats, before_stats);
                assert_eq!(dut.run.report(&dut.stats), before_report);
            }
            hardware.fail_abort = false;
            let expected = if command == b"V1 RESET" {
                "OK V1 RESET\n"
            } else {
                "OK V1 START\n"
            };
            assert_eq!(
                text(dut.respond(&mut hardware, command, &mut reply)),
                expected
            );
            assert!(!dut.abort_pending());
            assert_eq!(dut.run.tx_inflight, NO_BUFFER);
            assert_eq!(dut.run.tx_len, [0, 0]);
            assert_eq!(dut.stats, Stats::default());
            assert_eq!(hardware.stuck, 2);
        }
    }

    #[test]
    fn unresolved_rx_abort_rejects_baud_and_run_mutation_without_a_tx_attempt() {
        let mut tx = TxBuffers::new();
        let mut dut = Dut::new("rp", 1_000_000, &mut tx);
        let mut hardware = FakeHardware {
            rx_pending: true,
            ..FakeHardware::default()
        };
        let mut reply = [0; v1::V1_LINE];
        dut.record_tx(7);
        for command in [
            b"BAUD 115200".as_slice(),
            b"V1 START 1 1 2 3 0 10",
            b"V1 RESET",
            b"STATS RESET",
        ] {
            assert_eq!(
                text(dut.respond(&mut hardware, command, &mut reply)),
                "ERR dma_abort_timeout\n"
            );
            assert_eq!(dut.stats.tx, 7);
        }
        assert_eq!(hardware.aborts, 0);
        hardware.rx_pending = false;
        assert_eq!(
            text(dut.respond(&mut hardware, b"V1 RESET", &mut reply)),
            "OK V1 RESET\n"
        );
    }

    #[test]
    fn moving_controller_and_filling_other_slot_preserves_reserved_storage() {
        let mut tx = TxBuffers::new();
        let mut dut = Dut::new("rp", 1_000_000, &mut tx);
        let mut hardware = FakeHardware::default();
        let mut reply = [0; v1::V1_LINE];
        dut.respond(&mut hardware, b"V1 START 1 1 2 3 0 4096", &mut reply);
        dut.fill_tx(0);
        let first = dut.take_pending_tx().unwrap();
        let mut saved = [0; TX_CHUNK];
        saved[..first.len()].copy_from_slice(first);
        let ptr = first.as_ptr();
        let len = first.len();
        let mut moved = dut;
        moved.record_rx(1);
        moved.sync_credit();
        moved.fill_tx(1);
        assert!(moved.take_pending_tx().is_none());
        // SAFETY: the externally borrowed storage is live, and the reserved slot was never retired.
        assert_eq!(
            unsafe { core::slice::from_raw_parts(ptr, len) },
            &saved[..len]
        );
        moved.retire_tx();
        assert_eq!(moved.stats.tx, len as u32);
        let next = moved.take_pending_tx().unwrap();
        assert_ne!(ptr, next.as_ptr());
        assert!(next.len() <= TX_CHUNK);
    }

    #[test]
    fn protocol_dispatches_identity_baud_and_unknown_commands() {
        let mut hardware = FakeHardware {
            realized: Some(921_601),
            ..FakeHardware::default()
        };
        let mut tx = TxBuffers::new();
        let mut dut = Dut::new("rp", 12_000_000, &mut tx);
        let mut reply = [0; v1::V1_LINE];

        assert_eq!(
            text(dut.respond(&mut hardware, b"PING", &mut reply)),
            "OK PONG\n"
        );
        assert!(text(dut.respond(&mut hardware, b"ID", &mut reply)).contains("name=rp"));
        assert_eq!(
            text(dut.respond(&mut hardware, b"BAUD 921600", &mut reply)),
            "OK BAUD 921601\n"
        );
        assert_eq!(
            text(dut.respond(&mut hardware, b"SWO EMIT 1", &mut reply)),
            "ERR unimplemented\n"
        );
        assert_eq!(
            text(dut.respond(&mut hardware, b"SWO STATS", &mut reply)),
            "ERR unimplemented\n"
        );
    }

    #[test]
    fn reset_state_starts_clean_and_v1_reset_clears_run_counters() {
        let mut hardware = FakeHardware {
            now: 7,
            stuck: 3,
            ..FakeHardware::default()
        };
        let mut tx = TxBuffers::new();
        let mut dut = Dut::new("rp", 1_000_000, &mut tx);
        let mut reply = [0; v1::V1_LINE];

        assert!(text(dut.respond(&mut hardware, b"ID", &mut reply)).contains("stamp=0"));
        dut.respond(&mut hardware, b"STAMP 42", &mut reply);
        dut.record_rx(DR_FE | DR_OE | 1);
        dut.record_tx(2);
        assert_eq!(
            text(dut.respond(&mut hardware, b"V1 START 1 1 2 3 0 16", &mut reply)),
            "OK V1 START\n"
        );
        assert!(dut.armed());
        dut.record_rx(DR_FE | 1);
        assert_eq!(
            text(dut.respond(&mut hardware, b"V1 RESET", &mut reply)),
            "OK V1 RESET\n"
        );
        assert!(!dut.armed());
        assert_eq!(hardware.aborts, 2);
        assert!(text(dut.respond(&mut hardware, b"ID", &mut reply)).contains("stamp=42"));
        assert_eq!(
            text(dut.respond(&mut hardware, b"STATS", &mut reply)),
            "OK STATS rx=0 tx=0 ferr=0 ovr=0 drop=0 stuck=3\n"
        );

        let mut reset_tx = TxBuffers::new();
        let reset = Dut::new("rp", 1_000_000, &mut reset_tx);
        assert_eq!(reset.stamp, 0);
        assert!(!reset.armed());
    }
}
