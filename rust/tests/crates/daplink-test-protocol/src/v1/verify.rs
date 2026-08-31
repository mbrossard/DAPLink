//! The receive half: an incremental verifier that resynchronizes on the frames themselves.

use super::crc::Crc32;
use super::{Direction, Frame, FrameKind, FRAME_LEN};
use crate::prng::Prng;

/// What a receiver made of the stream it was given.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RxCounters {
    /// Payload bytes that matched the generator inside a span that framed correctly.
    pub verified: u64,
    /// Payload bytes that differed inside a span that framed correctly.
    pub corrupt: u64,
    /// Payload bytes the sender emitted and the receiver never saw, net, from the offsets.
    pub missing: u64,
    /// Payload bytes received beyond what the sender emitted, net, from the offsets.
    pub duplicate: u64,
    /// Payload bytes inside a resync window, which no rule can attribute byte by byte.
    pub unresolved: u64,
    /// Frames accepted, of every kind.
    pub frames: u32,
    /// Resynchronizations entered.
    pub resyncs: u32,
    /// An end-of-stream frame arrived.
    pub eos: bool,
    /// Final payload total the end-of-stream frame carried.
    pub eos_total: u64,
    /// Highest cumulative grant seen in a credit frame.
    pub credit: u64,
    /// Microseconds from arming to the last feed that carried bytes.
    pub elapsed_us: u32,
    /// Microseconds from arming to the first feed that carried bytes.
    pub onset_us: u32,
    /// Longest gap between two feeds that carried bytes.
    pub maxgap_us: u32,
}

impl RxCounters {
    /// Payload bytes accounted for on the sender's side of the ledger.
    pub const fn accounted(&self) -> u64 {
        self.verified
            .saturating_add(self.corrupt)
            .saturating_add(self.missing)
            .saturating_add(self.unresolved)
    }

    /// The run ended and every byte the sender says it sent is accounted for.
    pub const fn reconciles(&self) -> bool {
        self.eos && self.accounted() == self.eos_total
    }

    /// Nothing was lost, duplicated, corrupted, or left unattributed.
    pub const fn clean(&self) -> bool {
        self.corrupt == 0 && self.missing == 0 && self.duplicate == 0 && self.unresolved == 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// Comparing payload against the generator.
    Sync,
    /// Collecting the frame that closes the current span.
    Frame,
    /// Collecting the bytes after a marker, which may be a credit frame.
    Peek,
    /// Scanning for a frame to resynchronize on.
    Resync,
    /// The end-of-stream frame arrived.
    Done,
}

/// Marker-framed receiver: [`Verifier::arm`] before the sender starts, [`Verifier::feed`] for
/// every received byte, [`Verifier::counters`] for the ledger.
#[derive(Debug, Clone)]
pub struct Verifier {
    dir: Direction,
    seed: u32,
    interval: u32,
    budget: u64,
    credit: bool,
    state: State,
    prng: Prng,
    span_prng: Prng,
    /// Generator walked forward through a resync window, so its anchor is a short advance.
    look: Prng,
    pace_left: u32,
    pace_skip: u32,
    span_start: u64,
    offset: u64,
    span: Crc32,
    span_ok: u32,
    span_bad: u32,
    expect_seq: u32,
    window: u64,
    buf: [u8; FRAME_LEN],
    buf_len: usize,
    ring: [u8; FRAME_LEN],
    ring_pos: usize,
    ring_len: usize,
    counters: RxCounters,
    armed_us: u32,
    last_us: u32,
    seen: bool,
}

impl Verifier {
    /// Expect a stream of `dir` from `seed`, framed every `interval` bytes, ending at `budget`.
    pub fn new(dir: Direction, seed: u32, interval: u32, budget: u64) -> Self {
        let interval = if interval < FRAME_LEN as u32 {
            FRAME_LEN as u32
        } else {
            interval
        };
        Self {
            dir,
            seed,
            interval,
            budget,
            credit: false,
            state: State::Sync,
            prng: Prng::new(seed),
            span_prng: Prng::new(seed),
            look: Prng::new(seed),
            pace_left: interval,
            pace_skip: 0,
            span_start: 0,
            offset: 0,
            span: Crc32::new(),
            span_ok: 0,
            span_bad: 0,
            expect_seq: 0,
            window: 0,
            buf: [0; FRAME_LEN],
            buf_len: 0,
            ring: [0; FRAME_LEN],
            ring_pos: 0,
            ring_len: 0,
            counters: RxCounters::default(),
            armed_us: 0,
            last_us: 0,
            seen: false,
        }
    }

    /// Expect credit frames at the frame slots, which costs one frame of buffering per marker.
    pub fn with_credit(mut self) -> Self {
        self.credit = true;
        self
    }

    /// Start the clock: onset, elapsed and gap are measured from here, not from the first byte.
    pub fn arm(&mut self, now_us: u32) {
        self.armed_us = now_us;
        self.last_us = now_us;
        self.seen = false;
        self.counters.elapsed_us = 0;
        self.counters.onset_us = 0;
        self.counters.maxgap_us = 0;
    }

    /// Zero the run, keeping the configuration and reseeding the generator.
    pub fn reset(&mut self) {
        let credit = self.credit;
        let mut fresh = Self::new(self.dir, self.seed, self.interval, self.budget);
        if credit {
            fresh = fresh.with_credit();
        }
        *self = fresh;
    }

    /// The run's ledger.
    pub fn counters(&self) -> RxCounters {
        self.counters
    }

    /// Highest cumulative grant the far end has announced.
    pub fn credit(&self) -> u64 {
        self.counters.credit
    }

    /// The end-of-stream frame arrived, so the run is over.
    pub fn complete(&self) -> bool {
        self.state == State::Done
    }

    /// Classify `buf`, received at `now_us`; an empty slice is a poll and disturbs no timing.
    pub fn feed(&mut self, buf: &[u8], now_us: u32) {
        if !buf.is_empty() {
            if self.seen {
                let gap = now_us.wrapping_sub(self.last_us);
                if gap > self.counters.maxgap_us {
                    self.counters.maxgap_us = gap;
                }
            } else {
                self.counters.onset_us = now_us.wrapping_sub(self.armed_us);
                self.seen = true;
            }
            self.last_us = now_us;
            self.counters.elapsed_us = now_us.wrapping_sub(self.armed_us);
        }
        for &b in buf {
            self.byte(b);
        }
    }

    fn byte(&mut self, b: u8) {
        match self.state {
            State::Sync => self.payload_byte(b),
            State::Frame | State::Peek => {
                self.buf[self.buf_len] = b;
                self.buf_len += 1;
                if self.buf_len == FRAME_LEN {
                    if self.state == State::Frame {
                        self.close_span();
                    } else {
                        self.close_peek();
                    }
                }
            }
            State::Resync => self.scan(b),
            // Bytes past the end of the stream are bytes the sender did not send.
            State::Done => self.counters.duplicate += 1,
        }
    }

    /// Score one payload byte and stop at the frame slot the sender will have used.
    fn payload_byte(&mut self, b: u8) {
        if b == self.prng.next_byte() {
            self.span_ok += 1;
        } else {
            self.span_bad += 1;
        }
        self.span.push(b);
        self.offset += 1;
        self.window += 1;
        if self.offset == self.budget || self.offset - self.span_start == u64::from(self.interval) {
            self.state = State::Frame;
            self.buf_len = 0;
        }
    }

    /// Decide the span whose closing frame has just been collected.
    fn close_span(&mut self) {
        let frame = Frame::decode(&self.buf).filter(|frame| {
            frame.dir == self.dir
                && frame.kind != FrameKind::Credit
                && frame.seq == self.expect_seq
                && frame.offset == self.offset
        });
        let Some(frame) = frame else {
            self.enter_resync();
            return;
        };
        self.counters.verified += u64::from(self.span_ok);
        self.counters.corrupt += u64::from(self.span_bad);
        // The header CRC16 covers the span CRC32, so a clean span cannot disagree with it.
        if frame.crc != self.span.value() && self.span_bad == 0 {
            self.counters.corrupt += 1;
        }
        self.counters.frames += 1;
        self.commit(frame.offset, frame.seq);
        self.resume(&frame);
    }

    /// Pick up after an accepted frame: the stream is over, or a new span starts.
    fn resume(&mut self, frame: &Frame) {
        if frame.kind == FrameKind::EndOfStream {
            self.counters.eos = true;
            self.counters.eos_total = frame.offset;
            self.state = State::Done;
        } else if self.credit {
            self.state = State::Peek;
            self.buf_len = 0;
        } else {
            self.state = State::Sync;
        }
    }

    /// Take the bytes after a marker as a credit frame, or hand them back as payload.
    fn close_peek(&mut self) {
        let credit = Frame::decode(&self.buf)
            .filter(|frame| frame.dir == self.dir && frame.kind == FrameKind::Credit);
        if let Some(frame) = credit {
            self.counters.frames += 1;
            self.grant(frame.offset);
            self.buf_len = 0;
            return;
        }
        let buf = self.buf;
        let len = self.buf_len;
        self.buf_len = 0;
        self.state = State::Sync;
        for &b in &buf[..len] {
            self.byte(b);
        }
    }

    /// Give up on the current span: its bytes join the resync window unscored.
    fn enter_resync(&mut self) {
        self.counters.resyncs += 1;
        self.state = State::Resync;
        self.ring_len = 0;
        self.ring_pos = 0;
        self.span_ok = 0;
        self.span_bad = 0;
        // The window opens on the frame slot that failed, and the sender's next span follows it.
        self.look = self.prng;
        self.pace_skip = FRAME_LEN as u32;
        self.pace_left = self.interval;
        let buf = self.buf;
        let len = self.buf_len;
        self.buf_len = 0;
        for &b in &buf[..len] {
            self.scan(b);
        }
    }

    /// Walk the lookahead one step per window payload byte, skipping the sender's frame slots.
    fn pace(&mut self) {
        if self.pace_skip > 0 {
            self.pace_skip -= 1;
            return;
        }
        self.look.advance(1);
        self.pace_left -= 1;
        if self.pace_left == 0 {
            self.pace_left = self.interval;
            self.pace_skip = FRAME_LEN as u32;
        }
    }

    /// Look for a frame ending at this byte, counting everything else as window payload.
    fn scan(&mut self, b: u8) {
        self.window += 1;
        self.pace();
        self.ring[self.ring_pos] = b;
        self.ring_pos = (self.ring_pos + 1) % FRAME_LEN;
        if self.ring_len < FRAME_LEN {
            self.ring_len += 1;
            // A frame can end on the byte that fills the ring, so test it on the same pass.
            if self.ring_len < FRAME_LEN {
                return;
            }
        }
        if !self.ring_holds_magic() {
            return;
        }
        let mut candidate = [0u8; FRAME_LEN];
        for (i, slot) in candidate.iter_mut().enumerate() {
            *slot = self.ring[(self.ring_pos + i) % FRAME_LEN];
        }
        let frame = Frame::decode(&candidate).filter(|frame| frame.dir == self.dir);
        let Some(frame) = frame else {
            return;
        };
        if frame.kind == FrameKind::Credit {
            self.take_frame();
            self.grant(frame.offset);
            return;
        }
        if !self.plausible(&frame) {
            return;
        }
        self.take_frame();
        // A frame from before the window is replayed data: keep scanning for the live stream.
        if frame.offset < self.span_start {
            return;
        }
        self.anchor(&frame);
    }

    /// Consume the frame the ring is holding; its bytes were never payload.
    fn take_frame(&mut self) {
        self.window = self.window.saturating_sub(FRAME_LEN as u64);
        self.counters.frames += 1;
        self.ring_len = 0;
        self.ring_pos = 0;
    }

    /// A candidate frame is trusted only inside the span its sequence implies, never past budget.
    fn plausible(&self, frame: &Frame) -> bool {
        let interval = u64::from(self.interval);
        match frame.kind {
            FrameKind::Marker => {
                frame.offset == (u64::from(frame.seq) + 1) * interval && frame.offset <= self.budget
            }
            FrameKind::EndOfStream => {
                let base = u64::from(frame.seq) * interval;
                frame.offset >= base
                    && frame.offset <= base + interval
                    && frame.offset <= self.budget
            }
            FrameKind::Credit => true,
        }
    }

    /// Close a resync window against the offsets the two frames around it carry.
    fn anchor(&mut self, frame: &Frame) {
        let delta = frame.offset - self.span_start;
        let inner = self.frames_between(self.span_start, frame.offset);
        let received = self.window.saturating_sub(inner * FRAME_LEN as u64);
        self.counters.unresolved += delta.min(received);
        self.counters.missing += delta.saturating_sub(received);
        self.counters.duplicate += received.saturating_sub(delta);
        let (mut prng, owed) = self.nearest(frame.offset);
        prng.advance(owed);
        self.prng = prng;
        self.commit(frame.offset, frame.seq);
        self.resume(frame);
    }

    /// The generator standing closest at or before `offset`, and the bytes it still owes.
    fn nearest(&self, offset: u64) -> (Prng, u64) {
        let mut best = self.span_prng;
        for candidate in [self.prng, self.look] {
            if candidate.position() <= offset && candidate.position() > best.position() {
                best = candidate;
            }
        }
        (best, offset - best.position())
    }

    /// Frames the sender must have placed strictly between two frame offsets.
    fn frames_between(&self, from: u64, to: u64) -> u64 {
        if to == 0 {
            return 0;
        }
        let interval = u64::from(self.interval);
        let last = if to.is_multiple_of(interval) {
            to / interval - 1
        } else {
            to / interval
        };
        last.saturating_sub(from / interval)
    }

    /// Start a fresh span at an accepted frame.
    fn commit(&mut self, offset: u64, seq: u32) {
        self.offset = offset;
        self.span_start = offset;
        self.span_prng = self.prng;
        self.span = Crc32::new();
        self.span_ok = 0;
        self.span_bad = 0;
        self.expect_seq = seq.wrapping_add(1);
        self.window = 0;
    }

    /// Credit grants are cumulative and max-wins.
    fn grant(&mut self, total: u64) {
        if total > self.counters.credit {
            self.counters.credit = total;
        }
    }

    fn ring_holds_magic(&self) -> bool {
        super::MAGIC
            .iter()
            .enumerate()
            .all(|(i, &b)| self.ring[(self.ring_pos + i) % FRAME_LEN] == b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v1::{BurstSpec, DEFAULT_INTERVAL};
    use crate::v1::{Encoder, TxCounters};

    const IV: u32 = 64;
    const BUDGET: u64 = 200;
    const SEED: u32 = 1;

    /// One whole run: 64-byte spans closed by markers, ended at 200 payload bytes.
    fn run(out: &mut [u8]) -> usize {
        let mut encoder = Encoder::new(Direction::DutToHost, SEED, IV, BUDGET);
        let mut written = 0;
        while !encoder.complete() {
            let n = encoder.fill(&mut out[written..], 0);
            assert!(n > 0, "the encoder stalled at {written}");
            written += n;
        }
        written
    }

    fn armed() -> Verifier {
        let mut verifier = Verifier::new(Direction::DutToHost, SEED, IV, BUDGET);
        verifier.arm(0);
        verifier
    }

    /// Feed in ragged chunks, so frames and spans get cut at arbitrary boundaries.
    fn feed(verifier: &mut Verifier, wire: &[u8]) {
        let mut now = 0;
        for chunk in wire.chunks(7) {
            now += 100;
            verifier.feed(chunk, now);
        }
    }

    fn verify(wire: &[u8]) -> RxCounters {
        let mut verifier = armed();
        feed(&mut verifier, wire);
        verifier.counters()
    }

    /// Copy `wire` with one byte dropped.
    fn without(wire: &[u8], at: usize, out: &mut [u8]) -> usize {
        out[..at].copy_from_slice(&wire[..at]);
        out[at..wire.len() - 1].copy_from_slice(&wire[at + 1..]);
        wire.len() - 1
    }

    /// Copy `wire` with one byte inserted.
    fn with(wire: &[u8], at: usize, byte: u8, out: &mut [u8]) -> usize {
        out[..at].copy_from_slice(&wire[..at]);
        out[at] = byte;
        out[at + 1..wire.len() + 1].copy_from_slice(&wire[at..]);
        wire.len() + 1
    }

    #[test]
    fn a_clean_run_verifies_every_byte_and_reconciles() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let counters = verify(&wire[..n]);
        assert_eq!(
            counters,
            RxCounters {
                verified: BUDGET,
                corrupt: 0,
                missing: 0,
                duplicate: 0,
                unresolved: 0,
                frames: 4,
                resyncs: 0,
                eos: true,
                eos_total: BUDGET,
                credit: 0,
                elapsed_us: counters.elapsed_us,
                onset_us: counters.onset_us,
                maxgap_us: counters.maxgap_us,
            }
        );
        assert!(counters.reconciles() && counters.clean());
    }

    #[test]
    fn a_receiver_stays_in_sync_across_a_close_and_reconciles_on_the_total_sent() {
        // The receiver is armed for the whole budget and never told about the close.
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, SEED, IV, BUDGET);
        let mut n = encoder.fill(&mut wire[..100], 0);
        encoder.close();
        while !encoder.complete() {
            n += encoder.fill(&mut wire[n..], 0);
        }
        let counters = verify(&wire[..n]);
        assert_eq!(counters.eos_total, 2 * u64::from(IV));
        assert_eq!(counters.verified, 2 * u64::from(IV));
        assert_eq!(counters.resyncs, 0);
        assert!(counters.reconciles() && counters.clean());
    }

    #[test]
    fn a_resyncing_receiver_still_anchors_on_the_frame_that_closes_a_run() {
        // The loss lands in the last span, so the closing frame arrives mid-resync.
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, SEED, IV, BUDGET);
        let mut n = encoder.fill(&mut wire[..100], 0);
        encoder.close();
        while !encoder.complete() {
            n += encoder.fill(&mut wire[n..], 0);
        }
        let mut noisy = [0u8; 512];
        let n = with(&wire[..n], 110, 0x5A, &mut noisy);
        let counters = verify(&noisy[..n]);
        assert_eq!(counters.resyncs, 1);
        assert!(counters.eos, "the closing frame was not recognized");
        assert_eq!(counters.eos_total, 2 * u64::from(IV));
        assert_eq!(counters.duplicate, 1);
        assert!(counters.reconciles());
    }

    #[test]
    fn a_credit_run_closed_out_of_credit_still_reconciles() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, SEED, IV, BUDGET).with_credit();
        encoder.set_grant(20);
        let mut n = 0;
        loop {
            let filled = encoder.fill(&mut wire[n..], 0);
            n += filled;
            if filled == 0 {
                break;
            }
        }
        assert_eq!(n, 20, "the grant bounds what leaves the transmitter");
        encoder.close();
        while !encoder.complete() {
            n += encoder.fill(&mut wire[n..], 0);
        }
        let counters = verify(&wire[..n]);
        assert_eq!(counters.eos_total, u64::from(IV));
        assert!(counters.reconciles() && counters.clean());
    }

    #[test]
    fn a_clean_run_at_the_default_interval_verifies_every_byte() {
        const BYTES: u64 = 3 * DEFAULT_INTERVAL as u64 + 100;
        let mut wire = [0u8; 8192];
        let mut encoder = Encoder::new(Direction::DutToHost, 9, DEFAULT_INTERVAL, BYTES);
        let mut n = 0;
        while !encoder.complete() {
            n += encoder.fill(&mut wire[n..], 0);
        }
        let mut verifier = Verifier::new(Direction::DutToHost, 9, DEFAULT_INTERVAL, BYTES);
        verifier.arm(0);
        feed(&mut verifier, &wire[..n]);
        let counters = verifier.counters();
        assert_eq!(counters.verified, BYTES);
        assert_eq!(counters.frames, 4);
        assert!(counters.reconciles() && counters.clean());
        assert_eq!(
            encoder.counters(),
            TxCounters {
                payload: BYTES,
                frames: 4,
                credit: 0,
                complete: true,
            }
        );
    }

    #[test]
    fn a_corrupt_byte_in_sync_is_counted_without_a_resync() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        wire[100] ^= 0xFF;
        let counters = verify(&wire[..n]);
        assert_eq!(counters.corrupt, 1);
        assert_eq!(counters.verified, BUDGET - 1);
        assert_eq!(counters.resyncs, 0);
        assert_eq!(counters.frames, 4);
        assert!(counters.reconciles());
    }

    #[test]
    fn a_dropped_byte_is_missing_and_leaves_the_later_spans_alone() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let mut lossy = [0u8; 512];
        let len = without(&wire[..n], 10, &mut lossy);

        let counters = verify(&lossy[..len]);
        assert_eq!(counters.missing, 1);
        assert_eq!(counters.duplicate, 0);
        assert_eq!(counters.resyncs, 1);
        // The window swallows the span the byte fell out of and the one the verifier scanned.
        assert_eq!(counters.unresolved, 127);
        assert_eq!(counters.verified, BUDGET - 128);
        assert!(counters.reconciles());
    }

    #[test]
    fn a_deletion_at_a_frame_boundary_stays_bounded_by_its_own_span() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let mut lossy = [0u8; 512];
        // The last payload byte of span 0, so the verifier eats into the marker itself.
        let len = without(&wire[..n], 63, &mut lossy);

        let counters = verify(&lossy[..len]);
        assert_eq!(counters.missing, 1);
        assert_eq!(counters.duplicate, 0);
        assert_eq!(counters.unresolved, 127);
        assert_eq!(counters.verified, BUDGET - 128);
        assert!(counters.reconciles());
    }

    #[test]
    fn an_injected_byte_is_duplicate_and_costs_only_its_own_span() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let mut noisy = [0u8; 512];
        let len = with(&wire[..n], 10, 0x5A, &mut noisy);

        let counters = verify(&noisy[..len]);
        assert_eq!(counters.duplicate, 1);
        assert_eq!(counters.missing, 0);
        assert_eq!(counters.resyncs, 1);
        assert_eq!(counters.unresolved, u64::from(IV));
        assert_eq!(counters.verified, BUDGET - u64::from(IV));
        assert!(counters.reconciles());
    }

    #[test]
    fn an_insertion_at_a_frame_boundary_stays_bounded_by_its_own_span() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let mut noisy = [0u8; 512];
        // Between the last payload byte of span 0 and its marker.
        let len = with(&wire[..n], 64, 0xA5, &mut noisy);

        let counters = verify(&noisy[..len]);
        assert_eq!(counters.duplicate, 1);
        assert_eq!(counters.missing, 0);
        assert_eq!(counters.unresolved, u64::from(IV));
        assert_eq!(counters.verified, BUDGET - u64::from(IV));
        assert!(counters.reconciles());
    }

    #[test]
    fn a_replayed_span_is_duplicate_and_nothing_else() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        // Span 1 and its marker, sent twice.
        let mut echoed = [0u8; 512];
        echoed[..176].copy_from_slice(&wire[..176]);
        echoed[176..264].copy_from_slice(&wire[88..176]);
        echoed[264..n + 88].copy_from_slice(&wire[176..n]);

        let counters = verify(&echoed[..n + 88]);
        assert_eq!(counters.duplicate, u64::from(IV));
        assert_eq!(counters.missing, 0);
        assert_eq!(counters.unresolved, 0);
        assert_eq!(counters.verified, BUDGET);
        assert_eq!(counters.resyncs, 1);
        assert!(counters.reconciles());
    }

    #[test]
    fn a_false_magic_in_the_payload_is_rejected_by_the_frame_itself() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let mut lossy = [0u8; 512];
        let len = without(&wire[..n], 10, &mut lossy);
        let reference = verify(&lossy[..len]);

        // A frame the payload could plausibly spell out, at a plausible offset.
        let forged = Frame {
            kind: FrameKind::Marker,
            dir: Direction::DutToHost,
            seq: 1,
            offset: 128,
            crc: 0,
        }
        .encode();

        let mut broken_crc = lossy;
        broken_crc[100..124].copy_from_slice(&forged);
        broken_crc[122] ^= 0x01;
        assert_eq!(
            verify(&broken_crc[..len]),
            reference,
            "header CRC16 ignored"
        );

        let mut implausible = lossy;
        let mut off_offset = forged;
        off_offset[10] = 129;
        let header = crate::v1::crc16(&off_offset[..22]);
        off_offset[22..].copy_from_slice(&header.to_le_bytes());
        implausible[100..124].copy_from_slice(&off_offset);
        assert_eq!(
            verify(&implausible[..len]),
            reference,
            "seq and offset were not checked against each other"
        );

        // The controls only mean something because an intact frame there is accepted.
        let mut accepted = lossy;
        accepted[100..124].copy_from_slice(&forged);
        assert_ne!(verify(&accepted[..len]), reference);
    }

    #[test]
    fn a_run_without_its_end_of_stream_frame_does_not_reconcile() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let counters = verify(&wire[..n - FRAME_LEN]);
        assert!(!counters.eos);
        assert!(!counters.reconciles());
        assert_eq!(counters.eos_total, 0);
    }

    #[test]
    fn bytes_after_the_end_of_stream_frame_are_counted_duplicate() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        wire[n..n + 5].copy_from_slice(&[0; 5]);
        let counters = verify(&wire[..n + 5]);
        assert!(counters.eos);
        assert_eq!(counters.duplicate, 5);
    }

    #[test]
    fn timing_runs_from_arming_and_records_the_worst_gap() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let mut verifier = Verifier::new(Direction::DutToHost, SEED, IV, BUDGET);
        verifier.arm(1_000);
        // An empty feed is a poll, not an arrival.
        verifier.feed(&[], 2_000);
        verifier.feed(&wire[..64], 5_000);
        verifier.feed(&wire[64..128], 6_000);
        verifier.feed(&[], 8_000);
        verifier.feed(&wire[128..n], 9_000);
        let counters = verifier.counters();
        assert_eq!(counters.onset_us, 4_000);
        assert_eq!(counters.maxgap_us, 3_000);
        assert_eq!(counters.elapsed_us, 8_000);
        assert!(counters.reconciles());
    }

    #[test]
    fn credit_frames_carry_grants_and_the_highest_one_wins() {
        const CREDIT_BUDGET: u64 = 200;
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::HostToDut, SEED, IV, CREDIT_BUDGET);
        encoder.queue_credit(4096);
        let mut n = encoder.fill(&mut wire[..112], 0);
        // A later grant below the one already announced must not walk the window back.
        encoder.queue_credit(1000);
        while !encoder.complete() {
            n += encoder.fill(&mut wire[n..], 0);
        }

        let mut verifier =
            Verifier::new(Direction::HostToDut, SEED, IV, CREDIT_BUDGET).with_credit();
        verifier.arm(0);
        feed(&mut verifier, &wire[..n]);
        let counters = verifier.counters();
        assert_eq!(counters.credit, 4096);
        assert_eq!(verifier.credit(), 4096);
        // Credit frames are not payload: they advance no offset and enter no span.
        assert_eq!(counters.verified, CREDIT_BUDGET);
        assert_eq!(counters.frames, 6);
        assert!(counters.reconciles() && counters.clean());
    }

    #[test]
    fn a_burst_shaped_stream_verifies_like_any_other() {
        let mut wire = [0u8; 512];
        let mut encoder =
            Encoder::new(Direction::DutToHost, SEED, IV, BUDGET).with_burst(BurstSpec {
                bytes: 48,
                idle_ms: 1,
            });
        let mut n = 0;
        let mut now = 0;
        while !encoder.complete() {
            n += encoder.fill(&mut wire[n..], now);
            now += 1_000;
        }
        let counters = verify(&wire[..n]);
        assert_eq!(counters.verified, BUDGET);
        assert!(counters.reconciles() && counters.clean());
    }

    /// One whole run at the interval the rigs actually use, into a caller-sized buffer.
    fn wide_run(out: &mut [u8], budget: u64) -> usize {
        let mut encoder = Encoder::new(Direction::DutToHost, SEED, DEFAULT_INTERVAL, budget);
        let mut written = 0;
        while !encoder.complete() {
            written += encoder.fill(&mut out[written..], 0);
        }
        written
    }

    fn wide_armed(budget: u64) -> Verifier {
        let mut verifier = Verifier::new(Direction::DutToHost, SEED, DEFAULT_INTERVAL, budget);
        verifier.arm(0);
        verifier
    }

    #[test]
    fn an_anchor_advances_the_generator_by_what_the_window_lost_not_by_the_whole_span() {
        const BUDGET: u64 = 4 * DEFAULT_INTERVAL as u64;
        let mut wire = [0u8; 9000];
        let n = wide_run(&mut wire, BUDGET);
        let mut lossy = [0u8; 9000];
        let len = without(&wire[..n], 10, &mut lossy);

        // Stop one byte short of the marker the window closes on, which is the second one.
        let anchor = 2 * u64::from(DEFAULT_INTERVAL);
        let upto = 2 * (DEFAULT_INTERVAL as usize + FRAME_LEN) - 2;
        let mut verifier = wide_armed(BUDGET);
        verifier.feed(&lossy[..upto], 0);
        assert_eq!(verifier.state, State::Resync);
        // The span-start generator is two whole spans behind the anchor; the lookahead is not.
        assert_eq!(verifier.span_prng.position(), 0);
        let (_, owed) = verifier.nearest(anchor);
        assert!(owed <= 4, "the anchor owes {owed} generator bytes");

        verifier.feed(&lossy[upto..len], 0);
        let counters = verifier.counters();
        assert_eq!(counters.missing, 1);
        assert_eq!(counters.resyncs, 1);
        assert_eq!(counters.verified, BUDGET - counters.unresolved - 1);
        assert!(counters.reconciles());
    }

    #[test]
    fn an_anchor_past_a_whole_lost_span_still_lands_on_the_right_generator() {
        const BUDGET: u64 = 5 * DEFAULT_INTERVAL as u64;
        let mut wire = [0u8; 11_000];
        let n = wide_run(&mut wire, BUDGET);

        // Cut span 1 and its marker out entirely, so the lookahead cannot have reached the anchor.
        let span = DEFAULT_INTERVAL as usize + FRAME_LEN;
        let mut lossy = [0u8; 11_000];
        lossy[..span].copy_from_slice(&wire[..span]);
        lossy[span..n - span].copy_from_slice(&wire[2 * span..n]);
        let len = n - span;

        let mut verifier = wide_armed(BUDGET);
        feed(&mut verifier, &lossy[..len]);
        let counters = verifier.counters();
        assert_eq!(counters.corrupt, 0);
        assert_eq!(counters.duplicate, 0);
        // The window covers the lost span and the one the verifier scanned through.
        assert_eq!(
            counters.missing + counters.unresolved,
            2 * u64::from(DEFAULT_INTERVAL)
        );
        assert!(counters.reconciles());
        // Every later span was scored against the generator, so the anchor restored it exactly.
        assert_eq!(counters.verified, 3 * u64::from(DEFAULT_INTERVAL));
    }

    #[test]
    fn reset_starts_the_ledger_over() {
        let mut wire = [0u8; 512];
        let n = run(&mut wire);
        let mut verifier = armed();
        feed(&mut verifier, &wire[..n]);
        verifier.reset();
        assert_eq!(verifier.counters(), RxCounters::default());
        assert!(!verifier.complete());
        verifier.arm(0);
        feed(&mut verifier, &wire[..n]);
        assert_eq!(verifier.counters().verified, BUDGET);
    }
}
