//! The transmit half: payload and frames, emitted a bufferful at a time.

use super::crc::Crc32;
use super::{BurstSpec, Direction, Frame, FrameKind, FRAME_LEN};
use crate::prng::Prng;

/// What a transmitter has put on the wire.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TxCounters {
    /// Payload bytes emitted, frames excluded.
    pub payload: u64,
    /// Frames emitted, of every kind.
    pub frames: u32,
    /// Cumulative grant the transmitter obeys; 0 unless credit-limited.
    pub credit: u64,
    /// The budget was met and the end-of-stream frame has left the encoder.
    pub complete: bool,
}

/// Marker-framed transmitter, resumable at any byte boundary: [`Encoder::fill`] stops early on
/// the budget, a credit grant or a burst gap, and ends the run with one end-of-stream frame.
#[derive(Debug, Clone)]
pub struct Encoder {
    dir: Direction,
    seed: u32,
    interval: u32,
    budget: u64,
    prng: Prng,
    payload: u64,
    frames: u32,
    seq: u32,
    credit_seq: u32,
    span: Crc32,
    /// Where a close moved the end of the stream to, which never extends the budget.
    close_at: Option<u64>,
    grant: Option<u64>,
    pending: Option<u64>,
    credit_due: bool,
    frame: [u8; FRAME_LEN],
    frame_pos: usize,
    done: bool,
    burst: Option<BurstSpec>,
    burst_left: u32,
    idle_until: u32,
    idling: bool,
}

impl Encoder {
    /// Arm a transmitter for `budget` payload bytes ([`super::UNBOUNDED`] runs until aborted);
    /// an interval below one frame is raised to [`FRAME_LEN`], so both ends agree on any input.
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
            prng: Prng::new(seed),
            payload: 0,
            frames: 0,
            seq: 0,
            credit_seq: 0,
            span: Crc32::new(),
            close_at: None,
            grant: None,
            pending: None,
            credit_due: false,
            frame: [0; FRAME_LEN],
            frame_pos: FRAME_LEN,
            done: false,
            burst: None,
            burst_left: 0,
            idle_until: 0,
            idling: false,
        }
    }

    /// Shape the stream into bursts separated by idle gaps.
    pub fn with_burst(mut self, burst: BurstSpec) -> Self {
        self.burst_left = burst.bytes;
        self.burst = Some(burst);
        self
    }

    /// Obey credit grants, starting from a grant of 0.
    pub fn with_credit(mut self) -> Self {
        self.grant = Some(0);
        self
    }

    /// Raise the cumulative grant this transmitter may emit up to; ignored unless credit-limited.
    pub fn set_grant(&mut self, total: u64) {
        if let Some(grant) = self.grant.as_mut() {
            if total > *grant {
                *grant = total;
            }
        }
    }

    /// Announce a cumulative grant to the far end at the next marker.
    pub fn queue_credit(&mut self, total: u64) {
        if self.pending.is_none_or(|pending| total > pending) {
            self.pending = Some(total);
        }
    }

    /// End the stream at the span in flight, replacing its closing frame with end-of-stream.
    pub fn close(&mut self) {
        if self.done {
            return;
        }
        self.close_at = Some(self.next_slot().min(self.budget));
    }

    /// Whether the stream has been closed short of its budget.
    pub fn closing(&self) -> bool {
        self.close_at.is_some()
    }

    /// Zero the run, keeping the configuration and reseeding the generator.
    pub fn reset(&mut self) {
        let burst = self.burst;
        let credit = self.grant.is_some();
        let mut fresh = Self::new(self.dir, self.seed, self.interval, self.budget);
        if let Some(burst) = burst {
            fresh = fresh.with_burst(burst);
        }
        if credit {
            fresh = fresh.with_credit();
        }
        *self = fresh;
    }

    /// The run's ledger.
    pub fn counters(&self) -> TxCounters {
        TxCounters {
            payload: self.payload,
            frames: self.frames,
            credit: self.grant.unwrap_or(0),
            complete: self.done && self.frame_pos == FRAME_LEN,
        }
    }

    /// The whole budget, including its end-of-stream frame, has been handed out.
    pub fn complete(&self) -> bool {
        self.done && self.frame_pos == FRAME_LEN
    }

    /// Write the next bytes into `buf`: a short return means idle, out of credit, or finished,
    /// never that the buffer was too small.
    pub fn fill(&mut self, buf: &mut [u8], now_us: u32) -> usize {
        let mut written = 0;
        while written < buf.len() {
            if self.frame_pos < FRAME_LEN {
                let take = (FRAME_LEN - self.frame_pos).min(buf.len() - written);
                buf[written..written + take]
                    .copy_from_slice(&self.frame[self.frame_pos..self.frame_pos + take]);
                self.frame_pos += take;
                written += take;
                self.charge_burst(take as u32, now_us);
                continue;
            }
            if self.done || !self.burst_ready(now_us) {
                break;
            }
            if self.credit_due {
                self.credit_due = false;
                if let Some(total) = self.pending.take() {
                    self.emit(FrameKind::Credit, total);
                    continue;
                }
            }
            if self.payload == self.limit() {
                self.emit(FrameKind::EndOfStream, self.payload);
                self.done = true;
                continue;
            }
            if self.payload == self.next_slot() {
                self.emit(FrameKind::Marker, self.payload);
                self.credit_due = true;
                continue;
            }
            let room = self.payload_room(buf.len() - written);
            if room == 0 {
                break;
            }
            let span = &mut buf[written..written + room];
            self.prng.fill(span);
            self.span.extend(span);
            self.payload += room as u64;
            written += room;
            self.charge_burst(room as u32, now_us);
        }
        written
    }

    /// Payload offset of the next marker.
    fn next_slot(&self) -> u64 {
        (u64::from(self.seq) + 1) * u64::from(self.interval)
    }

    /// Payload offset the stream ends at: its budget, or where a close moved it.
    fn limit(&self) -> u64 {
        self.close_at.unwrap_or(self.budget)
    }

    /// Payload bytes emittable now, given the buffer, the slot, the budget, credit, and burst.
    fn payload_room(&self, buf_room: usize) -> usize {
        let mut room = (self.next_slot() - self.payload).min(self.limit() - self.payload);
        if let Some(grant) = self.grant.filter(|_| self.close_at.is_none()) {
            room = room.min(grant.saturating_sub(self.payload));
        }
        if let Some(burst) = self.burst {
            if burst.bytes > 0 {
                room = room.min(u64::from(self.burst_left));
            }
        }
        room.min(buf_room as u64) as usize
    }

    /// Queue one frame; a marker or end-of-stream also closes the payload span.
    fn emit(&mut self, kind: FrameKind, offset: u64) {
        let crc = match kind {
            FrameKind::Credit => 0,
            _ => self.span.value(),
        };
        let seq = match kind {
            FrameKind::Credit => self.credit_seq,
            _ => self.seq,
        };
        self.frame = Frame {
            kind,
            dir: self.dir,
            seq,
            offset,
            crc,
        }
        .encode();
        self.frame_pos = 0;
        self.frames += 1;
        match kind {
            FrameKind::Credit => self.credit_seq = self.credit_seq.wrapping_add(1),
            _ => {
                self.seq = self.seq.wrapping_add(1);
                self.span = Crc32::new();
            }
        }
    }

    /// True unless the transmitter is inside a burst gap.
    fn burst_ready(&mut self, now_us: u32) -> bool {
        let Some(burst) = self.burst else {
            return true;
        };
        if burst.bytes == 0 {
            return true;
        }
        if self.idling {
            if now_us.wrapping_sub(self.idle_until) as i32 >= 0 {
                self.idling = false;
                self.burst_left = burst.bytes;
            } else {
                return false;
            }
        }
        true
    }

    /// Count wire bytes against the burst, opening the idle gap when it is spent.
    fn charge_burst(&mut self, n: u32, now_us: u32) {
        let Some(burst) = self.burst else {
            return;
        };
        if burst.bytes == 0 {
            return;
        }
        self.burst_left = self.burst_left.saturating_sub(n);
        if self.burst_left == 0 {
            self.idling = true;
            self.idle_until = now_us.wrapping_add(burst.idle_ms.saturating_mul(1000));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v1::UNBOUNDED;

    const IV: u32 = 64;

    /// Run an encoder to its end, or until it stalls, and return the wire bytes.
    fn drain(encoder: &mut Encoder, out: &mut [u8], now_us: u32) -> usize {
        let mut written = 0;
        loop {
            let n = encoder.fill(&mut out[written..], now_us);
            written += n;
            if n == 0 {
                return written;
            }
        }
    }

    fn frame_at(wire: &[u8], at: usize) -> Frame {
        let mut bytes = [0u8; FRAME_LEN];
        bytes.copy_from_slice(&wire[at..at + FRAME_LEN]);
        Frame::decode(&bytes).expect("frame")
    }

    #[test]
    fn a_run_is_payload_spans_closed_by_frames_and_one_end_of_stream() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, 200);
        let n = drain(&mut encoder, &mut wire, 0);
        // 200 payload bytes, three markers, one end-of-stream frame.
        assert_eq!(n, 200 + 4 * FRAME_LEN);
        assert!(encoder.complete());
        assert_eq!(
            encoder.counters(),
            TxCounters {
                payload: 200,
                frames: 4,
                credit: 0,
                complete: true,
            }
        );

        let mut expect = Prng::new(1);
        let mut span = [0u8; IV as usize];
        expect.fill(&mut span);
        assert_eq!(&wire[..IV as usize], &span);

        let marker = frame_at(&wire, IV as usize);
        assert_eq!(marker.kind, FrameKind::Marker);
        assert_eq!(marker.dir, Direction::DutToHost);
        assert_eq!(marker.seq, 0);
        assert_eq!(marker.offset, u64::from(IV));
        assert_eq!(marker.crc, crate::v1::crc32(&span));

        let eos = frame_at(&wire, n - FRAME_LEN);
        assert_eq!(eos.kind, FrameKind::EndOfStream);
        assert_eq!(eos.seq, 3);
        assert_eq!(eos.offset, 200);
    }

    #[test]
    fn a_budget_on_a_marker_offset_ends_with_end_of_stream_alone() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::HostToDut, 7, IV, 128);
        let n = drain(&mut encoder, &mut wire, 0);
        assert_eq!(n, 128 + 2 * FRAME_LEN);
        let last = frame_at(&wire, n - FRAME_LEN);
        assert_eq!(last.kind, FrameKind::EndOfStream);
        assert_eq!(last.seq, 1);
        assert_eq!(last.offset, 128);
    }

    #[test]
    fn the_stream_is_the_same_however_the_buffer_is_cut() {
        let mut whole = [0u8; 512];
        let mut piecewise = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 3, IV, 200);
        let n = drain(&mut encoder, &mut whole, 0);

        let mut encoder = Encoder::new(Direction::DutToHost, 3, IV, 200);
        let mut written = 0;
        while !encoder.complete() {
            written += encoder.fill(&mut piecewise[written..written + 1], 0);
        }
        assert_eq!(written, n);
        assert_eq!(whole[..n], piecewise[..n]);
    }

    #[test]
    fn an_unbounded_run_never_ends_by_itself() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, UNBOUNDED);
        assert_eq!(encoder.fill(&mut wire, 0), wire.len());
        assert!(!encoder.complete());
    }

    #[test]
    fn a_credit_limited_transmitter_stops_at_the_grant_and_resumes_when_it_grows() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, 200).with_credit();
        let mut at = drain(&mut encoder, &mut wire, 0);
        assert_eq!(at, 0, "an ungranted transmitter emits nothing");
        encoder.set_grant(40);
        at += drain(&mut encoder, &mut wire[at..], 0);
        assert_eq!(at, 40);
        assert_eq!(encoder.counters().payload, 40);

        // A replayed lower grant changes nothing; a higher one releases the rest of the run.
        encoder.set_grant(10);
        at += drain(&mut encoder, &mut wire[at..], 0);
        assert_eq!(at, 40);
        encoder.set_grant(200);
        at += drain(&mut encoder, &mut wire[at..], 0);
        assert_eq!(at, 200 + 4 * FRAME_LEN);
        assert!(encoder.complete());
        assert_eq!(encoder.counters().credit, 200);
    }

    #[test]
    fn a_queued_grant_leaves_as_a_credit_frame_after_the_next_marker() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::HostToDut, 1, IV, 200);
        encoder.queue_credit(4096);
        let n = drain(&mut encoder, &mut wire, 0);
        assert_eq!(n, 200 + 5 * FRAME_LEN);

        let credit = frame_at(&wire, IV as usize + FRAME_LEN);
        assert_eq!(credit.kind, FrameKind::Credit);
        assert_eq!(credit.dir, Direction::HostToDut);
        assert_eq!(credit.seq, 0);
        assert_eq!(credit.offset, 4096);
        assert_eq!(credit.crc, 0);
        // The credit frame is not payload: the next span picks up where the marker left off.
        let marker = frame_at(&wire, IV as usize + 2 * FRAME_LEN + IV as usize);
        assert_eq!(marker.offset, u64::from(2 * IV));
    }

    #[test]
    fn a_burst_transmitter_falls_silent_until_its_idle_gap_expires() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, 256).with_burst(BurstSpec {
            bytes: 32,
            idle_ms: 1,
        });
        assert_eq!(encoder.fill(&mut wire, 0), 32);
        assert_eq!(encoder.fill(&mut wire, 500), 0);
        assert_eq!(encoder.fill(&mut wire, 1_000), 32);
        // The gap is measured from the clock the caller passed, not from the previous gap.
        assert_eq!(encoder.fill(&mut wire, 1_500), 0);
        assert_eq!(encoder.fill(&mut wire, 2_000), 32);
    }

    #[test]
    fn a_closed_stream_ends_at_the_next_frame_slot_with_end_of_stream() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, 4096);
        let mut at = encoder.fill(&mut wire[..40], 0);
        assert_eq!(at, 40);
        encoder.close();
        assert!(encoder.closing());
        at += drain(&mut encoder, &mut wire[at..], 0);
        // The span in flight is filled to the slot, and the frame there is end-of-stream.
        assert_eq!(at, IV as usize + FRAME_LEN);
        assert!(encoder.complete());
        assert_eq!(encoder.counters().payload, u64::from(IV));
        let eos = frame_at(&wire, IV as usize);
        assert_eq!(eos.kind, FrameKind::EndOfStream);
        assert_eq!(eos.seq, 0);
        assert_eq!(eos.offset, u64::from(IV));
        // The close belongs to the run, not to the configuration a reset keeps.
        encoder.reset();
        assert!(!encoder.closing());
        assert_eq!(
            drain(&mut encoder, &mut [0u8; 8192], 0),
            4096 + 64 * FRAME_LEN
        );
    }

    #[test]
    fn a_close_on_the_slot_ends_the_stream_there() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, 4096);
        assert_eq!(encoder.fill(&mut wire[..IV as usize], 0), IV as usize);
        encoder.close();
        let n = drain(&mut encoder, &mut wire, 0);
        assert_eq!(n, FRAME_LEN);
        assert_eq!(frame_at(&wire, 0).kind, FrameKind::EndOfStream);
        assert_eq!(encoder.counters().payload, u64::from(IV));
    }

    #[test]
    fn closing_an_unbounded_run_ends_it() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, UNBOUNDED);
        assert_eq!(encoder.fill(&mut wire[..10], 0), 10);
        encoder.close();
        drain(&mut encoder, &mut wire, 0);
        assert!(encoder.complete());
        assert_eq!(encoder.counters().payload, u64::from(IV));
    }

    #[test]
    fn a_close_releases_a_credit_limited_transmitter_for_its_last_span() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, 4096).with_credit();
        encoder.set_grant(20);
        let mut at = drain(&mut encoder, &mut wire, 0);
        assert_eq!(at, 20);
        // Out of credit and no grant coming: the close still ends the stream cleanly.
        encoder.close();
        at += drain(&mut encoder, &mut wire[at..], 0);
        assert_eq!(at, IV as usize + FRAME_LEN);
        assert!(encoder.complete());
        assert_eq!(frame_at(&wire, IV as usize).kind, FrameKind::EndOfStream);
        // The grant the transmitter obeyed is still what the ledger reports.
        assert_eq!(encoder.counters().credit, 20);
    }

    #[test]
    fn a_close_never_extends_a_run_and_a_finished_one_ignores_it() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 1, IV, 40);
        encoder.close();
        let n = drain(&mut encoder, &mut wire, 0);
        assert_eq!(n, 40 + FRAME_LEN, "the shorter budget still ends the run");
        assert_eq!(encoder.counters().payload, 40);
        // A run that already met its budget is over; closing it changes nothing.
        let counters = encoder.counters();
        encoder.close();
        assert_eq!(drain(&mut encoder, &mut wire, 0), 0);
        assert_eq!(encoder.counters(), counters);
    }

    #[test]
    fn reset_starts_the_run_over_without_losing_the_configuration() {
        let mut wire = [0u8; 512];
        let mut encoder = Encoder::new(Direction::DutToHost, 5, IV, 200).with_credit();
        encoder.set_grant(200);
        let n = drain(&mut encoder, &mut wire, 0);
        encoder.reset();
        assert_eq!(encoder.counters(), TxCounters::default());
        // Credit mode survives, so the run stalls again until the next grant.
        assert_eq!(drain(&mut encoder, &mut wire, 0), 0);
        encoder.set_grant(200);
        let mut again = [0u8; 512];
        assert_eq!(drain(&mut encoder, &mut again, 0), n);
        assert_eq!(wire[..n], again[..n]);
    }
}
