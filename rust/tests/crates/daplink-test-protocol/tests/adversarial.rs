//! Streams the encoder never produced, judged against ground truth carried by the mutation
//! itself, so the classifier's verdicts are checked for soundness rather than self-consistency.

use daplink_test_protocol::v1::{
    Direction, Encoder, Frame, FrameKind, RxCounters, Verifier, FRAME_LEN, MAGIC,
};

/// Cases per family; every case derives from its number alone, so a failure replays exactly.
const CASES: u64 = 500;

/// xorshift64* with a splitmix-style seed scramble.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }

    fn range(&mut self, lo: u64, hi: u64) -> u64 {
        lo + self.below(hi - lo + 1)
    }
}

/// One case's stream geometry, drawn from its RNG.
struct Case {
    seed: u32,
    interval: u32,
    budget: u64,
}

impl Case {
    fn draw(rng: &mut Rng) -> Self {
        let interval = [24, 32, 64, 100][rng.below(4) as usize];
        let spans = rng.range(3, 6);
        Self {
            seed: rng.next() as u32,
            interval,
            budget: spans * u64::from(interval) + rng.below(u64::from(interval)),
        }
    }

    /// The clean wire image of the whole run.
    fn wire(&self) -> Vec<u8> {
        let mut encoder = Encoder::new(Direction::DutToHost, self.seed, self.interval, self.budget);
        let mut out = vec![0u8; 4 * self.budget as usize + 512];
        let mut n = 0;
        while !encoder.complete() {
            let filled = encoder.fill(&mut out[n..], 0);
            assert!(filled > 0, "the encoder stalled at {n}");
            n += filled;
        }
        out.truncate(n);
        out
    }

    /// Feed `stream` in ragged random-sized chunks and return the ledger.
    fn verify(&self, stream: &[u8], rng: &mut Rng) -> RxCounters {
        let mut verifier =
            Verifier::new(Direction::DutToHost, self.seed, self.interval, self.budget);
        verifier.arm(0);
        let mut now = 0;
        let mut i = 0;
        while i < stream.len() {
            let n = (rng.range(1, 96) as usize).min(stream.len() - i);
            now += 100;
            verifier.feed(&stream[i..i + n], now);
            i += n;
        }
        verifier.counters()
    }

    /// Whether wire position `at` holds a payload byte rather than a frame byte.
    fn is_payload(&self, at: usize) -> bool {
        let period = self.interval as usize + FRAME_LEN;
        let within = at % period;
        within < self.interval as usize
            && (at / period) as u64 * u64::from(self.interval) + (within as u64) < self.budget
    }
}

/// The bound no honest receiver can break: scored payload never exceeds what was ever sent.
fn sound(case: u64, c: &RxCounters, budget: u64) {
    assert!(
        c.verified + c.corrupt <= budget,
        "case {case}: scored more payload than the sender emitted: {c:?}"
    );
}

#[test]
fn corruption_is_never_read_as_loss_or_duplication() {
    for case in 0..CASES {
        let mut rng = Rng::new(case ^ 0xC0FF_EE00);
        let geom = Case::draw(&mut rng);
        let clean = geom.wire();
        let mut stream = clean.clone();
        // Leave the end-of-stream frame alone so the run still closes.
        let editable = (stream.len() - FRAME_LEN) as u64;
        let mut payload_hits = 0u64;
        for _ in 0..rng.range(1, 6) {
            let at = rng.below(editable) as usize;
            if stream[at] != clean[at] {
                continue;
            }
            stream[at] ^= rng.range(1, 255) as u8;
            if geom.is_payload(at) {
                payload_hits += 1;
            }
        }
        let c = geom.verify(&stream, &mut rng);
        sound(case, &c, geom.budget);
        assert!(c.eos && c.reconciles(), "case {case}: {c:?}");
        assert_eq!(
            c.missing, 0,
            "case {case}: substitution read as loss: {c:?}"
        );
        assert_eq!(
            c.duplicate, 0,
            "case {case}: substitution read as duplication: {c:?}"
        );
        assert!(
            c.corrupt + c.unresolved >= payload_hits,
            "case {case}: corrupted bytes vanished from the ledger: {c:?}"
        );
        assert_eq!(
            c.verified + c.corrupt + c.unresolved,
            geom.budget,
            "case {case}: {c:?}"
        );
    }
}

#[test]
fn a_spliced_gap_is_charged_to_the_sender_at_exactly_its_size() {
    for case in 0..CASES {
        let mut rng = Rng::new(case ^ 0xDE1E_7E00);
        let geom = Case::draw(&mut rng);
        let clean = geom.wire();
        let d = rng.range(1, u64::from(geom.interval)) as usize;
        // The gap must end before the payload run the end-of-stream frame closes: a gap in
        // that tail leaves no later frame to anchor on, and only reconciliation can see it.
        let period = geom.interval as usize + FRAME_LEN;
        let full = (geom.budget / u64::from(geom.interval)) as usize;
        let limit = if !geom.budget.is_multiple_of(u64::from(geom.interval)) {
            full * period
        } else {
            (full - 1) * period
        };
        let at = rng.below((limit - d + 1) as u64) as usize;
        // From the gap's span, slot plus frame buffer must still fit before the end-of-stream
        // frame, or the verifier consumes it blind and only reconciliation sees the loss.
        if clean.len() - (at / period) * period - d < geom.interval as usize + FRAME_LEN {
            continue;
        }
        let mut stream = clean[..at].to_vec();
        stream.extend_from_slice(&clean[at + d..]);
        let c = geom.verify(&stream, &mut rng);
        sound(case, &c, geom.budget);
        assert!(c.eos && c.reconciles(), "case {case}: d={d} at={at}: {c:?}");
        assert_eq!(
            c.corrupt, 0,
            "case {case}: a gap scored as corruption: {c:?}"
        );
        assert_eq!(
            c.duplicate, 0,
            "case {case}: a gap scored as duplication: {c:?}"
        );
        assert_eq!(c.missing, d as u64, "case {case}: d={d} at={at}: {c:?}");
    }
}

#[test]
fn a_gap_in_the_final_tail_is_witnessed_by_reconciliation_alone() {
    for case in 0..CASES {
        let mut rng = Rng::new(case ^ 0x7A11_BA11);
        let geom = Case::draw(&mut rng);
        let clean = geom.wire();
        let period = geom.interval as usize + FRAME_LEN;
        let full = (geom.budget / u64::from(geom.interval)) as usize;
        let r = (geom.budget % u64::from(geom.interval)) as usize;
        // The payload run the end-of-stream frame closes; no later frame can anchor a gap here.
        let (tail_start, tail_len) = if r > 0 {
            (full * period, r)
        } else {
            ((full - 1) * period, geom.interval as usize)
        };
        let d = rng.range(1, tail_len as u64) as usize;
        let at = tail_start + rng.below((tail_len - d) as u64 + 1) as usize;
        let mut stream = clean[..at].to_vec();
        stream.extend_from_slice(&clean[at + d..]);
        let c = geom.verify(&stream, &mut rng);
        sound(case, &c, geom.budget);
        // The ledger cannot attribute the loss, but it must not paper it over either.
        assert!(
            !c.eos && !c.reconciles(),
            "case {case}: d={d} at={at}: {c:?}"
        );
        assert_eq!(c.corrupt, 0, "case {case}: d={d} at={at}: {c:?}");
        assert_eq!(c.duplicate, 0, "case {case}: d={d} at={at}: {c:?}");
        assert_eq!(
            c.verified,
            geom.budget - tail_len as u64,
            "case {case}: d={d} at={at}: {c:?}"
        );
    }
}

#[test]
fn a_replayed_span_is_pure_duplication() {
    for case in 0..CASES {
        let mut rng = Rng::new(case ^ 0x5EED_0000);
        let geom = Case::draw(&mut rng);
        let clean = geom.wire();
        let period = geom.interval as usize + FRAME_LEN;
        // A full span and its marker, sent again immediately after themselves.
        let s = rng.below(geom.budget / u64::from(geom.interval) - 1) as usize;
        let mut stream = clean[..(s + 1) * period].to_vec();
        stream.extend_from_slice(&clean[s * period..(s + 1) * period]);
        stream.extend_from_slice(&clean[(s + 1) * period..]);
        let c = geom.verify(&stream, &mut rng);
        sound(case, &c, geom.budget);
        assert!(c.eos && c.reconciles(), "case {case}: s={s}: {c:?}");
        assert_eq!(
            (c.missing, c.corrupt, c.unresolved),
            (0, 0, 0),
            "case {case}: s={s}: {c:?}"
        );
        assert_eq!(
            c.duplicate,
            u64::from(geom.interval),
            "case {case}: s={s}: {c:?}"
        );
        assert_eq!(c.verified, geom.budget, "case {case}: s={s}: {c:?}");
    }
}

#[test]
fn a_truncated_stream_never_claims_to_reconcile() {
    for case in 0..CASES {
        let mut rng = Rng::new(case ^ 0x7A11_0000);
        let geom = Case::draw(&mut rng);
        let clean = geom.wire();
        let keep = rng.below(clean.len() as u64) as usize;
        let c = geom.verify(&clean[..keep], &mut rng);
        sound(case, &c, geom.budget);
        assert!(
            !c.eos,
            "case {case}: keep={keep}: a cut-off run claims its end: {c:?}"
        );
        assert!(!c.reconciles(), "case {case}: keep={keep}: {c:?}");
        assert_eq!(c.corrupt, 0, "case {case}: keep={keep}: {c:?}");
        assert_eq!(c.duplicate, 0, "case {case}: keep={keep}: {c:?}");
    }
}

#[test]
fn garbage_with_marker_bait_earns_no_ledger_at_all() {
    for case in 0..CASES {
        let mut rng = Rng::new(case ^ 0x6A5B_A6E0);
        let geom = Case::draw(&mut rng);
        let n = rng.range(64, 2048) as usize;
        let mut stream: Vec<u8> = (0..n).map(|_| rng.next() as u8).collect();
        // Bait: bare magics, a truncated genuine frame, and a CRC-valid but implausible marker.
        for _ in 0..rng.range(1, 4) {
            let at = rng.below((n - 4) as u64) as usize;
            stream[at..at + 4].copy_from_slice(&MAGIC);
        }
        let genuine = Frame {
            kind: FrameKind::Marker,
            dir: Direction::DutToHost,
            seq: 2,
            offset: 3 * u64::from(geom.interval),
            crc: rng.next() as u32,
        }
        .encode();
        let cut = rng.range(5, FRAME_LEN as u64 - 1) as usize;
        let at = rng.below((n - cut) as u64) as usize;
        stream[at..at + cut].copy_from_slice(&genuine[..cut]);
        let forged = Frame {
            kind: FrameKind::Marker,
            dir: Direction::DutToHost,
            seq: 5,
            offset: 999_999_937,
            crc: 0,
        }
        .encode();
        let at = rng.below((n - FRAME_LEN) as u64) as usize;
        stream[at..at + FRAME_LEN].copy_from_slice(&forged);
        let c = geom.verify(&stream, &mut rng);
        assert_eq!(
            (c.verified, c.corrupt, c.missing, c.duplicate, c.unresolved),
            (0, 0, 0, 0, 0),
            "case {case}: a foreign stream earned a ledger: {c:?}"
        );
        assert_eq!(c.frames, 0, "case {case}: {c:?}");
        assert!(!c.eos && !c.reconciles(), "case {case}: {c:?}");
    }
}

#[test]
fn marker_bait_inside_a_resync_window_does_not_bend_the_ledger() {
    for case in 0..CASES {
        let mut rng = Rng::new(case ^ 0xBA17_0000);
        let geom = Case::draw(&mut rng);
        let mut clean = geom.wire();
        let period = geom.interval as usize + FRAME_LEN;
        let interval = geom.interval as usize;
        let spans = (geom.budget / u64::from(geom.interval)) as usize;
        // Bait span s+1's payload with a bare magic, then open the window over it by deleting
        // `d` bytes out of span s's payload; both spans land inside the one resync window.
        let s = rng.below(spans as u64 - 1) as usize;
        let bait = (s + 1) * period + rng.below(interval as u64 - 7) as usize;
        clean[bait..bait + 4].copy_from_slice(&MAGIC);
        let d = rng.range(1, interval as u64 - 1) as usize;
        let at = s * period + rng.below((interval - d) as u64 + 1) as usize;
        let mut stream = clean[..at].to_vec();
        stream.extend_from_slice(&clean[at + d..]);
        let c = geom.verify(&stream, &mut rng);
        sound(case, &c, geom.budget);
        assert!(c.eos && c.reconciles(), "case {case}: s={s} d={d}: {c:?}");
        assert_eq!(c.duplicate, 0, "case {case}: s={s} d={d}: {c:?}");
        assert_eq!(c.corrupt, 0, "case {case}: s={s} d={d}: {c:?}");
        assert_eq!(c.missing, d as u64, "case {case}: s={s} d={d}: {c:?}");
        assert_eq!(
            c.unresolved,
            2 * interval as u64 - d as u64,
            "case {case}: s={s} d={d}: {c:?}"
        );
    }
}
