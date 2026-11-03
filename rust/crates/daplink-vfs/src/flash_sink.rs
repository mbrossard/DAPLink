//! A [`TransferSink`] that programs decoded fragments into target flash via a [`FlashBackend`].

use crate::msc_io::SectorPoll;
use crate::render::StatusWriter;
use crate::transfer::{
    outcome_name, phase_name, DecodeError, TransferOutcome, TransferSink, TransferStats,
};
use daplink_core::{FlashBackend, FlashPhase, FlashPoll};
use daplink_decoder::Format;

fn in_range(ranges: &[(u32, u32)], addr: u32, len: usize) -> bool {
    let end = addr as u64 + len as u64;
    ranges.iter().any(|&(s, e)| addr >= s && end <= e as u64)
}

/// The largest decoded fragment one host sector can carry.
const MAX_FRAGMENT: usize = 512;

/// Where a resumable fragment is between backend steps.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SinkStep {
    /// Nothing outstanding.
    Idle,
    /// The backend's session is opening.
    Begin,
    /// The session is open; the staged fragment still has to pass the family and range gates.
    Check,
    /// Bytes of the staged fragment remain to be programmed.
    Program,
}

/// defmt-only counters for one drop, reset each transfer and reported at teardown.
#[cfg(feature = "defmt")]
#[derive(Default)]
struct DeferTrace {
    advance_ready: u32,
    advance_pending: u32,
    step_total: u32,
    finish_steps: u32,
    step_future: u32,
    step_now: u32,
}

/// Programs decoded image fragments into target flash, latching the first failing phase.
pub struct FlashSink<B: FlashBackend> {
    backend: B,
    auto_reset: bool,
    stats: TransferStats,
    /// Bytes in whole completed fragments, excluding partial acceptance.
    programmed: u32,
    latched: Option<FlashPhase>,
    swallow: bool,
    /// Whether `begin_with_hint` was attempted; lazy, so the first block's family can be the hint.
    started: bool,
    /// Whether a block was disregarded as foreign, which names the failure if nothing else arrives.
    foreign_family: bool,
    /// The family id declared by the block currently being delivered (`None` = it declares none).
    block_family: Option<u32>,
    /// Whether this transfer opened a flash session; one of the two reconnect grants.
    session_opened: bool,
    /// Whether this transfer was bound to a visible image dirent; the other reconnect grant.
    dirent_bound: bool,
    /// Whether teardown was already requested, so the backend finishes exactly once.
    released: bool,
    /// Where the fragment currently in flight is.
    step: SinkStep,
    /// The fragment in flight, held across the steps that program it.
    frag: [u8; MAX_FRAGMENT],
    /// Absolute target address of `frag`.
    frag_addr: u32,
    /// Valid bytes of `frag`.
    frag_len: usize,
    /// Bytes of `frag` already handed to the backend.
    frag_off: usize,
    /// Whether the caller pumps [`TransferSink::advance`].
    deferred: bool,
    /// The base outcome of a deferred teardown in flight, until `step` drains it.
    finishing: Option<TransferOutcome>,
    /// The absolute microsecond deadline the last backend step parked on.
    wake_at_us: Option<u64>,
    /// defmt-only deferral counters for the transfer in flight.
    #[cfg(feature = "defmt")]
    dbg: DeferTrace,
}

impl<B: FlashBackend> FlashSink<B> {
    /// Wrap `backend`, with auto-reset enabled.
    pub fn new(backend: B) -> Self {
        Self {
            backend,
            auto_reset: true,
            stats: TransferStats::default(),
            programmed: 0,
            latched: None,
            swallow: false,
            started: false,
            foreign_family: false,
            block_family: None,
            session_opened: false,
            dirent_bound: false,
            released: false,
            step: SinkStep::Idle,
            frag: [0; MAX_FRAGMENT],
            frag_addr: 0,
            frag_len: 0,
            frag_off: 0,
            deferred: false,
            finishing: None,
            wake_at_us: None,
            #[cfg(feature = "defmt")]
            dbg: DeferTrace::default(),
        }
    }

    /// Enable or disable resetting the target after a clean flash.
    pub fn set_auto_reset(&mut self, v: bool) {
        self.auto_reset = v;
    }

    /// Whether the target is reset after a clean flash.
    pub fn auto_reset(&self) -> bool {
        self.auto_reset
    }

    /// Borrow the backend.
    pub fn backend(&self) -> &B {
        &self.backend
    }

    /// The recorded outcome of the last transfer.
    pub fn outcome(&self) -> Option<TransferOutcome> {
        self.stats.outcome
    }

    /// Bytes delivered to the sink in the last transfer, programmed or not.
    pub fn bytes(&self) -> u32 {
        self.stats.bytes
    }

    /// Bytes in completed fragments; excludes partial acceptance and does not prove durability.
    pub fn programmed(&self) -> u32 {
        self.programmed
    }

    /// Fragment count of the last transfer.
    pub fn fragments(&self) -> u32 {
        self.stats.fragments
    }

    /// Close the session as soon as a failure latches instead of holding the wire to the timeout.
    fn release_early(&mut self) {
        if self.session_opened && !self.released {
            self.released = true;
            let _ = self.backend.finish_with_reset(false, false);
            #[cfg(feature = "defmt")]
            defmt::info!("flash sink: early session release (failure latched, stream swallowed)");
        }
    }

    /// Latch `phase`, swallow the rest of the stream, and drop whatever fragment was in flight.
    fn fail(&mut self, phase: FlashPhase, release: bool) {
        self.latched = Some(phase);
        self.swallow = true;
        self.step = SinkStep::Idle;
        self.frag_len = 0;
        self.frag_off = 0;
        if release {
            self.release_early();
        }
    }

    /// Drive the in-flight fragment to completion; the blocking form of `advance`.
    fn drain(&mut self) {
        while self.step != SinkStep::Idle {
            if let SectorPoll::Pending = self.advance_step(0) {
                if let Err(phase) = self.backend.settle() {
                    let opening = self.step == SinkStep::Begin;
                    if opening {
                        self.session_opened = false;
                    }
                    self.fail(phase, !opening);
                }
            }
        }
    }

    /// Step the deferred teardown by one bounded operation; `Pending` while flush or reset is owed.
    fn advance_finish(&mut self, now_us: u64) -> SectorPoll {
        let poll = self.backend.step(now_us);
        #[cfg(feature = "defmt")]
        self.dbg_note_step(poll, now_us, true);
        match poll {
            FlashPoll::Pending { wake_at_us } => {
                self.wake_at_us = Some(wake_at_us);
                SectorPoll::Pending
            }
            FlashPoll::Err(phase) => {
                self.latched = Some(phase);
                let outcome = self.finishing.take().expect("finishing while stepping");
                self.finalize(outcome);
                SectorPoll::Ready
            }
            FlashPoll::Done => {
                let outcome = self.finishing.take().expect("finishing while stepping");
                self.finalize(outcome);
                SectorPoll::Ready
            }
        }
    }

    /// Record the final outcome, folding a latched phase into a `FlashError`.
    fn finalize(&mut self, outcome: TransferOutcome) {
        self.stats.outcome = Some(match self.latched {
            Some(p) => TransferOutcome::FlashError(p),
            None => outcome,
        });
        // The engine's own trace never reports `FlashError`, so name the latched phase here.
        #[cfg(feature = "defmt")]
        if let Some(p) = self.latched {
            defmt::info!(
                "flash sink: transfer ABORT phase={=[u8]:a} (session released)",
                phase_name(p)
            );
        }
        // One compact tally per drop: write parks, backend steps, and the sleep/yield split.
        #[cfg(feature = "defmt")]
        defmt::info!(
            "flash defer: tally advance_ready={=u32} advance_pending={=u32} steps={=u32} finish_steps={=u32} pending_future={=u32} pending_now={=u32}",
            self.dbg.advance_ready,
            self.dbg.advance_pending,
            self.dbg.step_total,
            self.dbg.finish_steps,
            self.dbg.step_future,
            self.dbg.step_now
        );
    }

    /// Tally one backend step: total, teardown count, and whether it parked on a future deadline.
    #[cfg(feature = "defmt")]
    fn dbg_note_step(&mut self, poll: FlashPoll, now_us: u64, finish: bool) {
        self.dbg.step_total += 1;
        if finish {
            self.dbg.finish_steps += 1;
        }
        if let FlashPoll::Pending { wake_at_us } = poll {
            if wake_at_us > now_us {
                self.dbg.step_future += 1;
            } else {
                self.dbg.step_now += 1;
            }
        }
    }

    /// Advance the fragment in flight by at most one bounded device operation.
    fn advance_step(&mut self, now_us: u64) -> SectorPoll {
        loop {
            match self.step {
                SinkStep::Idle => return SectorPoll::Ready,
                SinkStep::Begin => {
                    let poll = self.backend.step(now_us);
                    #[cfg(feature = "defmt")]
                    self.dbg_note_step(poll, now_us, false);
                    match poll {
                        FlashPoll::Pending { wake_at_us } => {
                            self.wake_at_us = Some(wake_at_us);
                            return SectorPoll::Pending;
                        }
                        FlashPoll::Err(phase) => {
                            // The session never opened, so there is nothing to release.
                            self.fail(phase, false);
                            self.session_opened = false;
                            return SectorPoll::Ready;
                        }
                        FlashPoll::Done => self.step = SinkStep::Check,
                    }
                }
                SinkStep::Check => {
                    // A declared family disagreeing with the target's rejects before any erase.
                    if let Some(block) = self.block_family {
                        if !self.backend.accepts_uf2_family(block) {
                            self.fail(FlashPhase::Family, true);
                            return SectorPoll::Ready;
                        }
                    }
                    if !in_range(self.backend.flash_ranges(), self.frag_addr, self.frag_len) {
                        // Nothing is written, so this is `Range`, not `Program`.
                        self.fail(FlashPhase::Range, true);
                        return SectorPoll::Ready;
                    }
                    self.step = SinkStep::Program;
                }
                SinkStep::Program => {
                    let poll = self.backend.step(now_us);
                    #[cfg(feature = "defmt")]
                    self.dbg_note_step(poll, now_us, false);
                    match poll {
                        FlashPoll::Pending { wake_at_us } => {
                            self.wake_at_us = Some(wake_at_us);
                            return SectorPoll::Pending;
                        }
                        FlashPoll::Err(phase) => {
                            self.fail(phase, true);
                            return SectorPoll::Ready;
                        }
                        FlashPoll::Done => {}
                    }
                    if self.frag_off == self.frag_len {
                        // Counted whole, exactly as the unchunked program counted it.
                        self.programmed = self.programmed.saturating_add(self.frag_len as u32);
                        self.step = SinkStep::Idle;
                        self.frag_len = 0;
                        self.frag_off = 0;
                        return SectorPoll::Ready;
                    }
                    let addr = self.frag_addr + self.frag_off as u32;
                    match self
                        .backend
                        .program_chunk(addr, &self.frag[self.frag_off..self.frag_len])
                    {
                        Ok(taken) => self.frag_off += taken,
                        Err(phase) => {
                            self.fail(phase, true);
                            return SectorPoll::Ready;
                        }
                    }
                }
            }
        }
    }
}

impl<B: FlashBackend> TransferSink for FlashSink<B> {
    fn begin(&mut self, _fmt: Format, _sniffed: bool) {
        self.stats.reset();
        self.programmed = 0;
        self.latched = None;
        self.swallow = false;
        // The backend is begun lazily at the first fragment, not here.
        self.started = false;
        self.foreign_family = false;
        self.block_family = None;
        self.session_opened = false;
        self.dirent_bound = false;
        self.released = false;
        self.step = SinkStep::Idle;
        self.frag_len = 0;
        self.frag_off = 0;
        self.finishing = None;
        self.wake_at_us = None;
        #[cfg(feature = "defmt")]
        {
            self.dbg = DeferTrace::default();
        }
    }

    fn note_dirent_bound(&mut self) {
        self.dirent_bound = true;
    }

    fn note_uf2_family(&mut self, family: Option<u32>) -> bool {
        self.block_family = family;
        let Some(f) = family else { return true };
        // A selection left over from the last transfer must not judge this one.
        let ours = if self.started {
            self.backend.accepts_uf2_family(f)
        } else {
            self.backend.can_select_uf2_family(f)
        };
        self.foreign_family |= !ours;
        ours
    }

    fn fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), DecodeError> {
        // Only one fragment is ever in flight; a caller that does not step drains it here.
        self.drain();
        self.stats.record_fragment(data.len());
        if self.swallow {
            return Ok(());
        }
        if self.backend.requires_uf2_family() && self.block_family.is_none() {
            self.fail(FlashPhase::Family, true);
            return Ok(());
        }
        // A fragment wider than one host sector cannot be staged, and none exists.
        if data.len() > MAX_FRAGMENT {
            self.fail(FlashPhase::Range, true);
            return Ok(());
        }
        self.frag[..data.len()].copy_from_slice(data);
        self.frag_addr = addr;
        self.frag_len = data.len();
        self.frag_off = 0;
        // Begun at the first fragment, so the block's family can serve as the selection hint.
        if !self.started {
            self.started = true;
            if let Err(p) = self.backend.begin_start(self.block_family) {
                self.fail(p, false);
                return Ok(());
            }
            self.session_opened = true;
            self.step = SinkStep::Begin;
            // First fragment of the transfer: the caller's deferral state as the write arrives.
            #[cfg(feature = "defmt")]
            defmt::info!(
                "flash defer: begin deferred={=bool} family={}",
                self.deferred,
                self.block_family
            );
        } else {
            self.step = SinkStep::Check;
        }
        if !self.deferred {
            self.drain();
        }
        Ok(())
    }

    fn set_deferred(&mut self, on: bool) {
        self.deferred = on;
    }

    fn advance(&mut self, now_us: u64) -> SectorPoll {
        if self.finishing.is_some() {
            return self.advance_finish(now_us);
        }
        let poll = self.advance_step(now_us);
        #[cfg(feature = "defmt")]
        match poll {
            SectorPoll::Ready => self.dbg.advance_ready += 1,
            SectorPoll::Pending => self.dbg.advance_pending += 1,
        }
        poll
    }

    fn deferred_wake_us(&self) -> Option<u64> {
        self.wake_at_us
    }

    fn end(&mut self, outcome: TransferOutcome) {
        if self.finishing.is_some() || self.stats.outcome.is_some() {
            return;
        }
        // Whatever fragment is still in flight is programmed before the session is torn down.
        self.drain();
        // A stream that never delivered a fragment flashed nothing, so it did not succeed.
        let outcome = match outcome {
            TransferOutcome::Done if !self.started => {
                if self.foreign_family {
                    self.latched = Some(FlashPhase::Family);
                }
                TransferOutcome::NoData
            }
            other => other,
        };
        // A rejected begin owns no session, and an early release already requested teardown.
        if self.session_opened && !self.released {
            self.released = true;
            let clean = self.latched.is_none() && outcome == TransferOutcome::Done;
            if clean {
                // A deferred caller pumps the teardown via `advance`; a blocking caller finishes it here.
                match self.backend.finish_start(true, self.auto_reset) {
                    Ok(()) if self.deferred => {
                        #[cfg(feature = "defmt")]
                        defmt::info!("flash defer: end staged deferred teardown");
                        self.finishing = Some(outcome);
                        return;
                    }
                    Ok(()) => {
                        #[cfg(feature = "defmt")]
                        defmt::info!(
                            "flash defer: end drained inline deferred={=bool}",
                            self.deferred
                        );
                        if let Err(p) = self.backend.settle() {
                            self.latched = Some(p);
                        }
                    }
                    Err(p) => self.latched = Some(p),
                }
            } else {
                // No reset is asked for: the backend holds the core once anything was erased, else resets.
                let _ = self.backend.finish_with_reset(false, false);
            }
        }
        self.finalize(outcome);
    }

    fn render_status(&self, out: &mut StatusWriter<'_>) {
        // Programmed count first, so a mid-stream death never renders a success-shaped total.
        out.bytes(b"Flash: ");
        out.dec_u32(self.programmed);
        if self.programmed != self.stats.bytes {
            out.bytes(b" of ");
            out.dec_u32(self.stats.bytes);
        }
        out.bytes(b" bytes -> ");
        out.bytes(outcome_name(self.stats.outcome));
        if let Some(TransferOutcome::FlashError(p)) = self.stats.outcome {
            out.bytes(b" ");
            out.bytes(phase_name(p));
        }
        // Named only when the backend tracks a selection; single-target backends report `None`.
        if let Some(name) = self.backend.selected_target_name() {
            out.bytes(b" [");
            out.bytes(name.as_bytes());
            out.bytes(b"]");
        }
    }

    fn last_failed(&self) -> bool {
        self.stats.failed()
    }

    fn latched_phase(&self) -> Option<FlashPhase> {
        self.latched
    }

    fn set_sector_erase(&mut self, sector: bool) -> bool {
        self.backend.set_sector_erase(sector).is_ok()
    }

    fn wants_reconnect(&self) -> bool {
        // Only activity a user demonstrably caused: misread housekeeping would bounce forever.
        self.session_opened || self.dirent_bound
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::format;
    use std::string::String;
    use std::vec::Vec;

    struct MockBackend {
        ranges: &'static [(u32, u32)],
        calls: Vec<String>,
        fail_begin: bool,
        /// When set, `begin` fails in this phase: a selector-level rejection.
        begin_err: Option<FlashPhase>,
        fail_program: bool,
        fail_finish: bool,
        expected_family: Option<u32>,
        require_family: bool,
        target_name: Option<&'static str>,
        /// When set, `begin` overwrites `expected_family`: a backend that detects on selection.
        on_begin_family: Option<u32>,
        /// The family hint the sink handed to the last `begin_with_hint` (recorded for tests).
        begin_hint: Option<Option<u32>>,
        /// The reset hint the sink handed to the last `finish_with_reset` (recorded for tests).
        last_reset: Option<bool>,
    }

    impl MockBackend {
        fn new() -> Self {
            Self {
                ranges: &[(0x0800_0000, 0x0801_0000)],
                calls: Vec::new(),
                fail_begin: false,
                begin_err: None,
                fail_program: false,
                fail_finish: false,
                expected_family: None,
                require_family: false,
                target_name: None,
                on_begin_family: None,
                begin_hint: None,
                last_reset: None,
            }
        }

        fn with_name(name: &'static str) -> Self {
            Self {
                target_name: Some(name),
                ..Self::new()
            }
        }

        fn with_family(family: u32) -> Self {
            Self {
                expected_family: Some(family),
                ..Self::new()
            }
        }

        fn with_ranges(ranges: &'static [(u32, u32)]) -> Self {
            Self {
                ranges,
                ..Self::new()
            }
        }
    }

    impl FlashBackend for MockBackend {
        fn begin(&mut self) -> Result<(), FlashPhase> {
            self.calls.push(String::from("begin"));
            if let Some(p) = self.begin_err {
                return Err(p);
            }
            if self.fail_begin {
                return Err(FlashPhase::NoTarget);
            }
            // A detection backend reveals its family only after a successful begin.
            if let Some(f) = self.on_begin_family {
                self.expected_family = Some(f);
            }
            Ok(())
        }
        fn begin_with_hint(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
            self.begin_hint = Some(family);
            self.begin()
        }
        fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
            self.calls
                .push(format!("program:{:#x}:{}", addr, data.len()));
            if self.fail_program {
                Err(FlashPhase::Program)
            } else {
                Ok(())
            }
        }
        fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
            self.calls.push(format!("finish:{success}"));
            if self.fail_finish {
                Err(FlashPhase::Verify)
            } else {
                Ok(())
            }
        }
        fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
            // Delegate, so the recorded `calls` sequence is unchanged.
            self.last_reset = Some(reset);
            self.finish(success)
        }
        fn flash_ranges(&self) -> &[(u32, u32)] {
            self.ranges
        }
        fn expected_uf2_family(&self) -> Option<u32> {
            self.expected_family
        }
        fn requires_uf2_family(&self) -> bool {
            self.require_family
        }
        fn selected_target_name(&self) -> Option<&str> {
            self.target_name
        }
    }

    #[test]
    fn auto_reset_default_on() {
        let s = FlashSink::new(MockBackend::new());
        assert!(s.auto_reset());
    }

    #[test]
    fn happy_path_in_order() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.fragment(0x0800_0004, &[0xBB; 8]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
        assert!(!s.last_failed());
        assert_eq!(
            s.backend().calls,
            [
                "begin",
                "program:0x8000000:4",
                "program:0x8000004:8",
                "finish:true",
            ]
        );
    }

    #[test]
    fn begin_fail_swallows() {
        let mut s = FlashSink::new(MockBackend {
            fail_begin: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::NoTarget))
        );
        // A failed begin owns no session to program or finish.
        assert_eq!(s.backend().calls, ["begin"]);
    }

    #[test]
    fn program_fail_aborts_once_finish_false() {
        let mut s = FlashSink::new(MockBackend {
            fail_program: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        // A second fragment is swallowed (no more backend calls).
        s.fragment(0x0800_0004, &[0xBB; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Program))
        );
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x8000000:4", "finish:false"]
        );
    }

    #[test]
    fn out_of_range_latches_without_backend_call() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x2000_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Range))
        );
        assert_eq!(s.backend().calls, ["begin", "finish:false"]);
    }

    #[test]
    fn straddling_region_end_out_of_range() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        // Ends past the 0x0801_0000 boundary.
        s.fragment(0x0800_FFFC, &[0xAA; 8]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Range))
        );
        assert_eq!(s.backend().calls, ["begin", "finish:false"]);
    }

    #[test]
    fn finish_failure_latches() {
        let mut s = FlashSink::new(MockBackend {
            fail_finish: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Verify))
        );
    }

    #[test]
    fn non_done_aborts_best_effort() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::DecodeError);
        assert_eq!(s.outcome(), Some(TransferOutcome::DecodeError));
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x8000000:4", "finish:false"]
        );
    }

    /// The status line a finished sink renders, as a `String`.
    fn status_of<B: FlashBackend>(sink: &FlashSink<B>) -> String {
        let mut buf = [0u8; 64];
        let mut w = StatusWriter::new(&mut buf);
        sink.render_status(&mut w);
        String::from(std::str::from_utf8(w.as_bytes()).unwrap())
    }

    /// A refusal and a failed write have opposite remedies, so they must not render one line.
    #[test]
    fn a_refused_image_and_a_failed_write_do_not_report_alike() {
        // Refused: an image addressed for another part, on a target whose flash is elsewhere.
        let mut refused = FlashSink::new(MockBackend::with_ranges(&[(0x1000_0000, 0x1001_0000)]));
        refused.begin(Format::Uf2, false);
        refused.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        refused.end(TransferOutcome::Done);
        assert_eq!(
            refused.backend().calls,
            ["begin", "finish:false"],
            "a refused image is never handed to the backend"
        );

        // Failed: an in-range fragment the backend could not write.
        let mut failed = FlashSink::new(MockBackend {
            fail_program: true,
            ..MockBackend::new()
        });
        failed.begin(Format::Uf2, false);
        failed.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        failed.end(TransferOutcome::Done);
        assert_eq!(
            failed.backend().calls,
            ["begin", "program:0x8000000:4", "finish:false"],
            "a write failure is a write that was attempted"
        );

        let (refused, failed) = (status_of(&refused), status_of(&failed));
        assert_ne!(
            refused, failed,
            "a refused image must not report as a failed write"
        );
        assert!(refused.contains("Range"), "refused status: {refused}");
        assert!(failed.contains("Program"), "failed status: {failed}");
    }

    /// A session dying before anything is written must not report a programmed-looking count.
    #[test]
    fn a_failed_flash_reports_nothing_programmed() {
        let mut s = FlashSink::new(MockBackend {
            begin_err: Some(FlashPhase::Init),
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 256]).unwrap();
        s.fragment(0x0800_0100, &[0xBB; 256]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.programmed(), 0);
        assert_eq!(s.bytes(), 512, "the delivered count still records the drop");
        let text = status_of(&s);
        assert_eq!(text, "Flash: 0 of 512 bytes -> FlashError Init");
    }

    /// A clean flash programs everything delivered, and reports one number for both.
    #[test]
    fn a_clean_flash_reports_the_programmed_count() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.fragment(0x0800_0004, &[0xBB; 8]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.programmed(), 12);
        assert_eq!(s.programmed(), s.bytes());
        assert_eq!(status_of(&s), "Flash: 12 bytes -> Done");
    }

    /// A failure partway through reports what landed, not what arrived.
    #[test]
    fn a_mid_stream_failure_reports_only_what_landed() {
        let mut s = FlashSink::new(MockBackend::with_ranges(&[(0x0800_0000, 0x0800_0004)]));
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.fragment(0x0800_0004, &[0xBB; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.programmed(), 4);
        assert_eq!(s.bytes(), 8);
        assert_eq!(status_of(&s), "Flash: 4 of 8 bytes -> FlashError Range");
    }

    /// A failure at finish keeps the count; only the outcome word carries the failure.
    #[test]
    fn a_finish_failure_keeps_the_programmed_count() {
        let mut s = FlashSink::new(MockBackend {
            fail_finish: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.programmed(), 4);
        assert_eq!(status_of(&s), "Flash: 4 bytes -> FlashError Verify");
    }

    /// Counters do not survive into the next transfer.
    #[test]
    fn programmed_count_resets_between_transfers() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        s.begin(Format::Uf2, false);
        assert_eq!(s.programmed(), 0);
        s.end(TransferOutcome::NoData);
        assert_eq!(
            status_of(&s),
            "Flash: 0 bytes -> NoData (no image data recognized)"
        );
    }

    /// A fragment starting inside the region and running off its top is a refusal too.
    #[test]
    fn an_image_overrunning_the_region_reports_as_a_refusal() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_FFFC, &[0xAA; 8]).unwrap();
        s.end(TransferOutcome::Done);
        let text = status_of(&s);
        assert!(text.contains("Range"), "status: {text}");
        assert!(!text.contains("Program"), "status: {text}");
    }

    /// A family rejection reads the same either side of the session boundary.
    #[test]
    fn both_family_rejections_keep_one_report() {
        // Selector-level: `begin` itself rejects, before any fragment is judged.
        let mut selector = FlashSink::new(MockBackend {
            begin_err: Some(FlashPhase::Family),
            ..MockBackend::new()
        });
        selector.begin(Format::Uf2, false);
        selector.note_uf2_family(Some(0xBBBB_BBBB));
        selector.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        selector.end(TransferOutcome::Done);

        // In-session: the target resolved, and a block declares a different family.
        let mut session = FlashSink::new(MockBackend::with_family(0xAAAA_AAAA));
        session.begin(Format::Uf2, false);
        session.note_uf2_family(Some(0xBBBB_BBBB));
        session.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        session.end(TransferOutcome::Done);

        assert_eq!(selector.outcome(), session.outcome());
        assert_eq!(status_of(&selector), status_of(&session));
        assert!(status_of(&session).contains("Family"));
    }

    #[test]
    fn render_includes_phase() {
        let mut s = FlashSink::new(MockBackend {
            fail_program: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        let mut buf = [0u8; 64];
        let mut w = StatusWriter::new(&mut buf);
        s.render_status(&mut w);
        let text = std::str::from_utf8(w.as_bytes()).unwrap();
        assert!(text.contains("FlashError"));
        assert!(text.contains("Program"));
    }

    /// A corrupt block and a reordered drop have different remedies, so the status names which.
    #[test]
    fn a_corrupt_block_and_a_reordered_drop_do_not_report_alike() {
        let mut corrupt = FlashSink::new(MockBackend::new());
        corrupt.begin(Format::Uf2, false);
        corrupt.fragment(0x0800_0000, &[0xAA; 256]).unwrap();
        corrupt.end(TransferOutcome::DecodeError);
        assert_eq!(
            status_of(&corrupt),
            "Flash: 256 bytes -> DecodeError",
            "a corrupt block reports the bytes that landed and names the decode failure"
        );

        let mut reordered = FlashSink::new(MockBackend::new());
        reordered.begin(Format::Uf2, false);
        reordered.fragment(0x0800_0000, &[0xAA; 256]).unwrap();
        reordered.end(TransferOutcome::OutOfOrder);
        assert_eq!(status_of(&reordered), "Flash: 256 bytes -> OutOfOrder");
    }

    /// A magic-less sector in the declared window ends the transfer and releases the session once.
    #[test]
    fn corrupt_block_releases_the_session_at_that_write() {
        use crate::decoder_set::AllDecoders;
        use crate::transfer::Transfer;

        let mut block = [0u8; 512];
        block[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        block[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        block[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        block[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        block[32..36].copy_from_slice(&[1, 2, 3, 4]);
        block[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END

        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = FlashSink::new(MockBackend::new());
        t.file_event(
            &crate::transfer::FileEvent::Created {
                name: b"APP     UF2",
                size: 4 * 512,
                hidden: false,
                first_cluster: 2,
            },
            &mut sink,
        );
        t.data_sector(133, &block, &mut sink);
        // The host's third sector reaches the volume with its magic destroyed.
        t.data_sector(135, &[0x5Au8; 512], &mut sink);
        assert_eq!(sink.outcome(), Some(TransferOutcome::DecodeError));
        assert_eq!(
            sink.backend().calls,
            ["begin", "program:0x8000000:4", "finish:false"],
            "the session closes at the corrupt write, once"
        );
        assert!(!t.is_active(), "and no idle timeout is needed to close it");
    }

    /// A block whose declared family differs from the target's is rejected before any program.
    #[test]
    fn uf2_family_mismatch_rejected_before_program() {
        let mut s = FlashSink::new(MockBackend::with_family(0xAAAA_AAAA));
        s.begin(Format::Uf2, false);
        s.note_uf2_family(Some(0xBBBB_BBBB)); // wrong family for this target
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Family))
        );
        // `begin` connected, but the mismatching block never reached `program`.
        assert_eq!(s.backend().calls, ["begin", "finish:false"]);
    }

    /// A block whose family matches the target is programmed normally.
    #[test]
    fn uf2_family_match_programs() {
        let mut s = FlashSink::new(MockBackend::with_family(0xAAAA_AAAA));
        s.begin(Format::Uf2, false);
        s.note_uf2_family(Some(0xAAAA_AAAA));
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x8000000:4", "finish:true"]
        );
    }

    /// A block declaring no family cannot be cross-checked, so it is accepted.
    #[test]
    fn uf2_block_without_family_accepted() {
        let mut s = FlashSink::new(MockBackend::with_family(0xAAAA_AAAA));
        s.begin(Format::Uf2, false);
        s.note_uf2_family(None);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x8000000:4", "finish:true"]
        );
    }

    #[test]
    fn multi_target_later_blocks_require_the_selected_family() {
        for later in [None, Some(0xBBBB_BBBB)] {
            let mut s = FlashSink::new(MockBackend {
                require_family: true,
                ..MockBackend::with_family(0xAAAA_AAAA)
            });
            s.begin(Format::Uf2, false);
            s.note_uf2_family(Some(0xAAAA_AAAA));
            s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
            s.note_uf2_family(later);
            s.fragment(0x0800_0004, &[0xBB; 4]).unwrap();
            s.end(TransferOutcome::Done);
            assert_eq!(
                s.outcome(),
                Some(TransferOutcome::FlashError(FlashPhase::Family))
            );
            assert_eq!(s.programmed(), 4);
            assert_eq!(s.bytes(), 8);
            assert_eq!(
                s.backend().calls,
                ["begin", "program:0x8000000:4", "finish:false"]
            );
        }
    }

    #[test]
    fn multi_target_requires_flag_not_just_family_field() {
        use crate::decoder_set::AllDecoders;
        use crate::transfer::{FileEvent, Transfer};

        let mut block = [0u8; 512];
        block[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes());
        block[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes());
        block[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes());
        block[16..20].copy_from_slice(&4u32.to_le_bytes());
        block[28..32].copy_from_slice(&0xAAAA_AAAAu32.to_le_bytes());
        block[32..36].copy_from_slice(&[1, 2, 3, 4]);
        block[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes());
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = FlashSink::new(MockBackend {
            require_family: true,
            ..MockBackend::with_family(0xAAAA_AAAA)
        });
        t.file_event(
            &FileEvent::Created {
                name: b"APP     UF2",
                size: 512,
                hidden: false,
                first_cluster: 2,
            },
            &mut sink,
        );
        t.data_sector(133, &block, &mut sink);
        assert_eq!(
            sink.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Family))
        );
        assert!(sink.backend().calls.is_empty());
        assert_eq!(sink.programmed(), 0);
    }

    /// A target with no expected family never cross-checks.
    #[test]
    fn no_expected_family_skips_check() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.note_uf2_family(Some(0x1234_5678));
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
    }

    /// The family-mismatch failure surfaces the descriptive `Family` phase in the status line.
    #[test]
    fn render_includes_family_phase() {
        let mut s = FlashSink::new(MockBackend::with_family(0xAAAA_AAAA));
        s.begin(Format::Uf2, false);
        s.note_uf2_family(Some(0xBBBB_BBBB));
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        let mut buf = [0u8; 64];
        let mut w = StatusWriter::new(&mut buf);
        s.render_status(&mut w);
        let text = std::str::from_utf8(w.as_bytes()).unwrap();
        assert!(text.contains("FlashError"));
        assert!(text.contains("Family"));
    }

    /// The status line names the selected target when the backend reports one.
    #[test]
    fn render_includes_detected_target_name() {
        let mut s = FlashSink::new(MockBackend::with_name("RP2040"));
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        let mut buf = [0u8; 64];
        let mut w = StatusWriter::new(&mut buf);
        s.render_status(&mut w);
        let text = std::str::from_utf8(w.as_bytes()).unwrap();
        assert!(text.contains("[RP2040]"), "status names the target: {text}");
    }

    /// End to end through the engine and decoder: a mismatched family rejects in `Family`.
    #[test]
    fn uf2_family_mismatch_through_transfer_engine() {
        use crate::decoder_set::AllDecoders;
        use crate::transfer::Transfer;

        const FAMILY_FLAG: u32 = 0x0000_2000; // UF2 flag: family id present.
        let mut block = [0u8; 512];
        block[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        block[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        block[8..12].copy_from_slice(&FAMILY_FLAG.to_le_bytes()); // flags
        block[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        block[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        block[28..32].copy_from_slice(&0xBBBB_BBBBu32.to_le_bytes()); // family id (wrong)
        block[32..36].copy_from_slice(&[1, 2, 3, 4]);
        block[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END

        let mut t = Transfer::<AllDecoders>::new();
        // Target expects family 0xAAAA_AAAA; the block declares 0xBBBB_BBBB.
        let mut sink = FlashSink::new(MockBackend::with_family(0xAAAA_AAAA));
        t.file_event(
            &crate::transfer::FileEvent::Created {
                name: b"APP     UF2",
                size: 512,
                hidden: false,
                first_cluster: 2,
            },
            &mut sink,
        );
        t.data_sector(133, &block, &mut sink);
        assert_eq!(
            sink.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Family))
        );
        assert_eq!(sink.backend().calls, ["begin", "finish:false"]);
    }

    /// After a DUT swap the first correct drop is not rejected against the stale family.
    #[test]
    fn dut_swap_first_drop_accepted() {
        let mut s = FlashSink::new(MockBackend {
            expected_family: Some(0x1111_1111),
            on_begin_family: Some(0x2222_2222),
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.note_uf2_family(Some(0x2222_2222));
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x8000000:4", "finish:true"]
        );
        // The block's family reached `begin` as the selection hint.
        assert_eq!(s.backend().begin_hint, Some(Some(0x2222_2222)));
    }

    /// A wrong-family image is still rejected, against the detected target, not the stale one.
    #[test]
    fn wrong_family_rejected_against_detected_target() {
        let mut s = FlashSink::new(MockBackend {
            expected_family: Some(0x1111_1111),
            on_begin_family: Some(0x2222_2222),
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.note_uf2_family(Some(0x9999_9999));
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Family))
        );
        assert_eq!(s.backend().calls, ["begin", "finish:false"]);
    }

    /// A transfer whose blocks declare no family begins the backend with an empty hint.
    #[test]
    fn transfer_without_family_begins_with_no_hint() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.backend().begin_hint, Some(None));
    }

    /// The reset-after-flash policy reaches the backend's `finish_with_reset` at end of transfer.
    #[test]
    fn reset_policy_threads_into_finish() {
        let mut s = FlashSink::new(MockBackend::new());
        assert!(s.auto_reset());
        s.set_auto_reset(false);
        assert!(!s.auto_reset());
        // A following clean flash must finish with reset = false.
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
        assert_eq!(s.backend().last_reset, Some(false));
        // Re-enabled, the next clean flash finishes with reset = true.
        s.set_auto_reset(true);
        assert!(s.auto_reset());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.backend().last_reset, Some(true));
    }

    /// A failed flash asks for no reset, whatever the policy; what happens is the backend's call.
    #[test]
    fn a_failed_flash_passes_no_reset_request() {
        let mut s = FlashSink::new(MockBackend {
            fail_program: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x2000_0000, &[0xAA; 4]).unwrap(); // out of range -> latches
        s.end(TransferOutcome::Done);
        assert_eq!(s.backend().last_reset, Some(false));
    }

    /// A fragment programs at its declared address; one outside flash never reaches the backend.
    #[test]
    fn fragments_program_at_their_declared_address() {
        let mut s = FlashSink::new(MockBackend::with_ranges(&[(0x1000_0000, 0x1001_0000)]));
        s.begin(Format::Uf2, false);
        s.fragment(0x1000_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x10000000:4", "finish:true"]
        );

        // A block addressed for another part's flash is out of range.
        let mut s = FlashSink::new(MockBackend::with_ranges(&[(0x1000_0000, 0x1001_0000)]));
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Range))
        );
    }

    /// A mid-stream latch releases the session at once, and `end` must not finish it again.
    #[test]
    fn latched_failure_releases_session_early_exactly_once() {
        // Out-of-range latch: the session closes right at the latching fragment.
        let mut s = FlashSink::new(MockBackend::with_ranges(&[(0x1000_0000, 0x1001_0000)]));
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        assert_eq!(
            s.backend().calls,
            ["begin", "finish:false"],
            "the session is released at the latching fragment, not at end()"
        );
        // The rest of the stream is swallowed without touching the backend.
        s.fragment(0x0800_0004, &[0xBB; 4]).unwrap();
        s.end(TransferOutcome::Timeout);
        assert_eq!(
            s.backend().calls,
            ["begin", "finish:false"],
            "end() does not finish the already-released session again"
        );
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Range))
        );
        assert!(
            s.wants_reconnect(),
            "the session did open: bus still bounces"
        );

        // Family mismatch latch: same early release.
        let mut s = FlashSink::new(MockBackend::with_family(0xAAAA_AAAA));
        s.begin(Format::Uf2, false);
        s.note_uf2_family(Some(0xBBBB_BBBB));
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        assert_eq!(s.backend().calls, ["begin", "finish:false"]);
        s.end(TransferOutcome::Done);
        assert_eq!(s.backend().calls, ["begin", "finish:false"]);
        // An early release asks for no reset.
        assert_eq!(s.backend().last_reset, Some(false));
    }

    /// A clean transfer still finishes once, at end of transfer, with the reset policy applied.
    #[test]
    fn clean_transfer_still_finishes_at_end() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x8000000:4"],
            "no finish before end() on a clean stream"
        );
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.backend().calls,
            ["begin", "program:0x8000000:4", "finish:true"]
        );
    }

    #[test]
    fn terminal_end_is_idempotent_and_next_transfer_can_finish() {
        for (fail_program, fail_finish, outcome) in [
            (false, false, TransferOutcome::Done),
            (false, false, TransferOutcome::Timeout),
            (true, false, TransferOutcome::Done),
            (false, true, TransferOutcome::Done),
        ] {
            let mut s = FlashSink::new(MockBackend {
                fail_program,
                fail_finish,
                ..MockBackend::new()
            });
            s.begin(Format::Uf2, false);
            s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
            s.end(outcome);
            let calls = s.backend().calls.clone();
            let final_outcome = s.outcome();
            s.end(TransferOutcome::DecodeError);
            s.end(TransferOutcome::Done);
            assert_eq!(s.backend().calls, calls);
            assert_eq!(s.outcome(), final_outcome);
            assert_eq!(calls.iter().filter(|s| s.starts_with("finish:")).count(), 1);

            s.backend.fail_program = false;
            s.backend.fail_finish = false;
            s.begin(Format::Uf2, false);
            s.fragment(0x0800_0000, &[0xBB; 4]).unwrap();
            s.end(TransferOutcome::Done);
            assert_eq!(s.outcome(), Some(TransferOutcome::Done));
            assert_eq!(s.backend().calls.len(), calls.len() + 3);
        }
    }

    #[test]
    fn rejected_begin_is_not_finished_and_does_not_poison_next_transfer() {
        let mut s = FlashSink::new(MockBackend {
            fail_begin: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        s.end(TransferOutcome::Timeout);
        assert_eq!(s.backend().calls, ["begin"]);
        assert_eq!(s.backend().last_reset, None);
        assert!(!s.wants_reconnect());
        s.backend.fail_begin = false;
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
        assert_eq!(s.backend().last_reset, Some(true));
    }

    /// A transfer producing no fragments never begins or finishes the backend.
    #[test]
    fn zero_fragment_transfer_never_begins() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::NoData));
        assert!(
            s.backend().calls.is_empty(),
            "no begin/finish without fragments"
        );
    }

    /// An unbound transfer bounces the bus only if it opened a flash session.
    #[test]
    fn reconnect_only_after_a_session_opened() {
        // Fresh sink: nothing happened, nothing to announce.
        let s = FlashSink::new(MockBackend::new());
        assert!(!s.wants_reconnect());

        // Zero-fragment unbound transfer: a sniffed stream whose decode failed immediately.
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.end(TransferOutcome::DecodeError);
        assert!(!s.wants_reconnect(), "no session was opened: no bus bounce");

        // The shape a housekeeping write misread as an image takes: still no bounce.
        let mut s = FlashSink::new(MockBackend {
            fail_begin: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert!(!s.wants_reconnect(), "begin failed: no session, no bounce");

        // A transfer that opened a session bounces, success or failure alike.
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert!(s.wants_reconnect(), "a programmed transfer bounces the bus");

        // The next transfer's begin clears the previous grant.
        s.begin(Format::Uf2, false);
        s.end(TransferOutcome::DecodeError);
        assert!(
            !s.wants_reconnect(),
            "the grant does not leak across transfers"
        );
    }

    /// A dirent-bound drop bounces the bus on any terminal outcome, even before a session.
    #[test]
    fn bound_drop_rejected_before_session_still_reconnects() {
        let mut s = FlashSink::new(MockBackend {
            fail_begin: true,
            ..MockBackend::new()
        });
        s.begin(Format::Uf2, false);
        s.note_dirent_bound();
        s.fragment(0x0800_0000, &[0xAA; 4]).unwrap();
        s.end(TransferOutcome::Done);
        assert_eq!(
            s.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::NoTarget))
        );
        assert!(
            s.wants_reconnect(),
            "a bound drop's pre-session rejection must still bounce the bus"
        );
        // The status line names the failure that the bounce makes host-visible.
        let mut buf = [0u8; 64];
        let mut w = StatusWriter::new(&mut buf);
        s.render_status(&mut w);
        let text = std::str::from_utf8(w.as_bytes()).unwrap();
        assert!(text.contains("FlashError"), "status: {text}");
        assert!(text.contains("NoTarget"), "status: {text}");
    }

    /// The bound grant covers a pre-session death, and is cleared by the next `begin`.
    #[test]
    fn bound_drop_decode_failure_reconnects_and_grant_clears() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.note_dirent_bound();
        s.end(TransferOutcome::DecodeError);
        assert!(s.backend().calls.is_empty(), "backend never begun");
        assert!(
            s.wants_reconnect(),
            "a bound drop that failed to decode still bounces the bus"
        );
        // The next (unbound) transfer must not inherit the grant.
        s.begin(Format::Uf2, false);
        s.end(TransferOutcome::DecodeError);
        assert!(
            !s.wants_reconnect(),
            "the bound grant does not leak across transfers"
        );
    }

    /// End to end: a bound drop the backend rejects at `begin` must still bounce the bus.
    #[test]
    fn bound_drop_through_engine_reconnects_on_begin_failure() {
        use crate::decoder_set::AllDecoders;
        use crate::transfer::{FileEvent, Transfer};

        let mut block = [0u8; 512];
        block[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        block[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        block[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        block[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        block[32..36].copy_from_slice(&[1, 2, 3, 4]);
        block[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END

        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = FlashSink::new(MockBackend {
            fail_begin: true,
            ..MockBackend::new()
        });
        t.file_event(
            &FileEvent::Created {
                name: b"APP     UF2",
                size: 512,
                hidden: false,
                first_cluster: 2,
            },
            &mut sink,
        );
        t.data_sector(133, &block, &mut sink);
        assert_eq!(
            sink.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::NoTarget))
        );
        assert!(
            sink.wants_reconnect(),
            "the engine must report the bound drop so the rejection bounces the bus"
        );

        // Data-first order (dirent lands mid-stream): the late binding still grants the bounce.
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = FlashSink::new(MockBackend {
            fail_begin: true,
            ..MockBackend::new()
        });
        t.data_sector(133, &block, &mut sink);
        assert!(!sink.wants_reconnect(), "sniffed and sessionless: quiet");
        t.file_event(
            &FileEvent::Created {
                name: b"APP     UF2",
                size: 512,
                hidden: false,
                first_cluster: 2,
            },
            &mut sink,
        );
        assert!(
            sink.wants_reconnect(),
            "a visible dirent claiming the sniffed stream makes it a real drop"
        );
    }
}

#[cfg(test)]
pub(crate) mod step_tests {
    use super::*;
    use crate::transfer::TransferOutcome;
    use daplink_decoder::Format;
    use std::vec::Vec;

    /// A backend whose every operation costs `steps` calls and stages at most `chunk` bytes.
    pub(crate) struct StepBackend {
        ranges: &'static [(u32, u32)],
        /// Every `(addr, bytes)` the sink handed down, in order.
        pub written: Vec<(u32, Vec<u8>)>,
        /// Steps each device operation still owes.
        pending: u32,
        steps: u32,
        chunk: usize,
        /// Whether a session is open.
        pub begun: bool,
        pub finished: bool,
        /// When set, `finish_start` stages the teardown across `step` calls.
        defer_finish: bool,
        /// A staged teardown is in flight, waiting for its `pending` steps to drain.
        finishing: bool,
        step_err: Option<FlashPhase>,
        settle_err: Option<FlashPhase>,
        finish_calls: u32,
    }

    impl StepBackend {
        pub(crate) fn new(steps: u32, chunk: usize) -> Self {
            Self {
                ranges: &[(0x0800_0000, 0x0801_0000)],
                written: Vec::new(),
                pending: 0,
                steps,
                chunk,
                begun: false,
                finished: false,
                defer_finish: false,
                finishing: false,
                step_err: None,
                settle_err: None,
                finish_calls: 0,
            }
        }

        pub(crate) fn deferring_finish(steps: u32, chunk: usize) -> Self {
            Self {
                defer_finish: true,
                ..Self::new(steps, chunk)
            }
        }

        /// Every byte the sink programmed, concatenated in delivery order.
        pub(crate) fn bytes(&self) -> Vec<u8> {
            self.written.iter().flat_map(|(_, d)| d.clone()).collect()
        }
    }

    impl FlashBackend for StepBackend {
        fn begin(&mut self) -> Result<(), FlashPhase> {
            self.begun = true;
            self.pending = self.steps;
            Ok(())
        }

        fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
            self.written.push((addr, data.to_vec()));
            Ok(())
        }

        fn program_chunk(&mut self, addr: u32, data: &[u8]) -> Result<usize, FlashPhase> {
            let take = self.chunk.min(data.len());
            self.written.push((addr, data[..take].to_vec()));
            self.pending = self.steps;
            Ok(take)
        }

        fn step(&mut self, now_us: u64) -> FlashPoll {
            if let Some(phase) = self.step_err.take() {
                return FlashPoll::Err(phase);
            }
            if self.pending > 0 {
                self.pending -= 1;
                return FlashPoll::Pending {
                    wake_at_us: now_us + 10,
                };
            }
            // A staged teardown lands its `finished` flag when its last step drains.
            if self.finishing {
                self.finishing = false;
                self.finished = true;
            }
            FlashPoll::Done
        }

        fn settle(&mut self) -> Result<(), FlashPhase> {
            if let Some(phase) = self.settle_err.take() {
                self.pending = 0;
                return Err(phase);
            }
            self.pending = 0;
            if self.finishing {
                self.finishing = false;
                self.finished = true;
            }
            Ok(())
        }

        fn finish_start(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
            if self.defer_finish {
                self.finish_calls += 1;
                self.finishing = true;
                self.pending = self.steps;
                Ok(())
            } else {
                self.finish_with_reset(success, reset)
            }
        }

        fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
            self.finish_with_reset(success, true)
        }

        fn finish_with_reset(&mut self, _success: bool, _reset: bool) -> Result<(), FlashPhase> {
            self.finish_calls += 1;
            self.finished = true;
            self.pending = 0;
            Ok(())
        }

        fn flash_ranges(&self) -> &[(u32, u32)] {
            self.ranges
        }
    }

    /// Deliver `data` as one fragment, pumping `advance` when deferred; returns whether it parked.
    fn deliver(
        steps: u32,
        chunk: usize,
        defer: bool,
        data: &[u8],
    ) -> (FlashSink<StepBackend>, bool) {
        let mut sink = FlashSink::new(StepBackend::new(steps, chunk));
        sink.set_deferred(defer);
        sink.begin(Format::Unknown, false);
        sink.fragment(0x0800_0000, data).expect("fragment");
        let mut parked = false;
        let mut now_us = 0u64;
        if defer {
            while sink.advance(now_us) == SectorPoll::Pending {
                parked = true;
                now_us += 10;
            }
        }
        sink.end(TransferOutcome::Done);
        // A deferred caller pumps the teardown too, exactly as the transport's idle tick does.
        if defer {
            while sink.advance(now_us) == SectorPoll::Pending {
                now_us += 10;
            }
        }
        (sink, parked)
    }

    /// A fragment split across `Pending` programs and counts the same as one delivered inline.
    #[test]
    fn a_parked_fragment_matches_the_blocking_delivery() {
        let data: Vec<u8> = (0..200u32).map(|i| i as u8).collect();
        let (blocking, parked_none) = deliver(0, usize::MAX, false, &data);
        let (stepped, parked) = deliver(2, 7, true, &data);

        assert!(!parked_none, "an undeferred sink never parks");
        assert!(parked, "a deferred fragment parks while the backend steps");
        assert_eq!(stepped.programmed(), blocking.programmed());
        assert_eq!(stepped.bytes(), blocking.bytes());
        assert_eq!(stepped.fragments(), blocking.fragments());
        assert_eq!(stepped.outcome(), blocking.outcome());
        assert_eq!(stepped.backend().bytes(), blocking.backend().bytes());
        assert_eq!(stepped.backend().bytes(), data);
        assert!(stepped.backend().finished);
    }

    /// The fragment's bytes are counted whole, when the last chunk lands, never per chunk.
    #[test]
    fn a_parked_fragment_counts_its_bytes_once() {
        let data = [0xA5u8; 64];
        let mut sink = FlashSink::new(StepBackend::new(1, 8));
        sink.set_deferred(true);
        sink.begin(Format::Unknown, false);
        sink.fragment(0x0800_0000, &data).expect("fragment");
        let mut now_us = 0u64;
        while sink.advance(now_us) == SectorPoll::Pending {
            assert_eq!(
                sink.programmed(),
                0,
                "nothing counts until the fragment lands"
            );
            now_us += 10;
        }
        assert_eq!(sink.programmed(), 64);
    }

    /// A deferred teardown parks in `end`, and `advance` drives it to completion.
    #[test]
    fn a_deferred_teardown_steps_through_advance() {
        let data = [0x5Au8; 16];
        let mut sink = FlashSink::new(StepBackend::deferring_finish(3, 8));
        sink.set_deferred(true);
        sink.begin(Format::Unknown, false);
        sink.fragment(0x0800_0000, &data).expect("fragment");
        let mut now_us = 0u64;
        while sink.advance(now_us) == SectorPoll::Pending {
            now_us += 10;
        }
        // The teardown is only staged by `end`; nothing is finished until `advance` drains it.
        sink.end(TransferOutcome::Done);
        assert!(!sink.backend().finished, "end must not tear down inline");
        assert_eq!(
            sink.outcome(),
            None,
            "the outcome is not final until the teardown lands"
        );
        let mut parked = false;
        while sink.advance(now_us) == SectorPoll::Pending {
            parked = true;
            now_us += 10;
        }
        assert!(parked, "the teardown parked while it stepped");
        assert!(
            sink.backend().finished,
            "the teardown completed under advance"
        );
        assert_eq!(sink.outcome(), Some(TransferOutcome::Done));
        assert_eq!(sink.programmed(), 16);
    }

    #[test]
    fn failed_deferred_begin_owns_nothing_to_finish() {
        let mut sink = FlashSink::new(StepBackend {
            step_err: Some(FlashPhase::Init),
            ..StepBackend::new(2, 8)
        });
        sink.set_deferred(true);
        sink.begin(Format::Unknown, false);
        sink.fragment(0x0800_0000, &[0x5A; 16]).unwrap();
        assert_eq!(sink.advance(0), SectorPoll::Ready);
        sink.end(TransferOutcome::Done);
        sink.end(TransferOutcome::Timeout);
        assert_eq!(sink.backend().finish_calls, 0);
        assert!(!sink.wants_reconnect());
        assert_eq!(
            sink.outcome(),
            Some(TransferOutcome::FlashError(FlashPhase::Init))
        );
        sink.begin(Format::Unknown, false);
        sink.fragment(0x0800_0000, &[0x5A; 16]).unwrap();
        sink.end(TransferOutcome::Done);
        assert_eq!(sink.advance(0), SectorPoll::Ready);
        assert_eq!(sink.outcome(), Some(TransferOutcome::Done));
        assert_eq!(sink.backend().finish_calls, 1);
    }

    #[test]
    fn failed_begin_during_blocking_drain_is_not_finished() {
        for deferred in [false, true] {
            let mut sink = FlashSink::new(StepBackend {
                settle_err: Some(FlashPhase::Init),
                ..StepBackend::new(2, 8)
            });
            sink.set_deferred(deferred);
            sink.begin(Format::Unknown, false);
            sink.fragment(0x0800_0000, &[0x5A; 16]).unwrap();
            sink.end(TransferOutcome::Done);
            assert_eq!(sink.backend().finish_calls, 0);
            assert!(!sink.wants_reconnect());
            assert_eq!(
                sink.outcome(),
                Some(TransferOutcome::FlashError(FlashPhase::Init))
            );
        }
    }

    #[test]
    fn duplicate_end_neither_restarts_nor_aborts_deferred_teardown() {
        for error in [None, Some(FlashPhase::Verify)] {
            let mut sink = FlashSink::new(StepBackend::deferring_finish(3, 8));
            sink.set_deferred(true);
            sink.begin(Format::Unknown, false);
            sink.fragment(0x0800_0000, &[0x5A; 16]).unwrap();
            sink.end(TransferOutcome::Done);
            assert_eq!(sink.advance(0), SectorPoll::Pending);
            assert_eq!(sink.backend().pending, 2);
            sink.end(TransferOutcome::Timeout);
            sink.end(TransferOutcome::Done);
            assert_eq!(sink.backend().pending, 2);
            assert_eq!(sink.backend().finish_calls, 1);
            sink.backend.step_err = error;
            while sink.advance(10) == SectorPoll::Pending {}
            let outcome = sink.outcome();
            assert_eq!(
                outcome,
                Some(error.map_or(TransferOutcome::Done, TransferOutcome::FlashError))
            );
            sink.end(TransferOutcome::Timeout);
            assert_eq!(sink.outcome(), outcome);
            assert_eq!(sink.backend().finish_calls, 1);
        }
    }

    /// A sink whose caller stops pumping still programs everything: `end` drains what is left.
    #[test]
    fn end_drains_a_fragment_left_in_flight() {
        let data = [0x5Au8; 32];
        let mut sink = FlashSink::new(StepBackend::new(3, 8));
        sink.set_deferred(true);
        sink.begin(Format::Unknown, false);
        sink.fragment(0x0800_0000, &data).expect("fragment");
        sink.end(TransferOutcome::Done);
        assert_eq!(sink.programmed(), 32);
        assert_eq!(sink.backend().bytes(), data);
    }
}
