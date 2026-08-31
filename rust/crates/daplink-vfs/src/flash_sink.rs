//! A [`TransferSink`] that programs decoded fragments into target flash via a [`FlashBackend`].

use crate::render::StatusWriter;
use crate::transfer::{
    outcome_name, phase_name, DecodeError, TransferOutcome, TransferSink, TransferStats,
};
use daplink_core::{FlashBackend, FlashPhase};
use daplink_decoder::Format;

/// Whether `[addr, addr + len)` fits entirely within one of `ranges`.
fn in_range(ranges: &[(u32, u32)], addr: u32, len: usize) -> bool {
    let end = addr as u64 + len as u64;
    ranges.iter().any(|&(s, e)| addr >= s && end <= e as u64)
}

/// Programs decoded image fragments into target flash, latching the first failing phase.
pub struct FlashSink<B: FlashBackend> {
    backend: B,
    auto_reset: bool,
    stats: TransferStats,
    /// Bytes the backend accepted, as distinct from `stats.bytes`, which counts what the host
    /// delivered. They diverge exactly when a failure latches mid-stream.
    programmed: u32,
    latched: Option<FlashPhase>,
    swallow: bool,
    /// Whether [`FlashBackend::begin_with_hint`] has been attempted for this transfer. Lazy, at
    /// the first fragment, so the first block's noted family can serve as the selection hint.
    started: bool,
    /// The selected target's expected UF2 family id (`None` = no check), read after `begin`
    /// resolves the target, so a detection backend cross-checks against the attached chip.
    expected_family: Option<u32>,
    /// The family id declared by the block currently being delivered (`None` = it declares none).
    block_family: Option<u32>,
    /// Whether this transfer opened a flash session. One of the two
    /// [`wants_reconnect`](TransferSink::wants_reconnect) grants; cleared by `begin`.
    session_opened: bool,
    /// Whether this transfer was bound to a visible image dirent. The other `wants_reconnect`
    /// grant: a real drop bounces the bus even if it died before a session opened.
    dirent_bound: bool,
    /// Whether [`release_early`](Self::release_early) already closed the session, so
    /// [`end`](TransferSink::end) skips its own finish and the backend finishes exactly once.
    released: bool,
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
            expected_family: None,
            block_family: None,
            session_opened: false,
            dirent_bound: false,
            released: false,
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

    /// Mutably borrow the backend.
    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }

    /// The recorded outcome of the last transfer.
    pub fn outcome(&self) -> Option<TransferOutcome> {
        self.stats.outcome
    }

    /// Bytes delivered to the sink in the last transfer, programmed or not.
    pub fn bytes(&self) -> u32 {
        self.stats.bytes
    }

    /// Bytes the backend accepted in the last transfer.
    pub fn programmed(&self) -> u32 {
        self.programmed
    }

    /// Fragment count of the last transfer.
    pub fn fragments(&self) -> u32 {
        self.stats.fragments
    }

    /// Close the flash session as soon as a failure latches, rather than holding the debug wire
    /// until the idle timeout ends the swallowed stream.
    fn release_early(&mut self) {
        if self.session_opened && !self.released {
            self.released = true;
            let _ = self.backend.finish_with_reset(false, false);
            #[cfg(feature = "defmt")]
            defmt::info!("flash sink: early session release (failure latched, stream swallowed)");
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
        self.expected_family = None;
        self.block_family = None;
        // Neither reconnect grant is held yet.
        self.session_opened = false;
        self.dirent_bound = false;
        self.released = false;
    }

    fn note_dirent_bound(&mut self) {
        self.dirent_bound = true;
    }

    fn note_uf2_family(&mut self, family: Option<u32>) {
        self.block_family = family;
    }

    fn fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), DecodeError> {
        self.stats.record_fragment(data.len());
        if self.swallow {
            return Ok(());
        }
        // Begun at the first fragment, so the block's family can serve as the selection hint.
        if !self.started {
            self.started = true;
            if let Err(p) = self.backend.begin_with_hint(self.block_family) {
                self.latched = Some(p);
                self.swallow = true;
                return Ok(());
            }
            self.session_opened = true;
            self.expected_family = self.backend.expected_uf2_family();
        }
        // A declared family disagreeing with the target's rejects before any erase or program.
        if let (Some(expected), Some(block)) = (self.expected_family, self.block_family) {
            if block != expected {
                self.latched = Some(FlashPhase::Family);
                self.swallow = true;
                self.release_early();
                return Ok(());
            }
        }
        if in_range(self.backend.flash_ranges(), addr, data.len()) {
            match self.backend.program(addr, data) {
                Ok(()) => self.programmed = self.programmed.saturating_add(data.len() as u32),
                Err(p) => {
                    self.latched = Some(p);
                    self.swallow = true;
                    self.release_early();
                }
            }
        } else {
            // Nothing is written, so this is `Range`, not `Program`.
            self.latched = Some(FlashPhase::Range);
            self.swallow = true;
            self.release_early();
        }
        Ok(())
    }

    fn end(&mut self, outcome: TransferOutcome) {
        // A transfer with zero fragments never begins the backend, so nothing needs finishing.
        if self.started {
            let clean = self.latched.is_none() && outcome == TransferOutcome::Done;
            if clean {
                // A clean flash resets the target only when auto-reset is enabled.
                if let Err(p) = self.backend.finish_with_reset(true, self.auto_reset) {
                    self.latched = Some(p);
                }
            } else if !self.released {
                // A failed flash never resets, and never finishes a session already released.
                let _ = self.backend.finish_with_reset(false, false);
            }
        }
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
    }

    fn render_status(&self, out: &mut StatusWriter<'_>) {
        // The leading number is what was programmed; a differing delivered count follows it, so
        // a flash that died mid-stream can never render a success-shaped total.
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
        // Program was never attempted after a failed begin.
        assert_eq!(s.backend().calls, ["begin", "finish:false"]);
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

    /// A session that dies before anything is written must not report a byte count that reads
    /// like a programmed image.
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

    /// A failure at finish leaves every byte programmed, so the count stands and only the
    /// outcome word beside it carries the failure.
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

    /// A family rejection keeps one report either side of the session boundary: unlike `Range`,
    /// the user's remedy is the same sentence.
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

    /// After a DUT swap, the first correct drop is programmed rather than rejected against the
    /// stale prior-session family the backend still reports before `begin` runs detection.
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
        // Default policy is auto-reset on.
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

    /// A failed flash never resets, regardless of the policy.
    #[test]
    fn failed_flash_never_resets() {
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

    /// A failure latching mid-stream releases the session at once, and `end` must not finish the
    /// backend a second time.
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
        // An early release never resets the target.
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

    /// A transfer producing no fragments never begins or finishes the backend.
    #[test]
    fn zero_fragment_transfer_never_begins() {
        let mut s = FlashSink::new(MockBackend::new());
        s.begin(Format::Uf2, false);
        s.end(TransferOutcome::Done);
        assert_eq!(s.outcome(), Some(TransferOutcome::Done));
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

    /// A dirent-bound drop bounces the bus on any terminal outcome, including a rejection
    /// before any flash session opened.
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
