//! The drag-n-drop transfer engine: format detection, in-order streaming, and sinks.

use crate::decoder_set::DecoderSet;
use crate::fat;
use crate::render::StatusWriter;
use daplink_decoder::{Decoder, DecoderContext, Error, Format};

/// The single error currency for decoded-fragment delivery.
pub type DecodeError = Error;

pub use daplink_core::FlashPhase;

/// The final result of a transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferOutcome {
    /// Completed successfully.
    Done,
    /// Sectors arrived out of order.
    OutOfOrder,
    /// A decoder rejected the stream.
    DecodeError,
    /// The transfer stalled and could not be finished cleanly.
    Timeout,
    /// A directory entry bound a format, but no data sector was ever accepted, so nothing began.
    NoData,
    /// Flashing failed in the given phase.
    FlashError(FlashPhase),
}

/// Sink that receives decoded image fragments and transfer lifecycle events.
pub trait TransferSink {
    /// Begin a transfer of `fmt`; `sniffed` is true when content, not a dirent, named it.
    fn begin(&mut self, fmt: Format, sniffed: bool);
    /// Deliver a decoded fragment at absolute `addr`.
    fn fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), DecodeError>;
    /// End the transfer with the final `outcome`.
    fn end(&mut self, outcome: TransferOutcome);
    /// Note the family id of the UF2 block about to be delivered, immediately before its
    /// [`fragment`](Self::fragment); `None` when the block declares none.
    fn note_uf2_family(&mut self, _family: Option<u32>) {}
    /// Note that the started transfer is bound to a visible image directory entry. May fire more
    /// than once per transfer, and never for a bound drop that never streamed.
    fn note_dirent_bound(&mut self) {}
    /// Whether the following remount warrants a bus-level USB re-enumeration; consulted once per
    /// remount edge, after [`end`](Self::end). Opt-in: UNIT ATTENTION alone republishes.
    fn wants_reconnect(&self) -> bool {
        false
    }
    /// Render a short status line describing the last transfer.
    fn render_status(&self, _out: &mut StatusWriter<'_>) {}
    /// Whether the last transfer failed.
    fn last_failed(&self) -> bool {
        false
    }
    /// The flash phase a failure already latched mid-transfer, before [`end`](Self::end) runs.
    fn latched_phase(&self) -> Option<FlashPhase> {
        None
    }
    /// Select per-sector (`true`) or whole-chip (`false`) erase for later sessions; a `false`
    /// return means the sink has no such control.
    fn set_sector_erase(&mut self, _sector: bool) -> bool {
        false
    }
}

/// A root-directory change observed by diffing host writes.
#[derive(Clone, Copy)]
pub enum FileEvent<'a> {
    /// A new entry appeared.
    Created {
        /// The 8.3 name field.
        name: &'a [u8; 11],
        /// The entry's reported size.
        size: u32,
        /// Whether the entry is hidden (or a macOS sidecar).
        hidden: bool,
        /// The entry's start cluster, or `0` when the host has not yet allocated one.
        first_cluster: u32,
    },
    /// An existing entry's size changed.
    Changed {
        /// The 8.3 name field.
        name: &'a [u8; 11],
        /// The new reported size.
        size: u32,
        /// The entry's start cluster (`0` if still unallocated).
        first_cluster: u32,
    },
    /// An entry was removed.
    Deleted {
        /// The 8.3 name field.
        name: &'a [u8; 11],
    },
}

/// The transfer engine's decision about one data-region sector write.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SectorDecision {
    /// Not the bound file's first sector: ignored before any transfer started.
    WindowSkip,
    /// The content does not begin the bound format, and the file's cluster window is unknown.
    GuardSkip,
    /// No accepted format could be detected for an unbound first sector: ignored.
    SniffNone,
    /// A transfer started, decoding as `fmt`; `sniffed` is true when content named the format.
    Start {
        /// The bound/detected format.
        fmt: Format,
        /// Whether the format was sniffed from content (vs a directory extension).
        sniffed: bool,
    },
    /// Streamed the next in-order sector of the active transfer.
    Stream,
    /// A sector foreign to the streaming file -- below its start, past its known size, or
    /// lacking UF2 block magic -- arriving mid-transfer: ignored, never an abort.
    MetaIgnore,
    /// A re-write of an already-streamed sector: discarded and recorded, and fatal only if the
    /// stream then ends unable to finish.
    RewriteIgnore,
    /// An in-window sector ahead of the contiguous streaming front: discarded and recorded, and
    /// a hole in the image unless the front later re-covers it.
    GapDiscard,
    /// The bound decoder rejected the stream near the given byte offset.
    DecodeFail {
        /// Approximate byte offset (bytes streamed so far) at which decoding failed.
        offset: u32,
    },
    /// A data-region write while no transfer is active (finished, or nothing to bind).
    Inactive,
}

/// Whether an observer consumed an event or lets it pass to the transfer engine.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Disposition {
    /// Let the transfer engine also see the event.
    Pass,
    /// The observer handled it; suppress engine handling.
    Handled,
}

/// A hook that sees directory events before the transfer engine.
pub trait VfsObserver {
    /// Handle a directory `evt`, returning whether to pass it on.
    fn on_file_event(&mut self, evt: FileEvent<'_>) -> Disposition;
}

/// The default observer: passes every event through.
pub struct NoObserver;

impl VfsObserver for NoObserver {
    fn on_file_event(&mut self, _evt: FileEvent<'_>) -> Disposition {
        Disposition::Pass
    }
}

/// Human-readable format name.
pub(crate) fn fmt_name(f: Option<Format>) -> &'static [u8] {
    match f {
        Some(Format::Uf2) => b"uf2",
        Some(Format::Unknown) => b"unknown",
        None => b"none",
    }
}

/// Human-readable outcome name.
pub(crate) fn outcome_name(o: Option<TransferOutcome>) -> &'static [u8] {
    match o {
        None => b"none",
        Some(TransferOutcome::Done) => b"Done",
        Some(TransferOutcome::OutOfOrder) => b"OutOfOrder",
        Some(TransferOutcome::DecodeError) => b"DecodeError",
        Some(TransferOutcome::Timeout) => b"Timeout",
        Some(TransferOutcome::NoData) => b"NoData (no image data recognized)",
        Some(TransferOutcome::FlashError(_)) => b"FlashError",
    }
}

/// Human-readable flash-phase name.
pub(crate) fn phase_name(p: FlashPhase) -> &'static [u8] {
    match p {
        FlashPhase::NoTarget => b"NoTarget",
        FlashPhase::Init => b"Init",
        FlashPhase::Identity => b"Identity",
        FlashPhase::Family => b"Family",
        FlashPhase::Content => b"Content",
        FlashPhase::Range => b"Range",
        FlashPhase::Erase => b"Erase",
        FlashPhase::Program => b"Program",
        FlashPhase::Verify => b"Verify",
        FlashPhase::Uninit => b"Uninit",
    }
}

/// Bridges a [`TransferSink`] to the decoder's [`DecoderContext`].
pub(crate) struct SinkCtx<'a, S: TransferSink> {
    pub(crate) sink: &'a mut S,
}

impl<S: TransferSink> DecoderContext for SinkCtx<'_, S> {
    fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.sink.fragment(addr, data)
    }

    fn set_uf2_family(&mut self, family: Option<u32>) {
        self.sink.note_uf2_family(family);
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    InProgress,
    Finished,
}

/// The transfer state machine, generic over the firmware-composed accepted-format set `D`.
pub(crate) struct Transfer<D: DecoderSet> {
    state: State,
    fmt: Option<Format>,
    start_lba: u32,
    next_lba: u32,
    bytes_streamed: u32,
    expected_size: Option<u32>,
    bound_ext: Option<Format>,
    bound_name: Option<[u8; 11]>,
    /// The LBA of the bound file's first data sector, or `None` while no cluster is allocated.
    /// Only that sector may start the transfer; earlier data-region writes are ignored.
    win_start_lba: Option<u32>,
    decoder: Option<D::Running>,
    scratch: [u8; 1024],
    finished_latch: bool,
    /// A re-write of an already-streamed sector was discarded; benign unless the stream stalls.
    saw_rewrite: bool,
    /// An in-window sector ahead of the front was discarded; the image is torn unless the front
    /// later re-covered it.
    saw_gap: bool,
}

/// Where a mid-stream sector write falls relative to the active transfer.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SectorClass {
    /// Not the streaming file's data: below its start, past its window, or lacking UF2 magic.
    Foreign,
    /// The next in-order sector (`== next_lba`).
    AtFront,
    /// An already-streamed sector of the file (`start_lba <= lba < next_lba`).
    BehindFront,
    /// Ahead of the contiguous front but within the file's window.
    AheadOfFront,
}

/// UF2 block magic (first two words); every real block carries it, host metadata does not.
const UF2_MAGIC_START0: u32 = 0x0A32_4655;
const UF2_MAGIC_START1: u32 = 0x9E5D_5157;

/// Whether `sector` begins with the UF2 block magic.
fn uf2_magic(sector: &[u8; 512]) -> bool {
    u32::from_le_bytes([sector[0], sector[1], sector[2], sector[3]]) == UF2_MAGIC_START0
        && u32::from_le_bytes([sector[4], sector[5], sector[6], sector[7]]) == UF2_MAGIC_START1
}

impl<D: DecoderSet> Transfer<D> {
    pub(crate) fn new() -> Self {
        Self {
            state: State::Idle,
            fmt: None,
            start_lba: 0,
            next_lba: 0,
            bytes_streamed: 0,
            expected_size: None,
            bound_ext: None,
            bound_name: None,
            win_start_lba: None,
            decoder: None,
            scratch: [0u8; 1024],
            finished_latch: false,
            saw_rewrite: false,
            saw_gap: false,
        }
    }

    pub(crate) fn is_active(&self) -> bool {
        self.state == State::InProgress
    }

    pub(crate) fn can_finish(&self) -> bool {
        // UF2 ends at a block boundary, but a stall short of a declared size must not be Done.
        let size_met = self
            .expected_size
            .is_none_or(|exp| self.bytes_streamed >= exp);
        self.state == State::InProgress && self.fmt == Some(Format::Uf2) && size_met
    }

    /// The idle threshold, in ms, that applies once [`can_finish`](Self::can_finish) is true:
    /// tight for a size that was declared and met, longer for a stream still unsized.
    pub(crate) fn finishable_timeout_ms(&self) -> u32 {
        if self.expected_size.is_none() {
            crate::MSC_TIMEOUT_FINISHABLE_UNSIZED_MS
        } else {
            crate::MSC_TIMEOUT_FINISHABLE_MS
        }
    }

    /// Whether a dirent bound a format but no data sector ever started a transfer. Such a drop
    /// must fail loud through [`fail_pending`](Self::fail_pending) rather than vanish.
    pub(crate) fn is_pending(&self) -> bool {
        self.state == State::Idle && self.bound_ext.is_some()
    }

    /// Fail a bound-but-never-started drop loud: latch [`TransferOutcome::NoData`] into the sink
    /// and finish the machine. No flash session is opened, so the remount stays quiet.
    pub(crate) fn fail_pending<S: TransferSink>(&mut self, sink: &mut S) {
        debug_assert!(self.is_pending());
        if let Some(fmt) = self.bound_ext {
            // Named by a directory entry, so never a guess.
            sink.begin(fmt, false);
        }
        self.end(TransferOutcome::NoData, sink);
    }

    /// Return and clear the finished latch, resetting the machine when it fires.
    pub(crate) fn take_finished(&mut self) -> bool {
        if self.finished_latch {
            *self = Self::new();
            true
        } else {
            false
        }
    }

    /// Bind a format, expected size, and data window from a directory event, restricted to the
    /// composed set. An event that meets the streamed size completes the transfer inline.
    pub(crate) fn file_event<S: TransferSink>(&mut self, evt: &FileEvent, sink: &mut S) {
        let (name, size, hidden, first_cluster) = match *evt {
            FileEvent::Created {
                name,
                size,
                hidden,
                first_cluster,
            } => (name, size, hidden, first_cluster),
            // `Changed` carries no attribute byte, so a sidecar's close would otherwise rebind.
            FileEvent::Changed {
                name,
                size,
                first_cluster,
            } => (
                name,
                size,
                name[0] == b'.' && name[1] == b'_',
                first_cluster,
            ),
            FileEvent::Deleted { name } => {
                // In flight, a dirent diff may legitimately pass through a deleted-looking state.
                if self.state == State::Idle && self.bound_name == Some(*name) {
                    self.bound_ext = None;
                    self.bound_name = None;
                    self.expected_size = None;
                    self.win_start_lba = None;
                }
                return;
            }
        };
        if hidden {
            return;
        }
        let Some(f) = D::ext_format(&[name[8], name[9], name[10]]) else {
            return;
        };
        // In progress, only the streaming file's own entry may update the expected size.
        if self.state == State::InProgress
            && !self.entry_describes_active_stream(name, first_cluster)
        {
            return;
        }
        self.bound_ext = Some(f);
        // A reported size of 0 means "unknown", not "empty".
        self.expected_size = if size == 0 { None } else { Some(size) };
        self.bound_name = Some(*name);
        // Complete inline the moment a newly-known size is met, rather than an idle gap later.
        if self.state == State::InProgress {
            // Now dirent-bound, so tell the sink before any inline completion ends the transfer.
            sink.note_dirent_bound();
            if let Some(exp) = self.expected_size {
                if exp > 0 && self.bytes_streamed >= exp {
                    self.complete(sink);
                    return;
                }
            }
        }
        // `first_cluster == 0` leaves the window unknown; the content guard in `start` covers it.
        if self.state != State::InProgress {
            self.win_start_lba = if first_cluster >= 2 {
                Some(fat::cluster_to_lba(first_cluster))
            } else {
                None
            };
        }
    }

    /// Whether a directory entry arriving mid-stream describes the bytes being streamed. A named
    /// stream answers to its name; a sniff-started one only to a window beginning where it did.
    fn entry_describes_active_stream(&self, name: &[u8; 11], first_cluster: u32) -> bool {
        match self.bound_name {
            Some(active) => active == *name,
            None => first_cluster >= 2 && fat::cluster_to_lba(first_cluster) == self.start_lba,
        }
    }

    /// Whether `sector` is a plausible first sector for the bound format. Consulted only while
    /// the file's cluster window is unknown, to keep host metadata from starting a transfer.
    fn plausible_bound_start(&self, sector: &[u8; 512]) -> bool {
        match self.bound_ext {
            Some(Format::Uf2) => uf2_magic(sector),
            _ => true,
        }
    }

    /// Whether a mid-stream `lba` is foreign to the bound file: below its start, past its known
    /// size, or lacking UF2 magic. Foreign sectors are ignored, never an abort.
    fn is_foreign(&self, lba: u32, sector: &[u8; 512]) -> bool {
        if lba < self.start_lba {
            return true;
        }
        if self.size_end_lba().is_some_and(|end| lba >= end) {
            return true;
        }
        if self.fmt == Some(Format::Uf2) && !uf2_magic(sector) {
            return true;
        }
        false
    }

    /// The first LBA past the streaming file's data, or `None` while no entry declared a size.
    /// The engine's single statement of where the file ends, and what makes a sector foreign.
    fn size_end_lba(&self) -> Option<u32> {
        self.expected_size.map(|sz| {
            self.start_lba
                .saturating_add(fat::div_ceil_u32(sz, 512).max(1))
        })
    }

    /// Where a mid-stream `lba` falls relative to the active transfer; `InProgress` only.
    fn classify(&self, lba: u32, sector: &[u8; 512]) -> SectorClass {
        if self.is_foreign(lba, sector) {
            return SectorClass::Foreign;
        }
        if lba == self.next_lba {
            SectorClass::AtFront
        } else if lba < self.next_lba {
            // `is_foreign` already excluded `lba < start_lba`.
            SectorClass::BehindFront
        } else {
            SectorClass::AheadOfFront
        }
    }

    pub(crate) fn data_sector<S: TransferSink>(
        &mut self,
        lba: u32,
        sector: &[u8; 512],
        sink: &mut S,
    ) -> SectorDecision {
        match self.state {
            State::Idle => self.start(lba, sector, sink),
            // Discard and record all but the in-order front; no mid-stream write ever aborts.
            State::InProgress => match self.classify(lba, sector) {
                SectorClass::Foreign => SectorDecision::MetaIgnore,
                SectorClass::AtFront => self.stream_sector(lba, sector, sink),
                SectorClass::BehindFront => {
                    // Idempotent for a host re-flush, but recorded so a later stall fails loud.
                    self.saw_rewrite = true;
                    SectorDecision::RewriteIgnore
                }
                SectorClass::AheadOfFront => {
                    // Recorded so an unhealed gap classifies OutOfOrder, never Done with a hole.
                    self.saw_gap = true;
                    SectorDecision::GapDiscard
                }
            },
            State::Finished => SectorDecision::Inactive,
        }
    }

    fn start<S: TransferSink>(
        &mut self,
        lba: u32,
        sector: &[u8; 512],
        sink: &mut S,
    ) -> SectorDecision {
        // An unknown cluster window leaves the content guard to keep metadata from starting.
        if let Some(ws) = self.win_start_lba {
            if lba != ws {
                return SectorDecision::WindowSkip;
            }
        } else if !self.plausible_bound_start(sector) {
            return SectorDecision::GuardSkip;
        }
        // A host-supplied file extension is authoritative; only when unbound do we sniff.
        let sniffed = self.bound_ext.is_none();
        let fmt = if let Some(be) = self.bound_ext {
            be
        } else {
            match D::detect(sector) {
                Some(fmt) => fmt,
                None => return SectorDecision::SniffNone,
            }
        };
        let Some(dec) = D::make(fmt) else {
            return SectorDecision::SniffNone;
        };
        self.decoder = Some(dec);
        self.fmt = Some(fmt);
        self.state = State::InProgress;
        self.start_lba = lba;
        self.next_lba = lba;
        self.bytes_streamed = 0;
        sink.begin(fmt, sniffed);
        if !sniffed {
            // Noted before the first sector streams, so even a first-fragment failure knows.
            sink.note_dirent_bound();
        }
        match self.stream_sector(lba, sector, sink) {
            fail @ SectorDecision::DecodeFail { .. } => fail,
            _ => SectorDecision::Start { fmt, sniffed },
        }
    }

    fn feed<S: TransferSink>(&mut self, sector: &[u8], sink: &mut S) -> Result<(), Error> {
        let scratch = &mut self.scratch;
        let dec = self
            .decoder
            .as_mut()
            .expect("decoder present while streaming");
        let mut ctx = SinkCtx { sink };
        dec.input(sector, scratch, &mut ctx)
    }

    fn stream_sector<S: TransferSink>(
        &mut self,
        lba: u32,
        sector: &[u8; 512],
        sink: &mut S,
    ) -> SectorDecision {
        // Feed only the bytes inside a known size, so no FAT cluster slack is programmed.
        let feed = match self.expected_size {
            Some(exp) => {
                let remaining = exp.saturating_sub(self.bytes_streamed);
                &sector[..core::cmp::min(512u32, remaining) as usize]
            }
            None => &sector[..],
        };
        if self.feed(feed, sink).is_err() {
            let offset = self.bytes_streamed;
            self.end(TransferOutcome::DecodeError, sink);
            return SectorDecision::DecodeFail { offset };
        }
        self.next_lba = lba + 1;
        self.bytes_streamed = self.bytes_streamed.saturating_add(512);
        if let Some(exp) = self.expected_size {
            // Guarded so a completion can never fire with nothing streamed.
            if exp > 0 && self.bytes_streamed >= exp {
                self.complete(sink);
            }
        }
        SectorDecision::Stream
    }

    fn complete<S: TransferSink>(&mut self, sink: &mut S) {
        let res = {
            let scratch = &mut self.scratch;
            let dec = self
                .decoder
                .as_mut()
                .expect("decoder present at completion");
            let mut ctx = SinkCtx { sink: &mut *sink };
            dec.finish(scratch, &mut ctx)
        };
        match res {
            Ok(()) => self.end(TransferOutcome::Done, sink),
            Err(_) => self.end(TransferOutcome::DecodeError, sink),
        }
    }

    fn end<S: TransferSink>(&mut self, outcome: TransferOutcome, sink: &mut S) {
        #[cfg(feature = "defmt")]
        defmt::info!(
            "vfs xfer end outcome={=[u8]:a} bytes={=u32}",
            outcome_name(Some(outcome)),
            self.bytes_streamed
        );
        sink.end(outcome);
        self.state = State::Finished;
        self.finished_latch = true;
        self.decoder = None;
    }

    /// Classify and finish a stalled stream: a recorded gap tears the image whatever else holds,
    /// a finishable stream completes, and a recorded rewrite separates `OutOfOrder` from `Timeout`.
    pub(crate) fn on_timeout<S: TransferSink>(&mut self, sink: &mut S) {
        if self.state != State::InProgress {
            return;
        }
        if self.saw_gap {
            self.end(TransferOutcome::OutOfOrder, sink);
        } else if self.can_finish() {
            self.complete(sink);
        } else if self.saw_rewrite {
            self.end(TransferOutcome::OutOfOrder, sink);
        } else {
            self.end(TransferOutcome::Timeout, sink);
        }
    }
}

/// A sink that accepts and discards every fragment.
pub struct NullSink;

impl TransferSink for NullSink {
    fn begin(&mut self, _fmt: Format, _sniffed: bool) {}
    fn fragment(&mut self, _addr: u32, _data: &[u8]) -> Result<(), DecodeError> {
        Ok(())
    }
    fn end(&mut self, _outcome: TransferOutcome) {}
}

/// Per-transfer accumulator every concrete sink embeds: fragment/byte counters and the outcome.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TransferStats {
    /// Number of fragments seen.
    pub fragments: u32,
    /// Total bytes seen.
    pub bytes: u32,
    /// The recorded outcome.
    pub outcome: Option<TransferOutcome>,
}

impl TransferStats {
    /// Reset every counter and clear the outcome.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Record one fragment of `len` bytes (saturating).
    pub fn record_fragment(&mut self, len: usize) {
        self.fragments = self.fragments.saturating_add(1);
        self.bytes = self.bytes.saturating_add(len as u32);
    }

    /// Whether a recorded outcome exists and is not [`TransferOutcome::Done`].
    pub fn failed(&self) -> bool {
        matches!(self.outcome, Some(o) if o != TransferOutcome::Done)
    }
}

/// A sink that accumulates statistics about the last transfer.
#[derive(Clone, Copy, Debug)]
pub struct StatsSink {
    /// The detected format.
    pub fmt: Option<Format>,
    /// Fragment/byte counters and the recorded outcome.
    pub stats: TransferStats,
    /// Lowest fragment address.
    pub addr_min: u32,
    /// Highest fragment end address.
    pub addr_max: u32,
    /// The first decode error, if any.
    pub first_error: Option<DecodeError>,
}

impl Default for StatsSink {
    // Not derived: `addr_min` seeds to `u32::MAX`, not 0, so the first fragment sets it.
    fn default() -> Self {
        Self::new()
    }
}

impl StatsSink {
    /// Create an empty stats sink.
    pub fn new() -> Self {
        Self {
            fmt: None,
            stats: TransferStats::default(),
            addr_min: u32::MAX,
            addr_max: 0,
            first_error: None,
        }
    }

    /// The recorded outcome of the last transfer.
    pub fn outcome(&self) -> Option<TransferOutcome> {
        self.stats.outcome
    }
}

impl TransferSink for StatsSink {
    fn begin(&mut self, fmt: Format, _sniffed: bool) {
        self.fmt = Some(fmt);
        self.stats.reset();
        self.addr_min = u32::MAX;
        self.addr_max = 0;
        self.first_error = None;
    }

    fn fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), DecodeError> {
        self.stats.record_fragment(data.len());
        if addr < self.addr_min {
            self.addr_min = addr;
        }
        let end = addr.saturating_add(data.len() as u32);
        if end > self.addr_max {
            self.addr_max = end;
        }
        Ok(())
    }

    fn end(&mut self, outcome: TransferOutcome) {
        self.stats.outcome = Some(outcome);
    }

    fn render_status(&self, out: &mut StatusWriter<'_>) {
        out.bytes(b"Last transfer: ");
        out.bytes(fmt_name(self.fmt));
        // A never-started drop has no byte count or address range worth the bounded field.
        if self.stats.outcome != Some(TransferOutcome::NoData) {
            out.bytes(b" ");
            out.dec_u32(self.stats.bytes);
            out.bytes(b" bytes @");
            out.hex_u32(self.addr_min);
            out.bytes(b"..");
            out.hex_u32(self.addr_max);
        }
        out.bytes(b" -> ");
        out.bytes(outcome_name(self.stats.outcome));
    }

    fn last_failed(&self) -> bool {
        self.stats.failed()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder_set::AllDecoders;
    use std::vec::Vec;

    struct RecSink {
        began: bool,
        frags: Vec<(u32, Vec<u8>)>,
        outcome: Option<TransferOutcome>,
        /// Whether the engine reported this start as sniffed from content.
        sniffed: bool,
    }
    impl RecSink {
        fn new() -> Self {
            Self {
                began: false,
                frags: Vec::new(),
                outcome: None,
                sniffed: false,
            }
        }
    }
    impl TransferSink for RecSink {
        fn begin(&mut self, _fmt: Format, sniffed: bool) {
            self.began = true;
            self.sniffed = sniffed;
        }
        fn fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), DecodeError> {
            self.frags.push((addr, data.to_vec()));
            Ok(())
        }
        fn end(&mut self, outcome: TransferOutcome) {
            self.outcome = Some(outcome);
        }
    }

    /// One well-formed 512-byte UF2 block carrying `payload` at `addr`.
    fn uf2_block(addr: u32, payload: &[u8]) -> [u8; 512] {
        let mut b = [0u8; 512];
        b[0..4].copy_from_slice(&UF2_MAGIC_START0.to_le_bytes());
        b[4..8].copy_from_slice(&UF2_MAGIC_START1.to_le_bytes());
        b[12..16].copy_from_slice(&addr.to_le_bytes());
        b[16..20].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        b[32..32 + payload.len()].copy_from_slice(payload);
        b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes());
        b
    }

    /// A dirent create event for `name`, with the parameters each test varies.
    fn created(name: &[u8; 11], size: u32, first_cluster: u32) -> FileEvent<'_> {
        FileEvent::Created {
            name,
            size,
            hidden: false,
            first_cluster,
        }
    }

    #[test]
    fn uf2_detected_and_completes() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 512, 2), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2, 3, 4]), &mut sink);
        assert!(sink.began);
        assert_eq!(sink.frags.len(), 1);
        assert_eq!(sink.frags[0].0, 0x0800_0000);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    /// Host metadata cannot start a bound `.uf2` transfer while the window is unknown.
    #[test]
    fn metadata_still_guard_skipped_for_bound_uf2() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        let mut sidecar = [0u8; 512];
        sidecar[0..8].copy_from_slice(&[0x00, 0x05, 0x16, 0x07, 0x00, 0x02, 0x00, 0x00]);
        assert_eq!(
            t.data_sector(120, &sidecar, &mut sink),
            SectorDecision::GuardSkip
        );
        assert_eq!(
            t.data_sector(121, &[0u8; 512], &mut sink),
            SectorDecision::GuardSkip
        );
        assert!(!t.is_active());
    }

    /// A sector nothing named and nothing recognizes starts nothing at all.
    #[test]
    fn unrecognized_unbound_sector_starts_nothing() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        assert_eq!(
            t.data_sector(133, &[0xA5u8; 512], &mut sink),
            SectorDecision::SniffNone
        );
        assert!(!t.is_active());
        assert!(!sink.began);
    }

    /// Payload before the directory entry: its magic starts the transfer, the late entry sizes
    /// and completes it.
    #[test]
    fn sniffed_start_and_a_late_dirent_completes_it() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        assert_eq!(fat::cluster_to_lba(8), 181);
        let decision = t.data_sector(181, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert!(
            matches!(
                decision,
                SectorDecision::Start {
                    fmt: Format::Uf2,
                    sniffed: true
                }
            ),
            "the image's first sector must start the transfer, got {decision:?}"
        );
        assert!(sink.sniffed, "the sink is told the start was a guess");
        for lba in 182..184 {
            assert_eq!(
                t.data_sector(lba, &uf2_block(0x0800_0100, &[3, 4]), &mut sink),
                SectorDecision::Stream
            );
        }
        t.file_event(&created(b"APP     UF2", 3 * 512, 8), &mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
        assert_eq!(sink.frags.len(), 3, "every payload sector reached the sink");
    }

    /// Two images dropped together: the first entry to land must not claim the other's stream.
    #[test]
    fn a_foreign_dirent_cannot_claim_a_sniffed_stream() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        assert_eq!(fat::cluster_to_lba(8), 181);
        t.data_sector(181, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert!(t.is_active());
        for lba in 182..184 {
            assert_eq!(
                t.data_sector(lba, &uf2_block(0x0800_0100, &[3, 4]), &mut sink),
                SectorDecision::Stream
            );
        }
        // BASE.UF2's entry lands first and describes a different data window (cluster 16).
        t.file_event(&created(b"BASE    UF2", 100 * 1024, 16), &mut sink);
        assert!(
            t.is_active() && sink.outcome.is_none(),
            "a foreign entry must neither bind nor end the stream"
        );
        assert_eq!(t.bound_name, None, "and must not name it");
        assert_eq!(t.expected_size, None, "nor size it");
        // ALT.UF2's own entry then binds by its data window and completes the stream.
        t.file_event(&created(b"ALT     UF2", 3 * 512, 8), &mut sink);
        assert_eq!(t.bound_name, Some(*b"ALT     UF2"));
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
        assert_eq!(
            sink.frags.len(),
            3,
            "the stream kept its own bytes and its own size"
        );
    }

    /// A clusterless entry has no window to check, so it defers; the rewrite carrying one binds.
    #[test]
    fn a_clusterless_dirent_defers_until_it_carries_a_window() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.data_sector(181, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        assert_eq!(t.bound_name, None, "no window to check: no claim");
        assert!(t.is_active());
        t.file_event(
            &FileEvent::Changed {
                name: b"APP     UF2",
                size: 512,
                first_cluster: 8,
            },
            &mut sink,
        );
        assert_eq!(t.bound_name, Some(*b"APP     UF2"));
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    /// A close tail that re-flushes a streamed sector and touches one past the front must still
    /// let the finishable idle timeout complete the transfer.
    #[test]
    fn unsized_trailing_rewrite_and_housekeeping_finish_done() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"TEST-R~3UF2", 0, 0), &mut sink);
        let head = uf2_block(0x0800_0000, &[1, 2]);
        t.data_sector(181, &head, &mut sink);
        for lba in 182..201 {
            assert_eq!(
                t.data_sector(lba, &uf2_block(0x0800_0100, &[3, 4]), &mut sink),
                SectorDecision::Stream
            );
        }
        assert!(t.is_active());
        assert_eq!(
            t.data_sector(181, &head, &mut sink),
            SectorDecision::RewriteIgnore
        );
        // Magicless housekeeping past the front is foreign, not a hole.
        assert_eq!(
            t.data_sector(205, &[0x00u8; 512], &mut sink),
            SectorDecision::MetaIgnore
        );
        assert!(t.is_active());
        assert_eq!(sink.outcome, None);
        t.on_timeout(&mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
        assert_eq!(sink.frags.len(), 20);
    }

    /// A sector below the start LBA mid-stream is foreign, and ignored.
    #[test]
    fn pre_start_sector_is_foreign() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert_eq!(
            t.data_sector(100, &uf2_block(0x0800_0100, &[3, 4]), &mut sink),
            SectorDecision::MetaIgnore
        );
        assert_eq!(sink.outcome, None);
        assert!(t.is_active());
    }

    /// An in-window gap ahead of the front on a sized stream classifies OutOfOrder at timeout.
    #[test]
    fn sized_forward_gap_out_of_order_at_timeout() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 2048, 2), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert!(t.is_active());
        assert_eq!(
            t.data_sector(135, &uf2_block(0x0800_0200, &[5, 6]), &mut sink),
            SectorDecision::GapDiscard
        );
        assert!(t.is_active());
        assert_eq!(sink.outcome, None);
        t.on_timeout(&mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::OutOfOrder));
    }

    /// A gap the front later re-covers is a transient reorder, and must still complete.
    #[test]
    fn forward_gap_heals_when_front_recovers() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 1536, 2), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        let late = uf2_block(0x0800_0200, &[5, 6]);
        assert_eq!(
            t.data_sector(135, &late, &mut sink),
            SectorDecision::GapDiscard
        );
        assert_eq!(
            t.data_sector(134, &uf2_block(0x0800_0100, &[3, 4]), &mut sink),
            SectorDecision::Stream
        );
        assert_eq!(t.data_sector(135, &late, &mut sink), SectorDecision::Stream);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
        assert_eq!(sink.frags.len(), 3);
    }

    /// A recorded gap fails even a stream that could otherwise finish.
    #[test]
    fn unsized_uf2_gap_beats_finishability_at_timeout() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        let block = uf2_block(0x0800_0000, &[1, 2, 3, 4]);
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        t.data_sector(133, &block, &mut sink);
        assert!(t.can_finish(), "an unsized uf2 stream is finishable");
        assert_eq!(
            t.data_sector(135, &block, &mut sink),
            SectorDecision::GapDiscard
        );
        t.on_timeout(&mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::OutOfOrder));
    }

    /// A recorded rewrite turns a stall that cannot finish into OutOfOrder, not a bare Timeout.
    #[test]
    fn sized_rewrite_then_stall_out_of_order_at_timeout() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 2048, 2), &mut sink);
        let head = uf2_block(0x0800_0000, &[1, 2]);
        t.data_sector(133, &head, &mut sink);
        assert_eq!(
            t.data_sector(133, &head, &mut sink),
            SectorDecision::RewriteIgnore
        );
        assert!(t.is_active());
        t.on_timeout(&mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::OutOfOrder));
    }

    /// A magic-bearing sector at or past the known window end is foreign, and ignored.
    #[test]
    fn beyond_window_sector_is_foreign() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        // Two sectors: window is [133, 135).
        t.file_event(&created(b"APP     UF2", 1024, 2), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert_eq!(
            t.data_sector(135, &uf2_block(0x0900_0000, &[9]), &mut sink),
            SectorDecision::MetaIgnore
        );
        assert!(t.is_active());
        t.data_sector(134, &uf2_block(0x0800_0100, &[3, 4]), &mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    /// A UF2 stream ignores any mid-stream sector without block magic, wherever it lands.
    #[test]
    fn uf2_nonmagic_sector_is_foreign() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2, 3, 4]), &mut sink);
        assert_eq!(
            t.data_sector(135, &[0x5Au8; 512], &mut sink),
            SectorDecision::MetaIgnore
        );
        assert!(t.is_active());
        t.on_timeout(&mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    #[test]
    fn uf2_dir_entry_after_last_block_completes_done() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        // Data block arrives before the root-dir entry supplies the size.
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2, 3, 4]), &mut sink);
        assert!(t.is_active());
        t.file_event(&created(b"APP     UF2", 512, 2), &mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    #[test]
    fn timeout_uf2_without_size_done() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2, 3, 4]), &mut sink);
        assert!(t.can_finish());
        t.on_timeout(&mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    /// A malformed block mid-stream fails the transfer at the byte offset it reached.
    #[test]
    fn decode_failure_ends_the_transfer() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        // Magic on the first two words but a broken end magic: foreign to nothing, fed, rejected.
        let mut bad = uf2_block(0x0800_0100, &[3, 4]);
        bad[508..512].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(
            t.data_sector(134, &bad, &mut sink),
            SectorDecision::DecodeFail { offset: 512 }
        );
        assert_eq!(sink.outcome, Some(TransferOutcome::DecodeError));
    }

    #[test]
    fn sidecar_extension_ignored() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(
            &FileEvent::Created {
                name: b"._APP   UF2",
                size: 512,
                hidden: true,
                first_cluster: 2,
            },
            &mut sink,
        );
        assert_eq!(t.bound_ext, None);
    }

    #[test]
    fn stats_sink_resets_and_tracks() {
        let mut s = StatsSink::new();
        assert_eq!(s.addr_min, u32::MAX);
        s.begin(Format::Uf2, false);
        s.fragment(0x100, &[0u8; 16]).unwrap();
        s.fragment(0x200, &[0u8; 8]).unwrap();
        assert_eq!(s.stats.fragments, 2);
        assert_eq!(s.stats.bytes, 24);
        assert_eq!(s.addr_min, 0x100);
        assert_eq!(s.addr_max, 0x208);
        s.end(TransferOutcome::Done);
        assert!(!s.last_failed());
        // begin() resets the accumulators.
        s.begin(Format::Uf2, false);
        assert_eq!(s.stats.fragments, 0);
        assert_eq!(s.addr_min, u32::MAX);
    }

    #[test]
    fn size_zero_created_streams_then_finishes_on_dirent() {
        // A size-0 Created must not complete the transfer empty on the first data sector.
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        assert_eq!(t.bound_ext, Some(Format::Uf2));
        assert_eq!(t.expected_size, None);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2, 3, 4]), &mut sink);
        assert!(t.is_active());
        assert_eq!(sink.outcome, None);
        assert_eq!(sink.frags.len(), 1);
        t.file_event(
            &FileEvent::Changed {
                name: b"APP     UF2",
                size: 512,
                first_cluster: 2,
            },
            &mut sink,
        );
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    #[test]
    fn changed_sidecar_does_not_bind() {
        // A sidecar's close fires Changed for "._APP   UF2"; it must not bind a format.
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(
            &FileEvent::Changed {
                name: b"._APP   UF2",
                size: 512,
                first_cluster: 2,
            },
            &mut sink,
        );
        assert_eq!(t.bound_ext, None);
        assert_eq!(t.expected_size, None);
    }

    #[test]
    fn second_file_event_does_not_truncate_in_progress() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 1024, 2), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert!(t.is_active());
        // A second, unrelated file's directory entry appears mid-stream with a small size.
        t.file_event(&created(b"OTHER   UF2", 16, 4), &mut sink);
        assert!(t.is_active());
        assert_eq!(sink.outcome, None);
        assert_eq!(t.expected_size, Some(1024));
        t.data_sector(134, &uf2_block(0x0800_0100, &[3, 4]), &mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Done));
    }

    #[test]
    fn stats_sink_default_matches_new() {
        // Default must agree with new() (addr_min seeded to u32::MAX, not 0).
        assert_eq!(StatsSink::default().addr_min, StatsSink::new().addr_min);
        assert_eq!(StatsSink::default().addr_min, u32::MAX);
    }

    #[test]
    fn stats_last_failed_matches_non_done() {
        let mut s = StatsSink::new();
        s.begin(Format::Uf2, false);
        s.end(TransferOutcome::Timeout);
        assert!(s.last_failed());
    }

    /// A dirent rewrite landing after the whole image streamed completes it inline, not at the
    /// idle timeout.
    #[test]
    fn dirent_size_met_completes_inline() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        for lba in 134..138 {
            assert_eq!(
                t.data_sector(lba, &uf2_block(0x0800_0100, &[3, 4]), &mut sink),
                SectorDecision::Stream
            );
        }
        assert!(t.is_active());
        assert_eq!(sink.outcome, None);
        t.file_event(
            &FileEvent::Changed {
                name: b"APP     UF2",
                size: 2560,
                first_cluster: 2,
            },
            &mut sink,
        );
        assert_eq!(
            sink.outcome,
            Some(TransferOutcome::Done),
            "size met on the dirent event completes the transfer inline"
        );
        assert!(!t.is_active());
    }

    /// A stall short of a known size fails loud, never completing a truncated prefix as Done.
    #[test]
    fn known_size_unmet_stall_fails_loud() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 2048, 2), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert!(t.is_active());
        assert!(
            !t.can_finish(),
            "a stream short of its known size is not finishable"
        );
        t.on_timeout(&mut sink);
        assert_eq!(sink.outcome, Some(TransferOutcome::Timeout));
    }

    /// A size arriving mid-stall beyond what streamed must drop finishability, so the timeout
    /// fails loud rather than flashing the short prefix.
    #[test]
    fn late_dirent_size_beyond_streamed_fails_loud() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 0, 0), &mut sink);
        t.data_sector(133, &uf2_block(0x0800_0000, &[1, 2]), &mut sink);
        assert!(
            t.can_finish(),
            "an unsized uf2 stream starts out finishable"
        );
        t.file_event(
            &FileEvent::Changed {
                name: b"APP     UF2",
                size: 4096,
                first_cluster: 2,
            },
            &mut sink,
        );
        assert!(t.is_active(), "an unmet size never completes the transfer");
        assert_eq!(sink.outcome, None);
        assert!(!t.can_finish(), "the known unmet size drops finishability");
        t.on_timeout(&mut sink);
        assert_eq!(
            sink.outcome,
            Some(TransferOutcome::Timeout),
            "the stalled short stream fails loud, not Done"
        );
    }

    /// A bound drop whose data never starts latches NoData, with no flash session.
    #[test]
    fn bound_never_streamed_fails_pending_nodata() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 4096, 0), &mut sink);
        // Every data sector fails the content guard (housekeeping-like content).
        assert_eq!(
            t.data_sector(133, &[0u8; 512], &mut sink),
            SectorDecision::GuardSkip
        );
        assert!(!t.is_active());
        assert!(t.is_pending(), "a bound, never-started drop is pending");
        t.fail_pending(&mut sink);
        assert!(sink.began);
        assert_eq!(sink.outcome, Some(TransferOutcome::NoData));
        assert!(t.take_finished(), "the failure finishes the machine");
        assert!(!t.is_pending());
    }

    /// Deleting the bound entry before any data streamed unbinds it.
    #[test]
    fn deleted_dirent_unbinds_pending() {
        let mut t = Transfer::<AllDecoders>::new();
        let mut sink = RecSink::new();
        t.file_event(&created(b"APP     UF2", 512, 2), &mut sink);
        assert!(t.is_pending());
        t.file_event(
            &FileEvent::Deleted {
                name: b"APP     UF2",
            },
            &mut sink,
        );
        assert!(!t.is_pending(), "deleting the entry unbinds the drop");
    }
}
