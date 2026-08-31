#![no_std]
#![deny(missing_docs)]
#![cfg_attr(not(test), deny(clippy::unwrap_used))]
//! A virtual FAT16 volume and drag-n-drop transfer engine for a USB mass-storage debug probe.

#[cfg(test)]
extern crate std;

pub mod decoder_set;
pub mod fat;
pub mod fault;
pub mod files;
pub mod flash_sink;
pub mod msc_bot;
pub mod msc_io;
pub mod queue;
pub mod render;
pub mod stamp;
pub mod transfer;
pub mod vendor_flash;
pub mod volume;

pub use decoder_set::{AllDecoders, DecoderSet, Either};
pub use files::{DetailsFile, DeviceInfo};
pub use files::{FatShortName, FileAttrs, VfsFile};
pub use flash_sink::FlashSink;
pub use msc_bot::{run_command, BulkIn, BulkOut, MsClock};
pub use msc_io::{msc_worker_step, DirectIo, QueuedIo, SectorIo, SectorPoll, WorkerStep};
pub use render::StatusWriter;
pub use stamp::ResetStamp;
pub use static_cell::{ConstStaticCell, StaticCell};
pub use transfer::StatsSink;
pub use transfer::{
    DecodeError, Disposition, FileEvent, FlashPhase, NoObserver, NullSink, SectorDecision,
    TransferOutcome, TransferSink, VfsObserver,
};
pub use vendor_flash::{VendorFlash, VendorFlashStream};
pub use volume::{assemble, assemble_info, FlashVolume, VolumeStorage};

use transfer::Transfer;

/// Maximum number of files the volume can expose.
pub const MAX_FILES: usize = 8;

/// Root-directory slots tracked for host diffing.
const ROOT_SLOTS: usize = fat::ROOT_ENTRIES as usize;

/// Idle timeout, in ms, while a transfer is actively receiving data.
pub const MSC_TIMEOUT_ACTIVE_MS: u32 = 20_000;
/// Idle timeout, in ms, once a transfer could finish and a directory-declared size has been met.
pub const MSC_TIMEOUT_FINISHABLE_MS: u32 = 500;
/// Idle timeout, in ms, once a transfer could finish but no directory entry declared a size.
/// Longer than the sized window: an idle gap cannot tell a finished stream from a paused one.
pub const MSC_TIMEOUT_FINISHABLE_UNSIZED_MS: u32 = 5_000;
/// Time, in ms, the volume stays detached before remounting.
pub const MSC_RECONNECT_MS: u32 = 2_500;
/// Upper bound, in ms, on how long an armed detach waits for the transport's data phase to close.
pub const MSC_DETACH_DEFER_MS: u32 = 1_000;

/// Volume configuration.
pub struct VfsConfig<'a> {
    /// The volume label (uppercased and padded to 11 bytes).
    pub volume_label: &'a str,
    /// The 32-bit FAT volume id baked into the boot sector; per probe, so two drives attached at
    /// once are distinct volumes to the host. Defaults to [`fat::DEFAULT_VOLUME_ID`].
    pub volume_id: u32,
}

impl Default for VfsConfig<'_> {
    fn default() -> Self {
        Self {
            volume_label: "DAPLINK",
            volume_id: fat::DEFAULT_VOLUME_ID,
        }
    }
}

/// Whether the virtual medium is currently presented to the host.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MediaState {
    /// The medium is present.
    Ready,
    /// The medium is detached (a remount is in progress).
    Detached,
}

/// A per-file layout snapshot: size and contiguous cluster allocation.
#[derive(Clone, Copy)]
struct FileSnap {
    size: u32,
    first_cluster: u32,
    n_clusters: u32,
}

/// A shadow of one root-directory slot, used to diff host writes.
#[derive(Clone, Copy)]
struct ShadowEntry {
    present: bool,
    name: [u8; 11],
    size: u32,
}

/// The virtual FAT16 volume integrating rendering, host diffing, and the transfer engine, over
/// the firmware-composed accepted-format set `D` (see [`DecoderSet`]).
pub struct Vfs<'a, S: TransferSink, O: VfsObserver = NoObserver, D: DecoderSet = AllDecoders> {
    label: [u8; 11],
    volume_id: u32,
    files: &'a mut [&'a mut dyn VfsFile],
    snaps: [FileSnap; MAX_FILES],
    n_files: usize,
    sink: S,
    observer: O,
    transfer: Transfer<D>,
    media: MediaState,
    idle_ms: u32,
    /// Countdown, in ms, until a detached volume remounts (re-armed by [`Self::detach_now`]).
    remount_in_ms: u32,
    /// Whether the transport is inside a host command's data phase, per [`Self::set_data_phase`].
    data_phase_open: bool,
    /// A detach armed while the data phase was open, waiting for it to close.
    detach_pending: bool,
    /// Time accumulated with `detach_pending` set, against [`MSC_DETACH_DEFER_MS`].
    detach_defer_ms: u32,
    shadow: [ShadowEntry; ROOT_SLOTS],
    transfer_done_latch: bool,
    media_changed_latch: bool,
    reconnect_latch: bool,
    /// Optional reset-surviving stamp, written through with each remount's status.
    stamp: Option<&'a mut ResetStamp>,
}

/// Dirent flag bits for the defmt directory trace: hidden, generated, size-changed.
#[cfg(feature = "defmt")]
const FLAG_HIDDEN: u8 = 0b001;
#[cfg(feature = "defmt")]
const FLAG_GENERATED: u8 = 0b010;
#[cfg(feature = "defmt")]
const FLAG_CHANGED: u8 = 0b100;

/// Short tag for a [`SectorDecision`] used by the defmt trace.
#[cfg(feature = "defmt")]
fn decision_tag(d: SectorDecision) -> &'static str {
    match d {
        SectorDecision::WindowSkip => "WINDOW-SKIP(meta)",
        SectorDecision::GuardSkip => "GUARD-SKIP(meta)",
        SectorDecision::SniffNone => "SNIFF-NONE",
        SectorDecision::Start { .. } => "START",
        SectorDecision::Stream => "STREAM",
        SectorDecision::MetaIgnore => "META-IGNORE",
        SectorDecision::RewriteIgnore => "REWRITE-IGNORE",
        SectorDecision::GapDiscard => "GAP-DISCARD",
        SectorDecision::DecodeFail { .. } => "DECODE-FAIL",
        SectorDecision::Inactive => "INACTIVE",
    }
}

/// Short tag for a decoder [`daplink_decoder::Format`] used by the defmt trace.
#[cfg(feature = "defmt")]
fn format_tag(f: daplink_decoder::Format) -> &'static str {
    match f {
        daplink_decoder::Format::Uf2 => "uf2",
        daplink_decoder::Format::Unknown => "unknown",
    }
}

/// Emit a defmt trace line for one data-sector decision.
#[cfg(feature = "defmt")]
fn trace_sector(lba: u32, head: &[u8; 8], decision: SectorDecision) {
    match decision {
        SectorDecision::Start { fmt, sniffed } => defmt::info!(
            "vfs sector lba={=u32} head={=[u8]:02x} -> START fmt={=str} sniffed={=bool}",
            lba,
            &head[..],
            format_tag(fmt),
            sniffed
        ),
        SectorDecision::DecodeFail { offset } => defmt::warn!(
            "vfs sector lba={=u32} head={=[u8]:02x} -> DECODE-FAIL@{=u32}",
            lba,
            &head[..],
            offset
        ),
        other => defmt::trace!(
            "vfs sector lba={=u32} head={=[u8]:02x} -> {=str}",
            lba,
            &head[..],
            decision_tag(other)
        ),
    }
}

/// Trace one write to the discarded reserved region: the boot sector and both FAT copies.
#[cfg(feature = "defmt")]
fn trace_reserved(lba: u32, buf: &[u8; 512]) {
    if lba == 0 {
        defmt::trace!(
            "vfs reserved lba={=u32} head={=[u8]:02x} -> BOOT",
            lba,
            &buf[..8]
        );
        return;
    }
    // Each FAT sector holds 256 little-endian entries, covering clusters `idx * 256` upwards.
    let idx = (lba - 1) % fat::FAT_SECTORS;
    let base = idx * 256;
    let (mut used, mut lo, mut hi) = (0u32, 0u32, 0u32);
    for i in 0..256usize {
        if buf[2 * i] != 0 || buf[2 * i + 1] != 0 {
            if used == 0 {
                lo = base + i as u32;
            }
            hi = base + i as u32;
            used += 1;
        }
    }
    defmt::trace!(
        "vfs fat lba={=u32} copy={=u32} idx={=u32} head={=[u8]:02x} used={=u32} clu={=u32}..{=u32}",
        lba,
        if lba <= fat::FAT_SECTORS { 1 } else { 2 },
        idx,
        &buf[..8],
        used,
        lo,
        hi
    );
}

/// The FAT volume id for the probe carrying `serial`, falling back to
/// [`fat::DEFAULT_VOLUME_ID`] when the serial is not hexadecimal.
pub fn volume_id_from_serial(serial: &str) -> u32 {
    daplink_core::serial::volume_id(serial).unwrap_or(fat::DEFAULT_VOLUME_ID)
}

/// [`Vfs::snapshot`]'s body, over the parts rather than over `&mut self`.
fn snapshot_into(files: &[&mut dyn VfsFile], n_files: usize, snaps: &mut [FileSnap; MAX_FILES]) {
    let bytes_per = fat::SECTORS_PER_CLUSTER * fat::SECTOR_SIZE as u32;
    let mut cluster = 2u32;
    for i in 0..n_files {
        if !files[i].present() {
            snaps[i] = FileSnap {
                size: 0,
                first_cluster: cluster,
                n_clusters: 0,
            };
            continue;
        }
        let size = files[i].size();
        let mut n = fat::div_ceil_u32(size, bytes_per);
        if n == 0 {
            n = 1;
        }
        snaps[i] = FileSnap {
            size,
            first_cluster: cluster,
            n_clusters: n,
        };
        cluster += n;
    }
}

/// Uppercase and space-pad a label to 11 bytes.
fn label_bytes(s: &str) -> [u8; 11] {
    let mut b = [b' '; 11];
    for (i, c) in s.bytes().take(11).enumerate() {
        b[i] = c.to_ascii_uppercase();
    }
    b
}

impl<'a, S: TransferSink, O: VfsObserver, D: DecoderSet> Vfs<'a, S, O, D> {
    /// Create a volume over `files`, snapshotting their initial layout.
    pub fn new(
        cfg: VfsConfig<'a>,
        files: &'a mut [&'a mut dyn VfsFile],
        sink: S,
        observer: O,
    ) -> Self {
        // A tail expression, never a `let mut`: a local builds the volume in the caller's frame.
        let n = files.len().min(MAX_FILES);
        let mut snaps = [FileSnap {
            size: 0,
            first_cluster: 2,
            n_clusters: 1,
        }; MAX_FILES];
        snapshot_into(files, n, &mut snaps);
        Self {
            label: label_bytes(cfg.volume_label),
            volume_id: cfg.volume_id,
            files,
            snaps,
            n_files: n,
            sink,
            observer,
            transfer: Transfer::new(),
            media: MediaState::Ready,
            idle_ms: 0,
            remount_in_ms: 0,
            data_phase_open: false,
            detach_pending: false,
            detach_defer_ms: 0,
            shadow: [ShadowEntry {
                present: false,
                name: [0u8; 11],
                size: 0,
            }; ROOT_SLOTS],
            transfer_done_latch: false,
            media_changed_latch: false,
            reconnect_latch: false,
            stamp: None,
        }
    }

    /// Attach a reset-surviving [`ResetStamp`]: every remount's status is written through to it.
    pub fn set_reset_stamp(&mut self, stamp: &'a mut ResetStamp) {
        self.stamp = Some(stamp);
    }

    /// The current media state.
    pub fn media(&self) -> MediaState {
        self.media
    }

    /// Borrow the sink.
    pub fn sink(&self) -> &S {
        &self.sink
    }

    /// Mutably borrow the sink.
    pub fn sink_mut(&mut self) -> &mut S {
        &mut self.sink
    }

    /// Whether a transfer is currently in progress.
    pub fn is_transfer_active(&self) -> bool {
        self.transfer.is_active()
    }

    /// The idle timeout, in ms, that currently applies (0 when idle). A bound but never-started
    /// drop is under the active timeout too, so it fails loud rather than vanishing.
    pub fn transfer_timeout(&self) -> u32 {
        if self.transfer.is_active() {
            if self.transfer.can_finish() {
                self.transfer.finishable_timeout_ms()
            } else {
                MSC_TIMEOUT_ACTIVE_MS
            }
        } else if self.transfer.is_pending() {
            MSC_TIMEOUT_ACTIVE_MS
        } else {
            0
        }
    }

    /// Return and clear the "a transfer just finished" latch.
    pub fn take_transfer_finished(&mut self) -> bool {
        let f = self.transfer_done_latch;
        self.transfer_done_latch = false;
        f
    }

    /// Return and clear the "the medium just changed" latch, set on each `Detached -> Ready`
    /// remount edge (never by the initial mount) for the class to raise SCSI UNIT ATTENTION.
    pub fn take_media_changed(&mut self) -> bool {
        let f = self.media_changed_latch;
        self.media_changed_latch = false;
        f
    }

    /// Return and clear the "force a USB re-enumeration" latch, set on the same remount edge as
    /// [`take_media_changed`](Self::take_media_changed). The mechanism stays with the firmware.
    pub fn take_needs_reconnect(&mut self) -> bool {
        let f = self.reconnect_latch;
        self.reconnect_latch = false;
        f
    }

    /// Note whether the transport is inside a host command's data phase. Closing it commits a
    /// detach armed while it was open, so the medium only changes on a command boundary.
    pub fn set_data_phase(&mut self, open: bool) {
        self.data_phase_open = open;
        if !open && self.detach_pending {
            self.detach_now();
        }
    }

    /// Enter `Detached` and re-arm the remount countdown. Every `Ready -> Detached` edge goes
    /// through here, so the scheduled delay is always exactly [`MSC_RECONNECT_MS`].
    fn detach_now(&mut self) {
        self.detach_pending = false;
        self.detach_defer_ms = 0;
        self.media = MediaState::Detached;
        self.remount_in_ms = MSC_RECONNECT_MS;
    }

    /// Arm the detach a completed transfer earns. Detaching under an open data phase would cut a
    /// Bulk-Only command in half, so it waits, bounded by [`MSC_DETACH_DEFER_MS`].
    fn arm_remount(&mut self) {
        self.transfer_done_latch = true;
        if self.data_phase_open {
            self.detach_pending = true;
            self.detach_defer_ms = 0;
            return;
        }
        self.detach_now();
    }

    /// Advance timers by `elapsed_ms`, driving timeouts and the remount cycle.
    pub fn tick(&mut self, elapsed_ms: u32) -> MediaState {
        match self.media {
            MediaState::Ready => {
                if self.detach_pending {
                    // Bound the wait: a host that stopped mid-command must not strand the remount.
                    self.detach_defer_ms = self.detach_defer_ms.saturating_add(elapsed_ms);
                    if self.detach_defer_ms >= MSC_DETACH_DEFER_MS {
                        self.detach_now();
                        #[cfg(feature = "defmt")]
                        defmt::warn!("vfs detach: data phase never closed, detaching anyway");
                    }
                    return self.media;
                }
                if self.transfer.take_finished() {
                    self.arm_remount();
                    #[cfg(feature = "defmt")]
                    defmt::info!(
                        "vfs detach: transfer finished, remount in {=u32}ms",
                        self.remount_in_ms
                    );
                    return self.media;
                }
                if self.transfer.is_active() {
                    self.idle_ms = self.idle_ms.saturating_add(elapsed_ms);
                    let threshold = if self.transfer.can_finish() {
                        self.transfer.finishable_timeout_ms()
                    } else {
                        MSC_TIMEOUT_ACTIVE_MS
                    };
                    if self.idle_ms >= threshold {
                        self.transfer.on_timeout(&mut self.sink);
                        self.idle_ms = 0;
                        if self.transfer.take_finished() {
                            self.arm_remount();
                            #[cfg(feature = "defmt")]
                            defmt::info!(
                                "vfs detach: transfer finished (idle timeout), remount in {=u32}ms",
                                self.remount_in_ms
                            );
                        }
                    }
                } else if self.transfer.is_pending() {
                    // A bound drop whose data never started fails loud here; a write resets the clock.
                    self.idle_ms = self.idle_ms.saturating_add(elapsed_ms);
                    if self.idle_ms >= MSC_TIMEOUT_ACTIVE_MS {
                        self.transfer.fail_pending(&mut self.sink);
                        self.idle_ms = 0;
                        if self.transfer.take_finished() {
                            self.arm_remount();
                            #[cfg(feature = "defmt")]
                            defmt::info!(
                                "vfs detach: bound file never streamed, remount in {=u32}ms",
                                self.remount_in_ms
                            );
                        }
                    }
                } else {
                    self.idle_ms = 0;
                }
            }
            MediaState::Detached => {
                self.remount_in_ms = self.remount_in_ms.saturating_sub(elapsed_ms);
                if self.remount_in_ms == 0 {
                    self.remount();
                    self.media = MediaState::Ready;
                    // UNIT ATTENTION always; the bounce only when the sink says the activity was real.
                    self.media_changed_latch = true;
                    let reconnect = self.sink.wants_reconnect();
                    self.reconnect_latch |= reconnect;
                    #[cfg(feature = "defmt")]
                    defmt::info!(
                        "vfs remount: media changed, UNIT ATTENTION armed, reconnect={=bool}",
                        reconnect
                    );
                }
            }
        }
        self.media
    }

    /// Republish status to the files and re-snapshot after a transfer.
    fn remount(&mut self) {
        let mut buf = [0u8; 256];
        let failed = self.sink.last_failed();
        let len = {
            let mut w = StatusWriter::new(&mut buf);
            self.sink.render_status(&mut w);
            w.len()
        };
        let status = &buf[..len];
        #[cfg(feature = "defmt")]
        defmt::info!(
            "vfs remount: status=\"{=[u8]:a}\" failed={=bool}",
            status,
            failed
        );
        for f in self.files.iter_mut().take(self.n_files) {
            f.on_remount(status);
            f.on_outcome(failed);
        }
        if let Some(stamp) = self.stamp.as_mut() {
            stamp.store(status, failed);
            #[cfg(feature = "defmt")]
            defmt::info!(
                "vfs reset-stamp store: reset_count={=u32} failed={=bool}",
                stamp.reset_count(),
                failed
            );
        }
        self.snapshot();
        self.clear_shadow();
    }

    /// Recompute the contiguous cluster layout from the files' current sizes and presence.
    /// Presence is sampled here only -- at mount and at each remount -- never mid-mount.
    fn snapshot(&mut self) {
        snapshot_into(self.files, self.n_files, &mut self.snaps);
    }

    fn clear_shadow(&mut self) {
        self.shadow = [ShadowEntry {
            present: false,
            name: [0u8; 11],
            size: 0,
        }; ROOT_SLOTS];
    }

    fn build_chains(&self, out: &mut [(u32, u32); MAX_FILES]) -> usize {
        // Absent files (zero clusters) contribute no FAT chain.
        let mut n = 0;
        for snap in self.snaps.iter().take(self.n_files) {
            if snap.n_clusters > 0 {
                out[n] = (snap.first_cluster, snap.n_clusters);
                n += 1;
            }
        }
        n
    }

    /// Render logical block `lba` into `buf`.
    pub fn read_sector(&mut self, lba: u32, buf: &mut [u8; 512]) {
        if self.media == MediaState::Detached {
            *buf = [0u8; 512];
            return;
        }
        if lba == 0 {
            fat::boot_sector(buf, &self.label, self.volume_id);
        } else if lba <= 130 {
            let idx = (lba - 1) % fat::FAT_SECTORS;
            let mut ch = [(0u32, 0u32); MAX_FILES];
            let n = self.build_chains(&mut ch);
            fat::fat_sector(buf, idx, &ch[..n]);
        } else if lba <= 132 {
            self.read_root_dir(lba - 131, buf);
        } else {
            self.read_data(lba, buf);
        }
    }

    fn read_root_dir(&mut self, sec: u32, buf: &mut [u8; 512]) {
        *buf = [0u8; 512];
        // Compact after the volume label: a zeroed slot ends the directory for a FAT host.
        let mut present: [usize; MAX_FILES] = [0; MAX_FILES];
        let mut n_present = 0;
        for i in 0..self.n_files {
            if self.snaps[i].n_clusters > 0 {
                present[n_present] = i;
                n_present += 1;
            }
        }
        for i in 0..16u32 {
            let slot = sec * 16 + i;
            let off = i as usize * 32;
            if slot == 0 {
                buf[off..off + 11].copy_from_slice(&self.label);
                buf[off + 11] = FileAttrs::VOLUME_ID.0;
            } else {
                let pidx = (slot - 1) as usize;
                if pidx < n_present {
                    let fidx = present[pidx];
                    let name = *self.files[fidx].name().as_bytes();
                    let attr = self.files[fidx].attrs().0;
                    let snap = self.snaps[fidx];
                    buf[off..off + 11].copy_from_slice(&name);
                    buf[off + 11] = attr;
                    buf[off + 26..off + 28]
                        .copy_from_slice(&(snap.first_cluster as u16).to_le_bytes());
                    buf[off + 28..off + 32].copy_from_slice(&snap.size.to_le_bytes());
                }
            }
        }
    }

    fn read_data(&mut self, lba: u32, buf: &mut [u8; 512]) {
        *buf = [0u8; 512];
        let rel = lba - fat::DATA_START_LBA;
        let cluster = 2 + rel / fat::SECTORS_PER_CLUSTER;
        for i in 0..self.n_files {
            let snap = self.snaps[i];
            if cluster >= snap.first_cluster && cluster < snap.first_cluster + snap.n_clusters {
                let file_sector = rel - (snap.first_cluster - 2) * fat::SECTORS_PER_CLUSTER;
                let _ = self.files[i].read(file_sector, buf);
                let sector_start = file_sector as u64 * 512;
                let size = snap.size as u64;
                let valid = if sector_start >= size {
                    0
                } else {
                    core::cmp::min(512u64, size - sector_start) as usize
                };
                buf[valid..].fill(0);
                return;
            }
        }
    }

    /// Apply a host write to logical block `lba`.
    pub fn write_sector(&mut self, lba: u32, buf: &[u8; 512]) {
        // The medium is logically gone: the transport drains the command, but none of it applies.
        if self.media == MediaState::Detached || self.detach_pending {
            return;
        }
        self.idle_ms = 0;
        if lba <= 130 {
            // Boot-sector and FAT writes are ignored.
            #[cfg(feature = "defmt")]
            trace_reserved(lba, buf);
        } else if lba <= 132 {
            self.diff_root_dir(buf, lba - 131);
        } else {
            let _decision = self.transfer.data_sector(lba, buf, &mut self.sink);
            #[cfg(feature = "defmt")]
            {
                let mut head = [0u8; 8];
                head.copy_from_slice(&buf[..8]);
                trace_sector(lba, &head, _decision);
            }
        }
    }

    /// Route a directory event through the observer and, if passed, the transfer engine.
    fn deliver(observer: &mut O, transfer: &mut Transfer<D>, sink: &mut S, evt: FileEvent) {
        if observer.on_file_event(evt) == Disposition::Pass {
            transfer.file_event(&evt, sink);
        }
    }

    fn is_generated(&self, name: &[u8; 11]) -> bool {
        for i in 0..self.n_files {
            if self.files[i].name().as_bytes() == name {
                return true;
            }
        }
        false
    }

    fn diff_root_dir(&mut self, data: &[u8; 512], sec: u32) {
        for i in 0..16u32 {
            let slot = (sec * 16 + i) as usize;
            if slot >= ROOT_SLOTS {
                break;
            }
            let off = i as usize * 32;
            let entry = &data[off..off + 32];
            let first = entry[0];
            let attr = entry[11];
            let is_lfn = attr & 0x0F == 0x0F;
            let is_vol = attr & 0x08 != 0;
            let present = first != 0x00 && first != 0xE5 && !is_lfn && !is_vol;

            if present {
                let mut name = [0u8; 11];
                name.copy_from_slice(&entry[0..11]);
                let size = u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]);
                let first_cluster = u16::from_le_bytes([entry[26], entry[27]]) as u32;

                let generated = self.is_generated(&name);
                let hidden =
                    (attr & FileAttrs::HIDDEN.0) != 0 || (name[0] == b'.' && name[1] == b'_');
                let prev = self.shadow[slot];
                #[cfg(feature = "defmt")]
                {
                    let created = !prev.present;
                    let renamed = prev.present && prev.name != name;
                    let resized = prev.present && prev.name == name && prev.size != size;
                    if created || renamed || resized {
                        let mut flags = 0u8;
                        if hidden {
                            flags |= FLAG_HIDDEN;
                        }
                        if generated {
                            flags |= FLAG_GENERATED;
                        }
                        if resized {
                            flags |= FLAG_CHANGED;
                        }
                        defmt::info!(
                            "vfs dirent name={=[u8]:a} clu={=u16} sz={=u32} flags={=u8:08b}",
                            &name[..],
                            first_cluster as u16,
                            size,
                            flags
                        );
                    }
                }

                if generated {
                    self.shadow[slot] = ShadowEntry {
                        present: true,
                        name,
                        size,
                    };
                    continue;
                }

                if !prev.present {
                    Self::deliver(
                        &mut self.observer,
                        &mut self.transfer,
                        &mut self.sink,
                        FileEvent::Created {
                            name: &name,
                            size,
                            hidden,
                            first_cluster,
                        },
                    );
                } else if prev.name != name {
                    Self::deliver(
                        &mut self.observer,
                        &mut self.transfer,
                        &mut self.sink,
                        FileEvent::Deleted { name: &prev.name },
                    );
                    Self::deliver(
                        &mut self.observer,
                        &mut self.transfer,
                        &mut self.sink,
                        FileEvent::Created {
                            name: &name,
                            size,
                            hidden,
                            first_cluster,
                        },
                    );
                } else if prev.size != size && !hidden {
                    // A hidden entry's close must not rebind an in-flight transfer.
                    Self::deliver(
                        &mut self.observer,
                        &mut self.transfer,
                        &mut self.sink,
                        FileEvent::Changed {
                            name: &name,
                            size,
                            first_cluster,
                        },
                    );
                }
                self.shadow[slot] = ShadowEntry {
                    present: true,
                    name,
                    size,
                };
            } else {
                let prev = self.shadow[slot];
                if prev.present {
                    Self::deliver(
                        &mut self.observer,
                        &mut self.transfer,
                        &mut self.sink,
                        FileEvent::Deleted { name: &prev.name },
                    );
                }
                self.shadow[slot] = ShadowEntry {
                    present: false,
                    name: [0u8; 11],
                    size: 0,
                };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    struct DummyFile {
        name: FatShortName,
        size: u32,
        fill: u8,
    }
    impl DummyFile {
        fn new(n: &str, size: u32, fill: u8) -> Self {
            Self {
                name: FatShortName::new(n).unwrap(),
                size,
                fill,
            }
        }
    }
    impl VfsFile for DummyFile {
        fn name(&self) -> &FatShortName {
            &self.name
        }
        fn size(&self) -> u32 {
            self.size
        }
        fn read(&mut self, sector_offset: u32, buf: &mut [u8]) -> usize {
            let start = sector_offset as usize * 512;
            if start >= self.size as usize {
                return 0;
            }
            let n = core::cmp::min(buf.len(), self.size as usize - start);
            for b in buf[..n].iter_mut() {
                *b = self.fill;
            }
            n
        }
    }

    /// Build a root-directory entry for a dropped file.
    fn dir_entry(name: &[u8; 11], attr: u8, first_cluster: u16, size: u32) -> [u8; 32] {
        let mut e = [0u8; 32];
        e[0..11].copy_from_slice(name);
        e[11] = attr;
        e[26..28].copy_from_slice(&first_cluster.to_le_bytes());
        e[28..32].copy_from_slice(&size.to_le_bytes());
        e
    }

    #[test]
    fn volume_label_entry_zero() {
        let mut f0 = DummyFile::new("A.BIN", 10, 0xAB);
        let mut files: [&mut dyn VfsFile; 1] = [&mut f0];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        let mut buf = [0u8; 512];
        vfs.read_sector(131, &mut buf);
        assert_eq!(&buf[0..11], b"DAPLINK    ");
        assert_eq!(buf[11], FileAttrs::VOLUME_ID.0);
    }

    #[test]
    fn two_files_disjoint_chains_and_extract() {
        let mut f0 = DummyFile::new("A.BIN", 4096 + 100, 0x11); // 2 clusters
        let mut f1 = DummyFile::new("B.BIN", 10, 0x22); // 1 cluster
        let mut files: [&mut dyn VfsFile; 2] = [&mut f0, &mut f1];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);

        // Root-dir entries point at clusters 2 and 4.
        let mut buf = [0u8; 512];
        vfs.read_sector(131, &mut buf);
        let e1 = &buf[32..64];
        assert_eq!(&e1[0..11], b"A       BIN");
        assert_eq!(u16::from_le_bytes([e1[26], e1[27]]), 2);
        assert_eq!(u32::from_le_bytes([e1[28], e1[29], e1[30], e1[31]]), 4196);
        let e2 = &buf[64..96];
        assert_eq!(u16::from_le_bytes([e2[26], e2[27]]), 4);

        // FAT: cluster 2 -> 3 -> EOC, cluster 4 -> EOC.
        vfs.read_sector(1, &mut buf);
        assert_eq!(u16::from_le_bytes([buf[4], buf[5]]), 3);
        assert_eq!(u16::from_le_bytes([buf[6], buf[7]]), 0xFFFF);
        assert_eq!(u16::from_le_bytes([buf[8], buf[9]]), 0xFFFF);

        // Data of file B (cluster 4 -> LBA 133 + 2*8 = 149), zero-filled past its 10 bytes.
        vfs.read_sector(fat::cluster_to_lba(4), &mut buf);
        assert_eq!(&buf[0..10], &[0x22u8; 10]);
        assert_eq!(buf[10], 0);
    }

    #[test]
    fn zero_byte_file_single_cluster() {
        let mut f0 = DummyFile::new("EMPTY.BIN", 0, 0);
        let mut files: [&mut dyn VfsFile; 1] = [&mut f0];
        let vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        assert_eq!(vfs.snaps[0].n_clusters, 1);
        assert_eq!(vfs.snaps[0].first_cluster, 2);
    }

    /// One well-formed 512-byte UF2 block carrying `payload` at `addr`.
    fn uf2_block(addr: u32, payload: &[u8]) -> [u8; 512] {
        let mut b = [0u8; 512];
        b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        b[12..16].copy_from_slice(&addr.to_le_bytes());
        b[16..20].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        b[32..32 + payload.len()].copy_from_slice(payload);
        b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        b
    }

    /// Drive a real, sized transfer to completion so a following `tick` detaches the volume.
    fn drop_sized_image_and_detach<S: TransferSink, O: VfsObserver>(vfs: &mut Vfs<'_, S, O>) {
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
        vfs.write_sector(131, &root);
        vfs.write_sector(fat::DATA_START_LBA, &uf2_block(0x0800_0000, &[1, 2, 3, 4]));
        assert_eq!(vfs.tick(1), MediaState::Detached);
    }

    #[test]
    fn remount_latches_media_changed_once() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        // A fresh mount (starts Ready) never signals a media change.
        assert!(!vfs.take_media_changed());
        // A real completed transfer detaches the volume; no latch while detached.
        drop_sized_image_and_detach(&mut vfs);
        assert!(!vfs.take_media_changed());
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_media_changed(), "remount raises the latch");
        assert!(!vfs.take_media_changed(), "the latch is cleared on read");
    }

    /// The same sized drop, but entirely inside an open data phase.
    fn drop_sized_image_inside_a_data_phase<S: TransferSink, O: VfsObserver>(
        vfs: &mut Vfs<'_, S, O>,
    ) {
        vfs.set_data_phase(true);
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
        vfs.write_sector(131, &root);
        vfs.write_sector(fat::DATA_START_LBA, &uf2_block(0x0800_0000, &[1, 2, 3, 4]));
    }

    /// A transfer completing mid-command must not take the medium away under the transport.
    #[test]
    fn detach_waits_for_the_open_data_phase() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drop_sized_image_inside_a_data_phase(&mut vfs);
        assert_eq!(vfs.tick(1), MediaState::Ready, "the medium stays present");
        assert_eq!(vfs.tick(1), MediaState::Ready);
        // A uf2 block trailing a finished drop must not open a second transfer.
        let mut block = [0u8; 512];
        block[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes());
        block[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes());
        vfs.write_sector(fat::DATA_START_LBA + 1, &block);
        assert!(!vfs.is_transfer_active());
        vfs.set_data_phase(false);
        assert_eq!(vfs.media(), MediaState::Detached);
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(
            vfs.take_media_changed(),
            "the remount is deferred, not lost"
        );
    }

    /// The deferral is bounded: a phase that never closes must still yield the remount.
    #[test]
    fn a_data_phase_that_never_closes_still_detaches() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drop_sized_image_inside_a_data_phase(&mut vfs);
        // The first tick after the completion arms the detach; the grace runs from there.
        assert_eq!(vfs.tick(1), MediaState::Ready);
        assert_eq!(vfs.tick(MSC_DETACH_DEFER_MS - 1), MediaState::Ready);
        assert_eq!(vfs.tick(1), MediaState::Detached, "the grace expires");
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_media_changed());
    }

    /// A sink that opts in to the bus bounce on every remount (the trait default is `false`).
    struct ReconnectSink;
    impl TransferSink for ReconnectSink {
        fn begin(&mut self, _fmt: daplink_decoder::Format, _sniffed: bool) {}
        fn fragment(&mut self, _addr: u32, _data: &[u8]) -> Result<(), DecodeError> {
            Ok(())
        }
        fn end(&mut self, _outcome: TransferOutcome) {}
        fn wants_reconnect(&self) -> bool {
            true
        }
    }

    #[test]
    fn remount_latches_needs_reconnect_once_done_path() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            ReconnectSink,
            NoObserver,
        );
        // A fresh mount (starts Ready) never requests a reconnect.
        assert!(!vfs.take_needs_reconnect());
        // A completed transfer detaches the volume; no reconnect while detached.
        drop_sized_image_and_detach(&mut vfs);
        assert!(!vfs.take_needs_reconnect());
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_needs_reconnect(), "remount requests a reconnect");
        assert!(!vfs.take_needs_reconnect(), "the latch is cleared on read");
        assert!(vfs.take_media_changed(), "UA latch armed independently");
    }

    #[test]
    fn remount_latches_needs_reconnect_once_failure_path() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            ReconnectSink,
            NoObserver,
        );
        // A sized stream short of its size, with a consumed LBA re-written: OutOfOrder at timeout.
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 2048));
        vfs.write_sector(131, &root);
        let sector = uf2_block(0x0800_0000, &[1, 2, 3, 4]);
        vfs.write_sector(fat::DATA_START_LBA, &sector);
        assert!(vfs.is_transfer_active());
        vfs.write_sector(fat::DATA_START_LBA, &sector);
        assert!(
            vfs.is_transfer_active(),
            "the rewrite is recorded, not an eager abort"
        );
        assert_eq!(vfs.tick(MSC_TIMEOUT_ACTIVE_MS), MediaState::Detached);
        assert!(!vfs.is_transfer_active(), "the timeout fails the transfer");
        assert!(!vfs.take_needs_reconnect());
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(
            vfs.take_needs_reconnect(),
            "a failed transfer's remount also requests a reconnect"
        );
        assert!(!vfs.take_needs_reconnect(), "the latch is cleared on read");
    }

    /// A sink on the default `wants_reconnect` remounts via UNIT ATTENTION alone.
    #[test]
    fn default_sink_remount_never_reconnects() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drop_sized_image_and_detach(&mut vfs);
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_media_changed(), "UA still republishes the remount");
        assert!(
            !vfs.take_needs_reconnect(),
            "a sink on the default hook must never bounce the bus"
        );
    }

    /// A bound drop whose data never starts must fail loud, with no session and no bus bounce.
    #[test]
    fn bound_never_streamed_fails_loud_and_remounts_quietly() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(LoopBackend::default()),
            NoObserver,
        );
        // A visible APP.UF2 dirent binds the format, no cluster allocated yet.
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 0, 4096));
        vfs.write_sector(131, &root);
        // Every data sector fails the content guard: nothing ever starts.
        vfs.write_sector(fat::DATA_START_LBA, &[0u8; 512]);
        assert!(!vfs.is_transfer_active());
        assert_eq!(
            vfs.transfer_timeout(),
            MSC_TIMEOUT_ACTIVE_MS,
            "a bound, never-started drop is under the active timeout"
        );
        assert_eq!(vfs.tick(MSC_TIMEOUT_ACTIVE_MS - 1), MediaState::Ready);
        assert_eq!(vfs.tick(1), MediaState::Detached);
        assert!(vfs.take_transfer_finished());
        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::NoData));
        assert!(vfs.sink().last_failed(), "the status will name the failure");
        assert_eq!(vfs.sink().backend().begun, 0, "no flash session ever opens");
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_media_changed());
        assert!(
            !vfs.take_needs_reconnect(),
            "a never-started drop must not bounce the bus"
        );
    }

    #[test]
    fn detached_reads_zero_and_writes_drop() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        drop_sized_image_and_detach(&mut vfs);
        assert_eq!(vfs.media(), MediaState::Detached);
        let mut buf = [0xFFu8; 512];
        vfs.read_sector(0, &mut buf);
        assert_eq!(buf, [0u8; 512]);
        // A write while detached is dropped: no panic, no effect.
        vfs.write_sector(fat::DATA_START_LBA, &[0u8; 512]);
    }

    #[test]
    fn rootdir_size_grows_via_changed() {
        struct CapObs {
            events: Vec<(u32, bool)>, // (size, is_changed)
        }
        impl VfsObserver for CapObs {
            fn on_file_event(&mut self, evt: FileEvent<'_>) -> Disposition {
                match evt {
                    FileEvent::Created { size, .. } => self.events.push((size, false)),
                    FileEvent::Changed { size, .. } => self.events.push((size, true)),
                    FileEvent::Deleted { .. } => {}
                }
                Disposition::Pass
            }
        }
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            NullSink,
            CapObs { events: Vec::new() },
        );

        let mut sector = [0u8; 512];
        let e = dir_entry(b"APP     UF2", 0x20, 2, 100);
        sector[0..32].copy_from_slice(&e);
        vfs.write_sector(131, &sector);

        let e2 = dir_entry(b"APP     UF2", 0x20, 2, 4096);
        sector[0..32].copy_from_slice(&e2);
        vfs.write_sector(131, &sector);

        let ev = &vfs.observer.events;
        assert_eq!(ev[0], (100, false));
        assert_eq!(ev[1], (4096, true));
    }

    #[test]
    fn observer_handled_suppresses_binding() {
        struct Block;
        impl VfsObserver for Block {
            fn on_file_event(&mut self, _evt: FileEvent<'_>) -> Disposition {
                Disposition::Handled
            }
        }
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, Block);
        let mut sector = [0u8; 512];
        let e = dir_entry(b"APP     UF2", 0x20, 2, 512);
        sector[0..32].copy_from_slice(&e);
        vfs.write_sector(131, &sector);
        // A magicless sector cannot self-detect, so nothing starts.
        vfs.write_sector(133, &[0x00u8; 512]);
        assert!(!vfs.is_transfer_active());
    }

    #[test]
    fn macos_sidecar_ignored() {
        struct CapObs {
            hidden_flags: Vec<bool>,
        }
        impl VfsObserver for CapObs {
            fn on_file_event(&mut self, evt: FileEvent<'_>) -> Disposition {
                if let FileEvent::Created { hidden, .. } = evt {
                    self.hidden_flags.push(hidden);
                }
                Disposition::Pass
            }
        }
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            NullSink,
            CapObs {
                hidden_flags: Vec::new(),
            },
        );
        let mut sector = [0u8; 512];
        let e = dir_entry(b"._APP   BIN", 0x20, 2, 512);
        sector[0..32].copy_from_slice(&e);
        vfs.write_sector(131, &sector);
        assert_eq!(vfs.observer.hidden_flags, [true]);
        assert_eq!(vfs.transfer_timeout(), 0);
    }

    /// Minimal flash backend: counts sessions, and can model an empty wire (`fail_begin`).
    #[derive(Default)]
    struct LoopBackend {
        fail_begin: bool,
        begun: u32,
        /// Bytes this backend was asked to program, in order.
        programmed: u32,
    }
    impl daplink_core::FlashBackend for LoopBackend {
        fn begin(&mut self) -> Result<(), FlashPhase> {
            if self.fail_begin {
                return Err(FlashPhase::NoTarget);
            }
            self.begun += 1;
            Ok(())
        }
        fn program(&mut self, _addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
            self.programmed += data.len() as u32;
            Ok(())
        }
        fn finish(&mut self, _success: bool) -> Result<(), FlashPhase> {
            Ok(())
        }
        fn finish_with_reset(&mut self, _success: bool, _reset: bool) -> Result<(), FlashPhase> {
            Ok(())
        }
        fn flash_ranges(&self) -> &[(u32, u32)] {
            &[(0x0800_0000, 0x0801_0000)]
        }
    }

    /// A sector of compressed-stream-like host housekeeping content.
    fn housekeeping_sector() -> [u8; 512] {
        let mut s = [0u8; 512];
        for (i, chunk) in s.chunks_mut(4).enumerate() {
            let w = 0x9E37_79B9u32.wrapping_mul(i as u32 + 1) | 0x4000_0000;
            chunk.copy_from_slice(&w.to_le_bytes());
        }
        s
    }

    /// Mount-time host housekeeping must start no transfer and arm no reconnect, or its remount
    /// would produce the next mount's housekeeping forever.
    #[test]
    fn mount_time_housekeeping_never_reconnects() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(LoopBackend::default()),
            NoObserver,
        );
        for cycle in 0..3 {
            // 8.3 generation strips the sidecar's dot, so only its hidden attribute can refuse it.
            let mut root = [0u8; 512];
            root[0..32].copy_from_slice(&dir_entry(b"FSEVEN~1   ", 0x12, 3, 0));
            root[32..64].copy_from_slice(&dir_entry(b"SPOTLI~1   ", 0x12, 4, 0));
            root[64..96].copy_from_slice(&dir_entry(b"._STUFF UF2", 0x22, 5, 4096));
            root[96..128].copy_from_slice(&dir_entry(b"_STUFF~1UF2", 0x22, 0, 0));
            vfs.write_sector(131, &root);
            for i in 0..8u32 {
                vfs.write_sector(fat::DATA_START_LBA + 8 + i, &housekeeping_sector());
                assert!(
                    !vfs.is_transfer_active(),
                    "housekeeping content must not start a transfer (cycle {cycle}, sector {i})"
                );
            }
            for i in 0..2u32 {
                vfs.write_sector(fat::cluster_to_lba(6) + i, &housekeeping_sector());
                assert!(
                    !vfs.is_transfer_active(),
                    "sidecar content must not start a transfer (cycle {cycle}, sector {i})"
                );
            }
            // The sidecar's close resizes its entry; the hidden attribute must stop it binding.
            root[96..128].copy_from_slice(&dir_entry(b"_STUFF~1UF2", 0x22, 6, 1024));
            vfs.write_sector(131, &root);
            assert!(!vfs.is_transfer_active(), "sidecar close must not bind");
            // A wrongly-bound sidecar would fail loud as NoData here and detach the volume.
            for _ in 0..30 {
                assert_eq!(vfs.tick(1000), MediaState::Ready);
            }
            assert!(
                !vfs.take_needs_reconnect(),
                "housekeeping writes must never arm the reconnect latch (cycle {cycle})"
            );
            assert_eq!(vfs.sink().backend().begun, 0, "no flash session ever opens");
        }
    }

    /// A real drop rejected before a session opens must still bounce the bus: bound-ness, not
    /// the session, is what distinguishes a user drop from housekeeping.
    #[test]
    fn bound_drop_rejected_before_session_reconnects() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(LoopBackend {
                fail_begin: true,
                ..LoopBackend::default()
            }),
            NoObserver,
        );
        // A visible APP.UF2 dirent binds the drop, then its valid data streams.
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
        vfs.write_sector(131, &root);
        let mut block = [0u8; 512];
        block[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        block[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        block[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        block[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        block[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        vfs.write_sector(fat::DATA_START_LBA, &block);
        assert_eq!(vfs.tick(1), MediaState::Detached);
        assert!(vfs.take_transfer_finished());
        assert!(
            vfs.sink().last_failed(),
            "the status will name the rejection"
        );
        assert_eq!(vfs.sink().backend().begun, 0, "no session ever opened");
        // The remount must arm both latches: UNIT ATTENTION and the bus bounce.
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_media_changed());
        assert!(
            vfs.take_needs_reconnect(),
            "a bound drop's pre-session rejection must bounce the bus"
        );
    }

    /// An unbound sniffed transfer that never opens a session must remount quietly.
    #[test]
    fn sniffed_transfer_without_session_remounts_quietly() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(LoopBackend {
                fail_begin: true,
                ..LoopBackend::default()
            }),
            NoObserver,
        );
        // An unbound sector carrying the uf2 block magic starts a transfer on content alone.
        let mut sector = [0u8; 512];
        sector[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        sector[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        sector[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        sector[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        sector[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        vfs.write_sector(fat::DATA_START_LBA, &sector);
        assert!(vfs.is_transfer_active());
        // Never dirent-bound, so finishable but unsized: the longer idle timeout completes it.
        assert_eq!(
            vfs.tick(MSC_TIMEOUT_FINISHABLE_UNSIZED_MS),
            MediaState::Detached
        );
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_media_changed(), "UA latch still arms on remount");
        assert!(
            !vfs.take_needs_reconnect(),
            "a transfer that never opened a flash session must not bounce the bus"
        );
    }

    /// A drop whose host writes every data cluster before the directory entry: the payload
    /// starts the transfer on its magic, and the late entry completes it.
    #[test]
    fn data_first_drop_programs() {
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> = Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(LoopBackend::default()),
            NoObserver,
        );
        for i in 0..4u32 {
            let block = uf2_block(0x0800_0000 + i * 256, &[0xA5u8; 256]);
            vfs.write_sector(fat::DATA_START_LBA + i, &block);
            assert!(vfs.is_transfer_active(), "the payload started the transfer");
        }
        // Only now does the host write the directory entry, with the final size.
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 4 * 512));
        vfs.write_sector(131, &root);
        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(
            vfs.sink().backend().programmed,
            4 * 256,
            "the whole image reached the target, not just the block that started it"
        );
        assert_eq!(vfs.sink().backend().begun, 1);
        assert_eq!(vfs.tick(1), MediaState::Detached);
        assert_eq!(vfs.tick(MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_needs_reconnect());
    }
}
