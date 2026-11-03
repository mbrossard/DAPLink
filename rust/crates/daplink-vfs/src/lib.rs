#![no_std]
#![deny(missing_docs)]
#![cfg_attr(not(test), deny(clippy::unwrap_used))]
//! A virtual FAT16 volume for drag-n-drop: host sector writes are decoded and streamed to a sink.

#[cfg(test)]
extern crate std;

#[cfg(test)]
#[path = "../tests/common/mod.rs"]
mod fixtures;

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
mod vfs;
pub mod volume;

pub use decoder_set::{AllDecoders, DecoderSet, Either};
pub use files::{DetailsFile, DeviceInfo};
pub use files::{FatShortName, FileAttrs, VfsFile};
pub use flash_sink::FlashSink;
pub use msc_bot::{run_command, BulkIn, BulkOut, DeadlineSleep, MsClock, YieldSleep};
pub use msc_io::{msc_worker_step, DirectIo, QueuedIo, SectorIo, SectorPoll, WorkerStep};
pub use render::StatusWriter;
pub use stamp::BootSnapshot;
pub use static_cell::{ConstStaticCell, StaticCell};
pub use transfer::StatsSink;
pub use transfer::{
    DecodeError, Disposition, FileEvent, FlashPhase, NoObserver, NullSink, SectorDecision,
    TransferOutcome, TransferSink, VfsObserver,
};
pub use vendor_flash::{VendorFlash, VendorFlashStream};
pub use vfs::{MediaState, Vfs, VfsConfig};
pub use volume::{assemble, assemble_info, FlashVolume, VolumeStorage};

/// Maximum number of files the volume can expose.
pub const MAX_FILES: usize = 8;

/// Idle timeout, in ms, while a transfer is actively receiving data.
pub const MSC_TIMEOUT_ACTIVE_MS: u32 = 20_000;
/// Idle timeout, in ms, for a sized transfer that discarded a sector ahead of its front.
pub const MSC_TIMEOUT_GAPPED_MS: u32 = 1_000;
const _: () = assert!(MSC_TIMEOUT_GAPPED_MS < MSC_TIMEOUT_ACTIVE_MS);
/// Idle timeout, in ms, once a transfer could finish and a directory-declared size has been met.
pub const MSC_TIMEOUT_FINISHABLE_MS: u32 = 500;
/// Idle timeout, in ms, once an unsized transfer could finish; idle cannot tell done from paused.
pub const MSC_TIMEOUT_FINISHABLE_UNSIZED_MS: u32 = 5_000;
/// Time, in ms, the volume stays detached before remounting.
pub const MSC_RECONNECT_MS: u32 = 2_500;

/// Uptime that forgives the fault streak, above the slowest attach to a mounted disk seen on Linux.
pub const FAULT_STREAK_CLEAR_MS: u32 = 15_000;
/// Upper bound, in ms, on how long an armed detach waits for the transport's data phase to close.
pub const MSC_DETACH_DEFER_MS: u32 = 1_000;

/// The FAT volume id for the probe carrying `serial`, or the default when it is not hex.
pub fn volume_id_from_serial(serial: &str) -> u32 {
    daplink_core::serial::volume_id(serial).unwrap_or(fat::DEFAULT_VOLUME_ID)
}
