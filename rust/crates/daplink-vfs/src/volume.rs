//! One-call assembly of the drag-n-drop flash volume from caller-owned static storage.

use crate::{
    AllDecoders, ConstStaticCell, DetailsFile, DeviceInfo, NoObserver, StaticCell, TransferSink,
    Vfs, VfsConfig, VfsFile,
};

/// A one-file (`DETAILS.TXT`) volume with the default decoders and no observer, over sink `S`.
pub type FlashVolume<S> = Vfs<'static, S, NoObserver, AllDecoders>;

/// Caller-owned static storage for a flash volume.
pub struct VolumeStorage<S: TransferSink + 'static> {
    /// Const-initialized so the 512-byte body is baked in `.bss`, not copied from the init frame.
    details: ConstStaticCell<DetailsFile<'static>>,
    files: StaticCell<[&'static mut dyn VfsFile; 1]>,
    vfs: StaticCell<FlashVolume<S>>,
}

impl<S: TransferSink + 'static> VolumeStorage<S> {
    /// Create empty, uninitialized storage.
    pub const fn new() -> Self {
        Self {
            details: ConstStaticCell::new(DetailsFile::BLANK),
            files: StaticCell::new(),
            vfs: StaticCell::new(),
        }
    }
}

impl<S: TransferSink + 'static> Default for VolumeStorage<S> {
    fn default() -> Self {
        Self::new()
    }
}

/// Assemble the flash volume into `storage`; call at most once per `storage`.
pub fn assemble<S: TransferSink + 'static>(
    storage: &'static VolumeStorage<S>,
    sink: S,
    version: &'static str,
    serial: &'static str,
    label: &'static str,
) -> &'static mut FlashVolume<S> {
    assemble_info(
        storage,
        sink,
        DeviceInfo {
            version,
            serial,
            reset_count: 0,
        },
        None,
        label,
    )
}

/// [`assemble`] with an owned boot report and its independent retained-status writer.
pub fn assemble_info<S: TransferSink + 'static>(
    storage: &'static VolumeStorage<S>,
    sink: S,
    info: DeviceInfo<'static>,
    stamp: Option<(
        crate::BootSnapshot,
        crate::stamp::StatusWriter<'static>,
        crate::stamp::SteadyToken<'static>,
    )>,
    label: &'static str,
) -> &'static mut FlashVolume<S> {
    // Per-probe, so two drives attached at once are distinct volumes to the host.
    let volume_id = crate::volume_id_from_serial(info.serial);
    // `init_with`, never `init`: `init` moves its value, building each body in this frame.
    let details: &'static mut DetailsFile<'static> = storage.details.take();
    details.bake(info);
    if let Some((stamp, _, _)) = &stamp {
        if let Some((status, _failed)) = stamp.status() {
            details.seed_status(status);
        }
        if let Some(reason) = stamp.fault() {
            details.seed_fault(reason);
        }
    }
    let files: &'static mut [&'static mut dyn VfsFile; 1] = storage.files.init_with(|| [details]);
    let files: &'static mut [&'static mut dyn VfsFile] = &mut files[..];
    let vfs = storage.vfs.init_with(move || {
        Vfs::new(
            VfsConfig {
                volume_label: label,
                volume_id,
            },
            files,
            sink,
            NoObserver,
        )
    });
    if let Some((_, stamp, steady)) = stamp {
        vfs.set_reset_stamp(stamp);
        vfs.set_steady_token(steady);
    }
    vfs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixtures::dir_entry;
    use crate::{fat, MediaState, StatsSink, TransferOutcome};
    use daplink_decoder::Format;

    #[test]
    fn assemble_seeds_details_from_owned_boot_snapshot() {
        static VOLUME: VolumeStorage<StatsSink> = VolumeStorage::new();
        let mut bytes = [0; crate::stamp::RETAINED_SIZE];
        bytes[..4].copy_from_slice(&0x6472_7375u32.to_le_bytes());
        bytes[12..16].copy_from_slice(&3u32.to_le_bytes());
        bytes[16..19].copy_from_slice(b"old");
        bytes[84..88].copy_from_slice(&5u32.to_le_bytes());
        bytes[88..93].copy_from_slice(b"panic");
        let fixture = std::boxed::Box::leak(std::boxed::Box::new(
            crate::stamp::InitializedStamp::from_bytes(bytes),
        ));
        let mut boot = fixture.boot();
        boot.status.store(b"new", false);
        let vfs = assemble_info(
            &VOLUME,
            StatsSink::new(),
            DeviceInfo {
                version: "1",
                serial: "1234",
                reset_count: boot.snapshot.reset_count(),
            },
            Some((boot.snapshot, boot.status, boot.steady)),
            "DAPLINK",
        );
        let mut sector = [0; 512];
        vfs.read_sector(fat::DATA_START_LBA, &mut sector);
        let text = core::str::from_utf8(&sector).unwrap();
        assert!(text.contains("Status: old"));
        assert!(text.contains("Fault: panic"));
        assert!(text.contains("Reset count: 00000001"));
    }

    #[test]
    fn assemble_builds_volume_and_streams_transfer() {
        static VOLUME: VolumeStorage<StatsSink> = VolumeStorage::new();
        let vfs = assemble(&VOLUME, StatsSink::new(), "9.9.9", "CAFEBABE", "DAPLINK");

        // DETAILS.TXT is file 0, so it renders at the first data sector.
        let mut buf = [0u8; 512];
        vfs.read_sector(131, &mut buf);
        assert_eq!(&buf[0..11], b"DAPLINK    ");

        // The boot sector's volume id is this probe's, not the shared constant.
        let mut boot = [0u8; 512];
        vfs.read_sector(0, &mut boot);
        assert_eq!(
            u32::from_le_bytes([boot[39], boot[40], boot[41], boot[42]]),
            0xCAFE_BABE
        );
        assert_ne!(
            u32::from_le_bytes([boot[39], boot[40], boot[41], boot[42]]),
            fat::DEFAULT_VOLUME_ID
        );
        vfs.read_sector(fat::DATA_START_LBA, &mut buf);
        let s = core::str::from_utf8(&buf).unwrap();
        assert!(s.contains("Interface Version: 9.9.9"));
        assert!(s.contains("Unique ID: CAFEBABE"));

        // A one-block transfer end to end: the dirent binds APP.UF2, then the data streams.
        let mut root = [0u8; 512];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, 512));
        vfs.write_sector(131, &root);

        let mut sector = [0u8; 512];
        sector[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        sector[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        sector[12..16].copy_from_slice(&0x1000_0000u32.to_le_bytes()); // target addr
        sector[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        sector[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        vfs.write_sector(fat::DATA_START_LBA, &sector);

        // The sink recorded the decoded payload.
        let stats = vfs.sink();
        assert_eq!(stats.outcome(), Some(TransferOutcome::Done));
        assert_eq!(stats.fmt, Some(Format::Uf2));
        assert_eq!(stats.stats.bytes, 4);

        // Once the host is quiet the volume detaches for the remount cycle.
        assert_eq!(
            vfs.tick(crate::MSC_TIMEOUT_FINISHABLE_MS),
            MediaState::Detached
        );
    }
}
