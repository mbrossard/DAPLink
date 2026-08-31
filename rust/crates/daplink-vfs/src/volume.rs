//! One-call assembly of the drag-n-drop flash volume from caller-owned static storage.

use crate::{
    AllDecoders, ConstStaticCell, DetailsFile, DeviceInfo, NoObserver, StaticCell, TransferSink,
    Vfs, VfsConfig, VfsFile,
};

/// The concrete volume type [`assemble`] produces: a one-file volume (`DETAILS.TXT`) with the
/// default decoder set and no directory observer, over the firmware-owned sink `S`.
pub type FlashVolume<S> = Vfs<'static, S, NoObserver, AllDecoders>;

/// Caller-owned static storage for a flash volume: the one-shot [`StaticCell`]s [`assemble`]
/// initializes -- the `DETAILS.TXT` body, the file table, and the volume itself.
pub struct VolumeStorage<S: TransferSink + 'static> {
    /// A `ConstStaticCell`, so the 512-byte body starts in `.bss` already sized and is baked in
    /// place; a `StaticCell` would render a sector in the caller's init frame and copy it in.
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

/// Assemble the flash volume into caller-owned `storage`, returning the initialized volume.
/// Each cell initializes exactly once, so call this at most once per `storage`.
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

/// [`assemble`] with a full [`DeviceInfo`] and an optional reset-surviving [`ResetStamp`], whose
/// carried status seeds `DETAILS.TXT` before the first mount.
pub fn assemble_info<S: TransferSink + 'static>(
    storage: &'static VolumeStorage<S>,
    sink: S,
    info: DeviceInfo<'static>,
    stamp: Option<&'static mut crate::ResetStamp>,
    label: &'static str,
) -> &'static mut FlashVolume<S> {
    // Per-probe, so two drives attached at once are distinct volumes to the host.
    let volume_id = crate::volume_id_from_serial(info.serial);
    // `init_with`, never `init`: `init` moves its value, building each body in this frame.
    let details: &'static mut DetailsFile<'static> = storage.details.take();
    details.bake(info);
    if let Some(stamp) = &stamp {
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
    if let Some(stamp) = stamp {
        vfs.set_reset_stamp(stamp);
    }
    vfs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{fat, MediaState, StatsSink, TransferOutcome};
    use daplink_decoder::Format;

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

        // A tick then detaches the volume for the remount cycle.
        assert_eq!(vfs.tick(1), MediaState::Detached);
    }
}
