//! The volume as a real FAT checker reads it, in every state a host can see.

mod common;

use common::{dir_entry, uf2_numbered};
use daplink_core::{FlashBackend, FlashPhase};
use daplink_vfs::{
    fat, DetailsFile, DeviceInfo, FlashSink, MediaState, NoObserver, Vfs, VfsConfig, VfsFile,
    MSC_RECONNECT_MS, MSC_TIMEOUT_ACTIVE_MS,
};
use std::path::Path;
use std::process::Command;

const BASE: u32 = 0x1000_0000;

struct Backend;
impl FlashBackend for Backend {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        Ok(())
    }
    fn program(&mut self, _a: u32, _d: &[u8]) -> Result<(), FlashPhase> {
        Ok(())
    }
    fn finish(&mut self, _s: bool) -> Result<(), FlashPhase> {
        Ok(())
    }
    fn finish_with_reset(&mut self, _s: bool, _r: bool) -> Result<(), FlashPhase> {
        Ok(())
    }
    fn flash_ranges(&self) -> &[(u32, u32)] {
        &[(BASE, BASE + 0x20_0000)]
    }
    fn selected_target_name(&self) -> Option<&str> {
        Some("RP2040")
    }
}

type V<'a> = Vfs<'a, FlashSink<Backend>, NoObserver>;

fn dump(vfs: &mut V<'_>) -> Vec<u8> {
    let mut img = vec![0u8; fat::TOTAL_SECTORS as usize * 512];
    for lba in 0..fat::TOTAL_SECTORS {
        let mut s = [0u8; 512];
        vfs.read_sector(lba, &mut s);
        img[lba as usize * 512..(lba as usize + 1) * 512].copy_from_slice(&s);
    }
    img
}

/// `fsck.vfat -n` on the dumped image: `None` when no checker is installed.
fn fsck(path: &Path) -> Option<(i32, String)> {
    let out = Command::new("fsck.vfat")
        .args(["-n", "-v"])
        .arg(path)
        .output()
        .or_else(|_| {
            Command::new("/usr/sbin/fsck.vfat")
                .args(["-n", "-v"])
                .arg(path)
                .output()
        })
        .ok()?;
    let mut text = String::from_utf8_lossy(&out.stdout).into_owned();
    text.push_str(&String::from_utf8_lossy(&out.stderr));
    Some((out.status.code().unwrap_or(-1), text))
}

fn check(state: &str, vfs: &mut V<'_>) {
    let img = dump(vfs);
    let path = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{state}.img"));
    std::fs::write(&path, &img).unwrap();
    let checked = fsck(&path);
    std::fs::remove_file(&path).unwrap();
    let Some((code, text)) = checked else {
        eprintln!("fsck.vfat absent: {state} not checked");
        return;
    };
    assert_eq!(code, 0, "{state}: {text}");
}

fn root(name: &[u8; 11], size: u32) -> [u8; 512] {
    let mut r = [0u8; 512];
    r[32..64].copy_from_slice(&dir_entry(name, 0x20, 3, size));
    r
}

macro_rules! vol {
    ($v:ident) => {
        let mut details = DetailsFile::new(DeviceInfo {
            version: "0.1.0",
            serial: "E6614C30934B622C",
            reset_count: 0,
        });
        let mut files: [&mut dyn VfsFile; 1] = [&mut details];
        let mut $v: V<'_> = Vfs::new(
            VfsConfig {
                volume_label: "DAPLINK",
                volume_id: 0xE661_4C30,
            },
            &mut files[..],
            FlashSink::new(Backend),
            NoObserver,
        );
    };
}

fn remount(v: &mut V<'_>) {
    while v.media() == MediaState::Ready {
        v.tick(1);
    }
    assert_eq!(v.tick(MSC_RECONNECT_MS), MediaState::Ready);
}

#[test]
fn every_host_visible_state_passes_the_checker() {
    vol!(v);
    check("fresh", &mut v);

    // Mid-drop: the host's FAT, boot and directory writes are not stored, so a re-read is unchanged.
    let mut boot = [0u8; 512];
    v.read_sector(0, &mut boot);
    boot[37] |= 1;
    v.write_sector(0, &boot);
    let mut f = [0u8; 512];
    v.read_sector(1, &mut f);
    f[3] &= 0x3F;
    f[6..10].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]);
    v.write_sector(1, &f);
    v.write_sector(131, &root(b"APP     UF2", 1024));
    v.write_sector(
        fat::cluster_to_lba(3),
        &uf2_numbered(0, 2, BASE, &[0xA1; 256]),
    );
    check("mid_drop", &mut v);

    v.write_sector(
        fat::cluster_to_lba(3) + 1,
        &uf2_numbered(1, 2, BASE + 256, &[0xB2; 256]),
    );
    remount(&mut v);
    check("after_good_drop", &mut v);

    v.write_sector(131, &root(b"BAD     UF2", 512));
    v.write_sector(
        fat::cluster_to_lba(3),
        &uf2_numbered(0, 1, 0x2000_0000, &[0xA1; 256]),
    );
    remount(&mut v);
    check("after_range_failure", &mut v);

    v.write_sector(131, &root(b"EMPTY   UF2", 4096));
    assert_eq!(v.tick(MSC_TIMEOUT_ACTIVE_MS), MediaState::Detached);
    assert_eq!(v.tick(MSC_RECONNECT_MS), MediaState::Ready);
    check("after_no_data", &mut v);

    // The status field filled to its width, and a remount count past its field.
    for _ in 0..3 {
        v.write_sector(131, &root(b"BAD     UF2", 512));
        v.write_sector(
            fat::cluster_to_lba(3),
            &uf2_numbered(0, 1, 0x2000_0000, &[0xA1; 256]),
        );
        remount(&mut v);
    }
    check("after_repeated_remounts", &mut v);
}

#[test]
fn a_seeded_fault_and_status_passes_the_checker() {
    let mut details = DetailsFile::new(DeviceInfo {
        version: "0.1.0",
        serial: "S",
        reset_count: 99_999_999,
    });
    details.seed_status(&[b'A'; 200]);
    details.seed_fault(&[b'B'; 200]);
    let mut files: [&mut dyn VfsFile; 1] = [&mut details];
    let mut v: V<'_> = Vfs::new(
        VfsConfig::default(),
        &mut files[..],
        FlashSink::new(Backend),
        NoObserver,
    );
    check("seeded_fault_default_id", &mut v);
}
