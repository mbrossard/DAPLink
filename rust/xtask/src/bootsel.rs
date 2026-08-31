//! `Bootloader::BootselUf2` flashing: `0x9F` reboot, copy the `.uf2`, await re-enumeration.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::{DeviceInfo, MaybeFuture};

use crate::usb;

/// `bInterfaceClass` for a vendor-specific interface (what CMSIS-DAP v2 advertises).
const VENDOR_SPECIFIC_CLASS: u8 = 0xFF;
/// Vendor command id (`ID_DAP_Vendor31`) and reboot subcommand/guard magic, per the probe's
/// `0x9F` handler (`tools/probe-reboot/src/proto.rs` documents the full exchange).
const REBOOT_CMD_ID: u8 = 0x9F;
const SUB_REBOOT: u8 = 0x01;
const REBOOT_MAGIC: [u8; 4] = *b"BOOT";

const TRANSFER_TIMEOUT: Duration = Duration::from_secs(2);
const MOUNT_TIMEOUT: Duration = Duration::from_secs(15);
/// The volume label the RP2 ROM presents, which is also the directory udisks mounts it under.
const BOOTSEL_LABEL: &str = "RPI-RP2";
const REENUM_TIMEOUT: Duration = Duration::from_secs(15);
const POLL_INTERVAL: Duration = Duration::from_millis(200);
/// How long the re-enumerated PUT must stay on the bus before it counts as back.
const REENUM_SETTLE: Duration = Duration::from_millis(750);

/// Locate the CMSIS-DAP v2 interface (a vendor-specific interface with a bulk IN + bulk OUT
/// pair) and its endpoint addresses.
fn find_dap_v2_interface(device: &nusb::Device) -> Result<(u8, u8, u8), String> {
    let config = device
        .active_configuration()
        .map_err(|e| format!("reading the active configuration: {e}"))?;
    for group in config.interfaces() {
        let alt = group.first_alt_setting();
        if alt.class() != VENDOR_SPECIFIC_CLASS {
            continue;
        }
        let mut ep_out = None;
        let mut ep_in = None;
        for ep in alt.endpoints() {
            if ep.transfer_type() != TransferType::Bulk {
                continue;
            }
            match ep.direction() {
                Direction::Out if ep_out.is_none() => ep_out = Some(ep.address()),
                Direction::In if ep_in.is_none() => ep_in = Some(ep.address()),
                _ => {}
            }
        }
        if let (Some(out), Some(in_)) = (ep_out, ep_in) {
            return Ok((alt.interface_number(), out, in_));
        }
    }
    Err("no CMSIS-DAP v2 bulk interface found on the PUT".to_string())
}

/// Send the `0x9F` reboot command and drop the interface so the PUT can detach; a failed reply
/// read is not an error, the PUT detaching before it lands being a sign it worked.
fn arm_reboot(info: &DeviceInfo) -> Result<(), String> {
    let device = info
        .open()
        .wait()
        .map_err(|e| format!("opening the PUT: {e}"))?;
    let (iface_num, ep_out, ep_in) = find_dap_v2_interface(&device)?;
    let interface = device
        .claim_interface(iface_num)
        .wait()
        .map_err(|e| format!("claiming the CMSIS-DAP v2 interface: {e}"))?;
    let mut out = interface
        .endpoint::<Bulk, Out>(ep_out)
        .map_err(|e| format!("opening bulk OUT endpoint: {e}"))?;
    let mut in_ = interface
        .endpoint::<Bulk, In>(ep_in)
        .map_err(|e| format!("opening bulk IN endpoint: {e}"))?;

    let mut cmd = [0u8; 6];
    cmd[0] = REBOOT_CMD_ID;
    cmd[1] = SUB_REBOOT;
    cmd[2..6].copy_from_slice(&REBOOT_MAGIC);
    out.transfer_blocking(Buffer::from(cmd.as_slice()), TRANSFER_TIMEOUT)
        .into_result()
        .map_err(|e| format!("bulk OUT: {e}"))?;
    // Best-effort: a failed IN read here almost always means the PUT already detached.
    let _ = in_
        .transfer_blocking(Buffer::new(64), TRANSFER_TIMEOUT)
        .into_result();

    drop(out);
    drop(in_);
    drop(interface);
    drop(device);
    Ok(())
}

/// Where the BOOTSEL volume lands by default. macOS mounts removable media under `/Volumes`;
/// udisks puts it under `/run/media/<user>`. Either can be overridden with `--bootsel-mount`.
#[cfg(target_os = "linux")]
fn default_bootsel_mount() -> PathBuf {
    let user = std::env::var("USER").unwrap_or_else(|_| String::from("root"));
    PathBuf::from(format!("/run/media/{user}/{BOOTSEL_LABEL}"))
}

#[cfg(not(target_os = "linux"))]
fn default_bootsel_mount() -> PathBuf {
    PathBuf::from(format!("/Volumes/{BOOTSEL_LABEL}"))
}

/// Mount the ROM's volume when nothing else will: a host with no desktop session has no
/// automount, and an unmounted drive is indistinguishable from a probe that never rebooted.
#[cfg(target_os = "linux")]
fn try_mount() -> bool {
    use std::process::Command;
    let Ok(listed) = Command::new("lsblk").args(["-rno", "NAME,LABEL"]).output() else {
        return false;
    };
    let text = String::from_utf8_lossy(&listed.stdout);
    let Some(name) = text.lines().find_map(|line| {
        let mut it = line.split_whitespace();
        let name = it.next()?;
        (it.next()? == BOOTSEL_LABEL).then(|| name.to_string())
    }) else {
        return false;
    };
    Command::new("udisksctl")
        .args(["mount", "-b", &format!("/dev/{name}")])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

#[cfg(not(target_os = "linux"))]
fn try_mount() -> bool {
    false
}

fn wait_for_path(path: &Path, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        // The mount point can appear before the volume; only the ROM's marker file proves it.
        if path.join("INFO_UF2.TXT").exists() {
            return true;
        }
        // Nothing else is going to do it on a headless host.
        try_mount();
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

fn copy_uf2(uf2_path: &Path, mount: &Path) -> Result<(), String> {
    let name = uf2_path
        .file_name()
        .ok_or_else(|| format!("{} has no file name", uf2_path.display()))?;
    let dest = mount.join(name);
    let bytes =
        std::fs::read(uf2_path).map_err(|e| format!("reading {}: {e}", uf2_path.display()))?;
    let mut out =
        std::fs::File::create(&dest).map_err(|e| format!("creating {}: {e}", dest.display()))?;
    use std::io::Write;
    out.write_all(&bytes)
        .map_err(|e| format!("writing {}: {e}", dest.display()))?;
    // The ROM reboots on the last block, so a write still in the page cache is lost.
    out.sync_all()
        .map_err(|e| format!("syncing {}: {e}", dest.display()))?;
    Ok(())
}

/// Wait for the drive to leave: the ROM ejecting the volume is its acknowledgement that the
/// whole image arrived.
fn wait_for_path_gone(path: &Path, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if !path.join("INFO_UF2.TXT").exists() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Flash a `Bootloader::BootselUf2` row, `sel` naming the PUT's own identity rather than a
/// transport; an absent PUT with the drive already mounted skips the reboot-arm phase.
pub fn flash_bootsel(uf2_path: &Path, sel: &usb::Selector, mount_override: Option<&Path>) -> bool {
    let mount = mount_override
        .map(Path::to_path_buf)
        .unwrap_or_else(default_bootsel_mount);

    match sel.find() {
        Some(info) => {
            println!("phase locate-put: found {sel}");
            println!("phase arm-reboot: sending 0x9F reboot to the PUT");
            if let Err(e) = arm_reboot(&info) {
                eprintln!("phase arm-reboot: FAILED: {e}");
                return false;
            }
            println!("phase wait-mount: waiting for {}", mount.display());
            if !wait_for_path(&mount, MOUNT_TIMEOUT) {
                eprintln!(
                    "phase wait-mount: FAILED: {} did not appear within {MOUNT_TIMEOUT:?}",
                    mount.display()
                );
                return false;
            }
        }
        None if mount.join("INFO_UF2.TXT").exists() => {
            println!(
                "phase locate-put: PUT not present; {} already mounted, skipping reboot arm",
                mount.display()
            );
        }
        None => {
            eprintln!(
                "phase locate-put: FAILED: neither PUT {sel} nor {} found",
                mount.display()
            );
            return false;
        }
    }

    println!(
        "phase copy-uf2: copying {} to {}",
        uf2_path.display(),
        mount.display()
    );
    if let Err(e) = copy_uf2(uf2_path, &mount) {
        eprintln!("phase copy-uf2: FAILED: {e}");
        return false;
    }

    println!("phase wait-eject: waiting for {} to leave", mount.display());
    if !wait_for_path_gone(&mount, MOUNT_TIMEOUT) {
        eprintln!(
            "phase wait-eject: FAILED: {} still mounted within {MOUNT_TIMEOUT:?} -- the ROM did not accept the image",
            mount.display()
        );
        return false;
    }

    println!("phase wait-reenum: waiting for {sel}");
    if !usb::wait_settled(sel, REENUM_SETTLE, REENUM_TIMEOUT) {
        eprintln!(
            "phase wait-reenum: FAILED: PUT did not settle on the bus within {REENUM_TIMEOUT:?}"
        );
        return false;
    }
    println!("phase wait-reenum: PUT re-enumerated");
    true
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_bare_mount_directory_is_not_a_mounted_drive() {
        // The mount point can exist before the volume; only the ROM's marker file proves it.
        let dir = std::env::temp_dir().join("daplink-bootsel-marker-test");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        assert!(!super::wait_for_path(
            &dir,
            std::time::Duration::from_millis(50)
        ));
        std::fs::write(dir.join("INFO_UF2.TXT"), b"UF2 Bootloader").unwrap();
        assert!(super::wait_for_path(
            &dir,
            std::time::Duration::from_millis(50)
        ));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_drive_leaving_means_the_marker_disappears() {
        let dir = std::env::temp_dir().join("daplink-bootsel-eject-test");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("INFO_UF2.TXT"), b"UF2").unwrap();
        assert!(!super::wait_for_path_gone(
            &dir,
            std::time::Duration::from_millis(50)
        ));
        std::fs::remove_file(dir.join("INFO_UF2.TXT")).unwrap();
        assert!(super::wait_for_path_gone(
            &dir,
            std::time::Duration::from_millis(50)
        ));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
