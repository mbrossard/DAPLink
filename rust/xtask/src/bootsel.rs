//! `Bootloader::BootselUf2` flashing: `0x9F` reboot, copy the `.uf2`, await re-enumeration.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::{DeviceInfo, MaybeFuture};

use crate::registry::{UF2_FAMILY_RP2040, UF2_FAMILY_RP2350};
use crate::usb;

const VENDOR_SPECIFIC_CLASS: u8 = 0xFF;
const REBOOT_CMD_ID: u8 = 0x9F;
const SUB_REBOOT: u8 = 0x01;
const REBOOT_MAGIC: [u8; 4] = *b"BOOT";

const TRANSFER_TIMEOUT: Duration = Duration::from_secs(2);
const MOUNT_TIMEOUT: Duration = Duration::from_secs(15);
/// Long enough to mount a volume that is already there, on a host without an automounter.
const ABSENT_MOUNT_TIMEOUT: Duration = Duration::from_secs(3);
/// The RP2040 and RP2350 ROM volume labels; udisks mounts each under its own name.
const BOOTSEL_LABELS: [&str; 2] = ["RPI-RP2", "RP2350"];
/// How messages name a ROM volume of either label.
const BOOTSEL_VOLUME: &str = "RPI-RP2 or RP2350";
const REENUM_TIMEOUT: Duration = Duration::from_secs(15);
const POLL_INTERVAL: Duration = Duration::from_millis(200);
/// How long the re-enumerated PUT must stay on the bus before it counts as back.
const REENUM_SETTLE: Duration = Duration::from_millis(750);

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

#[derive(Debug, PartialEq, Eq)]
enum ArmAcknowledgement {
    Armed,
    Unconfirmed(String),
}

fn arm_transaction(
    submit: impl FnOnce() -> Result<(), String>,
    receive: impl FnOnce() -> Result<Vec<u8>, String>,
) -> Result<ArmAcknowledgement, String> {
    submit()?;
    let reply = match receive() {
        Ok(reply) => reply,
        Err(error) => return Ok(ArmAcknowledgement::Unconfirmed(error)),
    };
    match reply.as_slice() {
        [REBOOT_CMD_ID, 0, ..] => Ok(ArmAcknowledgement::Armed),
        [REBOOT_CMD_ID, status @ 1..=3, ..] => {
            let reason = match status {
                1 => "unsupported",
                2 => "bad magic",
                _ => "busy",
            };
            Err(format!("reboot refused: {reason}"))
        }
        _ => Ok(ArmAcknowledgement::Unconfirmed(format!(
            "unrecognized reboot acknowledgement: {reply:02x?}"
        ))),
    }
}

fn submitted_length(actual: usize, expected: usize) -> Result<(), String> {
    if actual != expected {
        return Err(format!(
            "bulk OUT incomplete: submitted {actual} of {expected} bytes"
        ));
    }
    Ok(())
}

fn admit_unconfirmed_recovery(physical_port_bound: bool) -> Result<(), String> {
    if !physical_port_bound {
        return Err(
            "unconfirmed arming: this host cannot bind ROM recovery to the PUT's physical port"
                .into(),
        );
    }
    Ok(())
}

/// Unconfirmed arming requires the existing same-port ROM-volume verification before copying.
fn arm_reboot(info: &DeviceInfo) -> Result<ArmAcknowledgement, String> {
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
    let acknowledgement = arm_transaction(
        || {
            let completion = out.transfer_blocking(Buffer::from(cmd.as_slice()), TRANSFER_TIMEOUT);
            completion.status.map_err(|e| format!("bulk OUT: {e}"))?;
            submitted_length(completion.actual_len, cmd.len())
        },
        || {
            in_.transfer_blocking(Buffer::new(64), TRANSFER_TIMEOUT)
                .into_result()
                .map(|reply| reply.into_vec())
                .map_err(|e| format!("bulk IN: {e}"))
        },
    );

    drop(out);
    drop(in_);
    drop(interface);
    drop(device);
    acknowledgement
}

/// The ROM's marker file; the mount point can appear before the volume, but not this.
const MARKER: &str = "INFO_UF2.TXT";

const UF2_BLOCK: usize = 512;
const UF2_MAGIC: [(usize, u32); 3] = [(0, 0x0A32_4655), (4, 0x9E5D_5157), (508, 0x0AB1_6F30)];
const UF2_FLAG_FAMILY: u32 = 0x2000;
/// RP2350 Arm secure, RISC-V and Arm non-secure.
const FAMILIES_RP2350: [u32; 3] = [UF2_FAMILY_RP2350, 0xe48b_ff5a, 0xe48b_ff5b];

/// The one family every block of a UF2 image is tagged with.
fn uf2_family(uf2: &[u8]) -> Result<u32, String> {
    if uf2.is_empty() || !uf2.len().is_multiple_of(UF2_BLOCK) {
        return Err(format!("{} bytes is not whole UF2 blocks", uf2.len()));
    }
    let word = |block: &[u8], off: usize| {
        u32::from_le_bytes(block[off..off + 4].try_into().expect("four bytes"))
    };
    let mut family = None;
    for (i, block) in uf2.chunks(UF2_BLOCK).enumerate() {
        if UF2_MAGIC
            .iter()
            .any(|&(off, magic)| word(block, off) != magic)
        {
            return Err(format!("block {i} is not a UF2 block"));
        }
        if word(block, 8) & UF2_FLAG_FAMILY == 0 {
            return Err(format!("block {i} carries no family id"));
        }
        let id = word(block, 28);
        match family {
            Some(first) if first != id => {
                return Err(format!("block {i} is family {id:#010x}, not {first:#010x}"));
            }
            _ => family = Some(id),
        }
    }
    family.ok_or_else(|| "no UF2 blocks".to_string())
}

/// Whether the ROM whose marker file reads `info` takes an image of `family`.
fn rom_takes(info: &str, family: u32) -> Result<(), String> {
    let id = info
        .lines()
        .find_map(|l| l.strip_prefix("Board-ID:"))
        .map(str::trim)
        .ok_or_else(|| format!("{MARKER} names no Board-ID"))?;
    let families: &[u32] = match id {
        "RPI-RP2" => &[UF2_FAMILY_RP2040],
        "RP2350" => &FAMILIES_RP2350,
        _ => return Err(format!("unknown ROM Board-ID {id:?}")),
    };
    if !families.contains(&family) {
        return Err(format!(
            "the image is UF2 family {family:#010x}, which the {id} ROM does not take"
        ));
    }
    Ok(())
}

/// Refuse an image the ROM at `mount` would not boot, before anything is written to it.
fn check_family(mount: &Path, family: u32) -> Result<(), String> {
    let marker = mount.join(MARKER);
    let info = std::fs::read_to_string(&marker)
        .map_err(|e| format!("reading {}: {e}", marker.display()))?;
    rom_takes(&info, family)
}

/// A USB device's sysfs name, `<bus>-<port>.<port>...`; the ROM re-enumerates under the PUT's.
fn usb_port(bus: &str, chain: &[u8]) -> Option<String> {
    let (first, rest) = chain.split_first()?;
    #[cfg(target_os = "linux")]
    let bus = {
        if bus.is_empty() || !bus.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        bus.parse::<u8>().ok()?.to_string()
    };
    Some(
        rest.iter()
            .fold(format!("{bus}-{first}"), |s, p| format!("{s}.{p}")),
    )
}

/// A ROM volume as `lsblk` lists it; an empty mount point means unmounted.
#[cfg(any(test, target_os = "linux"))]
#[derive(Debug, PartialEq)]
struct Volume {
    name: String,
    mountpoint: String,
}

/// Decode `lsblk`'s `\xHH` escapes.
#[cfg(any(test, target_os = "linux"))]
fn unescape(value: &str) -> String {
    let mut bytes = Vec::new();
    let mut raw = value.as_bytes();
    while let Some((&b, tail)) = raw.split_first() {
        let escaped = match tail {
            [b'x', hi, lo, ..] if b == b'\\' => std::str::from_utf8(&[*hi, *lo])
                .ok()
                .and_then(|h| u8::from_str_radix(h, 16).ok()),
            _ => None,
        };
        bytes.push(escaped.unwrap_or(b));
        raw = &tail[if escaped.is_some() { 3 } else { 0 }..];
    }
    String::from_utf8_lossy(&bytes).into_owned()
}

/// One `lsblk -P` line's `KEY="value"` pairs; quotes inside a value arrive escaped.
#[cfg(any(test, target_os = "linux"))]
fn lsblk_pairs(line: &str) -> Vec<(&str, String)> {
    let mut pairs = Vec::new();
    let mut rest = line;
    while let Some((key, tail)) = rest.split_once("=\"") {
        let Some((value, tail)) = tail.split_once('"') else {
            break;
        };
        pairs.push((key.trim(), unescape(value)));
        rest = tail;
    }
    pairs
}

/// The ROM-labelled volumes in `lsblk -P -o NAME,LABEL,MOUNTPOINT` output.
#[cfg(any(test, target_os = "linux"))]
fn rom_volumes(listing: &str) -> Vec<Volume> {
    let mut volumes = Vec::new();
    for line in listing.lines() {
        let (mut name, mut label, mut mountpoint) = (None, None, String::new());
        for (key, value) in lsblk_pairs(line) {
            match key {
                "NAME" => name = Some(value),
                "LABEL" => label = Some(value),
                "MOUNTPOINT" => mountpoint = value,
                _ => {}
            }
        }
        if let (Some(name), Some(label)) = (name, label) {
            if BOOTSEL_LABELS.contains(&label.as_str()) {
                volumes.push(Volume { name, mountpoint });
            }
        }
    }
    volumes
}

/// Whether a sysfs path runs through USB device `port`; whole components, so `6-1.2` is not `6-1.2.2`.
#[cfg(any(test, target_os = "linux"))]
fn through_port(sysfs: &Path, port: &str) -> bool {
    sysfs.components().any(|c| c.as_os_str() == port)
}

/// The one volume at `port`, or with no port the only one; several is a refusal.
#[cfg(any(test, target_os = "linux"))]
fn choose<'a>(
    volumes: &'a [Volume],
    port: Option<&str>,
    sysfs: impl Fn(&str) -> Option<PathBuf>,
) -> Result<Option<&'a Volume>, String> {
    let found: Vec<&Volume> = volumes
        .iter()
        .filter(|v| port.is_none_or(|p| sysfs(&v.name).is_some_and(|s| through_port(&s, p))))
        .collect();
    match found[..] {
        [] => Ok(None),
        [one] => Ok(Some(one)),
        _ => {
            let names: Vec<&str> = found.iter().map(|v| v.name.as_str()).collect();
            let names = names.join(", ");
            Err(match port {
                Some(p) => format!("several {BOOTSEL_VOLUME} volumes at USB port {p}: {names}"),
                None => format!(
                    "several {BOOTSEL_VOLUME} volumes ({names}) and no PUT on the bus to tell \
                     which is its own"
                ),
            })
        }
    }
}

/// The mounted ROM volume at `port`, or with no port the host's only one; mounts it if needed.
#[cfg(target_os = "linux")]
fn locate(port: Option<&str>, mount_override: Option<&Path>) -> Result<Option<PathBuf>, String> {
    use std::process::Command;
    let listed = Command::new("lsblk")
        .args(["-P", "-o", "NAME,LABEL,MOUNTPOINT"])
        .output()
        .map_err(|e| format!("running lsblk: {e}"))?;
    let volumes = rom_volumes(&String::from_utf8_lossy(&listed.stdout));
    let sysfs = |name: &str| std::fs::canonicalize(Path::new("/sys/class/block").join(name)).ok();
    let Some(volume) = choose(&volumes, port, sysfs)? else {
        return Ok(None);
    };
    if volume.mountpoint.is_empty() {
        // A headless host has no automounter.
        let _ = Command::new("udisksctl")
            .args(["mount", "-b", &format!("/dev/{}", volume.name)])
            .output();
        return Ok(None);
    }
    let mount = PathBuf::from(&volume.mountpoint);
    match mount_override {
        Some(want) if want != mount => Err(format!(
            "{} is mounted at {}, not at --bootsel-mount {}",
            volume.name,
            mount.display(),
            want.display()
        )),
        _ => Ok(Some(mount)),
    }
}

/// Whether a `/Volumes` entry is a ROM volume; a second one mounts as `RPI-RP2 1`.
#[cfg(any(test, not(target_os = "linux")))]
fn is_rom_volume_name(name: &str) -> bool {
    BOOTSEL_LABELS.iter().any(|label| {
        name == *label
            || name
                .strip_prefix(label)
                .and_then(|n| n.strip_prefix(' '))
                .is_some_and(|n| n.parse::<u32>().is_ok())
    })
}

/// The override, else the only ROM volume under `/Volumes`; nothing ties it to a USB port here.
#[cfg(not(target_os = "linux"))]
fn locate(_port: Option<&str>, mount_override: Option<&Path>) -> Result<Option<PathBuf>, String> {
    if let Some(mount) = mount_override {
        return Ok(Some(mount.to_path_buf()));
    }
    let mut found: Vec<String> = std::fs::read_dir("/Volumes")
        .map(|dir| {
            dir.filter_map(|e| e.ok()?.file_name().into_string().ok())
                .filter(|n| is_rom_volume_name(n))
                .collect()
        })
        .unwrap_or_default();
    match found.len() {
        0 => Ok(None),
        1 => Ok(Some(Path::new("/Volumes").join(found.remove(0)))),
        _ => Err(format!(
            "several {BOOTSEL_VOLUME} volumes ({}); pass --bootsel-mount",
            found.join(", ")
        )),
    }
}

/// Poll `locate` until the volume it names holds the ROM's marker; `None` on timeout.
fn wait_for_volume(
    timeout: Duration,
    mut locate: impl FnMut() -> Result<Option<PathBuf>, String>,
) -> Result<Option<PathBuf>, String> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(mount) = locate()?.filter(|m| m.join(MARKER).exists()) {
            return Ok(Some(mount));
        }
        if Instant::now() >= deadline {
            return Ok(None);
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

fn copy_uf2(uf2_path: &Path, bytes: &[u8], mount: &Path) -> Result<(), String> {
    let name = uf2_path
        .file_name()
        .ok_or_else(|| format!("{} has no file name", uf2_path.display()))?;
    let dest = mount.join(name);
    let mut out =
        std::fs::File::create(&dest).map_err(|e| format!("creating {}: {e}", dest.display()))?;
    use std::io::Write;
    out.write_all(bytes)
        .map_err(|e| format!("writing {}: {e}", dest.display()))?;
    // The ROM reboots on the last block, so a write still in the page cache is lost.
    out.sync_all()
        .map_err(|e| format!("syncing {}: {e}", dest.display()))?;
    Ok(())
}

/// Wait for the ROM marker to disappear; re-enumeration is checked separately.
fn wait_for_path_gone(path: &Path, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if !path.join(MARKER).exists() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Flash through the PUT's own ROM volume; an absent PUT uses the host's only one, if any.
pub fn flash_bootsel(uf2_path: &Path, sel: &usb::Selector, mount_override: Option<&Path>) -> bool {
    let image = std::fs::read(uf2_path)
        .map_err(|e| format!("reading {}: {e}", uf2_path.display()))
        .and_then(|bytes| Ok((uf2_family(&bytes)?, bytes)));
    let (family, image) = match image {
        Ok(image) => image,
        Err(e) => {
            eprintln!("phase read-uf2: FAILED: {}: {e}", uf2_path.display());
            return false;
        }
    };
    let mount = match sel.find() {
        Some(info) => {
            let Some(port) = usb_port(info.bus_id(), info.port_chain()) else {
                eprintln!(
                    "phase locate-put: FAILED: {sel} reports invalid USB bus or port (bus {:?})",
                    info.bus_id()
                );
                return false;
            };
            println!("phase locate-put: found {sel} at USB port {port}");
            // The PUT still runs its firmware, so a ROM volume found now is another board's.
            match locate(Some(&port), mount_override) {
                Ok(Some(found)) if found.join(MARKER).exists() => {
                    eprintln!(
                        "phase locate-put: FAILED: {} is a BOOTSEL volume before the PUT rebooted, \
                         so it cannot be told from the PUT's",
                        found.display()
                    );
                    return false;
                }
                Err(e) => {
                    eprintln!("phase locate-put: FAILED: {e}");
                    return false;
                }
                _ => {}
            }
            println!("phase arm-reboot: sending 0x9F reboot to the PUT");
            match arm_reboot(&info) {
                Ok(ArmAcknowledgement::Armed) => {
                    println!("phase arm-reboot: acknowledged arming");
                }
                Ok(ArmAcknowledgement::Unconfirmed(reason)) => {
                    if let Err(error) = admit_unconfirmed_recovery(cfg!(target_os = "linux")) {
                        eprintln!("phase arm-reboot: FAILED: {error} ({reason})");
                        return false;
                    }
                    println!(
                        "phase arm-reboot: UNCONFIRMED ({reason}); verifying the ROM volume at the original PUT port"
                    );
                }
                Err(e) => {
                    eprintln!("phase arm-reboot: FAILED: {e}");
                    return false;
                }
            }
            println!("phase wait-mount: waiting for an {BOOTSEL_VOLUME} volume at USB port {port}");
            match wait_for_volume(MOUNT_TIMEOUT, || locate(Some(&port), mount_override)) {
                Ok(Some(mount)) => mount,
                Ok(None) => {
                    eprintln!(
                        "phase wait-mount: FAILED: no {BOOTSEL_VOLUME} volume at USB port {port} \
                         within {MOUNT_TIMEOUT:?}"
                    );
                    return false;
                }
                Err(e) => {
                    eprintln!("phase wait-mount: FAILED: {e}");
                    return false;
                }
            }
        }
        None => match wait_for_volume(ABSENT_MOUNT_TIMEOUT, || locate(None, mount_override)) {
            Ok(Some(mount)) => {
                println!(
                    "phase locate-put: PUT not present; {} mounted, skipping reboot arm",
                    mount.display()
                );
                mount
            }
            Ok(None) => {
                eprintln!(
                    "phase locate-put: FAILED: neither PUT {sel} nor an {BOOTSEL_VOLUME} volume found"
                );
                return false;
            }
            Err(e) => {
                eprintln!("phase locate-put: FAILED: {e}");
                return false;
            }
        },
    };

    if let Err(e) = check_family(&mount, family) {
        eprintln!(
            "phase check-family: FAILED: {e}; {} is left in BOOTSEL",
            mount.display()
        );
        return false;
    }
    println!(
        "phase copy-uf2: copying {} to {}",
        uf2_path.display(),
        mount.display()
    );
    if let Err(e) = copy_uf2(uf2_path, &image, &mount) {
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
    use super::*;

    #[test]
    fn failed_or_partial_submission_never_reads_an_acknowledgement() {
        for actual in 0..6 {
            let result = arm_transaction(
                || submitted_length(actual, 6),
                || panic!("a partial command must not read an acknowledgement"),
            );
            assert!(result.unwrap_err().contains("bulk OUT incomplete"));
        }
        let result = arm_transaction(
            || Err("bulk OUT transport".into()),
            || panic!("a failed command must not read an acknowledgement"),
        );
        assert_eq!(result, Err("bulk OUT transport".into()));
        assert!(submitted_length(6, 6).is_ok());
        assert!(submitted_length(7, 6).is_err());
    }

    #[test]
    fn refused_reboots_do_not_enter_rom_volume_recovery() {
        for (status, reason) in [(1, "unsupported"), (2, "bad magic"), (3, "busy")] {
            let result = arm_transaction(|| Ok(()), || Ok(vec![REBOOT_CMD_ID, status]));
            assert_eq!(result, Err(format!("reboot refused: {reason}")));
        }
        assert_eq!(
            arm_transaction(|| Ok(()), || Ok(vec![REBOOT_CMD_ID, 0, 0])),
            Ok(ArmAcknowledgement::Armed)
        );
    }

    #[test]
    fn uncertain_arming_requires_existing_same_port_recovery() {
        for reply in [
            vec![],
            vec![REBOOT_CMD_ID],
            vec![0xff, 0],
            vec![REBOOT_CMD_ID, 4],
        ] {
            assert!(matches!(
                arm_transaction(|| Ok(()), || Ok(reply)),
                Ok(ArmAcknowledgement::Unconfirmed(_))
            ));
        }
        assert_eq!(
            arm_transaction(|| Ok(()), || Err("bulk IN lost".into())),
            Ok(ArmAcknowledgement::Unconfirmed("bulk IN lost".into()))
        );
        assert!(admit_unconfirmed_recovery(true).is_ok());
        assert!(admit_unconfirmed_recovery(false)
            .unwrap_err()
            .contains("physical port"));
    }

    const RP_SYSFS: &str =
        "/sys/devices/platform/CIXH2030:07/CIXH2031:07/xhci-hcd.15.auto/usb6/6-1/6-1.2";

    /// A mass-storage partition's sysfs path below USB device `port` on hub `6-1.2`.
    fn partition(hub_port: &str, port: &str, host: u32, disk: &str) -> PathBuf {
        PathBuf::from(format!(
            "{RP_SYSFS}/{hub_port}/{port}/{port}:1.0/host{host}/target{host}:0:0/{host}:0:0:0/block/{disk}/{disk}1"
        ))
    }

    /// Two RP2040s in BOOTSEL behind two hubs, beside a DAPLink drive and the host's own disks.
    const LSBLK: &str = r#"NAME="loop0" LABEL="" MOUNTPOINT="/snap/core24/1644"
NAME="sda" LABEL="" MOUNTPOINT=""
NAME="sda1" LABEL="RPI-RP2" MOUNTPOINT="/run/media/user/RPI-RP2"
NAME="sdb" LABEL="" MOUNTPOINT=""
NAME="sdb1" LABEL="RPI-RP2" MOUNTPOINT=""
NAME="sdc" LABEL="DAPLINK" MOUNTPOINT="/run/media/user/DAPLINK"
NAME="nvme0n1" LABEL="" MOUNTPOINT=""
NAME="nvme0n1p2" LABEL="" MOUNTPOINT="/"
"#;

    fn sysfs(name: &str) -> Option<PathBuf> {
        match name {
            "sda1" => Some(partition("6-1.2.2", "6-1.2.2.3", 0, "sda")),
            "sdb1" => Some(partition("6-1.2.4", "6-1.2.4.3", 1, "sdb")),
            _ => None,
        }
    }

    #[test]
    fn the_port_is_named_as_sysfs_names_the_device() {
        assert_eq!(usb_port("6", &[1, 2, 2, 3]).as_deref(), Some("6-1.2.2.3"));
        assert_eq!(usb_port("3", &[4]).as_deref(), Some("3-4"));
        assert_eq!(usb_port("3", &[]), None);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_bus_names_are_strict_canonical_decimal() {
        for (bus, want) in [
            ("006", "6-4"),
            ("6", "6-4"),
            ("001", "1-4"),
            ("255", "255-4"),
            ("000", "0-4"),
            ("000006", "6-4"),
        ] {
            assert_eq!(usb_port(bus, &[4]).as_deref(), Some(want));
            assert_eq!(usb_port(bus, &[]), None);
        }
        for bus in [
            "",
            "+6",
            "-6",
            " 6",
            "6 ",
            "6x",
            "0x6",
            "6-1",
            "\u{0666}",
            "256",
            "999999999999999999999999999999",
        ] {
            assert_eq!(usb_port(bus, &[4]), None, "{bus:?}");
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn padded_bus_selects_only_the_puts_volume() {
        let volumes = rom_volumes(LSBLK);
        for bus in ["006", "6"] {
            let port = usb_port(bus, &[1, 2, 4, 3]).unwrap();
            assert_eq!(choose(&volumes, Some(&port), sysfs), Ok(Some(&volumes[1])));
            assert_eq!(choose(&volumes[..1], Some(&port), sysfs), Ok(None));
            assert_eq!(choose(&volumes, Some(&port), |_| None), Ok(None));
            assert!(choose(&volumes, Some(&port), |_| sysfs("sdb1")).is_err());
            assert!(choose(&volumes, None, sysfs).is_err());
        }
    }

    #[cfg(not(target_os = "linux"))]
    #[test]
    fn non_linux_bus_identity_is_preserved() {
        assert_eq!(usb_port("0a", &[4]).as_deref(), Some("0a-4"));
        assert_eq!(usb_port("06", &[4]).as_deref(), Some("06-4"));
        assert_eq!(usb_port("0a", &[]), None);
    }

    #[test]
    fn lsblk_values_are_unescaped() {
        let pairs =
            lsblk_pairs(r#"NAME="sda1" LABEL="RPI-RP2" MOUNTPOINT="/media/a\x20b\x22c\x5c""#);
        assert_eq!(
            pairs,
            [
                ("NAME", "sda1".to_string()),
                ("LABEL", "RPI-RP2".to_string()),
                ("MOUNTPOINT", "/media/a b\"c\\".to_string()),
            ]
        );
        assert_eq!(unescape(r"\x2"), r"\x2");
        assert_eq!(unescape(r"\xzz"), r"\xzz");
    }

    #[test]
    fn only_rom_labelled_volumes_are_listed() {
        assert_eq!(
            rom_volumes(LSBLK),
            [
                Volume {
                    name: "sda1".into(),
                    mountpoint: "/run/media/user/RPI-RP2".into()
                },
                Volume {
                    name: "sdb1".into(),
                    mountpoint: String::new()
                },
            ]
        );
    }

    /// The bug: with two boards in BOOTSEL, the first listed volume is not necessarily the PUT's.
    #[test]
    fn the_volume_is_the_one_at_the_puts_port() {
        let volumes = rom_volumes(LSBLK);
        let at = |port| choose(&volumes, Some(port), sysfs).map(|v| v.map(|v| v.name.as_str()));
        assert_eq!(at("6-1.2.4.3"), Ok(Some("sdb1")));
        assert_eq!(at("6-1.2.2.3"), Ok(Some("sda1")));
        assert_eq!(at("6-1.2.2.2"), Ok(None));
        assert_eq!(at("6-1.2.4.33"), Ok(None));
        assert!(
            at("6-1.2").is_err(),
            "a port above both cannot tell them apart"
        );
    }

    #[test]
    fn a_port_is_matched_whole() {
        let p = partition("6-1.2.4", "6-1.2.4.13", 0, "sda");
        assert!(through_port(&p, "6-1.2.4.13"));
        assert!(!through_port(&p, "6-1.2.4.1"));
    }

    #[test]
    fn without_a_port_only_a_single_volume_is_taken() {
        let volumes = rom_volumes(LSBLK);
        let err = choose(&volumes, None, sysfs).unwrap_err();
        assert!(err.contains("sda1, sdb1"), "{err}");
        let one = &volumes[1..];
        assert_eq!(choose(one, None, |_| None), Ok(Some(&one[0])));
        assert_eq!(choose(&[], None, sysfs), Ok(None));
    }

    #[test]
    fn an_unresolvable_volume_is_not_at_any_port() {
        let volumes = rom_volumes(LSBLK);
        assert_eq!(choose(&volumes, Some("6-1.2.4.3"), |_| None), Ok(None));
    }

    #[test]
    fn a_numbered_volume_is_a_second_rom_volume() {
        assert!(is_rom_volume_name("RPI-RP2"));
        assert!(is_rom_volume_name("RPI-RP2 1"));
        assert!(!is_rom_volume_name("RPI-RP2 "));
        assert!(!is_rom_volume_name("RPI-RP21"));
        assert!(!is_rom_volume_name("DAPLINK"));
    }

    /// An RP2040 and an RP2350 in BOOTSEL, beside labels that only resemble a ROM's.
    const LSBLK_BOTH: &str = r#"NAME="sda" LABEL="" MOUNTPOINT=""
NAME="sda1" LABEL="RPI-RP2" MOUNTPOINT=""
NAME="sdb" LABEL="" MOUNTPOINT=""
NAME="sdb1" LABEL="RP2350" MOUNTPOINT="/run/media/user/RP2350"
NAME="sdc1" LABEL="rp2350" MOUNTPOINT=""
NAME="sdd1" LABEL="RP2350 1" MOUNTPOINT=""
NAME="sde1" LABEL="RP2040" MOUNTPOINT=""
"#;

    #[test]
    fn both_rom_labels_are_found_and_kept_apart() {
        let volumes = rom_volumes(LSBLK_BOTH);
        let names: Vec<&str> = volumes.iter().map(|v| v.name.as_str()).collect();
        assert_eq!(names, ["sda1", "sdb1"]);
        assert_eq!(volumes[1].mountpoint, "/run/media/user/RP2350");
        let at = |port| choose(&volumes, Some(port), sysfs).map(|v| v.map(|v| v.name.as_str()));
        assert_eq!(at("6-1.2.2.3"), Ok(Some("sda1")));
        assert_eq!(at("6-1.2.4.3"), Ok(Some("sdb1")));
        let err = choose(&volumes, None, sysfs).unwrap_err();
        assert!(err.contains("sda1, sdb1"), "{err}");
        assert_eq!(choose(&volumes[1..], None, |_| None), Ok(Some(&volumes[1])));
    }

    #[test]
    fn an_rp2350_volume_name_is_a_rom_volume() {
        assert!(is_rom_volume_name("RP2350"));
        assert!(is_rom_volume_name("RP2350 2"));
        assert!(!is_rom_volume_name("RP2350X"));
        assert!(!is_rom_volume_name("rp2350"));
    }

    fn uf2(family: u32) -> Vec<u8> {
        daplink_decoder::encode::bin_to_uf2(&[0x5a; 600], 0x1000_0000, family)
    }

    #[test]
    fn an_image_has_exactly_one_family() {
        assert_eq!(uf2_family(&uf2(UF2_FAMILY_RP2040)), Ok(UF2_FAMILY_RP2040));
        assert_eq!(uf2_family(&uf2(FAMILIES_RP2350[0])), Ok(FAMILIES_RP2350[0]));
        let mut mixed = uf2(UF2_FAMILY_RP2040);
        mixed.extend(uf2(FAMILIES_RP2350[0]));
        assert!(uf2_family(&mixed).unwrap_err().contains("block 3"));
        let mut untagged = uf2(UF2_FAMILY_RP2040);
        untagged[8..12].copy_from_slice(&0u32.to_le_bytes());
        assert!(uf2_family(&untagged).unwrap_err().contains("no family"));
        let mut bad = uf2(UF2_FAMILY_RP2040);
        bad[508] ^= 1;
        assert!(uf2_family(&bad).unwrap_err().contains("not a UF2"));
        assert!(uf2_family(&bad[..511]).is_err());
        assert!(uf2_family(&[]).is_err());
    }

    const INFO_RP2040: &str = "UF2 Bootloader v3.0\nModel: Raspberry Pi RP2\nBoard-ID: RPI-RP2\n";
    const INFO_RP2350: &str = "UF2 Bootloader v1.0\nModel: Raspberry Pi RP2350\nBoard-ID: RP2350\n";

    #[test]
    fn each_rom_takes_only_its_own_family() {
        assert!(rom_takes(INFO_RP2040, UF2_FAMILY_RP2040).is_ok());
        for family in FAMILIES_RP2350 {
            assert!(rom_takes(INFO_RP2350, family).is_ok());
            assert!(rom_takes(INFO_RP2040, family).is_err());
        }
        let err = rom_takes(INFO_RP2350, UF2_FAMILY_RP2040).unwrap_err();
        assert!(
            err.contains("0xe48bff56") && err.contains("RP2350"),
            "{err}"
        );
        // Absolute and data images would land on either ROM.
        for family in [0xe48b_ff57, 0xe48b_ff58] {
            assert!(rom_takes(INFO_RP2040, family).is_err());
            assert!(rom_takes(INFO_RP2350, family).is_err());
        }
        assert!(rom_takes("UF2 Bootloader v1.0\n", UF2_FAMILY_RP2040).is_err());
        assert!(rom_takes("Board-ID: OTHER\n", UF2_FAMILY_RP2040).is_err());
    }

    #[test]
    fn a_wrong_family_is_refused_before_the_copy() {
        let dir =
            std::env::temp_dir().join(format!("daplink-bootsel-family-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        assert!(check_family(&dir, UF2_FAMILY_RP2040).is_err());
        std::fs::write(dir.join(MARKER), INFO_RP2350).unwrap();
        assert!(check_family(&dir, FAMILIES_RP2350[0]).is_ok());
        assert!(check_family(&dir, UF2_FAMILY_RP2040).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_volume_counts_once_its_marker_is_there() {
        let dir =
            std::env::temp_dir().join(format!("daplink-bootsel-marker-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let short = Duration::from_millis(50);
        assert_eq!(wait_for_volume(short, || Ok(Some(dir.clone()))), Ok(None));
        assert_eq!(wait_for_volume(short, || Ok(None)), Ok(None));
        std::fs::write(dir.join(MARKER), b"UF2 Bootloader").unwrap();
        assert_eq!(
            wait_for_volume(short, || Ok(Some(dir.clone()))),
            Ok(Some(dir.clone()))
        );
        let mut calls = 0;
        let refused = wait_for_volume(Duration::from_secs(5), || {
            calls += 1;
            Err("ambiguous".to_string())
        });
        assert_eq!((refused, calls), (Err("ambiguous".to_string()), 1));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_drive_leaving_means_the_marker_disappears() {
        let dir =
            std::env::temp_dir().join(format!("daplink-bootsel-eject-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(MARKER), b"UF2").unwrap();
        assert!(!wait_for_path_gone(&dir, Duration::from_millis(50)));
        std::fs::remove_file(dir.join(MARKER)).unwrap();
        assert!(wait_for_path_gone(&dir, Duration::from_millis(50)));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
