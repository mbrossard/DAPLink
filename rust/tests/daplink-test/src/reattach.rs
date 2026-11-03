//! The reattach case: a USB reset, then the probe must answer; `reattach-suspend` first runtime-suspends it (Linux).

use std::time::{Duration, Instant};

use nusb::{ErrorKind, MaybeFuture};

use crate::{Ctx, Failure};

/// How long the probe is given to come back and answer; the budget the case asserts.
const REATTACH_TIMEOUT: Duration = Duration::from_secs(5);
const POLL_INTERVAL: Duration = Duration::from_millis(100);

/// Reset the probe's USB device, then require `DAP_Info` to answer inside the budget.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let device = crate::dap::open_device(&cx.selector).map_err(Failure::Fatal)?;
    reset(device, "reattach")?;

    await_answer("a USB reset", || {
        let version = crate::dap::open(&cx.selector).and_then(|mut dap| dap.dap_info(0x04))?;
        if version.is_empty() {
            return Err("the probe answered DAP_Info with an empty version".into());
        }
        Ok(())
    })
}

/// What a failed USB reset means for the case.
#[derive(Debug, PartialEq)]
enum ResetError {
    /// The backend has no reset (WinUSB): a host gap.
    Skip,
    /// Linux answers ENODEV when re-enumeration failed, then re-enumerates the device itself.
    Reenumerating,
    Failed,
}

fn reset_error_verdict(kind: ErrorKind) -> ResetError {
    match kind {
        ErrorKind::Unsupported => ResetError::Skip,
        ErrorKind::Disconnected => ResetError::Reenumerating,
        _ => ResetError::Failed,
    }
}

/// Reset the device and release it; a device the kernel is re-enumerating is polled like any other.
fn reset(device: nusb::Device, case: &str) -> Result<(), Failure> {
    let result = device.reset().wait();
    drop(device);
    match result {
        Ok(()) => Ok(()),
        Err(e) => match reset_error_verdict(e.kind()) {
            ResetError::Skip => Err(Failure::Skip(format!("{case}: no USB reset here ({e})"))),
            ResetError::Reenumerating => Ok(()),
            ResetError::Failed => Err(Failure::Failed(format!(
                "{case}: the USB reset failed: {e}"
            ))),
        },
    }
}

/// One poll of the answer loop, judged against the budget whichever way it went.
enum Step<T> {
    Answered(T),
    Late(Duration),
    Expired(String),
    Retry,
}

fn step<T>(outcome: Result<T, String>, took: Duration) -> Step<T> {
    match outcome {
        Ok(v) if took <= REATTACH_TIMEOUT => Step::Answered(v),
        Ok(_) => Step::Late(took),
        Err(e) if took >= REATTACH_TIMEOUT => Step::Expired(e),
        Err(_) => Step::Retry,
    }
}

/// Poll `try_once` until the probe answers; a reply after the budget fails like no reply at all.
fn await_answer<T>(
    after: &str,
    mut try_once: impl FnMut() -> Result<T, String>,
) -> Result<T, Failure> {
    let started = Instant::now();
    loop {
        let outcome = try_once();
        let took = started.elapsed();
        if outcome.is_ok() || took >= REATTACH_TIMEOUT {
            crate::wait::note("reattach", REATTACH_TIMEOUT, took);
        }
        match step(outcome, took) {
            Step::Answered(v) => return Ok(v),
            Step::Late(took) => {
                return Err(Failure::Failed(format!(
                    "the probe answered {:.1}s after {after}, past the {}s budget",
                    took.as_secs_f32(),
                    REATTACH_TIMEOUT.as_secs()
                )))
            }
            Step::Expired(e) => {
                return Err(Failure::Failed(format!(
                    "the probe did not answer within {}s of {after}: {e}",
                    REATTACH_TIMEOUT.as_secs()
                )))
            }
            Step::Retry => std::thread::sleep(POLL_INTERVAL),
        }
    }
}

/// Sent before the suspend and left unread, so an uncleared reply shows after the reset.
#[cfg(target_os = "linux")]
const SUSPEND_PRE_A: &[u8] = &[0x01, 0x00, 0x00]; // DAP_HostStatus(Connect, false)
#[cfg(target_os = "linux")]
const SUSPEND_PRE_B: &[u8] = &[0x04, 0x00, 0x64, 0x00, 0x00, 0x00]; // DAP_TransferConfigure
/// Sent after the reset; a leftover reply would carry 0x01 or 0x04 instead of 0x00.
#[cfg(target_os = "linux")]
const SUSPEND_POST: &[u8] = &[0x00, 0xFE]; // DAP_Info(PacketCount)

/// How long past `autosuspend_delay_ms` to wait before polling `runtime_status`.
#[cfg(target_os = "linux")]
const SUSPEND_MARGIN: Duration = Duration::from_millis(500);
/// Bound on waiting for `runtime_status` to read `suspended`.
#[cfg(target_os = "linux")]
const SUSPEND_TIMEOUT: Duration = Duration::from_secs(5);

/// Restores sysfs `power/control` to its original value whatever the verdict.
#[cfg(target_os = "linux")]
struct RestoreControl {
    path: std::path::PathBuf,
    original: String,
}

#[cfg(target_os = "linux")]
impl Drop for RestoreControl {
    fn drop(&mut self) {
        let _ = std::fs::write(&self.path, &self.original);
    }
}

/// Runtime-suspend the probe via sysfs, reset it, and require the first reply to answer the new request.
#[cfg(target_os = "linux")]
pub(crate) fn run_suspend(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let sysfs = device_sysfs_path(&cx.selector).map_err(Failure::Fatal)?;
    let control = sysfs.join("power/control");
    let status = sysfs.join("power/runtime_status");
    let delay_ms = match std::fs::read_to_string(sysfs.join("power/autosuspend_delay_ms"))
        .ok()
        .and_then(|s| parse_delay_ms(&s))
    {
        None => {
            return Err(Failure::Skip(
                "reattach-suspend: no power/autosuspend_delay_ms here".into(),
            ))
        }
        Some(ms) if ms < 0 => {
            return Err(Failure::Skip(
                "reattach-suspend: autosuspend is disabled for this device".into(),
            ))
        }
        Some(ms) => ms,
    };
    let original = std::fs::read_to_string(&control)
        .map_err(|e| Failure::Skip(format!("reattach-suspend: no power/control here ({e})")))?;
    // A SCSI disk is runtime-PM-forbidden at add time, and an active child keeps the probe awake.
    if let Some(disk) = forbidden_scsi_disk(&sysfs) {
        return Err(Failure::Skip(format!(
            "reattach-suspend: the mass-storage disk {} is runtime-PM-forbidden, so the host cannot \
             suspend the probe",
            disk.display()
        )));
    }

    // Queue two distinguishable, session-free requests without reading them.
    {
        let mut dap = crate::dap::open(&cx.selector).map_err(Failure::Fatal)?;
        dap.send(SUSPEND_PRE_A, Duration::from_secs(1))
            .map_err(Failure::Failed)?;
        dap.send(SUSPEND_PRE_B, Duration::from_secs(1))
            .map_err(Failure::Failed)?;
        // Dropped here, releasing the interface so the host can suspend it.
    }

    let _restore = RestoreControl {
        path: control.clone(),
        original,
    };
    if let Err(e) = std::fs::write(&control, "auto") {
        println!(
            "WARNING: reattach-suspend needs write access to {}; \
             see docs/reference/linux-udev.md for the udev rule.",
            control.display()
        );
        return Err(Failure::Skip(format!(
            "reattach-suspend: power/control not writable ({e})"
        )));
    }
    std::thread::sleep(Duration::from_millis(delay_ms.max(0) as u64) + SUSPEND_MARGIN);

    let waited = Instant::now();
    let deadline = waited + SUSPEND_TIMEOUT;
    loop {
        if std::fs::read_to_string(&status)
            .map(|s| is_suspended(&s))
            .unwrap_or(false)
        {
            crate::wait::note("reattach.suspend", SUSPEND_TIMEOUT, waited.elapsed());
            break;
        }
        if Instant::now() >= deadline {
            return Err(Failure::Skip(format!(
                "reattach-suspend: the host never suspended the probe (runtime_usage {}, \
                 runtime_active_kids {})",
                sysfs_value(&sysfs.join("power/runtime_usage")),
                sysfs_value(&sysfs.join("power/runtime_active_kids"))
            )));
        }
        std::thread::sleep(POLL_INTERVAL);
    }

    let device = crate::dap::open_device(&cx.selector).map_err(Failure::Fatal)?;
    reset(device, "reattach-suspend")?;

    let reply = await_answer("the post-suspend reset", || {
        crate::dap::open(&cx.selector).and_then(|mut dap| dap.command(SUSPEND_POST))
    })?;
    if reply.first() == Some(&0x00) {
        Ok(())
    } else {
        Err(Failure::Failed(format!(
            "the post-reset reply {reply:02x?} echoes a request queued before the suspend, not \
             the one just sent"
        )))
    }
}

/// A host with no Linux runtime power management to suspend the probe through.
#[cfg(not(target_os = "linux"))]
pub(crate) fn run_suspend(_cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    Err(Failure::Skip(
        "reattach-suspend: needs Linux runtime power management (sysfs power/control)".into(),
    ))
}

/// This device's sysfs directory, matched the same way every raw-bulk case finds it on the bus.
#[cfg(target_os = "linux")]
fn device_sysfs_path(
    selector: &probe_rs::probe::DebugProbeSelector,
) -> Result<std::path::PathBuf, String> {
    crate::dap::find_device(selector).map(|d| d.sysfs_path().to_path_buf())
}

/// The first SCSI device under this USB device whose `power/control` reads `on`, relative to `sysfs`.
#[cfg(target_os = "linux")]
fn forbidden_scsi_disk(sysfs: &std::path::Path) -> Option<std::path::PathBuf> {
    let dirs = |p: &std::path::Path, prefix: &str| -> Vec<std::path::PathBuf> {
        std::fs::read_dir(p)
            .map(|rd| {
                rd.flatten()
                    .map(|e| e.path())
                    .filter(|p| {
                        p.file_name()
                            .is_some_and(|n| n.to_string_lossy().starts_with(prefix))
                    })
                    .collect()
            })
            .unwrap_or_default()
    };
    let own = sysfs.file_name()?.to_string_lossy().into_owned();
    for interface in dirs(sysfs, &format!("{own}:")) {
        for host in dirs(&interface, "host") {
            for target in dirs(&host, "target") {
                for disk in dirs(&target, "") {
                    let Ok(control) = std::fs::read_to_string(disk.join("power/control")) else {
                        continue;
                    };
                    if control.trim() == "on" {
                        return disk.strip_prefix(sysfs).ok().map(Into::into);
                    }
                }
            }
        }
    }
    None
}

/// A sysfs attribute's text, or `?` when it cannot be read.
#[cfg(target_os = "linux")]
fn sysfs_value(path: &std::path::Path) -> String {
    std::fs::read_to_string(path)
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|_| "?".into())
}

/// Whether sysfs `power/runtime_status` reads as fully suspended.
#[cfg(target_os = "linux")]
fn is_suspended(status: &str) -> bool {
    status.trim() == "suspended"
}

/// Parse sysfs `power/autosuspend_delay_ms`; negative means autosuspend is disabled.
#[cfg(target_os = "linux")]
fn parse_delay_ms(text: &str) -> Option<i64> {
    text.trim().parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Only a backend without a reset skips; Linux's ENODEV is a re-enumeration to wait out.
    #[test]
    fn only_an_unsupported_reset_is_a_skip() {
        assert_eq!(
            reset_error_verdict(ErrorKind::Unsupported),
            ResetError::Skip
        );
        assert_eq!(
            reset_error_verdict(ErrorKind::Disconnected),
            ResetError::Reenumerating
        );
        assert_eq!(reset_error_verdict(ErrorKind::Busy), ResetError::Failed);
        assert_eq!(reset_error_verdict(ErrorKind::Other), ResetError::Failed);
    }

    /// A late answer is judged against the budget.
    #[test]
    fn a_late_answer_is_not_a_pass() {
        let s = Duration::from_millis;
        assert!(matches!(step(Ok(()), s(4_900)), Step::Answered(())));
        assert!(matches!(step(Ok(()), s(8_746)), Step::Late(t) if t == s(8_746)));
        assert!(matches!(step(Ok(()), s(19_166)), Step::Late(_)));
        assert!(matches!(step::<()>(Err("x".into()), s(1_000)), Step::Retry));
        assert!(matches!(step::<()>(Err("x".into()), s(5_000)), Step::Expired(e) if e == "x"));
    }
}

#[cfg(all(test, target_os = "linux"))]
mod linux_tests {
    use super::*;

    #[test]
    fn runtime_status_matches_only_the_suspended_state() {
        assert!(is_suspended("suspended\n"));
        assert!(!is_suspended("active\n"));
        assert!(!is_suspended("unsupported\n"));
    }

    #[test]
    fn a_negative_delay_means_autosuspend_is_disabled() {
        assert_eq!(parse_delay_ms("2000\n"), Some(2000));
        assert_eq!(parse_delay_ms("-1\n"), Some(-1));
        assert_eq!(parse_delay_ms("garbage"), None);
    }

    /// The disk is found under `<dev>:1.<n>/host*/target*/<h:c:t:l>`, and only when forbidden.
    #[test]
    fn a_forbidden_disk_is_found_under_its_interface() {
        let root = std::env::temp_dir().join(format!("reattach-sysfs-{}", std::process::id()));
        let dev = root.join("1-2.3");
        let disk = dev.join("1-2.3:1.2/host4/target4:0:0/4:0:0:0");
        std::fs::create_dir_all(disk.join("power")).unwrap();
        std::fs::create_dir_all(dev.join("1-2.3:1.0/power")).unwrap();
        std::fs::create_dir_all(dev.join("1-2.3:1.2/host4/target4:0:0/power")).unwrap();
        std::fs::write(dev.join("1-2.3:1.2/host4/target4:0:0/uevent"), "").unwrap();
        std::fs::write(dev.join("1-2.3:1.0/power/control"), "on\n").unwrap();
        std::fs::write(disk.join("power/control"), "auto\n").unwrap();
        assert_eq!(forbidden_scsi_disk(&dev), None);
        std::fs::write(disk.join("power/control"), "on\n").unwrap();
        assert_eq!(
            forbidden_scsi_disk(&dev).as_deref(),
            Some(std::path::Path::new("1-2.3:1.2/host4/target4:0:0/4:0:0:0"))
        );
        std::fs::remove_dir_all(&root).ok();
    }
}
