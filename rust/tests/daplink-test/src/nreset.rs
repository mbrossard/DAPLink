//! The nreset case: the reset line the probe ships but nothing else exercises.

use std::time::Instant;

use daplink_test_protocol as proto;

use crate::util::chain;
use crate::{Ctx, Failure};

/// Stamp RAM, pulse nRESET over raw bulk, and require a fresh boot with the stamp cleared.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    let dut_elf = cx.dut_elf.clone();
    // Pre-pulse: set the stamp and confirm it through the protocol.
    {
        let session = cx
            .session
            .as_mut()
            .ok_or_else(|| Failure::Fatal("needs a successful connect".into()))?;
        let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;
        let mut rtt = crate::rtt::attach(&mut core, dut_elf.as_deref()).map_err(Failure::Fatal)?;
        crate::rtt::exchange(&mut rtt, &mut core, b"STAMP 42\n", "OK STAMP 42")
            .map_err(Failure::Fatal)?;
        let id =
            crate::rtt::exchange(&mut rtt, &mut core, b"ID\n", "OK ID").map_err(Failure::Fatal)?;
        if crate::dutid::last_stamp(&id) != Some(42) {
            return Err(Failure::Fatal(format!("stamp not confirmed (saw {id:?})")));
        }
    }

    // The raw pulse claims the USB interface, so the probe-rs session goes first.
    cx.session = None;
    if let Err(e) = crate::dap::pulse_nreset(&cx.selector) {
        if !is_transport_error(&e) {
            return Err(Failure::Fatal(e));
        }
        // USB churn during the pulse itself: try to reattach anyway, same as a reattach failure.
        return recover_reattach(cx, full_run, e, Instant::now());
    }
    let reattach_started = Instant::now();
    match crate::connect::run(cx, full_run) {
        Ok(()) => {}
        Err(Failure::Fatal(e)) => return recover_reattach(cx, full_run, e, reattach_started),
        Err(other) => return Err(other),
    }
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("reattach lost the session".into()))?;
    let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;
    let mut rtt = crate::rtt::attach(&mut core, dut_elf.as_deref()).map_err(Failure::Fatal)?;

    // No boot event means the pulse never reached the target's reset pin.
    crate::rtt::expect(&mut rtt, &mut core, proto::EVT_BOOT)
        .map_err(|e| Failure::Failed(format!("no reboot: {e}")))?;
    let id = crate::rtt::exchange(&mut rtt, &mut core, b"ID\n", "OK ID").map_err(Failure::Fatal)?;
    match crate::dutid::last_stamp(&id) {
        Some(0) => Ok(()),
        other => Err(Failure::Failed(format!(
            "stamp survived the pulse (stamp={other:?}), the target did not reset"
        ))),
    }
}

/// Reattach once more so later cases keep a session; the case still fails.
fn recover_reattach(
    cx: &mut Ctx,
    full_run: bool,
    first_err: String,
    since_pulse: Instant,
) -> Result<(), Failure> {
    let ms = since_pulse.elapsed().as_millis();
    match crate::connect::run(cx, full_run) {
        Ok(()) => Err(Failure::Failed(format!(
            "reattach failed ({ms}ms after pulse): {first_err}"
        ))),
        Err(Failure::Fatal(second_err)) => Err(Failure::Failed(format!(
            "reattach failed twice, first {ms}ms after pulse: {first_err}; retry: {second_err}"
        ))),
        Err(other) => Err(other),
    }
}

/// Whether a raw-bulk error looks like USB/host-controller churn rather than a dead reset pin.
fn is_transport_error(msg: &str) -> bool {
    const MARKERS: &[&str] = &[
        "bulk write failed",
        "bulk read failed",
        "probe not found on the bus",
        "no vendor bulk interface",
        "no such device",
        "input/output error",
        "broken pipe",
        "disconnected",
        "resource busy",
        "timed out",
    ];
    let lower = msg.to_ascii_lowercase();
    MARKERS.iter().any(|m| lower.contains(m))
}

#[cfg(test)]
mod tests {
    use super::is_transport_error;

    #[test]
    fn transport_error_matches_known_markers() {
        assert!(is_transport_error("bulk write failed: pipe error"));
        assert!(is_transport_error("bulk read failed: timeout"));
        assert!(is_transport_error("probe not found on the bus"));
        assert!(is_transport_error("No such device (os error 19)"));
        assert!(is_transport_error("Device disconnected"));
    }

    #[test]
    fn transport_error_ignores_unrelated_text() {
        assert!(!is_transport_error(
            "stamp not confirmed (saw \"OK ID stamp=1\")"
        ));
        assert!(!is_transport_error("SWD ack was FAULT"));
    }
}
