//! The nreset case: the reset line the probe ships but nothing else exercises.

use daplink_test_protocol as proto;

use crate::util::chain;
use crate::{Ctx, Failure};

/// Stamp the DUT's RAM, pulse nRESET over raw bulk, and require a fresh boot event and the
/// stamp back at zero; a dead pin fails, losing the setup is fatal.
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
        if last_stamp(&id) != Some(42) {
            return Err(Failure::Fatal(format!("stamp not confirmed (saw {id:?})")));
        }
    }

    // The raw pulse claims the USB interface, so the probe-rs session goes first.
    cx.session = None;
    crate::dap::pulse_nreset(&cx.selector).map_err(Failure::Fatal)?;
    crate::connect::run(cx, full_run)?;
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
    match last_stamp(&id) {
        Some(0) => Ok(()),
        other => Err(Failure::Failed(format!(
            "stamp survived the pulse (stamp={other:?}), the target did not reset"
        ))),
    }
}

/// The `stamp=` field of the last `ID` reply in `text`; shared with the reset case.
pub(crate) fn last_stamp(text: &str) -> Option<u32> {
    text.lines().rev().find_map(proto::id_stamp)
}
