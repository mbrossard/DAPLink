//! The reset case: a debug-port core reset, distinct from nreset's pin pulse -- stamp the
//! DUT's RAM, reset through the attached session, and require a fresh boot with the stamp
//! back at zero.

use std::time::Duration;

use daplink_test_protocol as proto;

use crate::util::chain;
use crate::{Ctx, Failure};

pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let dut_elf = cx.dut_elf.clone();
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("needs a successful connect".into()))?;
    let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;

    // Pre-reset: set the stamp and confirm it through the protocol.
    {
        let mut rtt = crate::rtt::attach(&mut core, dut_elf.as_deref()).map_err(Failure::Fatal)?;
        crate::rtt::exchange(&mut rtt, &mut core, b"STAMP 42\n", "OK STAMP 42")
            .map_err(Failure::Fatal)?;
        let id =
            crate::rtt::exchange(&mut rtt, &mut core, b"ID\n", "OK ID").map_err(Failure::Fatal)?;
        if crate::nreset::last_stamp(&id) != Some(42) {
            return Err(Failure::Fatal(format!("stamp not confirmed (saw {id:?})")));
        }
    }

    // A core reset keeps the session; only RTT must be re-established afterwards.
    crate::reset(&mut core).map_err(|e| Failure::Failed(format!("core reset: {e}")))?;

    // The previous boot's stale control block lingers until the firmware re-inits it; the
    // settle shrinks the window in which the fresh attach could latch the stale one.
    std::thread::sleep(Duration::from_millis(200));
    let mut rtt = crate::rtt::attach(&mut core, dut_elf.as_deref()).map_err(Failure::Fatal)?;

    // No boot event means the reset request never took effect on the core.
    crate::rtt::expect(&mut rtt, &mut core, proto::EVT_BOOT)
        .map_err(|e| Failure::Failed(format!("no reboot: {e}")))?;
    let id = crate::rtt::exchange(&mut rtt, &mut core, b"ID\n", "OK ID").map_err(Failure::Fatal)?;
    match crate::nreset::last_stamp(&id) {
        Some(0) => Ok(()),
        other => Err(Failure::Failed(format!(
            "stamp survived the reset (stamp={other:?}), RAM was not re-initialized"
        ))),
    }
}
