//! The speed-jtag case: read throughput across the JTAG wire-clock ladder.

use probe_rs::probe::WireProtocol;

use crate::jtag::{reopen_swd, unroute_trace_pin};
use crate::speed;
use crate::{Ctx, Failure};

/// Unroute the shared trace pin over SWD, sweep over JTAG, then hand a fresh SWD session back.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let mut session = cx
        .session
        .take()
        .ok_or_else(|| Failure::Fatal("speed-jtag needs a successful connect".into()))?;
    let unrouted = unroute_trace_pin(&mut session, cx.dbgmcu_cr).map_err(Failure::Failed);
    drop(session);

    let result = unrouted.and_then(|()| {
        let steps = speed::sweep(&cx.selector, &cx.chip, WireProtocol::Jtag, cx.read_addr)?;
        for s in &steps {
            speed::print_step(s);
        }
        cx.last_speed = Some((WireProtocol::Jtag, steps));
        Ok(())
    });

    match reopen_swd(cx) {
        Ok(session) => cx.session = Some(session),
        Err(e) => return Err(Failure::Fatal(format!("re-open SWD: {e}"))),
    }
    result
}
