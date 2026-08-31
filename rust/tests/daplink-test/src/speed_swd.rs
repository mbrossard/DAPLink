//! The speed-swd case: instrument read throughput across the SWD wire-clock ladder.

use probe_rs::probe::WireProtocol;

use crate::jtag::reopen_swd;
use crate::speed;
use crate::{Ctx, Failure};

/// Drop the main SWD session, which holds the probe's USB interface, sweep the ladder, then
/// hand a fresh SWD session back to `cx` whatever the outcome.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let session = cx
        .session
        .take()
        .ok_or_else(|| Failure::Fatal("speed-swd needs a successful connect".into()))?;
    drop(session);

    let result = speed::sweep(&cx.selector, &cx.chip, WireProtocol::Swd, cx.read_addr).map(
        |steps: Vec<speed::Step>| {
            for s in &steps {
                speed::print_step(s);
            }
            cx.last_speed = Some((WireProtocol::Swd, steps));
        },
    );

    match reopen_swd(cx) {
        Ok(session) => cx.session = Some(session),
        Err(e) => return Err(Failure::Fatal(format!("re-open SWD: {e}"))),
    }
    result
}
