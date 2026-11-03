//! The speed-swd case: instrument read throughput across the SWD wire-clock ladder.

use probe_rs::probe::WireProtocol;

use crate::speed;
use crate::{Ctx, Failure};

/// Drop the main session, sweep the ladder, then always hand a fresh SWD session back.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    speed::run(cx, WireProtocol::Swd, |_| Ok(()))
}
