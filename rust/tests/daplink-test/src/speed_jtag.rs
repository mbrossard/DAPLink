//! The speed-jtag case: read throughput across the JTAG wire-clock ladder.

use probe_rs::probe::WireProtocol;

use crate::jtag::unroute_trace_pin;
use crate::speed;
use crate::{Ctx, Failure};

/// Unroute the shared trace pin over SWD, sweep over JTAG, then hand a fresh SWD session back.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let dbgmcu_cr = cx.dbgmcu_cr;
    speed::run(cx, WireProtocol::Jtag, |session| {
        unroute_trace_pin(session, dbgmcu_cr).map_err(Failure::Failed)
    })
}
