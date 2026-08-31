//! The jtag case: proves the memory roundtrip over JTAG, on the probe's JTDO/JTAG chain.

use probe_rs::probe::list::Lister;
use probe_rs::probe::WireProtocol;
use probe_rs::{MemoryInterface, Permissions};

use crate::util::chain;
use crate::{swd, Ctx, Failure};

/// Unroute the trace pin JTDO shares, run the shared roundtrip over JTAG, then hand a fresh SWD
/// session back to `cx` whatever the outcome.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let mut session = cx
        .session
        .take()
        .ok_or_else(|| Failure::Fatal("jtag needs a successful connect".into()))?;
    let unrouted = unroute_trace_pin(&mut session, cx.dbgmcu_cr).map_err(Failure::Failed);
    drop(session);

    let result = unrouted.and_then(|()| attempt(cx));

    match reopen_swd(cx) {
        Ok(session) => cx.session = Some(session),
        Err(e) => return Err(Failure::Fatal(format!("re-open SWD: {e}"))),
    }
    result
}

/// Clear DBGMCU_CR TRACE_IOEN where the target has that register: its trace pin carries JTDO
/// too, and trace routing the swo case left enabled would break the JTAG chain. A target
/// without the register shares no pin, so there is nothing to unroute -- and no write to land
/// on whatever else its address might mean there.
pub(crate) fn unroute_trace_pin(
    session: &mut probe_rs::Session,
    dbgmcu_cr: Option<u64>,
) -> Result<(), String> {
    let Some(addr) = dbgmcu_cr else {
        return Ok(());
    };
    let mut core = session.core(0).map_err(chain)?;
    let cr = core
        .read_word_32(addr)
        .map_err(|e| format!("unroute trace pin: {}", chain(e)))?;
    core.write_word_32(addr, cr & !(1 << 5))
        .map_err(|e| format!("unroute trace pin: {}", chain(e)))?;
    Ok(())
}

/// Open a JTAG session to the same probe/chip and run the shared roundtrip on core 0.
fn attempt(cx: &Ctx) -> Result<(), Failure> {
    let mut probe = Lister::new()
        .open(cx.selector.clone())
        .map_err(|e| Failure::Failed(format!("JTAG attach: {}", chain(e))))?;
    probe
        .select_protocol(WireProtocol::Jtag)
        .map_err(|e| Failure::Failed(format!("JTAG attach: {}", chain(e))))?;
    let mut session = probe
        .attach(&cx.chip, Permissions::default())
        .map_err(|e| Failure::Failed(format!("JTAG attach: {}", chain(e))))?;
    let mut core = session
        .core(0)
        .map_err(|e| Failure::Failed(format!("JTAG attach: {}", chain(e))))?;
    swd::roundtrip(&mut core).map_err(|e| Failure::Failed(format!("JTAG roundtrip: {e}")))
}

/// Re-open the SWD session, which later cases depend on as their substrate.
pub(crate) fn reopen_swd(cx: &Ctx) -> Result<probe_rs::Session, String> {
    let probe = Lister::new().open(cx.selector.clone()).map_err(chain)?;
    probe
        .attach(&cx.chip, Permissions::default())
        .map_err(chain)
}
