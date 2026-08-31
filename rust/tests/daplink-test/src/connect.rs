//! The connect case.

use probe_rs::probe::list::Lister;
use probe_rs::Permissions;

use crate::util::chain;
use crate::{Ctx, Failure};

/// Open the probe, attach to the target through it, and reach its core.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    inner(cx, full_run).map_err(Failure::Fatal)
}

fn inner(cx: &mut Ctx, _full_run: bool) -> Result<(), String> {
    let probe = Lister::new().open(cx.selector.clone()).map_err(chain)?;
    let mut session = probe
        .attach(&cx.chip, Permissions::default())
        .map_err(chain)?;
    session.core(0).map_err(chain)?;
    cx.session = Some(session);
    Ok(())
}
