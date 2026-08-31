//! The flash case.

use probe_rs::flashing::{download_file_with_options, DownloadOptions, ElfLoader};

use crate::util::chain;
use crate::{Ctx, Failure};

/// Program the DUT image through the probe, verify the readback, and reset the target; always
/// programs, since a skipped identical image proves nothing about the flash path.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    inner(cx, full_run).map_err(Failure::Fatal)
}

fn inner(cx: &mut Ctx, _full_run: bool) -> Result<(), String> {
    let elf = cx.dut_elf.clone().ok_or("needs --dut-elf")?;
    let session = cx.session.as_mut().ok_or("needs a successful connect")?;
    let mut options = DownloadOptions::default();
    options.verify = true;
    download_file_with_options(session, &elf, ElfLoader(Default::default()), options)
        .map_err(chain)?;
    let mut core = session.core(0).map_err(chain)?;
    crate::reset(&mut core)
}
