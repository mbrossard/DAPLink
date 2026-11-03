use probe_rs::flashing::{download_file_with_options, DownloadOptions, ElfLoader};

use crate::util::chain;
use crate::{connect, Ctx, Failure};

/// Program, verify and reset the DUT image, always programming; one reset-assisted retry on failure.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    // The DUT cases that follow run this image; without it they test nothing.
    inner(cx, full_run).map_err(Failure::Fatal)
}

fn inner(cx: &mut Ctx, _full_run: bool) -> Result<(), String> {
    let elf = cx.dut_elf.clone().ok_or("needs --dut-elf")?;
    if let Err(first) = download(cx, &elf) {
        println!("flash        download failed, retrying after a reset ({first})");
        retry(cx, &elf)
            .map_err(|second| format!("download failed ({first}); retry failed ({second})"))?;
        cx.last_note = Some("retried after a reset".to_string());
    }
    let session = cx.session.as_mut().ok_or("needs a successful connect")?;
    let mut core = session.core(0).map_err(chain)?;
    crate::reset(&mut core)
}

/// One flash attempt, verified.
fn download(cx: &mut Ctx, elf: &str) -> Result<(), String> {
    let session = cx.session.as_mut().ok_or("needs a successful connect")?;
    let mut options = DownloadOptions::default();
    options.verify = true;
    download_file_with_options(session, elf, ElfLoader(Default::default()), options).map_err(chain)
}

/// Reset the core, rebuilding the session first if it was lost, then download once more.
fn retry(cx: &mut Ctx, elf: &str) -> Result<(), String> {
    let core_reachable = cx.session.as_mut().is_some_and(|s| s.core(0).is_ok());
    match retry_plan(core_reachable) {
        RetryPlan::ResetOnly => {
            let session = cx.session.as_mut().ok_or("needs a successful connect")?;
            let mut core = session.core(0).map_err(chain)?;
            crate::reset(&mut core)?;
        }
        RetryPlan::Reattach => {
            cx.session = None;
            connect::run(cx, false).map_err(|e| match e {
                Failure::Failed(e) | Failure::Fatal(e) | Failure::Skip(e) => e,
            })?;
        }
    }
    download(cx, elf)
}

/// Whether a failed download leaves the session usable for a plain core reset.
#[derive(Debug, PartialEq)]
enum RetryPlan {
    ResetOnly,
    Reattach,
}

fn retry_plan(core_reachable: bool) -> RetryPlan {
    if core_reachable {
        RetryPlan::ResetOnly
    } else {
        RetryPlan::Reattach
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_surviving_session_only_needs_a_reset() {
        assert_eq!(retry_plan(true), RetryPlan::ResetOnly);
    }

    #[test]
    fn a_dead_session_needs_a_fresh_attach() {
        assert_eq!(retry_plan(false), RetryPlan::Reattach);
    }
}
