//! The swd case: proves the wire protocol end to end, attach plus a memory roundtrip over SWD.

use std::time::Duration;

use probe_rs::config::MemoryRegion;
use probe_rs::{Core, MemoryInterface};

use crate::util::chain;
use crate::{Ctx, Failure};

const HALT_TIMEOUT: Duration = Duration::from_millis(500);

/// Stop the core, write and read back a pattern at the top of target RAM, backing the region up
/// and restoring it, and resume.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    inner(cx).map_err(Failure::Fatal)
}

fn inner(cx: &mut Ctx) -> Result<(), String> {
    let session = cx.session.as_mut().ok_or("needs a successful connect")?;
    let mut core = session.core(0).map_err(chain)?;
    roundtrip(&mut core)
}

/// Stop the core, write and read back a pattern at the top of target RAM, restoring what was
/// there, and resume. Shared, so each wire protocol proves the same access over its own attach.
pub(crate) fn roundtrip(core: &mut Core) -> Result<(), String> {
    // The top 4 KiB, away from whatever the resident target firmware uses low in RAM.
    let ram = core
        .target()
        .memory_map
        .iter()
        .find_map(|r| match r {
            MemoryRegion::Ram(r) => Some(r.range.clone()),
            _ => None,
        })
        .ok_or("target has no RAM region")?;
    let addr = ram.end - 4096;

    let pattern: Vec<u32> = (0..1024u32)
        .map(|i| i.wrapping_mul(0x9E37_79B9) ^ i)
        .collect();
    core.halt(HALT_TIMEOUT).map_err(chain)?;
    let mut saved = vec![0u32; pattern.len()];
    core.read_32(addr, &mut saved).map_err(chain)?;
    core.write_32(addr, &pattern).map_err(chain)?;
    let mut back = vec![0u32; pattern.len()];
    core.read_32(addr, &mut back).map_err(chain)?;
    core.write_32(addr, &saved).map_err(chain)?;
    core.run().map_err(chain)?;
    if let Some(i) = (0..pattern.len()).find(|&i| back[i] != pattern[i]) {
        return Err(format!(
            "mismatch at {:#x}: wrote {:#010x}, read {:#010x}",
            addr + 4 * i as u64,
            pattern[i],
            back[i]
        ));
    }
    Ok(())
}
