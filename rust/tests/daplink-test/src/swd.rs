//! The swd case: proves the wire protocol end to end, attach plus a memory roundtrip over SWD.

use std::time::Duration;

use probe_rs::config::MemoryRegion;
use probe_rs::{Core, MemoryInterface};

use crate::util::chain;
use crate::{Ctx, Failure};

pub(crate) const HALT_TIMEOUT: Duration = Duration::from_millis(500);
/// Bytes of target RAM a case may pattern, halted and restored.
pub(crate) const SCRATCH_BYTES: usize = 4096;

/// Pattern-test the top of target RAM over SWD, restoring it.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    // Every later case rides on this wire, so an unreliable one is lost substrate.
    inner(cx).map_err(Failure::Fatal)
}

fn inner(cx: &mut Ctx) -> Result<(), String> {
    let session = cx.session.as_mut().ok_or("needs a successful connect")?;
    let mut core = session.core(0).map_err(chain)?;
    roundtrip(&mut core)
}

/// The top [`SCRATCH_BYTES`] of the first RAM region, away from the resident firmware.
pub(crate) fn scratch(core: &Core) -> Result<u64, String> {
    let ram = core
        .target()
        .memory_map
        .iter()
        .find_map(|r| match r {
            MemoryRegion::Ram(r) => Some(r.range.clone()),
            _ => None,
        })
        .ok_or("target has no RAM region")?;
    Ok(ram.end - SCRATCH_BYTES as u64)
}

/// Halt, pattern-test and restore the top of target RAM, then resume; shared by each wire protocol.
pub(crate) fn roundtrip(core: &mut Core) -> Result<(), String> {
    let addr = scratch(core)?;
    let pattern: Vec<u32> = (0..(SCRATCH_BYTES / 4) as u32)
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
