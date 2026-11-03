//! The fault case: an unmapped read must fault and the link must recover.

use probe_rs::architecture::arm::dp::{DpAddress, DpRegisterAddress};
use probe_rs::MemoryInterface;

use crate::util::chain;
use crate::{Ctx, Failure};

const CTRL_STAT: DpRegisterAddress = DpRegisterAddress {
    address: 0x4,
    bank: None,
};
/// DP DPIDR, and the ABORT register that shares its address on writes.
const DPIDR: DpRegisterAddress = DpRegisterAddress {
    address: 0x0,
    bank: None,
};

/// CTRL/STAT's sticky flags: STICKYORUN, STICKYCMP, STICKYERR, WDATAERR.
const STICKY_MASK: u32 = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 7);

/// ABORT's sticky clears, without DAPABORT (nothing is stuck in WAIT) or STKCMPCLR (MINDP-unsafe).
const ABORT_CLEAR_ALL: u32 = (1 << 2) | (1 << 3) | (1 << 4);

/// Read an unmapped address, then require the link back: flags cleared, DPIDR, an ordinary read.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let fault_addr = cx
        .fault_addr
        .ok_or_else(|| Failure::Skip("fault: no unmapped address known for this target".into()))?;
    let read_addr = cx.read_addr;
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("fault needs a successful connect".into()))?;

    {
        let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;
        match core.read_word_32(fault_addr) {
            Ok(word) => {
                return Err(Failure::Failed(format!(
                    "the read at the unmapped {fault_addr:#012x} returned {word:#010x} instead of \
                     faulting: an unanswerable read must not look like data"
                )))
            }
            // Recorded, not judged: which error an unmapped read earns is not pinned yet.
            Err(e) => cx.last_note = Some(format!("unmapped read: {}", chain(e))),
        }
    }

    let interface = session
        .get_arm_interface()
        .map_err(|e| Failure::Fatal(chain(e)))?;
    let dp = interface.current_debug_port().unwrap_or(DpAddress::Default);

    // probe-rs may already have cleared the fault, so this write is recovery, not the assertion.
    interface
        .write_raw_dp_register(dp, DPIDR, ABORT_CLEAR_ALL)
        .map_err(|e| Failure::Failed(format!("ABORT write: {}", chain(e))))?;
    let ctrl = interface
        .read_raw_dp_register(dp, CTRL_STAT)
        .map_err(|e| Failure::Failed(format!("CTRL/STAT read after ABORT: {}", chain(e))))?;
    if ctrl & STICKY_MASK != 0 {
        return Err(Failure::Failed(format!(
            "CTRL/STAT is {ctrl:#010x} after the ABORT write: sticky flags \
             {:#x} survived the clear",
            ctrl & STICKY_MASK
        )));
    }

    let dpidr = interface
        .read_raw_dp_register(dp, DPIDR)
        .map_err(|e| Failure::Failed(format!("DPIDR read after the fault: {}", chain(e))))?;
    if !dpidr_is_plausible(dpidr) {
        return Err(Failure::Failed(format!(
            "DPIDR reads {dpidr:#010x} after the fault; the debug port did not recover"
        )));
    }

    // The port answering is not the same as memory access working again.
    let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;
    core.read_word_32(read_addr).map_err(|e| {
        Failure::Failed(format!(
            "the read at the mapped {read_addr:#012x} still fails after recovery: {}",
            chain(e)
        ))
    })?;
    Ok(())
}

/// A DPIDR that could be a real one: bit 0 is RAO, and the two rail values mean a silent bus.
fn dpidr_is_plausible(dpidr: u32) -> bool {
    dpidr & 1 == 1 && dpidr != u32::MAX
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dpidr_rails_are_not_identities() {
        // Cortex-M4 with a JTAG-DP, and an M0+ SW-DP: both real, both accepted.
        assert!(dpidr_is_plausible(0x2BA0_1477));
        assert!(dpidr_is_plausible(0x0BC1_2477));
        // A floating wire reads all ones; a dead one reads zero. Neither is an identity.
        assert!(!dpidr_is_plausible(0xFFFF_FFFF));
        assert!(!dpidr_is_plausible(0x0000_0000));
    }

    /// The clear mask must carry neither DAPABORT nor STKCMPCLR.
    #[test]
    fn the_abort_write_clears_flags_without_aborting_a_transaction() {
        assert_eq!(ABORT_CLEAR_ALL, 0x1C);
        assert_eq!(ABORT_CLEAR_ALL & 1, 0, "DAPABORT must stay clear");
        assert_eq!(ABORT_CLEAR_ALL & 2, 0, "STKCMPCLR must stay clear");
    }
}
