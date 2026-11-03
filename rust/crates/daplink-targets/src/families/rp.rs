//! Quiescing a running RP2040 or RP2350 application before its flash is reprogrammed.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::ResetOps;

// The algorithm must also run with interrupts masked, which the RP family features turn on.
const _: () = assert!(daplink_flash::swd_host::MASK_INTERRUPTS);

/// Offsets of a peripheral register's atomic set and clear aliases.
const SET: u32 = 0x2000;
const CLR: u32 = 0x3000;
/// `WATCHDOG.CTRL.ENABLE`.
const WATCHDOG_ENABLE: u32 = 1 << 30;
/// `PSM.FRCE_OFF`.
const PSM_FRCE_OFF: u32 = 0x4;
/// `RESETS.RESET` and `RESETS.RESET_DONE`, and their DMA bit.
const RESETS_RESET: u32 = 0x0;
const RESETS_DONE: u32 = 0x8;
const RESETS_DMA: u32 = 1 << 2;
/// `DEMCR` and its `VC_HARDERR` bit.
const DEMCR: u32 = 0xE000_EDFC;
pub(crate) const VC_HARDERR: u32 = 1 << 10;
/// `MPU_CTRL`; `DHCSR.S_REGRDY`; `DCRDR`; `DCRSR`, its write bit and its CONTROL-and-masks selector.
const MPU_CTRL: u32 = 0xE000_ED94;
const DHCSR: u32 = 0xE000_EDF0;
const S_REGRDY: u32 = 1 << 16;
const DCRDR: u32 = 0xE000_EDF8;
const DCRSR: u32 = 0xE000_EDF4;
const DCRSR_WRITE: u32 = 1 << 16;
const CONTROL_MASKS: u32 = 0x14;
/// Reads allowed for a reset bit to show; each is a whole SWD transaction.
const POLLS: u32 = 64;

/// One chip's `WATCHDOG`, `PSM` and `RESETS` bases, its `PSM` core-1 bit, and its `DEMCR` value.
pub(crate) struct Blocks {
    pub(crate) watchdog: u32,
    pub(crate) psm: u32,
    pub(crate) psm_proc1: u32,
    pub(crate) resets: u32,
    pub(crate) demcr: u32,
}

/// Arm the fault catch, stop the watchdog, send core 1 to the bootrom and reset DMA, core 0 halted.
pub(crate) fn quiesce(ops: &mut dyn ResetOps, blocks: &Blocks) -> Result<(), Error> {
    ops.write_word(DEMCR, blocks.demcr)?;
    ops.write_word(blocks.watchdog + CLR, WATCHDOG_ENABLE)?;
    ops.write_word(blocks.psm + PSM_FRCE_OFF + SET, blocks.psm_proc1)?;
    // Read back, not `PSM.DONE`: no datasheet states `DONE` under a forced reset.
    let core1_off = poll(
        ops,
        blocks.psm + PSM_FRCE_OFF,
        blocks.psm_proc1,
        blocks.psm_proc1,
    );
    // Released even after a failed poll: the core-0 reset that follows would leave core 1 off.
    ops.write_word(blocks.psm + PSM_FRCE_OFF + CLR, blocks.psm_proc1)?;
    core1_off?;
    ops.write_word(blocks.resets + RESETS_RESET + SET, RESETS_DMA)?;
    ops.write_word(blocks.resets + RESETS_RESET + CLR, RESETS_DMA)?;
    poll(ops, blocks.resets + RESETS_DONE, RESETS_DMA, RESETS_DMA)
}

/// Clear what an app's privilege and MPU setup would fault the algorithm on; no reset does it here.
pub(crate) fn normalise(ops: &mut dyn ResetOps) -> Result<(), Error> {
    ops.write_word(MPU_CTRL, 0)?;
    clear_core_reg(ops, CONTROL_MASKS)
}

/// Zero the core register `DCRSR` selects with `reg`.
pub(crate) fn clear_core_reg(ops: &mut dyn ResetOps, reg: u32) -> Result<(), Error> {
    ops.write_word(DCRDR, 0)?;
    ops.write_word(DCRSR, DCRSR_WRITE | reg)?;
    poll(ops, DHCSR, S_REGRDY, S_REGRDY)
}

/// Read `addr` until its `mask` bits equal `want`.
fn poll(ops: &mut dyn ResetOps, addr: u32, mask: u32, want: u32) -> Result<(), Error> {
    for _ in 0..POLLS {
        if ops.read_word(addr)? & mask == want {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}
