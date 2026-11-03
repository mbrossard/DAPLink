//! STM32 system-memory DFU entry: a magic word in no-init RAM, latched across a system reset.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

use daplink_rs::{BootloaderEntry, KIND_ST_DFU};

/// Magic word marking a pending reboot-to-DFU across the reset.
const REBOOT_MAGIC: u32 = 0xB001_10AD;

/// A cell in `.uninit`, which the runtime does not zero, so the value survives the reset.
struct NoInitCell(UnsafeCell<u32>);

// SAFETY: written just before a system reset and read from `#[pre_init]`; never concurrent.
unsafe impl Sync for NoInitCell {}

#[link_section = ".uninit.DAPLINK_REBOOT_MAGIC"]
static MAGIC_CELL: NoInitCell = NoInitCell(UnsafeCell::new(0));

/// Latched by [`StmSystemDfu::arm`], observed by [`reboot_armed`]; zeroed each boot.
static ARMED: AtomicBool = AtomicBool::new(false);

/// Whether a reboot has been armed; the caller then calls [`StmSystemDfu::enter`].
pub fn reboot_armed() -> bool {
    ARMED.load(Ordering::SeqCst)
}

pub struct StmSystemDfu;

impl BootloaderEntry for StmSystemDfu {
    const KIND: u8 = KIND_ST_DFU;

    fn arm(&self) {
        ARMED.store(true, Ordering::SeqCst);
    }

    fn enter(&self) -> ! {
        // SAFETY: see `NoInitCell` -- this write is immediately followed by a reset.
        unsafe {
            core::ptr::write_volatile(MAGIC_CELL.0.get(), REBOOT_MAGIC);
        }
        cortex_m::peripheral::SCB::sys_reset()
    }
}

/// If a reboot-to-bootloader was requested, clear it and jump to the bootloader at `base`.
///
/// # Safety
/// Call only from `#[pre_init]`, with `base` a valid vector table.
pub unsafe fn check_and_enter_bootloader(base: u32) {
    if core::ptr::read_volatile(MAGIC_CELL.0.get()) == REBOOT_MAGIC {
        core::ptr::write_volatile(MAGIC_CELL.0.get(), 0);
        cortex_m::asm::bootload(base as *const u32);
    }
}
