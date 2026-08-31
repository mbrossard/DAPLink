//! LPC55 ROM ISP entry for the reboot-to-bootloader vendor command.

use core::sync::atomic::{AtomicBool, Ordering};

use daplink_rs::{BootloaderEntry, KIND_NXP_ISP};

/// Latched by [`Lpc55Boot::arm`], observed by [`reboot_armed`].
static REBOOT: AtomicBool = AtomicBool::new(false);

/// Whether a reboot-to-bootloader has been armed; the idle loop polls this and enters the ROM
/// after a short grace so the DAP response drains first.
pub fn reboot_armed() -> bool {
    REBOOT.load(Ordering::SeqCst)
}

/// The boot-ROM ISP entry: re-enumerates as the ROM's USB-HID ISP device (`0x1FC9:0x0021`).
pub struct Lpc55Boot;

impl BootloaderEntry for Lpc55Boot {
    const KIND: u8 = KIND_NXP_ISP;

    fn arm(&self) {
        REBOOT.store(true, Ordering::SeqCst);
    }

    fn enter(&self) -> ! {
        // Thread mode only: `boot_to_bootrom` tears down the NVIC before jumping.
        lpc55_hal::boot_to_bootrom()
    }
}
