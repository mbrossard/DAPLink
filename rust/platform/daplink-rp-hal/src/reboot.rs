//! RP2040 ROM bootloader entry for the reboot-to-bootloader vendor command.

use core::sync::atomic::{AtomicBool, Ordering};

use daplink_rs::{BootloaderEntry, DelayNs, KIND_UF2};

use crate::delay::CycleDelay;

/// Latched by [`RpUsbBoot::arm`], observed by [`reboot_watch`]. Plain load/store only (no CAS),
/// so it is sound on the RP2040's thumbv6m core without `portable-atomic`.
static REBOOT: AtomicBool = AtomicBool::new(false);

/// Grace before entering the ROM, so the host reads the acknowledgement before USB resets.
const REBOOT_GRACE_NS: u32 = 50_000_000;

/// Whether a reboot-to-bootloader has been armed by the vendor handler.
pub fn reboot_armed() -> bool {
    REBOOT.load(Ordering::SeqCst)
}

/// The RP2040 boot-ROM BOOTSEL entry: restarts the chip as the RPI-RP2 UF2 device.
pub struct RpUsbBoot;

impl BootloaderEntry for RpUsbBoot {
    const KIND: u8 = KIND_UF2;

    fn arm(&self) {
        REBOOT.store(true, Ordering::SeqCst);
    }

    fn enter(&self) -> ! {
        // Both masks zero: no activity LED, and both the mass-storage and PICOBOOT interfaces kept.
        rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
        loop {
            core::hint::spin_loop();
        }
    }
}

/// Idle-loop reboot trigger: call once per `#[idle]` iteration. The vendor handler runs in the USB
/// ISR and must not enter the ROM there; once armed this waits the grace out and never returns.
pub fn reboot_watch() {
    if reboot_armed() {
        CycleDelay.delay_ns(REBOOT_GRACE_NS);
        RpUsbBoot.enter();
    }
}
