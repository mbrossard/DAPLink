//! RP-family ROM bootloader entry for the reboot-to-bootloader vendor command.

use core::sync::atomic::{AtomicBool, Ordering};

use daplink_rs::{BootloaderEntry, KIND_UF2};
use embassy_time::Timer;

/// Latched by [`RpUsbBoot::arm`], observed by [`reboot_armed`]. Plain load/store only (no CAS),
/// so it is sound on the RP2040's thumbv6m core without `portable-atomic`.
static REBOOT: AtomicBool = AtomicBool::new(false);

/// Grace before entering the ROM, so the host reads the acknowledgement before USB resets.
const REBOOT_GRACE_MS: u64 = 50;
/// How often [`reboot_watch`] samples the armed flag while idle.
const REBOOT_POLL_MS: u64 = 25;

/// Whether a reboot-to-bootloader has been armed by the vendor handler.
pub fn reboot_armed() -> bool {
    REBOOT.load(Ordering::SeqCst)
}

/// The RP-family boot-ROM BOOTSEL entry: restarts the chip as the RPI-RP2 UF2 device. The
/// `embassy-rp` name is chip-agnostic, so one type serves both the Pico and the Pico 2.
pub struct RpUsbBoot;

impl BootloaderEntry for RpUsbBoot {
    const KIND: u8 = KIND_UF2;

    fn arm(&self) {
        REBOOT.store(true, Ordering::SeqCst);
    }

    fn enter(&self) -> ! {
        // Both masks zero: no activity LED, and both the mass-storage and PICOBOOT interfaces kept.
        embassy_rp::rom_data::reset_to_usb_boot(0, 0);
        loop {
            core::hint::spin_loop();
        }
    }
}

/// Thread-mode reboot trigger: wait for a reboot to be armed, wait the grace out so the DAP
/// response drains, then enter the ROM. Spawned as a task, keeping the ROM entry out of the ISR.
#[embassy_executor::task]
pub async fn reboot_watch() -> ! {
    loop {
        if reboot_armed() {
            Timer::after_millis(REBOOT_GRACE_MS).await;
            RpUsbBoot.enter();
        }
        Timer::after_millis(REBOOT_POLL_MS).await;
    }
}
