//! Probe bring-up: wire five SWJ pads into the bit-bang dependencies.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::{LateJtagDeps, ScanChainStorage};
use embassy_stm32::gpio::Pin;
use embassy_stm32::Peri;

use crate::{DynCycleDelay, IoPin};

/// The wired bit-bang dependencies the DAP engine is assembled from.
pub type ProbeDeps = LateJtagDeps<IoPin, DynCycleDelay>;

/// Wire the SWD+JTAG bit-bang dependencies from any five GPIOs; call once per image. nRESET
/// drives open-drain: it shares a wired-AND net with the target's own reset supervisor.
pub fn swd_jtag_deps(
    nreset: Peri<'static, impl Pin>,
    tdi: Peri<'static, impl Pin>,
    swdio: Peri<'static, impl Pin>,
    swclk: Peri<'static, impl Pin>,
    tdo: Peri<'static, impl Pin>,
    delay: DynCycleDelay,
) -> ProbeDeps {
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    LateJtagDeps::wired(
        IoPin::new_open_drain(nreset),
        IoPin::new(tdi),
        IoPin::new(swdio),
        IoPin::new(swclk),
        IoPin::new(tdo),
        delay,
        &SCAN,
    )
}
