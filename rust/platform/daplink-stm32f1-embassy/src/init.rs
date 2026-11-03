//! Probe bring-up: wire the SWJ pads into the bit-bang dependencies.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::{wired_swd_deps, LateJtagDeps, ScanChainStorage, SwdOnlyDeps};
use embassy_stm32::gpio::Pin;
use embassy_stm32::Peri;

use crate::{DynCycleDelay, IoPin};

pub type ProbeDeps = LateJtagDeps<IoPin, DynCycleDelay>;

/// The wired SWD-only dependencies, for builds that compose no JTAG.
pub type SwdDeps = SwdOnlyDeps<IoPin, DynCycleDelay>;

/// Wire the SWD+JTAG dependencies from five GPIOs, nRESET open-drain; call once per image.
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

/// Wire the SWD-only dependencies, TDI/TDO as no-connects; call once per image.
pub fn swd_only_deps(
    nreset: Peri<'static, impl Pin>,
    swdio: Peri<'static, impl Pin>,
    swclk: Peri<'static, impl Pin>,
    delay: DynCycleDelay,
) -> SwdDeps {
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    wired_swd_deps(
        IoPin::new_open_drain(nreset),
        IoPin::Nc,
        IoPin::new(swdio),
        IoPin::new(swclk),
        IoPin::Nc,
        delay,
        &SCAN,
    )
}
