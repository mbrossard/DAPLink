//! Probe bring-up: wire five SWJ pads into the bit-bang dependencies.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::{LateJtagDeps, ScanChainStorage};
use embassy_stm32::gpio::Pin;
use embassy_stm32::Peri;

use crate::{DynCycleDelay, IoPin};

pub type ProbeDeps = LateJtagDeps<IoPin, DynCycleDelay>;

/// Wire the bit-bang dependencies over `scan`, SWDIO and SWCLK at `OSPEEDR` class `line_speed`.
#[allow(clippy::too_many_arguments)]
pub fn swd_jtag_deps_at(
    nreset: Peri<'static, impl Pin>,
    tdi: Peri<'static, impl Pin>,
    swdio: Peri<'static, impl Pin>,
    swclk: Peri<'static, impl Pin>,
    tdo: Peri<'static, impl Pin>,
    delay: DynCycleDelay,
    line_speed: u32,
    scan: &'static ScanChainStorage<SCAN_CHAIN_SLOTS>,
) -> ProbeDeps {
    LateJtagDeps::wired(
        IoPin::new_open_drain(nreset),
        IoPin::new(tdi),
        IoPin::with_speed(swdio, line_speed),
        IoPin::with_speed(swclk, line_speed),
        IoPin::new(tdo),
        delay,
        scan,
    )
}
