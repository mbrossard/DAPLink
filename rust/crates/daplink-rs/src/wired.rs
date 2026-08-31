//! Wired bit-bang dependency constructor; scan storage is a caller-owned `'static` parameter.

use dap_rs::driver::bitbang::{BitbangAdapter, DelayCycles, InputOutputPin};
use dap_rs::jtag::TapConfig;
use static_cell::StaticCell;

use crate::swd_only;

/// Caller-owned `'static` storage for the adapter's scan chain; one TAP slot by default.
pub struct ScanChainStorage<const N: usize = 1>(StaticCell<[TapConfig; N]>);

impl<const N: usize> ScanChainStorage<N> {
    /// Create empty storage, usable in a `static` initializer.
    pub const fn new() -> Self {
        Self(StaticCell::new())
    }

    /// Initialize the scan chain and hand out the `'static` slice the adapter needs.
    ///
    /// # Panics
    /// Panics if called more than once on the same storage. Declare one storage per call site.
    pub fn init(&'static self) -> &'static mut [TapConfig] {
        self.0.init([TapConfig::INIT; N])
    }
}

impl<const N: usize> Default for ScanChainStorage<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Wire the bit-bang dependencies; all five SWJ pins are required, TDI/TDO carrying JTAG data.
#[allow(clippy::too_many_arguments)]
pub fn wired_deps<IO: InputOutputPin, D: DelayCycles, const N: usize>(
    nreset: IO,
    tdi: IO,
    swdio: IO,
    swclk: IO,
    tdo: IO,
    delay: D,
    scan: &'static ScanChainStorage<N>,
) -> BitbangAdapter<IO, D> {
    BitbangAdapter::new(nreset, tdi, swdio, swclk, tdo, delay, scan.init())
}

/// Wire the SWD-only dependencies; all five pins are required, though TDI/TDO may be no-connects.
#[allow(clippy::too_many_arguments)]
pub fn wired_swd_deps<IO: InputOutputPin, D: DelayCycles, const N: usize>(
    nreset: IO,
    tdi: IO,
    swdio: IO,
    swclk: IO,
    tdo: IO,
    delay: D,
    scan: &'static ScanChainStorage<N>,
) -> swd_only::Deps<IO, D> {
    swd_only::Deps::new(wired_deps(nreset, tdi, swdio, swclk, tdo, delay, scan))
}
