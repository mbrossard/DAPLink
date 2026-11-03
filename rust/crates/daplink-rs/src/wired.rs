//! Wired bit-bang dependency constructor; scan storage is a caller-owned `'static` parameter.

use dap_rs::driver::bitbang::{BitbangAdapter, DelayCycles, InputOutputPin, SwdAdapter};
use dap_rs::jtag::TapConfig;
use dap_rs::swd::{self, APnDP, DPRegister, Swd};
use dap_rs::swj::{Dependencies, Pins};
use daplink_core::dap_wire::IDLE_LOW;
use daplink_core::JtagDp;
use static_cell::StaticCell;

use crate::swd_only;
use crate::swd_wait::wait_retry;

/// Caller-owned `'static` storage for the adapter's scan chain and its JTAG-DP state.
pub struct ScanChainStorage<const N: usize = 1>(StaticCell<[TapConfig; N]>, JtagDp);

impl<const N: usize> ScanChainStorage<N> {
    /// Create empty storage, usable in a `static` initializer.
    pub const fn new() -> Self {
        Self(StaticCell::new(), JtagDp::new())
    }

    /// The chain's JTAG-DP state, shared by its engine and the probe answering for it.
    pub const fn jtag_dp(&'static self) -> &'static JtagDp {
        &self.1
    }

    /// Initialize the scan chain and hand out the `'static` slice the adapter needs.
    ///
    /// # Panics
    ///
    /// If called twice on the same storage.
    pub fn init(&'static self) -> &'static mut [TapConfig] {
        self.0.init([TapConfig::INIT; N])
    }
}

impl<const N: usize> Default for ScanChainStorage<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// A bit-bang line; nRESET moves only on level writes, so a mode switch keeps the host's level.
#[derive(Clone, Copy)]
pub struct SwjPin<IO> {
    io: IO,
    nreset: bool,
}

impl<IO: InputOutputPin> InputOutputPin for SwjPin<IO> {
    #[inline(always)]
    fn set_as_output(&mut self) {
        if !self.nreset {
            self.io.set_as_output();
        }
    }
    #[inline(always)]
    fn set_high(&mut self, high: bool) {
        self.io.set_high(high);
    }
    #[inline(always)]
    fn set_as_input(&mut self) {
        if !self.nreset {
            self.io.set_as_input();
        }
    }
    #[inline(always)]
    fn is_high(&mut self) -> bool {
        self.io.is_high()
    }
}

/// The bit-bang adapter as wired by [`wired_deps`].
pub type WiredAdapter<IO, D> = BitbangAdapter<SwjPin<IO>, D>;

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
) -> WiredAdapter<IO, D> {
    let line = |io, nreset| SwjPin { io, nreset };
    BitbangAdapter::new(
        line(nreset, true),
        line(tdi, false),
        line(swdio, false),
        line(swclk, false),
        line(tdo, false),
        delay,
        scan.init(),
    )
}

/// A read with CMSIS-DAP's WAIT retries, out of line so its paced cells stay in one body.
#[inline(never)]
pub(crate) fn read_retry<IO: InputOutputPin, D: DelayCycles>(
    swd: &mut SwdAdapter<SwjPin<IO>, D>,
    wait_retries: usize,
    port: APnDP,
    addr: DPRegister,
) -> swd::Result<u32> {
    wait_retry(wait_retries, || swd.read_inner(port, addr))
}

/// A write with CMSIS-DAP's WAIT retries, out of line as [`read_retry`].
#[inline(never)]
pub(crate) fn write_retry<IO: InputOutputPin, D: DelayCycles>(
    swd: &mut SwdAdapter<SwjPin<IO>, D>,
    wait_retries: usize,
    port: APnDP,
    addr: DPRegister,
    data: u32,
) -> swd::Result<()> {
    wait_retry(wait_retries, || swd.write_inner(port, addr, data))
}

/// The released turnaround dap-rs skips after a bad-parity read, at the adapter's pacing.
#[cold]
#[inline(never)]
pub(crate) fn trailing_turnaround<IO: InputOutputPin, D: DelayCycles>(
    swd: &mut SwdAdapter<SwjPin<IO>, D>,
) {
    let _ = swd.read_sequence(1, &mut [0]);
}

/// Clock `cycles` idle cells with SWDIO low, at the adapter's pacing.
#[inline(never)]
pub(crate) fn idle_cells<IO: InputOutputPin, D: DelayCycles>(
    swd: &mut SwdAdapter<SwjPin<IO>, D>,
    cycles: u8,
) {
    let _ = swd.write_sequence(cycles.into(), &IDLE_LOW);
}

/// dap-rs reports a no-ACK write as `AckUnknown(7)` (0x0F); a read reports `NoAck` (0x07).
#[inline(always)]
pub(crate) fn no_ack(e: swd::Error) -> swd::Error {
    match e {
        swd::Error::AckUnknown(7) => swd::Error::NoAck,
        e => e,
    }
}

/// Park the pads, then release nRESET, which [`SwjPin`] moves only on level writes.
pub(crate) fn park<IO: InputOutputPin, D: DelayCycles>(adapter: &mut WiredAdapter<IO, D>) {
    adapter.high_impedance_mode();
    adapter.process_swj_pins(Pins::NRESET, Pins::NRESET, 0);
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
) -> swd_only::SwdOnlyDeps<IO, D> {
    let inner = wired_deps(nreset, tdi, swdio, swclk, tdo, delay, scan);
    swd_only::SwdOnlyDeps::new(inner, scan.jtag_dp())
}
