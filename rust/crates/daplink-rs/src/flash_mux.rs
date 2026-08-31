//! Generic flash-mux assembly: the DAP engine and the flash backend over one arbitrated adapter.

use dap_rs::driver::bitbang::{
    BitbangAdapter, DelayCycles, InputOutputPin, JtagAdapter, SwdAdapter,
};
use dap_rs::jtag::Jtag;
use dap_rs::swd::Swd;
use dap_rs::swj::Dependencies;
use dap_rs::swo::Swo;
use daplink_core::vendor::{NoVendor, VendorHandler};
use daplink_core::DAP_FW_VERSION;
use daplink_flash::target::DefaultCallbacks;
use daplink_targets::{FixedTarget, Target, TargetSelector};
use embedded_hal::delay::DelayNs;

use crate::composed_jtag::{ComposedJtagDeps, ComposedJtagDrv, ComposedJtagSwd};
use crate::dap_probe::DapProbe;
use crate::flash_backend::HwBackend;
use crate::leds::NoopLeds;
use crate::swd_mux::{DapDeps, DapJtag, DapSwd, FlashSide, SwdMux};
use crate::wired::ScanChainStorage;

/// The single bit-bang adapter owned by the mux (SWD+JTAG capable).
pub type MuxDeps<IO, D> = BitbangAdapter<IO, D>;
/// The dap-rs SWD driver role of [`MuxDeps`].
pub type MuxSwd<IO, D> = SwdAdapter<IO, D>;
/// The dap-rs JTAG driver role of [`MuxDeps`].
pub type MuxJtag<IO, D> = JtagAdapter<IO, D>;

/// The flash-wins SWD/JTAG arbiter over one bit-bang adapter.
pub type FlashMux<IO, D> = SwdMux<MuxDeps<IO, D>, MuxSwd<IO, D>, MuxJtag<IO, D>>;

/// The DAP engine's `Dependencies` handle over the bit-bang mux.
pub type MuxDapDeps<IO, D> = DapDeps<MuxDeps<IO, D>, MuxSwd<IO, D>, MuxJtag<IO, D>>;
/// The DAP engine's SWD handle over the bit-bang mux.
pub type MuxDapSwd<IO, D> = DapSwd<MuxDeps<IO, D>, MuxSwd<IO, D>, MuxJtag<IO, D>>;
/// The DAP engine's JTAG handle over the bit-bang mux.
pub type MuxDapJtag<IO, D> = DapJtag<MuxDeps<IO, D>, MuxSwd<IO, D>, MuxJtag<IO, D>>;
/// The flash programmer's ungated bus handle over the bit-bang mux.
pub type FlashHandle<IO, D> = FlashSide<MuxDeps<IO, D>, MuxSwd<IO, D>, MuxJtag<IO, D>>;

/// The flash-wins arbiter over an accelerated-SWD backend with bit-bang JTAG composed on.
/// `D`/`S` are the SWD backend's dependency and driver roles; `IO`/`C` the JTAG lines and delay.
pub type ComposedJtagFlashMux<D, S, IO, C> =
    SwdMux<ComposedJtagDeps<D, IO, C>, ComposedJtagSwd<S, IO, C>, ComposedJtagDrv<D, IO, C>>;

/// The CMSIS-DAP engine over an arbitrated mux, generic over the three dap-rs role types, with
/// the vendor slot `V` defaulting to [`NoVendor`] and the LED slot `LEDS` to [`NoopLeds`].
pub type RoleMuxDapEngine<'a, DEPS, SWD, JTAG, D, S, V = NoVendor, LEDS = NoopLeds> = DapProbe<
    'a,
    DapDeps<DEPS, SWD, JTAG>,
    LEDS,
    D,
    DapJtag<DEPS, SWD, JTAG>,
    DapSwd<DEPS, SWD, JTAG>,
    S,
    V,
>;

/// The on-target flash backend over an arbitrated mux, generic over the three dap-rs role types,
/// the delay `D`, the target description `T` and the selector `Sel`.
pub type RoleMuxFlashBackend<DEPS, SWD, JTAG, D, T, Sel = FixedTarget> = HwBackend<
    'static,
    FlashSide<DEPS, SWD, JTAG>,
    D,
    T,
    DefaultCallbacks,
    FlashSide<DEPS, SWD, JTAG>,
    Sel,
    &'static SwdMux<DEPS, SWD, JTAG>,
    DapDeps<DEPS, SWD, JTAG>,
>;

/// The CMSIS-DAP engine over the bit-bang mux: gated whenever a flash session is live.
pub type MuxDapEngine<'a, IO, D, S, V = NoVendor, LEDS = NoopLeds> =
    RoleMuxDapEngine<'a, MuxDeps<IO, D>, MuxSwd<IO, D>, MuxJtag<IO, D>, D, S, V, LEDS>;

/// The on-target flash backend for a target `T` selected by `Sel`, over the bit-bang mux.
pub type MuxFlashBackend<IO, D, T, Sel = FixedTarget> =
    RoleMuxFlashBackend<MuxDeps<IO, D>, MuxSwd<IO, D>, MuxJtag<IO, D>, D, T, Sel>;

/// Assemble the CMSIS-DAP engine over the bit-bang mux, with `delay` for SWD bit/`WAIT` timing and
/// the caller-supplied `swo` slot and `max_packet_size`. `scan` is caller-owned, one per call site.
pub fn mux_dap_engine<IO, D, S, const N: usize>(
    mux: &'static FlashMux<IO, D>,
    delay: D,
    swo: S,
    max_packet_size: u16,
    scan: &'static ScanChainStorage<N>,
) -> MuxDapEngine<'static, IO, D, S>
where
    IO: InputOutputPin + 'static,
    D: DelayCycles + 'static,
    S: Swo,
{
    role_mux_dap_engine(mux, delay, swo, max_packet_size, scan)
}

/// [`mux_dap_engine`] with an explicit [`VendorHandler`] in the engine's vendor slot, so a
/// firmware can wire the flash vendor commands alongside the flash-wins mux.
pub fn mux_dap_engine_with_vendor<IO, D, S, V, const N: usize>(
    mux: &'static FlashMux<IO, D>,
    delay: D,
    swo: S,
    max_packet_size: u16,
    scan: &'static ScanChainStorage<N>,
    vendor: V,
) -> MuxDapEngine<'static, IO, D, S, V>
where
    IO: InputOutputPin + 'static,
    D: DelayCycles + 'static,
    S: Swo,
    V: VendorHandler,
{
    role_mux_dap_engine_with_vendor(mux, delay, swo, max_packet_size, scan, vendor, NoopLeds)
}

/// The role-generic body [`mux_dap_engine`] delegates to; each `role_*` item is generic over
/// dap-rs's `(DEPS, SWD, JTAG)` types, its plain-named sibling the bit-bang spelling of it.
pub fn role_mux_dap_engine<DEPS, SWD, JTAG, D, S, const N: usize>(
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    delay: D,
    swo: S,
    max_packet_size: u16,
    scan: &'static ScanChainStorage<N>,
) -> RoleMuxDapEngine<'static, DEPS, SWD, JTAG, D, S>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
    D: DelayNs + 'static,
    S: Swo,
{
    role_mux_dap_engine_with_vendor(mux, delay, swo, max_packet_size, scan, NoVendor, NoopLeds)
}

/// The role-generic body [`mux_dap_engine_with_vendor`] delegates to, with `leds` in the engine's
/// LED slot -- [`NoopLeds`] for a build with no status LED wired.
pub fn role_mux_dap_engine_with_vendor<DEPS, SWD, JTAG, D, S, V, LEDS, const N: usize>(
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    delay: D,
    swo: S,
    max_packet_size: u16,
    scan: &'static ScanChainStorage<N>,
    vendor: V,
    leds: LEDS,
) -> RoleMuxDapEngine<'static, DEPS, SWD, JTAG, D, S, V, LEDS>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
    D: DelayNs + 'static,
    S: Swo,
    V: VendorHandler,
    LEDS: dap_rs::dap::DapLeds,
{
    DapProbe::with_vendor(
        mux.dap_deps(scan.init()),
        leds,
        delay,
        swo,
        DAP_FW_VERSION,
        max_packet_size,
        vendor,
    )
}

/// Build the on-target flash backend over the bit-bang mux, with auto-reset after a clean flash.
/// `target` is composed, `table` the registry `Sel` selects from, `buffer` the page-staging scratch.
pub fn build_flash_backend<IO, D, T, Sel>(
    mux: &'static FlashMux<IO, D>,
    delay: D,
    target: &'static T,
    table: &'static [&'static dyn Target],
    buffer: &'static mut [u8],
) -> MuxFlashBackend<IO, D, T, Sel>
where
    IO: InputOutputPin + 'static,
    D: DelayCycles + 'static,
    T: Target,
    Sel: TargetSelector + Default,
{
    role_build_flash_backend(mux, delay, target, table, buffer)
}

/// The role-generic body [`build_flash_backend`] delegates to. It mints two ungated flash-side
/// handles from a transient session, which is dropped so the DAP engine is re-admitted meanwhile.
pub fn role_build_flash_backend<DEPS, SWD, JTAG, D, T, Sel>(
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    delay: D,
    target: &'static T,
    table: &'static [&'static dyn Target],
    buffer: &'static mut [u8],
) -> RoleMuxFlashBackend<DEPS, SWD, JTAG, D, T, Sel>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
    D: DelayNs + 'static,
    T: Target,
    Sel: TargetSelector + Default,
{
    let session = mux.begin_flash().expect("flash mux is free at init");
    let swd = session.flash_side();
    let reset = session.flash_side();
    drop(session);

    HwBackend::new(
        swd,
        delay,
        DefaultCallbacks,
        reset,
        Sel::default(),
        target,
        table,
        buffer,
        mux,
        true,
    )
}

#[cfg(test)]
mod tests {
    //! Both shipping compositions must fit the mux, and only instantiating them proves it.

    use super::*;
    use dap_rs::jtag::{self, SequenceInfo, TapConfig};
    use dap_rs::swd::{self, APnDP, DPRegister};
    use dap_rs::swj::Pins;
    use static_cell::StaticCell;

    use crate::composed_jtag::SwjPadLoan;
    use crate::late_jtag::JtagLines;

    struct Pin;
    impl InputOutputPin for Pin {
        fn set_as_output(&mut self) {}
        fn set_high(&mut self, _high: bool) {}
        fn set_as_input(&mut self) {}
        fn is_high(&mut self) -> bool {
            false
        }
    }

    #[derive(Clone, Copy)]
    struct Delay;
    impl DelayNs for Delay {
        fn delay_ns(&mut self, _ns: u32) {}
    }
    impl DelayCycles for Delay {
        fn cpu_clock(&self) -> u32 {
            1_000_000
        }
        fn delay_cycles(&mut self, _cycles: u32) {}
    }

    /// The bit-bang composition: one adapter carrying both roles.
    #[test]
    fn the_mux_arbitrates_a_bitbang_adapter() {
        static SCAN: StaticCell<[TapConfig; 1]> = StaticCell::new();
        static HANDLE: StaticCell<[TapConfig; 1]> = StaticCell::new();
        static MUX: StaticCell<FlashMux<Pin, Delay>> = StaticCell::new();
        let adapter = BitbangAdapter::new(
            Pin,
            Pin,
            Pin,
            Pin,
            Pin,
            Delay,
            SCAN.init([TapConfig::INIT; 1]),
        );
        let mux: &'static FlashMux<Pin, Delay> = MUX.init(SwdMux::new(adapter));

        let _deps: MuxDapDeps<Pin, Delay> = mux.dap_deps(HANDLE.init([TapConfig::INIT; 1]));
        let session = mux.begin_flash().expect("free at init");
        let _flash: FlashHandle<Pin, Delay> = session.flash_side();
        assert!(mux.is_flashing(), "the flash side holds the wire");
        drop(session);
        assert!(!mux.is_flashing());
    }

    /// A minimal accelerated-SWD backend: its own `Dependencies` and JTAG role, per the composition.
    struct Backend {
        swd_cfg: swd::Config,
        jtag_cfg: jtag::Config,
    }

    struct BackendSwd {
        swd_cfg: swd::Config,
        jtag_cfg: jtag::Config,
    }

    impl From<BackendSwd> for Backend {
        fn from(s: BackendSwd) -> Self {
            Backend {
                swd_cfg: s.swd_cfg,
                jtag_cfg: s.jtag_cfg,
            }
        }
    }
    impl From<Backend> for BackendSwd {
        fn from(d: Backend) -> Self {
            BackendSwd {
                swd_cfg: d.swd_cfg,
                jtag_cfg: d.jtag_cfg,
            }
        }
    }

    impl Dependencies<BackendSwd, Backend> for Backend {
        fn process_swj_pins(&mut self, output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
            output
        }
        fn process_swj_sequence(&mut self, _data: &[u8], _nbits: usize) {}
        fn process_swj_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn high_impedance_mode(&mut self) {}
        fn swd_config(&mut self) -> &mut swd::Config {
            &mut self.swd_cfg
        }
        fn jtag_config(&mut self) -> &mut jtag::Config {
            &mut self.jtag_cfg
        }
    }

    impl Swd<Backend> for BackendSwd {
        fn available(_deps: &Backend) -> bool {
            true
        }
        fn config(&mut self) -> &mut swd::Config {
            &mut self.swd_cfg
        }
        fn read_inner(&mut self, _apndp: APnDP, _a: DPRegister) -> swd::Result<u32> {
            Ok(0)
        }
        fn write_inner(&mut self, _apndp: APnDP, _a: DPRegister, _data: u32) -> swd::Result<()> {
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> swd::Result<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> swd::Result<()> {
            Ok(())
        }
    }

    impl Jtag<Backend> for Backend {
        fn available(_deps: &Backend) -> bool {
            false
        }
        fn config(&mut self) -> &mut jtag::Config {
            &mut self.jtag_cfg
        }
        fn sequence(&mut self, _info: SequenceInfo, _tdi: &[u8], _rxbuf: &mut [u8]) {}
        fn tms_sequence(&mut self, _tms: &[bool]) {}
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            false
        }
    }

    impl SwjPadLoan for Backend {
        fn lend_pads(&mut self) {}
        fn reclaim_pads(&mut self) {}
        fn set_swd_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn jtag_config_mut(&mut self) -> &mut jtag::Config {
            &mut self.jtag_cfg
        }
    }

    type ComposedMux = ComposedJtagFlashMux<Backend, BackendSwd, Pin, Delay>;

    /// The accelerated-SWD-plus-composed-JTAG shape fits the same mux and the same handles.
    #[test]
    fn the_mux_arbitrates_a_composed_jtag_backend() {
        static SCAN: StaticCell<[TapConfig; 1]> = StaticCell::new();
        static HANDLE: StaticCell<[TapConfig; 1]> = StaticCell::new();
        static MUX: StaticCell<ComposedMux> = StaticCell::new();
        let backend = Backend {
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(SCAN.init([TapConfig::INIT; 1])),
        };
        let deps = ComposedJtagDeps::new(backend, JtagLines::new(Pin, Pin, Pin, Pin), Delay);
        let mux: &'static ComposedMux = MUX.init(SwdMux::new(deps));

        let handle = mux.dap_deps(HANDLE.init([TapConfig::INIT; 1]));
        // The composed JTAG role is always available; SWD availability comes from the backend.
        assert!(<DapSwd<_, _, _> as Swd<_>>::available(&handle));
        assert!(<DapJtag<_, _, _> as Jtag<_>>::available(&handle));

        let session = mux.begin_flash().expect("free at init");
        assert!(mux.is_flashing());
        drop(session);
        assert!(!mux.is_flashing());
    }
}
