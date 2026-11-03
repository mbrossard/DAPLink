//! Generic flash-mux assembly: the DAP engine and the flash backend over one arbitrated adapter.

use dap_rs::jtag::Jtag;
use dap_rs::swd::Swd;
use dap_rs::swj::Dependencies;
use dap_rs::swo::Swo;
use daplink_core::vendor::{NoVendor, VendorHandler};
use daplink_core::{JtagDpSource, PacketSize, DAP_FW_VERSION};
use daplink_flash::target::DefaultCallbacks;
use daplink_targets::{FixedTarget, Target, TargetSelector};
use embedded_hal::delay::DelayNs;

use crate::dap_probe::{DapProbe, NoCommandFence};
use crate::flash_backend::HwBackend;
use crate::leds::NoopLeds;
use crate::swd_mux::{DapDeps, DapJtag, DapSwd, FlashSide, SwdMux};
use crate::wired::ScanChainStorage;

/// The CMSIS-DAP engine over an arbitrated mux, generic over the dap-rs role types.
pub type MuxDapEngine<
    'a,
    DEPS,
    SWD,
    JTAG,
    D,
    S,
    V = NoVendor,
    LEDS = NoopLeds,
    F = NoCommandFence,
> = DapProbe<
    'a,
    DapDeps<DEPS, SWD, JTAG>,
    LEDS,
    D,
    DapJtag<DEPS, SWD, JTAG>,
    DapSwd<DEPS, SWD, JTAG>,
    S,
    V,
    F,
>;

/// The on-target flash backend over an arbitrated mux, generic over the dap-rs role types.
pub type MuxFlashBackend<DEPS, SWD, JTAG, D, T, Sel = FixedTarget> = HwBackend<
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

/// Assemble the CMSIS-DAP engine over a mux; `scan` is caller-owned, one per call site.
pub fn mux_dap_engine<DEPS, SWD, JTAG, D, S, V, LEDS, const N: usize>(
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    delay: D,
    swo: S,
    max_packet_size: PacketSize,
    scan: &'static ScanChainStorage<N>,
    vendor: V,
    leds: LEDS,
) -> MuxDapEngine<'static, DEPS, SWD, JTAG, D, S, V, LEDS>
where
    DEPS: Dependencies<SWD, JTAG> + JtagDpSource + 'static,
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

/// Build the on-target flash backend over a mux, with auto-reset after a clean flash.
pub fn mux_flash_backend<DEPS, SWD, JTAG, D, T, Sel>(
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    delay: D,
    target: &'static T,
    table: &'static [&'static dyn Target],
    buffer: &'static mut [u8],
) -> MuxFlashBackend<DEPS, SWD, JTAG, D, T, Sel>
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
    //! The engine and backend assembled over the shipping bit-bang composition.

    use super::*;
    extern crate std;
    use crate::late_jtag::{LateJtag, LateJtagDeps, LateSwdDrv};
    use core::cell::{Cell, RefCell};
    use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin};
    use dap_rs::jtag::TapConfig;
    use dap_rs::swd::{APnDP, DPRegister, Swd};
    use daplink_flash::flash_algo::{FlashRegion, RamRegion, SectorInfo};
    use static_cell::StaticCell;
    use std::collections::VecDeque;

    #[derive(Clone)]
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

    type Mux = SwdMux<LateJtagDeps<Pin, Delay>, LateSwdDrv<Pin, Delay>, LateJtag<Pin, Delay>>;

    /// The late-sample composition over `wired_deps`: one adapter carrying both roles.
    #[test]
    fn the_mux_arbitrates_the_wired_bitbang_adapter() {
        static SCAN: ScanChainStorage = ScanChainStorage::new();
        static HANDLE: StaticCell<[TapConfig; 1]> = StaticCell::new();
        static MUX: StaticCell<Mux> = StaticCell::new();
        let deps = LateJtagDeps::wired(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN);
        let mux: &'static Mux = MUX.init(SwdMux::new(deps));

        let _deps = mux.dap_deps(HANDLE.init([TapConfig::INIT; 1]));
        let session = mux.begin_flash().expect("free at init");
        let _flash = session.flash_side();
        assert!(mux.is_flashing(), "the flash side holds the wire");
        drop(session);
        assert!(!mux.is_flashing());
    }

    /// A pin that records whether it drives, and answers reads from a queue.
    struct LineState {
        output: Cell<bool>,
        answers: RefCell<VecDeque<bool>>,
    }
    #[derive(Clone, Copy)]
    struct Rec(&'static LineState);
    impl InputOutputPin for Rec {
        fn set_as_output(&mut self) {
            self.0.output.set(true);
        }
        fn set_high(&mut self, _high: bool) {}
        fn set_as_input(&mut self) {
            self.0.output.set(false);
        }
        fn is_high(&mut self) -> bool {
            self.0.answers.borrow_mut().pop_front().unwrap_or(false)
        }
    }
    fn line() -> &'static LineState {
        std::boxed::Box::leak(std::boxed::Box::new(LineState {
            output: Cell::new(false),
            answers: RefCell::new(VecDeque::new()),
        }))
    }
    fn ok_ack(l: &LineState) {
        l.answers.borrow_mut().extend([true, false, false]);
    }
    type RecMux = SwdMux<LateJtagDeps<Rec, Delay>, LateSwdDrv<Rec, Delay>, LateJtag<Rec, Delay>>;
    fn rec_mux(swdio: &'static LineState, swclk: &'static LineState) -> &'static RecMux {
        let scan: &'static ScanChainStorage =
            std::boxed::Box::leak(std::boxed::Box::new(ScanChainStorage::new()));
        let deps = LateJtagDeps::wired(
            Rec(line()),
            Rec(line()),
            Rec(swdio),
            Rec(swclk),
            Rec(line()),
            Delay,
            scan,
        );
        std::boxed::Box::leak(std::boxed::Box::new(SwdMux::new(deps)))
    }

    /// The session's end releases what its last transfer left driven.
    #[test]
    fn flash_session_end_parks_the_pads_on_the_wire() {
        let (swdio, swclk) = (line(), line());
        let mux = rec_mux(swdio, swclk);
        let session = mux.begin_flash().expect("free");
        let mut side = session.flash_side();
        ok_ack(swdio);
        side.write_inner(APnDP::AP, DPRegister::RDBUFF, 0x05FA_0004)
            .expect("written");
        assert!(swclk.output.get(), "the session drives SWCLK");
        drop(session);
        assert!(!swclk.output.get(), "SWCLK released");
        assert!(!swdio.output.get(), "SWDIO released");
    }

    /// A parked bus still serves the DAP session that follows.
    #[test]
    fn a_dap_session_after_the_park_finds_its_pads() {
        let (swdio, swclk) = (line(), line());
        let mux = rec_mux(swdio, swclk);
        let session = mux.begin_flash().expect("free");
        let mut side = session.flash_side();
        ok_ack(swdio);
        side.read_inner(APnDP::DP, DPRegister::DPIDR).expect("read");
        drop(session);
        let scan: &'static ScanChainStorage<1> =
            std::boxed::Box::leak(std::boxed::Box::new(ScanChainStorage::new()));
        let mut engine = mux_dap_engine(
            mux,
            Delay,
            dap_rs::swo::NoSwo,
            daplink_core::DapQueue::<1, 64>::packet_size::<64>(),
            scan,
            NoVendor,
            NoopLeds,
        );
        let mut resp = [0u8; 64];
        engine.process_command(&[0x02, 0x01], &mut resp);
        ok_ack(swdio);
        engine.process_command(&[0x05, 0x00, 0x01, 0x02], &mut resp);
        assert_eq!(&resp[..3], &[0x05, 1, 1], "the read went through, ACK OK");
        assert!(swclk.output.get(), "SWCLK is driven again");
    }

    const CONNECT_SWD: [u8; 2] = [0x02, 1];
    const CLOCK_1MHZ: [u8; 5] = [0x11, 0x40, 0x42, 0x0F, 0x00];

    macro_rules! engine_on_pin {
        () => {{
            static SCAN: ScanChainStorage = ScanChainStorage::new();
            static MUX: StaticCell<Mux> = StaticCell::new();
            static DAP_SCAN: ScanChainStorage<2> = ScanChainStorage::new();
            let mux: &'static Mux = MUX.init(SwdMux::new(LateJtagDeps::wired(
                Pin, Pin, Pin, Pin, Pin, Delay, &SCAN,
            )));
            let engine = mux_dap_engine(
                mux,
                Delay,
                dap_rs::swo::NoSwo,
                daplink_core::DapQueue::<1, 512>::packet_size::<512>(),
                &DAP_SCAN,
                NoVendor,
                NoopLeds,
            );
            (mux, engine)
        }};
    }

    /// A host attaching during a drop sees no target, not a clock fault.
    #[test]
    fn a_flash_session_does_not_refuse_the_host_clock() {
        let (mux, mut engine) = engine_on_pin!();
        let mut resp = [0u8; 64];
        engine.process_command(&CONNECT_SWD, &mut resp);
        let session = mux.begin_flash().expect("free");
        engine.process_command(&CLOCK_1MHZ, &mut resp);
        assert_eq!(resp[1], 0x00, "DAP_SWJ_Clock during a session");
        engine.process_command(&[0x11, 0, 0, 0, 0], &mut resp);
        assert_eq!(resp[1], 0xFF, "a zero clock is refused as ever");
        drop(session);
    }

    /// A refused transfer reads as no target on either protocol.
    #[test]
    fn a_flash_session_answers_transfers_with_no_ack() {
        let (mux, mut engine) = engine_on_pin!();
        let mut resp = [0u8; 64];
        for (connect, setup) in [
            (&[0x02, 1][..], &[0x00, 0xF0][..]),
            (&[0x02, 2][..], &[0x15, 1, 4][..]),
        ] {
            engine.process_command(connect, &mut resp);
            engine.process_command(setup, &mut resp);
            let session = mux.begin_flash().expect("free");
            engine.process_command(&[0x05, 0, 1, 0x02], &mut resp);
            assert_eq!(resp[2], 0x07, "{connect:02x?}: {:02x?}", &resp[..4]);
            drop(session);
        }
    }

    /// Answers every vendor id with one marker byte.
    struct Marker;
    impl VendorHandler for Marker {
        fn handle(&mut self, _report: &[u8], response: &mut [u8]) -> Option<usize> {
            response[1] = 0x5A;
            Some(2)
        }
    }

    /// The engine reports the caller's packet size and the firmware version, and routes vendor ids.
    #[test]
    fn the_engine_answers_with_its_packet_size_version_and_vendor() {
        static SCAN: ScanChainStorage = ScanChainStorage::new();
        static MUX: StaticCell<Mux> = StaticCell::new();
        static DAP_SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mux: &'static Mux = MUX.init(SwdMux::new(LateJtagDeps::wired(
            Pin, Pin, Pin, Pin, Pin, Delay, &SCAN,
        )));
        let mut engine = mux_dap_engine(
            mux,
            Delay,
            dap_rs::swo::NoSwo,
            daplink_core::DapQueue::<1, 512>::packet_size::<512>(),
            &DAP_SCAN,
            Marker,
            NoopLeds,
        );
        let mut resp = [0u8; 64];

        assert_eq!(engine.process_command(&[0x00, 0xFF], &mut resp), 4);
        assert_eq!(&resp[..4], &[0x00, 2, 0x00, 0x02]);

        let n = engine.process_command(&[0x00, 0x04], &mut resp);
        assert_eq!(&resp[2..n - 1], DAP_FW_VERSION.as_bytes());

        // SWD and JTAG both: the engine sits on the mux's two-role DAP side.
        assert_eq!(engine.process_command(&[0x00, 0xF0], &mut resp), 3);
        assert_eq!(resp[2] & 0b11, 0b11);

        assert_eq!(engine.process_command(&[0x85], &mut resp), 2);
        assert_eq!(&resp[..2], &[0x85, 0x5A]);
    }

    struct NoFlash;
    impl Target for NoFlash {
        fn flash_regions(&self) -> &[FlashRegion<'_>] {
            &[]
        }
        fn ram_regions(&self) -> &[RamRegion] {
            &[]
        }
        fn sectors(&self) -> &[SectorInfo] {
            &[]
        }
    }

    /// Building the backend leaves the wire free, and the backend then claims it through the mux.
    #[test]
    fn the_backend_is_built_idle_and_arbitrates_through_the_mux() {
        use daplink_core::flash_sink::{FlashBackend, FlashPhase};

        static SCAN: ScanChainStorage = ScanChainStorage::new();
        static MUX: StaticCell<Mux> = StaticCell::new();
        static BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
        static TARGET: NoFlash = NoFlash;
        let table: &'static [&'static dyn Target] =
            std::boxed::Box::leak(std::boxed::Box::new([&TARGET as &dyn Target]));
        let mux: &'static Mux = MUX.init(SwdMux::new(LateJtagDeps::wired(
            Pin, Pin, Pin, Pin, Pin, Delay, &SCAN,
        )));
        let mut backend: MuxFlashBackend<_, _, _, _, _> =
            mux_flash_backend(mux, Delay, &TARGET, table, BUFFER.init([0; 256]));
        assert!(!mux.is_flashing(), "building must not keep the wire");

        let held = mux.begin_flash().expect("free after build");
        assert_eq!(backend.begin(), Err(FlashPhase::NoTarget), "busy mux");
        drop(held);

        // A floating wire answers nothing: the attempt fails past the claim and hands it back.
        assert_eq!(backend.begin(), Err(FlashPhase::Init));
        assert!(!mux.is_flashing());
    }
}
