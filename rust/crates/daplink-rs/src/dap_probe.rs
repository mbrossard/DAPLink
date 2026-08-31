//! The CMSIS-DAP engine facade that firmware drives.

use dap_rs::dap::{Dap, DapLeds};
use dap_rs::jtag::Jtag;
use dap_rs::swd::Swd;
use dap_rs::swj::Dependencies;
use dap_rs::swo::{NoSwo, Swo};
use daplink_core::vendor::{NoVendor, VendorHandler, VENDOR_CMD_FIRST, VENDOR_CMD_LAST};
use embedded_hal::delay::DelayNs;

use crate::composed_jtag::{ComposedJtagDeps, ComposedJtagDrv, ComposedJtagSwd};
use crate::late_jtag::{Deps, LateJtag, SwdDrv};
use crate::leds::NoopLeds;
use crate::swd_only::{Deps as SwdOnlyDeps, SwdDrv as SwdOnlySwdDrv};

/// The CMSIS-DAP engine facade: bytes in, bytes out, hardware reached only via `DEPS`.
/// `V` is the vendor slot: [`new`](DapProbe::new) fixes it to [`NoVendor`], `with_vendor` does not.
pub struct DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V = NoVendor> {
    dap: Dap<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO>,
    vendor: V,
    /// Advertised `DAP_Info(MaxPacketCount)`: how many commands the host may keep in flight.
    packet_count: u8,
}

impl<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO> DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, NoVendor>
where
    DEPS: Dependencies<SWD, JTAG>,
    LEDS: DapLeds,
    WAIT: DelayNs,
    JTAG: Jtag<DEPS>,
    SWD: Swd<DEPS>,
    SWO: Swo,
{
    /// Build the engine and release the target lines; `max_packet_size` is 64 for full-speed v2 bulk.
    pub fn new(
        dependencies: DEPS,
        leds: LEDS,
        wait: WAIT,
        swo: SWO,
        version_string: &'a str,
        max_packet_size: u16,
    ) -> Self {
        Self::with_vendor(
            dependencies,
            leds,
            wait,
            swo,
            version_string,
            max_packet_size,
            NoVendor,
        )
    }
}

impl<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V> DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V>
where
    DEPS: Dependencies<SWD, JTAG>,
    LEDS: DapLeds,
    WAIT: DelayNs,
    JTAG: Jtag<DEPS>,
    SWD: Swd<DEPS>,
    SWO: Swo,
    V: VendorHandler,
{
    /// [`new`](DapProbe::new) with an explicit vendor-command handler in the vendor slot.
    pub fn with_vendor(
        dependencies: DEPS,
        leds: LEDS,
        wait: WAIT,
        swo: SWO,
        version_string: &'a str,
        max_packet_size: u16,
        vendor: V,
    ) -> Self {
        let mut probe = Self {
            dap: Dap::new(
                dependencies,
                leds,
                wait,
                swo,
                version_string,
                max_packet_size,
            ),
            vendor,
            // Serial until a firmware opts in; matches dap-rs's hardcoded MaxPacketCount.
            packet_count: 1,
        };
        // Bit-bang dependencies come up driving SWDIO/SWCLK; release them before any host talks.
        probe.suspend();
        probe
    }

    /// Advertise `count` for `DAP_Info(MaxPacketCount)`; a pipelined firmware passes its queue
    /// depth. Clamped to at least 1, since 0 would tell the host it may never send a packet.
    pub fn with_packet_count(mut self, count: u8) -> Self {
        self.packet_count = count.max(1);
        self
    }

    /// Borrow the vendor handler.
    pub fn vendor(&self) -> &V {
        &self.vendor
    }

    /// Mutably borrow the vendor handler.
    pub fn vendor_mut(&mut self) -> &mut V {
        &mut self.vendor
    }

    /// Process one CMSIS-DAP command, returning the response length (0 = nothing to send).
    /// Vendor ids (`0x80..=0x9F`) reach the [`VendorHandler`] first; anything else falls through.
    pub fn process_command(&mut self, report: &[u8], response: &mut [u8]) -> usize {
        // The engine's response writer indexes byte 0 unconditionally.
        if response.is_empty() {
            return 0;
        }
        if V::HANDLES {
            if let Some(&cmd) = report.first() {
                if (VENDOR_CMD_FIRST..=VENDOR_CMD_LAST).contains(&cmd) {
                    response[0] = cmd;
                    if let Some(len) = self.vendor.handle(report, response) {
                        // A zero-length USB reply would hang the host; an over-long one is clamped.
                        if len > 0 {
                            return len.min(response.len());
                        }
                    }
                }
            }
        }
        #[cfg(feature = "guard")]
        if crate::guard::check(report, response.len()) == crate::guard::Verdict::Reject {
            response[0] = 0xFF;
            return 1;
        }
        let len = self.dap.process_command(report, response);
        // dap-rs hardcodes MaxPacketCount 1; patch that byte of its `[0x00, len, count]` framing.
        if report.first() == Some(&0x00)
            && report.get(1) == Some(&0xFE)
            && len >= 3
            && response[1] == 1
        {
            response[2] = self.packet_count;
        }
        len
    }

    /// Drop the engine back to its idle role and release the target bus pins to high impedance.
    pub fn suspend(&mut self) {
        self.dap.suspend()
    }
}

/// Bit-bang SWD+late-sample-JTAG DAP engine, generic only over pin `IO` and cycle delay `D`.
/// `LEDS` defaults to [`NoopLeds`]; a firmware wiring a status LED names it explicitly.
pub type BitbangDap<'a, IO, D, S = NoSwo, V = NoVendor, LEDS = NoopLeds> =
    DapProbe<'a, Deps<IO, D>, LEDS, D, LateJtag<IO, D>, SwdDrv<IO, D>, S, V>;

/// Bit-bang SWD-only DAP engine, for boards whose JTAG pins are not wired out.
pub type SwdOnlyDap<'a, IO, D, S = NoSwo, V = NoVendor, LEDS = NoopLeds> =
    DapProbe<'a, SwdOnlyDeps<IO, D>, LEDS, D, SwdOnlyDeps<IO, D>, SwdOnlySwdDrv<IO, D>, S, V>;

/// Accelerated-SWD DAP engine with bit-bang JTAG composed on, over the pad loan.
/// `D`/`S` are the SWD backend's dependency and driver roles; `IO`/`C` the JTAG lines and delay.
pub type ComposedJtagDap<'a, D, S, IO, C, SWO = NoSwo, V = NoVendor, LEDS = NoopLeds> = DapProbe<
    'a,
    ComposedJtagDeps<D, IO, C>,
    LEDS,
    C,
    ComposedJtagDrv<D, IO, C>,
    ComposedJtagSwd<S, IO, C>,
    SWO,
    V,
>;

#[cfg(test)]
mod tests {
    //! Vendor routing: only `0x80..=0x9F` reaches the handler, and declining falls through.

    use super::*;
    use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin};
    use daplink_core::DAP_FW_VERSION;

    use crate::wired::{wired_swd_deps, ScanChainStorage};

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

    /// Counts consultations and either answers with a 3-byte reply or declines.
    struct Recording {
        calls: u32,
        respond: bool,
    }

    impl VendorHandler for Recording {
        fn handle(&mut self, _report: &[u8], response: &mut [u8]) -> Option<usize> {
            self.calls += 1;
            if !self.respond {
                return None;
            }
            response[1] = 0xAB;
            response[2] = 0xCD;
            Some(3)
        }
    }

    type VendorProbe = SwdOnlyDap<'static, Pin, Delay, NoSwo, Recording>;

    fn probe(scan: &'static ScanChainStorage<1>, respond: bool) -> VendorProbe {
        DapProbe::with_vendor(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, scan),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            64,
            Recording { calls: 0, respond },
        )
    }

    #[test]
    fn vendor_ids_reach_the_handler_and_echo_the_command() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = probe(&SCAN, true);
        let mut resp = [0u8; 64];
        assert_eq!(p.process_command(&[0x81, 0x00], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x81, 0xAB, 0xCD]);
        assert_eq!(p.vendor().calls, 1);
        p.vendor_mut().calls = 0;
        assert_eq!(p.vendor().calls, 0);
    }

    #[test]
    fn only_the_reserved_range_is_routed() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = probe(&SCAN, true);
        let mut resp = [0u8; 64];
        for id in [0x7Fu8, 0xA0, 0x00] {
            let before = p.vendor().calls;
            p.process_command(&[id, 0x00], &mut resp);
            assert_eq!(p.vendor().calls, before, "id {id:#x} must not be routed");
        }
        for id in [0x80u8, 0x9F] {
            let before = p.vendor().calls;
            p.process_command(&[id, 0x00], &mut resp);
            assert_eq!(p.vendor().calls, before + 1, "id {id:#x} is an endpoint");
        }
    }

    #[test]
    fn declining_falls_through_to_the_engine() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = probe(&SCAN, false);
        let mut resp = [0u8; 64];
        assert_eq!(p.process_command(&[0x81, 0x00], &mut resp), 1);
        // The engine owns byte 0 on fall-through: an unhandled id is answered `0xFF`.
        assert_eq!(resp[0], 0xFF);
        assert_eq!(p.vendor().calls, 1);
    }

    /// The `NoVendor` fall-through: the branch is compiled out and every id reaches the engine.
    #[test]
    fn no_vendor_routes_everything_to_the_engine() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p: SwdOnlyDap<'static, Pin, Delay> = DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            64,
        );
        let mut resp = [0u8; 64];
        assert_eq!(p.process_command(&[0x8A, 0x00], &mut resp), 1);
        assert_eq!(resp[0], 0xFF);
    }

    /// The default MaxPacketCount is dap-rs's own 1; `with_packet_count` replaces just that byte.
    #[test]
    fn max_packet_count_defaults_to_one_and_follows_the_builder() {
        static SCAN_A: ScanChainStorage<1> = ScanChainStorage::new();
        static SCAN_B: ScanChainStorage<1> = ScanChainStorage::new();
        let mut serial: SwdOnlyDap<'static, Pin, Delay> = DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN_A),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            64,
        );
        let mut resp = [0u8; 64];
        assert_eq!(serial.process_command(&[0x00, 0xFE], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x00, 0x01, 1]);

        let mut pipelined: SwdOnlyDap<'static, Pin, Delay> = DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN_B),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            64,
        )
        .with_packet_count(4);
        assert_eq!(pipelined.process_command(&[0x00, 0xFE], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x00, 0x01, 4]);

        // A count of 0 would tell the host it may never send a packet, so it clamps up to 1.
        assert_eq!(
            pipelined
                .with_packet_count(0)
                .process_command(&[0x00, 0xFE], &mut resp),
            3
        );
        assert_eq!(resp[2], 1);
    }

    /// The patch touches MaxPacketCount only; another `DAP_Info` id is untouched.
    #[test]
    fn other_dap_info_ids_are_unaffected_by_the_packet_count() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p: SwdOnlyDap<'static, Pin, Delay> = DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            64,
        )
        .with_packet_count(4);
        let mut resp = [0u8; 64];
        let len = p.process_command(&[0x00, 0x04], &mut resp);
        let want = DAP_FW_VERSION.as_bytes();
        assert!(resp[..len].windows(want.len()).any(|w| w == want));
    }

    /// The guard sits between vendor routing and the engine and rejects a truncated command.
    #[test]
    #[cfg(feature = "guard")]
    fn guard_rejects_truncated_reports_ahead_of_the_engine() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p: SwdOnlyDap<'static, Pin, Delay> = DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            64,
        );
        let mut resp = [0u8; 64];
        // DAP_SWJ_Clock (0x11) needs a u32; two bytes short must not reach the engine.
        assert_eq!(p.process_command(&[0x11, 1, 2], &mut resp), 1);
        assert_eq!(resp[0], 0xFF);
    }

    /// A payload-less vendor id reaches the handler rather than being guard-rejected.
    #[test]
    #[cfg(feature = "guard")]
    fn guard_does_not_intercept_vendor_ids() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = probe(&SCAN, true);
        let mut resp = [0u8; 64];
        assert_eq!(p.process_command(&[0x81], &mut resp), 3);
        assert_eq!(p.vendor().calls, 1);
    }
}
