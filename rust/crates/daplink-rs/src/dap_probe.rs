//! The CMSIS-DAP engine facade that firmware drives.

use dap_rs::dap::{Dap, DapLeds};
use dap_rs::jtag::Jtag;
use dap_rs::swd::Swd;
use dap_rs::swj::Dependencies;
use dap_rs::swo::{NoSwo, Swo};
use daplink_core::vendor::{NoVendor, VendorHandler, VENDOR_CMD_FIRST, VENDOR_CMD_LAST};
use daplink_core::{Identity, JtagDp, JtagDpSource, PacketSize};
use embedded_hal::delay::DelayNs;

use crate::late_jtag::{LateJtag, LateJtagDeps, LateSwdDrv};
use crate::leds::NoopLeds;
use crate::swd_only::{SwdOnlyDeps, SwdOnlySwdDrv};
use crate::transfer_count::TransferCount;

const DAP_INFO: u8 = 0x00;
const DAP_CONNECT: u8 = 0x02;
const DAP_DISCONNECT: u8 = 0x03;
const DAP_TRANSFER_CONFIGURE: u8 = 0x04;
const DAP_TRANSFER: u8 = 0x05;
const DAP_TRANSFER_BLOCK: u8 = 0x06;
const DAP_WRITE_ABORT: u8 = 0x08;
const DAP_JTAG_CONFIGURE: u8 = 0x15;
/// The status dap-rs writes for `TransferResult::Fault`.
const DAP_RS_FAULT: u8 = 0x08;
const DAP_INFO_PACKET_COUNT: u8 = 0xFE;

/// Finish hardware work that may outlive one synchronous CMSIS-DAP command.
pub trait CommandFence {
    /// Wait until the command's externally visible work is complete.
    fn finish_command(&self);
}

/// The default command fence for engines that finish every transfer before returning.
pub struct NoCommandFence;

impl CommandFence for NoCommandFence {
    fn finish_command(&self) {}
}

impl<F: CommandFence + ?Sized> CommandFence for &F {
    fn finish_command(&self) {
        F::finish_command(self)
    }
}

/// The CMSIS-DAP engine facade; `V` is the vendor slot and `F` an optional completion fence.
pub struct DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V = NoVendor, F = NoCommandFence> {
    dap: Dap<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO>,
    vendor: V,
    fence: F,
    /// Advertised `DAP_Info(MaxPacketCount)`: how many commands the host may keep in flight.
    packet_count: u8,
    /// `DAP_Info` `0x04`, answered here so the reply carries the NUL the length counts.
    version_string: &'a str,
    /// The identity strings answered for `DAP_Info` `0x01`, `0x02`, `0x03`, `0x07` and `0x08`.
    identity: Option<Identity>,
    /// The JTAG engine's ACK state, reset per session, and the host's SWD idle cycles.
    jtag_dp: Option<&'static JtagDp>,
    count: TransferCount,
}

/// Write one `DAP_Info` string reply; `len` counts the NUL, empty answers 0, overlong truncates.
fn write_info_string(text: &str, response: &mut [u8]) -> usize {
    response[0] = DAP_INFO;
    if text.is_empty() || response.len() < 3 {
        response[1] = 0;
        return 2;
    }
    let room = (response.len() - 3).min(u8::MAX as usize - 1);
    let bytes = &text.as_bytes()[..text.len().min(room)];
    response[2..2 + bytes.len()].copy_from_slice(bytes);
    response[2 + bytes.len()] = 0;
    response[1] = bytes.len() as u8 + 1;
    3 + bytes.len()
}

impl<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO> DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, NoVendor>
where
    DEPS: Dependencies<SWD, JTAG> + JtagDpSource,
    LEDS: DapLeds,
    WAIT: DelayNs,
    JTAG: Jtag<DEPS>,
    SWD: Swd<DEPS>,
    SWO: Swo,
{
    /// Build the engine and release the target lines; `max_packet_size` is advertised in `DAP_Info`.
    pub fn new(
        dependencies: DEPS,
        leds: LEDS,
        wait: WAIT,
        swo: SWO,
        version_string: &'a str,
        max_packet_size: PacketSize,
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

impl<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V>
    DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V, NoCommandFence>
where
    DEPS: Dependencies<SWD, JTAG> + JtagDpSource,
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
        max_packet_size: PacketSize,
        vendor: V,
    ) -> Self {
        let jtag_dp = dependencies.jtag_dp();
        let mut probe = Self {
            dap: Dap::new(
                dependencies,
                leds,
                wait,
                swo,
                version_string,
                max_packet_size.get(),
            ),
            vendor,
            fence: NoCommandFence,
            packet_count: 1,
            version_string,
            identity: None,
            jtag_dp,
            count: TransferCount::new(),
        };
        // Bit-bang dependencies come up driving SWDIO/SWCLK; park them before any host talks.
        probe.suspend();
        probe
    }
}

impl<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V, F>
    DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V, F>
where
    DEPS: Dependencies<SWD, JTAG> + JtagDpSource,
    LEDS: DapLeds,
    WAIT: DelayNs,
    JTAG: Jtag<DEPS>,
    SWD: Swd<DEPS>,
    SWO: Swo,
    V: VendorHandler,
    F: CommandFence,
{
    /// Advertise `count` for `DAP_Info(MaxPacketCount)`, clamped to at least 1.
    pub fn with_packet_count(mut self, count: u8) -> Self {
        self.packet_count = count.max(1);
        self
    }

    /// Answer the `DAP_Info` identity ids from `identity`; without it they answer length 0.
    pub fn with_identity(mut self, identity: Identity) -> Self {
        self.identity = Some(identity);
        self
    }

    /// The string this probe answers for `DAP_Info` id `id`, or `None` to leave it to the engine.
    fn info_string(&self, id: u8) -> Option<&str> {
        match id {
            0x04 => Some(self.version_string),
            0x09 => Some(daplink_core::DAP_FW_ROW),
            _ => {
                let identity = self.identity?;
                match id {
                    0x01 => Some(identity.vendor),
                    0x02 => Some(identity.product),
                    0x03 => Some(identity.serial),
                    0x07 => Some(identity.board_vendor),
                    0x08 => Some(identity.board_name),
                    _ => None,
                }
            }
        }
    }

    /// Replace the command-completion policy without rebuilding the DAP engine.
    pub fn with_command_fence<G>(
        self,
        fence: G,
    ) -> DapProbe<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO, V, G>
    where
        G: CommandFence,
    {
        DapProbe {
            dap: self.dap,
            vendor: self.vendor,
            fence,
            packet_count: self.packet_count,
            version_string: self.version_string,
            identity: self.identity,
            jtag_dp: self.jtag_dp,
            count: self.count,
        }
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
                            // A vendor flash command may use the async SWD engine too.
                            self.fence.finish_command();
                            return len.min(response.len());
                        }
                    }
                }
            }
        }
        if crate::guard::check(report, response.len()) == crate::guard::Verdict::Reject {
            response[0] = 0xFF;
            return 1;
        }
        // dap-rs omits the NUL the string length must count, and knows nothing of the board.
        if report.first() == Some(&DAP_INFO) && response.len() >= 2 {
            if let Some(text) = report.get(1).and_then(|&id| self.info_string(id)) {
                return write_info_string(text, response);
            }
        }
        let cmd = report.first().copied();
        if let Some(dp) = self.jtag_dp {
            match cmd {
                Some(DAP_CONNECT | DAP_DISCONNECT | DAP_JTAG_CONFIGURE) => dp.reset(),
                Some(DAP_TRANSFER_CONFIGURE) => dp.observe(report),
                Some(DAP_TRANSFER | DAP_TRANSFER_BLOCK) => {
                    dp.take_failure();
                }
                _ => {}
            }
        }
        let empty_block =
            cmd == Some(DAP_TRANSFER_BLOCK) && report[2..4] == [0, 0] && response.len() >= 4;
        let len = if empty_block {
            // Arm leaves the bus alone; an empty transfer restores dap-rs's mode as a block does.
            self.dap
                .process_command(&[DAP_TRANSFER, report[1], 0], response);
            response[..4].copy_from_slice(&[DAP_TRANSFER_BLOCK, 0, 0, 0]);
            4
        } else {
            self.dap.process_command(report, response)
        };
        // dap-rs reports every JTAG failure as 0x08; the engine recorded which it was.
        let status = match cmd {
            Some(DAP_TRANSFER) => Some(2),
            Some(DAP_TRANSFER_BLOCK) => Some(3),
            _ => None,
        };
        if let (Some(dp), Some(at)) = (self.jtag_dp, status.filter(|&at| at < len)) {
            let failure = dp.take_failure();
            if failure != 0 && response[at] == DAP_RS_FAULT {
                response[at] = failure;
            }
        }
        // dap-rs's block counts and empty-transfer statuses differ from Arm's; answer as Arm does.
        if !empty_block {
            self.count.correct(report, response, len);
        }
        self.count.observe(report, &response[..len]);
        // The dap-rs commands that can end on an SWD write suffix.
        if matches!(
            report.first().copied(),
            Some(DAP_TRANSFER | DAP_TRANSFER_BLOCK | DAP_WRITE_ABORT)
        ) {
            self.fence.finish_command();
        }
        // dap-rs hardcodes MaxPacketCount 1; patch that byte of its `[0x00, len, count]` framing.
        if report.first() == Some(&DAP_INFO)
            && report.get(1) == Some(&DAP_INFO_PACKET_COUNT)
            && len >= 3
            && response[1] == 1
        {
            response[2] = self.packet_count;
        }
        len
    }

    /// Drop the engine back to its idle role and park the pads; RP and MCU-Link keep SWCLK driven.
    pub fn suspend(&mut self) {
        self.vendor.session_reset();
        self.dap.suspend()
    }
}

/// Bit-bang SWD + late-sample JTAG engine, generic over pin `IO` and cycle delay `D`.
pub type BitbangDap<'a, IO, D, S = NoSwo, V = NoVendor, LEDS = NoopLeds, F = NoCommandFence> =
    DapProbe<'a, LateJtagDeps<IO, D>, LEDS, D, LateJtag<IO, D>, LateSwdDrv<IO, D>, S, V, F>;

/// Bit-bang SWD-only DAP engine, for boards whose JTAG pins are not wired out.
pub type SwdOnlyDap<'a, IO, D, S = NoSwo, V = NoVendor, LEDS = NoopLeds, F = NoCommandFence> =
    DapProbe<'a, SwdOnlyDeps<IO, D>, LEDS, D, SwdOnlyDeps<IO, D>, SwdOnlySwdDrv<IO, D>, S, V, F>;

#[cfg(test)]
mod tests {
    //! Vendor routing: only `0x80..=0x9F` reaches the handler, and declining falls through.

    use super::*;
    use core::cell::Cell;
    use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin};
    use daplink_core::{DapQueue, DAP_FW_VERSION};

    use crate::wired::{wired_swd_deps, ScanChainStorage};

    const PACKET: PacketSize = DapQueue::<1, 64>::packet_size::<64>();
    const PACKET_512: PacketSize = DapQueue::<1, 512>::packet_size::<512>();

    #[derive(Clone, Copy)]
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
        resets: u32,
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

        fn session_reset(&mut self) {
            self.resets += 1;
        }
    }

    struct RecordingFence<'a>(&'a Cell<u32>);

    impl CommandFence for RecordingFence<'_> {
        fn finish_command(&self) {
            self.0.set(self.0.get() + 1);
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
            PACKET,
            Recording {
                calls: 0,
                resets: 0,
                respond,
            },
        )
    }

    fn fenced_probe<'f>(
        scan: &'static ScanChainStorage<1>,
        calls: &'f Cell<u32>,
    ) -> SwdOnlyDap<'static, Pin, Delay, NoSwo, NoVendor, NoopLeds, RecordingFence<'f>> {
        DapProbe::with_vendor(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, scan),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            PACKET,
            NoVendor,
        )
        .with_command_fence(RecordingFence(calls))
    }

    /// TDO reads low here, so every JTAG ACK is `0b000`: no ACK.
    #[test]
    fn jtag_failures_keep_their_status_and_sessions_forget_the_protocol() {
        use daplink_core::jtag_dp::{STATUS_FAULT, STATUS_OK};
        static SCAN: ScanChainStorage<2> = ScanChainStorage::new();
        let deps = LateJtagDeps::wired(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN);
        let mut p: BitbangDap<'static, Pin, Delay> =
            DapProbe::new(deps, NoopLeds, Delay, NoSwo, DAP_FW_VERSION, PACKET);
        let dp = SCAN.jtag_dp();
        let mut resp = [0u8; 64];
        p.process_command(&[0x02, 0x02], &mut resp);
        p.process_command(&[0x15, 0x01, 0x04], &mut resp);
        assert_eq!(p.process_command(&[0x05, 0, 1, 0x02], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x05, 0, 0x07], "no ACK, not retried as WAIT");
        p.process_command(&[0x05, 0, 0], &mut resp);
        assert_eq!(
            &resp[..3],
            &[0x05, 0, 0],
            "empty, as Arm answers it; dap-rs says 0x08"
        );

        for session in [&[0x02, 0x02][..], &[0x03], &[0x15, 0x01, 0x04]] {
            dp.status(0, 0b100);
            assert_eq!(dp.status(0, 0b010), STATUS_FAULT);
            p.process_command(session, &mut resp);
            assert_eq!(dp.status(0, 0b010), STATUS_OK, "{session:02x?}");
        }
    }

    #[test]
    fn vendor_ids_reach_the_handler_and_echo_the_command() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = probe(&SCAN, true);
        let mut resp = [0u8; 64];
        assert_eq!(p.process_command(&[0x81, 0x00], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x81, 0xAB, 0xCD]);
        assert_eq!(p.vendor().calls, 1);
        let resets = p.vendor().resets;
        p.suspend();
        assert_eq!(p.vendor().resets, resets + 1);
        p.vendor_mut().calls = 0;
        assert_eq!(p.vendor().calls, 0);
    }

    #[test]
    fn handled_vendor_commands_run_one_completion_fence() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let calls = Cell::new(0);
        let mut p = DapProbe::with_vendor(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            PACKET,
            Recording {
                calls: 0,
                resets: 0,
                respond: true,
            },
        )
        .with_command_fence(RecordingFence(&calls));
        let mut resp = [0u8; 64];

        assert_eq!(p.process_command(&[0x81, 0x00], &mut resp), 3);
        assert_eq!(calls.get(), 1);
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

    #[test]
    fn transfer_commands_run_one_completion_fence() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let calls = Cell::new(0);
        let mut p = fenced_probe(&SCAN, &calls);
        let mut resp = [0u8; 64];

        p.process_command(&[0x02, 0x01], &mut resp);
        p.process_command(&[0x00, 0xFE], &mut resp);
        assert_eq!(calls.get(), 0);

        p.process_command(&[DAP_TRANSFER, 0, 0], &mut resp);
        assert_eq!(calls.get(), 1);
        p.process_command(&[DAP_TRANSFER_BLOCK, 0, 0, 0, 0], &mut resp);
        assert_eq!(calls.get(), 2);
        p.process_command(&[DAP_WRITE_ABORT, 0, 0, 0, 0, 0], &mut resp);
        assert_eq!(calls.get(), 3);

        // dap-rs's TransferAbort is synchronous and cannot leave accelerator work pending.
        assert_eq!(p.process_command(&[0x07], &mut resp), 0);
        assert_eq!(calls.get(), 3);
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
            PACKET,
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
            PACKET,
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
            PACKET,
        )
        .with_packet_count(4);
        assert_eq!(pipelined.process_command(&[0x00, 0xFE], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x00, 0x01, 4]);

        // The byte patch must preserve packet counts greater than four.
        let mut pipelined = pipelined.with_packet_count(8);
        assert_eq!(pipelined.process_command(&[0x00, 0xFE], &mut resp), 3);
        assert_eq!(resp[2], 8);

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
            PACKET,
        )
        .with_packet_count(4);
        let mut resp = [0u8; 64];
        let len = p.process_command(&[0x00, 0x04], &mut resp);
        let want = DAP_FW_VERSION.as_bytes();
        assert!(resp[..len].windows(want.len()).any(|w| w == want));
    }

    static IDENTITY: Identity = Identity {
        vendor: "daplink-rs",
        product: "daplink-rs CMSIS-DAP (rp2040-pico)",
        serial: "5BCCF132445D5C49",
        board_vendor: "",
        board_name: "rp2040-pico",
    };

    fn info_probe(scan: &'static ScanChainStorage<1>) -> SwdOnlyDap<'static, Pin, Delay> {
        DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, scan),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            PACKET,
        )
    }

    /// One `DAP_Info` string reply, as `(length byte, string bytes)`, checked for its framing.
    fn info_string(len: usize, resp: &[u8]) -> (u8, &[u8]) {
        assert_eq!(resp[0], 0x00, "the reply echoes the DAP_Info command id");
        let count = resp[1] as usize;
        assert_eq!(
            len,
            2 + count,
            "the reply is the length byte plus its string"
        );
        if count > 0 {
            assert_eq!(resp[len - 1], 0, "the string is NUL-terminated");
        }
        (resp[1], &resp[2..len.saturating_sub(1)])
    }

    /// Every identity id is answered with its string and a length that counts the terminator.
    #[test]
    fn identity_ids_answer_nul_terminated_strings() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = info_probe(&SCAN).with_identity(IDENTITY);
        let mut resp = [0u8; 64];
        for (id, want) in [
            (0x01u8, IDENTITY.vendor),
            (0x02, IDENTITY.product),
            (0x03, IDENTITY.serial),
            (0x08, IDENTITY.board_name),
        ] {
            let len = p.process_command(&[0x00, id], &mut resp);
            let (count, text) = info_string(len, &resp);
            assert_eq!(count as usize, want.len() + 1, "id {id:#04x} length");
            assert_eq!(text, want.as_bytes(), "id {id:#04x} string");
        }
    }

    /// An identity field left empty answers length 0, as does an id with no identity wired at all.
    #[test]
    fn a_missing_field_answers_length_zero() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut resp = [0u8; 64];
        let mut p = info_probe(&SCAN).with_identity(IDENTITY);
        assert_eq!(p.process_command(&[0x00, 0x07], &mut resp), 2);
        assert_eq!(&resp[..2], &[0x00, 0x00]);

        static BARE_SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut bare = info_probe(&BARE_SCAN);
        for id in [0x01u8, 0x02, 0x03, 0x07, 0x08] {
            assert_eq!(bare.process_command(&[0x00, id], &mut resp), 2);
            assert_eq!(&resp[..2], &[0x00, 0x00], "id {id:#04x}");
        }
    }

    #[test]
    fn replacing_identity_uses_the_new_owned_value() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let first = Identity {
            board_vendor: "first vendor",
            ..IDENTITY
        };
        let mut p = info_probe(&SCAN).with_identity(first);
        let mut resp = [0u8; 64];
        let len = p.process_command(&[0x00, 0x07], &mut resp);
        assert_eq!(info_string(len, &resp).1, b"first vendor");

        let replacement = Identity {
            vendor: "second vendor",
            product: "second product",
            serial: "second serial",
            board_vendor: "second board vendor",
            board_name: "second board",
        };
        p = p.with_identity(replacement);
        for (id, want) in [
            (0x01u8, replacement.vendor),
            (0x02, replacement.product),
            (0x03, replacement.serial),
            (0x07, replacement.board_vendor),
            (0x08, replacement.board_name),
        ] {
            let len = p.process_command(&[0x00, id], &mut resp);
            assert_eq!(info_string(len, &resp).1, want.as_bytes());
        }
    }

    #[test]
    fn probes_keep_independent_identity_copies() {
        static FIRST_SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        static SECOND_SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut identity = IDENTITY;
        let mut first = info_probe(&FIRST_SCAN).with_identity(identity);
        identity.serial = "another serial";
        let mut second = info_probe(&SECOND_SCAN).with_identity(identity);
        let mut resp = [0u8; 64];
        for _ in 0..2 {
            let len = first.process_command(&[0x00, 0x03], &mut resp);
            assert_eq!(info_string(len, &resp).1, IDENTITY.serial.as_bytes());
            let len = second.process_command(&[0x00, 0x03], &mut resp);
            assert_eq!(info_string(len, &resp).1, b"another serial");
        }
        first = first.with_identity(Identity::EMPTY);
        assert_eq!(first.process_command(&[0x00, 0x03], &mut resp), 2);
        let len = second.process_command(&[0x00, 0x03], &mut resp);
        assert_eq!(info_string(len, &resp).1, b"another serial");
    }

    #[test]
    fn empty_identity_fields_all_answer_length_zero() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = info_probe(&SCAN)
            .with_identity(IDENTITY)
            .with_identity(Identity::EMPTY);
        let mut resp = [0xAAu8; 64];
        for id in [0x01u8, 0x02, 0x03, 0x07, 0x08] {
            assert_eq!(p.process_command(&[0x00, id], &mut resp), 2);
            assert_eq!(&resp[..2], &[0x00, 0x00], "id {id:#04x}");
            assert_eq!(resp[2], 0xAA, "empty fields do not emit a terminator");
        }
    }

    #[test]
    fn identity_survives_tuple_vendor_and_fence_composition() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let calls = Cell::new(0);
        let mut p: SwdOnlyDap<'static, Pin, Delay, NoSwo, _, _, _> = DapProbe::with_vendor(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            PACKET,
            (
                NoVendor,
                Recording {
                    calls: 0,
                    resets: 0,
                    respond: true,
                },
            ),
        )
        .with_identity(IDENTITY)
        .with_command_fence(RecordingFence(&calls))
        .with_packet_count(4);
        let mut resp = [0u8; 64];
        let len = p.process_command(&[0x00, 0x03], &mut resp);
        assert_eq!(resp[1] as usize, IDENTITY.serial.len() + 1);
        assert_eq!(info_string(len, &resp).1, IDENTITY.serial.as_bytes());
        assert_eq!(p.vendor().1.calls, 0);
        assert_eq!(p.process_command(&[0x81], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x81, 0xAB, 0xCD]);
        assert_eq!(p.vendor().1.calls, 1);
        assert_eq!(calls.get(), 1);
        assert_eq!(p.process_command(&[0x00, 0xFE], &mut resp), 3);
        assert_eq!(&resp[..3], &[0x00, 0x01, 4]);
        let mut p = p
            .with_command_fence(NoCommandFence)
            .with_identity(Identity::EMPTY);
        assert_eq!(p.process_command(&[0x00, 0x03], &mut resp), 2);
        assert_eq!(&resp[..2], &[0x00, 0x00]);
    }

    /// `DAP_Info(0x04)` gains the terminator dap-rs omits, with or without an identity.
    #[test]
    fn the_firmware_version_is_nul_terminated() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = info_probe(&SCAN);
        let mut resp = [0u8; 64];
        let len = p.process_command(&[0x00, 0x04], &mut resp);
        let (count, text) = info_string(len, &resp);
        assert_eq!(count as usize, DAP_FW_VERSION.len() + 1);
        assert_eq!(text, DAP_FW_VERSION.as_bytes());
    }

    /// `DAP_Info(0x09)` names the row, with or without an identity, and leaves `0x04` alone.
    #[test]
    fn the_firmware_row_answers_on_its_own_id() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = info_probe(&SCAN);
        let mut resp = [0u8; 64];
        let len = p.process_command(&[0x00, 0x09], &mut resp);
        let (count, text) = info_string(len, &resp);
        assert_eq!(count as usize, daplink_core::DAP_FW_ROW.len() + 1);
        assert_eq!(text, daplink_core::DAP_FW_ROW.as_bytes());
        let len = p.process_command(&[0x00, 0x04], &mut resp);
        assert_eq!(info_string(len, &resp).1, DAP_FW_VERSION.as_bytes());
    }

    /// A string longer than the response packet is truncated to what fits, terminator included.
    #[test]
    fn a_string_too_long_for_the_packet_is_truncated() {
        let mut resp = [0u8; 8];
        let len = write_info_string("0123456789", &mut resp);
        assert_eq!(len, 8);
        assert_eq!(&resp[..8], b"\x00\x0601234\x00");
    }

    /// Ids this probe does not own still reach the engine.
    #[test]
    fn unowned_dap_info_ids_still_reach_the_engine() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = info_probe(&SCAN).with_identity(IDENTITY);
        let mut resp = [0u8; 64];
        // MaxPacketSize is the engine's to answer, and it is a u16, not a string.
        assert_eq!(p.process_command(&[0x00, 0xFF], &mut resp), 4);
        assert_eq!(&resp[..4], &[0x00, 0x02, 64, 0]);
    }

    /// The guard sits between vendor routing and the engine and rejects a truncated command.
    #[test]
    fn guard_rejects_truncated_reports_ahead_of_the_engine() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p: SwdOnlyDap<'static, Pin, Delay> = DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, &SCAN),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            PACKET,
        );
        let mut resp = [0u8; 64];
        // DAP_SWJ_Clock (0x11) needs a u32; two bytes short must not reach the engine.
        assert_eq!(p.process_command(&[0x11, 1, 2], &mut resp), 1);
        assert_eq!(resp[0], 0xFF);
    }

    /// A payload-less vendor id reaches the handler rather than being guard-rejected.
    #[test]
    fn guard_does_not_intercept_vendor_ids() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = probe(&SCAN, true);
        let mut resp = [0u8; 64];
        assert_eq!(p.process_command(&[0x81], &mut resp), 3);
        assert_eq!(p.vendor().calls, 1);
    }

    /// The engine under the guard, at one response capacity, driven by a report and a buffer.
    fn sweep_probe(
        scan: &'static ScanChainStorage<1>,
        size: PacketSize,
    ) -> SwdOnlyDap<'static, Pin, Delay> {
        DapProbe::new(
            wired_swd_deps(Pin, Pin, Pin, Pin, Pin, Delay, scan),
            NoopLeds,
            Delay,
            NoSwo,
            DAP_FW_VERSION,
            size,
        )
    }

    /// One command through the guard and engine: the reply fits, and a rejection is the guard's own.
    fn dispatch_bounds(p: &mut SwdOnlyDap<'static, Pin, Delay>, report: &[u8], resp: &mut [u8]) {
        let cap = resp.len();
        let rejected = crate::guard::check(report, cap) == crate::guard::Verdict::Reject;
        let len = p.process_command(report, resp);
        assert!(
            len <= cap,
            "reply {len} overran a {cap}-byte buffer: {report:?}"
        );
        if rejected {
            assert_eq!(len, 1, "a rejected report answers one byte: {report:?}");
            assert_eq!(resp[0], 0xFF, "a rejected report answers 0xFF: {report:?}");
        }
    }

    #[test]
    fn every_truncated_prefix_through_the_engine_stays_inside_the_response_buffer() {
        static SCAN_64: ScanChainStorage<1> = ScanChainStorage::new();
        static SCAN_512: ScanChainStorage<1> = ScanChainStorage::new();
        let mut buf = [0u8; 512];
        for (size, scan) in [(PACKET, &SCAN_64), (PACKET_512, &SCAN_512)] {
            let cap = usize::from(size.get());
            let mut p = sweep_probe(scan, size);
            for cmd in 0x00u8..=0x1E {
                let (full, _) = crate::guard::tests::canonical(cmd);
                for len in 0..=full.len() {
                    dispatch_bounds(&mut p, &full[..len], &mut buf[..cap]);
                }
            }
        }
    }

    #[test]
    fn arbitrary_reports_through_the_engine_stay_inside_the_response_buffer() {
        static SCAN_64: ScanChainStorage<1> = ScanChainStorage::new();
        static SCAN_512: ScanChainStorage<1> = ScanChainStorage::new();
        let mut buf = [0u8; 512];
        for (size, scan) in [(PACKET, &SCAN_64), (PACKET_512, &SCAN_512)] {
            let cap = usize::from(size.get());
            let mut p = sweep_probe(scan, size);
            let mut rng = crate::guard::tests::Lcg::new(0x0DA9_1147);
            for _ in 0..10_000 {
                let report = rng.report(cap);
                dispatch_bounds(&mut p, &report, &mut buf[..cap]);
            }
        }
    }

    #[test]
    fn vendor_and_unknown_ids_reach_the_engine_rather_than_the_guard() {
        static SCAN: ScanChainStorage<1> = ScanChainStorage::new();
        let mut p = probe(&SCAN, false);
        let mut resp = [0u8; 64];
        for cmd in [0x80u8, 0x9F, 0xA0, 0x1F, 0x42, 0xFF] {
            let before = p.vendor().calls;
            // A bare id with no payload is never a guard rejection on these ranges.
            assert_eq!(
                crate::guard::check(&[cmd], 64),
                crate::guard::Verdict::Dispatch
            );
            assert_eq!(p.process_command(&[cmd], &mut resp), 1, "cmd {cmd:#04x}");
            // The engine, not the guard, answers the unknown id; the reply byte is the same.
            assert_eq!(resp[0], 0xFF, "cmd {cmd:#04x}");
            let routed = (0x80..=0x9F).contains(&cmd);
            assert_eq!(p.vendor().calls, before + u32::from(routed));
        }
        // The atomic-command stubs are known ids: the engine echoes rather than refusing them.
        for cmd in [0x7Eu8, 0x7F] {
            assert_eq!(p.process_command(&[cmd], &mut resp), 1);
            assert_eq!(resp[0], cmd);
        }
    }
}
