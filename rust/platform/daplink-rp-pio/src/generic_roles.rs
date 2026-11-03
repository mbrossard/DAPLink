//! The PIO0 engine state and its three role views; the accelerated compositions wrap them.

use daplink_rs::dap_rs::jtag::{self, SequenceInfo};
use daplink_rs::dap_rs::swd::{self, RnW};
use daplink_rs::dap_rs::swj::Pins;
use daplink_rs::daplink_core::JtagDp;
use daplink_rs::{jtag_result, ScanChainStorage};

use crate::clock::{
    clock_divider, jtag_clock_divider, swd_cycles_per_bit, PioClockDivider, JTAG_MIN_DIVIDER,
    MIN_DIVIDER, SLOWEST_DIVIDER,
};
use crate::jtag::{JtagScan, PioJtag, PioJtagShifter};
use crate::program::{JtagProgramEntries, ProgramEntries};
use crate::sequence::{PioSwd, XferError};
use crate::shifter::{PioShifter, SwjPinsSio};

struct Inner<S> {
    engine: PioSwd<S>,
    jtag: Option<PioJtag>,
    sys_hz: u32,
    jtag_divider: PioClockDivider,
    swd_config: swd::Config,
    jtag_config: jtag::Config,
    jtag_dp: &'static JtagDp,
}

impl<S: PioShifter> Inner<S> {
    /// Both dividers, each clamped to the slowest; `false` when either was clamped.
    fn clock_dividers(&self, max_frequency: u32) -> (u16, PioClockDivider, bool) {
        let swd = swd_cycles_per_bit(self.sys_hz)
            .and_then(|cycles| clock_divider(self.sys_hz, max_frequency, cycles, MIN_DIVIDER));
        let jtag = if self.jtag.is_some() {
            jtag_clock_divider(self.sys_hz, max_frequency)
        } else {
            swd.map(PioClockDivider::integer)
        };
        (
            swd.unwrap_or(SLOWEST_DIVIDER.integer),
            jtag.unwrap_or(SLOWEST_DIVIDER),
            swd.is_some() && jtag.is_some(),
        )
    }

    fn set_swd_clock(&mut self, max_frequency: u32) -> bool {
        let (swd_divider, jtag_divider, exact) = self.clock_dividers(max_frequency);
        self.engine.set_clock_divider(swd_divider);
        self.jtag_divider = jtag_divider;
        exact
    }

    fn high_impedance_mode(&mut self) {
        self.engine.drain_and_release();
    }
}

impl<S: PioShifter + PioJtagShifter> Inner<S> {
    fn set_pio_clock(&mut self, max_frequency: u32) -> bool {
        let exact = self.set_swd_clock(max_frequency);
        let divider = self.jtag_divider;
        self.engine.shifter_mut().set_jtag_clock_divider(divider);
        exact
    }
}

/// The PIO0 SWJ state while neither protocol is selected.
pub struct PioSwdDeps<S> {
    inner: Inner<S>,
}

/// The PIO0 state while SWD is selected; sequences run here, register transfers on PIO1.
pub(crate) struct PioSwdDrv<S> {
    inner: Inner<S>,
}

/// The active PIO JTAG driver, carrying the parked SWD state machine across the role switch.
pub(crate) struct PioJtagDrv<S> {
    inner: Inner<S>,
}

impl<S: PioShifter + PioJtagShifter> PioJtagDrv<S> {
    pub(crate) fn complete_external_jtag(&mut self) {
        if let Some(jtag) = self.inner.jtag {
            let mut captured = [0];
            jtag.sequence(
                self.inner.engine.shifter_mut(),
                SequenceInfo {
                    n_bits: 1,
                    capture: true,
                    tms: false,
                },
                &[1],
                &mut captured,
            );
        }
    }

    pub(crate) fn suspend_external_jtag(&mut self) {
        self.inner.engine.shifter_mut().park_jtag();
    }

    pub(crate) fn resume_external_jtag(&mut self) {
        if let Some(jtag) = self.inner.jtag {
            self.inner
                .engine
                .shifter_mut()
                .enter_jtag(jtag.dispatcher());
        }
    }

    #[cfg(test)]
    pub(crate) fn test_shifter_mut(&mut self) -> &mut S {
        self.inner.engine.shifter_mut()
    }
}

impl<S: PioShifter> PioSwdDeps<S> {
    /// Build the dependencies over a running state machine and its loaded entry PCs.
    pub fn new<const N: usize>(
        shifter: S,
        entries: ProgramEntries,
        sys_hz: u32,
        scan: &'static ScanChainStorage<N>,
    ) -> Self {
        Self::new_inner(shifter, entries, None, sys_hz, scan)
    }

    /// Build dependencies with a second state machine loaded for JTAG sequences.
    pub fn new_jtag<const N: usize>(
        shifter: S,
        swd_entries: ProgramEntries,
        jtag_entries: JtagProgramEntries,
        sys_hz: u32,
        scan: &'static ScanChainStorage<N>,
    ) -> Self {
        Self::new_inner(shifter, swd_entries, Some(jtag_entries), sys_hz, scan)
    }

    fn new_inner<const N: usize>(
        shifter: S,
        swd_entries: ProgramEntries,
        jtag_entries: Option<JtagProgramEntries>,
        sys_hz: u32,
        scan: &'static ScanChainStorage<N>,
    ) -> Self {
        Self {
            inner: Inner {
                engine: PioSwd::new(shifter, swd_entries),
                jtag: jtag_entries.map(PioJtag::new),
                sys_hz,
                jtag_divider: jtag_clock_divider(sys_hz, 1_000_000)
                    .unwrap_or(PioClockDivider::integer(JTAG_MIN_DIVIDER)),
                swd_config: swd::Config::default(),
                jtag_config: jtag::Config::new(scan.init()),
                jtag_dp: scan.jtag_dp(),
            },
        }
    }
}

impl<S: PioShifter> PioSwdDrv<S> {
    pub(crate) fn suspend_for_accelerator(&mut self) {
        self.inner.engine.drain_and_release();
        self.inner.engine.shifter_mut().set_enabled(false);
    }

    pub(crate) fn resume_from_accelerator(&mut self) {
        self.inner.engine.recover();
    }
}

impl<S> PioSwdDeps<S> {
    pub(crate) fn jtag_dp(&self) -> &'static JtagDp {
        self.inner.jtag_dp
    }
    pub(crate) fn into_swd(self) -> PioSwdDrv<S> {
        PioSwdDrv { inner: self.inner }
    }
}

impl<S> PioSwdDrv<S> {
    pub(crate) fn jtag_dp(&self) -> &'static JtagDp {
        self.inner.jtag_dp
    }
    pub(crate) fn into_deps(self) -> PioSwdDeps<S> {
        PioSwdDeps { inner: self.inner }
    }
}

impl<S: PioShifter + PioJtagShifter> PioSwdDeps<S> {
    /// Program the JTAG divider, then hand the pads to the JTAG state machine.
    pub(crate) fn into_jtag(mut self) -> PioJtagDrv<S> {
        if let Some(jtag) = self.inner.jtag {
            let divider = self.inner.jtag_divider;
            self.inner
                .engine
                .shifter_mut()
                .set_jtag_clock_divider(divider);
            self.inner.engine.lend_to_jtag(jtag.dispatcher());
        }
        PioJtagDrv { inner: self.inner }
    }
}

impl<S: PioShifter + PioJtagShifter> PioJtagDrv<S> {
    pub(crate) fn into_deps(mut self) -> PioSwdDeps<S> {
        if self.inner.jtag.is_some() {
            self.inner.engine.reclaim_from_jtag();
        }
        PioSwdDeps { inner: self.inner }
    }
}

pub(crate) fn map_err(e: XferError) -> swd::Error {
    match e {
        XferError::AckWait => swd::Error::AckWait,
        XferError::AckFault => swd::Error::AckFault,
        XferError::NoAck => swd::Error::NoAck,
        XferError::BadParity => swd::Error::BadParity,
        XferError::Unknown(v) => swd::Error::AckUnknown(v),
    }
}

/// The SWJ operations the accelerated compositions delegate to PIO0.
impl<S: PioShifter + SwjPinsSio> PioSwdDeps<S> {
    pub(crate) fn swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.inner.engine.lend_pads();
        let read = self
            .inner
            .engine
            .shifter_mut()
            .swj_pins(output, mask, wait_us);
        self.inner.engine.reclaim_pads();
        read
    }
    pub(crate) fn swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.inner.engine.set_direction(true);
        self.inner.engine.write_sequence(nbits, data);
    }
    /// Park the pads, SWCLK excepted, which the PIO keeps driven, then release nRESET.
    pub(crate) fn high_impedance_mode(&mut self) {
        self.inner.high_impedance_mode();
        self.inner.engine.shifter_mut().release_nreset();
    }
}

impl<S: PioShifter> PioSwdDeps<S> {
    /// Program the SWD divider alone, for a composition with no JTAG state machine.
    pub(crate) fn set_swd_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_swd_clock(max_frequency)
    }
    pub(crate) fn swd_config(&mut self) -> &mut swd::Config {
        &mut self.inner.swd_config
    }
    pub(crate) fn jtag_config(&mut self) -> &mut jtag::Config {
        &mut self.inner.jtag_config
    }
    pub(crate) fn jtag_available(&self) -> bool {
        self.inner.jtag.is_some()
    }
}

impl<S: PioShifter + PioJtagShifter> PioSwdDeps<S> {
    /// Program both dividers, for a composition carrying the JTAG state machine.
    pub(crate) fn set_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_pio_clock(max_frequency)
    }
}

impl<S: PioShifter> PioSwdDrv<S> {
    pub(crate) fn config(&mut self) -> &mut swd::Config {
        &mut self.inner.swd_config
    }
    pub(crate) fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_swd_clock(max_frequency)
    }
    pub(crate) fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.inner.engine.set_direction(true);
        self.inner.engine.write_sequence(num_bits, data);
        Ok(())
    }
    pub(crate) fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.inner.engine.set_direction(false);
        self.inner.engine.read_sequence(num_bits, data);
        Ok(())
    }
}

impl<S: PioShifter + PioJtagShifter> PioJtagDrv<S> {
    pub(crate) fn config(&mut self) -> &mut jtag::Config {
        &mut self.inner.jtag_config
    }
    pub(crate) fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        let Some(jtag) = self.inner.jtag else {
            rxbuf.fill(0);
            return;
        };
        jtag.sequence(self.inner.engine.shifter_mut(), info, tdi, rxbuf);
    }
    pub(crate) fn tms_sequence(&mut self, tms: &[bool]) {
        if let Some(jtag) = self.inner.jtag {
            jtag.tms_sequence(self.inner.engine.shifter_mut(), tms);
        }
    }
    pub(crate) fn shift_ir(&mut self, ir: u32) {
        let tap = self.inner.jtag_config.current_tap();
        if let Some(jtag) = self.inner.jtag {
            jtag.shift_ir(
                self.inner.engine.shifter_mut(),
                ir,
                tap.ir_before,
                tap.ir_length,
                tap.ir_after,
            );
        }
    }
    pub(crate) fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &daplink_rs::dap_rs::dap::TransferConfig,
        data: u32,
    ) -> jtag::TransferResult {
        let Some(jtag) = self.inner.jtag else {
            return jtag::TransferResult::Fault;
        };
        let device_index = self.inner.jtag_config.index;
        let device_count = self.inner.jtag_config.device_count;
        let scan = JtagScan {
            device_index,
            device_count,
        };
        let shifter = self.inner.engine.shifter_mut();
        match jtag.transfer(shifter, scan, r_nw, a2a3, transfer_config, data) {
            Some((ack, data)) => jtag_result(self.inner.jtag_dp, device_index, ack, data),
            None => jtag::TransferResult::Fault,
        }
    }
    pub(crate) fn jtag_dp(&self) -> &'static JtagDp {
        self.inner.jtag_dp
    }
    pub(crate) fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_pio_clock(max_frequency)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;
    use std::vec::Vec;

    #[derive(Debug, PartialEq, Eq)]
    enum Op {
        Divider(u16),
        Enable(bool),
        OenReleased,
        SwclkLow,
        ClearFifos,
        Restart,
        Jmp(u8),
        PindirsIdle,
        JtagDivider(PioClockDivider),
        EnterJtag(u8),
        ExitJtag,
    }

    struct MockShifter {
        tx: Vec<u32>,
        ops: Vec<Op>,
    }

    impl PioShifter for MockShifter {
        fn put(&mut self, word: u32) {
            self.tx.push(word);
        }
        fn get(&mut self) -> u32 {
            0
        }
        fn set_clock_divider(&mut self, divider: u16) {
            self.ops.push(Op::Divider(divider));
        }
        fn set_enabled(&mut self, enabled: bool) {
            self.ops.push(Op::Enable(enabled));
        }
        fn clear_fifos(&mut self) {
            self.ops.push(Op::ClearFifos);
        }
        fn restart(&mut self) {
            self.ops.push(Op::Restart);
        }
        fn exec_jmp(&mut self, addr: u8) {
            self.ops.push(Op::Jmp(addr));
        }
        fn force_oen_released(&mut self) {
            self.ops.push(Op::OenReleased);
        }
        fn force_swclk_low(&mut self) {
            self.ops.push(Op::SwclkLow);
        }
        fn force_pindirs_idle(&mut self) {
            self.ops.push(Op::PindirsIdle);
        }
        fn pads_to_sio(&mut self) {}
        fn pads_to_pio(&mut self) {}
    }

    impl SwjPinsSio for MockShifter {
        fn swj_pins(&mut self, _output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
            Pins::empty()
        }
        fn release_nreset(&mut self) {}
    }

    impl PioJtagShifter for MockShifter {
        fn jtag_put(&mut self, _word: u32) {}
        fn jtag_get(&mut self) -> u32 {
            0
        }
        fn set_jtag_clock_divider(&mut self, divider: PioClockDivider) {
            self.ops.push(Op::JtagDivider(divider));
        }
        fn enter_jtag(&mut self, dispatcher: u8) {
            self.ops.push(Op::EnterJtag(dispatcher));
        }
        fn exit_jtag(&mut self) {
            self.ops.push(Op::ExitJtag);
        }
    }

    fn deps(sys_hz: u32) -> PioSwdDeps<MockShifter> {
        PioSwdDeps {
            inner: Inner {
                engine: PioSwd::new(
                    MockShifter {
                        tx: Vec::new(),
                        ops: Vec::new(),
                    },
                    crate::program::DIRECT.entries(0),
                ),
                jtag: None,
                sys_hz,
                jtag_divider: PioClockDivider::integer(42),
                swd_config: swd::Config::default(),
                jtag_config: jtag::Config::new(&mut []),
                jtag_dp: std::boxed::Box::leak(std::boxed::Box::new(JtagDp::new())),
            },
        }
    }

    fn set_clock(deps: &mut PioSwdDeps<MockShifter>, hz: u32) -> bool {
        deps.set_swd_clock(hz)
    }

    #[test]
    fn paired_clock_hook_programs_independent_swd_and_jtag_dividers() {
        let sys_hz = 150_000_000;
        let request = 25_000_000;
        let mut deps = deps(sys_hz);
        deps.inner.jtag = Some(PioJtag::new(
            crate::program::select_jtag_program(sys_hz).unwrap().entries,
        ));
        let accepted = deps.set_swj_clock(request);
        assert!(accepted);
        let cycles = swd_cycles_per_bit(sys_hz).unwrap();
        let swd = clock_divider(sys_hz, request, cycles, MIN_DIVIDER).unwrap();
        let jtag = jtag_clock_divider(sys_hz, request).unwrap();
        assert_eq!(deps.inner.jtag_divider, jtag);
        assert_eq!(
            deps.inner.engine.shifter_mut().ops,
            vec![Op::Divider(swd), Op::JtagDivider(jtag)]
        );
    }

    #[test]
    fn paired_clock_hook_clamps_both_dividers_below_the_slowest() {
        let sys_hz = 150_000_000;
        let mut deps = deps(sys_hz);
        deps.inner.jtag = Some(PioJtag::new(
            crate::program::select_jtag_program(sys_hz).unwrap().entries,
        ));
        assert!(!deps.set_swj_clock(1));
        assert_eq!(
            deps.inner.engine.shifter_mut().ops,
            vec![Op::Divider(0), Op::JtagDivider(SLOWEST_DIVIDER)]
        );
    }

    #[test]
    fn public_clock_hook_clamps_and_reports_unrepresentable_stock_requests() {
        for sys_hz in [125_000_000, 150_000_000] {
            let cycles = swd_cycles_per_bit(sys_hz).unwrap();
            let capacity = u64::from(cycles) * u64::from(crate::clock::MAX_DIVIDER);
            let accepted = (u64::from(sys_hz).div_ceil(capacity)) as u32;
            let rejected = accepted - 1;
            let encoded = clock_divider(sys_hz, accepted, cycles, MIN_DIVIDER).unwrap();
            let mut deps = deps(sys_hz);
            assert!(set_clock(&mut deps, accepted));
            assert_eq!(
                deps.inner.engine.shifter_mut().ops,
                vec![Op::Divider(encoded)]
            );
            assert!(!set_clock(&mut deps, rejected));
            assert!(!set_clock(&mut deps, 0));
            assert_eq!(
                deps.inner.engine.shifter_mut().ops,
                vec![Op::Divider(encoded), Op::Divider(0), Op::Divider(0)]
            );
        }
    }

    #[test]
    fn high_impedance_hook_uses_ordered_non_destructive_release() {
        let mut deps = deps(125_000_000);
        deps.high_impedance_mode();
        let entries = deps.inner.engine.entries();
        let shifter = deps.inner.engine.shifter_mut();
        assert!(shifter.ops.is_empty());
        assert_eq!(
            shifter.tx,
            vec![crate::codec::cmd_word(entries.release_cmd, false, 1)]
        );
    }
}
