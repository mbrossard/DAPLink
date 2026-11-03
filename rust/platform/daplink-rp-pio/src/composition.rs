//! One composition combining complete-transfer SWD and JTAG on a shared accelerator.

use daplink_rs::dap_rs::dap::TransferConfig;
use daplink_rs::dap_rs::jtag::{self, Jtag, SequenceInfo, TransferResult};
use daplink_rs::dap_rs::swd::{self, RnW};
use daplink_rs::dap_rs::swj::{Dependencies, Pins};

use crate::generic_roles::PioJtagDrv;
use crate::swd_transaction_hybrid::{
    PioSwdTransactionDeps, PioSwdTransactionDrv, PioSwdTransferAccelerator,
};
use crate::{PioJtagShifter, PioShifter, SwjPinsSio};
use daplink_rs::daplink_core::{JtagDp, JtagDpSource};
use daplink_rs::jtag_result;

pub(crate) const MAX_BYPASS_BEFORE: u8 = 26;
// Only the selected-first-of-two-TAPs geometry is eligible for accelerated transfers.
const QUALIFIED_GEOMETRY: (u8, u8) = (0, 1);
const QUALIFIED_IR: (u16, u8, u16) = (0, 4, 5);

/// Hardware operations required by the dedicated complete-transfer state machine.
pub trait PioJtagAccelerator {
    /// Program the JTAG clock ceiling; one below the slowest divider runs there and returns `false`.
    fn set_clock(&mut self, max_frequency: u32) -> bool;

    /// Claim pads from an engine parked with TCK low and select one scan-chain geometry.
    fn enter(&mut self, bypass_before: u8, bypass_after: u8);

    /// Hand pads to a generic engine already staged with TCK low, then release them.
    fn exit(&mut self);

    /// One complete request/data scan, leaving early on WAIT only: the ACK and the captured word.
    fn transfer(&mut self, request: u8, data: u32) -> (u32, u32);

    /// Queue a supported IR scan; implementations must drain it before exit or clock changes.
    fn shift_ir(&mut self, _ir: u32) -> bool {
        false
    }
}

/// Dependencies carrying generic PIO0 SWJ and the shared transfer accelerator.
pub type PioTransferDeps<S, A> = PioSwdTransactionDeps<S, A>;

/// Active complete-transfer SWD role.
pub type PioTransferSwd<S, A> = PioSwdTransactionDrv<S, A>;

/// Active JTAG role with generic sequences and accelerated eligible transfers.
pub struct PioTransferJtag<S, A> {
    generic: PioJtagDrv<S>,
    accelerator: A,
    active_geometry: Option<(u8, u8)>,
    // Set when dap-rs may have reconfigured the chain; the next transfer re-checks its geometry.
    config_touched: bool,
}

impl<S, A> From<PioTransferDeps<S, A>> for PioTransferJtag<S, A>
where
    S: PioShifter + PioJtagShifter,
{
    fn from(deps: PioTransferDeps<S, A>) -> Self {
        let (generic, accelerator) = deps.into_parts();
        Self {
            generic: generic.into_jtag(),
            accelerator,
            active_geometry: None,
            config_touched: false,
        }
    }
}

impl<S, A> PioTransferJtag<S, A>
where
    S: PioShifter + PioJtagShifter,
    A: PioJtagAccelerator,
{
    fn ensure_generic(&mut self) {
        if self.active_geometry.take().is_some() {
            // Stage PIO0 low before PIO1 moves FUNCSEL, so the handoff cannot create a TCK edge.
            self.generic.resume_external_jtag();
            self.accelerator.exit();
        }
    }

    fn accelerated(&mut self, request: u8, data: u32) -> TransferResult {
        let (ack, data) = self.accelerator.transfer(request, data);
        let index = self.generic.config().index;
        jtag_result(self.generic.jtag_dp(), index, ack, data)
    }

    fn ensure_accelerator(&mut self, bypass_before: u8, bypass_after: u8) {
        let geometry = (bypass_before, bypass_after);
        if self.active_geometry == Some(geometry) {
            return;
        }
        self.ensure_generic();
        self.generic.complete_external_jtag();
        self.generic.suspend_external_jtag();
        self.accelerator.enter(bypass_before, bypass_after);
        self.active_geometry = Some(geometry);
    }
}

impl<S, A> From<PioTransferJtag<S, A>> for PioTransferDeps<S, A>
where
    S: PioShifter + PioJtagShifter,
    A: PioJtagAccelerator,
{
    fn from(mut jtag: PioTransferJtag<S, A>) -> Self {
        jtag.ensure_generic();
        PioSwdTransactionDeps::new(jtag.generic.into_deps(), jtag.accelerator)
    }
}

impl<S, A> JtagDpSource for PioTransferDeps<S, A> {
    fn jtag_dp(&self) -> Option<&'static JtagDp> {
        Some(self.generic.jtag_dp())
    }
}

impl<S, A> Dependencies<PioTransferSwd<S, A>, PioTransferJtag<S, A>> for PioTransferDeps<S, A>
where
    S: PioShifter + PioJtagShifter + SwjPinsSio,
    A: PioSwdTransferAccelerator + PioJtagAccelerator,
{
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.generic.swj_pins(output, mask, wait_us)
    }

    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.generic.swj_sequence(data, nbits);
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        let generic = self.generic.set_swj_clock(max_frequency);
        let accelerator =
            PioSwdTransferAccelerator::set_clock(&mut self.accelerator, max_frequency);
        generic && accelerator
    }

    fn high_impedance_mode(&mut self) {
        self.generic.high_impedance_mode();
    }

    fn swd_config(&mut self) -> &mut swd::Config {
        self.generic.swd_config()
    }

    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.generic.jtag_config()
    }
}

impl<S, A> Jtag<PioTransferDeps<S, A>> for PioTransferJtag<S, A>
where
    S: PioShifter + PioJtagShifter + SwjPinsSio,
    A: PioSwdTransferAccelerator + PioJtagAccelerator,
{
    fn available(deps: &PioTransferDeps<S, A>) -> bool {
        deps.generic.jtag_available()
    }

    fn config(&mut self) -> &mut jtag::Config {
        self.config_touched = true;
        self.generic.config()
    }

    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        self.ensure_generic();
        self.generic.sequence(info, tdi, rxbuf);
    }

    fn tms_sequence(&mut self, tms: &[bool]) {
        self.ensure_generic();
        self.generic.tms_sequence(tms);
    }

    fn shift_ir(&mut self, ir: u32) {
        let tap = self.generic.config().current_tap();
        let ir_geometry = (tap.ir_before, tap.ir_length, tap.ir_after);
        if self.active_geometry.is_some()
            && ir_geometry == QUALIFIED_IR
            && self.accelerator.shift_ir(ir)
        {
            return;
        }
        self.ensure_generic();
        self.generic.shift_ir(ir);
    }

    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let request = (a2a3 << 1) | (r_nw as u8);
        if !self.config_touched
            && transfer_config.idle_cycles == 0
            && self.active_geometry == Some(QUALIFIED_GEOMETRY)
        {
            return self.accelerated(request, data);
        }

        let config = self.generic.config();
        let before = config.index;
        let after = before
            .checked_add(1)
            .and_then(|first_after| config.device_count.checked_sub(first_after));
        let geometry = after.map(|after| (before, after));
        let eligible = transfer_config.idle_cycles == 0
            && before <= MAX_BYPASS_BEFORE
            && geometry == Some(QUALIFIED_GEOMETRY);
        if eligible {
            let after = geometry.map_or(0, |(_, after)| after);
            self.ensure_accelerator(before, after);
            self.config_touched = false;
            self.accelerated(request, data)
        } else {
            self.ensure_generic();
            self.config_touched = false;
            self.generic.transfer(r_nw, a2a3, transfer_config, data)
        }
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        let generic = self.generic.set_clock(max_frequency);
        let accelerator = PioJtagAccelerator::set_clock(&mut self.accelerator, max_frequency);
        generic && accelerator
    }
}

/// DAP engine combining complete-transfer PIO SWD and JTAG; SWD writes may complete after return, hence the fence.
pub type PioTransferDap<'a, S, A, WAIT, SWO, V, LEDS, F> = daplink_rs::DapProbe<
    'a,
    PioTransferDeps<S, A>,
    LEDS,
    WAIT,
    PioTransferJtag<S, A>,
    PioTransferSwd<S, A>,
    SWO,
    V,
    F,
>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::PioClockDivider;
    use crate::generic_roles::PioSwdDeps;
    use crate::program::{JtagProgramEntries, ProgramEntries};
    use crate::sequence::XferError;
    use daplink_rs::dap_rs::swd::Swd;
    use daplink_rs::ScanChainStorage;
    use std::boxed::Box;
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::vec::Vec;

    #[derive(Debug, PartialEq, Eq)]
    enum Event {
        SwdEnter,
        SwdExit,
        SwdRead,
        JtagEnter,
        JtagExit,
        JtagTransfer,
    }

    struct MockShifter;

    impl PioShifter for MockShifter {
        fn put(&mut self, _word: u32) {}
        fn get(&mut self) -> u32 {
            0
        }
        fn set_clock_divider(&mut self, _divider: u16) {}
        fn set_enabled(&mut self, _enabled: bool) {}
        fn clear_fifos(&mut self) {}
        fn restart(&mut self) {}
        fn exec_jmp(&mut self, _addr: u8) {}
        fn force_oen_released(&mut self) {}
        fn force_swclk_low(&mut self) {}
        fn force_pindirs_idle(&mut self) {}
        fn pads_to_sio(&mut self) {}
        fn pads_to_pio(&mut self) {}
    }

    impl PioJtagShifter for MockShifter {
        fn jtag_put(&mut self, _word: u32) {}
        fn jtag_get(&mut self) -> u32 {
            0
        }
        fn set_jtag_clock_divider(&mut self, _divider: PioClockDivider) {}
        fn enter_jtag(&mut self, _dispatcher: u8) {}
        fn exit_jtag(&mut self) {}
    }

    impl SwjPinsSio for MockShifter {
        fn swj_pins(&mut self, _output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
            Pins::empty()
        }
        fn release_nreset(&mut self) {}
    }

    struct MockAccelerator(Rc<RefCell<Vec<Event>>>);

    impl PioSwdTransferAccelerator for MockAccelerator {
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn enter(&mut self) {
            self.0.borrow_mut().push(Event::SwdEnter);
        }
        fn exit(&mut self) {
            self.0.borrow_mut().push(Event::SwdExit);
        }
        fn read_transfer(&mut self, _ap: bool, _addr: u8) -> Result<u32, XferError> {
            self.0.borrow_mut().push(Event::SwdRead);
            Ok(0)
        }
        fn write_transfer(&mut self, _ap: bool, _addr: u8, _data: u32) -> Result<(), XferError> {
            Ok(())
        }
    }

    impl PioJtagAccelerator for MockAccelerator {
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn enter(&mut self, _before: u8, _after: u8) {
            self.0.borrow_mut().push(Event::JtagEnter);
        }
        fn exit(&mut self) {
            self.0.borrow_mut().push(Event::JtagExit);
        }
        fn transfer(&mut self, _request: u8, _data: u32) -> (u32, u32) {
            self.0.borrow_mut().push(Event::JtagTransfer);
            (0b010, 0)
        }
    }

    fn deps(events: Rc<RefCell<Vec<Event>>>) -> PioTransferDeps<MockShifter, MockAccelerator> {
        let swd = ProgramEntries {
            turnaround_cmd: 0,
            write_cmd: 0,
            get_next_cmd: 3,
            read_cmd: 8,
            release_cmd: 11,
        };
        let jtag = JtagProgramEntries {
            get_next_cmd: 13,
            tms_low_data_cmd: 17,
            tms_low_inline_cmd: 18,
            tms_high_data_cmd: 23,
            tms_high_inline_cmd: 24,
        };
        let scan = Box::leak(Box::new(ScanChainStorage::<8>::new()));
        let mut generic = PioSwdDeps::new_jtag(MockShifter, swd, jtag, 200_000_000, scan);
        {
            let config = generic.jtag_config();
            config.device_count = 2;
            config.index = 0;
            config.scan_chain[0] = jtag::TapConfig {
                ir_length: 4,
                ir_before: 0,
                ir_after: 5,
            };
            config.scan_chain[1] = jtag::TapConfig {
                ir_length: 5,
                ir_before: 4,
                ir_after: 0,
            };
        }
        PioSwdTransactionDeps::new(generic, MockAccelerator(events))
    }

    #[test]
    fn one_accelerator_value_crosses_swd_jtag_and_back() {
        let events = Rc::new(RefCell::new(Vec::new()));
        let deps = deps(Rc::clone(&events));
        let mut swd: PioTransferSwd<_, _> = deps.into();
        assert_eq!(
            swd.read_inner(swd::APnDP::DP, swd::DPRegister::DPIDR),
            Ok(0)
        );
        let deps: PioTransferDeps<_, _> = swd.into();

        let mut jtag: PioTransferJtag<_, _> = deps.into();
        assert_eq!(
            jtag.transfer(
                RnW::R,
                0,
                &TransferConfig {
                    idle_cycles: 0,
                    wait_retries: 0,
                    match_retries: 0,
                    match_mask: 0,
                },
                0,
            ),
            TransferResult::Ok(0)
        );
        let deps: PioTransferDeps<_, _> = jtag.into();

        let mut swd: PioTransferSwd<_, _> = deps.into();
        assert_eq!(
            swd.read_inner(swd::APnDP::DP, swd::DPRegister::DPIDR),
            Ok(0)
        );
        let _: PioTransferDeps<_, _> = swd.into();

        assert_eq!(
            *events.borrow(),
            [
                Event::SwdEnter,
                Event::SwdRead,
                Event::SwdExit,
                Event::JtagEnter,
                Event::JtagTransfer,
                Event::JtagExit,
                Event::SwdEnter,
                Event::SwdRead,
                Event::SwdExit,
            ]
        );
    }

    mod jtag_eligibility {
        use super::*;

        #[derive(Debug, PartialEq, Eq)]
        enum Event {
            GenericEnter,
            GenericExit,
            GenericPark,
            GenericCompletion,
            AcceleratorEnter(u8, u8),
            AcceleratorExit,
            AcceleratorTransfer(u8, u32),
            AcceleratorShiftIr(u32),
        }

        #[derive(Debug, PartialEq, Eq)]
        enum HandoffEvent {
            GenericParkedLow,
            AcceleratorClaimedLow,
            GenericResumedLow,
            AcceleratorReleased,
        }

        struct MockShifter {
            events: Vec<Event>,
            handoff: Rc<RefCell<Vec<HandoffEvent>>>,
        }

        impl PioShifter for MockShifter {
            fn put(&mut self, _word: u32) {}
            fn get(&mut self) -> u32 {
                0
            }
            fn set_clock_divider(&mut self, _divider: u16) {}
            fn set_enabled(&mut self, _enabled: bool) {}
            fn clear_fifos(&mut self) {}
            fn restart(&mut self) {}
            fn exec_jmp(&mut self, _addr: u8) {}
            fn force_oen_released(&mut self) {}
            fn force_swclk_low(&mut self) {}
            fn force_pindirs_idle(&mut self) {}
            fn pads_to_sio(&mut self) {}
            fn pads_to_pio(&mut self) {}
        }

        impl PioJtagShifter for MockShifter {
            fn jtag_put(&mut self, _word: u32) {}
            fn jtag_get(&mut self) -> u32 {
                self.events.push(Event::GenericCompletion);
                // A one- or two-TAP request capture answering WAIT.
                0b001 << 29
            }
            fn set_jtag_clock_divider(&mut self, _divider: PioClockDivider) {}
            fn enter_jtag(&mut self, _dispatcher: u8) {
                self.events.push(Event::GenericEnter);
                self.handoff
                    .borrow_mut()
                    .push(HandoffEvent::GenericResumedLow);
            }
            fn exit_jtag(&mut self) {
                self.events.push(Event::GenericExit);
            }
            fn park_jtag(&mut self) {
                self.events.push(Event::GenericPark);
                self.handoff
                    .borrow_mut()
                    .push(HandoffEvent::GenericParkedLow);
            }
        }

        impl SwjPinsSio for MockShifter {
            fn swj_pins(&mut self, _output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
                Pins::empty()
            }
            fn release_nreset(&mut self) {}
        }

        struct MockAccelerator {
            events: Vec<Event>,
            handoff: Rc<RefCell<Vec<HandoffEvent>>>,
            result: (u32, u32),
        }

        impl PioJtagAccelerator for MockAccelerator {
            fn set_clock(&mut self, _max_frequency: u32) -> bool {
                true
            }
            fn enter(&mut self, before: u8, after: u8) {
                self.events.push(Event::AcceleratorEnter(before, after));
                self.handoff
                    .borrow_mut()
                    .push(HandoffEvent::AcceleratorClaimedLow);
            }
            fn exit(&mut self) {
                self.events.push(Event::AcceleratorExit);
                self.handoff
                    .borrow_mut()
                    .push(HandoffEvent::AcceleratorReleased);
            }
            fn transfer(&mut self, request: u8, data: u32) -> (u32, u32) {
                self.events.push(Event::AcceleratorTransfer(request, data));
                self.result
            }
            fn shift_ir(&mut self, ir: u32) -> bool {
                self.events.push(Event::AcceleratorShiftIr(ir));
                true
            }
        }

        // The JTAG role requires both accelerator traits; SWD is not exercised here.
        impl PioSwdTransferAccelerator for MockAccelerator {
            fn set_clock(&mut self, _max_frequency: u32) -> bool {
                true
            }
            fn enter(&mut self) {}
            fn exit(&mut self) {}
            fn read_transfer(&mut self, _ap: bool, _addr: u8) -> Result<u32, XferError> {
                Ok(0)
            }
            fn write_transfer(
                &mut self,
                _ap: bool,
                _addr: u8,
                _data: u32,
            ) -> Result<(), XferError> {
                Ok(())
            }
        }

        fn transfer_config(idle_cycles: u8) -> TransferConfig {
            TransferConfig {
                idle_cycles,
                wait_retries: 0,
                match_retries: 0,
                match_mask: 0,
            }
        }

        type MockDriver = PioTransferJtag<MockShifter, MockAccelerator>;

        fn driver_with_handoff_log() -> (MockDriver, Rc<RefCell<Vec<HandoffEvent>>>) {
            let handoff = Rc::new(RefCell::new(Vec::new()));
            let shifter = MockShifter {
                events: Vec::new(),
                handoff: Rc::clone(&handoff),
            };
            let swd = ProgramEntries {
                turnaround_cmd: 0,
                write_cmd: 0,
                get_next_cmd: 3,
                read_cmd: 8,
                release_cmd: 11,
            };
            let jtag = JtagProgramEntries {
                get_next_cmd: 13,
                tms_low_data_cmd: 17,
                tms_low_inline_cmd: 18,
                tms_high_data_cmd: 23,
                tms_high_inline_cmd: 24,
            };
            let scan = Box::leak(Box::new(ScanChainStorage::<8>::new()));
            let mut generic = PioSwdDeps::new_jtag(shifter, swd, jtag, 200_000_000, scan);
            {
                let config = generic.jtag_config();
                config.device_count = 2;
                config.index = 0;
                config.scan_chain[0] = jtag::TapConfig {
                    ir_length: 4,
                    ir_before: 0,
                    ir_after: 5,
                };
                config.scan_chain[1] = jtag::TapConfig {
                    ir_length: 5,
                    ir_before: 4,
                    ir_after: 0,
                };
            }
            let driver = PioTransferDeps::new(
                generic,
                MockAccelerator {
                    events: Vec::new(),
                    handoff: Rc::clone(&handoff),
                    result: (0b010, 0x1234_5678),
                },
            )
            .into();
            handoff.borrow_mut().clear();
            (driver, handoff)
        }

        fn driver() -> MockDriver {
            driver_with_handoff_log().0
        }

        #[test]
        fn eligible_transfers_stay_on_the_accelerator() {
            let mut driver = driver();
            let config = transfer_config(0);
            assert_eq!(
                driver.transfer(RnW::R, 2, &config, 0xa5a5_5a5a),
                TransferResult::Ok(0x1234_5678)
            );
            assert_eq!(
                driver.transfer(RnW::W, 1, &config, 0x55aa_aa55),
                TransferResult::Ok(0x1234_5678)
            );
            assert_eq!(
                driver.generic.test_shifter_mut().events,
                [
                    Event::GenericEnter,
                    Event::GenericCompletion,
                    Event::GenericPark,
                ]
            );
            assert_eq!(
                driver.accelerator.events,
                [
                    Event::AcceleratorEnter(0, 1),
                    Event::AcceleratorTransfer(5, 0xa5a5_5a5a),
                    Event::AcceleratorTransfer(2, 0x55aa_aa55),
                ]
            );
            assert!(!driver.config_touched);
        }

        #[test]
        fn configuration_access_revalidates_an_active_accelerator() {
            let mut driver = driver();
            let config = transfer_config(0);
            let _ = driver.transfer(RnW::R, 0, &config, 0);
            driver.config().device_count = 1;
            assert!(driver.config_touched);

            // The mock generic scan reads WAIT; the stale two-TAP accelerator would report OK.
            assert_eq!(driver.transfer(RnW::R, 0, &config, 0), TransferResult::Wait);
            assert!(!driver.config_touched);
            assert!(driver.active_geometry.is_none());
        }

        #[test]
        fn idle_cycles_force_fallback_and_restore_generic_pio() {
            let (mut driver, handoff) = driver_with_handoff_log();
            let fast = transfer_config(0);
            let slow = transfer_config(1);
            let _ = driver.transfer(RnW::R, 0, &fast, 0);
            assert_eq!(
                *handoff.borrow(),
                [
                    HandoffEvent::GenericParkedLow,
                    HandoffEvent::AcceleratorClaimedLow,
                ]
            );
            // The mock generic scan reads WAIT.
            assert_eq!(driver.transfer(RnW::R, 0, &slow, 0), TransferResult::Wait);
            assert_eq!(
                *handoff.borrow(),
                [
                    HandoffEvent::GenericParkedLow,
                    HandoffEvent::AcceleratorClaimedLow,
                    HandoffEvent::GenericResumedLow,
                    HandoffEvent::AcceleratorReleased,
                ]
            );
            assert_eq!(driver.accelerator.events[2], Event::AcceleratorExit);
            assert_eq!(
                driver.generic.test_shifter_mut().events,
                [
                    Event::GenericEnter,
                    Event::GenericCompletion,
                    Event::GenericPark,
                    Event::GenericEnter,
                    Event::GenericCompletion,
                ]
            );
        }

        #[test]
        fn unqualified_one_tap_geometry_uses_generic_pio() {
            let mut driver = driver();
            driver.generic.config().device_count = 1;
            let result = driver.transfer(RnW::R, 0, &transfer_config(0), 0);
            assert_eq!(result, TransferResult::Wait);
            assert!(driver.accelerator.events.is_empty());
        }

        #[test]
        fn instruction_scans_stay_on_an_active_accelerator() {
            let mut driver = driver();
            let config = transfer_config(0);
            let _ = driver.transfer(RnW::R, 0, &config, 0);
            driver.shift_ir(0x0a);
            let _ = driver.transfer(RnW::R, 0, &config, 0);
            assert_eq!(
                driver.generic.test_shifter_mut().events,
                [
                    Event::GenericEnter,
                    Event::GenericCompletion,
                    Event::GenericPark,
                ]
            );
            assert_eq!(
                driver.accelerator.events,
                [
                    Event::AcceleratorEnter(0, 1),
                    Event::AcceleratorTransfer(1, 0),
                    Event::AcceleratorShiftIr(0x0a),
                    Event::AcceleratorTransfer(1, 0),
                ]
            );
        }

        #[test]
        fn other_instruction_shapes_return_to_the_generic_engine() {
            let mut driver = driver();
            let config = transfer_config(0);
            let _ = driver.transfer(RnW::R, 0, &config, 0);
            driver.config().scan_chain[0].ir_after = 4;
            assert!(driver.active_geometry.is_some());
            driver.shift_ir(0x0a);
            assert!(driver.active_geometry.is_none());
            assert_eq!(
                driver.accelerator.events.last(),
                Some(&Event::AcceleratorExit)
            );
            assert!(!driver
                .accelerator
                .events
                .contains(&Event::AcceleratorShiftIr(0x0a)));
        }
    }
}
