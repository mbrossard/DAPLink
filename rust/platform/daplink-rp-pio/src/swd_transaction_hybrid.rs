//! Composition of generic SWJ operations with a dedicated SWD transfer state machine.

use daplink_rs::dap_rs::jtag::{self, Jtag, SequenceInfo};
use daplink_rs::dap_rs::swd::{self, APnDP, DPRegister, Swd};
use daplink_rs::dap_rs::swj::{Dependencies, Pins};
use daplink_rs::daplink_core::dap_wire::IDLE_LOW;
use daplink_rs::wait_retry;

use crate::generic_roles::{map_err, PioSwdDeps, PioSwdDrv};
use crate::sequence::XferError;
use crate::shifter::{PioShifter, SwjPinsSio};

/// One separately initialized state machine that executes fixed SWD register transfers.
pub trait PioSwdTransferAccelerator {
    /// Program the accelerator's SWCLK ceiling; one below its slowest divider runs there and returns `false`.
    fn set_clock(&mut self, max_frequency: u32) -> bool;

    /// Take the already-released pads from the generic engine.
    fn enter(&mut self);

    /// Finish all wire activity and return released pads to the generic engine.
    fn exit(&mut self);

    /// Read data/parity; a release suffix may remain until exit or the command fence.
    fn read_transfer(&mut self, ap: bool, addr: u8) -> Result<u32, XferError>;

    /// Accept ACK and queue data/parity; exit or the command fence must complete the suffix.
    fn write_transfer(&mut self, ap: bool, addr: u8, data: u32) -> Result<(), XferError>;
}

/// Dependencies retaining the existing generic backend beside one transfer accelerator.
pub struct PioSwdTransactionDeps<S, A> {
    pub(crate) generic: PioSwdDeps<S>,
    pub(crate) accelerator: A,
}

/// Active fixed-turnaround SWD role; register transfers do not fall back for other configurations.
pub struct PioSwdTransactionDrv<S, A> {
    generic: PioSwdDrv<S>,
    accelerator: A,
    accelerator_active: bool,
}

impl<S, A> PioSwdTransactionDeps<S, A> {
    /// Compose a generic backend and an inactive transfer accelerator.
    pub const fn new(generic: PioSwdDeps<S>, accelerator: A) -> Self {
        Self {
            generic,
            accelerator,
        }
    }

    pub(crate) fn into_parts(self) -> (PioSwdDeps<S>, A) {
        (self.generic, self.accelerator)
    }
}

impl<S, A> PioSwdTransactionDrv<S, A>
where
    S: PioShifter,
    A: PioSwdTransferAccelerator,
{
    fn ensure_accelerator(&mut self) {
        if self.accelerator_active {
            return;
        }
        self.generic.suspend_for_accelerator();
        self.accelerator.enter();
        self.accelerator_active = true;
    }

    fn ensure_generic(&mut self) {
        if !self.accelerator_active {
            return;
        }
        self.accelerator.exit();
        self.generic.resume_from_accelerator();
        self.accelerator_active = false;
    }

    /// One read and its idle cycles; out of line so the retry loop inlines into dap-rs's.
    #[inline(never)]
    fn read_once(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.ensure_accelerator();
        let r = self
            .accelerator
            .read_transfer(apndp == APnDP::AP, a as u8)
            .map_err(map_err);
        if matches!(r, Ok(_) | Err(swd::Error::BadParity)) {
            self.idle();
        }
        r
    }

    /// The host's SWD idle cycles after an OK ACK.
    #[inline(always)]
    fn idle(&mut self) {
        let cycles = self.generic.jtag_dp().swd_idle();
        if cycles != 0 {
            self.idle_cells(cycles);
        }
    }

    /// Idle cells with SWDIO low, on the generic engine: the transfer program clocks none.
    #[inline(never)]
    fn idle_cells(&mut self, cycles: u8) {
        self.ensure_generic();
        let _ = self.generic.write_sequence(cycles.into(), &IDLE_LOW);
    }
}

impl<S, A> From<PioSwdTransactionDeps<S, A>> for PioSwdTransactionDrv<S, A> {
    fn from(deps: PioSwdTransactionDeps<S, A>) -> Self {
        Self {
            generic: deps.generic.into_swd(),
            accelerator: deps.accelerator,
            accelerator_active: false,
        }
    }
}

impl<S, A> From<PioSwdTransactionDrv<S, A>> for PioSwdTransactionDeps<S, A>
where
    S: PioShifter,
    A: PioSwdTransferAccelerator,
{
    fn from(mut swd: PioSwdTransactionDrv<S, A>) -> Self {
        swd.ensure_generic();
        Self {
            generic: swd.generic.into_deps(),
            accelerator: swd.accelerator,
        }
    }
}

impl<S, A> Dependencies<PioSwdTransactionDrv<S, A>, PioSwdTransactionDeps<S, A>>
    for PioSwdTransactionDeps<S, A>
where
    S: PioShifter + SwjPinsSio,
    A: PioSwdTransferAccelerator,
{
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.generic.swj_pins(output, mask, wait_us)
    }

    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.generic.swj_sequence(data, nbits);
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        let generic = self.generic.set_swd_clock(max_frequency);
        let accelerator = self.accelerator.set_clock(max_frequency);
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

impl<S, A> Swd<PioSwdTransactionDeps<S, A>> for PioSwdTransactionDrv<S, A>
where
    S: PioShifter + SwjPinsSio,
    A: PioSwdTransferAccelerator,
{
    fn available(_deps: &PioSwdTransactionDeps<S, A>) -> bool {
        true
    }

    fn config(&mut self) -> &mut swd::Config {
        self.generic.config()
    }

    fn read(&mut self, wait_retries: usize, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        wait_retry(wait_retries, || self.read_inner(apndp, a))
    }

    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.read_once(apndp, a)
    }

    fn write(
        &mut self,
        wait_retries: usize,
        apndp: APnDP,
        a: DPRegister,
        data: u32,
    ) -> swd::Result<()> {
        wait_retry(wait_retries, || self.write_inner(apndp, a, data))
    }

    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        self.ensure_accelerator();
        let r = self
            .accelerator
            .write_transfer(apndp == APnDP::AP, a as u8, data)
            .map_err(map_err);
        if r.is_ok() {
            self.idle();
        }
        r
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        let generic = self.generic.set_clock(max_frequency);
        let accelerator = self.accelerator.set_clock(max_frequency);
        generic && accelerator
    }

    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.ensure_generic();
        self.generic.write_sequence(num_bits, data)
    }

    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.ensure_generic();
        self.generic.read_sequence(num_bits, data)
    }
}

impl<S, A> Jtag<PioSwdTransactionDeps<S, A>> for PioSwdTransactionDeps<S, A>
where
    S: PioShifter + SwjPinsSio,
    A: PioSwdTransferAccelerator,
{
    fn available(_deps: &PioSwdTransactionDeps<S, A>) -> bool {
        false
    }

    fn config(&mut self) -> &mut jtag::Config {
        self.generic.jtag_config()
    }

    fn sequence(&mut self, _info: SequenceInfo, _tdi: &[u8], rxbuf: &mut [u8]) {
        rxbuf.fill(0);
    }

    fn tms_sequence(&mut self, _tms: &[bool]) {}

    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        false
    }
}

/// ACK-continuation SWD for a board without JTAG; SWD writes may complete after return, hence the fence.
pub type PioSwdTransactionOnlyDap<'a, S, A, WAIT, SWO, V, LEDS, F> = daplink_rs::DapProbe<
    'a,
    PioSwdTransactionDeps<S, A>,
    LEDS,
    WAIT,
    PioSwdTransactionDeps<S, A>,
    PioSwdTransactionDrv<S, A>,
    SWO,
    V,
    F,
>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::program::DIRECT;
    use daplink_rs::ScanChainStorage;
    use std::sync::{Arc, Mutex};
    use std::vec::Vec;

    #[derive(Clone, Debug, PartialEq, Eq)]
    enum Event {
        GenericPut,
        GenericClock,
        GenericEnable(bool),
        GenericRecover,
        AcceleratorClock(u32),
        AcceleratorEnter,
        AcceleratorExit,
        AcceleratorRead,
        AcceleratorWrite,
    }

    #[derive(Clone)]
    struct MockGeneric(Arc<Mutex<Vec<Event>>>);

    impl PioShifter for MockGeneric {
        fn put(&mut self, _word: u32) {
            self.0.lock().unwrap().push(Event::GenericPut);
        }
        fn get(&mut self) -> u32 {
            0
        }
        fn set_clock_divider(&mut self, _divider: u16) {
            self.0.lock().unwrap().push(Event::GenericClock);
        }
        fn set_enabled(&mut self, enabled: bool) {
            self.0.lock().unwrap().push(Event::GenericEnable(enabled));
        }
        fn clear_fifos(&mut self) {
            self.0.lock().unwrap().push(Event::GenericRecover);
        }
        fn restart(&mut self) {}
        fn exec_jmp(&mut self, _addr: u8) {}
        fn force_oen_released(&mut self) {}
        fn force_swclk_low(&mut self) {}
        fn force_pindirs_idle(&mut self) {}
        fn pads_to_sio(&mut self) {}
        fn pads_to_pio(&mut self) {}
    }

    impl SwjPinsSio for MockGeneric {
        fn swj_pins(&mut self, _output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
            Pins::empty()
        }
        fn release_nreset(&mut self) {}
    }

    /// The accelerator, answering WAIT to the first `.1` reads.
    struct MockAccelerator(Arc<Mutex<Vec<Event>>>, usize);

    impl PioSwdTransferAccelerator for MockAccelerator {
        fn set_clock(&mut self, max_frequency: u32) -> bool {
            self.0
                .lock()
                .unwrap()
                .push(Event::AcceleratorClock(max_frequency));
            max_frequency != 123_456
        }
        fn enter(&mut self) {
            self.0.lock().unwrap().push(Event::AcceleratorEnter);
        }
        fn exit(&mut self) {
            self.0.lock().unwrap().push(Event::AcceleratorExit);
        }
        fn read_transfer(&mut self, _ap: bool, _addr: u8) -> Result<u32, XferError> {
            self.0.lock().unwrap().push(Event::AcceleratorRead);
            if self.1 == BAD_PARITY {
                return Err(XferError::BadParity);
            }
            if self.1 > 0 {
                self.1 -= 1;
                return Err(XferError::AckWait);
            }
            Ok(0x1234_5678)
        }
        fn write_transfer(&mut self, _ap: bool, _addr: u8, _data: u32) -> Result<(), XferError> {
            self.0.lock().unwrap().push(Event::AcceleratorWrite);
            Ok(())
        }
    }

    static SCAN_OWNERSHIP: ScanChainStorage = ScanChainStorage::new();
    static SCAN_CONVERSION: ScanChainStorage = ScanChainStorage::new();
    static SCAN_CLOCK: ScanChainStorage = ScanChainStorage::new();
    static SCAN_RETRY: ScanChainStorage = ScanChainStorage::new();
    static SCAN_IDLE: ScanChainStorage = ScanChainStorage::new();
    static SCAN_PARITY: ScanChainStorage = ScanChainStorage::new();
    const BAD_PARITY: usize = usize::MAX;

    fn deps(
        events: &Arc<Mutex<Vec<Event>>>,
        scan: &'static ScanChainStorage,
    ) -> PioSwdTransactionDeps<MockGeneric, MockAccelerator> {
        waiting_deps(events, scan, 0)
    }

    fn waiting_deps(
        events: &Arc<Mutex<Vec<Event>>>,
        scan: &'static ScanChainStorage,
        waits: usize,
    ) -> PioSwdTransactionDeps<MockGeneric, MockAccelerator> {
        let generic = PioSwdDeps::new(
            MockGeneric(events.clone()),
            DIRECT.entries(0),
            200_000_000,
            scan,
        );
        PioSwdTransactionDeps::new(generic, MockAccelerator(events.clone(), waits))
    }

    fn reads(events: &Arc<Mutex<Vec<Event>>>) -> usize {
        let events = events.lock().unwrap();
        events
            .iter()
            .filter(|e| **e == Event::AcceleratorRead)
            .count()
    }

    #[test]
    fn n_wait_retries_make_n_plus_one_attempts() {
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut swd: PioSwdTransactionDrv<_, _> = waiting_deps(&events, &SCAN_RETRY, 3).into();
        let mut read = |retries| {
            Swd::<PioSwdTransactionDeps<_, _>>::read(
                &mut swd,
                retries,
                APnDP::AP,
                DPRegister::RDBUFF,
            )
        };
        assert_eq!(read(0), Err(swd::Error::AckWait));
        assert_eq!(reads(&events), 1);
        assert_eq!(read(2), Ok(0x1234_5678));
        assert_eq!(reads(&events), 4);
    }

    #[test]
    fn idle_cycles_follow_an_ok_ack_on_the_generic_engine() {
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut swd: PioSwdTransactionDrv<_, _> = waiting_deps(&events, &SCAN_IDLE, 1).into();
        let read = |swd: &mut PioSwdTransactionDrv<_, _>| {
            Swd::<PioSwdTransactionDeps<_, _>>::read_inner(swd, APnDP::DP, DPRegister::DPIDR)
        };
        SCAN_IDLE.jtag_dp().observe(&[4, 3, 0, 0, 0, 0]);
        assert_eq!(read(&mut swd), Err(swd::Error::AckWait));
        assert!(!events.lock().unwrap().contains(&Event::AcceleratorExit));
        assert_eq!(read(&mut swd), Ok(0x1234_5678));
        let events = events.lock().unwrap();
        let exit = events.iter().position(|e| *e == Event::AcceleratorExit);
        let last_read = events.iter().rposition(|e| *e == Event::AcceleratorRead);
        assert!(exit > last_read);
        assert_eq!(events.last(), Some(&Event::GenericPut));
    }

    #[test]
    fn transfers_take_pads_once_and_sequences_restore_generic_ownership() {
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut swd: PioSwdTransactionDrv<_, _> = deps(&events, &SCAN_OWNERSHIP).into();

        assert_eq!(
            Swd::<PioSwdTransactionDeps<_, _>>::read_inner(&mut swd, APnDP::DP, DPRegister::DPIDR,),
            Ok(0x1234_5678)
        );
        assert_eq!(
            Swd::<PioSwdTransactionDeps<_, _>>::write_inner(
                &mut swd,
                APnDP::AP,
                DPRegister::CTRLSTAT,
                1,
            ),
            Ok(())
        );
        Swd::<PioSwdTransactionDeps<_, _>>::write_sequence(&mut swd, 8, &[0xff]).unwrap();

        let events = events.lock().unwrap();
        assert_eq!(
            events
                .iter()
                .filter(|event| matches!(event, Event::AcceleratorEnter))
                .count(),
            1
        );
        let enter = events
            .iter()
            .position(|event| *event == Event::AcceleratorEnter)
            .unwrap();
        let disable = events
            .iter()
            .position(|event| *event == Event::GenericEnable(false))
            .unwrap();
        let exit = events
            .iter()
            .position(|event| *event == Event::AcceleratorExit)
            .unwrap();
        let recover = events
            .iter()
            .position(|event| *event == Event::GenericRecover)
            .unwrap();
        assert!(disable < enter);
        assert!(exit < recover);
        assert!(events[enter + 1..exit].contains(&Event::AcceleratorRead));
        assert!(events[enter + 1..exit].contains(&Event::AcceleratorWrite));
    }

    #[test]
    fn role_conversion_returns_active_accelerator_pads_before_moving_state() {
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut swd: PioSwdTransactionDrv<_, _> = deps(&events, &SCAN_CONVERSION).into();
        let _ =
            Swd::<PioSwdTransactionDeps<_, _>>::read_inner(&mut swd, APnDP::DP, DPRegister::DPIDR);
        let _deps = PioSwdTransactionDeps::from(swd);

        let events = events.lock().unwrap();
        let exit = events
            .iter()
            .rposition(|event| *event == Event::AcceleratorExit)
            .unwrap();
        let recover = events
            .iter()
            .rposition(|event| *event == Event::GenericRecover)
            .unwrap();
        assert!(exit < recover);
    }

    #[test]
    fn clamped_clock_requests_update_both_engines_and_report_an_error() {
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut swd: PioSwdTransactionDrv<_, _> = deps(&events, &SCAN_CLOCK).into();

        assert!(!Swd::<PioSwdTransactionDeps<_, _>>::set_clock(&mut swd, 0));
        assert!(!Swd::<PioSwdTransactionDeps<_, _>>::set_clock(
            &mut swd, 123_456
        ));
        assert!(Swd::<PioSwdTransactionDeps<_, _>>::set_clock(
            &mut swd, 1_000_000
        ));
        assert_eq!(
            *events.lock().unwrap(),
            [
                Event::GenericClock,
                Event::AcceleratorClock(0),
                Event::GenericClock,
                Event::AcceleratorClock(123_456),
                Event::GenericClock,
                Event::AcceleratorClock(1_000_000),
            ]
        );
    }

    #[test]
    fn idle_cycles_follow_a_bad_parity_too() {
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut swd: PioSwdTransactionDrv<_, _> =
            waiting_deps(&events, &SCAN_PARITY, BAD_PARITY).into();
        SCAN_PARITY.jtag_dp().observe(&[4, 3, 0, 0, 0, 0]);
        assert_eq!(
            Swd::<PioSwdTransactionDeps<_, _>>::read_inner(&mut swd, APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::BadParity)
        );
        assert!(events.lock().unwrap().contains(&Event::AcceleratorExit));
    }
}
