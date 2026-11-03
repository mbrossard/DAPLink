//! The accelerated compositions instantiate generic PIO0 beside the PIO1 transfer engines.

use daplink_rp_pio::{
    PinMap, PioClockDivider, PioJtagAccelerator, PioJtagShifter, PioShifter, PioSwdDeps,
    PioSwdPins, PioSwdTransactionDeps, PioSwdTransactionOnlyDap, PioSwdTransferAccelerator,
    PioTransferDap, PioTransferDeps, SwjPinsSio, XferError,
};
use daplink_rs::dap_rs::dap::DelayNs;
use daplink_rs::dap_rs::swj::Pins;
use daplink_rs::daplink_core::{DapQueue, PacketSize};
use daplink_rs::{DelayCycles, NoCommandFence, NoSwo, NoVendor, NoopLeds, ScanChainStorage};
use std::sync::{Arc, Mutex};

struct MockShifter;

impl PioShifter for MockShifter {
    fn put(&mut self, _word: u32) {}
    fn get(&mut self) -> u32 {
        0b001 << 30
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
    fn set_jtag_clock_divider(&mut self, _divider: PioClockDivider) {}
    fn enter_jtag(&mut self, _dispatcher: u8) {}
    fn exit_jtag(&mut self) {}
}

#[derive(Clone)]
struct MockDelay;

impl DelayNs for MockDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}
impl DelayCycles for MockDelay {
    fn cpu_clock(&self) -> u32 {
        125_000_000
    }
    fn delay_cycles(&mut self, _cycles: u32) {}
}

struct MockAccelerator;

impl PioJtagAccelerator for MockAccelerator {
    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        true
    }
    fn enter(&mut self, _bypass_before: u8, _bypass_after: u8) {}
    fn exit(&mut self) {}
    fn transfer(&mut self, _request: u8, _data: u32) -> (u32, u32) {
        (0b010, 0)
    }
}

impl PioSwdTransferAccelerator for MockAccelerator {
    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        true
    }
    fn enter(&mut self) {}
    fn exit(&mut self) {}
    fn read_transfer(&mut self, _ap: bool, _addr: u8) -> Result<u32, XferError> {
        Ok(0)
    }
    fn write_transfer(&mut self, _ap: bool, _addr: u8, _data: u32) -> Result<(), XferError> {
        Ok(())
    }
}

/// The shared-transfer engine: both optimized protocols own the same accelerator value.
type TransferDap<'a> = PioTransferDap<
    'a,
    MockShifter,
    MockAccelerator,
    MockDelay,
    NoSwo,
    NoVendor,
    NoopLeds,
    NoCommandFence,
>;

/// The engine a split-SWDIO, JTAG-less board runs: generic PIO0 sequences, PIO1 SWD, no JTAG.
type SplitDap<'a> = PioSwdTransactionOnlyDap<
    'a,
    MockShifter,
    MockAccelerator,
    MockDelay,
    NoSwo,
    NoVendor,
    NoopLeds,
    NoCommandFence,
>;

static TRANSFER_SCAN: ScanChainStorage = ScanChainStorage::new();
/// A second storage: each is claimed once, and the two engines below are built independently.
static SPLIT_SCAN: ScanChainStorage = ScanChainStorage::new();

#[derive(Clone, Debug, PartialEq, Eq)]
enum Event {
    Enable(bool),
    OenReleased,
    SwclkLow,
    ClearFifos,
    Restart,
    Jmp(u8),
    PindirsIdle,
    PadsToSio,
    PadsToPio,
    JtagDivider(PioClockDivider),
    EnterJtag(u8),
    ExitJtag,
    NresetReleased(usize),
}

#[derive(Default)]
struct Trace {
    pending: Vec<u32>,
    completed: Vec<u32>,
    events: Vec<Event>,
}

#[derive(Clone)]
struct TraceShifter(Arc<Mutex<Trace>>);

impl PioShifter for TraceShifter {
    fn put(&mut self, word: u32) {
        self.0.lock().unwrap().pending.push(word);
    }
    fn get(&mut self) -> u32 {
        let mut trace = self.0.lock().unwrap();
        let pending = core::mem::take(&mut trace.pending);
        trace.completed.extend(pending);
        0
    }
    fn set_clock_divider(&mut self, _divider: u16) {}
    fn set_enabled(&mut self, enabled: bool) {
        self.0.lock().unwrap().events.push(Event::Enable(enabled));
    }
    fn clear_fifos(&mut self) {
        let mut trace = self.0.lock().unwrap();
        trace.pending.clear();
        trace.events.push(Event::ClearFifos);
    }
    fn restart(&mut self) {
        self.0.lock().unwrap().events.push(Event::Restart);
    }
    fn exec_jmp(&mut self, addr: u8) {
        self.0.lock().unwrap().events.push(Event::Jmp(addr));
    }
    fn force_oen_released(&mut self) {
        self.0.lock().unwrap().events.push(Event::OenReleased);
    }
    fn force_swclk_low(&mut self) {
        self.0.lock().unwrap().events.push(Event::SwclkLow);
    }
    fn force_pindirs_idle(&mut self) {
        self.0.lock().unwrap().events.push(Event::PindirsIdle);
    }
    fn pads_to_sio(&mut self) {
        self.0.lock().unwrap().events.push(Event::PadsToSio);
    }
    fn pads_to_pio(&mut self) {
        self.0.lock().unwrap().events.push(Event::PadsToPio);
    }
}

impl SwjPinsSio for TraceShifter {
    fn swj_pins(&mut self, _output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
        Pins::empty()
    }
    fn release_nreset(&mut self) {
        let mut trace = self.0.lock().unwrap();
        let completed = trace.completed.len();
        trace.events.push(Event::NresetReleased(completed));
    }
}

impl PioJtagShifter for TraceShifter {
    fn jtag_put(&mut self, word: u32) {
        self.0.lock().unwrap().pending.push(word);
    }
    fn jtag_get(&mut self) -> u32 {
        let mut trace = self.0.lock().unwrap();
        let pending = core::mem::take(&mut trace.pending);
        trace.completed.extend(pending);
        0
    }
    fn set_jtag_clock_divider(&mut self, divider: PioClockDivider) {
        self.0
            .lock()
            .unwrap()
            .events
            .push(Event::JtagDivider(divider));
    }
    fn enter_jtag(&mut self, dispatcher: u8) {
        self.0
            .lock()
            .unwrap()
            .events
            .push(Event::EnterJtag(dispatcher));
    }
    fn exit_jtag(&mut self) {
        self.0.lock().unwrap().events.push(Event::ExitJtag);
    }
}

type TraceSwdDap<'a> = PioSwdTransactionOnlyDap<
    'a,
    TraceShifter,
    MockAccelerator,
    MockDelay,
    NoSwo,
    NoVendor,
    NoopLeds,
    NoCommandFence,
>;
type TraceJtagDap<'a> = PioTransferDap<
    'a,
    TraceShifter,
    MockAccelerator,
    MockDelay,
    NoSwo,
    NoVendor,
    NoopLeds,
    NoCommandFence,
>;

static CONSTRUCTOR_SCAN: ScanChainStorage = ScanChainStorage::new();
static SWD_CALLBACK_SCAN: ScanChainStorage = ScanChainStorage::new();
static JTAG_CALLBACK_SCAN: ScanChainStorage = ScanChainStorage::new();
const PACKET: PacketSize = DapQueue::<1, 64>::packet_size::<64>();

const DEFAULT_JTAG_DIVIDER: PioClockDivider =
    match daplink_rp_pio::clock::jtag_clock_divider(125_000_000, 1_000_000) {
        Some(divider) => divider,
        None => PioClockDivider::integer(1),
    };

#[test]
fn the_shared_transfer_engine_composes_both_protocol_roles() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let generic = PioSwdDeps::new_jtag(
        MockShifter,
        entries,
        daplink_rp_pio::select_jtag_program(125_000_000)
            .unwrap()
            .entries,
        125_000_000,
        &TRANSFER_SCAN,
    );
    let deps = PioTransferDeps::new(generic, MockAccelerator);
    let mut dap: TransferDap =
        TransferDap::new(deps, daplink_rs::NoopLeds, MockDelay, NoSwo, "test", PACKET);

    let mut response = [0u8; 64];
    assert_eq!(dap.process_command(&[0x02, 1], &mut response), 2);
    assert_eq!(&response[..2], &[0x02, 1]);
    assert_eq!(dap.process_command(&[0x02, 2], &mut response), 2);
    assert_eq!(&response[..2], &[0x02, 2]);
}

#[test]
fn the_split_pin_engine_composes_and_answers_a_command() {
    let pins = PioSwdPins {
        swclk: 12,
        swdio_out: 14,
        swdio_in: Some(13),
        oen: None,
        nreset: None,
    };
    let prog = daplink_rp_pio::select_program(pins.oen.is_some(), 125_000_000).unwrap();
    let map = PinMap::new(&pins, prog.side_set_bits);
    assert_eq!((map.out_base, map.in_base), (14, 13));

    let generic = PioSwdDeps::new(MockShifter, prog.entries(0), 125_000_000, &SPLIT_SCAN);
    let deps = PioSwdTransactionDeps::new(generic, MockAccelerator);
    let mut dap: SplitDap =
        SplitDap::new(deps, daplink_rs::NoopLeds, MockDelay, NoSwo, "test", PACKET);

    let mut response = [0u8; 64];
    let len = dap.process_command(&[0x00, 0x01], &mut response);
    assert!(len > 0);
    assert_eq!(response[0], 0x00);
}

#[test]
fn constructor_runs_the_ordered_release_callback() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let trace = Arc::new(Mutex::new(Trace::default()));
    let generic = PioSwdDeps::new(
        TraceShifter(trace.clone()),
        entries,
        125_000_000,
        &CONSTRUCTOR_SCAN,
    );
    let deps = PioSwdTransactionDeps::new(generic, MockAccelerator);
    let _dap: TraceSwdDap =
        TraceSwdDap::new(deps, daplink_rs::NoopLeds, MockDelay, NoSwo, "test", PACKET);

    let trace = trace.lock().unwrap();
    assert!(trace.pending.is_empty());
    assert_eq!(
        trace.completed,
        vec![daplink_rp_pio::cmd_word(entries.release_cmd, false, 1)]
    );
    assert_eq!(trace.events, vec![Event::NresetReleased(1)]);
}

#[test]
fn swd_disconnect_drains_a_queued_sequence_before_releasing_nreset() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let trace = Arc::new(Mutex::new(Trace::default()));
    let generic = PioSwdDeps::new(
        TraceShifter(trace.clone()),
        entries,
        125_000_000,
        &SWD_CALLBACK_SCAN,
    );
    let deps = PioSwdTransactionDeps::new(generic, MockAccelerator);
    let mut dap: TraceSwdDap =
        TraceSwdDap::new(deps, daplink_rs::NoopLeds, MockDelay, NoSwo, "test", PACKET);
    {
        let mut trace = trace.lock().unwrap();
        trace.completed.clear();
        trace.events.clear();
    }

    let mut response = [0u8; 64];
    assert_eq!(dap.process_command(&[0x1D, 1, 8, 0xA5], &mut response), 2);
    assert_eq!(&response[..2], &[0x1D, 0]);
    let queued = trace.lock().unwrap().pending.clone();
    assert!(!queued.is_empty());

    assert_eq!(dap.process_command(&[0x03], &mut response), 2);
    assert_eq!(&response[..2], &[0x03, 0]);
    let trace = trace.lock().unwrap();
    assert!(trace.pending.is_empty());
    assert_eq!(&trace.completed[..queued.len()], queued.as_slice());
    assert_eq!(
        trace.completed.last(),
        Some(&daplink_rp_pio::cmd_word(entries.release_cmd, false, 1))
    );
    // nRESET is released only once every queued word, the release included, has completed.
    assert_eq!(
        trace.events,
        vec![Event::NresetReleased(trace.completed.len())]
    );
}

#[test]
fn jtag_disconnect_reclaims_with_forced_recovery_then_releases_cleanly() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let trace = Arc::new(Mutex::new(Trace::default()));
    let deps = PioSwdDeps::new_jtag(
        TraceShifter(trace.clone()),
        entries,
        daplink_rp_pio::select_jtag_program(125_000_000)
            .unwrap()
            .entries,
        125_000_000,
        &JTAG_CALLBACK_SCAN,
    );
    let mut dap: TraceJtagDap = TraceJtagDap::new(
        PioTransferDeps::new(deps, MockAccelerator),
        daplink_rs::NoopLeds,
        MockDelay,
        NoSwo,
        "test",
        PACKET,
    );
    {
        let mut trace = trace.lock().unwrap();
        trace.completed.clear();
        trace.events.clear();
    }

    let mut response = [0u8; 64];
    assert_eq!(dap.process_command(&[0x02, 2], &mut response), 2);
    assert_eq!(&response[..2], &[0x02, 2]);
    assert_eq!(
        trace.lock().unwrap().events,
        vec![
            Event::JtagDivider(DEFAULT_JTAG_DIVIDER),
            Event::EnterJtag(
                daplink_rp_pio::select_jtag_program(125_000_000)
                    .unwrap()
                    .entries
                    .get_next_cmd
            ),
        ]
    );

    assert_eq!(dap.process_command(&[0x03], &mut response), 2);
    assert_eq!(&response[..2], &[0x03, 0]);
    let trace = trace.lock().unwrap();
    assert_eq!(
        trace.events,
        vec![
            Event::JtagDivider(DEFAULT_JTAG_DIVIDER),
            Event::EnterJtag(
                daplink_rp_pio::select_jtag_program(125_000_000)
                    .unwrap()
                    .entries
                    .get_next_cmd
            ),
            Event::ExitJtag,
            Event::Enable(false),
            Event::ClearFifos,
            Event::Restart,
            Event::OenReleased,
            Event::SwclkLow,
            Event::Jmp(entries.get_next_cmd),
            Event::PindirsIdle,
            Event::Enable(true),
            Event::NresetReleased(2),
        ]
    );
    assert!(trace.pending.is_empty());
    assert_eq!(
        trace.completed,
        vec![
            daplink_rp_pio::cmd_word(entries.release_cmd, false, 1),
            daplink_rp_pio::cmd_word(entries.release_cmd, false, 1),
        ]
    );
}
