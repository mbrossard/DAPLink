//! The accelerated composition instantiates: PIO SWD as the SWD driver, bit-bang JTAG over the
//! pad loan, and the engine facade over both.

use daplink_rp_pio::{
    PinMap, PioShifter, PioSwdDeps, PioSwdJtagDap, PioSwdOnlyDap, PioSwdPins, SwjPinsSio,
};
use daplink_rs::dap_rs::dap::DelayNs;
use daplink_rs::dap_rs::swj::Pins;
use daplink_rs::{
    ComposedJtagDeps, DelayCycles, InputOutputPin, JtagLines, NoSwo, ScanChainStorage,
};
use std::sync::{Arc, Mutex};

/// A shifter that answers every read with an OK acknowledge frame, so no transfer wedges.
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
}

#[derive(Clone)]
struct MockPin;

impl InputOutputPin for MockPin {
    fn set_as_output(&mut self) {}
    fn set_high(&mut self, _high: bool) {}
    fn set_as_input(&mut self) {}
    fn is_high(&mut self) -> bool {
        false
    }
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

/// The engine the accel firmware will run: the alias a consumer names.
type AccelDap<'a> = PioSwdJtagDap<'a, MockShifter, MockPin, MockDelay, NoSwo>;

/// The engine a split-SWDIO, JTAG-less board runs: the same PIO SWD driver, no JTAG composed on.
type SplitDap<'a> = PioSwdOnlyDap<'a, MockShifter, MockDelay, NoSwo>;

static SCAN: ScanChainStorage = ScanChainStorage::new();
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
}

type TraceSwdDap<'a> = PioSwdOnlyDap<'a, TraceShifter, MockDelay, NoSwo>;
type TraceJtagDap<'a> = PioSwdJtagDap<'a, TraceShifter, MockPin, MockDelay, NoSwo>;

static CONSTRUCTOR_SCAN: ScanChainStorage = ScanChainStorage::new();
static SWD_CALLBACK_SCAN: ScanChainStorage = ScanChainStorage::new();
static JTAG_CALLBACK_SCAN: ScanChainStorage = ScanChainStorage::new();

#[test]
fn the_accel_engine_composes_and_answers_a_command() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let deps = PioSwdDeps::new(MockShifter, entries, 125_000_000, &SCAN);
    let lines = JtagLines::new(MockPin, MockPin, MockPin, MockPin);
    let mut dap: AccelDap = AccelDap::new(
        ComposedJtagDeps::new(deps, lines, MockDelay),
        daplink_rs::NoopLeds,
        MockDelay,
        NoSwo,
        "test",
        64,
    );

    // DAP_Info / vendor id (command 0x00, id 0x01) answers, so the engine is wired end to end.
    let mut response = [0u8; 64];
    let len = dap.process_command(&[0x00, 0x01], &mut response);
    assert!(len > 0);
    assert_eq!(response[0], 0x00);
}

#[test]
fn the_split_pin_engine_composes_and_answers_a_command() {
    // The Debug Probe wiring: SWCLK GP12, SWDIO driven on GP14 and read back on GP13.
    let pins = PioSwdPins {
        swclk: 12,
        swdio_out: 14,
        swdio_in: Some(13),
        oen: None,
        nreset: None,
    };
    let prog = daplink_rp_pio::select_program(pins.oen.is_some());
    let map = PinMap::new(&pins, prog.side_set_bits);
    assert_eq!((map.out_base, map.in_base), (14, 13));

    let deps = PioSwdDeps::new(MockShifter, prog.entries(0), 125_000_000, &SPLIT_SCAN);
    let mut dap: SplitDap = SplitDap::new(deps, daplink_rs::NoopLeds, MockDelay, NoSwo, "test", 64);

    // DAP_Info / vendor id (command 0x00, id 0x01) answers, so the engine is wired end to end.
    let mut response = [0u8; 64];
    let len = dap.process_command(&[0x00, 0x01], &mut response);
    assert!(len > 0);
    assert_eq!(response[0], 0x00);
}

#[test]
fn constructor_runs_the_ordered_release_callback() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let trace = Arc::new(Mutex::new(Trace::default()));
    let deps = PioSwdDeps::new(
        TraceShifter(trace.clone()),
        entries,
        125_000_000,
        &CONSTRUCTOR_SCAN,
    );
    let _dap: TraceSwdDap =
        TraceSwdDap::new(deps, daplink_rs::NoopLeds, MockDelay, NoSwo, "test", 64);

    let trace = trace.lock().unwrap();
    assert!(trace.pending.is_empty());
    assert_eq!(
        trace.completed,
        vec![daplink_rp_pio::cmd_word(entries.release_cmd, false, 1)]
    );
    assert!(trace.events.is_empty());
}

#[test]
fn swd_disconnect_drains_a_queued_sequence_before_release() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let trace = Arc::new(Mutex::new(Trace::default()));
    let deps = PioSwdDeps::new(
        TraceShifter(trace.clone()),
        entries,
        125_000_000,
        &SWD_CALLBACK_SCAN,
    );
    let mut dap: TraceSwdDap =
        TraceSwdDap::new(deps, daplink_rs::NoopLeds, MockDelay, NoSwo, "test", 64);
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
    assert!(!trace.events.contains(&Event::ClearFifos));
}

#[test]
fn jtag_disconnect_reclaims_with_forced_recovery_then_releases_cleanly() {
    let entries = daplink_rp_pio::program::DIRECT.entries(0);
    let trace = Arc::new(Mutex::new(Trace::default()));
    let deps = PioSwdDeps::new(
        TraceShifter(trace.clone()),
        entries,
        125_000_000,
        &JTAG_CALLBACK_SCAN,
    );
    let lines = JtagLines::new(MockPin, MockPin, MockPin, MockPin);
    let mut dap: TraceJtagDap = TraceJtagDap::new(
        ComposedJtagDeps::new(deps, lines, MockDelay),
        daplink_rs::NoopLeds,
        MockDelay,
        NoSwo,
        "test",
        64,
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
        vec![Event::Enable(false), Event::PadsToSio]
    );

    assert_eq!(dap.process_command(&[0x03], &mut response), 2);
    assert_eq!(&response[..2], &[0x03, 0]);
    let trace = trace.lock().unwrap();
    assert_eq!(
        trace.events,
        vec![
            Event::Enable(false),
            Event::PadsToSio,
            Event::PadsToPio,
            Event::Enable(false),
            Event::ClearFifos,
            Event::Restart,
            Event::OenReleased,
            Event::SwclkLow,
            Event::Jmp(entries.get_next_cmd),
            Event::PindirsIdle,
            Event::Enable(true),
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
