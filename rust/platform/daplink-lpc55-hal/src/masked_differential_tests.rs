//! The masked block's framing, on simulated words, ahead of dap-rs against dap-rs alone.

use super::*;
extern crate std;
use std::boxed::Box;
use std::vec;

use dap_differential::{
    check, jtag_transfer, steps, verdict, waive, Check, Cmd, Item, Latch, Latched, Line, Policy,
    Probe, Profile, Report, RuleId, Shared, SimDelay, SimPin, SimWords, Stream, Target, Waiver,
};
use daplink_rs::dap_rs::dap::Dap;
use daplink_rs::dap_rs::driver::bitbang::{BitbangAdapter, JtagAdapter, SwdAdapter};
use daplink_rs::NoSwo;
use std::rc::Rc;

use super::protocol_tests::{serial, LIVE, P};

type Inner = BitbangAdapter<SimPin, SimDelay>;

/// The reference's bit-bang dependencies, publishing SWD mode the way the masked engine's do.
struct Deps(Inner, Rc<Latch>);
struct SwdDrv(SwdAdapter<SimPin, SimDelay>, Rc<Latch>);
struct JtagDrv(JtagAdapter<SimPin, SimDelay>, Rc<Latch>);

impl From<Deps> for SwdDrv {
    fn from(d: Deps) -> Self {
        SWD_ACTIVE.store(true, Ordering::Relaxed);
        SwdDrv(d.0.into(), d.1)
    }
}

impl From<SwdDrv> for Deps {
    fn from(s: SwdDrv) -> Self {
        SWD_ACTIVE.store(false, Ordering::Relaxed);
        Deps(s.0.into(), s.1)
    }
}

impl From<Deps> for JtagDrv {
    fn from(d: Deps) -> Self {
        JtagDrv(d.0.into(), d.1)
    }
}

impl From<JtagDrv> for Deps {
    fn from(j: JtagDrv) -> Self {
        Deps(j.0.into(), j.1)
    }
}

impl Dependencies<SwdDrv, JtagDrv> for Deps {
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.0.process_swj_pins(output, mask, wait_us)
    }
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.0.process_swj_sequence(data, nbits)
    }
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.0.process_swj_clock(max_frequency)
    }
    fn high_impedance_mode(&mut self) {
        self.0.high_impedance_mode()
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        self.0.swd_config()
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.0.jtag_config()
    }
}

/// The engine's own [`MaskedSwdDrv`] transfers: its WAIT count, on the block's simulated words.
impl Swd<Deps> for SwdDrv {
    fn available(_deps: &Deps) -> bool {
        true
    }
    fn config(&mut self) -> &mut swd::Config {
        self.0.config()
    }
    fn read(&mut self, wait_retries: usize, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        daplink_rs::wait_retry(wait_retries, || self.read_inner(apndp, a))
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        super::protocol_tests::read(request_byte(apndp, RnW::R, a))
    }
    fn write(
        &mut self,
        wait_retries: usize,
        apndp: APnDP,
        a: DPRegister,
        data: u32,
    ) -> swd::Result<()> {
        daplink_rs::wait_retry(wait_retries, || self.write_inner(apndp, a, data))
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        super::protocol_tests::write(request_byte(apndp, RnW::W, a), data)
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_clock(max_frequency)
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.0.write_sequence(num_bits, data)
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.0.read_sequence(num_bits, data)
    }
}

impl Jtag<Deps> for JtagDrv {
    fn available(_deps: &Deps) -> bool {
        true
    }
    fn config(&mut self) -> &mut jtag::Config {
        self.0.config()
    }
    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        self.0.sequence(info, tdi, rxbuf)
    }
    fn tms_sequence(&mut self, tms: &[bool]) {
        self.0.tms_sequence(tms)
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_clock(max_frequency)
    }
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let latch = self.1.clone();
        jtag_transfer(self, &latch, r_nw, a2a3, config.idle_cycles, data)
    }
}

/// The MCU-Link composition: the block first, then dap-rs over the SWD-mode publishing deps.
fn composed(sim: &Shared) -> Box<dyn Probe> {
    let pin = |line| SimPin::new(sim, line);
    let latch = Rc::new(Latch::default());
    let deps = Deps(
        BitbangAdapter::new(
            SimPin::open_drain(sim, Line::Nreset),
            pin(Line::Tdi),
            pin(Line::Swdio),
            pin(Line::Swclk),
            pin(Line::Tdo),
            SimDelay,
            dap_differential::scan_chain(8),
        ),
        latch.clone(),
    );
    SWD_ACTIVE.store(false, Ordering::Relaxed);
    let mut dap: Dap<'static, Deps, _, _, JtagDrv, SwdDrv, _> =
        Dap::new(deps, NoopLeds, SimDelay, NoSwo, "diff", 512);
    dap.suspend();
    LIVE.with(|l| *l.borrow_mut() = Some(SimWords::gpio(sim)));
    let mut block = MaskedBlock::new(P);
    let inner = move |report: &[u8], response: &mut [u8]| match block.process(report, response) {
        Some(len) => len,
        None => {
            waive(RuleId::CountCompleted);
            dap.process_command(report, response)
        }
    };
    Box::new(Latched { inner, latch })
}

/// What dap-rs answers where the firmware runs its own engine.
static POLICY: Policy = Policy {
    waivers: &[
        Waiver {
            rule: RuleId::ValueMatch,
            reason: "value-match reads go to dap-rs, which compares an AP match one read behind",
            on: Waiver::swd,
            marked: false,
        },
        Waiver {
            rule: RuleId::CountCompleted,
            reason: "the reports the block declines reach dap-rs, which counts the failed transfer",
            on: Waiver::swd,
            marked: true,
        },
    ],
};

/// How the policy judges `stream`, which must not diverge.
fn judged(stream: &Stream) -> Report {
    let c = Check::new(Profile::SWD, 0..0, &POLICY);
    verdict(&c, stream, composed).unwrap_or_else(|d| panic!("{d}"))
}

fn sweep(profile: Profile, seeds: core::ops::Range<u64>) {
    let _serial = serial();
    let result = check(&Check::new(profile, seeds, &POLICY), composed);
    LIVE.with(|l| *l.borrow_mut() = None);
    if let Err(d) = result {
        panic!("{d}");
    }
}

#[test]
fn swd_sessions_match_dap_rs() {
    sweep(Profile::SWD, 0..300);
}

#[test]
fn bad_read_parity_matches_dap_rs() {
    let profile = Profile {
        parity: true,
        ..Profile::SWD
    };
    sweep(profile, 300..450);
    sweep(Profile::SWD_CTRL_PARITY, 0..450);
}

/// The block follows the driver's own mode, so even arbitrary streams leave nothing to rewrite.
#[test]
fn chaos_matches_dap_rs() {
    sweep(Profile::CHAOS, 0..300);
}

/// The blocks count what completed, as Arm does; dap-rs counts the transfer that failed.
#[test]
fn known_divergence_the_count_leaves_out_a_failed_transfer() {
    let _serial = serial();
    let stream = dap_differential::session::sticky_fault();
    let (t, o) = steps(&stream, composed);
    for at in [7, 8] {
        let (o, t) = (&o[at].response[..], &t[at].response[..]);
        assert_eq!((o, t), (&[6, 0, 0, 4][..], &[6, 1, 0, 4][..]), "step {at}");
    }
    assert!(judged(&stream).fired.contains(&RuleId::CountCompleted));
    LIVE.with(|l| *l.borrow_mut() = None);
}

/// N WAIT retries make N + 1 attempts, as CMSIS-DAP counts them; dap-rs makes N.
#[test]
fn known_divergence_the_block_makes_one_wait_attempt_more() {
    let _serial = serial();
    let target = Target {
        waits: vec![1],
        ..Target::default()
    };
    for (retries, theirs, ours) in [(0, (0, 2), (2, 2)), (1, (2, 2), (3, 1))] {
        let stream = Stream {
            target: target.clone(),
            cmds: vec![
                Cmd::Connect(1),
                dap_differential::session::swd_select(),
                Cmd::Transfer(
                    0,
                    vec![Item {
                        req: 0x02,
                        word: None,
                    }],
                ),
                Cmd::Configure(0, retries, 0),
                Cmd::Block(0, 1, 0x03, vec![]),
            ],
        };
        let (t, o) = steps(&stream, composed);
        let (t, o) = (&t[5], &o[5]);
        assert_eq!((t.rates.len(), t.response[3]), theirs);
        assert_eq!((o.rates.len(), o.response[3]), ours);
        assert!(judged(&stream).fired.contains(&RuleId::WaitAttempts));
    }
    LIVE.with(|l| *l.borrow_mut() = None);
}
