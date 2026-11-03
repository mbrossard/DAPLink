//! The SPI block ahead of the session engine against dap-rs alone, both on one simulated target.

extern crate std;

use std::boxed::Box;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::format;
use std::string::String;
use std::vec;
use std::vec::Vec;

use dap_differential::{
    check, steps, verdict, Check, Cmd, Item, Line, Mode, Policy, Probe, Profile, Report, RuleId,
    Shared, SimDelay, SimPin, SimWords, Step, Stream, Target, Waiver,
};
use daplink_rs::daplink_core::{DapQueue, PacketSize};
use daplink_rs::{DapProbe, NoSwo, NoVendor, NoopLeds};
use daplink_spi_swd::{
    Dir, JtagPacing, JtagPortOps, PortResult, SpiLadder, SpiPortOps, SpiSwdDeps,
};

use super::*;
use crate::spi_swd::SpiSwdJtagDap;

const CPU_HZ: u32 = 250_000_000;
const PACKET: PacketSize = DapQueue::<1, 512>::packet_size::<512>();
const KERNELS: [u32; 3] = [64_000_000, 100_000_000, 80_000_000];
const LADDER: SpiLadder<usize, 3> = SpiLadder {
    kernels: [(0, KERNELS[0]), (1, KERNELS[1]), (2, KERNELS[2])],
    max_sck_hz: 50_000_000,
};

std::thread_local! {
    /// The kernel index this thread's board last routed.
    static MUX: Cell<usize> = const { Cell::new(0) };
}

/// The H523 kernel set; nothing here touches a register.
struct Board;

impl SpiSwdBoard for Board {
    const CLK_PIN: usize = 10;
    const IO_PIN: usize = 15;
    const TDI_PIN: usize = 7;
    const TDO_PIN: usize = 6;
    const SPI_AF: u8 = 5;
    type Source = usize;
    fn clk_port() -> pac::gpio::Gpio {
        pac::GPIOB
    }
    fn jtag_port() -> pac::gpio::Gpio {
        pac::GPIOA
    }
    fn spi() -> pac::spi::Spi {
        pac::SPI2
    }
    fn init_clocks() {}
    fn route(index: usize) {
        MUX.with(|m| m.set(index));
    }
}

/// The SCK a rung yields on the kernel the mux currently routes.
fn sck(mbr: u8) -> u32 {
    KERNELS[MUX.with(Cell::get)] >> (mbr + 1)
}

/// The STM32 seam on the simulated pads, its sessions labelled with their SCK.
struct SimPort {
    sim: Shared,
    session: Cell<(u8, u16, Dir)>,
    tx: RefCell<Vec<u32>>,
    rx: RefCell<VecDeque<u32>>,
}

impl SimPort {
    fn new(sim: &Shared) -> Self {
        let mut s = sim.borrow_mut();
        s.set_latch(Line::Tdi, false);
        s.set_output(Line::Tdi, true);
        drop(s);
        Self {
            sim: sim.clone(),
            session: Cell::new((8, 1, Dir::Tx)),
            tx: RefCell::new(Vec::new()),
            rx: RefCell::new(VecDeque::new()),
        }
    }
}

impl JtagPortOps for SimPort {
    fn jtag_pads_acquire(&mut self) {
        let mut s = self.sim.borrow_mut();
        s.set_latch(Line::Tdi, false);
        s.set_output(Line::Tdi, true);
        s.set_output(Line::Tdo, false);
    }
    fn jtag_pads_release(&mut self) {
        self.sim.borrow_mut().set_output(Line::Tdi, false);
    }
    fn tck_low(&self) {
        self.sim.borrow_mut().set_latch(Line::Swclk, false);
    }
    fn tck_high(&self) {
        self.sim.borrow_mut().set_latch(Line::Swclk, true);
    }
    fn tms_set(&self, high: bool) {
        self.sim.borrow_mut().set_latch(Line::Swdio, high);
    }
    fn tdi_set(&self, high: bool) {
        self.sim.borrow_mut().set_latch(Line::Tdi, high);
    }
    fn tdo_get(&self) -> u32 {
        self.sim.borrow().level(Line::Tdo) as u32
    }
    fn tdi_is_high(&mut self) -> bool {
        self.sim.borrow().level(Line::Tdi)
    }
    fn tdo_is_high(&mut self) -> bool {
        self.sim.borrow().level(Line::Tdo)
    }
}

impl SpiPortOps for SimPort {
    const DUPLEX: bool = false;
    const EXACT_WRITE_FRAMING: bool = true;

    type Source = usize;
    fn route(&mut self, index: usize) {
        Board::route(index)
    }
    // GPIO cells run at the request; the next SPI session relabels them.
    fn set_gpio_clock(&mut self, requested_hz: u32) {
        self.sim.borrow_mut().rate = Some(requested_hz);
    }
    fn configure(&mut self) {}
    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
        self.session.set((bits, frames, dir));
        self.sim.borrow_mut().rate = Some(sck(mbr));
    }
    fn enable(&self) {}
    fn disable(&self) {}
    fn start(&self) {
        let (bits, frames, dir) = self.session.get();
        let mut s = self.sim.borrow_mut();
        if dir == Dir::Tx {
            for f in self.tx.borrow_mut().drain(..) {
                for i in 0..bits {
                    s.spi_cell(Some(f >> i & 1 != 0));
                }
            }
        } else {
            for _ in 0..frames {
                let v = (0..bits).fold(0, |v, i| v | u32::from(s.spi_cell(None)) << i);
                self.rx.borrow_mut().push_back(v);
            }
        }
    }
    fn push8(&self, byte: u8) -> PortResult<()> {
        self.tx.borrow_mut().push(u32::from(byte));
        Ok(())
    }
    fn push16(&self, half: u16) -> PortResult<()> {
        self.tx.borrow_mut().push(u32::from(half));
        Ok(())
    }
    fn wait_eot(&self) -> PortResult<()> {
        Ok(())
    }
    fn pop32(&self) -> PortResult<u32> {
        Ok(self.rx.borrow_mut().pop_front().unwrap_or(0))
    }
    fn clear_flags(&self) -> PortResult<()> {
        Ok(())
    }
    fn pads_to_spi(&mut self) {
        let mut s = self.sim.borrow_mut();
        s.set_peripheral(Line::Swclk, true);
        s.set_peripheral(Line::Swdio, true);
    }
    fn clk_pad_to_spi(&mut self) {
        self.sim.borrow_mut().set_peripheral(Line::Swclk, true);
    }
    fn drive(&mut self) {}
    fn release(&mut self) {}
}

/// No measured ladder: every scan is counted.
struct Counted;

impl JtagPacing for Counted {
    const FAST_HALF_CYCLES: u32 = 0;
    fn pace_low(&self) {}
    fn pace_high(&self) {}
}

/// Words on the SPI pads at the block's rung; the chip's own word layer only runs on the chip.
struct Words(SimWords);

impl WordOps for Words {
    fn read(&mut self, req: u8) -> swd::Result<u32> {
        self.0.read(req)
    }
    fn write(&mut self, req: u8, data: u32) -> swd::Result<()> {
        self.0.write(req, data)
    }
    fn idle(&mut self, cycles: u8) {
        self.0.idle(cycles)
    }
}

/// The session engine under dap-rs, as the block leaves commands to it.
fn engine(
    sim: &Shared,
) -> SpiSwdJtagDap<'static, SimPort, SimPin, SimDelay, Counted, NoSwo, NoVendor, NoopLeds, 3> {
    MUX.with(|m| m.set(0));
    let pin = |line| SimPin::new(sim, line);
    let deps = SpiSwdDeps::new(
        SimPort::new(sim),
        pin(Line::Swclk),
        pin(Line::Swdio),
        SimPin::open_drain(sim, Line::Nreset),
        SimDelay,
        Counted,
        LADDER,
        dap_differential::scan_chain(8),
        Box::leak(Box::new(daplink_spi_swd::KeptAck::new(Box::leak(
            Box::default(),
        )))),
    );
    DapProbe::new(deps, NoopLeds, SimDelay, NoSwo, "diff", PACKET)
}

/// The H5 SPI image's composition: the block first, then the session engine under dap-rs.
fn composed(sim: &Shared) -> Box<dyn Probe> {
    let mut dap = engine(sim);
    let mut block = SpiBlock::<Board, 3>::new(CPU_HZ, LADDER);
    let sim = sim.clone();
    Box::new(move |report: &[u8], response: &mut [u8]| {
        let words = |rung: Rung| {
            let w = SimWords::spi(&sim);
            sim.borrow_mut().rate = Some(sck(rung.mbr));
            Words(w)
        };
        match block.process_on(report, response, words) {
            Some(len) => len,
            None => {
                let len = dap.process_command(report, response);
                block.observe(report, &response[..len.min(response.len())]);
                len
            }
        }
    })
}

/// The block's rung mirrors the engine's: one SCK per clock request.
fn one_rung(stream: &Stream, steps: &[Step]) -> Option<String> {
    let mut rate = None;
    for (i, step) in steps.iter().enumerate().skip(1) {
        if matches!(stream.cmds[i - 1], Cmd::Clock(_)) {
            rate = None;
        }
        for &r in step.rates.iter().flatten() {
            match rate {
                Some(was) if was != r => {
                    return Some(format!(
                        "step {i}: SCK {r} after {was} with no clock request"
                    ))
                }
                _ => rate = Some(r),
            }
        }
    }
    None
}

static POLICY: Policy = Policy {
    waivers: &[Waiver {
        rule: RuleId::ValueMatch,
        reason: "value-match reads go to dap-rs, which compares an AP match one read behind",
        on: Waiver::swd,
        marked: false,
    }],
};

fn sweep(profile: Profile, seeds: core::ops::Range<u64>) {
    let mut c = Check::new(profile, seeds, &POLICY);
    c.invariant = Some(&one_rung);
    if let Err(d) = check(&c, composed) {
        panic!("{d}");
    }
}

#[test]
fn swd_sessions_match_dap_rs() {
    sweep(Profile::SWD, 0..300);
}

#[test]
fn bad_read_parity_matches_the_reference() {
    let profile = Profile {
        parity: true,
        ..Profile::SWD
    };
    sweep(profile, 300..450);
    sweep(Profile::SWD_CTRL_PARITY, 0..450);
}

#[test]
fn jtag_sessions_match_dap_rs() {
    sweep(Profile::JTAG, 0..150);
}

#[test]
fn adiv6_and_absent_jtag_targets_match_the_reference() {
    sweep(Profile::JTAG_ADIV6, 0..150);
    sweep(Profile::JTAG_ABSENT, 0..100);
}

/// How the policy judges `cmds` on a fresh target, which must not diverge.
fn judged(cmds: Vec<Cmd>) -> Report {
    judged_on(Target::default(), cmds, false)
}

/// [`judged`] on `target`, optionally comparing the target's state too.
fn judged_on(target: Target, cmds: Vec<Cmd>, state: bool) -> Report {
    let mut c = Check::new(Profile::SWD, 0..0, &POLICY);
    c.invariant = Some(&one_rung);
    c.state = state;
    verdict(&c, &Stream { target, cmds }, composed).unwrap_or_else(|d| panic!("{d}"))
}

/// A block counts what completed, as Arm does; dap-rs counts the transfer that failed.
#[test]
fn known_divergence_the_count_after_a_failure() {
    // The engine's responses and dap-rs a867ca7's; a DAP_Transfer still counts as dap-rs does.
    let want: [(&[u8], &[u8]); 5] = [
        (&[5, 1, 4], &[5, 1, 4]),
        (&[5, 1, 1], &[5, 1, 0]),
        (&[6, 0, 0, 0], &[6, 1, 0, 4]),
        (&[6, 0, 0, 4], &[6, 1, 0, 4]),
        (&[6, 0, 0, 4], &[6, 1, 0, 4]),
    ];
    let stream = dap_differential::session::sticky_fault();
    let (t, o) = steps(&stream, composed);
    for (at, (ours, theirs)) in want.into_iter().enumerate() {
        let (o, t) = (&o[4 + at].response[..], &t[4 + at].response[..]);
        assert_eq!((o, t), (ours, theirs), "step {}", 4 + at);
    }
    assert!(o[6].rates.is_empty());
    assert!(judged(stream.cmds).fired.contains(&RuleId::CountCompleted));
}

/// N WAIT retries make N + 1 attempts, as CMSIS-DAP counts them; dap-rs makes N.
#[test]
fn known_divergence_the_block_makes_one_wait_attempt_more() {
    let target = Target {
        waits: vec![1],
        ..Target::default()
    };
    for (retries, theirs, ours) in [(0, (0, 2), (2, 2)), (1, (2, 2), (3, 1))] {
        let stream = Stream {
            target: target.clone(),
            cmds: vec![
                Cmd::Connect(1),
                Cmd::Clock(24_000_000),
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
        let (t, o) = (&t[6], &o[6]);
        assert_eq!((t.rates.len(), t.response[3]), theirs);
        assert_eq!((o.rates.len(), o.response[3]), ours);
        let r = judged_on(stream.target, stream.cmds, false);
        assert!(r.fired.contains(&RuleId::WaitAttempts));
    }
}

/// The host's idle cycles follow each OK ACK, a bad parity included; dap-rs clocks none.
#[test]
fn known_divergence_idle_cycles_after_an_ok_ack() {
    let waits = Target {
        waits: vec![1],
        ..Target::default()
    };
    let poisoned = Target {
        poison: Some(dap_differential::target::DPIDR),
        ..Target::default()
    };
    let read = |req| Item { req, word: None };
    let write = |req, word| Item {
        req,
        word: Some(word),
    };
    let cases = [
        // A posted AP read: after the AP read, none after the RDBUFF WAIT that runs out.
        (waits.clone(), 0, Cmd::Transfer(0, vec![read(0x03)]), 3),
        (waits.clone(), 5, Cmd::Transfer(0, vec![read(0x03)]), 6),
        (
            waits.clone(),
            5,
            Cmd::Transfer(0, vec![read(0x02), write(0x08, 0)]),
            6,
        ),
        (poisoned.clone(), 5, Cmd::Transfer(0, vec![read(0x02)]), 3),
        // The priming read, the AP read and RDBUFF, the last two after a WAIT each.
        (waits.clone(), 5, Cmd::Block(0, 2, 0x03, vec![]), 9),
        (waits, 5, Cmd::Block(0, 2, 0x01, vec![1, 2]), 6),
        (poisoned, 5, Cmd::Block(0, 2, 0x02, vec![]), 3),
    ];
    for (target, retries, cmd, cells) in cases {
        let stream = Stream {
            target,
            cmds: vec![
                Cmd::Connect(1),
                Cmd::Clock(24_000_000),
                dap_differential::session::swd_select(),
                Cmd::Configure(3, retries, 0),
                cmd,
            ],
        };
        let (t, o) = steps(&stream, composed);
        assert_eq!((t[5].idle, o[5].idle), (0, cells), "{stream}");
    }
}

/// The block clocks the idle cells the session engine it runs ahead of clocks.
#[test]
fn idle_cycles_match_the_session_engine() {
    let profile = Profile {
        parity: true,
        ..Profile::SWD
    };
    let mut idled = 0;
    for seed in 0..200 {
        let s = dap_differential::session::stream(seed, &profile);
        let sim = dap_differential::shared(&s.target);
        let ours = dap_differential::run(&s, composed(&sim).as_mut(), &sim);
        let sim = dap_differential::shared(&s.target);
        let mut alone = engine(&sim);
        let mut alone = |r: &[u8], o: &mut [u8]| alone.process_command(r, o);
        let theirs = dap_differential::run(&s, &mut alone, &sim);
        for (at, (o, t)) in ours.iter().zip(&theirs).enumerate() {
            // A run of AP reads posts once here, so idle cells compare where the packets do.
            if o.rates.len() == t.rates.len() {
                assert_eq!(o.idle, t.idle, "seed {seed} step {at}\n{s}");
                idled += (o.idle > 0) as u32;
            }
        }
    }
    assert!(idled > 0);
}

#[test]
fn a_jtag_command_is_observed() {
    let cmds = vec![
        Cmd::Connect(0),
        Cmd::Clock(1_000_000),
        Cmd::Idcode(0),
        Cmd::Block(0, 1, 0x0F, vec![]),
    ];
    judged(cmds);
}

#[test]
fn an_intercept_leaves_dap_rs_connected() {
    let cmds = vec![
        Cmd::Connect(0),
        Cmd::Clock(24_000_000),
        dap_differential::session::swd_select(),
        Cmd::Transfer(
            0,
            vec![dap_differential::Item {
                req: 0x01,
                word: Some(0),
            }],
        ),
        Cmd::JtagConfigure(vec![4]),
    ];
    judged(cmds);
}

#[test]
fn a_clock_below_the_slowest_rung_leaves_the_transfers_to_the_engine() {
    let read = |req| dap_differential::Item { req, word: None };
    let cmds = vec![
        Cmd::Connect(0),
        Cmd::Clock(24_000_000),
        dap_differential::session::swd_select(),
        Cmd::Transfer(0, vec![read(0x02)]),
        Cmd::Clock(100_000),
        Cmd::Transfer(0, vec![read(0x02)]),
        Cmd::Block(0, 2, 0x06, vec![]),
    ];
    judged(cmds);
}

#[test]
fn a_failed_ack_below_the_slowest_rung_leaves_no_park_cells() {
    let target = Target {
        mode: Mode::Jtag,
        ..Target::default()
    };
    let read = |req| dap_differential::Item { req, word: None };
    let cmds = vec![
        Cmd::Connect(1),
        Cmd::Clock(100_000),
        Cmd::Transfer(0, vec![read(0x02)]),
        Cmd::Block(0, 1, 0x06, vec![]),
    ];
    judged_on(target, cmds, true);
}
