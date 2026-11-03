//! The session engine against dap-rs's own bit-bang engine, on one simulated target.

extern crate std;

use std::boxed::Box;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::vec;
use std::vec::Vec;

use dap_differential::session::{jtag_select, swd_select};
use dap_differential::{
    check, steps, verdict, Check, Cmd, Divergence, Effect, Item, Kind, Line, Mode, Policy, Probe,
    Profile, Report, RuleId, Shared, SimDelay, SimPin, Step, Stream, Target, Waiver,
};
use dap_rs::swo::NoSwo;
use daplink_core::DapQueue;
use daplink_rs::DapProbe;

use crate::{
    Dir, JtagPacing, JtagPortOps, NoFastJtag, PortResult, SpiJtag, SpiLadder, SpiPortOps, SpiSwd,
    SpiSwdDeps,
};

/// The H523's SPI2 kernel and ceiling, so the ladder has the shipping rungs.
const KERNEL_HZ: u32 = 64_000_000;
const MAX_SCK_HZ: u32 = 50_000_000;

/// The chip seam on the simulated pads: sessions clock SPI cells, TAP lines are GPIO latches.
struct SimPort<const EXACT: bool> {
    sim: Shared,
    session: RefCell<(u8, u16, Dir)>,
    tx: RefCell<Vec<u32>>,
    rx: RefCell<VecDeque<u32>>,
}

impl<const EXACT: bool> SimPort<EXACT> {
    /// The STM32 port's boot state: TDI a low output, TDO an input.
    fn new(sim: &Shared) -> Self {
        let mut s = sim.borrow_mut();
        s.set_latch(Line::Tdi, false);
        s.set_output(Line::Tdi, true);
        s.set_output(Line::Tdo, false);
        drop(s);
        Self {
            sim: sim.clone(),
            session: RefCell::new((8, 1, Dir::Tx)),
            tx: RefCell::new(Vec::new()),
            rx: RefCell::new(VecDeque::new()),
        }
    }
}

impl<const EXACT: bool> JtagPortOps for SimPort<EXACT> {
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

impl<const EXACT: bool> SpiPortOps for SimPort<EXACT> {
    const DUPLEX: bool = false;
    type Source = ();
    const EXACT_WRITE_FRAMING: bool = EXACT;

    fn configure(&mut self) {}

    // GPIO cells run at the request; the next SPI session relabels them.
    fn set_gpio_clock(&mut self, requested_hz: u32) {
        self.sim.borrow_mut().rate = Some(requested_hz);
    }

    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
        *self.session.borrow_mut() = (bits, frames, dir);
        self.sim.borrow_mut().rate = Some(KERNEL_HZ / (2 << mbr));
    }

    fn enable(&self) {}

    fn disable(&self) {}

    fn start(&self) {
        let (bits, frames, dir) = *self.session.borrow();
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

/// A measured-looking ladder: fast below eight half-cycles, so both scan paths run.
struct Ladder;

impl JtagPacing for Ladder {
    const FAST_HALF_CYCLES: u32 = 8;

    fn pace_low(&self) {}

    fn pace_high(&self) {}
}

/// The session engine under the probe facade the firmwares use.
type Engine<const EXACT: bool, J> = DapProbe<
    'static,
    SpiSwdDeps<SimPort<EXACT>, SimPin, SimDelay, J>,
    dap_differential::NoLeds,
    SimDelay,
    SpiJtag<SimPort<EXACT>, SimPin, SimDelay, J>,
    SpiSwd<SimPort<EXACT>, SimPin, SimDelay, J>,
    NoSwo,
>;

/// The session engine under the probe facade, parked as a firmware parks it.
fn engine<const EXACT: bool, J: JtagPacing + 'static>(sim: &Shared, pacing: J) -> Box<dyn Probe> {
    let pin = |line| SimPin::new(sim, line);
    let deps = SpiSwdDeps::new(
        SimPort::<EXACT>::new(sim),
        pin(Line::Swclk),
        pin(Line::Swdio),
        SimPin::open_drain(sim, Line::Nreset),
        SimDelay,
        pacing,
        SpiLadder {
            kernels: [((), KERNEL_HZ)],
            max_sck_hz: MAX_SCK_HZ,
        },
        dap_differential::scan_chain(8),
        Box::leak(Box::new(crate::KeptAck::new(Box::leak(Box::default())))),
    );
    let packet = DapQueue::<1, 512>::packet_size::<512>();
    let mut dap: Engine<EXACT, J> = DapProbe::new(
        deps,
        dap_differential::NoLeds,
        SimDelay,
        NoSwo,
        "diff",
        packet,
    );
    Box::new(move |report: &[u8], response: &mut [u8]| dap.process_command(report, response))
}

static POLICY: Policy = Policy {
    waivers: &[Waiver {
        rule: RuleId::ValueMatch,
        reason: "value-match reads go to dap-rs, which compares an AP match one read behind",
        on: Waiver::swd,
        marked: false,
    }],
};

fn sweep(
    profile: Profile,
    seeds: core::ops::Range<u64>,
    make: impl FnMut(&Shared) -> Box<dyn Probe>,
) {
    if let Err(d) = check(&Check::new(profile, seeds, &POLICY), make) {
        panic!("{d}");
    }
}

#[test]
fn swd_sessions_match_dap_rs() {
    sweep(Profile::SWD, 0..300, |sim| engine::<true, _>(sim, Ladder));
}

#[test]
fn padded_write_framing_matches_dap_rs() {
    sweep(Profile::SWD, 300..450, |sim| {
        engine::<false, _>(sim, NoFastJtag)
    });
}

#[test]
fn bad_ctrl_parity_matches_the_reference() {
    sweep(Profile::SWD_CTRL_PARITY, 0..450, |sim| {
        engine::<true, _>(sim, Ladder)
    });
}

#[test]
fn powered_up_ctrl_parity_reaches_the_spi_engine() {
    sweep(Profile::SWD_CTRL_PARITY, 0..450, |sim| {
        engine::<true, _>(sim, Ladder)
    });
    sweep(Profile::SWD_CTRL_PARITY, 0..450, |sim| {
        engine::<false, _>(sim, NoFastJtag)
    });
    for make in [
        (|sim: &Shared| engine::<true, _>(sim, Ladder)) as fn(&Shared) -> Box<dyn Probe>,
        |sim: &Shared| engine::<false, _>(sim, NoFastJtag),
    ] {
        let cmds = vec![
            Cmd::Connect(1),
            swd_select(),
            Cmd::Transfer(0, vec![read(0x02), write(0x04, 0x5000_0000)]),
            Cmd::Transfer(0, vec![read(0x06)]),
            Cmd::Abort(0, 0x1E),
        ];
        let (_, ours) = steps(
            &Stream {
                target: Target {
                    poison: Some(Profile::SWD_CTRL_PARITY.poison),
                    ..Target::default()
                },
                cmds,
            },
            make,
        );
        assert_eq!(ours[3].response[1..3], [2, 1]);
        assert_eq!(ours[4].response, [5, 1, 0x0F]);
        assert_eq!(ours[5].response, [8, 0]);
        assert_eq!(ours[5].effects, [Effect::Abort(0x1E)]);
    }
}

#[test]
fn jtag_sessions_match_dap_rs() {
    sweep(Profile::JTAG, 0..250, |sim| engine::<true, _>(sim, Ladder));
}

#[test]
fn scaled_jtag_scans_match_dap_rs() {
    sweep(Profile::JTAG, 250..400, |sim| {
        engine::<true, _>(sim, NoFastJtag)
    });
}

#[test]
fn adiv6_jtag_sessions_match_the_reference() {
    sweep(Profile::JTAG_ADIV6, 0..250, |sim| {
        engine::<true, _>(sim, Ladder)
    });
    sweep(Profile::JTAG_ADIV6, 250..350, |sim| {
        engine::<true, _>(sim, NoFastJtag)
    });
}

#[test]
fn absent_jtag_targets_match_the_reference() {
    sweep(Profile::JTAG_ABSENT, 0..150, |sim| {
        engine::<true, _>(sim, Ladder)
    });
}

/// How the policy judges `cmds` on `target`, optionally comparing the target's state too.
fn verdict_on(target: Target, cmds: Vec<Cmd>, state: bool) -> Result<Report, Box<Divergence>> {
    let mut c = Check::new(Profile::SWD, 0..0, &POLICY);
    c.state = state;
    verdict(&c, &Stream { target, cmds }, |sim| {
        engine::<true, _>(sim, Ladder)
    })
}

/// [`verdict_on`], which must not diverge.
fn judged(target: Target, cmds: Vec<Cmd>, state: bool) -> Report {
    verdict_on(target, cmds, state).unwrap_or_else(|d| panic!("{d}"))
}

fn swd() -> Target {
    Target::default()
}

/// The reference's and the engine's steps on `cmds` from a fresh `target`.
fn both(target: Target, cmds: &[Cmd]) -> (Vec<Step>, Vec<Step>) {
    let stream = Stream {
        target,
        cmds: cmds.to_vec(),
    };
    steps(&stream, |sim| engine::<true, _>(sim, Ladder))
}

fn read(req: u8) -> Item {
    Item { req, word: None }
}

fn write(req: u8, word: u32) -> Item {
    Item {
        req,
        word: Some(word),
    }
}

#[test]
fn known_divergence_a_write_no_ack_status() {
    let cmds = vec![Cmd::Connect(1), Cmd::Transfer(0, vec![write(0x05, 0)])];
    let (t, o) = both(swd(), &cmds);
    assert_eq!(o[2].response, [0x05, 1, 0x07]);
    assert_eq!(t[2].response, [0x05, 1, 0x0F]);
    assert!(judged(swd(), cmds, false)
        .fired
        .contains(&RuleId::NoAckStatus));
}

/// `RuleId::TdiLevel`, left open: TDI reads back low while parked, where dap-rs leaves it high.
#[test]
fn known_divergence_tdi_held_low_while_parked() {
    let cmds = vec![Cmd::Pins(0, 0, 0)];
    let (t, o) = both(swd(), &cmds);
    assert_eq!(
        (&o[1].response[..], o[1].levels[2]),
        (&[0x10, 0xAA][..], false)
    );
    assert_eq!(
        (&t[1].response[..], t[1].levels[2]),
        (&[0x10, 0xAE][..], true)
    );
    judged(swd(), cmds, false);
}

/// `RuleId::ParkedPinSelect`, left open: the engine drives a parked SWCLK it selects.
#[test]
fn known_divergence_swj_pins_drives_a_parked_pin() {
    let cmds = vec![Cmd::Pins(1, 1, 0)];
    let (t, o) = both(swd(), &cmds);
    assert_eq!(
        (&o[1].response[..], o[1].drives[0]),
        (&[0x10, 0xAB][..], Some(true))
    );
    assert_eq!(
        (&t[1].response[..], t[1].drives[0]),
        (&[0x10, 0xAE][..], None)
    );
    judged(swd(), cmds, false);
}

#[test]
fn known_divergence_nreset_held_across_the_next_transfer() {
    let cmds = vec![
        Cmd::Connect(1),
        Cmd::Pins(0, 0x80, 0),
        Cmd::Transfer(0, vec![read(0x02)]),
    ];
    let (t, o) = both(swd(), &cmds);
    assert_eq!((&o[3].effects[..], o[3].levels[4]), (&[][..], false));
    assert_eq!(
        (&t[3].effects[..], t[3].levels[4]),
        (&[Effect::Reset(true)][..], true)
    );
    // The reference cannot be normalized to `RuleId::NresetHeld`; streams release every pulse.
    let d = verdict_on(swd(), cmds, false).unwrap_err();
    assert_eq!((d.kind, d.at), (Kind::Effects, 3));
}

#[test]
fn a_clock_below_the_slowest_rung_runs_on_gpio() {
    let cmds = vec![
        Cmd::Connect(1),
        Cmd::Clock(100_000),
        swd_select(),
        Cmd::Transfer(0, vec![read(0x02), write(0x05, 0), read(0x02)]),
    ];
    judged(swd(), cmds, true);
}

#[test]
fn swj_pins_leave_no_stray_swclk_edge() {
    let cmds = vec![
        Cmd::Connect(1),
        swd_select(),
        Cmd::Pins(0x02, 0x03, 0),
        Cmd::Transfer(0, vec![read(0x02)]),
    ];
    judged(swd(), cmds, false);
}

#[test]
fn a_sequence_after_swj_pins_leaves_no_stray_swclk_edge() {
    let cmds = vec![
        Cmd::Connect(1),
        swd_select(),
        Cmd::Pins(0x02, 0x03, 0),
        Cmd::SwjSequence(8, vec![0]),
        Cmd::Transfer(0, vec![read(0x02)]),
    ];
    judged(swd(), cmds, true);
}

#[test]
fn jtag_entry_leaves_no_stray_tck_edge() {
    let target = Target {
        mode: Mode::Jtag,
        ..Target::default()
    };
    let cmds = vec![
        Cmd::Connect(2),
        jtag_select(),
        Cmd::JtagConfigure(vec![4]),
        Cmd::JtagSequence(vec![(0x46, vec![0x3F]), (0x01, vec![0])]),
        Cmd::Pins(0x02, 0x03, 0),
        Cmd::Transfer(0, vec![read(0x06)]),
    ];
    judged(target, cmds, false);
}

#[test]
fn a_failed_ack_leaves_no_park_cells() {
    let target = Target {
        mode: Mode::Jtag,
        ..Target::default()
    };
    let cmds = vec![Cmd::Connect(1), Cmd::Transfer(0, vec![read(0x02)])];
    judged(target, cmds, true);
}

/// `RuleId::ParkedAfterConnect`, left open: the pads stay parked until the first wire command.
#[test]
fn known_divergence_connect_leaves_the_pads_parked() {
    let cmds = vec![
        swd_select(),
        Cmd::Disconnect,
        Cmd::Connect(0),
        Cmd::Pins(0x80, 0x80, 0),
    ];
    let (t, o) = both(swd(), &cmds);
    assert_eq!(
        (&o[4].response[..], o[4].drives[0]),
        (&[0x10, 0xAA][..], None)
    );
    assert_eq!(
        (&t[4].response[..], t[4].drives[0]),
        (&[0x10, 0xAF][..], Some(true))
    );
    judged(swd(), cmds, false);
}

/// `RuleId::SwdioLevel`, left open: SWDIO floats high after a sequence; dap-rs drives it low.
#[test]
fn known_divergence_swdio_floats_after_a_sequence() {
    let cmds = vec![swd_select(), Cmd::Pins(0x80, 0x80, 0)];
    let (t, o) = both(swd(), &cmds);
    assert_eq!(
        (&o[2].response[..], o[2].levels[1]),
        (&[0x10, 0xAB][..], true)
    );
    assert_eq!(
        (&t[2].response[..], t[2].levels[1]),
        (&[0x10, 0xA9][..], false)
    );
    judged(swd(), cmds, false);
}

#[test]
fn known_divergence_the_turnaround_after_a_bad_parity() {
    let target = Target {
        poison: Some(dap_differential::target::DPIDR),
        ..Target::default()
    };
    let cmds = vec![
        Cmd::Connect(1),
        swd_select(),
        Cmd::Transfer(0, vec![read(0x02)]),
        Cmd::Abort(0, 0x1E),
    ];
    // The read fails alike; only the engine's turnaround leaves the abort framed.
    let (t, o) = both(target.clone(), &cmds);
    assert_eq!(o[3].response, t[3].response);
    assert_eq!(
        (&o[4].response[..], &o[4].effects[..]),
        (&[0x08, 0][..], &[Effect::Abort(0x1E)][..])
    );
    assert_eq!(
        (&t[4].response[..], &t[4].effects[..]),
        (&[0x08, 0xFF][..], &[][..])
    );
    // Under the policy the comparison stops, explained, at the read.
    let r = judged(target, cmds, false);
    assert_eq!((r.explained, r.unchecked), (1, 1));
}

/// N WAIT retries make N + 1 attempts, as CMSIS-DAP counts them; dap-rs makes N.
#[test]
fn known_divergence_one_wait_attempt_more() {
    let target = Target {
        waits: vec![1],
        ..Target::default()
    };
    // Packets and status of a posted AP read, whose RDBUFF read waits once: dap-rs's, the engine's.
    for (retries, theirs, ours) in [(0, (0, 2), (2, 2)), (1, (2, 2), (3, 1))] {
        let cmds = vec![
            Cmd::Connect(1),
            swd_select(),
            Cmd::Configure(0, retries, 0),
            Cmd::Transfer(0, vec![read(0x03)]),
        ];
        let stream = Stream {
            target: target.clone(),
            cmds: cmds.clone(),
        };
        let (t, o) = steps(&stream, |sim| engine::<true, _>(sim, Ladder));
        assert_eq!((t[4].rates.len(), t[4].response[2]), theirs);
        assert_eq!((o[4].rates.len(), o[4].response[2]), ours);
        let r = judged(target.clone(), cmds, false);
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
    let cases = [
        // A posted AP read: after the AP read, none after the RDBUFF WAIT that runs out.
        (waits.clone(), 0, vec![read(0x03)], 3),
        (waits.clone(), 5, vec![read(0x03)], 6),
        (waits, 5, vec![read(0x02), write(0x08, 0)], 6),
        (poisoned, 5, vec![read(0x02)], 3),
    ];
    for (target, retries, items, cells) in cases {
        let stream = Stream {
            target,
            cmds: vec![
                Cmd::Connect(1),
                swd_select(),
                Cmd::Configure(3, retries, 0),
                Cmd::Transfer(0, items),
            ],
        };
        let (t, o) = steps(&stream, |sim| engine::<true, _>(sim, Ladder));
        assert_eq!((t[4].idle, o[4].idle), (0, cells), "{stream}");
    }
}
