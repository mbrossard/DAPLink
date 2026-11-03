//! The bit-bang engines against dap-rs alone, both on one simulated target.

extern crate std;
use std::boxed::Box;
use std::vec;
use std::vec::Vec;

use dap_differential::session::swd_select;
use dap_differential::{
    check, steps, verdict, Check, Cmd, Effect, Item, Line, Policy, Probe, Profile, Report, RuleId,
    Shared, SimDelay, SimPin, Step, Stream, Target, Waiver,
};

use crate::dap_probe::{BitbangDap, DapProbe, SwdOnlyDap};
use crate::late_jtag::LateJtagDeps;
use crate::leds::NoopLeds;
use crate::wired::{wired_swd_deps, ScanChainStorage};
use dap_rs::swd::{self, APnDP, DPRegister};
use dap_rs::swo::NoSwo;
use daplink_core::{DapQueue, PacketSize};

const PACKET: PacketSize = DapQueue::<1, 512>::packet_size::<512>();

fn scan() -> &'static ScanChainStorage<8> {
    Box::leak(Box::new(ScanChainStorage::new()))
}

/// The late-sample engine, as the bit-bang images compose it.
fn late(sim: &Shared) -> Box<dyn Probe> {
    let pin = |line| SimPin::new(sim, line);
    let deps = LateJtagDeps::wired(
        SimPin::open_drain(sim, Line::Nreset),
        pin(Line::Tdi),
        pin(Line::Swdio),
        pin(Line::Swclk),
        pin(Line::Tdo),
        SimDelay,
        scan(),
    );
    let mut dap: BitbangDap<'static, SimPin, SimDelay> =
        DapProbe::new(deps, NoopLeds, SimDelay, NoSwo, "diff", PACKET);
    Box::new(move |report: &[u8], response: &mut [u8]| dap.process_command(report, response))
}

/// The SWD-only engine.
fn swd_only(sim: &Shared) -> Box<dyn Probe> {
    let pin = |line| SimPin::new(sim, line);
    let deps = wired_swd_deps(
        SimPin::open_drain(sim, Line::Nreset),
        pin(Line::Tdi),
        pin(Line::Swdio),
        pin(Line::Swclk),
        pin(Line::Tdo),
        SimDelay,
        scan(),
    );
    let mut dap: SwdOnlyDap<'static, SimPin, SimDelay> =
        DapProbe::new(deps, NoopLeds, SimDelay, NoSwo, "diff", PACKET);
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

fn sweep(profile: Profile, seeds: core::ops::Range<u64>, make: fn(&Shared) -> Box<dyn Probe>) {
    if let Err(d) = check(&Check::new(profile, seeds, &POLICY), make) {
        panic!("{d}");
    }
}

#[test]
fn late_swd_sessions_match_dap_rs() {
    sweep(Profile::SWD, 0..300, late);
}

#[test]
fn late_jtag_sessions_match_dap_rs() {
    sweep(Profile::JTAG, 0..200, late);
}

#[test]
fn swd_only_sessions_match_dap_rs() {
    sweep(Profile::SWD, 0..300, swd_only);
}

#[test]
fn bad_ctrl_parity_matches_the_reference() {
    sweep(Profile::SWD_CTRL_PARITY, 0..450, late);
    sweep(Profile::SWD_CTRL_PARITY, 0..450, swd_only);
}

#[test]
fn powered_up_ctrl_parity_reaches_each_engine() {
    for make in [late, swd_only] {
        sweep(Profile::SWD_CTRL_PARITY, 0..450, make);
        let cmds = vec![
            Cmd::Connect(1),
            swd_select(),
            Cmd::Transfer(
                0,
                vec![
                    read(0x02),
                    Item {
                        req: 0x04,
                        word: Some(0x5000_0000),
                    },
                ],
            ),
            Cmd::Transfer(0, vec![read(0x06)]),
            Cmd::Abort(0, 0x1E),
        ];
        let (_, ours) = both(
            Target {
                poison: Some(Profile::SWD_CTRL_PARITY.poison),
                ..Target::default()
            },
            &cmds,
            make,
        );
        assert_eq!(ours[3].response[1..3], [2, 1]);
        assert_eq!(ours[4].response, [5, 1, 0x0F]);
        assert_eq!(ours[5].response, [8, 0]);
        assert_eq!(ours[5].effects, [Effect::Abort(0x1E)]);
    }
}

/// How the policy judges `cmds` on `target`, which must not diverge.
fn judged(target: Target, cmds: Vec<Cmd>, make: fn(&Shared) -> Box<dyn Probe>) -> Report {
    let c = Check::new(Profile::SWD, 0..0, &POLICY);
    verdict(&c, &Stream { target, cmds }, make).unwrap_or_else(|d| panic!("{d}"))
}

/// The reference's and `make`'s steps on `cmds` from a fresh `target`.
fn both(
    target: Target,
    cmds: &[Cmd],
    make: fn(&Shared) -> Box<dyn Probe>,
) -> (Vec<Step>, Vec<Step>) {
    let stream = Stream {
        target,
        cmds: cmds.to_vec(),
    };
    steps(&stream, make)
}

#[test]
fn known_divergence_a_write_no_ack_status() {
    let write = Item {
        req: 0x05,
        word: Some(0),
    };
    for make in [late, swd_only] {
        let cmds = vec![Cmd::Connect(1), Cmd::Transfer(0, vec![write])];
        let (t, o) = both(Target::default(), &cmds, make);
        assert_eq!(o[2].response, [0x05, 1, 0x07]);
        assert_eq!(t[2].response, [0x05, 1, 0x0F]);
        let r = judged(Target::default(), cmds, make);
        assert!(r.fired.contains(&RuleId::NoAckStatus));
    }
}

#[test]
fn known_divergence_the_turnaround_after_a_bad_parity() {
    let target = Target {
        poison: Some(dap_differential::target::DPIDR),
        ..Target::default()
    };
    let read = Item {
        req: 0x02,
        word: None,
    };
    for make in [late, swd_only] {
        let cmds = vec![
            Cmd::Connect(1),
            swd_select(),
            Cmd::Transfer(0, vec![read]),
            Cmd::Abort(0, 0x1E),
        ];
        // The read fails alike; dap-rs then misframes the abort.
        let (t, o) = both(target.clone(), &cmds, make);
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
        let r = judged(target.clone(), cmds, make);
        assert_eq!((r.explained, r.unchecked), (1, 1));
    }
}

fn read(req: u8) -> Item {
    Item { req, word: None }
}

/// A block counts what completed, as Arm does; DAP_Transfer counts the failed one, as dap-rs does.
#[test]
fn known_divergence_the_count_after_a_failure() {
    // Responses, then SWD packets, the engine's and dap-rs a867ca7's.
    type Answers = (&'static [u8], &'static [u8], (usize, usize));
    let want: [Answers; 5] = [
        (&[5, 1, 4], &[5, 1, 4], (1, 1)),
        (&[5, 1, 1], &[5, 1, 0], (0, 0)),
        (&[6, 0, 0, 0], &[6, 1, 0, 4], (0, 1)),
        (&[6, 0, 0, 4], &[6, 1, 0, 4], (1, 1)),
        (&[6, 0, 0, 4], &[6, 1, 0, 4], (1, 1)),
    ];
    for make in [late, swd_only] {
        let cmds = dap_differential::session::sticky_fault().cmds;
        let (t, o) = both(Target::default(), &cmds, make);
        // The posted read is counted by both, its word absent.
        assert_eq!(
            (&o[3].response[..3], &t[3].response[..3]),
            (&[5, 6, 4][..], &[5, 6, 4][..])
        );
        for (at, (ours, theirs, packets)) in want.into_iter().enumerate() {
            let (o, t) = (&o[4 + at], &t[4 + at]);
            assert_eq!(
                (&o.response[..], &t.response[..]),
                (ours, theirs),
                "step {}",
                4 + at
            );
            assert_eq!((o.rates.len(), t.rates.len()), packets, "step {}", 4 + at);
        }
        // Arm's DAP.c answers the first 05 00 04: the faulting AP read was never posted.
        let arm = [5u8, 0, 4];
        assert_eq!(o[4].response, [arm[0], arm[1] + 1, arm[2]]);
        let r = judged(Target::default(), cmds, make);
        assert!(r.fired.contains(&RuleId::CountCompleted));
    }
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
        let stream = Stream {
            target: target.clone(),
            cmds: vec![
                Cmd::Connect(1),
                swd_select(),
                Cmd::Configure(0, retries, 0),
                Cmd::Transfer(0, vec![read(0x03)]),
            ],
        };
        for make in [late, swd_only] {
            let (t, o) = steps(&stream, make);
            let (t, o) = (&t[4], &o[4]);
            assert_eq!((t.rates.len(), t.response[2]), theirs);
            assert_eq!((o.rates.len(), o.response[2]), ours);
            let r = judged(target.clone(), stream.cmds.clone(), make);
            assert!(r.fired.contains(&RuleId::WaitAttempts));
        }
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
    let select = Item {
        req: 0x08,
        word: Some(0),
    };
    let cases = [
        // A posted AP read: after the AP read, none after the RDBUFF WAIT that runs out.
        (waits.clone(), 0, vec![read(0x03)], 3),
        (waits.clone(), 5, vec![read(0x03)], 6),
        (waits, 5, vec![read(0x02), select], 6),
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
        for make in [late, swd_only] {
            let (t, o) = steps(&stream, make);
            assert_eq!((t[4].idle, o[4].idle), (0, cells), "{stream}");
        }
    }
}

const SWD_SELECT: [u8; 17] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00,
];

/// The cells one transfer clocks on a selected target, idle cycles set to three.
fn cells<D, S: dap_rs::swd::Swd<D>>(
    target: &Target,
    make: &impl Fn(&Shared, &'static ScanChainStorage<8>) -> S,
    transfer: impl FnOnce(&mut S),
) -> usize {
    let sim = dap_differential::shared(target);
    let storage = scan();
    let mut drv = make(&sim, storage);
    storage.jtag_dp().observe(&[4, 3, 0, 0, 0, 0]);
    drv.write_sequence(136, &SWD_SELECT).unwrap();
    let _ = drv.read_inner(APnDP::DP, DPRegister::DPIDR);
    sim.borrow_mut().record(true);
    let before = sim.borrow().trace().len();
    transfer(&mut drv);
    let n = sim.borrow().trace().len();
    n - before
}

/// `read_inner` and `write_inner`, which the flash mux calls per attempt, owe what the loops owe.
fn inner_transfers_owe_what_the_loops_owe<D, S: dap_rs::swd::Swd<D>>(
    make: impl Fn(&Shared, &'static ScanChainStorage<8>) -> S,
) {
    let poisoned = Target {
        poison: Some(dap_differential::target::DPIDR),
        ..Target::default()
    };
    let read = |s: &mut S| {
        assert_eq!(
            s.read(0, APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::BadParity)
        );
    };
    let inner = |s: &mut S| {
        assert_eq!(
            s.read_inner(APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::BadParity)
        );
    };
    assert_eq!(
        cells(&poisoned, &make, read),
        cells(&poisoned, &make, inner)
    );

    let target = Target::default();
    let write = |s: &mut S| s.write(0, APnDP::DP, DPRegister::DPIDR, 0x1E).unwrap();
    let inner = |s: &mut S| s.write_inner(APnDP::DP, DPRegister::DPIDR, 0x1E).unwrap();
    assert_eq!(cells(&target, &make, write), cells(&target, &make, inner));

    let sim = dap_differential::shared(&target);
    let mut unselected = make(&sim, scan());
    assert_eq!(
        unselected.write_inner(APnDP::DP, DPRegister::DPIDR, 0x1E),
        Err(swd::Error::NoAck)
    );
}

#[test]
fn the_swd_only_inner_transfers_settle_like_its_loops() {
    inner_transfers_owe_what_the_loops_owe(|sim, storage| {
        let pin = |line| SimPin::new(sim, line);
        let drv: crate::swd_only::SwdOnlySwdDrv<SimPin, SimDelay> = wired_swd_deps(
            SimPin::open_drain(sim, Line::Nreset),
            pin(Line::Tdi),
            pin(Line::Swdio),
            pin(Line::Swclk),
            pin(Line::Tdo),
            SimDelay,
            storage,
        )
        .into();
        drv
    });
}

#[test]
fn the_late_sample_inner_transfers_settle_like_its_loops() {
    inner_transfers_owe_what_the_loops_owe(|sim, storage| {
        let pin = |line| SimPin::new(sim, line);
        let drv: crate::late_jtag::LateSwdDrv<SimPin, SimDelay> = LateJtagDeps::wired(
            SimPin::open_drain(sim, Line::Nreset),
            pin(Line::Tdi),
            pin(Line::Swdio),
            pin(Line::Swclk),
            pin(Line::Tdo),
            SimDelay,
            storage,
        )
        .into();
        drv
    });
}

/// dap-rs's JTAG write arm takes bit 5 as a real write, so its timestamp must be charged.
#[test]
fn a_jtag_timestamped_bit_5_write_cannot_overrun_the_reply() {
    let sim = dap_differential::shared(&Target::default());
    let mut probe = late(&sim);
    let mut resp = [0u8; 64];
    for c in [
        vec![0x02, 0x02],
        dap_differential::session::jtag_select().encode(),
        vec![0x14, 2, 0x46, 0x3F, 0x01, 0x00],
        vec![0x15, 1, 4],
    ] {
        probe.process(&c, &mut resp);
    }
    let mut report = vec![0x05, 0, 16, 0xA8, 0, 0, 0, 0];
    report.extend([0x0E; 15]);
    assert_eq!(probe.process(&report, &mut resp), 1);
    assert_eq!(resp[0], 0xFF);
}
