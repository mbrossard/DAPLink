//! The block ahead of the bit-bang engine against dap-rs alone, both on one simulated target.

extern crate std;

use std::boxed::Box;
use std::vec;
use std::vec::Vec;

use super::tests::P;
use super::*;

mod differential {
    use super::*;
    use dap_differential::session::{jtag_select, stream, swd_select};
    use dap_differential::{
        check, run, shared, steps, verdict, Check, Cmd, Divergence, Effect, Item, Kind, Line,
        Policy, Probe, Profile, Report, RuleId, Shared, SimDelay, SimPin, Step, Stream, Target,
        Waiver,
    };
    use daplink_rs::daplink_core::{DapQueue, PacketSize};
    use daplink_rs::{BitbangDap, DapProbe, LateJtagDeps, NoSwo, NoopLeds, ScanChainStorage};

    const PACKET: PacketSize = DapQueue::<1, 512>::packet_size::<512>();

    /// The block's wire on the simulated pads: stores become latch and direction writes.
    struct SimWire(Shared);

    impl Wire for SimWire {
        fn clk(&mut self, high: bool) {
            self.0.borrow_mut().set_latch(Line::Swclk, high);
        }
        fn swdio(&mut self, bit: u32) {
            self.0.borrow_mut().set_latch(Line::Swdio, bit & 1 != 0);
        }
        fn sample(&mut self) -> u32 {
            self.0.borrow().level(Line::Swdio) as u32
        }
        fn set_out(&mut self, out: bool) {
            self.0.borrow_mut().set_output(Line::Swdio, out);
        }
        fn entry_out(&mut self) -> bool {
            self.0.borrow().pad(Line::Swdio).output
        }
    }

    /// The bit-bang engine alone, as single transfers and every other command reach it.
    fn engine(sim: &Shared) -> BitbangDap<'static, SimPin, SimDelay> {
        let scan: &'static ScanChainStorage<8> = Box::leak(Box::new(ScanChainStorage::new()));
        let pin = |line| SimPin::new(sim, line);
        let deps = LateJtagDeps::wired(
            SimPin::open_drain(sim, Line::Nreset),
            pin(Line::Tdi),
            pin(Line::Swdio),
            pin(Line::Swclk),
            pin(Line::Tdo),
            SimDelay,
            scan,
        );
        DapProbe::new(deps, NoopLeds, SimDelay, NoSwo, "diff", PACKET)
    }

    /// The H5 bit-bang image's composition: the block first, then the late-sample engine.
    fn composed(sim: &Shared) -> Box<dyn Probe> {
        let mut dap = engine(sim);
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        let sim = sim.clone();
        Box::new(move |report: &[u8], response: &mut [u8]| {
            let wire = || SimWire(sim.clone());
            match block.process_on(report, response, wire) {
                Some(len) => len,
                None => {
                    let len = dap.process_command(report, response);
                    block.observe(report, &response[..len.min(response.len())]);
                    len
                }
            }
        })
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
        if let Err(d) = check(&Check::new(profile, seeds, &POLICY), composed) {
            panic!("{d}");
        }
    }

    #[test]
    fn swd_sessions_match_dap_rs() {
        sweep(Profile::SWD, 0..300);
    }

    #[test]
    fn powered_up_ctrl_parity_reaches_the_block() {
        sweep(Profile::SWD_CTRL_PARITY, 0..450);
        let cmds = vec![
            Cmd::Connect(1),
            swd_select(),
            Cmd::Transfer(
                0,
                vec![
                    Item {
                        req: 0x02,
                        word: None,
                    },
                    write(0x04, 0x5000_0000),
                ],
            ),
            Cmd::Block(0, 1, 0x06, vec![]),
            Cmd::Abort(0, 0x1E),
        ];
        let (_, ours) = both(
            Target {
                poison: Some(Profile::SWD_CTRL_PARITY.poison),
                ..Target::default()
            },
            &cmds,
        );
        assert_eq!(ours[3].response[1..3], [2, 1]);
        assert_eq!(ours[4].response, [6, 0, 0, 0x0F]);
        assert_eq!(ours[5].response, [8, 0]);
        assert_eq!(ours[5].effects, [Effect::Abort(0x1E)]);
    }

    #[test]
    fn jtag_sessions_match_dap_rs() {
        sweep(Profile::JTAG, 0..200);
    }

    #[test]
    fn adiv6_and_absent_jtag_targets_match_the_reference() {
        sweep(Profile::JTAG_ADIV6, 0..200);
        sweep(Profile::JTAG_ABSENT, 0..100);
    }

    #[test]
    fn bad_ctrl_parity_matches_the_reference() {
        sweep(Profile::SWD_CTRL_PARITY, 0..450);
    }

    /// dap-rs misframes the packet after a bad parity, so this compares with the engine alone.
    #[test]
    fn bad_read_parity_matches_the_engine() {
        let profile = Profile {
            parity: true,
            ..Profile::SWD
        };
        let (mut poisoned, mut idled) = (0, 0);
        for seed in 300..450 {
            let s = stream(seed, &profile);
            poisoned += s.target.poison.is_some() as u32;
            let sim = shared(&s.target);
            let ours = run(&s, composed(&sim).as_mut(), &sim);
            let sim = shared(&s.target);
            let mut alone = engine(&sim);
            let mut alone = |r: &[u8], o: &mut [u8]| alone.process_command(r, o);
            let theirs = run(&s, &mut alone, &sim);
            for (at, (o, t)) in ours.iter().zip(&theirs).enumerate() {
                let pins = at > 0 && matches!(s.cmds[at - 1], Cmd::Pins(..));
                // `RuleId::TdiLevel` and `RuleId::SwdioLevel`: the block restores SWDIO's direction.
                let mask = |r: &[u8]| {
                    let mut r = r.to_vec();
                    if pins && r.len() == 2 {
                        r[1] &= !0x06;
                    }
                    r
                };
                assert_eq!(
                    mask(&o.response),
                    mask(&t.response),
                    "seed {seed} step {at}\n{s}"
                );
                assert_eq!(o.effects, t.effects, "seed {seed} step {at}\n{s}");
                // A run of AP reads posts once here, so idle cells compare where the packets do.
                if o.rates.len() == t.rates.len() {
                    assert_eq!(o.idle, t.idle, "seed {seed} step {at}\n{s}");
                    idled += (o.idle > 0) as u32;
                }
            }
        }
        assert!(poisoned > 0 && idled > 0);
    }

    /// How the policy judges `cmds` on a fresh `target`.
    fn verdict_on(target: Target, cmds: Vec<Cmd>) -> Result<Report, Box<Divergence>> {
        let c = Check::new(Profile::SWD, 0..0, &POLICY);
        verdict(&c, &Stream { target, cmds }, composed)
    }

    /// [`verdict_on`], which must not diverge.
    fn judged(target: Target, cmds: Vec<Cmd>) -> Report {
        verdict_on(target, cmds).unwrap_or_else(|d| panic!("{d}"))
    }

    /// The reference's and the composition's steps on `cmds` from a fresh `target`.
    fn both(target: Target, cmds: &[Cmd]) -> (Vec<Step>, Vec<Step>) {
        let stream = Stream {
            target,
            cmds: cmds.to_vec(),
        };
        steps(&stream, composed)
    }

    #[test]
    fn known_divergence_a_write_no_ack_status() {
        for (cmd, ours, theirs) in [
            (
                Cmd::Transfer(0, vec![write(0x05, 0)]),
                vec![0x05, 1, 0x07],
                vec![0x05, 1, 0x0F],
            ),
            (
                Cmd::Block(0, 1, 0x05, vec![0]),
                vec![0x06, 0, 0, 0x07],
                vec![0x06, 1, 0, 0x0F],
            ),
        ] {
            // No line reset: the target is not selected and nothing drives an ACK.
            let cmds = vec![Cmd::Connect(1), cmd];
            let (t, o) = both(Target::default(), &cmds);
            assert_eq!((&o[2].response, &t[2].response), (&ours, &theirs));
            let r = judged(Target::default(), cmds);
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
        let cases: [(Cmd, &[u8], &[u8]); 2] = [
            (Cmd::Transfer(0, vec![read]), &[5, 1, 0x0F], &[5, 1, 0x0F]),
            (
                Cmd::Block(0, 1, 0x02, vec![]),
                &[6, 0, 0, 0x0F],
                &[6, 1, 0, 0x0F],
            ),
        ];
        for (cmd, ours, theirs) in cases {
            let cmds = vec![Cmd::Connect(1), swd_select(), cmd, Cmd::Abort(0, 0x1E)];
            // The read fails alike, a block counted only by dap-rs; dap-rs misframes the abort.
            let (t, o) = both(target.clone(), &cmds);
            assert_eq!((&o[3].response[..], &t[3].response[..]), (ours, theirs));
            assert_eq!(
                (&o[4].response[..], &o[4].effects[..]),
                (&[0x08, 0][..], &[Effect::Abort(0x1E)][..])
            );
            assert_eq!(
                (&t[4].response[..], &t[4].effects[..]),
                (&[0x08, 0xFF][..], &[][..])
            );
            // Under the policy the comparison stops, explained, at the read.
            let r = judged(target.clone(), cmds);
            assert_eq!((r.explained, r.unchecked), (1, 1));
        }
    }

    /// A block counts what completed, as Arm does; dap-rs counts the transfer that failed.
    #[test]
    fn known_divergence_the_count_after_a_failure() {
        // The block's responses and dap-rs a867ca7's; a DAP_Transfer still counts as dap-rs does.
        let want: [(&[u8], &[u8]); 5] = [
            (&[5, 1, 4], &[5, 1, 4]),
            (&[5, 1, 1], &[5, 1, 0]),
            (&[6, 0, 0, 0], &[6, 1, 0, 4]),
            (&[6, 0, 0, 4], &[6, 1, 0, 4]),
            (&[6, 0, 0, 4], &[6, 1, 0, 4]),
        ];
        let stream = dap_differential::session::sticky_fault();
        let (t, o) = both(stream.target.clone(), &stream.cmds);
        for (at, (ours, theirs)) in want.into_iter().enumerate() {
            let (o, t) = (&o[4 + at].response[..], &t[4 + at].response[..]);
            assert_eq!((o, t), (ours, theirs), "step {}", 4 + at);
        }
        assert!(o[6].rates.is_empty());
        let r = judged(stream.target, stream.cmds);
        assert!(r.fired.contains(&RuleId::CountCompleted));
    }

    fn write(req: u8, word: u32) -> Item {
        Item {
            req,
            word: Some(word),
        }
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
                    swd_select(),
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
            let r = judged(stream.target, stream.cmds);
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
                    swd_select(),
                    Cmd::Configure(3, retries, 0),
                    cmd,
                ],
            };
            let (t, o) = steps(&stream, composed);
            assert_eq!((t[4].idle, o[4].idle), (0, cells), "{stream}");
        }
    }

    /// nRESET stays asserted across SWD entry, for connect-under-reset.
    #[test]
    fn known_divergence_nreset_held_across_swd_entry() {
        let cmds = vec![
            Cmd::Connect(1),
            Cmd::Pins(0, 0x80, 0),
            Cmd::Block(0, 1, 0x0D, vec![1]),
        ];
        let (t, o) = both(Target::default(), &cmds);
        assert_eq!((&o[3].effects[..], o[3].levels[4]), (&[][..], false));
        assert_eq!(
            (&t[3].effects[..], t[3].levels[4]),
            (&[Effect::Reset(true)][..], true)
        );
        // The reference cannot be normalized to `RuleId::NresetHeld`; streams release every pulse.
        let d = verdict_on(Target::default(), cmds).unwrap_err();
        assert_eq!((d.kind, d.at), (Kind::Effects, 3));
    }

    #[test]
    fn disconnect_releases_a_held_nreset() {
        let cmds = vec![Cmd::Connect(1), Cmd::Pins(0, 0x80, 0), Cmd::Disconnect];
        judged(Target::default(), cmds);
    }

    #[test]
    fn an_intercept_leaves_dap_rs_connected() {
        let cmds = vec![
            Cmd::Connect(0),
            swd_select(),
            Cmd::Transfer(0, vec![write(0x01, 0)]),
            Cmd::JtagConfigure(vec![4]),
        ];
        judged(Target::default(), cmds);
    }

    #[test]
    fn an_intercept_leaves_tdi_released() {
        let cmds = vec![
            Cmd::Connect(0),
            swd_select(),
            Cmd::Transfer(0, vec![write(0x01, 0)]),
            Cmd::Pins(0x80, 0x80, 0),
        ];
        judged(Target::default(), cmds);
    }

    #[test]
    fn a_jtag_command_is_observed() {
        let cmds = vec![
            Cmd::Connect(0),
            Cmd::Idcode(0),
            Cmd::Block(0, 1, 0x0F, vec![]),
        ];
        judged(Target::default(), cmds);
    }

    /// `RuleId::SwdioLevel`, left open: SWDIO reads back high after a block; dap-rs leaves it low.
    #[test]
    fn known_divergence_the_block_restores_the_swdio_direction() {
        let cmds = vec![
            Cmd::Connect(0),
            swd_select(),
            Cmd::Transfer(
                0,
                vec![
                    write(0x20, 0),
                    Item {
                        req: 0x03,
                        word: None,
                    },
                ],
            ),
            Cmd::Transfer(0, vec![write(0x05, 0x2000_0040)]),
            Cmd::Pins(0x80, 0x80, 0),
        ];
        let (t, o) = both(Target::default(), &cmds);
        assert_eq!(
            (&o[5].response[..], o[5].levels[1]),
            (&[0x10, 0xAF][..], true)
        );
        assert_eq!(
            (&t[5].response[..], t[5].levels[1]),
            (&[0x10, 0xAD][..], false)
        );
        judged(Target::default(), cmds);
    }

    /// nRESET stays asserted across JTAG entry, for connect-under-reset.
    #[test]
    fn known_divergence_nreset_held_across_late_jtag_entry() {
        let cmds = vec![
            Cmd::Pins(0, 0x80, 0),
            Cmd::JtagSequence(vec![(0x46, vec![0x3F])]),
        ];
        let (t, o) = both(Target::default(), &cmds);
        assert_eq!((&o[2].effects[..], o[2].levels[4]), (&[][..], false));
        assert_eq!(
            (&t[2].effects[..], t[2].levels[4]),
            (&[Effect::Reset(true)][..], true)
        );
        let d = verdict_on(Target::default(), cmds).unwrap_err();
        assert_eq!((d.kind, d.at), (Kind::Effects, 2));
    }

    /// `RuleId::SwdioLevel`, left open: TMS rests low after late JTAG; dap-rs leaves it high.
    #[test]
    fn known_divergence_late_jtag_exit_keeps_tms_low() {
        let cmds = vec![
            Cmd::Connect(2),
            Cmd::JtagConfigure(vec![4]),
            Cmd::Transfer(0, vec![write(0x01, 0)]),
            Cmd::Pins(0x80, 0x80, 0),
        ];
        let (t, o) = both(Target::default(), &cmds);
        assert_eq!(
            (&o[4].response[..], o[4].levels[1]),
            (&[0x10, 0xAD][..], false)
        );
        assert_eq!(
            (&t[4].response[..], t[4].levels[1]),
            (&[0x10, 0xAF][..], true)
        );
        judged(Target::default(), cmds);
    }

    /// `RuleId::TdiLevel`, left open: TDI rests high between scans; dap-rs leaves it low.
    #[test]
    fn known_divergence_late_jtag_holds_tdi_high_between_scans() {
        let target = Target {
            chain: vec![5, 4, 7],
            adi: 1,
            ..Target::default()
        };
        let cmds = vec![
            Cmd::JtagConfigure(vec![5, 4, 7]),
            Cmd::Idcode(2),
            jtag_select(),
            Cmd::Pins(0x80, 0x80, 0),
        ];
        let (t, o) = both(target.clone(), &cmds);
        assert_eq!(
            (&o[4].response[..], o[4].levels[2]),
            (&[0x10, 0xAF][..], true)
        );
        assert_eq!(
            (&t[4].response[..], t[4].levels[2]),
            (&[0x10, 0xAB][..], false)
        );
        judged(target, cmds);
    }
}
