use super::*;
use crate::adi::jtag_dp::{decode_ack, Unknown};
use crate::corpus::wire_examples::{trace, Cells, Example, Expect, EXAMPLES};
use crate::wire::swd::{Transfer, ACK_OK, ACK_WAIT};
use crate::wire::Class;

fn bits(v: u64, n: usize) -> Vec<bool> {
    (0..n).map(|k| v >> k & 1 != 0).collect()
}

fn expected(e: &[Expect]) -> Vec<Transaction> {
    e.iter()
        .map(|x| match *x {
            Expect::Swd(t) => Transaction::Swd(t),
            Expect::Targetsel(v, ok) => Transaction::Targetsel(v, ok),
            Expect::Scan(ir, n, tdi, tdo) => Transaction::Scan(Scan {
                ir,
                tdi: bits(tdi, n),
                tdo: bits(tdo, n),
            }),
            Expect::Xpacc(x) => Transaction::Xpacc(x),
        })
        .collect()
}

/// The transactions, an xPACC's request zeroed where the scan was too short to carry it.
fn seen(o: &Observed) -> Vec<Transaction> {
    let mut out = o.transactions.clone();
    for u in &o.unobserved {
        if let (Why::Truncated, Some(k)) = (u.why, u.transaction) {
            if let Transaction::Xpacc(x) = &mut out[k] {
                (x.read, x.a, x.data) = (false, 0, 0);
            }
        }
    }
    out
}

fn run(e: &Example, framing: Framing) -> Observed {
    observe(&e.trace(), &e.config(framing))
}

#[test]
fn every_worked_example_is_observed_in_both_framings() {
    for e in EXAMPLES {
        for framing in [Framing::Strict, Framing::Resync] {
            let o = run(e, framing);
            assert_eq!(seen(&o), expected(e.expect), "{} {framing:?}", e.name);
            assert_eq!(o.roles.len(), e.trace().len());
            for u in &o.unobserved {
                let fine = match u.why {
                    Why::Unsynced => framing == Framing::Resync,
                    Why::Truncated => u.transaction.is_some(),
                    _ => false,
                };
                assert!(fine, "{} {framing:?}: {u:?}", e.name);
            }
        }
    }
}

#[test]
fn a_tap_walks_from_shift_dr_to_idle_through_reset() {
    let e = EXAMPLES
        .iter()
        .find(|e| e.name.starts_with("Shift-DR"))
        .unwrap();
    let o = run(e, Framing::Strict);
    let tail: Vec<Phase> = o.roles[o.roles.len() - 7..]
        .iter()
        .map(|r| r.unwrap().phase)
        .collect();
    let want = [
        TapState::ShiftDr,
        TapState::Exit1Dr,
        TapState::UpdateDr,
        TapState::SelectDr,
        TapState::SelectIr,
        TapState::Reset,
        TapState::Idle,
    ];
    assert_eq!(tail, want.map(Phase::Tap));
}

#[test]
fn a_read_has_the_target_drive_its_ack_data_and_parity() {
    let o = run(&EXAMPLES[0], Framing::Strict);
    let roles: Vec<CellRole> = o.roles[54..54 + 48].iter().map(|r| r.unwrap()).collect();
    let phases: Vec<Phase> = roles.iter().map(|r| r.phase).collect();
    let mut want = vec![Phase::Request; 8];
    want.push(Phase::Turnaround);
    want.extend([Phase::Ack; 3]);
    want.extend([Phase::ReadData; 32]);
    want.extend([
        Phase::ReadParity,
        Phase::Turnaround,
        Phase::Idle,
        Phase::Idle,
    ]);
    assert_eq!(phases, want);
    let target = roles.iter().filter(|r| r.driver == Drive::Target).count();
    assert_eq!(target, 3 + 32 + 1);
}

fn swd(cells: &[Cells], framing: Framing) -> Observed {
    let cfg = Config {
        framing,
        protocol: Protocol::Zero,
        ..Config::default()
    };
    observe(&trace(cells), &cfg)
}

fn swd_packets(o: &Observed) -> Vec<Transfer> {
    o.transactions
        .iter()
        .filter_map(|t| match t {
            Transaction::Swd(t) => Some(*t),
            _ => None,
        })
        .collect()
}

const LINE_RESET: [Cells; 2] = [Cells::Host(u64::MAX, 52), Cells::Host(0, 2)];
const DPIDR: u64 = 0x2BA0_1477;

fn dpidr_read(ack: u64, data: u64, parity: u64) -> [Cells; 6] {
    [
        Cells::Host(0xA5, 8),
        Cells::Released(1),
        Cells::Target(ack, 3),
        Cells::Target(data, 32),
        Cells::Target(parity, 1),
        Cells::Released(1),
    ]
}

#[test]
fn a_bad_ack_is_reported_raw_without_a_data_phase() {
    for framing in [Framing::Strict, Framing::Resync] {
        let mut cells = LINE_RESET.to_vec();
        cells.extend([
            Cells::Host(0xA5, 8),
            Cells::Released(1),
            Cells::Target(0b011, 3),
            Cells::Released(1),
            Cells::Host(0, 2),
        ]);
        let p = swd_packets(&swd(&cells, framing));
        assert_eq!(
            (p.len(), p[0].ack, p[0].data),
            (1, 0b011, None),
            "{framing:?}"
        );
    }
}

#[test]
fn a_flipped_read_bit_fails_parity() {
    let mut cells = LINE_RESET.to_vec();
    cells.extend(dpidr_read(1, DPIDR ^ 1 << 9, 0));
    cells.push(Cells::Host(0, 2));
    let p = swd_packets(&swd(&cells, Framing::Strict));
    assert_eq!(p[0].data, Some(DPIDR as u32 ^ 1 << 9));
    assert!(!p[0].parity_ok);
}

#[test]
fn a_missing_trailing_turnaround_misframes_strict_and_resync_recovers_it() {
    let mut cells = LINE_RESET.to_vec();
    let mut read = dpidr_read(1, DPIDR, 0);
    cells.extend(&read[..5]);
    read[5] = Cells::Host(0, 2);
    cells.extend(read);
    let strict = swd_packets(&swd(&cells, Framing::Strict));
    assert_eq!(strict.len(), 1);
    let o = swd(&cells, Framing::Resync);
    let p = swd_packets(&o);
    assert_eq!(p.len(), 2);
    assert_eq!((p[1].ack, p[1].data), (ACK_OK, Some(DPIDR as u32)));
    let turnaround = 54 + 45;
    assert_eq!(o.roles[turnaround].map(|r| r.phase), Some(Phase::Request));
}

fn orun_wait_read(set: bool) -> Vec<Cells> {
    let mut cells = LINE_RESET.to_vec();
    cells.extend([
        Cells::Host(0xA9, 8),
        Cells::Released(1),
        Cells::Target(1, 3),
        Cells::Released(1),
        Cells::Host(u64::from(set), 32),
        Cells::Host(u64::from(set), 1),
        Cells::Host(0, 2),
        Cells::Host(0x9F, 8),
        Cells::Released(1),
        Cells::Target(ACK_WAIT as u64, 3),
        Cells::Released(33 + 1),
    ]);
    cells.extend(dpidr_read(1, DPIDR, 0));
    cells.push(Cells::Host(0, 2));
    cells
}

#[test]
fn orundetect_from_a_ctrl_stat_write_frames_the_undriven_data_phase() {
    let p = swd_packets(&swd(&orun_wait_read(true), Framing::Strict));
    assert_eq!(p.len(), 3);
    assert_eq!((p[1].ack, p[1].data), (ACK_WAIT, None));
    assert_eq!(p[2].data, Some(DPIDR as u32));
}

#[test]
fn a_wait_data_phase_without_orundetect_misframes() {
    let p = swd_packets(&swd(&orun_wait_read(false), Framing::Strict));
    assert!(p.iter().all(|t| t.data != Some(DPIDR as u32)), "{p:?}");
    let cfg = Config {
        orundetect: false,
        framing: Framing::Strict,
        ..Config::default()
    };
    let o = observe(&trace(&orun_wait_read(true)), &cfg);
    assert!(swd_packets(&o).iter().all(|t| t.data != Some(DPIDR as u32)));
}

fn targetsel() -> Vec<Cells> {
    let mut cells = LINE_RESET.to_vec();
    cells.extend([
        Cells::Host(0x99, 8),
        Cells::Released(5),
        Cells::Host(0x0100_2927, 32),
        Cells::Host(0, 1),
        Cells::Host(0, 2),
    ]);
    cells.extend(dpidr_read(1, DPIDR, 0));
    cells.push(Cells::Host(0, 2));
    cells
}

#[test]
fn targetsel_has_no_ack_and_is_reported_on_its_own() {
    let cfg = Config {
        link: LinkConfig { multidrop: true },
        framing: Framing::Strict,
        ..Config::default()
    };
    let o = observe(&trace(&targetsel()), &cfg);
    assert_eq!(o.transactions[0], Transaction::Targetsel(0x0100_2927, true));
    let released = o.roles[62..67]
        .iter()
        .all(|r| r.unwrap().driver == Drive::Released);
    assert!(released);
    assert_eq!(swd_packets(&o)[0].data, Some(DPIDR as u32));
}

#[test]
fn targetsel_without_multidrop_misframes() {
    let o = swd(&targetsel(), Framing::Strict);
    assert!(swd_packets(&o).iter().all(|t| t.data != Some(DPIDR as u32)));
}

/// TAP reset, the IR loaded with `ir`, Run-Test/Idle.
fn jtag_start(ir: u64) -> Vec<Cells> {
    vec![
        Cells::Tms(0b11111, 5),
        Cells::Tms(0, 1),
        Cells::Tms(0b0011, 4),
        Cells::Shift(ir, 1, 4),
        Cells::Tms(0b01, 2),
    ]
}

fn dr(tdi: u64, tdo: u64, n: u8) -> [Cells; 3] {
    [
        Cells::Tms(0b001, 3),
        Cells::Shift(tdi, tdo, n),
        Cells::Tms(0b01, 2),
    ]
}

fn jtag(cells: &[Cells], taps: Vec<TapConfig>, framing: Framing) -> Observed {
    let cfg = Config {
        framing,
        mode: Mode::Jtag,
        taps,
        protocol: Protocol::Zero,
        ..Config::default()
    };
    observe(&trace(cells), &cfg)
}

fn xpaccs(o: &Observed) -> Vec<Xpacc> {
    o.transactions
        .iter()
        .filter_map(|t| match t {
            Transaction::Xpacc(x) => Some(*x),
            _ => None,
        })
        .collect()
}

fn last_scan_len(o: &Observed) -> usize {
    match o.transactions.last() {
        Some(Transaction::Scan(s)) => s.tdi.len(),
        t => panic!("{t:?}"),
    }
}

#[test]
fn test_logic_reset_puts_idcode_back_in_the_ir() {
    let mut cells = jtag_start(0b1010);
    cells.extend(dr(0b011, 0b010, 35));
    cells.extend([Cells::Tms(0b111, 3), Cells::Tms(0, 1)]);
    cells.extend(dr(0, 0x4BA0_0477, 32));
    let o = jtag(&cells, vec![TapConfig::dp(4)], Framing::Strict);
    assert_eq!(xpaccs(&o).len(), 1);
    assert_eq!(last_scan_len(&o), 32);
}

/// A DP behind a 5-bit TAP holding `other_ir`, read through a 42-cell scan.
fn behind(other_ir: u64) -> Vec<Cells> {
    let mut cells = vec![
        Cells::Tms(0b11111, 5),
        Cells::Tms(0, 1),
        Cells::Tms(0b0011, 4),
        Cells::Shift(other_ir << 4 | 0b1010, 1 << 4 | 1, 9),
        Cells::Tms(0b01, 2),
    ];
    cells.extend(dr(0b111, 0b010, 35 + 7));
    cells
}

#[test]
fn a_tap_out_of_bypass_hides_the_access_until_its_length_is_known() {
    let taps = |other| {
        vec![
            TapConfig::dp(4),
            TapConfig {
                ir_len: 5,
                idcode: None,
                other,
            },
        ]
    };
    let o = jtag(&behind(0b00010), taps(None), Framing::Strict);
    assert!(xpaccs(&o).is_empty());
    assert!(o.unobserved.iter().any(|u| u.why == Why::Length));
    let o = jtag(&behind(0b00010), taps(Some(7)), Framing::Strict);
    let x = xpaccs(&o);
    assert_eq!((x.len(), x[0].read, x[0].a), (1, true, 3));
}

/// JTAG traffic, a switch to SWD with `select`, a DPIDR read, a switch back, then a DR scan.
fn round_trip_through_swd(select: u64) -> Vec<Cells> {
    let mut cells = jtag_start(0b1010);
    cells.extend(dr(0b011, 0b010, 35));
    cells.extend([
        Cells::Host(u64::MAX, 50),
        Cells::Host(select, 16),
        Cells::Host(u64::MAX, 50),
        Cells::Host(0, 2),
    ]);
    cells.extend(dpidr_read(1, DPIDR, 0));
    cells.extend([
        Cells::Host(0, 2),
        Cells::Host(u64::MAX, 50),
        Cells::Host(0xE73C, 16),
        Cells::Host(u64::MAX, 5),
        Cells::Tms(0, 1),
    ]);
    cells.extend(dr(0, 0x4BA0_0477, 32));
    cells
}

#[test]
fn a_switch_to_swd_stops_the_tap_and_a_switch_back_resets_it() {
    for framing in [Framing::Strict, Framing::Resync] {
        let o = jtag(
            &round_trip_through_swd(0xE79E),
            vec![TapConfig::dp(4)],
            framing,
        );
        assert_eq!(swd_packets(&o).len(), 1, "{framing:?}");
        assert_eq!(xpaccs(&o).len(), 1, "{framing:?}");
        assert_eq!(last_scan_len(&o), 32, "{framing:?}");
    }
}

#[test]
fn a_corrupt_select_sequence_leaves_the_tap_clocking() {
    let o = jtag(
        &round_trip_through_swd(0xE79F),
        vec![TapConfig::dp(4)],
        Framing::Strict,
    );
    assert!(swd_packets(&o).is_empty());
    assert!(o
        .roles
        .iter()
        .all(|r| matches!(r.unwrap().phase, Phase::Tap(_))));
}

#[test]
fn a_scan_resumed_through_pause_is_one_access() {
    let mut cells = jtag_start(0b1010);
    cells.extend([
        Cells::Tms(0b001, 3),
        Cells::Tms(0, 9),
        Cells::Tms(0b0101, 4),
        Cells::Tms(0, 24),
        Cells::Tms(0b1, 1),
        Cells::Tms(0b01, 2),
    ]);
    let o = jtag(&cells, vec![TapConfig::dp(4)], Framing::Strict);
    assert_eq!(xpaccs(&o).len(), 1, "{o:?}");
    let Transaction::Scan(s) = &o.transactions[1] else {
        panic!("{o:?}");
    };
    assert_eq!(s.tdi.len(), 35);
}

#[test]
fn a_capture_that_starts_mid_packet_resynchronizes_on_headers() {
    let mut cells = vec![
        Cells::Target(DPIDR >> 7, 25),
        Cells::Target(0, 1),
        Cells::Released(1),
    ];
    for _ in 0..3 {
        cells.extend(dpidr_read(1, DPIDR, 0));
        cells.push(Cells::Host(0, 2));
    }
    assert!(swd_packets(&swd(&cells, Framing::Strict)).is_empty());
    let o = swd(&cells, Framing::Resync);
    assert_eq!(swd_packets(&o).len(), 3);
    assert_eq!(o.roles[27].map(|r| r.phase), Some(Phase::Request));
    assert!(o.roles[..27].iter().all(Option::is_none));
    assert_eq!(o.unobserved[0].cells, 0..27);
}

#[test]
fn a_tap_resynchronizes_on_five_tms_ones() {
    let mut cells = vec![Cells::Tms(0b0110, 4)];
    cells.extend(jtag_start(0b1010));
    cells.extend(dr(0b011, 0b010, 35));
    let o = jtag(&cells, vec![TapConfig::dp(4)], Framing::Resync);
    assert_eq!(xpaccs(&o).len(), 1);
    assert!(o.roles[..4].iter().all(Option::is_none));
    let first = o.roles.iter().position(Option::is_some).unwrap();
    assert!(first <= 4 + 5);
}

#[test]
fn a_glitch_is_placed_before_its_cell_and_is_not_a_cell() {
    let mut cells = LINE_RESET.to_vec();
    cells.extend(dpidr_read(1, DPIDR, 0));
    let mut t = trace(&cells);
    let clean = observe(&t, &Config::default());
    for (at, phase) in [
        (60, Phase::Request),
        (62, Phase::Turnaround),
        (70, Phase::ReadData),
    ] {
        let mut glitched = t.clone();
        glitched.insert(at, TraceEvent::Glitch { stamp: 1000 });
        let o = observe(&glitched, &Config::default());
        assert_eq!(o.transactions, clean.transactions);
        let g = o.glitches[0];
        assert_eq!((g.cell, g.role.map(|r| r.phase)), (at, Some(phase)));
    }
    t.insert(70, TraceEvent::Gap { stamp: 1000 });
    assert_eq!(observe(&t, &Config::default()).breaks, [70]);
}

#[test]
fn an_unknown_protocol_is_learned_from_a_protocol_1_ok() {
    let mut cells = jtag_start(0b1010);
    cells.extend(dr(0b011, 0b010, 35));
    cells.extend(dr(0b111, 0b100, 35));
    cells.extend(dr(0b111, 0b010, 35));
    let cfg = Config {
        mode: Mode::Jtag,
        framing: Framing::Strict,
        ..Config::default()
    };
    let o = observe(&trace(&cells), &cfg);
    let x = xpaccs(&o);
    let acks: Vec<JtagAck> = x.iter().map(|x| x.ack).collect();
    assert_eq!(acks, [JtagAck::OkFault, JtagAck::Ok, JtagAck::Fault]);
    assert_eq!(o.protocol, Protocol::One);
    let unsure = o.unobserved.iter().filter(|u| u.why == Why::Result).count();
    assert_eq!(unsure, 1, "only the first OK/FAULT could have been a FAULT");
    assert_eq!(x[2].result, ReadResult::Unknown(Unknown::Fault));
}

#[test]
fn decode_ack_reads_each_protocol() {
    let row = |p| (0..8).map(|b| decode_ack(b, p)).collect::<Vec<_>>();
    let (inv, w) = (JtagAck::Invalid, JtagAck::Wait);
    let (of, ok, f) = (JtagAck::OkFault, JtagAck::Ok, JtagAck::Fault);
    let tail = [inv(5), inv(6), inv(7)];
    let zero = [inv(0), w, of, inv(3), inv(4)];
    assert_eq!(row(Protocol::Zero), [&zero[..], &tail].concat());
    let one = [inv(0), w, f, inv(3), ok];
    assert_eq!(row(Protocol::One), [&one[..], &tail].concat());
    let unknown = [inv(0), w, of, inv(3), ok];
    assert_eq!(row(Protocol::Unknown), [&unknown[..], &tail].concat());
    assert_eq!(Protocol::Unknown.learn(0b100), Protocol::One);
    assert_eq!(Protocol::Zero.learn(0b100), Protocol::Zero);
}

fn phases(o: &Observed, cells: Range<usize>) -> Vec<Option<Phase>> {
    o.roles[cells].iter().map(|r| r.map(|r| r.phase)).collect()
}

#[test]
fn resync_frames_no_header_that_a_line_reset_completes() {
    let mut cells = LINE_RESET.to_vec();
    cells.extend(dpidr_read(1, DPIDR, 0));
    let o = swd(&cells, Framing::Resync);
    assert!(
        o.roles[..49].iter().all(Option::is_none),
        "{:?}",
        phases(&o, 40..50)
    );
    assert_eq!(o.roles[49].map(|r| r.phase), Some(Phase::LineReset));
    assert_eq!(swd_packets(&o).len(), 1);
}

#[test]
fn resync_frames_no_header_in_a_line_resets_ones() {
    let cells = [
        Cells::Host(u64::MAX, 54),
        Cells::Host(0, 1),
        Cells::Host(0xC5, 8),
        Cells::Host(0, 4),
    ];
    let o = swd(&cells, Framing::Resync);
    assert!(swd_packets(&o).is_empty(), "{:?}", o.transactions);
    let mut want = vec![Some(Phase::LineReset); 5];
    want.push(Some(Phase::Idle));
    assert_eq!(phases(&o, 49..55), want);
}

#[test]
fn a_malformed_header_leaves_the_cells_before_it_their_roles() {
    let mut cells = LINE_RESET.to_vec();
    cells.extend(dpidr_read(1, DPIDR, 0));
    cells.extend([Cells::Host(0, 2), Cells::Host(0xFF, 8), Cells::Host(0, 2)]);
    let o = swd(&cells, Framing::Resync);
    let turn = 54 + 45;
    let classes: Vec<Class> = o.roles[turn..turn + 3]
        .iter()
        .map(|&r| Class::of(r))
        .collect();
    let want = [Class::TrailingTurnaround, Class::Idle, Class::Idle];
    assert_eq!(classes, want);
}

#[test]
fn classes_follow_each_packet() {
    let mut cells = LINE_RESET.to_vec();
    cells.extend(dpidr_read(1, DPIDR, 0));
    cells.extend([
        Cells::Host(0, 1),
        Cells::Host(0x9F, 8),
        Cells::Released(1),
        Cells::Target(ACK_WAIT as u64, 3),
        Cells::Released(1),
        Cells::Host(0, 1),
    ]);
    let o = swd(&cells, Framing::Strict);
    let mut want = vec![Class::Resident; 54];
    want.extend([Class::Request; 8]);
    want.push(Class::Turnaround);
    want.extend([Class::Ack; 3]);
    want.extend([Class::ReadData; 32]);
    want.extend([Class::ReadParity, Class::TrailingTurnaround, Class::Idle]);
    want.extend([Class::Request; 8]);
    want.push(Class::Turnaround);
    want.extend([Class::Ack; 3]);
    want.extend([Class::WaitTurnaround, Class::Idle]);
    let classes: Vec<Class> = o.roles.iter().map(|&r| Class::of(r)).collect();
    assert_eq!(classes, want);
}
