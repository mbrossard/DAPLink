use std::collections::BTreeMap;
use std::ops::Range;

use super::*;
use crate::adi::jtag_dp::ReadResult;
use crate::differential::{dap_rs_only, reference, run, Probe};
use crate::session::{stream, Profile};
use crate::target::{shared, Log, Target};

/// The observer told what the simulated target is, from power-up.
fn config(t: &Target) -> Config {
    let lens: Vec<u8> = if t.absent {
        Vec::new()
    } else {
        t.chain.iter().map(|&l| l.max(1)).collect()
    };
    let taps = lens
        .iter()
        .enumerate()
        .map(|(i, &ir_len)| TapConfig {
            ir_len,
            idcode: Some(if i == t.adi {
                jtag_dp::idcode_ir(ir_len)
            } else {
                1
            }),
            other: Some(21),
        })
        .collect();
    Config {
        framing: Framing::Strict,
        mode: t.mode,
        taps,
        dp: t.adi,
        protocol: if t.adiv6 {
            Protocol::One
        } else {
            Protocol::Zero
        },
        ..Config::default()
    }
}

#[derive(Clone, Copy)]
enum Host {
    Reference,
    DapRs,
}

fn recorded(seed: u64, profile: &Profile, host: Host) -> (Target, Log, Vec<TraceEvent>) {
    let s = stream(seed, profile);
    let sim = shared(&s.target);
    sim.borrow_mut().record(true);
    let mut probe: Box<dyn Probe> = match host {
        Host::Reference => Box::new(reference(&sim)),
        Host::DapRs => Box::new(dap_rs_only(&sim)),
    };
    run(&s, probe.as_mut(), &sim);
    let sim = sim.borrow();
    (s.target, sim.log().clone(), sim.trace())
}

/// `t` with what `why` says the observer could not see blanked.
fn blank(t: &Transaction, why: &[Why]) -> Transaction {
    let mut t = t.clone();
    if let Transaction::Xpacc(x) = &mut t {
        if why.contains(&Why::Truncated) {
            (x.read, x.a, x.data) = (false, 0, 0);
        }
        if why.contains(&Why::Truncated) && matches!(x.result, ReadResult::Valid(_))
            || why.contains(&Why::Result)
        {
            x.result = ReadResult::Valid(0);
        }
    }
    t
}

/// What the target logged that the observer did not, or saw differently, unexplained.
fn unexplained(log: &Log, o: &Observed) -> Option<String> {
    if let Some(i) = (0..log.cells.len()).find(|&i| o.roles[i] != Some(log.cells[i].role)) {
        return Some(format!(
            "cell {i}: {:?} observed {:?}",
            log.cells[i].role, o.roles[i]
        ));
    }
    let mut why: Vec<Vec<Why>> = vec![Vec::new(); o.transactions.len()];
    let mut missing = Vec::new();
    for u in &o.unobserved {
        match u.transaction {
            Some(k) => why[k].push(u.why),
            None if matches!(u.why, Why::ShortScan | Why::Length | Why::Instruction) => {
                missing.push(u.cells.end - 1)
            }
            None => {}
        }
    }
    let mut theirs = log.transactions.iter().peekable();
    for (k, t) in o.transactions.iter().enumerate() {
        let skip = matches!(t, Transaction::Scan(_)) && missing.contains(&o.at[k]);
        let Some(want) = theirs.next() else {
            return Some(format!("transaction {k}: {t:?} observed only"));
        };
        if blank(want, &why[k]) != blank(t, &why[k]) {
            return Some(format!("transaction {k}: {t:?} observed, {want:?} logged"));
        }
        if skip && matches!(theirs.peek(), Some(Transaction::Xpacc(_))) {
            theirs.next();
        }
    }
    theirs.next().map(|t| format!("{t:?} logged only"))
}

/// Strict observation of every recorded stream equals the target's logs but where it says not.
fn round_trip(profile: Profile, seeds: Range<u64>, host: Host) -> BTreeMap<Why, usize> {
    let mut seen = BTreeMap::new();
    for seed in seeds {
        let (target, log, trace) = recorded(seed, &profile, host);
        let o = observe(&trace, &config(&target));
        if let Some(d) = unexplained(&log, &o) {
            panic!("seed {seed}: {d}");
        }
        for u in &o.unobserved {
            *seen.entry(u.why).or_insert(0) += 1;
        }
    }
    seen
}

#[test]
fn the_observer_frames_swd_as_the_target_does() {
    let seen = round_trip(Profile::SWD, 0..450, Host::Reference);
    assert_eq!(seen.keys().collect::<Vec<_>>(), [&Why::Truncated]);
}

#[test]
fn the_observer_frames_swd_with_bad_parity_as_the_target_does() {
    let parity = Profile {
        parity: true,
        ..Profile::SWD
    };
    round_trip(parity, 300..450, Host::Reference);
}

#[test]
fn the_observer_frames_jtag_as_the_target_does() {
    round_trip(Profile::JTAG, 0..400, Host::Reference);
    round_trip(Profile::JTAG, 0..400, Host::DapRs);
}

#[test]
fn the_observer_frames_adiv6_jtag_as_the_target_does() {
    round_trip(Profile::JTAG_ADIV6, 0..350, Host::Reference);
    round_trip(Profile::JTAG_ADIV6, 0..350, Host::DapRs);
}

#[test]
fn the_observer_frames_an_empty_chain_as_the_target_does() {
    let seen = round_trip(Profile::JTAG_ABSENT, 0..150, Host::Reference);
    assert!(seen.is_empty(), "{seen:?}");
}

#[test]
fn the_observer_frames_chaos_as_the_target_does() {
    round_trip(Profile::CHAOS, 0..300, Host::Reference);
}
