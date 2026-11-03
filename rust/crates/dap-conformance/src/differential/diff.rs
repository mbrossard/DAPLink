//! Run a stream through two engines, compare them under a policy, shrink a divergence.

use std::cell::RefCell;
use std::fmt;
use std::ops::Range;

use super::reference::reference;
use crate::rule::{Basis, Policy, RuleId, RULES};
use crate::session::{stream, Cmd, Profile, Stream};
use crate::target::{shared, Effect, LinkState, Mode, Shared};
use crate::wire::swd;

/// A CMSIS-DAP command processor: the reference, or a probe's path in front of its engine.
pub trait Probe {
    /// Answer one report, returning the response length.
    fn process(&mut self, report: &[u8], response: &mut [u8]) -> usize;
}

impl<F: FnMut(&[u8], &mut [u8]) -> usize> Probe for F {
    fn process(&mut self, report: &[u8], response: &mut [u8]) -> usize {
        self(report, response)
    }
}

/// What one command left behind.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Step {
    /// The response bytes.
    pub response: Vec<u8>,
    /// What the target observed while the command ran.
    pub effects: Vec<Effect>,
    /// What the probe drives on each line afterwards.
    pub drives: [Option<bool>; 5],
    /// The level on each line afterwards.
    pub levels: [bool; 5],
    /// The clock labels of the SWD packets the command sent.
    pub rates: Vec<Option<u32>>,
    /// The SWD idle cells the command clocked; not compared.
    pub idle: u32,
    /// The target's protocol position afterwards.
    pub state: LinkState,
}

std::thread_local! {
    /// Per command of the current run: the rules the path waived on it, as [`RuleId`] bits.
    static WAIVED: RefCell<Vec<u64>> = const { RefCell::new(Vec::new()) };
    /// Per command of the current run: the rules whose case arose in the reference.
    static FIRED: RefCell<Vec<u64>> = const { RefCell::new(Vec::new()) };
}

fn mark(marks: &'static std::thread::LocalKey<RefCell<Vec<u64>>>, rule: RuleId) {
    marks.with(|m| {
        if let Some(last) = m.borrow_mut().last_mut() {
            *last |= rule.bit();
        }
    });
}

/// Exempt the path's current report from `rule`, which its policy waives on marked reports.
pub fn waive(rule: RuleId) {
    mark(&WAIVED, rule);
}

/// The reference met `rule`'s case in the current report.
pub(crate) fn fire(rule: RuleId) {
    mark(&FIRED, rule);
}

/// Run `stream` through `probe` on `sim`; the first step is construction.
pub fn run(stream: &Stream, probe: &mut dyn Probe, sim: &Shared) -> Vec<Step> {
    for marks in [&WAIVED, &FIRED] {
        marks.with(|m| m.borrow_mut().clear());
    }
    let mut steps = vec![collect(sim, Vec::new())];
    for cmd in &stream.cmds {
        for marks in [&WAIVED, &FIRED] {
            marks.with(|m| m.borrow_mut().push(0));
        }
        let mut response = vec![0u8; 512];
        let len = probe.process(&cmd.encode(), &mut response);
        response.truncate(len);
        steps.push(collect(sim, response));
    }
    steps
}

fn collect(sim: &Shared, response: Vec<u8>) -> Step {
    let mut s = sim.borrow_mut();
    Step {
        response,
        effects: std::mem::take(&mut s.effects),
        drives: s.drives(),
        levels: s.levels(),
        rates: std::mem::take(&mut s.rates),
        idle: std::mem::take(&mut s.idle),
        state: s.link_state(),
    }
}

/// Which comparison failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// The response bytes differ.
    Response,
    /// The target observed different things.
    Effects,
    /// The target was left at a different protocol position.
    State,
    /// A pin command left a line at a different level.
    Pads,
    /// The path's own invariant failed.
    Invariant,
    /// The path broke an enforced rule.
    Rule(RuleId),
}

/// A stream on which the path and dap-rs disagree.
#[derive(Clone, Debug)]
pub struct Divergence {
    /// The seed that produced the unshrunk stream.
    pub seed: u64,
    /// The comparison that failed.
    pub kind: Kind,
    /// The command it failed on; 0 is construction.
    pub at: usize,
    /// Both sides of the difference.
    pub detail: String,
    /// The shrunk stream.
    pub stream: Stream,
}

impl fmt::Display for Divergence {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "seed {}: {:?} diverged at step {}",
            self.seed, self.kind, self.at
        )?;
        writeln!(f, "{}", self.detail)?;
        write!(f, "{}", self.stream)
    }
}

/// Why a check failed.
#[derive(Clone, Debug)]
pub enum Failure {
    /// A stream on which the path and the reference disagree.
    Diverged(Divergence),
    /// A waiver in force on the check's profile that matched nothing over its seeds.
    Unfired(RuleId),
}

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Failure::Diverged(d) => d.fmt(f),
            Failure::Unfired(rule) => write!(f, "the waiver of {rule} matched nothing"),
        }
    }
}

/// What a check established.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Report {
    /// Commands run.
    pub commands: usize,
    /// Streams whose comparison stopped at a rule the reference breaks and the path keeps.
    pub explained: usize,
    /// Commands those stops left uncompared.
    pub unchecked: usize,
    /// Rules whose normalization, explained stop or waiver applied somewhere.
    pub fired: Vec<RuleId>,
}

/// An invariant of a path's own run: a description of the first violation, if any.
pub type Invariant = dyn Fn(&Stream, &[Step]) -> Option<String>;

/// A check over one path: the streams it draws and the policy it holds the path to.
pub struct Check<'a> {
    /// The command mix.
    pub profile: Profile,
    /// The seeds to draw streams from.
    pub seeds: Range<u64>,
    /// The rules the path is held to.
    pub policy: &'a Policy,
    /// Opt-in: also compare the target's protocol position, flagging a stray edge where it happens.
    pub state: bool,
    /// An invariant of the path's own run, checked after the comparison.
    pub invariant: Option<&'a Invariant>,
}

impl<'a> Check<'a> {
    /// A check of `profile` over `seeds` under `policy`: responses, effects, pin levels; no state.
    pub fn new(profile: Profile, seeds: Range<u64>, policy: &'a Policy) -> Self {
        Self {
            profile,
            seeds,
            policy,
            state: false,
            invariant: None,
        }
    }

    /// Whether `rule` holds on a report the path marked with `waived`.
    fn enforced(&self, rule: RuleId, waived: u64) -> bool {
        if rule.rule().basis == Basis::Unspecified {
            return false;
        }
        match self.policy.waiver(rule, &self.profile) {
            None => true,
            Some(w) => w.marked && waived & rule.bit() == 0,
        }
    }

    /// The rules a waiver on marked reports lets the path waive, and of those, the ones in force.
    fn marked(&self) -> (u64, u64) {
        let marked = self.policy.waivers.iter().filter(|w| w.marked);
        marked.fold((0, 0), |(any, here), w| {
            let on = if (w.on)(&self.profile) {
                w.rule.bit()
            } else {
                0
            };
            (any | w.rule.bit(), here | on)
        })
    }
}

/// The rules the reference is normalized to where enforced.
const NORMALIZED: [RuleId; 4] = [
    RuleId::WaitAttempts,
    RuleId::ValueMatch,
    RuleId::CountCompleted,
    RuleId::NoAckStatus,
];

/// The rules the target reports a breach of as an effect.
const EFFECT_RULES: [(Effect, RuleId); 2] = [
    (Effect::Protocol, RuleId::SwdHeader),
    (Effect::Contention, RuleId::SwdContention),
];

/// How one stream compared.
enum Verdict {
    Agree,
    Diverged(Kind, usize, String),
    /// Stopped at this step: the reference broke a rule that desynchronizes the wire.
    Explained(usize),
}

struct Compared {
    verdict: Verdict,
    /// Rules whose normalization, stop or waiver applied.
    fired: u64,
    /// Of those, the waivers.
    waived: u64,
}

const PIN_LINES: [(u8, usize); 4] = [(0x01, 0), (0x02, 1), (0x04, 2), (0x80, 4)];

/// `DAP_SWJ_Pins` bits whose readback and level the `Unspecified` rules leave open here.
fn unspecified_pins(t: &Step, o: &Step) -> u8 {
    // TdiLevel, SwdioLevel, and ParkedAfterConnect or ParkedPinSelect when one side parks SWCLK.
    let parked = t.drives[0].is_some() != o.drives[0].is_some();
    0x06 | u8::from(parked)
}

/// The response status byte's index in a transfer or block reply.
fn status_at(cmd: &Cmd) -> Option<usize> {
    match cmd {
        Cmd::Transfer(..) => Some(2),
        Cmd::Block(..) => Some(3),
        _ => None,
    }
}

fn compare(
    check: &Check<'_>,
    stream: &Stream,
    theirs: &[Step],
    ours: &[Step],
    fired: &[u64],
    waived: &[u64],
) -> Compared {
    let (mut all, mut waivers) = (0, 0);
    let mut verdict = Verdict::Agree;
    for (at, (t, o)) in theirs.iter().zip(ours).enumerate() {
        let cmd = at.checked_sub(1).map(|i| &stream.cmds[i]);
        let report = at.checked_sub(1);
        let here = report.map_or(0, |i| fired[i]);
        let waived = report.map_or(0, |i| waived[i]);
        for r in RULES.iter().filter(|r| here & r.id.bit() != 0) {
            all |= r.id.bit();
            if !check.enforced(r.id, waived) {
                waivers |= r.id.bit();
            }
        }
        let more = |fx: &Effect| {
            let n = |s: &Step| s.effects.iter().filter(|e| *e == fx).count();
            n(o) > n(t)
        };
        let breach = EFFECT_RULES
            .iter()
            .find(|(fx, rule)| check.enforced(*rule, waived) && more(fx));
        if let Some((fx, rule)) = breach {
            let detail = format!("path caused {fx:?} where the reference did not");
            verdict = Verdict::Diverged(Kind::Rule(*rule), at, detail);
            break;
        }
        let pins = matches!(cmd, Some(Cmd::Pins(..)));
        let open = unspecified_pins(t, o);
        let (mut tr, mut or) = (t.response.clone(), o.response.clone());
        if pins && tr.len() == 2 && or.len() == 2 {
            tr[1] &= !open;
            or[1] &= !open;
        }
        if let Some(s) = cmd.and_then(status_at) {
            // ProtocolErrorAck: bits 2:0 of a protocol error mean nothing.
            if tr
                .get(s)
                .zip(or.get(s))
                .is_some_and(|(a, b)| a & b & 0x08 != 0)
            {
                tr[s] &= !7;
                or[s] &= !7;
            }
        }
        if tr != or {
            let detail = format!("dap-rs {:02x?}\npath   {:02x?}", t.response, o.response);
            verdict = Verdict::Diverged(Kind::Response, at, detail);
            break;
        }
        if t.effects != o.effects {
            let detail = format!("dap-rs {:?}\npath   {:?}", t.effects, o.effects);
            verdict = Verdict::Diverged(Kind::Effects, at, detail);
            break;
        }
        let turn = |s: &LinkState| s.mode == Mode::Swd && s.swd == swd::State::Turn;
        let desync = here & RuleId::TrailingTurnaround.bit() != 0
            && check.enforced(RuleId::TrailingTurnaround, waived);
        if desync && turn(&o.state) {
            let detail = format!("path left the frame open after a bad parity: {}", o.state);
            verdict = Verdict::Diverged(Kind::Rule(RuleId::TrailingTurnaround), at, detail);
            break;
        }
        if check.state && !desync && t.state != o.state {
            let detail = format!("dap-rs {}\npath   {}", t.state, o.state);
            verdict = Verdict::Diverged(Kind::State, at, detail);
            break;
        }
        if pins || matches!(cmd, Some(Cmd::Disconnect)) {
            let differs = PIN_LINES
                .iter()
                .any(|&(bit, i)| open & bit == 0 && t.levels[i] != o.levels[i]);
            if differs {
                let detail = format!(
                    "dap-rs {:?} {:?}\npath   {:?} {:?}",
                    t.levels, t.drives, o.levels, o.drives
                );
                verdict = Verdict::Diverged(Kind::Pads, at, detail);
                break;
            }
        }
        if desync {
            verdict = Verdict::Explained(at);
            break;
        }
    }
    if matches!(verdict, Verdict::Diverged(..)) {
        return Compared {
            verdict,
            fired: all,
            waived: waivers,
        };
    }
    let mut request = None;
    for (at, step) in ours.iter().enumerate().skip(1) {
        if let Cmd::Clock(hz) = stream.cmds[at - 1] {
            request = (hz != 0).then_some(hz);
        }
        let Some(r) = step
            .rates
            .iter()
            .flatten()
            .find(|&&r| request.is_some_and(|q| r > q))
        else {
            continue;
        };
        if !check.enforced(RuleId::ClockCeiling, 0) {
            all |= RuleId::ClockCeiling.bit();
            waivers |= RuleId::ClockCeiling.bit();
            continue;
        }
        let detail = format!("path clocked {r} above the request {request:?}");
        verdict = Verdict::Diverged(Kind::Rule(RuleId::ClockCeiling), at, detail);
        break;
    }
    if !matches!(verdict, Verdict::Diverged(..)) {
        if let Some(d) = check.invariant.and_then(|i| i(stream, ours)) {
            verdict = Verdict::Diverged(Kind::Invariant, ours.len() - 1, d);
        }
    }
    Compared {
        verdict,
        fired: all,
        waived: waivers,
    }
}

fn diverges<F>(check: &Check<'_>, stream: &Stream, ours: &mut F) -> Compared
where
    F: FnMut(&Shared) -> Box<dyn Probe>,
{
    // The path runs first: the reports it marks set what the reference is normalized to.
    let sim = shared(&stream.target);
    let mut probe = ours(&sim);
    let mine = run(stream, probe.as_mut(), &sim);
    let (marked, in_force) = check.marked();
    let waived: Vec<u64> = WAIVED.with(|m| m.take());
    for (i, w) in waived.iter().enumerate() {
        assert!(
            w & !marked == 0,
            "report {i} waives a rule its policy does not waive on marked reports"
        );
    }
    let waived: Vec<u64> = waived.into_iter().map(|w| w & in_force).collect();
    let bits = waived
        .iter()
        .map(|&w| {
            let on = NORMALIZED.iter().filter(|r| check.enforced(**r, w));
            on.fold(0, |bits, r| bits | r.bit())
        })
        .collect();
    let sim = shared(&stream.target);
    let mut dap = reference(&sim);
    dap.latch.follow_bits(bits);
    let theirs = run(stream, &mut dap, &sim);
    let fired = FIRED.with(|m| m.take());
    compare(check, stream, &theirs, &mine, &fired, &waived)
}

/// Both sides' steps on `stream`, the reference unnormalized: dap-rs's answers.
pub fn steps<F>(stream: &Stream, ours: F) -> (Vec<Step>, Vec<Step>)
where
    F: FnOnce(&Shared) -> Box<dyn Probe>,
{
    let sim = shared(&stream.target);
    let theirs = run(stream, &mut reference(&sim), &sim);
    let sim = shared(&stream.target);
    let mine = run(stream, ours(&sim).as_mut(), &sim);
    (theirs, mine)
}

fn diverged(c: Compared) -> Option<(Kind, usize, String)> {
    match c.verdict {
        Verdict::Diverged(kind, at, detail) => Some((kind, at, detail)),
        _ => None,
    }
}

/// Drop commands, command parts and target behaviour while the same symptom remains.
fn shrink<F>(check: &Check<'_>, mut stream: Stream, symptom: (Kind, String), ours: &mut F) -> Stream
where
    F: FnMut(&Shared) -> Box<dyn Probe>,
{
    let still = |s: &Stream, ours: &mut F| {
        diverged(diverges(check, s, ours)).is_some_and(|d| (d.0, d.2) == symptom)
    };
    let mut budget = 4000;
    loop {
        let mut changed = false;
        let mut i = stream.cmds.len();
        while i > 0 && budget > 0 {
            i -= 1;
            budget -= 1;
            let mut s = stream.clone();
            s.cmds.remove(i);
            if still(&s, ours) {
                stream = s;
                changed = true;
                continue;
            }
            for smaller in stream.cmds[i].smaller() {
                budget -= 1;
                let mut s = stream.clone();
                s.cmds[i] = smaller;
                if still(&s, ours) {
                    stream = s;
                    changed = true;
                    break;
                }
            }
        }
        let mut t = stream.clone();
        if !t.target.waits.is_empty() {
            t.target.waits.clear();
            if still(&t, ours) {
                stream = t;
                changed = true;
            }
        }
        if !changed || budget <= 0 {
            return stream;
        }
    }
}

fn rules(bits: u64) -> Vec<RuleId> {
    RULES
        .iter()
        .map(|r| r.id)
        .filter(|r| bits & r.bit() != 0)
        .collect()
}

/// Run every seed of `check`; the first divergence comes back shrunk, then any unfired waiver.
pub fn check<F>(check: &Check<'_>, mut ours: F) -> Result<Report, Box<Failure>>
where
    F: FnMut(&Shared) -> Box<dyn Probe>,
{
    let mut report = Report::default();
    let (mut fired, mut waived) = (0, 0);
    for seed in check.seeds.clone() {
        let s = stream(seed, &check.profile);
        report.commands += s.cmds.len();
        let c = diverges(check, &s, &mut ours);
        fired |= c.fired;
        waived |= c.waived;
        match c.verdict {
            Verdict::Agree => {}
            Verdict::Explained(at) => {
                report.explained += 1;
                report.unchecked += s.cmds.len() - at;
            }
            Verdict::Diverged(kind, _, detail) => {
                let s = shrink(check, s, (kind, detail), &mut ours);
                let (kind, at, detail) =
                    diverged(diverges(check, &s, &mut ours)).expect("a shrunk stream diverges");
                return Err(Box::new(Failure::Diverged(Divergence {
                    seed,
                    kind,
                    at,
                    detail,
                    stream: s,
                })));
            }
        }
    }
    let in_force = check
        .policy
        .waivers
        .iter()
        .filter(|w| (w.on)(&check.profile));
    if let Some(w) = in_force.into_iter().find(|w| waived & w.rule.bit() == 0) {
        return Err(Box::new(Failure::Unfired(w.rule)));
    }
    report.fired = rules(fired);
    Ok(report)
}

/// Every seed of `check`, one shrunk divergence per kind and failing command kind.
pub fn survey<F>(check: &Check<'_>, mut ours: F) -> Vec<Divergence>
where
    F: FnMut(&Shared) -> Box<dyn Probe>,
{
    let mut found: Vec<Divergence> = Vec::new();
    for seed in check.seeds.clone() {
        let s = stream(seed, &check.profile);
        let Some((kind, at, detail)) = diverged(diverges(check, &s, &mut ours)) else {
            continue;
        };
        let name = |s: &Stream, at: usize| {
            at.checked_sub(1)
                .map(|i| std::mem::discriminant(&s.cmds[i]))
        };
        let key = (kind, name(&s, at));
        if found.iter().any(|d| (d.kind, name(&d.stream, d.at)) == key) {
            continue;
        }
        let s = shrink(check, s, (kind, detail), &mut ours);
        if let Some((kind, at, detail)) = diverged(diverges(check, &s, &mut ours)) {
            found.push(Divergence {
                seed,
                kind,
                at,
                detail,
                stream: s,
            });
        }
    }
    found
}

/// How `check`'s policy judges the path on `stream` alone; no waiver need fire.
pub fn verdict<F>(
    check: &Check<'_>,
    stream: &Stream,
    mut ours: F,
) -> Result<Report, Box<Divergence>>
where
    F: FnMut(&Shared) -> Box<dyn Probe>,
{
    let c = diverges(check, stream, &mut ours);
    let mut report = Report {
        commands: stream.cmds.len(),
        fired: rules(c.fired),
        ..Report::default()
    };
    match c.verdict {
        Verdict::Agree => Ok(report),
        Verdict::Explained(at) => {
            report.explained = 1;
            report.unchecked = stream.cmds.len() - at;
            Ok(report)
        }
        Verdict::Diverged(kind, at, detail) => Err(Box::new(Divergence {
            seed: 0,
            kind,
            at,
            detail,
            stream: stream.clone(),
        })),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::differential::reference::{dap_rs_only, Reference};
    use crate::differential::DAP_RS;
    use crate::session::{swd_select, Item};
    use crate::target::{Line, Target, DPIDR};

    #[test]
    fn the_reference_agrees_with_itself() {
        let c = Check::new(Profile::CHAOS, 0..200, &DAP_RS);
        let r = check(&c, |sim| Box::new(reference(sim))).unwrap_or_else(|d| panic!("{d}"));
        assert!(r.commands > 1000);
    }

    #[test]
    fn the_reference_connects_and_reads_dpidr() {
        let s = Stream {
            target: Target {
                mode: crate::target::Mode::Jtag,
                ..Target::default()
            },
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
            ],
        };
        let sim = shared(&s.target);
        let steps = run(&s, &mut reference(&sim), &sim);
        let mut want = vec![0x05, 1, 1];
        want.extend(DPIDR.to_le_bytes());
        assert_eq!(steps[3].response, want);
    }

    #[test]
    fn only_low_cells_between_packets_count_as_idle() {
        let read = Item {
            req: 0x02,
            word: None,
        };
        let s = Stream {
            target: Target::default(),
            cmds: vec![
                Cmd::Connect(1),
                swd_select(),
                Cmd::Configure(2, 5, 0),
                Cmd::Transfer(0, vec![read]),
                Cmd::SwjSequence(3, vec![0]),
            ],
        };
        let sim = shared(&s.target);
        let steps = run(&s, &mut reference(&sim), &sim);
        assert_eq!((steps[4].idle, steps[5].idle), (0, 3));
    }

    #[test]
    fn a_diverging_path_is_caught_and_shrunk() {
        let c = Check::new(Profile::SWD, 0..50, &DAP_RS);
        let d = check(&c, |sim| {
            let mut dap = reference(sim);
            Box::new(move |report: &[u8], response: &mut [u8]| {
                let len = dap.process(report, response);
                if report[0] == 0x06 && len > 4 {
                    response[4] ^= 1;
                }
                len
            })
        })
        .unwrap_err();
        let Failure::Diverged(d) = *d else {
            panic!("{d}")
        };
        assert_eq!(d.kind, Kind::Response);
        assert!(d.stream.cmds.len() <= 8, "{d}");
    }

    #[test]
    fn the_protocol_position_is_compared_only_when_opted_in() {
        let s = Stream {
            target: Target::default(),
            cmds: vec![Cmd::Connect(1), swd_select()],
        };
        // One cell with SWDIO high after the select: a Start bit, so only the link state differs.
        let stray = |sim: &Shared| -> Box<dyn Probe> {
            let (sim, mut dap) = (sim.clone(), reference(sim));
            Box::new(move |report: &[u8], response: &mut [u8]| {
                let len = dap.process(report, response);
                if report[0] == 0x12 {
                    let mut s = sim.borrow_mut();
                    s.set_latch(Line::Swdio, true);
                    s.set_output(Line::Swdio, true);
                    s.set_latch(Line::Swclk, false);
                    s.set_latch(Line::Swclk, true);
                }
                len
            })
        };
        let mut c = Check::new(Profile::SWD, 0..0, &DAP_RS);
        verdict(&c, &s, stray).unwrap_or_else(|d| panic!("{d}"));
        c.state = true;
        let d = verdict(&c, &s, stray).unwrap_err();
        assert_eq!((d.kind, d.at), (Kind::State, 2), "{d}");
    }

    #[test]
    fn a_waiver_that_matches_nothing_fails_the_check() {
        let mut waivers = DAP_RS.waivers.to_vec();
        waivers.push(crate::rule::Waiver {
            rule: RuleId::TrailingTurnaround,
            ..DAP_RS.waivers[0]
        });
        let policy = Policy {
            waivers: waivers.leak(),
        };
        let c = Check::new(Profile::SWD, 0..20, &policy);
        let f = check(&c, |sim| Box::new(reference(sim))).unwrap_err();
        assert!(
            matches!(*f, Failure::Unfired(RuleId::TrailingTurnaround)),
            "{f}"
        );
    }

    #[test]
    fn a_packet_above_the_clock_request_breaks_its_rule() {
        let c = Check::new(Profile::SWD, 0..20, &DAP_RS);
        let f = check(&c, |sim| {
            let (sim, mut dap) = (sim.clone(), reference(sim));
            Box::new(move |report: &[u8], response: &mut [u8]| {
                sim.borrow_mut().rate = Some(u32::MAX);
                dap.process(report, response)
            })
        })
        .unwrap_err();
        let Failure::Diverged(d) = *f else {
            panic!("{f}")
        };
        assert_eq!(d.kind, Kind::Rule(RuleId::ClockCeiling), "{d}");
    }

    /// Clock `cells` on SWDIO, `None` releasing it.
    fn clock(sim: &Shared, cells: impl IntoIterator<Item = Option<bool>>) {
        let mut s = sim.borrow_mut();
        for bit in cells {
            match bit {
                Some(b) => {
                    s.set_latch(Line::Swdio, b);
                    s.set_output(Line::Swdio, true);
                }
                None => s.set_output(Line::Swdio, false),
            }
            s.set_latch(Line::Swclk, false);
            s.set_latch(Line::Swclk, true);
        }
    }

    fn header(req: u8) -> impl Iterator<Item = Option<bool>> {
        (0..8).map(move |i| Some(req >> i & 1 != 0))
    }

    type After = fn(&mut Reference, &Shared, &[u8], &mut [u8], usize);

    /// The reference with `after` run on each report: its first divergence from the reference.
    fn mutated(profile: Profile, after: After) -> Divergence {
        let c = Check::new(profile, 0..50, &DAP_RS);
        let f = check(&c, |sim| {
            let (sim, mut dap) = (sim.clone(), reference(sim));
            Box::new(move |report: &[u8], response: &mut [u8]| {
                let len = dap.process(report, response);
                after(&mut dap, &sim, report, response, len);
                len
            })
        })
        .unwrap_err();
        let Failure::Diverged(d) = *f else {
            panic!("{f}")
        };
        d
    }

    #[test]
    fn a_malformed_header_breaks_its_rule() {
        let d = mutated(Profile::SWD, |_, sim, report, _, _| {
            if report[0] == 0x05 {
                clock(sim, header(0xA5 ^ 0x20));
            }
        });
        assert_eq!(d.kind, Kind::Rule(RuleId::SwdHeader), "{d}");
    }

    #[test]
    fn a_high_idle_cell_frames_a_malformed_header() {
        let d = mutated(Profile::SWD, |_, sim, report, _, _| {
            if report[0] == 0x05 {
                clock(sim, [Some(true)]);
            }
        });
        assert_eq!(d.kind, Kind::Rule(RuleId::SwdHeader), "{d}");
    }

    #[test]
    fn driving_through_a_target_cell_breaks_its_rule() {
        let d = mutated(Profile::SWD, |_, sim, report, _, _| {
            if report[0] == 0x05 {
                clock(sim, header(0xA5).chain([Some(false); 38]));
            }
        });
        assert_eq!(d.kind, Kind::Rule(RuleId::SwdContention), "{d}");
    }

    /// The items of a `DAP_Transfer` report after its first `done`, as their own report.
    fn rest(report: &[u8], done: usize) -> Vec<u8> {
        let (mut at, n) = (3, usize::from(report[2]));
        for _ in 0..done {
            let req = report[at];
            at += if req & 0x02 == 0 || req & 0x10 != 0 {
                5
            } else {
                1
            };
        }
        let mut r = vec![0x05, report[1], (n - done) as u8];
        r.extend(&report[at..]);
        r
    }

    #[test]
    fn running_past_a_failed_transfer_diverges() {
        let d = mutated(Profile::SWD, |dap, _, report, response, len| {
            let done = usize::from(response[1]);
            if report[0] == 0x05 && len > 2 && response[2] != 1 && done < usize::from(report[2]) {
                dap.inner
                    .process_command(&rest(report, done), &mut [0; 512]);
            }
        });
        assert!(matches!(d.kind, Kind::Response | Kind::Effects), "{d}");
    }

    #[test]
    fn arms_count_for_a_failed_swd_transfer_diverges() {
        let d = mutated(Profile::SWD, |_, _, report, response, len| {
            if report[0] == 0x05 && len > 2 && response[2] & 7 != 1 && response[1] > 0 {
                response[1] -= 1;
            }
        });
        assert_eq!(d.kind, Kind::Response, "{d}");
    }

    #[test]
    fn dap_rs_alone_misreads_a_protocol_1_ack() {
        let c = Check::new(Profile::JTAG_ADIV6, 0..50, &DAP_RS);
        let f = check(&c, |sim| Box::new(dap_rs_only(sim))).unwrap_err();
        assert!(
            matches!(*f, Failure::Diverged(ref d) if d.kind == Kind::Response),
            "{f}"
        );
    }

    #[test]
    fn dap_rs_alone_answers_an_absent_tap_with_wait() {
        let c = Check::new(Profile::JTAG_ABSENT, 0..50, &DAP_RS);
        let f = check(&c, |sim| Box::new(dap_rs_only(sim))).unwrap_err();
        let Failure::Diverged(d) = *f else {
            panic!("{f}")
        };
        assert!(d.detail.ends_with("02]"), "{d}");
    }

    /// FNV-1a over every field of the reference's steps on `seeds`; `marked` marks every report.
    fn digest(profile: &Profile, seeds: Range<u64>, marked: bool, record: bool) -> u64 {
        let mut h = 0xCBF2_9CE4_8422_2325u64;
        for seed in seeds {
            let s = stream(seed, profile);
            let sim = shared(&s.target);
            sim.borrow_mut().record(record);
            let mut dap = reference(&sim);
            if marked {
                dap.latch
                    .follow(crate::rule::RuleId::WaitAttempts, vec![true; s.cmds.len()]);
            }
            for t in run(&s, &mut dap, &sim) {
                let text = format!(
                    "{seed}|{:?}|{:?}|{:?}|{:?}|{:?}|{}|{}\n",
                    t.response, t.effects, t.drives, t.levels, t.rates, t.idle, t.state
                );
                for b in text.bytes() {
                    h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01B3);
                }
            }
        }
        h
    }

    /// The digests per profile over the consumers' seed ranges, plain and with every report marked.
    fn digests(record: bool) -> Vec<String> {
        let parity = Profile {
            parity: true,
            ..Profile::SWD
        };
        let cases = [
            ("swd", Profile::SWD, 0..450),
            ("swd-parity", parity, 300..450),
            ("swd-ctrl-parity", Profile::SWD_CTRL_PARITY, 0..450),
            ("jtag", Profile::JTAG, 0..400),
            ("jtag-adiv6", Profile::JTAG_ADIV6, 0..350),
            ("jtag-dpv1", Profile::JTAG_DPV1, 0..400),
            ("jtag-absent", Profile::JTAG_ABSENT, 0..150),
            ("chaos", Profile::CHAOS, 0..300),
        ];
        cases
            .iter()
            .map(|(name, p, seeds)| {
                let plain = digest(p, seeds.clone(), false, record);
                let marked = digest(p, seeds.clone(), true, record);
                format!("{name} {plain:016x} {marked:016x}")
            })
            .collect()
    }

    const DIGESTS: [&str; 8] = [
        "swd 6b97bf95d4863c0b f7490fe4ec6662ba",
        "swd-parity 2ba0dca418cc1ecb 331a660d5699c7f6",
        "swd-ctrl-parity b8a806d5a24d7ac8 280ba97249a8d900",
        "jtag d18cb035629aac4f d18cb035629aac4f",
        "jtag-adiv6 e0160de385b516ab e0160de385b516ab",
        "jtag-dpv1 1577e505ca24d136 1577e505ca24d136",
        "jtag-absent c7b63e8d83dcd154 c7b63e8d83dcd154",
        "chaos c9c9a18221a44d41 85be97b3f6bc561c",
    ];

    #[test]
    fn the_reference_keeps_its_digest_over_the_consumers_seeds() {
        assert_eq!(digests(false), DIGESTS);
    }

    #[test]
    fn recording_leaves_the_reference_digest_unchanged() {
        assert_eq!(digests(true), DIGESTS);
    }
}
