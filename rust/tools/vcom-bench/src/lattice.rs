//! The shared baud lattice and ceiling search, kept free of hardware so both are unit-testable.

use crate::stream::Grade;

pub const DEFAULT_BUDGET_PCT: f64 = 2.0;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rung {
    pub nominal: u32,
    pub dut_realized: u32,
    pub probe_realized: Option<u32>,
}

impl Rung {
    /// Percentage of the DUT's realized rate (or the nominal, when the probe doesn't report).
    pub fn mismatch_pct(&self) -> f64 {
        let reference = f64::from(self.dut_realized);
        if reference == 0.0 {
            return f64::INFINITY;
        }
        let other = f64::from(self.probe_realized.unwrap_or(self.nominal));
        (other - reference).abs() * 100.0 / reference
    }

    pub fn agreed(&self, budget_pct: f64) -> bool {
        self.mismatch_pct() <= budget_pct
    }
}

/// The standard rate ladder up to `dut_max`, plus fractions of `dut_max` to probe the edge.
pub fn candidate_rates(dut_max: u32) -> Vec<u32> {
    const STANDARD: &[u32] = &[
        9_600, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800, 921_600, 1_000_000, 1_500_000,
        2_000_000, 3_000_000, 4_000_000, 5_000_000, 6_000_000, 8_000_000, 10_000_000,
    ];
    let mut rates: Vec<u32> = STANDARD.iter().copied().filter(|&r| r <= dut_max).collect();
    for frac in [
        (dut_max / 4) * 3,
        (dut_max / 8) * 7,
        (dut_max / 16) * 15,
        dut_max,
    ] {
        if frac > 0 {
            rates.push(frac);
        }
    }
    rates.sort_unstable();
    rates.dedup();
    rates
}

/// Consecutive screen failures before giving up on a ceiling.
pub const DEFAULT_SCREEN_MAX: usize = 8;

/// Rungs sustained at and below the screened boundary: the boundary rung and the one under it.
pub const DEFAULT_SUSTAIN_RUNGS: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// Whether a short message still round-trips: the console question, asked of every rung and
    /// independent of what a saturating stream can hold.
    Message,
    /// Which rungs hold a short saturating stream, across the whole pruned lattice.
    Sweep,
    /// Top-down from the highest swept rung: where the sustained boundary is.
    Screen,
    /// At and just below the boundary: what the pairing holds over time.
    Sustain,
}

impl Stage {
    pub fn key(self) -> &'static str {
        match self {
            Stage::Message => "message",
            Stage::Sweep => "sweep",
            Stage::Screen => "screen",
            Stage::Sustain => "sustain",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StagedLimits {
    pub screen_max: usize,
    pub sustain_rungs: usize,
}

impl Default for StagedLimits {
    fn default() -> Self {
        StagedLimits {
            screen_max: DEFAULT_SCREEN_MAX,
            sustain_rungs: DEFAULT_SUSTAIN_RUNGS,
        }
    }
}

/// What the staged search produced: one ceiling per stage, and every run it took to get them.
#[derive(Debug, Default)]
pub struct StagedOutcome {
    /// Highest rung a short message still echoed byte-exact.
    pub message: Option<usize>,
    /// A rung below the highest passing message rung failed, so the console ceiling has a hole.
    pub message_gap: bool,
    /// Highest rung the stream sweep passed.
    pub swept: Option<usize>,
    /// Highest rung the screen held.
    pub screened: Option<usize>,
    /// Highest rung a sustain window held byte-exact with every counter at zero.
    pub sustained: Option<usize>,
    /// Highest rung that carried its budget with all loss counted; at or above `sustained`.
    pub sustained_qualified: Option<usize>,
    pub trace: Vec<(Stage, ProbeResult)>,
    /// A rung above a failing one passed the sweep, so the lattice is not monotone.
    pub non_monotone: bool,
    /// A rung the sweep failed held its screen, so the sweep understated the pairing.
    pub sweep_pessimistic: bool,
    pub screen_exhausted: bool,
}

/// Escalate a pairing through sweep, screen, and sustain, stopping at the first ceiling each stage
/// confirms.
pub fn staged(
    len: usize,
    limits: StagedLimits,
    mut test: impl FnMut(Stage, usize) -> Grade,
) -> StagedOutcome {
    let mut out = StagedOutcome::default();
    macro_rules! run {
        ($stage:expr, $index:expr, $out:expr) => {
            run_graded($stage, $index, $out, &mut test) >= Grade::Qualified
        };
    }

    // The console question first, and over the whole lattice: a short message can survive a rate
    // at which a saturating duplex stream misses its bar, so this ceiling is its own.
    let mut message = None;
    for index in 0..len {
        if run!(Stage::Message, index, &mut out) {
            out.message_gap |= message.is_none_or(|last| last + 1 != index) && index > 0;
            message = Some(index);
        }
    }
    out.message = message;

    let mut correct: Vec<usize> = Vec::new();
    for index in 0..len {
        if run!(Stage::Sweep, index, &mut out) {
            out.non_monotone |= correct.last().is_none_or(|&last| last + 1 != index) && index > 0;
            correct.push(index);
        }
    }
    out.swept = correct.last().copied();

    // The sweep's window is short enough that startup transients dominate it, so a rung it failed
    // above the swept ceiling is a candidate, not a verdict. Re-ask those at the screen window
    // before the search files the rung below as the top: a sweep failure must never become a
    // ceiling unconfirmed. A rung that holds is screened, so the walk costs a window only where it
    // buys one, plus the single failing window that stops it.
    let mut screened = None;
    if let Some(&top) = correct.last() {
        for index in top + 1..len {
            if !run!(Stage::Screen, index, &mut out) {
                break;
            }
            out.sweep_pessimistic = true;
            correct.push(index);
            screened = Some(index);
        }
    }

    if screened.is_none() {
        for (tried, &index) in correct.iter().rev().enumerate() {
            if tried >= limits.screen_max {
                out.screen_exhausted = true;
                break;
            }
            if run!(Stage::Screen, index, &mut out) {
                screened = Some(index);
                break;
            }
        }
    }
    out.screened = screened;

    let Some(screened) = screened else {
        return out;
    };
    // Descend until both ceilings are known: a rung can carry its budget with counted loss well
    // above the highest rung that is byte-exact, and the pairing's numbers are both of them.
    let below = correct.iter().rev().skip_while(|&&i| i > screened);
    for &index in below.take(limits.sustain_rungs) {
        let grade = run_graded(Stage::Sustain, index, &mut out, &mut test);
        if grade >= Grade::Qualified && out.sustained_qualified.is_none() {
            out.sustained_qualified = Some(index);
        }
        if grade == Grade::Clean {
            out.sustained = Some(index);
            break;
        }
    }
    out
}

fn run_graded(
    stage: Stage,
    index: usize,
    out: &mut StagedOutcome,
    test: &mut impl FnMut(Stage, usize) -> Grade,
) -> Grade {
    let grade = test(stage, index);
    out.trace.push((
        stage,
        ProbeResult {
            index,
            pass: grade >= Grade::Qualified,
        },
    ));
    grade
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProbeResult {
    /// Index into the pruned rung list, ascending by rate.
    pub index: usize,
    pub pass: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn result(out: &StagedOutcome, stage: Stage, index: usize) -> Option<bool> {
        out.trace
            .iter()
            .find(|(s, p)| *s == stage && p.index == index)
            .map(|(_, p)| p.pass)
    }

    fn pairing(
        correct: &[bool],
        screen: usize,
        sustain: usize,
    ) -> (StagedOutcome, Vec<(Stage, usize)>) {
        messaging(correct, correct, screen, sustain)
    }

    fn messaging(
        message: &[bool],
        correct: &[bool],
        screen: usize,
        sustain: usize,
    ) -> (StagedOutcome, Vec<(Stage, usize)>) {
        let mut runs = Vec::new();
        let out = staged(correct.len(), StagedLimits::default(), |stage, i| {
            runs.push((stage, i));
            let pass = match stage {
                Stage::Message => message[i],
                Stage::Sweep => correct[i],
                Stage::Screen => correct[i] && i <= screen,
                Stage::Sustain => correct[i] && i <= sustain,
            };
            if pass {
                Grade::Clean
            } else {
                Grade::Fail
            }
        });
        (out, runs)
    }

    #[test]
    fn the_sweep_covers_every_rung_and_bounds_the_stages_above_it() {
        let correct: Vec<bool> = (0..12).map(|i| i <= 8).collect();
        let (out, runs) = pairing(&correct, 6, 5);
        assert_eq!(out.swept, Some(8));
        assert_eq!(out.screened, Some(6));
        assert_eq!(out.sustained, Some(5));
        assert!(!out.non_monotone);
        let swept: Vec<usize> = runs
            .iter()
            .filter(|(s, _)| *s == Stage::Sweep)
            .map(|(_, i)| *i)
            .collect();
        assert_eq!(swept, (0..12).collect::<Vec<_>>());
        // Rung 9 is the one confirming screen above the swept ceiling; nothing else above it runs.
        assert!(runs
            .iter()
            .all(|(s, i)| matches!(s, Stage::Message | Stage::Sweep) || correct[*i] || *i == 9));
    }

    /// The sweep window is short enough to fail a rung a longer one holds. Where that rung is the
    /// top of the lattice the old search filed the rung below as the ceiling with nothing to catch
    /// it, so the screen re-asks every rung above the swept ceiling before accepting it.
    #[test]
    fn a_rung_the_sweep_failed_at_the_top_is_re_asked_before_a_ceiling_is_filed() {
        let mut runs = Vec::new();
        // The sweep fails 10 and 11; both hold a screen, and 11 also sustains.
        let out = staged(12, StagedLimits::default(), |stage, i| {
            runs.push((stage, i));
            let pass = match stage {
                Stage::Sweep => i <= 9,
                _ => true,
            };
            if pass {
                Grade::Clean
            } else {
                Grade::Fail
            }
        });
        assert_eq!(out.swept, Some(9));
        assert!(out.sweep_pessimistic);
        assert_eq!(out.screened, Some(11));
        assert_eq!(out.sustained, Some(11));
        assert_eq!(
            runs.iter()
                .filter(|(s, _)| *s == Stage::Screen)
                .map(|(_, i)| *i)
                .collect::<Vec<_>>(),
            vec![10, 11]
        );
    }

    #[test]
    fn a_confirmed_sweep_failure_costs_one_window_and_leaves_the_ceiling_alone() {
        let correct: Vec<bool> = (0..12).map(|i| i <= 8).collect();
        let (out, runs) = pairing(&correct, 8, 8);
        assert_eq!(out.screened, Some(8));
        assert!(!out.sweep_pessimistic);
        assert_eq!(
            runs.iter()
                .filter(|(s, i)| *s == Stage::Screen && !correct[*i])
                .count(),
            1
        );
    }

    #[test]
    fn a_short_message_carries_past_the_rung_where_the_stream_gives_up() {
        // The shape the F411 pairings show: a console works two rungs above the stream ceiling.
        let message: Vec<bool> = (0..12).map(|i| i <= 8).collect();
        let correct: Vec<bool> = (0..12).map(|i| i <= 6).collect();
        let (out, runs) = messaging(&message, &correct, 6, 6);
        assert_eq!(out.message, Some(8));
        assert_eq!(out.swept, Some(6));
        assert!(!out.message_gap);
        let asked: Vec<usize> = runs
            .iter()
            .filter(|(s, _)| *s == Stage::Message)
            .map(|(_, i)| *i)
            .collect();
        assert_eq!(asked, (0..12).collect::<Vec<_>>());
    }

    #[test]
    fn a_message_ceiling_with_a_hole_under_it_is_reported_as_one() {
        let message: Vec<bool> = (0..8).map(|i| i != 3).collect();
        let (out, _) = messaging(&message, &[true; 8], 7, 7);
        assert_eq!(out.message, Some(7));
        assert!(out.message_gap);
    }

    #[test]
    fn the_screen_starts_at_the_highest_correct_rung_and_walks_down() {
        let correct: Vec<bool> = (0..10).map(|i| i <= 7).collect();
        let (out, runs) = pairing(&correct, 5, 5);
        let screened: Vec<usize> = runs
            .iter()
            .filter(|(s, _)| *s == Stage::Screen)
            .map(|(_, i)| *i)
            .collect();
        // Rung 8 is the confirming screen above the swept ceiling, then the walk down.
        assert_eq!(screened, vec![8, 7, 6, 5]);
        assert_eq!(out.screened, Some(5));
        assert_eq!(out.sustained, Some(5));
    }

    #[test]
    fn sustain_stops_at_the_first_rung_that_holds_and_reports_that_one() {
        let correct = [true; 8];
        let (out, runs) = pairing(&correct, 7, 6);
        let sustained: Vec<usize> = runs
            .iter()
            .filter(|(s, _)| *s == Stage::Sustain)
            .map(|(_, i)| *i)
            .collect();
        assert_eq!(sustained, vec![7, 6]);
        assert_eq!(out.sustained, Some(6));
        assert_eq!(result(&out, Stage::Sustain, 7), Some(false));
    }

    #[test]
    fn the_sustain_tier_descends_until_a_rung_holds() {
        let correct = [true; 6];
        let (out, _) = pairing(&correct, 5, 3);
        assert_eq!(out.screened, Some(5));
        // 5 and 4 fail the long window; the search keeps stepping down instead of giving up.
        assert_eq!(out.sustained, Some(3));
        assert_eq!(result(&out, Stage::Sustain, 5), Some(false));
        assert_eq!(result(&out, Stage::Sustain, 4), Some(false));
        assert_eq!(result(&out, Stage::Sustain, 3), Some(true));
    }

    #[test]
    fn a_non_monotone_lattice_is_swept_whole_and_the_hump_is_taken() {
        let correct: Vec<bool> = (0..12).map(|i| i <= 3 || (8..=9).contains(&i)).collect();
        let (out, runs) = pairing(&correct, 9, 8);
        assert!(out.non_monotone);
        assert_eq!(out.swept, Some(9));
        assert_eq!(out.screened, Some(9));
        assert_eq!(out.sustained, Some(8));
        for i in 4..8 {
            assert!(!runs.contains(&(Stage::Screen, i)));
            assert!(!runs.contains(&(Stage::Sustain, i)));
        }
    }

    #[test]
    fn a_hump_with_nothing_below_it_is_still_the_ceiling() {
        let correct: Vec<bool> = (0..10).map(|i| (4..=6).contains(&i)).collect();
        let (out, _) = pairing(&correct, 6, 6);
        assert!(out.non_monotone);
        assert_eq!(out.swept, Some(6));
        assert_eq!(out.sustained, Some(6));
    }

    #[test]
    fn a_lattice_that_sweeps_clean_nowhere_costs_only_the_sweep() {
        let correct = [false; 9];
        let (out, runs) = pairing(&correct, 0, 0);
        assert_eq!(out.swept, None);
        assert_eq!(out.screened, None);
        assert_eq!(out.sustained, None);
        assert!(runs
            .iter()
            .all(|(s, _)| matches!(s, Stage::Message | Stage::Sweep)));
        assert_eq!(runs.len(), 18);
    }

    #[test]
    fn the_screen_gives_up_rather_than_walking_a_whole_lattice_down() {
        let correct = [true; 20];
        let (out, runs) = pairing(&correct, 0, 0);
        assert!(out.screen_exhausted);
        assert_eq!(out.screened, None);
        assert_eq!(out.sustained, None);
        let screens = runs.iter().filter(|(s, _)| *s == Stage::Screen).count();
        assert_eq!(screens, DEFAULT_SCREEN_MAX);
    }

    #[test]
    fn an_empty_lattice_runs_nothing() {
        let out = staged(0, StagedLimits::default(), |_, _| {
            unreachable!("no rung to run")
        });
        assert_eq!(out.swept, None);
        assert!(out.trace.is_empty());
    }

    #[test]
    fn one_rung_that_holds_costs_one_window_per_stage() {
        let (out, runs) = pairing(&[true], 0, 0);
        assert_eq!(out.sustained, Some(0));
        assert_eq!(
            runs,
            vec![
                (Stage::Message, 0),
                (Stage::Sweep, 0),
                (Stage::Screen, 0),
                (Stage::Sustain, 0)
            ]
        );
    }

    #[test]
    fn the_f401_edge_rung_agrees_inside_the_default_budget() {
        let rung = Rung {
            nominal: 2_625_000,
            dut_realized: 2_625_000,
            probe_realized: Some(2_666_666),
        };
        assert!(rung.mismatch_pct() > 1.5 && rung.mismatch_pct() < 1.7);
        assert!(rung.agreed(DEFAULT_BUDGET_PCT));
    }

    #[test]
    fn a_probe_stuck_on_its_old_rate_is_pruned() {
        let rung = Rung {
            nominal: 4_000_000,
            dut_realized: 4_000_000,
            probe_realized: Some(2_666_666),
        };
        assert!(!rung.agreed(DEFAULT_BUDGET_PCT));
    }

    #[test]
    fn an_unreported_probe_rate_falls_back_to_nominal_agreement() {
        let close = Rung {
            nominal: 921_600,
            dut_realized: 923_077,
            probe_realized: None,
        };
        assert!(close.agreed(DEFAULT_BUDGET_PCT));
        let far = Rung {
            nominal: 4_000_000,
            dut_realized: 3_500_000,
            probe_realized: None,
        };
        assert!(!far.agreed(DEFAULT_BUDGET_PCT));
    }

    #[test]
    fn candidates_cover_the_standard_ladder_and_probe_the_edge() {
        let rates = candidate_rates(2_625_000);
        assert!(rates.contains(&9_600));
        assert!(rates.contains(&921_600));
        assert!(rates.contains(&2_000_000));
        assert!(rates.contains(&2_625_000));
        assert!(rates.iter().any(|&r| r > 2_000_000 && r < 2_625_000));
        assert!(rates.iter().all(|&r| r <= 2_625_000));
        let mut sorted = rates.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(rates, sorted, "sorted and deduplicated");
    }

    #[test]
    fn a_faster_dut_raises_the_ladder_without_a_code_change() {
        let rates = candidate_rates(7_812_500);
        assert!(rates.contains(&6_000_000));
        assert!(rates.contains(&7_812_500));
        assert!(!rates.contains(&10_000_000));
    }
}
