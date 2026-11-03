//! The `ladder` verb: the staged search for a pairing's ceilings, and the full sweep.

use anyhow::{bail, Result};

use super::outcomes;
use super::report::{divergence, opt_rate, pairing_report, verdict_word};
use super::session::{
    attach_armed, declined_note, one_window, plan_for, prepare_settled, quiesce_line,
    settled_probe_baud, unsettled_note, windows_of, BaudSettle, Bench, Bracket, DisarmOnExit,
    InFlight, Rig, StreamWindow,
};
use crate::budget::{self, BusBudget, Clamp};
use crate::cli::LadderArgs;
use crate::counters::Conservation;
use crate::dut::Dut;
use crate::lattice::{self, Rung, Stage};
use crate::record::staged_summary;
use crate::stream::{self, Direction, Plan, Run, Shape};
use crate::tty::{self, Ledger, Tty};
use crate::{lifecycle, probe, record};

pub(super) fn ladder_verb(args: LadderArgs) -> Result<()> {
    let mut bench = Bench::resolve(&args.rig)?;
    // The only verb that exposes the divisor budget; every rate confirmation is judged against it.
    bench.budget_pct = args.budget_pct;
    let mut disarm = DisarmOnExit::new(&bench);
    disarm.finish(ladder_on(&bench, args))
}

fn ladder_on(bench: &Bench, args: LadderArgs) -> Result<()> {
    // Phase 1: the DUT's side of the lattice, over RTT.
    eprintln!("deriving the DUT lattice...");
    let mut dutctl = bench.open_dut()?;
    let dut_max = dutctl.baud_max()?;
    eprintln!("  DUT BAUD MAX {dut_max}");
    let candidates = lattice::candidate_rates(dut_max);
    let mut dut_realized: Vec<Option<u32>> = Vec::with_capacity(candidates.len());
    for &rate in &candidates {
        dut_realized.push(dutctl.set_baud(rate)?);
    }
    drop(dutctl);

    // Phase 2: the probe's side, via line coding plus the realized-rate vendor query.
    eprintln!("deriving the probe lattice...");
    let mut tty = Tty::open_selected(&bench.selector, candidates[0])?;
    let mut programmed = Some(candidates[0]);
    let mut probe_reported = true;
    let mut rungs: Vec<Rung> = Vec::new();
    let mut declined: Vec<u32> = Vec::new();
    let bus;
    {
        let mut link = probe::DapLink::open(&bench.selector, bench.vendor)?;
        bus = BusBudget::for_packet(link.max_packet_size());
        for (&rate, &dut_r) in candidates.iter().zip(&dut_realized) {
            let Some(dut_r) = dut_r else {
                declined.push(rate);
                continue;
            };
            let queued = quiesce_line(&mut link, &mut tty, programmed)?;
            if let InFlight::Bytes(n) = queued {
                if n != 0 {
                    eprintln!(
                        "  {n} bytes were still queued at {} when {rate} was requested",
                        opt_rate(programmed)
                    );
                }
            }
            tty.set_baud(rate)?;
            let settle = settled_probe_baud(&mut link, rate, args.budget_pct)?;
            programmed = settle.realized().or(Some(rate));
            match settle {
                BaudSettle::Unsettled(u) => {
                    eprintln!("  {}", unsettled_note(rate, u));
                    declined.push(rate);
                    continue;
                }
                BaudSettle::Declined { applied, realized } => {
                    eprintln!("  {}", declined_note(rate, applied, realized));
                    declined.push(rate);
                    continue;
                }
                _ => {}
            }
            let probe_r = settle.realized();
            probe_reported &= probe_r.is_some();
            rungs.push(Rung {
                nominal: rate,
                dut_realized: dut_r,
                probe_realized: probe_r,
            });
        }
    }

    let shape = Shape::Duplex;
    let plan_at = |rung: &Rung, secs: f64| {
        plan_for(
            shape,
            &args.traffic,
            rung.dut_realized,
            rung.probe_realized,
            secs,
            1.0,
            stream::CREDIT_WINDOW,
        )
    };

    // Prune the rate-mismatched rungs, then the ones the bus cannot serve.
    let mut kept: Vec<Rung> = Vec::new();
    let mut clamped: Vec<(Rung, Clamp)> = Vec::new();
    for rung in rungs.iter().filter(|r| r.agreed(args.budget_pct)) {
        let clamp = budget::clamp(
            &plan_at(rung, 1.0).demand(args.traffic.dap.nominal_bps()),
            &bus,
        );
        if clamp.fits || args.traffic.allow_over_budget {
            kept.push(rung.clone());
        } else {
            clamped.push((rung.clone(), clamp));
        }
    }
    println!(
        "lattice (DUT max {dut_max}, budget {:.1}%):",
        args.budget_pct
    );
    println!(
        "  {:>10}  {:>12}  {:>14}  {:>8}  kept",
        "nominal", "DUT realized", "probe realized", "mismatch"
    );
    for r in &rungs {
        println!(
            "  {:>10}  {:>12}  {:>14}  {:>7.2}%  {}",
            r.nominal,
            r.dut_realized,
            opt_rate(r.probe_realized),
            r.mismatch_pct(),
            if !r.agreed(args.budget_pct) {
                "no (rate mismatch)".to_string()
            } else if let Some((_, clamp)) = clamped.iter().find(|(c, _)| c.nominal == r.nominal) {
                format!("no ({:.1} slots/ms over the bus budget)", clamp.demand)
            } else {
                "yes".to_string()
            }
        );
    }
    for rate in &declined {
        println!("  {rate:>10}  declined by the DUT");
    }
    if !probe_reported {
        println!(
            "  note: the probe does not report its realized rate; pruning used the DUT side only"
        );
    }
    let paced = clamped.iter().rev().find(|(rung, _)| {
        budget::clamp(
            &plan_at(rung, 1.0)
                .as_return_priority()
                .demand(args.traffic.dap.nominal_bps()),
            &bus,
        )
        .fits
    });
    if let Some((rung, clamp)) = paced {
        println!(
            "  the symmetric point at {} asks {:.2} of {:.2} slots/ms; measure it as \
             `run --mode return-priority --baud {}` instead",
            rung.nominal, clamp.demand, clamp.budget, rung.nominal,
        );
    }
    if kept.is_empty() {
        bail!("no rung survives pruning; the pairing shares no rate the bus can serve");
    }

    // Phase 3: the search over the kept rungs, staged unless --full asks for the whole sweep.
    if !args.full {
        return staged_ladder(bench, &mut tty, &args, &bus, dut_max, &rungs, &kept);
    }

    let (secs, runs, phase) = args.windows.resolve(stream::DISCOVERY_SECS, "discovery");
    if runs > 1 {
        eprintln!("  note: the ladder measures each rung once; --runs applies to `run`");
    }
    let mut dut = None;
    let mut rig = Rig {
        bench,
        verb: "ladder",
        dut: &mut dut,
        tty: &mut tty,
        holds_records: false,
        held: Vec::new(),
    };
    full_sweep(&mut rig, &kept, &args, &bus, |r| plan_at(r, secs), phase)
}

pub(crate) struct Row {
    pub(crate) stage: Stage,
    pub(crate) rung: Rung,
    /// The window, or why it could not be run at all.
    pub(crate) outcome: std::result::Result<Run, String>,
    /// The probe's own ledger over the bracket this window ran in, where one could be taken.
    pub(crate) conservation: Option<Conservation>,
    /// The DUT had to be reset after this window before the search could go on.
    pub(crate) recovered: bool,
}

impl Row {
    pub(crate) fn pass(&self) -> bool {
        self.outcome.as_ref().is_ok_and(|run| run.verdict().pass)
    }
}

/// Bytes in a short message: about a line of console output.
pub(super) const MESSAGE_BYTES: usize = 64;

/// A short message at one rung, in the two shapes a terminal produces.
pub(crate) struct Message {
    /// A whole line handed over at once.
    pub(crate) line: Ledger,
    /// The same bytes one at a time, which is what typing looks like.
    pub(crate) keystrokes: Ledger,
    /// What the DUT's UART counted while the two exchanges ran.
    pub(crate) stats: Option<daplink_test_protocol::Stats>,
}

impl Message {
    pub(crate) fn exact(&self) -> bool {
        self.line.byte_exact && self.keystrokes.byte_exact
    }

    pub(crate) fn pass(&self) -> bool {
        self.exact() && self.stats.is_none_or(|s| s.stuck == 0)
    }
}

pub(crate) struct MessageRow {
    pub(crate) rung: Rung,
    /// The exchange, or why it could not be run at all.
    pub(crate) outcome: std::result::Result<Message, String>,
}

pub(super) struct Staged<'a> {
    pub(super) bench: &'a Bench,
    pub(super) args: &'a LadderArgs,
    tty: &'a mut Tty,
    bus: &'a BusBudget,
    pub(super) kept: &'a [Rung],
    /// Reopened per sustain window, so its bracket carries the probe's counters instead of a null.
    dut: Option<Dut>,
    pub(super) rows: Vec<Row>,
    pub(super) messages: Vec<MessageRow>,
    operational: Result<()>,
}

impl Staged<'_> {
    fn release(&mut self) {
        self.dut = None;
    }

    fn message(&mut self, index: usize) -> stream::Grade {
        if self.operational.is_err() {
            return stream::Grade::Fail;
        }
        let rung = self.kept[index].clone();
        eprintln!(
            "  message at {}: {MESSAGE_BYTES} B line, then {MESSAGE_BYTES} keystrokes",
            rung.nominal
        );
        let outcome = (|| -> Result<Message> {
            let (bench, tty) = (self.bench, &mut *self.tty);
            // The exchange is scored against the rate the probe confirmed, not one slept toward.
            prepare_settled(bench, &mut self.dut, tty, rung.nominal)?;
            let dut = attach_armed(bench, &mut self.dut)?;
            let baud = rung
                .dut_realized
                .min(rung.probe_realized.unwrap_or(rung.nominal));
            let payload = tty::pattern(MESSAGE_BYTES);
            let line = tty.echo_paced(&payload, payload.len(), baud)?;
            let keystrokes = tty.echo_paced(&payload, 1, baud)?;
            let stats = Some(dut.stats()?);
            Ok(Message {
                line,
                keystrokes,
                stats,
            })
        })();

        let grade = match &outcome {
            Ok(message) if message.pass() => stream::Grade::Clean,
            Ok(_) => stream::Grade::Fail,
            Err(_) => stream::Grade::Fail,
        };
        match &outcome {
            Ok(message) => {
                eprintln!(
                    "    {} -- line {}, keystrokes {}",
                    verdict_word(message.pass()),
                    divergence(&message.line),
                    divergence(&message.keystrokes),
                );
                if message.exact() && !message.pass() {
                    eprintln!("    the DUT latched a stuck DMA abort; its ledger is suspect");
                }
            }
            Err(e) => eprintln!("    the exchange errored ({e:#}); counted as FAIL"),
        }
        if grade == stream::Grade::Fail {
            // A failed exchange can leave bytes in flight; the next rung starts from a reset DUT.
            self.bench.arm_reset();
            self.release();
        }
        let outcome = outcome.map_err(|error| {
            let message = format!("{error:#}");
            if !outcomes::rate_declined(&error) {
                self.fail(error);
            }
            message
        });
        self.messages.push(MessageRow { rung, outcome });
        grade
    }

    /// Graded failures recover for the next rung; operational errors terminate the search.
    fn window(&mut self, stage: Stage, index: usize, secs: f64, warmup: f64) -> stream::Grade {
        if self.operational.is_err() {
            return stream::Grade::Fail;
        }
        let rung = self.kept[index].clone();
        let bracket = stage == Stage::Sustain;
        eprintln!("  {} at {}: {:.0}s window", stage.key(), rung.nominal, secs);

        let plan = plan_for(
            Shape::Duplex,
            &self.args.traffic,
            rung.dut_realized,
            rung.probe_realized,
            secs,
            1.0,
            stream::CREDIT_WINDOW,
        );
        let (outcome, conservation) = lifecycle::run(&mut StreamWindow {
            bench: self.bench,
            dut: &mut self.dut,
            tty: self.tty,
            rung: &rung,
            prepared: None,
            bracket: if bracket {
                Bracket::Required
            } else {
                Bracket::None
            },
            body: |dut: &mut Dut, tty: &mut Tty| {
                windows_of(dut, tty, &plan, &self.args.traffic, warmup)
            },
        });

        let (outcome, conservation) = match conservation {
            Ok(conservation) => (outcome, conservation),
            Err(error) => (outcomes::after(outcome, Err(error)), None),
        };
        let outcome = match outcome {
            Ok((warmed, mut run)) => {
                let clamp =
                    budget::clamp(&plan.demand(self.args.traffic.dap.nominal_bps()), self.bus);
                if let Some(mut warmed) = warmed {
                    warmed.conservation = conservation;
                    self.append(&rung, &warmed, "warmup", clamp);
                }
                run.conservation = conservation;
                self.append(&rung, &run, stage.key(), clamp);
                let verdict = run.verdict();
                eprintln!("    {}", verdict_word(verdict.pass));
                for reason in &verdict.reasons {
                    eprintln!("      {reason}");
                }
                for note in &verdict.notes {
                    eprintln!("      note: {note}");
                }
                Ok(run)
            }
            Err(e) => {
                eprintln!("    window errored ({e:#}); counted as FAIL");
                let message = format!("{e:#}");
                if !outcomes::rate_declined(&e) {
                    self.fail(e);
                }
                Err(message)
            }
        };

        let grade = outcome
            .as_ref()
            .map_or(stream::Grade::Fail, |run: &Run| run.grade());
        let recovered = grade == stream::Grade::Fail && self.reset_after(&outcome);
        self.rows.push(Row {
            stage,
            rung,
            outcome,
            conservation,
            recovered,
        });
        grade
    }

    fn reset_after(&mut self, outcome: &std::result::Result<Run, String>) -> bool {
        if !self.bench.reset_dut {
            return false;
        }
        if let (false, Some(dut)) = (outcome.is_err(), self.dut.as_mut()) {
            match dut.reset() {
                Ok(()) => return true,
                Err(e) => {
                    eprintln!("warning: the DUT reset failed: {e:#}");
                    self.fail(e);
                }
            }
        }
        self.bench.arm_reset();
        self.release();
        true
    }

    fn append(&mut self, rung: &Rung, run: &Run, phase: &'static str, clamp: Clamp) {
        let result = self.bench.append_v1(&record::V1Record {
            probe: &self.bench.selector,
            firmware: self.bench.firmware(),
            verb: "ladder",
            phase,
            baud_nominal: rung.nominal,
            probe_realized: rung.probe_realized,
            run,
            clamp: Some(clamp),
            note: None,
        });
        let prior = std::mem::replace(&mut self.operational, Ok(()));
        self.operational = outcomes::combine(prior, result);
    }

    fn fail(&mut self, error: anyhow::Error) {
        let prior = std::mem::replace(&mut self.operational, Ok(()));
        self.operational = outcomes::combine(prior, Err(error));
    }
}

/// The default `ladder`: sweep, screen and sustain a pairing, then print its numbers.
fn staged_ladder(
    bench: &Bench,
    tty: &mut Tty,
    args: &LadderArgs,
    bus: &BusBudget,
    dut_max: u32,
    rungs: &[Rung],
    kept: &[Rung],
) -> Result<()> {
    let (sustain_secs, runs, sustain_phase) =
        args.windows.resolve(args.stages.sustain_secs, "sustain");
    if runs > 1 {
        eprintln!(
            "  note: the staged search sustains each rung once; the {runs}-window qualification \
             is `run --qualify --baud <ceiling>`"
        );
    }
    println!(
        "staged search over {} rungs: sweep {:.1}s, screen {:.1}s, sustain {:.1}s \
         ({sustain_phase}, {:.1}s warmup), checkpoints every {:.1}s",
        kept.len(),
        args.stages.sweep_secs,
        args.stages.screen_secs,
        sustain_secs,
        args.windows.warmup,
        args.traffic.checkpoint,
    );
    let mut staged = Staged {
        bench,
        args,
        tty,
        bus,
        kept,
        dut: None,
        rows: Vec::new(),
        messages: Vec::new(),
        operational: Ok(()),
    };
    let limits = args.stages.limits();
    // Only the sustain tier certifies, so only it pays for a warmup.
    let outcome = lattice::staged(kept.len(), limits, |stage, index| match stage {
        Stage::Message => staged.message(index),
        Stage::Sweep => staged.window(stage, index, args.stages.sweep_secs, 0.0),
        Stage::Screen => staged.window(stage, index, args.stages.screen_secs, 0.0),
        Stage::Sustain => staged.window(stage, index, sustain_secs, args.windows.warmup),
    });

    let rp = match (
        outcome.sustained,
        args.stages.rp_secs > 0.0 && staged.operational.is_ok(),
    ) {
        (Some(index), true) => return_priority(&mut staged, index, args.stages.rp_secs),
        _ => None,
    };
    staged.release();

    let report = pairing_report(&staged, dut_max, rungs, &outcome, &rp);
    println!();
    print!("{}", report.render());
    let summary = staged_summary(&report, args, sustain_secs, record::unix_now());
    let recording = outcomes::combine(
        staged.operational,
        outcomes::recorded(
            record::append_line(&bench.record_path, &summary),
            &bench.record_path,
        ),
    );
    outcomes::combine(
        recording,
        outcomes::ladder(
            args.strict,
            outcome.sustained.is_some(),
            (args.stages.rp_secs > 0.0).then(|| rp.as_ref().is_some_and(|run| run.verdict().pass)),
        ),
    )
}

fn return_priority(staged: &mut Staged, index: usize, secs: f64) -> Option<Run> {
    let rung = staged.kept[index].clone();
    eprintln!("  return priority at {}: {secs:.0}s window", rung.nominal);
    let Staged {
        bench,
        dut,
        tty,
        args,
        bus,
        ..
    } = staged;
    let plan = plan_for(
        Shape::ReturnPriority,
        &args.traffic,
        rung.dut_realized,
        rung.probe_realized,
        secs,
        stream::RETURN_PRIORITY_FRAC,
        stream::CREDIT_WINDOW,
    );
    let clamp = budget::clamp(&plan.demand(args.traffic.dap.nominal_bps()), bus);
    match one_window(bench, dut, tty, &rung, &plan, &args.traffic, 0.0) {
        Ok((_, run)) => {
            staged.append(&rung, &run, "return-priority", clamp);
            Some(run)
        }
        Err(e) => {
            eprintln!("    the return-priority window errored ({e:#})");
            staged.fail(e);
            None
        }
    }
}

fn full_sweep(
    rig: &mut Rig,
    kept: &[Rung],
    args: &LadderArgs,
    bus: &BusBudget,
    plan_at: impl Fn(&Rung) -> Plan,
    phase: &'static str,
) -> Result<()> {
    println!("full sweep over {} rungs:", kept.len());
    let mut rows: Vec<(&Rung, Option<Run>)> = Vec::new();
    for rung in kept {
        eprintln!("  rung {}:", rung.nominal);
        let plan = plan_at(rung);
        let clamp = budget::clamp(&plan.demand(args.traffic.dap.nominal_bps()), bus);
        let run = match rig.window(
            rung,
            &plan,
            &args.traffic,
            args.windows.warmup,
            phase,
            Some(clamp),
        ) {
            Ok(run) => Some(run),
            Err(error) if outcomes::rate_declined(&error) => {
                eprintln!("    rate declined ({error:#}); counted as FAIL");
                None
            }
            Err(error) => return Err(error),
        };
        rows.push((rung, run));
    }
    println!(
        "  {:>10}  {:>7}  {:>13}  {:>13}  {:>9}  {:>9}",
        "baud", "verdict", "host -> DUT", "DUT -> host", "out B/s", "back B/s"
    );
    for (rung, run) in &rows {
        let Some(run) = run else {
            println!(
                "  {:>10}  {:>7}  {:>13}  {:>13}  {:>9}  {:>9}",
                rung.nominal, "errored", "--", "--", "--", "--"
            );
            continue;
        };
        let frac = |dir| run.budget_frac(dir) * 100.0;
        let bps = |dir| run.goodput_bps(dir);
        println!(
            "  {:>10}  {:>7}  {:>12.1}%  {:>12.1}%  {:>9.0}  {:>9.0}",
            rung.nominal,
            verdict_word(run.verdict().pass),
            frac(Direction::HostToDut),
            frac(Direction::DutToHost),
            bps(Direction::HostToDut),
            bps(Direction::DutToHost),
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn held(run: Run, phase: &'static str) -> super::super::session::Held {
        super::super::session::Held {
            rung: rung(115200),
            run,
            phase,
            clamp: None,
        }
    }

    #[test]
    fn run_batch_finalization_grades_only_measured_windows_and_preserves_append_errors() {
        let clean = run_at(&rung(115200), 1.0, 1.0);
        let mut failed = clean.clone();
        failed.stuck = 1;
        for strict in [false, true] {
            for last in [clean.clone(), failed.clone()] {
                let all_pass = last.verdict().pass;
                let mut windows = [
                    held(failed.clone(), "warmup"),
                    held(clean.clone(), "qualify"),
                    held(last, "qualify"),
                ];
                let result =
                    crate::command::finish_runs(Ok(()), Ok(None), &mut windows, strict, 2, |_| {
                        Ok(())
                    });
                assert_eq!(result.is_ok(), !strict || all_pass);
            }
            for fail_at in 0..2 {
                let mut windows = [held(clean.clone(), "warmup"), held(clean.clone(), "run")];
                let mut writes = 0;
                let result =
                    crate::command::finish_runs(Ok(()), Ok(None), &mut windows, strict, 1, |_| {
                        let index = writes;
                        writes += 1;
                        outcomes::recorded(
                            if index == fail_at {
                                Err(std::io::Error::other("injected append"))
                            } else {
                                Ok(())
                            },
                            std::path::Path::new("records"),
                        )
                    });
                assert!(result.is_err());
                assert_eq!(writes, 2);
            }
        }
    }

    #[test]
    fn required_final_counter_failure_emits_no_buffered_pass_records() {
        let mut windows = [held(run_at(&rung(115200), 1.0, 1.0), "run")];
        let mut writes = 0;
        let error = crate::command::finish_runs(
            Err(anyhow::anyhow!("primary transport")),
            Err(anyhow::anyhow!("counter transport")),
            &mut windows,
            false,
            1,
            |_| {
                writes += 1;
                Ok(())
            },
        )
        .unwrap_err();
        assert_eq!(writes, 0);
        assert!(format!("{error:#}").contains("counter transport"));
        assert_eq!(error.root_cause().to_string(), "primary transport");
    }

    #[test]
    fn full_sweep_recording_seam_propagates_warmup_and_measured_append_failures() {
        let clean = run_at(&rung(115200), 1.0, 1.0);
        for failed_phase in ["warmup", "discovery"] {
            let result = super::super::session::record_windows(
                Some(clean.clone()),
                clean.clone(),
                "discovery",
                |_, phase| {
                    outcomes::recorded(
                        if phase == failed_phase {
                            Err(std::io::Error::other("injected append"))
                        } else {
                            Ok(())
                        },
                        std::path::Path::new("records"),
                    )
                },
            );
            assert!(result.is_err());
        }
    }

    #[test]
    fn every_recorded_phase_propagates_unwritable_destination() {
        let bench =
            crate::command::tests::bench(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")));
        let run = run_at(&rung(115200), 1.0, 1.0);
        for verb in ["run", "ladder"] {
            for phase in [
                "warmup",
                "run",
                "qualify",
                "soak",
                "sweep",
                "screen",
                "sustain",
                "return-priority",
            ] {
                let error = bench
                    .append_v1(&record::V1Record {
                        probe: &bench.selector,
                        firmware: bench.firmware(),
                        verb,
                        phase,
                        baud_nominal: 115200,
                        probe_realized: Some(115200),
                        run: &run,
                        clamp: None,
                        note: None,
                    })
                    .unwrap_err();
                assert!(error.downcast_ref::<outcomes::RecordingError>().is_some());
            }
        }
        assert!(outcomes::recorded(
            record::append_line(&bench.record_path, "{}"),
            &bench.record_path
        )
        .is_err());
        let default = crate::command::tests::bench(record::default_path());
        assert!(outcomes::recorded(
            Err(std::io::Error::other("default-path failure")),
            &default.record_path
        )
        .is_err());
    }

    #[test]
    fn strict_is_opt_in_and_full_sweep_is_exploratory() {
        use clap::Parser;
        for strict in [false, true] {
            let mut arguments = vec![
                "vcom-bench",
                "run",
                "--probe",
                "0d28:0204:test",
                "--baud",
                "115200",
                "--runs",
                "0",
            ];
            if strict {
                arguments.push("--strict");
            }
            let Command::Run(args) = Cli::try_parse_from(arguments).unwrap().command else {
                panic!("run");
            };
            assert_eq!(args.strict, strict);
            assert_eq!(args.windows.resolve(1.0, "run").1, 1);
            let mut arguments = vec!["vcom-bench", "ladder", "--probe", "0d28:0204:test"];
            if strict {
                arguments.push("--strict");
            }
            let Command::Ladder(args) = Cli::try_parse_from(arguments).unwrap().command else {
                panic!("ladder");
            };
            assert_eq!(args.strict, strict);
        }
        assert!(Cli::try_parse_from([
            "vcom-bench",
            "ladder",
            "--probe",
            "0d28:0204:test",
            "--strict",
            "--full"
        ])
        .is_err());
    }

    #[test]
    fn certification_does_not_promote_qualified_or_search_passes() {
        let (_, _, mut outcome) = pairing();
        assert!(outcomes::ladder(true, outcome.sustained.is_some(), Some(true)).is_ok());
        outcome.sustained = None;
        outcome.sustained_qualified = Some(1);
        assert!(outcomes::ladder(true, outcome.sustained.is_some(), Some(true)).is_err());
        assert!(outcomes::ladder(false, outcome.sustained.is_some(), Some(false)).is_ok());
    }

    #[test]
    fn window_exit_policy_uses_existing_verdict_for_every_mode() {
        for shape in [
            Shape::Duplex,
            Shape::ReturnPriority,
            Shape::Credit,
            Shape::Out,
            Shape::Back,
        ] {
            let mut run = run_at(&rung(115200), 1.0, 1.0);
            run.plan.shape = shape;
            let passed = u32::from(run.verdict().pass);
            assert!(outcomes::run(false, passed, 1).is_ok());
            assert_eq!(outcomes::run(true, passed, 1).is_ok(), run.verdict().pass);
            run.stuck = 1;
            assert!(!run.verdict().pass);
            assert!(outcomes::run(true, u32::from(run.verdict().pass), 1).is_err());
        }
    }
    use crate::budget::{DapAchieved, DapProfile};
    use crate::cli::{Cli, Command, SUSTAIN_SECS, SWEEP_SECS};
    use crate::command::tests::rung;
    use crate::command::Report;
    use crate::counters::VcomCounters;
    use crate::lattice::ProbeResult;
    use crate::lattice::StagedOutcome;
    use crate::stream::{Abandon, Checkpoint, Seeds, Series, Slice};
    use clap::Parser;
    use daplink_test_protocol::v1::{RxCounters, TxCounters, V1Report};
    use std::time::Duration;

    /// A window that delivered `frac` of its payload budget in both directions.
    fn run_at(rung: &Rung, secs: f64, frac: f64) -> Run {
        let plan = Plan {
            shape: Shape::Duplex,
            seeds: Seeds::default(),
            interval: stream::DEFAULT_INTERVAL,
            secs,
            dut_baud: rung.dut_realized,
            probe_baud: rung.dut_realized,
            host_frac: 1.0,
            burst: None,
            burst_host: false,
            burst_dut: false,
            credit_window: stream::CREDIT_WINDOW,
        };
        let moved = (f64::from(plan.budget_d2h()) * frac) as u32;
        let frames = moved / plan.interval + 1;
        let report = V1Report {
            tx: moved,
            tx_frames: frames,
            verified: moved,
            rx_frames: frames,
            eos: true,
            ..V1Report::default()
        };
        let host_rx = RxCounters {
            verified: u64::from(moved),
            frames,
            eos: true,
            eos_total: u64::from(moved),
            ..RxCounters::default()
        };
        let host_tx = TxCounters {
            payload: u64::from(moved),
            frames,
            credit: 0,
        };
        Run {
            legs: stream::legs(&plan, Some(host_tx), Some(host_rx), &report),
            plan,
            ferr: 0,
            ovr: 0,
            drop: 0,
            stuck: 0,
            measured_secs: secs + 1.0,
            timed_out: false,
            closed: false,
            abandoned: None,
            series: Series::default(),
            dap: DapAchieved::default(),
            credit: None,
            conservation: None,
        }
    }

    /// A checkpoint series in which both directions carried `fracs`.
    fn series(fracs: &[f64]) -> Series {
        Series {
            period: 5.0,
            points: fracs
                .iter()
                .enumerate()
                .map(|(i, &frac)| Checkpoint {
                    at_secs: (i + 1) as f64 * 5.0,
                    h2d: Some(Slice {
                        frac: 1.0,
                        ..Slice::default()
                    }),
                    d2h: Some(Slice {
                        frac,
                        ..Slice::default()
                    }),
                })
                .collect(),
        }
    }

    fn row(stage: Stage, rung: Rung, run: Run) -> Row {
        Row {
            stage,
            rung,
            outcome: Ok(run),
            conservation: None,
            recovered: false,
        }
    }

    fn pairing() -> (Vec<Rung>, Vec<Row>, StagedOutcome) {
        let kept = vec![rung(230_400), rung(921_600), rung(1_500_000)];
        let mut boundary = run_at(&kept[2], 60.0, 12.4 / 60.0);
        boundary.series = series(&[1.0, 0.71]);
        boundary.abandoned = Some(Abandon {
            at_secs: 12.4,
            checkpoint: 2,
            reason: "DUT -> host: 0 corrupt, 41216 missing, 0 duplicate, 0 unresolved bytes in \
                     this checkpoint"
                .to_string(),
        });
        let mut held = run_at(&kept[1], 60.0, 1.0);
        held.series = series(&[1.0, 1.0, 0.999]);
        let rows = vec![
            row(Stage::Sweep, kept[0].clone(), run_at(&kept[0], 2.0, 1.0)),
            row(Stage::Sweep, kept[1].clone(), run_at(&kept[1], 2.0, 1.0)),
            row(Stage::Sweep, kept[2].clone(), run_at(&kept[2], 2.0, 1.0)),
            row(Stage::Screen, kept[2].clone(), run_at(&kept[2], 10.0, 1.0)),
            row(Stage::Sustain, kept[2].clone(), boundary),
            row(Stage::Sustain, kept[1].clone(), held),
        ];
        let outcome = StagedOutcome {
            sustained_qualified: None,
            message: Some(2),
            message_gap: false,
            swept: Some(2),
            screened: Some(2),
            sustained: Some(1),
            trace: vec![(
                Stage::Sustain,
                ProbeResult {
                    index: 1,
                    pass: true,
                },
            )],
            non_monotone: false,
            sweep_pessimistic: false,
            screen_exhausted: false,
        };
        (kept, rows, outcome)
    }

    fn ledger(written: u64, first_divergence: Option<u64>) -> Ledger {
        Ledger {
            written,
            read: first_divergence.unwrap_or(written),
            byte_exact: first_divergence.is_none(),
            first_divergence,
            elapsed: Duration::from_millis(3),
            throughput_bps: 0.0,
            head: Vec::new(),
        }
    }

    fn message_row(rung: Rung, first_divergence: Option<u64>, ovr: u32) -> MessageRow {
        MessageRow {
            rung,
            outcome: Ok(Message {
                line: ledger(MESSAGE_BYTES as u64, None),
                keystrokes: ledger(MESSAGE_BYTES as u64, first_divergence),
                stats: Some(daplink_test_protocol::Stats {
                    rx: 128,
                    tx: 128,
                    ovr,
                    ..daplink_test_protocol::Stats::default()
                }),
            }),
        }
    }

    fn messages(kept: &[Rung]) -> Vec<MessageRow> {
        vec![
            message_row(kept[0].clone(), None, 0),
            message_row(kept[1].clone(), None, 0),
            message_row(kept[2].clone(), None, 0),
        ]
    }

    fn report<'a>(
        kept: &'a [Rung],
        rows: &'a [Row],
        messages: &'a [MessageRow],
        outcome: &'a StagedOutcome,
    ) -> Report<'a> {
        Report {
            probe: "0d28:0204:AA",
            firmware: record::tests::FIRMWARE,
            chip: "STM32F401CEUx",
            port: "/dev/ttyACM0",
            records: "artifacts/vcom-bench.jsonl".to_string(),
            dut_max: 2_625_000,
            dap: DapProfile::None,
            interval: stream::DEFAULT_INTERVAL,
            sweep_secs: SWEEP_SECS,
            candidates: 5,
            mismatched: 1,
            clamped: 1,
            kept,
            rows,
            messages,
            outcome,
            rp: &None,
        }
    }

    #[test]
    fn the_report_states_both_ceilings_and_what_the_rungs_between_them_did() {
        let (kept, rows, outcome) = pairing();
        let text = report(&kept, &rows, &messages(&kept), &outcome).render();
        for needle in [
            "short-message ceiling: 1500000 nominal",
            "stream sweep ceiling: 1500000 nominal",
            "sustained ceiling:   921600 nominal",
            "return priority at the ceiling: not measured",
            "lattice: 5 rates asked, 3 kept (1 rate mismatch, 1 over the bus budget)",
            "abandoned at checkpoint 2 (12.4s)",
            "held 12.4s of 60.0s",
            "t=  10.0s",
            "DUT -> host: declining",
            "probe counters: not served",
            "records: artifacts/vcom-bench.jsonl",
        ] {
            assert!(text.contains(needle), "{text}\nis missing {needle}");
        }
        assert_eq!(
            text.lines().filter(|l| l.contains("  sweep ")).count(),
            3,
            "{text}"
        );
    }

    #[test]
    fn a_stuck_dma_abort_fails_a_byte_exact_message_row() {
        let (kept, rows, outcome) = pairing();
        let mut msgs = messages(&kept);
        let Ok(message) = &mut msgs[2].outcome else {
            unreachable!("the fixture row is Ok");
        };
        message.stats.as_mut().expect("the fixture has stats").stuck = 1;
        assert!(message.exact() && !message.pass());
        let text = report(&kept, &rows, &msgs, &outcome).render();
        let row = text
            .lines()
            .find(|l| l.trim_start().starts_with("1500000  "))
            .expect("the 1500000 message row");
        assert!(row.contains("FAIL"), "{row}");
        let json = summary_json(&report(&kept, &rows, &msgs, &outcome));
        assert_eq!(json["messages"][2]["pass"], false);
        assert_eq!(json["messages"][2]["stuck"], 1);
    }

    fn summary_json(report: &Report) -> serde_json::Value {
        let cli = Cli::parse_from(["vcom-bench", "ladder", "--probe", "0d28:0204:AA"]);
        let Command::Ladder(args) = cli.command else {
            unreachable!()
        };
        serde_json::from_str(&staged_summary(report, &args, SUSTAIN_SECS, 0))
            .expect("the complete summary is JSON")
    }

    #[test]
    fn summary_distinguishes_qualified_from_clean_sustained_ceiling() {
        let (kept, rows, mut outcome) = pairing();
        outcome.sustained = None;
        outcome.sustained_qualified = Some(1);
        let json = summary_json(&report(&kept, &rows, &messages(&kept), &outcome));
        assert!(json["sustained_ceiling"].is_null());
        assert_eq!(json["sustained_qualified"], kept[1].nominal);
    }

    #[test]
    fn complete_summary_corpus() {
        let (kept, mut rows, mut outcome) = pairing();
        outcome.sustained_qualified = Some(1);
        let mut msgs = messages(&kept);
        let cli = Cli::parse_from(["vcom-bench", "ladder", "--probe", "0d28:0204:AA"]);
        let Command::Ladder(args) = cli.command else {
            unreachable!()
        };
        for case in ["summary-clean", "summary-failed", "summary-qualified"] {
            if case == "summary-failed" {
                outcome.message = Some(1);
                outcome.message_gap = true;
                msgs[0] = message_row(kept[0].clone(), Some(0), 0);
                msgs[1].outcome.as_mut().unwrap().stats = None;
                msgs[2]
                    .outcome
                    .as_mut()
                    .unwrap()
                    .stats
                    .as_mut()
                    .unwrap()
                    .stuck = 1;
                msgs.push(MessageRow {
                    rung: rung(2_000_000),
                    outcome: Err("cancelled: \"reader\"\n\\transport".into()),
                });
                rows[0].outcome = Err("setup failed".into());
            }
            if case == "summary-qualified" {
                outcome.sustained = None;
                outcome.sustained_qualified = Some(1);
                let held = rows.last_mut().unwrap().outcome.as_mut().unwrap();
                held.conservation = Some(Conservation {
                    before: VcomCounters::default(),
                    after: VcomCounters {
                        uart_rx: 2,
                        cdc_in: 1,
                        host_dropped: 1,
                        ..VcomCounters::default()
                    },
                });
                assert_eq!(held.grade(), stream::Grade::Qualified);
            }
            let report = report(&kept, &rows, &msgs, &outcome);
            record::tests::assert_record(case, &staged_summary(&report, &args, SUSTAIN_SECS, 0));
        }
    }

    #[test]
    fn summary_message_verdicts_preserve_missing_evidence_and_errors() {
        let (kept, rows, outcome) = pairing();
        let mut msgs = messages(&kept);
        if let Ok(m) = &mut msgs[1].outcome {
            m.stats = None;
        }
        msgs[2].outcome = Err("exchange failed".into());
        let json = summary_json(&report(&kept, &rows, &msgs, &outcome));
        for (index, message) in msgs.iter().enumerate() {
            assert_eq!(
                json["messages"][index]["pass"],
                message.outcome.as_ref().is_ok_and(Message::pass)
            );
        }
        assert_eq!(json["messages"][0]["stuck"], 0);
        assert!(json["messages"][1]["stuck"].is_null());
        assert!(json["messages"][2]["stuck"].is_null());
        msgs[0] = message_row(kept[0].clone(), Some(0), 0);
        let json = summary_json(&report(&kept, &rows, &msgs, &outcome));
        assert_eq!(json["messages"][0]["pass"], false);
    }

    #[test]
    fn the_report_says_plainly_when_there_is_no_ceiling_to_report() {
        let (kept, rows, mut outcome) = pairing();
        outcome.sustained = None;
        let text = report(&kept, &rows, &messages(&kept), &outcome).render();
        assert!(
            text.contains(
                "sustained ceiling:   none -- no rung below the screened boundary held byte-exact"
            ),
            "{text}"
        );
    }

    #[test]
    fn a_non_monotone_lattice_is_called_out_where_the_ceiling_is_read() {
        let (kept, rows, mut outcome) = pairing();
        outcome.non_monotone = true;
        let text = report(&kept, &rows, &messages(&kept), &outcome).render();
        assert!(text.contains("the lattice is not monotone"), "{text}");
    }
}
