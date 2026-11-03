//! Command dispatch, and the `echo` and `run` verbs.

mod ladder;
mod outcomes;
mod plan;
mod report;
mod service;
mod session;
pub(crate) use ladder::Message;
pub(crate) use report::Report;

use std::time::Duration;

use anyhow::{Context, Result};

use crate::cli::{Command, EchoArgs, Mode, RunArgs};
use crate::dut::Dut;
use crate::lattice::Rung;
use crate::{lattice, lifecycle, record, stream, tty};
use report::{
    mode_name, opt_rate, print_conservation, print_ledger, print_run, print_verdict, rung_line,
};
use session::{
    admit, plan_for, prepare_settled, rearm, run_at, windows_of, Bench, Bracket, DisarmOnExit,
    Held, Prepared, StreamWindow,
};
use tty::Tty;

const SETTLE: Duration = Duration::from_millis(150);
const STATS_LAG: Duration = Duration::from_millis(50);
/// How often the silent window checks the clock; it costs no bus traffic.
const WINDOW_POLL: Duration = Duration::from_millis(20);
/// Settle time for a rate change on an idle line; the derived bound adds the drain.
const LATTICE_SETTLE_BASE: Duration = Duration::from_millis(400);
const LATTICE_SETTLE_MAX: Duration = Duration::from_secs(10);
const LATTICE_SETTLE_POLL: Duration = Duration::from_millis(10);

pub(crate) fn run(command: Command) -> Result<()> {
    match command {
        Command::Echo(args) => echo_verb(args),
        Command::Run(args) => run_verb(args),
        Command::Ladder(args) => ladder::ladder_verb(args),
        Command::Contract(args) => service::contract_verb(args),
        Command::Plan(args) => plan::plan_verb(args),
    }
}

fn echo_verb(args: EchoArgs) -> Result<()> {
    let bench = Bench::resolve(&args.rig)?;
    let mut disarm = DisarmOnExit::new(&bench);
    disarm.finish(echo_on(&bench, args))
}

fn echo_on(bench: &Bench, args: EchoArgs) -> Result<()> {
    let mut tty = Tty::open_selected(&bench.selector, args.baud)?;
    let mut dut = None;

    let payload = tty::pattern(args.len);
    let (out, probe_realized) =
        run_at(bench, &mut dut, &mut tty, args.baud, |t| match args.mode {
            Mode::Paced => t.echo_paced(&payload, args.chunk, args.baud),
            Mode::Batched => t.echo_batched(&payload, args.chunk, args.baud),
            Mode::Out => {
                let mut ledger = t.echo_batched(&payload, args.chunk, args.baud)?;
                // Write-only attribution: the echo is drained, not judged.
                ledger.byte_exact = false;
                Ok(ledger)
            }
        })?;

    let mode = mode_name(args.mode);
    println!(
        "echo {} baud {} (DUT realized {}, probe realized {}) len {} chunk {}",
        mode,
        args.baud,
        out.dut_realized,
        opt_rate(probe_realized),
        args.len,
        args.chunk
    );
    print_ledger(&out);
    bench.append(&record::RunRecord {
        probe: &bench.selector,
        firmware: bench.firmware(),
        verb: "echo",
        phase: "run",
        baud_nominal: args.baud,
        dut_realized: Some(out.dut_realized),
        probe_realized,
        len: args.len as u64,
        chunk: args.chunk,
        mode,
        secs: 0.0,
        host_written: out.ledger.written,
        host_read: out.ledger.read,
        byte_exact: out.ledger.byte_exact,
        first_divergence: out.ledger.first_divergence,
        throughput_bps: out.ledger.throughput_bps,
        dut_stats: Some(out.stats),
        dap_roundtrips: None,
        pass: (args.mode != Mode::Out).then_some(out.ledger.byte_exact && out.stats.stuck == 0),
        note: None,
    })?;
    Ok(())
}

fn run_verb(args: RunArgs) -> Result<()> {
    let bench = Bench::resolve(&args.rig)?;
    let mut disarm = DisarmOnExit::new(&bench);
    disarm.finish(run_on(&bench, args))
}

fn run_on(bench: &Bench, args: RunArgs) -> Result<()> {
    let mut tty = Tty::open_selected(&bench.selector, args.baud)?;
    let (_, bus) = bench.probe_facts()?;
    let mut dut = None;
    let Prepared {
        dut_realized,
        probe_realized,
    } = prepare_settled(bench, &mut dut, &mut tty, args.baud)?;
    let rung = Rung {
        nominal: args.baud,
        dut_realized,
        probe_realized,
    };
    // A pairing whose divisor lattices land apart misframes every character on the return leg.
    if !rung.agreed(lattice::DEFAULT_BUDGET_PCT) {
        eprintln!(
            "warning: the DUT realized {dut_realized} and the probe {}; the ends disagree by more \
             than the divisor budget, so this rate is not one the pairing can carry",
            opt_rate(probe_realized)
        );
    }
    let (secs, runs, phase) = args.windows.resolve(stream::DISCOVERY_SECS, "run");
    let plan = plan_for(
        args.mode,
        &args.traffic,
        dut_realized,
        probe_realized,
        secs,
        args.host_frac,
        args.credit_window,
    );
    println!(
        "{} at {} ({:.1}s window, {} run{}, marker interval {} B)",
        plan.shape.key(),
        rung_line(&rung),
        secs,
        runs,
        if runs == 1 { "" } else { "s" },
        plan.interval,
    );
    let clamp = admit(&plan, &bus, &args.traffic)?;

    let mut held = Vec::new();
    let (outcome, conservation) = lifecycle::run(&mut StreamWindow {
        bench,
        dut: &mut dut,
        tty: &mut tty,
        rung: &rung,
        prepared: Some(Prepared {
            dut_realized,
            probe_realized,
        }),
        bracket: Bracket::Required,
        body: |dut: &mut Dut, tty: &mut Tty| {
            for i in 0..runs {
                if i > 0 {
                    rearm(dut, tty)?;
                }
                if runs > 1 {
                    println!("run {} of {runs}:", i + 1);
                }
                let (warmed, run) =
                    windows_of(dut, tty, &plan, &args.traffic, args.windows.warmup)?;
                if let Some(run) = warmed {
                    held.push(Held {
                        rung: rung.clone(),
                        run,
                        phase: "warmup",
                        clamp: Some(clamp),
                    });
                }
                print_run(&run);
                held.push(Held {
                    rung: rung.clone(),
                    run,
                    phase,
                    clamp: Some(clamp),
                });
            }
            Ok(())
        },
    });
    finish_runs(outcome, conservation, &mut held, args.strict, runs, |h| {
        bench.append_v1(&record::V1Record {
            probe: &bench.selector,
            firmware: bench.firmware(),
            verb: "run",
            phase: h.phase,
            baud_nominal: h.rung.nominal,
            probe_realized: h.rung.probe_realized,
            run: &h.run,
            clamp: h.clamp,
            note: None,
        })
    })
}

fn finish_runs(
    outcome: Result<()>,
    conservation: Result<Option<crate::counters::Conservation>>,
    held: &mut [Held],
    strict: bool,
    runs: u32,
    mut append: impl FnMut(&Held) -> Result<()>,
) -> Result<()> {
    let (conservation, cleanup) = match conservation.context("final probe counter read") {
        Ok(conservation) => (conservation, Ok(())),
        Err(error) => (None, Err(error)),
    };
    if cleanup.is_err() {
        return outcomes::combine(outcome, cleanup);
    }
    print_conservation(conservation.as_ref());

    let mut passed = 0u32;
    let mut recording = Ok(());
    for h in held {
        // The one bracket covers the whole measured batch, including warmup.
        h.run.conservation = conservation;
        recording = outcomes::combine(recording, append(h));
        if h.phase != "warmup" {
            print_verdict(&h.run);
            passed += u32::from(h.run.verdict().pass);
        }
    }
    println!("{passed} of {runs} windows passed");
    outcomes::combine(
        outcomes::combine(outcomes::combine(outcome, cleanup), recording),
        outcomes::run(strict, passed, runs),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::counters::{self, VcomCounters};
    use crate::lattice::Rung;
    use crate::{contract, probe};
    use session::{
        forward_in_flight, settle_bound, settle_step, stale_denominator, unsettled_note,
        BaudSettle, InFlight, Unsettled,
    };

    pub(super) fn rung(nominal: u32) -> Rung {
        Rung {
            nominal,
            dut_realized: nominal,
            probe_realized: Some(nominal),
        }
    }

    pub(super) fn bench(record_path: std::path::PathBuf) -> Bench {
        Bench {
            selector: "0d28:0204:test".into(),
            firmware_row: firmware_row::resolve(Some("firmware/mculink"), None),
            firmware_version: None,
            chip: "STM32F401RETx",
            port_name: "unused".into(),
            record_path,
            dut_elf: None,
            reset_dut: false,
            vendor: false,
            pending_reset: std::cell::Cell::new(false),
            budget_pct: lattice::DEFAULT_BUDGET_PCT,
            unconfirmed_noted: std::cell::Cell::new(false),
        }
    }

    #[test]
    fn echo_append_failure_is_operational() {
        let bench = bench(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")));
        let error = bench
            .append(&record::RunRecord {
                probe: &bench.selector,
                firmware: bench.firmware(),
                verb: "echo",
                phase: "run",
                baud_nominal: 115200,
                dut_realized: Some(115200),
                probe_realized: Some(115200),
                len: 0,
                chunk: 64,
                mode: "batched",
                secs: 0.0,
                host_written: 0,
                host_read: 0,
                byte_exact: true,
                first_divergence: None,
                throughput_bps: 0.0,
                dut_stats: None,
                dap_roundtrips: None,
                pass: Some(true),
                note: None,
            })
            .unwrap_err();
        assert!(error.downcast_ref::<outcomes::RecordingError>().is_some());
    }

    fn reading(realized: u32, applied: Option<u32>, settled: Option<bool>) -> probe::VcomBaud {
        probe::VcomBaud {
            realized: (realized != 0).then_some(realized),
            applied,
            settled,
        }
    }

    #[test]
    fn a_change_in_flight_never_concludes_the_wait() {
        // The probe is still at the old rate, which happens to be the one this poll asked for.
        assert_eq!(
            settle_step(reading(115_207, Some(115_200), Some(false)), 115_200, 2.0),
            None
        );
        // And at the new one, which it has not applied.
        assert_eq!(
            settle_step(
                reading(2_000_000, Some(115_200), Some(false)),
                2_000_000,
                2.0
            ),
            None
        );
    }

    #[test]
    fn an_applied_change_concludes_the_wait_agreeing_or_not() {
        assert_eq!(
            settle_step(
                reading(2_000_000, Some(2_000_000), Some(true)),
                2_000_000,
                2.0
            ),
            Some(BaudSettle::Settled(2_000_000))
        );
        // Applied, and the divisor does not reach it: a determination, not an expiry.
        assert_eq!(
            settle_step(
                reading(1_500_000, Some(2_000_000), Some(true)),
                2_000_000,
                2.0
            ),
            Some(BaudSettle::Disagreed {
                realized: 1_500_000
            })
        );
    }

    #[test]
    fn a_rate_the_probe_kept_another_for_is_declined_not_rated() {
        assert_eq!(
            settle_step(reading(9_600, Some(9_600), Some(true)), 6_000_000, 2.0),
            Some(BaudSettle::Declined {
                applied: Some(9_600),
                realized: 9_600
            })
        );
        assert_eq!(
            BaudSettle::Declined {
                applied: None,
                realized: 9_600
            }
            .realized(),
            None
        );
    }

    #[test]
    fn without_applied_ness_agreement_is_the_only_evidence() {
        assert_eq!(
            settle_step(reading(921_600, None, None), 921_600, 2.0),
            Some(BaudSettle::Settled(921_600))
        );
        assert_eq!(
            settle_step(reading(115_207, None, None), 921_600, 2.0),
            None
        );
    }

    /// An expiry is a graded outcome quoting no rate, so no leg can be rated against a stale one.
    #[test]
    fn an_expiry_quotes_no_rate_and_says_why() {
        let expiry = Unsettled {
            last: Some(115_207),
            applied_sourced: true,
            bound: Duration::from_millis(2_530),
            queued: InFlight::Bytes(2_047),
        };
        assert_eq!(BaudSettle::Unsettled(expiry).realized(), None);
        let note = unsettled_note(2_000_000, expiry);
        assert!(note.contains("did not settle at 2000000"), "{note}");
        assert!(note.contains("within 2530 ms"), "{note}");
        assert!(note.contains("change in flight"), "{note}");
        assert!(note.contains("115207"), "{note}");
        assert!(note.contains("2047 bytes were still queued"), "{note}");

        let inferred = unsettled_note(
            2_000_000,
            Unsettled {
                last: None,
                applied_sourced: false,
                bound: LATTICE_SETTLE_BASE,
                queued: InFlight::Unknown,
            },
        );
        assert!(
            inferred.contains("does not report applied-ness"),
            "{inferred}"
        );
        assert!(inferred.contains("no realized rate"), "{inferred}");
        assert!(
            inferred.contains("does not report what it has queued"),
            "{inferred}"
        );
    }

    fn counters(valid: u32, cdc_out: u32, uart_tx: u32) -> VcomCounters {
        VcomCounters {
            cdc_out,
            uart_tx,
            valid: Some(valid),
            ..VcomCounters::default()
        }
    }

    /// The forward ledger's deficit is the queue; a legacy reply's zeroes are silence, not rest.
    #[test]
    fn the_in_flight_count_comes_from_the_forward_ledger_or_says_it_cannot() {
        let both = (1 << counters::SLOT_CDC_OUT) | (1 << counters::SLOT_UART_TX);
        assert_eq!(
            forward_in_flight(&counters(both, 5_000, 3_500)),
            InFlight::Bytes(1_500)
        );
        assert_eq!(forward_in_flight(&counters(0, 0, 0)), InFlight::Unknown);
        assert_eq!(
            forward_in_flight(&VcomCounters::default()),
            InFlight::Unknown
        );
    }

    #[test]
    fn a_confirmed_rate_that_contradicts_the_lattice_is_a_stale_denominator() {
        let rung = |probe_realized| Rung {
            nominal: 2_000_000,
            dut_realized: 2_000_000,
            probe_realized,
        };
        let prepared = |probe_realized| Prepared {
            dut_realized: 2_000_000,
            probe_realized,
        };
        let budget = lattice::DEFAULT_BUDGET_PCT;
        assert!(!stale_denominator(
            &prepared(Some(2_000_000)),
            &rung(Some(2_000_000)),
            budget
        ));
        // A percent apart is inside the divisor budget; a leg pinned at the previous rung is not.
        assert!(!stale_denominator(
            &prepared(Some(1_980_000)),
            &rung(Some(2_000_000)),
            budget
        ));
        assert!(stale_denominator(
            &prepared(Some(9_600)),
            &rung(Some(2_000_000)),
            budget
        ));
        // A probe serving no realized rate leaves nothing to contradict.
        assert!(!stale_denominator(
            &prepared(None),
            &rung(Some(2_000_000)),
            budget
        ));
        assert!(!stale_denominator(
            &prepared(Some(2_000_000)),
            &rung(None),
            budget
        ));
    }

    #[test]
    fn the_bound_covers_the_queue_at_the_rate_it_drains_at() {
        let base = LATTICE_SETTLE_BASE;
        // 2047 bytes at 9600 baud take 2.13 s.
        assert_eq!(
            settle_bound(base, InFlight::Bytes(2_047), Some(9_600)),
            base + Duration::from_secs_f64(20_470.0 / 9_600.0)
        );
        // The same queue at 4 Mbaud costs 5 ms, so the base is what bounds it.
        assert!(
            settle_bound(base, InFlight::Bytes(2_047), Some(4_000_000))
                < base + Duration::from_millis(6)
        );
        // A quiesced line derives the base alone.
        assert_eq!(settle_bound(base, InFlight::Bytes(0), Some(9_600)), base);
        // Nothing to derive from is the base, not an unbounded wait.
        assert_eq!(settle_bound(base, InFlight::Unknown, Some(9_600)), base);
        assert_eq!(settle_bound(base, InFlight::Bytes(2_047), None), base);
        assert_eq!(settle_bound(base, InFlight::Bytes(2_047), Some(0)), base);
        // A deficit sampled across a change can wrap; the cap is what keeps that finite.
        assert_eq!(
            settle_bound(base, InFlight::Bytes(u32::MAX), Some(9_600)),
            LATTICE_SETTLE_MAX
        );
    }

    /// The contract case takes the same derivation from its own quiet-line base.
    #[test]
    fn a_quiet_line_coding_transition_keeps_the_contract_base() {
        let base = contract::LINE_SETTLE_TIMEOUT;
        assert_eq!(settle_bound(base, InFlight::Bytes(0), Some(9_600)), base);
        assert_eq!(settle_bound(base, InFlight::Unknown, Some(9_600)), base);
        assert_eq!(settle_bound(base, InFlight::Bytes(2_047), None), base);
    }

    /// A loaded transition is bounded by its own queue, not by the quiet-line figure.
    #[test]
    fn a_loaded_line_coding_transition_waits_for_the_queue_it_leaves() {
        let base = contract::LINE_SETTLE_TIMEOUT;
        let loaded = settle_bound(base, InFlight::Bytes(2_047), Some(9_600));
        assert_eq!(loaded, base + Duration::from_secs_f64(20_470.0 / 9_600.0));
        assert!(loaded > Duration::from_millis(1_910), "{loaded:?}");
        assert!(base < Duration::from_millis(1_910));
        // The backstop still ends the wait when the deficit is nonsense.
        assert_eq!(
            settle_bound(base, InFlight::Bytes(u32::MAX), Some(9_600)),
            LATTICE_SETTLE_MAX
        );
    }

    #[test]
    fn rest_is_something_only_the_forward_ledger_can_attest() {
        assert!(InFlight::Bytes(1).pending());
        assert!(!InFlight::Bytes(0).pending());
        assert!(!InFlight::Unknown.pending());
    }

    #[test]
    fn an_unreported_query_quotes_no_rate_either() {
        assert_eq!(BaudSettle::Unreported.realized(), None);
    }
}
