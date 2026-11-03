//! The `contract` verb: the VCOM service contract, case by case.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

use super::report::verdict_word;
use super::session::{
    attach_armed, forward_in_flight, prepare_settled, settle_bound, Bench, DisarmOnExit, InFlight,
};
use super::SETTLE;
use crate::budget::{DapAchieved, DapProfile};
use crate::cli::ContractArgs;
use crate::counters::VcomCounters;
use crate::dut::Dut;
use crate::tty::{self, ClaimedCdc, Tty};
use crate::{contract, probe, record};

pub(super) fn contract_verb(args: ContractArgs) -> Result<()> {
    let bench = Bench::resolve(&args.rig)?;
    let mut disarm = DisarmOnExit::new(&bench);
    disarm.finish(contract_on(&bench, args))
}

fn contract_on(bench: &Bench, args: ContractArgs) -> Result<()> {
    use contract::{CaseResult, Latencies};
    let cycles = contract::cycles(args.qualify);
    let mut tty = Tty::open_selected(&bench.selector, args.baud)?;
    let mut failed_cases = 0usize;
    let (counter_baseline, mut dut) = prepare_contract_case(bench, &mut tty, args.baud)?;
    let payload1 = [0x5a];
    let mut unloaded = Latencies::default();
    let mut exact = true;
    for _ in 0..contract::EXCHANGES {
        let at = Instant::now();
        exact &= tty.echo_paced(&payload1, 1, args.baud)?.byte_exact;
        unloaded.push(at.elapsed());
    }
    let mut loaded = Latencies::default();
    let stop = AtomicBool::new(false);
    let dap = std::thread::scope(|scope| -> Result<DapAchieved> {
        let load = scope.spawn(|| dut.dap_load(DapProfile::Representative, &stop));
        for _ in 0..contract::EXCHANGES {
            let at = Instant::now();
            exact &= tty.echo_paced(&payload1, 1, args.baud)?.byte_exact;
            loaded.push(at.elapsed());
        }
        stop.store(true, Ordering::Relaxed);
        load.join()
            .map_err(|_| anyhow!("contract DAP load thread panicked"))?
    })?;
    let stats = dut.stats()?;
    let failure = contract::small_exchange_pass(&unloaded, &loaded)
        .err()
        .or_else(|| (!exact).then(|| "an exchange was not byte-exact".into()))
        .or_else(|| {
            (stats.ferr != 0 || stats.ovr != 0 || stats.drop != 0)
                .then(|| "DUT reported receive loss".into())
        })
        .or_else(|| {
            (stats.stuck != 0).then(|| "DUT latched a stuck DMA abort; counters suspect".into())
        })
        .or_else(|| (dap.commands == 0).then(|| "DAP made no progress".into()));
    let mut small = CaseResult {
        case: "small-exchange",
        tier: None,
        baud: args.baud,
        exact,
        counter_classification: None,
        dap_progress: dap.commands > 0,
        recovery: None,
        latencies: loaded,
        pass: failure.is_none(),
        failure,
    };
    drop(dut);
    finish_contract_case(bench, counter_baseline, &mut small)?;
    failed_cases += usize::from(!small.pass);

    let (counter_baseline, mut dut) = prepare_contract_case(bench, &mut tty, args.baud)?;

    let burst = tty::pattern(2048);
    let first = tty.echo_batched(&burst, 64, args.baud)?;
    std::thread::sleep(if args.qualify {
        contract::QUALIFY_IDLE
    } else {
        contract::IDLE
    });
    let at = Instant::now();
    let second = tty.echo_batched(&burst, 64, args.baud)?;
    let recovery = at.elapsed();
    dut.ping()?;
    let exact = first.byte_exact && second.byte_exact;
    let mut idle = CaseResult {
        case: "idle-to-burst",
        tier: args.qualify.then(|| "five-minute".into()),
        baud: args.baud,
        exact,
        counter_classification: None,
        dap_progress: true,
        recovery: Some(recovery),
        latencies: Latencies::default(),
        pass: exact,
        failure: (!exact).then(|| "one of the two 2048-byte bursts was not exact".into()),
    };
    drop(dut);
    finish_contract_case(bench, counter_baseline, &mut idle)?;
    failed_cases += usize::from(!idle.pass);

    for pause in contract::PAUSES {
        let (counter_baseline, mut dut) = prepare_contract_case(bench, &mut tty, args.baud)?;
        let at = Instant::now();
        let ledger = tty.echo_after_reader_pause(&burst, pause, args.baud)?;
        let recovery = at.elapsed().saturating_sub(pause);
        dut.ping()?;
        let stats = dut.stats()?;
        let attributed = stats.ferr != 0 || stats.ovr != 0 || stats.drop != 0 || !ledger.byte_exact;
        let failure = contract::pause_pass(pause, ledger.byte_exact, ledger.read > 0, attributed)
            .err()
            .or_else(|| {
                (stats.stuck != 0).then(|| "DUT latched a stuck DMA abort; counters suspect".into())
            });
        let mut result = CaseResult {
            case: "reader-pause",
            tier: Some(format!("{}ms", pause.as_millis())),
            baud: args.baud,
            exact: ledger.byte_exact,
            counter_classification: None,
            dap_progress: true,
            recovery: Some(recovery),
            latencies: Latencies::default(),
            pass: failure.is_none(),
            failure,
        };
        drop(dut);
        finish_contract_case(bench, counter_baseline, &mut result)?;
        failed_cases += usize::from(!result.pass);
    }

    let (counter_baseline, dut) = prepare_contract_case(bench, &mut tty, args.baud)?;
    let mut dut = Some(dut);
    drop(tty);
    let mut cdc = ClaimedCdc::open(&bench.selector)?;
    let mut line_latencies = Latencies::default();
    let mut line_exact = true;
    let mut line_failure = None;
    let mut line_dap_commands = 0usize;
    'cycles: for cycle in 1..=cycles {
        for baud in contract::LINE_CODING {
            let at = Instant::now();
            let Some(dut_realized) = dut
                .as_mut()
                .expect("the target session is attached between transitions")
                .set_baud(baud)?
            else {
                line_failure = Some(format!(
                    "line-coding cycle {cycle} rate {baud}: DUT declined the rate"
                ));
                break 'cycles;
            };
            if let Err(stop) =
                line_coding_cycle(bench, &mut cdc, &mut dut, cycle, baud, dut_realized)
            {
                line_exact &= !stop.inexact;
                line_failure = Some(stop.failure);
                break 'cycles;
            }
            line_dap_commands += 1;
            line_latencies.push(at.elapsed());
        }
    }
    drop(dut);
    let mut line = CaseResult {
        case: "line-coding",
        tier: Some(format!("{cycles}-cycles")),
        baud: args.baud,
        exact: line_exact,
        counter_classification: None,
        dap_progress: line_dap_commands == cycles * contract::LINE_CODING.len(),
        recovery: line_latencies.max_us().map(Duration::from_micros),
        latencies: line_latencies,
        pass: line_exact && line_failure.is_none(),
        failure: line_failure.or_else(|| {
            (!line_exact).then(|| "an exchange after a line-coding transition was not exact".into())
        }),
    };
    finish_contract_case(bench, counter_baseline, &mut line)?;
    failed_cases += usize::from(!line.pass);

    drop(cdc);
    let mut tty = Tty::open_selected(&bench.selector, args.baud)?;

    let (counter_baseline, mut dut) = prepare_contract_case(bench, &mut tty, args.baud)?;
    let mut reopen_latencies = Latencies::default();
    let mut reopen_exact = true;
    for _ in 0..cycles {
        tty.set_open(false)?;
        let at = Instant::now();
        tty.set_open(true)?;
        std::thread::sleep(SETTLE);
        reopen_exact &= tty.echo_paced(&tty::pattern(64), 64, args.baud)?.byte_exact;
        dut.ping()?;
        reopen_latencies.push(at.elapsed());
    }
    let mut reopen = CaseResult {
        case: "cdc-open-close",
        tier: Some(format!("{cycles}-cycles")),
        baud: args.baud,
        exact: reopen_exact,
        counter_classification: None,
        dap_progress: true,
        recovery: reopen_latencies.max_us().map(Duration::from_micros),
        latencies: reopen_latencies,
        pass: reopen_exact,
        failure: (!reopen_exact).then(|| "an exchange after CDC reopen was not exact".into()),
    };
    drop(dut);
    finish_contract_case(bench, counter_baseline, &mut reopen)?;
    failed_cases += usize::from(!reopen.pass);
    contract::suite_pass(failed_cases).map_err(anyhow::Error::msg)
}

/// Why a line-coding cycle ended the case, and whether its echo came back inexact.
struct LineStop {
    failure: String,
    inexact: bool,
}

/// One accepted transition: scrub rate, then `baud`, each verified, then an echo and a DAP command.
fn line_coding_cycle(
    bench: &Bench,
    cdc: &mut ClaimedCdc,
    dut: &mut Option<Dut>,
    cycle: usize,
    baud: u32,
    dut_realized: u32,
) -> std::result::Result<(), LineStop> {
    let failed = |failure| LineStop {
        failure,
        inexact: false,
    };
    let stage = move |name: &'static str| {
        move |error: anyhow::Error| LineStop {
            failure: contract::line_runtime_failure(cycle, baud, name, format_args!("{error:#}")),
            inexact: false,
        }
    };
    let scrub = ClaimedCdc::scrub_baud(baud);
    cdc.set_baud(scrub).map_err(stage("scrub control"))?;
    drop(dut.take());
    std::thread::sleep(contract::LINE_CONTROL_QUIET);
    settle_contract_line_baud(bench, cycle, baud, contract::LineStage::Scrub, scrub)
        .map_err(stage("scrub verify"))?
        .map_err(failed)?;
    cdc.set_baud(baud).map_err(stage("target control"))?;
    std::thread::sleep(contract::LINE_CONTROL_QUIET);
    settle_contract_line_baud(
        bench,
        cycle,
        baud,
        contract::LineStage::Target,
        dut_realized,
    )
    .map_err(stage("target verify"))?
    .map_err(failed)?;
    let session = Dut::open(&bench.selector, bench.chip, bench.dut_elf.as_deref())
        .map_err(stage("target reattach"))?;
    let session = dut.insert(session);
    let ledger = cdc.echo(&tty::pattern(64)).map_err(stage("bulk echo"))?;
    if !ledger.byte_exact {
        return Err(LineStop {
            failure: contract::line_echo_failure(
                cycle,
                baud,
                ledger.written,
                ledger.read,
                ledger.first_divergence,
                &ledger.head,
            ),
            inexact: true,
        });
    }
    session.dap_command().map_err(stage("DAP"))?;
    Ok(())
}

fn settle_contract_line_baud(
    bench: &Bench,
    cycle: usize,
    requested: u32,
    stage: contract::LineStage,
    expected: u32,
) -> Result<Result<u32, String>> {
    let mut link = probe::DapLink::open(&bench.selector, bench.vendor)?;
    let mut wait: Option<(Instant, Duration, InFlight)> = None;
    loop {
        let reading = link.vcom_baud()?;
        // An unserved query never settles; say so at once instead of burning the timeout.
        if reading.is_none() {
            return Ok(Err(format!(
                "line-coding cycle {cycle} rate {requested} {} stage: this probe serves no \
                 realized-baud query, so the transition cannot be verified",
                stage.key()
            )));
        }
        // A reading taken mid-transaction describes the rate being left, not the transition.
        let in_flight = reading.is_some_and(|b| b.settled == Some(false));
        let realized = reading.and_then(|b| b.realized);
        let failure = contract::line_transition_failure(
            cycle, requested, stage, expected, realized, in_flight,
        );
        if failure.is_none() {
            return Ok(Ok(
                realized.expect("a successful transition has a realized baud")
            ));
        }
        let (deadline, bound, queued) = match wait {
            Some(pending) => pending,
            None => {
                // A transition behind queued data waits for that data at the rate it is leaving.
                let queued = link
                    .vcom_counters()?
                    .map_or(InFlight::Unknown, |c| forward_in_flight(&c));
                let bound = settle_bound(contract::LINE_SETTLE_TIMEOUT, queued, realized);
                *wait.insert((Instant::now() + bound, bound, queued))
            }
        };
        if Instant::now() >= deadline {
            return Ok(Err(contract::line_settle_expiry(
                &failure.expect("an unsettled transition has a failure"),
                bound,
                &queued.clause(),
            )));
        }
        std::thread::sleep(contract::LINE_SETTLE_POLL);
    }
}

fn prepare_contract_case(
    bench: &Bench,
    tty: &mut Tty,
    baud: u32,
) -> Result<(Option<VcomCounters>, Dut)> {
    let mut slot = None;
    prepare_settled(bench, &mut slot, tty, baud)?;
    let before = bench.probe_counters()?;
    attach_armed(bench, &mut slot)?;
    Ok((before, slot.take().expect("the session was just attached")))
}

fn finish_contract_case(
    bench: &Bench,
    before: Option<VcomCounters>,
    result: &mut contract::CaseResult,
) -> Result<()> {
    let after = match bench.probe_counters() {
        Ok(after) => after,
        Err(error) => {
            result.pass = false;
            if result.failure.is_none() {
                result.failure = Some(format!("probe counter read failed: {error:#}"));
            }
            return super::outcomes::combine(Err(error), record_contract(bench, result));
        }
    };
    result.counter_classification = contract::counter_classification(before, after);
    contract::apply_counter_classification(result);
    record_contract(bench, result)
}

fn record_contract(bench: &Bench, result: &contract::CaseResult) -> Result<()> {
    println!("contract {:<16} {}", result.case, verdict_word(result.pass));
    if let Some(reason) = &result.failure {
        println!("  {reason}");
    }
    super::outcomes::recorded(
        record::append_contract(
            &bench.record_path,
            &bench.selector,
            bench.firmware(),
            result,
        ),
        &bench.record_path,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contract_append_failure_is_not_a_measurement_verdict() {
        let bench =
            crate::command::tests::bench(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")));
        let result = contract::CaseResult {
            case: "small-exchange",
            tier: None,
            baud: 115200,
            exact: true,
            counter_classification: None,
            dap_progress: true,
            recovery: None,
            latencies: contract::Latencies::default(),
            pass: true,
            failure: None,
        };
        let error = record_contract(&bench, &result).unwrap_err();
        assert!(error
            .downcast_ref::<super::super::outcomes::RecordingError>()
            .is_some());
    }
}
