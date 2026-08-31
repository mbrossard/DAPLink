//! `vcom-bench` -- host-side VCOM lattice, duplex stream, and operating-ceiling search.

mod budget;
mod contract;
mod dut;
mod lattice;
mod probe;
mod record;
mod stream;
mod tty;

use std::cell::Cell;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use clap::{Parser, Subcommand, ValueEnum};
use daplink_test_protocol::v1::BurstSpec;
use daplink_test_protocol::Stats;

use budget::{BusBudget, Clamp, DapAchieved, DapProfile};
use dut::Dut;
use lattice::{Rung, Stage, StagedLimits, StagedOutcome};
use stream::{
    Clock, Conservation, Credit, Direction, Plan, Run, Sampler, Seeds, Shape, VcomCounters,
};
use tty::{ClaimedCdc, Ledger, Tty};

/// The wait a rate change gets where the probe reports no applied-ness at all, and after a CDC
/// reopen. It is a guess, which is why every other path waits on the probe's own word instead.
const SETTLE: Duration = Duration::from_millis(150);
const STATS_LAG: Duration = Duration::from_millis(50);
/// How often the silent window looks at the clock; it costs no bus traffic of any kind.
const WINDOW_POLL: Duration = Duration::from_millis(20);
/// What a rate change costs on a line already at rest; the derived bound adds the drain to it.
const LATTICE_SETTLE_BASE: Duration = Duration::from_millis(400);
/// Ceiling on the derived bound. A counter pair sampled across a rate change can read a deficit
/// that is not one, and no drain of a real transmit ring reaches this.
const LATTICE_SETTLE_MAX: Duration = Duration::from_secs(10);
const LATTICE_SETTLE_POLL: Duration = Duration::from_millis(10);
const SWEEP_SECS: f64 = 2.0;
/// Screen window per rung, seconds. A screen locates the boundary; it never certifies one.
const SCREEN_SECS: f64 = 10.0;
/// Sustain window per rung, seconds: the only tier a reported ceiling comes from.
const SUSTAIN_SECS: f64 = 60.0;

/// Measure a probe/DUT VCOM pairing: derived rate lattice, v1 duplex stream ledgers, and the
/// operating ceiling the pairing holds.
#[derive(Parser, Debug)]
#[command(name = "vcom-bench", version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Diagnostic only: one shape-controlled echo run at a fixed rate, kept as a regression
    /// reference.
    Echo(EchoArgs),
    /// One operating point: the v1 stream at a fixed rate, in the shape and under the DAP profile
    /// asked for.
    Run(RunArgs),
    /// The pairing's numbers in one command: derive the lattice, escalate through sweep, screen and
    /// sustain, and print the report.
    Ladder(LadderArgs),
    /// User-shaped VCOM service contract at ordinary rates and traffic shapes.
    Contract(ContractArgs),
    /// What a default `ladder` would do to a pairing, and what it would cost, without touching one.
    Plan(PlanArgs),
}

#[derive(clap::Args, Debug)]
struct ContractArgs {
    #[command(flatten)]
    rig: RigArgs,
    /// Ordinary service rate used except by the line-coding case.
    #[arg(long, default_value_t = contract::DEFAULT_BAUD)]
    baud: u32,
    /// Use 100 transition/reopen cycles and a five-minute idle instead of 10 cycles and 10 seconds.
    #[arg(long)]
    qualify: bool,
}

#[derive(clap::Args, Debug)]
struct RigArgs {
    /// Probe selector `VID:PID:serial`; the serial also selects the tty.
    #[arg(long)]
    probe: String,
    /// Wired target name (`stm32f401`, `rp2040`, `rp2350`).
    #[arg(long, default_value = "stm32f401")]
    target: String,
    /// DUT ELF, for the RTT control-block symbol fast path.
    #[arg(long)]
    dut_elf: Option<String>,
    /// Record file override; defaults to `artifacts/vcom-bench.jsonl` in the checkout.
    #[arg(long)]
    record: Option<PathBuf>,
    /// Leave the DUT as it was found instead of resetting it into a known state.
    #[arg(long)]
    no_dut_reset: bool,
    /// Never send the probe's vendor queries; counters and realized baud read as unserved.
    #[arg(long)]
    no_vendor: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Mode {
    /// One chunk in flight: write a chunk, require its echo before the next.
    Paced,
    /// Stream every chunk while a reader collects the echo concurrently.
    Batched,
    /// Write-only attribution: stream the payload, discard the echo, read the DUT counters.
    Out,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum BurstDir {
    Host,
    Dut,
    Both,
}

#[derive(clap::Args, Debug)]
struct EchoArgs {
    #[command(flatten)]
    rig: RigArgs,
    /// Rate to ask of both ends.
    #[arg(long)]
    baud: u32,
    /// Payload length in bytes.
    #[arg(long, default_value_t = 256)]
    len: usize,
    /// Host write granularity in bytes.
    #[arg(long, default_value_t = 64)]
    chunk: usize,
    /// Submission shape.
    #[arg(long, value_enum, default_value_t = Mode::Batched)]
    mode: Mode,
}

#[derive(clap::Args, Debug, Clone)]
struct Windows {
    /// Measured window, seconds; the default is the verb's own.
    #[arg(long)]
    window: Option<f64>,
    /// Untimed warmup before a measured window, seconds; the staged ladder spends it on sustain
    /// windows only.
    #[arg(long, default_value_t = stream::WARMUP_SECS)]
    warmup: f64,
    /// Measured windows per operating point.
    #[arg(long)]
    runs: Option<u32>,
    /// Qualify a ceiling: three 60 s windows, all of which must pass.
    #[arg(long)]
    qualify: bool,
    /// Soak a ceiling: one 600 s window, which is what makes it release evidence.
    #[arg(long)]
    soak: bool,
}

impl Windows {
    /// The window, the run count, and the phase they are recorded under.
    fn resolve(&self, default_secs: f64, default_phase: &'static str) -> (f64, u32, &'static str) {
        let (secs, runs, phase) = match (self.qualify, self.soak) {
            (true, _) => (stream::QUALIFY_SECS, stream::QUALIFY_RUNS, "qualify"),
            (_, true) => (stream::SOAK_SECS, 1, "soak"),
            _ => (default_secs, 1, default_phase),
        };
        (
            self.window.unwrap_or(secs),
            self.runs.unwrap_or(runs).max(1),
            phase,
        )
    }
}

#[derive(clap::Args, Debug, Clone)]
struct Traffic {
    /// Payload bytes between marker frames.
    #[arg(long, default_value_t = stream::DEFAULT_INTERVAL)]
    interval: u32,
    /// DAP traffic interleaved with the window, as a named profile.
    #[arg(long, value_enum, default_value_t = DapProfile::None)]
    dap: DapProfile,
    /// Drain margin after the nominal window, before the ledgers are collected, milliseconds.
    #[arg(long, default_value_t = 1_000)]
    drain_ms: u64,
    /// Seconds between reads of the running ledger inside a window; 0 takes none.
    #[arg(long, default_value_t = stream::CHECKPOINT_SECS)]
    checkpoint: f64,
    /// Delivered fraction a checkpoint holds before the rest of the window is abandoned.
    #[arg(long, default_value_t = stream::CHECKPOINT_FLOOR)]
    checkpoint_floor: f64,
    /// Shape a transmitter into bursts of this many wire bytes.
    #[arg(long)]
    burst: Option<u32>,
    /// Idle gap between bursts, milliseconds.
    #[arg(long, default_value_t = 1)]
    idle: u32,
    /// Which transmitter the burst shape applies to.
    #[arg(long, value_enum, default_value_t = BurstDir::Both)]
    burst_dir: BurstDir,
    /// Run an operating point the aggregate packet budget refuses.
    #[arg(long)]
    allow_over_budget: bool,
}

impl Traffic {
    fn burst_spec(&self) -> Option<BurstSpec> {
        self.burst.map(|bytes| BurstSpec {
            bytes,
            idle_ms: self.idle,
        })
    }

    fn drain(&self) -> Duration {
        Duration::from_millis(self.drain_ms)
    }
}

#[derive(clap::Args, Debug)]
struct RunArgs {
    #[command(flatten)]
    rig: RigArgs,
    #[command(flatten)]
    traffic: Traffic,
    #[command(flatten)]
    windows: Windows,
    /// Rate to ask of both ends.
    #[arg(long)]
    baud: u32,
    /// What each end transmits.
    #[arg(long, value_enum, default_value_t = Shape::Duplex)]
    mode: Shape,
    /// Fraction of the line rate the host leg is paced to in a return-priority run.
    #[arg(long, default_value_t = stream::RETURN_PRIORITY_FRAC)]
    host_frac: f64,
    /// Bytes the host keeps granted ahead of its verifier in credit mode.
    #[arg(long, default_value_t = stream::CREDIT_WINDOW)]
    credit_window: u64,
}

#[derive(clap::Args, Debug, Clone)]
struct Stages {
    /// Stream sweep window per rung, seconds.
    #[arg(long, default_value_t = SWEEP_SECS)]
    sweep_secs: f64,
    /// Screen window per rung, seconds.
    #[arg(long, default_value_t = SCREEN_SECS)]
    screen_secs: f64,
    /// Sustain window per rung, seconds; `--window`, `--qualify` and `--soak` override it.
    #[arg(long, default_value_t = SUSTAIN_SECS)]
    sustain_secs: f64,
    /// Return-priority window at the ceiling, seconds; 0 skips it.
    #[arg(long, default_value_t = SCREEN_SECS)]
    rp_secs: f64,
    /// Rungs the screen tries below the stream sweep ceiling before giving up.
    #[arg(long, default_value_t = lattice::DEFAULT_SCREEN_MAX)]
    screen_max: usize,
    /// Rungs sustained at and below the screened boundary.
    #[arg(long, default_value_t = lattice::DEFAULT_SUSTAIN_RUNGS)]
    sustain_rungs: usize,
}

impl Stages {
    fn limits(&self) -> StagedLimits {
        StagedLimits {
            screen_max: self.screen_max,
            sustain_rungs: self.sustain_rungs,
        }
    }
}

#[derive(clap::Args, Debug)]
struct LadderArgs {
    #[command(flatten)]
    rig: RigArgs,
    #[command(flatten)]
    traffic: Traffic,
    #[command(flatten)]
    windows: Windows,
    #[command(flatten)]
    stages: Stages,
    /// Mismatch budget between the two ends' realized rates, percent.
    #[arg(long, default_value_t = lattice::DEFAULT_BUDGET_PCT)]
    budget_pct: f64,
    /// Sweep every kept rung at one window length instead of staging: the regression option.
    #[arg(long)]
    full: bool,
}

#[derive(clap::Args, Debug)]
struct PlanArgs {
    #[command(flatten)]
    traffic: Traffic,
    #[command(flatten)]
    windows: Windows,
    #[command(flatten)]
    stages: Stages,
    /// The DUT's `BAUD MAX` policy ceiling, which the lattice derives from.
    #[arg(long)]
    dut_max: u32,
    /// Bulk packet size the probe's USB service budget follows from.
    #[arg(long, default_value_t = 64)]
    packet: usize,
}

fn main() -> Result<()> {
    match Cli::parse().command {
        Command::Echo(args) => echo_verb(args),
        Command::Run(args) => run_verb(args),
        Command::Ladder(args) => ladder_verb(args),
        Command::Contract(args) => contract_verb(args),
        Command::Plan(args) => plan_verb(args),
    }
}

struct Bench {
    selector: String,
    chip: &'static str,
    port_name: String,
    record_path: PathBuf,
    dut_elf: Option<String>,
    reset_dut: bool,
    /// Whether the vendor queries may be sent at all; `--no-vendor` clears it.
    vendor: bool,
    /// Armed at startup and again by recovery; consumed by the next [`Bench::open_dut`].
    pending_reset: Cell<bool>,
    /// How far a realized rate may sit from the requested one; `ladder` exposes it as
    /// `--budget-pct` and every other verb takes the default.
    budget_pct: f64,
    /// A probe that serves no applied-ness is said so once, not once per window.
    unconfirmed_noted: Cell<bool>,
}

impl Bench {
    fn resolve(rig: &RigArgs) -> Result<Bench> {
        let chip = dut::chip_for(&rig.target)?;
        let sel: dap_bench::transport::ProbeSelector = rig
            .probe
            .parse()
            .map_err(|e| anyhow::anyhow!("{e}"))
            .with_context(|| format!("parse probe selector '{}'", rig.probe))?;
        let serial = sel
            .serial
            .as_deref()
            .context("the probe selector needs a serial to find the tty")?;
        let port_name = tty::find_port(sel.vendor_id, sel.product_id, serial)?;
        Ok(Bench {
            selector: rig.probe.clone(),
            chip,
            port_name,
            record_path: rig.record.clone().unwrap_or_else(record::default_path),
            dut_elf: rig.dut_elf.clone(),
            reset_dut: !rig.no_dut_reset,
            vendor: !rig.no_vendor,
            pending_reset: Cell::new(!rig.no_dut_reset),
            budget_pct: lattice::DEFAULT_BUDGET_PCT,
            unconfirmed_noted: Cell::new(false),
        })
    }

    /// Resets once per invocation, and again only where recovery arms one: a DUT on its own USB
    /// lead churns that port every reset, and one reset per case has taken a host controller down.
    fn open_dut(&self) -> Result<Dut> {
        let mut dut = Dut::open(&self.selector, self.chip, self.dut_elf.as_deref())?;
        if self.pending_reset.replace(false) {
            dut.reset()
                .context("reset the DUT into a known state (--no-dut-reset skips it)")?;
        }
        Ok(dut)
    }

    /// An over-ceiling rung can wedge the DUT's UART hard enough that only a reset clears it.
    fn arm_reset(&self) {
        self.pending_reset.set(self.reset_dut);
    }

    /// Must not run while a probe-rs session holds the probe's DAP interface.
    fn probe_facts(&self) -> Result<(Option<u32>, BusBudget)> {
        let mut link = probe::DapLink::open(&self.selector, self.vendor)?;
        let budget = BusBudget::for_packet(link.max_packet_size());
        Ok((link.vcom_baud()?.and_then(|b| b.realized), budget))
    }

    /// The probe's realized rate, polled until the probe reports the change applied, under a bound
    /// derived from what the probe still has queued. A window opened before it settles rates that
    /// direction against the previous rung's clock, so an expiry refuses the leg outright rather
    /// than rating it against whatever was read last.
    fn probe_baud_settled(&self, requested: u32) -> Result<Option<u32>> {
        let mut link = probe::DapLink::open(&self.selector, self.vendor)?;
        match settled_probe_baud(&mut link, requested, self.budget_pct)? {
            BaudSettle::Settled(r) => Ok(Some(r)),
            BaudSettle::Disagreed { realized } => {
                eprintln!(
                    "warning: the probe applied {requested} and its divisor reaches {realized}; \
                     this direction is rated against {realized} and is not a measurement of \
                     {requested}"
                );
                Ok(Some(realized))
            }
            BaudSettle::Unreported => Ok(None),
            BaudSettle::Unsettled(u) => bail!("{}", unsettled_note(requested, u)),
        }
    }

    /// Same exclusion as [`Bench::probe_facts`].
    fn probe_counters(&self) -> Result<Option<VcomCounters>> {
        probe::DapLink::open(&self.selector, self.vendor)?.vcom_counters()
    }

    fn append(&self, r: &record::RunRecord) {
        if let Err(e) = record::append(&self.record_path, r) {
            eprintln!("warning: record append failed: {e}");
        }
    }

    fn append_v1(&self, r: &record::V1Record) {
        if let Err(e) = record::append_v1(&self.record_path, r) {
            eprintln!("warning: record append failed: {e}");
        }
    }
}

struct Outcome {
    ledger: Ledger,
    stats: Stats,
    dut_realized: u32,
}

fn run_at(
    bench: &Bench,
    dut: &mut Option<Dut>,
    tty: &mut Tty,
    baud: u32,
    body: impl FnOnce(&mut Tty) -> Result<Ledger>,
) -> Result<(Outcome, Option<u32>)> {
    let prepared = prepare_settled(bench, dut, tty, baud)?;
    let dutctl = attach_armed(bench, dut)?;
    let ledger = body(tty)?;
    std::thread::sleep(STATS_LAG);
    let stats = dutctl.stats()?;
    Ok((
        Outcome {
            ledger,
            stats,
            dut_realized: prepared.dut_realized,
        },
        prepared.probe_realized,
    ))
}

/// What both ends took, once the probe confirmed the rate applied.
struct Prepared {
    dut_realized: u32,
    /// The rate the probe reported realized when it said the change had applied, or `None` where
    /// its firmware serves no applied-ness at all.
    probe_realized: Option<u32>,
}

/// Bring both ends to `baud` and wait for the probe to say it applied the rate, returning with
/// the DUT session released.
///
/// A window opened on a sleep can be measuring the previous rung's rate, and since each leg is
/// scored against its own transmitter's clock, that rates every figure in it against the wrong
/// denominator. The probe's word arrives on the `0x9E` vendor query, which needs the CMSIS-DAP
/// interface the DUT session holds -- so the rate is requested and confirmed with the session
/// down, and the caller reopens it with [`attach_armed`]. Leaving the session released is also
/// what lets a counter bracket open here, between the transition and the window, at no cost.
fn prepare_settled(
    bench: &Bench,
    dut: &mut Option<Dut>,
    tty: &mut Tty,
    baud: u32,
) -> Result<Prepared> {
    if dut.is_none() {
        *dut = Some(bench.open_dut()?);
    }
    let dut_realized = dut
        .as_mut()
        .expect("the session was just opened")
        .set_baud(baud)?
        .with_context(|| format!("the DUT declined BAUD {baud}"))?;
    // Free the DAP interface: nothing can ask the probe anything while the session holds it.
    *dut = None;
    tty.set_baud(baud)?;
    let probe_realized = bench.probe_baud_settled(baud)?;
    if probe_realized.is_none() {
        if !bench.unconfirmed_noted.replace(true) {
            eprintln!(
                "warning: the probe serves no applied-ness for a rate change, so every window \
                 below opens on a {} ms wait rather than on a confirmed rate",
                SETTLE.as_millis()
            );
        }
        std::thread::sleep(SETTLE);
    }
    // The transition's own bytes are not the window's.
    tty.drain()?;
    Ok(Prepared {
        dut_realized,
        probe_realized,
    })
}

/// Reopen the session after a rate transition, without a reset: the rate is already programmed and
/// a reset would both discard it and glitch the line.
fn attach_session<'a>(bench: &Bench, dut: &'a mut Option<Dut>) -> Result<&'a mut Dut> {
    if dut.is_none() {
        *dut = Some(Dut::open(
            &bench.selector,
            bench.chip,
            bench.dut_elf.as_deref(),
        )?);
    }
    Ok(dut.as_mut().expect("the session is open"))
}

/// [`attach_session`], then zero the DUT's engines. A prior v1 run leaves the DUT's UART owned by
/// a spent stream engine, and only `v1_reset` returns it to the echo path; it is not a DUT reset,
/// so `--no-dut-reset` must not skip it. Both follow the transition, which charges the DUT's
/// counters framing errors the stream never made.
fn attach_armed<'a>(bench: &Bench, dut: &'a mut Option<Dut>) -> Result<&'a mut Dut> {
    let session = attach_session(bench, dut)?;
    session.v1_reset()?;
    session.stats_reset()?;
    Ok(session)
}

/// A leg is scored against the rate its transmitter is clocked at, so a probe that confirms a rate
/// the lattice did not derive for this rung would have the window rated against a stale
/// denominator. The rungs reaching a window all agreed at derivation, so this is a disagreement
/// with the probe's earlier self rather than a mismatched pairing.
fn stale_denominator(prepared: &Prepared, rung: &Rung, budget_pct: f64) -> bool {
    match (prepared.probe_realized, rung.probe_realized) {
        (Some(confirmed), Some(derived)) => !rate_agreed(derived, confirmed, budget_pct),
        _ => false,
    }
}

/// Refuse the window rather than rate it against a rate the probe is not running.
fn denominator_checked(prepared: &Prepared, rung: &Rung, budget_pct: f64) -> Result<()> {
    if stale_denominator(prepared, rung, budget_pct) {
        bail!(
            "the probe confirmed {} applied where the lattice derived {} for this rung",
            opt_rate(prepared.probe_realized),
            opt_rate(rung.probe_realized),
        );
    }
    Ok(())
}

/// A checkpoint only reads the host's own ledgers; it never pauses the stream.
fn measure(dutctl: &mut Dut, tty: &mut Tty, plan: &Plan, traffic: &Traffic) -> Result<Run> {
    let (profile, drain) = (traffic.dap, traffic.drain());
    tty.drain()?;
    dutctl.v1_reset()?;

    let clock = Clock::new();
    let reader = match plan.host_verifier() {
        Some(mut verifier) => {
            // Armed before the DUT starts, so a stalled onset or lost leading byte lands inside the
            // window.
            verifier.arm(clock.now_us());
            Some(tty.v1_reader(verifier, clock)?)
        }
        None => None,
    };
    dutctl.v1_start(&plan.start_line())?;
    let started = Instant::now();
    let writer = match plan.host_encoder() {
        Some(encoder) => {
            let credit = (plan.shape == Shape::Credit)
                .then(|| reader.as_ref().map(|r| (*plan, r.verified())))
                .flatten();
            Some(tty.v1_writer(encoder, plan.host_pace_bps(), clock, credit)?)
        }
        None => None,
    };

    // done() says nothing about the far end's drain, so the wait always runs to the deadline.
    let deadline = started + Duration::from_secs_f64(plan.secs) + drain;
    let done = || {
        reader.as_ref().is_none_or(tty::Reader::done)
            && writer.as_ref().is_none_or(tty::Writer::done)
    };
    let stop = AtomicBool::new(false);
    let mut sampler = Sampler::new(plan, traffic.checkpoint, traffic.checkpoint_floor);
    let mut abandoned = None;
    let dap = {
        let dutctl = &mut *dutctl;
        let (reader, writer) = (&reader, &writer);
        std::thread::scope(|scope| -> Result<DapAchieved> {
            let load = (profile != DapProfile::None)
                .then(|| scope.spawn(|| dutctl.dap_load(profile, &stop)));
            while Instant::now() < deadline {
                std::thread::sleep(WINDOW_POLL);
                if done() {
                    continue;
                }
                abandoned = sampler.sample(
                    started.elapsed().as_secs_f64(),
                    writer.as_ref().map(tty::Writer::snapshot),
                    reader.as_ref().map(tty::Reader::snapshot),
                );
                if abandoned.is_some() {
                    break;
                }
            }
            stop.store(true, Ordering::Relaxed);
            match load {
                Some(handle) => handle
                    .join()
                    .map_err(|_| anyhow!("the DAP load thread panicked"))?,
                None => Ok(DapAchieved::default()),
            }
        })?
    };

    // A run that met its budget ended by itself; any other is closed at both ends, not dropped.
    let closed = !done();
    if closed {
        if let Some(writer) = &writer {
            writer.close();
        }
        dutctl
            .v1_stop()
            .context("close the run that was still running")?;
        let closing = Instant::now() + drain;
        while !done() && Instant::now() < closing {
            std::thread::sleep(WINDOW_POLL);
        }
    }

    let measured_secs = started.elapsed().as_secs_f64();
    let timed_out = !done();
    let granted = writer.as_ref().map_or(0, tty::Writer::granted);
    let host_tx = writer.map(tty::Writer::finish).transpose()?;
    let host_rx = reader.map(tty::Reader::finish).transpose()?;
    let report = dutctl.v1_report()?;
    // Sticky since boot, so any nonzero read condemns this window and everything after it.
    let stuck = dutctl.stats()?.stuck;
    Ok(Run {
        legs: stream::legs(plan, host_tx, host_rx, &report),
        plan: *plan,
        ferr: report.ferr,
        ovr: report.ovr,
        drop: report.drop,
        stuck,
        measured_secs,
        timed_out,
        closed,
        abandoned,
        series: sampler.finish(),
        dap,
        credit: (plan.shape == Shape::Credit).then(|| Credit {
            granted,
            obeyed: u64::from(report.credit),
        }),
        // Filled in once the probe-rs session lets go of the probe's DAP interface.
        conservation: None,
    })
}

/// The untimed warmup is a real run whose ledger is recorded but never scored.
fn one_window(
    bench: &Bench,
    dut: &mut Option<Dut>,
    tty: &mut Tty,
    rung: &Rung,
    plan: &Plan,
    traffic: &Traffic,
    warmup: f64,
) -> Result<(Option<Run>, Run)> {
    let prepared = prepare_settled(bench, dut, tty, rung.nominal)?;
    denominator_checked(&prepared, rung, bench.budget_pct)?;
    let dutctl = attach_armed(bench, dut)?;
    windows_of(dutctl, tty, plan, traffic, warmup)
}

/// The windows alone, for a caller that has already brought both ends to the rung's rate and
/// needs its counter bracket to open after that rather than across it.
fn windows_of(
    dutctl: &mut Dut,
    tty: &mut Tty,
    plan: &Plan,
    traffic: &Traffic,
    warmup: f64,
) -> Result<(Option<Run>, Run)> {
    let warmed = match warmup > 0.0 {
        true => Some(measure(
            dutctl,
            tty,
            &Plan {
                secs: warmup,
                ..*plan
            },
            traffic,
        )?),
        false => None,
    };
    let run = measure(dutctl, tty, plan, traffic)?;
    Ok((warmed, run))
}

struct Rig<'a> {
    bench: &'a Bench,
    verb: &'static str,
    /// A slot rather than a session: preparation puts the session down to confirm the rate.
    dut: &'a mut Option<Dut>,
    tty: &'a mut Tty,
    /// Hold each record until the verb can read the probe's counters, once the DUT session lets go
    /// of the DAP interface.
    holds_records: bool,
    held: Vec<Held>,
}

struct Held {
    rung: Rung,
    run: Run,
    phase: &'static str,
    clamp: Option<Clamp>,
}

impl Rig<'_> {
    fn window(
        &mut self,
        rung: &Rung,
        plan: &Plan,
        traffic: &Traffic,
        warmup: f64,
        phase: &'static str,
        clamp: Option<Clamp>,
    ) -> Result<Run> {
        let (warmed, run) =
            one_window(self.bench, self.dut, self.tty, rung, plan, traffic, warmup)?;
        if let Some(warmed) = warmed {
            self.record(rung, &warmed, "warmup", clamp);
        }
        self.record(rung, &run, phase, clamp);
        Ok(run)
    }

    fn record(&mut self, rung: &Rung, run: &Run, phase: &'static str, clamp: Option<Clamp>) {
        if self.holds_records {
            self.held.push(Held {
                rung: rung.clone(),
                run: run.clone(),
                phase,
                clamp,
            });
            return;
        }
        self.bench.append_v1(&record::V1Record {
            probe: &self.bench.selector,
            verb: self.verb,
            phase,
            baud_nominal: rung.nominal,
            probe_realized: rung.probe_realized,
            run,
            clamp,
            note: None,
        });
    }
}

/// Every byte is charged to the direction that carried it.
fn print_run(run: &Run) {
    let plan = &run.plan;
    println!(
        "  {:<12} {:>12} {:>8} {:>8} {:>6} {:>8} {:>6} {:>12} {:>9} {:>9} {:>10}",
        "direction",
        "verified",
        "corrupt",
        "missing",
        "dup",
        "unresolv",
        "resync",
        "B/s",
        "of budget",
        "onset us",
        "maxgap us",
    );
    for dir in stream::DIRECTIONS {
        let Some(leg) = run.leg(dir) else {
            println!("  {:<12} {:>12}", stream::label(dir), "silent");
            continue;
        };
        let rx = &leg.rx;
        println!(
            "  {:<12} {:>12} {:>8} {:>8} {:>6} {:>8} {:>6} {:>12.0} {:>8.1}% {:>9} {:>10}",
            stream::label(dir),
            rx.verified,
            rx.corrupt,
            rx.missing,
            rx.duplicate,
            rx.unresolved,
            rx.resyncs,
            run.goodput_bps(dir),
            run.budget_frac(dir) * 100.0,
            rx.onset_us,
            rx.maxgap_us,
        );
        println!(
            "    sent {} payload in {} frames, end-of-stream {}, ledgers {}",
            leg.tx_payload,
            leg.tx_frames,
            if rx.eos { "seen" } else { "MISSING" },
            if leg.reconciles() {
                "reconcile"
            } else {
                "DO NOT reconcile"
            },
        );
    }
    println!(
        "  window {:.1}s (held {:.1}s, measured {:.1}s{}), payload budget {:.0} / {:.0} B/s \
         (h2d / d2h), DUT fe/oe/drop {}/{}/{}",
        plan.secs,
        run.sustained_secs(),
        run.measured_secs,
        if run.closed { ", host-closed" } else { "" },
        plan.payload_budget_bps(Direction::HostToDut),
        plan.payload_budget_bps(Direction::DutToHost),
        run.ferr,
        run.ovr,
        run.drop,
    );
    print_series(run);
    if run.dap.profile != DapProfile::None {
        println!(
            "  DAP {} achieved {:.0} commands/s, {:.0} B/s",
            run.dap.profile.key(),
            run.dap.cps(),
            run.dap.bps(),
        );
    }
    if let Some(credit) = run.credit {
        println!(
            "  credit granted {} B, obeyed {} B, window {} B",
            credit.granted, credit.obeyed, plan.credit_window
        );
    }
}

fn print_series(run: &Run) {
    let mut out = String::new();
    render_series(&mut out, run, "  ");
    print!("{out}");
}

fn render_series(out: &mut String, run: &Run, indent: &str) {
    let series = &run.series;
    if series.points.is_empty() {
        return;
    }
    let _ = writeln!(
        out,
        "{indent}checkpoints every {:.1}s (host -> DUT offered / DUT -> host delivered):",
        series.period,
    );
    for point in &series.points {
        let frac = |slice: Option<&stream::Slice>| match slice {
            Some(s) => format!("{:>6.1}%", s.frac * 100.0),
            None => format!("{:>7}", "--"),
        };
        let _ = write!(
            out,
            "{indent}  t={:>6.1}s  {} / {}",
            point.at_secs,
            frac(point.h2d.as_ref()),
            frac(point.d2h.as_ref()),
        );
        if let Some(d2h) = point.d2h.filter(|s| s.lost() > 0 || s.resyncs > 0) {
            let _ = write!(
                out,
                "  DUT -> host {} corrupt, {} missing, {} duplicate, {} unresolved, {} resyncs",
                d2h.corrupt, d2h.missing, d2h.duplicate, d2h.unresolved, d2h.resyncs,
            );
        }
        out.push('\n');
    }
    for dir in stream::DIRECTIONS {
        let trend = series.trend(dir);
        if trend != stream::Trend::Unsampled {
            let _ = writeln!(
                out,
                "{indent}  {}: {} -- {}",
                stream::label(dir),
                trend.key(),
                trend.reads_as(),
            );
        }
    }
}

fn print_verdict(run: &Run) {
    let verdict = run.verdict();
    println!("  verdict: {}", verdict_word(verdict.pass));
    for reason in &verdict.reasons {
        println!("    {reason}");
    }
    for note in &verdict.notes {
        println!("    note: {note}");
    }
}

/// The plan a verb's flags describe at one realized rate.
fn plan_for(
    shape: Shape,
    traffic: &Traffic,
    dut_baud: u32,
    probe_baud: Option<u32>,
    secs: f64,
    host_frac: f64,
    credit_window: u64,
) -> Plan {
    Plan {
        shape,
        seeds: Seeds::default(),
        interval: traffic.interval,
        secs,
        dut_baud,
        // A probe that does not report its rate is taken at the DUT's, which is the best the
        // pairing can say and what every budget assumed before the query existed.
        probe_baud: probe_baud.unwrap_or(dut_baud),
        host_frac: if shape == Shape::ReturnPriority {
            host_frac
        } else {
            1.0
        },
        burst: traffic.burst_spec(),
        burst_host: matches!(traffic.burst_dir, BurstDir::Host | BurstDir::Both),
        burst_dut: matches!(traffic.burst_dir, BurstDir::Dut | BurstDir::Both),
        credit_window,
    }
}

/// An over-budget point is refused up front, not measured: the bus can't serve it either way.
fn admit(plan: &Plan, budget: &BusBudget, traffic: &Traffic) -> Result<Clamp> {
    let demand = plan.demand(traffic.dap.nominal_bps());
    let clamp = budget::clamp(&demand, budget);
    println!(
        "  aggregate budget: {:.2} of {:.2} 64-byte slots/ms ({:.0}% of a {} bulk budget, DAP \
         profile {})",
        clamp.demand,
        clamp.budget,
        clamp.load() * 100.0,
        if clamp.high_speed {
            "high-speed"
        } else {
            "full-speed"
        },
        traffic.dap.key(),
    );
    if !clamp.fits && !traffic.allow_over_budget {
        let paced = budget::clamp(
            &plan.as_return_priority().demand(traffic.dap.nominal_bps()),
            budget,
        );
        let alternative = if paced.fits {
            "run it as `--mode return-priority` (host leg paced, return leg at line rate), lower \
             the rate,"
        } else {
            "lower the rate (a paced host leg does not fit either),"
        };
        bail!(
            "the point asks {:.2} slots/ms of a {:.2} slots/ms bus; {alternative} or pass \
             --allow-over-budget to measure it anyway",
            clamp.demand,
            clamp.budget,
        );
    }
    Ok(clamp)
}

fn echo_verb(args: EchoArgs) -> Result<()> {
    let bench = Bench::resolve(&args.rig)?;
    let mut tty = Tty::open(&bench.port_name, args.baud)?;
    let mut dut = None;

    let payload = tty::pattern(args.len);
    let (out, probe_realized) =
        run_at(&bench, &mut dut, &mut tty, args.baud, |t| match args.mode {
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
    });
    Ok(())
}

fn contract_verb(args: ContractArgs) -> Result<()> {
    use contract::{CaseResult, Latencies};

    let bench = Bench::resolve(&args.rig)?;
    let cycles = contract::cycles(args.qualify);
    let mut tty = Tty::open(&bench.port_name, args.baud)?;
    let mut failed_cases = 0usize;
    let (counter_baseline, mut dut) = prepare_contract_case(&bench, &mut tty, args.baud)?;
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
    finish_contract_case(&bench, counter_baseline, &mut small)?;
    failed_cases += usize::from(!small.pass);

    let (counter_baseline, mut dut) = prepare_contract_case(&bench, &mut tty, args.baud)?;

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
    finish_contract_case(&bench, counter_baseline, &mut idle)?;
    failed_cases += usize::from(!idle.pass);

    for pause in contract::PAUSES {
        let (counter_baseline, mut dut) = prepare_contract_case(&bench, &mut tty, args.baud)?;
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
        finish_contract_case(&bench, counter_baseline, &mut result)?;
        failed_cases += usize::from(!result.pass);
    }

    let (counter_baseline, dut) = prepare_contract_case(&bench, &mut tty, args.baud)?;
    let mut dut = Some(dut);
    drop(tty);
    let mut cdc = ClaimedCdc::open(&bench.selector)?;
    let mut line_latencies = Latencies::default();
    let mut line_exact = true;
    let mut line_failure = None;
    let mut line_dap_commands = 0usize;
    macro_rules! line_try {
        ($label:lifetime, $operation:expr, $stage:literal, $cycle:expr, $baud:expr) => {
            match $operation {
                Ok(value) => value,
                Err(error) => {
                    line_failure = Some(contract::line_runtime_failure(
                        $cycle,
                        $baud,
                        $stage,
                        format_args!("{error:#}"),
                    ));
                    break $label;
                }
            }
        };
    }
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
            let scrub = ClaimedCdc::scrub_baud(baud);
            line_try!('cycles, cdc.set_baud(scrub), "scrub control", cycle, baud);
            drop(dut.take());
            std::thread::sleep(contract::LINE_CONTROL_QUIET);
            if let Err(failure) = line_try!(
                'cycles,
                settle_contract_line_baud(&bench, cycle, baud, contract::LineStage::Scrub, scrub,),
                "scrub verify",
                cycle,
                baud
            ) {
                line_failure = Some(failure);
                break 'cycles;
            }
            line_try!('cycles, cdc.set_baud(baud), "target control", cycle, baud);
            std::thread::sleep(contract::LINE_CONTROL_QUIET);
            if let Err(failure) = line_try!(
                'cycles,
                settle_contract_line_baud(
                    &bench,
                    cycle,
                    baud,
                    contract::LineStage::Target,
                    dut_realized,
                ),
                "target verify",
                cycle,
                baud
            ) {
                line_failure = Some(failure);
                break 'cycles;
            }
            dut = Some(line_try!(
                'cycles,
                Dut::open(&bench.selector, bench.chip, bench.dut_elf.as_deref()),
                "target reattach",
                cycle,
                baud
            ));
            let ledger = line_try!('cycles, cdc.echo(&tty::pattern(64)), "bulk echo", cycle, baud);
            if !ledger.byte_exact {
                line_exact = false;
                line_failure = Some(contract::line_echo_failure(
                    cycle,
                    baud,
                    ledger.written,
                    ledger.read,
                    ledger.first_divergence,
                ));
                break 'cycles;
            }
            line_try!(
                'cycles,
                dut.as_mut()
                    .expect("the target session was just attached")
                    .dap_command(),
                "DAP",
                cycle,
                baud
            );
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
    finish_contract_case(&bench, counter_baseline, &mut line)?;
    failed_cases += usize::from(!line.pass);

    drop(cdc);
    let mut tty = reopen_contract_tty(&bench.port_name, args.baud)?;

    let (counter_baseline, mut dut) = prepare_contract_case(&bench, &mut tty, args.baud)?;
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
    finish_contract_case(&bench, counter_baseline, &mut reopen)?;
    failed_cases += usize::from(!reopen.pass);
    contract::suite_pass(failed_cases).map_err(anyhow::Error::msg)
}

fn reopen_contract_tty(port_name: &str, baud: u32) -> Result<Tty> {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        match Tty::open(port_name, baud) {
            Ok(tty) => return Ok(tty),
            Err(_) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(error) => return Err(error).context("reopen CDC tty after releasing USB claims"),
        }
    }
}

fn settle_contract_line_baud(
    bench: &Bench,
    cycle: usize,
    requested: u32,
    stage: contract::LineStage,
    expected: u32,
) -> Result<Result<u32, String>> {
    let deadline = Instant::now() + contract::LINE_SETTLE_TIMEOUT;
    let mut link = probe::DapLink::open(&bench.selector, bench.vendor)?;
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
        if Instant::now() >= deadline {
            return Ok(Err(failure.expect("an unsettled transition has a failure")));
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
    // Preparation returns with the session down and the probe's rate confirmed, which is where
    // the bracket belongs: after the reset and the transition, before the case's first byte.
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
            record_contract(bench, result);
            return Ok(());
        }
    };
    result.counter_classification = contract::counter_classification(before, after);
    contract::apply_counter_classification(result);
    record_contract(bench, result);
    Ok(())
}

fn record_contract(bench: &Bench, result: &contract::CaseResult) {
    println!("contract {:<16} {}", result.case, verdict_word(result.pass));
    if let Some(reason) = &result.failure {
        println!("  {reason}");
    }
    if let Err(e) = record::append_contract(&bench.record_path, &bench.selector, result) {
        eprintln!("warning: record append failed: {e}");
    }
}

fn print_ledger(out: &Outcome) {
    let l = &out.ledger;
    let s = &out.stats;
    println!("  leg                          bytes");
    println!("  host -> probe (written)      {}", l.written);
    println!("  probe -> DUT  (DUT rx)       {}", s.rx);
    println!("  DUT echo      (DUT tx)       {}", s.tx);
    println!("  DUT -> host   (read)         {}", l.read);
    println!(
        "  DUT ferr/ovr/drop/stuck      {}/{}/{}/{}",
        s.ferr, s.ovr, s.drop, s.stuck
    );
    if s.stuck != 0 {
        println!("  the DUT latched a stuck DMA abort; its counters are suspect");
    }
    let line = f64::from(out.dut_realized) / 10.0;
    let frac = if line > 0.0 {
        l.throughput_bps / line
    } else {
        0.0
    };
    println!(
        "  byte-exact {}   {:.3}s   throughput {:.1} B/s ({:.1}% of line rate)",
        if l.byte_exact { "yes" } else { "NO " },
        l.elapsed.as_secs_f64(),
        l.throughput_bps,
        frac * 100.0
    );
    if let Some(d) = l.first_divergence {
        println!("  first divergence at byte {d}");
    }
}

fn mode_name(mode: Mode) -> &'static str {
    match mode {
        Mode::Paced => "paced",
        Mode::Batched => "batched",
        Mode::Out => "out",
    }
}

fn opt_rate(rate: Option<u32>) -> String {
    rate.map_or_else(|| "unreported".to_string(), |r| r.to_string())
}

fn verdict_word(pass: bool) -> &'static str {
    if pass {
        "pass"
    } else {
        "FAIL"
    }
}

/// How an echo came back: byte-exact, or the offset where it stopped being so.
fn divergence(ledger: &Ledger) -> String {
    match (ledger.byte_exact, ledger.first_divergence) {
        (true, _) => "exact".to_string(),
        (false, Some(at)) => format!("at {at}"),
        (false, None) => "short".to_string(),
    }
}

fn rung_line(r: &Rung) -> String {
    format!(
        "{} nominal (DUT realized {}, probe realized {})",
        r.nominal,
        r.dut_realized,
        opt_rate(r.probe_realized)
    )
}

/// The `run` verb: one operating point, warmed up and then measured as many times as asked.
fn run_verb(args: RunArgs) -> Result<()> {
    let bench = Bench::resolve(&args.rig)?;
    let mut tty = Tty::open(&bench.port_name, args.baud)?;
    let (_, bus) = bench.probe_facts()?;
    let mut dut = None;
    let Prepared {
        dut_realized,
        probe_realized,
    } = prepare_settled(&bench, &mut dut, &mut tty, args.baud)?;
    // Baseline after both ends take the rate: the reset and the receiver's resynchronisation each
    // leave an error the stream never made. Preparation left the session released for it.
    let baseline = bench.probe_counters()?;
    attach_armed(&bench, &mut dut)?;
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

    // Recorded after the loop, so a window that errors leaves its predecessors recorded.
    let mut failure = None;
    let mut held = {
        let mut rig = Rig {
            bench: &bench,
            verb: "run",
            dut: &mut dut,
            tty: &mut tty,
            holds_records: true,
            held: Vec::new(),
        };
        for i in 0..runs {
            if runs > 1 {
                println!("run {} of {runs}:", i + 1);
            }
            match rig.window(
                &rung,
                &plan,
                &args.traffic,
                args.windows.warmup,
                phase,
                Some(clamp),
            ) {
                Ok(run) => print_run(&run),
                Err(e) => {
                    failure = Some(e);
                    break;
                }
            }
        }
        rig.held
    };

    // The counters live behind the probe's DAP interface, which the DUT session held until here.
    drop(dut.take());
    let after = match bench.probe_counters() {
        Ok(after) => after,
        // A probe that has gone unreachable is the window's failure to report, not this read's.
        Err(e) if failure.is_some() => {
            eprintln!("warning: the probe counter read failed: {e:#}");
            None
        }
        Err(e) => return Err(e),
    };
    let conservation = baseline
        .zip(after)
        .map(|(before, after)| Conservation { before, after });
    print_conservation(conservation.as_ref());

    let mut passed = 0u32;
    for h in &mut held {
        h.run.conservation = conservation;
        bench.append_v1(&record::V1Record {
            probe: &bench.selector,
            verb: "run",
            phase: h.phase,
            baud_nominal: h.rung.nominal,
            probe_realized: h.rung.probe_realized,
            run: &h.run,
            clamp: h.clamp,
            note: None,
        });
        if h.phase != "warmup" {
            print_verdict(&h.run);
            passed += u32::from(h.run.verdict().pass);
        }
    }
    if let Some(e) = failure {
        return Err(e);
    }
    println!("{passed} of {runs} windows passed");
    Ok(())
}

fn print_conservation(conservation: Option<&Conservation>) {
    let Some(c) = conservation else {
        println!("  probe counters: not served by this firmware");
        return;
    };
    let d = c.delta();
    let counter = |value: u32, slot: usize| stream::render_counter(&d, value, slot);
    println!(
        "  probe counters over these windows: {} off the UART, {} to the host, {} host-dropped, \
         {} ring-full, {} framing, {} overruns",
        d.uart_rx,
        d.cdc_in,
        d.host_dropped,
        counter(d.ring_full, stream::SLOT_RING_FULL),
        counter(d.frame_err, stream::SLOT_FRAME_ERR),
        counter(d.overrun, stream::SLOT_OVERRUN),
    );
    let sourced = |bits: u32| d.valid.is_some_and(|mask| mask & bits != 0);
    if sourced(1 << stream::SLOT_CDC_OUT | 1 << stream::SLOT_UART_TX) {
        println!(
            "  forward leg: {} in from the host, {} onto the wire",
            counter(d.cdc_out, stream::SLOT_CDC_OUT),
            counter(d.uart_tx, stream::SLOT_UART_TX),
        );
    }
    if d.dma_faulted() {
        println!(
            "  probe DMA faults in these windows: {} receive -- a channel was reinitialised \
             under live traffic",
            d.rx_dma_err,
        );
    }
    println!("  ledger {}", c.classification().key());
}

/// Bytes the probe has taken from the host and not yet put on the wire. A line-coding change
/// cannot reach the divisor until they leave at the rate they were queued at, so this is what a
/// settle wait is actually waiting for.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum InFlight {
    /// The forward ledger's deficit: accepted from CDC OUT, not yet transmitted.
    Bytes(u32),
    /// No forward ledger, so the drain cannot be bounded from the probe's own word.
    Unknown,
}

impl InFlight {
    fn bytes(self) -> Option<u32> {
        match self {
            InFlight::Bytes(n) => Some(n),
            InFlight::Unknown => None,
        }
    }

    /// Whether the probe attests bytes still to go. Only the forward ledger can attest either way:
    /// a ring mark never falls, so it can never say the transmitter emptied.
    fn pending(self) -> bool {
        matches!(self, InFlight::Bytes(1..))
    }

    fn clause(self) -> String {
        match self {
            InFlight::Bytes(n) => format!("{n} bytes were still queued at the old rate"),
            InFlight::Unknown => "the probe does not report what it has queued".to_string(),
        }
    }
}

/// A legacy reply carries no coverage word and no forward ledger, so its zeroes are silence rather
/// than an empty queue; only an explicitly set bit is the probe attesting a slot.
fn attests(valid: Option<u32>, slot: usize) -> bool {
    valid.is_some_and(|mask| mask & (1 << slot) != 0)
}

fn forward_in_flight(c: &VcomCounters) -> InFlight {
    if attests(c.valid, stream::SLOT_CDC_OUT) && attests(c.valid, stream::SLOT_UART_TX) {
        InFlight::Bytes(c.cdc_out.wrapping_sub(c.uart_tx))
    } else {
        InFlight::Unknown
    }
}

/// The wait a rate change is given: what the change costs on a quiet line, plus the time the bytes
/// already queued need to leave at the rate they were queued at -- ten bits per byte, the 8N1
/// frame the bridge runs. One fixed bound cannot serve both ends of the lattice: 400 ms is four
/// times too short for a full ring at 9600 and absurdly generous at 4 Mbaud.
fn settle_bound(queued: InFlight, old_baud: Option<u32>) -> Duration {
    let drain = queued
        .bytes()
        .zip(old_baud.filter(|&baud| baud > 0))
        .map(|(bytes, baud)| Duration::from_secs_f64(f64::from(bytes) * 10.0 / f64::from(baud)))
        .unwrap_or_default();
    (LATTICE_SETTLE_BASE + drain).min(LATTICE_SETTLE_MAX)
}

/// Bring the line to rest before asking for a rate change, so the transition is deterministic
/// rather than however loaded the previous rung left it. The ladder measures sustained throughput
/// at a rate; a loaded transition is a different phenomenon and belongs in the contract, not
/// inside a rung. What is still queued when this returns is what the settle wait must allow for.
fn quiesce_line(
    link: &mut probe::DapLink,
    tty: &mut Tty,
    old_baud: Option<u32>,
) -> Result<InFlight> {
    tty.drain()?;
    let read = |link: &mut probe::DapLink| -> Result<InFlight> {
        Ok(link
            .vcom_counters()?
            .map_or(InFlight::Unknown, |c| forward_in_flight(&c)))
    };
    // Only the forward ledger can say the transmitter emptied; a ring mark never falls, so there
    // is nothing to wait on where the probe does not source one.
    let mut queued = read(link)?;
    if !queued.pending() {
        return Ok(queued);
    }
    let deadline = Instant::now() + settle_bound(queued, old_baud);
    while Instant::now() < deadline {
        std::thread::sleep(LATTICE_SETTLE_POLL);
        queued = read(link)?;
        if !queued.pending() {
            break;
        }
    }
    Ok(queued)
}

/// What an expired settle wait knew: the rate last read, whether the probe sources applied-ness at
/// all, the bound that expired, and what the probe still had queued when the wait began.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Unsettled {
    last: Option<u32>,
    applied_sourced: bool,
    bound: Duration,
    queued: InFlight,
}

/// What a wait for the probe's rate to stop moving concluded. The bound is still a bound -- an
/// unbounded wait on a probe that never applies is a worse failure -- but expiring is now an
/// outcome the caller has to handle rather than a stale reading substituted in silence.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum BaudSettle {
    /// The probe applied the rate, and its realized rate agrees with the one requested.
    Settled(u32),
    /// The probe applied the rate, and its divisor does not reach it. Rating a leg against
    /// `realized` is honest; calling it a measurement of the requested rate is not.
    Disagreed { realized: u32 },
    /// The bound expired with the change still in flight, or -- on a firmware that cannot report
    /// applied-ness -- with the realized rate still not agreeing. Nothing here is settled.
    Unsettled(Unsettled),
    /// The firmware does not serve the query at all.
    Unreported,
}

impl BaudSettle {
    /// The rate a leg would be rated against, where one may be quoted at all.
    fn realized(self) -> Option<u32> {
        match self {
            BaudSettle::Settled(r) | BaudSettle::Disagreed { realized: r } => Some(r),
            BaudSettle::Unsettled(_) | BaudSettle::Unreported => None,
        }
    }
}

fn rate_agreed(requested: u32, realized: u32, budget_pct: f64) -> bool {
    lattice::Rung {
        nominal: requested,
        dut_realized: requested,
        probe_realized: Some(realized),
    }
    .agreed(budget_pct)
}

/// Decide one poll of the query: `None` keeps polling, `Some` is the conclusion.
fn settle_step(reading: probe::VcomBaud, requested: u32, budget_pct: f64) -> Option<BaudSettle> {
    let agreed = reading
        .realized
        .is_some_and(|r| rate_agreed(requested, r, budget_pct));
    match reading.settled {
        // The firmware knows. Waiting past its own word only re-reads the same answer.
        Some(true) => Some(match reading.realized {
            Some(r) if agreed => BaudSettle::Settled(r),
            Some(r) => BaudSettle::Disagreed { realized: r },
            None => BaudSettle::Unsettled(Unsettled {
                last: None,
                applied_sourced: true,
                bound: Duration::ZERO,
                queued: InFlight::Unknown,
            }),
        }),
        Some(false) => None,
        // No applied-ness on the wire: agreement is the only evidence there is.
        None => agreed.then(|| BaudSettle::Settled(reading.realized.expect("agreement needs one"))),
    }
}

/// Wait for the probe to stop moving between rates. A line-coding change is applied from the
/// probe's own context, so what ends the wait is the probe saying it applied one -- not a reading
/// that happens to agree, and never the timeout, which now reports rather than substitutes.
fn settled_probe_baud(
    link: &mut probe::DapLink,
    requested: u32,
    budget_pct: f64,
) -> Result<BaudSettle> {
    let mut last = None;
    let mut wait: Option<(Instant, Duration, InFlight)> = None;
    loop {
        let Some(reading) = link.vcom_baud()? else {
            // No realized-baud query to poll: grant the divisor the base settle blind.
            std::thread::sleep(LATTICE_SETTLE_BASE);
            return Ok(BaudSettle::Unreported);
        };
        if let Some(outcome) = settle_step(reading, requested, budget_pct) {
            return Ok(outcome);
        }
        last = reading.realized.or(last);
        let (deadline, span, queued) = match wait {
            Some(pending) => pending,
            None => {
                // The rate still on the wire is the one the queue behind the change drains at.
                let queued = link
                    .vcom_counters()?
                    .map_or(InFlight::Unknown, |c| forward_in_flight(&c));
                let span = settle_bound(queued, reading.realized);
                *wait.insert((Instant::now() + span, span, queued))
            }
        };
        if Instant::now() >= deadline {
            return Ok(BaudSettle::Unsettled(Unsettled {
                last,
                applied_sourced: reading.settled.is_some(),
                bound: span,
                queued,
            }));
        }
        std::thread::sleep(LATTICE_SETTLE_POLL);
    }
}

/// One line about an expiry, so the reason a rate was never confirmed is on the record rather
/// than folded into whatever number happened to be read last.
fn unsettled_note(requested: u32, u: Unsettled) -> String {
    let last = match u.last {
        Some(r) => format!("the last realized rate read was {r}"),
        None => "no realized rate was read at all".to_string(),
    };
    // The probe's own word, with nothing behind it: the bound is not what failed here.
    if u.bound.is_zero() {
        return format!(
            "the probe reported {requested} applied with no realized rate to show for it, and \
             {last}"
        );
    }
    let why = if u.applied_sourced {
        "the probe still had the change in flight"
    } else {
        "this firmware does not report applied-ness, so settling could only be inferred"
    };
    format!(
        "the probe did not settle at {requested} within {} ms: {why}, {last}, and {}",
        u.bound.as_millis(),
        u.queued.clause(),
    )
}

/// The `ladder` verb: derive the lattice, prune it, and find the ceiling the pairing holds.
fn ladder_verb(args: LadderArgs) -> Result<()> {
    let mut bench = Bench::resolve(&args.rig)?;
    // The only verb that exposes the divisor budget; every rate confirmation is judged against it.
    bench.budget_pct = args.budget_pct;

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
    let mut tty = Tty::open(&bench.port_name, candidates[0])?;
    // Opening the port is what programmed the first rate, so it is the rate the next change waits
    // behind.
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
            // The transition is taken from rest, so what the rung then measures is the rate, not
            // however loaded the previous one left the line.
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
            // A rung the probe never confirmed is not a rung: declining it drops it from the
            // lattice, where before it entered rated against whatever was last read.
            if let BaudSettle::Unsettled(u) = settle {
                eprintln!("  {}", unsettled_note(rate, u));
                declined.push(rate);
                continue;
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
        return staged_ladder(&bench, &mut tty, &args, &bus, dut_max, &rungs, &kept);
    }

    let (secs, runs, phase) = args.windows.resolve(stream::DISCOVERY_SECS, "discovery");
    if runs > 1 {
        eprintln!("  note: the ladder measures each rung once; --runs applies to `run`");
    }
    let mut dut = None;
    let mut rig = Rig {
        bench: &bench,
        verb: "ladder",
        dut: &mut dut,
        tty: &mut tty,
        holds_records: false,
        held: Vec::new(),
    };
    full_sweep(&mut rig, &kept, &args, &bus, |r| plan_at(r, secs), phase)
}

struct Row {
    stage: Stage,
    rung: Rung,
    /// The window, or why it could not be run at all.
    outcome: std::result::Result<Run, String>,
    /// The probe's own ledger over the bracket this window ran in, where one could be taken.
    conservation: Option<Conservation>,
    /// The DUT had to be reset after this window before the search could go on.
    recovered: bool,
}

impl Row {
    fn pass(&self) -> bool {
        self.outcome.as_ref().is_ok_and(|run| run.verdict().pass)
    }
}

/// Bytes in a short message: about a line of console output, and about as many keystrokes.
const MESSAGE_BYTES: usize = 64;

/// A short message at one rung, in the two shapes a terminal produces.
struct Message {
    /// A whole line handed over at once.
    line: Ledger,
    /// The same bytes one at a time, which is what typing looks like.
    keystrokes: Ledger,
    /// What the DUT's UART counted while the two exchanges ran.
    stats: Option<daplink_test_protocol::Stats>,
}

impl Message {
    fn exact(&self) -> bool {
        self.line.byte_exact && self.keystrokes.byte_exact
    }

    /// A byte-exact exchange still fails when the DUT latched a stuck DMA abort, because the
    /// ledger that vouches for it is suspect.
    fn pass(&self) -> bool {
        self.exact() && self.stats.is_none_or(|s| s.stuck == 0)
    }
}

struct MessageRow {
    rung: Rung,
    /// The exchange, or why it could not be run at all.
    outcome: std::result::Result<Message, String>,
}

struct Staged<'a> {
    bench: &'a Bench,
    args: &'a LadderArgs,
    tty: &'a mut Tty,
    bus: &'a BusBudget,
    kept: &'a [Rung],
    /// Reopened per sustain window, so its bracket carries the probe's counters instead of a null.
    dut: Option<Dut>,
    rows: Vec<Row>,
    messages: Vec<MessageRow>,
}

impl Staged<'_> {
    fn release(&mut self) {
        self.dut = None;
    }

    fn counters(&self) -> Option<VcomCounters> {
        match self.bench.probe_counters() {
            Ok(counters) => counters,
            Err(e) => {
                eprintln!("warning: the probe counter read failed: {e:#}");
                None
            }
        }
    }

    /// Does a short message still come back byte-exact at this rung? A saturating stream can miss
    /// its bar where a log line still survives, so this ceiling is asked of every rung.
    fn message(&mut self, index: usize) -> stream::Grade {
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
            // The deadline follows the slowest leg, so a probe that lands under the asked rate
            // is given the time its own bytes take rather than being failed for arithmetic.
            let baud = rung
                .dut_realized
                .min(rung.probe_realized.unwrap_or(rung.nominal));
            let payload = tty::pattern(MESSAGE_BYTES);
            let line = tty.echo_paced(&payload, payload.len(), baud)?;
            let keystrokes = tty.echo_paced(&payload, 1, baud)?;
            let stats = dut.stats().ok();
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
        self.messages.push(MessageRow {
            rung,
            outcome: outcome.map_err(|e| format!("{e:#}")),
        });
        grade
    }

    /// A window that errors fails only that rung; the DUT is reset and the search goes on.
    fn window(&mut self, stage: Stage, index: usize, secs: f64, warmup: f64) -> stream::Grade {
        let rung = self.kept[index].clone();
        // Only the sustain tier reads the probe's counters; every rung's rate transition puts the
        // session down anyway, so the bracket costs no session cycle of its own.
        let bracket = stage == Stage::Sustain;
        eprintln!("  {} at {}: {:.0}s window", stage.key(), rung.nominal, secs);

        // The bracket opens after the session and after the probe confirms the rate applied: the
        // reset and the receiver's resynchronisation each leave an error the stream never made,
        // and a window opened before the rate lands is scored against the previous rung's clock.
        let mut before = None;
        let outcome = (|| -> Result<(Plan, (Option<Run>, Run))> {
            let (bench, tty) = (self.bench, &mut *self.tty);
            // Returns with the session released, which is where the vendor query can be reached:
            // the confirmation and the bracket read share one release rather than two.
            let prepared = prepare_settled(bench, &mut self.dut, tty, rung.nominal)?;
            denominator_checked(&prepared, &rung, self.args.budget_pct)?;
            before = bracket.then(|| self.counters()).flatten();
            let dut = attach_armed(self.bench, &mut self.dut)?;
            let (tty, args) = (&mut *self.tty, &*self.args);
            let plan = plan_for(
                Shape::Duplex,
                &args.traffic,
                rung.dut_realized,
                rung.probe_realized,
                secs,
                1.0,
                stream::CREDIT_WINDOW,
            );
            windows_of(dut, tty, &plan, &args.traffic, warmup).map(|got| (plan, got))
        })();

        if bracket {
            self.release();
        }
        let after = bracket.then(|| self.counters()).flatten();
        let conservation = before
            .zip(after)
            .map(|(before, after)| Conservation { before, after });

        let outcome = match outcome {
            Ok((plan, (warmed, mut run))) => {
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
                Err(format!("{e:#}"))
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

    /// A rung whose window errored takes its session with it; the next session resets the DUT on
    /// the way in.
    fn reset_after(&mut self, outcome: &std::result::Result<Run, String>) -> bool {
        if !self.bench.reset_dut {
            return false;
        }
        if let (false, Some(dut)) = (outcome.is_err(), self.dut.as_mut()) {
            match dut.reset() {
                Ok(()) => return true,
                Err(e) => eprintln!("warning: the DUT reset failed: {e:#}"),
            }
        }
        self.bench.arm_reset();
        self.release();
        true
    }

    fn append(&self, rung: &Rung, run: &Run, phase: &'static str, clamp: Clamp) {
        self.bench.append_v1(&record::V1Record {
            probe: &self.bench.selector,
            verb: "ladder",
            phase,
            baud_nominal: rung.nominal,
            probe_realized: rung.probe_realized,
            run,
            clamp: Some(clamp),
            note: None,
        });
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
    };
    let limits = args.stages.limits();
    // Only the sustain tier certifies, so only it pays for a warmup.
    let outcome = lattice::staged(kept.len(), limits, |stage, index| match stage {
        Stage::Message => staged.message(index),
        Stage::Sweep => staged.window(stage, index, args.stages.sweep_secs, 0.0),
        Stage::Screen => staged.window(stage, index, args.stages.screen_secs, 0.0),
        Stage::Sustain => staged.window(stage, index, sustain_secs, args.windows.warmup),
    });

    let rp = match (outcome.sustained, args.stages.rp_secs > 0.0) {
        (Some(index), true) => return_priority(&mut staged, index, args.stages.rp_secs),
        _ => None,
    };
    staged.release();

    let report = render_report(&staged, dut_max, rungs, &outcome, &rp);
    println!();
    print!("{report}");
    let summary = staged_summary(&staged, dut_max, sustain_secs, &outcome);
    if let Err(e) = record::append_line(&bench.record_path, &summary) {
        eprintln!("warning: record append failed: {e}");
    }
    Ok(())
}

/// One asymmetric window at the ceiling: the host leg paced down so the return leg keeps the bridge
/// to itself.
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
            None
        }
    }
}

/// Wall time one window costs beyond its own seconds: setup, settling, draining, and control
/// exchanges.
const WINDOW_OVERHEAD_SECS: f64 = 1.5;
/// Wall time one probe-rs session costs to open. Every rung pays one: confirming the probe applied
/// the rung's rate needs the DAP interface a session holds, so preparation puts the session down
/// and opens it again afterwards.
const SESSION_SECS: f64 = 3.0;

/// The `plan` verb: what the default run would cost, without touching a rig.
fn plan_verb(args: PlanArgs) -> Result<()> {
    let bus = BusBudget::for_packet(args.packet);
    let rates = lattice::candidate_rates(args.dut_max);
    let (sustain_secs, _, sustain_phase) =
        args.windows.resolve(args.stages.sustain_secs, "sustain");
    let plan_at = |rate: u32| {
        plan_for(
            Shape::Duplex,
            &args.traffic,
            rate,
            // The plan verb has no hardware to ask, so both ends are taken at the same rate.
            None,
            args.stages.sweep_secs,
            1.0,
            stream::CREDIT_WINDOW,
        )
    };
    println!(
        "plan for a DUT max of {} against a {} bulk budget, DAP profile {}",
        args.dut_max,
        if bus.high_speed() {
            "high-speed"
        } else {
            "full-speed"
        },
        args.traffic.dap.key(),
    );
    println!("  the rungs both ends realize are only known on the rig; this assumes they agree");
    println!(
        "  {:>10}  {:>10}  {:>10}  kept",
        "rung", "slots/ms", "budget"
    );
    let mut kept = 0usize;
    for &rate in &rates {
        let clamp = budget::clamp(&plan_at(rate).demand(args.traffic.dap.nominal_bps()), &bus);
        let fits = clamp.fits || args.traffic.allow_over_budget;
        kept += usize::from(fits);
        println!(
            "  {:>10}  {:>10.2}  {:>10.2}  {}",
            rate,
            clamp.demand,
            clamp.budget,
            if fits {
                "yes".to_string()
            } else {
                let paced = budget::clamp(
                    &plan_at(rate)
                        .as_return_priority()
                        .demand(args.traffic.dap.nominal_bps()),
                    &bus,
                );
                format!(
                    "no (over the bus budget; a paced host leg {})",
                    if paced.fits { "fits" } else { "does not fit" }
                )
            },
        );
    }
    if kept == 0 {
        bail!("no rung fits the bus budget; the pairing has nothing the staged search can run");
    }

    let window = |secs: f64| secs + args.traffic.drain_ms as f64 / 1000.0 + WINDOW_OVERHEAD_SECS;
    // A rung is a window plus the session its rate confirmation cycles.
    let rung = |secs: f64| window(secs) + SESSION_SECS;
    // The sustain tier pays a second session: its counter bracket reattaches after the read.
    let sustain = rung(sustain_secs) + window(args.windows.warmup) + SESSION_SECS;
    // Two short exchanges plus the confirmed baud change dominate a message rung; the bytes
    // themselves are nothing even at the slowest rung.
    let messages = kept as f64 * rung(0.0);
    let sweep = kept as f64 * rung(args.stages.sweep_secs);
    let best =
        messages + sweep + rung(args.stages.screen_secs) + sustain + rung(args.stages.rp_secs);
    let worst = messages
        + sweep
        + kept.min(args.stages.screen_max) as f64 * rung(args.stages.screen_secs)
        + kept.min(args.stages.sustain_rungs) as f64 * sustain;
    println!();
    println!(
        "stages: a short message at each of {kept} rungs, sweep {:.1}s over the same rungs, \
         screen {:.1}s over at most {} of them, sustain \
         {:.1}s ({sustain_phase}) after a {:.1}s warmup at up to {} rungs, return priority \
         {:.1}s at the ceiling",
        args.stages.sweep_secs,
        args.stages.screen_secs,
        kept.min(args.stages.screen_max),
        sustain_secs,
        args.windows.warmup,
        kept.min(args.stages.sustain_rungs),
        args.stages.rp_secs,
    );
    println!(
        "checkpoints every {:.1}s: {} inside a sweep window, {} inside a screen, {} inside a \
         sustain window",
        args.traffic.checkpoint,
        stream::checkpoints_inside(args.stages.sweep_secs, args.traffic.checkpoint),
        stream::checkpoints_inside(args.stages.screen_secs, args.traffic.checkpoint),
        stream::checkpoints_inside(sustain_secs, args.traffic.checkpoint),
    );
    println!(
        "estimated wall time: {} if the first screened rung holds, {} if the search spends \
         everything it is allowed",
        clock_time(best),
        clock_time(worst),
    );
    Ok(())
}

fn clock_time(secs: f64) -> String {
    let whole = secs.max(0.0).round() as u64;
    format!("{}m {:02}s", whole / 60, whole % 60)
}

/// Both directions' series shapes, or a dash where no checkpoint fell inside the window.
fn shape_of(run: &Run) -> String {
    if run.series.points.is_empty() {
        return "--".to_string();
    }
    format!(
        "{}/{}",
        run.series.trend(Direction::HostToDut).key(),
        run.series.trend(Direction::DutToHost).key(),
    )
}

fn render_report<'a>(
    staged: &'a Staged,
    dut_max: u32,
    rungs: &[Rung],
    outcome: &'a StagedOutcome,
    rp: &'a Option<Run>,
) -> String {
    let (bench, args) = (staged.bench, staged.args);
    let mismatched = rungs.iter().filter(|r| !r.agreed(args.budget_pct)).count();
    Report {
        probe: &bench.selector,
        chip: bench.chip,
        port: &bench.port_name,
        records: bench.record_path.display().to_string(),
        dut_max,
        dap: args.traffic.dap,
        interval: args.traffic.interval,
        sweep_secs: args.stages.sweep_secs,
        candidates: rungs.len(),
        mismatched,
        clamped: rungs.len() - mismatched - staged.kept.len(),
        kept: staged.kept,
        rows: &staged.rows,
        messages: &staged.messages,
        outcome,
        rp,
    }
    .render()
}

struct Report<'a> {
    probe: &'a str,
    chip: &'a str,
    port: &'a str,
    records: String,
    dut_max: u32,
    dap: DapProfile,
    interval: u32,
    sweep_secs: f64,
    candidates: usize,
    mismatched: usize,
    clamped: usize,
    kept: &'a [Rung],
    rows: &'a [Row],
    messages: &'a [MessageRow],
    outcome: &'a StagedOutcome,
    rp: &'a Option<Run>,
}

impl Report<'_> {
    fn row(&self, stage: Stage, index: usize) -> Option<&Row> {
        let rung = &self.kept[index];
        self.rows
            .iter()
            .find(|r| r.stage == stage && r.rung.nominal == rung.nominal)
    }

    fn render(&self) -> String {
        let Report {
            kept,
            outcome,
            rp,
            dut_max,
            ..
        } = *self;
        let mut s = String::with_capacity(4096);
        let (mismatched, clamped) = (self.mismatched, self.clamped);
        let _ = writeln!(s, "== vcom-bench pairing report ==");
        let _ = writeln!(
            s,
            "probe {}  target {}  tty {}",
            self.probe, self.chip, self.port
        );
        let _ = writeln!(
            s,
            "lattice: {} rates asked, {} kept ({mismatched} rate mismatch, {clamped} over the bus \
         budget), DUT max {dut_max}",
            self.candidates,
            kept.len(),
        );
        let _ = writeln!(
            s,
            "DAP profile {}, marker interval {} B, pass bar {:.0}% of the payload budget",
            self.dap.key(),
            self.interval,
            stream::PASS_FRAC * 100.0,
        );

        let ceiling = |index: Option<usize>| match index {
            Some(i) => rung_line(&kept[i]),
            None => "none".to_string(),
        };
        let _ = writeln!(s);
        // The ceiling a terminal user feels, and the one they feel first: it can sit well above
        // the rate at which a saturating stream still makes its bar.
        let _ = writeln!(
            s,
            "short-message ceiling: {} -- {MESSAGE_BYTES} B line and {MESSAGE_BYTES} keystrokes \
             echoed byte-exact",
            ceiling(outcome.message),
        );
        if outcome.message_gap {
            let _ = writeln!(
                s,
                "  a rung below that one failed, so the short-message ceiling has a hole under it"
            );
        }
        let _ = writeln!(
            s,
            "stream sweep ceiling: {} over {:.1}s",
            ceiling(outcome.swept),
            self.sweep_secs
        );
        if outcome.sweep_pessimistic {
            let _ = writeln!(
                s,
                "  a rung above it failed that window and held the screen, so the sweep                  understated this pairing and its ceiling is the screened rung"
            );
        }
        // Two ceilings, because they answer different questions: the highest rate that loses
        // nothing at all, and the highest that carries its budget with every loss counted.
        match outcome.sustained_qualified {
            Some(i) if Some(i) != outcome.sustained => {
                let _ = write!(s, "sustained at 99.9%:  {}", ceiling(Some(i)));
                match self
                    .row(Stage::Sustain, i)
                    .and_then(|row| row.outcome.as_ref().ok())
                {
                    Some(run) => {
                        let d = run.conservation.map(|c| c.delta());
                        let _ = writeln!(
                            s,
                            " -- {:.1}% / {:.1}% of budget over {:.1}s, {} counted loss events",
                            run.budget_frac(Direction::HostToDut) * 100.0,
                            run.budget_frac(Direction::DutToHost) * 100.0,
                            run.plan.secs,
                            d.map_or(0, |d| d.host_dropped
                                + d.ring_full
                                + d.frame_err
                                + d.overrun),
                        );
                    }
                    None => {
                        let _ = writeln!(s);
                    }
                }
            }
            _ => {}
        }
        match outcome
            .sustained
            .and_then(|i| self.row(Stage::Sustain, i).map(|row| (i, row)))
        {
            Some((i, row)) => {
                let _ = writeln!(s, "sustained ceiling:   {}", ceiling(Some(i)));
                if let Ok(run) = &row.outcome {
                    let _ = writeln!(
                    s,
                    "  over {:.1}s: host -> DUT {:.1}% of budget ({:.0} B/s), DUT -> host {:.1}% \
                     ({:.0} B/s)",
                    run.plan.secs,
                    run.budget_frac(Direction::HostToDut) * 100.0,
                    run.goodput_bps(Direction::HostToDut),
                    run.budget_frac(Direction::DutToHost) * 100.0,
                    run.goodput_bps(Direction::DutToHost),
                );
                }
            }
            None => {
                let _ = writeln!(
                    s,
                    "sustained ceiling:   none -- {}",
                    if outcome.screened.is_some() {
                        "no rung below the screened boundary held byte-exact"
                    } else if outcome.swept.is_none() {
                        "no rung swept clean, so there was nothing to screen"
                    } else if outcome.screen_exhausted {
                        "the screen tried its limit of rungs without finding one that held"
                    } else {
                        "every rung the sweep kept failed its screen"
                    }
                );
            }
        }
        match rp {
            Some(run) => {
                let _ = writeln!(
                s,
                "return priority at the ceiling: host leg at {:.0}% of the line rate, DUT -> host \
                 {:.1}% of budget ({:.0} B/s) over {:.1}s -- {}",
                run.plan.host_frac * 100.0,
                run.budget_frac(Direction::DutToHost) * 100.0,
                run.goodput_bps(Direction::DutToHost),
                run.plan.secs,
                verdict_word(run.verdict().pass),
            );
            }
            None => {
                let _ = writeln!(s, "return priority at the ceiling: not measured");
            }
        }

        let _ = writeln!(s);
        let _ = writeln!(s, "per-rung short messages");
        let _ = writeln!(
            s,
            "  {:>10}  {:>7}  {:>9}  {:>11}  {:>6}  {:>6}",
            "rung", "verdict", "line", "keystrokes", "ferr", "ovr",
        );
        for row in self.messages {
            match &row.outcome {
                Ok(message) => {
                    let _ = writeln!(
                        s,
                        "  {:>10}  {:>7}  {:>9}  {:>11}  {:>6}  {:>6}",
                        row.rung.nominal,
                        verdict_word(message.pass()),
                        divergence(&message.line),
                        divergence(&message.keystrokes),
                        message
                            .stats
                            .map_or_else(|| "--".to_string(), |s| s.ferr.to_string()),
                        message
                            .stats
                            .map_or_else(|| "--".to_string(), |s| s.ovr.to_string()),
                    );
                }
                Err(e) => {
                    let _ = writeln!(
                        s,
                        "  {:>10}  {:>7}  the exchange could not be run -- {e}",
                        row.rung.nominal, "ERROR",
                    );
                }
            }
        }

        let _ = writeln!(s);
        let _ = writeln!(s, "per-rung verdicts");
        let _ = writeln!(
            s,
            "  {:>10}  {:>7}  {:>7}  {:>7}  {:>8}  {:>8}  {:>7}  shape",
            "rung", "stage", "window", "verdict", "h2d", "d2h", "held",
        );
        for row in self.rows {
            let Ok(run) = &row.outcome else {
                let _ = writeln!(
                    s,
                    "  {:>10}  {:>7}  {:>7}  {:>7}  {:>8}  {:>8}  {:>7}  window errored",
                    row.rung.nominal,
                    row.stage.key(),
                    "--",
                    "ERROR",
                    "--",
                    "--",
                    "--",
                );
                continue;
            };
            let _ = writeln!(
                s,
                "  {:>10}  {:>7}  {:>6.1}s  {:>7}  {:>7.1}%  {:>7.1}%  {:>6.1}s  {}",
                row.rung.nominal,
                row.stage.key(),
                run.plan.secs,
                verdict_word(run.verdict().pass),
                run.budget_frac(Direction::HostToDut) * 100.0,
                run.budget_frac(Direction::DutToHost) * 100.0,
                run.sustained_secs(),
                shape_of(run),
            );
        }
        if outcome.non_monotone {
            let _ = writeln!(
                s,
                "  the lattice is not monotone: a rung above a failing one swept clean, and the \
             ceiling is the highest rung that held, not the first boundary"
            );
        }

        let _ = writeln!(s);
        let _ = writeln!(s, "detail");
        for row in self.rows {
            let Ok(run) = &row.outcome else {
                let _ = writeln!(
                    s,
                    "  {} {}: the window could not be run -- {}",
                    row.rung.nominal,
                    row.stage.key(),
                    row.outcome.as_ref().err().map_or("", String::as_str),
                );
                continue;
            };
            let verdict = run.verdict();
            if verdict.pass && run.series.points.is_empty() {
                continue;
            }
            let _ = writeln!(
                s,
                "  {} {} {} (held {:.1}s of {:.1}s)",
                row.rung.nominal,
                row.stage.key(),
                verdict_word(verdict.pass),
                run.sustained_secs(),
                run.plan.secs,
            );
            for reason in &verdict.reasons {
                let _ = writeln!(s, "    {reason}");
            }
            for note in &verdict.notes {
                let _ = writeln!(s, "    note: {note}");
            }
            render_series(&mut s, run, "    ");
            if row.recovered {
                let _ = writeln!(s, "    the DUT was reset before the search went on");
            }
        }

        let _ = writeln!(s);
        let sustain_dap = outcome
            .sustained
            .and_then(|i| self.row(Stage::Sustain, i))
            .and_then(|row| row.outcome.as_ref().ok())
            .map(|run| &run.dap);
        match sustain_dap {
            Some(dap) => {
                let _ = writeln!(
                    s,
                    "DAP load in the sustained window: profile {}, {:.0} commands/s, {:.0} B/s",
                    dap.profile.key(),
                    dap.cps(),
                    dap.bps(),
                );
            }
            None => {
                let _ = writeln!(s, "DAP load: profile {}", self.dap.key());
            }
        }
        let bracket = outcome
            .sustained
            .and_then(|i| self.row(Stage::Sustain, i))
            .and_then(|row| row.conservation);
        match bracket {
            Some(c) => {
                let d = c.delta();
                let counter = |value: u32, slot: usize| stream::render_counter(&d, value, slot);
                let _ = writeln!(
                    s,
                    "probe counters over the sustained rung's bracket: {} off the UART, {} to the \
                 host, {} host-dropped, {} ring-full, {} framing, {} overruns -- {}",
                    d.uart_rx,
                    d.cdc_in,
                    d.host_dropped,
                    counter(d.ring_full, stream::SLOT_RING_FULL),
                    counter(d.frame_err, stream::SLOT_FRAME_ERR),
                    counter(d.overrun, stream::SLOT_OVERRUN),
                    if c.clean() {
                        "reconciles"
                    } else {
                        "DOES NOT reconcile"
                    },
                );
            }
            None => {
                let _ = writeln!(
                    s,
                    "probe counters: not served by this firmware, or no sustained rung to bracket"
                );
            }
        }
        let _ = writeln!(s, "records: {}", self.records);
        s
    }
}

/// Sweep every kept rung, reporting each direction's goodput and the joint verdict.
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
        // A rung that errors is a FAIL for that rung, not the end of the sweep.
        let run = rig
            .window(
                rung,
                &plan,
                &args.traffic,
                args.windows.warmup,
                phase,
                Some(clamp),
            )
            .map_err(|e| eprintln!("    window errored ({e:#}); counted as FAIL"))
            .ok();
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

/// Render the staged search's summary as its own JSON line.
fn staged_summary(
    staged: &Staged,
    dut_max: u32,
    sustain_secs: f64,
    outcome: &StagedOutcome,
) -> String {
    let (bench, args, kept) = (staged.bench, staged.args, staged.kept);
    let rate = |index: Option<usize>| {
        index.map_or_else(|| "null".to_string(), |i| kept[i].nominal.to_string())
    };
    let mut s = String::with_capacity(1024);
    let _ = write!(
        s,
        "{{\"schema_version\":{},\"tool\":\"vcom-bench\",\"unix_s\":{},\"probe\":{},\
         \"verb\":\"ladder\",\"phase\":\"summary\",\"shape\":\"v1\",\"mode\":\"staged\",\
         \"dut_baud_max\":{dut_max},\"budget_pct\":{},\"interval\":{},\"dap\":{},\
         \"sweep_s\":{},\"screen_s\":{},\"sustain_s\":{},\"checkpoint_s\":{},\
         \"message_ceiling\":{},\"message_gap\":{},\"sweep_ceiling\":{},\
         \"screened_ceiling\":{},\"sustained_ceiling\":{},\
         \"non_monotone\":{},\"sweep_pessimistic\":{},\"screen_exhausted\":{},\
         \"windows\":[",
        record::SCHEMA_VERSION,
        record::unix_now(),
        record::quote(&bench.selector),
        args.budget_pct,
        args.traffic.interval,
        record::quote(args.traffic.dap.key()),
        args.stages.sweep_secs,
        args.stages.screen_secs,
        sustain_secs,
        args.traffic.checkpoint,
        rate(outcome.message),
        outcome.message_gap,
        rate(outcome.swept),
        rate(outcome.screened),
        rate(outcome.sustained),
        outcome.non_monotone,
        outcome.sweep_pessimistic,
        outcome.screen_exhausted,
    );
    for (i, row) in staged.rows.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let _ = write!(
            s,
            "{{\"stage\":{},\"nominal\":{},\"pass\":{},\"held_s\":{:.2}}}",
            record::quote(row.stage.key()),
            row.rung.nominal,
            row.pass(),
            row.outcome.as_ref().map_or(0.0, Run::sustained_secs),
        );
    }
    s.push_str("],\"messages\":[");
    for (i, row) in staged.messages.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let stats = row.outcome.as_ref().ok().and_then(|m| m.stats);
        let _ = write!(
            s,
            "{{\"nominal\":{},\"pass\":{},\"ferr\":{},\"ovr\":{}}}",
            row.rung.nominal,
            row.outcome.as_ref().is_ok_and(Message::exact),
            stats.map_or_else(|| "null".to_string(), |s| s.ferr.to_string()),
            stats.map_or_else(|| "null".to_string(), |s| s.ovr.to_string()),
        );
    }
    s.push_str("]}");
    s
}

/// Render the ladder summary as its own JSON line.
#[cfg(test)]
mod tests {
    use super::*;
    use daplink_test_protocol::v1::{RxCounters, TxCounters, V1Report};
    use lattice::ProbeResult;
    use stream::{Abandon, Checkpoint, Series, Slice};

    fn rung(nominal: u32) -> Rung {
        Rung {
            nominal,
            dut_realized: nominal,
            probe_realized: Some(nominal),
        }
    }

    fn reading(realized: u32, applied: Option<u32>, settled: Option<bool>) -> probe::VcomBaud {
        probe::VcomBaud {
            realized: (realized != 0).then_some(realized),
            applied,
            settled,
        }
    }

    /// The defect this reply shape exists for: a rate change still in flight must never conclude
    /// the wait, however well the rate being left happens to match the one asked for.
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

    /// A firmware that cannot report applied-ness leaves agreement as the only evidence, which is
    /// exactly the inference the reply shape exists to replace.
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
        let both = (1 << stream::SLOT_CDC_OUT) | (1 << stream::SLOT_UART_TX);
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

    /// A window is scored against the rate the probe confirmed for it, so a confirmation that
    /// contradicts the lattice refuses the window instead of rating it.
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

    /// One fixed bound is wrong at both ends of the lattice, so the drain is derived from the
    /// bytes the probe still holds and the rate they leave at.
    #[test]
    fn the_bound_covers_the_queue_at_the_rate_it_drains_at() {
        // 2047 bytes at 9600 is 2.13 s, which the old 400 ms bound expired four times over.
        assert_eq!(
            settle_bound(InFlight::Bytes(2_047), Some(9_600)),
            LATTICE_SETTLE_BASE + Duration::from_secs_f64(20_470.0 / 9_600.0)
        );
        // The same queue at 4 Mbaud costs 5 ms, so the base is what bounds it.
        assert!(
            settle_bound(InFlight::Bytes(2_047), Some(4_000_000))
                < LATTICE_SETTLE_BASE + Duration::from_millis(6)
        );
        // A quiesced line derives the base alone.
        assert_eq!(
            settle_bound(InFlight::Bytes(0), Some(9_600)),
            LATTICE_SETTLE_BASE
        );
        // Nothing to derive from is the base, not an unbounded wait.
        assert_eq!(
            settle_bound(InFlight::Unknown, Some(9_600)),
            LATTICE_SETTLE_BASE
        );
        assert_eq!(
            settle_bound(InFlight::Bytes(2_047), None),
            LATTICE_SETTLE_BASE
        );
        assert_eq!(
            settle_bound(InFlight::Bytes(2_047), Some(0)),
            LATTICE_SETTLE_BASE
        );
        // A deficit sampled across a change can wrap; the cap is what keeps that finite.
        assert_eq!(
            settle_bound(InFlight::Bytes(u32::MAX), Some(9_600)),
            LATTICE_SETTLE_MAX
        );
    }

    /// Only the deficit can say the transmitter emptied; without the forward ledger quiescing
    /// has nothing to wait on.
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
            complete: true,
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

    /// A lattice that sweeps clean to 1.5 M, screens the boundary at 921600, and dies 12 s into
    /// sustain above it.
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
