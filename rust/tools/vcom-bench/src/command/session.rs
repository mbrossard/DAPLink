//! Session preparation, line settling, and stream windows.

use std::cell::Cell;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use daplink_test_protocol::Stats;

use super::report::opt_rate;
use super::{
    LATTICE_SETTLE_BASE, LATTICE_SETTLE_MAX, LATTICE_SETTLE_POLL, SETTLE, STATS_LAG, WINDOW_POLL,
};
use crate::budget::{self, BusBudget, Clamp, DapAchieved, DapProfile};
use crate::cli::{BurstDir, RigArgs, Traffic};
use crate::counters::{self, VcomCounters};
use crate::dut::{self, Dut};
use crate::lattice::{self, Rung};
use crate::stream::{self, Clock, Credit, Plan, Run, Sampler, Seeds, Shape};
use crate::tty::{self, Ledger, Tty};
use crate::{lifecycle, probe, record};

pub(super) struct Bench {
    pub(super) selector: String,
    pub(super) firmware_row: firmware_row::Resolved,
    pub(super) firmware_version: Option<String>,
    pub(super) chip: &'static str,
    pub(super) port_name: String,
    pub(super) record_path: PathBuf,
    pub(super) dut_elf: Option<String>,
    pub(super) reset_dut: bool,
    /// Whether the vendor queries may be sent at all; `--no-vendor` clears it.
    pub(super) vendor: bool,
    /// Armed at startup and again by recovery; consumed by the next [`Bench::open_dut`].
    pub(super) pending_reset: Cell<bool>,
    pub(super) budget_pct: f64,
    /// A probe that serves no applied-ness is said so once, not once per window.
    pub(super) unconfirmed_noted: Cell<bool>,
}

impl Bench {
    /// The bench a measurement runs on; a probe that cannot be asked is a record without an answer.
    pub(super) fn resolve(rig: &RigArgs) -> Result<Bench> {
        let mut bench = Self::unidentified(rig)?;
        let (row, version) = probe::DapLink::open(&bench.selector, bench.vendor)
            .map_or((None, None), |mut link| link.firmware_identity());
        bench.firmware_row = firmware_row::resolve(row.as_deref(), rig.firmware_row.as_deref());
        bench.firmware_version = version;
        println!("firmware row {}", bench.firmware_row.describe());
        for warning in &bench.firmware_row.warnings {
            eprintln!("warning: {warning}");
        }
        Ok(bench)
    }

    pub(super) fn unidentified(rig: &RigArgs) -> Result<Bench> {
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
            firmware_row: firmware_row::resolve(None, None),
            firmware_version: None,
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

    pub(super) fn firmware(&self) -> record::Firmware<'_> {
        record::Firmware {
            row: self.firmware_row.row.as_deref(),
            source: self.firmware_row.source.key(),
            version: self.firmware_version.as_deref(),
        }
    }

    pub(super) fn open_dut(&self) -> Result<Dut> {
        let mut dut = Dut::open(&self.selector, self.chip, self.dut_elf.as_deref())?;
        if self.pending_reset.get() {
            dut.reset()
                .context("reset the DUT into a known state (--no-dut-reset skips it)")?;
            self.pending_reset.set(false);
        }
        Ok(dut)
    }

    /// An over-ceiling rung can wedge the DUT's UART hard enough that only a reset clears it.
    pub(super) fn arm_reset(&self) {
        self.pending_reset.set(self.reset_dut);
    }

    /// Must not run while a probe-rs session holds the probe's DAP interface.
    pub(super) fn probe_facts(&self) -> Result<(Option<u32>, BusBudget)> {
        let mut link = probe::DapLink::open(&self.selector, self.vendor)?;
        let budget = BusBudget::for_packet(link.max_packet_size());
        Ok((link.vcom_baud()?.and_then(|b| b.realized), budget))
    }

    /// The probe's `DAP_Info` packet geometry and bulk endpoint size; same exclusion as above.
    pub(super) fn probe_geometry(&self) -> Result<(probe::PacketGeometry, usize)> {
        let mut link = probe::DapLink::open(&self.selector, self.vendor)?;
        let endpoint = link.max_packet_size();
        Ok((link.packet_geometry()?, endpoint))
    }

    pub(super) fn probe_baud_settled(&self, requested: u32) -> Result<Option<u32>> {
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
            BaudSettle::Declined { applied, realized } => Err(super::outcomes::RateDeclined(
                declined_note(requested, applied, realized),
            )
            .into()),
            BaudSettle::Unsettled(u) => bail!("{}", unsettled_note(requested, u)),
        }
    }

    /// Same exclusion as [`Bench::probe_facts`].
    pub(super) fn probe_counters(&self) -> Result<Option<VcomCounters>> {
        probe::DapLink::open(&self.selector, self.vendor)?.vcom_counters()
    }

    pub(super) fn append(&self, r: &record::RunRecord) -> Result<()> {
        super::outcomes::recorded(record::append(&self.record_path, r), &self.record_path)
    }

    pub(super) fn append_v1(&self, r: &record::V1Record) -> Result<()> {
        super::outcomes::recorded(record::append_v1(&self.record_path, r), &self.record_path)
    }
}

pub(super) struct Outcome {
    pub(super) ledger: Ledger,
    pub(super) stats: Stats,
    pub(super) dut_realized: u32,
}

pub(super) fn run_at(
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
pub(super) struct Prepared {
    pub(super) dut_realized: u32,
    pub(super) probe_realized: Option<u32>,
}

#[derive(Clone, Copy, PartialEq)]
pub(super) enum Bracket {
    None,
    Required,
}

pub(super) struct StreamWindow<'a, F> {
    pub(super) bench: &'a Bench,
    pub(super) dut: &'a mut Option<Dut>,
    pub(super) tty: &'a mut Tty,
    pub(super) rung: &'a Rung,
    pub(super) prepared: Option<Prepared>,
    pub(super) bracket: Bracket,
    pub(super) body: F,
}

impl<F, O> lifecycle::Window for StreamWindow<'_, F>
where
    F: FnMut(&mut Dut, &mut Tty) -> Result<O>,
{
    type Output = O;

    fn prepare(&mut self) -> Result<()> {
        let prepared = match self.prepared.take() {
            Some(prepared) => prepared,
            None => prepare_settled(self.bench, self.dut, self.tty, self.rung.nominal)?,
        };
        denominator_checked(&prepared, self.rung, self.bench.budget_pct)
    }

    fn baseline(&mut self) -> Result<Option<VcomCounters>> {
        self.counters()
    }

    fn measure(&mut self) -> Result<O> {
        let dut = attach_armed(self.bench, self.dut)?;
        let outcome = (self.body)(dut, self.tty);
        if outcome.is_err() {
            return super::outcomes::after(
                outcome,
                rearm(dut, self.tty).context("failed window cleanup"),
            );
        }
        outcome
    }

    fn release(&mut self, failed: bool) {
        if failed {
            self.bench.arm_reset();
        }
        if failed || self.bracket != Bracket::None {
            *self.dut = None;
        }
    }

    fn after(&mut self) -> Result<Option<VcomCounters>> {
        self.counters()
    }
}

impl<F> StreamWindow<'_, F> {
    pub(super) fn counters(&self) -> Result<Option<VcomCounters>> {
        if self.bracket == Bracket::None {
            return Ok(None);
        }
        self.bench.probe_counters()
    }
}

pub(super) fn prepare_settled(
    bench: &Bench,
    dut: &mut Option<Dut>,
    tty: &mut Tty,
    baud: u32,
) -> Result<Prepared> {
    if dut.is_none() {
        *dut = Some(bench.open_dut()?);
    }
    let session = dut.as_mut().expect("the session was just opened");
    // Stop even a burst-idle engine before the released counter baseline.
    session.v1_reset()?;
    let dut_realized = session
        .set_baud(baud)?
        .ok_or_else(|| super::outcomes::RateDeclined(format!("the DUT declined BAUD {baud}")))?;
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

pub(super) fn attach_session<'a>(bench: &Bench, dut: &'a mut Option<Dut>) -> Result<&'a mut Dut> {
    if dut.is_none() {
        *dut = Some(Dut::open(
            &bench.selector,
            bench.chip,
            bench.dut_elf.as_deref(),
        )?);
    }
    Ok(dut.as_mut().expect("the session is open"))
}

pub(super) fn attach_armed<'a>(bench: &Bench, dut: &'a mut Option<Dut>) -> Result<&'a mut Dut> {
    let session = attach_session(bench, dut)?;
    session.stats_reset()?;
    Ok(session)
}

pub(super) fn stale_denominator(prepared: &Prepared, rung: &Rung, budget_pct: f64) -> bool {
    match (prepared.probe_realized, rung.probe_realized) {
        (Some(confirmed), Some(derived)) => !rate_agreed(derived, confirmed, budget_pct),
        _ => false,
    }
}

/// Refuse the window rather than rate it against a rate the probe is not running.
pub(super) fn denominator_checked(prepared: &Prepared, rung: &Rung, budget_pct: f64) -> Result<()> {
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
pub(super) fn measure(
    dutctl: &mut Dut,
    tty: &mut Tty,
    plan: &Plan,
    traffic: &Traffic,
) -> Result<Run> {
    let (profile, drain) = (traffic.dap, traffic.drain());
    let clock = Clock::new();
    let reader = match plan.host_verifier() {
        Some(mut verifier) => {
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
pub(super) fn one_window(
    bench: &Bench,
    dut: &mut Option<Dut>,
    tty: &mut Tty,
    rung: &Rung,
    plan: &Plan,
    traffic: &Traffic,
    warmup: f64,
) -> Result<(Option<Run>, Run)> {
    lifecycle::run(&mut StreamWindow {
        bench,
        dut,
        tty,
        rung,
        prepared: None,
        bracket: Bracket::None,
        body: |dut: &mut Dut, tty: &mut Tty| windows_of(dut, tty, plan, traffic, warmup),
    })
    .0
}

pub(super) fn windows_of(
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
    if warmed.is_some() {
        rearm(dutctl, tty)?;
    }
    let run = measure(dutctl, tty, plan, traffic)?;
    Ok((warmed, run))
}

/// Completes `V1 RESET` on normal exit, with a Drop fallback for unwinding.
pub(super) struct DisarmOnExit<'a> {
    bench: &'a Bench,
    armed: bool,
}

impl<'a> DisarmOnExit<'a> {
    pub(super) fn new(bench: &'a Bench) -> Self {
        Self { bench, armed: true }
    }

    fn disarm(&self) -> Result<()> {
        let bench = self.bench;
        Dut::open(&bench.selector, bench.chip, bench.dut_elf.as_deref())
            .and_then(|mut dut| dut.v1_reset())
            .context("V1 RESET on exit; the DUT may not echo")
    }

    pub(super) fn finish(&mut self, primary: Result<()>) -> Result<()> {
        self.armed = false;
        super::outcomes::combine(primary, self.disarm())
    }
}

impl Drop for DisarmOnExit<'_> {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        if let Err(error) = self.disarm() {
            eprintln!("warning: V1 RESET on exit failed; the DUT may not echo: {error:#}");
        }
    }
}

pub(super) fn rearm(dut: &mut Dut, tty: &mut Tty) -> Result<()> {
    dut.v1_reset()?;
    tty.drain()
}

pub(super) struct Rig<'a> {
    pub(super) bench: &'a Bench,
    pub(super) verb: &'static str,
    /// A slot rather than a session: preparation puts the session down to confirm the rate.
    pub(super) dut: &'a mut Option<Dut>,
    pub(super) tty: &'a mut Tty,
    pub(super) holds_records: bool,
    pub(super) held: Vec<Held>,
}

pub(super) struct Held {
    pub(super) rung: Rung,
    pub(super) run: Run,
    pub(super) phase: &'static str,
    pub(super) clamp: Option<Clamp>,
}

impl Rig<'_> {
    pub(super) fn window(
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
        self.record_windows(rung, warmed, run, phase, clamp)
    }

    pub(super) fn record_windows(
        &mut self,
        rung: &Rung,
        warmed: Option<Run>,
        run: Run,
        phase: &'static str,
        clamp: Option<Clamp>,
    ) -> Result<Run> {
        record_windows(warmed, run, phase, |run, phase| {
            self.record(rung, run, phase, clamp)
        })
    }

    pub(super) fn record(
        &mut self,
        rung: &Rung,
        run: &Run,
        phase: &'static str,
        clamp: Option<Clamp>,
    ) -> Result<()> {
        if self.holds_records {
            self.held.push(Held {
                rung: rung.clone(),
                run: run.clone(),
                phase,
                clamp,
            });
            return Ok(());
        }
        self.bench.append_v1(&record::V1Record {
            probe: &self.bench.selector,
            firmware: self.bench.firmware(),
            verb: self.verb,
            phase,
            baud_nominal: rung.nominal,
            probe_realized: rung.probe_realized,
            run,
            clamp,
            note: None,
        })
    }
}

pub(super) fn record_windows(
    warmed: Option<Run>,
    run: Run,
    phase: &'static str,
    mut append: impl FnMut(&Run, &'static str) -> Result<()>,
) -> Result<Run> {
    if let Some(warmed) = warmed {
        append(&warmed, "warmup")?;
    }
    append(&run, phase)?;
    Ok(run)
}

/// Every byte is charged to the direction that carried it.
pub(super) fn plan_for(
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
pub(super) fn admit(plan: &Plan, budget: &BusBudget, traffic: &Traffic) -> Result<Clamp> {
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

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum InFlight {
    /// The forward ledger's deficit: accepted from CDC OUT, not yet transmitted.
    Bytes(u32),
    /// No forward ledger, so the drain cannot be bounded from the probe's own word.
    Unknown,
}

impl InFlight {
    pub(super) fn bytes(self) -> Option<u32> {
        match self {
            InFlight::Bytes(n) => Some(n),
            InFlight::Unknown => None,
        }
    }

    pub(super) fn pending(self) -> bool {
        matches!(self, InFlight::Bytes(1..))
    }

    pub(super) fn clause(self) -> String {
        match self {
            InFlight::Bytes(n) => format!("{n} bytes were still queued at the old rate"),
            InFlight::Unknown => "the probe does not report what it has queued".to_string(),
        }
    }
}

pub(super) fn attests(valid: Option<u32>, slot: usize) -> bool {
    valid.is_some_and(|mask| mask & (1 << slot) != 0)
}

pub(super) fn forward_in_flight(c: &VcomCounters) -> InFlight {
    if attests(c.valid, counters::SLOT_CDC_OUT) && attests(c.valid, counters::SLOT_UART_TX) {
        InFlight::Bytes(c.cdc_out.wrapping_sub(c.uart_tx))
    } else {
        InFlight::Unknown
    }
}

/// `base` is the quiet-line cost of the change; the queue's drain at the old rate is added to it.
pub(super) fn settle_bound(base: Duration, queued: InFlight, old_baud: Option<u32>) -> Duration {
    let drain = queued
        .bytes()
        .zip(old_baud.filter(|&baud| baud > 0))
        .map(|(bytes, baud)| Duration::from_secs_f64(f64::from(bytes) * 10.0 / f64::from(baud)))
        .unwrap_or_default();
    (base + drain).min(LATTICE_SETTLE_MAX)
}

pub(super) fn quiesce_line(
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
    let mut queued = read(link)?;
    if !queued.pending() {
        return Ok(queued);
    }
    let deadline = Instant::now() + settle_bound(LATTICE_SETTLE_BASE, queued, old_baud);
    while Instant::now() < deadline {
        std::thread::sleep(LATTICE_SETTLE_POLL);
        queued = read(link)?;
        if !queued.pending() {
            break;
        }
    }
    Ok(queued)
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) struct Unsettled {
    pub(super) last: Option<u32>,
    pub(super) applied_sourced: bool,
    pub(super) bound: Duration,
    pub(super) queued: InFlight,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum BaudSettle {
    /// The probe applied the rate, and its realized rate agrees with the one requested.
    Settled(u32),
    Disagreed {
        realized: u32,
    },
    /// The probe kept another rate: the request is outside its divisor lattice.
    Declined {
        applied: Option<u32>,
        realized: u32,
    },
    Unsettled(Unsettled),
    /// The firmware does not serve the query at all.
    Unreported,
}

impl BaudSettle {
    /// The rate a leg would be rated against, where one may be quoted at all.
    pub(super) fn realized(self) -> Option<u32> {
        match self {
            BaudSettle::Settled(r) | BaudSettle::Disagreed { realized: r } => Some(r),
            BaudSettle::Declined { .. } | BaudSettle::Unsettled(_) | BaudSettle::Unreported => None,
        }
    }
}

pub(super) fn rate_agreed(requested: u32, realized: u32, budget_pct: f64) -> bool {
    lattice::Rung {
        nominal: requested,
        dut_realized: requested,
        probe_realized: Some(realized),
    }
    .agreed(budget_pct)
}

/// Decide one poll of the query: `None` keeps polling, `Some` is the conclusion.
pub(super) fn settle_step(
    reading: probe::VcomBaud,
    requested: u32,
    budget_pct: f64,
) -> Option<BaudSettle> {
    let agreed = reading
        .realized
        .is_some_and(|r| rate_agreed(requested, r, budget_pct));
    match reading.settled {
        // The firmware knows. Waiting past its own word only re-reads the same answer.
        Some(true) => Some(match reading.realized {
            Some(r) if reading.applied != Some(requested) => BaudSettle::Declined {
                applied: reading.applied,
                realized: r,
            },
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

pub(super) fn settled_probe_baud(
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
                let span = settle_bound(LATTICE_SETTLE_BASE, queued, reading.realized);
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

pub(super) fn declined_note(requested: u32, applied: Option<u32>, realized: u32) -> String {
    let kept = applied.map_or("no rate".to_string(), |a| a.to_string());
    format!(
        "the probe declined {requested}, which its divisors cannot reach, and kept {kept} \
         (realized {realized})"
    )
}

pub(super) fn unsettled_note(requested: u32, u: Unsettled) -> String {
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
