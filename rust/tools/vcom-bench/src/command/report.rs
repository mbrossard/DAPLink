//! Human reports from completed measurements.

use std::fmt::Write as _;

use super::ladder::{MessageRow, Row, Staged, MESSAGE_BYTES};
use super::session::Outcome;
use crate::budget::DapProfile;
use crate::cli::Mode;
use crate::counters::{self, Conservation};
use crate::lattice::{Rung, Stage, StagedOutcome};
use crate::stream::{self, Direction, Run};
use crate::tty::Ledger;

pub(super) fn print_run(run: &Run) {
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

pub(super) fn print_series(run: &Run) {
    let mut out = String::new();
    render_series(&mut out, run, "  ");
    print!("{out}");
}

pub(super) fn render_series(out: &mut String, run: &Run, indent: &str) {
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

pub(super) fn print_verdict(run: &Run) {
    let verdict = run.verdict();
    println!("  verdict: {}", verdict_word(verdict.pass));
    for reason in &verdict.reasons {
        println!("    {reason}");
    }
    for note in &verdict.notes {
        println!("    note: {note}");
    }
}

pub(super) fn print_ledger(out: &Outcome) {
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

pub(super) fn mode_name(mode: Mode) -> &'static str {
    match mode {
        Mode::Paced => "paced",
        Mode::Batched => "batched",
        Mode::Out => "out",
    }
}

pub(super) fn opt_rate(rate: Option<u32>) -> String {
    rate.map_or_else(|| "unreported".to_string(), |r| r.to_string())
}

pub(super) fn verdict_word(pass: bool) -> &'static str {
    if pass {
        "pass"
    } else {
        "FAIL"
    }
}

/// How an echo came back: byte-exact, or the offset where it stopped being so.
pub(super) fn divergence(ledger: &Ledger) -> String {
    match (ledger.byte_exact, ledger.first_divergence) {
        (true, _) => "exact".to_string(),
        (false, Some(at)) => format!("at {at}"),
        (false, None) => "short".to_string(),
    }
}

pub(super) fn rung_line(r: &Rung) -> String {
    format!(
        "{} nominal (DUT realized {}, probe realized {})",
        r.nominal,
        r.dut_realized,
        opt_rate(r.probe_realized)
    )
}

pub(super) fn print_conservation(conservation: Option<&Conservation>) {
    let Some(c) = conservation else {
        println!("  probe counters: not served by this firmware");
        return;
    };
    if let Some((slot, before, after)) = c.restart_slot() {
        println!(
            "  probe counters: the probe restarted inside the window ({slot} {before} -> \
             {after}); not evidence"
        );
        return;
    }
    let d = c.delta();
    let counter = |value: u32, slot: usize| counters::render_counter(&d, value, slot);
    println!(
        "  probe counters over these windows: {} off the UART, {} to the host, {} host-dropped, \
         {} ring-full, {} framing, {} overruns",
        d.uart_rx,
        d.cdc_in,
        d.host_dropped,
        counter(d.ring_full, counters::SLOT_RING_FULL),
        counter(d.frame_err, counters::SLOT_FRAME_ERR),
        counter(d.overrun, counters::SLOT_OVERRUN),
    );
    let sourced = |bits: u32| d.valid.is_some_and(|mask| mask & bits != 0);
    if sourced(1 << counters::SLOT_CDC_OUT | 1 << counters::SLOT_UART_TX) {
        println!(
            "  forward leg: {} in from the host, {} onto the wire",
            counter(d.cdc_out, counters::SLOT_CDC_OUT),
            counter(d.uart_tx, counters::SLOT_UART_TX),
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

pub(super) fn shape_of(run: &Run) -> String {
    if run.series.points.is_empty() {
        return "--".to_string();
    }
    format!(
        "{}/{}",
        run.series.trend(Direction::HostToDut).key(),
        run.series.trend(Direction::DutToHost).key(),
    )
}

pub(super) fn pairing_report<'a>(
    staged: &'a Staged,
    dut_max: u32,
    rungs: &[Rung],
    outcome: &'a StagedOutcome,
    rp: &'a Option<Run>,
) -> Report<'a> {
    let (bench, args) = (staged.bench, staged.args);
    let mismatched = rungs.iter().filter(|r| !r.agreed(args.budget_pct)).count();
    Report {
        probe: &bench.selector,
        firmware: bench.firmware(),
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
}

pub(crate) struct Report<'a> {
    pub(crate) probe: &'a str,
    pub(crate) firmware: crate::record::Firmware<'a>,
    pub(crate) chip: &'a str,
    pub(crate) port: &'a str,
    pub(crate) records: String,
    pub(crate) dut_max: u32,
    pub(crate) dap: DapProfile,
    pub(crate) interval: u32,
    pub(crate) sweep_secs: f64,
    pub(crate) candidates: usize,
    pub(crate) mismatched: usize,
    pub(crate) clamped: usize,
    pub(crate) kept: &'a [Rung],
    pub(crate) rows: &'a [Row],
    pub(crate) messages: &'a [MessageRow],
    pub(crate) outcome: &'a StagedOutcome,
    pub(crate) rp: &'a Option<Run>,
}

impl Report<'_> {
    pub(super) fn row(&self, stage: Stage, index: usize) -> Option<&Row> {
        let rung = &self.kept[index];
        self.rows
            .iter()
            .find(|r| r.stage == stage && r.rung.nominal == rung.nominal)
    }

    pub(super) fn render(&self) -> String {
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
                if let Some((slot, before, after)) = c.restart_slot() {
                    let _ = writeln!(
                        s,
                        "probe counters: the probe restarted inside the window ({slot} {before} \
                         -> {after}); not evidence"
                    );
                } else {
                    let d = c.delta();
                    let counter =
                        |value: u32, slot: usize| counters::render_counter(&d, value, slot);
                    let _ = writeln!(
                        s,
                        "probe counters over the sustained rung's bracket: {} off the UART, {} to \
                     the host, {} host-dropped, {} ring-full, {} framing, {} overruns -- {}",
                        d.uart_rx,
                        d.cdc_in,
                        d.host_dropped,
                        counter(d.ring_full, counters::SLOT_RING_FULL),
                        counter(d.frame_err, counters::SLOT_FRAME_ERR),
                        counter(d.overrun, counters::SLOT_OVERRUN),
                        if c.clean() {
                            "reconciles"
                        } else {
                            "DOES NOT reconcile"
                        },
                    );
                }
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
