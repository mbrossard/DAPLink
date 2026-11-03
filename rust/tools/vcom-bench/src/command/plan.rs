//! The `plan` verb: what a default `ladder` would cost, before any of it runs.

use anyhow::{bail, Context, Result};

use super::session::{plan_for, Bench};
use crate::budget::{self, BusBudget};
use crate::cli::{PlanArgs, DEFAULT_PACKET};
use crate::lattice;
use crate::stream::{self, Shape};

const WINDOW_OVERHEAD_SECS: f64 = 1.5;

const SESSION_SECS: f64 = 3.0;

/// Where one of the plan's inputs came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Source {
    /// Read off the probe this invocation named.
    Live,
    /// Supplied on the command line.
    Given,
}

impl Source {
    fn label(self) -> &'static str {
        match self {
            Source::Live => "live",
            Source::Given => "given",
        }
    }
}

/// A live reading and a supplied value either agree, or the plan names the one it ignores.
fn reconcile(
    what: &str,
    flag: &str,
    live: Option<u32>,
    given: Option<u32>,
) -> Result<Option<(u32, Source)>> {
    match (live, given) {
        (Some(live), Some(given)) if live != given => bail!(
            "the {what} read from the rig is {live} but --{flag} {given} was given; drop \
             --{flag} to plan against the rig, or drop --probe to plan for {given}"
        ),
        (Some(live), _) => Ok(Some((live, Source::Live))),
        (None, Some(given)) => Ok(Some((given, Source::Given))),
        (None, None) => Ok(None),
    }
}

/// The packet geometry and DUT ceiling the plan derives from, and what each is known from.
#[derive(Debug)]
struct PlanInputs {
    packet: usize,
    packet_source: Source,
    packet_count: Option<u8>,
    dut_max: u32,
    dut_max_source: Source,
    /// What the probe declined to answer, printed so no assumption is silent.
    notes: Vec<String>,
}

fn plan_inputs(args: &PlanArgs) -> Result<PlanInputs> {
    let (mut live_packet, mut packet_count, mut live_dut_max) = (None, None, None);
    let mut notes = Vec::new();
    if let Some(rig) = args.rig() {
        let bench = Bench::unidentified(&rig)?;
        let (geometry, endpoint) = bench.probe_geometry()?;
        packet_count = geometry.count;
        live_packet = Some(u32::from(geometry.size.unwrap_or(endpoint as u16)));
        if geometry.size.is_none() {
            notes.push(format!(
                "the probe declined DAP_Info(PacketSize); its bulk endpoint size {endpoint} \
                 stood in"
            ));
        }
        if packet_count.is_none() {
            notes.push("the probe declined DAP_Info(PacketCount)".to_string());
        }
        match bench.open_dut().and_then(|mut dut| dut.baud_max()) {
            Ok(max) => live_dut_max = Some(max),
            Err(e) => notes.push(format!(
                "the DUT did not answer BAUD MAX ({e:#}); the ceiling is whatever --dut-max says"
            )),
        }
    }
    let packet = reconcile(
        "probe packet size",
        "packet",
        live_packet,
        args.packet.map(|p| p as u32),
    )?
    .unwrap_or((DEFAULT_PACKET as u32, Source::Given));
    let (dut_max, dut_max_source) =
        reconcile("DUT BAUD MAX", "dut-max", live_dut_max, args.dut_max)?.context(
            "plan needs a rate ceiling: pass --dut-max, or --probe to read it from the wired DUT",
        )?;
    Ok(PlanInputs {
        packet: packet.0 as usize,
        packet_source: packet.1,
        packet_count,
        dut_max,
        dut_max_source,
        notes,
    })
}

/// The `plan` verb: what the default run would cost, against a live probe or a stated pairing.
pub(super) fn plan_verb(args: PlanArgs) -> Result<()> {
    let inputs = plan_inputs(&args)?;
    let bus = BusBudget::for_packet(inputs.packet);
    let rates = lattice::candidate_rates(inputs.dut_max);
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
        inputs.dut_max,
        if bus.high_speed() {
            "high-speed"
        } else {
            "full-speed"
        },
        args.traffic.dap.key(),
    );
    println!(
        "  packet size {} ({}), packet count {}, DUT max {} ({})",
        inputs.packet,
        inputs.packet_source.label(),
        inputs
            .packet_count
            .map_or_else(|| "unread".to_string(), |c| format!("{c} (live)")),
        inputs.dut_max,
        inputs.dut_max_source.label(),
    );
    for note in &inputs.notes {
        println!("  note: {note}");
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::{Cli, Command};
    use clap::Parser;

    fn plan_args(argv: &[&str]) -> PlanArgs {
        let mut all = vec!["vcom-bench", "plan"];
        all.extend_from_slice(argv);
        let Command::Plan(args) = Cli::parse_from(all).command else {
            unreachable!("the plan verb was asked for")
        };
        args
    }

    #[test]
    fn plan_takes_a_probe_selector_and_still_takes_a_stated_pairing() {
        let live = plan_args(&["--probe", "0d28:0204:AA"]);
        assert_eq!(live.probe.as_deref(), Some("0d28:0204:AA"));
        assert_eq!((live.dut_max, live.packet), (None, None));
        let rig = live.rig().expect("a selector makes a rig");
        assert_eq!(rig.probe, "0d28:0204:AA");
        // A preview reads the DUT; it does not put it through a reset.
        assert!(rig.no_dut_reset);

        let stated = plan_args(&["--dut-max", "2625000", "--packet", "512"]);
        assert!(stated.probe.is_none() && stated.rig().is_none());
        assert_eq!(
            (stated.dut_max, stated.packet),
            (Some(2_625_000), Some(512))
        );
    }

    #[test]
    fn a_stated_pairing_needs_no_hardware_and_keeps_the_assumed_packet_size() {
        let inputs = plan_inputs(&plan_args(&["--dut-max", "2625000"])).expect("no rig is needed");
        assert_eq!((inputs.packet, inputs.dut_max), (64, 2_625_000));
        assert_eq!(inputs.packet_source, Source::Given);
        assert_eq!(inputs.dut_max_source, Source::Given);
        assert!(inputs.packet_count.is_none() && inputs.notes.is_empty());
    }

    #[test]
    fn plan_without_a_ceiling_names_both_ways_to_supply_one() {
        let err = plan_inputs(&plan_args(&[])).expect_err("nothing bounds the lattice");
        let text = format!("{err:#}");
        assert!(
            text.contains("--dut-max") && text.contains("--probe"),
            "{text}"
        );
    }

    #[test]
    fn a_packet_size_contradicting_the_probe_is_refused_rather_than_inherited() {
        let err = reconcile("probe packet size", "packet", Some(512), Some(64))
            .expect_err("the two disagree");
        let text = format!("{err:#}");
        assert!(
            text.contains("512") && text.contains("--packet 64"),
            "{text}"
        );
        assert!(text.contains("--probe"), "{text}");
    }

    #[test]
    fn agreement_and_absence_both_resolve_to_one_source() {
        assert_eq!(
            reconcile("DUT BAUD MAX", "dut-max", Some(2_625_000), Some(2_625_000)).unwrap(),
            Some((2_625_000, Source::Live))
        );
        assert_eq!(
            reconcile("DUT BAUD MAX", "dut-max", Some(2_625_000), None).unwrap(),
            Some((2_625_000, Source::Live))
        );
        assert_eq!(
            reconcile("DUT BAUD MAX", "dut-max", None, Some(9_600)).unwrap(),
            Some((9_600, Source::Given))
        );
        assert_eq!(
            reconcile("DUT BAUD MAX", "dut-max", None, None).unwrap(),
            None
        );
    }
}
