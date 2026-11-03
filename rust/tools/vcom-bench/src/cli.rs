//! Command-line configuration and traffic/window options.

pub(super) const SWEEP_SECS: f64 = 2.0;
const SCREEN_SECS: f64 = 10.0;
pub(super) const SUSTAIN_SECS: f64 = 60.0;
use crate::lattice::StagedLimits;
use crate::stream::Shape;
use crate::{budget::DapProfile, contract, lattice, stream};
use clap::{Parser, Subcommand, ValueEnum};
use daplink_test_protocol::v1::BurstSpec;
use std::path::PathBuf;
use std::time::Duration;

/// Measure a probe/DUT VCOM pairing: derived rate lattice, v1 duplex stream ledgers, and the
/// operating ceiling the pairing holds.
#[derive(Parser, Debug)]
#[command(name = "vcom-bench", version, about)]
pub(super) struct Cli {
    #[command(subcommand)]
    pub(super) command: Command,
}

#[derive(Subcommand, Debug)]
pub(super) enum Command {
    /// Diagnostic: one echo run at a fixed rate, in the submission shape asked for.
    Echo(EchoArgs),
    /// Run the v1 stream at one fixed rate, in the shape and under the DAP profile asked for.
    Run(RunArgs),
    /// Find the sustained ceiling: derive the rate lattice, then sweep, screen and sustain.
    Ladder(LadderArgs),
    /// Run the VCOM service-contract cases at ordinary rates and traffic shapes.
    Contract(ContractArgs),
    /// Print what a default `ladder` would run and what it would cost, without running it.
    Plan(PlanArgs),
}

#[derive(clap::Args, Debug)]
pub(super) struct ContractArgs {
    #[command(flatten)]
    pub(super) rig: RigArgs,
    /// Ordinary service rate used except by the line-coding case.
    #[arg(long, default_value_t = contract::DEFAULT_BAUD)]
    pub(super) baud: u32,
    /// Use 100 transition/reopen cycles and a five-minute idle instead of 10 cycles and 10 seconds.
    #[arg(long)]
    pub(super) qualify: bool,
}

#[derive(clap::Args, Debug)]
pub(super) struct RigArgs {
    /// Probe selector `VID:PID:serial`; the serial also selects the tty.
    #[arg(long)]
    pub(super) probe: String,
    /// Wired target name (`stm32f401`, `rp2040`, `rp2350`).
    #[arg(long, default_value = "stm32f401")]
    pub(super) target: String,
    /// DUT ELF, for the RTT control-block symbol fast path.
    #[arg(long)]
    pub(super) dut_elf: Option<String>,
    /// Record file override; defaults to `artifacts/vcom-bench.jsonl` in the checkout.
    #[arg(long)]
    pub(super) record: Option<PathBuf>,
    /// Leave the DUT as it was found instead of resetting it into a known state.
    #[arg(long)]
    pub(super) no_dut_reset: bool,
    /// Never send the probe's vendor queries; counters and realized baud read as unserved.
    #[arg(long)]
    pub(super) no_vendor: bool,
    /// Firmware registry row on the probe, for an image too old to name its own.
    ///
    /// `cargo xtask list` names the rows; `external:<name>` names an image outside them.
    #[arg(long)]
    pub(super) firmware_row: Option<String>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub(super) enum Mode {
    /// One chunk in flight: write a chunk, require its echo before the next.
    Paced,
    /// Stream every chunk while a reader collects the echo concurrently.
    Batched,
    /// Write-only attribution: stream the payload, discard the echo, read the DUT counters.
    Out,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub(super) enum BurstDir {
    Host,
    Dut,
    Both,
}

#[derive(clap::Args, Debug)]
pub(super) struct EchoArgs {
    #[command(flatten)]
    pub(super) rig: RigArgs,
    /// Rate to ask of both ends.
    #[arg(long)]
    pub(super) baud: u32,
    /// Payload length in bytes.
    #[arg(long, default_value_t = 256)]
    pub(super) len: usize,
    /// Host write granularity in bytes.
    #[arg(long, default_value_t = 64)]
    pub(super) chunk: usize,
    /// Submission shape.
    #[arg(long, value_enum, default_value_t = Mode::Batched)]
    pub(super) mode: Mode,
}

#[derive(clap::Args, Debug, Clone)]
pub(super) struct Windows {
    /// Measured window, seconds; the default is the verb's own.
    #[arg(long)]
    pub(super) window: Option<f64>,
    /// Untimed warmup before a measured window, seconds; the staged ladder spends it on sustain
    /// windows only.
    #[arg(long, default_value_t = stream::WARMUP_SECS)]
    pub(super) warmup: f64,
    /// Measured windows per operating point.
    #[arg(long)]
    pub(super) runs: Option<u32>,
    /// Qualify a ceiling: three 60 s windows, all of which must pass.
    #[arg(long)]
    pub(super) qualify: bool,
    /// Soak a ceiling: one 600 s window.
    #[arg(long)]
    pub(super) soak: bool,
}

impl Windows {
    /// The window, the run count, and the phase they are recorded under.
    pub(super) fn resolve(
        &self,
        default_secs: f64,
        default_phase: &'static str,
    ) -> (f64, u32, &'static str) {
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
pub(super) struct Traffic {
    /// Payload bytes between marker frames.
    #[arg(long, default_value_t = stream::DEFAULT_INTERVAL)]
    pub(super) interval: u32,
    /// DAP traffic interleaved with the window, as a named profile.
    #[arg(long, value_enum, default_value_t = DapProfile::None)]
    pub(super) dap: DapProfile,
    /// Drain margin after the nominal window, before the ledgers are collected, milliseconds.
    #[arg(long, default_value_t = 1_000)]
    pub(super) drain_ms: u64,
    /// Seconds between reads of the running ledger inside a window; 0 takes none.
    #[arg(long, default_value_t = stream::CHECKPOINT_SECS)]
    pub(super) checkpoint: f64,
    /// Delivered fraction a checkpoint holds before the rest of the window is abandoned.
    #[arg(long, default_value_t = stream::CHECKPOINT_FLOOR)]
    pub(super) checkpoint_floor: f64,
    /// Shape a transmitter into bursts of this many wire bytes.
    #[arg(long)]
    pub(super) burst: Option<u32>,
    /// Idle gap between bursts, milliseconds.
    #[arg(long, default_value_t = 1)]
    pub(super) idle: u32,
    /// Which transmitter the burst shape applies to.
    #[arg(long, value_enum, default_value_t = BurstDir::Both)]
    pub(super) burst_dir: BurstDir,
    /// Run an operating point the aggregate packet budget refuses.
    #[arg(long)]
    pub(super) allow_over_budget: bool,
}

impl Traffic {
    pub(super) fn burst_spec(&self) -> Option<BurstSpec> {
        self.burst.map(|bytes| BurstSpec {
            bytes,
            idle_ms: self.idle,
        })
    }

    pub(super) fn drain(&self) -> Duration {
        Duration::from_millis(self.drain_ms)
    }
}

#[derive(clap::Args, Debug)]
pub(super) struct RunArgs {
    /// Fail the command if any graded window fails.
    #[arg(long)]
    pub(super) strict: bool,
    #[command(flatten)]
    pub(super) rig: RigArgs,
    #[command(flatten)]
    pub(super) traffic: Traffic,
    #[command(flatten)]
    pub(super) windows: Windows,
    /// Rate to ask of both ends.
    #[arg(long)]
    pub(super) baud: u32,
    /// What each end transmits.
    #[arg(long, value_enum, default_value_t = Shape::Duplex)]
    pub(super) mode: Shape,
    /// Fraction of the line rate the host leg is paced to in a return-priority run.
    #[arg(long, default_value_t = stream::RETURN_PRIORITY_FRAC)]
    pub(super) host_frac: f64,
    /// Bytes the host keeps granted ahead of its verifier in credit mode.
    #[arg(long, default_value_t = stream::CREDIT_WINDOW)]
    pub(super) credit_window: u64,
}

#[derive(clap::Args, Debug, Clone)]
pub(super) struct Stages {
    /// Stream sweep window per rung, seconds.
    #[arg(long, default_value_t = SWEEP_SECS)]
    pub(super) sweep_secs: f64,
    /// Screen window per rung, seconds.
    #[arg(long, default_value_t = SCREEN_SECS)]
    pub(super) screen_secs: f64,
    /// Sustain window per rung, seconds; `--window`, `--qualify` and `--soak` override it.
    #[arg(long, default_value_t = SUSTAIN_SECS)]
    pub(super) sustain_secs: f64,
    /// Return-priority window at the ceiling, seconds; 0 skips it.
    #[arg(long, default_value_t = SCREEN_SECS)]
    pub(super) rp_secs: f64,
    /// Rungs the screen tries below the stream sweep ceiling before giving up.
    #[arg(long, default_value_t = lattice::DEFAULT_SCREEN_MAX)]
    pub(super) screen_max: usize,
    /// Rungs sustained at and below the screened boundary.
    #[arg(long, default_value_t = lattice::DEFAULT_SUSTAIN_RUNGS)]
    pub(super) sustain_rungs: usize,
}

impl Stages {
    pub(super) fn limits(&self) -> StagedLimits {
        StagedLimits {
            screen_max: self.screen_max,
            sustain_rungs: self.sustain_rungs,
        }
    }
}

#[derive(clap::Args, Debug)]
pub(super) struct LadderArgs {
    /// Require a clean sustain result and any requested return-priority window to pass.
    #[arg(long, conflicts_with = "full")]
    pub(super) strict: bool,
    #[command(flatten)]
    pub(super) rig: RigArgs,
    #[command(flatten)]
    pub(super) traffic: Traffic,
    #[command(flatten)]
    pub(super) windows: Windows,
    #[command(flatten)]
    pub(super) stages: Stages,
    /// Mismatch budget between the two ends' realized rates, percent.
    #[arg(long, default_value_t = lattice::DEFAULT_BUDGET_PCT)]
    pub(super) budget_pct: f64,
    /// Sweep every kept rung at one window length instead of staging.
    #[arg(long)]
    pub(super) full: bool,
}

#[derive(clap::Args, Debug)]
pub(super) struct PlanArgs {
    #[command(flatten)]
    pub(super) traffic: Traffic,
    #[command(flatten)]
    pub(super) windows: Windows,
    #[command(flatten)]
    pub(super) stages: Stages,
    /// Probe selector `VID:PID:serial`, to read the packet geometry and DUT ceiling live.
    ///
    /// The DUT ceiling is read only if the DUT answers; otherwise `--dut-max` supplies it.
    #[arg(long)]
    pub(super) probe: Option<String>,
    /// Wired target name, used only with `--probe`.
    #[arg(long, default_value = "stm32f401")]
    pub(super) target: String,
    /// DUT ELF, for the RTT control-block symbol fast path.
    #[arg(long)]
    pub(super) dut_elf: Option<String>,
    /// The DUT's `BAUD MAX` policy ceiling, which the lattice derives from; read live with
    /// `--probe`.
    #[arg(long)]
    pub(super) dut_max: Option<u32>,
    /// Bulk packet size the probe's USB service budget follows from; read live with `--probe`.
    #[arg(long)]
    pub(super) packet: Option<usize>,
}

/// Assumed packet size where neither a probe nor `--packet` says otherwise.
pub(super) const DEFAULT_PACKET: usize = 64;

impl PlanArgs {
    /// The rig to read from, where one was given; a plan reads the DUT without resetting it.
    pub(super) fn rig(&self) -> Option<RigArgs> {
        self.probe.clone().map(|probe| RigArgs {
            probe,
            target: self.target.clone(),
            dut_elf: self.dut_elf.clone(),
            record: None,
            no_dut_reset: true,
            no_vendor: false,
            firmware_row: None,
        })
    }
}
