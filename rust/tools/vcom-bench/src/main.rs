//! `vcom-bench` -- VCOM measurement and operating-ceiling search.

mod budget;
mod cli;
mod command;
mod contract;
mod counters;
mod dut;
mod lattice;
mod lifecycle;
mod probe;
mod record;
mod stream;
mod tty;

use clap::Parser;

fn main() -> anyhow::Result<()> {
    command::run(cli::Cli::parse().command)
}
