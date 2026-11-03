//! Staged ladder summary serialization.

use super::*;
use crate::cli::LadderArgs;
use crate::command::{Message, Report};
use crate::record;

#[derive(Serialize)]
struct RowWire {
    stage: &'static str,
    nominal: u32,
    pass: bool,
    #[serde(serialize_with = "ser_f2")]
    held_s: f64,
}

#[derive(Serialize)]
struct MessageWire<'a> {
    nominal: u32,
    pass: bool,
    ferr: Option<u32>,
    ovr: Option<u32>,
    stuck: Option<u32>,
    exact: Option<bool>,
    error: Option<&'a str>,
}

#[derive(Serialize)]
struct SummaryWire<'a> {
    schema_version: u32,
    tool: &'static str,
    unix_s: u64,
    probe: &'a str,
    firmware_row: Option<&'a str>,
    firmware_row_source: &'a str,
    firmware_version: Option<&'a str>,
    verb: &'static str,
    phase: &'static str,
    shape: &'static str,
    mode: &'static str,
    dut_baud_max: u32,
    #[serde(serialize_with = "ser_bare")]
    budget_pct: f64,
    interval: u32,
    dap: &'static str,
    #[serde(serialize_with = "ser_bare")]
    sweep_s: f64,
    #[serde(serialize_with = "ser_bare")]
    screen_s: f64,
    #[serde(serialize_with = "ser_bare")]
    sustain_s: f64,
    #[serde(serialize_with = "ser_bare")]
    checkpoint_s: f64,
    message_ceiling: Option<u32>,
    message_gap: bool,
    sweep_ceiling: Option<u32>,
    screened_ceiling: Option<u32>,
    sustained_ceiling: Option<u32>,
    sustained_qualified: Option<u32>,
    non_monotone: bool,
    sweep_pessimistic: bool,
    screen_exhausted: bool,
    windows: Vec<RowWire>,
    messages: Vec<MessageWire<'a>>,
}

pub(crate) fn staged_summary(
    report: &Report,
    args: &LadderArgs,
    sustain_secs: f64,
    unix_s: u64,
) -> String {
    let (kept, outcome) = (report.kept, report.outcome);
    let rate = |index: Option<usize>| index.map(|i| kept[i].nominal);

    let windows = report
        .rows
        .iter()
        .map(|row| RowWire {
            stage: row.stage.key(),
            nominal: row.rung.nominal,
            pass: row.pass(),
            held_s: row.outcome.as_ref().map_or(0.0, Run::sustained_secs),
        })
        .collect();

    let messages = report
        .messages
        .iter()
        .map(|row| {
            let stats = row.outcome.as_ref().ok().and_then(|m| m.stats);
            MessageWire {
                nominal: row.rung.nominal,
                pass: row.outcome.as_ref().is_ok_and(Message::pass),
                ferr: stats.map(|s| s.ferr),
                ovr: stats.map(|s| s.ovr),
                stuck: stats.map(|s| s.stuck),
                exact: row.outcome.as_ref().ok().map(Message::exact),
                error: row.outcome.as_ref().err().map(String::as_str),
            }
        })
        .collect();

    let wire = SummaryWire {
        schema_version: record::SCHEMA_VERSION,
        tool: "vcom-bench",
        unix_s,
        probe: report.probe,
        firmware_row: report.firmware.row,
        firmware_row_source: report.firmware.source,
        firmware_version: report.firmware.version,
        verb: "ladder",
        phase: "summary",
        shape: "v1",
        mode: "staged",
        dut_baud_max: report.dut_max,
        budget_pct: args.budget_pct,
        interval: args.traffic.interval,
        dap: args.traffic.dap.key(),
        sweep_s: args.stages.sweep_secs,
        screen_s: args.stages.screen_secs,
        sustain_s: sustain_secs,
        checkpoint_s: args.traffic.checkpoint,
        message_ceiling: rate(outcome.message),
        message_gap: outcome.message_gap,
        sweep_ceiling: rate(outcome.swept),
        screened_ceiling: rate(outcome.screened),
        sustained_ceiling: rate(outcome.sustained),
        sustained_qualified: rate(outcome.sustained_qualified),
        non_monotone: outcome.non_monotone,
        sweep_pessimistic: outcome.sweep_pessimistic,
        screen_exhausted: outcome.screen_exhausted,
        windows,
        messages,
    };
    to_json(&wire)
}
