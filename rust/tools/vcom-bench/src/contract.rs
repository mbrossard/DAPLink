//! User-shaped VCOM service checks and their per-case results.

use std::time::Duration;

use crate::stream::{Conservation, ConservationClass, VcomCounters};

pub const DEFAULT_BAUD: u32 = 115_200;
pub const EXCHANGES: usize = 1_000;
pub const DEFAULT_CYCLES: usize = 10;
pub const QUALIFY_CYCLES: usize = 100;
pub const IDLE: Duration = Duration::from_secs(10);
pub const QUALIFY_IDLE: Duration = Duration::from_secs(5 * 60);
pub const LINE_SETTLE_TIMEOUT: Duration = Duration::from_millis(500);
pub const LINE_SETTLE_POLL: Duration = Duration::from_millis(5);
pub const LINE_CONTROL_QUIET: Duration = Duration::from_millis(150);
pub const PAUSES: [Duration; 3] = [
    Duration::from_millis(50),
    Duration::from_millis(250),
    Duration::from_millis(1_000),
];
pub const LINE_CODING: [u32; 4] = [9_600, 115_200, 921_600, 115_200];

pub fn cycles(qualify: bool) -> usize {
    if qualify {
        QUALIFY_CYCLES
    } else {
        DEFAULT_CYCLES
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Latencies {
    samples_us: Vec<u64>,
}

impl Latencies {
    pub fn push(&mut self, elapsed: Duration) {
        self.samples_us
            .push(elapsed.as_micros().min(u128::from(u64::MAX)) as u64);
    }
    pub fn count(&self) -> usize {
        self.samples_us.len()
    }
    pub fn percentile_us(&self, percentile: u8) -> Option<u64> {
        if self.samples_us.is_empty() {
            return None;
        }
        let mut sorted = self.samples_us.clone();
        sorted.sort_unstable();
        let rank = (sorted.len() * usize::from(percentile.clamp(1, 100))).div_ceil(100) - 1;
        sorted.get(rank).copied()
    }
    pub fn max_us(&self) -> Option<u64> {
        self.samples_us.iter().copied().max()
    }
}

#[derive(Clone, Debug)]
pub struct CaseResult {
    pub case: &'static str,
    pub tier: Option<String>,
    pub baud: u32,
    pub exact: bool,
    pub counter_classification: Option<ConservationClass>,
    pub dap_progress: bool,
    pub recovery: Option<Duration>,
    pub latencies: Latencies,
    pub pass: bool,
    pub failure: Option<String>,
}

pub fn counter_classification(
    before: Option<VcomCounters>,
    after: Option<VcomCounters>,
) -> Option<ConservationClass> {
    before
        .zip(after)
        .map(|(before, after)| Conservation { before, after }.classification())
}

pub fn apply_counter_classification(result: &mut CaseResult) {
    let failure = match result.counter_classification {
        Some(ConservationClass::ObservedLoss) => "probe counters observed VCOM loss",
        Some(ConservationClass::IdentityFailed) => "probe counter identity failed",
        // The disposition `stuck` has: a channel reinitialised mid-case voids its evidence.
        Some(ConservationClass::DmaFaulted) => "probe recovered a DMA fault; case is not evidence",
        _ => return,
    };
    result.pass = false;
    if result.failure.is_none() {
        result.failure = Some(failure.into());
    }
}

pub fn suite_pass(failed_cases: usize) -> Result<(), String> {
    if failed_cases == 0 {
        Ok(())
    } else {
        Err(format!("{failed_cases} contract case(s) failed"))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineStage {
    Scrub,
    Target,
}

impl LineStage {
    pub fn key(self) -> &'static str {
        match self {
            LineStage::Scrub => "scrub",
            LineStage::Target => "target",
        }
    }
}

pub fn line_transition_failure(
    cycle: usize,
    requested: u32,
    stage: LineStage,
    expected: u32,
    probe_realized: Option<u32>,
    in_flight: bool,
) -> Option<String> {
    let stage = stage.key();
    // The probe's own word beats the number: mid-transaction it still reads the rate being left.
    if in_flight {
        return Some(format!(
            "line-coding cycle {cycle} rate {requested} {stage} stage: the probe has not applied \
             the rate yet"
        ));
    }
    let realized = match probe_realized {
        Some(realized) => realized,
        None => {
            return Some(format!(
                "line-coding cycle {cycle} rate {requested} {stage} stage: probe realized baud \
                 unavailable"
            ));
        }
    };
    let mismatch = realized.abs_diff(expected) as f64 * 100.0 / f64::from(expected);
    (mismatch > crate::lattice::DEFAULT_BUDGET_PCT).then(|| {
        format!(
            "line-coding cycle {cycle} rate {requested} {stage} stage: probe realized {realized}, \
             expected {expected}"
        )
    })
}

pub fn line_runtime_failure(
    cycle: usize,
    requested: u32,
    stage: &str,
    detail: impl std::fmt::Display,
) -> String {
    format!("line-coding cycle {cycle} rate {requested} {stage} stage: {detail}")
}

pub fn line_echo_failure(
    cycle: usize,
    requested: u32,
    written: u64,
    read: u64,
    first_divergence: Option<u64>,
) -> String {
    format!(
        "line-coding cycle {cycle} rate {requested} bulk echo stage: wrote {written}, read {read}, \
         first divergence {}",
        first_divergence.map_or_else(|| "none".into(), |offset| offset.to_string())
    )
}

pub fn small_exchange_pass(no_dap: &Latencies, with_dap: &Latencies) -> Result<(), String> {
    let unloaded = no_dap
        .percentile_us(99)
        .ok_or("no unloaded latency samples")?;
    let loaded = with_dap
        .percentile_us(99)
        .ok_or("no loaded latency samples")?;
    let allowed = 10_000u64.max(unloaded.saturating_mul(2));
    if loaded > allowed {
        return Err(format!("DAP-loaded p99 {loaded} us exceeds {allowed} us"));
    }
    let max = with_dap
        .max_us()
        .unwrap_or(0)
        .max(no_dap.max_us().unwrap_or(0));
    if max >= 50_000 {
        return Err(format!(
            "maximum exchange latency {max} us is not below 50000 us"
        ));
    }
    Ok(())
}

pub fn pause_pass(
    pause: Duration,
    exact: bool,
    resumed: bool,
    attributed: bool,
) -> Result<(), String> {
    if !resumed {
        return Err("traffic did not resume and terminate cleanly".into());
    }
    if pause == PAUSES[0] && !exact {
        return Err("50 ms reader pause was not byte-exact".into());
    }
    if !exact && !attributed {
        return Err("loss was neither named nor explicitly partial".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn latency(values: &[u64]) -> Latencies {
        let mut out = Latencies::default();
        for value in values {
            out.push(Duration::from_micros(*value));
        }
        out
    }
    #[test]
    fn nearest_rank_percentiles_are_stable() {
        let values: Vec<_> = (1..=100).collect();
        let samples = latency(&values);
        assert_eq!(samples.percentile_us(50), Some(50));
        assert_eq!(samples.percentile_us(99), Some(99));
    }
    #[test]
    fn small_exchange_enforces_tail_and_maximum() {
        assert!(small_exchange_pass(&latency(&[4_000]), &latency(&[9_000])).is_ok());
        assert!(small_exchange_pass(&latency(&[4_000]), &latency(&[11_000])).is_err());
        assert!(small_exchange_pass(&latency(&[30_000]), &latency(&[50_000])).is_err());
    }
    #[test]
    fn pause_policy_is_tiered() {
        assert!(pause_pass(PAUSES[0], false, true, true).is_err());
        assert!(pause_pass(PAUSES[1], false, true, true).is_ok());
        assert!(pause_pass(PAUSES[2], false, true, false).is_err());
        assert!(pause_pass(PAUSES[2], true, false, true).is_err());
    }

    #[test]
    fn counter_classification_requires_both_samples() {
        let sample = VcomCounters {
            valid: Some(crate::stream::VALID_RX_LEDGER),
            ..VcomCounters::default()
        };
        // Full coverage without a grade word can no longer claim completeness.
        assert_eq!(
            counter_classification(Some(sample), Some(sample)),
            Some(ConservationClass::PartialClean)
        );
        assert_eq!(counter_classification(Some(sample), None), None);
        assert_eq!(counter_classification(None, Some(sample)), None);
    }

    #[test]
    fn counter_classification_uses_the_case_delta() {
        let before = VcomCounters {
            uart_rx: u32::MAX - 2,
            cdc_in: u32::MAX - 2,
            valid: Some(crate::stream::VALID_RX_LEDGER),
            ..VcomCounters::default()
        };
        let after = VcomCounters {
            uart_rx: 2,
            cdc_in: 1,
            host_dropped: 1,
            valid: Some(crate::stream::VALID_RX_LEDGER),
            ..VcomCounters::default()
        };
        assert_eq!(
            counter_classification(Some(before), Some(after)),
            Some(ConservationClass::ObservedLoss)
        );
    }

    #[test]
    fn observed_loss_and_identity_failure_fail_a_case() {
        for classification in [
            ConservationClass::ObservedLoss,
            ConservationClass::IdentityFailed,
            ConservationClass::DmaFaulted,
        ] {
            let mut result = CaseResult {
                case: "test",
                tier: None,
                baud: DEFAULT_BAUD,
                exact: true,
                counter_classification: Some(classification),
                dap_progress: true,
                recovery: None,
                latencies: Latencies::default(),
                pass: true,
                failure: None,
            };
            apply_counter_classification(&mut result);
            assert!(!result.pass);
            assert!(result.failure.is_some());
        }
    }

    #[test]
    fn clean_and_unknown_coverage_do_not_fail_an_exact_case() {
        for classification in [
            None,
            Some(ConservationClass::CompleteClean),
            Some(ConservationClass::PartialClean),
        ] {
            let mut result = CaseResult {
                case: "test",
                tier: None,
                baud: DEFAULT_BAUD,
                exact: true,
                counter_classification: classification,
                dap_progress: true,
                recovery: None,
                latencies: Latencies::default(),
                pass: true,
                failure: None,
            };
            apply_counter_classification(&mut result);
            assert!(result.pass);
            assert!(result.failure.is_none());
        }
    }

    #[test]
    fn a_failed_case_fails_the_contract_command() {
        assert!(suite_pass(0).is_ok());
        assert_eq!(suite_pass(2), Err("2 contract case(s) failed".into()));
    }

    #[test]
    fn line_transition_settle_names_unavailable_and_wrong_rates() {
        assert_eq!(
            line_transition_failure(3, 9_600, LineStage::Target, 9_615, Some(9_600), false),
            None
        );
        assert_eq!(
            line_transition_failure(4, 921_600, LineStage::Scrub, 9_600, None, false),
            Some(
                "line-coding cycle 4 rate 921600 scrub stage: probe realized baud unavailable"
                    .into()
            )
        );
        assert_eq!(
            line_transition_failure(5, 115_200, LineStage::Target, 115_200, Some(9_600), false),
            Some(
                "line-coding cycle 5 rate 115200 target stage: probe realized 9600, expected 115200"
                    .into()
            )
        );
        // A rate matching the expectation is still no transition while the probe holds it pending.
        assert_eq!(
            line_transition_failure(6, 9_600, LineStage::Target, 9_615, Some(9_615), true),
            Some(
                "line-coding cycle 6 rate 9600 target stage: the probe has not applied the rate yet"
                    .into()
            )
        );
    }

    #[test]
    fn line_runtime_failure_names_its_cycle_rate_and_stage() {
        assert_eq!(
            line_runtime_failure(14, 921_600, "target control", "transfer cancelled"),
            "line-coding cycle 14 rate 921600 target control stage: transfer cancelled"
        );
    }

    #[test]
    fn line_echo_failure_preserves_the_host_ledger() {
        assert_eq!(
            line_echo_failure(7, 921_600, 64, 61, Some(17)),
            "line-coding cycle 7 rate 921600 bulk echo stage: wrote 64, read 61, first divergence 17"
        );
    }

    #[test]
    fn qualification_selects_the_full_cycle_count() {
        assert_eq!(cycles(false), 10);
        assert_eq!(cycles(true), 100);
    }
}
