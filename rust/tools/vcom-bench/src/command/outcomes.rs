//! Exit policy and operational error preservation.

use anyhow::{ensure, Context, Result};

pub(super) fn combine(primary: Result<()>, secondary: Result<()>) -> Result<()> {
    after(primary, secondary)
}

pub(super) fn after<T>(primary: Result<T>, secondary: Result<()>) -> Result<T> {
    match (primary, secondary) {
        (Err(primary), Err(secondary)) => Err(primary.context(SecondaryFailure(secondary))),
        (primary, Ok(())) => primary,
        (Ok(_), Err(secondary)) => Err(secondary),
    }
}

#[derive(Debug)]
struct SecondaryFailure(anyhow::Error);

impl std::fmt::Display for SecondaryFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "also failed: {:#}", self.0)
    }
}

#[derive(Debug)]
pub(super) struct RateDeclined(pub(super) String);

impl std::fmt::Display for RateDeclined {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for RateDeclined {}

pub(super) fn rate_declined(error: &anyhow::Error) -> bool {
    error.downcast_ref::<RateDeclined>().is_some()
        && error.downcast_ref::<SecondaryFailure>().is_none()
}

pub(super) fn run(strict: bool, passed: u32, windows: u32) -> Result<()> {
    ensure!(
        !strict || (windows > 0 && passed == windows),
        "measurement verdict: {passed} of {windows} windows passed"
    );
    Ok(())
}

pub(super) fn ladder(strict: bool, certified: bool, rp: Option<bool>) -> Result<()> {
    if strict {
        ensure!(
            certified,
            "measurement verdict: no clean certified sustain result"
        );
        ensure!(
            rp != Some(false),
            "measurement verdict: requested return-priority window did not pass"
        );
    }
    Ok(())
}

#[derive(Debug)]
pub(super) struct RecordingError(pub(super) std::io::Error);

impl std::fmt::Display for RecordingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "record append failed: {}", self.0)
    }
}

impl std::error::Error for RecordingError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.0)
    }
}

pub(super) fn recorded(result: std::io::Result<()>, path: &std::path::Path) -> Result<()> {
    result
        .map_err(RecordingError)
        .with_context(|| format!("record {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strict_run_rejects_failed_and_mixed_windows() {
        for strict in [false, true] {
            for (passed, count) in [(3, 3), (0, 3), (2, 3), (0, 0)] {
                assert_eq!(
                    run(strict, passed, count).is_ok(),
                    !strict || (count > 0 && passed == count)
                );
            }
        }
    }

    #[test]
    fn ladder_requires_clean_certification_and_requested_return_priority_only() {
        for strict in [false, true] {
            for certified in [false, true] {
                for rp in [None, Some(false), Some(true)] {
                    assert_eq!(
                        ladder(strict, certified, rp).is_ok(),
                        !strict || (certified && rp != Some(false))
                    );
                }
            }
        }
    }

    #[test]
    fn secondary_failure_keeps_primary_cause() {
        let error = combine(
            Err(anyhow::anyhow!("measurement transport")),
            Err(anyhow::anyhow!("cleanup")),
        )
        .unwrap_err();
        assert!(format!("{error:#}").contains("cleanup"));
        assert_eq!(error.root_cause().to_string(), "measurement transport");
    }

    #[test]
    fn cleanup_and_final_counter_failures_cannot_overwrite_or_hide_primary() {
        for primary_failed in [false, true] {
            for cleanup_failed in [false, true] {
                for counter_failed in [false, true] {
                    let result = |failed: bool, cause: &'static str| {
                        if failed {
                            Err(anyhow::anyhow!(cause))
                        } else {
                            Ok(())
                        }
                    };
                    let outcome = combine(
                        combine(
                            result(primary_failed, "primary"),
                            result(counter_failed, "counter read"),
                        ),
                        result(cleanup_failed, "cleanup"),
                    );
                    assert_eq!(
                        outcome.is_err(),
                        primary_failed || counter_failed || cleanup_failed
                    );
                    if let Err(error) = outcome {
                        let display = format!("{error:#}");
                        for (failed, cause) in [
                            (primary_failed, "primary"),
                            (counter_failed, "counter read"),
                            (cleanup_failed, "cleanup"),
                        ] {
                            if failed {
                                assert!(display.contains(cause));
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn injected_append_failure_is_operational_in_both_modes() {
        for strict in [false, true] {
            let append = recorded(
                Err(std::io::Error::other("injected")),
                std::path::Path::new("record.jsonl"),
            );
            let error = combine(append, run(strict, 1, 1)).unwrap_err();
            assert!(error.downcast_ref::<RecordingError>().is_some());
        }
    }

    #[test]
    fn rate_refusal_is_exploratory_but_transport_record_and_cleanup_errors_are_not() {
        let refusal =
            || -> Result<()> { Err(anyhow::Error::new(RateDeclined("rate refused".into()))) };
        assert!(rate_declined(&refusal().unwrap_err()));
        assert!(!rate_declined(&anyhow::anyhow!("transport")));
        assert!(!rate_declined(
            &combine(refusal(), Err(anyhow::anyhow!("cleanup"))).unwrap_err()
        ));
        assert!(!rate_declined(
            &recorded(
                Err(std::io::Error::other("disk")),
                std::path::Path::new("records")
            )
            .unwrap_err()
        ));
    }
}
