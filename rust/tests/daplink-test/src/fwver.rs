//! Run-header staleness check: the probe's version string against the harness's own.

use probe_rs::probe::DebugProbeSelector;

use daplink_core::{DAP_FW_REVISION, DAP_FW_VERSION};

use crate::dap;

/// Read the probe's firmware-version string; call before probe-rs opens the probe, since this
/// claims the same vendor interface.
pub(crate) fn read(selector: &DebugProbeSelector) -> Result<String, String> {
    let mut raw = dap::open(selector)?;
    let bytes = raw.dap_info(0x04)?;
    let version = core::str::from_utf8(&bytes).map_err(|_| "version is not UTF-8")?;
    let version = version.trim_end_matches('\0');
    if version.is_empty() {
        return Err("probe reports no firmware-version string".into());
    }
    Ok(version.to_string())
}

/// The run-header warning when the probe's version differs from the harness's, or `None` when
/// they match or the harness has no revision to judge against.
pub(crate) fn staleness_warning(probe_version: &str) -> Option<String> {
    staleness_warning_against(probe_version, DAP_FW_VERSION, DAP_FW_REVISION)
}

/// [`staleness_warning`] with the harness version injected, so the judgement is testable.
fn staleness_warning_against(probe: &str, harness: &str, harness_rev: &str) -> Option<String> {
    if harness_rev == "unknown" || probe == harness {
        return None;
    }
    Some(format!(
        "WARNING: probe reports firmware version `{probe}`, but this harness was built from \
         `{harness}`.\n\
         WARNING: the probe image predates or differs from the tree; a capability case failing \
         on it may mean a stale image, not a defect -- reflash the probe before treating such a \
         failure as real."
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Warn exactly when the strings differ and the harness knows its own revision.
    #[test]
    fn warns_only_on_a_known_mismatch() {
        let harness = "2.1.1.g0123456789ab";
        assert!(staleness_warning_against(harness, harness, "g0123456789ab").is_none());
        assert!(staleness_warning_against("2.1.1", harness, "g0123456789ab").is_some());
        assert!(
            staleness_warning_against("2.1.1.g0123456789ab-dirty", harness, "g0123456789ab")
                .is_some(),
            "a dirty-built image is not the harness's revision"
        );
        assert!(
            staleness_warning_against("2.1.1", "2.1.1.unknown", "unknown").is_none(),
            "a harness without a revision cannot judge staleness"
        );
    }

    /// The warning names both versions, so the header alone says what is on the probe.
    #[test]
    fn warning_names_both_versions() {
        let text = staleness_warning_against("2.1.1", "2.1.1.gfeedc0ffee12", "gfeedc0ffee12")
            .expect("mismatch must warn");
        assert!(text.contains("`2.1.1`"));
        assert!(text.contains("`2.1.1.gfeedc0ffee12`"));
        assert!(text.starts_with("WARNING:"));
    }
}
