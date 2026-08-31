//! Rig capability profiles: which wiring a named rig provides, for `--profile` gating.

use std::fmt;

/// A wiring a case may require beyond a bare probe-target SWD link.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Req {
    Jtag,
    Swo,
    Vcom,
    Nreset,
}

impl fmt::Display for Req {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Req::Jtag => "jtag",
            Req::Swo => "swo",
            Req::Vcom => "vcom",
            Req::Nreset => "nreset",
        })
    }
}

impl std::str::FromStr for Req {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "jtag" => Ok(Req::Jtag),
            "swo" => Ok(Req::Swo),
            "vcom" => Ok(Req::Vcom),
            "nreset" => Ok(Req::Nreset),
            other => Err(format!("unknown requirement `{other}`")),
        }
    }
}

/// Which side withheld a requirement: the probe's rig profile, the target's own wiring, or an
/// operator override.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Side {
    Profile,
    Target,
    Disabled,
}

/// A named rig wiring and the requirements it satisfies.
pub(crate) struct Profile {
    pub(crate) name: &'static str,
    pub(crate) supports: &'static [Req],
}

/// Known rig profiles; names the rig shape, not a bench serial.
pub(crate) const PROFILES: &[Profile] = &[
    Profile {
        name: "rp2040-pico",
        supports: &[Req::Jtag, Req::Swo, Req::Vcom, Req::Nreset],
    },
    // Pico 2 probe on a Pico DUT: full wiring -- SWD, JTAG, SWO, nRESET and VCOM.
    Profile {
        name: "rp2350-pico2",
        supports: &[Req::Jtag, Req::Swo, Req::Vcom, Req::Nreset],
    },
    // H523 carrier probe on an STM32F4 DUT: full wiring -- SWD, JTAG, SWO, nRESET and VCOM.
    Profile {
        name: "stm32h523",
        supports: &[Req::Jtag, Req::Swo, Req::Vcom, Req::Nreset],
    },
    // MCU-Link probe on an STM32F4 DUT: full wiring -- SWD, JTAG, SWO, nRESET and VCOM.
    Profile {
        name: "mculink",
        supports: &[Req::Jtag, Req::Swo, Req::Vcom, Req::Nreset],
    },
    // nanoDAP probe on an STM32F4 DUT: no SWO route, USART2 being this board's VCOM.
    Profile {
        name: "stm32f103-nanodap",
        supports: &[Req::Jtag, Req::Vcom, Req::Nreset],
    },
    // Debug Probe PUT: no debug header, so no JTAG/SWO/nRESET route -- SWD and VCOM only.
    Profile {
        name: "rp2040-debugprobe",
        supports: &[Req::Vcom],
    },
];

/// Look up a profile by name.
pub(crate) fn profile(name: &str) -> Option<&'static Profile> {
    PROFILES.iter().find(|p| p.name == name)
}

/// The first requirement unmet by (profile and target caps) minus disabled, and which side
/// withheld it; an absent profile gates nothing.
pub(crate) fn missing_requirement(
    profile: Option<&Profile>,
    target_caps: Option<&[Req]>,
    disabled: &[Req],
    requires: &[Req],
) -> Option<(Req, Side)> {
    let profile = profile?;
    requires.iter().copied().find_map(|r| {
        if !profile.supports.contains(&r) {
            Some((r, Side::Profile))
        } else if target_caps.is_some_and(|caps| !caps.contains(&r)) {
            Some((r, Side::Target))
        } else if disabled.contains(&r) {
            Some((r, Side::Disabled))
        } else {
            None
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unprofiled_run_gates_nothing() {
        assert_eq!(missing_requirement(None, None, &[], &[Req::Jtag]), None);
    }

    #[test]
    fn satisfied_requirement_does_not_skip() {
        let p = profile("rp2040-pico").expect("rp2040-pico registered");
        assert_eq!(
            missing_requirement(Some(p), None, &[], &[Req::Nreset]),
            None
        );
    }

    #[test]
    fn unsupported_requirement_skips_with_reason() {
        let p = Profile {
            name: "bare",
            supports: &[],
        };
        assert_eq!(
            missing_requirement(Some(&p), None, &[], &[Req::Vcom]),
            Some((Req::Vcom, Side::Profile))
        );
    }

    #[test]
    fn first_missing_requirement_is_reported() {
        let p = Profile {
            name: "bare",
            supports: &[Req::Jtag],
        };
        assert_eq!(
            missing_requirement(Some(&p), None, &[], &[Req::Jtag, Req::Swo]),
            Some((Req::Swo, Side::Profile))
        );
    }

    #[test]
    fn no_requirements_never_skips() {
        let p = Profile {
            name: "bare",
            supports: &[],
        };
        assert_eq!(missing_requirement(Some(&p), None, &[], &[]), None);
    }

    #[test]
    fn target_caps_intersect_with_profile() {
        let p = profile("rp2040-pico").expect("rp2040-pico registered");
        assert_eq!(
            missing_requirement(Some(p), Some(&[Req::Vcom, Req::Nreset]), &[], &[Req::Jtag]),
            Some((Req::Jtag, Side::Target))
        );
    }

    #[test]
    fn profile_side_withholding_wins_over_target() {
        let p = Profile {
            name: "bare",
            supports: &[],
        };
        assert_eq!(
            missing_requirement(Some(&p), Some(&[]), &[], &[Req::Jtag]),
            Some((Req::Jtag, Side::Profile))
        );
    }

    #[test]
    fn absent_target_caps_imposes_no_restriction() {
        let p = profile("rp2040-pico").expect("rp2040-pico registered");
        assert_eq!(missing_requirement(Some(p), None, &[], &[Req::Jtag]), None);
    }

    #[test]
    fn disabled_requirement_skips_with_reason() {
        let p = profile("rp2040-pico").expect("rp2040-pico registered");
        assert_eq!(
            missing_requirement(Some(p), None, &[Req::Swo], &[Req::Swo]),
            Some((Req::Swo, Side::Disabled))
        );
    }

    #[test]
    fn profile_and_target_withholding_win_over_disabled() {
        let p = Profile {
            name: "bare",
            supports: &[],
        };
        assert_eq!(
            missing_requirement(Some(&p), Some(&[]), &[Req::Jtag], &[Req::Jtag]),
            Some((Req::Jtag, Side::Profile))
        );
        let p = profile("rp2040-pico").expect("rp2040-pico registered");
        assert_eq!(
            missing_requirement(Some(p), Some(&[]), &[Req::Jtag], &[Req::Jtag]),
            Some((Req::Jtag, Side::Target))
        );
    }

    #[test]
    fn debugprobe_profile_supports_only_vcom() {
        let p = profile("rp2040-debugprobe").expect("rp2040-debugprobe registered");
        assert_eq!(missing_requirement(Some(p), None, &[], &[Req::Vcom]), None);
        for req in [Req::Jtag, Req::Swo, Req::Nreset] {
            assert_eq!(
                missing_requirement(Some(p), None, &[], &[req]),
                Some((req, Side::Profile))
            );
        }
    }
}
