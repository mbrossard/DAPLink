//! Command-line argument parsing for `flash-probe-rs`.

use anyhow::{anyhow, bail, Result};
use daplink_flash::EraseMode;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetKind {
    /// STMicroelectronics STM32F401CE.
    Stm32f401,
    /// Raspberry Pi RP2040.
    Rp2040,
}

impl TargetKind {
    fn parse(s: &str) -> Result<Self> {
        match s {
            "stm32f401" => Ok(Self::Stm32f401),
            "rp2040" => Ok(Self::Rp2040),
            other => bail!("unknown --target '{other}' (expected stm32f401 or rp2040)"),
        }
    }
}

/// Default SWD clock in kHz when `--speed` is not given.
const DEFAULT_SPEED_KHZ: u32 = 1000;

fn parse_erase_mode(s: &str) -> Result<EraseMode> {
    match s {
        "chip" => Ok(EraseMode::Chip),
        "sector" => Ok(EraseMode::Sector),
        other => bail!("unknown --erase-mode '{other}' (expected chip or sector)"),
    }
}

/// Usage summary shown on argument errors.
pub const USAGE: &str = "usage: flash-probe-rs --target <stm32f401|rp2040> \
[--image <file>] [--probe <selector>] [--speed <khz>] [--erase-mode <chip|sector>]
Erases and programs target flash; without --image, a one-sector self-test at the flash base.
The target is left halted, not reset.";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Args {
    pub target: TargetKind,
    /// Image to program; absent runs an erase/program/verify self-test at the flash base.
    pub image: Option<String>,
    /// Probe selector (`VID:PID[:serial]`); omission requires exactly one available probe.
    pub probe: Option<String>,
    /// SWD clock in kHz.
    pub speed_khz: u32,
    /// `chip` erases up front (sector erase where unsupported), `sector` before each first program.
    pub erase_mode: EraseMode,
}

impl Args {
    /// Parse from the process arguments (skipping the program name).
    pub fn from_env() -> Result<Option<Self>> {
        Self::parse(std::env::args().skip(1))
    }

    pub fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Option<Self>> {
        let mut target = None;
        let mut image = None;
        let mut probe = None;
        let mut speed_khz = DEFAULT_SPEED_KHZ;
        let mut erase_mode = EraseMode::Chip;

        let mut it = args.into_iter();
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--target" => target = Some(TargetKind::parse(&take_value(&mut it, "--target")?)?),
                "--image" => image = Some(take_value(&mut it, "--image")?),
                "--probe" => probe = Some(take_value(&mut it, "--probe")?),
                "--speed" => {
                    let raw = take_value(&mut it, "--speed")?;
                    speed_khz = raw.parse().map_err(|_| {
                        anyhow!("--speed expects an integer kHz value, got '{raw}'")
                    })?;
                }
                "--erase-mode" => {
                    erase_mode = parse_erase_mode(&take_value(&mut it, "--erase-mode")?)?;
                }
                "-h" | "--help" => return Ok(None),
                other => bail!("unexpected argument '{other}'\n{USAGE}"),
            }
        }

        let target = target.ok_or_else(|| anyhow!("--target is required\n{USAGE}"))?;
        Ok(Some(Self {
            target,
            image,
            probe,
            speed_khz,
            erase_mode,
        }))
    }
}

fn take_value<I: Iterator<Item = String>>(it: &mut I, flag: &str) -> Result<String> {
    it.next().ok_or_else(|| anyhow!("{flag} expects a value"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args> {
        Args::parse(args.iter().map(|s| s.to_string()))?.ok_or_else(|| anyhow!("help"))
    }

    #[test]
    fn help_needs_no_target() {
        for flag in ["-h", "--help"] {
            assert_eq!(Args::parse([flag.to_string()]).unwrap(), None);
        }
    }

    #[test]
    fn minimal_target_uses_defaults() {
        let a = parse(&["--target", "rp2040"]).unwrap();
        assert_eq!(a.target, TargetKind::Rp2040);
        assert_eq!(a.image, None);
        assert_eq!(a.probe, None);
        assert_eq!(a.speed_khz, DEFAULT_SPEED_KHZ);
        assert_eq!(a.erase_mode, EraseMode::Chip);
    }

    #[test]
    fn all_flags_parse() {
        let a = parse(&[
            "--target",
            "rp2040",
            "--image",
            "fw.bin",
            "--probe",
            "0d28:0204",
            "--speed",
            "4000",
            "--erase-mode",
            "sector",
        ])
        .unwrap();
        assert_eq!(a.target, TargetKind::Rp2040);
        assert_eq!(a.image.as_deref(), Some("fw.bin"));
        assert_eq!(a.probe.as_deref(), Some("0d28:0204"));
        assert_eq!(a.speed_khz, 4000);
        assert_eq!(a.erase_mode, EraseMode::Sector);
    }

    #[test]
    fn erase_mode_chip_parses() {
        let a = parse(&["--target", "rp2040", "--erase-mode", "chip"]).unwrap();
        assert_eq!(a.erase_mode, EraseMode::Chip);
    }

    #[test]
    fn unknown_erase_mode_is_rejected() {
        assert!(parse(&["--target", "rp2040", "--erase-mode", "bogus"]).is_err());
    }

    #[test]
    fn stm32f401_target_parses() {
        let a = parse(&["--target", "stm32f401"]).unwrap();
        assert_eq!(a.target, TargetKind::Stm32f401);
    }

    #[test]
    fn missing_target_is_rejected() {
        assert!(parse(&["--speed", "1000"]).is_err());
    }

    #[test]
    fn unknown_target_is_rejected() {
        assert!(parse(&["--target", "nrf52"]).is_err());
    }

    #[test]
    fn missing_value_is_rejected() {
        assert!(parse(&["--target"]).is_err());
    }

    #[test]
    fn non_numeric_speed_is_rejected() {
        assert!(parse(&["--target", "rp2040", "--speed", "fast"]).is_err());
    }

    #[test]
    fn unknown_flag_is_rejected() {
        assert!(parse(&["--target", "rp2040", "--bogus"]).is_err());
    }
}
