//! Command-line argument parsing for `flash-probe-rs`.

use anyhow::{anyhow, bail, Result};
use daplink_flash::EraseMode;

/// The supported flash targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetKind {
    /// STMicroelectronics STM32F401CE (512 KiB flash based at `0x0800_0000`).
    Stm32f401,
    /// Raspberry Pi RP2040 (external QSPI flash based at `0x1000_0000`).
    Rp2040,
}

impl TargetKind {
    /// The flash base address programmed and verified by this tool for the target.
    pub fn flash_base(self) -> u32 {
        match self {
            TargetKind::Stm32f401 => 0x0800_0000,
            TargetKind::Rp2040 => 0x1000_0000,
        }
    }

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

/// Parse the `--erase-mode` value.
fn parse_erase_mode(s: &str) -> Result<EraseMode> {
    match s {
        "chip" => Ok(EraseMode::Chip),
        "sector" => Ok(EraseMode::Sector),
        other => bail!("unknown --erase-mode '{other}' (expected chip or sector)"),
    }
}

/// One-line usage summary shown on argument errors.
pub const USAGE: &str = "usage: flash-probe-rs --target <stm32f401|rp2040> \
[--image <file>] [--probe <selector>] [--speed <khz>] [--erase-mode <chip|sector>]";

/// Parsed command-line arguments.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Args {
    /// Which target to flash.
    pub target: TargetKind,
    /// Image to program; absent runs an erase/program/verify self-test at the flash base.
    pub image: Option<String>,
    /// Probe selector (`VID:PID[:serial]`); the first probe is used when absent.
    pub probe: Option<String>,
    /// SWD clock in kHz.
    pub speed_khz: u32,
    /// `chip` erases the whole chip up front (falling back to sector erase where unsupported),
    /// `sector` erases lazily before each sector's first program. The self-test ignores it.
    pub erase_mode: EraseMode,
}

impl Args {
    /// Parse from the process arguments (skipping the program name).
    pub fn from_env() -> Result<Self> {
        Self::parse(std::env::args().skip(1))
    }

    /// Parse from an iterator of argument strings.
    pub fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Self> {
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
                "-h" | "--help" => bail!("{USAGE}"),
                other => bail!("unexpected argument '{other}'\n{USAGE}"),
            }
        }

        let target = target.ok_or_else(|| anyhow!("--target is required\n{USAGE}"))?;
        Ok(Self {
            target,
            image,
            probe,
            speed_khz,
            erase_mode,
        })
    }
}

/// Consume the value that must follow a flag, or error naming the flag.
fn take_value<I: Iterator<Item = String>>(it: &mut I, flag: &str) -> Result<String> {
    it.next().ok_or_else(|| anyhow!("{flag} expects a value"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args> {
        Args::parse(args.iter().map(|s| s.to_string()))
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
    fn flash_base_matches_target() {
        assert_eq!(TargetKind::Stm32f401.flash_base(), 0x0800_0000);
        assert_eq!(TargetKind::Rp2040.flash_base(), 0x1000_0000);
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
