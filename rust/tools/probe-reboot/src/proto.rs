//! The reboot-to-bootloader vendor-command wire protocol, host side.

use anyhow::{bail, Result};

/// Vendor command id (`ID_DAP_Vendor31`).
pub const REBOOT_CMD_ID: u8 = 0x9F;
/// Discovery/echo subcommand.
pub const SUB_DISCOVER: u8 = 0x00;
/// Reboot subcommand.
pub const SUB_REBOOT: u8 = 0x01;
/// Guard magic on the reboot subcommand.
pub const REBOOT_MAGIC: [u8; 4] = *b"BOOT";
/// Identity magic in a discovery reply: marks a daplink-rs probe.
pub const REPLY_MAGIC: [u8; 4] = *b"dlrs";

/// The discovery command bytes to send: `[0x9F, 0x00]`.
pub fn discovery_cmd() -> [u8; 2] {
    [REBOOT_CMD_ID, SUB_DISCOVER]
}

/// The reboot command bytes to send: `[0x9F, 0x01, b"BOOT"]`.
pub fn reboot_cmd() -> [u8; 6] {
    let mut c = [0u8; 6];
    c[0] = REBOOT_CMD_ID;
    c[1] = SUB_REBOOT;
    c[2..6].copy_from_slice(&REBOOT_MAGIC);
    c
}

/// Which ROM bootloader the probe enters, from the discovery `kind` byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// No reachable ROM update path: the probe answers, but reboot is unsupported.
    None,
    /// UF2 / BOOTSEL mass-storage (RP2040 / RP2350).
    Uf2,
    /// ST system-memory USB DFU.
    StDfu,
    /// NXP ISP HID.
    NxpIsp,
    /// A kind byte this tool does not recognize.
    Unknown(u8),
}

impl Kind {
    /// Decode the discovery `kind` byte.
    pub fn from_u8(v: u8) -> Kind {
        match v {
            0x00 => Kind::None,
            0x01 => Kind::Uf2,
            0x02 => Kind::StDfu,
            0x03 => Kind::NxpIsp,
            other => Kind::Unknown(other),
        }
    }

    /// A short human-readable name for the bootloader.
    pub fn label(self) -> &'static str {
        match self {
            Kind::None => "none (no ROM update path)",
            Kind::Uf2 => "UF2 / BOOTSEL mass-storage",
            Kind::StDfu => "ST system DFU",
            Kind::NxpIsp => "NXP ISP HID",
            Kind::Unknown(_) => "unknown",
        }
    }

    /// Whether this probe can actually reboot to a bootloader.
    pub fn supports_reboot(self) -> bool {
        !matches!(self, Kind::None | Kind::Unknown(_))
    }

    /// What re-enumeration to expect, and the follow-up flashing command, after a reboot.
    pub fn followup_hint(self) -> &'static str {
        match self {
            Kind::Uf2 => "the RPI-RP2 drive will appear; copy the .uf2 onto it",
            Kind::StDfu => "the ST DFU device (0483:df11) will appear; reflash with `dfu-util`",
            Kind::NxpIsp => "the NXP ISP device (1FC9:0021) will appear; reflash with `blhost`",
            Kind::None => {
                "this board has no ROM bootloader; update it over SWD with a second probe"
            }
            Kind::Unknown(_) => "unrecognized bootloader kind; nothing to wait for",
        }
    }
}

/// A parsed discovery reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Discovery {
    /// The vendor-command protocol version byte.
    pub version: u8,
    /// The board's bootloader kind.
    pub kind: Kind,
}

/// Parse a discovery reply `[0x9F, b"dlrs", version, kind]`; a wrong id echo or magic means the
/// probe is not a daplink-rs probe.
pub fn parse_discovery(resp: &[u8]) -> Result<Discovery> {
    if resp.len() < 7 {
        bail!("discovery reply too short ({} bytes, need 7)", resp.len());
    }
    if resp[0] != REBOOT_CMD_ID {
        bail!(
            "unexpected command echo {:#04x} (not a 0x9F reply)",
            resp[0]
        );
    }
    if resp[1..5] != REPLY_MAGIC {
        bail!("not a daplink-rs probe (missing 'dlrs' magic)");
    }
    Ok(Discovery {
        version: resp[5],
        kind: Kind::from_u8(resp[6]),
    })
}

/// The reboot status byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Armed; the probe is about to detach.
    Armed,
    /// Reboot unsupported on this board.
    Unsupported,
    /// The magic was missing/wrong.
    BadMagic,
    /// The probe is busy (e.g. a flash transfer is in progress).
    Busy,
    /// A status byte this tool does not recognize.
    Unknown(u8),
}

impl Status {
    /// Decode the status byte.
    pub fn from_u8(v: u8) -> Status {
        match v {
            0x00 => Status::Armed,
            0x01 => Status::Unsupported,
            0x02 => Status::BadMagic,
            0x03 => Status::Busy,
            other => Status::Unknown(other),
        }
    }

    /// A human-readable description.
    pub fn label(self) -> &'static str {
        match self {
            Status::Armed => "armed (probe detaching)",
            Status::Unsupported => "unsupported on this board",
            Status::BadMagic => "rejected: bad magic",
            Status::Busy => "busy (transfer in progress)",
            Status::Unknown(_) => "unknown status",
        }
    }
}

/// Parse a reboot-status reply (`| 0x9F | status |`).
pub fn parse_status(resp: &[u8]) -> Result<Status> {
    if resp.len() < 2 {
        bail!("status reply too short ({} bytes, need 2)", resp.len());
    }
    if resp[0] != REBOOT_CMD_ID {
        bail!(
            "unexpected command echo {:#04x} (not a 0x9F reply)",
            resp[0]
        );
    }
    Ok(Status::from_u8(resp[1]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commands_have_the_expected_bytes() {
        assert_eq!(discovery_cmd(), [0x9F, 0x00]);
        assert_eq!(reboot_cmd(), [0x9F, 0x01, b'B', b'O', b'O', b'T']);
    }

    #[test]
    fn parses_a_uf2_discovery_reply() {
        let reply = [0x9F, b'd', b'l', b'r', b's', 0x01, 0x01];
        let d = parse_discovery(&reply).unwrap();
        assert_eq!(d.version, 1);
        assert_eq!(d.kind, Kind::Uf2);
        assert!(d.kind.supports_reboot());
    }

    #[test]
    fn parses_every_kind_byte() {
        assert_eq!(Kind::from_u8(0), Kind::None);
        assert_eq!(Kind::from_u8(1), Kind::Uf2);
        assert_eq!(Kind::from_u8(2), Kind::StDfu);
        assert_eq!(Kind::from_u8(3), Kind::NxpIsp);
        assert_eq!(Kind::from_u8(9), Kind::Unknown(9));
        assert!(!Kind::None.supports_reboot());
        assert!(!Kind::Unknown(9).supports_reboot());
    }

    #[test]
    fn rejects_a_foreign_probe_reply() {
        // Another probe answering 0x9F without the magic (or 0xFF invalid) is "not ours".
        let foreign = [0x9F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        assert!(parse_discovery(&foreign).is_err());
        let too_short = [0x9F, b'd', b'l'];
        assert!(parse_discovery(&too_short).is_err());
    }

    #[test]
    fn parses_status_bytes() {
        assert_eq!(parse_status(&[0x9F, 0x00]).unwrap(), Status::Armed);
        assert_eq!(parse_status(&[0x9F, 0x01]).unwrap(), Status::Unsupported);
        assert_eq!(parse_status(&[0x9F, 0x02]).unwrap(), Status::BadMagic);
        assert_eq!(parse_status(&[0x9F, 0x03]).unwrap(), Status::Busy);
        assert_eq!(parse_status(&[0x9F, 0x7F]).unwrap(), Status::Unknown(0x7F));
        assert!(parse_status(&[0x9F]).is_err());
    }
}
