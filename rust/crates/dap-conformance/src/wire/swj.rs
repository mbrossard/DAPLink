//! SWJ-DP sequences: the line reset and the JTAG-to-SWD and SWD-to-JTAG switches.

/// The JTAG-to-SWD select sequence, first bit in bit 0.
pub const JTAG_TO_SWD: u16 = 0xE79E;
/// The SWD-to-JTAG select sequence, first bit in bit 0.
pub const SWD_TO_JTAG: u16 = 0xE73C;
/// High cells that make a line reset.
pub const LINE_RESET: u32 = 50;

/// The protocol an SWJ-DP is in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Serial Wire Debug.
    Swd,
    /// JTAG.
    Jtag,
}

/// A sequence the detector completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Swj {
    /// The fiftieth consecutive high cell in SWD.
    LineReset,
    /// A select sequence after a line reset switched to this protocol.
    Switch(Mode),
}

/// Watches SWDIO/TMS for line resets and select sequences.
#[derive(Clone, Debug, Default)]
pub struct Detector {
    ones: u32,
    select: Option<(u8, u16)>,
}

impl Detector {
    /// One cell of SWDIO/TMS while the SWJ-DP is in `mode`.
    pub fn clock(&mut self, bit: bool, mode: Mode) -> Option<Swj> {
        if let Some((n, v)) = self.select {
            let v = v | (u16::from(bit) << n);
            self.select = if n == 15 { None } else { Some((n + 1, v)) };
            let to = match v {
                JTAG_TO_SWD if n == 15 => Some(Mode::Swd),
                SWD_TO_JTAG if n == 15 => Some(Mode::Jtag),
                _ => None,
            };
            if let Some(to) = to.filter(|&m| m != mode) {
                self.ones = 0;
                return Some(Swj::Switch(to));
            }
        }
        if bit {
            self.ones += 1;
            if self.ones == LINE_RESET && mode == Mode::Swd {
                return Some(Swj::LineReset);
            }
        } else {
            if self.ones >= LINE_RESET && self.select.is_none() {
                self.select = Some((1, 0));
            }
            self.ones = 0;
        }
        None
    }
}
