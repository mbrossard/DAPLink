//! `DAP_TransferBlock` counts and empty transfers answered as the Arm firmware answers them.

const DAP_CONNECT: u8 = 0x02;
const DAP_TRANSFER: u8 = 0x05;
const DAP_TRANSFER_BLOCK: u8 = 0x06;
const DAP_JTAG_SEQUENCE: u8 = 0x14;
const DAP_JTAG_IDCODE: u8 = 0x16;
const DAP_SWD_SEQUENCE: u8 = 0x1D;
const OK: u8 = 1;

/// dap-rs counts the transfer that stopped an SWD block; this follows its mode to correct that.
pub(crate) struct TransferCount {
    /// 1 once dap-rs's last mode is SWD; a `u8`, since a `bool`'s niche reorders the engine.
    swd: u8,
}

impl TransferCount {
    pub(crate) const fn new() -> Self {
        Self { swd: 0 }
    }

    /// Follow dap-rs through `report`, which it answered with `response`.
    pub(crate) fn observe(&mut self, report: &[u8], response: &[u8]) {
        let Some(&cmd) = report.first() else {
            return;
        };
        if response.first() != Some(&cmd) {
            return;
        }
        match cmd {
            DAP_CONNECT => match response.get(1) {
                Some(1) => self.swd = 1,
                Some(2) => self.swd = 0,
                _ => {}
            },
            DAP_SWD_SEQUENCE => self.swd = 1,
            DAP_JTAG_SEQUENCE | DAP_JTAG_IDCODE => self.swd = 0,
            _ => {}
        }
    }

    /// Correct dap-rs's answer `response[..len]` to `report`.
    pub(crate) fn correct(&self, report: &[u8], response: &mut [u8], len: usize) {
        match report.first().copied() {
            Some(DAP_TRANSFER) if len >= 3 => {
                if report[2] == 0 {
                    // dap-rs's JTAG path answers 0x08.
                    response[2] = 0;
                } else if response[2] == 0 && response[1] > 0 {
                    // Only match mask writes ran; dap-rs leaves the status unset.
                    response[2] = OK;
                }
            }
            Some(DAP_TRANSFER_BLOCK) if self.swd != 0 && len >= 4 && response[3] != OK => {
                let done = if report[4] & 2 != 0 {
                    (len - 4) / 4
                } else {
                    usize::from(u16::from_le_bytes([response[1], response[2]])).saturating_sub(1)
                };
                response[1..3].copy_from_slice(&(done as u16).to_le_bytes());
            }
            _ => {}
        }
    }
}
