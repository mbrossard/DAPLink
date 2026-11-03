//! dap-rs's transport mode, mirrored from the commands it answers, for paths that run ahead of it.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    None,
    Swd,
    Jtag,
}

/// dap-rs's current mode, or the one it restores, and whether it holds that mode's pads now.
#[derive(Clone, Copy, Debug)]
pub struct DapMode {
    mode: Mode,
    held: bool,
}

impl Default for DapMode {
    fn default() -> Self {
        Self::new()
    }
}

impl DapMode {
    /// dap-rs at boot: no mode, nothing to restore.
    pub const fn new() -> Self {
        Self {
            mode: Mode::None,
            held: false,
        }
    }

    /// dap-rs holds SWD with its pads, so an SWD transfer may run ahead of it.
    pub fn swd_held(&self) -> bool {
        self.held && self.mode == Mode::Swd
    }

    /// dap-rs holds JTAG with its pads.
    pub fn jtag_held(&self) -> bool {
        self.held && self.mode == Mode::Jtag
    }

    /// dap-rs parked the pads; its next transfer re-enters the mode it restores.
    pub fn suspend(&mut self) {
        self.held = false;
    }

    /// Follow the mode change dap-rs made answering `report` with `response`.
    pub fn observe(&mut self, report: &[u8], response: &[u8]) {
        let Some(&cmd) = report.first() else {
            return;
        };
        // A rejected report never reached dap-rs.
        if response.first() != Some(&cmd) {
            return;
        }
        match cmd {
            0x02 => match response.get(1) {
                Some(1) => self.enter(Mode::Swd),
                Some(2) => self.enter(Mode::Jtag),
                _ => {}
            },
            0x03 | 0x10 | 0x12 => self.held = false,
            // Transfers and ABORT restore the last mode first.
            0x05 | 0x06 | 0x08 => self.held = self.mode != Mode::None,
            0x14 | 0x16 => self.enter(Mode::Jtag),
            0x1D => self.enter(Mode::Swd),
            // A batch may switch modes unseen, so nothing runs ahead until a mode is entered again.
            0x7E | 0x7F => *self = Self::new(),
            _ => {}
        }
    }

    fn enter(&mut self, mode: Mode) {
        self.mode = mode;
        self.held = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seen(m: &mut DapMode, report: &[u8], response: &[u8]) -> (bool, bool) {
        m.observe(report, response);
        (m.swd_held(), m.jtag_held())
    }

    #[test]
    fn the_mode_follows_dap_rs_state_transitions() {
        let mut m = DapMode::new();
        assert_eq!(
            seen(&mut m, &[0x05, 0, 1, 2], &[0x05, 0, 0]),
            (false, false)
        );
        assert_eq!(seen(&mut m, &[0x02, 0], &[0x02, 1]), (true, false));
        assert_eq!(seen(&mut m, &[0x10, 0, 0x80], &[0x10, 0]), (false, false));
        assert_eq!(seen(&mut m, &[0x06, 0, 1, 0, 2], &[0x06, 0]), (true, false));
        assert_eq!(seen(&mut m, &[0x12, 8, 0], &[0x12, 0]), (false, false));
        assert_eq!(seen(&mut m, &[0x08, 0], &[0x08, 0]), (true, false));
        assert_eq!(seen(&mut m, &[0x16, 0], &[0x16, 0xFF]), (false, true));
        assert_eq!(seen(&mut m, &[0x03], &[0x03, 0]), (false, false));
        assert_eq!(seen(&mut m, &[0x05, 0, 0], &[0x05, 0, 0]), (false, true));
        assert_eq!(seen(&mut m, &[0x1D, 0], &[0x1D, 0]), (true, false));
        assert_eq!(seen(&mut m, &[0x14, 0], &[0x14, 0]), (false, true));
        assert_eq!(seen(&mut m, &[0x7F, 0], &[0x7F]), (false, false));
        assert_eq!(seen(&mut m, &[0x05, 0, 0], &[0x05, 0, 0]), (false, false));
    }

    #[test]
    fn a_failed_connect_or_a_rejected_report_changes_nothing() {
        let mut m = DapMode::new();
        m.observe(&[0x02, 1], &[0x02, 1]);
        assert_eq!(seen(&mut m, &[0x02, 9], &[0x02, 0]), (true, false));
        assert_eq!(seen(&mut m, &[0x1D], &[0xFF]), (true, false));
        assert_eq!(seen(&mut m, &[0x10, 0], &[0xFF]), (true, false));
        assert_eq!(seen(&mut m, &[0x0A], &[0x0A, 0, 0]), (true, false));
        m.suspend();
        assert_eq!((m.swd_held(), m.jtag_held()), (false, false));
    }
}
