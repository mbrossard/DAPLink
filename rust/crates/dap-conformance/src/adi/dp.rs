//! The debug port's registers, shared by SWD and JTAG, and its one AP's access path.

use super::jtag_dp::DpVersion;
use super::mem_ap::MemAp;
use super::regs::*;
use crate::target::Effect;
use crate::wire::swd::{Request, ACK_FAULT, ACK_OK, ACK_WAIT};

/// DP state: CTRL/STAT, SELECT, the sticky flags, the posted read and the AP's WAIT debt.
#[derive(Clone, Debug, Default)]
pub struct Dp {
    /// DPIDR.
    pub dpidr: u32,
    /// The JTAG-DP's DP version.
    pub jtag_version: DpVersion,
    /// The held CTRL/STAT bits.
    pub ctrl: u32,
    /// SELECT.
    pub select: u32,
    /// RDBUFF: the last AP read's result.
    pub rdbuff: u32,
    /// What RESEND repeats.
    pub resend: u32,
    /// CTRL/STAT.STICKYERR.
    pub stickyerr: bool,
    /// CTRL/STAT.WDATAERR.
    pub wdataerr: bool,
    /// WAITs owed before the AP is ready again.
    pub busy: u32,
    /// WAIT debts per AP access, cycled.
    pub waits: Vec<u8>,
    /// The next entry of `waits`.
    pub wait_at: usize,
}

impl Dp {
    /// The SWD ACK for `req`: FAULT on a sticky error, WAIT while the AP is busy.
    pub fn swd_ack(&mut self, req: Request) -> u8 {
        let exempt = !req.ap && ((req.read && req.a <= 1) || (!req.read && req.a == 0));
        if (self.stickyerr || self.wdataerr) && !exempt {
            ACK_FAULT
        } else if self.busy > 0 && (req.ap || (req.read && req.a == DP_RDBUFF)) {
            self.busy -= 1;
            ACK_WAIT
        } else {
            ACK_OK
        }
    }

    /// An OK SWD read: an AP read returns the posted result and posts its own.
    pub fn swd_read(&mut self, req: Request, ap: &mut MemAp, fx: &mut Vec<Effect>) -> u32 {
        let v = if req.ap {
            let posted = self.rdbuff;
            self.rdbuff = self.ap(ap, true, req.a, 0, fx);
            posted
        } else {
            self.read(req.a, false)
        };
        self.resend = v;
        v
    }

    /// An OK SWD write's data phase; bad parity sets WDATAERR instead.
    pub fn swd_write(
        &mut self,
        req: Request,
        data: u32,
        parity_ok: bool,
        ap: &mut MemAp,
        fx: &mut Vec<Effect>,
    ) {
        if !parity_ok {
            self.wdataerr = true;
            fx.push(Effect::WriteParity);
        } else if req.ap {
            self.ap(ap, false, req.a, data, fx);
        } else {
            self.write(req.a, data, false, fx);
        }
    }

    fn ctrl_stat(&self) -> u32 {
        let c = self.ctrl;
        c | (c >> 28 & 1) << 29
            | (c >> 30 & 1) << 31
            | u32::from(self.stickyerr) << 5
            | u32::from(self.wdataerr) << 7
    }

    /// A DP register read over SWD or, with `jtag`, DPACC; UNKNOWN reads as the complement.
    pub fn read(&mut self, a: u8, jtag: bool) -> u32 {
        match a {
            DP_DPIDR if jtag && self.jtag_version == DpVersion::V0 => !self.dpidr,
            DP_DPIDR => self.dpidr,
            DP_CTRL_STAT if self.select & 0xF == 0 => self.ctrl_stat(),
            DP_CTRL_STAT => 0,
            DP_SELECT if jtag => match self.jtag_version {
                DpVersion::V0 => self.select,
                DpVersion::V1 | DpVersion::V2 => !self.select,
                DpVersion::V3 => 0,
            },
            DP_SELECT => self.resend,
            _ if jtag => 0,
            _ => self.rdbuff,
        }
    }

    /// A DP register write over SWD or, with `jtag`, DPACC, where address 0 is not ABORT.
    pub fn write(&mut self, a: u8, data: u32, jtag: bool, fx: &mut Vec<Effect>) {
        if a == DP_DPIDR {
            if !jtag {
                self.abort(data, fx);
            }
            return;
        }
        fx.push(Effect::Dp { addr: a << 2, data });
        match a {
            DP_CTRL_STAT if self.select & 0xF == 0 => {
                self.ctrl = data & CTRL_MASK;
                if jtag && data & STICKYERR != 0 {
                    self.stickyerr = false;
                }
            }
            DP_SELECT => self.select = data,
            _ => {}
        }
    }

    /// An ABORT write.
    pub fn abort(&mut self, data: u32, fx: &mut Vec<Effect>) {
        fx.push(Effect::Abort(data));
        self.take_abort(data);
    }

    /// An ABORT scan: a DPv0 JTAG-DP takes DAPABORT only.
    pub fn jtag_abort(&mut self, data: u32, fx: &mut Vec<Effect>) {
        fx.push(Effect::Abort(data));
        self.take_abort(match self.jtag_version {
            DpVersion::V0 => data & DAPABORT,
            _ => data,
        });
    }

    fn take_abort(&mut self, data: u32) {
        if data & DAPABORT != 0 {
            self.busy = 0;
        }
        if data & STKERRCLR != 0 {
            self.stickyerr = false;
        }
        if data & WDERRCLR != 0 {
            self.wdataerr = false;
        }
    }

    /// One AP register access through SELECT; arms the WAITs owed before the AP is ready again.
    pub fn ap(
        &mut self,
        ap: &mut MemAp,
        read: bool,
        a: u8,
        data: u32,
        fx: &mut Vec<Effect>,
    ) -> u32 {
        let apsel = (self.select >> 24) as u8;
        let addr = (self.select & 0xF0) as u8 | a << 2;
        let value = if apsel == 0 {
            let (value, fault) = ap.access(read, addr, data);
            if fault {
                self.stickyerr = true;
                fx.push(Effect::Sticky);
            }
            value
        } else {
            0
        };
        fx.push(if read {
            Effect::ApRead { apsel, addr, value }
        } else {
            Effect::ApWrite { apsel, addr, data }
        });
        self.busy = if self.waits.is_empty() {
            0
        } else {
            u32::from(self.waits[self.wait_at % self.waits.len()])
        };
        self.wait_at += 1;
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DPIDR: u32 = 0x2BA0_1477;
    const VERSIONS: [DpVersion; 4] = [DpVersion::V0, DpVersion::V1, DpVersion::V2, DpVersion::V3];

    fn dp(jtag_version: DpVersion) -> Dp {
        Dp {
            dpidr: DPIDR,
            jtag_version,
            ..Dp::default()
        }
    }

    #[test]
    fn dpacc_reads_at_0x0_and_0x8_follow_the_dp_version() {
        let select = 0x0100_00F0;
        for (version, at_0, at_8) in [
            (DpVersion::V0, !DPIDR, select),
            (DpVersion::V1, DPIDR, !select),
            (DpVersion::V2, DPIDR, !select),
            (DpVersion::V3, DPIDR, 0),
        ] {
            let mut dp = dp(version);
            dp.write(DP_SELECT, select, true, &mut Vec::new());
            assert_eq!(dp.read(DP_DPIDR, true), at_0, "{version:?}");
            assert_eq!(dp.read(DP_SELECT, true), at_8, "{version:?}");
            assert_eq!(dp.read(DP_DPIDR, false), DPIDR, "{version:?}");
        }
    }

    #[test]
    fn a_dpv0_abort_scan_takes_dapabort_only() {
        for version in VERSIONS {
            let mut dp = Dp {
                busy: 3,
                stickyerr: true,
                ..dp(version)
            };
            let mut fx = Vec::new();
            dp.jtag_abort(0x1F, &mut fx);
            assert_eq!(fx, [Effect::Abort(0x1F)]);
            assert_eq!((dp.busy, dp.stickyerr), (0, version == DpVersion::V0));
            dp.write(DP_CTRL_STAT, STICKYERR, true, &mut fx);
            assert!(!dp.stickyerr, "{version:?}");
        }
    }

    #[test]
    fn an_swd_abort_clears_whatever_the_jtag_dp_version() {
        for version in VERSIONS {
            let mut dp = Dp {
                stickyerr: true,
                ..dp(version)
            };
            dp.abort(STKERRCLR, &mut Vec::new());
            assert!(!dp.stickyerr, "{version:?}");
        }
    }
}
