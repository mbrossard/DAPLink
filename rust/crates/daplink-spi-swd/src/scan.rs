//! The specialized fast JTAG scans: one DPACC/APACC transfer and one IR shift over the
//! [`JtagPortOps`] seam, wire-identical to dap-rs's provided per-chunk choreography (same TAP
//! walks, bit orders, TMS placement, JTAG-DP V0 ACK read, multi-TAP bypass fills) but run as a
//! single straight-line routine per word.

use dap_rs::jtag::{self, TransferResult};
use dap_rs::swd::RnW;

use crate::port::JtagPortOps;

/// The fast JTAG word path as a seam a backend implements over its own JTAG state: the pin seam,
/// the scan-chain configuration, and the two half-period pacers buy the two scans. The pacing
/// halves are the whole timing policy and are called once per TCK; the low half carries the TDO
/// sampling obligation.
pub trait JtagScan {
    /// The pin seam carrying TCK/TMS/TDI/TDO.
    type Port: JtagPortOps;

    /// The pin seam.
    fn scan_port(&self) -> &Self::Port;

    /// The scan-chain configuration the bypass fills and IR geometry are counted from.
    fn scan_config(&mut self) -> &jtag::Config;

    /// Pace the low half of a clock: the TCK-fall to TDO-sample margin, the longer of the two.
    fn scan_wait_low(&mut self);

    /// Pace the high half: TCK-high pulse width only.
    fn scan_wait_high(&mut self);

    /// Pace a sampled or driven data low half, defaulting to the control pacing.
    #[inline(always)]
    fn scan_wait_data_low(&mut self) {
        self.scan_wait_low();
    }

    /// Pace a sampled or driven data high half, defaulting to the control pacing.
    #[inline(always)]
    fn scan_wait_data_high(&mut self) {
        self.scan_wait_high();
    }

    /// One fast TAP clock at the current TMS/TDI levels, nothing sampled.
    #[inline(always)]
    fn scan_clock(&mut self) {
        self.scan_port().tck_low();
        self.scan_wait_low();
        self.scan_port().tck_high();
        self.scan_wait_high();
    }

    /// One fast TAP clock sampling TDO after the low half (before the rising edge).
    #[inline(always)]
    fn scan_clock_in(&mut self) -> u32 {
        self.scan_port().tck_low();
        self.scan_wait_data_low();
        let bit = self.scan_port().tdo_get();
        self.scan_port().tck_high();
        self.scan_wait_data_high();
        bit
    }

    /// One TAP data-output clock at the current TMS/TDI levels, nothing sampled.
    #[inline(always)]
    fn scan_clock_out(&mut self) {
        self.scan_port().tck_low();
        self.scan_wait_data_low();
        self.scan_port().tck_high();
        self.scan_wait_data_high();
    }

    /// The specialized DPACC/APACC scan, for a `Jtag::transfer` override. Note `write_abort`
    /// does not route through `transfer`, so it keeps the generic mechanics regardless.
    #[inline(always)]
    fn transfer_word(&mut self, rnw: RnW, a2a3: u8, idle_cycles: u8, data: u32) -> TransferResult {
        // Bypass counts before/after the selected TAP; saturate so an impossible state degrades
        // to a zero-length fill rather than a huge loop.
        let before = self.scan_config().index as u32;
        let after = (self.scan_config().device_count as u32).saturating_sub(before + 1);

        // Run-Test/Idle -> Shift-DR: TMS 1,0,0.
        self.scan_port().tms_set(true);
        self.scan_clock();
        self.scan_port().tms_set(false);
        self.scan_clock();
        self.scan_clock();

        // Bypass-before fill: TDI high, one TCK per TAP in front of the selected one.
        self.scan_port().tdi_set(true);
        for _ in 0..before {
            self.scan_clock();
        }

        // 3-bit RnW + A[3:2], LSB-first, capturing the ACK.
        let req = ((a2a3 as u32) << 1) | rnw as u32;
        let mut ack = 0u32;
        for i in 0..3 {
            self.scan_port().tdi_set((req >> i) & 1 != 0);
            ack |= self.scan_clock_in() << i;
        }

        let result = if ack == 0x02 {
            let captured = if rnw == RnW::R {
                // Read data phase: TDI held low, TDO accumulated LSB-first.
                self.scan_port().tdi_set(false);
                let mut v = 0u32;
                for i in 0..31 {
                    v |= self.scan_clock_in() << i;
                }
                if after == 0 {
                    self.scan_port().tms_set(true); // final data bit exits Shift-DR
                }
                v | (self.scan_clock_in() << 31)
            } else {
                // Write data phase: TDI per bit, nothing captured.
                let mut w = data;
                for _ in 0..31 {
                    self.scan_port().tdi_set(w & 1 != 0);
                    self.scan_clock_out();
                    w >>= 1;
                }
                if after == 0 {
                    self.scan_port().tms_set(true); // final data bit exits Shift-DR
                }
                self.scan_port().tdi_set(w & 1 != 0);
                self.scan_clock_out();
                0
            };

            // Bypass-after fill: TDI high, TMS low except the last bit, which exits Shift-DR.
            if after > 0 {
                self.scan_port().tdi_set(true);
                for _ in 0..after - 1 {
                    self.scan_clock();
                }
                self.scan_port().tms_set(true);
                self.scan_clock();
            }

            // Exit1-DR -> Update-DR -> Run-Test/Idle: TMS 1,0.
            self.scan_port().tms_set(true);
            self.scan_clock();
            self.scan_port().tms_set(false);
            self.scan_clock();
            TransferResult::Ok(captured)
        } else {
            // Non-OK ACK: no data phase; Shift-DR -> Exit1 -> Update -> Idle: TMS 1,1,0.
            self.scan_port().tms_set(true);
            self.scan_clock();
            self.scan_clock();
            self.scan_port().tms_set(false);
            self.scan_clock();
            TransferResult::Wait
        };

        // Idle cycles in Run-Test/Idle (TMS already low), TDI high as the provided fill.
        self.scan_port().tdi_set(true);
        for _ in 0..idle_cycles {
            self.scan_clock();
        }

        result
    }

    /// The specialized IR scan, for a `Jtag::shift_ir` override: counted bypass fills carry
    /// multi-TAP generality; wire-identical to the provided `shift_ir`.
    #[inline(always)]
    fn shift_ir_word(&mut self, ir: u32) {
        let tap = self.scan_config().current_tap();
        let ir_length = tap.ir_length as u32;
        let before = tap.ir_before as u32;
        let after = tap.ir_after as u32;

        self.shift_ir_geometry(ir, ir_length, before, after);
    }

    /// Shift an instruction with already-resolved chain geometry.
    #[inline(always)]
    fn shift_ir_geometry(&mut self, ir: u32, ir_length: u32, before: u32, after: u32) {
        // Run-Test/Idle -> Shift-IR: TMS 1,1,0,0.
        self.scan_port().tms_set(true);
        self.scan_clock();
        self.scan_clock();
        self.scan_port().tms_set(false);
        self.scan_clock();
        self.scan_clock();

        // Bypass-before fill (the TAPs in front, held in BYPASS = all ones).
        self.scan_port().tdi_set(true);
        for _ in 0..before {
            self.scan_clock();
        }

        // IR bits, LSB-first; the last bit exits Shift-IR only when there is no bypass-after.
        let mut v = ir;
        if ir_length > 0 {
            for _ in 0..ir_length - 1 {
                self.scan_port().tdi_set(v & 1 != 0);
                self.scan_clock_out();
                v >>= 1;
            }
            if after == 0 {
                self.scan_port().tms_set(true); // final IR bit exits Shift-IR
            }
            self.scan_port().tdi_set(v & 1 != 0);
            self.scan_clock_out();
        }

        // Bypass-after fill (the TAPs behind), last bit exits Shift-IR.
        if after > 0 {
            self.scan_port().tdi_set(true);
            for _ in 0..after - 1 {
                self.scan_clock();
            }
            self.scan_port().tms_set(true);
            self.scan_clock();
        }

        // Exit1-IR -> Update-IR -> Run-Test/Idle: TMS 1,0.
        self.scan_port().tms_set(true);
        self.scan_clock();
        self.scan_port().tms_set(false);
        self.scan_clock();
    }
}
