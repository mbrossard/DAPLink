//! Specialized JTAG DPACC/APACC and IR scans, wire-identical to dap-rs's per-chunk choreography.

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use dap_rs::jtag::{self, TransferResult};
use dap_rs::swd::RnW;
use daplink_core::jtag_dp::{ACK_OK_FAULT, ACK_WAIT, STATUS_OK, STATUS_WAIT};
use daplink_core::JtagDp;

use crate::port::JtagPortOps;
use crate::timing::{HalfCycles, JtagTimingPlan, PacingPlan};

/// The fast JTAG scan seam: the pin seam, chain configuration and the two half-period pacers.
pub trait JtagScan {
    /// The pin seam carrying TCK/TMS/TDI/TDO.
    type Port: JtagPortOps;

    /// The pin seam.
    fn scan_port(&self) -> &Self::Port;

    /// The scan-chain configuration the bypass fills and IR geometry are counted from.
    fn scan_config(&mut self) -> &jtag::Config;

    /// Keep a scan's ACK in memory, freeing its register for the data cells;
    /// `transfer_word` keeps every ACK but `0b010`.
    fn scan_keep_ack(&mut self, ack: u32);

    /// A scan's result after its last cell, `data` the captured word.
    fn scan_complete(&mut self, data: u32) -> TransferResult;

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

    /// Pace an unsampled data-output low half, defaulting to sampled data pacing.
    #[inline(always)]
    fn scan_wait_output_low(&mut self) {
        self.scan_wait_data_low();
    }

    /// Pace an ACK low half, defaulting to the data pacing.
    #[inline(always)]
    fn scan_wait_ack_low(&mut self) {
        self.scan_wait_data_low();
    }

    /// Pace an ACK high half, defaulting to the data pacing.
    #[inline(always)]
    fn scan_wait_ack_high(&mut self) {
        self.scan_wait_data_high();
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

    /// One ACK clock sampling TDO after the low half.
    #[inline(always)]
    fn scan_clock_ack_in(&mut self) -> u32 {
        self.scan_port().tck_low();
        self.scan_wait_ack_low();
        let bit = self.scan_port().tdo_get();
        self.scan_port().tck_high();
        self.scan_wait_ack_high();
        bit
    }

    /// One TAP data-output clock at the current TMS/TDI levels, nothing sampled.
    #[inline(always)]
    fn scan_clock_out(&mut self) {
        self.scan_port().tck_low();
        self.scan_wait_output_low();
        self.scan_port().tck_high();
        self.scan_wait_data_high();
    }

    /// The read data phase as straight-line cells; `None`, the default, keeps the counted loop.
    #[inline(always)]
    fn scan_read_cells(&mut self, _after: u32) -> Option<u32> {
        None
    }

    /// The write data phase as straight-line cells, driving `data` on TDI.
    #[inline(always)]
    fn scan_write_cells(&mut self, _data: u32, _after: u32) -> Option<()> {
        None
    }

    /// The specialized DPACC/APACC scan, leaving early on WAIT only: `Ok` once complete.
    #[inline(always)]
    fn transfer_word(&mut self, rnw: RnW, a2a3: u8, idle_cycles: u8, data: u32) -> TransferResult {
        // Saturating, so an impossible state degrades to an empty fill.
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
            ack |= self.scan_clock_ack_in() << i;
        }

        let complete = ack == ACK_OK_FAULT || {
            core::hint::cold_path();
            self.scan_keep_ack(ack);
            ack != ACK_WAIT
        };

        let captured = if complete {
            let captured = if rnw == RnW::R {
                self.scan_port().tdi_set(false);
                if let Some(v) = self.scan_read_cells(after) {
                    v
                } else {
                    let mut v = 0u32;
                    for i in 0..31 {
                        v |= self.scan_clock_in() << i;
                    }
                    if after == 0 {
                        self.scan_port().tms_set(true); // final data bit exits Shift-DR
                    }
                    v | (self.scan_clock_in() << 31)
                }
            } else {
                if self.scan_write_cells(data, after).is_none() {
                    let mut w = data;
                    for _ in 0..31 {
                        self.scan_port().tdi_put(w);
                        self.scan_clock_out();
                        w >>= 1;
                    }
                    if after == 0 {
                        self.scan_port().tms_set(true); // final data bit exits Shift-DR
                    }
                    self.scan_port().tdi_put(w);
                    self.scan_clock_out();
                }
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
            captured
        } else {
            // WAIT: no data phase; Shift-DR -> Exit1 -> Update -> Idle: TMS 1,1,0.
            self.scan_port().tms_set(true);
            self.scan_clock();
            self.scan_clock();
            self.scan_port().tms_set(false);
            self.scan_clock();
            0
        };

        // Idle cycles in Run-Test/Idle (TMS already low), TDI high as the provided fill.
        self.scan_port().tdi_set(true);
        for _ in 0..idle_cycles {
            self.scan_clock();
        }

        self.scan_complete(captured)
    }

    /// The specialized IR scan for a `Jtag::shift_ir` override.
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
                self.scan_port().tdi_put(v);
                self.scan_clock_out();
                v >>= 1;
            }
            if after == 0 {
                self.scan_port().tms_set(true); // final IR bit exits Shift-IR
            }
            self.scan_port().tdi_put(v);
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

/// The state a prepared scan borrows: the pin seam, chain configuration and both pacing sources.
pub trait JtagScanState {
    /// The pin seam carrying TCK/TMS/TDI/TDO.
    type Port: JtagPortOps;

    /// The pin seam.
    fn scan_port(&self) -> &Self::Port;

    /// The scan-chain configuration the bypass fills and IR geometry are counted from.
    fn scan_config(&mut self) -> &jtag::Config;

    /// Keep a scan's ACK; see [`JtagScan::scan_keep_ack`].
    fn scan_keep_ack(&mut self, ack: u32);

    /// A scan's result; see [`JtagScan::scan_complete`].
    fn scan_complete(&mut self, data: u32) -> TransferResult;

    /// Spin `cycles` core cycles, the counted pacing below the fast threshold.
    fn scan_spin(&mut self, cycles: u32);

    /// Pace a counted low half, preserving the default counted delay.
    #[inline(always)]
    fn scan_spin_low(&mut self, cycles: u32) {
        self.scan_spin(cycles);
    }

    /// Pace a counted high half, preserving the default counted delay.
    #[inline(always)]
    fn scan_spin_high(&mut self, cycles: u32) {
        self.scan_spin(cycles);
    }

    /// Pace an unsampled data-output low half, defaulting to sampled low pacing.
    #[inline(always)]
    fn scan_spin_output_low(&mut self, cycles: u32) {
        self.scan_spin_low(cycles);
    }

    /// The compile-time ladder's low half.
    fn scan_pace_low(&self);

    /// The compile-time ladder's high half.
    fn scan_pace_high(&self);
}

/// A scan paced by the compile-time ladder: both halves are straight-line blocks.
pub struct FastScan<'a, S: JtagScanState>(pub &'a mut S);

impl<S: JtagScanState> JtagScan for FastScan<'_, S> {
    type Port = S::Port;

    #[inline(always)]
    fn scan_port(&self) -> &Self::Port {
        self.0.scan_port()
    }

    #[inline(always)]
    fn scan_config(&mut self) -> &jtag::Config {
        self.0.scan_config()
    }

    #[inline(always)]
    fn scan_keep_ack(&mut self, ack: u32) {
        self.0.scan_keep_ack(ack);
    }

    #[inline(always)]
    fn scan_complete(&mut self, data: u32) -> TransferResult {
        self.0.scan_complete(data)
    }

    #[inline(always)]
    fn scan_wait_low(&mut self) {
        self.0.scan_pace_low();
    }

    #[inline(always)]
    fn scan_wait_high(&mut self) {
        self.0.scan_pace_high();
    }

    /// Straight-line cells: a counted loop's branch would tie each cell's timing to its alignment.
    #[inline(always)]
    fn scan_write_cells(&mut self, data: u32, after: u32) -> Option<()> {
        macro_rules! cells {
            ($($i:literal)*) => {$(
                self.scan_port().tdi_put(data >> $i);
                self.scan_clock_out();
            )*};
        }
        cells!(0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30);
        self.scan_port().tms_set(after == 0); // final data bit exits Shift-DR
        self.scan_port().tdi_put(data >> 31);
        self.scan_clock_out();
        Some(())
    }
}

/// A scan paced by counted spins prepared at the last clock change.
pub struct ScaledScan<'a, S: JtagScanState> {
    state: &'a mut S,
    control: HalfCycles,
    data: HalfCycles,
}

impl<'a, S: JtagScanState> ScaledScan<'a, S> {
    /// Borrow `state` for one scan at the prepared control and data half-periods.
    pub fn new(state: &'a mut S, control: HalfCycles, data: HalfCycles) -> Self {
        Self {
            state,
            control,
            data,
        }
    }
}

impl<S: JtagScanState> JtagScan for ScaledScan<'_, S> {
    type Port = S::Port;

    #[inline(always)]
    fn scan_port(&self) -> &Self::Port {
        self.state.scan_port()
    }

    #[inline(always)]
    fn scan_config(&mut self) -> &jtag::Config {
        self.state.scan_config()
    }

    #[inline(always)]
    fn scan_keep_ack(&mut self, ack: u32) {
        self.state.scan_keep_ack(ack);
    }

    #[inline(always)]
    fn scan_complete(&mut self, data: u32) -> TransferResult {
        self.state.scan_complete(data)
    }

    #[inline(always)]
    fn scan_wait_low(&mut self) {
        self.state.scan_spin_low(self.control.low());
    }

    #[inline(always)]
    fn scan_wait_high(&mut self) {
        self.state.scan_spin_high(self.control.high());
    }

    #[inline(always)]
    fn scan_wait_data_low(&mut self) {
        self.state.scan_spin_low(self.data.low());
    }

    #[inline(always)]
    fn scan_wait_data_high(&mut self) {
        self.state.scan_spin_high(self.data.high());
    }

    #[inline(always)]
    fn scan_wait_output_low(&mut self) {
        self.state.scan_spin_output_low(self.data.low());
    }
}

/// Keep each pacing specialization in one symbol with an exclusive state borrow.
#[inline(never)]
fn transfer_word_entry<S: JtagScanState, const FAST: bool>(
    state: &mut S,
    halves: (HalfCycles, HalfCycles),
    rnw: RnW,
    a2a3: u8,
    idle_cycles: u8,
    data: u32,
) -> TransferResult {
    if FAST {
        FastScan(state).transfer_word(rnw, a2a3, idle_cycles, data)
    } else {
        ScaledScan::new(state, halves.0, halves.1).transfer_word(rnw, a2a3, idle_cycles, data)
    }
}

/// One monomorphized IR scan, kept out of its caller for the same reason.
#[inline(never)]
fn shift_ir_entry<S: JtagScanState, const FAST: bool>(
    state: &mut S,
    halves: (HalfCycles, HalfCycles),
    ir: u32,
    ir_length: u32,
    before: u32,
    after: u32,
) {
    if FAST {
        FastScan(state).shift_ir_geometry(ir, ir_length, before, after);
    } else {
        ScaledScan::new(state, halves.0, halves.1).shift_ir_geometry(ir, ir_length, before, after);
    }
}

/// An engine's kept ACK: zero while its scans end on `0b010` and every TAP is protocol 0.
pub struct KeptAck {
    slot: AtomicU32,
    dp: &'static JtagDp,
}

impl KeptAck {
    const KEPT: u32 = 1 << 3;
    const PROTOCOL1: u32 = 1 << 4;

    /// Nothing kept, decoding through `dp`.
    pub const fn new(dp: &'static JtagDp) -> Self {
        Self {
            slot: AtomicU32::new(0),
            dp,
        }
    }

    /// The JTAG-DP state the ACKs decode through.
    pub fn dp(&self) -> &'static JtagDp {
        self.dp
    }

    /// Keep a scan's ACK other than `0b010`.
    #[inline(always)]
    pub fn keep(&self, ack: u32) {
        self.slot.store(ack | Self::KEPT, Relaxed);
    }

    /// Whether a scan completes without a decode: nothing kept, and every TAP protocol 0.
    #[inline(always)]
    pub fn clear(&self) -> bool {
        self.slot.load(Relaxed) == 0
    }

    /// A scan's result, `index` read only if it must decode.
    #[inline(always)]
    pub fn complete(&self, index: impl FnOnce() -> u8, data: u32) -> TransferResult {
        if self.clear() {
            TransferResult::Ok(data)
        } else {
            self.decode(index(), data)
        }
    }

    /// FAULT and no ACK both stop dap-rs without a retry.
    #[cold]
    #[inline(never)]
    fn decode(&self, index: u8, data: u32) -> TransferResult {
        let slot = self.slot.load(Relaxed);
        let ack = if slot & Self::KEPT != 0 {
            slot & 0b111
        } else {
            ACK_OK_FAULT
        };
        let status = self.dp.status(index, ack);
        let slot = if self.dp.any_protocol1() {
            Self::PROTOCOL1
        } else {
            0
        };
        self.slot.store(slot, Relaxed);
        match status {
            STATUS_OK => TransferResult::Ok(data),
            STATUS_WAIT => TransferResult::Wait,
            _ => TransferResult::Fault,
        }
    }
}

/// Run one DPACC/APACC scan at the pacing `plan` prepared at the last clock change.
#[inline(always)]
pub fn scan_transfer_word<S: JtagScanState>(
    state: &mut S,
    plan: JtagTimingPlan,
    rnw: RnW,
    a2a3: u8,
    idle_cycles: u8,
    data: u32,
) -> TransferResult {
    let pacing = match rnw {
        RnW::R => plan.read(),
        RnW::W => plan.write(),
    };
    match pacing {
        PacingPlan::Fast => transfer_word_entry::<_, true>(
            state,
            (plan.control(), plan.control()),
            rnw,
            a2a3,
            idle_cycles,
            data,
        ),
        PacingPlan::Scaled(data_half) => transfer_word_entry::<_, false>(
            state,
            (plan.control(), data_half),
            rnw,
            a2a3,
            idle_cycles,
            data,
        ),
    }
}

/// Run one IR scan at the pacing `plan` prepared at the last clock change.
#[inline(always)]
pub fn scan_shift_ir_word<S: JtagScanState>(state: &mut S, plan: JtagTimingPlan, ir: u32) {
    let tap = state.scan_config().current_tap();
    let ir_length = u32::from(tap.ir_length);
    let before = u32::from(tap.ir_before);
    let after = u32::from(tap.ir_after);
    match plan.ir() {
        PacingPlan::Fast => shift_ir_entry::<_, true>(
            state,
            (plan.control(), plan.control()),
            ir,
            ir_length,
            before,
            after,
        ),
        PacingPlan::Scaled(data_half) => shift_ir_entry::<_, false>(
            state,
            (plan.control(), data_half),
            ir,
            ir_length,
            before,
            after,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::timing::{prepare_jtag_timing, HalfCycleProfile, PacingProfile};
    extern crate std;
    use self::std::boxed::Box;
    use self::std::cell::{Cell, RefCell};
    use self::std::vec;
    use self::std::vec::Vec;

    /// A [`JtagPortOps`] recording (TMS, TDI) at every rising edge and replaying scripted TDO.
    #[derive(Default)]
    struct MockPort {
        tms: Cell<bool>,
        tdi: Cell<bool>,
        cells: RefCell<Vec<(bool, bool)>>,
        samples: RefCell<Vec<u32>>,
        sample: Cell<usize>,
        acquired: Cell<usize>,
        released: Cell<usize>,
    }

    impl JtagPortOps for MockPort {
        fn jtag_pads_acquire(&mut self) {
            self.acquired.set(self.acquired.get() + 1);
        }

        fn jtag_pads_release(&mut self) {
            self.released.set(self.released.get() + 1);
        }

        fn tck_low(&self) {}

        fn tck_high(&self) {
            self.cells
                .borrow_mut()
                .push((self.tms.get(), self.tdi.get()));
        }

        fn tms_set(&self, high: bool) {
            self.tms.set(high);
        }

        fn tdi_set(&self, high: bool) {
            self.tdi.set(high);
        }

        fn tdo_get(&self) -> u32 {
            let i = self.sample.get();
            self.sample.set(i + 1);
            self.samples.borrow().get(i).copied().unwrap_or(0)
        }

        fn tdi_is_high(&mut self) -> bool {
            self.tdi.get()
        }

        fn tdo_is_high(&mut self) -> bool {
            false
        }
    }

    struct MockScan {
        port: MockPort,
        config: jtag::Config,
        ack: u32,
    }

    impl JtagScan for MockScan {
        type Port = MockPort;

        fn scan_port(&self) -> &MockPort {
            &self.port
        }

        fn scan_config(&mut self) -> &jtag::Config {
            &self.config
        }

        fn scan_keep_ack(&mut self, ack: u32) {
            self.ack = ack;
        }

        fn scan_complete(&mut self, data: u32) -> TransferResult {
            if self.ack == ACK_WAIT {
                TransferResult::Wait
            } else {
                TransferResult::Ok(data)
            }
        }

        fn scan_wait_low(&mut self) {}

        fn scan_wait_high(&mut self) {}
    }

    impl MockScan {
        /// Build a chain from IR lengths in `DAP_JTAG_Configure` order: index 0 is nearest TDO.
        fn new(ir_lens: &[u8], index: u8, samples: &[u32]) -> Self {
            let chain: &'static mut [jtag::TapConfig] =
                Box::leak(vec![jtag::TapConfig::INIT; ir_lens.len()].into_boxed_slice());
            let mut config = jtag::Config::new(chain);
            config.device_count = ir_lens.len() as u8;
            config.index = index;
            let total: u16 = ir_lens.iter().map(|&n| u16::from(n)).sum();
            let mut before = 0u16;
            for (n, &len) in ir_lens.iter().enumerate() {
                config.scan_chain[n] = jtag::TapConfig {
                    ir_length: len,
                    ir_before: before,
                    ir_after: total - before - u16::from(len),
                };
                before += u16::from(len);
            }
            let port = MockPort {
                samples: RefCell::new(samples.to_vec()),
                ..MockPort::default()
            };
            Self {
                port,
                config,
                ack: 0,
            }
        }

        fn cells(&self) -> Vec<(bool, bool)> {
            self.port.cells.borrow().clone()
        }
    }

    /// The value a run of recorded cells carries on TDI, least significant bit first.
    fn tdi_word(cells: &[(bool, bool)]) -> u32 {
        cells
            .iter()
            .enumerate()
            .fold(0, |acc, (i, cell)| acc | ((cell.1 as u32) << i))
    }

    /// The scripted TDO for an OK ACK followed by a data word, least significant bit first.
    fn ok_ack_then(value: u32) -> Vec<u32> {
        let mut s = vec![0, 1, 0];
        s.extend((0..32).map(|bit| (value >> bit) & 1));
        s
    }

    /// Index 1 of two: the last data cell leaves Shift-DR.
    #[test]
    fn transfer_word_reads_the_tap_behind_one_bypass() {
        let value = 0xa53c_19e7;
        let mut scan = MockScan::new(&[4, 5], 1, &ok_ack_then(value));

        assert_eq!(
            scan.transfer_word(RnW::R, 0, 0, 0),
            TransferResult::Ok(value)
        );

        let cells = scan.cells();
        assert_eq!(
            cells.len(),
            41,
            "3 TMS + 1 bypass + 3 ACK + 32 data + 2 TMS"
        );
        assert_eq!(
            &cells[..3],
            &[(true, false), (false, false), (false, false)]
        );
        assert_eq!(cells[3], (false, true), "the bypass cell holds TDI high");
        assert!(cells[7..38].iter().all(|c| !c.1), "TDI low through a read");
        assert!(cells[7..38].iter().all(|c| !c.0), "and stays in Shift-DR");
        assert_eq!(
            cells[38],
            (true, false),
            "the last data cell exits Shift-DR"
        );
        assert_eq!(&cells[39..], &[(true, false), (false, false)]);
        assert_eq!(scan.port.sample.get(), 35);
    }

    /// Index 0 of two: the exit moves onto the bypass fill cell behind.
    #[test]
    fn transfer_word_writes_the_tap_in_front_of_one_bypass() {
        let data = 0x9249_6db6u32;
        let mut scan = MockScan::new(&[4, 5], 0, &[0, 1, 0]);

        assert_eq!(
            scan.transfer_word(RnW::W, 2, 0, data),
            TransferResult::Ok(0)
        );

        let cells = scan.cells();
        assert_eq!(
            cells.len(),
            41,
            "3 TMS + 3 ACK + 32 data + 1 bypass + 2 TMS"
        );
        // A[3:2] = 2 with RnW::W is the request word 0b100, least significant bit first.
        assert_eq!(tdi_word(&cells[3..6]), 0b100);
        assert_eq!(tdi_word(&cells[6..38]), data);
        assert!(
            cells[6..38].iter().all(|c| !c.0),
            "the data word never exits"
        );
        assert_eq!(
            cells[38],
            (true, true),
            "the bypass fill cell exits instead"
        );
        assert_eq!(&cells[39..], &[(true, true), (false, true)]);
        assert_eq!(scan.port.sample.get(), 3, "a write captures only the ACK");
    }

    /// A DP executes every ACK but WAIT at Update-DR, so those scans shift the whole request.
    #[test]
    fn transfer_word_completes_every_ack_but_wait() {
        let data = 0x9249_6db6u32;
        for ack in [0b100u32, 0b000, 0b111] {
            let mut scan = MockScan::new(&[4], 0, &[ack & 1, ack >> 1 & 1, ack >> 2]);
            assert_eq!(
                scan.transfer_word(RnW::W, 2, 0, data),
                TransferResult::Ok(0)
            );
            assert_eq!(scan.ack, ack);
            let cells = scan.cells();
            assert_eq!(cells.len(), 3 + 3 + 32 + 2, "ACK {ack:03b}");
            assert_eq!(tdi_word(&cells[6..38]), data);
        }
    }

    /// A clean `0b010` skips the decode until a TAP turns protocol 1; a reset TAP clears it again.
    #[test]
    fn kept_ack_decodes_only_what_the_fast_path_cannot() {
        use daplink_core::jtag_dp::{STATUS_FAULT, STATUS_NO_ACK};
        let dp: &'static JtagDp = Box::leak(Box::default());
        let kept = KeptAck::new(dp);
        let at = |index| move || index;
        assert_eq!(kept.complete(|| unreachable!(), 7), TransferResult::Ok(7));
        kept.keep(0b001);
        assert_eq!(kept.complete(at(0), 7), TransferResult::Wait);
        assert_eq!(dp.take_failure(), 0);
        kept.keep(0b000);
        assert_eq!(kept.complete(at(0), 7), TransferResult::Fault);
        assert_eq!(dp.take_failure(), STATUS_NO_ACK);
        kept.keep(0b100);
        assert_eq!(kept.complete(at(1), 7), TransferResult::Ok(7));
        assert_eq!(kept.complete(at(1), 7), TransferResult::Fault);
        assert_eq!(dp.take_failure(), STATUS_FAULT);
        assert_eq!(kept.complete(at(0), 7), TransferResult::Ok(7));
        dp.reset();
        assert_eq!(kept.complete(at(1), 7), TransferResult::Ok(7));
        assert_eq!(kept.complete(|| unreachable!(), 7), TransferResult::Ok(7));
    }

    /// A WAIT ACK drops the data phase and walks back to Run-Test/Idle without shifting a word.
    #[test]
    fn transfer_word_abandons_the_scan_on_a_wait_ack() {
        let mut scan = MockScan::new(&[4, 5], 1, &[1, 0, 0]);
        assert_eq!(scan.transfer_word(RnW::R, 0, 0, 0), TransferResult::Wait);

        let cells = scan.cells();
        assert_eq!(cells.len(), 10, "3 TMS + 1 bypass + 3 ACK + 3 TMS");
        // TDI still carries the last request bit: the abandon walk drives no fill.
        assert_eq!(&cells[7..], &[(true, false), (true, false), (false, false)]);
    }

    /// The TAP behind: the fill in front is all ones, and the instruction takes the TAP's IR width.
    #[test]
    fn shift_ir_geometry_fills_in_front_of_the_selected_tap() {
        let mut scan = MockScan::new(&[4, 5], 1, &[]);
        scan.shift_ir_geometry(0x0E, 5, 4, 0);

        let cells = scan.cells();
        assert_eq!(cells.len(), 15, "4 TMS + 4 fill + 5 IR + 2 TMS");
        assert_eq!(tdi_word(&cells[4..8]), 0b1111);
        assert_eq!(tdi_word(&cells[8..13]), 0x0E);
        assert!(!cells[11].0, "only the last IR cell exits Shift-IR");
        assert_eq!(cells[12], (true, false));
        assert_eq!(&cells[13..], &[(true, false), (false, false)]);
    }

    /// The TAP in front: the exit moves onto the last fill cell behind it.
    #[test]
    fn shift_ir_geometry_fills_behind_the_selected_tap() {
        let mut scan = MockScan::new(&[4, 5], 0, &[]);
        scan.shift_ir_geometry(0x0A, 4, 0, 5);

        let cells = scan.cells();
        assert_eq!(cells.len(), 15, "4 TMS + 4 IR + 5 fill + 2 TMS");
        assert_eq!(tdi_word(&cells[4..8]), 0x0A);
        assert!(cells[4..12].iter().all(|c| !c.0), "no early exit");
        assert!(cells[8..13].iter().all(|c| c.1), "the fill behind is ones");
        assert_eq!(cells[12], (true, true), "the last fill cell exits Shift-IR");
    }

    /// `shift_ir_word` reads the same geometry from the configured chain.
    #[test]
    fn shift_ir_word_takes_its_geometry_from_the_chain() {
        for (index, wire) in [(0u8, 0x0Au32), (1, 0x0E)] {
            let mut from_chain = MockScan::new(&[4, 5], index, &[]);
            from_chain.shift_ir_word(wire);

            let tap = from_chain.config.current_tap();
            let mut explicit = MockScan::new(&[4, 5], index, &[]);
            explicit.shift_ir_geometry(
                wire,
                u32::from(tap.ir_length),
                u32::from(tap.ir_before),
                u32::from(tap.ir_after),
            );
            assert_eq!(from_chain.cells(), explicit.cells());
        }
    }

    /// Every scan starts and ends in Run-Test/Idle, so scans compose back to back.
    #[test]
    fn scans_hand_the_tap_back_in_run_test_idle() {
        let mut scan = MockScan::new(&[4, 5], 1, &ok_ack_then(0));
        scan.port.jtag_pads_acquire();
        scan.shift_ir_word(0x0A);
        let after_ir = scan.cells().len();
        assert_eq!(scan.cells()[after_ir - 1], (false, false));

        assert!(matches!(
            scan.transfer_word(RnW::R, 0, 0, 0),
            TransferResult::Ok(_)
        ));
        let cells = scan.cells();
        assert_eq!(cells[after_ir], (true, false), "Run-Test/Idle -> Select-DR");
        assert_eq!(cells[cells.len() - 1], (false, false));

        scan.port.jtag_pads_release();
        assert_eq!(scan.port.acquired.get(), 1);
        assert_eq!(scan.port.released.get(), 1);
    }
    /// A [`JtagScanState`] recording each half's pacing: spin cycle count, or zero for a ladder half.
    struct MockState {
        port: MockPort,
        config: jtag::Config,
        paced: RefCell<Vec<u32>>,
    }

    impl JtagScanState for MockState {
        type Port = MockPort;

        fn scan_port(&self) -> &MockPort {
            &self.port
        }

        fn scan_config(&mut self) -> &jtag::Config {
            &self.config
        }

        fn scan_keep_ack(&mut self, _ack: u32) {}

        fn scan_complete(&mut self, data: u32) -> TransferResult {
            TransferResult::Ok(data)
        }

        fn scan_spin(&mut self, cycles: u32) {
            self.paced.borrow_mut().push(cycles);
        }

        fn scan_pace_low(&self) {
            self.paced.borrow_mut().push(0);
        }

        fn scan_pace_high(&self) {
            self.paced.borrow_mut().push(0);
        }
    }

    impl MockState {
        fn new(ir_lens: &[u8], index: u8, samples: &[u32]) -> Self {
            let scan = MockScan::new(ir_lens, index, samples);
            Self {
                port: scan.port,
                config: scan.config,
                paced: RefCell::new(Vec::new()),
            }
        }

        fn paced(&self) -> Vec<u32> {
            self.paced.borrow().clone()
        }
    }

    /// The nominal half-period every prepared plan below is built at.
    const NOMINAL: u32 = 5;

    /// A profile that prepares exactly `pacing` at [`NOMINAL`].
    fn phase(pacing: PacingPlan) -> PacingProfile {
        match pacing {
            PacingPlan::Fast => PacingProfile::new(NOMINAL, HalfCycleProfile::uncalibrated()),
            PacingPlan::Scaled(h) => PacingProfile::new(
                0,
                HalfCycleProfile::new(u32::MAX, u32::MAX, h.low(), h.high()),
            ),
        }
    }

    /// A plan whose four phases are set independently.
    fn plan(
        read: PacingPlan,
        write: PacingPlan,
        ir: PacingPlan,
        control: HalfCycles,
    ) -> JtagTimingPlan {
        prepare_jtag_timing(
            100,
            10,
            1,
            HalfCycleProfile::new(u32::MAX, u32::MAX, control.low(), control.high()),
            phase(read),
            phase(write),
            phase(ir),
        )
        .expect("a non-zero request prepares a plan")
    }

    /// The prepared word path is wire-identical to the per-bit one: same cells, same result.
    #[test]
    fn a_prepared_scan_shifts_the_same_cells_as_the_per_bit_path() {
        let value = 0xa53c_19e7;
        for (rnw, index) in [(RnW::R, 1), (RnW::R, 0), (RnW::W, 1), (RnW::W, 0)] {
            let mut reference = MockScan::new(&[4, 5], index, &ok_ack_then(value));
            let result = reference.transfer_word(rnw, 0, 0, value);
            let read = if rnw == RnW::R { value } else { 0 };
            assert_eq!(result, TransferResult::Ok(read));

            for pacing in [PacingPlan::Fast, PacingPlan::Scaled(HalfCycles::new(3, 2))] {
                let mut state = MockState::new(&[4, 5], index, &ok_ack_then(value));
                let p = plan(pacing, pacing, PacingPlan::Fast, HalfCycles::new(7, 5));
                assert_eq!(scan_transfer_word(&mut state, p, rnw, 0, 0, value), result);
                assert_eq!(
                    state.port.cells.borrow().clone(),
                    reference.cells(),
                    "{pacing:?} {rnw:?} TAP {index} changed the wire form"
                );
            }
        }
    }

    /// The plan is looked up per direction: a read takes the read phase, a write the write one.
    #[test]
    fn the_transfer_direction_selects_its_prepared_phase() {
        let p = plan(
            PacingPlan::Scaled(HalfCycles::new(3, 2)),
            PacingPlan::Fast,
            PacingPlan::Fast,
            HalfCycles::new(7, 5),
        );

        let mut read = MockState::new(&[4], 0, &ok_ack_then(0));
        assert!(matches!(
            scan_transfer_word(&mut read, p, RnW::R, 0, 0, 0),
            TransferResult::Ok(_)
        ));
        // Three TAP walks in, then the ACK and data cells; only the walks take control pacing.
        let paced = read.paced();
        assert_eq!(&paced[..6], &[7, 5, 7, 5, 7, 5]);
        assert_eq!(
            &paced[6..76],
            &[3, 2].repeat(35)[..],
            "3 ACK + 32 data cells"
        );
        assert_eq!(
            &paced[76..],
            &[7, 5, 7, 5],
            "the exit walk is control pacing again"
        );

        let mut write = MockState::new(&[4], 0, &[0, 1, 0]);
        assert!(matches!(
            scan_transfer_word(&mut write, p, RnW::W, 0, 0, 0),
            TransferResult::Ok(_)
        ));
        assert!(
            write.paced().iter().all(|&c| c == 0),
            "the write phase is the ladder, which spins for nothing"
        );
    }

    /// The IR scan takes its own prepared phase, independent of both transfer directions.
    #[test]
    fn the_ir_scan_takes_its_own_prepared_phase() {
        let p = plan(
            PacingPlan::Fast,
            PacingPlan::Fast,
            PacingPlan::Scaled(HalfCycles::new(9, 4)),
            HalfCycles::new(7, 5),
        );

        let mut state = MockState::new(&[4, 5], 1, &[]);
        scan_shift_ir_word(&mut state, p, 0x0E);

        let mut reference = MockScan::new(&[4, 5], 1, &[]);
        reference.shift_ir_word(0x0E);
        assert_eq!(state.port.cells.borrow().clone(), reference.cells());

        let paced = state.paced();
        // Four TAP walks plus four bypass fill cells at the control period, then the IR cells.
        assert_eq!(
            &paced[..16],
            &[7, 5].repeat(8)[..],
            "4 TAP walks + 4 bypass fill cells"
        );
        assert_eq!(&paced[16..26], &[9, 4].repeat(5)[..], "5 IR cells");
        assert_eq!(
            &paced[26..],
            &[7, 5, 7, 5],
            "the exit walk is control pacing again"
        );
    }
}
