//! Late-sample bit-bang JTAG: TDO sampled after a full bit period, levels set before drive enable.

use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin, SwdAdapter};
use dap_rs::jtag::{self, Jtag, SequenceInfo};
use dap_rs::swd::{self, APnDP, DPRegister, Swd};
use dap_rs::swj::{Dependencies, Pins};

use dap_rs::dap::TransferConfig;
use dap_rs::jtag::TransferResult;
use dap_rs::swd::RnW;
use daplink_core::{JtagDp, JtagDpSource};

use crate::jtag_transfer::{chunked_scan, jtag_result};
use crate::wired::{
    no_ack, park, read_retry, wired_deps, write_retry, ScanChainStorage, SwjPin, WiredAdapter,
};

/// Handles to the four JTAG-only pads: TCK/TMS share SWCLK/SWDIO's pads.
pub(crate) struct JtagLines<IO> {
    tck: IO,
    tms: IO,
    tdi: IO,
    tdo: IO,
}

impl<IO> JtagLines<IO> {
    /// Bundle the four line handles.
    pub(crate) fn new(tck: IO, tms: IO, tdi: IO, tdo: IO) -> Self {
        Self { tck, tms, tdi, tdo }
    }
}

/// The JTAG bit-period timing, carried across every mode switch.
struct Timing<D> {
    delay: D,
    /// Last resolved half-period in core cycles (0 until a clock has been requested).
    half_cycles: u32,
}

impl<D: DelayCycles> Timing<D> {
    /// Resolve the half-period from a clock request, matching the inner adapter's own derivation.
    fn set_clock(&mut self, max_frequency: u32) {
        if max_frequency > 0 {
            self.half_cycles = (self.delay.cpu_clock() / 2).div_ceil(max_frequency).max(1);
        }
    }

    #[inline(always)]
    fn wait(&mut self) {
        self.delay.delay_cycles(self.half_cycles);
    }
}

/// Shift up to 8 bits with a fixed TMS level, TDI from `tdi` LSB-first; returns captured TDO.
fn shift<IO: InputOutputPin, D: DelayCycles>(
    lines: &mut JtagLines<IO>,
    timing: &mut Timing<D>,
    tms: bool,
    tdi: u32,
    num_bits: usize,
) -> u32 {
    lines.tms.set_high(tms);
    lines.tms.set_as_output();
    let mut tdo = 0;
    for i in 0..num_bits {
        lines.tdi.set_high(tdi & (1 << i) != 0);
        lines.tck.set_high(false);
        timing.wait();
        lines.tck.set_high(true);
        timing.wait();
        tdo |= (lines.tdo.is_high() as u32) << i;
    }
    tdo
}

/// `DAP_JTAG_Sequence` body: shift `info.n_bits` bits of `tdi`, capturing into `rxbuf` if set.
fn sequence<IO: InputOutputPin, D: DelayCycles>(
    lines: &mut JtagLines<IO>,
    timing: &mut Timing<D>,
    info: SequenceInfo,
    tdi: &[u8],
    mut rxbuf: &mut [u8],
) {
    let mut n_bits = info.n_bits as usize;
    for &byte in tdi {
        if n_bits == 0 {
            break;
        }
        let bits = n_bits.min(8);
        n_bits -= bits;
        let tdo = shift(lines, timing, info.tms, byte as u32, bits);
        if info.capture && !rxbuf.is_empty() {
            rxbuf[0] = tdo as u8;
            rxbuf = &mut rxbuf[1..];
        }
    }
}

/// TMS-run body: TDI held high, no TDO capture.
fn tms_sequence<IO: InputOutputPin, D: DelayCycles>(
    lines: &mut JtagLines<IO>,
    timing: &mut Timing<D>,
    tms: &[bool],
) {
    lines.tdi.set_high(true);
    lines.tdi.set_as_output();
    lines.tms.set_as_output();
    for &bit in tms {
        lines.tms.set_high(bit);
        lines.tck.set_high(false);
        timing.wait();
        lines.tck.set_high(true);
        timing.wait();
    }
}

/// The late-sample bit cell and its four line handles, shareable with an accelerated SWD backend.
pub(crate) struct LateJtagShifter<IO, D> {
    lines: JtagLines<IO>,
    timing: Timing<D>,
}

impl<IO: InputOutputPin, D: DelayCycles> LateJtagShifter<IO, D> {
    /// Build the shifter from the line handles and the firmware's timing source.
    pub fn new(lines: JtagLines<IO>, delay: D) -> Self {
        Self {
            lines,
            timing: Timing {
                delay,
                half_cycles: 0,
            },
        }
    }

    /// Record a clock request and resolve the JTAG half-period.
    pub fn set_clock(&mut self, max_frequency: u32) {
        self.timing.set_clock(max_frequency);
    }

    /// Drive the three outputs and release TDO; levels are whatever was staged last.
    pub fn stage_jtag_dirs(&mut self) {
        self.lines.tck.set_as_output();
        self.lines.tms.set_as_output();
        self.lines.tdi.set_as_output();
        self.lines.tdo.set_as_input();
    }

    /// `DAP_JTAG_Sequence` body over the late-sample bit cell.
    pub fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        sequence(&mut self.lines, &mut self.timing, info, tdi, rxbuf)
    }

    /// TMS-run body: TDI held high, no TDO capture.
    pub fn tms_sequence(&mut self, tms: &[bool]) {
        tms_sequence(&mut self.lines, &mut self.timing, tms)
    }

    #[inline(always)]
    fn clock(&mut self) {
        self.lines.tck.set_high(false);
        self.timing.wait();
        self.lines.tck.set_high(true);
        self.timing.wait();
    }

    /// Clock `cycles` SWD idle cells with SWDIO driven low.
    #[inline(never)]
    fn idle(&mut self, cycles: u8) {
        self.lines.tms.set_high(false);
        self.lines.tms.set_as_output();
        for _ in 0..cycles {
            self.clock();
        }
    }
}

/// SWJ dependencies with the late-sample JTAG shifter composed in over an inner [`WiredAdapter`].
pub struct LateJtagDeps<IO: InputOutputPin + Clone, D: DelayCycles + Clone> {
    inner: WiredAdapter<IO, D>,
    shifter: LateJtagShifter<IO, D>,
    dp: &'static JtagDp,
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> LateJtagDeps<IO, D> {
    /// Compose the shifter over an adapter; `lines` must be a second handle to its four JTAG pads.
    pub(crate) fn new(
        inner: WiredAdapter<IO, D>,
        lines: JtagLines<IO>,
        delay: D,
        dp: &'static JtagDp,
    ) -> Self {
        Self {
            inner,
            shifter: LateJtagShifter::new(lines, delay),
            dp,
        }
    }

    /// Wire the dependencies from pins: the bit-bang adapter plus a duplicated JTAG-pad handle.
    #[allow(clippy::too_many_arguments)]
    pub fn wired<const N: usize>(
        nreset: IO,
        tdi: IO,
        swdio: IO,
        swclk: IO,
        tdo: IO,
        delay: D,
        scan: &'static ScanChainStorage<N>,
    ) -> Self {
        let lines = JtagLines::new(swclk.clone(), swdio.clone(), tdi.clone(), tdo.clone());
        let inner = wired_deps(nreset, tdi, swdio, swclk, tdo, delay.clone(), scan);
        Self::new(inner, lines, delay, scan.jtag_dp())
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> JtagDpSource for LateJtagDeps<IO, D> {
    fn jtag_dp(&self) -> Option<&'static JtagDp> {
        Some(self.dp)
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<LateSwdDrv<IO, D>>
    for LateJtagDeps<IO, D>
{
    fn from(drv: LateSwdDrv<IO, D>) -> Self {
        Self {
            inner: drv.inner.into(),
            shifter: drv.shifter,
            dp: drv.dp,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<LateJtag<IO, D>>
    for LateJtagDeps<IO, D>
{
    fn from(drv: LateJtag<IO, D>) -> Self {
        Self {
            inner: drv.inner,
            shifter: drv.shifter,
            dp: drv.dp,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone>
    Dependencies<LateSwdDrv<IO, D>, LateJtag<IO, D>> for LateJtagDeps<IO, D>
{
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.inner.process_swj_pins(output, mask, wait_us)
    }
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.inner.process_swj_sequence(data, nbits)
    }
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        // Recorded here so a JTAG mode switch shifts at this resolution even if unrepeated.
        self.shifter.set_clock(max_frequency);
        self.inner.process_swj_clock(max_frequency)
    }
    fn high_impedance_mode(&mut self) {
        park(&mut self.inner)
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        self.inner.swd_config()
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.inner.jtag_config()
    }
}

/// Active SWD driver: the inner [`SwdAdapter`], carrying the JTAG lines across the switch.
pub struct LateSwdDrv<IO: InputOutputPin + Clone, D: DelayCycles + Clone> {
    inner: SwdAdapter<SwjPin<IO>, D>,
    shifter: LateJtagShifter<IO, D>,
    dp: &'static JtagDp,
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<LateJtagDeps<IO, D>>
    for LateSwdDrv<IO, D>
{
    fn from(deps: LateJtagDeps<IO, D>) -> Self {
        Self {
            inner: deps.inner.into(),
            shifter: deps.shifter,
            dp: deps.dp,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> LateSwdDrv<IO, D> {
    /// What an OK read ACK still owes: the turnaround dap-rs skips on a bad parity, then idle.
    #[inline(always)]
    fn settle(&mut self, r: &swd::Result<u32>) {
        match r {
            Ok(_) => {}
            Err(swd::Error::BadParity) => self.shifter.clock(),
            Err(_) => return,
        }
        self.idle();
    }

    /// The host's idle cycles, after an OK ACK.
    #[inline(always)]
    fn idle(&mut self) {
        let cycles = self.dp.swd_idle();
        if cycles != 0 {
            self.shifter.idle(cycles);
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> Swd<LateJtagDeps<IO, D>>
    for LateSwdDrv<IO, D>
{
    fn available(_deps: &LateJtagDeps<IO, D>) -> bool {
        true
    }
    fn config(&mut self) -> &mut swd::Config {
        self.inner.config()
    }
    fn read(&mut self, wait_retries: usize, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        let r = read_retry(&mut self.inner, wait_retries, port, addr);
        self.settle(&r);
        r
    }
    fn read_inner(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        let r = self.inner.read_inner(port, addr);
        self.settle(&r);
        r
    }
    fn write(
        &mut self,
        wait_retries: usize,
        port: APnDP,
        addr: DPRegister,
        data: u32,
    ) -> swd::Result<()> {
        let r = write_retry(&mut self.inner, wait_retries, port, addr, data).map_err(no_ack);
        if r.is_ok() {
            self.idle();
        }
        r
    }
    fn write_inner(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        let r = self.inner.write_inner(port, addr, data).map_err(no_ack);
        if r.is_ok() {
            self.idle();
        }
        r
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.shifter.set_clock(max_frequency);
        self.inner.set_clock(max_frequency)
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.inner.write_sequence(num_bits, data)
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.inner.read_sequence(num_bits, data)
    }
}

/// Active JTAG driver: shifts with the late-sample cell; configuration stays with the inner adapter.
pub struct LateJtag<IO: InputOutputPin + Clone, D: DelayCycles + Clone> {
    inner: WiredAdapter<IO, D>,
    shifter: LateJtagShifter<IO, D>,
    dp: &'static JtagDp,
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> LateJtag<IO, D> {
    /// The JTAG-DP state this driver's transfers decode through.
    pub fn jtag_dp(&self) -> &'static JtagDp {
        self.dp
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<LateJtagDeps<IO, D>>
    for LateJtag<IO, D>
{
    fn from(mut deps: LateJtagDeps<IO, D>) -> Self {
        // Pad directions for the shifter's own handles only; SWD entry reasserts its own.
        deps.shifter.stage_jtag_dirs();
        Self {
            inner: deps.inner,
            shifter: deps.shifter,
            dp: deps.dp,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> Jtag<LateJtagDeps<IO, D>>
    for LateJtag<IO, D>
{
    fn available(_deps: &LateJtagDeps<IO, D>) -> bool {
        true
    }

    fn config(&mut self) -> &mut jtag::Config {
        self.inner.jtag_config()
    }

    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        self.shifter.sequence(info, tdi, rxbuf)
    }

    fn tms_sequence(&mut self, tms: &[bool]) {
        self.shifter.tms_sequence(tms)
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.shifter.set_clock(max_frequency);
        self.inner.process_swj_clock(max_frequency)
    }

    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let (ack, data) = chunked_scan(self, r_nw, a2a3, transfer_config.idle_cycles, data);
        let index = self.config().index;
        jtag_result(self.dp, index, ack, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use self::std::boxed::Box;
    use self::std::cell::RefCell;
    use self::std::rc::Rc;
    use self::std::vec;
    use self::std::vec::Vec;

    #[derive(Clone)]
    struct MockPin {
        as_output: bool,
        level: bool,
    }
    impl MockPin {
        const fn new() -> Self {
            Self {
                as_output: false,
                level: false,
            }
        }
    }
    impl InputOutputPin for MockPin {
        fn set_as_output(&mut self) {
            self.as_output = true;
        }
        fn set_high(&mut self, high: bool) {
            self.level = high;
        }
        fn set_as_input(&mut self) {
            self.as_output = false;
        }
        fn is_high(&mut self) -> bool {
            self.level
        }
    }

    #[derive(Clone)]
    struct MockDelay;
    impl dap_rs::dap::DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }
    impl DelayCycles for MockDelay {
        fn cpu_clock(&self) -> u32 {
            1_000_000
        }
        fn delay_cycles(&mut self, _cycles: u32) {}
    }

    fn lines() -> JtagLines<MockPin> {
        JtagLines::new(
            MockPin::new(),
            MockPin::new(),
            MockPin::new(),
            MockPin::new(),
        )
    }

    #[derive(Clone, Copy)]
    enum Wire {
        Tck,
        Tms,
        Tdi,
        Tdo,
    }

    #[derive(Default)]
    struct Trace {
        tck: bool,
        tms: bool,
        tdi: bool,
        rises: Vec<(bool, bool)>,
        samples: Vec<bool>,
        sample: usize,
        sample_levels: Vec<bool>,
    }

    #[derive(Clone)]
    struct TracePin {
        wire: Wire,
        trace: Rc<RefCell<Trace>>,
    }

    impl TracePin {
        fn new(wire: Wire, trace: &Rc<RefCell<Trace>>) -> Self {
            Self {
                wire,
                trace: trace.clone(),
            }
        }
    }

    impl InputOutputPin for TracePin {
        fn set_as_output(&mut self) {}

        fn set_high(&mut self, high: bool) {
            let mut trace = self.trace.borrow_mut();
            match self.wire {
                Wire::Tck => {
                    trace.tck = high;
                    if high {
                        let cell = (trace.tms, trace.tdi);
                        trace.rises.push(cell);
                    }
                }
                Wire::Tms => trace.tms = high,
                Wire::Tdi => trace.tdi = high,
                _ => {}
            }
        }

        fn set_as_input(&mut self) {}

        fn is_high(&mut self) -> bool {
            let mut trace = self.trace.borrow_mut();
            let tck = trace.tck;
            trace.sample_levels.push(tck);
            let value = trace.samples[trace.sample];
            trace.sample += 1;
            value
        }
    }

    fn traced_shifter(
        samples: &[bool],
    ) -> (LateJtagShifter<TracePin, MockDelay>, Rc<RefCell<Trace>>) {
        let trace = Rc::new(RefCell::new(Trace {
            samples: samples.to_vec(),
            ..Trace::default()
        }));
        let lines = JtagLines::new(
            TracePin::new(Wire::Tck, &trace),
            TracePin::new(Wire::Tms, &trace),
            TracePin::new(Wire::Tdi, &trace),
            TracePin::new(Wire::Tdo, &trace),
        );
        (LateJtagShifter::new(lines, MockDelay), trace)
    }

    #[test]
    fn zero_bit_sequence_does_not_shift() {
        let mut lines = lines();
        let mut timing = Timing {
            delay: MockDelay,
            half_cycles: 1,
        };
        let info = SequenceInfo {
            n_bits: 0,
            capture: true,
            tms: false,
        };
        let mut rxbuf = [0xAAu8];
        sequence(&mut lines, &mut timing, info, &[0xFF], &mut rxbuf);
        // Untouched: the zero-bit guard breaks out before the first shift.
        assert_eq!(rxbuf, [0xAA]);
    }

    /// A `Jtag` over the traced shifter, so dap-rs's own scan choreography drives the recorded wire.
    struct TracedJtag {
        shifter: LateJtagShifter<TracePin, MockDelay>,
        config: jtag::Config,
    }

    impl Jtag<TracedJtag> for TracedJtag {
        fn available(_deps: &TracedJtag) -> bool {
            true
        }

        fn config(&mut self) -> &mut jtag::Config {
            &mut self.config
        }

        fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
            self.shifter.sequence(info, tdi, rxbuf)
        }

        fn tms_sequence(&mut self, tms: &[bool]) {
            self.shifter.tms_sequence(tms)
        }

        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
    }

    /// Build a traced chain from IR lengths in `DAP_JTAG_Configure` order, index 0 nearest TDO.
    fn traced_chain(
        ir_lens: &[u8],
        index: u8,
        samples: &[bool],
    ) -> (TracedJtag, Rc<RefCell<Trace>>) {
        let trace = Rc::new(RefCell::new(Trace {
            samples: samples.to_vec(),
            ..Trace::default()
        }));
        let lines = JtagLines::new(
            TracePin::new(Wire::Tck, &trace),
            TracePin::new(Wire::Tms, &trace),
            TracePin::new(Wire::Tdi, &trace),
            TracePin::new(Wire::Tdo, &trace),
        );
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
        let shifter = LateJtagShifter::new(lines, MockDelay);
        (TracedJtag { shifter, config }, trace)
    }

    /// The value a run of recorded cells carries on TDI, least significant bit first.
    fn tdi_word(cells: &[(bool, bool)]) -> u32 {
        cells
            .iter()
            .enumerate()
            .fold(0, |acc, (i, cell)| acc | ((cell.1 as u32) << i))
    }

    /// The per-chunk scan clocks dap-rs's cells on OK and WAIT, and completes `0b100` and no ACK.
    #[test]
    fn the_chunked_scan_matches_dap_rs_but_leaves_early_only_on_wait() {
        use crate::jtag_transfer::chunked_scan;
        let config = TransferConfig {
            idle_cycles: 2,
            wait_retries: 0,
            match_retries: 0,
            match_mask: 0,
        };
        for (lens, index) in [(&[4][..], 0), (&[4, 5][..], 0), (&[5, 4, 7][..], 1)] {
            for ack in [0b010u32, 0b001, 0b100, 0b111] {
                for r_nw in [RnW::R, RnW::W] {
                    // Every cell samples TDO: the bypass fill in front, the ACK, then the data.
                    let mut samples = vec![false; usize::from(index)];
                    samples.extend((0..3).map(|i| ack >> i & 1 != 0));
                    samples.extend((0..32).map(|i| 0x5a5a_c3c3u32 >> i & 1 != 0));
                    samples.extend([false; 16]);
                    let (mut ours, ours_trace) = traced_chain(lens, index, &samples);
                    let (mut theirs, theirs_trace) = traced_chain(lens, index, &samples);
                    let (_, data) = chunked_scan(&mut ours, r_nw, 3, 2, 0x1234_5678);
                    let result = theirs.transfer(r_nw, 3, &config, 0x1234_5678);
                    let (ours, theirs) = (ours_trace.borrow(), theirs_trace.borrow());
                    if matches!(ack, 0b010 | 0b001) {
                        assert_eq!(ours.rises, theirs.rises, "ACK {ack:03b}");
                        let expect = match result {
                            TransferResult::Ok(v) => v,
                            _ => 0,
                        };
                        assert_eq!(data, expect);
                    } else {
                        let data_cells = 32 + usize::from(lens.len() as u8 - index - 1) + 2;
                        assert_eq!(ours.rises.len(), theirs.rises.len() - 3 + data_cells);
                    }
                }
            }
        }
    }

    /// `DAP_JTAG_IDCODE` shifts 0x0E whatever the IR length; a five-bit STM32 TAP decodes it as BYPASS.
    #[test]
    fn the_idcode_opcode_selects_bypass_on_a_five_bit_tap() {
        // Four-bit ARM JTAG-DP nearest TDO, five-bit boundary-scan TAP behind it.
        let (mut jtag, trace) = traced_chain(&[4, 5], 1, &[false; 42]);

        jtag.shift_ir(0x0E);
        let ir_cells = trace.borrow().rises.len();
        assert_eq!(ir_cells, 15, "4 TMS + 4 bypass + 5 IR + 2 TMS");
        {
            let t = trace.borrow();
            // The TAPs in front of the selected one take all ones, which is BYPASS.
            assert_eq!(tdi_word(&t.rises[4..8]), 0b1111);
            // The selected five-bit TAP takes 0x0E, not its own 0x1E IDCODE opcode.
            assert_eq!(tdi_word(&t.rises[8..13]), 0x0E);
            assert!(!t.rises[11].0, "only the last IR cell exits Shift-IR");
            assert!(t.rises[12].0);
        }

        // Both TAPs now sit in BYPASS, so the data register is two bits and reads back as zero.
        assert_eq!(jtag.shift_dr(0), 0);

        let t = trace.borrow();
        assert_eq!(
            t.rises.len(),
            ir_cells + 38,
            "3 TMS + 1 bypass + 32 + 2 TMS"
        );
        assert_eq!(t.sample, 42, "TDO is sampled on every shifted cell");
        let dr = &t.rises[ir_cells..];
        // The one-bit capturing prologue for the TAP in front, TDI high.
        assert_eq!(dr[3], (false, true));
        assert!(dr[4..36].iter().all(|cell| !cell.1), "TDI low through DR");
        assert!(dr[4..35].iter().all(|cell| !cell.0), "stays in Shift-DR");
        assert!(dr[35].0, "the last data cell exits Shift-DR");
    }

    /// A partial-byte capture must zero-extend, or a stray high bit turns an OK ACK into Wait.
    #[test]
    fn a_partial_byte_capture_zero_extends_through_the_ack() {
        let value = 0xa53c_19e7u32;
        // One bypass cell, the OK ACK 0b010 least significant bit first, then the data word.
        let mut samples = vec![false, false, true, false];
        samples.extend((0..32).map(|bit| value & (1 << bit) != 0));
        let (mut jtag, trace) = traced_chain(&[4, 5], 1, &samples);

        let config = dap_rs::dap::TransferConfig {
            idle_cycles: 0,
            wait_retries: 0,
            match_retries: 0,
            match_mask: 0,
        };
        assert_eq!(
            jtag.transfer(RnW::R, 0, &config, 0),
            TransferResult::Ok(value)
        );
        assert_eq!(trace.borrow().sample, 36);
    }

    #[test]
    fn sequence_samples_tdo_after_the_rising_edge() {
        let (mut shifter, trace) = traced_shifter(&[true, false, true]);
        let info = SequenceInfo {
            n_bits: 3,
            capture: true,
            tms: false,
        };
        let mut rxbuf = [0u8];
        shifter.sequence(info, &[0b101], &mut rxbuf);
        assert_eq!(rxbuf[0], 0b101);
        assert_eq!(trace.borrow().rises.len(), 3);
        assert_eq!(trace.borrow().sample_levels, [true, true, true]);
    }
}
