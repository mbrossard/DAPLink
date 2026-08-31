//! Late-sample bit-bang JTAG: TDO sampled after a full bit period, levels set before drive enable.

use dap_rs::driver::bitbang::{BitbangAdapter, DelayCycles, InputOutputPin, SwdAdapter};
use dap_rs::jtag::{self, Jtag, SequenceInfo};
use dap_rs::swd::{self, APnDP, DPRegister, Swd};
use dap_rs::swj::{Dependencies, Pins};

#[cfg(feature = "rp2040-jtag-accel")]
use dap_rs::jtag::TransferResult;
#[cfg(feature = "rp2040-jtag-accel")]
use dap_rs::swd::RnW;

use crate::wired::{wired_deps, ScanChainStorage};

/// Handles to the four JTAG-only pads: TCK/TMS share SWCLK/SWDIO's pads.
pub struct JtagLines<IO> {
    tck: IO,
    tms: IO,
    tdi: IO,
    tdo: IO,
}

impl<IO> JtagLines<IO> {
    /// Bundle the four line handles.
    pub fn new(tck: IO, tms: IO, tdi: IO, tdo: IO) -> Self {
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

/// The late-sample bit cell and the four line handles it drives, without any SWD backend.
/// Split out so an accelerated SWD backend can borrow the same shifter for its JTAG role.
pub struct LateJtagShifter<IO, D> {
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

    /// Stage the JTAG idle state before a pad handover, so no stale TCK level reaches the wire.
    pub fn lines_to_jtag(&mut self) {
        self.lines.tck.set_high(false);
        self.stage_jtag_dirs();
    }

    /// Release all four lines; the counterpart of [`lines_to_jtag`](Self::lines_to_jtag).
    pub fn release_lines(&mut self) {
        self.lines.tck.set_as_input();
        self.lines.tms.set_as_input();
        self.lines.tdi.set_as_input();
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

    #[cfg(feature = "rp2040-jtag-accel")]
    #[inline(always)]
    fn clock(&mut self) {
        self.lines.tck.set_high(false);
        self.timing.wait();
        self.lines.tck.set_high(true);
        self.timing.wait();
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    #[inline(always)]
    fn clock_in(&mut self) -> u32 {
        self.clock();
        self.lines.tdo.is_high() as u32
    }

    /// Execute one complete DPACC/APACC scan without decomposing its data word into byte calls.
    #[cfg(feature = "rp2040-jtag-accel")]
    pub fn transfer_word(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        idle_cycles: u8,
        data: u32,
        before: u32,
        after: u32,
    ) -> TransferResult {
        macro_rules! read_cells {
            ($value:ident; $($bit:literal),+ $(,)?) => {
                $($value |= self.clock_in() << $bit;)+
            };
        }
        macro_rules! write_cells {
            ($value:ident; $($bit:literal),+ $(,)?) => {
                $(
                    self.lines.tdi.set_high(($value >> $bit) & 1 != 0);
                    self.clock();
                )+
            };
        }

        // Run-Test/Idle -> Shift-DR: TMS 1,0,0.
        self.lines.tms.set_high(true);
        self.clock();
        self.lines.tms.set_high(false);
        self.clock();
        self.clock();

        self.lines.tdi.set_high(true);
        for _ in 0..before {
            self.clock();
        }

        let req = ((a2a3 as u32) << 1) | r_nw as u32;
        let mut ack = 0u32;
        write_cells!(req; 0);
        ack |= self.lines.tdo.is_high() as u32;
        write_cells!(req; 1);
        ack |= (self.lines.tdo.is_high() as u32) << 1;
        write_cells!(req; 2);
        ack |= (self.lines.tdo.is_high() as u32) << 2;

        let result = if ack == 0x02 {
            let captured = if r_nw == RnW::R {
                self.lines.tdi.set_high(false);
                let mut value = 0u32;
                read_cells!(value;
                    0, 1, 2, 3, 4, 5, 6, 7,
                    8, 9, 10, 11, 12, 13, 14, 15,
                    16, 17, 18, 19, 20, 21, 22, 23,
                    24, 25, 26, 27, 28, 29, 30
                );
                if after == 0 {
                    self.lines.tms.set_high(true);
                }
                value | (self.clock_in() << 31)
            } else {
                write_cells!(data;
                    0, 1, 2, 3, 4, 5, 6, 7,
                    8, 9, 10, 11, 12, 13, 14, 15,
                    16, 17, 18, 19, 20, 21, 22, 23,
                    24, 25, 26, 27, 28, 29, 30
                );
                if after == 0 {
                    self.lines.tms.set_high(true);
                }
                self.lines.tdi.set_high(data >> 31 != 0);
                self.clock();
                0
            };

            if after > 0 {
                self.lines.tdi.set_high(true);
                for _ in 0..after - 1 {
                    self.clock();
                }
                self.lines.tms.set_high(true);
                self.clock();
            }

            // Exit1-DR -> Update-DR -> Run-Test/Idle.
            self.lines.tms.set_high(true);
            self.clock();
            self.lines.tms.set_high(false);
            self.clock();
            TransferResult::Ok(captured)
        } else {
            // Shift-DR -> Exit1-DR -> Update-DR -> Run-Test/Idle.
            self.lines.tms.set_high(true);
            self.clock();
            self.clock();
            self.lines.tms.set_high(false);
            self.clock();
            TransferResult::Wait
        };

        self.lines.tdi.set_high(true);
        for _ in 0..idle_cycles {
            self.clock();
        }
        result
    }
}

/// SWJ dependencies with the late-sample JTAG shifter composed in over an inner [`BitbangAdapter`].
pub struct Deps<IO: InputOutputPin + Clone, D: DelayCycles + Clone> {
    inner: BitbangAdapter<IO, D>,
    shifter: LateJtagShifter<IO, D>,
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> Deps<IO, D> {
    /// Compose the shifter over an adapter; `lines` must be a second handle to its four JTAG pads.
    pub fn new(inner: BitbangAdapter<IO, D>, lines: JtagLines<IO>, delay: D) -> Self {
        Self {
            inner,
            shifter: LateJtagShifter::new(lines, delay),
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
        Self::new(inner, lines, delay)
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<SwdDrv<IO, D>> for Deps<IO, D> {
    fn from(drv: SwdDrv<IO, D>) -> Self {
        Self {
            inner: drv.inner.into(),
            shifter: drv.shifter,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<LateJtag<IO, D>> for Deps<IO, D> {
    fn from(drv: LateJtag<IO, D>) -> Self {
        Self {
            inner: drv.inner,
            shifter: drv.shifter,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone>
    Dependencies<SwdDrv<IO, D>, LateJtag<IO, D>> for Deps<IO, D>
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
        self.inner.high_impedance_mode()
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        self.inner.swd_config()
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.inner.jtag_config()
    }
}

/// Active SWD driver: the inner [`SwdAdapter`] verbatim, carrying the JTAG lines across the switch.
pub struct SwdDrv<IO: InputOutputPin + Clone, D: DelayCycles + Clone> {
    inner: SwdAdapter<IO, D>,
    shifter: LateJtagShifter<IO, D>,
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<Deps<IO, D>> for SwdDrv<IO, D> {
    fn from(deps: Deps<IO, D>) -> Self {
        Self {
            inner: deps.inner.into(),
            shifter: deps.shifter,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> Swd<Deps<IO, D>> for SwdDrv<IO, D> {
    fn available(_deps: &Deps<IO, D>) -> bool {
        true
    }
    fn config(&mut self) -> &mut swd::Config {
        self.inner.config()
    }
    fn read_inner(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        self.inner.read_inner(port, addr)
    }
    fn write_inner(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        self.inner.write_inner(port, addr, data)
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

/// Active JTAG driver: shifting runs locally against the duplicated [`JtagLines`] with the
/// late-sample bit cell; configuration and the mode-switch pins stay with the inner adapter.
pub struct LateJtag<IO: InputOutputPin + Clone, D: DelayCycles + Clone> {
    inner: BitbangAdapter<IO, D>,
    shifter: LateJtagShifter<IO, D>,
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> From<Deps<IO, D>> for LateJtag<IO, D> {
    fn from(mut deps: Deps<IO, D>) -> Self {
        // Pad directions for the shifter's own handles only; SWD entry reasserts its own.
        deps.shifter.stage_jtag_dirs();
        Self {
            inner: deps.inner,
            shifter: deps.shifter,
        }
    }
}

impl<IO: InputOutputPin + Clone, D: DelayCycles + Clone> Jtag<Deps<IO, D>> for LateJtag<IO, D> {
    fn available(_deps: &Deps<IO, D>) -> bool {
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
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "rp2040-jtag-accel")]
    extern crate std;
    #[cfg(feature = "rp2040-jtag-accel")]
    use self::std::cell::RefCell;
    #[cfg(feature = "rp2040-jtag-accel")]
    use self::std::rc::Rc;
    #[cfg(feature = "rp2040-jtag-accel")]
    use self::std::vec;
    #[cfg(feature = "rp2040-jtag-accel")]
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

    #[cfg(feature = "rp2040-jtag-accel")]
    #[derive(Clone, Copy)]
    enum Wire {
        Tck,
        Tms,
        Tdi,
        Tdo,
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    #[derive(Default)]
    struct Trace {
        tms: bool,
        tdi: bool,
        rises: Vec<(bool, bool)>,
        samples: Vec<bool>,
        sample: usize,
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    #[derive(Clone)]
    struct TracePin {
        wire: Wire,
        trace: Rc<RefCell<Trace>>,
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    impl TracePin {
        fn new(wire: Wire, trace: &Rc<RefCell<Trace>>) -> Self {
            Self {
                wire,
                trace: trace.clone(),
            }
        }
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    impl InputOutputPin for TracePin {
        fn set_as_output(&mut self) {}

        fn set_high(&mut self, high: bool) {
            let mut trace = self.trace.borrow_mut();
            match self.wire {
                Wire::Tck if high => {
                    let cell = (trace.tms, trace.tdi);
                    trace.rises.push(cell);
                }
                Wire::Tms => trace.tms = high,
                Wire::Tdi => trace.tdi = high,
                _ => {}
            }
        }

        fn set_as_input(&mut self) {}

        fn is_high(&mut self) -> bool {
            let mut trace = self.trace.borrow_mut();
            let value = trace.samples[trace.sample];
            trace.sample += 1;
            value
        }
    }

    #[cfg(feature = "rp2040-jtag-accel")]
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

    #[cfg(feature = "rp2040-jtag-accel")]
    #[test]
    fn transfer_word_reads_one_device_and_frames_the_tap() {
        let value = 0xa53c_19e7u32;
        let mut samples = vec![false, true, false];
        samples.extend((0..32).map(|bit| value & (1 << bit) != 0));
        let (mut shifter, trace) = traced_shifter(&samples);

        assert_eq!(
            shifter.transfer_word(RnW::R, 2, 2, 0, 0, 0),
            TransferResult::Ok(value)
        );

        let trace = trace.borrow();
        assert_eq!(trace.sample, 35);
        assert_eq!(trace.rises.len(), 42);
        assert_eq!(
            &trace.rises[..3],
            &[(true, false), (false, false), (false, false)]
        );
        assert_eq!(
            &trace.rises[3..6],
            &[(false, true), (false, false), (false, true)]
        );
        assert!(trace.rises[6..37]
            .iter()
            .all(|cell| *cell == (false, false)));
        assert_eq!(trace.rises[37], (true, false));
        assert_eq!(&trace.rises[38..40], &[(true, false), (false, false)]);
        assert_eq!(&trace.rises[40..], &[(false, true), (false, true)]);
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    #[test]
    fn transfer_word_writes_through_a_multi_device_chain() {
        let data = 0x9249_6db6u32;
        let (mut shifter, trace) = traced_shifter(&[false, true, false]);

        assert_eq!(
            shifter.transfer_word(RnW::W, 1, 0, data, 1, 2),
            TransferResult::Ok(0)
        );

        let trace = trace.borrow();
        assert_eq!(trace.sample, 3);
        assert_eq!(trace.rises.len(), 43);
        assert_eq!(trace.rises[3], (false, true));
        assert_eq!(
            &trace.rises[4..7],
            &[(false, false), (false, true), (false, false)]
        );
        for bit in 0..31 {
            assert_eq!(trace.rises[7 + bit], (false, data & (1 << bit) != 0));
        }
        assert_eq!(trace.rises[38], (false, data >> 31 != 0));
        assert_eq!(trace.rises[39], (false, true));
        assert_eq!(trace.rises[40], (true, true));
        assert_eq!(&trace.rises[41..], &[(true, true), (false, true)]);
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    #[test]
    fn transfer_word_stops_on_non_ok_ack() {
        let (mut shifter, trace) = traced_shifter(&[true, false, false]);
        assert_eq!(
            shifter.transfer_word(RnW::R, 0, 0, 0, 0, 0),
            TransferResult::Wait
        );

        let trace = trace.borrow();
        assert_eq!(trace.sample, 3);
        assert_eq!(trace.rises.len(), 9);
        assert_eq!(
            &trace.rises[6..],
            &[(true, false), (true, false), (false, false)]
        );
    }

    #[test]
    fn sequence_samples_tdo_after_the_rising_edge() {
        let mut lines = lines();
        lines.tdo.level = true;
        let mut timing = Timing {
            delay: MockDelay,
            half_cycles: 1,
        };
        let info = SequenceInfo {
            n_bits: 3,
            capture: true,
            tms: false,
        };
        let mut rxbuf = [0u8];
        sequence(&mut lines, &mut timing, info, &[0b101], &mut rxbuf);
        assert_eq!(rxbuf[0], 0b111);
    }
}
