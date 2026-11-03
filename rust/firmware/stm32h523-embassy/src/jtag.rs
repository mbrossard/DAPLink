//! This board's bit-bang JTAG: the `accel` nop pacing, and the default engine's lean scans.

#[cfg(feature = "accel")]
use daplink_spi_swd::{nops, JtagPacing};

#[cfg(feature = "accel")]
use crate::fits::ACCEL_JTAG;

/// Nops pacing the low half of a fast JTAG clock: the TCK-fall to TDO-sample margin.
#[cfg(feature = "accel")]
const LOW_NOPS: usize = ACCEL_JTAG.low_nops;

/// Nops pacing the high half: TCK-high pulse width only.
#[cfg(feature = "accel")]
const HIGH_NOPS: usize = ACCEL_JTAG.high_nops;

#[cfg(feature = "accel")]
pub struct H5FastJtag;

#[cfg(feature = "accel")]
impl JtagPacing for H5FastJtag {
    const FAST_HALF_CYCLES: u32 = ACCEL_JTAG.fast_half_cycles;

    #[inline(always)]
    fn pace_low(&self) {
        nops::<LOW_NOPS>();
    }

    #[inline(always)]
    fn pace_high(&self) {
        nops::<HIGH_NOPS>();
    }
}

#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
pub use composed::{fast_jtag_deps, FastJtagDeps, FastJtagDrv, FastSwd, JtagBlock};

/// Lean scans and the JTAG block over the default engine's probe pads.
#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
mod composed {
    use daplink_rs::dap_rs::dap::TransferConfig;
    use daplink_rs::dap_rs::jtag::{self, Jtag, SequenceInfo, TransferResult};
    use daplink_rs::dap_rs::swd::{self, APnDP, DPRegister, RnW, Swd};
    use daplink_rs::dap_rs::swj::{Dependencies, Pins};
    use daplink_rs::daplink_core::jtag_dp::{ACK_OK_FAULT, ACK_WAIT, STATUS_OK, STATUS_WAIT};
    use daplink_rs::daplink_core::{JtagDp, JtagDpSource, SCAN_CHAIN_SLOTS};
    use daplink_rs::{
        DelayCycles, DynCycleDelay, LateJtag, LateJtagDeps, LateSwdDrv, ScanChainStorage,
    };
    use daplink_spi_swd::{JtagPortOps, JtagScan, KeptAck};
    use daplink_stm32_embassy::pace::{halves, pinned, Spin};
    use daplink_stm32_embassy::DapMode;
    use daplink_stm32_embassy::{IoPin, JtagPadMap, StmJtagPads};

    use crate::fits;
    use crate::pads::H5Pads;

    type Pads = StmJtagPads<H5Pads>;
    type Inner = LateJtagDeps<IoPin, DynCycleDelay>;
    type InnerSwd = LateSwdDrv<IoPin, DynCycleDelay>;
    type InnerJtag = LateJtag<IoPin, DynCycleDelay>;

    /// The kept ACK of every lean scan, at a fixed address so no scan carries a pointer to it.
    static KEPT: KeptAck = KeptAck::new(crate::LINE_SCAN.jtag_dp());

    #[derive(Clone, Copy)]
    struct Lean {
        control: (Spin, Spin),
        ack: (Spin, Spin),
        read: (Spin, Spin),
        write: (Spin, Spin),
        ir: (Spin, Spin),
    }

    impl Lean {
        fn new(cpu_hz: u32, request: u32) -> Self {
            Self {
                control: halves(&fits::JTAG_CONTROL, cpu_hz, request),
                ack: halves(&fits::JTAG_ACK, cpu_hz, request),
                read: halves(&fits::JTAG_READ, cpu_hz, request),
                write: halves(&fits::JTAG_WRITE, cpu_hz, request),
                ir: halves(&fits::JTAG_IR, cpu_hz, request),
            }
        }
    }

    #[derive(Clone, Copy)]
    struct LeanScan<'a> {
        pads: &'a Pads,
        config: &'a jtag::Config,
        control: (Spin, Spin),
        ack: (Spin, Spin),
        data: (Spin, Spin),
    }

    /// Expand `$cell` for bits 0 to 30: straight-line cells, no loop branch in any of them.
    macro_rules! cells31 {
        ($i:ident => $cell:expr) => {
            cells31!(@ $i $cell; 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
                16 17 18 19 20 21 22 23 24 25 26 27 28 29 30)
        };
        (@ $i:ident $cell:expr; $($n:literal)*) => {
            $({
                let $i: u32 = $n;
                $cell;
            })*
        };
    }

    impl JtagScan for LeanScan<'_> {
        type Port = Pads;

        #[inline(always)]
        fn scan_port(&self) -> &Pads {
            self.pads
        }

        #[inline(always)]
        fn scan_config(&mut self) -> &jtag::Config {
            self.config
        }

        #[inline(always)]
        fn scan_keep_ack(&mut self, ack: u32) {
            KEPT.keep(ack);
        }

        #[inline(always)]
        fn scan_complete(&mut self, data: u32) -> TransferResult {
            KEPT.complete(|| self.config.index, data)
        }

        #[inline(always)]
        fn scan_wait_low(&mut self) {
            self.control.0.run();
        }

        #[inline(always)]
        fn scan_wait_high(&mut self) {
            self.control.1.run();
        }

        #[inline(always)]
        fn scan_wait_data_low(&mut self) {
            self.data.0.run();
        }

        #[inline(always)]
        fn scan_wait_data_high(&mut self) {
            self.data.1.run();
        }

        #[inline(always)]
        fn scan_wait_ack_low(&mut self) {
            self.ack.0.run();
        }

        #[inline(always)]
        fn scan_wait_ack_high(&mut self) {
            self.ack.1.run();
        }

        #[inline(always)]
        fn scan_read_cells(&mut self, after: u32) -> Option<u32> {
            let exit = pinned((after == 0) as u32);
            let (mut v, mut b) = (pinned(0), 0u32);
            // Each cell folds the previous sample in its low half, ahead of its own spin.
            cells31!(i => {
                self.pads.tck_low();
                // One above its place, so every fold is the same shifted `orr`.
                if i > 0 {
                    v = pinned(v | pinned(b) << i);
                }
                self.scan_wait_data_low();
                let raw = pinned(tdo_raw());
                self.pads.tck_high();
                b = pinned((raw >> H5Pads::TDO_PIN) & 1);
                self.scan_wait_data_high();
            });
            self.pads.tms_set(exit != 0); // final data bit exits Shift-DR
            self.pads.tck_low();
            v = pinned(v | pinned(b) << 31) >> 1;
            self.scan_wait_data_low();
            b = self.pads.tdo_get();
            self.pads.tck_high();
            self.scan_wait_data_high();
            Some(v | b << 31)
        }

        #[inline(always)]
        fn scan_write_cells(&mut self, data: u32, after: u32) -> Option<()> {
            let exit = pinned((after == 0) as u32);
            cells31!(i => {
                self.pads.tdi_put(pinned(data >> i));
                self.scan_clock_out();
            });
            self.pads.tms_set(exit != 0); // final data bit exits Shift-DR
            self.pads.tdi_put(data >> 31);
            self.scan_clock_out();
            Some(())
        }
    }

    const TDI_MASK: u32 = 1u32 << H5Pads::TDI_PIN;
    const _: () = {
        assert!(H5Pads::TCK_PIN < 16 && H5Pads::TMS_PIN < 16);
        assert!(H5Pads::TDI_PIN < 16 && H5Pads::TDO_PIN < 16);
        assert!(H5Pads::TCK_PIN != H5Pads::TMS_PIN);
        assert!(H5Pads::TDI_PIN != H5Pads::TDO_PIN);
    };

    #[inline(always)]
    fn tdi_word(bit: u32) -> u32 {
        TDI_MASK << (((bit ^ 1) & 1) << 4)
    }

    #[inline(always)]
    fn tdo_raw() -> u32 {
        // SAFETY: the mapped TDI/TDO port's `IDR` is a plain read.
        unsafe { (H5Pads::tdi_port().idr().as_ptr() as *const u32).read_volatile() }
    }

    #[inline(always)]
    fn tdi_store(word: u32) {
        // SAFETY: mapped `BSRR` is write-1-to-act, and the word names the TDI pad only.
        unsafe { (H5Pads::tdi_port().bsrr().as_ptr() as *mut u32).write_volatile(word) };
    }

    /// The DPACC/APACC scan, leaving early on WAIT only; `Err` holds a captured word to decode.
    #[inline(always)]
    fn scan_word(
        s: &mut LeanScan<'_>,
        r_nw: RnW,
        a2a3: u8,
        idle: u8,
        data: u32,
    ) -> Result<u32, u32> {
        // A private copy, so the pinned cells keep its spins in registers.
        let s = &mut { *s };
        let index = u32::from(s.config.index);
        let before = pinned(index);
        let after = pinned(u32::from(s.config.device_count).saturating_sub(index + 1));
        let req = (u32::from(a2a3) << 1) | r_nw as u32;
        let (t0, t1, t2) = (
            pinned(tdi_word(req)),
            pinned(tdi_word(req >> 1)),
            pinned(tdi_word(req >> 2)),
        );

        // Run-Test/Idle -> Shift-DR: TMS 1,0,0, then the fill in front with TDI high.
        s.pads.tms_set(true);
        s.scan_clock();
        s.pads.tms_set(false);
        s.scan_clock();
        s.scan_clock();
        tdi_store(tdi_word(1));
        for _ in 0..before {
            s.scan_clock();
        }

        // RnW and A[3:2] out, the ACK in, LSB first.
        tdi_store(t0);
        let mut ack = pinned(s.scan_clock_ack_in());
        tdi_store(t1);
        ack = pinned(ack | s.scan_clock_ack_in() << 1);
        tdi_store(t2);
        ack |= s.scan_clock_ack_in() << 2;

        let complete = ack == ACK_OK_FAULT || {
            core::hint::cold_path();
            s.scan_keep_ack(ack);
            ack != ACK_WAIT
        };

        let captured = if complete {
            let captured = if r_nw == RnW::R {
                tdi_store(tdi_word(0));
                s.scan_read_cells(after).unwrap_or(0)
            } else {
                s.scan_write_cells(data, after);
                0
            };
            if after > 0 {
                tdi_store(tdi_word(1));
                for _ in 0..after - 1 {
                    s.scan_clock();
                }
                s.pads.tms_set(true);
                s.scan_clock();
            }
            // Exit1-DR -> Update-DR -> Run-Test/Idle: TMS 1,0.
            s.pads.tms_set(true);
            s.scan_clock();
            s.pads.tms_set(false);
            s.scan_clock();
            captured
        } else {
            // No data phase: Shift-DR -> Exit1 -> Update -> Idle, TMS 1,1,0.
            s.pads.tms_set(true);
            s.scan_clock();
            s.scan_clock();
            s.pads.tms_set(false);
            s.scan_clock();
            0
        };

        tdi_store(tdi_word(1));
        for _ in 0..idle {
            s.scan_clock();
        }
        if KEPT.clear() {
            Ok(captured)
        } else {
            Err(captured)
        }
    }

    #[inline(never)]
    #[link_section = ".ramcode.jtag_block"]
    fn lean_word(
        mut scan: LeanScan<'_>,
        r_nw: RnW,
        a2a3: u8,
        idle: u8,
        data: u32,
    ) -> TransferResult {
        match scan_word(&mut scan, r_nw, a2a3, idle, data) {
            Ok(v) => TransferResult::Ok(v),
            Err(v) => KEPT.complete(|| scan.config.index, v),
        }
    }

    #[inline(never)]
    #[link_section = ".ramcode.jtag_block"]
    fn lean_ir(mut scan: LeanScan<'_>, ir: u32) {
        scan.shift_ir_word(ir);
    }

    const IR_DPACC: u32 = 0x0A;
    const IR_APACC: u32 = 0x0B;

    /// A transfer's CMSIS-DAP status; a FAULT is the one its decode recorded.
    fn status(r: TransferResult, dp: &JtagDp) -> u8 {
        match r {
            TransferResult::Ok(_) => STATUS_OK,
            TransferResult::Wait => STATUS_WAIT,
            TransferResult::Fault => dp.take_failure(),
            TransferResult::Mismatch => 0x10,
        }
    }

    /// A block transfer's first attempt; a word to decode, and dap-rs's WAIT retries, go out of line.
    #[inline(always)]
    fn block_word(
        scan: &mut LeanScan<'_>,
        r_nw: RnW,
        a2a3: u8,
        idle: u8,
        retries: usize,
        data: u32,
    ) -> Result<u32, TransferResult> {
        match scan_word(scan, r_nw, a2a3, idle, data) {
            Ok(v) => Ok(v),
            Err(v) => match settle(*scan, r_nw, a2a3, idle, retries, data, v) {
                TransferResult::Ok(v) => Ok(v),
                r => Err(r),
            },
        }
    }

    /// Decode the first attempt's kept ACK, then run up to `retries` more attempts while WAIT.
    #[cold]
    #[inline(never)]
    fn settle(
        scan: LeanScan<'_>,
        r_nw: RnW,
        a2a3: u8,
        idle: u8,
        retries: usize,
        data: u32,
        captured: u32,
    ) -> TransferResult {
        let mut r = KEPT.complete(|| scan.config.index, captured);
        for _ in 0..retries {
            if r != TransferResult::Wait {
                break;
            }
            r = lean_word(scan, r_nw, a2a3, idle, data);
        }
        r
    }

    /// A read block: the post, then `count` reads, the last from RDBUFF; one scan call site.
    #[inline(never)]
    #[link_section = ".ramcode.jtag_block"]
    #[allow(clippy::too_many_arguments)] // The block's whole state, passed in registers.
    fn read_block(
        mut scan: LeanScan<'_>,
        ir: (Spin, Spin),
        ap: bool,
        a2a3: u8,
        count: usize,
        idle: u8,
        retries: usize,
        out: &mut [u8],
    ) -> (usize, TransferResult) {
        let data = scan.data;
        lean_ir(
            LeanScan { data: ir, ..scan },
            if ap { IR_APACC } else { IR_DPACC },
        );
        let mut k = 0;
        loop {
            let last = k == count;
            if last && ap {
                lean_ir(LeanScan { data: ir, ..scan }, IR_DPACC);
            }
            scan.data = data;
            let rq = if last { 3 } else { a2a3 };
            let v = match block_word(&mut scan, RnW::R, rq, idle, retries, 0) {
                Ok(v) => v,
                Err(r) => return (k.saturating_sub(1), r),
            };
            if k > 0 {
                out[4 * k..4 * k + 4].copy_from_slice(&v.to_le_bytes());
            }
            if last {
                return (count, TransferResult::Ok(v));
            }
            k += 1;
        }
    }

    /// A write block up to the first incomplete transfer, then the RDBUFF check of the last write.
    #[inline(never)]
    #[link_section = ".ramcode.jtag_block"]
    #[allow(clippy::too_many_arguments)] // The block's whole state, passed in registers.
    fn write_block(
        mut scan: LeanScan<'_>,
        ir: (Spin, Spin),
        read: (Spin, Spin),
        ap: bool,
        a2a3: u8,
        idle: u8,
        retries: usize,
        words: &[u8],
    ) -> (usize, TransferResult) {
        lean_ir(
            LeanScan { data: ir, ..scan },
            if ap { IR_APACC } else { IR_DPACC },
        );
        for (done, w) in words.chunks_exact(4).enumerate() {
            let v = u32::from_le_bytes([w[0], w[1], w[2], w[3]]);
            if let Err(r) = block_word(&mut scan, RnW::W, a2a3, idle, retries, v) {
                return (done, r);
            }
        }
        // Check the last write off RDBUFF; its ACK is the status.
        if ap {
            lean_ir(LeanScan { data: ir, ..scan }, IR_DPACC);
        }
        let mut left = retries;
        loop {
            let r = lean_word(LeanScan { data: read, ..scan }, RnW::R, 3, idle, 0);
            if r != TransferResult::Wait || left == 0 {
                return (words.len() / 4, r);
            }
            left -= 1;
        }
    }

    static BLOCK_CHAIN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();

    /// `DAP_TransferBlock` on JTAG ahead of dap-rs, its settings mirrored from dap-rs's answers.
    pub struct JtagBlock {
        pads: Pads,
        config: jtag::Config,
        cpu_hz: u32,
        dap: DapMode,
        lean: Lean,
        idle: u8,
        retries: usize,
    }

    impl JtagBlock {
        /// dap-rs's defaults: 1 MHz, one idle cycle, five WAIT retries, no chain.
        pub fn new(cpu_hz: u32) -> Self {
            Self {
                pads: Pads::new(),
                config: jtag::Config::new(BLOCK_CHAIN.init()),
                cpu_hz,
                dap: DapMode::new(),
                lean: Lean::new(cpu_hz, 1_000_000),
                idle: 1,
                retries: 5,
            }
        }

        pub fn process(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
            if !self.dap.jtag_held() || report.first() != Some(&0x06) {
                return None;
            }
            let idx = *report.get(1)?;
            let count = usize::from(u16::from_le_bytes([*report.get(2)?, *report.get(3)?]));
            let treq = *report.get(4)?;
            let read = treq & 2 != 0;
            if response.len() < 4 + if read { 4 * count } else { 0 } {
                return None;
            }
            let words = if read {
                &[][..]
            } else {
                report.get(5..5 + 4 * count)?
            };
            response[0] = 0x06;
            response[1..4].fill(0);
            if idx >= self.config.device_count || count == 0 {
                return Some(4);
            }
            self.config.index = idx;
            let lean = self.lean;
            let scan = LeanScan {
                pads: &self.pads,
                config: &self.config,
                control: lean.control,
                ack: lean.ack,
                data: if read { lean.read } else { lean.write },
            };
            let (ap, a2a3) = (treq & 1 != 0, (treq >> 2) & 3);
            let (done, r) = if read {
                read_block(
                    scan,
                    lean.ir,
                    ap,
                    a2a3,
                    count,
                    self.idle,
                    self.retries,
                    response,
                )
            } else {
                write_block(
                    scan,
                    lean.ir,
                    lean.read,
                    ap,
                    a2a3,
                    self.idle,
                    self.retries,
                    words,
                )
            };
            response[1..3].copy_from_slice(&(done as u16).to_le_bytes());
            response[3] = status(r, KEPT.dp());
            Some(4 + if read { 4 * done } else { 0 })
        }

        pub fn suspend(&mut self) {
            self.dap.suspend();
        }

        pub fn observe(&mut self, report: &[u8], response: &[u8]) {
            self.dap.observe(report, response);
            let ok = response.get(1) == Some(&0);
            match report.first() {
                Some(0x04) if ok && report.len() >= 4 => {
                    self.idle = report[1];
                    self.retries = usize::from(u16::from_le_bytes([report[2], report[3]]));
                }
                Some(0x11) if ok && report.len() >= 5 => {
                    let hz = u32::from_le_bytes([report[1], report[2], report[3], report[4]]);
                    if hz != 0 {
                        self.lean = Lean::new(self.cpu_hz, hz);
                    }
                }
                Some(0x15) if ok && report.len() >= 2 => {
                    let count = usize::from(report[1]);
                    let Some(lengths) = report.get(2..2 + count) else {
                        return;
                    };
                    if count >= self.config.scan_chain.len() {
                        return;
                    }
                    self.config.device_count = count as u8;
                    let total: u16 = lengths.iter().map(|&l| u16::from(l)).sum();
                    let mut before = 0u16;
                    for (tap, &len) in self.config.scan_chain.iter_mut().zip(lengths) {
                        tap.ir_length = len;
                        tap.ir_before = before;
                        before += u16::from(len);
                        tap.ir_after = total - before;
                    }
                }
                _ => {}
            }
        }
    }

    struct Fast {
        pads: Pads,
        delay: DynCycleDelay,
        lean: Lean,
    }

    impl Fast {
        fn set_clock(&mut self, max_frequency: u32) {
            if max_frequency != 0 {
                self.lean = Lean::new(self.delay.cpu_clock(), max_frequency);
            }
        }
    }

    pub struct FastJtagDeps {
        inner: Inner,
        fast: Fast,
    }

    pub struct FastSwd {
        inner: InnerSwd,
        fast: Fast,
    }

    pub struct FastJtagDrv {
        inner: InnerJtag,
        fast: Fast,
    }

    /// The fast engine over `inner`, which must be wired over `LINE_SCAN`.
    pub fn fast_jtag_deps(inner: Inner, delay: DynCycleDelay) -> FastJtagDeps {
        assert!(
            inner
                .jtag_dp()
                .is_some_and(|dp| core::ptr::eq(dp, KEPT.dp())),
            "the lean scans decode through the line's JtagDp"
        );
        FastJtagDeps {
            inner,
            fast: Fast {
                pads: Pads::new(),
                delay,
                lean: Lean::new(delay.cpu_clock(), 1_000_000),
            },
        }
    }

    impl From<FastJtagDeps> for FastSwd {
        fn from(deps: FastJtagDeps) -> Self {
            Self {
                inner: deps.inner.into(),
                fast: deps.fast,
            }
        }
    }

    impl From<FastSwd> for FastJtagDeps {
        fn from(drv: FastSwd) -> Self {
            Self {
                inner: drv.inner.into(),
                fast: drv.fast,
            }
        }
    }

    impl From<FastJtagDeps> for FastJtagDrv {
        fn from(deps: FastJtagDeps) -> Self {
            let mut fast = deps.fast;
            // Ahead of the shifter's staging, so the pads see TCK's direction before the switch.
            fast.pads.jtag_pads_acquire();
            let inner: InnerJtag = deps.inner.into();
            Self { inner, fast }
        }
    }

    impl From<FastJtagDrv> for FastJtagDeps {
        fn from(drv: FastJtagDrv) -> Self {
            let mut fast = drv.fast;
            fast.pads.jtag_pads_release();
            Self {
                inner: drv.inner.into(),
                fast,
            }
        }
    }

    impl JtagDpSource for FastJtagDeps {
        fn jtag_dp(&self) -> Option<&'static JtagDp> {
            self.inner.jtag_dp()
        }
    }

    impl Dependencies<FastSwd, FastJtagDrv> for FastJtagDeps {
        fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
            self.inner.process_swj_pins(output, mask, wait_us)
        }

        fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
            self.inner.process_swj_sequence(data, nbits)
        }

        fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
            // Primed here too, so a clock set before `DAP_Connect(JTAG)` carries over.
            self.fast.set_clock(max_frequency);
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

    impl Swd<FastJtagDeps> for FastSwd {
        fn available(_deps: &FastJtagDeps) -> bool {
            true
        }

        fn config(&mut self) -> &mut swd::Config {
            self.inner.config()
        }

        fn read(&mut self, wait_retries: usize, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
            self.inner.read(wait_retries, port, addr)
        }

        fn read_inner(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
            self.inner.read_inner(port, addr)
        }

        fn write(
            &mut self,
            wait_retries: usize,
            port: APnDP,
            addr: DPRegister,
            data: u32,
        ) -> swd::Result<()> {
            self.inner.write(wait_retries, port, addr, data)
        }

        fn write_inner(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
            self.inner.write_inner(port, addr, data)
        }

        fn set_clock(&mut self, max_frequency: u32) -> bool {
            self.fast.set_clock(max_frequency);
            self.inner.set_clock(max_frequency)
        }

        fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
            self.inner.write_sequence(num_bits, data)
        }

        fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
            self.inner.read_sequence(num_bits, data)
        }
    }

    impl Jtag<FastJtagDeps> for FastJtagDrv {
        fn available(_deps: &FastJtagDeps) -> bool {
            true
        }

        fn config(&mut self) -> &mut jtag::Config {
            self.inner.config()
        }

        fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
            self.inner.sequence(info, tdi, rxbuf)
        }

        fn tms_sequence(&mut self, tms: &[bool]) {
            self.inner.tms_sequence(tms)
        }

        fn set_clock(&mut self, max_frequency: u32) -> bool {
            self.fast.set_clock(max_frequency);
            self.inner.set_clock(max_frequency)
        }

        /// The specialized DPACC/APACC scan; `write_abort` keeps the generic mechanics.
        fn transfer(
            &mut self,
            r_nw: RnW,
            a2a3: u8,
            transfer_config: &TransferConfig,
            data: u32,
        ) -> TransferResult {
            let lean = self.fast.lean;
            let halves = match r_nw {
                RnW::R => lean.read,
                RnW::W => lean.write,
            };
            let scan = LeanScan {
                pads: &self.fast.pads,
                config: self.inner.config(),
                control: lean.control,
                ack: lean.ack,
                data: halves,
            };
            lean_word(scan, r_nw, a2a3, transfer_config.idle_cycles, data)
        }

        /// The specialized IR scan in place of the provided per-chunk `shift_ir`.
        fn shift_ir(&mut self, ir: u32) {
            let lean = self.fast.lean;
            let scan = LeanScan {
                pads: &self.fast.pads,
                config: self.inner.config(),
                control: lean.control,
                ack: lean.ack,
                data: lean.ir,
            };
            lean_ir(scan, ir);
        }
    }
}
