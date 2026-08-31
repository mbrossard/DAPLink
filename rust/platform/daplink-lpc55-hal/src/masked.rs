//! Masked-GPIO accelerated SWD backend (feature `masked-swd`): with `MASK0` routing `MPIN0` to the
//! SWDIO bit alone, one store drives the line and one load samples it, with no branch and no RMW.

use daplink_rs::dap_rs::dap::TransferConfig;
use daplink_rs::dap_rs::driver::bitbang::BitbangAdapter;
use daplink_rs::dap_rs::jtag::{self, Jtag, SequenceInfo, TransferResult};
use daplink_rs::dap_rs::swd::{self, APnDP, DPRegister, RnW, Swd};
use daplink_rs::dap_rs::swj::{Dependencies, Pins};
use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::{wired_deps, NoopLeds, ScanChainStorage};
use daplink_rs::{DelayCycles, InputOutputPin, PreparedSpin};
use daplink_spi_swd::nops;
use daplink_spi_swd::session::{parity32, request_byte};
use daplink_spi_swd::{
    prepare_jtag_timing, prepare_swd_timing, HalfCycleProfile, HalfCycles, JtagPacing, JtagPortOps,
    JtagScan, JtagTimingPlan, PacingPlan, PacingProfile, SwdTimingPlan,
};
use lpc55_hal::raw;

use crate::delay::DynCycleDelay;
use crate::pin::IoPin;

/// SWDIO bit position in GPIO port 0 (PIO0_2).
const SWDIO_PIN: u32 = 2;
/// SWDIO line mask.
const SWDIO_MASK: u32 = 1 << SWDIO_PIN;
/// SWDIO direction-buffer enable mask (TXEN = PIO0_28).
const TXEN_MASK: u32 = 1 << 28;
/// SWCLK line mask (PIO0_0).
const SWCLK_MASK: u32 = 1 << 0;
/// JTAG TDI line mask (PIO0_1).
const TDI_MASK: u32 = 1 << 1;
/// JTAG TDO bit position (PIO0_3, shared with SWO capture).
const TDO_PIN: u32 = 3;

/// The injected accel pacing: SWD transfer halves on top of the fast-JTAG ladder; nop counts and
/// thresholds are firmware-owned constants.
pub trait MaskedPacing: JtagPacing + Copy {
    /// Half-period spin count at or below which the fast SWD word routines run.
    const SWD_FAST_HALF_CYCLES: u32;

    /// Half-period count at or below which fast JTAG scans run; zero keeps every scan scaled.
    const JTAG_FAST_HALF_CYCLES: u32;

    /// Minimum residual for each scaled JTAG scan half.
    const JTAG_SCALED_FLOOR_CYCLES: u32;
    /// Minimum half-period for generic JTAG sequences.
    const JTAG_SEQUENCE_FLOOR_CYCLES: u32;

    /// Fixed scaled-path work for TAP-state clocks, as low/high core cycles.
    const JTAG_CONTROL_FIXED_CYCLES: (u32, u32);
    /// Fixed scaled-path work for sampled clocks, as low/high core cycles.
    const JTAG_READ_FIXED_CYCLES: (u32, u32);
    /// Fixed scaled-path work for driven clocks, as low/high core cycles.
    const JTAG_WRITE_FIXED_CYCLES: (u32, u32);
    /// Fixed scaled-path work for IR data clocks, as low/high core cycles.
    const JTAG_IR_FIXED_CYCLES: (u32, u32);

    /// Pace the low half of fast SWD control phases.
    fn swd_control_low(&self);

    /// Pace the high half of fast SWD control phases.
    fn swd_control_high(&self);

    /// Pace the low half of a fast SWD read data phase.
    fn swd_read_low(&self);

    /// Pace the high half of a fast SWD read data phase.
    fn swd_read_high(&self);

    /// Pace the low half of a fast SWD write data phase.
    fn swd_write_low(&self);

    /// Pace the high half of a fast SWD write data phase.
    fn swd_write_high(&self);
}

/// The port-0 GPIO register addresses, resolved once so the word loops keep them in registers.
#[derive(Clone, Copy)]
struct Regs {
    mask: usize,
    mpin: usize,
    set: usize,
    clr: usize,
    pin: usize,
    dirset: usize,
    dirclr: usize,
}

impl Regs {
    fn new() -> Self {
        // SAFETY: the GPIO block is a fixed peripheral; `addr_of!` forms no reference.
        let gpio = raw::GPIO::ptr();
        unsafe {
            Regs {
                mask: core::ptr::addr_of!((*gpio).mask[0]) as usize,
                mpin: core::ptr::addr_of!((*gpio).mpin[0]) as usize,
                set: core::ptr::addr_of!((*gpio).set[0]) as usize,
                clr: core::ptr::addr_of!((*gpio).clr[0]) as usize,
                pin: core::ptr::addr_of!((*gpio).pin[0]) as usize,
                dirset: core::ptr::addr_of!((*gpio).dirset[0]) as usize,
                dirclr: core::ptr::addr_of!((*gpio).dirclr[0]) as usize,
            }
        }
    }

    /// Route `MPIN0` to the SWDIO bit alone; `SET`/`CLR`/`PIN`/`DIR*` are unaffected by `MASK`.
    #[inline]
    fn take_wire(self) {
        // SAFETY: resolved GPIO register; plain data write.
        unsafe { (self.mask as *mut u32).write_volatile(!SWDIO_MASK) };
    }

    /// Restore `MASK0` to its reset value on every SWD-mode exit, keeping the routing local.
    #[inline]
    fn release_wire(self) {
        // SAFETY: resolved GPIO register; plain data write.
        unsafe { (self.mask as *mut u32).write_volatile(0) };
    }

    /// Drive SWDIO through the masked write latch; `bit` must be 0 or 1.
    #[inline(always)]
    fn swdio_out(self, bit: u32) {
        // SAFETY: resolved GPIO register; `MASK0` confines the write to the SWDIO bit.
        unsafe { (self.mpin as *mut u32).write_volatile(bit << SWDIO_PIN) };
    }

    /// Sample SWDIO through the masked pin register: the level in bit [`SWDIO_PIN`], rest zero.
    #[inline(always)]
    fn swdio_in(self) -> u32 {
        // SAFETY: resolved GPIO register; plain read.
        unsafe { (self.mpin as *const u32).read_volatile() }
    }

    /// SWCLK falling edge.
    #[inline(always)]
    fn clk_low(self) {
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (self.clr as *mut u32).write_volatile(SWCLK_MASK) };
    }

    /// SWCLK rising edge.
    #[inline(always)]
    fn clk_high(self) {
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (self.set as *mut u32).write_volatile(SWCLK_MASK) };
    }

    /// Turnaround to drive SWDIO: TXEN before pad direction, so the line is never doubly driven.
    #[inline(always)]
    fn drive_swdio(self) {
        // SAFETY: resolved GPIO registers; dedicated write-1-to-act bits.
        unsafe {
            (self.set as *mut u32).write_volatile(TXEN_MASK);
            (self.dirset as *mut u32).write_volatile(SWDIO_MASK);
        }
    }

    /// Turnaround to release SWDIO: pad to input before TXEN, the exact reverse of the drive.
    #[inline(always)]
    fn release_swdio(self) {
        // SAFETY: resolved GPIO registers; dedicated write-1-to-act bits.
        unsafe {
            (self.dirclr as *mut u32).write_volatile(SWDIO_MASK);
            (self.clr as *mut u32).write_volatile(TXEN_MASK);
        }
    }
}

/// The two clock-half pacing strategies, selected once per transfer.
trait ClockHalves: Copy {
    /// Pace the low half.
    fn wait_low(self);

    /// Pace the high half.
    fn wait_high(self);

    /// Pace a data-phase low half, defaulting to the control pacing.
    #[inline(always)]
    fn wait_data_low(self) {
        self.wait_low();
    }

    /// Pace a data-phase high half, defaulting to the control pacing.
    #[inline(always)]
    fn wait_data_high(self) {
        self.wait_high();
    }
}

/// Fast read pacing: the injected nop blocks at or below the fast threshold.
#[derive(Clone, Copy)]
struct ReadPacedHalves<P: MaskedPacing>(P);

impl<P: MaskedPacing> ClockHalves for ReadPacedHalves<P> {
    #[inline(always)]
    fn wait_low(self) {
        self.0.swd_control_low();
    }

    #[inline(always)]
    fn wait_high(self) {
        self.0.swd_control_high();
    }

    #[inline(always)]
    fn wait_data_low(self) {
        self.0.swd_read_low();
    }

    #[inline(always)]
    fn wait_data_high(self) {
        self.0.swd_read_high();
    }
}

/// Fast write pacing: the injected nop blocks at or below the fast threshold.
#[derive(Clone, Copy)]
struct WritePacedHalves<P: MaskedPacing>(P);

impl<P: MaskedPacing> ClockHalves for WritePacedHalves<P> {
    #[inline(always)]
    fn wait_low(self) {
        self.0.swd_control_low();
    }

    #[inline(always)]
    fn wait_high(self) {
        self.0.swd_control_high();
    }

    #[inline(always)]
    fn wait_data_low(self) {
        self.0.swd_write_low();
    }

    #[inline(always)]
    fn wait_data_high(self) {
        self.0.swd_write_high();
    }
}

/// Scaled pacing for lower requested clocks: the injected cycle-counting spin.
#[derive(Clone, Copy)]
struct SpinHalves {
    delay: DynCycleDelay,
    cycles: HalfCycles,
}

impl ClockHalves for SpinHalves {
    #[inline(always)]
    fn wait_low(self) {
        let mut delay = self.delay;
        delay.delay_cycles(self.cycles.low());
    }

    #[inline(always)]
    fn wait_high(self) {
        let mut delay = self.delay;
        delay.delay_cycles(self.cycles.high());
    }
}

/// Launder a trip count through an empty asm block so the data phases stay counted loops.
// `reg` is a whole register on the pinned thumb target; only the host gate's wider registers
// make the operand's width worth spelling out.
#[allow(asm_sub_register)]
#[inline(always)]
fn opaque(mut count: u32) -> u32 {
    // SAFETY: an empty asm block; `count` passes through a register unchanged.
    unsafe {
        core::arch::asm!(
            "/* {0} */",
            inout(reg) count,
            options(nomem, nostack, preserves_flags)
        );
    }
    count
}

/// One outgoing bit: present the data, then clock it (target samples on the rising edge).
#[inline(always)]
fn bit_out<H: ClockHalves>(r: Regs, h: H, bit: u32) {
    r.swdio_out(bit);
    r.clk_low();
    h.wait_low();
    r.clk_high();
    h.wait_high();
}

/// One incoming bit, sampled after the low half-period (before the rising edge).
#[inline(always)]
fn bit_in<H: ClockHalves>(r: Regs, h: H) -> u32 {
    r.clk_low();
    h.wait_low();
    let bit = r.swdio_in() >> SWDIO_PIN;
    r.clk_high();
    h.wait_high();
    bit
}

/// One outgoing data bit using the separately calibrated data-phase pacing.
#[inline(always)]
fn data_bit_out<H: ClockHalves>(r: Regs, h: H, bit: u32) {
    r.swdio_out(bit);
    r.clk_low();
    h.wait_data_low();
    r.clk_high();
    h.wait_data_high();
}

/// One incoming data bit using the separately calibrated data-phase pacing.
#[inline(always)]
fn data_bit_in<H: ClockHalves>(r: Regs, h: H) -> u32 {
    r.clk_low();
    h.wait_data_low();
    let bit = r.swdio_in() >> SWDIO_PIN;
    r.clk_high();
    h.wait_data_high();
    bit
}

/// One clock with no data movement (turnaround cycles).
#[inline(always)]
fn clock_only<H: ClockHalves>(r: Regs, h: H) {
    r.clk_low();
    h.wait_low();
    r.clk_high();
    h.wait_high();
}

/// The write data phase's shift register: data LSB-first with the parity folded in as it goes.
#[derive(Clone, Copy)]
struct WriteShift {
    v: u32,
    parity: u32,
}

impl WriteShift {
    #[inline(always)]
    const fn new(data: u32) -> Self {
        WriteShift { v: data, parity: 0 }
    }

    #[inline(always)]
    const fn next_bit(&mut self) -> u32 {
        let bit = self.v & 1;
        self.parity ^= bit;
        self.v >>= 1;
        bit
    }

    #[inline(always)]
    const fn parity(self) -> u32 {
        self.parity
    }
}

// The fold checked against the shared `parity32` at compile time: this crate has no host tests.
const _: () = {
    const fn folded_parity(data: u32) -> u32 {
        let mut shift = WriteShift::new(data);
        let mut i = 0;
        while i < 32 {
            shift.next_bit();
            i += 1;
        }
        shift.parity()
    }
    let mut bit = 0;
    while bit < 32 {
        let w = 1u32 << bit;
        assert!(folded_parity(w) == parity32(w) as u32);
        assert!(folded_parity(!w) == parity32(!w) as u32);
        bit += 1;
    }
    assert!(folded_parity(0) == parity32(0) as u32);
    assert!(folded_parity(0xDEAD_BEEF) == parity32(0xDEAD_BEEF) as u32);
};

macro_rules! read_sample_pad {
    (natural) => {};
    (nop) => {
        nops::<1>();
    };
}

macro_rules! read_data_cell {
    ($r:ident, $h:ident, $data:ident, $sample_pad:tt) => {{
        $r.clk_low();
        $h.wait_data_low();
        read_sample_pad!($sample_pad);
        $data = ($data >> 1) | ($r.swdio_in() << 29);
        $r.clk_high();
        $h.wait_data_high();
    }};
}

macro_rules! write_data_cell {
    ($r:ident, $h:ident, $shift:ident, $slot:tt) => {{
        data_bit_out($r, $h, $shift.next_bit());
    }};
}

macro_rules! define_fast_data_phase {
    ($bits:expr, $groups:expr; $($sample_pad:tt),+ $(,)?) => {
        #[inline(always)]
        fn read_fast_data<H: ClockHalves>(r: Regs, h: H, mut data: u32) -> u32 {
            let mut groups = opaque($groups);
            while groups != 0 {
                $(read_data_cell!(r, h, data, $sample_pad);)+
                groups -= 1;
            }
            data
        }

        #[inline(always)]
        fn write_fast_data<H: ClockHalves>(r: Regs, h: H, mut shift: WriteShift) -> WriteShift {
            let mut groups = opaque($groups);
            while groups != 0 {
                $(write_data_cell!(r, h, shift, $sample_pad);)+
                groups -= 1;
            }
            shift
        }

        const FAST_DATA_GROUP_BITS: usize = [$(stringify!($sample_pad)),+].len();
        const _: () = assert!(FAST_DATA_GROUP_BITS == $bits);
        const _: () = assert!($bits * $groups == 32);
    };
}

// Four cells per group remove most loop branches without duplicating the transfer semantics.
define_fast_data_phase!(4, 8; natural, nop, natural, nop);

/// Shift the 8-bit request out LSB-first, unrolled (short enough that the unroll costs little).
#[inline(always)]
fn request_out<H: ClockHalves>(r: Regs, h: H, req: u8) {
    let mut v = req as u32;
    bit_out(r, h, v & 1);
    v >>= 1;
    bit_out(r, h, v & 1);
    v >>= 1;
    bit_out(r, h, v & 1);
    v >>= 1;
    bit_out(r, h, v & 1);
    v >>= 1;
    bit_out(r, h, v & 1);
    v >>= 1;
    bit_out(r, h, v & 1);
    v >>= 1;
    bit_out(r, h, v & 1);
    v >>= 1;
    bit_out(r, h, v & 1);
}

macro_rules! counted_read_data {
    ($r:ident, $h:ident, $data:ident) => {{
        let mut bits = opaque(32);
        while bits != 0 {
            read_data_cell!($r, $h, $data, natural);
            bits -= 1;
        }
    }};
}

macro_rules! counted_write_data {
    ($r:ident, $h:ident, $shift:ident) => {{
        let mut bits = opaque(32);
        while bits != 0 {
            write_data_cell!($r, $h, $shift, 0);
            bits -= 1;
        }
    }};
}

macro_rules! fast_read_data {
    ($r:ident, $h:ident, $data:ident) => {
        $data = read_fast_data($r, $h, $data);
    };
}

macro_rules! fast_write_data {
    ($r:ident, $h:ident, $shift:ident) => {
        $shift = write_fast_data($r, $h, $shift);
    };
}

macro_rules! read_word_body {
    ($r:ident, $h:ident, $req:ident, $data_phase:ident) => {{
        $r.drive_swdio();
        request_out($r, $h, $req);
        $r.release_swdio();
        clock_only($r, $h);

        let ack = bit_in($r, $h) | bit_in($r, $h) << 1 | bit_in($r, $h) << 2;
        if let Err(e) = swd::Ack::try_ok(ack as u8) {
            clock_only($r, $h);
            return Err(e);
        }

        let mut data: u32 = 0;
        $data_phase!($r, $h, data);

        let parity = data_bit_in($r, $h);
        clock_only($r, $h);
        if parity != parity32(data) as u32 {
            return Err(swd::Error::BadParity);
        }

        Ok(data)
    }};
}

macro_rules! write_word_body {
    ($r:ident, $h:ident, $req:ident, $data:ident, $data_phase:ident) => {{
        $r.drive_swdio();
        request_out($r, $h, $req);
        $r.release_swdio();
        clock_only($r, $h);

        let ack = bit_in($r, $h) | bit_in($r, $h) << 1 | bit_in($r, $h) << 2;
        clock_only($r, $h);
        swd::Ack::try_ok(ack as u8)?;

        $r.drive_swdio();
        let mut shift = WriteShift::new($data);
        $data_phase!($r, $h, shift);
        data_bit_out($r, $h, shift.parity());

        Ok(())
    }};
}

/// One full SWD read transfer in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code.masked_swd"]
fn read_word<H: ClockHalves>(r: Regs, h: H, req: u8) -> swd::Result<u32> {
    read_word_body!(r, h, req, counted_read_data)
}

/// The grouped read uses the same transfer body and changes only its data phase.
#[inline(never)]
#[link_section = ".sramx_code.masked_swd"]
fn read_word_fast<H: ClockHalves>(r: Regs, h: H, req: u8) -> swd::Result<u32> {
    read_word_body!(r, h, req, fast_read_data)
}

/// One full SWD write transfer in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code.masked_swd"]
fn write_word<H: ClockHalves>(r: Regs, h: H, req: u8, data: u32) -> swd::Result<()> {
    write_word_body!(r, h, req, data, counted_write_data)
}

/// The grouped write uses the same transfer body and changes only its data phase.
#[inline(never)]
#[link_section = ".sramx_code.masked_swd"]
fn write_word_fast<H: ClockHalves>(r: Regs, h: H, req: u8, data: u32) -> swd::Result<()> {
    write_word_body!(r, h, req, data, fast_write_data)
}

/// The masked-GPIO engine state, including duplicate pin handles for the JTAG mechanics.
struct Masked<P: MaskedPacing> {
    regs: Regs,
    /// Bit-bang adapter over duplicate handles of the same pads: SWJ traffic, sequences, parking.
    bb: BitbangAdapter<IoPin, DynCycleDelay>,
    nreset: IoPin,
    tdi: IoPin,
    tms_swdio: IoPin,
    tck_swclk: IoPin,
    tdo: IoPin,
    delay: DynCycleDelay,
    pacing: P,
    swd_timing: SwdTimingPlan,
    jtag_timing: PreparedJtagTiming,
}

#[derive(Clone, Copy)]
struct PreparedJtagHalves {
    low: PreparedSpin,
    high: PreparedSpin,
}

impl PreparedJtagHalves {
    fn new(cycles: HalfCycles) -> Self {
        Self {
            low: PreparedSpin::new(cycles.low()),
            high: PreparedSpin::new(cycles.high()),
        }
    }
}

#[derive(Clone, Copy)]
enum PreparedJtagPacing {
    Fast,
    Scaled(PreparedJtagHalves),
}

impl PreparedJtagPacing {
    fn new(plan: PacingPlan) -> Self {
        match plan {
            PacingPlan::Fast => Self::Fast,
            PacingPlan::Scaled(cycles) => Self::Scaled(PreparedJtagHalves::new(cycles)),
        }
    }
}

#[derive(Clone, Copy)]
struct PreparedJtagTiming {
    sequence_half: u32,
    control: PreparedJtagHalves,
    read: PreparedJtagPacing,
    write: PreparedJtagPacing,
    ir: PreparedJtagPacing,
}

impl PreparedJtagTiming {
    fn new(plan: JtagTimingPlan) -> Self {
        Self {
            sequence_half: plan.sequence_half_cycles(),
            control: PreparedJtagHalves::new(plan.control()),
            read: PreparedJtagPacing::new(plan.read()),
            write: PreparedJtagPacing::new(plan.write()),
            ir: PreparedJtagPacing::new(plan.ir()),
        }
    }
}

impl<P: MaskedPacing> Masked<P> {
    const fn swd_pacing() -> PacingProfile {
        PacingProfile::new(P::SWD_FAST_HALF_CYCLES, HalfCycleProfile::uncalibrated())
    }

    const fn jtag_read_pacing() -> PacingProfile {
        let (fixed_low, fixed_high) = P::JTAG_READ_FIXED_CYCLES;
        PacingProfile::new(
            P::JTAG_FAST_HALF_CYCLES,
            HalfCycleProfile::new(
                fixed_low,
                fixed_high,
                P::JTAG_SCALED_FLOOR_CYCLES,
                P::JTAG_SCALED_FLOOR_CYCLES,
            ),
        )
    }

    const fn jtag_control_pacing() -> HalfCycleProfile {
        let (fixed_low, fixed_high) = P::JTAG_CONTROL_FIXED_CYCLES;
        HalfCycleProfile::new(
            fixed_low,
            fixed_high,
            P::JTAG_SCALED_FLOOR_CYCLES,
            P::JTAG_SCALED_FLOOR_CYCLES,
        )
    }

    const fn jtag_write_pacing() -> PacingProfile {
        let (fixed_low, fixed_high) = P::JTAG_WRITE_FIXED_CYCLES;
        PacingProfile::new(
            P::JTAG_FAST_HALF_CYCLES,
            HalfCycleProfile::new(
                fixed_low,
                fixed_high,
                P::JTAG_SCALED_FLOOR_CYCLES,
                P::JTAG_SCALED_FLOOR_CYCLES,
            ),
        )
    }

    const fn jtag_ir_pacing() -> PacingProfile {
        let (fixed_low, fixed_high) = P::JTAG_IR_FIXED_CYCLES;
        PacingProfile::new(
            P::JTAG_FAST_HALF_CYCLES,
            HalfCycleProfile::new(
                fixed_low,
                fixed_high,
                P::JTAG_SCALED_FLOOR_CYCLES,
                P::JTAG_SCALED_FLOOR_CYCLES,
            ),
        )
    }

    fn new(
        delay: DynCycleDelay,
        pacing: P,
        scan: &'static ScanChainStorage<SCAN_CHAIN_SLOTS>,
    ) -> Self {
        // The MCU-Link pin map; the handles are `Copy`, one set for the adapter, one kept here.
        let nreset = crate::pin::nreset();
        let tdi = IoPin::output(0, 1);
        let tms_swdio = IoPin::buffered(0, 2, 0, 28);
        let tck_swclk = IoPin::output(0, 0);
        let tdo = IoPin::input(0, 3);

        let mut bb = wired_deps(nreset, tdi, tms_swdio, tck_swclk, tdo, delay, scan);
        bb.process_swj_clock(2_000_000);

        let swd_timing = prepare_swd_timing(
            delay.cpu_clock(),
            2_000_000,
            Self::swd_pacing(),
            Self::swd_pacing(),
        )
        .expect("the default SWD frequency is non-zero");
        let jtag_timing = PreparedJtagTiming::new(
            prepare_jtag_timing(
                delay.cpu_clock(),
                2_000_000,
                P::JTAG_SEQUENCE_FLOOR_CYCLES,
                Self::jtag_control_pacing(),
                Self::jtag_read_pacing(),
                Self::jtag_write_pacing(),
                Self::jtag_ir_pacing(),
            )
            .expect("the default JTAG frequency is non-zero"),
        );

        Self {
            regs: Regs::new(),
            bb,
            nreset,
            tdi,
            tms_swdio,
            tck_swclk,
            tdo,
            delay,
            pacing,
            swd_timing,
            jtag_timing,
        }
    }

    /// Recompute the SWD half-period and pick fast or scaled pacing, keeping the bit-bang side in
    /// step. `false` only for a zero request.
    fn set_swd_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        let Some(timing) = prepare_swd_timing(
            self.delay.cpu_clock(),
            max_frequency,
            Self::swd_pacing(),
            Self::swd_pacing(),
        ) else {
            return false;
        };
        self.bb.process_swj_clock(max_frequency);
        self.swd_timing = timing;
        true
    }

    /// Recompute JTAG scan and sequence pacing with their independent floors.
    /// `false` only for a zero request.
    fn set_jtag_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        let Some(timing) = prepare_jtag_timing(
            self.delay.cpu_clock(),
            max_frequency,
            P::JTAG_SEQUENCE_FLOOR_CYCLES,
            Self::jtag_control_pacing(),
            Self::jtag_read_pacing(),
            Self::jtag_write_pacing(),
            Self::jtag_ir_pacing(),
        ) else {
            return false;
        };
        self.jtag_timing = PreparedJtagTiming::new(timing);
        true
    }

    #[inline]
    fn read_transfer(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        let req = request_byte(port, RnW::R, addr);
        // The fast/scaled split lives here, outside the bit loops.
        match self.swd_timing.read() {
            PacingPlan::Fast => read_word_fast(self.regs, ReadPacedHalves(self.pacing), req),
            PacingPlan::Scaled(cycles) => read_word(
                self.regs,
                SpinHalves {
                    delay: self.delay,
                    cycles,
                },
                req,
            ),
        }
    }

    #[inline]
    fn write_transfer(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        let req = request_byte(port, RnW::W, addr);
        match self.swd_timing.write() {
            PacingPlan::Fast => {
                write_word_fast(self.regs, WritePacedHalves(self.pacing), req, data)
            }
            PacingPlan::Scaled(cycles) => write_word(
                self.regs,
                SpinHalves {
                    delay: self.delay,
                    cycles,
                },
                req,
                data,
            ),
        }
    }

    /// Counted pacing for generic JTAG mechanics.
    #[inline(always)]
    fn jtag_wait(&mut self) {
        let mut delay = self.delay;
        delay.delay_cycles(self.jtag_timing.sequence_half);
    }

    /// Shift up to 8 TDI bits LSB-first at a fixed (already-set) TMS level, capturing TDO.
    fn jtag_shift(&mut self, tdi: u32, num_bits: usize) -> u32 {
        let mut tdo = 0;
        for i in 0..num_bits {
            self.tdi_set(tdi & (1 << i) != 0);
            self.tck_low();
            self.jtag_wait();
            tdo |= self.tdo_get() << i;
            self.tck_high();
            self.jtag_wait();
        }
        tdo
    }

    /// Walk a TMS bit sequence (TAP-state walk) with TDI unchanged.
    fn tms_walk(&mut self, tms: &[bool]) {
        for &bit in tms {
            self.tms_set(bit);
            self.tck_low();
            self.jtag_wait();
            self.tck_high();
            self.jtag_wait();
        }
    }

    /// JTAG pin roles on mode entry: TMS idle-high and driven, TCK/TDI outputs, TDO input.
    fn pins_to_jtag(&mut self) {
        self.nreset.set_high(true);
        self.nreset.set_as_output();

        self.tms_swdio.set_high(true);
        self.tms_swdio.set_as_output();

        self.tck_swclk.set_as_output();

        self.tdi.set_as_output();
        self.tdo.set_as_input();
    }

    /// SWD pin roles on SWD-mode entry: SWDIO idle-high and driven, JTAG-only lines released.
    fn pins_to_swd(&mut self) {
        self.tms_swdio.set_high(true);
        self.tms_swdio.set_as_output();

        self.tck_swclk.set_as_output();

        self.nreset.set_as_input();
        self.tdi.set_as_input();
        self.tdo.set_as_input();
    }

    /// Read a bit sequence with SWDIO released, LSB-first; not a hot path, so always the spin.
    fn read_bit_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) {
        let r = self.regs;
        let h = SpinHalves {
            delay: self.delay,
            cycles: HalfCycles::symmetric(self.swd_timing.sequence_half_cycles()),
        };
        r.release_swdio();
        for byte in data.iter_mut() {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            let mut v: u32 = 0;
            for i in 0..bits {
                v |= bit_in(r, h) << i;
            }
            *byte = v as u8;
            num_bits -= bits;
        }
    }
}

/// The four TAP lines as single constant-mask register accesses, for the shared word scans.
impl<P: MaskedPacing> JtagPortOps for Masked<P> {
    fn jtag_pads_acquire(&mut self) {
        self.tdi.set_high(false);
        self.tdi.set_as_output();
        self.tdo.set_as_input();
    }

    fn jtag_pads_release(&mut self) {
        self.tdi.set_as_input();
    }

    #[inline(always)]
    fn tck_low(&self) {
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (self.regs.clr as *mut u32).write_volatile(SWCLK_MASK) };
    }

    #[inline(always)]
    fn tck_high(&self) {
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (self.regs.set as *mut u32).write_volatile(SWCLK_MASK) };
    }

    #[inline(always)]
    fn tms_set(&self, high: bool) {
        let addr = if high { self.regs.set } else { self.regs.clr };
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (addr as *mut u32).write_volatile(SWDIO_MASK) };
    }

    #[inline(always)]
    fn tdi_set(&self, high: bool) {
        let addr = if high { self.regs.set } else { self.regs.clr };
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (addr as *mut u32).write_volatile(TDI_MASK) };
    }

    #[inline(always)]
    fn tdo_get(&self) -> u32 {
        // SAFETY: resolved GPIO register; plain read (`PIN` is unaffected by `MASK`).
        (unsafe { (self.regs.pin as *const u32).read_volatile() } >> TDO_PIN) & 1
    }

    fn tdi_is_high(&mut self) -> bool {
        self.tdi.is_high()
    }

    fn tdo_is_high(&mut self) -> bool {
        self.tdo.is_high()
    }
}

/// Masked-GPIO dependencies: the DAP-engine `DEPS` type.
pub struct MaskedDeps<P: MaskedPacing>(Masked<P>);

/// Active masked-GPIO SWD driver: the specialized word transfers; sequences stay generic.
pub struct MaskedSwdDrv<P: MaskedPacing>(Masked<P>);

/// Active JTAG driver: generic shift mechanics plus the shared fast word scans.
pub struct MaskedJtagDrv<P: MaskedPacing>(Masked<P>);

impl<P: MaskedPacing> From<MaskedDeps<P>> for MaskedSwdDrv<P> {
    fn from(mut d: MaskedDeps<P>) -> Self {
        // SWD selected: restore the SWD pin roles and route `MPIN0` for the word path.
        d.0.pins_to_swd();
        d.0.regs.take_wire();
        MaskedSwdDrv(d.0)
    }
}
impl<P: MaskedPacing> From<MaskedSwdDrv<P>> for MaskedDeps<P> {
    fn from(s: MaskedSwdDrv<P>) -> Self {
        // Every SWD-mode exit releases `MASK0`, so no other mode ever sees `MPIN0` routed.
        s.0.regs.release_wire();
        MaskedDeps(s.0)
    }
}

impl<P: MaskedPacing> From<MaskedDeps<P>> for MaskedJtagDrv<P> {
    fn from(mut d: MaskedDeps<P>) -> Self {
        d.0.pins_to_jtag();
        MaskedJtagDrv(d.0)
    }
}
impl<P: MaskedPacing> From<MaskedJtagDrv<P>> for MaskedDeps<P> {
    fn from(mut j: MaskedJtagDrv<P>) -> Self {
        j.0.pins_to_swd();
        MaskedDeps(j.0)
    }
}

impl<P: MaskedPacing> Dependencies<MaskedSwdDrv<P>, MaskedJtagDrv<P>> for MaskedDeps<P> {
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        // nRESET's buffered turnaround is folded into its open-drain `IoPin`.
        self.0.bb.process_swj_pins(output, mask, wait_us)
    }
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.0.bb.process_swj_sequence(data, nbits)
    }
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        // Prime both timings, so a clock set before `DAP_Connect(JTAG)` carries over.
        let jtag_ok = self.0.set_jtag_clock(max_frequency);
        self.0.set_swd_clock(max_frequency) || jtag_ok
    }
    fn high_impedance_mode(&mut self) {
        // Parking must never leave `MPIN0` routed, however this mode was reached.
        self.0.regs.release_wire();
        self.0.bb.high_impedance_mode()
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        self.0.bb.swd_config()
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.0.bb.jtag_config()
    }
}

struct FastJtagScan<'a, P: MaskedPacing>(&'a mut Masked<P>);

impl<P: MaskedPacing> JtagScan for FastJtagScan<'_, P> {
    type Port = Masked<P>;

    #[inline(always)]
    fn scan_port(&self) -> &Self::Port {
        self.0
    }

    #[inline(always)]
    fn scan_config(&mut self) -> &jtag::Config {
        self.0.bb.jtag_config()
    }

    #[inline(always)]
    fn scan_wait_low(&mut self) {
        self.0.pacing.pace_low();
    }

    #[inline(always)]
    fn scan_wait_high(&mut self) {
        self.0.pacing.pace_high();
    }
}

struct ScaledJtagScan<'a, P: MaskedPacing> {
    inner: &'a mut Masked<P>,
    control: PreparedJtagHalves,
    data: PreparedJtagHalves,
}

/// One monomorphized DPACC/APACC scan in SRAMX; the scan body is forced into this entry.
#[inline(never)]
#[link_section = ".sramx_code.masked_jtag"]
fn jtag_transfer_word<S: JtagScan>(
    mut scan: S,
    r_nw: RnW,
    a2a3: u8,
    idle_cycles: u8,
    data: u32,
) -> TransferResult {
    scan.transfer_word(r_nw, a2a3, idle_cycles, data)
}

/// One monomorphized IR scan in SRAMX; the scan body is forced into this entry.
#[inline(never)]
#[link_section = ".sramx_code.masked_jtag"]
fn jtag_shift_ir_word<S: JtagScan>(mut scan: S, ir: u32, ir_length: u32, before: u32, after: u32) {
    scan.shift_ir_geometry(ir, ir_length, before, after);
}

impl<P: MaskedPacing> JtagScan for ScaledJtagScan<'_, P> {
    type Port = Masked<P>;

    #[inline(always)]
    fn scan_port(&self) -> &Self::Port {
        self.inner
    }

    #[inline(always)]
    fn scan_config(&mut self) -> &jtag::Config {
        self.inner.bb.jtag_config()
    }

    #[inline(always)]
    fn scan_wait_low(&mut self) {
        self.control.low.run();
    }

    #[inline(always)]
    fn scan_wait_high(&mut self) {
        self.control.high.run();
    }

    #[inline(always)]
    fn scan_wait_data_low(&mut self) {
        self.data.low.run();
    }

    #[inline(always)]
    fn scan_wait_data_high(&mut self) {
        self.data.high.run();
    }
}

impl<'a, P: MaskedPacing> ScaledJtagScan<'a, P> {
    fn new(
        inner: &'a mut Masked<P>,
        control: PreparedJtagHalves,
        data: PreparedJtagHalves,
    ) -> Self {
        Self {
            inner,
            control,
            data,
        }
    }
}

impl<P: MaskedPacing> Jtag<MaskedDeps<P>> for MaskedJtagDrv<P> {
    fn available(_deps: &MaskedDeps<P>) -> bool {
        true
    }

    fn config(&mut self) -> &mut jtag::Config {
        self.0.bb.jtag_config()
    }

    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], mut rxbuf: &mut [u8]) {
        // TMS is fixed for the whole sequence (dap-rs breaks TAP walks into `tms_sequence`).
        self.0.tms_set(info.tms);
        let mut n_bits = info.n_bits;
        for &byte in tdi {
            let bits = n_bits.min(8) as usize;
            n_bits -= bits as u8;
            let tdo = self.0.jtag_shift(byte as u32, bits);
            if info.capture && !rxbuf.is_empty() {
                rxbuf[0] = tdo as u8;
                rxbuf = &mut rxbuf[1..];
            }
        }
    }

    fn tms_sequence(&mut self, tms: &[bool]) {
        self.0.tms_walk(tms);
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_jtag_clock(max_frequency)
    }

    /// The specialized DPACC/APACC scan; `write_abort`'s provided implementation bypasses this
    /// method and keeps the generic mechanics.
    #[inline(never)]
    #[link_section = ".sramx_code.masked_jtag"]
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let pacing = match r_nw {
            RnW::R => self.0.jtag_timing.read,
            RnW::W => self.0.jtag_timing.write,
        };
        let control = self.0.jtag_timing.control;
        match pacing {
            PreparedJtagPacing::Fast => jtag_transfer_word(
                FastJtagScan(&mut self.0),
                r_nw,
                a2a3,
                transfer_config.idle_cycles,
                data,
            ),
            PreparedJtagPacing::Scaled(cycles) => jtag_transfer_word(
                ScaledJtagScan::new(&mut self.0, control, cycles),
                r_nw,
                a2a3,
                transfer_config.idle_cycles,
                data,
            ),
        }
    }

    /// The specialized IR scan in place of the provided per-chunk `shift_ir`.
    fn shift_ir(&mut self, ir: u32) {
        let tap = self.0.bb.jtag_config().current_tap();
        let ir_length = tap.ir_length as u32;
        let before = tap.ir_before as u32;
        let after = tap.ir_after as u32;
        let control = self.0.jtag_timing.control;
        match self.0.jtag_timing.ir {
            PreparedJtagPacing::Fast => {
                jtag_shift_ir_word(FastJtagScan(&mut self.0), ir, ir_length, before, after)
            }
            PreparedJtagPacing::Scaled(cycles) => jtag_shift_ir_word(
                ScaledJtagScan::new(&mut self.0, control, cycles),
                ir,
                ir_length,
                before,
                after,
            ),
        }
    }
}

impl<P: MaskedPacing> Swd<MaskedDeps<P>> for MaskedSwdDrv<P> {
    fn available(_deps: &MaskedDeps<P>) -> bool {
        true
    }
    fn config(&mut self) -> &mut swd::Config {
        self.0.bb.swd_config()
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.0.read_transfer(apndp, a)
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        self.0.write_transfer(apndp, a, data)
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_swd_clock(max_frequency)
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        // A direction-out sequence is byte-identical to a SWJ sequence; `MASK0` never affects it.
        self.0.bb.process_swj_sequence(data, num_bits);
        Ok(())
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.0.read_bit_sequence(num_bits, data);
        Ok(())
    }
}

/// Build the masked-GPIO dependencies over the MCU-Link pin map with the injected delay and
/// pacing. Owns its scan-chain `'static`: callable only once per image.
pub fn masked_deps<P: MaskedPacing>(delay: DynCycleDelay, pacing: P) -> MaskedDeps<P> {
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    MaskedDeps(Masked::new(delay, pacing, &SCAN))
}

/// The SWD+JTAG DAP engine over the masked-GPIO backend.
pub type MaskedSwdDap<
    'a,
    P,
    SWO = daplink_rs::NoSwo,
    V = daplink_rs::daplink_core::vendor::NoVendor,
> = daplink_rs::DapProbe<
    'a,
    MaskedDeps<P>,
    NoopLeds,
    DynCycleDelay,
    MaskedJtagDrv<P>,
    MaskedSwdDrv<P>,
    SWO,
    V,
>;
