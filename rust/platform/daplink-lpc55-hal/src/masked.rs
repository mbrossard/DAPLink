//! Masked-GPIO SWD and JTAG: `MASK0` routes `MPIN0` to SWDIO; one store drives, one load samples.
// Sections: pins and registers, pacing, SWD bodies, SWD transfer blocks, JTAG lines and scans,
// SRAMX scan entries, engine, drivers; host tests last.

use daplink_core::dap_wire::{
    encode_block_response, transfer_idle_cycles, transfer_wait_attempts, BlockRequest,
};
use daplink_rs::dap_rs::dap::TransferConfig;
use daplink_rs::dap_rs::jtag::{self, Jtag, SequenceInfo, TransferResult};
use daplink_rs::dap_rs::swd::{self, APnDP, DPRegister, RnW, Swd};
use daplink_rs::dap_rs::swj::{Dependencies, Pins};
use daplink_rs::daplink_core::{JtagDp, JtagDpSource, SCAN_CHAIN_SLOTS};
use daplink_rs::{wait_retry, wired_deps, NoopLeds, ScanChainStorage, WiredAdapter};
use daplink_rs::{DelayCycles, InputOutputPin};
use daplink_spi_swd::nops;
use daplink_spi_swd::session::{parity32, request_byte};
use daplink_spi_swd::{
    prepare_jtag_timing, prepare_swd_timing, FrequencyPacing, HalfCycleProfile, HalfCycles,
    JtagPacing, JtagPortOps, JtagScan, JtagTimingPlan, KeptAck, PacingPlan, PacingProfile,
    SwdTimingPlan,
};
use lpc55_hal::raw;

use core::num::NonZeroU32;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::pin::{IoPin, SWDIO_PIN, TCK_PIN, TDI_PIN, TDO_PIN, TXEN_PIN};
use crate::DynCycleDelay;

// Pins and registers.

const SWDIO_MASK: u32 = 1 << SWDIO_PIN;
const TXEN_MASK: u32 = 1 << TXEN_PIN;
const SWCLK_MASK: u32 = 1 << TCK_PIN;
const TDI_MASK: u32 = 1 << TDI_PIN;
/// Port-0 byte pin registers (UM11126 16.5.1): a write loads bit 0, a read returns 0 or 1.
const B0: *mut u8 = raw::GPIO::PTR.cast::<u8>().cast_mut();
const _: () = assert!(core::mem::offset_of!(raw::gpio::RegisterBlock, b) == 0);

fn tdi_pin() -> IoPin {
    IoPin::output(0, TDI_PIN as u8)
}

fn tdo_pin() -> IoPin {
    IoPin::input(0, TDO_PIN as u8)
}

/// Write bit 0 of `v` to a port-0 byte pin register; the other bits are ignored.
#[inline(always)]
fn byte_put(pin: u32, v: u32) {
    // SAFETY: fixed GPIO byte pin register; plain data write.
    unsafe { B0.add(pin as usize).write_volatile(v as u8) };
}

/// Read a port-0 byte pin register: the pin level as 0 or 1.
#[inline(always)]
fn byte_get(pin: u32) -> u32 {
    // SAFETY: fixed GPIO byte pin register; plain read.
    unsafe { B0.add(pin as usize).read_volatile() as u32 }
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
        #[cfg(test)]
        {
            protocol_tests::mask_taken();
        }
        #[cfg(not(test))]
        // SAFETY: resolved GPIO register; plain data write.
        unsafe {
            (self.mask as *mut u32).write_volatile(!SWDIO_MASK)
        };
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

// Codegen pins: every clock cell's length and fetch alignment is fixed, not left to LLVM.

/// `v` through an empty asm block, so LLVM cannot fold or rematerialize it.
// Only the host gate's wider registers make the operand width worth spelling out.
#[allow(asm_sub_register)]
#[inline(always)]
fn in_register(mut v: u32) -> u32 {
    // SAFETY: an empty asm block; `v` passes through a register unchanged.
    unsafe {
        core::arch::asm!(
            "/* {0} */",
            inout(reg) v,
            options(nomem, nostack, preserves_flags)
        );
    }
    v
}

/// A 32-bit `str.w` of `v` to `addr`: a TCK edge's length never depends on register choice.
#[inline(always)]
fn store_word(addr: usize, v: u32) {
    // SAFETY: a GPIO register the caller names; one plain store.
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!(
            "str.w {v}, [{a}]",
            v = in(reg) v,
            a = in(reg) addr,
            options(nostack, preserves_flags)
        );
    }
    // SAFETY: as above.
    #[cfg(not(target_arch = "arm"))]
    unsafe {
        (addr as *mut u32).write_volatile(v)
    };
}

/// [`store_word`] starting a fetch word, whatever code precedes it.
#[inline(always)]
fn store_word_aligned(addr: usize, v: u32) {
    // SAFETY: as [`store_word`].
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!(
            ".p2align 2",
            "str.w {v}, [{a}]",
            v = in(reg) v,
            a = in(reg) addr,
            options(nostack, preserves_flags)
        );
    }
    #[cfg(not(target_arch = "arm"))]
    store_word(addr, v);
}

/// Expand `$cell` once per data bit as straight-line cells; a counted loop would branch in each.
macro_rules! cells {
    (31, $i:tt => $cell:expr) => {
        cells!(@ $i $cell; 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
            16 17 18 19 20 21 22 23 24 25 26 27 28 29 30)
    };
    (32, $i:tt => $cell:expr) => {
        cells!(@ $i $cell; 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
            16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31)
    };
    (@ $i:tt $cell:expr; $($n:literal)*) => {
        $({
            let $i = $n;
            $cell;
        })*
    };
}

// Pacing.

/// The injected accel pacing: SWD transfer halves on top of the fast-JTAG ladder.
pub trait MaskedPacing: JtagPacing + Copy {
    /// Half-period spin count at or below which the fast SWD word routines run.
    const SWD_FAST_HALF_CYCLES: u32;

    /// Minimum residual spin for each scaled SWD half; the default is one spin iteration.
    const SWD_SCALED_FLOOR_CYCLES: u32 = SPIN_CYCLES;
    /// Fixed scaled-path work in request clocks, as low/high cycles.
    const SWD_CONTROL_FIXED_CYCLES: (u32, u32) = (0, 0);
    /// Fixed scaled-path work in ACK and turnaround clocks, as low/high cycles.
    const SWD_ACK_FIXED_CYCLES: (u32, u32) = (0, 0);
    /// Fixed scaled-path work in read data clocks, as low/high cycles.
    const SWD_READ_FIXED_CYCLES: (u32, u32) = (0, 0);
    /// Fixed scaled-path work in write data clocks, as low/high cycles.
    const SWD_WRITE_FIXED_CYCLES: (u32, u32) = (0, 0);

    /// Request frequency at or above which fast JTAG scans run; zero keeps every scan scaled.
    const JTAG_FAST_MIN_HZ: u32;

    /// Minimum half-period for generic JTAG sequences.
    const JTAG_SEQUENCE_FLOOR_CYCLES: u32;

    /// Shortest scaled JTAG low half (TCK fall to TDO sample), in picoseconds.
    const JTAG_LOW_FLOOR_PS: u32;
    /// Shortest scaled JTAG high half, in picoseconds.
    const JTAG_HIGH_FLOOR_PS: u32;
    /// Shortest scaled JTAG period, in picoseconds.
    const JTAG_PERIOD_FLOOR_PS: u32;

    /// Fixed scaled-path work for TAP-state clocks, as low/high core cycles.
    const JTAG_CONTROL_FIXED_CYCLES: (u32, u32);
    /// Fixed scaled-path work for ACK clocks, as low/high core cycles.
    const JTAG_ACK_FIXED_CYCLES: (u32, u32);
    /// Fixed scaled-path work for read data clocks, as low/high core cycles.
    const JTAG_READ_FIXED_CYCLES: (u32, u32);
    /// Fixed scaled-path work for write data clocks, as low/high core cycles.
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

/// How a transfer paces each half of its cells, per clock class; chosen once per transfer.
trait ClockHalves: Copy {
    /// Spin-paced halves take the straight-line SWD data phase; nop-paced ones the grouped one.
    const PREPARED: bool = false;

    fn wait_low(self);

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

    /// Pace the low half of a sampled control clock (ACK, turnaround), defaulting to control.
    #[inline(always)]
    fn wait_ack_low(self) {
        self.wait_low();
    }

    /// Pace the high half of a sampled control clock, defaulting to control.
    #[inline(always)]
    fn wait_ack_high(self) {
        self.wait_high();
    }
}

/// Fast SWD read pacing: the injected nop blocks at or below the fast threshold.
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

/// Fast SWD write pacing: the injected nop blocks at or below the fast threshold.
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

/// Fast JTAG pacing: the injected nop ladder on every clock class.
#[derive(Clone, Copy)]
struct JtagPacedHalves<P: MaskedPacing>(P);

impl<P: MaskedPacing> ClockHalves for JtagPacedHalves<P> {
    #[inline(always)]
    fn wait_low(self) {
        self.0.pace_low();
    }

    #[inline(always)]
    fn wait_high(self) {
        self.0.pace_high();
    }
}

/// Sequence pacing: the injected cycle-counting delay, one count for every half.
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

/// Core cycles one `subs`/`bne` spin iteration costs in SRAMX.
const SPIN_CYCLES: u32 = 3;

/// A prepared spin count: word-aligned, so one fetch per iteration and no 16-byte pad.
#[derive(Clone, Copy)]
struct Spin(u32);

impl Spin {
    #[inline(always)]
    fn run(self) {
        let i = self.0;
        // SAFETY: registers only; the count is at least one; flags are clobbered.
        #[cfg(target_arch = "arm")]
        unsafe {
            core::arch::asm!(
                ".p2align 2",
                "2:",
                "subs {i}, #1",
                "bne 2b",
                i = inout(reg) i => _,
                options(nomem, nostack)
            );
        }
        #[cfg(not(target_arch = "arm"))]
        for _ in 0..i {
            core::hint::spin_loop();
        }
    }
}

/// Scaled pacing: prepared spins for each clock class.
#[derive(Clone, Copy)]
struct PreparedHalves {
    ctl_low: Spin,
    ctl_high: Spin,
    ack_low: Spin,
    ack_high: Spin,
    data_low: Spin,
    data_high: Spin,
}

impl ClockHalves for PreparedHalves {
    const PREPARED: bool = true;

    #[inline(always)]
    fn wait_low(self) {
        self.ctl_low.run();
    }

    #[inline(always)]
    fn wait_high(self) {
        self.ctl_high.run();
    }

    #[inline(always)]
    fn wait_data_low(self) {
        self.data_low.run();
    }

    #[inline(always)]
    fn wait_data_high(self) {
        self.data_high.run();
    }

    #[inline(always)]
    fn wait_ack_low(self) {
        self.ack_low.run();
    }

    #[inline(always)]
    fn wait_ack_high(self) {
        self.ack_high.run();
    }
}

/// Round the period, not each half: the low half keeps its floored margin, the high half the rest.
const fn split(nominal: u32, fixed: (u32, u32), floor: u32) -> (Spin, Spin) {
    let low = max_u32(nominal.saturating_sub(fixed.0), floor);
    let high = max_u32(nominal.saturating_sub(fixed.1), floor);
    let it_low = max_u32(low.div_ceil(SPIN_CYCLES), 1);
    let it_high = max_u32((low + high).div_ceil(SPIN_CYCLES).saturating_sub(it_low), 1);
    (Spin(it_low), Spin(it_high))
}

const fn max_u32(a: u32, b: u32) -> u32 {
    if a > b {
        a
    } else {
        b
    }
}

/// Prepare one direction's halves from the nominal half-period and the pacing's fixed-work tuples.
fn prepare_halves<P: MaskedPacing>(nominal: u32, data_fixed: (u32, u32)) -> PreparedHalves {
    let floor = P::SWD_SCALED_FLOOR_CYCLES;
    let (ctl_low, ctl_high) = split(nominal, P::SWD_CONTROL_FIXED_CYCLES, floor);
    let (ack_low, ack_high) = split(nominal, P::SWD_ACK_FIXED_CYCLES, floor);
    let (data_low, data_high) = split(nominal, data_fixed, floor);
    PreparedHalves {
        ctl_low,
        ctl_high,
        ack_low,
        ack_high,
        data_low,
        data_high,
    }
}

/// The scaled halves for a direction's plan; the fast plan gets a never-run placeholder.
fn prepared_for<P: MaskedPacing>(plan: PacingPlan, data_fixed: (u32, u32)) -> PreparedHalves {
    match plan {
        PacingPlan::Fast => prepare_halves::<P>(1, data_fixed),
        PacingPlan::Scaled(cycles) => prepare_halves::<P>(cycles.low(), data_fixed),
    }
}

/// Prepared spin iterations for one scaled JTAG clock class.
#[derive(Clone, Copy)]
struct JtagHalves {
    low: NonZeroU32,
    high: NonZeroU32,
}

/// One JTAG clock class's pacing: the ladder, or its prepared spins.
#[derive(Clone, Copy)]
enum PreparedJtagPacing {
    Fast,
    Scaled(JtagHalves),
}

/// Every JTAG clock class prepared at the last clock change.
#[derive(Clone, Copy)]
struct PreparedJtagTiming {
    sequence_half: u32,
    control: JtagHalves,
    ack: JtagHalves,
    read: PreparedJtagPacing,
    write: PreparedJtagPacing,
    ir: PreparedJtagPacing,
}

/// `ps` rounded up to whole core cycles.
const fn ps_cycles(ps: u32, cpu_hz: u32) -> u32 {
    (ps as u64 * cpu_hz as u64).div_ceil(1_000_000_000_000) as u32
}

/// Target (low, high) core cycles: the whole request period split once, then floored.
const fn jtag_targets(cpu_hz: u32, request: u32, floors_ps: (u32, u32, u32)) -> (u32, u32) {
    let period = cpu_hz.div_ceil(if request == 0 { 1 } else { request });
    let low = max_u32(period.div_ceil(2), ps_cycles(floors_ps.0, cpu_hz));
    let high = max_u32(period.saturating_sub(low), ps_cycles(floors_ps.1, cpu_hz));
    let short = ps_cycles(floors_ps.2, cpu_hz).saturating_sub(low + high);
    (low, high + short)
}

/// Whole spin iterations covering `target` less `fixed`, at least one.
const fn spin_for(target: u32, fixed: u32) -> NonZeroU32 {
    match NonZeroU32::new(target.saturating_sub(fixed).div_ceil(SPIN_CYCLES)) {
        Some(n) => n,
        None => NonZeroU32::MIN,
    }
}

const fn jtag_halves(target: (u32, u32), fixed: (u32, u32)) -> JtagHalves {
    JtagHalves {
        low: spin_for(target.0, fixed.0),
        high: spin_for(target.1, fixed.1),
    }
}

/// `cycles` in picoseconds, rounded down.
const fn cycles_ps(cycles: u64, cpu_hz: u32) -> u64 {
    cycles * 1_000_000_000_000 / cpu_hz as u64
}

/// Core cycles each scaled SWD class spends outside its spins: request, ACK, read, write.
const SWD_CLASS_WORK: [u32; 4] = [1, 0, 3, 3];

/// Fail the build if `P` plans any scaled cell at `cpu_hz`, down to 100 kHz, faster than requested.
pub const fn verify_fit<P: MaskedPacing>(cpu_hz: u32) {
    let swd = [
        P::SWD_CONTROL_FIXED_CYCLES,
        P::SWD_ACK_FIXED_CYCLES,
        P::SWD_READ_FIXED_CYCLES,
        P::SWD_WRITE_FIXED_CYCLES,
    ];
    let profile = PacingProfile::new(P::SWD_FAST_HALF_CYCLES, HalfCycleProfile::uncalibrated());
    let mut n = 1;
    while n <= cpu_hz / 2 / 100_000 {
        let request = (cpu_hz / 2).div_ceil(n);
        if let Some(timing) = prepare_swd_timing(cpu_hz, request, profile, profile) {
            if let PacingPlan::Scaled(c) = timing.write() {
                let mut class = 0;
                while class < swd.len() {
                    let (low, high) = split(c.low(), swd[class], P::SWD_SCALED_FLOOR_CYCLES);
                    let cycles = (low.0 + high.0) * SPIN_CYCLES + SWD_CLASS_WORK[class];
                    assert!(
                        cycles as u64 * request as u64 >= cpu_hz as u64,
                        "a scaled SWD cell runs fast"
                    );
                    class += 1;
                }
            }
        }
        n += 1;
    }

    let jtag = [
        P::JTAG_CONTROL_FIXED_CYCLES,
        P::JTAG_ACK_FIXED_CYCLES,
        P::JTAG_READ_FIXED_CYCLES,
        P::JTAG_WRITE_FIXED_CYCLES,
        P::JTAG_IR_FIXED_CYCLES,
    ];
    let floors = (
        P::JTAG_LOW_FLOOR_PS,
        P::JTAG_HIGH_FLOOR_PS,
        P::JTAG_PERIOD_FLOOR_PS,
    );
    let mut period = 2;
    while period <= cpu_hz / 100_000 {
        // The fastest request whose period rounds up to `period`.
        let request = (cpu_hz - 1) / (period - 1);
        if P::JTAG_FAST_MIN_HZ == 0 || request < P::JTAG_FAST_MIN_HZ {
            let target = jtag_targets(cpu_hz, request, floors);
            let mut class = 0;
            while class < jtag.len() {
                let fixed = jtag[class];
                let h = jtag_halves(target, fixed);
                let low = (h.low.get() * SPIN_CYCLES + fixed.0) as u64;
                let high = (h.high.get() * SPIN_CYCLES + fixed.1) as u64;
                assert!(
                    (low + high) * request as u64 >= cpu_hz as u64,
                    "a scaled JTAG cell runs fast"
                );
                assert!(
                    cycles_ps(low, cpu_hz) >= floors.0 as u64,
                    "a scaled JTAG low half is short"
                );
                assert!(
                    cycles_ps(high, cpu_hz) >= floors.1 as u64,
                    "a scaled JTAG high half is short"
                );
                assert!(
                    cycles_ps(low + high, cpu_hz) >= floors.2 as u64,
                    "a scaled JTAG period is short"
                );
                class += 1;
            }
        }
        period += 1;
    }
}

impl PreparedJtagTiming {
    fn new<P: MaskedPacing>(plan: JtagTimingPlan, cpu_hz: u32, request: u32) -> Self {
        let floors = (
            P::JTAG_LOW_FLOOR_PS,
            P::JTAG_HIGH_FLOOR_PS,
            P::JTAG_PERIOD_FLOOR_PS,
        );
        let target = jtag_targets(cpu_hz, request, floors);
        let class = |plan: PacingPlan, fixed| match plan {
            PacingPlan::Fast => PreparedJtagPacing::Fast,
            PacingPlan::Scaled(_) => PreparedJtagPacing::Scaled(jtag_halves(target, fixed)),
        };
        Self {
            sequence_half: plan.sequence_half_cycles(),
            control: jtag_halves(target, P::JTAG_CONTROL_FIXED_CYCLES),
            ack: jtag_halves(target, P::JTAG_ACK_FIXED_CYCLES),
            read: class(plan.read(), P::JTAG_READ_FIXED_CYCLES),
            write: class(plan.write(), P::JTAG_WRITE_FIXED_CYCLES),
            ir: class(plan.ir(), P::JTAG_IR_FIXED_CYCLES),
        }
    }

    /// A scaled scan's halves: control and ACK as prepared, `data` for the scan's own class.
    #[inline(always)]
    fn scaled_halves(&self, data: JtagHalves) -> PreparedHalves {
        let spin = |h: JtagHalves| (Spin(h.low.get()), Spin(h.high.get()));
        let (ctl_low, ctl_high) = spin(self.control);
        let (ack_low, ack_high) = spin(self.ack);
        let (data_low, data_high) = spin(data);
        PreparedHalves {
            ctl_low,
            ctl_high,
            ack_low,
            ack_high,
            data_low,
            data_high,
        }
    }
}

// SWD bodies.

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
    h.wait_ack_low();
    let bit = r.swdio_in() >> SWDIO_PIN;
    r.clk_high();
    h.wait_ack_high();
    bit
}

/// One outgoing data bit on the data-phase pacing.
#[inline(always)]
fn data_bit_out<H: ClockHalves>(r: Regs, h: H, bit: u32) {
    r.swdio_out(bit);
    r.clk_low();
    h.wait_data_low();
    r.clk_high();
    h.wait_data_high();
}

/// One incoming data bit on the data-phase pacing.
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
    h.wait_ack_low();
    r.clk_high();
    h.wait_ack_high();
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

// Const checks compare the shift parity with parity32.
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

/// One read data cell: the sample enters at bit 31 and shifts down; `late` samples a nop later.
#[inline(always)]
fn read_cell<H: ClockHalves>(r: Regs, h: H, data: u32, late: bool) -> u32 {
    r.clk_low();
    h.wait_data_low();
    if late {
        nops::<1>();
    }
    let data = (data >> 1) | (r.swdio_in() << (31 - SWDIO_PIN));
    r.clk_high();
    h.wait_data_high();
    data
}

/// The 32 read data cells: straight-line on prepared spins, else eight counted groups of four.
#[inline(always)]
fn read_data<H: ClockHalves>(r: Regs, h: H, mut data: u32) -> u32 {
    if H::PREPARED {
        cells!(32, _ => data = read_cell(r, h, data, false));
        return data;
    }
    let mut groups = in_register(8);
    while groups != 0 {
        data = read_cell(r, h, data, false);
        data = read_cell(r, h, data, true);
        data = read_cell(r, h, data, false);
        data = read_cell(r, h, data, true);
        groups -= 1;
    }
    data
}

/// The 32 write data cells, shaped as [`read_data`].
#[inline(always)]
fn write_data<H: ClockHalves>(r: Regs, h: H, mut shift: WriteShift) -> WriteShift {
    if H::PREPARED {
        cells!(32, _ => data_bit_out(r, h, shift.next_bit()));
        return shift;
    }
    let mut groups = in_register(8);
    while groups != 0 {
        data_bit_out(r, h, shift.next_bit());
        data_bit_out(r, h, shift.next_bit());
        data_bit_out(r, h, shift.next_bit());
        data_bit_out(r, h, shift.next_bit());
        groups -= 1;
    }
    shift
}

/// Shift an SWD request out LSB-first.
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

// Word bodies are macros so the SRAMX entries and block loops expand the same text: behind an
// inlined call, LLVM moves the write's return value into its last clock cell.

/// One SWD read transfer: request, turnaround, ACK, data and parity.
macro_rules! read_word_body {
    ($r:ident, $h:ident, $req:ident) => {{
        $r.drive_swdio();
        request_out($r, $h, $req);
        $r.release_swdio();
        clock_only($r, $h);

        let ack = bit_in($r, $h) | bit_in($r, $h) << 1 | bit_in($r, $h) << 2;
        if let Err(e) = swd::Ack::try_ok(ack as u8) {
            clock_only($r, $h);
            return Err(e);
        }

        let data = read_data($r, $h, 0);
        let parity = data_bit_in($r, $h);
        clock_only($r, $h);
        if parity != parity32(data) as u32 {
            return Err(swd::Error::BadParity);
        }

        Ok(data)
    }};
}

/// One SWD write transfer: request, turnaround, ACK, turnaround, data and parity.
macro_rules! write_word_body {
    ($r:ident, $h:ident, $req:ident, $data:ident) => {{
        $r.drive_swdio();
        request_out($r, $h, $req);
        $r.release_swdio();
        clock_only($r, $h);

        let ack = bit_in($r, $h) | bit_in($r, $h) << 1 | bit_in($r, $h) << 2;
        clock_only($r, $h);
        swd::Ack::try_ok(ack as u8)?;

        $r.drive_swdio();
        let shift = write_data($r, $h, WriteShift::new($data));
        data_bit_out($r, $h, shift.parity());

        Ok(())
    }};
}

/// One SWD read transfer in SRAMX, paced by `H` (nop-paced fast or prepared spins).
#[inline(never)]
#[link_section = ".sramx_code"]
fn read_word_fast<H: ClockHalves>(r: Regs, h: H, req: u8) -> swd::Result<u32> {
    read_word_body!(r, h, req)
}

/// One SWD write transfer in SRAMX, paced by `H`.
#[inline(never)]
#[link_section = ".sramx_code"]
fn write_word_fast<H: ClockHalves>(r: Regs, h: H, req: u8, data: u32) -> swd::Result<()> {
    write_word_body!(r, h, req, data)
}

/// A block's read, the word body inlined into the loop; host tests script the target here.
#[inline(always)]
fn block_read<H: ClockHalves>(r: Regs, h: H, req: u8) -> swd::Result<u32> {
    #[cfg(test)]
    {
        let _ = (r, h);
        protocol_tests::read(req)
    }
    #[cfg(not(test))]
    read_word_body!(r, h, req)
}

/// A block's write, as [`block_read`].
#[inline(always)]
fn block_write<H: ClockHalves>(r: Regs, h: H, req: u8, data: u32) -> swd::Result<()> {
    #[cfg(test)]
    {
        let _ = (r, h);
        protocol_tests::write(req, data)
    }
    #[cfg(not(test))]
    write_word_body!(r, h, req, data)
}

/// The same transfer up to `attempts` times while it answers WAIT, `AckWait` if they run out.
#[inline(always)]
fn retry<T>(attempts: usize, mut transfer: impl FnMut() -> swd::Result<T>) -> swd::Result<T> {
    let mut left = attempts;
    let mut res = Err(swd::Error::AckWait);
    while left != 0 {
        res = transfer();
        if !matches!(res, Err(swd::Error::AckWait)) {
            break;
        }
        left -= 1;
    }
    res
}

/// How a block ended: the transfers run, a failed one included, the status byte and the words.
struct BlockEnd {
    ran: u16,
    status: u8,
    words: usize,
}

/// The transfer status byte as dap-rs encodes it.
#[inline(always)]
fn status_byte(e: swd::Error) -> u8 {
    match e {
        swd::Error::AckWait => 2,
        swd::Error::AckFault => 4,
        swd::Error::NoAck => 7,
        _ => (1 << 3) | 7,
    }
}

/// Store a read word in its response slot; a failed read ends the block, its slot untouched.
macro_rules! retire_read {
    ($result:expr, $slot:expr, $done:expr) => {
        match $result {
            Ok(v) => {
                let b = v.to_le_bytes();
                let slot = $slot;
                slot[0] = b[0];
                slot[1] = b[1];
                slot[2] = b[2];
                slot[3] = b[3];
            }
            Err(e) => {
                return BlockEnd {
                    ran: ($done + 1) as u16,
                    status: status_byte(e),
                    words: $done,
                };
            }
        }
    };
}

/// A whole `DAP_TransferBlock` read in SRAMX; an AP read block is posted, ending on DP RDBUFF.
#[inline(never)]
#[link_section = ".sramx_code"]
#[allow(clippy::too_many_arguments)] // Keep the SRAMX entry's transfer state explicit.
fn read_block_fast<H: ClockHalves>(
    r: Regs,
    h: H,
    req: u8,
    last: u8,
    posted: bool,
    count: usize,
    attempts: usize,
    out: &mut [u8],
) -> BlockEnd {
    if posted {
        if let Err(e) = retry(attempts, || block_read(r, h, req)) {
            return BlockEnd {
                ran: 1,
                status: status_byte(e),
                words: 0,
            };
        }
    }
    let mut done = 0usize;
    let regular = count - usize::from(posted);
    for slot in out[4..4 + 4 * regular].chunks_exact_mut(4) {
        retire_read!(retry(attempts, || block_read(r, h, req)), slot, done);
        done += 1;
    }
    if posted {
        let result = retry(attempts, || block_read(r, h, last));
        retire_read!(result, &mut out[4 + 4 * done..8 + 4 * done], done);
    }
    BlockEnd {
        ran: count as u16,
        status: 1,
        words: count,
    }
}

/// A whole `DAP_TransferBlock` write in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code"]
fn write_block_fast<H: ClockHalves>(
    r: Regs,
    h: H,
    req: u8,
    attempts: usize,
    data: &[u8],
) -> BlockEnd {
    let mut done = 0usize;
    for chunk in data.chunks_exact(4) {
        let v = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        if let Err(e) = retry(attempts, || block_write(r, h, req, v)) {
            return BlockEnd {
                ran: (done + 1) as u16,
                status: status_byte(e),
                words: 0,
            };
        }
        done += 1;
    }
    BlockEnd {
        ran: done as u16,
        status: 1,
        words: 0,
    }
}

// SWD transfer blocks.

/// Whether dap-rs currently holds the SWD driver, published by the mode transitions below.
static SWD_ACTIVE: AtomicBool = AtomicBool::new(false);

/// The prepared SWD read and write halves (low, high), zero meaning the fast plan.
static SWD_PLAN: [AtomicU32; 4] = [const { AtomicU32::new(0) }; 4];

fn publish_swd_plan(timing: SwdTimingPlan) {
    let encode = |plan: PacingPlan| match plan {
        PacingPlan::Fast => (0, 0),
        PacingPlan::Scaled(c) => (c.low(), c.high()),
    };
    let (rl, rh) = encode(timing.read());
    let (wl, wh) = encode(timing.write());
    SWD_PLAN[0].store(rl, Ordering::Relaxed);
    SWD_PLAN[1].store(rh, Ordering::Relaxed);
    SWD_PLAN[2].store(wl, Ordering::Relaxed);
    SWD_PLAN[3].store(wh, Ordering::Relaxed);
}

fn published_swd_plan(write: bool) -> PacingPlan {
    let i = if write { 2 } else { 0 };
    let low = SWD_PLAN[i].load(Ordering::Relaxed);
    let high = SWD_PLAN[i + 1].load(Ordering::Relaxed);
    if low == 0 {
        PacingPlan::Fast
    } else {
        PacingPlan::Scaled(HalfCycles::new(low, high))
    }
}

/// `DAP_TransferBlock` ahead of dap-rs, as one SRAMX loop.
pub struct MaskedBlock<P: MaskedPacing> {
    pacing: P,
    wait_attempts: usize,
    idle_cycles: u8,
}

impl<P: MaskedPacing> MaskedBlock<P> {
    /// dap-rs's default of five WAIT retries until a `DAP_TransferConfigure` arrives.
    pub const fn new(pacing: P) -> Self {
        Self {
            pacing,
            wait_attempts: 6,
            idle_cycles: 0,
        }
    }

    /// Handle a report ahead of the engine; `None` leaves it to dap-rs unchanged.
    /// Call it on every report, so it mirrors each `DAP_TransferConfigure` the engine sees.
    pub fn process(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        match *report.first()? {
            // Mirror the host's WAIT retries as attempts and its idle cycles; dap-rs still answers.
            0x04 => {
                if let Some(attempts) = transfer_wait_attempts(report) {
                    self.wait_attempts = attempts;
                }
                if let Some(idle) = transfer_idle_cycles(report) {
                    self.idle_cycles = idle;
                }
                None
            }
            // SRAMX has no room for words that clock idle cycles; dap-rs's driver clocks them.
            0x06 if SWD_ACTIVE.load(Ordering::Relaxed) && self.idle_cycles == 0 => {
                self.transfer_block(report, response)
            }
            _ => None,
        }
    }

    fn transfer_block(&self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        if report.len() < 5 || response.len() < 4 || report[2..4] == [0, 0] {
            return None;
        }
        // Keep MASK0 acquisition before the payload/capacity fallback checks.
        let r = Regs::new();
        r.take_wire();
        let BlockRequest {
            count,
            request: treq,
        } = BlockRequest::parse(report, response.len())?;
        let ap = treq & 1 != 0;
        let read = treq & 2 != 0;
        let port = if ap { APnDP::AP } else { APnDP::DP };
        let addr = DPRegister::try_from((treq >> 2) & 3).ok()?;
        let attempts = self.wait_attempts;
        let end = if read {
            let req = request_byte(port, RnW::R, addr);
            let last = if ap {
                request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF)
            } else {
                req
            };
            match published_swd_plan(false) {
                PacingPlan::Fast => read_block_fast(
                    r,
                    ReadPacedHalves(self.pacing),
                    req,
                    last,
                    ap,
                    count,
                    attempts,
                    response,
                ),
                PacingPlan::Scaled(cycles) => read_block_fast(
                    r,
                    prepare_halves::<P>(cycles.low(), P::SWD_READ_FIXED_CYCLES),
                    req,
                    last,
                    ap,
                    count,
                    attempts,
                    response,
                ),
            }
        } else {
            let req = request_byte(port, RnW::W, addr);
            let data = &report[5..5 + 4 * count];
            match published_swd_plan(true) {
                PacingPlan::Fast => {
                    write_block_fast(r, WritePacedHalves(self.pacing), req, attempts, data)
                }
                PacingPlan::Scaled(cycles) => write_block_fast(
                    r,
                    prepare_halves::<P>(cycles.low(), P::SWD_WRITE_FIXED_CYCLES),
                    req,
                    attempts,
                    data,
                ),
            }
        };
        Some(encode_block_response(
            response, end.ran, end.status, end.words,
        ))
    }
}

// JTAG lines and scans.

/// How the TAP lines are driven, each access one store or load.
trait Lines: Copy {
    fn tck_low(self);

    /// TCK falling edge opening a read data cell.
    #[inline(always)]
    fn tck_low_sampling(self) {
        self.tck_low();
    }

    fn tck_high(self);

    fn tms_set(self, high: bool);

    /// Drive TMS from bit 0 of `bits`.
    #[inline(always)]
    fn tms_put(self, bits: u32) {
        byte_put(SWDIO_PIN, bits);
    }

    /// Drive TDI from bit 0 of `bits`.
    #[inline(always)]
    fn tdi_put(self, bits: u32) {
        byte_put(TDI_PIN, bits);
    }

    /// Sample TDO as 0 or 1.
    #[inline(always)]
    fn tdo_get(self) -> u32 {
        byte_get(TDO_PIN)
    }
}

/// The fast scans' lines, by value: behind a borrow LLVM re-derives the addresses in every cell.
#[derive(Clone, Copy)]
struct SetClr {
    set: usize,
    clr: usize,
}

impl Lines for SetClr {
    #[inline(always)]
    fn tck_low(self) {
        store_word(self.clr, SWCLK_MASK);
    }

    /// Each read cell starts a fetch word, keeping its nop runs' timing.
    #[inline(always)]
    fn tck_low_sampling(self) {
        store_word_aligned(self.clr, SWCLK_MASK);
    }

    #[inline(always)]
    fn tck_high(self) {
        store_word(self.set, SWCLK_MASK);
    }

    #[inline(always)]
    fn tms_set(self, high: bool) {
        let addr = if high { self.set } else { self.clr };
        // SAFETY: a GPIO SET/CLR register; dedicated write-1-to-act bit.
        unsafe { (addr as *mut u32).write_volatile(SWDIO_MASK) };
    }
}

/// The scaled scans' lines: every line on its byte pin.
#[derive(Clone, Copy)]
struct BytePins;

impl Lines for BytePins {
    #[inline(always)]
    fn tck_low(self) {
        byte_put(TCK_PIN, 0);
    }

    #[inline(always)]
    fn tck_high(self) {
        byte_put(TCK_PIN, 1);
    }

    #[inline(always)]
    fn tms_set(self, high: bool) {
        byte_put(SWDIO_PIN, high as u32);
    }
}

/// The generic sequences' lines: all four on the port words, one store or load each.
#[derive(Clone, Copy)]
struct Words(Regs);

impl Lines for Words {
    #[inline(always)]
    fn tck_low(self) {
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (self.0.clr as *mut u32).write_volatile(SWCLK_MASK) };
    }

    #[inline(always)]
    fn tck_high(self) {
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (self.0.set as *mut u32).write_volatile(SWCLK_MASK) };
    }

    #[inline(always)]
    fn tms_set(self, high: bool) {
        let addr = if high { self.0.set } else { self.0.clr };
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (addr as *mut u32).write_volatile(SWDIO_MASK) };
    }

    #[inline(always)]
    fn tdi_put(self, bits: u32) {
        let addr = if bits & 1 != 0 {
            self.0.set
        } else {
            self.0.clr
        };
        // SAFETY: resolved GPIO register; dedicated write-1-to-act bit.
        unsafe { (addr as *mut u32).write_volatile(TDI_MASK) };
    }

    #[inline(always)]
    fn tdo_get(self) -> u32 {
        // SAFETY: resolved GPIO register; plain read (`PIN` is unaffected by `MASK`).
        (unsafe { (self.0.pin as *const u32).read_volatile() } >> TDO_PIN) & 1
    }
}

/// A scan's lines as the shared scans' port seam.
#[derive(Clone, Copy)]
struct TapPort<L>(L);

impl<L: Lines> JtagPortOps for TapPort<L> {
    fn jtag_pads_acquire(&mut self) {
        let (mut tdi, mut tdo) = (tdi_pin(), tdo_pin());
        tdi.set_high(false);
        tdi.set_as_output();
        tdo.set_as_input();
    }

    fn jtag_pads_release(&mut self) {
        tdi_pin().set_as_input();
    }

    #[inline(always)]
    fn tck_low(&self) {
        self.0.tck_low();
    }

    #[inline(always)]
    fn tck_high(&self) {
        self.0.tck_high();
    }

    #[inline(always)]
    fn tms_set(&self, high: bool) {
        self.0.tms_set(high);
    }

    #[inline(always)]
    fn tdi_set(&self, high: bool) {
        self.0.tdi_put(high as u32);
    }

    #[inline(always)]
    fn tdi_put(&self, bits: u32) {
        self.0.tdi_put(bits);
    }

    #[inline(always)]
    fn tdo_get(&self) -> u32 {
        self.0.tdo_get()
    }

    fn tdi_is_high(&mut self) -> bool {
        tdi_pin().is_high()
    }

    fn tdo_is_high(&mut self) -> bool {
        tdo_pin().is_high()
    }
}

/// Where a scan reads the chain configuration its bypass fills are counted from.
trait Chain {
    fn config(&mut self) -> &jtag::Config;

    /// Keep a scan's ACK.
    fn keep(&mut self, ack: u32);

    /// A scan's result.
    fn complete(&mut self, data: u32) -> TransferResult;
}

/// The scaled scan's chain: the configuration and the engine's kept ACK.
struct ScaledChain<'a> {
    config: &'a jtag::Config,
    kept: &'a KeptAck,
}

impl Chain for ScaledChain<'_> {
    #[inline(always)]
    fn config(&mut self) -> &jtag::Config {
        self.config
    }

    #[inline(always)]
    fn keep(&mut self, ack: u32) {
        self.kept.keep(ack);
    }

    #[inline(always)]
    fn complete(&mut self, data: u32) -> TransferResult {
        self.kept.complete(|| self.config.index, data)
    }
}

impl<P: MaskedPacing> Chain for &mut Masked<P> {
    #[inline(always)]
    fn config(&mut self) -> &jtag::Config {
        self.bb.jtag_config()
    }

    #[inline(always)]
    fn keep(&mut self, ack: u32) {
        self.kept.keep(ack);
    }

    #[inline(always)]
    fn complete(&mut self, data: u32) -> TransferResult {
        let Masked { bb, kept, .. } = &mut **self;
        kept.complete(|| bb.jtag_config().index, data)
    }
}

/// The fast write's chain: no engine pointer to hold through the cells; its caller decodes.
struct WriteChain<'a> {
    config: &'a jtag::Config,
    kept: &'a KeptAck,
}

impl Chain for WriteChain<'_> {
    #[inline(always)]
    fn config(&mut self) -> &jtag::Config {
        self.config
    }

    #[inline(always)]
    fn keep(&mut self, ack: u32) {
        self.kept.keep(ack);
    }

    #[inline(always)]
    fn complete(&mut self, data: u32) -> TransferResult {
        TransferResult::Ok(data)
    }
}

/// One JTAG scan; `repr(C)` keeps the SRAMX entries' loads through a pointer fixed.
#[repr(C)]
struct Scan<L, H, C> {
    halves: H,
    chain: C,
    port: TapPort<L>,
}

/// The fast scan: SET/CLR edges, the injected ladder on every half, the chain through the engine.
type FastJtagScan<'a, P> = Scan<SetClr, JtagPacedHalves<P>, &'a mut Masked<P>>;

/// The fast write's arguments: the chain's configuration alone, its kept ACK passed apart.
type FastWriteScan<'a, P> = Scan<SetClr, JtagPacedHalves<P>, &'a jtag::Config>;

/// The scaled scan: byte pin edges, each clock class on its prepared spins.
type ScaledJtagScan<'a> = Scan<BytePins, PreparedHalves, ScaledChain<'a>>;

impl<L: Lines, H: ClockHalves, C: Chain> JtagScan for Scan<L, H, C> {
    type Port = TapPort<L>;

    #[inline(always)]
    fn scan_port(&self) -> &Self::Port {
        &self.port
    }

    #[inline(always)]
    fn scan_config(&mut self) -> &jtag::Config {
        self.chain.config()
    }

    #[inline(always)]
    fn scan_keep_ack(&mut self, ack: u32) {
        self.chain.keep(ack);
    }

    #[inline(always)]
    fn scan_complete(&mut self, data: u32) -> TransferResult {
        self.chain.complete(data)
    }

    #[inline(always)]
    fn scan_wait_low(&mut self) {
        self.halves.wait_low();
    }

    #[inline(always)]
    fn scan_wait_high(&mut self) {
        self.halves.wait_high();
    }

    #[inline(always)]
    fn scan_wait_data_low(&mut self) {
        self.halves.wait_data_low();
    }

    #[inline(always)]
    fn scan_wait_data_high(&mut self) {
        self.halves.wait_data_high();
    }

    #[inline(always)]
    fn scan_wait_ack_low(&mut self) {
        self.halves.wait_ack_low();
    }

    #[inline(always)]
    fn scan_wait_ack_high(&mut self) {
        self.halves.wait_ack_high();
    }

    #[inline(always)]
    fn scan_clock_in(&mut self) -> u32 {
        self.port.0.tck_low_sampling();
        self.scan_wait_data_low();
        let bit = self.port.0.tdo_get();
        self.port.0.tck_high();
        self.scan_wait_data_high();
        bit
    }

    /// Straight-line cells, each folding its sample into `v` in place.
    #[inline(always)]
    fn scan_read_cells(&mut self, after: u32) -> Option<u32> {
        // TMS is low here, so one unconditional store serves both chain shapes.
        let exit = in_register((after == 0) as u32);
        let mut v = 0;
        cells!(31, i => v = in_register(v | self.scan_clock_in() << i));
        self.port.0.tms_put(exit); // final data bit exits Shift-DR
        Some(v | (self.scan_clock_in() << 31))
    }

    /// Straight-line cells, each shifting its own bit out of `data`.
    #[inline(always)]
    fn scan_write_cells(&mut self, data: u32, after: u32) -> Option<()> {
        let exit = in_register((after == 0) as u32);
        cells!(31, i => {
            self.port.0.tdi_put(in_register(data >> i));
            self.scan_clock_out();
        });
        self.port.0.tms_put(exit); // final data bit exits Shift-DR
        self.port.0.tdi_put(data >> 31);
        self.scan_clock_out();
        Some(())
    }
}

// SRAMX scan entries: their timing assumes every function starts on a fetch word.

/// The fast read scan in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code"]
fn jtag_fast_read<P: MaskedPacing>(
    engine: &mut Masked<P>,
    a2a3: u8,
    idle_cycles: u8,
) -> TransferResult {
    engine
        .fast_scan()
        .transfer_word(RnW::R, a2a3, idle_cycles, 0)
}

/// The fast write scan in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code"]
fn jtag_fast_write<P: MaskedPacing>(
    scan: FastWriteScan<'_, P>,
    a2a3: u8,
    idle_cycles: u8,
    data: u32,
    kept: &KeptAck,
) {
    let chain = WriteChain {
        config: scan.chain,
        kept,
    };
    let mut scan = Scan {
        halves: scan.halves,
        chain,
        port: scan.port,
    };
    scan.transfer_word(RnW::W, a2a3, idle_cycles, data);
}

/// The scaled DPACC/APACC scan in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code"]
fn jtag_scaled_word(
    mut scan: ScaledJtagScan<'_>,
    r_nw: RnW,
    a2a3: u8,
    idle_cycles: u8,
    data: u32,
) -> TransferResult {
    scan.transfer_word(r_nw, a2a3, idle_cycles, data)
}

/// The fast IR scan in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code"]
fn jtag_fast_ir<P: MaskedPacing>(
    mut scan: FastJtagScan<'_, P>,
    ir: u32,
    ir_length: u32,
    before: u32,
    after: u32,
) {
    // Load the stacked argument before the first edge, not between two cells.
    let after = in_register(after);
    scan.shift_ir_geometry(ir, ir_length, before, after);
}

/// The scaled IR scan in SRAMX.
#[inline(never)]
#[link_section = ".sramx_code"]
fn jtag_scaled_ir(mut scan: ScaledJtagScan<'_>, ir: u32, ir_length: u32, before: u32, after: u32) {
    scan.shift_ir_geometry(ir, ir_length, before, after);
}

// Engine.

/// The masked-GPIO engine state, including duplicate pin handles for the JTAG mechanics.
struct Masked<P: MaskedPacing> {
    regs: Regs,
    /// Bit-bang adapter over duplicate handles of the same pads: SWJ traffic, sequences, parking.
    bb: WiredAdapter<IoPin, DynCycleDelay>,
    tdi: IoPin,
    tms_swdio: IoPin,
    tck_swclk: IoPin,
    tdo: IoPin,
    delay: DynCycleDelay,
    pacing: P,
    swd_timing: SwdTimingPlan,
    swd_read_prepared: PreparedHalves,
    swd_write_prepared: PreparedHalves,
    jtag_timing: PreparedJtagTiming,
    kept: KeptAck,
}

impl<P: MaskedPacing> Masked<P> {
    const fn swd_pacing() -> PacingProfile {
        PacingProfile::new(P::SWD_FAST_HALF_CYCLES, HalfCycleProfile::uncalibrated())
    }

    /// Prepare every JTAG class for `request`; `None` only for a zero request.
    fn jtag_timing(cpu_hz: u32, request: u32) -> Option<PreparedJtagTiming> {
        let pacing =
            FrequencyPacing::new(P::JTAG_FAST_MIN_HZ, HalfCycleProfile::uncalibrated()).at(request);
        let plan = prepare_jtag_timing(
            cpu_hz,
            request,
            P::JTAG_SEQUENCE_FLOOR_CYCLES,
            HalfCycleProfile::uncalibrated(),
            pacing,
            pacing,
            pacing,
        )?;
        Some(PreparedJtagTiming::new::<P>(plan, cpu_hz, request))
    }

    fn new(
        delay: DynCycleDelay,
        pacing: P,
        scan: &'static ScanChainStorage<SCAN_CHAIN_SLOTS>,
    ) -> Self {
        // The MCU-Link pin map; the handles are `Copy`, one set for the adapter, one kept here.
        let nreset = crate::pin::nreset();
        let tdi = tdi_pin();
        let tms_swdio = IoPin::buffered(0, SWDIO_PIN as u8, 0, TXEN_PIN as u8);
        let tck_swclk = IoPin::output(0, TCK_PIN as u8);
        let tdo = tdo_pin();

        let mut bb = wired_deps(nreset, tdi, tms_swdio, tck_swclk, tdo, delay, scan);
        bb.process_swj_clock(2_000_000);

        let swd_timing = prepare_swd_timing(
            delay.cpu_clock(),
            2_000_000,
            Self::swd_pacing(),
            Self::swd_pacing(),
        )
        .expect("the default SWD frequency is non-zero");
        publish_swd_plan(swd_timing);
        let jtag_timing = Self::jtag_timing(delay.cpu_clock(), 2_000_000)
            .expect("the default JTAG frequency is non-zero");

        Self {
            regs: Regs::new(),
            bb,
            tdi,
            tms_swdio,
            tck_swclk,
            tdo,
            delay,
            pacing,
            swd_timing,
            swd_read_prepared: prepared_for::<P>(swd_timing.read(), P::SWD_READ_FIXED_CYCLES),
            swd_write_prepared: prepared_for::<P>(swd_timing.write(), P::SWD_WRITE_FIXED_CYCLES),
            jtag_timing,
            kept: KeptAck::new(scan.jtag_dp()),
        }
    }

    /// Recompute SWD pacing, fast or scaled; `false` only for a zero request.
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
        self.swd_read_prepared = prepared_for::<P>(timing.read(), P::SWD_READ_FIXED_CYCLES);
        self.swd_write_prepared = prepared_for::<P>(timing.write(), P::SWD_WRITE_FIXED_CYCLES);
        publish_swd_plan(timing);
        true
    }

    /// Recompute JTAG scan and sequence pacing; `false` only for a zero request.
    fn set_jtag_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        let Some(timing) = Self::jtag_timing(self.delay.cpu_clock(), max_frequency) else {
            return false;
        };
        // `DAP_SWJ_Sequence` in JTAG mode runs on the adapter, which keeps its own clock.
        self.bb.process_swj_clock(max_frequency);
        self.jtag_timing = timing;
        true
    }

    #[inline]
    fn read_transfer(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        let req = request_byte(port, RnW::R, addr);
        // The fast/scaled split lives here, outside the bit loops.
        match self.swd_timing.read() {
            PacingPlan::Fast => read_word_fast(self.regs, ReadPacedHalves(self.pacing), req),
            PacingPlan::Scaled(_) => read_word_fast(self.regs, self.swd_read_prepared, req),
        }
    }

    #[inline]
    fn write_transfer(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        let req = request_byte(port, RnW::W, addr);
        match self.swd_timing.write() {
            PacingPlan::Fast => {
                write_word_fast(self.regs, WritePacedHalves(self.pacing), req, data)
            }
            PacingPlan::Scaled(_) => write_word_fast(self.regs, self.swd_write_prepared, req, data),
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
        let lines = Words(self.regs);
        let mut tdo = 0;
        for i in 0..num_bits {
            lines.tdi_put(tdi >> i);
            lines.tck_low();
            self.jtag_wait();
            tdo |= lines.tdo_get() << i;
            lines.tck_high();
            self.jtag_wait();
        }
        tdo
    }

    /// Walk a TMS bit sequence (TAP-state walk) with TDI unchanged.
    fn tms_walk(&mut self, tms: &[bool]) {
        let lines = Words(self.regs);
        for &bit in tms {
            lines.tms_set(bit);
            lines.tck_low();
            self.jtag_wait();
            lines.tck_high();
            self.jtag_wait();
        }
    }

    /// JTAG pin roles on mode entry: TMS idle-high and driven, TCK/TDI outputs, TDO input.
    fn pins_to_jtag(&mut self) {
        // nRESET is left at the host's level, as on SWD entry.
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

        self.tdi.set_as_input();
        self.tdo.set_as_input();
    }

    /// The host's SWD idle cycles after an OK ACK.
    #[inline(always)]
    fn idle(&mut self) {
        let cycles = self.kept.dp().swd_idle();
        if cycles != 0 {
            self.idle_cells(cycles);
        }
    }

    /// Idle cells with SWDIO driven low; not a hot path, so always the spin.
    #[inline(never)]
    fn idle_cells(&mut self, cycles: u8) {
        let r = self.regs;
        let h = SpinHalves {
            delay: self.delay,
            cycles: HalfCycles::symmetric(self.swd_timing.sequence_half_cycles()),
        };
        r.drive_swdio();
        for _ in 0..cycles {
            bit_out(r, h, 0);
        }
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

    /// The fast scan over this engine's registers and chain.
    #[inline(always)]
    fn fast_scan(&mut self) -> FastJtagScan<'_, P> {
        Scan {
            halves: JtagPacedHalves(self.pacing),
            port: TapPort(SetClr {
                set: self.regs.set,
                clr: self.regs.clr,
            }),
            chain: self,
        }
    }

    /// The scaled scan, `data` pacing the scan's own data class.
    #[inline(always)]
    fn scaled_scan(&mut self, data: JtagHalves) -> ScaledJtagScan<'_> {
        Scan {
            halves: self.jtag_timing.scaled_halves(data),
            chain: ScaledChain {
                config: self.bb.jtag_config(),
                kept: &self.kept,
            },
            port: TapPort(BytePins),
        }
    }
}

// Drivers.

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
        SWD_ACTIVE.store(true, Ordering::Relaxed);
        MaskedSwdDrv(d.0)
    }
}
impl<P: MaskedPacing> From<MaskedSwdDrv<P>> for MaskedDeps<P> {
    fn from(s: MaskedSwdDrv<P>) -> Self {
        // Every SWD-mode exit releases `MASK0`, so no other mode ever sees `MPIN0` routed.
        SWD_ACTIVE.store(false, Ordering::Relaxed);
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

impl<P: MaskedPacing> JtagDpSource for MaskedDeps<P> {
    fn jtag_dp(&self) -> Option<&'static JtagDp> {
        Some(self.0.kept.dp())
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
        self.0.bb.high_impedance_mode();
        self.0.bb.process_swj_pins(Pins::NRESET, Pins::NRESET, 0);
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        self.0.bb.swd_config()
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.0.bb.jtag_config()
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
        Words(self.0.regs).tms_set(info.tms);
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

    /// The specialized DPACC/APACC scan; `write_abort` bypasses it for the generic mechanics.
    #[inline(never)]
    #[link_section = ".sramx_code"]
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let idle = transfer_config.idle_cycles;
        let pacing = match r_nw {
            RnW::R => self.0.jtag_timing.read,
            RnW::W => self.0.jtag_timing.write,
        };
        match (pacing, r_nw) {
            (PreparedJtagPacing::Fast, RnW::R) => jtag_fast_read(&mut self.0, a2a3, idle),
            (PreparedJtagPacing::Fast, RnW::W) => {
                let Masked {
                    regs,
                    bb,
                    pacing,
                    kept,
                    ..
                } = &mut self.0;
                let scan = Scan {
                    halves: JtagPacedHalves(*pacing),
                    port: TapPort(SetClr {
                        set: regs.set,
                        clr: regs.clr,
                    }),
                    chain: &*bb.jtag_config(),
                };
                jtag_fast_write(scan, a2a3, idle, data, kept);
                kept.complete(|| bb.jtag_config().index, 0)
            }
            (PreparedJtagPacing::Scaled(halves), _) => {
                jtag_scaled_word(self.0.scaled_scan(halves), r_nw, a2a3, idle, data)
            }
        }
    }

    /// The specialized IR scan in place of the provided per-chunk `shift_ir`.
    fn shift_ir(&mut self, ir: u32) {
        let tap = self.0.bb.jtag_config().current_tap();
        let (length, before, after) = (
            u32::from(tap.ir_length),
            u32::from(tap.ir_before),
            u32::from(tap.ir_after),
        );
        match self.0.jtag_timing.ir {
            PreparedJtagPacing::Fast => {
                jtag_fast_ir(self.0.fast_scan(), ir, length, before, after);
            }
            PreparedJtagPacing::Scaled(halves) => {
                jtag_scaled_ir(self.0.scaled_scan(halves), ir, length, before, after);
            }
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
    fn read(&mut self, wait_retries: usize, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        wait_retry(wait_retries, || self.read_inner(apndp, a))
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        let r = self.0.read_transfer(apndp, a);
        if matches!(r, Ok(_) | Err(swd::Error::BadParity)) {
            self.0.idle();
        }
        r
    }
    fn write(
        &mut self,
        wait_retries: usize,
        apndp: APnDP,
        a: DPRegister,
        data: u32,
    ) -> swd::Result<()> {
        wait_retry(wait_retries, || self.write_inner(apndp, a, data))
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        let r = self.0.write_transfer(apndp, a, data);
        if r.is_ok() {
            self.0.idle();
        }
        r
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_swd_clock(max_frequency)
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        // Direction-out sequences share the generic SWJ serializer; `MASK0` does not affect it.
        self.0.bb.process_swj_sequence(data, num_bits);
        Ok(())
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.0.read_bit_sequence(num_bits, data);
        Ok(())
    }
}

/// Build MCU-Link masked-GPIO dependencies with injected delay and pacing.
///
/// # Panics
/// A second construction panics because all calls share one scan-chain storage cell.
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

#[cfg(test)]
#[path = "masked_protocol_tests.rs"]
mod protocol_tests;

#[cfg(test)]
#[path = "masked_differential_tests.rs"]
mod differential_tests;

#[cfg(test)]
mod tests {
    use super::*;

    /// The iteration counts `split` prepared, as plain integers.
    fn halves(nominal: u32, fixed: (u32, u32), floor: u32) -> (u32, u32) {
        let (low, high) = split(nominal, fixed, floor);
        (low.0, high.0)
    }

    /// The period is rounded once: the halves cover the whole period, the low half rounding up.
    #[test]
    fn the_period_rounds_once_rather_than_each_half_separately() {
        let (low, high) = halves(10, (0, 0), 0);
        assert_eq!(low, 10u32.div_ceil(SPIN_CYCLES));
        assert_eq!(low + high, 20u32.div_ceil(SPIN_CYCLES));
        // Rounding each half on its own would have cost an extra iteration.
        assert!(low + high < 2 * 10u32.div_ceil(SPIN_CYCLES));
    }

    /// Each half subtracts its own fixed work, so an asymmetric pacing keeps its shape.
    #[test]
    fn each_half_subtracts_its_own_fixed_work() {
        let (low, high) = halves(30, (0, 12), 0);
        assert_eq!(low, 30u32.div_ceil(SPIN_CYCLES));
        assert_eq!(low + high, (30 + 18u32).div_ceil(SPIN_CYCLES));
        assert!(high < low, "the high half absorbs the fixed work");
    }

    /// The floor applies before rounding, so fixed work cannot eat the sampling margin.
    #[test]
    fn the_residual_floor_applies_to_each_half_before_the_rounding() {
        // The low half is floored, the high half is not.
        let (low, high) = halves(30, (29, 0), 10);
        assert_eq!(low, 10u32.div_ceil(SPIN_CYCLES));
        assert_eq!(low + high, (10 + 30u32).div_ceil(SPIN_CYCLES));
        // Both halves floored: the request is entirely absorbed by the fixed work.
        assert_eq!(halves(1, (100, 100), 30), (10, 10));
    }

    /// Every prepared half is at least one iteration, whatever the request saturates to.
    #[test]
    fn a_saturated_request_still_prepares_one_iteration_per_half() {
        for (nominal, fixed, floor) in [
            (0u32, (0u32, 0u32), 0u32),
            (0, (u32::MAX, u32::MAX), 0),
            (5, (u32::MAX, 0), 0),
            (u32::MAX / 2, (0, 0), 0),
        ] {
            let (low, high) = halves(nominal, fixed, floor);
            assert!(low >= 1 && high >= 1, "{nominal} {fixed:?} {floor}");
        }
    }

    /// The default floor is one spin iteration, so no prepared half is shorter than the loop.
    #[test]
    fn the_default_floor_is_one_spin_iteration() {
        let (low, high) = halves(1, (0, 0), SPIN_CYCLES);
        assert_eq!((low, high), (1, 1));
    }

    /// MCU-Link scaled SWD (fit, work) per class: request, ACK, read, write.
    const SWD_CLASSES: [((u32, u32), u32); 4] = [
        ((1, 0), SWD_CLASS_WORK[0]),
        ((0, 0), SWD_CLASS_WORK[1]),
        ((2, 0), SWD_CLASS_WORK[2]),
        ((1, 2), SWD_CLASS_WORK[3]),
    ];

    /// Realized cycles of a scaled SWD cell whose class spends `work` outside the spins.
    fn swd_cell(nominal: u32, fit: (u32, u32), work: u32) -> u64 {
        let (low, high) = halves(nominal, fit, SPIN_CYCLES);
        u64::from((low + high) * SPIN_CYCLES + work)
    }

    /// Each scaled SWD plan down to 100 kHz at its slowest, hardest-to-cover request.
    fn swd_plans(cpu: u32) -> impl Iterator<Item = (u32, u32)> {
        let profile = PacingProfile::new(3, HalfCycleProfile::uncalibrated());
        (4..=cpu / 2 / 100_000).map(move |n| {
            let request = (cpu / 2).div_ceil(n);
            let timing = prepare_swd_timing(cpu, request, profile, profile).unwrap();
            let PacingPlan::Scaled(c) = timing.write() else {
                panic!("{cpu} {request}");
            };
            assert_eq!(c.low(), n, "{cpu} {request}");
            (request, n)
        })
    }

    #[test]
    fn scaled_swd_cells_never_outrun_the_request() {
        for cpu in [150_000_000, 96_000_000] {
            for (request, n) in swd_plans(cpu) {
                for (fit, work) in SWD_CLASSES {
                    let cycles = swd_cell(n, fit, work);
                    assert!(
                        cycles * u64::from(request) >= u64::from(cpu),
                        "{request} {fit:?}"
                    );
                }
            }
        }
    }

    /// A fit one cycle above its class's work runs some plan short, as (2, 0) did.
    #[test]
    fn a_fit_above_the_cell_work_runs_short() {
        for cpu in [150_000_000, 96_000_000] {
            let short = swd_plans(cpu)
                .any(|(request, n)| swd_cell(n, (2, 0), 1) * u64::from(request) < u64::from(cpu));
            assert!(short, "{cpu}");
        }
    }

    /// MCU-Link core clocks, their fast JTAG handoffs and scaled floors in picoseconds.
    const JTAG_CLOCKS: [(u32, u32, (u32, u32, u32)); 2] = [
        (150_000_000, 14_285_715, (45_000, 20_000, 72_500)),
        (96_000_000, 12_500_000, (50_000, 20_000, 72_500)),
    ];

    /// Realized (low, high) cycles when each half costs its spins plus `fixed`.
    fn realized(h: JtagHalves, fixed: (u32, u32)) -> (u64, u64) {
        let cycles = |n: NonZeroU32, f: u32| u64::from(n.get() * SPIN_CYCLES + f);
        (cycles(h.low, fixed.0), cycles(h.high, fixed.1))
    }

    #[test]
    fn scaled_jtag_cells_never_outrun_the_request_or_the_floors() {
        for (cpu, fast_min, floors) in JTAG_CLOCKS {
            let requests = (1_000_000..fast_min).step_by(997).chain([fast_min - 1]);
            for request in requests {
                let target = jtag_targets(cpu, request, floors);
                for fixed in [(0, 0), (3, 4), (5, 6)] {
                    let (low, high) = realized(jtag_halves(target, fixed), fixed);
                    let ps = |c: u64| c * 1_000_000_000_000 / u64::from(cpu);
                    assert!(
                        (low + high) * u64::from(request) >= u64::from(cpu),
                        "{request}"
                    );
                    assert!(ps(low) >= u64::from(floors.0), "{cpu} {request}");
                    assert!(ps(high) >= u64::from(floors.1), "{cpu} {request}");
                    assert!(ps(low + high) >= u64::from(floors.2), "{cpu} {request}");
                }
            }
        }
    }

    #[test]
    fn the_whole_period_is_split_once() {
        let [(cpu150, _, floors150), (cpu96, _, floors96)] = JTAG_CLOCKS;
        assert_eq!(jtag_targets(cpu150, 14_285_714, floors150), (7, 4));
        assert_eq!(jtag_targets(cpu150, 2_000_000, floors150), (38, 37));
        assert_eq!(jtag_targets(cpu96, 12_499_999, floors96), (5, 3));
        assert_eq!(jtag_targets(cpu96, 2_000_000, floors96), (24, 24));
    }

    #[test]
    fn scaled_jtag_spins_round_up_to_whole_iterations() {
        let h = jtag_halves((7, 4), (0, 0));
        assert_eq!((h.low.get(), h.high.get()), (3, 2));
        let h = jtag_halves((7, 4), (4, 9));
        assert_eq!((h.low.get(), h.high.get()), (1, 1));
    }

    extern crate std;
    use std::boxed::Box;
    use std::cell::RefCell;
    use std::vec;
    use std::vec::Vec;

    /// The wire the mock lines drive: (TMS, TDI) latched at each rising edge, TDO scripted.
    #[derive(Default)]
    struct Wire {
        tms: bool,
        tdi: bool,
        cells: Vec<(bool, bool)>,
        tdo: Vec<u32>,
        sampled: usize,
    }

    std::thread_local! {
        static WIRE: RefCell<Wire> = RefCell::new(Wire::default());
    }

    #[derive(Clone, Copy)]
    struct MockLines;

    impl Lines for MockLines {
        fn tck_low(self) {}

        fn tck_high(self) {
            WIRE.with(|w| {
                let mut w = w.borrow_mut();
                let cell = (w.tms, w.tdi);
                w.cells.push(cell);
            });
        }

        fn tms_set(self, high: bool) {
            WIRE.with(|w| w.borrow_mut().tms = high);
        }

        fn tms_put(self, bits: u32) {
            self.tms_set(bits & 1 != 0);
        }

        fn tdi_put(self, bits: u32) {
            WIRE.with(|w| w.borrow_mut().tdi = bits & 1 != 0);
        }

        fn tdo_get(self) -> u32 {
            WIRE.with(|w| {
                let mut w = w.borrow_mut();
                w.sampled += 1;
                w.tdo.get(w.sampled - 1).copied().unwrap_or(0)
            })
        }
    }

    #[derive(Clone, Copy)]
    struct NoWait;

    impl ClockHalves for NoWait {
        fn wait_low(self) {}

        fn wait_high(self) {}
    }

    type MockScan = Scan<MockLines, NoWait, ScaledChain<'static>>;

    /// The same scan without its straight-line cells, so the shared counted loops run.
    struct Looped(MockScan);

    impl JtagScan for Looped {
        type Port = TapPort<MockLines>;

        fn scan_port(&self) -> &Self::Port {
            &self.0.port
        }

        fn scan_config(&mut self) -> &jtag::Config {
            self.0.chain.config
        }

        fn scan_keep_ack(&mut self, ack: u32) {
            self.0.scan_keep_ack(ack);
        }

        fn scan_complete(&mut self, data: u32) -> TransferResult {
            self.0.scan_complete(data)
        }

        fn scan_wait_low(&mut self) {}

        fn scan_wait_high(&mut self) {}
    }

    /// A chain of TAPs with `ir_lens`, the scan addressing `index`.
    fn chain(ir_lens: &[u8], index: u8) -> &'static jtag::Config {
        let taps = Box::leak(vec![jtag::TapConfig::INIT; ir_lens.len()].into_boxed_slice());
        let mut config = jtag::Config::new(taps);
        config.device_count = ir_lens.len() as u8;
        config.index = index;
        Box::leak(Box::new(config))
    }

    /// Run `scan` on a fresh wire replaying `tdo`: its result and the latched cells.
    fn on_wire<T>(tdo: &[u32], scan: impl FnOnce() -> T) -> (T, Vec<(bool, bool)>) {
        WIRE.with(|w| {
            *w.borrow_mut() = Wire {
                tdo: tdo.to_vec(),
                ..Wire::default()
            }
        });
        let result = scan();
        (result, WIRE.with(|w| w.borrow().cells.clone()))
    }

    #[test]
    fn straight_line_data_cells_match_the_counted_loops() {
        let mut tdo = vec![0, 1, 0];
        tdo.extend((0..32).map(|bit| (0x8e1f_62d5u32 >> bit) & 1));
        for (ir_lens, index) in [(&[4u8][..], 0u8), (&[4, 5][..], 0), (&[4, 5][..], 1)] {
            for (rnw, data) in [(RnW::R, 0), (RnW::W, 0x9249_6db6)] {
                let scan = || Scan {
                    halves: NoWait,
                    chain: ScaledChain {
                        config: chain(ir_lens, index),
                        kept: Box::leak(Box::new(KeptAck::new(Box::leak(Box::default())))),
                    },
                    port: TapPort(MockLines),
                };
                let straight = on_wire(&tdo, || scan().transfer_word(rnw, 3, 2, data));
                let looped = on_wire(&tdo, || Looped(scan()).transfer_word(rnw, 3, 2, data));
                assert_eq!(straight, looped, "{ir_lens:?} {index} {rnw:?}");
            }
        }
    }

    /// A scaled scan keeps what is not protocol 0's OK, so the decode sees protocol 1 and no ACK.
    #[test]
    fn a_scaled_scan_keeps_every_other_ack_for_the_decode() {
        for (ack, ok) in [([0, 0, 1], true), ([1, 1, 1], false)] {
            let dp: &'static JtagDp = Box::leak(Box::default());
            let mut tdo = vec![ack[0], ack[1], ack[2]];
            tdo.extend([0; 32]);
            let mut scan = Scan {
                halves: NoWait,
                chain: ScaledChain {
                    config: chain(&[4], 0),
                    kept: Box::leak(Box::new(KeptAck::new(dp))),
                },
                port: TapPort(MockLines),
            };
            let (result, _) = on_wire(&tdo, || scan.transfer_word(RnW::R, 3, 2, 0));
            assert_eq!(matches!(result, TransferResult::Ok(_)), ok, "{ack:?}");
            assert_eq!(dp.any_protocol1(), ok);
        }
    }

    /// The transfer status byte a block end reports, as dap-rs encodes it.
    #[test]
    fn the_block_status_byte_matches_the_dap_encoding() {
        assert_eq!(status_byte(swd::Error::AckWait), 2);
        assert_eq!(status_byte(swd::Error::AckFault), 4);
        assert_eq!(status_byte(swd::Error::NoAck), 7);
        // Anything else is a protocol error: the value bit set beside the fault code.
        assert_eq!(status_byte(swd::Error::BadParity), (1 << 3) | 7);
        assert_eq!(status_byte(swd::Error::AckProtocol), (1 << 3) | 7);
    }
}
