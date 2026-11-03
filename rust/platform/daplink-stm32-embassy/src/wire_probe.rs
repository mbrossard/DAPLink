//! GPIO cell-cost probe: fixed cell counts on the H5 probe pads, timed externally.

use core::marker::PhantomData;
use cortex_m::asm::delay;
use daplink_rs::PreparedSpin;

use crate::swj::SwjPads;

/// The pad bit patterns for one wiring; no other pad and no clock tree is touched.
struct Bits<M>(PhantomData<M>);

impl<M: SwjPads> Bits<M> {
    const SWCLK_SET: u32 = 1 << M::SWCLK;
    const SWCLK_CLR: u32 = Self::SWCLK_SET << 16;
    const SWDIO_SET: u32 = 1 << M::SWDIO;
    const BOTH_CLR: u32 = Self::SWCLK_CLR | (Self::SWDIO_SET << 16);

    /// Two-bit field pair for the two pads in `MODER`, `OSPEEDR` and `PUPDR`.
    const PAIR_MASK: u32 = (0b11 << (2 * M::SWCLK)) | (0b11 << (2 * M::SWDIO));
    const PAIR_OUT: u32 = (0b01 << (2 * M::SWCLK)) | (0b01 << (2 * M::SWDIO));
    const PAIR_PULL_UP: u32 = (0b01 << (2 * M::SWCLK)) | (0b01 << (2 * M::SWDIO));
    const PAIR_VHS: u32 = Self::PAIR_MASK;
}

// Cells per segment: fixed counts, so a segment's period divided by its count is the cell cost.
/// (a) Raw `BSRR` store rate.
pub const TOGGLE_CELLS: u32 = 80_000;
/// (b) Merged driven cell.
pub const MERGED_CELLS: u32 = 60_000;
/// (c) Unmerged driven cell.
pub const UNMERGED_CELLS: u32 = 45_000;
/// (d) Read cell.
pub const READ_CELLS: u32 = 40_000;
/// (e) Merged driven cell from RAM.
pub const RAM_CELLS: u32 = 60_000;
/// (f) Merged driven cell with a residual spin in each half.
pub const SPUN_CELLS: u32 = 25_000;

/// Residual spin per half in segment (f), in core cycles.
pub const SPIN_CYCLES: u32 = 6;

/// Nominal low separator delay, in microseconds.
const SEPARATOR_US: u32 = 200;
const REPEATS: u32 = 12;

/// Alternating data pattern.
const PATTERN: u32 = 0xAAAA_AAAA;

/// (a) The raw store rate: SWCLK set and reset, nothing else in the loop.
#[inline(never)]
fn segment_toggle<M: SwjPads>(bsrr: *mut u32) {
    for _ in 0..TOGGLE_CELLS {
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe {
            bsrr.write_volatile(Bits::<M>::SWCLK_SET);
            bsrr.write_volatile(Bits::<M>::SWCLK_CLR);
        }
    }
}

/// (b) The merged driven cell: data and clock-low in one store, then clock-high.
#[inline(never)]
fn segment_merged<M: SwjPads>(bsrr: *mut u32) {
    let mut v = PATTERN;
    for _ in 0..MERGED_CELLS {
        let low = (Bits::<M>::SWDIO_SET << (((v & 1) ^ 1) << 4)) | Bits::<M>::SWCLK_CLR;
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe {
            bsrr.write_volatile(low);
            bsrr.write_volatile(Bits::<M>::SWCLK_SET);
        }
        v = v.rotate_right(1);
    }
}

/// (c) The unmerged driven cell: data, clock-low and clock-high as three stores.
#[inline(never)]
fn segment_unmerged<M: SwjPads>(bsrr: *mut u32) {
    let mut v = PATTERN;
    for _ in 0..UNMERGED_CELLS {
        let data = Bits::<M>::SWDIO_SET << (((v & 1) ^ 1) << 4);
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe {
            bsrr.write_volatile(data);
            bsrr.write_volatile(Bits::<M>::SWCLK_CLR);
            bsrr.write_volatile(Bits::<M>::SWCLK_SET);
        }
        v = v.rotate_right(1);
    }
}

/// (d) The read cell: clock-low, an `IDR` load folded into the accumulator, clock-high.
#[inline(never)]
fn segment_read<M: SwjPads>(bsrr: *mut u32, idr: *const u32) {
    let mut acc = 0u32;
    for _ in 0..READ_CELLS {
        // SAFETY: resolved GPIO registers; `BSRR` is write-1-to-act and `IDR` is a plain read.
        unsafe {
            bsrr.write_volatile(Bits::<M>::SWCLK_CLR);
            acc = (acc >> 1) | (idr.read_volatile() >> M::SWDIO) << 31;
            bsrr.write_volatile(Bits::<M>::SWCLK_SET);
        }
    }
    core::hint::black_box(acc);
}

/// (e) Segment (b) from RAM, to compare flash fetch against SRAM.
#[inline(never)]
#[link_section = ".ramcode.wire_probe"]
fn segment_merged_ram<M: SwjPads>(bsrr: *mut u32) {
    let mut v = PATTERN;
    for _ in 0..RAM_CELLS {
        let low = (Bits::<M>::SWDIO_SET << (((v & 1) ^ 1) << 4)) | Bits::<M>::SWCLK_CLR;
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe {
            bsrr.write_volatile(low);
            bsrr.write_volatile(Bits::<M>::SWCLK_SET);
        }
        v = v.rotate_right(1);
    }
}

/// (f) Segment (b) with a residual spin in each half.
#[inline(never)]
fn segment_spun<M: SwjPads>(bsrr: *mut u32) {
    let spin = PreparedSpin::new(SPIN_CYCLES);
    let mut v = PATTERN;
    for _ in 0..SPUN_CELLS {
        let low = (Bits::<M>::SWDIO_SET << (((v & 1) ^ 1) << 4)) | Bits::<M>::SWCLK_CLR;
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe { bsrr.write_volatile(low) };
        spin.run();
        // SAFETY: as above.
        unsafe { bsrr.write_volatile(Bits::<M>::SWCLK_SET) };
        spin.run();
        v = v.rotate_right(1);
    }
}

/// Emit six fixed-count segments separated by low gaps; leave both pads pulled-up inputs.
pub fn run_wire_probe<M: SwjPads>(cpu_hz: u32) {
    let b = M::swj_port();
    let moder = b.moder().as_ptr() as *mut u32;
    let otyper = b.otyper().as_ptr() as *mut u32;
    let ospeedr = b.ospeedr().as_ptr() as *mut u32;
    let pupdr = b.pupdr().as_ptr() as *mut u32;
    let bsrr = b.bsrr().as_ptr() as *mut u32;
    let idr = b.idr().as_ptr() as *const u32;

    // SAFETY: resolved GPIO registers; only the two pads' fields are changed.
    unsafe {
        otyper.write_volatile(
            otyper.read_volatile() & !(Bits::<M>::SWCLK_SET | Bits::<M>::SWDIO_SET),
        );
        pupdr.write_volatile(pupdr.read_volatile() & !Bits::<M>::PAIR_MASK);
        ospeedr.write_volatile(ospeedr.read_volatile() | Bits::<M>::PAIR_VHS);
        bsrr.write_volatile(Bits::<M>::BOTH_CLR);
        moder.write_volatile((moder.read_volatile() & !Bits::<M>::PAIR_MASK) | Bits::<M>::PAIR_OUT);
    }

    let gap = cpu_hz / (1_000_000 / SEPARATOR_US);
    let separator = || {
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe { bsrr.write_volatile(Bits::<M>::BOTH_CLR) };
        delay(gap);
    };

    for _ in 0..REPEATS {
        separator();
        segment_toggle::<M>(bsrr);
        separator();
        segment_merged::<M>(bsrr);
        separator();
        segment_unmerged::<M>(bsrr);
        separator();
        segment_read::<M>(bsrr, idr);
        separator();
        segment_merged_ram::<M>(bsrr);
        separator();
        segment_spun::<M>(bsrr);
        // A longer low gap marks the sequence boundary.
        // SAFETY: the caller's resolved `BSRR`; write-1-to-act.
        unsafe { bsrr.write_volatile(Bits::<M>::BOTH_CLR) };
        delay(gap * 10);
    }
    separator();

    // Leave input mode with reset speed and the pull-up bias.
    // SAFETY: resolved GPIO registers; only the two pads' fields are changed.
    unsafe {
        moder.write_volatile(moder.read_volatile() & !Bits::<M>::PAIR_MASK);
        ospeedr.write_volatile(ospeedr.read_volatile() & !Bits::<M>::PAIR_MASK);
        pupdr.write_volatile(
            (pupdr.read_volatile() & !Bits::<M>::PAIR_MASK) | Bits::<M>::PAIR_PULL_UP,
        );
    }
}
