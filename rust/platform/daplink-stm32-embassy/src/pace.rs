//! The spin the bit-bang cells pace with.

use core::num::NonZeroU32;

use daplink_spi_swd::pace::ClassFit;

/// A prepared spin of at least one iteration, fetch-word aligned so no pad access runs before it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Spin(NonZeroU32);

impl Spin {
    pub const fn get(self) -> u32 {
        self.0.get()
    }

    #[inline(always)]
    pub fn run(self) {
        let n = self.0.get();
        // SAFETY: registers only; the count is non-zero; flags are clobbered.
        #[cfg(target_arch = "arm")]
        unsafe {
            // The count's copy is inside the block, so it lands in this half in every cell.
            core::arch::asm!(
                "mov {i}, {n}",
                ".p2align 2",
                "2:",
                "subs {i}, #1",
                "bne 2b",
                n = in(reg) n,
                i = out(reg) _,
                options(nomem, nostack)
            );
        }
        #[cfg(not(target_arch = "arm"))]
        for _ in 0..n {
            core::hint::spin_loop();
        }
    }
}

/// `fit`'s spins for a request; this spin must cost `daplink_spi_swd::pace::SPIN_CYCLES`.
pub const fn halves(fit: &ClassFit, cpu_hz: u32, request_hz: u32) -> (Spin, Spin) {
    let (low, high) = fit.halves(cpu_hz, request_hz);
    (Spin(low), Spin(high))
}

/// `v` through an empty asm block, so a straight-line cell updates it in place.
#[inline(always)]
pub fn in_register(mut v: u32) -> u32 {
    // SAFETY: an empty asm block; `v` passes through a register unchanged.
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("/* {0} */", inout(reg) v, options(nomem, nostack, preserves_flags));
    }
    #[cfg(not(target_arch = "arm"))]
    {
        v = core::hint::black_box(v);
    }
    v
}

/// [`in_register`], also ordered against the pad accesses so its work stays in its cell.
#[inline(always)]
pub fn pinned(mut v: u32) -> u32 {
    // SAFETY: an empty asm block; `v` passes through a register unchanged.
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("/* {0} */", inout(reg) v, options(nostack, preserves_flags));
    }
    #[cfg(not(target_arch = "arm"))]
    {
        v = core::hint::black_box(v);
    }
    v
}
