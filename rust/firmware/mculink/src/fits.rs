//! The MCU-Link masked engine's pacing fits, per core clock, and their compile-time checks.

use daplink_lpc55_hal::{nops, verify_fit, JtagPacing, MaskedPacing};

use crate::app::CPU_HZ;

/// One core clock's accel pacing; nop counts are calibrated per clock, not scaled between them.
struct MaskedFit {
    cpu_hz: u32,
    /// Half-period cycle count at or below which the nop-paced JTAG paths run.
    fast_half_cycles: u32,
    /// Half-period cycle count at or below which the nop-paced SWD paths run.
    swd_fast_half_cycles: u32,
    /// Scaled SWD residual floor per half.
    swd_scaled_floor_cycles: u32,
    /// Scaled SWD fixed (low, high) cycles: request, ACK/turnaround, read, write.
    swd_fixed_cycles: [(u32, u32); 4],
    /// Nop-paced SWD (low, high) counts: control, read, write.
    swd_nops: [(usize, usize); 3],
    /// Requests at or above this run the nop-paced JTAG body.
    jtag_fast_min_hz: u32,
    /// The nop-paced JTAG body's shortest cell, in picoseconds.
    jtag_fastest_cell_ps: u32,
    /// Scaled JTAG floors in picoseconds: low half, high half and period.
    jtag_floors_ps: (u32, u32, u32),
    jtag_sequence_floor_cycles: u32,
    /// Nop-paced JTAG (low, high) counts: low is the TCK-fall to TDO-sample margin.
    jtag_nops: (usize, usize),
    /// Scaled JTAG fixed (low, high) cycles: control, ACK, read, write and IR.
    jtag_fixed_cycles: [(u32, u32); 5],
}

impl MaskedFit {
    /// The nop-paced JTAG body is never faster than the threshold request.
    const fn check(&self) {
        assert!(
            1_000_000_000_000 / self.jtag_fast_min_hz as u64 <= self.jtag_fastest_cell_ps as u64
        );
    }
}

const FIT_150MHZ: MaskedFit = MaskedFit {
    cpu_hz: 150_000_000,
    fast_half_cycles: 4,
    swd_fast_half_cycles: 3,
    swd_scaled_floor_cycles: 3,
    swd_fixed_cycles: [(1, 0), (0, 0), (2, 0), (1, 2)],
    swd_nops: [(6, 4), (5, 4), (6, 4)],
    jtag_fast_min_hz: 14_285_715,
    jtag_fastest_cell_ps: 72_500,
    jtag_floors_ps: (45_000, 20_000, 72_500),
    jtag_sequence_floor_cycles: 7,
    jtag_nops: (13, 3),
    jtag_fixed_cycles: [(0, 0), (0, 1), (0, 1), (0, 1), (0, 2)],
};

const FIT_96MHZ: MaskedFit = MaskedFit {
    cpu_hz: 96_000_000,
    fast_half_cycles: 4,
    swd_fast_half_cycles: 3,
    swd_scaled_floor_cycles: 3,
    swd_fixed_cycles: [(1, 0), (0, 0), (2, 0), (1, 2)],
    swd_nops: [(5, 4), (5, 4), (5, 4)],
    jtag_fast_min_hz: 12_500_000,
    jtag_fastest_cell_ps: 82_500,
    jtag_floors_ps: (50_000, 20_000, 72_500),
    jtag_sequence_floor_cycles: 4,
    jtag_nops: (9, 1),
    jtag_fixed_cycles: [(0, 0), (0, 1), (0, 1), (0, 1), (0, 2)],
};

#[cfg(feature = "core-96mhz")]
const FIT: MaskedFit = FIT_96MHZ;
#[cfg(not(feature = "core-96mhz"))]
const FIT: MaskedFit = FIT_150MHZ;

const _: () = {
    FIT_150MHZ.check();
    FIT_96MHZ.check();
    assert!(FIT.cpu_hz == CPU_HZ);
    verify_fit::<AccelPacing>(CPU_HZ);
};

#[derive(Clone, Copy)]
pub(crate) struct AccelPacing;

impl JtagPacing for AccelPacing {
    const FAST_HALF_CYCLES: u32 = FIT.fast_half_cycles;

    #[inline(always)]
    fn pace_low(&self) {
        nops::<{ FIT.jtag_nops.0 }>();
    }

    #[inline(always)]
    fn pace_high(&self) {
        nops::<{ FIT.jtag_nops.1 }>();
    }
}

impl MaskedPacing for AccelPacing {
    const SWD_FAST_HALF_CYCLES: u32 = FIT.swd_fast_half_cycles;
    const SWD_SCALED_FLOOR_CYCLES: u32 = FIT.swd_scaled_floor_cycles;
    const SWD_CONTROL_FIXED_CYCLES: (u32, u32) = FIT.swd_fixed_cycles[0];
    const SWD_ACK_FIXED_CYCLES: (u32, u32) = FIT.swd_fixed_cycles[1];
    const SWD_READ_FIXED_CYCLES: (u32, u32) = FIT.swd_fixed_cycles[2];
    const SWD_WRITE_FIXED_CYCLES: (u32, u32) = FIT.swd_fixed_cycles[3];
    const JTAG_FAST_MIN_HZ: u32 = FIT.jtag_fast_min_hz;
    const JTAG_SEQUENCE_FLOOR_CYCLES: u32 = FIT.jtag_sequence_floor_cycles;
    const JTAG_LOW_FLOOR_PS: u32 = FIT.jtag_floors_ps.0;
    const JTAG_HIGH_FLOOR_PS: u32 = FIT.jtag_floors_ps.1;
    const JTAG_PERIOD_FLOOR_PS: u32 = FIT.jtag_floors_ps.2;
    const JTAG_CONTROL_FIXED_CYCLES: (u32, u32) = FIT.jtag_fixed_cycles[0];
    const JTAG_ACK_FIXED_CYCLES: (u32, u32) = FIT.jtag_fixed_cycles[1];
    const JTAG_READ_FIXED_CYCLES: (u32, u32) = FIT.jtag_fixed_cycles[2];
    const JTAG_WRITE_FIXED_CYCLES: (u32, u32) = FIT.jtag_fixed_cycles[3];
    const JTAG_IR_FIXED_CYCLES: (u32, u32) = FIT.jtag_fixed_cycles[4];

    #[inline(always)]
    fn swd_control_low(&self) {
        nops::<{ FIT.swd_nops[0].0 }>();
    }

    #[inline(always)]
    fn swd_control_high(&self) {
        nops::<{ FIT.swd_nops[0].1 }>();
    }

    #[inline(always)]
    fn swd_read_low(&self) {
        nops::<{ FIT.swd_nops[1].0 }>();
    }

    #[inline(always)]
    fn swd_read_high(&self) {
        nops::<{ FIT.swd_nops[1].1 }>();
    }

    #[inline(always)]
    fn swd_write_low(&self) {
        nops::<{ FIT.swd_nops[2].0 }>();
    }

    #[inline(always)]
    fn swd_write_high(&self) {
        nops::<{ FIT.swd_nops[2].1 }>();
    }
}
