//! Prepared GPIO timing plans for SWD and JTAG transfer kernels.

use crate::proto::half_period_cycles;
use core::num::NonZeroU32;

/// Low- and high-half residual cycle counts.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct HalfCycles {
    low: NonZeroU32,
    high: NonZeroU32,
}

impl HalfCycles {
    /// Construct distinct low- and high-half counts, each floored at one.
    pub const fn new(low: u32, high: u32) -> Self {
        Self {
            low: nonzero(low),
            high: nonzero(high),
        }
    }

    /// Construct equal low- and high-half counts.
    pub const fn symmetric(cycles: u32) -> Self {
        Self::new(cycles, cycles)
    }

    /// Low-half residual cycles.
    pub const fn low(self) -> u32 {
        self.low.get()
    }

    /// High-half residual cycles.
    pub const fn high(self) -> u32 {
        self.high.get()
    }
}

/// Fixed work to subtract and the minimum residual for each clock half.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct HalfCycleProfile {
    fixed_low: u32,
    fixed_high: u32,
    floor_low: u32,
    floor_high: u32,
}

/// Fast-path threshold and scaled half-cycle preparation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PacingProfile {
    fast_half_cycles: u32,
    scaled: HalfCycleProfile,
}

impl PacingProfile {
    /// Select fast pacing at or below `fast_half_cycles`; zero keeps scaled pacing.
    pub const fn new(fast_half_cycles: u32, scaled: HalfCycleProfile) -> Self {
        Self {
            fast_half_cycles,
            scaled,
        }
    }
}

impl HalfCycleProfile {
    /// Construct a profile from per-half fixed work and residual floors.
    pub const fn new(fixed_low: u32, fixed_high: u32, floor_low: u32, floor_high: u32) -> Self {
        Self {
            fixed_low,
            fixed_high,
            floor_low,
            floor_high,
        }
    }

    /// Preserve the full nominal half-period with no residual floor.
    pub const fn uncalibrated() -> Self {
        Self::new(0, 0, 0, 0)
    }

    const fn apply(self, nominal: u32) -> HalfCycles {
        let low = max_u32(nominal.saturating_sub(self.fixed_low), self.floor_low);
        let high = max_u32(nominal.saturating_sub(self.fixed_high), self.floor_high);
        HalfCycles::new(low, high)
    }
}

const fn nonzero(v: u32) -> NonZeroU32 {
    match NonZeroU32::new(v) {
        Some(v) => v,
        None => NonZeroU32::MIN,
    }
}

const fn max_u32(a: u32, b: u32) -> u32 {
    if a > b {
        a
    } else {
        b
    }
}

/// Prepared transfer pacing selected before entering a transfer kernel.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PacingPlan {
    /// Use the compile-time straight-line pacing profile.
    Fast,
    /// Use the prepared low- and high-half residual counts.
    Scaled(HalfCycles),
}

/// Prepared SWD timing, including the generic sequence period.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SwdTimingPlan {
    sequence_half: u32,
    read: PacingPlan,
    write: PacingPlan,
}

impl SwdTimingPlan {
    /// Half-period used by generic SWD input sequences.
    pub const fn sequence_half_cycles(self) -> u32 {
        self.sequence_half
    }

    /// Prepared word-read pacing.
    pub const fn read(self) -> PacingPlan {
        self.read
    }

    /// Prepared word-write pacing.
    pub const fn write(self) -> PacingPlan {
        self.write
    }
}

/// Prepared JTAG timing, including the generic sequence period.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct JtagTimingPlan {
    sequence_half: u32,
    control: HalfCycles,
    read: PacingPlan,
    write: PacingPlan,
    ir: PacingPlan,
}

impl JtagTimingPlan {
    /// Half-period used by generic JTAG sequences.
    pub const fn sequence_half_cycles(self) -> u32 {
        self.sequence_half
    }

    /// Prepared pacing for TAP-state and bypass clocks within scaled scans.
    pub const fn control(self) -> HalfCycles {
        self.control
    }

    /// Prepared DP/AP read pacing.
    pub const fn read(self) -> PacingPlan {
        self.read
    }

    /// Prepared DP/AP write pacing.
    pub const fn write(self) -> PacingPlan {
        self.write
    }

    /// Prepared IR scan pacing.
    pub const fn ir(self) -> PacingPlan {
        self.ir
    }
}

const fn prepare_pacing(nominal: u32, profile: PacingProfile) -> PacingPlan {
    if profile.fast_half_cycles != 0 && nominal <= profile.fast_half_cycles {
        PacingPlan::Fast
    } else {
        PacingPlan::Scaled(profile.scaled.apply(nominal))
    }
}

/// Prepare SWD sequence, read, and write timing; zero frequency is rejected.
pub const fn prepare_swd_timing(
    cpu_hz: u32,
    max_frequency: u32,
    read: PacingProfile,
    write: PacingProfile,
) -> Option<SwdTimingPlan> {
    if max_frequency == 0 {
        return None;
    }
    let nominal = half_period_cycles(cpu_hz, max_frequency);
    Some(SwdTimingPlan {
        sequence_half: nominal,
        read: prepare_pacing(nominal, read),
        write: prepare_pacing(nominal, write),
    })
}

/// Prepare JTAG sequence and scan timing; zero frequency is rejected.
pub const fn prepare_jtag_timing(
    cpu_hz: u32,
    max_frequency: u32,
    sequence_floor: u32,
    control: HalfCycleProfile,
    read: PacingProfile,
    write: PacingProfile,
    ir: PacingProfile,
) -> Option<JtagTimingPlan> {
    if max_frequency == 0 {
        return None;
    }
    let nominal = half_period_cycles(cpu_hz, max_frequency);
    Some(JtagTimingPlan {
        sequence_half: max_u32(nominal, sequence_floor),
        control: control.apply(nominal),
        read: prepare_pacing(nominal, read),
        write: prepare_pacing(nominal, write),
        ir: prepare_pacing(nominal, ir),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const NONE: HalfCycleProfile = HalfCycleProfile::uncalibrated();
    const FAST4: PacingProfile = PacingProfile::new(4, NONE);
    const SCALED: PacingProfile = PacingProfile::new(0, NONE);

    #[test]
    fn zero_frequency_is_rejected() {
        assert_eq!(prepare_swd_timing(150_000_000, 0, FAST4, FAST4), None);
        assert_eq!(
            prepare_jtag_timing(150_000_000, 0, 4, NONE, SCALED, FAST4, FAST4),
            None
        );
    }

    #[test]
    fn mculink_boundary_clamps_to_one_fast_plan() {
        let below = prepare_swd_timing(150_000_000, 18_749_999, FAST4, FAST4).unwrap();
        assert_eq!(below.read(), PacingPlan::Scaled(HalfCycles::symmetric(5)));
        assert_eq!(below.write(), PacingPlan::Scaled(HalfCycles::symmetric(5)));

        let boundary = prepare_swd_timing(150_000_000, 18_750_000, FAST4, FAST4).unwrap();
        let just_above = prepare_swd_timing(150_000_000, 18_750_001, FAST4, FAST4).unwrap();
        let above = prepare_swd_timing(150_000_000, u32::MAX, FAST4, FAST4).unwrap();
        assert_eq!(boundary.read(), PacingPlan::Fast);
        assert_eq!(boundary.write(), PacingPlan::Fast);
        assert_eq!(just_above.read(), boundary.read());
        assert_eq!(just_above.write(), boundary.write());
        assert_eq!(above.read(), boundary.read());
        assert_eq!(above.write(), boundary.write());
    }

    #[test]
    fn scaled_timing_rounds_toward_a_slower_clock() {
        for request in [1, 1_000_000, 4_000_000, 12_000_001, 18_749_999] {
            let plan = prepare_swd_timing(150_000_000, request, FAST4, FAST4).unwrap();
            let PacingPlan::Scaled(half) = plan.read() else {
                panic!("request {request} unexpectedly selected fast pacing");
            };
            assert_eq!(half.low(), half.high());
            assert!(75_000_000 / half.low() <= request);
        }
    }

    #[test]
    fn profiles_prepare_distinct_read_write_and_jtag_halves() {
        let read = HalfCycleProfile::new(1, 2, 2, 3);
        let write = HalfCycleProfile::new(3, 4, 1, 1);
        let swd = prepare_swd_timing(
            100,
            10,
            PacingProfile::new(1, read),
            PacingProfile::new(1, write),
        )
        .unwrap();
        assert_eq!(swd.sequence_half_cycles(), 5);
        assert_eq!(swd.read(), PacingPlan::Scaled(HalfCycles::new(4, 3)));
        assert_eq!(swd.write(), PacingPlan::Scaled(HalfCycles::new(2, 1)));

        let jtag_profile = HalfCycleProfile::new(2, 1, 4, 2);
        let jtag = prepare_jtag_timing(
            100,
            10,
            1,
            NONE,
            PacingProfile::new(1, jtag_profile),
            PacingProfile::new(1, NONE),
            PacingProfile::new(1, NONE),
        )
        .unwrap();
        assert_eq!(jtag.control(), HalfCycles::symmetric(5));
        assert_eq!(jtag.read(), PacingPlan::Scaled(HalfCycles::new(4, 4)));
        assert_eq!(jtag.write(), PacingPlan::Scaled(HalfCycles::symmetric(5)));
        assert_eq!(jtag.ir(), PacingPlan::Scaled(HalfCycles::symmetric(5)));
    }

    #[test]
    fn jtag_directions_select_independently_at_the_handoff() {
        let scaled_floor = HalfCycleProfile::new(0, 0, 4, 4);
        let read = PacingProfile::new(0, scaled_floor);
        let boundary =
            prepare_jtag_timing(150_000_000, 18_750_000, 4, NONE, read, FAST4, FAST4).unwrap();
        let above =
            prepare_jtag_timing(150_000_000, u32::MAX, 4, NONE, read, FAST4, FAST4).unwrap();
        assert_eq!(
            boundary.read(),
            PacingPlan::Scaled(HalfCycles::symmetric(4))
        );
        assert_eq!(above.read(), boundary.read());
        assert_eq!(boundary.write(), PacingPlan::Fast);
        assert_eq!(above.write(), PacingPlan::Fast);
        assert_eq!(boundary.ir(), PacingPlan::Fast);
        assert_eq!(above.ir(), PacingPlan::Fast);
        assert_eq!(boundary.sequence_half_cycles(), 4);
        assert_eq!(above.sequence_half_cycles(), 4);
    }

    #[test]
    fn fixed_work_subtraction_saturates_before_the_floor() {
        let profile = HalfCycleProfile::new(u32::MAX, u32::MAX, 0, 7);
        let plan = prepare_swd_timing(
            100,
            10,
            PacingProfile::new(1, profile),
            PacingProfile::new(1, NONE),
        )
        .unwrap();
        assert_eq!(plan.read(), PacingPlan::Scaled(HalfCycles::new(1, 7)));
    }

    #[test]
    fn a_zero_threshold_disables_fast_pacing() {
        let plan =
            prepare_jtag_timing(150_000_000, u32::MAX, 0, NONE, SCALED, SCALED, SCALED).unwrap();
        assert_eq!(plan.read(), PacingPlan::Scaled(HalfCycles::symmetric(1)));
        assert_eq!(plan.write(), PacingPlan::Scaled(HalfCycles::symmetric(1)));
        assert_eq!(plan.ir(), PacingPlan::Scaled(HalfCycles::symmetric(1)));
        assert_eq!(plan.sequence_half_cycles(), 1);
    }

    #[test]
    fn jtag_sequence_and_scan_floors_clamp_independently() {
        let scan = PacingProfile::new(0, HalfCycleProfile::new(3, 1, 6, 6));
        let at_floor =
            prepare_jtag_timing(150_000_000, 12_000_000, 7, NONE, scan, scan, scan).unwrap();
        let above = prepare_jtag_timing(150_000_000, u32::MAX, 7, NONE, scan, scan, scan).unwrap();
        assert_eq!(at_floor.sequence_half_cycles(), 7);
        assert_eq!(above.sequence_half_cycles(), 7);
        assert_eq!(at_floor.read(), PacingPlan::Scaled(HalfCycles::new(6, 6)));
        assert_eq!(above.read(), at_floor.read());
        assert_eq!(above.write(), at_floor.write());
        assert_eq!(above.ir(), at_floor.ir());
    }

    #[test]
    fn prepared_plans_keep_a_compact_state() {
        assert_eq!(core::mem::size_of::<PacingPlan>(), 8);
        assert_eq!(core::mem::size_of::<SwdTimingPlan>(), 20);
        assert_eq!(core::mem::size_of::<JtagTimingPlan>(), 36);
    }
}
