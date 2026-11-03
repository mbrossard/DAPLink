//! This board's cell fits at 250 MHz; each scaled class is checked never faster at compile time.

#[cfg(not(feature = "accel"))]
use daplink_spi_swd::pace::{ClassFit, Fixed, Floors};

#[cfg(not(feature = "accel"))]
use crate::clocks::CPU_HZ;

/// A build-time override, decimal; an empty or non-decimal value fails the build.
const fn env_count(value: Option<&str>, default: u32) -> u32 {
    match value {
        None => default,
        Some(text) => {
            let bytes = text.as_bytes();
            assert!(!bytes.is_empty(), "a pacing override must not be empty");
            let mut i = 0;
            let mut count = 0u32;
            while i < bytes.len() {
                let digit = bytes[i];
                assert!(
                    digit >= b'0' && digit <= b'9',
                    "a pacing override must be a decimal count"
                );
                count = count * 10 + (digit - b'0') as u32;
                i += 1;
            }
            count
        }
    }
}

#[cfg(not(feature = "accel"))]
const fn fit(low: i32, high: i32, period: i32, floors: Floors) -> ClassFit {
    ClassFit {
        fixed: Fixed { low, high, period },
        floors,
    }
}

#[cfg(not(feature = "accel"))]
const fn floors(low_ps: u32, high_ps: u32, period_ps: u32) -> Floors {
    Floors {
        low_ps,
        high_ps,
        period_ps,
    }
}

/// SWD block cells the probe drives.
#[cfg(not(feature = "accel"))]
const BLOCK_DRIVEN: Floors = floors(20_000, 20_000, 0);
/// Target data follows the rise and is sampled by the next low: the window is rise to rise.
#[cfg(not(feature = "accel"))]
const BLOCK_SAMPLED: Floors = floors(20_000, 20_000, 44_000);

#[cfg(not(feature = "accel"))]
pub const BLOCK_CONTROL: ClassFit = fit(-1, 0, 2, BLOCK_DRIVEN);
#[cfg(not(feature = "accel"))]
pub const BLOCK_ACK: ClassFit = fit(-1, 0, 2, BLOCK_SAMPLED);
#[cfg(not(feature = "accel"))]
pub const BLOCK_READ: ClassFit = fit(5, 2, 8, BLOCK_SAMPLED);
#[cfg(not(feature = "accel"))]
pub const BLOCK_WRITE: ClassFit = fit(-1, 3, 4, BLOCK_DRIVEN);

/// The block's highest wire clock: at 25 MHz every block class reaches its plateau.
#[cfg(not(feature = "accel"))]
pub const BLOCK_MAX_HZ: u32 = env_count(option_env!("DAPLINK_BLOCK_MAX_HZ"), 25_000_000);

/// Lean JTAG cells the probe drives: full-swing 20 ns halves.
#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
const JTAG_DRIVEN: Floors = floors(20_000, 20_000, 0);
/// Sampled lows: twice the TDO output delay, plus a sample and a cycle of lead.
#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
const JTAG_SAMPLED: Floors = floors(36_000, 20_000, 0);

#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
pub const JTAG_CONTROL: ClassFit = fit(-2, 0, 2, JTAG_DRIVEN);
#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
pub const JTAG_ACK: ClassFit = fit(1, 5, 8, JTAG_SAMPLED);
#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
pub const JTAG_READ: ClassFit = fit(2, 2, 5, JTAG_SAMPLED);
#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
pub const JTAG_WRITE: ClassFit = fit(-1, 7, 8, JTAG_DRIVEN);
#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
pub const JTAG_IR: ClassFit = fit(-4, 7, 5, JTAG_DRIVEN);

#[cfg(not(feature = "accel"))]
const _: () = {
    BLOCK_CONTROL.verify(CPU_HZ);
    BLOCK_ACK.verify(CPU_HZ);
    BLOCK_READ.verify(CPU_HZ);
    BLOCK_WRITE.verify(CPU_HZ);
};

#[cfg(not(any(feature = "accel", feature = "plain-bitbang")))]
const _: () = {
    JTAG_CONTROL.verify(CPU_HZ);
    JTAG_ACK.verify(CPU_HZ);
    JTAG_READ.verify(CPU_HZ);
    JTAG_WRITE.verify(CPU_HZ);
    JTAG_IR.verify(CPU_HZ);
};

/// dap-rs's resident cells: fixed work per half, and the shortest half, covering a sampled low.
#[cfg(not(feature = "accel"))]
pub struct Resident {
    pub fixed_cycles: u32,
    pub floor_ps: u32,
}

/// No fixed work: a write's ACK-to-turnaround cell is two halves of bare store and spin.
#[cfg(not(feature = "accel"))]
pub const RESIDENT: Resident = Resident {
    fixed_cycles: 0,
    floor_ps: 44_000,
};

/// The `accel` JTAG nop ladder, and the half period at or below which it runs.
#[cfg(feature = "accel")]
pub struct Ladder {
    pub low_nops: usize,
    pub high_nops: usize,
    pub fast_half_cycles: u32,
}

#[cfg(feature = "accel")]
pub const ACCEL_JTAG: Ladder = Ladder {
    low_nops: env_count(option_env!("DAPLINK_JTAG_LOW_NOPS"), 18) as usize,
    high_nops: env_count(option_env!("DAPLINK_JTAG_HIGH_NOPS"), 8) as usize,
    fast_half_cycles: env_count(option_env!("DAPLINK_JTAG_FAST_HALF_CYCLES"), 8),
};
