//! ADI register addresses, as `A[3:2]` for the DP and byte offsets for the MEM-AP, and their bits.

/// DPIDR on reads, ABORT on writes.
pub const DP_DPIDR: u8 = 0;
/// CTRL/STAT.
pub const DP_CTRL_STAT: u8 = 1;
/// SELECT; RESEND on an SWD read.
pub const DP_SELECT: u8 = 2;
/// RDBUFF.
pub const DP_RDBUFF: u8 = 3;

/// ABORT.DAPABORT.
pub const DAPABORT: u32 = 1 << 0;
/// ABORT.STKERRCLR.
pub const STKERRCLR: u32 = 1 << 2;
/// ABORT.WDERRCLR.
pub const WDERRCLR: u32 = 1 << 3;

/// CTRL/STAT.STICKYERR; on JTAG a write of one clears it.
pub const STICKYERR: u32 = 1 << 5;
/// CTRL/STAT.WDATAERR.
pub const WDATAERR: u32 = 1 << 7;
/// The CTRL/STAT bits the DP holds: the power-up requests, MASKLANE and ORUNDETECT.
pub const CTRL_MASK: u32 = 0x5000_0F01;

/// CSW.
pub const CSW: u8 = 0x00;
/// TAR.
pub const TAR: u8 = 0x04;
/// DRW.
pub const DRW: u8 = 0x0C;
/// BD0; BD1 to BD3 follow.
pub const BD0: u8 = 0x10;
/// BASE.
pub const BASE: u8 = 0xF8;
/// IDR.
pub const IDR: u8 = 0xFC;
