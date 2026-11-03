//! The SWD pads a CPU-paced cell drives, as a per-board pin map.

use embassy_stm32::pac;

/// One board wiring, a zero-sized type so every member is a constant.
pub trait SwjPads {
    /// Pin index of SWCLK within [`swj_port`](Self::swj_port).
    const SWCLK: usize;
    /// Pin index of SWDIO within [`swj_port`](Self::swj_port).
    const SWDIO: usize;

    /// GPIO block carrying SWCLK and SWDIO.
    fn swj_port() -> pac::gpio::Gpio;
}
