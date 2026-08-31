//! STM32H523 board half of the SPI-session SWD seam, composed only under `accel`.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::ScanChainStorage;
use daplink_spi_swd::{nops, JtagPacing, SpiSwdDeps};
use daplink_stm32_embassy::{DynCycleDelay, IoPin, SpiSwdBoard, StmSpiPort};

use embassy_stm32::gpio::Flex;
use embassy_stm32::pac;
use embassy_stm32::pac::rcc::vals::Spi2sel;
use embassy_stm32::peripherals::{PA6, PA7, PB0, PB13, PB14, SPI2};
use embassy_stm32::Peri;

/// SPI master clock ceiling, below the 32 MHz rung this kernel can reach but this board cannot
/// attach on, so no host request selects it.
#[cfg(not(feature = "no-clamp"))]
pub const SPI2_MAX_SCK_HZ: u32 = 30_000_000;

/// Bench-only (`no-clamp`): the unclamped ladder, re-exposing the rung that cannot attach.
#[cfg(feature = "no-clamp")]
pub const SPI2_MAX_SCK_HZ: u32 = u32::MAX;

/// This board's half of the seam: SWCLK/TCK = PB13, SWDIO/TMS = PB14 (the MISO pad, reached as
/// the half-duplex data line via `CFG2.IOSWP`), TDI = PA7, TDO = PA6.
pub struct Board;

impl SpiSwdBoard for Board {
    const CLK_PIN: usize = 13;
    const IO_PIN: usize = 14;
    const TDI_PIN: usize = 7;
    const TDO_PIN: usize = 6;
    const SPI_AF: u8 = 5;

    fn clk_port() -> pac::gpio::Gpio {
        pac::GPIOB
    }
    fn jtag_port() -> pac::gpio::Gpio {
        pac::GPIOA
    }
    fn spi() -> pac::spi::Spi {
        pac::SPI2
    }

    fn init_clocks() {
        // Route the SPI2 kernel clock to per_ck: the reset-default pll1_q source is not enabled.
        pac::RCC.ccipr3().modify(|w| w.set_spi2sel(Spi2sel::PER));
        pac::RCC.apb1lenr().modify(|w| w.set_spi2en(true));
    }
}

/// This firmware's composed dependency type: the session engine over the shared STM32 seam.
pub type AccelDeps = SpiSwdDeps<StmSpiPort<Board>, IoPin, DynCycleDelay, H5FastJtag>;

/// Wire the session transport on the board's probe pads; `kernel_hz` is the SPI2 kernel clock,
/// `delay` times the GPIO fallback and the JTAG bit path.
#[allow(clippy::too_many_arguments)]
pub fn accel_deps(
    _spi: Peri<'static, SPI2>,
    swclk: Peri<'static, PB13>,
    swdio: Peri<'static, PB14>,
    nreset: Peri<'static, PB0>,
    tdi: Peri<'static, PA7>,
    tdo: Peri<'static, PA6>,
    delay: DynCycleDelay,
    kernel_hz: u32,
) -> AccelDeps {
    let port = StmSpiPort::<Board>::new(Flex::new(tdi), Flex::new(tdo));
    let swclk = IoPin::new(swclk);
    let swdio = IoPin::new(swdio);
    // nRESET shares its net with the target's reset supervisor, so it drives open-drain.
    let nreset = IoPin::new_open_drain(nreset);

    // The `DAP_JTAG_Configure` bound is exclusive, so a single-TAP chain needs two slots.
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    SpiSwdDeps::new(
        port,
        swclk,
        swdio,
        nreset,
        delay,
        H5FastJtag,
        kernel_hz,
        SPI2_MAX_SCK_HZ,
        SCAN.init(),
    )
}

/// Nops pacing the low half of a fast JTAG clock: the TCK-fall to TDO-sample margin.
const LOW_NOPS: usize = 18;

/// Nops pacing the high half: TCK-high pulse width only.
const HIGH_NOPS: usize = 8;

/// This board's fast-JTAG pacing ladder.
pub struct H5FastJtag;

impl JtagPacing for H5FastJtag {
    /// Half-period cycle count at or below which the fast word path runs.
    const FAST_HALF_CYCLES: u32 = 8;

    #[inline(always)]
    fn pace_low(&self) {
        nops::<LOW_NOPS>();
    }

    #[inline(always)]
    fn pace_high(&self) {
        nops::<HIGH_NOPS>();
    }
}
