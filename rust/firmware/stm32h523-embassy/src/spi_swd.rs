//! STM32H523 board half of the SPI-session SWD seam, composed only under `accel`.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::ScanChainStorage;
use daplink_spi_swd::{KeptAck, SpiLadder, SpiSwdDeps};
#[cfg(feature = "flash")]
use daplink_spi_swd::{SpiJtag, SpiSwd};
#[cfg(not(feature = "accel-duplex"))]
use daplink_stm32_embassy::StmSpiPort;
#[cfg(feature = "accel-duplex")]
use daplink_stm32_embassy::{DuplexBoard, StmDuplexPort};
use daplink_stm32_embassy::{DynCycleDelay, IoPin, SpiSwdBoard};

use crate::jtag::H5FastJtag;

use embassy_stm32::gpio::Flex;
use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::Ospeedr;
use embassy_stm32::pac::rcc::vals::Spi2sel;
use embassy_stm32::peripherals::{PA6, PA7, PB0, PB10, PB14, PB15, SPI2};
use embassy_stm32::Peri;

/// This board's half of the seam: SCK on PB10, data on PB15 (PB14 under `accel-duplex`).
pub struct Board;

#[cfg(not(feature = "accel-duplex"))]
pub type Port = StmSpiPort<Board>;
#[cfg(feature = "accel-duplex")]
pub type Port = StmDuplexPort<Board>;

#[cfg(feature = "accel-duplex")]
impl DuplexBoard for Board {
    const DRIVE_PIN: usize = 15;
}

impl SpiSwdBoard for Board {
    const CLK_PIN: usize = 10;
    const IO_PIN: usize = if cfg!(feature = "accel-duplex") {
        14
    } else {
        15
    };
    const TDI_PIN: usize = 7;
    const TDO_PIN: usize = 6;
    const SPI_AF: u8 = 5;
    // Speed class 11: class 01's slower edges skew a 20 ns half-period.
    const PAD_SPEED: Ospeedr = Ospeedr::VERY_HIGH_SPEED;
    type Source = Spi2sel;

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
        // Start the SPI2 kernel on per_ck; `route` switches it to PLL1 Q or PLL2 P.
        pac::RCC.ccipr3().modify(|w| w.set_spi2sel(Spi2sel::PER));
        pac::RCC.apb1lenr().modify(|w| w.set_spi2en(true));
    }

    fn route(source: Spi2sel) {
        pac::RCC.ccipr3().modify(|w| w.set_spi2sel(source));
    }
}

pub type AccelDeps = SpiSwdDeps<Port, IoPin, DynCycleDelay, H5FastJtag, 3>;
#[cfg(feature = "flash")]
pub type AccelSwd = SpiSwd<Port, IoPin, DynCycleDelay, H5FastJtag, 3>;
#[cfg(feature = "flash")]
pub type AccelJtag = SpiJtag<Port, IoPin, DynCycleDelay, H5FastJtag, 3>;

/// `ladder` is SPI2's clock plan; `delay` times the GPIO fallback and JTAG.
#[allow(clippy::too_many_arguments)]
pub fn accel_deps(
    _spi: Peri<'static, SPI2>,
    swclk: Peri<'static, PB10>,
    pb15: Peri<'static, PB15>,
    pb14: Peri<'static, PB14>,
    nreset: Peri<'static, PB0>,
    tdi: Peri<'static, PA7>,
    tdo: Peri<'static, PA6>,
    delay: DynCycleDelay,
    ladder: SpiLadder<Spi2sel, 3>,
) -> AccelDeps {
    let speed = Board::PAD_SPEED.to_bits() as u32;
    // Half duplex: data on PB15, PB14 an unpulled input. Duplex: PB14 senses, PB15 transmits.
    #[cfg(not(feature = "accel-duplex"))]
    let (swdio, other) = (IoPin::with_speed(pb15, speed), Flex::new(pb14));
    #[cfg(feature = "accel-duplex")]
    let (swdio, other) = (IoPin::with_speed(pb14, speed), Flex::new(pb15));
    let port = Port::new(Flex::new(tdi), Flex::new(tdo), other);
    let swclk = IoPin::with_speed(swclk, speed);
    // nRESET shares the target's reset supervisor net, so it drives open-drain.
    let nreset = IoPin::new_open_drain(nreset);

    // The `DAP_JTAG_Configure` bound is exclusive: one TAP needs two slots.
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    static KEPT: KeptAck = KeptAck::new(SCAN.jtag_dp());
    SpiSwdDeps::new(
        port,
        swclk,
        swdio,
        nreset,
        delay,
        H5FastJtag,
        ladder,
        SCAN.init(),
        &KEPT,
    )
}
