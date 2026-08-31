//! nanoDAP board half of the SPI-session SWD seam, composed only under `accel`.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::ScanChainStorage;
use daplink_spi_swd::{NoFastJtag, SpiSwdDeps};
use daplink_stm32f1_embassy::{DynCycleDelay, F1SpiPort, IoPin, SpiSwdBoard};

use embassy_stm32::pac;
use embassy_stm32::peripherals::{PA5, PA7, PB0, PB13, PB14, SPI2};
use embassy_stm32::Peri;

/// SPI master clock ceiling: 18 MHz, SPI2's datasheet limit on a 36 MHz APB1.
pub const SPI2_MAX_SCK_HZ: u32 = 18_000_000;

/// This board's half of the seam: SWCLK/TCK = PB13, SWDIO/TMS = PB14 (the MISO pad, observe-only,
/// which forces the hybrid rendering), TDI = PA7, TDO = PA5; MOSI (PB15) is the D+ switch.
pub struct Board;

impl SpiSwdBoard for Board {
    const CLK_PIN: usize = 13;
    const IO_PIN: usize = 14;
    const TDI_PIN: usize = 7;
    const TDO_PIN: usize = 5;

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
        // No kernel mux on the F1: SPI2 divides PCLK1 directly, so only the gate is written.
        pac::RCC.apb1enr().modify(|w| w.set_spi2en(true));
        let _ = pac::RCC.apb1enr().read();
    }
}

/// This firmware's composed dependency type: the session engine over the F1 seam, with the
/// counted spin pacing every JTAG clock.
pub type AccelDeps = SpiSwdDeps<F1SpiPort<Board>, IoPin, DynCycleDelay, NoFastJtag>;

/// Wire the session transport on the board's probe pads; `kernel_hz` is PCLK1, `delay` times the
/// GPIO-rendered spans, and `jtag_delay` paces the sequences and the whole JTAG bit path.
#[allow(clippy::too_many_arguments)]
pub fn accel_deps(
    _spi: Peri<'static, SPI2>,
    swclk: Peri<'static, PB13>,
    swdio: Peri<'static, PB14>,
    nreset: Peri<'static, PB0>,
    tdi: Peri<'static, PA7>,
    tdo: Peri<'static, PA5>,
    delay: DynCycleDelay,
    jtag_delay: DynCycleDelay,
    kernel_hz: u32,
) -> AccelDeps {
    // The dedicated JTAG lines are claimed by name and reached through the seam's constants.
    let _ = (tdi, tdo);
    let port = F1SpiPort::<Board>::new(delay, kernel_hz);
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
        jtag_delay,
        NoFastJtag,
        kernel_hz,
        SPI2_MAX_SCK_HZ,
        SCAN.init(),
    )
}
