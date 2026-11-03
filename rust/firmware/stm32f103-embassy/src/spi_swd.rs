//! nanoDAP board half of the SPI-session SWD seam, composed only under `accel`.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::ScanChainStorage;
use daplink_spi_swd::{KeptAck, NoFastJtag, SpiLadder, SpiSwdDeps};
#[cfg(feature = "flash")]
use daplink_spi_swd::{SpiJtag, SpiSwd};
use daplink_stm32f1_embassy::{DynCycleDelay, IoPin};
use daplink_stm32f1_spi_swd::{pac, F1SpiPort, SpiSwdBoard};

use embassy_stm32::peripherals::{PA5, PA7, PB0, PB13, PB14, SPI2};
use embassy_stm32::Peri;

/// SPI2's 18 MHz datasheet limit on a 36 MHz APB1.
pub const SPI2_MAX_SCK_HZ: u32 = 18_000_000;

/// SWCLK/TCK = PB13, SWDIO/TMS = PB14 (MISO, observe-only), TDI = PA7, TDO = PA5; PB15 is D+.
pub struct Board;

impl SpiSwdBoard for Board {
    const CLK_PIN: usize = 13;
    const IO_PIN: usize = 14;
    const TDI_PIN: usize = 7;
    const TDO_PIN: usize = 5;

    fn clk_port() -> &'static pac::gpioa::RegisterBlock {
        // SAFETY: the singleton register block is shared only through atomic/set-or-read access.
        unsafe { &*pac::GPIOB::ptr() }
    }
    fn jtag_port() -> &'static pac::gpioa::RegisterBlock {
        // SAFETY: the singleton register block is shared only through atomic/set-or-read access.
        unsafe { &*pac::GPIOA::ptr() }
    }
    fn spi() -> &'static pac::spi2::RegisterBlock {
        // SAFETY: this composition is the sole owner of SPI2.
        unsafe { &*pac::SPI2::ptr() }
    }

    fn init_clocks() {
        // No kernel mux on the F1: only the SPI2 gate is written.
        // SAFETY: single-threaded bring-up; the RMW touches only SPI2EN.
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.apb1enr().modify(|_, w| w.spi2en().set_bit());
        let _ = rcc.apb1enr().read();
    }
}

pub type AccelDeps = SpiSwdDeps<F1SpiPort<Board>, IoPin, DynCycleDelay, NoFastJtag>;
#[cfg(feature = "flash")]
pub type AccelSwd = SpiSwd<F1SpiPort<Board>, IoPin, DynCycleDelay, NoFastJtag>;
#[cfg(feature = "flash")]
pub type AccelJtag = SpiJtag<F1SpiPort<Board>, IoPin, DynCycleDelay, NoFastJtag>;

/// `kernel_hz` is PCLK1; `jtag_delay` paces sequences and the JTAG bit path.
/// `_spi`, `tdi` and `tdo` are claimed so nothing else configures them; the port programs them.
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
    let _ = (tdi, tdo);
    let port = F1SpiPort::<Board>::new(delay, kernel_hz);
    let swclk = IoPin::new(swclk);
    let swdio = IoPin::new(swdio);
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
        jtag_delay,
        NoFastJtag,
        SpiLadder {
            kernels: [((), kernel_hz)],
            max_sck_hz: SPI2_MAX_SCK_HZ,
        },
        SCAN.init(),
        &KEPT,
    )
}
