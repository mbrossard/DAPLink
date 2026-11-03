//! ST-Link/V2 board half of the F1 SPI-session seam.

use daplink_rs::daplink_core::SCAN_CHAIN_SLOTS;
use daplink_rs::{DelayCycles, PreparedSpin, ScanChainStorage};
use daplink_spi_swd::{nops, JtagPacing, KeptAck, SpiLadder, SpiSwdDeps};
use daplink_stm32f1_hal::{clocks, DynCycleDelay, IoPin, Port};
use daplink_stm32f1_spi_swd::{pac, prepare_flash_gpio_half, F1SpiPort, SpiSwdBoard};

const SPI2_KERNEL_HZ: u32 = 36_000_000;

/// SPI2 on its fixed APB1 kernel, limited to 18 MHz on STM32F103.
const SPI2: SpiLadder<(), 1> = SpiLadder {
    kernels: [((), SPI2_KERNEL_HZ)],
    max_sck_hz: 18_000_000,
};

/// Counted JTAG low halves require at least two delay-loop iterations before sampling.
const JTAG_MIN_LOW_CYCLES: u32 = 6;
/// Counted JTAG high halves retain one delay-loop iteration.
const JTAG_MIN_HIGH_CYCLES: u32 = 3;

#[derive(Copy, Clone)]
pub struct F1JtagPacing {
    low: PreparedSpin,
    high: PreparedSpin,
}

impl JtagPacing for F1JtagPacing {
    const FAST_HALF_CYCLES: u32 = 2;

    fn prepare_counted(&mut self, cycles: u32) {
        // Retain one extra loop per half below the shortest counted timing plateau.
        let half = cycles.div_ceil(2) + if cycles > 12 { 3 } else { 0 };
        self.low = PreparedSpin::new(half.max(JTAG_MIN_LOW_CYCLES));
        self.high = PreparedSpin::new(half.max(JTAG_MIN_HIGH_CYCLES));
    }

    #[inline(always)]
    fn counted_low<D: DelayCycles>(&self, _delay: &mut D, _cycles: u32) {
        self.low.run();
    }

    #[inline(always)]
    fn counted_high<D: DelayCycles>(&self, _delay: &mut D, _cycles: u32) {
        self.high.run();
    }

    #[inline(always)]
    fn counted_output_low<D: DelayCycles>(&self, _delay: &mut D, _cycles: u32) {
        // An unsampled low needs no TDO sampling margin, so it takes the high count.
        self.high.run();
    }

    #[inline(always)]
    fn pace_low(&self) {
        nops::<4>();
    }

    #[inline(always)]
    fn pace_high(&self) {
        nops::<0>();
    }
}

pub struct Board;

impl SpiSwdBoard for Board {
    const CLK_PIN: usize = 13;
    const IO_PIN: usize = 14;
    const TDI_PIN: usize = 7;
    const TDO_PIN: usize = 6;

    fn prepare_gpio_half(_delay: DynCycleDelay, cycles: u32) -> PreparedSpin {
        prepare_flash_gpio_half(cycles)
    }

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
        // SAFETY: single-threaded bring-up; the RMW touches only SPI2EN.
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.apb1enr().modify(|_, w| w.spi2en().set_bit());
        let _ = rcc.apb1enr().read();
    }
}

/// Accelerated dependencies on the same pads as the default engine.
pub fn accel_deps(
    delay: DynCycleDelay,
) -> SpiSwdDeps<F1SpiPort<Board>, IoPin, DynCycleDelay, F1JtagPacing> {
    let port = F1SpiPort::<Board>::new(delay, SPI2_KERNEL_HZ);
    static SCAN: ScanChainStorage<SCAN_CHAIN_SLOTS> = ScanChainStorage::new();
    static KEPT: KeptAck = KeptAck::new(SCAN.jtag_dp());
    SpiSwdDeps::new(
        port,
        IoPin::new(Port::B, 13),
        IoPin::new(Port::B, 14),
        IoPin::open_drain(Port::B, 0),
        DynCycleDelay::new(clocks::CPU_HZ),
        F1JtagPacing {
            low: PreparedSpin::new(JTAG_MIN_LOW_CYCLES),
            high: PreparedSpin::new(JTAG_MIN_HIGH_CYCLES),
        },
        SPI2,
        SCAN.init(),
        &KEPT,
    )
}
