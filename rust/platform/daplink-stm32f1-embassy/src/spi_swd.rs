//! F1 register seam for the SPI-session SWD engine: the block can clock and observe the wire, never drive it.

use core::cell::Cell;
use core::marker::PhantomData;

use daplink_rs::DelayCycles;
use daplink_spi_swd::session::half_period_cycles;
use daplink_spi_swd::{Dir, JtagPortOps, SpiJtag, SpiPortOps, SpiSwd, SpiSwdDeps};
use embassy_stm32::pac;
use embassy_stm32::pac::spi::vals::{Br, Cpha, Cpol, Dff, Lsbfirst, Mstr};

use crate::DynCycleDelay;

/// CNF+MODE nibbles (RM0008): push-pull output at 50 MHz, alternate-function push-pull at
/// 50 MHz, input floating, input with pull (ODR selects up/down).
const CFG_OUTPUT_PP_50M: u32 = 0b0011;
const CFG_AF_PP_50M: u32 = 0b1011;
const CFG_INPUT_FLOATING: u32 = 0b0100;
const CFG_INPUT_PULL: u32 = 0b1000;

/// The per-board half of the seam, a zero-sized type so every member monomorphizes to a constant.
pub trait SpiSwdBoard {
    /// Pin index of SWCLK / TCK within [`clk_port`](Self::clk_port) -- also the SPI SCK pad.
    const CLK_PIN: usize;
    /// Pin index of SWDIO / TMS within [`clk_port`](Self::clk_port) -- also the SPI MISO pad.
    const IO_PIN: usize;
    /// Pin index of TDI within [`jtag_port`](Self::jtag_port).
    const TDI_PIN: usize;
    /// Pin index of TDO within [`jtag_port`](Self::jtag_port).
    const TDO_PIN: usize;

    /// GPIO block carrying SWCLK/SWDIO.
    fn clk_port() -> pac::gpio::Gpio;
    /// GPIO block carrying TDI/TDO.
    fn jtag_port() -> pac::gpio::Gpio;
    /// The SPI register block whose SCK/MISO pads are the two probe lines.
    fn spi() -> pac::spi::Spi;
    /// Ungate the SPI peripheral clock; there is no kernel mux on this family.
    fn init_clocks();
}

/// Bound on every SPI status poll: an undrivable rung must fail the transfer, not the USB task.
const SPI_POLL_LIMIT: u32 = 200_000;

/// Receive frames one session can retain; the engine's framing plan never exceeds one.
const RX_FRAMES: usize = 4;

/// The F1 seam: session geometry latched by `program`, rendered by `push8`/`start`.
pub struct F1SpiPort<M: SpiSwdBoard> {
    delay: DynCycleDelay,
    kernel_hz: u32,
    bits: Cell<u8>,
    frames: Cell<u16>,
    dir: Cell<Dir>,
    mbr: Cell<u8>,
    /// GPIO half-period (CPU cycles) tracking the session's baud rung.
    half: Cell<u32>,
    rx: [Cell<u32>; RX_FRAMES],
    rx_count: Cell<usize>,
    rx_pos: Cell<usize>,
    _board: PhantomData<M>,
}

/// Rewrite one pin's CNF+MODE nibble in CRL/CRH.
fn set_cfg(port: pac::gpio::Gpio, pin: usize, cfg: u32) {
    let shift = 4 * (pin % 8);
    let cr = port.cr(pin / 8);
    let v = (cr.read().0 & !(0b1111 << shift)) | (cfg << shift);
    cr.write_value(pac::gpio::regs::Cr(v));
}

impl<M: SpiSwdBoard> F1SpiPort<M> {
    /// Claim the SPI block and park TDI as a push-pull output idle low, TDO as a plain input.
    pub fn new(delay: DynCycleDelay, kernel_hz: u32) -> Self {
        M::init_clocks();

        M::jtag_port().bsrr().write(|w| w.set_br(M::TDI_PIN, true));
        set_cfg(M::jtag_port(), M::TDI_PIN, CFG_OUTPUT_PP_50M);
        set_cfg(M::jtag_port(), M::TDO_PIN, CFG_INPUT_FLOATING);

        Self {
            delay,
            kernel_hz,
            bits: Cell::new(8),
            frames: Cell::new(0),
            dir: Cell::new(Dir::Tx),
            mbr: Cell::new(7),
            half: Cell::new(1),
            rx: [const { Cell::new(0) }; RX_FRAMES],
            rx_count: Cell::new(0),
            rx_pos: Cell::new(0),
            _board: PhantomData,
        }
    }

    #[inline(always)]
    fn clk_high(&self) {
        M::clk_port().bsrr().write(|w| w.set_bs(M::CLK_PIN, true));
    }

    #[inline(always)]
    fn clk_low(&self) {
        M::clk_port().bsrr().write(|w| w.set_br(M::CLK_PIN, true));
    }

    #[inline(always)]
    fn io_set(&self, high: bool) {
        M::clk_port().bsrr().write(|w| {
            if high {
                w.set_bs(M::IO_PIN, true);
            } else {
                w.set_br(M::IO_PIN, true);
            }
        });
    }

    #[inline(always)]
    fn io_get(&self) -> u32 {
        (M::clk_port().idr().read().0 >> M::IO_PIN) & 1
    }

    #[inline(always)]
    fn wait_half(&self) {
        let mut d = self.delay;
        d.delay_cycles(self.half.get());
    }

    /// Park SWCLK as a driven-high GPIO (the mode-3 idle level, latched before any mode change).
    fn clk_to_gpio(&self) {
        self.clk_high();
        set_cfg(M::clk_port(), M::CLK_PIN, CFG_OUTPUT_PP_50M);
    }

    /// Shift up to 8 bits of `byte` out LSB-first on GPIO; the target samples the rising edge.
    fn gpio_tx(&self, byte: u8, bits: usize) {
        for i in 0..bits {
            self.io_set(byte & (1 << i) != 0);
            self.clk_low();
            self.wait_half();
            self.clk_high();
            self.wait_half();
        }
    }

    /// Clock `bits` in LSB-first on GPIO, sampling in the low half of each cell.
    fn gpio_rx(&self, bits: usize) -> u32 {
        let mut v = 0u32;
        for i in 0..bits {
            self.clk_low();
            self.wait_half();
            v |= self.io_get() << i;
            self.clk_high();
            self.wait_half();
        }
        v
    }

    /// Clock 16 target-driven bits through the SPI block: one dummy full-duplex frame at the rung.
    fn spi_rx16(&self) -> u32 {
        let s = M::spi();
        s.cr1().write(|w| {
            w.set_cpha(Cpha::SECOND_EDGE);
            w.set_cpol(Cpol::IDLE_HIGH);
            w.set_mstr(Mstr::MASTER);
            w.set_br(Br::from_bits(self.mbr.get()));
            w.set_lsbfirst(Lsbfirst::LSBFIRST);
            w.set_ssi(true);
            w.set_ssm(true);
            w.set_dff(Dff::BITS16);
            w.set_spe(true);
        });
        // `DR` then `SR` drops what a bounded-out pass left and clears a latched overrun.
        let _ = s.dr().read();
        let _ = s.sr().read();

        // Mode 3 idles SCK high and the pad is parked driven-high, so the handover is edge-free.
        set_cfg(M::clk_port(), M::CLK_PIN, CFG_AF_PP_50M);

        s.dr().write(|w| w.set_dr(0));
        let mut spins = 0u32;
        while !s.sr().read().rxne() {
            spins += 1;
            if spins > SPI_POLL_LIMIT {
                break;
            }
        }
        let v = s.dr().read().dr() as u32;
        spins = 0;
        while s.sr().read().bsy() {
            spins += 1;
            if spins > SPI_POLL_LIMIT {
                break;
            }
        }
        self.clk_to_gpio();
        s.cr1().write(|w| {
            w.set_mstr(Mstr::MASTER);
            w.set_ssi(true);
            w.set_ssm(true);
        });
        v
    }

    fn push_rx(&self, v: u32) {
        let n = self.rx_count.get();
        debug_assert!(n < RX_FRAMES, "receive session exceeds the frame cap");
        self.rx[n.min(RX_FRAMES - 1)].set(v);
        self.rx_count.set(n + 1);
    }
}

impl<M: SpiSwdBoard> SpiPortOps for F1SpiPort<M> {
    fn configure(&mut self) {
        M::spi().cr2().write(|_| {});
    }

    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
        self.mbr.set(mbr);
        self.bits.set(bits);
        self.frames.set(frames);
        self.dir.set(dir);
        self.half.set(half_period_cycles(
            self.delay.cpu_clock(),
            self.kernel_hz >> (mbr + 1),
        ));
    }

    fn enable(&self) {
        // The release to pull-up on receive is what serves an ACK session's leading turnaround.
        match self.dir.get() {
            Dir::Tx => set_cfg(M::clk_port(), M::IO_PIN, CFG_OUTPUT_PP_50M),
            Dir::Rx => {
                self.io_set(true);
                set_cfg(M::clk_port(), M::IO_PIN, CFG_INPUT_PULL);
            }
        }
    }

    fn disable(&self) {}

    fn start(&self) {
        if self.dir.get() == Dir::Rx {
            let bits = self.bits.get() as usize;
            for _ in 0..self.frames.get() {
                // A 17-bit frame is 16 SPI bits plus one GPIO bit; narrower ones are all GPIO.
                let v = if bits == 17 {
                    self.spi_rx16() | (self.gpio_rx(1) << 16)
                } else {
                    self.gpio_rx(bits)
                };
                self.push_rx(v);
            }
        }
    }

    fn push8(&self, byte: u8) {
        // Transmit frames render immediately; the wire tolerates the inter-frame gap.
        if self.dir.get() == Dir::Tx {
            self.gpio_tx(byte, (self.bits.get() as usize).min(8));
        }
    }

    fn wait_eot(&self) {}

    fn pop32(&self) -> u32 {
        let i = self.rx_pos.get();
        self.rx_pos.set(i + 1);
        self.rx[i.min(RX_FRAMES - 1)].get()
    }

    fn clear_flags(&self) {
        self.rx_count.set(0);
        self.rx_pos.set(0);
    }

    fn pads_to_spi(&mut self) {
        // Session idle: SWCLK a driven-high GPIO, SWDIO released to its pull-up.
        self.clk_to_gpio();
        self.io_set(true);
        set_cfg(M::clk_port(), M::IO_PIN, CFG_INPUT_PULL);
    }

    fn clk_pad_to_spi(&mut self) {
        self.clk_to_gpio();
    }
}

impl<M: SpiSwdBoard> JtagPortOps for F1SpiPort<M> {
    fn jtag_pads_acquire(&mut self) {
        M::jtag_port().bsrr().write(|w| w.set_br(M::TDI_PIN, true));
        set_cfg(M::jtag_port(), M::TDI_PIN, CFG_OUTPUT_PP_50M);
        set_cfg(M::jtag_port(), M::TDO_PIN, CFG_INPUT_FLOATING);
    }

    fn jtag_pads_release(&mut self) {
        M::jtag_port().bsrr().write(|w| w.set_bs(M::TDI_PIN, true));
        set_cfg(M::jtag_port(), M::TDI_PIN, CFG_INPUT_PULL);
    }

    #[inline(always)]
    fn tck_low(&self) {
        self.clk_low();
    }

    #[inline(always)]
    fn tck_high(&self) {
        self.clk_high();
    }

    #[inline(always)]
    fn tms_set(&self, high: bool) {
        self.io_set(high);
    }

    #[inline(always)]
    fn tdi_set(&self, high: bool) {
        M::jtag_port().bsrr().write(|w| {
            if high {
                w.set_bs(M::TDI_PIN, true);
            } else {
                w.set_br(M::TDI_PIN, true);
            }
        });
    }

    #[inline(always)]
    fn tdo_get(&self) -> u32 {
        (M::jtag_port().idr().read().0 >> M::TDO_PIN) & 1
    }

    fn tdi_is_high(&mut self) -> bool {
        M::jtag_port().idr().read().0 & (1 << M::TDI_PIN) != 0
    }

    fn tdo_is_high(&mut self) -> bool {
        M::jtag_port().idr().read().0 & (1 << M::TDO_PIN) != 0
    }
}

/// The SPI-session DAP engine over a board seam `M`: SPI-assisted SWD plus GPIO bit-bang JTAG.
pub type SpiSwdJtagDap<
    'a,
    M,
    IO,
    D,
    J,
    SWO = daplink_rs::NoSwo,
    V = daplink_rs::NoVendor,
    LEDS = daplink_rs::NoopLeds,
> = daplink_rs::DapProbe<
    'a,
    SpiSwdDeps<F1SpiPort<M>, IO, D, J>,
    LEDS,
    D,
    SpiJtag<F1SpiPort<M>, IO, D, J>,
    SpiSwd<F1SpiPort<M>, IO, D, J>,
    SWO,
    V,
>;
