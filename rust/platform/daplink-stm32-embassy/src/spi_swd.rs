//! Shared STM32 register seam for the SPI-session SWD engine; per-board facts sit behind [`SpiSwdBoard`].

use daplink_spi_swd::{Dir, JtagPortOps, SpiJtag, SpiPortOps, SpiSwd, SpiSwdDeps};
use embassy_stm32::gpio::{Flex, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::{Moder, Ospeedr};
use embassy_stm32::pac::spi::vals::{Comm, Cpha, Cpol, Hddir, Lsbfirst, Master, Mbr};

/// The per-board half of the seam, a zero-sized type so every member monomorphizes to a constant.
pub trait SpiSwdBoard {
    /// Pin index of SWCLK / TCK within [`clk_port`](Self::clk_port) -- also the SPI SCK pad.
    const CLK_PIN: usize;
    /// Pin index of SWDIO / TMS within [`clk_port`](Self::clk_port) -- the half-duplex data pad.
    const IO_PIN: usize;
    /// Pin index of TDI within [`jtag_port`](Self::jtag_port).
    const TDI_PIN: usize;
    /// Pin index of TDO within [`jtag_port`](Self::jtag_port).
    const TDO_PIN: usize;
    /// Alternate-function number mapping the two shared pads to the SPI block.
    const SPI_AF: u8;

    /// GPIO block carrying SWCLK/SWDIO.
    fn clk_port() -> pac::gpio::Gpio;
    /// GPIO block carrying TDI/TDO.
    fn jtag_port() -> pac::gpio::Gpio;
    /// The SPI register block this engine drives.
    fn spi() -> pac::spi::Spi;

    /// Select the SPI kernel clock source and ungate the peripheral clock.
    fn init_clocks();
}

/// The SPI block plus the GPIO bit operations on the probe pads; JTAG edges go through `BSRR`/`IDR`.
pub struct StmSpiPort<M: SpiSwdBoard> {
    tdi: Flex<'static>,
    tdo: Flex<'static>,
    _board: core::marker::PhantomData<M>,
}

impl<M: SpiSwdBoard> StmSpiPort<M> {
    /// Claim the SPI block and park TDI/TDO in their idle roles; `tdi`/`tdo` must match
    /// [`SpiSwdBoard::TDI_PIN`] and [`SpiSwdBoard::TDO_PIN`].
    pub fn new(mut tdi: Flex<'static>, mut tdo: Flex<'static>) -> Self {
        M::init_clocks();

        let g = M::clk_port();
        g.afr(1).modify(|w| {
            // AFR high half covers pins 8..=15, indexed by `pin - 8`.
            w.set_afr(M::CLK_PIN - 8, M::SPI_AF);
            w.set_afr(M::IO_PIN - 8, M::SPI_AF);
        });
        // High slew on both pads: the data pad drives on transmit phases.
        g.ospeedr().modify(|w| {
            w.set_ospeedr(M::CLK_PIN, Ospeedr::VERY_HIGH_SPEED);
            w.set_ospeedr(M::IO_PIN, Ospeedr::VERY_HIGH_SPEED);
        });

        // TDO takes no pull, so it never loads the net the target drives.
        tdi.set_low();
        tdi.set_as_output(Speed::VeryHigh);
        tdo.set_as_input(Pull::None);

        Self {
            tdi,
            tdo,
            _board: core::marker::PhantomData,
        }
    }

    #[inline(always)]
    fn regs(&self) -> pac::spi::Spi {
        M::spi()
    }
}

/// Set one pin's mode-register field on the port carrying the shared pads.
#[inline(always)]
fn set_mode<M: SpiSwdBoard>(pin: usize, mode: Moder) {
    M::clk_port().moder().modify(|w| w.set_moder(pin, mode));
}

/// Bound on every SPI status poll: an undrivable rung must fail the transfer, not the USB task.
const SPI_POLL_LIMIT: u32 = 200_000;

impl<M: SpiSwdBoard> SpiPortOps for StmSpiPort<M> {
    fn configure(&mut self) {
        let s = self.regs();
        s.cr1().modify(|w| w.set_spe(false));
        // `SSI` high before master mode: with `SSM`, `SS` low there is a mode fault demoting to slave.
        s.cr1().modify(|w| w.set_ssi(true));
        s.cfg2().write(|w| {
            w.set_master(Master::MASTER);
            w.set_comm(Comm::HALF_DUPLEX);
            w.set_ioswp(true);
            w.set_lsbfirst(Lsbfirst::LSBFIRST);
            w.set_cpol(Cpol::IDLE_HIGH);
            w.set_cpha(Cpha::SECOND_EDGE);
            w.set_ssm(true);
            w.set_afcntr(true);
        });
        s.ifcr().write(|w| w.set_modfc(true));
    }

    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
        let s = self.regs();
        s.cfg1().modify(|w| {
            w.set_mbr(Mbr::from_bits(mbr));
            w.set_dsize(bits - 1);
        });
        s.cr2().write(|w| w.set_tsize(frames));
        s.cr1().modify(|w| {
            w.set_hddir(match dir {
                Dir::Tx => Hddir::TRANSMITTER,
                Dir::Rx => Hddir::RECEIVER,
            })
        });
    }

    fn enable(&self) {
        self.regs().cr1().modify(|w| w.set_spe(true));
    }

    fn disable(&self) {
        self.regs().cr1().modify(|w| w.set_spe(false));
    }

    fn start(&self) {
        self.regs().cr1().modify(|w| w.set_cstart(true));
    }

    fn push8(&self, byte: u8) {
        let mut spins = 0u32;
        while !self.regs().sr().read().txp() {
            spins += 1;
            if spins > SPI_POLL_LIMIT {
                break;
            }
        }
        self.regs().txdr8().write_value(byte);
    }

    fn wait_eot(&self) {
        let mut spins = 0u32;
        while !self.regs().sr().read().eot() {
            spins += 1;
            if spins > SPI_POLL_LIMIT {
                break;
            }
        }
    }

    fn pop32(&self) -> u32 {
        let mut spins = 0u32;
        while !self.regs().sr().read().rxp() {
            spins += 1;
            if spins > SPI_POLL_LIMIT {
                break;
            }
        }
        self.regs().rxdr32().read()
    }

    fn clear_flags(&self) {
        self.regs().ifcr().write(|w| {
            w.set_eotc(true);
            w.set_txtfc(true);
            w.set_ovrc(true);
        });
    }

    fn pads_to_spi(&mut self) {
        set_mode::<M>(M::CLK_PIN, Moder::ALTERNATE);
        set_mode::<M>(M::IO_PIN, Moder::ALTERNATE);
    }

    fn clk_pad_to_spi(&mut self) {
        set_mode::<M>(M::CLK_PIN, Moder::ALTERNATE);
    }
}

impl<M: SpiSwdBoard> JtagPortOps for StmSpiPort<M> {
    fn jtag_pads_acquire(&mut self) {
        self.tdi.set_low();
        self.tdi.set_as_output(Speed::VeryHigh);
        self.tdo.set_as_input(Pull::None);
    }

    fn jtag_pads_release(&mut self) {
        self.tdi.set_as_input(Pull::None);
    }

    #[inline(always)]
    fn tck_low(&self) {
        M::clk_port().bsrr().write(|w| w.set_br(M::CLK_PIN, true));
    }

    #[inline(always)]
    fn tck_high(&self) {
        M::clk_port().bsrr().write(|w| w.set_bs(M::CLK_PIN, true));
    }

    #[inline(always)]
    fn tms_set(&self, high: bool) {
        M::clk_port().bsrr().write(|w| {
            if high {
                w.set_bs(M::IO_PIN, true);
            } else {
                w.set_br(M::IO_PIN, true);
            }
        });
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
        M::jtag_port().idr().read().idr(M::TDO_PIN) as u32
    }

    fn tdi_is_high(&mut self) -> bool {
        self.tdi.is_high()
    }

    fn tdo_is_high(&mut self) -> bool {
        self.tdo.is_high()
    }
}

/// The SPI-session DAP engine over a board seam `M`: SPI SWD plus GPIO bit-bang JTAG.
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
    SpiSwdDeps<StmSpiPort<M>, IO, D, J>,
    LEDS,
    D,
    SpiJtag<StmSpiPort<M>, IO, D, J>,
    SpiSwd<StmSpiPort<M>, IO, D, J>,
    SWO,
    V,
>;
