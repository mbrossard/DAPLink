//! Shared STM32 register seam for the SPI-session SWD engine.

use core::cell::Cell;
use daplink_spi_swd::{
    Dir, JtagPortOps, PortResult, SpiFault, SpiJtag, SpiPortOps, SpiSwd, SpiSwdDeps,
};
use embassy_stm32::gpio::{Flex, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::{Moder, Ospeedr};
use embassy_stm32::pac::spi::regs::{Cfg1, Cfg2, Cr1};
use embassy_stm32::pac::spi::vals::{Comm, Cpha, Cpol, Hddir, Lsbfirst, Master, Mbr};

/// The per-board half of the seam, a zero-sized type so every member is a constant.
pub trait SpiSwdBoard {
    /// Pin index of SWCLK / TCK within [`clk_port`](Self::clk_port); also the SPI SCK pad.
    const CLK_PIN: usize;
    /// Pin index of SWDIO / TMS within [`clk_port`](Self::clk_port); the half-duplex data pad.
    const IO_PIN: usize;
    /// Pin index of TDI within [`jtag_port`](Self::jtag_port).
    const TDI_PIN: usize;
    /// Pin index of TDO within [`jtag_port`](Self::jtag_port).
    const TDO_PIN: usize;
    /// Alternate function mapping the two shared pads to the SPI block.
    const SPI_AF: u8;
    /// `OSPEEDR` class for the two shared pads.
    const PAD_SPEED: Ospeedr = Ospeedr::VERY_HIGH_SPEED;
    /// The kernel clock source the board's [`SpiLadder`](daplink_spi_swd::SpiLadder) names.
    type Source: Copy;

    /// GPIO block carrying SWCLK/SWDIO.
    fn clk_port() -> pac::gpio::Gpio;
    /// GPIO block carrying TDI/TDO.
    fn jtag_port() -> pac::gpio::Gpio;
    /// The SPI block the shared pads route to.
    fn spi() -> pac::spi::Spi;

    /// Clock the SPI block, its kernel on the ladder's first source; [`StmSpiPort::new`] calls it.
    fn init_clocks();

    /// Route the kernel mux to `source`; `init_clocks` starts on the ladder's first.
    fn route(_source: Self::Source) {}
}

/// The half-duplex SPI block plus the GPIO bit operations on the probe pads.
pub struct StmSpiPort<M: SpiSwdBoard> {
    tdi: Flex<'static>,
    tdo: Flex<'static>,
    // An unused pad on the probe nets, held as an unpulled input.
    _spare: Flex<'static>,
    // Register images the engine fully owns, so no session reads a register back.
    cr1: Cell<u32>,
    pub(crate) cfg2: Cell<u32>,
    _board: core::marker::PhantomData<M>,
}

impl<M: SpiSwdBoard> StmSpiPort<M> {
    /// Configure mapped SPI; `tdi`/`tdo` must match the map and the shared pins be in 8..=15.
    pub fn new(mut tdi: Flex<'static>, mut tdo: Flex<'static>, mut spare: Flex<'static>) -> Self {
        const {
            assert!(
                M::CLK_PIN >= 8 && M::CLK_PIN <= 15 && M::IO_PIN >= 8 && M::IO_PIN <= 15,
                "the shared SPI pads must be in AFRH, pins 8..=15"
            )
        };
        M::init_clocks();

        let g = M::clk_port();
        g.afr(1).modify(|w| {
            // AFR high covers pins 8..=15.
            w.set_afr(M::CLK_PIN - 8, M::SPI_AF);
            w.set_afr(M::IO_PIN - 8, M::SPI_AF);
        });
        g.ospeedr().modify(|w| {
            w.set_ospeedr(M::CLK_PIN, M::PAD_SPEED);
            w.set_ospeedr(M::IO_PIN, M::PAD_SPEED);
        });
        spare.set_as_input(Pull::None);

        // TDO takes no pull, so it never loads the net the target drives.
        tdi.set_low();
        tdi.set_as_output(Speed::VeryHigh);
        tdo.set_as_input(Pull::None);

        Self {
            tdi,
            tdo,
            _spare: spare,
            cr1: Cell::new(0),
            cfg2: Cell::new(0),
            _board: core::marker::PhantomData,
        }
    }

    /// Write the CR1 image after `f` edits it; `CSTART` self-clears, so it stays out of the image.
    #[inline(always)]
    fn write_cr1(&self, f: impl FnOnce(&mut Cr1)) {
        let mut v = Cr1(self.cr1.get());
        f(&mut v);
        self.cr1.set(v.0);
        self.regs().cr1().write_value(v);
    }

    /// Program one session's frame geometry and baud rung; only while the block is disabled.
    #[inline(always)]
    pub(crate) fn program_geometry(&self, mbr: u8, bits: u8, frames: u16) {
        let s = self.regs();
        // Every CFG1 field is the engine's.
        let mut cfg1 = Cfg1(0);
        cfg1.set_mbr(Mbr::from_bits(mbr));
        cfg1.set_dsize(bits - 1);
        s.cfg1().write_value(cfg1);
        s.cr2().write(|w| w.set_tsize(frames));
    }

    /// Spin on a status predicate, returning the status word that satisfied it.
    #[inline(always)]
    fn poll(&self, ready: impl Fn(pac::spi::regs::Sr) -> bool) -> PortResult<pac::spi::regs::Sr> {
        let sr = self.regs().sr().read();
        if ready(sr) {
            return Ok(sr);
        }
        self.poll_bounded(ready)
    }

    #[inline(never)]
    fn poll_bounded(
        &self,
        ready: impl Fn(pac::spi::regs::Sr) -> bool,
    ) -> PortResult<pac::spi::regs::Sr> {
        for _ in 0..SPI_POLL_LIMIT {
            let sr = self.regs().sr().read();
            if ready(sr) {
                return Ok(sr);
            }
        }
        Err(SpiFault::Timeout)
    }

    #[inline(always)]
    pub(crate) fn regs(&self) -> pac::spi::Spi {
        M::spi()
    }
}

#[inline(always)]
pub(crate) fn set_mode<M: SpiSwdBoard>(pin: usize, mode: Moder) {
    M::clk_port().moder().modify(|w| w.set_moder(pin, mode));
}

/// Bound on every SPI status poll: an undrivable rung must fail the transfer, not the USB task.
pub(crate) const SPI_POLL_LIMIT: u32 = 200_000;

impl<M: SpiSwdBoard> SpiPortOps for StmSpiPort<M> {
    const DUPLEX: bool = false;

    type Source = M::Source;

    #[inline(always)]
    fn route(&mut self, source: M::Source) {
        M::route(source)
    }

    fn configure(&mut self) {
        let s = self.regs();
        // `SSI` high before master mode, or `SSM` raises a mode fault.
        let mut cr1 = Cr1(0);
        cr1.set_ssi(true);
        self.cr1.set(cr1.0);
        s.cr1().write_value(cr1);
        let mut cfg2 = Cfg2(0);
        cfg2.set_master(Master::MASTER);
        cfg2.set_comm(Comm::HALF_DUPLEX);
        cfg2.set_lsbfirst(Lsbfirst::LSBFIRST);
        cfg2.set_cpol(Cpol::IDLE_HIGH);
        cfg2.set_cpha(Cpha::SECOND_EDGE);
        cfg2.set_ssm(true);
        cfg2.set_afcntr(true);
        self.cfg2.set(cfg2.0);
        s.cfg2().write_value(cfg2);
        s.ifcr().write(|w| w.set_modfc(true));
    }

    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
        self.program_geometry(mbr, bits, frames);
        self.write_cr1(|w| {
            w.set_hddir(match dir {
                Dir::Tx => Hddir::TRANSMITTER,
                Dir::Rx => Hddir::RECEIVER,
            })
        });
    }

    fn enable(&self) {
        self.write_cr1(|w| w.set_spe(true));
    }

    fn disable(&self) {
        self.write_cr1(|w| w.set_spe(false));
    }

    fn start(&self) {
        let mut v = Cr1(self.cr1.get());
        v.set_cstart(true);
        self.regs().cr1().write_value(v);
    }

    fn push8(&self, byte: u8) -> PortResult<()> {
        self.poll(|sr| sr.txp())?;
        self.regs().txdr8().write_value(byte);
        Ok(())
    }

    fn push16(&self, half: u16) -> PortResult<()> {
        self.poll(|sr| sr.txp())?;
        self.regs().txdr16().write_value(half);
        Ok(())
    }

    fn wait_eot(&self) -> PortResult<()> {
        // The status word that showed EOT also settles the overrun.
        if self.poll(|sr| sr.eot())?.ovr() {
            return Err(SpiFault::Overrun);
        }
        Ok(())
    }

    fn pop32(&self) -> PortResult<u32> {
        self.poll(|sr| sr.rxp())?;
        Ok(self.regs().rxdr32().read())
    }

    fn clear_flags(&self) -> PortResult<()> {
        // The overrun was read by `wait_eot`, so no status read here.
        self.regs().ifcr().write(|w| {
            w.set_eotc(true);
            w.set_txtfc(true);
            w.set_ovrc(true);
        });
        Ok(())
    }

    fn pads_to_spi(&mut self) {
        set_mode::<M>(M::CLK_PIN, Moder::ALTERNATE);
        set_mode::<M>(M::IO_PIN, Moder::ALTERNATE);
    }

    fn clk_pad_to_spi(&mut self) {
        set_mode::<M>(M::CLK_PIN, Moder::ALTERNATE);
    }

    fn drive(&mut self) {}

    fn release(&mut self) {}
}

impl<M: SpiSwdBoard> JtagPortOps for StmSpiPort<M> {
    fn jtag_pads_acquire(&mut self) {
        self.tdi.set_low();
        self.tdi.set_as_output(Speed::VeryHigh);
        self.tdo.set_as_input(Pull::None);
    }

    fn jtag_pads_release(&mut self) {
        // The low drive `new` set, so SWD sees TDI as before any JTAG session.
        self.tdi.set_low();
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

/// The SPI-session DAP engine over a port `M`: SPI SWD plus GPIO bit-bang JTAG.
pub type SpiSwdJtagDap<
    'a,
    M,
    IO,
    D,
    J,
    SWO = daplink_rs::NoSwo,
    V = daplink_rs::NoVendor,
    LEDS = daplink_rs::NoopLeds,
    const K: usize = 1,
> = daplink_rs::DapProbe<
    'a,
    SpiSwdDeps<M, IO, D, J, K>,
    LEDS,
    D,
    SpiJtag<M, IO, D, J, K>,
    SpiSwd<M, IO, D, J, K>,
    SWO,
    V,
>;
