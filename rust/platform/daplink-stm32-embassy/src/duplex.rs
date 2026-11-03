//! The full-duplex pin map: data sensed on MISO, driven from MOSI in transmit sessions.

use daplink_spi_swd::{Dir, JtagPortOps, PortResult, SpiPortOps};
use embassy_stm32::gpio::Flex;
use embassy_stm32::pac::gpio::vals::Moder;
use embassy_stm32::pac::spi::regs::Cfg2;
use embassy_stm32::pac::spi::vals::Comm;

use crate::spi_swd::{set_mode, SpiSwdBoard, StmSpiPort};

/// A board whose data net also reaches the SPI transmit pad; [`SpiSwdBoard::IO_PIN`] is MISO.
pub trait DuplexBoard: SpiSwdBoard {
    /// Pin index of MOSI within [`clk_port`](SpiSwdBoard::clk_port), 8..=15.
    const DRIVE_PIN: usize;
}

/// [`StmSpiPort`] with a separate transmit pad, released between sessions.
pub struct StmDuplexPort<M: DuplexBoard> {
    inner: StmSpiPort<M>,
}

impl<M: DuplexBoard> StmDuplexPort<M> {
    /// [`StmSpiPort::new`] with `drive` as the transmit pad, high-impedance until claimed.
    pub fn new(tdi: Flex<'static>, tdo: Flex<'static>, drive: Flex<'static>) -> Self {
        const {
            assert!(
                M::DRIVE_PIN >= 8 && M::DRIVE_PIN <= 15,
                "the SPI transmit pad must be in AFRH, pins 8..=15"
            )
        };
        let inner = StmSpiPort::new(tdi, tdo, drive);
        let g = M::clk_port();
        g.afr(1).modify(|w| w.set_afr(M::DRIVE_PIN - 8, M::SPI_AF));
        g.ospeedr()
            .modify(|w| w.set_ospeedr(M::DRIVE_PIN, M::PAD_SPEED));
        Self { inner }
    }
}

impl<M: DuplexBoard> SpiPortOps for StmDuplexPort<M> {
    const DUPLEX: bool = true;
    const EXACT_WRITE_FRAMING: bool = <StmSpiPort<M> as SpiPortOps>::EXACT_WRITE_FRAMING;
    const EOT_DELAY: bool = <StmSpiPort<M> as SpiPortOps>::EOT_DELAY;

    type Source = M::Source;

    #[inline(always)]
    fn route(&mut self, source: M::Source) {
        self.inner.route(source)
    }

    fn configure(&mut self) {
        self.inner.configure();
        // Every session rewrites `COMM` while the block is off.
        let mut cfg2 = Cfg2(self.inner.cfg2.get());
        cfg2.set_comm(Comm::FULL_DUPLEX);
        self.inner.cfg2.set(cfg2.0);
        self.inner.regs().cfg2().write_value(cfg2);
    }

    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
        self.inner.program_geometry(mbr, bits, frames);
        // Simplex per session: receive needs no transmit frames, transmit cannot overrun.
        let mut cfg2 = Cfg2(self.inner.cfg2.get());
        cfg2.set_comm(match dir {
            Dir::Tx => Comm::TRANSMITTER,
            Dir::Rx => Comm::RECEIVER,
        });
        if self.inner.cfg2.get() != cfg2.0 {
            self.inner.regs().cfg2().write_value(cfg2);
            self.inner.cfg2.set(cfg2.0);
        }
    }

    #[inline(always)]
    fn enable(&self) {
        self.inner.enable()
    }

    #[inline(always)]
    fn disable(&self) {
        self.inner.disable()
    }

    #[inline(always)]
    fn start(&self) {
        self.inner.start()
    }

    #[inline(always)]
    fn push8(&self, byte: u8) -> PortResult<()> {
        self.inner.push8(byte)
    }

    #[inline(always)]
    fn push16(&self, half: u16) -> PortResult<()> {
        self.inner.push16(half)
    }

    #[inline(always)]
    fn wait_eot(&self) -> PortResult<()> {
        self.inner.wait_eot()
    }

    #[inline(always)]
    fn pop32(&self) -> PortResult<u32> {
        self.inner.pop32()
    }

    #[inline(always)]
    fn clear_flags(&self) -> PortResult<()> {
        self.inner.clear_flags()
    }

    #[inline(always)]
    fn pads_to_spi(&mut self) {
        self.inner.pads_to_spi()
    }

    #[inline(always)]
    fn clk_pad_to_spi(&mut self) {
        self.inner.clk_pad_to_spi()
    }

    fn drive(&mut self) {
        set_mode::<M>(M::DRIVE_PIN, Moder::ALTERNATE);
    }

    fn release(&mut self) {
        set_mode::<M>(M::DRIVE_PIN, Moder::INPUT);
    }
}

impl<M: DuplexBoard> JtagPortOps for StmDuplexPort<M> {
    #[inline(always)]
    fn jtag_pads_acquire(&mut self) {
        self.inner.jtag_pads_acquire()
    }

    #[inline(always)]
    fn jtag_pads_release(&mut self) {
        self.inner.jtag_pads_release()
    }

    #[inline(always)]
    fn tck_low(&self) {
        self.inner.tck_low()
    }

    #[inline(always)]
    fn tck_high(&self) {
        self.inner.tck_high()
    }

    #[inline(always)]
    fn tms_set(&self, high: bool) {
        self.inner.tms_set(high)
    }

    #[inline(always)]
    fn tdi_set(&self, high: bool) {
        self.inner.tdi_set(high)
    }

    #[inline(always)]
    fn tdi_put(&self, bits: u32) {
        self.inner.tdi_put(bits)
    }

    #[inline(always)]
    fn tdo_get(&self) -> u32 {
        self.inner.tdo_get()
    }

    #[inline(always)]
    fn tdi_is_high(&mut self) -> bool {
        self.inner.tdi_is_high()
    }

    #[inline(always)]
    fn tdo_is_high(&mut self) -> bool {
        self.inner.tdo_is_high()
    }
}
