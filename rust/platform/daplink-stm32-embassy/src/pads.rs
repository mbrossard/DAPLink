//! The TAP-line half of a probe seam, with no SPI block behind it.

use core::marker::PhantomData;

use daplink_spi_swd::JtagPortOps;
use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::{Moder, Ospeedr, Pupdr};

/// The per-board pin map, a zero-sized type so every member is a constant.
pub trait JtagPadMap {
    /// Pin index of TCK within [`tck_port`](Self::tck_port).
    const TCK_PIN: usize;
    /// Pin index of TMS within [`tck_port`](Self::tck_port).
    const TMS_PIN: usize;
    /// Pin index of TDI within [`tdi_port`](Self::tdi_port).
    const TDI_PIN: usize;
    /// Pin index of TDO within [`tdi_port`](Self::tdi_port).
    const TDO_PIN: usize;

    /// `OSPEEDR` class for TCK and TMS.
    const TAP_SPEED: Ospeedr = Ospeedr::VERY_HIGH_SPEED;

    /// GPIO block carrying TCK/TMS, the pads SWD shares.
    fn tck_port() -> pac::gpio::Gpio;
    fn tdi_port() -> pac::gpio::Gpio;
}

/// The four TAP lines over a pin map.
pub struct StmJtagPads<M: JtagPadMap>(PhantomData<M>);

impl<M: JtagPadMap> StmJtagPads<M> {
    /// Raw mapped pads; the composition must reserve them.
    pub const fn new() -> Self {
        Self(PhantomData)
    }
}

impl<M: JtagPadMap> Default for StmJtagPads<M> {
    fn default() -> Self {
        Self::new()
    }
}

#[inline(always)]
fn write_pin(port: pac::gpio::Gpio, pin: usize, high: bool) {
    port.bsrr().write(|w| {
        if high {
            w.set_bs(pin, true);
        } else {
            w.set_br(pin, true);
        }
    });
}

impl<M: JtagPadMap> JtagPortOps for StmJtagPads<M> {
    fn jtag_pads_acquire(&mut self) {
        // Levels before directions, so enabling a drive cannot clock the TAP.
        // A TCK already driven keeps its level: raising one parked low would clock the TAP.
        if M::tck_port().moder().read().moder(M::TCK_PIN) != Moder::OUTPUT {
            write_pin(M::tck_port(), M::TCK_PIN, true);
        }
        write_pin(M::tck_port(), M::TMS_PIN, true);
        write_pin(M::tdi_port(), M::TDI_PIN, false);

        M::tck_port().ospeedr().modify(|w| {
            w.set_ospeedr(M::TCK_PIN, M::TAP_SPEED);
            w.set_ospeedr(M::TMS_PIN, M::TAP_SPEED);
        });
        M::tdi_port()
            .ospeedr()
            .modify(|w| w.set_ospeedr(M::TDI_PIN, Ospeedr::VERY_HIGH_SPEED));
        // TDO takes no pull, so it never loads the net the target drives.
        M::tdi_port()
            .pupdr()
            .modify(|w| w.set_pupdr(M::TDO_PIN, Pupdr::FLOATING));

        M::tck_port().moder().modify(|w| {
            w.set_moder(M::TCK_PIN, Moder::OUTPUT);
            w.set_moder(M::TMS_PIN, Moder::OUTPUT);
        });
        M::tdi_port().moder().modify(|w| {
            w.set_moder(M::TDI_PIN, Moder::OUTPUT);
            w.set_moder(M::TDO_PIN, Moder::INPUT);
        });
    }

    fn jtag_pads_release(&mut self) {
        // TCK/TMS stay outputs: the SWD side reasserts its own directions on mode entry.
        M::tdi_port()
            .moder()
            .modify(|w| w.set_moder(M::TDI_PIN, Moder::INPUT));
    }

    #[inline(always)]
    fn tck_low(&self) {
        M::tck_port().bsrr().write(|w| w.set_br(M::TCK_PIN, true));
    }

    #[inline(always)]
    fn tck_high(&self) {
        M::tck_port().bsrr().write(|w| w.set_bs(M::TCK_PIN, true));
    }

    #[inline(always)]
    fn tms_set(&self, high: bool) {
        write_pin(M::tck_port(), M::TMS_PIN, high);
    }

    #[inline(always)]
    fn tdi_set(&self, high: bool) {
        write_pin(M::tdi_port(), M::TDI_PIN, high);
    }

    #[inline(always)]
    fn tdo_get(&self) -> u32 {
        M::tdi_port().idr().read().idr(M::TDO_PIN) as u32
    }

    fn tdi_is_high(&mut self) -> bool {
        M::tdi_port().odr().read().odr(M::TDI_PIN) as u32 != 0
    }

    fn tdo_is_high(&mut self) -> bool {
        self.tdo_get() != 0
    }
}
