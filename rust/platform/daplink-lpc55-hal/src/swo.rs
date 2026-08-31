//! Flexcomm3 USART-RX SWO capture on PIO0_3, runtime-muxed with the bit-bang JTAG TDO line.

use daplink_rs::{RingSwo, SwoBackend};
use lpc55_hal as hal;
use lpc55_hal::drivers::pins::Pio0_3;
use lpc55_hal::drivers::Serial;
use lpc55_hal::prelude::*;
use lpc55_hal::raw;
use lpc55_hal::typestates::pin::flexcomm::NoTx;
use lpc55_hal::typestates::pin::state;
use lpc55_hal::Pin;

/// Placeholder baud at construction; the host sets the real rate before capture starts.
const INIT_BAUD: u32 = 115_200;

/// FC3 function clock: two thirds of the Flexcomm source, realizing the 500k..4M ladder exactly.
const SWO_CLOCK_HZ: u32 = 32_000_000;

/// `FLEXFRG3CTRL.MULT` for [`SWO_CLOCK_HZ`]: FRG output is `src / (1 + MULT / 256)`; 128 -> 3/2.
const FRG3_MULT: u8 = 128;

/// The MCU-Link SWO slot: the Flexcomm3 USART3-RX mechanism over the DMA ring it fills.
pub type Fc3Swo = RingSwo<Fc3SwoHw, crate::dma_swo::DmaSwoSource>;

/// Flexcomm3 USART SWO mechanism plus the PIO0_3 TDO<->SWO pad mux; the registers hold the state.
pub struct Fc3SwoHw;

#[inline]
fn regs() -> &'static raw::usart0::RegisterBlock {
    // SAFETY: USART3 is owned by this seam; config runs only while capture is stopped.
    unsafe { &*raw::USART3::ptr() }
}

/// Route PIO0_3 to Flexcomm3 (`FC3_RXD`, SWO capture) or back to GPIO (JTAG TDO): only `FUNC`
/// differs, so the pad is an input either way. No JTAG path reclaims it while capture runs.
#[inline]
fn route_pio0_3(to_fc3: bool) {
    // A write to a clock-gated IOCON is silently ignored, and bring-up gates it off; idempotent.
    // SAFETY: SYSCON is a fixed peripheral; `ahbclkctrlset` touches only the set bit.
    unsafe { &*raw::SYSCON::ptr() }.ahbclkctrlset[0].write(|w| unsafe { w.bits(1 << 13) });
    // SAFETY: PIO0_3 is owned by the SWO/TDO pair and accessed non-concurrently; this
    // reprograms only that pad's IOCON word.
    let iocon = unsafe { &*raw::IOCON::ptr() };
    iocon.pio0_3.modify(|_, w| {
        let w = if to_fc3 {
            w.func().alt1()
        } else {
            w.func().alt0()
        };
        w.mode()
            .inactive()
            .slew()
            .standard()
            .invert()
            .disabled()
            .digimode()
            .digital()
            .od()
            .normal()
    });
}

impl SwoBackend for Fc3SwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        // Shared divisor search off FC3's own kernel; an unrealizable rate reports 0.
        match crate::baud::flexcomm_osr_brg(SWO_CLOCK_HZ, baud, crate::baud::SWO_OSR_MIN) {
            Some((osr, brg)) => {
                let regs = regs();
                regs.brg
                    .write(|w| unsafe { w.brgval().bits((brg - 1) as u16) });
                regs.osr
                    .write(|w| unsafe { w.osrval().bits((osr - 1) as u8) });
                crate::baud::realized(SWO_CLOCK_HZ, osr, brg)
            }
            None => 0,
        }
    }

    fn prepare(&mut self) {
        // Route the pad first, then discard, so no mux glitch appears as the first SWO data.
        route_pio0_3(true);
        // The teardown is the discard, and bounded -- unlike a read-until-empty on a live line.
        crate::dma_swo::stop();
    }

    fn arm(&mut self) {
        crate::dma_swo::arm();
    }

    fn stop(&mut self) {
        crate::dma_swo::stop();
        route_pio0_3(false);
    }
}

/// Bring Flexcomm3 up as an RX-only USART SWO capture; call once per image. PIO0_3 is left in GPIO
/// (TDO) mode, and capture muxes it to FC3_RXD only while running.
pub(crate) fn fc3_swo(
    flexcomm3: hal::peripherals::flexcomm::Flexcomm3,
    rx_pin: Pin<Pio0_3, state::Unused>,
    iocon: &mut hal::Iocon<hal::typestates::init_state::Enabled>,
    syscon: &mut hal::Syscon,
    clocks_token: &hal::typestates::ClocksSupportFlexcommToken,
) -> Fc3Swo {
    let usart = flexcomm3.enabled_as_usart(syscon, clocks_token);

    // Rehome FC3's function clock off the HAL's 12 MHz FRO onto the Flexcomm source via FRG3.
    // SAFETY: single-threaded bring-up; the writes touch only FCCLKSEL3 and FLEXFRG3CTRL.
    let syscon_raw = unsafe { &*raw::SYSCON::ptr() };
    syscon_raw.fcclksel3().modify(|_, w| w.sel().enum_0x3());
    syscon_raw
        .flexfrg3ctrl()
        .write(|w| unsafe { w.div().bits(crate::clocks::FRG_DIV).mult().bits(FRG3_MULT) });

    let rx = rx_pin.into_usart3_rx_pin(iocon);
    let config = hal::drivers::serial::config::Config::default().speed(INIT_BAUD.Hz());
    // No `Drop` glue: falling out of scope leaves every configured register as set.
    let _serial = Serial::new(usart, (NoTx, rx), config);

    // SAFETY: `_serial` owns USART3 here; exclusive access at construction.
    let regs = unsafe { &*raw::USART3::ptr() };
    // Trigger at >= 1 entry: at the HAL's >= 2 default a lone final byte sits stranded.
    regs.fifotrig.modify(|_, w| unsafe { w.rxlvl().bits(0) });

    // Undo the pad commit so the bit-bang JTAG TDO line reads PIO0_3 out of reset.
    route_pio0_3(false);

    RingSwo::new(Fc3SwoHw, crate::dma_swo::DmaSwoSource::new())
}
