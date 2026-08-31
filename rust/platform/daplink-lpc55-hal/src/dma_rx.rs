//! Flexcomm0 receive over DMA0 channel 4, as a [`DmaRxChannel`] the shared ring reads from: one
//! descriptor linked to itself, so the transfer counter alone gives the write position.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use daplink_rs::DmaRxChannel;
use lpc55_hal::raw;

use crate::dma;

/// Flexcomm0 receive. The request map fixes it here (Flexcomm3 is 8 and 9, Flexcomm2 10 and 11),
/// so the channel does not follow from the Flexcomm index.
const CHANNEL: usize = 4;

/// Below the SWO capture channel, which is the faster line.
const PRIORITY: u8 = 1;

/// Whether the channel is armed. A flag, not a register read: DMA0 is unclocked until it is.
static ARMED: AtomicBool = AtomicBool::new(false);

/// Channel errors the controller latched, and the re-arms recovering from them; the error count
/// also rides the `0x9D` ledger, the SWD names remain for a bench with only a debug probe.
#[no_mangle]
pub static VCOM_RX_DMA_ERR: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static VCOM_RX_REARMS: AtomicU32 = AtomicU32::new(0);

pub(crate) fn armed() -> bool {
    ARMED.load(Ordering::Relaxed)
}

/// Arm the channel over the receive buffer and hand the UART's FIFO to it. The caller owns
/// Flexcomm0 and must not also drain that FIFO by hand afterwards.
pub(crate) fn arm() {
    let usart = unsafe { &*raw::USART0::ptr() };
    // Taken from the peripheral block: a hand-written offset would read as "never transfers".
    let fiford = core::ptr::addr_of!(usart.fiford) as u32;
    dma::arm_rx(
        CHANNEL,
        unsafe { core::ptr::addr_of_mut!((*dma::mem()).vcom) },
        fiford,
        PRIORITY,
    );

    // FIFOCFG is shared with the preempting FLEXCOMM0 handler, so its read-modify-write is masked.
    usart.fifointenclr.write(|w| w.rxlvl().set_bit());
    cortex_m::interrupt::free(|_| {
        usart
            .fifocfg
            .modify(|_, w| w.emptyrx().set_bit().dmarx().set_bit());
    });
    ARMED.store(true, Ordering::Relaxed);
    VCOM_RX_REARMS.fetch_add(1, Ordering::Relaxed);
}

/// Stop the channel and take the FIFO back, so `!armed()` means this seam owns the FIFO.
fn quiesce() {
    let usart = unsafe { &*raw::USART0::ptr() };
    cortex_m::interrupt::free(|_| {
        usart.fifocfg.modify(|_, w| w.dmarx().clear_bit());
    });
    dma::stop(CHANNEL);
    ARMED.store(false, Ordering::Relaxed);
}

/// The receive half of the UART, as the ring's channel.
pub struct Lpc55DmaRx;

impl Lpc55DmaRx {
    /// The channel handle and the memory it fills. Nothing is received until [`arm`] runs.
    pub(crate) fn new() -> (Self, &'static [u8]) {
        (Lpc55DmaRx, unsafe {
            &*core::ptr::addr_of!((*dma::mem()).vcom.buf)
        })
    }
}

impl DmaRxChannel for Lpc55DmaRx {
    fn remaining(&self) -> usize {
        dma::remaining(CHANNEL)
    }

    fn sample(&mut self) -> (usize, bool) {
        let (remaining, lapped, errored) = dma::sample_rx(CHANNEL);
        if errored {
            VCOM_RX_DMA_ERR.fetch_add(1, Ordering::Relaxed);
            quiesce();
        }
        (remaining, lapped)
    }

    fn reload(&mut self, _len: usize) {
        quiesce();
        arm();
    }

    fn running(&self) -> bool {
        armed()
    }
}
