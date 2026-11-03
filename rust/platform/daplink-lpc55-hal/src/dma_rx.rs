//! Flexcomm0 receive over DMA0 channel 4 through one self-linked descriptor.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::dma;
use daplink_rs::DmaRxChannel;

fn usart() -> &'static lpc55_hal::raw::usart0::RegisterBlock {
    #[cfg(test)]
    {
        dma::stop_tests::usart()
    }
    #[cfg(not(test))]
    unsafe {
        &*lpc55_hal::raw::USART0::ptr()
    }
}

/// Flexcomm0 receive; the request map does not follow the Flexcomm index.
const CHANNEL: usize = 4;

/// Below the SWO capture channel, which is the faster line.
const PRIORITY: u8 = 1;

/// Whether the channel is armed. A flag, not a register read: DMA0 is unclocked until it is.
static ARMED: AtomicBool = AtomicBool::new(false);

/// Latched channel errors and the re-arms recovering from them, readable over SWD.
#[no_mangle]
pub static VCOM_RX_DMA_ERR: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static VCOM_RX_REARMS: AtomicU32 = AtomicU32::new(0);

pub(crate) fn armed() -> bool {
    ARMED.load(Ordering::Relaxed)
}

/// Arm the channel and hand it the UART FIFO; the caller must not drain that FIFO by hand after.
pub(crate) fn arm() {
    let usart = usart();
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
    dma::masked(|| {
        usart
            .fifocfg
            .modify(|_, w| w.emptyrx().set_bit().dmarx().set_bit());
    });
    ARMED.store(true, Ordering::Relaxed);
    VCOM_RX_REARMS.fetch_add(1, Ordering::Relaxed);
}

/// Stop the channel and take the FIFO back, so `!armed()` means this seam owns the FIFO.
fn quiesce() {
    let usart = usart();
    dma::masked(|| {
        usart.fifocfg.modify(|_, w| w.dmarx().clear_bit());
    });
    dma::stop(CHANNEL);
    ARMED.store(false, Ordering::Relaxed);
}

pub struct Lpc55DmaRx;

impl Lpc55DmaRx {
    /// The channel handle and the memory it fills; nothing is received until [`arm`] runs.
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

#[cfg(test)]
pub(crate) fn stop_test() {
    use dma::stop_tests::{assert_unreleased, failed, script};
    arm();
    let mut rx = Lpc55DmaRx;
    let rearms = VCOM_RX_REARMS.load(Ordering::Relaxed);
    script(CHANNEL, None);
    failed(|| rx.reload(dma::RX_BUF_LEN));
    assert_unreleased(CHANNEL);
    assert!(armed());
    assert_eq!(VCOM_RX_REARMS.load(Ordering::Relaxed), rearms);
    script(CHANNEL, None);
    dma::error_sample();
    failed(|| {
        rx.sample();
    });
    assert_unreleased(CHANNEL);
    assert!(armed());
    assert_eq!(VCOM_RX_DMA_ERR.load(Ordering::Relaxed), 1);
    script(CHANNEL, Some(1));
    rx.reload(dma::RX_BUF_LEN);
    assert!(armed());
    assert_eq!(VCOM_RX_REARMS.load(Ordering::Relaxed), rearms + 1);
}
