//! The VCOM bridge, run in the USB1 task; FLEXCOMM0 and DMA0 wake it through [`VCOM_KICK`].

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use daplink_lpc55_hal::{
    observe_rx_errors, vcom_rx_discards, vcom_rx_dma_errors, vcom_tx_sent, Vcom,
};
use daplink_rs::VcomCounters;

use crate::app::{Usb, CDC_EP};
#[cfg(feature = "probe-timing")]
use crate::probe_timing as timing;

/// How long a partial CDC packet waits for company.
const CDC_COALESCE_MS: u32 = 1;

/// When a packet was last offered to CDC IN; the coalescing deadline runs from here.
static LAST_FLUSH_MS: AtomicU32 = AtomicU32::new(0);

/// Set by the UART and DMA handlers: the next pass owes the VCOM bridge a service.
pub(crate) static VCOM_KICK: AtomicBool = AtomicBool::new(false);

static VCOM_UART_RX: AtomicU32 = AtomicU32::new(0);
/// Bytes of those staged toward CDC IN, before discards.
static VCOM_CDC_IN: AtomicU32 = AtomicU32::new(0);
static VCOM_CDC_OUT: AtomicU32 = AtomicU32::new(0);
static VCOM_HOST_DROPPED: AtomicU32 = AtomicU32::new(0);

/// Service the bridge on port activity, a UART or DMA wake, and each millisecond; true if it ran.
pub(crate) fn service(usb: &mut Usb, uart: &mut Vcom, vcom_ms: &mut u32) -> bool {
    let now_ms = crate::ELAPSED_MS.load(Ordering::Relaxed);
    let vcom_due =
        usb.vcom_activity() || VCOM_KICK.swap(false, Ordering::Relaxed) || now_ms != *vcom_ms;
    if vcom_due {
        *vcom_ms = now_ms;
        // Return UART bytes first: a receive backlog makes a divisor change unsafe.
        #[cfg(feature = "probe-timing")]
        let vcom_started = timing::now();
        vcom_return(usb, uart);
        #[cfg(feature = "probe-timing")]
        timing::max(
            &timing::PROBE_VCOM_MAX_US,
            timing::cycles_since(vcom_started),
        );

        // Follow the host's line coding between exchanges, before host traffic is consumed.
        let requested = usb.vcom_line_coding_baud();
        if requested != 0 {
            uart.request_baud(requested);
        }

        // Backpressure CDC OUT until the new divisor applies.
        if requested == 0 || uart.baud_applied(requested) {
            vcom_host_traffic(usb, uart);
        }
    }
    vcom_due
}

/// The VCOM bridge's conservation ledger, for the `0x9D` vendor query.
pub(crate) fn vcom_counters() -> VcomCounters {
    use daplink_rs::{
        vcom_episodes, vcom_grade, VCOM_GRADE_EXACT, VCOM_GRADE_LOWER_BOUND, VCOM_SLOT_CDC_IN,
        VCOM_SLOT_CDC_OUT, VCOM_SLOT_FRAME_ERR, VCOM_SLOT_HOST_DROPPED, VCOM_SLOT_OVERRUN,
        VCOM_SLOT_RING_FULL, VCOM_SLOT_RX_DMA_ERR, VCOM_SLOT_UART_RX, VCOM_SLOT_UART_TX,
    };
    let rx = vcom_rx_discards();
    // Deduct discards from `cdc_in` in one step, so the query never reads a torn pair.
    let host_dropped = VCOM_HOST_DROPPED.load(Ordering::Relaxed);
    VcomCounters {
        uart_rx: VCOM_UART_RX.load(Ordering::Relaxed),
        cdc_in: VCOM_CDC_IN
            .load(Ordering::Relaxed)
            .wrapping_sub(host_dropped),
        host_dropped,
        ring_full: rx.ring_full,
        frame_err: rx.frame_err,
        overrun: rx.overrun,
        cdc_out: VCOM_CDC_OUT.load(Ordering::Relaxed),
        uart_tx: vcom_tx_sent(),
        rx_dma_err: vcom_rx_dma_errors(),
        valid: daplink_rs::VCOM_VALID_IDENTITY
            | daplink_rs::VCOM_VALID_CDC_OUT
            | daplink_rs::VCOM_VALID_RING_FULL
            | daplink_rs::VCOM_VALID_FRAME_ERR
            | daplink_rs::VCOM_VALID_OVERRUN
            | daplink_rs::VCOM_VALID_UART_TX
            | daplink_rs::VCOM_VALID_RX_DMA_ERR,
        // Lower bounds where the hardware under-reports.
        grade: vcom_grade(VCOM_SLOT_UART_RX, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_CDC_IN, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_HOST_DROPPED, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_CDC_OUT, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_LOWER_BOUND)
            | vcom_grade(VCOM_SLOT_FRAME_ERR, VCOM_GRADE_LOWER_BOUND)
            | vcom_grade(VCOM_SLOT_OVERRUN, VCOM_GRADE_LOWER_BOUND)
            | vcom_grade(VCOM_SLOT_UART_TX, VCOM_GRADE_LOWER_BOUND)
            | vcom_grade(VCOM_SLOT_RX_DMA_ERR, VCOM_GRADE_EXACT),
        units: vcom_episodes(VCOM_SLOT_FRAME_ERR)
            | vcom_episodes(VCOM_SLOT_OVERRUN)
            | vcom_episodes(VCOM_SLOT_RX_DMA_ERR),
    }
}

/// Return UART bytes only once the CDC buffer can take them whole.
fn vcom_return(usb: &mut Usb, uart: &mut Vcom) {
    let mut rx = [0u8; CDC_EP];
    let mut taken = 0;
    let mut forwarded = 0;
    // Unconditional and first: an unsampled receive channel cannot be resynchronized.
    uart.service_rx();
    // A closed session's backlog is not the next opener's.
    if usb.vcom_take_close_edge() {
        let mut dropped = 0;
        loop {
            let n = uart.read(&mut rx);
            dropped += n;
            if n < rx.len() {
                break;
            }
        }
        // Taken, staged and discarded at once, so `cdc_in`, staged less discarded, holds.
        VCOM_UART_RX.fetch_add(dropped as u32, Ordering::Relaxed);
        VCOM_CDC_IN.fetch_add(dropped as u32, Ordering::Relaxed);
        VCOM_HOST_DROPPED.fetch_add(dropped as u32, Ordering::Relaxed);
    }
    loop {
        let room = usb.vcom_write_capacity().min(rx.len());
        if room == 0 {
            break;
        }
        let n = uart.read(&mut rx[..room]);
        if n == 0 {
            break;
        }
        taken += n;
        let staged = usb.vcom_write_buffered(&rx[..n]);
        debug_assert_eq!(staged, n);
        forwarded += staged;
    }
    // A partial packet waits for a quiet line or the coalescing deadline.
    let staged = usb.vcom_staged();
    let now = crate::ELAPSED_MS.load(Ordering::Relaxed);
    let waited = now.wrapping_sub(LAST_FLUSH_MS.load(Ordering::Relaxed));
    if staged >= CDC_EP || (staged > 0 && (taken == 0 || waited >= CDC_COALESCE_MS)) {
        usb.vcom_flush();
        LAST_FLUSH_MS.store(now, Ordering::Relaxed);
        #[cfg(feature = "probe-timing")]
        timing::bump(&timing::PROBE_CDC_IN_PKTS, 1);
    }
    // Charged after the pass settles, so the query always sees a consistent ledger.
    VCOM_UART_RX.fetch_add(taken as u32, Ordering::Relaxed);
    VCOM_CDC_IN.fetch_add(forwarded as u32, Ordering::Relaxed);
    // A close or a bus reset throws the stage away; the host never got those bytes.
    VCOM_HOST_DROPPED.fetch_add(usb.vcom_take_discarded() as u32, Ordering::Relaxed);
}

/// Consume CDC packets while the UART transmit ring has room.
fn vcom_host_traffic(usb: &mut Usb, uart: &mut Vcom) {
    // Also here: on DMA, FLEXCOMM0 fires for rate changes and transmit drains, not errors.
    observe_rx_errors();
    let mut tx = [0u8; CDC_EP];
    loop {
        let room = uart.write_capacity().min(tx.len());
        if room == 0 {
            #[cfg(feature = "probe-timing")]
            timing::bump(&timing::PROBE_TX_FULL, 1);
            break;
        }
        let n = usb.vcom_read(&mut tx[..room]);
        if n == 0 {
            // Room and nothing to take: the host paces this leg.
            #[cfg(feature = "probe-timing")]
            timing::bump(&timing::PROBE_TX_STARVED, 1);
            break;
        }
        VCOM_CDC_OUT.fetch_add(n as u32, Ordering::Relaxed);
        let queued = uart.write(&tx[..n]);
        debug_assert_eq!(queued, n);
        #[cfg(feature = "probe-timing")]
        {
            timing::bump(&timing::PROBE_CDC_OUT_PKTS, 1);
            timing::bump(&timing::PROBE_CDC_OUT_BYTES, n as u32);
        }
    }
}
