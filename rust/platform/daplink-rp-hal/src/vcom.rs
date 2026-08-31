//! Interrupt-fed UART1 VCOM transport for the Pico layout.

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::NVIC;
use daplink_rs::{
    vcom_episodes, vcom_grade, ByteConsumer, ByteProducer, ByteRing, VcomCounters,
    VCOM_GRADE_EXACT, VCOM_GRADE_LOWER_BOUND, VCOM_SLOT_CDC_IN, VCOM_SLOT_CDC_OUT,
    VCOM_SLOT_FRAME_ERR, VCOM_SLOT_HOST_DROPPED, VCOM_SLOT_OVERRUN, VCOM_SLOT_RING_FULL,
    VCOM_SLOT_UART_RX, VCOM_SLOT_UART_TX, VCOM_VALID_CDC_OUT, VCOM_VALID_FRAME_ERR,
    VCOM_VALID_IDENTITY, VCOM_VALID_OVERRUN, VCOM_VALID_RING_FULL, VCOM_VALID_UART_TX,
};
use rp2040_hal::pac::{Interrupt, UART1};
use static_cell::StaticCell;

use crate::CPU_HZ;

const RX_RING: usize = 2048;
const TX_RING: usize = 128;
const IRQ_FIFO_BUDGET: usize = 32;
const CLK_PERI_HZ: u32 = CPU_HZ;

pub type VcomRxProducer = ByteProducer<'static, RX_RING>;
pub type VcomRxConsumer = ByteConsumer<'static, RX_RING>;
pub type VcomTxProducer = ByteProducer<'static, TX_RING>;
pub type VcomTxConsumer = ByteConsumer<'static, TX_RING>;

pub struct VcomIrq {
    pub rx: VcomRxProducer,
    pub tx: VcomTxConsumer,
}

pub struct VcomUsb {
    pub rx: VcomRxConsumer,
    pub tx: VcomTxProducer,
}

static UART_RX: AtomicU32 = AtomicU32::new(0);
/// Bytes staged toward the CDC IN endpoint, before the discards deducted at the query.
static CDC_IN: AtomicU32 = AtomicU32::new(0);
/// Staged bytes a port close or a bus reset threw away; the only source `host_dropped` has.
static DISCARDED: AtomicU32 = AtomicU32::new(0);
static CDC_OUT: AtomicU32 = AtomicU32::new(0);
static RING_FULL: AtomicU32 = AtomicU32::new(0);
static FRAME_ERR: AtomicU32 = AtomicU32::new(0);
static OVERRUN: AtomicU32 = AtomicU32::new(0);
static UART_TX: AtomicU32 = AtomicU32::new(0);
static REALIZED_BAUD: AtomicU32 = AtomicU32::new(115_207);
static APPLIED_BAUD: AtomicU32 = AtomicU32::new(115_200);
static PENDING_BAUD: AtomicU32 = AtomicU32::new(0);

fn add(counter: &AtomicU32, value: u32) {
    cortex_m::interrupt::free(|_| {
        counter.store(
            counter.load(Ordering::Relaxed).wrapping_add(value),
            Ordering::Relaxed,
        );
    });
}

pub fn vcom_split() -> (VcomIrq, VcomUsb) {
    static RX: StaticCell<ByteRing<RX_RING>> = StaticCell::new();
    static TX: StaticCell<ByteRing<TX_RING>> = StaticCell::new();
    let (rx_producer, rx_consumer) = RX.init(ByteRing::new()).split();
    let (tx_producer, tx_consumer) = TX.init(ByteRing::new()).split();
    (
        VcomIrq {
            rx: rx_producer,
            tx: tx_consumer,
        },
        VcomUsb {
            rx: rx_consumer,
            tx: tx_producer,
        },
    )
}

pub fn vcom_irq_service(vcom: &mut VcomIrq) {
    // SAFETY: UART1 FIFO access belongs exclusively to this interrupt.
    let uart = unsafe { &*UART1::ptr() };
    if uart.uartris().read().oeris().bit_is_set() {
        add(&OVERRUN, 1);
    }

    let mut received = 0;
    while received < IRQ_FIFO_BUDGET && uart.uartfr().read().rxfe().bit_is_clear() {
        let word = uart.uartdr().read();
        if word.fe().bit_is_set() || word.pe().bit_is_set() || word.be().bit_is_set() {
            add(&FRAME_ERR, 1);
        } else if !vcom.rx.push(word.data().bits()) {
            add(&RING_FULL, 1);
        }
        received += 1;
    }
    uart.uarticr()
        .write(|w| unsafe { w.bits((1 << 7) | (1 << 8) | (1 << 9) | (1 << 10)) });

    let mut transmitted = 0;
    while transmitted < IRQ_FIFO_BUDGET && uart.uartfr().read().txff().bit_is_clear() {
        let Some(byte) = vcom.tx.pop() else {
            break;
        };
        uart.uartdr().write(|w| unsafe { w.data().bits(byte) });
        transmitted += 1;
    }
    // Credited at `UARTDR`, so it runs ahead of the wire by at most 33 bytes -- the 32-entry
    // PL011 transmit FIFO plus the shifter -- and is exact whenever the line is quiet.
    add(&UART_TX, transmitted as u32);
    let mut pending = PENDING_BAUD.load(Ordering::Relaxed);
    if vcom.tx.is_empty() && pending != 0 && uart.uartfr().read().busy().bit_is_clear() {
        program_baud(uart, pending);
        // A plain store, since ARMv6-M has no CAS: a request arriving during the programming is
        // cleared with it, and the host's next line coding re-requests it.
        PENDING_BAUD.store(0, Ordering::Release);
        pending = 0;
    }
    uart.uartimsc()
        .modify(|_, w| w.txim().bit(!vcom.tx.is_empty() || pending != 0));
}

/// Unmask TXIM and pend UART1: the PL011 raises that interrupt on a transmit-FIFO level
/// transition, so an already-idle transmitter would otherwise never enter the handler.
pub fn vcom_enable_tx_interrupt() {
    // SAFETY: USB only changes the TX mask after publishing bytes to the IRQ-owned consumer.
    let uart = unsafe { &*UART1::ptr() };
    uart.uartimsc().modify(|_, w| w.txim().set_bit());
    NVIC::pend(Interrupt::UART1_IRQ);
}

/// Divisor for a requested rate. A CDC line coding cannot be refused, so a rate the lattice
/// cannot serve takes the fastest divisor and `REALIZED_BAUD` reports what that yields.
fn vcom_div_x64(baud: u32) -> u32 {
    pl011_baud::div_x64(CLK_PERI_HZ, baud).unwrap_or(pl011_baud::DIV_X64_MIN)
}

fn program_baud(uart: &rp2040_hal::pac::uart1::RegisterBlock, baud: u32) {
    let (ibrd, fbrd) = pl011_baud::split(vcom_div_x64(baud));
    uart.uartcr().modify(|_, w| w.uarten().clear_bit());
    uart.uartibrd()
        .write(|w| unsafe { w.baud_divint().bits(ibrd) });
    uart.uartfbrd()
        .write(|w| unsafe { w.baud_divfrac().bits(fbrd) });
    uart.uartlcr_h().modify(|_, w| w);
    uart.uartcr().modify(|_, w| w.uarten().set_bit());
    let held = (u32::from(uart.uartibrd().read().baud_divint().bits()) << 6)
        | u32::from(uart.uartfbrd().read().baud_divfrac().bits());
    REALIZED_BAUD.store(pl011_baud::realized(CLK_PERI_HZ, held), Ordering::Relaxed);
    APPLIED_BAUD.store(baud, Ordering::Release);
}

pub fn vcom_request_baud(baud: u32) {
    if baud != 0 {
        PENDING_BAUD.store(baud, Ordering::Release);
        vcom_enable_tx_interrupt();
    }
}

/// Credit a drain: `taken` off the receive ring, `forwarded` into the CDC stage. The caller
/// bounds the drain by the stage's room, so a short write breaks `uart_rx == cdc_in` instead of
/// being absorbed.
pub fn vcom_account_delivery(taken: usize, forwarded: usize) {
    add(&UART_RX, taken as u32);
    add(&CDC_IN, forwarded as u32);
}

/// Charge staged bytes the CDC stream discarded; the credit moves at the query, not here.
pub fn vcom_account_discard(n: usize) {
    add(&DISCARDED, n as u32);
}

/// Charge bytes the CDC OUT endpoint handed over.
pub fn vcom_account_host_traffic(accepted: usize) {
    add(&CDC_OUT, accepted as u32);
}

pub fn vcom_realized_baud() -> u32 {
    REALIZED_BAUD.load(Ordering::Relaxed)
}

/// The seam's answer to the `0x9E` query. UART1 defers a requested rate until the transmit ring
/// and the PL011 are both idle, so pending is loaded first: a change starting between the two
/// loads then reads unsettled, never settled at the rate it is leaving.
pub fn vcom_baud_state() -> daplink_rs::VcomBaudState {
    let pending = PENDING_BAUD.load(Ordering::Acquire);
    daplink_rs::VcomBaudState::deferred(
        REALIZED_BAUD.load(Ordering::Relaxed),
        APPLIED_BAUD.load(Ordering::Relaxed),
        pending == 0,
    )
}

pub fn vcom_counters() -> VcomCounters {
    // Deducting one free-running discard total as the reply is built moves the credit across in
    // one step, so a query never reads the pair half updated.
    let discarded = DISCARDED.load(Ordering::Relaxed);
    VcomCounters {
        uart_rx: UART_RX.load(Ordering::Relaxed),
        cdc_in: CDC_IN.load(Ordering::Relaxed).wrapping_sub(discarded),
        host_dropped: discarded,
        ring_full: RING_FULL.load(Ordering::Relaxed),
        frame_err: FRAME_ERR.load(Ordering::Relaxed),
        overrun: OVERRUN.load(Ordering::Relaxed),
        cdc_out: CDC_OUT.load(Ordering::Relaxed),
        uart_tx: UART_TX.load(Ordering::Relaxed),
        rx_dma_err: 0,
        valid: VCOM_VALID_IDENTITY
            | VCOM_VALID_CDC_OUT
            | VCOM_VALID_RING_FULL
            | VCOM_VALID_FRAME_ERR
            | VCOM_VALID_OVERRUN
            | VCOM_VALID_UART_TX,
        // Per-byte at the FIFO and the endpoints; `OVERRUN` folds a pass's overruns into one
        // episode and `UART_TX` is credited before the wire, so those two stay lower bounds.
        grade: vcom_grade(VCOM_SLOT_UART_RX, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_CDC_IN, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_HOST_DROPPED, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_CDC_OUT, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_FRAME_ERR, VCOM_GRADE_EXACT)
            | vcom_grade(VCOM_SLOT_OVERRUN, VCOM_GRADE_LOWER_BOUND)
            | vcom_grade(VCOM_SLOT_UART_TX, VCOM_GRADE_LOWER_BOUND),
        units: vcom_episodes(VCOM_SLOT_OVERRUN),
    }
}

#[cfg(test)]
mod tests {
    use super::vcom_div_x64;

    #[test]
    fn a_rate_the_lattice_cannot_serve_takes_the_fastest_divisor() {
        assert_eq!(vcom_div_x64(u32::MAX), pl011_baud::DIV_X64_MIN);
    }
}
