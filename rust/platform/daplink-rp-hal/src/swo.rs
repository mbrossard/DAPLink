//! Raw PL011 UART0-RX SWO capture for the Pico layout (UART0 RX on GP13).

use daplink_rs::{ByteConsumer, ByteProducer, ByteRing, RingSwo, SwoBackend};
use rp2040_hal::gpio::bank0::Gpio13;
use rp2040_hal::gpio::{Function, FunctionUart, Pin, PullType, PullUp};
use rp2040_hal::pac;
use static_cell::ConstStaticCell;

/// SWO capture ring size, matching the RP embassy seam: 8 KiB is 20.5 ms of hold at 4 MBd.
const N: usize = 8192;

/// 8-bit word length code for `UARTLCR_H.WLEN`.
const WLEN_8BIT: u8 = 3;
/// Placeholder divisor baud; the host reprograms the real rate via `DAP_SWO_Baudrate`.
const INIT_BAUD: u32 = 115_200;

/// Producer half of the SWO ring, held by the UART0 interrupt.
pub type SwoProducer = ByteProducer<'static, N>;
/// The Pico SWO slot: the raw UART0-RX mechanism paired with the ring consumer.
pub type Uart0Swo = RingSwo<UartSwoHw, ByteConsumer<'static, N>>;

/// Raw UART0 SWO mechanism: PL011 fixed-point baud programming and RX-interrupt mask/unmask. The
/// pad is muxed to the UART once at construction and never switched back.
pub struct UartSwoHw {
    uart: pac::UART0,
    /// The `clk_peri` rate the divisors are computed against, sampled once at construction.
    peri_hz: u32,
}

impl SwoBackend for UartSwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        // Trace capture can refuse: a rate the lattice cannot serve reports 0 and programs
        // nothing, leaving the previous divisors in place.
        let Some(dx64) = pl011_baud::div_x64(self.peri_hz, baud) else {
            return 0;
        };
        let (int, frac) = pl011_baud::split(dx64);
        self.uart
            .uartibrd()
            .write(|w| unsafe { w.baud_divint().bits(int) });
        self.uart
            .uartfbrd()
            .write(|w| unsafe { w.baud_divfrac().bits(frac) });
        // PL011 requires a UARTLCR_H write to latch the new divisors.
        self.uart
            .uartlcr_h()
            .modify(|_, w| unsafe { w.wlen().bits(WLEN_8BIT).fen().set_bit() });
        pl011_baud::realized(self.peri_hz, dx64)
    }

    fn prepare(&mut self) {
        // The pad is already routed; drain stale bytes before capture arms.
        while self.uart.uartfr().read().rxfe().bit_is_clear() {
            let _ = self.uart.uartdr().read().data().bits();
        }
    }

    fn arm(&mut self) {
        // Capture is gated by the interrupt mask; the UART stays enabled for the image's life.
        self.uart.uartimsc().modify(|_, w| {
            w.rxim().set_bit();
            w.rtim().set_bit()
        });
    }

    fn stop(&mut self) {
        self.uart.uartimsc().modify(|_, w| {
            w.rxim().clear_bit();
            w.rtim().clear_bit()
        });
    }
}

/// UART0 interrupt body: drain the RX FIFO into the ring producer, at most one FIFO's worth per
/// entry so a line that never idles cannot hold the interrupt. Sole producer for that ring.
pub fn swo_drain(producer: &mut SwoProducer) {
    // SAFETY: single-core, and the only other RX FIFO reads are `Swo` calls that never preempt this.
    let uart = unsafe { &*pac::UART0::ptr() };
    daplink_rs::drain_fifo(producer, daplink_rs::FIFO_DRAIN_BUDGET, || {
        uart.uartfr()
            .read()
            .rxfe()
            .bit_is_clear()
            .then(|| uart.uartdr().read().data().bits())
    });
}

/// Bring UART0 up as an RX-only SWO capture, permanently routing `rx` (GP13) to it, and return the
/// SWO slot plus the ring producer for the RX interrupt. Call once per image.
pub fn uart0_swo<F: Function, P: PullType>(
    uart: pac::UART0,
    resets: &mut pac::RESETS,
    rx: Pin<Gpio13, F, P>,
    peri_hz: u32,
) -> (Uart0Swo, SwoProducer) {
    // Bring UART0 out of reset and wait for it to be ready.
    resets.reset().modify(|_, w| w.uart0().clear_bit());
    while resets.reset_done().read().uart0().bit_is_clear() {}

    // Routed permanently, idled at mark so an undriven trace source reads high.
    let mut rx = rx
        .into_function::<FunctionUart>()
        .into_pull_type::<PullUp>();
    rx.set_input_enable(true);

    // Disable while configuring, then program the placeholder divisor.
    uart.uartcr().write(|w| w);
    let (ibrd, fbrd) = pl011_baud::split(
        pl011_baud::div_x64(peri_hz, INIT_BAUD).unwrap_or(pl011_baud::DIV_X64_MIN),
    );
    uart.uartibrd()
        .write(|w| unsafe { w.baud_divint().bits(ibrd) });
    uart.uartfbrd()
        .write(|w| unsafe { w.baud_divfrac().bits(fbrd) });

    // 8N1, FIFOs enabled (also latches the divisor).
    uart.uartlcr_h()
        .write(|w| unsafe { w.wlen().bits(WLEN_8BIT).fen().set_bit() });
    // Enable UART + RX only (RX-only SWO capture; TX stays off).
    uart.uartcr().write(|w| {
        w.uarten().set_bit();
        w.rxe().set_bit()
    });

    // `ConstStaticCell` so the ring lands in `.bss` rather than through init's stack.
    static RING: ConstStaticCell<ByteRing<N>> = ConstStaticCell::new(ByteRing::new());
    let (producer, consumer): (SwoProducer, ByteConsumer<'static, N>) = RING.take().split();
    (
        RingSwo::new(UartSwoHw { uart, peri_hz }, consumer),
        producer,
    )
}
