//! DMA-backed PL011 UART-RX SWO capture for the Pico layout (UART0 RX on GP13).

use core::cell::{Cell, RefCell, UnsafeCell};
use core::sync::atomic::{compiler_fence, Ordering};

use daplink_rp_dma_uart::seq::DmaChannel;
use daplink_rs::{Activity, DmaRxChannel, DmaRxRing, RingSwo, SwoBackend, SwoSource};
use embassy_rp::dma::ChannelInstance;
use embassy_rp::gpio::Pin;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::TreqSel;
use embassy_rp::pac::uart::Uart as UartRegs;
use embassy_rp::Peri;
use static_cell::ConstStaticCell;

/// SWO capture ring capacity in bytes.
const RING_LEN: usize = 8192;

/// `log2(RING_LEN)`, the DMA write-address wrap width; the buffer is aligned to match.
const RING_SIZE_LOG2: u8 = 13;

// One fact: a mismatch would wrap the controller somewhere other than the buffer.
const _: () = assert!(RING_LEN == 1 << RING_SIZE_LOG2);

const FUNCSEL_UART: u8 = 2;
const WLEN_8BIT: u8 = 3;
/// Placeholder divisor baud; the host reprograms the real rate via `DAP_SWO_Baudrate`.
const INIT_BAUD: u32 = 115_200;
/// Transfer count an RP2040 channel is armed with; it has no self-triggering counter mode.
#[cfg(feature = "rp2040")]
const RP2040_COUNT: u32 = u32::MAX;

/// The Pico SWO slot: the raw UART0-RX mechanism paired with its DMA ring.
pub type Uart0Swo = RingSwo<UartSwoHw, DmaSwoRx>;

/// The one DMA channel filling the capture ring; `Copy` so the mechanism and the source both hold it.
#[derive(Clone, Copy)]
pub struct DmaCapture {
    ch: usize,
    /// Ring buffer base address, aligned to [`RING_LEN`].
    base: u32,
    src: u32,
}

impl DmaCapture {
    fn chan(&self) -> DmaChannel {
        // SAFETY: uart0_swo consumes the token; mechanism and source serialize their copies.
        unsafe { DmaChannel::new(self.ch) }
    }

    fn regs(&self) -> pac::dma::Channel {
        // SAFETY: all configuration stays in this module's serialized capture lifecycle.
        unsafe { self.chan().regs() }
    }

    fn enabled(&self) -> bool {
        self.chan().enabled()
    }

    /// Arm at the ring's base: byte-wide, wrapping, paced by the UART0 RX DREQ, chained to nothing.
    fn start(&self) {
        let ch = self.regs();
        // SAFETY: stopped claimed channel, owned UART0 source and static aligned byte ring.
        unsafe { self.chan().set_addrs(self.src, self.base) };
        self.load_count();
        compiler_fence(Ordering::SeqCst);
        ch.ctrl_trig().write(|w| {
            w.set_en(true);
            w.set_incr_read(false);
            w.set_incr_write(true);
            // Wrapping the WRITE address inside the aligned ring is what makes it circular.
            w.set_ring_sel(true);
            w.set_ring_size(RING_SIZE_LOG2);
            w.set_treq_sel(TreqSel::UART0_RX);
            // A channel chained to itself is a channel with chaining disabled.
            w.set_chain_to(self.ch as u8);
            w.set_irq_quiet(true);
        });
        compiler_fence(Ordering::SeqCst);
    }

    /// Stop capture; abort timeout permanently halts without releasing or rebasing the ring.
    fn halt(&self) {
        // SAFETY: both capture halves retain the channel and its static ring during abort.
        if !unsafe { self.chan().abort() } {
            crate::dma_tx::abort_failed();
        }
    }
}

/// Chip-specific halves of the transfer counter: RP2350 re-triggers itself, RP2040 cannot.
#[cfg(feature = "rp2350")]
impl DmaCapture {
    /// Arm one lap in self-triggering mode: count a ring down, re-trigger, reload.
    fn load_count(&self) {
        // SAFETY: start configures only the stopped channel and preserves the ring extent.
        unsafe { self.chan().set_count_retriggering(RING_LEN as u32) };
    }

    /// Units left in the current lap, straight off the reloading counter.
    fn lap_remaining(&self) -> usize {
        self.chan().remaining() as usize
    }

    /// The channel re-triggers itself, so the count can never run out.
    fn exhausted(&self) -> bool {
        false
    }
}

#[cfg(feature = "rp2040")]
impl DmaCapture {
    /// Arm the counter at its maximum: with no self-trigger, one long count carries the ring.
    fn load_count(&self) {
        // SAFETY: start configures only the stopped channel; address wrapping bounds the run.
        unsafe { self.chan().set_count(RP2040_COUNT) };
    }

    /// Units left in the current lap, derived from how far the one long count has run.
    fn lap_remaining(&self) -> usize {
        rp2040_lap_remaining(self.chan().remaining())
    }

    /// Whether the count ran out and the channel stopped mid-session.
    fn exhausted(&self) -> bool {
        self.enabled() && self.chan().remaining() == 0
    }
}

/// Units left in the lap once the long count has `remaining` transfers to go.
#[cfg(feature = "rp2040")]
fn rp2040_lap_remaining(remaining: u32) -> usize {
    let written = RP2040_COUNT.wrapping_sub(remaining) as usize;
    RING_LEN - (written & (RING_LEN - 1))
}

impl DmaRxChannel for DmaCapture {
    fn remaining(&self) -> usize {
        self.lap_remaining()
    }

    fn reload(&mut self, _len: usize) {
        self.halt();
        self.start();
    }

    fn running(&self) -> bool {
        self.enabled()
    }
}

/// Raw UART0 SWO mechanism: PL011 baud programming and capture arming; the pad is muxed once.
pub struct UartSwoHw {
    uart: UartRegs,
    /// The `clk_peri` rate the divisors are computed against, sampled once at construction.
    peri_hz: u32,
    /// The capture channel this mechanism arms.
    cap: DmaCapture,
}

impl SwoBackend for UartSwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        let Some(dx64) = pl011_baud::div_x64(self.peri_hz, baud) else {
            return 0;
        };
        let (int, frac) = pl011_baud::split(dx64);
        self.uart.uartibrd().write(|w| w.set_baud_divint(int));
        self.uart.uartfbrd().write(|w| w.set_baud_divfrac(frac));
        // PL011 requires a UARTLCR_H write to latch the new divisors.
        self.uart.uartlcr_h().modify(|w| {
            w.set_wlen(WLEN_8BIT);
            w.set_fen(true);
        });
        pl011_baud::realized(self.peri_hz, dx64)
    }

    fn prepare(&mut self) {
        // Stop first: in the other order the channel refills from bytes that predate the session.
        self.cap.halt();
        while !self.uart.uartfr().read().rxfe() {
            let _ = self.uart.uartdr().read().data();
        }
    }

    fn arm(&mut self) {
        // Pairs with the cursor re-base the source did between `prepare` and here.
        self.cap.start();
    }

    fn stop(&mut self) {
        self.cap.halt();
    }
}

/// Consumer half of the capture ring; interior mutability because `DAP_SWO_Status` counts via `&self`.
pub struct DmaSwoRx {
    /// Cursor and lap detection over the capture channel.
    ring: RefCell<DmaRxRing<DmaCapture>>,
    /// The buffer the controller fills; read volatile, never written here.
    mem: &'static RingMem,
    /// The capture channel, for the counter-exhaustion check the cursor cannot make.
    cap: DmaCapture,
    /// Sticky overrun, set by a lap or by a counter that ran out mid-session.
    overrun: Cell<bool>,
    /// Capture activity LED, pulsed once per non-empty read.
    led: Option<&'static mut dyn Activity>,
}

impl DmaSwoRx {
    /// Wire an activity LED, pulsed once per read that yields bytes. Call once, at bring-up.
    pub fn attach_led(&mut self, led: &'static mut dyn Activity) {
        self.led = Some(led);
    }

    /// Re-arm a counter that ran out mid-session; it discards the ring, so it is an overrun.
    fn rearm_if_exhausted(&self, ring: &mut DmaRxRing<DmaCapture>) {
        rearm_if(self.cap.exhausted(), ring, &self.overrun);
    }
}

#[inline(always)]
fn rearm_if<C: DmaRxChannel>(exhausted: bool, ring: &mut DmaRxRing<C>, overrun: &Cell<bool>) {
    if exhausted {
        ring.restart();
        overrun.set(true);
    }
}

impl SwoSource for DmaSwoRx {
    fn len(&self) -> usize {
        let mut ring = self.ring.borrow_mut();
        self.rearm_if_exhausted(&mut ring);
        if ring.observe() > 0 {
            self.overrun.set(true);
        }
        ring.available()
    }

    fn capacity(&self) -> usize {
        RING_LEN
    }

    fn drain(&mut self, buf: &mut [u8]) -> usize {
        let n = {
            let ring = &mut *self.ring.borrow_mut();
            self.rearm_if_exhausted(ring);
            let (n, lost) = ring.read_with(buf, |index| Some(self.mem.read(index)));
            if lost > 0 {
                self.overrun.set(true);
            }
            n
        };
        if n > 0 {
            if let Some(led) = self.led.as_mut() {
                led.pulse();
            }
        }
        n
    }

    fn clear(&mut self) {
        // Between `prepare` and `arm`, so this re-base is paired with a counter reload.
        self.ring.borrow_mut().reset_cursor();
        self.overrun.set(false);
    }

    fn take_overrun(&mut self) -> bool {
        self.overrun.replace(false)
    }
}

/// Capture ring memory, aligned to its own size so the write-address wrap lands inside it.
#[repr(C, align(8192))]
struct RingMem(UnsafeCell<[u8; RING_LEN]>);

impl RingMem {
    fn read(&self, index: usize) -> u8 {
        assert!(index < RING_LEN);
        // SAFETY: static initialized storage; no references to DMA-written bytes are formed.
        unsafe { self.0.get().cast::<u8>().add(index).read_volatile() }
    }
}

// The alignment attribute takes a literal, so only this ties it to the wrap width.
const _: () = assert!(core::mem::align_of::<RingMem>() == RING_LEN);

/// Bring UART0 up as RX-only SWO capture on GP13 with `dma` filling the ring; call once per image.
pub fn uart0_swo<P: Pin, D: ChannelInstance>(
    _uart: Peri<'static, embassy_rp::peripherals::UART0>,
    rx: Peri<'static, P>,
    dma: Peri<'static, D>,
) -> Uart0Swo {
    let uart = pac::UART0;
    pac::RESETS.reset().modify(|w| w.set_uart0(false));
    while !pac::RESETS.reset_done().read().uart0() {}

    // Route the RX pad to the UART, permanently.
    let gp = rx.pin() as usize;
    pac::IO_BANK0
        .gpio(gp)
        .ctrl()
        .write(|w| w.set_funcsel(FUNCSEL_UART));
    pac::PADS_BANK0.gpio(gp).modify(|w| {
        w.set_ie(true);
        // Idle the line at mark; a driven trace source overrides either pull.
        w.set_pde(false);
        w.set_pue(true);
        // RP235x pads reset isolated, and this raw path never clears the latch otherwise.
        #[cfg(feature = "rp2350")]
        w.set_iso(false);
    });

    uart.uartcr().write(|_w| {});
    let peri_hz = embassy_rp::clocks::clk_peri_freq();
    let (ibrd, fbrd) = pl011_baud::split(
        pl011_baud::div_x64(peri_hz, INIT_BAUD).unwrap_or(pl011_baud::DIV_X64_MIN),
    );
    uart.uartibrd().write(|w| w.set_baud_divint(ibrd));
    uart.uartfbrd().write(|w| w.set_baud_divfrac(fbrd));

    // 8N1, FIFOs enabled (also latches the divisor).
    uart.uartlcr_h().write(|w| {
        w.set_wlen(WLEN_8BIT);
        w.set_fen(true);
    });
    uart.uartcr().write(|w| {
        w.set_uarten(true);
        w.set_rxe(true);
    });
    // No UART condition may raise an interrupt here, and a warm boot can leave the mask set.
    uart.uartimsc().write(|_w| {});
    // Whole-register write, leaving DMAONERR clear: a framing error must not gate RX requests off.
    uart.uartdmacr().write(|w| w.set_rxdmae(true));

    static RING_MEM: ConstStaticCell<RingMem> =
        ConstStaticCell::new(RingMem(UnsafeCell::new([0; RING_LEN])));
    let mem = RING_MEM.take();
    let base = mem.0.get() as u32;
    let mem: &'static RingMem = mem;

    // The channel index comes from the `Peripherals` singleton, so no other owner can hold it.
    let _ = dma;
    let cap = DmaCapture {
        ch: D::number() as usize,
        base,
        src: uart.uartdr().as_ptr() as u32,
    };
    // Left stopped: the host arms capture with DAP_SWO_Control(Start).
    cap.halt();

    RingSwo::new(
        UartSwoHw { uart, peri_hz, cap },
        DmaSwoRx {
            ring: RefCell::new(DmaRxRing::new(cap, RING_LEN)),
            mem,
            cap,
            overrun: Cell::new(false),
            led: None,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "rp2040")]
    #[test]
    fn rp2040_lap_wraps_with_the_write_address() {
        assert_eq!(rp2040_lap_remaining(RP2040_COUNT), RING_LEN);
        assert_eq!(rp2040_lap_remaining(RP2040_COUNT - 1), RING_LEN - 1);
        assert_eq!(
            rp2040_lap_remaining(RP2040_COUNT - (RING_LEN as u32 - 1)),
            1
        );
        assert_eq!(
            rp2040_lap_remaining(RP2040_COUNT - RING_LEN as u32),
            RING_LEN
        );
        // An exhausted count has written u32::MAX bytes, leaving the address one short of a lap.
        assert_eq!(rp2040_lap_remaining(0), 1);
        for remaining in [0, 1, 7, 8191, 8192, 8193, 1 << 20, RP2040_COUNT] {
            let written = RP2040_COUNT.wrapping_sub(remaining) as usize;
            assert_eq!(
                RING_LEN - rp2040_lap_remaining(remaining),
                written % RING_LEN
            );
        }
    }

    struct Chan<'a> {
        remaining: &'a Cell<usize>,
        reloads: &'a Cell<u32>,
    }

    impl DmaRxChannel for Chan<'_> {
        fn remaining(&self) -> usize {
            self.remaining.get()
        }

        fn reload(&mut self, len: usize) {
            self.remaining.set(len);
            self.reloads.set(self.reloads.get() + 1);
        }
    }

    #[test]
    fn an_exhausted_count_restarts_the_ring_and_flags_an_overrun() {
        let (remaining, reloads, overrun) = (Cell::new(11), Cell::new(0), Cell::new(false));
        let mut ring = DmaRxRing::new(
            Chan {
                remaining: &remaining,
                reloads: &reloads,
            },
            16,
        );
        ring.observe();
        assert_eq!(ring.available(), 5);
        rearm_if(false, &mut ring, &overrun);
        assert_eq!(
            (ring.available(), reloads.get(), overrun.get()),
            (5, 0, false)
        );
        rearm_if(true, &mut ring, &overrun);
        assert_eq!(
            (ring.available(), reloads.get(), overrun.get()),
            (0, 1, true)
        );
        assert_eq!(ring.write_pos(), 0);
    }
}
