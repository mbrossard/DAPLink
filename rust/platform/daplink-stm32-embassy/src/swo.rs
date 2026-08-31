//! DMA-backed USART-RX SWO capture for STM32 parts with a GPDMA.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{fence, AtomicU32, Ordering};

use daplink_rs::{DmaRxChannel, DmaRxRing, RingSwo, SwoBackend, SwoSource};
use embassy_stm32::dma::ChannelInstance;
use embassy_stm32::gpio::Pin;
use embassy_stm32::pac;
use embassy_stm32::pac::gpdma::vals::{Ap, Dreq, Dw};
use embassy_stm32::pac::gpio::vals::{Moder, Pupdr};
use embassy_stm32::Peri;
use static_cell::ConstStaticCell;

const RING_LEN: usize = 8192;

// BNDT is 16 bits of bytes, so the whole ring must fit one block.
const _: () = assert!(RING_LEN <= 0xFFFF);

/// GPIO ports sit `0x400` apart on every STM32.
const PORT_STRIDE: usize = 0x400;

/// Placeholder divisor; the host programs the real capture rate with `DAP_SWO_Baudrate`.
const INIT_BAUD: u32 = 115_200;

/// Smallest programmable divisor: below 16 the oversampling-by-16 receiver cannot sample.
const BRR_MIN: u32 = 16;
/// Largest programmable divisor: `BRR` is 16 bits.
const BRR_MAX: u32 = 0xFFFF;

/// Bound on the suspend handshake spin, so a wedged channel cannot hang the DAP loop.
const SUSPEND_SPINS: u32 = 10_000;

/// Suspends whose bound expired with the channel still busy, so the reset landed on running
/// hardware; the trace the seam hands out across one is not evidence.
static DMA_FAULTS: AtomicU32 = AtomicU32::new(0);

/// Trace-DMA faults the capture channel recovered from; a diagnostic reads it over SWD, the
/// `0x9D` reply having no slot for it.
pub fn dma_faults() -> u32 {
    DMA_FAULTS.load(Ordering::Relaxed)
}

/// Suspend `ch` and wait for it to let go, charging a fault when the bound expires; the caller
/// resets the channel either way, so an expired bound is recorded rather than passed over.
fn suspend(ch: pac::gpdma::Channel) {
    if !ch.cr().read().en() {
        return;
    }
    // Reset is only valid on a suspended or idle channel.
    ch.cr().modify(|w| w.set_susp(true));
    let mut spins = 0;
    while spins < SUSPEND_SPINS {
        let sr = ch.sr().read();
        if sr.suspf() || sr.idlef() {
            return;
        }
        spins += 1;
    }
    DMA_FAULTS.fetch_add(1, Ordering::Relaxed);
}

/// Bound on the `prepare` FIFO drain: the RX FIFO holds 8 bytes plus the shift register.
const FIFO_DRAIN_BOUND: u32 = 16;

/// The STM32 SWO slot: the raw USART-RX mechanism paired with its GPDMA ring.
pub type UartSwo = RingSwo<UartSwoHw, DmaSwoRx>;

/// Linked-list item closing the ring: `CDAR` then `CLLR`, in fetch order. 8-byte aligned so it
/// cannot straddle the 64 KiB window `CLBAR` addresses.
#[repr(C, align(8))]
struct Lli([u32; 2]);

/// The GPDMA channel that fills the capture ring.
#[derive(Clone, Copy)]
pub struct DmaCapture {
    dma: pac::gpdma::Gpdma,
    ch: usize,
    /// Hardware request line for the capture USART's RX.
    request: u8,
    base: u32,
    /// The USART receive data register the channel reads.
    src: u32,
    /// Address of the linked-list item, written once at bring-up.
    lli: u32,
}

impl DmaCapture {
    fn regs(&self) -> pac::gpdma::Channel {
        self.dma.ch(self.ch)
    }

    fn enabled(&self) -> bool {
        self.regs().cr().read().en()
    }

    /// Arm the transfer at the ring's base, looped through the linked-list item.
    fn start(&self) {
        let ch = self.regs();
        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| {
            w.set_dtef(true);
            w.set_htf(true);
            w.set_suspf(true);
            w.set_tcf(true);
            w.set_tof(true);
            w.set_ulef(true);
            w.set_usef(true);
        });
        ch.lbar().write(|w| w.set_lba((self.lli >> 16) as u16));
        ch.tr1().write(|w| {
            w.set_sdw(Dw::BYTE);
            w.set_ddw(Dw::BYTE);
            w.set_sinc(false);
            w.set_dinc(true);
            w.set_sap(Ap::PORT1);
            w.set_dap(Ap::PORT0);
        });
        ch.tr2().write(|w| {
            w.set_dreq(Dreq::SOURCE_PERIPHERAL);
            w.set_reqsel(self.request);
        });
        ch.tr3().write(|_w| {});
        ch.br1().write(|w| w.set_bndt(RING_LEN as u16));
        ch.sar().write_value(self.src);
        ch.dar().write_value(self.base);
        ch.llr().write_value(llr_value(self.lli));
        // The linked-list item and the configuration must be visible before the enable.
        fence(Ordering::SeqCst);
        // A whole-register write: no interrupt enables, so the masked NVIC line stays silent.
        ch.cr().write(|w| w.set_en(true));
    }

    /// Stop the transfer and release the channel, leaving the ring memory alone.
    fn halt(&self) {
        let ch = self.regs();
        suspend(ch);
        ch.cr().write(|w| w.set_reset(true));
        // The DMA's ring writes must be visible before the CPU reads the backlog out.
        fence(Ordering::SeqCst);
    }
}

/// `CLLR` image for the item at `lli`: reload `CDAR` and `CLLR`; `UB1` clear reloads `BNDT`.
fn llr_value(lli: u32) -> pac::gpdma::regs::ChLlr {
    let mut v = pac::gpdma::regs::ChLlr(0);
    v.set_uda(true);
    v.set_ull(true);
    v.set_la(((lli & 0xFFFF) >> 2) as u16);
    v
}

impl DmaRxChannel for DmaCapture {
    fn remaining(&self) -> usize {
        // Bytes left in the current lap; at the wrap, 0 and the reload both fold to the base.
        self.regs().br1().read().bndt() as usize
    }

    fn reload(&mut self, _len: usize) {
        self.halt();
        self.start();
    }

    fn running(&self) -> bool {
        self.enabled()
    }
}

/// Raw USART SWO mechanism: divisor programming, plus arming and halting the capture channel.
pub struct UartSwoHw {
    uart: pac::usart::Usart,
    /// The USART kernel-clock rate the divisors are computed against, sampled at construction.
    kernel_hz: u32,
    cap: DmaCapture,
}

impl SwoBackend for UartSwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        // Round-to-nearest for OVER16; a rate outside the programmable range reports 0.
        let brr = (self.kernel_hz + baud / 2) / baud;
        if !(BRR_MIN..=BRR_MAX).contains(&brr) {
            return 0;
        }
        // `BRR` is writable only while the USART is disabled; `RE` rides along in `CR1`.
        self.uart.cr1().modify(|w| w.set_ue(false));
        self.uart.brr().write(|w| w.set_brr(brr as u16));
        self.uart.cr1().modify(|w| w.set_ue(true));
        (self.kernel_hz + brr / 2) / brr
    }

    fn prepare(&mut self) {
        // Stop first, then drain: the other order refills from bytes that predate the session.
        self.cap.halt();
        let mut drained = 0;
        while drained < FIFO_DRAIN_BOUND && self.uart.isr().read().rxne() {
            let _ = self.uart.rdr().read();
            drained += 1;
        }
    }

    fn arm(&mut self) {
        // The source re-based its cursor between `prepare` and here; this reload pairs with it.
        self.cap.start();
    }

    fn stop(&mut self) {
        self.cap.halt();
    }
}

/// Consumer half of the capture ring: cursor arithmetic over the buffer the controller fills.
pub struct DmaSwoRx {
    /// Cursor and lap detection over the capture channel.
    ring: RefCell<DmaRxRing<DmaCapture>>,
    /// The buffer the controller fills; read volatile, never written here.
    mem: &'static [u8],
    /// Sticky overrun, set by a lap.
    overrun: Cell<bool>,
}

impl SwoSource for DmaSwoRx {
    fn len(&self) -> usize {
        let mut ring = self.ring.borrow_mut();
        if ring.observe() > 0 {
            self.overrun.set(true);
        }
        ring.available()
    }

    fn capacity(&self) -> usize {
        RING_LEN
    }

    fn drain(&mut self, buf: &mut [u8]) -> usize {
        let (n, lost) = self.ring.borrow_mut().read(self.mem, buf);
        if lost > 0 {
            self.overrun.set(true);
        }
        n
    }

    fn clear(&mut self) {
        // Halted by `prepare` and re-armed by `arm`, so the re-base is paired with a reload.
        self.ring.borrow_mut().reset_cursor();
        self.overrun.set(false);
    }

    fn take_overrun(&mut self) -> bool {
        self.overrun.replace(false)
    }
}

/// Bring a USART up (raw) as an RX-only SWO capture, permanently routing `rx` to it. The caller
/// enables both peripheral clocks and supplies the kernel-clock rate, the pad's AF, and the
/// channel's controller, index and request line. Call once per image.
#[allow(clippy::too_many_arguments)]
pub fn uart_swo(
    uart: pac::usart::Usart,
    kernel_hz: u32,
    rx: Peri<'static, impl Pin>,
    rx_af: u8,
    dma: pac::gpdma::Gpdma,
    ch_index: usize,
    request: u8,
    channel: Peri<'static, impl ChannelInstance>,
) -> UartSwo {
    // AF before the mode switch, so the pad never presents a stale function; pull-up idles at mark.
    let port = unsafe {
        // SAFETY: the PAC's own GPIOA block plus the fixed stride, for a port the caller owns.
        pac::gpio::Gpio::from_ptr(
            (pac::GPIOA.as_ptr() as usize + PORT_STRIDE * rx.port() as usize) as *mut (),
        )
    };
    let pin = rx.pin() as usize;
    port.afr(pin / 8).modify(|w| w.set_afr(pin % 8, rx_af));
    port.pupdr().modify(|w| w.set_pupdr(pin, Pupdr::PULL_UP));
    port.moder().modify(|w| w.set_moder(pin, Moder::ALTERNATE));

    // No USART condition may raise an interrupt; `OVRDIS` disables overrun detection here.
    uart.cr1().write(|_w| {});
    uart.presc().write(|_w| {});
    uart.brr()
        .write(|w| w.set_brr(((kernel_hz + INIT_BAUD / 2) / INIT_BAUD) as u16));
    uart.cr2().write(|_w| {});
    uart.cr3().write(|w| {
        w.set_dmar(true);
        w.set_ovrdis(true);
    });
    uart.cr1().write(|w| {
        w.set_fifoen(true);
        w.set_re(true);
        w.set_ue(true);
    });

    static RING_MEM: ConstStaticCell<[u8; RING_LEN]> = ConstStaticCell::new([0; RING_LEN]);
    let mem = RING_MEM.take();
    let base = mem.as_ptr() as u32;
    let mem: &'static [u8] = &*mem;

    // Constant for the life of the image; only the controller ever reads it back.
    static LLI: ConstStaticCell<Lli> = ConstStaticCell::new(Lli([0; 2]));
    let lli = LLI.take();
    let lli_addr = lli as *const Lli as u32;
    // SAFETY: `lli` is exclusively owned here; the words are `CDAR` then `CLLR`, the fetch order.
    unsafe {
        core::ptr::write_volatile(&mut lli.0[0], base);
        core::ptr::write_volatile(&mut lli.0[1], llr_value(lli_addr).0);
    }

    // The channel peripheral is claimed for ownership; the registers go through `dma`/`ch_index`.
    let _ = channel;
    let cap = DmaCapture {
        dma,
        ch: ch_index,
        request,
        base,
        src: uart.rdr().as_ptr() as u32,
        lli: lli_addr,
    };
    // Left stopped: the host arms capture with DAP_SWO_Control(Start).
    cap.halt();

    RingSwo::new(
        UartSwoHw {
            uart,
            kernel_hz,
            cap,
        },
        DmaSwoRx {
            ring: RefCell::new(DmaRxRing::new(cap, RING_LEN)),
            mem,
            overrun: Cell::new(false),
        },
    )
}
