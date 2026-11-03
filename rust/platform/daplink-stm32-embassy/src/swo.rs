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

const PORT_STRIDE: usize = 0x400;

/// Placeholder baud until the host sets one with `DAP_SWO_Baudrate`.
const INIT_BAUD: u32 = 115_200;

/// Smallest divisor the oversampling-by-16 receiver can sample at.
const BRR_MIN: u32 = 16;
const BRR_MAX: u32 = 0xFFFF;

/// Suspends whose bound expired with the channel still busy.
static DMA_FAULTS: AtomicU32 = AtomicU32::new(0);

/// Trace-DMA faults the capture channel recovered from.
pub fn dma_faults() -> u32 {
    DMA_FAULTS.load(Ordering::Relaxed)
}

/// Bound on the `prepare` drain: the 8-byte RX FIFO plus the shift register.
const FIFO_DRAIN_BOUND: u32 = 16;

/// The STM32 SWO slot: the raw USART-RX mechanism paired with its GPDMA ring.
pub type UartSwo = RingSwo<UartSwoHw, DmaSwoRx>;

/// Linked-list item closing the ring: `CDAR` then `CLLR`; aligned so it never straddles 64 KiB.
#[repr(C, align(8))]
struct Lli([u32; 2]);

#[derive(Clone, Copy)]
pub struct DmaCapture {
    dma: pac::gpdma::Gpdma,
    ch: usize,
    request: u8,
    base: u32,
    src: u32,
    /// Written once at bring-up.
    lli: u32,
}

impl DmaCapture {
    fn regs(&self) -> pac::gpdma::Channel {
        self.dma.ch(self.ch)
    }

    fn enabled(&self) -> bool {
        self.regs().cr().read().en()
    }

    fn start(&self) {
        let ch = self.regs();
        crate::gpdma::halt(ch, &DMA_FAULTS);
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
        // The linked-list item and configuration must be visible before the enable.
        fence(Ordering::SeqCst);
        // No interrupt enables: the masked NVIC line stays silent.
        ch.cr().write(|w| w.set_en(true));
    }

    /// Stop and release the channel, leaving the ring memory alone.
    fn halt(&self) {
        crate::gpdma::halt(self.regs(), &DMA_FAULTS);
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
        // At the wrap, 0 and the reload both fold to the base.
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

/// Raw USART SWO mechanism.
pub struct UartSwoHw {
    uart: pac::usart::Usart,
    /// USART kernel clock, sampled at construction.
    kernel_hz: u32,
    cap: DmaCapture,
}

impl SwoBackend for UartSwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        // Round-to-nearest for OVER16; an unrepresentable rate reports 0.
        let brr = (self.kernel_hz + baud / 2) / baud;
        if !(BRR_MIN..=BRR_MAX).contains(&brr) {
            return 0;
        }
        // `BRR` is writable only while the USART is disabled.
        self.uart.cr1().modify(|w| w.set_ue(false));
        self.uart.brr().write(|w| w.set_brr(brr as u16));
        self.uart.cr1().modify(|w| w.set_ue(true));
        (self.kernel_hz + brr / 2) / brr
    }

    fn prepare(&mut self) {
        // Stop before draining, or the channel refills with stale bytes.
        self.cap.halt();
        let mut drained = 0;
        while drained < FIFO_DRAIN_BOUND && self.uart.isr().read().rxne() {
            let _ = self.uart.rdr().read();
            drained += 1;
        }
    }

    fn arm(&mut self) {
        // Pairs with the cursor re-base in `clear`.
        self.cap.start();
    }

    fn stop(&mut self) {
        self.cap.halt();
    }
}

/// Consumer half of the capture ring.
pub struct DmaSwoRx {
    ring: RefCell<DmaRxRing<DmaCapture>>,
    /// Read volatile, never written here.
    mem: &'static [u8],
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
        // Pairs with the counter reload in `arm`.
        self.ring.borrow_mut().reset_cursor();
        self.overrun.set(false);
    }

    fn take_overrun(&mut self) -> bool {
        self.overrun.replace(false)
    }
}

/// Bring a USART up as RX-only SWO capture on `rx`; the caller enables both clocks. Call once.
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
    // AF before the mode switch, so the pad never presents a stale function.
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

    // No USART condition may raise an interrupt.
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

    // Constant for the image's life; only the controller reads it.
    static LLI: ConstStaticCell<Lli> = ConstStaticCell::new(Lli([0; 2]));
    let lli = LLI.take();
    let lli_addr = lli as *const Lli as u32;
    // SAFETY: `lli` is exclusively owned here.
    unsafe {
        core::ptr::write_volatile(&mut lli.0[0], base);
        core::ptr::write_volatile(&mut lli.0[1], llr_value(lli_addr).0);
    }

    // Claimed for ownership; registers go through `dma`/`ch_index`.
    let _ = channel;
    let cap = DmaCapture {
        dma,
        ch: ch_index,
        request,
        base,
        src: uart.rdr().as_ptr() as u32,
        lli: lli_addr,
    };
    // Left stopped until `DAP_SWO_Control(Start)`.
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

#[cfg(test)]
pub(crate) fn stop_test() {
    extern crate std;
    use crate::gpdma::tests::{registers, reset_count, script, trapped, uart};
    use daplink_rs::dap_rs::swo::{Swo, SwoControl};
    let dma = registers();
    let uart = uart();
    let cap = DmaCapture {
        dma,
        ch: 0,
        request: 1,
        base: 0x2000_0100,
        src: 0x4000_0000,
        lli: 0x2000_1000,
    };
    let mem = std::boxed::Box::leak(std::boxed::Box::new([0u8; RING_LEN]));
    let mut swo = RingSwo::new(
        UartSwoHw {
            uart,
            kernel_hz: 100_000_000,
            cap,
        },
        DmaSwoRx {
            ring: RefCell::new(DmaRxRing::new(cap, RING_LEN)),
            mem,
            overrun: Cell::new(false),
        },
    );
    assert_eq!(swo.set_baudrate(1_000_000), 1_000_000);
    script(cap.regs(), (true, false, true), None, false);
    trapped(|| {
        swo.set_control(SwoControl::Start);
    });
    assert!(!swo.is_active());
    assert_eq!(reset_count(cap.regs()), 0);
    assert_eq!(cap.regs().dar().read(), 0);
    script(cap.regs(), (false, true, false), None, false);
    assert!(swo.set_control(SwoControl::Start));
    assert!(swo.is_active());
    let dar = cap.regs().dar().read();
    script(cap.regs(), (true, false, false), None, false);
    trapped(|| {
        swo.set_control(SwoControl::Stop);
    });
    assert!(swo.is_active());
    assert_eq!(cap.regs().dar().read(), dar);
    script(cap.regs(), (true, true, true), None, false);
    assert!(swo.set_control(SwoControl::Stop));
    assert!(!swo.is_active());
}
