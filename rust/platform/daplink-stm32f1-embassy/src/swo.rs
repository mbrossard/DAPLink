//! DMA-backed UART-mode SWO capture on USART1 RX (PA10).

use core::cell::{Cell, RefCell};

use daplink_rs::{DmaRxRing, RingSwo, SwoBackend, SwoSource};
use embassy_stm32::gpio::Pin as _;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{DMA1_CH5, PA10, USART1};
use embassy_stm32::Peri;
use static_cell::ConstStaticCell;

use crate::dma::BdmaRx;
use crate::io_pin::{Line, CFG_INPUT_PULL};

/// SWO capture ring size, bounded by this part's 20 KiB SRAM.
const RING_LEN: usize = 2048;

/// Placeholder baud until the host sets one with `DAP_SWO_Baudrate`.
const INIT_BAUD: u32 = 115_200;

/// Smallest divisor the oversampling-by-16 receiver can sample at.
const BRR_MIN: u32 = 16;
const BRR_MAX: u32 = 0xFFFF;

/// Bound on the `prepare` drain: a single-byte data register plus the shift register.
const DRAIN_BOUND: u32 = 4;

/// The STM32F1 SWO slot: the raw USART-RX mechanism paired with its DMA ring.
pub type UartSwo = RingSwo<UartSwoHw, DmaSwoRx>;

/// Raw USART SWO mechanism.
pub struct UartSwoHw {
    uart: pac::usart::Usart,
    /// USART kernel clock, sampled at construction.
    kernel_hz: u32,
    cap: BdmaRx,
}

impl SwoBackend for UartSwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        // An unrepresentable rate reports 0.
        let brr = (self.kernel_hz + baud / 2) / baud;
        if !(BRR_MIN..=BRR_MAX).contains(&brr) {
            return 0;
        }
        // `BRR` changes only while the USART is disabled.
        self.uart.cr1().modify(|w| w.set_ue(false));
        self.uart.brr().write(|w| w.set_brr(brr as u16));
        self.uart.cr1().modify(|w| w.set_ue(true));
        (self.kernel_hz + brr / 2) / brr
    }

    fn prepare(&mut self) {
        // Stop before draining, or the channel refills with stale bytes.
        self.cap.stop();
        let mut drained = 0;
        while drained < DRAIN_BOUND && self.uart.sr().read().rxne() {
            let _ = self.uart.dr().read();
            drained += 1;
        }
    }

    fn arm(&mut self) {
        // Pairs with the cursor re-base in `clear`.
        self.cap.start();
    }

    fn stop(&mut self) {
        self.cap.stop();
    }
}

/// Consumer half of the capture ring.
pub struct DmaSwoRx {
    ring: RefCell<DmaRxRing<BdmaRx>>,
    /// The buffer the channel fills; read volatile, never written here.
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

/// Bring USART1 up as RX-only SWO capture on PA10 and DMA1 CH5; the caller enables both clocks.
pub fn usart1_swo(
    usart: Peri<'static, USART1>,
    kernel_hz: u32,
    rx: Peri<'static, PA10>,
    rx_dma: Peri<'static, DMA1_CH5>,
) -> UartSwo {
    // An RX-only pad takes no AF selection on this family; park it pulled up.
    let pad = Line::new(rx.port(), rx.pin());
    pad.set_high(true);
    pad.set_cfg(CFG_INPUT_PULL);
    // Claimed for ownership; the seam holds the raw registers.
    let _ = rx;
    let _ = rx_dma;

    // DMA on RXNE only; no USART condition may raise an interrupt.
    uart_setup(usart, kernel_hz);
    let uart = pac::USART1;

    static RING_MEM: ConstStaticCell<[u8; RING_LEN]> = ConstStaticCell::new([0; RING_LEN]);
    let mem: &'static [u8] = RING_MEM.take();

    let cap = BdmaRx::new(4, uart.dr().as_ptr() as u32, mem);

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

/// Raw receive-only bring-up, left for the host to arm with `DAP_SWO_Control`.
fn uart_setup(usart: Peri<'static, USART1>, kernel_hz: u32) {
    let _ = usart;
    let uart = pac::USART1;
    uart.cr1().write(|_w| {});
    uart.brr()
        .write(|w| w.set_brr(((kernel_hz + INIT_BAUD / 2) / INIT_BAUD) as u16));
    uart.cr2().write(|_w| {});
    uart.cr3().write(|w| w.set_dmar(true));
    uart.cr1().write(|w| {
        w.set_re(true);
        w.set_ue(true);
    });
}
