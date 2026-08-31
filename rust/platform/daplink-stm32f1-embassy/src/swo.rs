//! DMA-backed UART-mode SWO capture on USART1 RX (PA10), the ST-Link trace wiring.

use core::cell::{Cell, RefCell};

use daplink_rs::{DmaRxRing, RingSwo, SwoBackend, SwoSource};
use embassy_stm32::gpio::Pin as _;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{DMA1_CH5, PA10, USART1};
use embassy_stm32::Peri;
use static_cell::ConstStaticCell;

use crate::dma::BdmaRx;
use crate::io_pin::{Line, CFG_INPUT_PULL};

/// SWO capture ring, sized against this part's 20 KiB of SRAM.
const RING_LEN: usize = 2048;

/// Placeholder divisor; the host programs the real capture rate with `DAP_SWO_Baudrate`.
const INIT_BAUD: u32 = 115_200;

/// Smallest programmable divisor: below 16 the oversampling-by-16 receiver cannot sample.
const BRR_MIN: u32 = 16;
/// Largest programmable divisor: `BRR` is 16 bits.
const BRR_MAX: u32 = 0xFFFF;

/// Bound on the `prepare` drain: a single-byte data register plus the shift register.
const DRAIN_BOUND: u32 = 4;

/// The STM32F1 SWO slot: the raw USART-RX mechanism paired with its DMA ring.
pub type UartSwo = RingSwo<UartSwoHw, DmaSwoRx>;

/// Raw USART SWO mechanism: divisor programming, plus arming and halting the capture channel.
pub struct UartSwoHw {
    uart: pac::usart::Usart,
    /// The USART kernel-clock rate the divisors are computed against, sampled at construction.
    kernel_hz: u32,
    /// The capture channel this mechanism arms.
    cap: BdmaRx,
}

impl SwoBackend for UartSwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        // Round-to-nearest; a rate outside the programmable range reports 0.
        let brr = (self.kernel_hz + baud / 2) / baud;
        if !(BRR_MIN..=BRR_MAX).contains(&brr) {
            return 0;
        }
        // `BRR` should only change while the USART is disabled; `RE` rides along in `CR1`.
        self.uart.cr1().modify(|w| w.set_ue(false));
        self.uart.brr().write(|w| w.set_brr(brr as u16));
        self.uart.cr1().modify(|w| w.set_ue(true));
        (self.kernel_hz + brr / 2) / brr
    }

    fn prepare(&mut self) {
        // Stop first, then drain: the other order refills from bytes that predate the session.
        self.cap.stop();
        let mut drained = 0;
        while drained < DRAIN_BOUND && self.uart.sr().read().rxne() {
            let _ = self.uart.dr().read();
            drained += 1;
        }
    }

    fn arm(&mut self) {
        // The source re-based its cursor between `prepare` and here; this reload pairs with it.
        self.cap.start();
    }

    fn stop(&mut self) {
        self.cap.stop();
    }
}

/// Consumer half of the capture ring: cursor arithmetic over the buffer the controller fills.
pub struct DmaSwoRx {
    /// Cursor and lap detection over the capture channel.
    ring: RefCell<DmaRxRing<BdmaRx>>,
    /// The buffer the channel fills; read volatile, never written here.
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

/// Bring USART1 up (raw) as an RX-only SWO capture on PA10, ringed through DMA1 CH5 (fixed in
/// silicon, RM0008 table 78). The caller enables both clocks and samples `kernel_hz` from RCC.
pub fn usart1_swo(
    usart: Peri<'static, USART1>,
    kernel_hz: u32,
    rx: Peri<'static, PA10>,
    rx_dma: Peri<'static, DMA1_CH5>,
) -> UartSwo {
    // A receive-only USART pad takes no AF selection on this family; park it pulled up at mark.
    let pad = Line::new(rx.port(), rx.pin());
    pad.set_high(true);
    pad.set_cfg(CFG_INPUT_PULL);
    // Claimed for ownership; the seam holds the raw registers.
    let _ = rx;
    let _ = rx_dma;

    // The DMA request raises on RXNE, and no USART condition may ever raise an interrupt.
    uart_setup(usart, kernel_hz);
    let uart = pac::USART1;

    static RING_MEM: ConstStaticCell<[u8; RING_LEN]> = ConstStaticCell::new([0; RING_LEN]);
    let mem: &'static [u8] = RING_MEM.take();

    // USART1_RX is DMA1 channel 5, i.e. index 4.
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

/// Raw receive-only bring-up; left stopped, since the host arms capture with `DAP_SWO_Control`.
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
