//! DMA-backed UART-mode SWO capture on USART1 RX (PA10).

use core::cell::{Cell, RefCell};

use daplink_rs::{DmaRxRing, RingSwo, SwoBackend, SwoSource};
use static_cell::ConstStaticCell;
use stm32f1xx_hal::pac;

use crate::dma::BdmaRx;

/// SWO capture ring size, bounded by this part's 20 KiB SRAM.
const RING_LEN: usize = 2048;

/// Placeholder baud until the host sets one with `DAP_SWO_Baudrate`.
const INIT_BAUD: u32 = 115_200;

/// Smallest divisor the oversampling-by-16 receiver can sample at.
const BRR_MIN: u32 = 16;
const BRR_MAX: u32 = 0xFFFF;

/// Bound on the `prepare` drain: a single-byte data register plus the shift register.
const DRAIN_BOUND: u32 = 4;

/// The STM32F1 SWO slot: the raw USART1-RX mechanism paired with its DMA ring.
pub type Usart1Swo = RingSwo<UartSwoHw, DmaSwoRx>;

fn regs() -> &'static pac::usart1::RegisterBlock {
    // SAFETY: `usart1_swo` consumed the peripheral; config changes only while the channel is stopped.
    unsafe { &*pac::USART1::ptr() }
}

/// Raw USART1 SWO mechanism.
pub struct UartSwoHw {
    /// USART kernel clock (PCLK2).
    kernel_hz: u32,
    cap: BdmaRx,
}

impl SwoBackend for UartSwoHw {
    fn set_baudrate(&mut self, baud: u32) -> u32 {
        // An unrepresentable rate reports 0, so the host renegotiates.
        let brr = (self.kernel_hz + baud / 2) / baud;
        if !(BRR_MIN..=BRR_MAX).contains(&brr) {
            return 0;
        }
        // `BRR` changes only while the USART is disabled.
        regs().cr1().modify(|_, w| w.ue().clear_bit());
        regs().brr().write(|w| unsafe { w.bits(brr as u16) });
        regs().cr1().modify(|_, w| w.ue().set_bit());
        (self.kernel_hz + brr / 2) / brr
    }

    fn prepare(&mut self) {
        // Stop before draining, or the channel refills with stale bytes.
        self.cap.stop();
        let mut drained = 0;
        while drained < DRAIN_BOUND && regs().sr().read().rxne().bit_is_set() {
            let _ = regs().dr().read();
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

/// Bring USART1 up as RX-only SWO capture on DMA1 CH5; the caller configures PA10. Call once.
pub fn usart1_swo(usart: pac::USART1, kernel_hz: u32) -> Usart1Swo {
    // Only RE/UE/DMAR are set: no USART condition may raise an interrupt.
    // SAFETY: single RMW of RCC APB2ENR during single-threaded bring-up.
    let rcc = unsafe { &*pac::RCC::ptr() };
    rcc.apb2enr().modify(|_, w| w.usart1en().set_bit());
    let _ = usart;
    let uart = regs();
    uart.cr1().reset();
    uart.brr()
        .write(|w| unsafe { w.bits(((kernel_hz + INIT_BAUD / 2) / INIT_BAUD) as u16) });
    uart.cr2().reset();
    uart.cr3().write(|w| w.dmar().set_bit());
    uart.cr1().write(|w| {
        w.re().set_bit();
        w.ue().set_bit()
    });

    static RING_MEM: ConstStaticCell<[u8; RING_LEN]> = ConstStaticCell::new([0; RING_LEN]);
    let mem: &'static [u8] = RING_MEM.take();

    let cap = BdmaRx::new(4, uart.dr().as_ptr() as u32, mem);

    RingSwo::new(
        UartSwoHw { kernel_hz, cap },
        DmaSwoRx {
            ring: RefCell::new(DmaRxRing::new(cap, RING_LEN)),
            mem,
            overrun: Cell::new(false),
        },
    )
}
