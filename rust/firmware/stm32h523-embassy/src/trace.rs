//! `dap-trace` marks for the USB and DAP interrupts, each carrying the ICACHE miss monitor.

use daplink_embassy_usb::trace;
use embassy_stm32::interrupt::typelevel::{Handler, USB_DRD_FS};
use embassy_stm32::interrupt::InterruptExt;

use crate::DAP_IRQ;

/// Trace codes for interrupt entry and exit; the argument is the ICACHE miss monitor.
const TRACE_USB_ENTER: u8 = trace::EV_USER + 1;
const TRACE_USB_EXIT: u8 = trace::EV_USER + 2;
const TRACE_DTS_ENTER: u8 = trace::EV_USER + 3;
const TRACE_DTS_EXIT: u8 = trace::EV_USER + 4;

/// ICACHE_CR, whose miss-monitor enable and reset bits the marks drive.
const ICACHE_CR: *mut u32 = 0x4003_0400 as *mut u32;

/// Stamps each USB correct-transfer interrupt; both CTR flags are read, as `ISTR.DIR` shows one.
pub(crate) struct UsbTraceHandler;

impl Handler<USB_DRD_FS> for UsbTraceHandler {
    unsafe fn on_interrupt() {
        // Preempting DTS, a restart would cut the executor's count short.
        trace::mark(TRACE_USB_ENTER, icache_miss_entry(DAP_IRQ.is_active()));
        let usb = embassy_stm32::pac::USB;
        // The driver never enables SOF, so arm it here; the frame stamp dates every other event.
        if !usb.cntr().read().sofm() {
            usb.cntr().modify(|w| w.set_sofm(true));
        }
        let istr = usb.istr().read();
        let frame = (usb.fnr().read().0 & 0x3fff) as u16;
        if istr.sof() {
            trace::mark(trace::EV_SOF, frame);
            let mut clear = embassy_stm32::pac::usb::regs::Istr(!0);
            clear.set_sof(false);
            usb.istr().write_value(clear);
        }
        if istr.ctr() {
            let ep = istr.ep_id() as usize;
            let epr = usb.epr(ep).read();
            if epr.ctr_rx() {
                trace::mark_aux(trace::EV_CTR_OUT, ep as u8, frame);
            }
            if epr.ctr_tx() {
                trace::mark_aux(trace::EV_CTR_IN, ep as u8, frame);
            }
        }
    }
}

pub(crate) struct UsbTraceExit;

impl Handler<USB_DRD_FS> for UsbTraceExit {
    unsafe fn on_interrupt() {
        trace::mark(TRACE_USB_EXIT, icache_misses());
    }
}

pub(crate) fn dts_enter() {
    trace::mark(TRACE_DTS_ENTER, icache_miss_entry(false));
}

pub(crate) fn dts_exit() {
    trace::mark(TRACE_DTS_EXIT, icache_misses());
}

/// Turn the miss monitor on (ICACHE_CR MISSMEN); call once the cache is enabled.
pub(crate) fn enable_miss_monitor() {
    // SAFETY: ICACHE_CR; only the miss monitor enable is set.
    unsafe { core::ptr::write_volatile(ICACHE_CR, core::ptr::read_volatile(ICACHE_CR) | 1 << 17) }
}

/// The ICACHE miss monitor (RM0481 ICACHE_MMONR), which saturates at 16 bits.
fn icache_misses() -> u16 {
    // SAFETY: a read-only monitor register.
    unsafe { core::ptr::read_volatile(0x4003_0414 as *const u32) as u16 }
}

/// An entry mark's miss count: restarted from 0, or snapshotted under an outer handler.
fn icache_miss_entry(nested: bool) -> u16 {
    if nested {
        return icache_misses();
    }
    // SAFETY: ICACHE_CR; MISSMRST is set, then cleared, since counting needs the reset released.
    unsafe {
        let v = core::ptr::read_volatile(ICACHE_CR);
        core::ptr::write_volatile(ICACHE_CR, v | 1 << 19);
        core::ptr::write_volatile(ICACHE_CR, v & !(1 << 19));
    }
    0
}
