//! Diagnostic USB1 pass timing and counters, read out of RAM over SWD.

use core::sync::atomic::{AtomicU32, Ordering};

/// USB1 and FLEXCOMM0 entries, and worst core cycles per pass.
#[no_mangle]
pub static PROBE_USB1_PASSES: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_FC0_ENTRIES: AtomicU32 = AtomicU32::new(0);
// The `_US` statics hold core cycles; the bench reads them by these names.
#[no_mangle]
pub static PROBE_USB1_MAX_US: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_VCOM_MAX_US: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_USB1_TOTAL_US: AtomicU32 = AtomicU32::new(0);

/// CDC packets each way.
#[no_mangle]
pub static PROBE_CDC_OUT_PKTS: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_CDC_OUT_BYTES: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_CDC_IN_PKTS: AtomicU32 = AtomicU32::new(0);

/// Level-driven bits still asserted on entry, sampled before `poll`.
#[no_mangle]
pub static PROBE_IS_EP1OUT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_IS_EP3OUT: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_IS_EP3IN: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_IS_FRAME: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_IS_DEV: AtomicU32 = AtomicU32::new(0);
/// DMA0 handler entries, read against the transmit path's armed count.
#[no_mangle]
pub static PROBE_DMA0_ENTRIES: AtomicU32 = AtomicU32::new(0);

/// Passes that found the DAP request queue full, so EP1 OUT was left unread.
#[no_mangle]
pub static PROBE_DAP_FULL: AtomicU32 = AtomicU32::new(0);

/// Passes entering with EP3 OUT armed versus with a packet waiting.
#[no_mangle]
pub static PROBE_EP3OUT_ARMED: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_EP3OUT_LANDED: AtomicU32 = AtomicU32::new(0);

/// Sample EP3 OUT's active bit (bit 31 of its first OUT buffer entry).
pub fn sample_ep3out() {
    // SAFETY: a plain volatile read of the controller's command/status list.
    let entry = unsafe { core::ptr::read_volatile(0x4010_0030 as *const u32) };
    if entry & (1 << 31) != 0 {
        bump(&PROBE_EP3OUT_ARMED, 1);
    } else {
        bump(&PROBE_EP3OUT_LANDED, 1);
    }
}

/// Charge one pass's worth of USB1 `INTSTAT`.
pub fn sample_intstat() {
    // SAFETY: a plain volatile read of a memory-mapped status register.
    let bits = unsafe { core::ptr::read_volatile(0x4009_4020 as *const u32) };
    for (mask, cell) in [
        (1 << 2, &PROBE_IS_EP1OUT),
        (1 << 6, &PROBE_IS_EP3OUT),
        (1 << 7, &PROBE_IS_EP3IN),
        (1 << 30, &PROBE_IS_FRAME),
        (1 << 31, &PROBE_IS_DEV),
    ] {
        if bits & mask != 0 {
            bump(cell, 1);
        }
    }
}

/// Forward-leg passes stopped by a full transmit ring, then passes with room and nothing to send.
#[no_mangle]
pub static PROBE_TX_FULL: AtomicU32 = AtomicU32::new(0);
#[no_mangle]
pub static PROBE_TX_STARVED: AtomicU32 = AtomicU32::new(0);

/// Cycle-stamped events: code in bits 31:28, payload in 27:20, cycles in 19:0.
#[no_mangle]
pub static PROBE_EVT: [AtomicU32; 512] = [const { AtomicU32::new(0) }; 512];
#[no_mangle]
pub static PROBE_EVT_AT: AtomicU32 = AtomicU32::new(0);

pub fn evt(code: u32, payload: u32) {
    let c = now();
    let i = PROBE_EVT_AT.fetch_add(1, Ordering::Relaxed) as usize & 511;
    PROBE_EVT[i].store(
        (code << 28) | ((payload & 0xFF) << 20) | (c & 0x000F_FFFF),
        Ordering::Relaxed,
    );
}

pub fn intstat() -> u32 {
    // SAFETY: a plain volatile read of a memory-mapped status register.
    unsafe { core::ptr::read_volatile(0x4009_4020 as *const u32) }
}

pub fn start() {
    let mut core = unsafe { cortex_m::Peripherals::steal() };
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
}

pub fn now() -> u32 {
    // A debugger detach clears DEMCR.TRCENA and freezes CYCCNT; re-arm when it drops.
    const TRCENA: u32 = 1 << 24;
    let dcb = unsafe { &*cortex_m::peripheral::DCB::PTR };
    if dcb.demcr.read() & TRCENA == 0 {
        unsafe { dcb.demcr.modify(|d| d | TRCENA) };
    }
    cortex_m::peripheral::DWT::cycle_count()
}

pub fn cycles_since(start: u32) -> u32 {
    now().wrapping_sub(start)
}

pub fn max(cell: &AtomicU32, us: u32) {
    cell.fetch_max(us, Ordering::Relaxed);
}

pub fn bump(cell: &AtomicU32, by: u32) {
    cell.fetch_add(by, Ordering::Relaxed);
}
