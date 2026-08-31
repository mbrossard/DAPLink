//! Self-verifying bring-up of the LPC55 high-speed USB PHY: the APB block can come out of the
//! HAL's bring-up wedged, and [`verify_and_recover`] detects that and re-runs the PHY program.

use core::sync::atomic::{AtomicU32, Ordering};

use lpc55_hal as hal;

// `USBPHY_CTRL` bits: only the one-shot aliases; a read-modify-write of a live `CTRL` wedges it.
const CTRL_SFTRST: u32 = 1 << 31;
const CTRL_CLKGATE: u32 = 1 << 30;
const CTRL_ENAUTOCLR_CLKGATE: u32 = 1 << 19;

// `USBPHY_PLL_SIC` bits; bit 16 is reserved and the manual requires it written zero.
const PLL_SIC_LOCK: u32 = 1 << 31;
const PLL_SIC_DIV_SEL: u32 = 0x7 << 22;
const PLL_SIC_DIV_SEL_16MHZ: u32 = 6 << 22;
const PLL_SIC_REG_ENABLE: u32 = 1 << 21;
const PLL_SIC_RESERVED: u32 = 1 << 16;
const PLL_SIC_POWER: u32 = 1 << 12;
const PLL_SIC_EN_USB_CLKS: u32 = 1 << 6;

// `SYSCON.PRESETCTRL2.USB1_PHY_RST`, written through the one-shot SET and CLR aliases.
const USB1_PHY_RST: u32 = 1 << 7;

/// Settling delay in core cycles, above the PHY PLL regulator's 15 us minimum at any core clock.
const SETTLE_CYCLES: u32 = 8_000;

/// Iteration budget for [`wait_for_pll_lock`].
const PLL_LOCK_POLL_LIMIT: u32 = 100_000;

/// Recovery attempts before giving up.
const RETRY_LIMIT: u32 = 3;

/// Bring-up status word, readable over SWD by name. Bits 31:28 phase, 27:24 attempt (0 = the
/// HAL's own sequence), 23:20 the evidence bits, 15:0 the top half of `USBPHY_CTRL`.
#[no_mangle]
pub static USB_PHY_STATUS: AtomicU32 = AtomicU32::new(STATUS_START);

/// [`USB_PHY_STATUS`]: the check has not run yet.
pub const STATUS_START: u32 = 0x0000_0000;
/// [`USB_PHY_STATUS`]: latched immediately before the first look at the HAL's bring-up.
pub const STATUS_VERIFY: u32 = 0x1000_0000;
/// [`USB_PHY_STATUS`]: latched immediately before a `USB1_PHY_RST` toggle.
pub const STATUS_RECOVER: u32 = 0x2000_0000;
/// [`USB_PHY_STATUS`]: every attempt ran and none left the PHY healthy.
pub const STATUS_EXHAUSTED: u32 = 0x3000_0000;
/// [`USB_PHY_STATUS`]: the PHY responds, its PLL is locked, and the UTMI clock is running.
pub const STATUS_OK: u32 = 0x4000_0000;

/// [`USB_PHY_STATUS`] bit 23: the register block responds with distinct per-register values.
pub const STATUS_PHY_ALIVE: u32 = 1 << 23;
/// [`USB_PHY_STATUS`] bit 22: `USBPHY_PLL_SIC.PLL_LOCK` was set.
pub const STATUS_PLL_LOCK: u32 = 1 << 22;
/// [`USB_PHY_STATUS`] bit 21: `USBPHY_CTRL.CLKGATE` was clear (the UTMI clock is running).
pub const STATUS_PHY_UNGATED: u32 = 1 << 21;
/// [`USB_PHY_STATUS`] bit 20: `USBPHY_PWD` was zero, so the transceivers are powered.
pub const STATUS_PHY_POWERED: u32 = 1 << 20;

/// What "the PHY came up" means here. The `PWD` term is not redundant: it is what puts the pull-up
/// on the wire, so without it the other three hold on a probe the host cannot see.
const GOOD: u32 = STATUS_PHY_ALIVE | STATUS_PLL_LOCK | STATUS_PHY_UNGATED | STATUS_PHY_POWERED;

/// Verify the PHY the HAL just brought up and recover it where wedged; call once, right after
/// `enabled_as_device`.
pub(crate) fn verify_and_recover() {
    // SAFETY: a fixed peripheral block reached through the PAC pointer; runs single-threaded
    // during bring-up, before the USB bus exists.
    let phy = unsafe { &*hal::raw::USBPHY::PTR };

    USB_PHY_STATUS.store(STATUS_VERIFY, Ordering::Relaxed);
    wait_for_pll_lock(phy);
    let ev = evidence(phy);
    if ev & GOOD == GOOD {
        USB_PHY_STATUS.store(STATUS_OK | ev, Ordering::Relaxed);
        return;
    }
    recover(phy, ev);
}

/// Reset and re-program the PHY until healthy, [`RETRY_LIMIT`] attempts at most.
#[cold]
#[inline(never)]
fn recover(phy: &hal::raw::usbphy::RegisterBlock, mut ev: u32) {
    // SAFETY: as in `verify_and_recover`; nothing else writes SYSCON during bring-up.
    let sc = unsafe { &*hal::raw::SYSCON::PTR };

    for attempt in 1..=RETRY_LIMIT {
        USB_PHY_STATUS.store(STATUS_RECOVER | (attempt << 24) | ev, Ordering::Relaxed);
        ev = reset_and_program(phy, sc);
        if ev & GOOD == GOOD {
            USB_PHY_STATUS.store(STATUS_OK | (attempt << 24) | ev, Ordering::Relaxed);
            return;
        }
    }

    USB_PHY_STATUS.store(
        STATUS_EXHAUSTED | (RETRY_LIMIT << 24) | ev,
        Ordering::Relaxed,
    );
}

/// Snapshot the PHY as the evidence bits plus the top half of `USBPHY_CTRL`. `PWD` and `CTRL`
/// differ on a live block, so reading them equal marks it wedged.
fn evidence(phy: &hal::raw::usbphy::RegisterBlock) -> u32 {
    let ctrl = phy.ctrl.read().bits();
    let pwd = phy.pwd.read().bits();
    let alive = if pwd == ctrl { 0 } else { STATUS_PHY_ALIVE };
    let powered = if pwd == 0 { STATUS_PHY_POWERED } else { 0 };
    // `PLL_SIC.PLL_LOCK` is bit 31 and `STATUS_PLL_LOCK` bit 22, hence the nine-bit shift.
    let lock = (phy.pll_sic.read().bits() >> 9) & STATUS_PLL_LOCK;
    let ungated = if ctrl & CTRL_CLKGATE == 0 {
        STATUS_PHY_UNGATED
    } else {
        0
    };
    alive | lock | ungated | powered | (ctrl >> 16)
}

/// One recovery attempt: reset the APB registers, re-run the manual's register program in its
/// order, wait for lock, and return the resulting [`evidence`].
#[inline(never)]
fn reset_and_program(
    phy: &hal::raw::usbphy::RegisterBlock,
    sc: &hal::raw::syscon::RegisterBlock,
) -> u32 {
    // Reset with clock and power left on: the one knob that brings a wedged block back.
    sc.presetctrlset[2].write(|w| unsafe { w.bits(USB1_PHY_RST) });
    sc.presetctrlclr[2].write(|w| unsafe { w.bits(USB1_PHY_RST) });
    cortex_m::asm::delay(SETTLE_CYCLES);

    phy.ctrl_clr.write(|w| unsafe { w.bits(CTRL_SFTRST) });

    // `PLL_DIV_SEL` is a multi-bit field with no alias, so it alone stays a read-modify-write.
    phy.pll_sic
        .modify(|r, w| unsafe { w.bits((r.bits() & !PLL_SIC_DIV_SEL) | PLL_SIC_DIV_SEL_16MHZ) });
    phy.pll_sic_set
        .write(|w| unsafe { w.bits(PLL_SIC_REG_ENABLE) });
    phy.pll_sic_clr
        .write(|w| unsafe { w.bits(PLL_SIC_RESERVED) });
    cortex_m::asm::delay(SETTLE_CYCLES);

    phy.pll_sic_set.write(|w| unsafe { w.bits(PLL_SIC_POWER) });
    phy.pll_sic_set
        .write(|w| unsafe { w.bits(PLL_SIC_EN_USB_CLKS) });
    // Ungate last, then power the transceivers before arming the clock-gate auto-clear.
    phy.ctrl_clr.write(|w| unsafe { w.bits(CTRL_CLKGATE) });
    phy.pwd.write(|w| unsafe { w.bits(0) });
    phy.ctrl_set
        .write(|w| unsafe { w.bits(CTRL_ENAUTOCLR_CLKGATE) });

    wait_for_pll_lock(phy);
    evidence(phy)
}

/// Poll, boundedly, for `PLL_SIC.PLL_LOCK`; the bound turns a lock that never arrives into a
/// report rather than a hang.
fn wait_for_pll_lock(phy: &hal::raw::usbphy::RegisterBlock) {
    let mut budget = PLL_LOCK_POLL_LIMIT;
    while budget > 0 && phy.pll_sic.read().bits() & PLL_SIC_LOCK == 0 {
        budget -= 1;
    }
}
