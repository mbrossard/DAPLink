//! Supply/flash-timing preflight, PLL0 lock, and mux switch for the 150 MHz core clock.

use core::sync::atomic::{AtomicU32, Ordering};

use lpc55_hal as hal;

// NXP's power library, linked by the firmware: sets the supply from undocumented factory data.
extern "C" {
    fn POWER_SetVoltageForFreq(system_freq_hz: u32);
}

/// `SYSCON.DIEID.REV_ID` of revision-1B silicon, the only revision UM11126 rates for 150 MHz.
const REV_ID_1B: u8 = 1;

/// `FLASHTIM` encoding for the 130-150 MHz band (12 system-clock flash access cycles).
const FLASHTIM_150MHZ: u8 = 11;

/// Flash controller command: set the internal read mode from `DATAW[0]`.
const FLASH_CMD_SET_READ_MODE: u32 = 2;

/// Every `FLASH.INT_STATUS` flag, for the clear-before-command write.
const FLASH_INT_STATUS_ALL: u32 = 0x1f;

/// `FLASH.INT_STATUS` bits: `FAIL`, `ERR`, `DONE`, `ECC_ERR`.
const FLASH_STATUS_FAIL: u32 = 1 << 0;
const FLASH_STATUS_ERR: u32 = 1 << 1;
const FLASH_STATUS_DONE: u32 = 1 << 2;
const FLASH_STATUS_ECC_ERR: u32 = 1 << 3;

/// Poll budget for flash `SET_READ_MODE`: bounded, so a wedged controller faults, not hangs.
const FLASH_CMD_POLL_LIMIT: u32 = 100_000;

/// `SYSCON.PLL0STAT`: the `LOCK` bit.
const PLL0STAT_LOCK: u32 = 1 << 0;

/// Poll budget for `PLL0STAT.LOCK`: far past the specified settling time, and still bounded.
const PLL0_LOCK_POLL_LIMIT: u32 = 1_000_000;

/// PLL0 recipe for 150 MHz from the 12 MHz FRO; [`latch_readback`] re-reads it and flags drift.
const PLL0_150MHZ_N: u8 = 8;
const PLL0_150MHZ_M: u16 = 200;
const PLL0_150MHZ_P: u8 = 1;
const PLL0_150MHZ_SELP: u8 = 31;
const PLL0_150MHZ_SELI: u8 = 53;

/// Why the 150 MHz path refused. Every variant leaves the mux untouched: the core is still on
/// FRO12, running and debuggable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerError {
    /// `SYSCON.DIEID.REV_ID` (carried) is not [`REV_ID_1B`]; the die is not rated for 150 MHz.
    UnsupportedRevision(u8),
    /// The flash `SET_READ_MODE` command failed; carries the raw `FLASH.INT_STATUS`.
    FlashReadMode(u32),
    /// The flash `SET_READ_MODE` command never raised `DONE` within the poll budget.
    FlashTimeout,
    /// PLL0 never raised `LOCK` within the poll budget; carries the raw `SYSCON.PLL0STAT`.
    Pll0NoLock(u32),
}

/// Bring-up status word for the 150 MHz path, readable over SWD by symbol name. Bits 31:24 name
/// the step (the `STATUS_*` constants); the low bits carry the raw register value.
#[no_mangle]
pub static CORE_150MHZ_STATUS: AtomicU32 = AtomicU32::new(STATUS_START);

/// Clock readback for the 150 MHz path. Word 0: post-switch summary; words 1-4: PLL0 registers
/// as programmed; words 5-6: the pre-mux snapshot.
#[no_mangle]
pub static CORE_150MHZ_READBACK: [AtomicU32; 7] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// [`CORE_150MHZ_READBACK`] word 0, bit 25: the PLL0 registers read back after the switch
/// differ from the ones [`prelock_pll0_150mhz`] programmed.
pub const READBACK_PLL_RECIPE_DRIFT: u32 = 1 << 25;

/// [`CORE_150MHZ_STATUS`]: the preflight has not run yet.
pub const STATUS_START: u32 = 0;
/// [`CORE_150MHZ_STATUS`]: the die is not revision 1B; the low nibble carries `REV_ID`.
pub const STATUS_BAD_REVISION: u32 = 0x1000_0000;
/// [`CORE_150MHZ_STATUS`]: flash `SET_READ_MODE` failed; low bits carry `FLASH.INT_STATUS`.
pub const STATUS_FLASH_FAILED: u32 = 0x2000_0000;
/// [`CORE_150MHZ_STATUS`]: flash `SET_READ_MODE` never completed.
pub const STATUS_FLASH_TIMEOUT: u32 = 0x3000_0000;
/// [`CORE_150MHZ_STATUS`]: supply and both flash-timing controls set; PLL0 untouched.
pub const STATUS_PREFLIGHT_OK: u32 = 0x4000_0000;
/// [`CORE_150MHZ_STATUS`]: latched immediately before the first PLL0 register write.
pub const STATUS_PLL_PROGRAM: u32 = 0x4100_0000;
/// [`CORE_150MHZ_STATUS`]: PLL0 programmed and powered; the bounded lock wait is next.
pub const STATUS_PLL_PROGRAMMED: u32 = 0x4200_0000;
/// [`CORE_150MHZ_STATUS`]: PLL0 never locked; low byte carries `PLL0STAT`. The mux never moved.
pub const STATUS_PLL_LOCK_TIMEOUT: u32 = 0x4300_0000;
/// [`CORE_150MHZ_STATUS`]: PLL0 reported `LOCK`; low byte carries `PLL0STAT`.
pub const STATUS_PLL_LOCKED: u32 = 0x4400_0000;
/// [`CORE_150MHZ_STATUS`]: latched immediately before the mux moves onto the known-locked PLL0.
pub const STATUS_MUX_SWITCH: u32 = 0x4500_0000;
/// [`CORE_150MHZ_STATUS`]: the main clock is on PLL0 and the core survived the switch.
pub const STATUS_RUNNING: u32 = 0x5000_0000;

/// Raise the core supply and both flash-timing controls for 150 MHz, in UM11126's
/// upward-transition order, re-selecting the reset FRO12 clock first. Refuses non-1B silicon.
pub fn preflight_150mhz(
    flash: &hal::raw::FLASH,
    syscon: &mut hal::Syscon,
) -> Result<(), PowerError> {
    let rev = syscon.rev_id();
    if rev != REV_ID_1B {
        CORE_150MHZ_STATUS.store(STATUS_BAD_REVISION | u32::from(rev), Ordering::Relaxed);
        return Err(PowerError::UnsupportedRevision(rev));
    }

    // SAFETY: the `&mut Syscon` proves no other owner writes the block; the HAL keeps its raw
    // field crate-private, so the registers are reached through the PAC block pointer.
    let sc = unsafe { &*hal::raw::SYSCON::PTR };

    // Guarantee the starting point the upward sequence assumes: main clock on FRO12, AHB /1.
    sc.mainclksela.modify(|_, w| w.sel().enum_0x0());
    sc.mainclkselb.modify(|_, w| w.sel().enum_0x0());
    sc.ahbclkdiv.modify(|_, w| unsafe { w.div().bits(0) });

    // SAFETY: the linked routine takes one `u32` and touches only the PMC and read-only factory
    // flash, which no Rust code in this image aliases; called on FRO12 as UM11126 requires.
    unsafe { POWER_SetVoltageForFreq(crate::clocks::CPU_HZ) };

    set_flash_access_cycles(flash, sc, FLASHTIM_150MHZ)?;

    CORE_150MHZ_STATUS.store(STATUS_PREFLIGHT_OK, Ordering::Relaxed);
    Ok(())
}

/// Program the flash controller's internal read mode, then `SYSCON.FMCCR.FLASHTIM`, to the same
/// encoding: both must be raised before the clock, and `FMCCR` alone can fault the first fetch.
fn set_flash_access_cycles(
    flash: &hal::raw::FLASH,
    syscon: &hal::raw::syscon::RegisterBlock,
    wait_states: u8,
) -> Result<(), PowerError> {
    // Clear every status flag so the DONE polled for below belongs to this command.
    flash
        .int_clr_status
        .write(|w| unsafe { w.bits(FLASH_INT_STATUS_ALL) });

    // The command parameter lives in the low nibble of DATAW[0]; the rest is preserved.
    let dataw0 = flash.dataw[0].read().bits();
    flash.dataw[0].write(|w| unsafe { w.bits((dataw0 & !0xf) | u32::from(wait_states & 0xf)) });

    flash
        .cmd
        .write(|w| unsafe { w.bits(FLASH_CMD_SET_READ_MODE) });

    let mut status = 0;
    let mut budget = FLASH_CMD_POLL_LIMIT;
    while budget > 0 {
        status = flash.int_status.read().bits();
        if status & FLASH_STATUS_DONE != 0 {
            break;
        }
        budget -= 1;
    }
    if status & FLASH_STATUS_DONE == 0 {
        CORE_150MHZ_STATUS.store(STATUS_FLASH_TIMEOUT, Ordering::Relaxed);
        return Err(PowerError::FlashTimeout);
    }
    if status & (FLASH_STATUS_FAIL | FLASH_STATUS_ERR | FLASH_STATUS_ECC_ERR) != 0 {
        CORE_150MHZ_STATUS.store(STATUS_FLASH_FAILED | status, Ordering::Relaxed);
        return Err(PowerError::FlashReadMode(status));
    }

    syscon
        .fmccr
        .modify(|_, w| unsafe { w.flashtim().bits(wait_states) });
    Ok(())
}

/// Program PLL0 with the 150 MHz recipe and wait, bounded, for `PLL0STAT.LOCK` without moving
/// the mux, so a recipe that cannot lock leaves the core on FRO12 rather than on a dead clock.
pub(crate) fn prelock_pll0_150mhz() -> Result<(), PowerError> {
    // SAFETY: as in `preflight_150mhz`; the caller holds the Syscon and Pmc singletons across it.
    let sc = unsafe { &*hal::raw::SYSCON::PTR };
    let pmc = unsafe { &*hal::raw::PMC::PTR };

    CORE_150MHZ_STATUS.store(STATUS_PLL_PROGRAM, Ordering::Relaxed);

    sc.pll0clksel.write(|w| w.sel().enum_0x0()); // FRO12 input

    pmc.pdruncfg0
        .modify(|_, w| w.pden_pll0().poweredoff().pden_pll0_sscg().poweredoff());

    sc.pll0ctrl.write(|w| unsafe {
        w.clken()
            .enable()
            .seli()
            .bits(PLL0_150MHZ_SELI)
            .selp()
            .bits(PLL0_150MHZ_SELP)
    });

    // N, P and M dividers each latch on a second write with the request bit set.
    sc.pll0ndec
        .write(|w| unsafe { w.ndiv().bits(PLL0_150MHZ_N) });
    sc.pll0ndec
        .write(|w| unsafe { w.ndiv().bits(PLL0_150MHZ_N).nreq().set_bit() });

    sc.pll0pdec
        .write(|w| unsafe { w.pdiv().bits(PLL0_150MHZ_P) });
    sc.pll0pdec
        .write(|w| unsafe { w.pdiv().bits(PLL0_150MHZ_P).preq().set_bit() });

    sc.pll0sscg0.write(|w| unsafe { w.md_lbs().bits(0) });

    sc.pll0sscg1
        .write(|w| unsafe { w.mdiv_ext().bits(PLL0_150MHZ_M).sel_ext().set_bit() });
    sc.pll0sscg1.write(|w| unsafe {
        w.mdiv_ext()
            .bits(PLL0_150MHZ_M)
            .sel_ext()
            .set_bit()
            .mreq()
            .set_bit()
            .md_req()
            .set_bit()
    });

    pmc.pdruncfg0
        .modify(|_, w| w.pden_pll0().poweredon().pden_pll0_sscg().poweredon());

    CORE_150MHZ_STATUS.store(STATUS_PLL_PROGRAMMED, Ordering::Relaxed);

    let mut stat = 0;
    let mut budget = PLL0_LOCK_POLL_LIMIT;
    while budget > 0 {
        stat = sc.pll0stat.read().bits();
        if stat & PLL0STAT_LOCK != 0 {
            break;
        }
        budget -= 1;
    }

    latch_pll0_registers(sc, pmc);

    if stat & PLL0STAT_LOCK == 0 {
        CORE_150MHZ_STATUS.store(STATUS_PLL_LOCK_TIMEOUT | (stat & 0xff), Ordering::Relaxed);
        return Err(PowerError::Pll0NoLock(stat));
    }

    CORE_150MHZ_STATUS.store(STATUS_PLL_LOCKED | (stat & 0xff), Ordering::Relaxed);
    Ok(())
}

/// Snapshot the PLL0 registers into [`CORE_150MHZ_READBACK`] words 1-4 and return the three
/// recipe words (status excluded so post-switch comparison ignores `PLL0STAT` churn).
fn latch_pll0_registers(
    sc: &hal::raw::syscon::RegisterBlock,
    pmc: &hal::raw::pmc::RegisterBlock,
) -> [u32; 3] {
    let recipe = [
        sc.pll0ctrl.read().bits(),
        (sc.pll0ndec.read().bits() & 0xffff) | (sc.pll0pdec.read().bits() << 16),
        sc.pll0sscg1.read().bits(),
    ];
    let pdruncfg0 = pmc.pdruncfg0.read();
    let status = (sc.pll0stat.read().bits() & 0xff)
        | ((sc.pll0clksel.read().bits() & 0xf) << 8)
        | (u32::from(pdruncfg0.pden_pll0().bit()) << 16)
        | (u32::from(pdruncfg0.pden_pll0_sscg().bit()) << 17);

    for (slot, word) in CORE_150MHZ_READBACK[1..].iter().zip(recipe) {
        slot.store(word, Ordering::Relaxed);
    }
    CORE_150MHZ_READBACK[4].store(status, Ordering::Relaxed);
    recipe
}

/// Latch [`STATUS_MUX_SWITCH`] and the pre-mux clock-tree snapshot (readback words 5-6): the only
/// record a boot that dies at the switch leaves.
pub(crate) fn latch_mux_switch() {
    CORE_150MHZ_STATUS.store(STATUS_MUX_SWITCH, Ordering::Relaxed);

    // SAFETY: a read-only snapshot from the single-threaded bring-up path that owns the blocks.
    let sc = unsafe { &*hal::raw::SYSCON::PTR };
    let pmc = unsafe { &*hal::raw::PMC::PTR };
    let pdruncfg0 = pmc.pdruncfg0.read();

    let mux = (sc.mainclksela.read().bits() & 0x3)
        | ((sc.mainclkselb.read().bits() & 0x3) << 4)
        | ((sc.pll0stat.read().bits() & 0xff) << 8)
        | ((sc.pll0clksel.read().bits() & 0xf) << 16)
        | (u32::from(pdruncfg0.pden_pll0().bit()) << 24)
        | (u32::from(pdruncfg0.pden_pll0_sscg().bit()) << 25)
        | ((sc.fmccr.read().flashtim().bits() as u32 & 0xf) << 28);

    CORE_150MHZ_READBACK[5].store(mux, Ordering::Relaxed);
    CORE_150MHZ_READBACK[6].store(sc.pll0ctrl.read().bits(), Ordering::Relaxed);
}

/// Move the main clock mux onto PLL0 with AHB /1; the A-side selector is pointed at FRO12 first so
/// the mux has a live fallback source. Call only after a successful [`prelock_pll0_150mhz`].
pub(crate) fn switch_main_clock_to_pll0() {
    // SAFETY: as in `preflight_150mhz`; the caller holds the Syscon singleton across this call.
    let sc = unsafe { &*hal::raw::SYSCON::PTR };

    sc.mainclksela.modify(|_, w| w.sel().enum_0x0());
    sc.mainclkselb.modify(|_, w| w.sel().enum_0x1());
    sc.ahbclkdiv.modify(|_, w| unsafe { w.div().bits(0) });
}

/// Latch the post-switch summary into [`CORE_150MHZ_READBACK`] word 0, separating "running at
/// 150 MHz" from "still on FRO12", and flag [`READBACK_PLL_RECIPE_DRIFT`] on a recipe mismatch.
pub(crate) fn latch_readback() {
    // SAFETY: as in `latch_mux_switch` -- a snapshot from the owning bring-up path.
    let sc = unsafe { &*hal::raw::SYSCON::PTR };
    let sel = u32::from(sc.mainclkselb.read().sel().bits());
    let div = u32::from(sc.ahbclkdiv.read().div().bits());
    let flashtim = u32::from(sc.fmccr.read().flashtim().bits());
    let lock = u32::from(sc.pll0stat.read().lock().bit());

    // SAFETY: as above; the caller still holds the Pmc singleton.
    let pmc = unsafe { &*hal::raw::PMC::PTR };
    let programmed: [u32; 3] =
        core::array::from_fn(|i| CORE_150MHZ_READBACK[i + 1].load(Ordering::Relaxed));
    // Re-latching the post-switch values is deliberate: they are what the part actually runs on.
    let drift = if latch_pll0_registers(sc, pmc) == programmed {
        0
    } else {
        READBACK_PLL_RECIPE_DRIFT
    };

    let readback = sel | (div << 8) | (flashtim << 16) | (lock << 24) | drift;
    CORE_150MHZ_READBACK[0].store(readback, Ordering::Relaxed);
    CORE_150MHZ_STATUS.store(STATUS_RUNNING, Ordering::Relaxed);
}
