//! Instruction-cache bring-up (ART ICACHE), and the H5 MPU region its enable depends on.

use cortex_m::asm::{dsb, isb};

/// The factory OTP + read-only flash window: one 4 KiB region.
const FACTORY_RO_BASE: u32 = 0x08FF_F000;
const FACTORY_RO_LAST: u32 = 0x08FF_FFFF;

const _: () = assert!(
    FACTORY_RO_BASE.is_multiple_of(32) && (FACTORY_RO_LAST + 1).is_multiple_of(32),
    "ARMv8-M MPU regions are 32-byte granular"
);

/// MPU region and MAIR attribute index used for the factory window; nothing else programs the MPU.
const FACTORY_RO_REGION: u32 = 0;
const FACTORY_RO_ATTR_INDEX: u32 = 0;

/// `MAIR` byte: Normal memory, outer and inner non-cacheable.
const MAIR_NORMAL_NON_CACHEABLE: u32 = 0x44;
/// `MPU_RBAR.AP` = read-only at any privilege level (the window is factory read-only anyway).
const RBAR_AP_READ_ONLY: u32 = 0b11 << 1;
/// `MPU_RBAR.XN`: nothing is ever fetched from the factory window.
const RBAR_XN: u32 = 0x1;
const RLAR_ENABLE: u32 = 0x1;
const MPU_CTRL_ENABLE: u32 = 0x1;
/// `MPU_CTRL.PRIVDEFENA`: keep the default memory map in force outside the programmed region.
const MPU_CTRL_PRIVDEFENA: u32 = 0x4;

// Raw addresses: the typed cortex-m block compiles these fields only for ARMv8-M.
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RLAR: *mut u32 = 0xE000_EDA0 as *mut u32;
const MPU_MAIR0: *mut u32 = 0xE000_EDC0 as *mut u32;

/// Iterations of the `ICACHE_SR.BUSYF` poll before the enable gives up waiting.
const BUSYF_POLL_LIMIT: u32 = 100_000;

// The ICACHE block sits at the same address on every part served here.
const ICACHE_CR: *mut u32 = 0x4003_0400 as *mut u32;
const ICACHE_SR: *const u32 = 0x4003_0404 as *const u32;
/// `ICACHE_CR.EN`.
const CR_EN: u32 = 0x1;
/// `ICACHE_SR.BUSYF`; bit 1 is `BSYENDF`, which latches on completion and never clears itself.
const SR_BUSYF: u32 = 0x1;

/// Outcome of [`enable_instruction_cache`], reported so a cache that fails to come up is visible.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum IcacheEnable {
    /// Any pending invalidation finished within the budget and `CR.EN` read back set.
    Enabled,
    /// The `BUSYF` poll timed out; `EN` was set anyway (enabling while busy is not an error).
    BusyTimeout {
        /// Last `ICACHE_SR` sampled by the poll.
        sr: u32,
    },
    /// `CR.EN` did not read back as set after the write.
    NotEnabled {
        /// `ICACHE_CR` as read back after the enabling write.
        cr: u32,
    },
}

#[allow(dead_code)]
impl IcacheEnable {
    /// A short static description, for a firmware's bring-up log.
    pub fn describe(&self) -> &'static str {
        match self {
            IcacheEnable::Enabled => "enabled",
            IcacheEnable::BusyTimeout { .. } => "BUSYF poll timed out, enabled anyway",
            IcacheEnable::NotEnabled { .. } => "CR.EN did not read back set",
        }
    }

    /// The register word that explains the outcome; zero when there is nothing to explain.
    pub fn detail(&self) -> u32 {
        match *self {
            IcacheEnable::Enabled => 0,
            IcacheEnable::BusyTimeout { sr } => sr,
            IcacheEnable::NotEnabled { cr } => cr,
        }
    }
}

/// Mark the factory read-only window non-cacheable and enable the MPU, default map retained.
/// Call before [`enable_instruction_cache`] and before any access to that window.
pub fn configure_factory_ro_region() {
    // RBAR: BASE[31:5] | SH[4:3] (non-shareable, 0) | AP[2:1] | XN[0].
    let rbar = (FACTORY_RO_BASE & !0x1F) | RBAR_AP_READ_ONLY | RBAR_XN;
    // RLAR: LIMIT[31:5] (the region's LAST byte) | AttrIndx[3:1] | EN[0].
    let rlar = (FACTORY_RO_LAST & !0x1F) | (FACTORY_RO_ATTR_INDEX << 1) | RLAR_ENABLE;

    // SAFETY: architectural MPU addresses, programmed with the MPU off before any task exists.
    unsafe {
        core::ptr::write_volatile(MPU_CTRL, 0);
        dsb();
        core::ptr::write_volatile(MPU_MAIR0, MAIR_NORMAL_NON_CACHEABLE);
        core::ptr::write_volatile(MPU_RNR, FACTORY_RO_REGION);
        core::ptr::write_volatile(MPU_RBAR, rbar);
        core::ptr::write_volatile(MPU_RLAR, rlar);
        core::ptr::write_volatile(MPU_CTRL, MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA);
    }

    // Complete the writes, then discard anything fetched under the old attributes.
    dsb();
    isb();
}

/// Enable the instruction cache, waiting out (bounded) the reset-launched invalidation. Call
/// after `embassy_stm32::init`, and on the H5 after [`configure_factory_ro_region`].
#[must_use]
pub fn enable_instruction_cache() -> IcacheEnable {
    let mut spins = 0u32;
    // SAFETY (every access below): architectural ICACHE addresses; the RMW keeps the way field.
    while unsafe { core::ptr::read_volatile(ICACHE_SR) } & SR_BUSYF != 0 {
        spins += 1;
        if spins >= BUSYF_POLL_LIMIT {
            let sr = unsafe { core::ptr::read_volatile(ICACHE_SR) };
            // Enabling during an invalidation is not an error; spinning forever is a dead board.
            unsafe {
                let cr = core::ptr::read_volatile(ICACHE_CR);
                core::ptr::write_volatile(ICACHE_CR, cr | CR_EN);
            }
            return IcacheEnable::BusyTimeout { sr };
        }
    }

    unsafe {
        let cr = core::ptr::read_volatile(ICACHE_CR);
        core::ptr::write_volatile(ICACHE_CR, cr | CR_EN);
    }

    let cr = unsafe { core::ptr::read_volatile(ICACHE_CR) };
    if cr & CR_EN != 0 {
        IcacheEnable::Enabled
    } else {
        IcacheEnable::NotEnabled { cr }
    }
}
