//! Instruction-cache bring-up, and the H5 MPU region its enable depends on.

use cortex_m::asm::{dsb, isb};

/// The factory OTP and read-only flash window.
const FACTORY_RO_BASE: u32 = 0x08FF_F000;
const FACTORY_RO_LAST: u32 = 0x08FF_FFFF;

const _: () = assert!(
    FACTORY_RO_BASE.is_multiple_of(32) && (FACTORY_RO_LAST + 1).is_multiple_of(32),
    "ARMv8-M MPU regions are 32-byte granular"
);

/// MPU region and MAIR index for the factory window; nothing else programs the MPU.
const FACTORY_RO_REGION: u32 = 0;
const FACTORY_RO_ATTR_INDEX: u32 = 0;

/// `MAIR` byte: Normal memory, outer and inner non-cacheable.
const MAIR_NORMAL_NON_CACHEABLE: u32 = 0x44;
/// `MPU_RBAR.AP`: read-only at any privilege level.
const RBAR_AP_READ_ONLY: u32 = 0b11 << 1;
const RBAR_XN: u32 = 0x1;
const RLAR_ENABLE: u32 = 0x1;
const MPU_CTRL_ENABLE: u32 = 0x1;
/// `MPU_CTRL.PRIVDEFENA`: the default memory map outside programmed regions.
const MPU_CTRL_PRIVDEFENA: u32 = 0x4;

// Raw addresses: the typed cortex-m block compiles these fields only for ARMv8-M.
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RLAR: *mut u32 = 0xE000_EDA0 as *mut u32;
const MPU_MAIR0: *mut u32 = 0xE000_EDC0 as *mut u32;

const BUSYF_POLL_LIMIT: u32 = 100_000;

const ICACHE_CR: *mut u32 = 0x4003_0400 as *mut u32;
const ICACHE_SR: *const u32 = 0x4003_0404 as *const u32;
const CR_EN: u32 = 0x1;
/// `ICACHE_SR.BUSYF`; `BSYENDF` (bit 1) latches and never clears itself.
const SR_BUSYF: u32 = 0x1;

/// Outcome of [`enable_instruction_cache`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IcacheEnable {
    /// Invalidation finished and `CR.EN` read back set.
    Enabled,
    /// The `BUSYF` poll timed out; `EN` was set anyway.
    BusyTimeout {
        /// Last `ICACHE_SR` sampled.
        sr: u32,
    },
    NotEnabled {
        /// `ICACHE_CR` as read back.
        cr: u32,
    },
}

impl IcacheEnable {
    /// A short description for a bring-up log.
    pub fn describe(&self) -> &'static str {
        match self {
            IcacheEnable::Enabled => "enabled",
            IcacheEnable::BusyTimeout { .. } => "BUSYF poll timed out, enabled anyway",
            IcacheEnable::NotEnabled { .. } => "CR.EN did not read back set",
        }
    }

    /// The register word explaining the outcome; zero for `Enabled`.
    pub fn detail(&self) -> u32 {
        match *self {
            IcacheEnable::Enabled => 0,
            IcacheEnable::BusyTimeout { sr } => sr,
            IcacheEnable::NotEnabled { cr } => cr,
        }
    }
}

/// Mark the factory window non-cacheable and enable the MPU; call before touching that window.
pub fn configure_factory_ro_region() {
    let rbar = (FACTORY_RO_BASE & !0x1F) | RBAR_AP_READ_ONLY | RBAR_XN;
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

/// Enable the instruction cache; on the H5, call after [`configure_factory_ro_region`].
#[must_use]
pub fn enable_instruction_cache() -> IcacheEnable {
    let mut spins = 0u32;
    // SAFETY (every access below): architectural ICACHE addresses; the RMW keeps the way field.
    while unsafe { core::ptr::read_volatile(ICACHE_SR) } & SR_BUSYF != 0 {
        spins += 1;
        if spins >= BUSYF_POLL_LIMIT {
            let sr = unsafe { core::ptr::read_volatile(ICACHE_SR) };
            // Enabling during an invalidation is not an error; spinning forever would be.
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
