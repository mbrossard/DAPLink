//! Flash-wins arbitration over one SWD/JTAG adapter: gated DAP handles, an ungated flash handle.

use core::cell::{Cell, UnsafeCell};
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;
use dap_rs::dap::{Request, ResponseWriter, TransferConfig};
use dap_rs::jtag::{self, Jtag, SequenceInfo, TapConfig, TransferResult};
use dap_rs::swd::{self, APnDP, DPRegister, RnW, Swd};
use dap_rs::swj::{Dependencies, Pins};

use daplink_core::probe::{FlashArbiter, ProbeInfo, ResetControl};

/// WAIT-retry budget for the cold-target presence read.
const WAIT_RETRIES: usize = 100;

/// Retry budget for a flash-side checkout blocked by an in-flight DAP op; finite so a flash path
/// that has itself preempted the holder on one core cannot deadlock.
const FLASH_BUSY_RETRIES: usize = 10_000;

/// The single adapter, held as exactly one of its three dap-rs roles.
enum Role<DEPS, SWD, JTAG> {
    Deps(DEPS),
    Swd(SWD),
    Jtag(JTAG),
    /// Occupies the slot for the few instructions of an in-place role conversion.
    Moving,
}

/// The arbitration state guarded by the mux's critical section. [`SwdMux::held`] deliberately
/// lives outside it, which is what lets a release be a lone store.
#[derive(Clone, Copy)]
struct MuxState {
    flash_active: bool,
}

/// Convert whatever role is present into `Dependencies`, the hub dap-rs routes conversions through.
fn role_to_deps<DEPS, SWD, JTAG>(role: Role<DEPS, SWD, JTAG>) -> DEPS
where
    DEPS: From<SWD> + From<JTAG>,
{
    match role {
        Role::Deps(deps) => deps,
        Role::Swd(swd) => DEPS::from(swd),
        Role::Jtag(jtag) => DEPS::from(jtag),
        Role::Moving => unreachable!("adapter role observed while in transit"),
    }
}

// The runners work the slot in place: the adapter moves only when the protocol actually changes.

/// Convert the slot to the wanted role if it is not already there, exactly once.
macro_rules! ensure_role {
    ($role:expr, $variant:path, $target:ty) => {
        if !matches!($role, $variant(_)) {
            // `Moving` first, so the slot never holds a logically-moved-from adapter.
            let owned = core::mem::replace($role, Role::Moving);
            *$role = $variant(<$target>::from(role_to_deps(owned)));
        }
    };
}

/// Run `f` with the adapter as `Dependencies`, converting only if needed.
fn run_as_deps<DEPS, SWD, JTAG, R>(
    role: &mut Role<DEPS, SWD, JTAG>,
    f: impl FnOnce(&mut DEPS) -> R,
) -> R
where
    DEPS: Dependencies<SWD, JTAG>,
    SWD: Swd<DEPS>,
    JTAG: Jtag<DEPS>,
{
    ensure_role!(role, Role::Deps, DEPS);
    match role {
        Role::Deps(deps) => f(deps),
        _ => unreachable!("slot was just placed in the Deps role"),
    }
}

/// Run `f` with the adapter as the SWD driver, converting only if needed.
fn run_as_swd<DEPS, SWD, JTAG, R>(
    role: &mut Role<DEPS, SWD, JTAG>,
    f: impl FnOnce(&mut SWD) -> R,
) -> R
where
    DEPS: Dependencies<SWD, JTAG>,
    SWD: Swd<DEPS>,
    JTAG: Jtag<DEPS>,
{
    ensure_role!(role, Role::Swd, SWD);
    match role {
        Role::Swd(swd) => f(swd),
        _ => unreachable!("slot was just placed in the Swd role"),
    }
}

/// Run `f` with the adapter as the JTAG driver, converting only if needed.
fn run_as_jtag<DEPS, SWD, JTAG, R>(
    role: &mut Role<DEPS, SWD, JTAG>,
    f: impl FnOnce(&mut JTAG) -> R,
) -> R
where
    DEPS: Dependencies<SWD, JTAG>,
    SWD: Swd<DEPS>,
    JTAG: Jtag<DEPS>,
{
    ensure_role!(role, Role::Jtag, JTAG);
    match role {
        Role::Jtag(jtag) => f(jtag),
        _ => unreachable!("slot was just placed in the Jtag role"),
    }
}

/// Shared owner of one SWD/JTAG adapter, arbitrating DAP against flash with a flash-wins policy.
/// The adapter is borrowed for a bus op, and [`Checkout`] carries the exclusion that makes it sound.
pub struct SwdMux<DEPS, SWD, JTAG> {
    state: Mutex<Cell<MuxState>>,
    /// Set while a bus op holds the exclusive `&mut Role` outside any critical section. Plain
    /// load/store only, sound on thumbv6m; the critical section supplies the read-modify-write.
    held: AtomicBool,
    /// The one adapter, touched only through a live [`Checkout`].
    role: UnsafeCell<Role<DEPS, SWD, JTAG>>,
    #[cfg(test)]
    counters: Counters,
}

/// Test-only accounting of what each claim cost: claims granted, and critical sections entered.
#[cfg(test)]
#[derive(Default)]
struct Counters {
    locked: core::sync::atomic::AtomicU32,
    sections: core::sync::atomic::AtomicU32,
}

#[cfg(test)]
impl Counters {
    const fn new() -> Self {
        Self {
            locked: core::sync::atomic::AtomicU32::new(0),
            sections: core::sync::atomic::AtomicU32::new(0),
        }
    }
    fn bump_locked(&self) {
        self.locked
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
    fn bump_section(&self) {
        self.sections
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
    fn read(&self) -> u32 {
        self.locked.load(core::sync::atomic::Ordering::Relaxed)
    }
    fn read_sections(&self) -> u32 {
        self.sections.load(core::sync::atomic::Ordering::Relaxed)
    }
    fn reset(&self) {
        self.locked.store(0, core::sync::atomic::Ordering::Relaxed);
        self.sections
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }
}

// SAFETY: every path to `role` goes through `SwdMux::take`, which sets `held` inside a critical
// section and so admits at most one owning `Checkout` -- and thus at most one live `&mut Role` --
// across all cores, interrupts and tasks. Acquirers exclude each other, only the unique holder
// clears the flag, and the `Release`/`Acquire` pair publishes its adapter writes to the next one.
unsafe impl<DEPS: Send, SWD: Send, JTAG: Send> Sync for SwdMux<DEPS, SWD, JTAG> {}

/// A live, exclusive checkout of the adapter: the right to drive the bus until dropped. The guard
/// clears `held` on every exit path, so a bus op that errors cannot strand the adapter.
struct Checkout<'m, DEPS, SWD, JTAG> {
    mux: &'m SwdMux<DEPS, SWD, JTAG>,
}

impl<DEPS, SWD, JTAG> Checkout<'_, DEPS, SWD, JTAG> {
    /// The adapter slot, exclusively.
    fn role(&mut self) -> &mut Role<DEPS, SWD, JTAG> {
        // SAFETY: this `Checkout` exists, so `held` is set and no other one can be created until
        // it is dropped; `&mut self` makes this the only live borrow derived from it.
        unsafe { &mut *self.mux.role.get() }
    }
}

impl<DEPS, SWD, JTAG> Drop for Checkout<'_, DEPS, SWD, JTAG> {
    fn drop(&mut self) {
        self.mux.clear_held();
    }
}

impl<DEPS, SWD, JTAG> SwdMux<DEPS, SWD, JTAG> {
    /// Wrap a fresh adapter, in its `Dependencies` role with no flash session active.
    pub const fn new(adapter: DEPS) -> Self {
        Self {
            state: Mutex::new(Cell::new(MuxState {
                flash_active: false,
            })),
            held: AtomicBool::new(false),
            role: UnsafeCell::new(Role::Deps(adapter)),
            #[cfg(test)]
            counters: Counters::new(),
        }
    }

    /// The mux's one critical section: every read-modify-write of [`MuxState`] goes through here.
    fn with_state<R>(&self, f: impl FnOnce(&mut MuxState) -> R) -> R {
        critical_section::with(|cs| {
            #[cfg(test)]
            self.counters.bump_section();
            let cell = self.state.borrow(cs);
            let mut s = cell.get();
            let out = f(&mut s);
            cell.set(s);
            out
        })
    }

    /// Release the adapter: a lone store, because the holder doing it is unique.
    fn clear_held(&self) {
        self.held.store(false, Ordering::Release);
    }

    /// The one place `held` is set: test and claim collapsed into a single critical section, which
    /// stands in for the compare-and-swap the smallest supported parts do not have.
    fn take(
        &self,
        admit: impl FnOnce(&mut MuxState) -> bool,
    ) -> Option<Checkout<'_, DEPS, SWD, JTAG>> {
        self.with_state(|s| {
            let admitted = admit(s);
            // Inside the critical section this load/store pair is an effective test-and-set.
            let ok = admitted && !self.held.load(Ordering::Acquire);
            if ok {
                self.held.store(true, Ordering::Relaxed);
            }
            #[cfg(test)]
            if ok {
                self.counters.bump_locked();
            }
            // `then`, not `then_some`: a guard built on the refused path would clear the holder's.
            ok.then(|| Checkout { mux: self })
        })
    }

    /// Single ungated checkout probe; `None` means another bus op holds the adapter.
    fn checkout(&self) -> Option<Checkout<'_, DEPS, SWD, JTAG>> {
        self.take(|_| true)
    }

    /// Flash-side checkout: wait out an in-flight DAP op within [`FLASH_BUSY_RETRIES`].
    fn checkout_flash(&self) -> Option<Checkout<'_, DEPS, SWD, JTAG>> {
        self.checkout_flash_with(FLASH_BUSY_RETRIES, core::hint::spin_loop)
    }

    /// [`checkout_flash`](Self::checkout_flash) with budget and relax hook injected for tests.
    fn checkout_flash_with(
        &self,
        retries: usize,
        mut relax: impl FnMut(),
    ) -> Option<Checkout<'_, DEPS, SWD, JTAG>> {
        for _ in 0..retries {
            if let Some(co) = self.checkout() {
                return Some(co);
            }
            relax();
        }
        self.checkout()
    }

    /// Gated checkout (DAP side): also refuses while a flash session is live.
    fn checkout_gated(&self) -> Option<Checkout<'_, DEPS, SWD, JTAG>> {
        self.take(|s| !s.flash_active)
    }

    /// Claims granted so far.
    #[cfg(test)]
    fn claims(&self) -> u32 {
        self.counters.read()
    }

    /// Critical sections entered on the checkout path; one per attempted claim, none per release.
    #[cfg(test)]
    fn sections(&self) -> u32 {
        self.counters.read_sections()
    }

    /// Zero the counters, so a test measures one command rather than a whole fixture.
    #[cfg(test)]
    fn reset_claims(&self) {
        self.counters.reset()
    }
}

impl<DEPS, SWD, JTAG> SwdMux<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    /// Sample a static property of the composed adapter -- what it is, not what it is doing.
    /// Callers are init-time only, so nothing else holds the slot and the checkout cannot fail.
    fn inspect_deps<R>(&self, f: impl FnOnce(&DEPS) -> R) -> R {
        let mut checkout = self
            .take(|_| true)
            .expect("mux checked out during init-time inspection");
        let slot = checkout.role();
        let deps = role_to_deps(core::mem::replace(slot, Role::Moving));
        let out = f(&deps);
        *slot = Role::Deps(deps);
        out
    }

    /// Build the DAP engine's `Dependencies` handle over `scan_chain`. Availability is sampled
    /// once, here, so it describes the composition and not whoever happens to hold the wire.
    pub fn dap_deps(
        &'static self,
        scan_chain: &'static mut [TapConfig],
    ) -> DapDeps<DEPS, SWD, JTAG> {
        let (swd_available, jtag_available) =
            self.inspect_deps(|deps| (SWD::available(deps), JTAG::available(deps)));
        DapDeps {
            mux: self,
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_chain),
            swd_available,
            jtag_available,
        }
    }

    /// Whether a flash session currently holds the shared bus.
    pub fn is_flashing(&self) -> bool {
        self.with_state(|s| s.flash_active)
    }

    /// Begin an exclusive flash session, or `None` if one is already live.
    pub fn begin_flash(&'static self) -> Option<FlashSession<DEPS, SWD, JTAG>> {
        self.with_state(|s| {
            if s.flash_active {
                None
            } else {
                s.flash_active = true;
                #[cfg(feature = "defmt")]
                defmt::info!("flash session: begin");
                Some(FlashSession { mux: self })
            }
        })
    }

    // Bus-op wrappers: short CS to check out, run `f` outside any CS, release with a lone store.

    /// The checkout -> run skeleton shared by every bus-op wrapper; `absent` is the refusal value.
    fn with_role<R>(
        &self,
        co: Option<Checkout<'_, DEPS, SWD, JTAG>>,
        absent: R,
        run: impl FnOnce(&mut Role<DEPS, SWD, JTAG>) -> R,
    ) -> R {
        match co {
            Some(mut co) => run(co.role()),
            None => absent,
        }
    }

    /// DAP-side gated bus op as the `Dependencies` role.
    fn gated_deps<R>(&self, gated: R, f: impl FnOnce(&mut DEPS) -> R) -> R {
        self.with_role(self.checkout_gated(), gated, |role| run_as_deps(role, f))
    }

    /// DAP-side gated bus op as the SWD role.
    fn gated_swd<R>(&self, gated: R, f: impl FnOnce(&mut SWD) -> R) -> R {
        self.with_role(self.checkout_gated(), gated, |role| run_as_swd(role, f))
    }

    /// DAP-side gated bus op as the JTAG role.
    fn gated_jtag<R>(&self, gated: R, f: impl FnOnce(&mut JTAG) -> R) -> R {
        self.with_role(self.checkout_gated(), gated, |role| run_as_jtag(role, f))
    }

    /// Flash-side ungated bus op as the `Dependencies` role; `busy` only once the wait budget is
    /// spent, which on one core means the flash path preempted the holder.
    fn flash_deps<R>(&self, busy: R, f: impl FnOnce(&mut DEPS) -> R) -> R {
        self.with_role(self.checkout_flash(), busy, |role| run_as_deps(role, f))
    }

    /// Flash-side ungated bus op as the SWD role.
    fn flash_swd<R>(&self, busy: R, f: impl FnOnce(&mut SWD) -> R) -> R {
        self.with_role(self.checkout_flash(), busy, |role| run_as_swd(role, f))
    }
}

impl<DEPS, SWD, JTAG> FlashArbiter for &'static SwdMux<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    type Session = FlashSession<DEPS, SWD, JTAG>;
    fn begin_session(&self) -> Option<Self::Session> {
        self.begin_flash()
    }
}

// DAP-side handles (gated), each with its own configs so host state cannot reach the flash side.

/// The DAP engine's `Dependencies` handle over the mux.
pub struct DapDeps<DEPS, SWD, JTAG>
where
    DEPS: 'static,
    SWD: 'static,
    JTAG: 'static,
{
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    swd_cfg: swd::Config,
    jtag_cfg: jtag::Config,
    swd_available: bool,
    jtag_available: bool,
}

/// The DAP engine's SWD handle over the mux.
pub struct DapSwd<DEPS, SWD, JTAG>
where
    DEPS: 'static,
    SWD: 'static,
    JTAG: 'static,
{
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    swd_cfg: swd::Config,
    jtag_cfg: jtag::Config,
    swd_available: bool,
    jtag_available: bool,
}

/// The DAP engine's JTAG handle over the mux.
pub struct DapJtag<DEPS, SWD, JTAG>
where
    DEPS: 'static,
    SWD: 'static,
    JTAG: 'static,
{
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    swd_cfg: swd::Config,
    jtag_cfg: jtag::Config,
    swd_available: bool,
    jtag_available: bool,
}

// The three handles carry identical state; this is the whole graph dap-rs walks.
impl<DEPS, SWD, JTAG> From<DapSwd<DEPS, SWD, JTAG>> for DapDeps<DEPS, SWD, JTAG> {
    fn from(h: DapSwd<DEPS, SWD, JTAG>) -> Self {
        DapDeps {
            mux: h.mux,
            swd_cfg: h.swd_cfg,
            jtag_cfg: h.jtag_cfg,
            swd_available: h.swd_available,
            jtag_available: h.jtag_available,
        }
    }
}
impl<DEPS, SWD, JTAG> From<DapJtag<DEPS, SWD, JTAG>> for DapDeps<DEPS, SWD, JTAG> {
    fn from(h: DapJtag<DEPS, SWD, JTAG>) -> Self {
        DapDeps {
            mux: h.mux,
            swd_cfg: h.swd_cfg,
            jtag_cfg: h.jtag_cfg,
            swd_available: h.swd_available,
            jtag_available: h.jtag_available,
        }
    }
}
impl<DEPS, SWD, JTAG> From<DapDeps<DEPS, SWD, JTAG>> for DapSwd<DEPS, SWD, JTAG> {
    fn from(h: DapDeps<DEPS, SWD, JTAG>) -> Self {
        DapSwd {
            mux: h.mux,
            swd_cfg: h.swd_cfg,
            jtag_cfg: h.jtag_cfg,
            swd_available: h.swd_available,
            jtag_available: h.jtag_available,
        }
    }
}
impl<DEPS, SWD, JTAG> From<DapDeps<DEPS, SWD, JTAG>> for DapJtag<DEPS, SWD, JTAG> {
    fn from(h: DapDeps<DEPS, SWD, JTAG>) -> Self {
        DapJtag {
            mux: h.mux,
            swd_cfg: h.swd_cfg,
            jtag_cfg: h.jtag_cfg,
            swd_available: h.swd_available,
            jtag_available: h.jtag_available,
        }
    }
}

impl<DEPS, SWD, JTAG> Dependencies<DapSwd<DEPS, SWD, JTAG>, DapJtag<DEPS, SWD, JTAG>>
    for DapDeps<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.mux.gated_deps(Pins::empty(), |deps| {
            deps.process_swj_pins(output, mask, wait_us)
        })
    }
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.mux
            .gated_deps((), |deps| deps.process_swj_sequence(data, nbits))
    }
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.mux
            .gated_deps(false, |deps| deps.process_swj_clock(max_frequency))
    }
    fn high_impedance_mode(&mut self) {
        self.mux.gated_deps((), |deps| deps.high_impedance_mode())
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        &mut self.swd_cfg
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        &mut self.jtag_cfg
    }
}

impl<DEPS, SWD, JTAG> Swd<DapDeps<DEPS, SWD, JTAG>> for DapSwd<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    /// The composition's SWD availability, sampled once at handle construction.
    fn available(deps: &DapDeps<DEPS, SWD, JTAG>) -> bool {
        deps.swd_available
    }
    fn config(&mut self) -> &mut swd::Config {
        &mut self.swd_cfg
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.mux
            .gated_swd(Err(swd::Error::NoAck), |swd| swd.read_inner(apndp, a))
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        self.mux.gated_swd(Err(swd::Error::NoAck), |swd| {
            swd.write_inner(apndp, a, data)
        })
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.mux
            .gated_swd(false, |swd| swd.set_clock(max_frequency))
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.mux.gated_swd(Err(swd::Error::NoAck), |swd| {
            swd.write_sequence(num_bits, data)
        })
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.mux.gated_swd(Err(swd::Error::NoAck), |swd| {
            swd.read_sequence(num_bits, data)
        })
    }
}

/// One checkout's worth of JTAG, lent to dap-rs's per-chunk scan choreography, so a whole scan
/// costs one claim. `drv` is `None` on a refused claim, reproducing the per-chunk refusal.
struct LentJtag<'a, DEPS, JTAG> {
    drv: Option<&'a mut JTAG>,
    /// The handle's own config, so host `DAP_JTAG_Configure` state shapes the choreography.
    cfg: &'a mut jtag::Config,
    _deps: PhantomData<fn() -> DEPS>,
}

impl<'a, DEPS, JTAG> LentJtag<'a, DEPS, JTAG> {
    fn new(drv: Option<&'a mut JTAG>, cfg: &'a mut jtag::Config) -> Self {
        Self {
            drv,
            cfg,
            _deps: PhantomData,
        }
    }
}

// `Jtag<DEPS>` requires `From<DEPS>`; naming itself satisfies that, and nothing ever converts.
impl<DEPS, JTAG> Jtag<Self> for LentJtag<'_, DEPS, JTAG>
where
    JTAG: Jtag<DEPS>,
{
    fn available(_lent: &Self) -> bool {
        true
    }
    fn timestamp(&self) -> u32 {
        self.drv.as_ref().map_or(0, |drv| drv.timestamp())
    }
    fn config(&mut self) -> &mut jtag::Config {
        self.cfg
    }
    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        if let Some(drv) = self.drv.as_mut() {
            drv.sequence(info, tdi, rxbuf);
        }
    }
    fn tms_sequence(&mut self, tms: &[bool]) {
        if let Some(drv) = self.drv.as_mut() {
            drv.tms_sequence(tms);
        }
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.drv
            .as_mut()
            .is_some_and(|drv| drv.set_clock(max_frequency))
    }
}

impl<DEPS, SWD, JTAG> DapJtag<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    /// Take one gated checkout and run `f` against a [`LentJtag`] over it. A refused checkout still
    /// runs `f` driverless, so response framing advances as it does on a per-primitive refusal.
    fn lend<R>(&mut self, f: impl FnOnce(&mut LentJtag<'_, DEPS, JTAG>) -> R) -> R {
        let mux = self.mux;
        let cfg = &mut self.jtag_cfg;
        match mux.checkout_gated() {
            Some(mut co) => run_as_jtag(co.role(), |drv| f(&mut LentJtag::new(Some(drv), cfg))),
            None => f(&mut LentJtag::new(None, cfg)),
        }
    }
}

impl<DEPS, SWD, JTAG> Jtag<DapDeps<DEPS, SWD, JTAG>> for DapJtag<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    /// The composition's JTAG availability, sampled once at handle construction.
    fn available(deps: &DapDeps<DEPS, SWD, JTAG>) -> bool {
        deps.jtag_available
    }
    fn config(&mut self) -> &mut jtag::Config {
        &mut self.jtag_cfg
    }
    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        self.mux
            .gated_jtag((), |jtag| jtag.sequence(info, tdi, rxbuf))
    }
    fn tms_sequence(&mut self, tms: &[bool]) {
        self.mux.gated_jtag((), |jtag| jtag.tms_sequence(tms))
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.mux
            .gated_jtag(false, |jtag| jtag.set_clock(max_frequency))
    }

    // Overriding dap-rs's provided methods hoists the claim from each primitive to the whole scan.

    fn sequences(&mut self, req: Request, resp: &mut ResponseWriter) {
        self.lend(|lent| lent.sequences(req, resp))
    }
    fn shift_ir(&mut self, ir: u32) {
        self.lend(|lent| lent.shift_ir(ir))
    }
    fn shift_dr(&mut self, dr: u32) -> u32 {
        self.lend(|lent| lent.shift_dr(dr))
    }
    fn write_abort(&mut self, data: u32) {
        self.lend(|lent| lent.write_abort(data))
    }
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        self.lend(|lent| lent.transfer(r_nw, a2a3, transfer_config, data))
    }
}

// Flash-side handle (ungated).

/// RAII guard for an exclusive flash session; dropping it re-admits the DAP engine.
pub struct FlashSession<DEPS, SWD, JTAG>
where
    DEPS: 'static,
    SWD: 'static,
    JTAG: 'static,
{
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
}

impl<DEPS, SWD, JTAG> FlashSession<DEPS, SWD, JTAG> {
    /// The handle the flash programmer drives the bus through for this session's lifetime.
    pub fn flash_side(&self) -> FlashSide<DEPS, SWD, JTAG> {
        FlashSide {
            mux: self.mux,
            swd_cfg: swd::Config::default(),
        }
    }
}

impl<DEPS, SWD, JTAG> Drop for FlashSession<DEPS, SWD, JTAG> {
    fn drop(&mut self) {
        self.mux.with_state(|s| s.flash_active = false);
        #[cfg(feature = "defmt")]
        defmt::info!("flash session: end");
    }
}

/// The flash programmer's bus handle: ungated, because a live session has already gated the DAP
/// engine.
pub struct FlashSide<DEPS, SWD, JTAG>
where
    DEPS: 'static,
    SWD: 'static,
    JTAG: 'static,
{
    mux: &'static SwdMux<DEPS, SWD, JTAG>,
    swd_cfg: swd::Config,
}

impl<DEPS, SWD, JTAG> From<DapDeps<DEPS, SWD, JTAG>> for FlashSide<DEPS, SWD, JTAG> {
    fn from(h: DapDeps<DEPS, SWD, JTAG>) -> Self {
        FlashSide {
            mux: h.mux,
            swd_cfg: h.swd_cfg,
        }
    }
}

impl<DEPS, SWD, JTAG> Swd<DapDeps<DEPS, SWD, JTAG>> for FlashSide<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    /// The same composition-derived answer the DAP side reports; it drives the same backend.
    fn available(deps: &DapDeps<DEPS, SWD, JTAG>) -> bool {
        deps.swd_available
    }
    fn config(&mut self) -> &mut swd::Config {
        &mut self.swd_cfg
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.mux
            .flash_swd(Err(swd::Error::NoAck), |swd| swd.read_inner(apndp, a))
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        self.mux.flash_swd(Err(swd::Error::NoAck), |swd| {
            swd.write_inner(apndp, a, data)
        })
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.mux
            .flash_swd(false, |swd| swd.set_clock(max_frequency))
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.mux.flash_swd(Err(swd::Error::NoAck), |swd| {
            swd.write_sequence(num_bits, data)
        })
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.mux.flash_swd(Err(swd::Error::NoAck), |swd| {
            swd.read_sequence(num_bits, data)
        })
    }
}

impl<DEPS, SWD, JTAG> ResetControl for FlashSide<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    type Error = core::convert::Infallible;
    fn set_reset(&mut self, asserted: bool) -> Result<(), Self::Error> {
        // nRESET is active-low: assert drives the pin low, release drives it high.
        let output = if asserted {
            Pins::empty()
        } else {
            Pins::NRESET
        };
        self.mux.flash_deps((), |deps| {
            deps.process_swj_pins(output, Pins::NRESET, 0);
        });
        Ok(())
    }
}

impl<DEPS, SWD, JTAG> ProbeInfo for FlashSide<DEPS, SWD, JTAG>
where
    DEPS: Dependencies<SWD, JTAG> + 'static,
    SWD: Swd<DEPS> + 'static,
    JTAG: Jtag<DEPS> + 'static,
{
    fn read_dpidr(&mut self) -> Option<u32> {
        // One checkout spans the wake sequence and the DPIDR read, so neither is re-arbitrated.
        self.mux.flash_swd(None, |swd| {
            // The wake is best-effort; the DPIDR reads decide, retried in case the first races it.
            let _ = daplink_flash::swd_wake_preamble(swd);
            for _ in 0..4 {
                if let Ok(dpidr) = swd.read_dp(WAIT_RETRIES, DPRegister::DPIDR) {
                    return Some(dpidr);
                }
            }
            None
        })
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use core::cell::Cell;
    use core::cell::RefCell as StdRefCell;
    use dap_rs::dap::Command;
    use static_cell::StaticCell;
    use std::boxed::Box;
    use std::string::{String, ToString};
    use std::vec::Vec;

    /// Shared, append-only call log for the mock adapter roles.
    type Log = StdRefCell<Vec<String>>;

    fn note(log: &Log, s: &str) {
        log.borrow_mut().push(s.to_string());
    }

    /// A never-failing read budget: the default unless a test injects a fault.
    fn no_fail() -> &'static Cell<u32> {
        Box::leak(Box::new(Cell::new(0)))
    }

    /// What a mock composition's two transports report from `available()`.
    #[derive(Clone, Copy)]
    struct Avail {
        swd: bool,
        jtag: bool,
    }

    /// The full-capability composition every test uses unless it is about what the mux advertises.
    const BOTH: Avail = Avail {
        swd: true,
        jtag: true,
    };

    /// Three mock roles with identical state, each conversion logged so a test can count them.
    struct MDeps {
        log: &'static Log,
        fail_reads: &'static Cell<u32>,
        swd_cfg: swd::Config,
        jtag_cfg: jtag::Config,
        avail: Avail,
    }
    struct MSwd {
        log: &'static Log,
        fail_reads: &'static Cell<u32>,
        swd_cfg: swd::Config,
        jtag_cfg: jtag::Config,
        avail: Avail,
    }
    struct MJtag {
        log: &'static Log,
        fail_reads: &'static Cell<u32>,
        swd_cfg: swd::Config,
        jtag_cfg: jtag::Config,
        avail: Avail,
    }

    impl From<MSwd> for MDeps {
        fn from(v: MSwd) -> Self {
            note(v.log, "conv:swd->deps");
            MDeps {
                log: v.log,
                fail_reads: v.fail_reads,
                swd_cfg: v.swd_cfg,
                jtag_cfg: v.jtag_cfg,
                avail: v.avail,
            }
        }
    }
    impl From<MJtag> for MDeps {
        fn from(v: MJtag) -> Self {
            note(v.log, "conv:jtag->deps");
            MDeps {
                log: v.log,
                fail_reads: v.fail_reads,
                swd_cfg: v.swd_cfg,
                jtag_cfg: v.jtag_cfg,
                avail: v.avail,
            }
        }
    }
    impl From<MDeps> for MSwd {
        fn from(v: MDeps) -> Self {
            note(v.log, "conv:deps->swd");
            MSwd {
                log: v.log,
                fail_reads: v.fail_reads,
                swd_cfg: v.swd_cfg,
                jtag_cfg: v.jtag_cfg,
                avail: v.avail,
            }
        }
    }
    impl From<MDeps> for MJtag {
        fn from(v: MDeps) -> Self {
            note(v.log, "conv:deps->jtag");
            MJtag {
                log: v.log,
                fail_reads: v.fail_reads,
                swd_cfg: v.swd_cfg,
                jtag_cfg: v.jtag_cfg,
                avail: v.avail,
            }
        }
    }

    impl Dependencies<MSwd, MJtag> for MDeps {
        fn process_swj_pins(&mut self, output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
            note(self.log, "deps:pins");
            output
        }
        fn process_swj_sequence(&mut self, _data: &[u8], _nbits: usize) {
            note(self.log, "deps:seq");
        }
        fn process_swj_clock(&mut self, _max_frequency: u32) -> bool {
            note(self.log, "deps:clock");
            true
        }
        fn high_impedance_mode(&mut self) {
            note(self.log, "deps:hiz");
        }
        fn swd_config(&mut self) -> &mut swd::Config {
            &mut self.swd_cfg
        }
        fn jtag_config(&mut self) -> &mut jtag::Config {
            &mut self.jtag_cfg
        }
    }

    impl Swd<MDeps> for MSwd {
        fn available(deps: &MDeps) -> bool {
            deps.avail.swd
        }
        fn config(&mut self) -> &mut swd::Config {
            &mut self.swd_cfg
        }
        fn read_inner(&mut self, _apndp: APnDP, _a: DPRegister) -> swd::Result<u32> {
            // The driver's own TAP count, so a host-mutated handle config is visibly separate.
            note(
                self.log,
                &std::format!("drv:dc{}", self.jtag_cfg.device_count),
            );
            note(self.log, "swd:read");
            if self.fail_reads.get() > 0 {
                self.fail_reads.set(self.fail_reads.get() - 1);
                return Err(swd::Error::NoAck);
            }
            Ok(0x1234_5678)
        }
        fn write_inner(&mut self, _apndp: APnDP, _a: DPRegister, _data: u32) -> swd::Result<()> {
            note(self.log, "swd:write");
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            note(self.log, "swd:clock");
            true
        }
        fn write_sequence(&mut self, num_bits: usize, _data: &[u8]) -> swd::Result<()> {
            note(self.log, &std::format!("swd:wseq:{num_bits}"));
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> swd::Result<()> {
            note(self.log, "swd:rseq");
            Ok(())
        }
    }

    impl Jtag<MDeps> for MJtag {
        fn available(deps: &MDeps) -> bool {
            deps.avail.jtag
        }
        fn config(&mut self) -> &mut jtag::Config {
            &mut self.jtag_cfg
        }
        fn sequence(&mut self, info: SequenceInfo, _tdi: &[u8], rxbuf: &mut [u8]) {
            note(
                self.log,
                &std::format!("jtag:seq:{}:dc{}", info.n_bits, self.jtag_cfg.device_count),
            );
            // Stand in for the target's OK ack, the one 2-bit capture in a DPACC/APACC scan.
            if info.n_bits == 2 && info.capture {
                if let Some(first) = rxbuf.first_mut() {
                    *first = 0x02;
                }
            }
        }
        fn tms_sequence(&mut self, tms: &[bool]) {
            note(self.log, &std::format!("jtag:tms:{}", tms.len()));
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            note(self.log, "jtag:clock");
            false
        }
    }

    type Mux = SwdMux<MDeps, MSwd, MJtag>;

    fn leak_log() -> &'static Log {
        Box::leak(Box::new(StdRefCell::new(Vec::new())))
    }

    /// A DAP-side SWD transfer delegates to the real adapter; a run of them converts the role once.
    #[test]
    fn dap_swd_delegates_and_converts_once() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let mut dap_swd = DapSwd::from(mux.dap_deps(scan_handle));

        let _ = dap_swd.read_inner(APnDP::DP, DPRegister::DPIDR);
        let _ = dap_swd.read_inner(APnDP::DP, DPRegister::DPIDR);

        let log = log.borrow();
        assert_eq!(
            log.iter().filter(|s| *s == "swd:read").count(),
            2,
            "both transfers reach the real adapter"
        );
        assert_eq!(
            log.iter().filter(|s| *s == "conv:deps->swd").count(),
            1,
            "the role is converted exactly once"
        );
    }

    /// While a flash session is live a DAP-side transfer returns NoAck and never reaches the wire.
    #[test]
    fn dap_swd_gated_while_flash_active() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let mut dap_swd = DapSwd::from(mux.dap_deps(scan_handle));

        let session = mux.begin_flash().expect("session");
        let before = log.borrow().len();
        let r = dap_swd.read_inner(APnDP::DP, DPRegister::DPIDR);
        assert_eq!(r, Err(swd::Error::NoAck), "gated transfer returns NoAck");
        assert_eq!(
            log.borrow().len(),
            before,
            "gated transfer must not touch the adapter"
        );
        drop(session);

        let _ = dap_swd.read_inner(APnDP::DP, DPRegister::DPIDR);
        assert!(log.borrow().iter().any(|s| s == "swd:read"));
    }

    /// A DAP-side `Dependencies` operation is gated to `false` while flashing.
    #[test]
    fn dap_deps_clock_gated_while_flash_active() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let mut deps = mux.dap_deps(scan_handle);

        assert!(deps.process_swj_clock(1_000_000), "delegates when idle");
        let session = mux.begin_flash().expect("session");
        assert!(
            !deps.process_swj_clock(1_000_000),
            "gated to false while flashing"
        );
        drop(session);
    }

    /// Only one session may be live at a time; the guard re-admits others on drop.
    #[test]
    fn session_is_exclusive_and_releases_on_drop() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));

        let first = mux.begin_flash();
        assert!(first.is_some(), "first session granted");
        assert!(
            mux.begin_flash().is_none(),
            "second session refused while first is live"
        );
        drop(first);
        assert!(
            mux.begin_flash().is_some(),
            "a new session is granted after release"
        );
    }

    /// A flash-side op waits out an in-flight DAP op instead of failing the bring-up on contact.
    #[test]
    fn flash_op_succeeds_when_holder_releases_between_attempts() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));

        let session = mux.begin_flash().expect("session");
        // The one host transfer already in flight when the session opened.
        let held = StdRefCell::new(Some(mux.checkout().expect("nothing else in flight")));
        let attempts = Cell::new(0usize);
        // The relax hook stands in for the holder making progress while the flash side waits.
        let co = mux.checkout_flash_with(8, || {
            attempts.set(attempts.get() + 1);
            if attempts.get() == 3 {
                drop(held.borrow_mut().take().expect("released exactly once"));
            }
        });
        let co = co.expect("the waited-out checkout succeeds");
        assert_eq!(attempts.get(), 3, "succeeds on the probe after the release");
        drop(co);

        let mut side = session.flash_side();
        assert_eq!(
            side.read_inner(APnDP::DP, DPRegister::DPIDR),
            Ok(0x1234_5678),
            "the flash transfer reaches the real adapter"
        );
        drop(session);
    }

    /// A holder that never releases spends exactly the budget and returns the busy fallback.
    #[test]
    fn flash_op_bounded_failure_when_holder_never_releases() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));

        let session = mux.begin_flash().expect("session");
        let held = mux.checkout().expect("nothing else in flight");

        let attempts = Cell::new(0usize);
        assert!(
            mux.checkout_flash_with(5, || attempts.set(attempts.get() + 1))
                .is_none(),
            "a never-released holder still fails the checkout"
        );
        assert_eq!(attempts.get(), 5, "exactly the injected budget is spent");

        let mut side = session.flash_side();
        let before = log.borrow().len();
        assert_eq!(
            side.read_inner(APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::NoAck),
            "bounded failure keeps the busy fallback"
        );
        assert_eq!(
            log.borrow().len(),
            before,
            "a busy flash op must not touch the adapter"
        );

        drop(held);
        assert_eq!(
            side.read_inner(APnDP::DP, DPRegister::DPIDR),
            Ok(0x1234_5678)
        );
        drop(session);
    }

    /// `read_dpidr` issues the cold-target wake preamble before reading DPIDR, in that order.
    #[test]
    fn read_dpidr_emits_switch_preamble_then_reads() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));

        let session = mux.begin_flash().expect("session");
        let mut side = session.flash_side();
        assert_eq!(side.read_dpidr(), Some(0x1234_5678));

        let seq: Vec<String> = log
            .borrow()
            .iter()
            .filter(|s| s.starts_with("swd:"))
            .cloned()
            .collect();
        assert_eq!(
            seq,
            [
                "swd:wseq:51",
                "swd:wseq:16",
                "swd:wseq:51",
                "swd:wseq:8",
                "swd:read",
            ]
        );
        drop(session);
    }

    /// The flash side drives nRESET ungated even while a session is active -- it *is* the session.
    #[test]
    fn flash_side_reset_drives_pins_ungated() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));

        let session = mux.begin_flash().expect("session");
        let mut side = session.flash_side();
        assert!(side.set_reset(true).is_ok());
        assert!(side.set_reset(false).is_ok());
        assert_eq!(
            log.borrow().iter().filter(|s| *s == "deps:pins").count(),
            2,
            "both reset edges reach the adapter despite the live session"
        );
        drop(session);
    }

    /// A bus op that errors still restores the role, so the next transfer reaches the adapter.
    #[test]
    fn erroring_bus_op_still_restores_role() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let fail_reads: &'static Cell<u32> = Box::leak(Box::new(Cell::new(1)));
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads,
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let mut dap_swd = DapSwd::from(mux.dap_deps(scan_handle));

        let first = dap_swd.read_inner(APnDP::DP, DPRegister::DPIDR);
        assert_eq!(first, Err(swd::Error::NoAck), "injected fault surfaces");

        let second = dap_swd.read_inner(APnDP::DP, DPRegister::DPIDR);
        assert_eq!(
            second,
            Ok(0x1234_5678),
            "the adapter was restored, so the follow-up op reaches it"
        );
        assert_eq!(
            log.borrow().iter().filter(|s| *s == "swd:read").count(),
            2,
            "both transfers reached the real adapter"
        );
    }

    /// The exclusion the borrowed checkout rests on: a re-entrant transfer is refused, so two
    /// `&mut` to the adapter cannot coexist.
    #[test]
    fn reentrant_dap_transfer_is_refused_while_a_checkout_is_live() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let mut dap_swd = DapSwd::from(mux.dap_deps(scan_handle));

        let held = mux.checkout().expect("nothing else in flight");
        let before = log.borrow().len();
        assert_eq!(
            dap_swd.read_inner(APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::NoAck),
            "a re-entrant transfer fails fast instead of aliasing the adapter"
        );
        assert_eq!(
            log.borrow().len(),
            before,
            "the refused transfer must not touch the adapter"
        );

        assert!(mux.checkout().is_none(), "the mux admits one checkout");
        drop(held);
        assert!(
            mux.checkout().is_some(),
            "released, so the next op is admitted"
        );
    }

    /// The JTAG entries (raw wire traffic) currently in the log.
    fn jtag_entries(log: &Log) -> Vec<String> {
        log.borrow()
            .iter()
            .filter(|s| s.starts_with("jtag:"))
            .cloned()
            .collect()
    }

    /// A host `DAP_JTAG_Configure` lands in the handle's config, and that is the config the wire
    /// sees, while the shared driver stays on its own default (`dc0`).
    #[test]
    fn host_jtag_config_reaches_the_wire_through_the_handle() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let mut dap_jtag = DapJtag::from(mux.dap_deps(scan_handle));

        let cfg = dap_jtag.config();
        cfg.device_count = 1;
        cfg.index = 0;
        cfg.scan_chain[0].ir_length = 6;

        dap_jtag.shift_ir(0x0A);

        assert_eq!(
            jtag_entries(log),
            [
                "jtag:tms:4",
                "jtag:seq:5:dc0",
                "jtag:seq:1:dc0",
                "jtag:tms:2",
            ],
            "handle config drove the wire while the driver's own config stayed default"
        );
    }

    /// Host DAP configuration is handle-local, so a flash interleave has nothing to clobber and
    /// the post-session traffic is identical to the pre-session traffic.
    #[test]
    fn dap_config_stays_handle_local_across_flash_interleave() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let mut dap_jtag = DapJtag::from(mux.dap_deps(scan_handle));

        let cfg = dap_jtag.config();
        cfg.device_count = 1;
        cfg.scan_chain[0].ir_length = 6;

        dap_jtag.shift_ir(0x0A);
        let before = jtag_entries(log);
        log.borrow_mut().clear();

        let session = mux.begin_flash().expect("session");
        let mut side = session.flash_side();
        let _ = side.read_inner(APnDP::DP, DPRegister::DPIDR);
        assert!(
            log.borrow().iter().any(|s| s == "drv:dc0"),
            "flash transfer ran on driver defaults"
        );
        assert!(
            !log.borrow().iter().any(|s| s.contains("dc1")),
            "the DAP-side config must not leak into the flash side"
        );
        drop(session);
        log.borrow_mut().clear();

        dap_jtag.shift_ir(0x0A);
        assert_eq!(jtag_entries(log), before);
    }

    // What the mux advertises: dap-rs reads `available` once, so it must describe the composition.

    /// Build a mux over a composition with the given capabilities and query what it advertises.
    fn advertised(avail: Avail) -> (bool, bool) {
        // Leaked per call, so each capability mix gets its own mux.
        let log = leak_log();
        let scan_real: &'static mut [TapConfig] =
            Box::leak(Box::new([TapConfig::INIT; 2])).as_mut_slice();
        let scan_handle: &'static mut [TapConfig] =
            Box::leak(Box::new([TapConfig::INIT; 2])).as_mut_slice();
        let mux: &'static Mux = Box::leak(Box::new(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail,
        })));
        let deps = mux.dap_deps(scan_handle);
        (
            <DapSwd<MDeps, MSwd, MJtag> as Swd<_>>::available(&deps),
            <DapJtag<MDeps, MSwd, MJtag> as Jtag<_>>::available(&deps),
        )
    }

    /// A backend whose JTAG role is inert must report JTAG unavailable.
    #[test]
    fn mux_does_not_advertise_an_inert_jtag_role() {
        assert_eq!(
            advertised(Avail {
                swd: true,
                jtag: false
            }),
            (true, false),
            "SWD-only composition: SWD advertised, JTAG not"
        );
    }

    /// The mirror case, so a constant answer of either polarity fails one of the two tests.
    #[test]
    fn mux_advertises_a_real_jtag_role() {
        assert_eq!(advertised(BOTH), (true, true), "both transports advertised");
    }

    /// The SWD side delegates too: the answer comes from the backend, not from a constant.
    #[test]
    fn mux_does_not_advertise_an_inert_swd_role() {
        assert_eq!(
            advertised(Avail {
                swd: false,
                jtag: true
            }),
            (false, true),
            "JTAG-only composition: JTAG advertised, SWD not"
        );
    }

    /// The flash-side handle answers the same composition question as the DAP side.
    #[test]
    fn flash_side_reports_the_same_swd_availability() {
        for avail in [
            BOTH,
            Avail {
                swd: false,
                jtag: true,
            },
        ] {
            let log = leak_log();
            let scan_real: &'static mut [TapConfig] =
                Box::leak(Box::new([TapConfig::INIT; 2])).as_mut_slice();
            let scan_handle: &'static mut [TapConfig] =
                Box::leak(Box::new([TapConfig::INIT; 2])).as_mut_slice();
            let mux: &'static Mux = Box::leak(Box::new(SwdMux::new(MDeps {
                log,
                fail_reads: no_fail(),
                swd_cfg: swd::Config::default(),
                jtag_cfg: jtag::Config::new(scan_real),
                avail,
            })));
            let deps = mux.dap_deps(scan_handle);
            assert_eq!(
                <FlashSide<MDeps, MSwd, MJtag> as Swd<_>>::available(&deps),
                avail.swd,
            );
        }
    }

    /// Availability describes the composition, not the runtime state of the wire.
    #[test]
    fn advertised_capabilities_are_not_gated_by_a_flash_session() {
        static MUX: StaticCell<Mux> = StaticCell::new();
        static SCAN_REAL: StaticCell<[TapConfig; 2]> = StaticCell::new();
        static SCAN_HANDLE: StaticCell<[TapConfig; 2]> = StaticCell::new();
        let log = leak_log();
        let scan_real = SCAN_REAL.init([TapConfig::INIT; 2]);
        let mux: &'static Mux = MUX.init(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: BOTH,
        }));
        let scan_handle = SCAN_HANDLE.init([TapConfig::INIT; 2]);
        let deps = mux.dap_deps(scan_handle);

        let idle = (
            <DapSwd<MDeps, MSwd, MJtag> as Swd<_>>::available(&deps),
            <DapJtag<MDeps, MSwd, MJtag> as Jtag<_>>::available(&deps),
        );
        assert_eq!(idle, (true, true));

        let session = mux.begin_flash().expect("session");
        assert_eq!(
            (
                <DapSwd<MDeps, MSwd, MJtag> as Swd<_>>::available(&deps),
                <DapJtag<MDeps, MSwd, MJtag> as Jtag<_>>::available(&deps),
            ),
            idle,
            "a live flash session must not change what the probe advertises"
        );

        let held = mux.checkout().expect("nothing else in flight");
        assert_eq!(
            (
                <DapSwd<MDeps, MSwd, MJtag> as Swd<_>>::available(&deps),
                <DapJtag<MDeps, MSwd, MJtag> as Jtag<_>>::available(&deps),
            ),
            idle,
            "an in-flight bus op must not change what the probe advertises"
        );
        drop(held);
        drop(session);
    }

    /// The sampled availability survives the `From` graph the dap-rs state machine walks.
    #[test]
    fn advertised_capabilities_survive_role_round_trips() {
        let log = leak_log();
        let scan_real: &'static mut [TapConfig] =
            Box::leak(Box::new([TapConfig::INIT; 2])).as_mut_slice();
        let scan_handle: &'static mut [TapConfig] =
            Box::leak(Box::new([TapConfig::INIT; 2])).as_mut_slice();
        let mux: &'static Mux = Box::leak(Box::new(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(scan_real),
            avail: Avail {
                swd: true,
                jtag: false,
            },
        })));

        let deps = mux.dap_deps(scan_handle);
        let deps = DapDeps::from(DapSwd::from(deps));
        let deps = DapDeps::from(DapJtag::from(deps));
        assert_eq!(
            (
                <DapSwd<MDeps, MSwd, MJtag> as Swd<_>>::available(&deps),
                <DapJtag<MDeps, MSwd, MJtag> as Jtag<_>>::available(&deps),
            ),
            (true, false),
            "the composition's answer round-trips through the handle conversions"
        );
    }

    // What a command costs: one claim is one critical section, and the release is a lone store.

    /// What [`fixture`] hands a claim-counting test.
    type Fixture = (
        &'static Mux,
        &'static Log,
        DapJtag<MDeps, MSwd, MJtag>,
        DapSwd<MDeps, MSwd, MJtag>,
    );

    /// A leaked scan-chain buffer, for the handles a claim-counting test mints.
    fn leak_scan() -> &'static mut [TapConfig] {
        Box::leak(Box::new([TapConfig::INIT; 2])).as_mut_slice()
    }

    /// A fresh mux, log and both handles, built before the counters are zeroed so a test measures
    /// only what it does itself.
    fn fixture() -> Fixture {
        let log = leak_log();
        let mux: &'static Mux = Box::leak(Box::new(SwdMux::new(MDeps {
            log,
            fail_reads: no_fail(),
            swd_cfg: swd::Config::default(),
            jtag_cfg: jtag::Config::new(leak_scan()),
            avail: BOTH,
        })));
        let mut jtag_handle = DapJtag::from(mux.dap_deps(leak_scan()));
        let swd_handle = DapSwd::from(mux.dap_deps(leak_scan()));
        let cfg = jtag_handle.config();
        cfg.device_count = 1;
        cfg.index = 0;
        cfg.scan_chain[0].ir_length = 6;
        log.borrow_mut().clear();
        mux.reset_claims();
        (mux, log, jtag_handle, swd_handle)
    }

    /// The transfer config `DAP_TransferConfigure` leaves behind, at dap-rs's defaults.
    fn transfer_config() -> TransferConfig {
        TransferConfig {
            idle_cycles: 1,
            wait_retries: 5,
            match_retries: 8,
            match_mask: 0,
        }
    }

    /// Every critical section the mux takes goes through the counted helper, so the claim-cost
    /// claims below are about the mux and not about where the counter was placed.
    #[test]
    fn the_section_counter_sees_every_mux_critical_section() {
        let (mux, _log, _jtag, _swd) = fixture();

        assert!(!mux.is_flashing());
        assert_eq!(mux.sections(), 1, "a state query is a critical section");

        let session = mux.begin_flash().expect("session");
        assert_eq!(mux.sections(), 2, "opening a session is another");
        drop(session);
        assert_eq!(mux.sections(), 3, "and so is closing it");
        assert_eq!(mux.claims(), 0, "none of which granted a checkout");
    }

    /// A claim costs one critical section, not two: the release is a lone atomic store.
    #[test]
    fn a_claim_costs_one_critical_section_not_two() {
        let (mux, _log, _jtag, mut swd) = fixture();

        let _ = swd.read_inner(APnDP::DP, DPRegister::DPIDR);

        assert_eq!(mux.claims(), 1, "one claim");
        assert_eq!(
            mux.sections(),
            1,
            "and it cost one critical section -- the release takes none"
        );
    }

    /// A refused claim also costs one critical section and grants nothing.
    #[test]
    fn a_refused_claim_costs_one_critical_section_and_grants_none() {
        let (mux, _log, _jtag, mut swd) = fixture();
        let session = mux.begin_flash().expect("session");
        mux.reset_claims();

        assert_eq!(
            swd.read_inner(APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::NoAck),
            "gated"
        );

        assert_eq!(mux.claims(), 0, "nothing was granted");
        assert_eq!(mux.sections(), 1, "the admission test still cost one");
        drop(session);
    }

    /// dap-rs turns an SWD block into one role call per word, each one claim and one section.
    #[test]
    fn a_run_of_swd_words_costs_one_section_each() {
        let (mux, _log, _jtag, mut swd) = fixture();

        for _ in 0..15 {
            let _ = swd.read_inner(APnDP::DP, DPRegister::DPIDR);
        }

        assert_eq!(mux.claims(), 15, "one claim per word");
        assert_eq!(mux.sections(), 15, "and 15 critical sections, not 30");
    }

    /// The JTAG hoist and the cheaper claim compose: a whole scan is one claim and one section.
    #[test]
    fn a_jtag_transfer_costs_one_section_for_its_whole_scan() {
        let (mux, log, mut handle, _swd) = fixture();

        let _ = handle.transfer(RnW::R, 0, &transfer_config(), 0);

        assert_eq!(jtag_entries(log).len(), 10, "ten primitives, unchanged");
        assert_eq!(mux.claims(), 1);
        assert_eq!(mux.sections(), 1, "one section for ten primitives");
    }

    /// The unhoisted cost, for comparison: a bare primitive claims the adapter for itself.
    #[test]
    fn each_bare_primitive_claims_the_adapter_for_itself() {
        let (mux, _log, mut handle, _swd) = fixture();

        handle.tms_sequence(&[true, false]);
        handle.sequence(
            SequenceInfo {
                n_bits: 8,
                capture: false,
                tms: false,
            },
            &[0xAA],
            &mut [0u8],
        );
        handle.tms_sequence(&[true]);

        assert_eq!(mux.claims(), 3, "one claim per primitive");
    }

    /// A JTAG register transfer is ten primitives and one claim: same wire, different arbitration.
    #[test]
    fn a_jtag_transfer_claims_once_for_its_whole_scan() {
        let (mux, log, mut handle, _swd) = fixture();

        let result = handle.transfer(RnW::R, 0, &transfer_config(), 0);

        assert!(
            matches!(result, TransferResult::Ok(_)),
            "the mock acks, so the scan runs its data phase"
        );
        assert_eq!(
            jtag_entries(log).len(),
            10,
            "dap-rs still builds the scan out of ten primitives"
        );
        assert_eq!(mux.claims(), 1, "which cost one claim, not ten");
    }

    /// The same hoist on the IR shift that precedes a block: four primitives, one claim.
    #[test]
    fn a_jtag_ir_shift_claims_once() {
        let (mux, log, mut handle, _swd) = fixture();

        handle.shift_ir(0x0A);

        assert_eq!(
            jtag_entries(log),
            [
                "jtag:tms:4",
                "jtag:seq:5:dc0",
                "jtag:seq:1:dc0",
                "jtag:tms:2"
            ],
            "unchanged traffic"
        );
        assert_eq!(mux.claims(), 1);
    }

    /// `DAP_JTAG_Sequence` is a whole command's worth of raw sequences, and it too claims once.
    #[test]
    fn a_jtag_sequence_command_claims_once() {
        let (mux, log, mut handle, _swd) = fixture();
        let report = [0x14u8, 3, 8, 0xAA, 8, 0xBB, 8, 0xCC];
        let mut rbuf = [0u8; 32];
        let mut resp = ResponseWriter::new(Command::DAP_JTAG_Sequence, &mut rbuf);

        handle.sequences(Request::from_report(&report).expect("report"), &mut resp);

        assert_eq!(
            jtag_entries(log),
            ["jtag:seq:8:dc0", "jtag:seq:8:dc0", "jtag:seq:8:dc0"],
            "all three sequences reach the driver"
        );
        assert_eq!(mux.claims(), 1);
    }

    /// A refused hoisted method still advances the engine's response framing, so a host gets a
    /// correctly-sized reply rather than a short packet.
    #[test]
    fn a_gated_jtag_sequence_command_still_frames_its_response() {
        let (mux, log, mut handle, _swd) = fixture();
        // One captured 8-bit sequence: the driver would fill one response byte.
        let report = [0x14u8, 1, 0x88, 0xAA];
        let mut rbuf = [0u8; 32];

        let mut resp = ResponseWriter::new(Command::DAP_JTAG_Sequence, &mut rbuf);
        handle.sequences(Request::from_report(&report).expect("report"), &mut resp);
        let ungated_len = resp.idx;

        let session = mux.begin_flash().expect("session");
        let before = log.borrow().len();
        let mut rbuf = [0u8; 32];
        let mut resp = ResponseWriter::new(Command::DAP_JTAG_Sequence, &mut rbuf);
        handle.sequences(Request::from_report(&report).expect("report"), &mut resp);

        assert_eq!(resp.idx, ungated_len, "same framing whether gated or not");
        assert_eq!(
            log.borrow().len(),
            before,
            "a gated command must not touch the adapter"
        );
        drop(session);
    }

    /// A gated JTAG transfer answers `Wait` without touching the adapter.
    #[test]
    fn a_gated_jtag_transfer_answers_wait_and_stays_off_the_wire() {
        let (mux, log, mut handle, _swd) = fixture();
        let session = mux.begin_flash().expect("session");

        let result = handle.transfer(RnW::R, 0, &transfer_config(), 0);

        assert_eq!(result, TransferResult::Wait);
        assert!(log.borrow().is_empty(), "nothing reached the adapter");
        drop(session);
    }
}
