//! The hardware flash backend: lends SWD, delay and staging buffer to a session, reclaims on finish.

use dap_rs::swd::Swd;
use embedded_hal::delay::DelayNs;

use daplink_core::flash_sink::{FlashBackend, FlashPhase, FlashPoll, TargetWindows};
use daplink_core::probe::{FlashArbiter, ProbeInfo, ResetControl};
use daplink_flash::target::{Target, TargetCallbacks};
use daplink_flash::target_flash::InitPhase;
use daplink_flash::{
    EraseMode, Error, FlashManager, FlushRun, ResetRun, StepPoll, SyscallRun, TargetFlash,
};
use daplink_targets::{ImageHint, SelectError, TargetSelector};

use crate::dap_probe::CommandFence;

/// Maximum number of programmable flash ranges cached per session.
const MAX_RANGES: usize = 8;

/// Maximum cached flash/RAM window pairings; raise it for another table row or region.
const MAX_COMPOSED_WINDOWS: usize = 5;

/// Maximum number of RAM ranges cached for the selected target.
const MAX_RAM_RANGES: usize = 2;

/// Fences each blocking operation's return (including errors) and each explicit `step`.
pub struct FencedFlashBackend<B, F> {
    backend: B,
    fence: F,
}

impl<B, F> FencedFlashBackend<B, F> {
    /// Pair one backend with the command fence issued by its wire engine.
    pub const fn new(backend: B, fence: F) -> Self {
        Self { backend, fence }
    }
}

impl<B: FlashBackend, F: CommandFence> FlashBackend for FencedFlashBackend<B, F> {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        let result = self.backend.begin();
        self.fence.finish_command();
        result
    }

    fn begin_with_hint(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        let result = self.backend.begin_with_hint(family);
        self.fence.finish_command();
        result
    }

    fn begin_start(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        // No fence: nothing is observable until `step` reports the session open.
        self.backend.begin_start(family)
    }

    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
        let result = self.backend.program(addr, data);
        self.fence.finish_command();
        result
    }

    fn program_chunk(&mut self, addr: u32, data: &[u8]) -> Result<usize, FlashPhase> {
        // Staging touches no wire; the fence lands on the `step` that programs the page.
        self.backend.program_chunk(addr, data)
    }

    fn step(&mut self, now_us: u64) -> FlashPoll {
        let result = self.backend.step(now_us);
        // The wire work of a step must land before anything the caller can observe.
        self.fence.finish_command();
        result
    }

    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        let result = self.backend.finish(success);
        self.fence.finish_command();
        result
    }

    fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
        let result = self.backend.finish_with_reset(success, reset);
        self.fence.finish_command();
        result
    }

    fn finish_start(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
        self.backend.finish_start(success, reset)
    }

    fn settle(&mut self) -> Result<(), FlashPhase> {
        let result = self.backend.settle();
        self.fence.finish_command();
        result
    }

    fn set_sector_erase(&mut self, sector: bool) -> Result<(), FlashPhase> {
        let result = self.backend.set_sector_erase(sector);
        self.fence.finish_command();
        result
    }

    fn erase_all(&mut self) -> Result<(), FlashPhase> {
        let result = self.backend.erase_all();
        self.fence.finish_command();
        result
    }

    fn flash_ranges(&self) -> &[(u32, u32)] {
        self.backend.flash_ranges()
    }

    fn ram_ranges(&self) -> &[(u32, u32)] {
        self.backend.ram_ranges()
    }

    fn candidate_windows(&self) -> &[TargetWindows] {
        self.backend.candidate_windows()
    }

    fn expected_uf2_family(&self) -> Option<u32> {
        self.backend.expected_uf2_family()
    }

    fn accepts_uf2_family(&self, family: u32) -> bool {
        self.backend.accepts_uf2_family(family)
    }

    fn can_select_uf2_family(&self, family: u32) -> bool {
        self.backend.can_select_uf2_family(family)
    }

    fn requires_uf2_family(&self) -> bool {
        self.backend.requires_uf2_family()
    }

    fn selected_target_name(&self) -> Option<&str> {
        self.backend.selected_target_name()
    }
}

/// Report a window that did not fit its table; overflow only makes range checks refuse more.
fn note_dropped_window(table: &'static str) {
    let _ = table;
    debug_assert!(false, "composed target windows exceeded a fixed capacity");
}

/// The page-buffering manager wrapping the on-target flash driver for one session.
type SessionManager<'a, S, D, T, C, DEPS> = FlashManager<'a, TargetFlash<'a, S, D, T, C, DEPS>>;

/// The session's flash driver, driving whichever `dyn Target` the selector resolved.
type SessionFlash<'a, S, D, C, DEPS> = TargetFlash<'a, S, D, dyn Target + 'a, C, DEPS>;

/// Where a stepped teardown is.
enum FinishStage<'a, S, D, C, DEPS> {
    /// Flush the page the manager still holds staged, resumably.
    Flush(Option<FlushRun>),
    /// The manager is released; tear the resident algorithm down.
    Uninit(SessionFlash<'a, S, D, C, DEPS>, &'a mut [u8]),
    /// Drive the post-flash reset.
    PostReset(SessionFlash<'a, S, D, C, DEPS>, &'a mut [u8], ResetRun),
}

/// The bounded device work one session has deferred to [`FlashBackend::step`].
enum Deferred<'a, S, D, C, DEPS> {
    /// Nothing outstanding.
    None,
    /// Bring-up in flight, with the session's parts held aside until a manager can wrap them.
    Init(
        SessionFlash<'a, S, D, C, DEPS>,
        &'a mut [u8],
        Option<ResetRun>,
    ),
    /// The manager's lazy start-up: `init`, then any up-front chip erase.
    Prepare,
    /// An up-front chip erase in flight (Chip erase mode only).
    PrepareErase(SyscallRun),
    /// A staged page waiting to be erased, programmed and verified; `Some` holds the in-flight run.
    Program(Option<FlushRun>),
    /// Teardown.
    Finish(FinishStage<'a, S, D, C, DEPS>),
}

/// Concrete [`FlashBackend`] over the on-target flash-algorithm runtime.
/// Between sessions it holds the SWD driver and delay, which move into a session together;
/// programming also lends the buffer, mass erase does not.
pub struct HwBackend<'a, S, D, T, C, R, Sel, A, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target,
    C: TargetCallbacks<S, D> + Clone,
    R: ResetControl + ProbeInfo,
    Sel: TargetSelector,
    A: FlashArbiter,
{
    swd: Option<S>,
    delay: Option<D>,
    buffer: Option<&'a mut [u8]>,
    callbacks: C,
    reset: R,
    selector: Sel,
    target: &'a T,
    table: &'a [&'a dyn Target],
    /// The target the selector resolved for the current or last session.
    selected: Option<&'a dyn Target>,
    auto_reset: bool,
    manager: Option<SessionManager<'a, S, D, dyn Target + 'a, C, DEPS>>,
    ranges: heapless::Vec<(u32, u32), MAX_RANGES>,
    /// The selected target's RAM regions, cached alongside `ranges` at `begin`.
    ram_ranges: heapless::Vec<(u32, u32), MAX_RAM_RANGES>,
    /// Each composed target's flash windows paired with its own RAM windows, never pooled.
    composed_windows: heapless::Vec<TargetWindows, MAX_COMPOSED_WINDOWS>,
    arbiter: A,
    session: Option<A::Session>,
    /// Erase granularity for each session: per-sector unless a host selects chip erase.
    sector_erase: bool,
    /// Work deferred to [`FlashBackend::step`].
    deferred: Deferred<'a, S, D, C, DEPS>,
    /// A teardown flush failure, reported once the teardown it interrupted has finished.
    latched_finish: Option<FlashPhase>,
}

impl<'a, S, D, T, C, R, Sel, A, DEPS> HwBackend<'a, S, D, T, C, R, Sel, A, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target,
    C: TargetCallbacks<S, D> + Clone,
    R: ResetControl + ProbeInfo,
    Sel: TargetSelector,
    A: FlashArbiter,
{
    /// Build a backend from its owned parts; `swd`, `delay` and `buffer` are lent per session.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        swd: S,
        delay: D,
        callbacks: C,
        reset: R,
        selector: Sel,
        target: &'a T,
        table: &'a [&'a dyn Target],
        buffer: &'a mut [u8],
        arbiter: A,
        auto_reset: bool,
    ) -> Self {
        // The composed windows are a fact about the firmware, not about a session.
        let mut composed_windows = heapless::Vec::new();
        for entry in table {
            for flash in entry.flash_regions() {
                for ram in entry.ram_regions() {
                    let window = TargetWindows {
                        flash: (flash.start, flash.end),
                        ram: (ram.start, ram.end),
                    };
                    if composed_windows.push(window).is_err() {
                        note_dropped_window("MAX_COMPOSED_WINDOWS");
                    }
                }
            }
        }
        Self {
            swd: Some(swd),
            delay: Some(delay),
            buffer: Some(buffer),
            callbacks,
            reset,
            selector,
            target,
            table,
            selected: None,
            auto_reset,
            manager: None,
            ranges: heapless::Vec::new(),
            ram_ranges: heapless::Vec::new(),
            composed_windows,
            arbiter,
            session: None,
            sector_erase: true,
            deferred: Deferred::None,
            latched_finish: None,
        }
    }

    /// Read the attached DPIDR outside any session; `None` if nothing answers.
    pub fn read_dpidr(&mut self) -> Option<u32> {
        self.reset.read_dpidr()
    }

    /// Reclaim the SWD driver, delay and buffer from a finished or aborted manager.
    fn recover_from_manager(&mut self, parts: (SessionFlash<'a, S, D, C, DEPS>, &'a mut [u8])) {
        let (flash, buffer) = parts;
        let (swd, delay) = flash.into_parts();
        self.swd = Some(swd);
        self.delay = Some(delay);
        self.buffer = Some(buffer);
    }

    /// Restore the passed parts, drop any session, and report an init failure; never touches the target.
    /// No caller reaches it with a debug session still established.
    fn fail_init(
        &mut self,
        swd: Option<S>,
        delay: Option<D>,
        buffer: Option<&'a mut [u8]>,
    ) -> FlashPhase {
        self.swd = swd;
        self.delay = delay;
        self.buffer = buffer;
        self.session = None;
        FlashPhase::Init
    }

    /// Map a flash-runtime error onto the phase currency of the [`FlashBackend`] port.
    fn program_phase(err: Error) -> FlashPhase {
        match err {
            Error::EraseFailed | Error::EraseNotSupported => FlashPhase::Erase,
            // An address the target maps no flash region to; sectors resolve only at a flush.
            Error::AddressOutOfRange { .. } | Error::NoFlashRegion { .. } => FlashPhase::Range,
            Error::VerifyFailed { .. } => FlashPhase::Verify,
            _ => FlashPhase::Program,
        }
    }

    /// Map a bring-up error onto the phase currency of the [`FlashBackend`] port.
    fn init_phase(err: Error) -> FlashPhase {
        match err {
            Error::IdentityMismatch { .. } => FlashPhase::Identity,
            _ => FlashPhase::Init,
        }
    }

    /// Run every deferred step to completion, spending each deadline in the session's delay source.
    fn drive(&mut self) -> Result<(), FlashPhase> {
        // A virtual clock: the deadlines are compared only against time already spent.
        let mut now_us = 0u64;
        loop {
            match self.step(now_us) {
                FlashPoll::Done => return Ok(()),
                FlashPoll::Err(phase) => return Err(phase),
                FlashPoll::Pending { wake_at_us } => {
                    if wake_at_us > now_us {
                        let wait = (wake_at_us - now_us).min(u64::from(u32::MAX)) as u32;
                        self.deferred_delay(wait);
                        now_us = wake_at_us;
                    }
                }
            }
        }
    }

    /// Spend `us` in whichever part of the session currently owns the delay source.
    fn deferred_delay(&mut self, us: u32) {
        match &mut self.deferred {
            Deferred::Init(flash, _, _)
            | Deferred::Finish(FinishStage::Uninit(flash, _))
            | Deferred::Finish(FinishStage::PostReset(flash, _, _)) => flash.delay_us(us),
            // The manager owns the delay source once it wraps the flash driver.
            Deferred::Prepare
            | Deferred::PrepareErase(_)
            | Deferred::Program(_)
            | Deferred::Finish(FinishStage::Flush(_)) => {
                if let Some(manager) = self.manager.as_mut() {
                    manager.delay_us(us);
                }
            }
            _ => {}
        }
    }

    /// One step of the bring-up: the reset-and-halt run, then the identity gate and the manager.
    fn step_init(
        &mut self,
        mut flash: SessionFlash<'a, S, D, C, DEPS>,
        buffer: &'a mut [u8],
        mut run: Option<ResetRun>,
        now_us: u64,
    ) -> FlashPoll {
        if let Some(active) = run.as_mut() {
            match flash.advance_reset(active, now_us) {
                StepPoll::Pending { wake_at_us } => {
                    self.deferred = Deferred::Init(flash, buffer, run);
                    return FlashPoll::Pending { wake_at_us };
                }
                StepPoll::Err(err) => {
                    flash.init_failed();
                    let (swd, delay) = flash.into_parts();
                    let _ = self.fail_init(Some(swd), Some(delay), Some(buffer));
                    return FlashPoll::Err(Self::init_phase(err));
                }
                StepPoll::Done => run = None,
            }
        }
        debug_assert!(run.is_none());
        if let Err(err) = flash.init_end() {
            let (swd, delay) = flash.into_parts();
            let _ = self.fail_init(Some(swd), Some(delay), Some(buffer));
            return FlashPoll::Err(Self::init_phase(err));
        }

        // Unreachable: `new` rejects only what `page_buffer_ok` already rejected at `begin_start`.
        let Ok(mut manager) = FlashManager::new(flash, buffer) else {
            self.session = None;
            return FlashPoll::Err(FlashPhase::Init);
        };
        manager.set_erase_mode(if self.sector_erase {
            EraseMode::Sector
        } else {
            EraseMode::Chip
        });
        // Verify inline: the staging buffer still holds the expected bytes only at this moment.
        manager.set_verify(true);
        self.manager = Some(manager);
        self.deferred = Deferred::Prepare;
        FlashPoll::Pending { wake_at_us: now_us }
    }

    /// One step of the teardown: the staged pages, the algorithm uninit, the post-flash reset.
    fn step_finish(&mut self, stage: FinishStage<'a, S, D, C, DEPS>, now_us: u64) -> FlashPoll {
        match stage {
            FinishStage::Flush(None) => {
                let Some(manager) = self.manager.as_mut() else {
                    self.session = None;
                    return FlashPoll::Err(FlashPhase::Program);
                };
                match manager.flush_begin() {
                    Ok(run) => {
                        self.deferred = Deferred::Finish(FinishStage::Flush(Some(run)));
                        FlashPoll::Pending { wake_at_us: now_us }
                    }
                    Err(err) => {
                        let (flash, buffer) = self.release_torn();
                        self.deferred = Deferred::Finish(FinishStage::Uninit(flash, buffer));
                        // The teardown still runs; the phase is reported once it has.
                        self.latched_finish = Some(Self::program_phase(err));
                        FlashPoll::Pending { wake_at_us: now_us }
                    }
                }
            }
            FinishStage::Flush(Some(mut run)) => {
                let Some(manager) = self.manager.as_mut() else {
                    self.session = None;
                    return FlashPoll::Err(FlashPhase::Program);
                };
                match manager.advance_flush(&mut run, now_us) {
                    StepPoll::Pending { wake_at_us } => {
                        self.deferred = Deferred::Finish(FinishStage::Flush(Some(run)));
                        FlashPoll::Pending { wake_at_us }
                    }
                    StepPoll::Done => {
                        let (flash, buffer) = self.manager.take().expect("checked above").release();
                        self.deferred = Deferred::Finish(FinishStage::Uninit(flash, buffer));
                        FlashPoll::Pending { wake_at_us: now_us }
                    }
                    StepPoll::Err(err) => {
                        let (flash, buffer) = self.release_torn();
                        self.deferred = Deferred::Finish(FinishStage::Uninit(flash, buffer));
                        // The teardown still runs; the phase is reported once it has.
                        self.latched_finish = Some(Self::program_phase(err));
                        FlashPoll::Pending { wake_at_us: now_us }
                    }
                }
            }
            FinishStage::Uninit(mut flash, buffer) => match flash.uninit_begin() {
                Some(run) => {
                    self.deferred = Deferred::Finish(FinishStage::PostReset(flash, buffer, run));
                    FlashPoll::Pending { wake_at_us: now_us }
                }
                None => self.end_session(flash, buffer),
            },
            FinishStage::PostReset(mut flash, buffer, mut run) => {
                match flash.advance_reset(&mut run, now_us) {
                    StepPoll::Pending { wake_at_us } => {
                        self.deferred =
                            Deferred::Finish(FinishStage::PostReset(flash, buffer, run));
                        FlashPoll::Pending { wake_at_us }
                    }
                    // Best-effort: flashing already completed, so a failed reset must not fail it.
                    StepPoll::Done | StepPoll::Err(_) => self.end_session(flash, buffer),
                }
            }
        }
    }

    /// Release the manager after a failed closing flush: a torn image is left halted, not run.
    fn release_torn(&mut self) -> (SessionFlash<'a, S, D, C, DEPS>, &'a mut [u8]) {
        let manager = self.manager.take().expect("checked above");
        let torn = manager.erased_any();
        let (mut flash, buffer) = manager.release();
        if torn {
            flash.set_post_flash_reset(false);
        }
        (flash, buffer)
    }

    /// Fail the opening like any `begin` failure: abort inside the claim, then release it.
    fn fail_prepare(&mut self, err: Error) -> FlashPoll {
        if let Some(manager) = self.manager.take() {
            let parts = manager.abort();
            self.recover_from_manager(parts);
        }
        self.session = None;
        FlashPoll::Err(Self::program_phase(err))
    }

    /// Close the session out: mark the driver torn down, reclaim the lent parts, drop the claim.
    fn end_session(
        &mut self,
        mut flash: SessionFlash<'a, S, D, C, DEPS>,
        buffer: &'a mut [u8],
    ) -> FlashPoll {
        flash.uninit_end();
        self.recover_from_manager((flash, buffer));
        self.session = None;
        match self.latched_finish.take() {
            Some(phase) => FlashPoll::Err(phase),
            None => FlashPoll::Done,
        }
    }

    /// Claim the bus, resolve a target and cache its ranges; a rejection releases the claim first.
    fn claim_and_select(&mut self, family: Option<u32>) -> Result<&'a dyn Target, FlashPhase> {
        self.selected = None;
        self.ranges.clear();
        self.ram_ranges.clear();
        let Some(session) = self.arbiter.begin_session() else {
            return Err(FlashPhase::NoTarget);
        };
        self.session = Some(session);

        let hint = match family {
            Some(f) => ImageHint::uf2(f),
            None => ImageHint::NONE,
        };
        let selected = match self.selector.select(hint, self.table) {
            Ok(target) => target,
            Err(err) => {
                self.session = None;
                return Err(match err {
                    SelectError::NoTarget => FlashPhase::NoTarget,
                    SelectError::ImageMismatch => FlashPhase::Family,
                });
            }
        };
        self.selected = Some(selected);

        self.ranges.clear();
        for region in selected.flash_regions() {
            if self.ranges.push((region.start, region.end)).is_err() {
                note_dropped_window("MAX_RANGES");
            }
        }
        self.ram_ranges.clear();
        for region in selected.ram_regions() {
            if self.ram_ranges.push((region.start, region.end)).is_err() {
                note_dropped_window("MAX_RAM_RANGES");
            }
        }

        Ok(selected)
    }
}

impl<'a, S, D, T, C, R, Sel, A, DEPS> FlashBackend for HwBackend<'a, S, D, T, C, R, Sel, A, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target,
    C: TargetCallbacks<S, D> + Clone,
    R: ResetControl + ProbeInfo,
    Sel: TargetSelector,
    A: FlashArbiter,
{
    fn begin(&mut self) -> Result<(), FlashPhase> {
        self.begin_with_hint(None)
    }

    fn begin_with_hint(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        self.begin_start(family)?;
        self.drive()
    }

    fn begin_start(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        let selected = self.claim_and_select(family)?;

        let swd = self.swd.take();
        let delay = self.delay.take();
        let buffer = self.buffer.take();
        let (swd, delay, buffer) = match (swd, delay, buffer) {
            (Some(s), Some(d), Some(b)) => (s, d, b),
            (s, d, b) => return Err(self.fail_init(s, d, b)),
        };

        // Same predicate `FlashManager::new` applies, while all three parts can still be restored.
        if !daplink_flash::page_buffer_ok(buffer.len()) {
            return Err(self.fail_init(Some(swd), Some(delay), Some(buffer)));
        }

        let mut flash: SessionFlash<'a, S, D, C, DEPS> =
            TargetFlash::new(swd, delay, selected, self.callbacks.clone());
        // The boot-time policy governs teardowns with no flag; `finish_with_reset` overrides it.
        flash.set_post_flash_reset(self.auto_reset);
        let run = match flash.init_begin() {
            // `init_begin` has already driven the target back to running on its way out.
            Err(err) => {
                let (swd, delay) = flash.into_parts();
                let _ = self.fail_init(Some(swd), Some(delay), Some(buffer));
                return Err(Self::init_phase(err));
            }
            Ok(InitPhase::Already) | Ok(InitPhase::Ready) => None,
            Ok(InitPhase::Reset(run)) => Some(run),
        };
        self.deferred = Deferred::Init(flash, buffer, run);
        Ok(())
    }

    fn set_sector_erase(&mut self, sector: bool) -> Result<(), FlashPhase> {
        self.sector_erase = sector;
        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), FlashPhase> {
        // A mass-erase carries no image, so the selector runs exactly as for a hintless `begin`.
        let selected = self.claim_and_select(None)?;

        // Unlike `begin_with_hint`, a bare mass-erase never touches the staging buffer.
        let swd = self.swd.take();
        let delay = self.delay.take();
        let (swd, delay) = match (swd, delay) {
            (Some(s), Some(d)) => (s, d),
            (s, d) => {
                self.swd = s;
                self.delay = d;
                self.session = None;
                return Err(FlashPhase::Init);
            }
        };

        let mut flash: SessionFlash<'_, S, D, C, DEPS> =
            TargetFlash::new(swd, delay, selected, self.callbacks.clone());
        flash.set_post_flash_reset(self.auto_reset);
        if let Err(err) = flash.init() {
            let (swd, delay) = flash.into_parts();
            self.swd = Some(swd);
            self.delay = Some(delay);
            self.session = None;
            return Err(match err {
                Error::IdentityMismatch { .. } => FlashPhase::Identity,
                _ => FlashPhase::Init,
            });
        }

        // Tear down unconditionally, so no erase error leaves the target halted mid-algorithm.
        let erase_result = flash.erase_chip();
        let uninit_result = flash.uninit();
        let (swd, delay) = flash.into_parts();
        self.swd = Some(swd);
        self.delay = Some(delay);
        self.session = None;

        if erase_result.is_err() {
            // Every `erase_chip` failure reports the `Erase` phase, like the default it overrides.
            return Err(FlashPhase::Erase);
        }
        if uninit_result.is_err() {
            return Err(FlashPhase::Uninit);
        }
        Ok(())
    }

    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
        let mut addr = addr;
        let mut data = data;
        while !data.is_empty() {
            let taken = self.program_chunk(addr, data)?;
            self.drive()?;
            addr += taken as u32;
            data = &data[taken..];
        }
        self.drive()
    }

    fn program_chunk(&mut self, addr: u32, data: &[u8]) -> Result<usize, FlashPhase> {
        let manager = self.manager.as_mut().ok_or(FlashPhase::Program)?;
        let taken = manager.stage(addr, data).map_err(Self::program_phase)?;
        if manager.flush_due() {
            self.deferred = Deferred::Program(None);
        }
        Ok(taken)
    }

    fn step(&mut self, now_us: u64) -> FlashPoll {
        match core::mem::replace(&mut self.deferred, Deferred::None) {
            Deferred::None => FlashPoll::Done,
            Deferred::Init(flash, buffer, run) => self.step_init(flash, buffer, run, now_us),
            Deferred::Prepare => match self.manager.as_mut() {
                Some(manager) => match manager.prepare_begin() {
                    Ok(None) => FlashPoll::Done,
                    Ok(Some(run)) => {
                        self.deferred = Deferred::PrepareErase(run);
                        FlashPoll::Pending { wake_at_us: now_us }
                    }
                    Err(err) => self.fail_prepare(err),
                },
                None => FlashPoll::Err(FlashPhase::Init),
            },
            Deferred::PrepareErase(mut run) => match self.manager.as_mut() {
                Some(manager) => match manager.advance_prepare(&mut run, now_us) {
                    StepPoll::Pending { wake_at_us } => {
                        self.deferred = Deferred::PrepareErase(run);
                        FlashPoll::Pending { wake_at_us }
                    }
                    StepPoll::Done => match manager.prepare_finish(run) {
                        Ok(()) => FlashPoll::Done,
                        Err(err) => self.fail_prepare(err),
                    },
                    StepPoll::Err(err) => self.fail_prepare(err),
                },
                None => FlashPoll::Err(FlashPhase::Init),
            },
            Deferred::Program(None) => match self.manager.as_mut() {
                Some(manager) => match manager.flush_begin() {
                    Ok(run) => {
                        self.deferred = Deferred::Program(Some(run));
                        FlashPoll::Pending { wake_at_us: now_us }
                    }
                    Err(err) => FlashPoll::Err(Self::program_phase(err)),
                },
                None => FlashPoll::Err(FlashPhase::Program),
            },
            Deferred::Program(Some(mut run)) => match self.manager.as_mut() {
                Some(manager) => match manager.advance_flush(&mut run, now_us) {
                    StepPoll::Pending { wake_at_us } => {
                        self.deferred = Deferred::Program(Some(run));
                        FlashPoll::Pending { wake_at_us }
                    }
                    StepPoll::Done => FlashPoll::Done,
                    StepPoll::Err(err) => FlashPoll::Err(Self::program_phase(err)),
                },
                None => FlashPoll::Err(FlashPhase::Program),
            },
            Deferred::Finish(stage) => self.step_finish(stage, now_us),
        }
    }

    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        // No live policy supplied: fall back to the boot-time composition.
        self.finish_with_reset(success, self.auto_reset)
    }

    fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
        self.finish_start(success, reset)?;
        self.drive()
    }

    fn settle(&mut self) -> Result<(), FlashPhase> {
        self.drive()
    }

    fn finish_start(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
        self.deferred = Deferred::None;
        let manager = match self.manager.take() {
            Some(manager) => manager,
            None => {
                self.session = None;
                return Ok(());
            }
        };

        let mut manager = manager;
        if success {
            manager.set_post_flash_reset(reset);
        } else {
            // A failure flushes only what needs no erase; past the first erase the image is torn,
            // so the core stays halted instead of running it.
            manager.keep_accepted();
            if manager.erased_any() {
                manager.set_post_flash_reset(false);
            }
        }
        self.manager = Some(manager);
        self.deferred = Deferred::Finish(FinishStage::Flush(None));
        Ok(())
    }

    fn flash_ranges(&self) -> &[(u32, u32)] {
        &self.ranges
    }

    fn ram_ranges(&self) -> &[(u32, u32)] {
        &self.ram_ranges
    }

    fn candidate_windows(&self) -> &[TargetWindows] {
        &self.composed_windows
    }

    fn expected_uf2_family(&self) -> Option<u32> {
        // A multi-target composition has no fallback family before selection.
        match self.selected {
            Some(target) => target.uf2_family(),
            None if self.table.len() == 1 => self.target.uf2_family(),
            None => None,
        }
    }

    fn accepts_uf2_family(&self, family: u32) -> bool {
        self.selected
            .is_none_or(|t| t.uf2_family().is_none() || t.accepts_uf2_family(family))
    }

    fn can_select_uf2_family(&self, family: u32) -> bool {
        self.table.is_empty() || self.table.iter().any(|t| t.accepts_uf2_family(family))
    }

    fn requires_uf2_family(&self) -> bool {
        self.table.len() > 1
    }

    fn selected_target_name(&self) -> Option<&str> {
        self.selected
            .map(|t| t.name())
            .filter(|name| !name.is_empty())
    }
}

/// One staging buffer shared by backends the flash arbiter already keeps from overlapping.
pub struct StagingPool<'a> {
    slot: critical_section::Mutex<core::cell::Cell<Option<&'a mut [u8]>>>,
}

impl<'a> StagingPool<'a> {
    /// An empty pool.
    pub const fn new() -> Self {
        Self {
            slot: critical_section::Mutex::new(core::cell::Cell::new(None)),
        }
    }

    /// Return `buffer` to the pool; it holds one buffer, so this replaces any other.
    pub fn put(&self, buffer: &'a mut [u8]) {
        critical_section::with(|cs| self.slot.borrow(cs).set(Some(buffer)));
    }

    fn take(&self) -> Option<&'a mut [u8]> {
        critical_section::with(|cs| self.slot.borrow(cs).take())
    }
}

impl Default for StagingPool<'_> {
    fn default() -> Self {
        Self::new()
    }
}

/// A backend whose staging buffer can be handed over between sessions.
pub trait LendStaging<'a> {
    /// Give the backend the buffer its next session borrows.
    fn lend_staging(&mut self, buffer: &'a mut [u8]);
    /// Take the buffer back; `None` while a session holds it.
    fn reclaim_staging(&mut self) -> Option<&'a mut [u8]>;
}

/// Borrows the staging buffer from a [`StagingPool`] for each session and returns it after.
pub struct PooledFlashBackend<'a, B> {
    backend: B,
    pool: &'a StagingPool<'a>,
}

impl<'a, B: LendStaging<'a>> PooledFlashBackend<'a, B> {
    /// Wrap `backend`, moving any non-empty buffer it was built with into `pool`.
    pub fn new(mut backend: B, pool: &'a StagingPool<'a>) -> Self {
        if let Some(buffer) = backend.reclaim_staging().filter(|b| !b.is_empty()) {
            pool.put(buffer);
        }
        Self { backend, pool }
    }

    /// Lend the pooled buffer to a session about to open; it is absent while another holds it.
    fn lend(&mut self) {
        if let Some(buffer) = self.pool.take() {
            self.backend.lend_staging(buffer);
        }
    }

    /// Return the buffer once no session holds it.
    fn reclaim(&mut self) {
        if let Some(buffer) = self.backend.reclaim_staging() {
            self.pool.put(buffer);
        }
    }

    fn reclaiming<R>(&mut self, op: impl FnOnce(&mut B) -> R) -> R {
        let result = op(&mut self.backend);
        self.reclaim();
        result
    }
}

impl<'a, B: FlashBackend + LendStaging<'a>> FlashBackend for PooledFlashBackend<'a, B> {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        self.lend();
        self.reclaiming(|b| b.begin())
    }

    fn begin_with_hint(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        self.lend();
        self.reclaiming(|b| b.begin_with_hint(family))
    }

    fn begin_start(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        self.lend();
        self.reclaiming(|b| b.begin_start(family))
    }

    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
        self.reclaiming(|b| b.program(addr, data))
    }

    fn program_chunk(&mut self, addr: u32, data: &[u8]) -> Result<usize, FlashPhase> {
        self.reclaiming(|b| b.program_chunk(addr, data))
    }

    fn step(&mut self, now_us: u64) -> FlashPoll {
        self.reclaiming(|b| b.step(now_us))
    }

    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        self.reclaiming(|b| b.finish(success))
    }

    fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
        self.reclaiming(|b| b.finish_with_reset(success, reset))
    }

    fn finish_start(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
        self.reclaiming(|b| b.finish_start(success, reset))
    }

    fn settle(&mut self) -> Result<(), FlashPhase> {
        self.reclaiming(|b| b.settle())
    }

    fn set_sector_erase(&mut self, sector: bool) -> Result<(), FlashPhase> {
        self.backend.set_sector_erase(sector)
    }

    fn erase_all(&mut self) -> Result<(), FlashPhase> {
        self.backend.erase_all()
    }

    fn flash_ranges(&self) -> &[(u32, u32)] {
        self.backend.flash_ranges()
    }

    fn ram_ranges(&self) -> &[(u32, u32)] {
        self.backend.ram_ranges()
    }

    fn candidate_windows(&self) -> &[TargetWindows] {
        self.backend.candidate_windows()
    }

    fn expected_uf2_family(&self) -> Option<u32> {
        self.backend.expected_uf2_family()
    }

    fn accepts_uf2_family(&self, family: u32) -> bool {
        self.backend.accepts_uf2_family(family)
    }

    fn can_select_uf2_family(&self, family: u32) -> bool {
        self.backend.can_select_uf2_family(family)
    }

    fn requires_uf2_family(&self) -> bool {
        self.backend.requires_uf2_family()
    }

    fn selected_target_name(&self) -> Option<&str> {
        self.backend.selected_target_name()
    }
}

impl<'a, S, D, T, C, R, Sel, A, DEPS> LendStaging<'a> for HwBackend<'a, S, D, T, C, R, Sel, A, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target,
    C: TargetCallbacks<S, D> + Clone,
    R: ResetControl + ProbeInfo,
    Sel: TargetSelector,
    A: FlashArbiter,
{
    fn lend_staging(&mut self, buffer: &'a mut [u8]) {
        self.buffer = Some(buffer);
    }

    fn reclaim_staging(&mut self) -> Option<&'a mut [u8]> {
        self.buffer.take()
    }
}

#[cfg(test)]
mod fence_regression {
    extern crate std;

    use super::*;
    use core::cell::RefCell;
    use std::rc::Rc;
    use std::vec::Vec;

    type Log = Rc<RefCell<Vec<&'static str>>>;

    struct MockBackend {
        log: Log,
    }

    impl FlashBackend for MockBackend {
        fn begin(&mut self) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("begin");
            Ok(())
        }

        fn begin_with_hint(&mut self, _family: Option<u32>) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("begin_with_hint");
            Ok(())
        }

        fn program(&mut self, _addr: u32, _data: &[u8]) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("program_error");
            Err(FlashPhase::Program)
        }

        fn finish(&mut self, _success: bool) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("finish");
            Ok(())
        }

        fn finish_with_reset(&mut self, _success: bool, _reset: bool) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("finish_with_reset");
            Ok(())
        }

        fn set_sector_erase(&mut self, _sector: bool) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("set_sector_erase");
            Ok(())
        }

        fn erase_all(&mut self) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("erase_error");
            Err(FlashPhase::Erase)
        }

        fn flash_ranges(&self) -> &[(u32, u32)] {
            &[(0x0800_0000, 0x0801_0000)]
        }

        fn requires_uf2_family(&self) -> bool {
            true
        }
    }

    struct RecordingFence(Log);

    impl CommandFence for RecordingFence {
        fn finish_command(&self) {
            self.0.borrow_mut().push("fence");
        }
    }

    struct DeferredBackend {
        log: Log,
        start_error: bool,
        terminal_error: bool,
        pending: bool,
    }

    impl DeferredBackend {
        fn drive(&mut self) -> Result<(), FlashPhase> {
            assert_eq!(self.step(0), FlashPoll::Pending { wake_at_us: 13 });
            self.log.borrow_mut().push("delay:13");
            match self.step(13) {
                FlashPoll::Done => Ok(()),
                FlashPoll::Err(e) => Err(e),
                FlashPoll::Pending { .. } => panic!("unexpected extra poll"),
            }
        }
    }

    impl FlashBackend for DeferredBackend {
        fn begin(&mut self) -> Result<(), FlashPhase> {
            self.begin_start(None)?;
            self.drive()
        }
        fn begin_start(&mut self, _: Option<u32>) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("begin:start");
            if self.start_error {
                Err(FlashPhase::Init)
            } else {
                Ok(())
            }
        }
        fn program(&mut self, _: u32, _: &[u8]) -> Result<(), FlashPhase> {
            self.log.borrow_mut().push("program");
            self.drive()
        }
        fn settle(&mut self) -> Result<(), FlashPhase> {
            self.drive()
        }
        fn step(&mut self, now_us: u64) -> FlashPoll {
            if self.pending {
                assert_eq!(now_us, 0);
                self.pending = false;
                self.log.borrow_mut().push("step:pending");
                FlashPoll::Pending { wake_at_us: 13 }
            } else {
                assert_eq!(now_us, 13);
                self.pending = true;
                if self.terminal_error {
                    self.log.borrow_mut().push("step:error");
                    FlashPoll::Err(FlashPhase::Program)
                } else {
                    self.log.borrow_mut().push("step:done");
                    FlashPoll::Done
                }
            }
        }
        fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
            self.finish_with_reset(success, false)
        }
        fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
            self.finish_start(success, reset)?;
            if success {
                self.drive()
            } else {
                Ok(())
            }
        }
        fn finish_start(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
            assert!(!reset);
            self.log.borrow_mut().push(if success {
                "finish:start"
            } else {
                "abort:cleanup"
            });
            if self.start_error {
                Err(FlashPhase::Uninit)
            } else {
                Ok(())
            }
        }
        fn flash_ranges(&self) -> &[(u32, u32)] {
            &[]
        }
    }

    #[test]
    fn blocking_start_failures_still_have_an_outer_fence() {
        let log = Log::default();
        let mut backend = FencedFlashBackend::new(
            DeferredBackend {
                log: log.clone(),
                start_error: true,
                terminal_error: false,
                pending: true,
            },
            RecordingFence(log.clone()),
        );
        assert_eq!(backend.begin(), Err(FlashPhase::Init));
        assert_eq!(backend.begin_with_hint(Some(3)), Err(FlashPhase::Init));
        assert_eq!(backend.finish(true), Err(FlashPhase::Uninit));
        assert_eq!(
            backend.finish_with_reset(true, false),
            Err(FlashPhase::Uninit)
        );
        assert_eq!(
            &*log.borrow(),
            &[
                "begin:start",
                "fence",
                "begin:start",
                "fence",
                "finish:start",
                "fence",
                "finish:start",
                "fence",
            ]
        );
        std::println!("start-errors: {:?}", log.borrow());
    }

    #[test]
    fn blocking_pending_and_errors_fence_once_but_explicit_steps_fence_each_call() {
        let log = Log::default();
        let mut backend = FencedFlashBackend::new(
            DeferredBackend {
                log: log.clone(),
                start_error: false,
                terminal_error: true,
                pending: true,
            },
            RecordingFence(log.clone()),
        );
        assert_eq!(backend.program(0, &[1]), Err(FlashPhase::Program));
        assert_eq!(backend.settle(), Err(FlashPhase::Program));
        assert_eq!(backend.finish(false), Ok(()));
        assert_eq!(backend.begin_start(None), Ok(()));
        assert_eq!(backend.step(0), FlashPoll::Pending { wake_at_us: 13 });
        assert_eq!(backend.step(13), FlashPoll::Err(FlashPhase::Program));
        assert_eq!(backend.finish_start(false, false), Ok(()));
        assert_eq!(
            &*log.borrow(),
            &[
                "program",
                "step:pending",
                "delay:13",
                "step:error",
                "fence",
                "step:pending",
                "delay:13",
                "step:error",
                "fence",
                "abort:cleanup",
                "fence",
                "begin:start",
                "step:pending",
                "fence",
                "step:error",
                "fence",
                "abort:cleanup",
            ]
        );
        std::println!("pending-errors-abort: {:?}", log.borrow());
    }

    #[test]
    fn every_public_flash_operation_fences_success_and_error_returns() {
        let log = Rc::new(RefCell::new(Vec::new()));
        let mut backend = FencedFlashBackend::new(
            MockBackend { log: log.clone() },
            RecordingFence(log.clone()),
        );

        assert_eq!(backend.begin(), Ok(()));
        assert_eq!(backend.begin_with_hint(Some(1)), Ok(()));
        assert_eq!(backend.program(0x0800_0000, &[0]), Err(FlashPhase::Program));
        assert_eq!(backend.finish(true), Ok(()));
        assert_eq!(backend.finish_with_reset(true, true), Ok(()));
        assert_eq!(backend.set_sector_erase(true), Ok(()));
        assert_eq!(backend.erase_all(), Err(FlashPhase::Erase));
        assert_eq!(
            *log.borrow(),
            [
                "begin",
                "fence",
                "begin_with_hint",
                "fence",
                "program_error",
                "fence",
                "finish",
                "fence",
                "finish_with_reset",
                "fence",
                "set_sector_erase",
                "fence",
                "erase_error",
                "fence",
            ]
        );
    }

    #[test]
    fn metadata_queries_do_not_consume_a_completion_boundary() {
        let log = Rc::new(RefCell::new(Vec::new()));
        let backend = FencedFlashBackend::new(
            MockBackend { log: log.clone() },
            RecordingFence(log.clone()),
        );

        assert_eq!(backend.flash_ranges(), &[(0x0800_0000, 0x0801_0000)]);
        assert!(backend.ram_ranges().is_empty());
        assert!(backend.candidate_windows().is_empty());
        assert_eq!(backend.expected_uf2_family(), None);
        assert!(backend.requires_uf2_family());
        assert_eq!(backend.selected_target_name(), None);
        assert!(log.borrow().is_empty());
    }
}

#[cfg(test)]
mod hw_regression {
    extern crate std;

    use super::*;
    use core::cell::Cell;
    use dap_rs::swd::{APnDP, DPRegister};
    use daplink_flash::flash_algo::{
        FlashAlgorithm, FlashRegion, RamRegion, SectorInfo, SyscallParams,
    };
    use daplink_flash::target::DefaultCallbacks;
    use daplink_targets::FixedTarget;
    use std::boxed::Box;

    // DCRDR carries the algorithm status word (0 = success); every other read returns all-ones.
    const DCRDR: u32 = 0xE000_EDF8;
    // DCRSR selects which core register a DCRDR access targets.
    const DCRSR: u32 = 0xE000_EDF4;
    // Core-register write selector for r15 (PC): register number plus bit 16.
    const PC_WRITE_SELECT: u32 = 15 | (1 << 16);
    // Entry addresses of the mock algorithm's routines (algo_start + offset).
    const PROGRAM_ENTRY: u32 = 0x2000_00A0;
    const VERIFY_ENTRY: u32 = 0x2000_00C0;
    const ERASE_CHIP_ENTRY: u32 = 0x2000_0060;
    // The DHCSR write that releases the core to run: the tell-tale of a reset-to-run.
    const DHCSR: u32 = 0xE000_EDF0;
    const DHCSR_RELEASE_RUN: u32 = 0xA05F_0000;
    // The DHCSR write a syscall resume issues (DBGKEY | C_DEBUGEN): resets the halt-poll count.
    const DHCSR_RESUME: u32 = 0xA05F_0001;
    // DHCSR S_HALT bit; a probe with it clear reports the core still running the algorithm.
    const S_HALT: u32 = 1 << 17;
    const S_LOCKUP: u32 = 1 << 19;
    // Core-register read selector for r15, and where the mock algorithm's routines return to.
    const PC_READ_SELECT: u32 = 15;
    const TRAP_PC: u32 = 0x2000_0000;

    /// SWD driver mock: every poll and syscall succeeds unless a one-shot failure is armed.
    struct MockSwd {
        cfg: dap_rs::swd::Config,
        /// Current TAR, auto-incrementing on every DRW access exactly as on silicon.
        last_tar: u32,
        /// The posted read awaiting delivery by the next AP or RDBUFF read.
        posted: u32,
        fail_init: bool,
        /// Whether the last DCRSR write selected r15 for a read.
        pc_selected: bool,
        /// Last value staged into DCRDR.
        pending_dcrdr: u32,
        /// Entry address of the routine most recently loaded into the PC.
        last_entry: u32,
        fail_program: bool,
        fail_verify: bool,
        fail_erase: bool,
        /// Fail `set_reset` only once the reset-to-run phase has begun.
        fail_reset: bool,
        /// Set once DHCSR has been released to run, which only the post-flash reset does.
        run_released: bool,
        /// Optional counter of those releases, so a test can prove the target was let go.
        releases: Option<&'static Cell<u32>>,
        /// Optional counter of program-page syscalls.
        programs: Option<&'static Cell<u32>>,
        /// DHCSR probes to answer "still running" after each syscall resume before reporting halted.
        polls_before_halt: u32,
        /// Probes still owed on the syscall in flight, reloaded on every resume.
        polls_left: u32,
        /// Live-session count to check each wire access against, and the count of those outside one.
        watch: Option<(&'static Cell<i32>, &'static Cell<u32>)>,
    }

    impl MockSwd {
        fn new(fail_init: bool) -> Self {
            Self {
                cfg: dap_rs::swd::Config::default(),
                last_tar: 0,
                posted: 0,
                fail_init,
                pc_selected: false,
                pending_dcrdr: 0,
                last_entry: 0,
                fail_program: false,
                fail_verify: false,
                fail_erase: false,
                fail_reset: false,
                run_released: false,
                releases: None,
                programs: None,
                polls_before_halt: 0,
                polls_left: 0,
                watch: None,
            }
        }

        /// Count into `stray` every wire access made while `live` holds no session.
        fn watching(mut self, live: &'static Cell<i32>, stray: &'static Cell<u32>) -> Self {
            self.watch = Some((live, stray));
            self
        }

        fn note_access(&self) {
            if let Some((live, stray)) = self.watch {
                if live.get() == 0 {
                    stray.set(stray.get() + 1);
                }
            }
        }

        /// Count every release-to-run into `cell`.
        fn counting_releases(mut self, cell: &'static Cell<u32>) -> Self {
            self.releases = Some(cell);
            self
        }

        /// Count every program-page syscall into `cell`.
        fn counting_programs(mut self, cell: &'static Cell<u32>) -> Self {
            self.programs = Some(cell);
            self
        }

        /// Report the core still running for `polls` probes after each syscall resume.
        fn halting_after(mut self, polls: u32) -> Self {
            self.polls_before_halt = polls;
            self
        }

        /// Arm a one-shot ProgramPage failure.
        fn with_fail_program(mut self) -> Self {
            self.fail_program = true;
            self
        }

        /// Arm a one-shot Verify failure: the page programs cleanly and reads back wrong.
        fn with_fail_verify(mut self) -> Self {
            self.fail_verify = true;
            self
        }

        /// Arm a one-shot EraseChip failure.
        fn with_fail_erase(mut self) -> Self {
            self.fail_erase = true;
            self
        }

        /// Fail the post-flash reset-to-run nRESET assertion; the init reset-and-halt still works.
        fn with_fail_reset(mut self) -> Self {
            self.fail_reset = true;
            self
        }

        /// The word held at `addr`: DCRDR gives the algorithm's r0 status, everything else all-ones.
        fn mem(&mut self, addr: u32) -> u32 {
            if addr == DHCSR {
                // Report the core still running until the scripted probe count is spent.
                if self.polls_left > 0 {
                    self.polls_left -= 1;
                    return !(S_HALT | S_LOCKUP);
                }
                return 0xFFFF_FFFF;
            }
            if addr != DCRDR {
                return 0xFFFF_FFFF;
            }
            if self.pc_selected {
                return TRAP_PC;
            }
            if self.fail_program && self.last_entry == PROGRAM_ENTRY {
                self.fail_program = false;
                1
            } else if self.fail_verify && self.last_entry == VERIFY_ENTRY {
                self.fail_verify = false;
                1
            } else if self.fail_erase && self.last_entry == ERASE_CHIP_ENTRY {
                self.fail_erase = false;
                1
            } else {
                0
            }
        }
    }

    // `Swd<DEPS>: From<DEPS>`; the reflexive blanket impl satisfies the bound for `DEPS = MockSwd`.
    impl Swd<MockSwd> for MockSwd {
        fn available(_deps: &MockSwd) -> bool {
            true
        }
        fn config(&mut self) -> &mut dap_rs::swd::Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> dap_rs::swd::Result<u32> {
            self.note_access();
            match (apndp, a) {
                // A posted AP read returns the previous read's data and starts this one.
                (APnDP::AP, _) => {
                    let addr = self.last_tar;
                    self.last_tar = self.last_tar.wrapping_add(4);
                    let value = self.mem(addr);
                    Ok(core::mem::replace(&mut self.posted, value))
                }
                (APnDP::DP, DPRegister::RDBUFF) => Ok(self.posted),
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(
            &mut self,
            apndp: APnDP,
            a: DPRegister,
            data: u32,
        ) -> dap_rs::swd::Result<()> {
            self.note_access();
            // TAR lives at AP offset 0x04, encoded as `DPRegister::CTRLSTAT`.
            if apndp == APnDP::AP && a == DPRegister::CTRLSTAT {
                self.last_tar = data;
            }
            // DRW writes (AP offset 0x0C) carry memory data and auto-increment TAR like a read.
            if apndp == APnDP::AP && a == DPRegister::RDBUFF {
                let addr = self.last_tar;
                self.last_tar = self.last_tar.wrapping_add(4);
                if addr == DCRSR {
                    self.pc_selected = data == PC_READ_SELECT;
                }
                if addr == DCRDR {
                    self.pending_dcrdr = data;
                } else if addr == DCRSR && data == PC_WRITE_SELECT {
                    self.last_entry = self.pending_dcrdr;
                    if let (PROGRAM_ENTRY, Some(cell)) = (self.last_entry, self.programs) {
                        cell.set(cell.get() + 1);
                    }
                } else if addr == DHCSR && data == DHCSR_RELEASE_RUN {
                    self.run_released = true;
                    if let Some(cell) = self.releases {
                        cell.set(cell.get() + 1);
                    }
                } else if addr == DHCSR && data == DHCSR_RESUME {
                    // A syscall (or the reset-and-halt arm) resumed the core: reload the count.
                    self.polls_left = self.polls_before_halt;
                }
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, num_bits: usize, _data: &[u8]) -> dap_rs::swd::Result<()> {
            self.note_access();
            // The JTAG-to-SWD switch word is the 16-bit sequence in `init_debug`.
            if self.fail_init && num_bits == 16 {
                self.fail_init = false;
                return Err(dap_rs::swd::Error::NoAck);
            }
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> dap_rs::swd::Result<()> {
            Ok(())
        }
    }

    impl ResetControl for MockSwd {
        type Error = ();
        fn set_reset(&mut self, _asserted: bool) -> Result<(), ()> {
            // Fail only the post-flash reset-to-run, so the init reset-and-halt still succeeds.
            if self.fail_reset && self.run_released {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    /// Probe/reset mock used for target selection and the post-program reset pulse.
    struct MockProbe {
        fail_reset: bool,
    }

    impl ResetControl for MockProbe {
        type Error = ();
        fn set_reset(&mut self, _asserted: bool) -> Result<(), ()> {
            if self.fail_reset {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    impl ProbeInfo for MockProbe {
        fn read_dpidr(&mut self) -> Option<u32> {
            Some(0x0BC1_1477)
        }
    }

    /// Minimal single-region target: 1 KiB of flash in one sector.
    static ALGO_BLOB: [u8; 4] = [0; 4];
    static ALGO: FlashAlgorithm<'static> = FlashAlgorithm {
        init: 0x20,
        uninit: 0x40,
        erase_chip: 0x60,
        erase_sector: 0x80,
        program_page: 0xA0,
        // Present, like the RP bootrom shims: the session's verify runs on the target.
        verify: Some(0xC0),
        syscall: SyscallParams {
            breakpoint: 0x2000_0000,
            static_base: 0x2000_0100,
            stack_pointer: 0x2000_1000,
        },
        program_buffer: 0x2000_0200,
        program_buffer_size: 1024,
        algo_start: 0x2000_0000,
        algo_blob: &ALGO_BLOB,
        verify_returns_address: false,
        single_init_type: false,
        skip_chip_erase: false,
    };
    static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
        start: 0x0800_0000,
        end: 0x0800_0400,
        is_default: true,
        algo: &ALGO,
    }];
    static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
        start: 0x2000_0000,
        end: 0x2000_2000,
    }];
    static SECTORS: [SectorInfo; 1] = [SectorInfo {
        start: 0x0800_0000,
        size: 0x400,
        count: 1,
    }];

    struct MockTarget;
    impl Target for MockTarget {
        fn flash_regions(&self) -> &[FlashRegion<'_>] {
            &FLASH_REGIONS
        }
        fn ram_regions(&self) -> &[RamRegion] {
            &RAM_REGIONS
        }
        fn sectors(&self) -> &[SectorInfo] {
            &SECTORS
        }
        fn uf2_family(&self) -> Option<u32> {
            Some(0x1111_1111)
        }
    }

    // A second target at a different flash base and family, distinguishable from the composed one.
    static FLASH_REGIONS_B: [FlashRegion<'static>; 1] = [FlashRegion {
        start: 0x1000_0000,
        end: 0x1000_0400,
        is_default: true,
        algo: &ALGO,
    }];
    static SECTORS_B: [SectorInfo; 1] = [SectorInfo {
        start: 0x1000_0000,
        size: 0x400,
        count: 1,
    }];
    // Deliberately not `MockTarget`'s RAM: sharing it would hide a pooled composition.
    static RAM_REGIONS_B: [RamRegion; 1] = [RamRegion {
        start: 0x2400_0000,
        end: 0x2408_0000,
    }];

    struct SecondTarget;
    impl Target for SecondTarget {
        fn flash_regions(&self) -> &[FlashRegion<'_>] {
            &FLASH_REGIONS_B
        }
        fn ram_regions(&self) -> &[RamRegion] {
            &RAM_REGIONS_B
        }
        fn sectors(&self) -> &[SectorInfo] {
            &SECTORS_B
        }
        fn uf2_family(&self) -> Option<u32> {
            Some(0x2222_2222)
        }
    }

    // A third target whose algorithm exports no `Verify`, so its session compares by read-back.
    static ALGO_NOVERIFY: FlashAlgorithm<'static> = FlashAlgorithm {
        init: 0x20,
        uninit: 0x40,
        erase_chip: 0x60,
        erase_sector: 0x80,
        program_page: 0xA0,
        verify: None,
        syscall: SyscallParams {
            breakpoint: 0x2000_0000,
            static_base: 0x2000_0100,
            stack_pointer: 0x2000_1000,
        },
        program_buffer: 0x2000_0200,
        program_buffer_size: 1024,
        algo_start: 0x2000_0000,
        algo_blob: &ALGO_BLOB,
        verify_returns_address: false,
        single_init_type: false,
        skip_chip_erase: false,
    };
    static FLASH_REGIONS_C: [FlashRegion<'static>; 1] = [FlashRegion {
        start: 0x0900_0000,
        end: 0x0900_0400,
        is_default: true,
        algo: &ALGO_NOVERIFY,
    }];
    static SECTORS_C: [SectorInfo; 1] = [SectorInfo {
        start: 0x0900_0000,
        size: 0x400,
        count: 1,
    }];

    struct ReadbackTarget;
    impl Target for ReadbackTarget {
        fn flash_regions(&self) -> &[FlashRegion<'_>] {
            &FLASH_REGIONS_C
        }
        fn ram_regions(&self) -> &[RamRegion] {
            &RAM_REGIONS
        }
        fn sectors(&self) -> &[SectorInfo] {
            &SECTORS_C
        }
        fn uf2_family(&self) -> Option<u32> {
            Some(0x3333_3333)
        }
    }

    /// Selector that records the image hint it was handed and resolves to the first table row.
    #[derive(Clone, Copy)]
    struct HintSelector {
        seen: &'static Cell<Option<u32>>,
    }
    impl TargetSelector for HintSelector {
        fn select<'t>(
            &mut self,
            hint: ImageHint,
            table: &'t [&'t dyn Target],
        ) -> Result<&'t dyn Target, SelectError> {
            self.seen.set(hint.uf2_family);
            table.first().copied().ok_or(SelectError::NoTarget)
        }
    }

    /// The dropped image's family reaches the selector as the hint; a plain `begin` hands it `None`.
    #[test]
    fn begin_with_hint_plumbs_family_to_selector() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let seen: &'static Cell<Option<u32>> = Box::leak(Box::new(Cell::new(None)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            HintSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            HintSelector { seen },
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin_with_hint(Some(0x1111_1111)).expect("begin");
        assert_eq!(
            seen.get(),
            Some(0x1111_1111),
            "the noted UF2 family reaches the selector as the hint"
        );
        be.finish(true).expect("finish");

        be.begin().expect("begin");
        assert_eq!(seen.get(), None, "plain begin passes an empty hint");
        be.finish(true).expect("finish");
    }

    /// Selector that rejects every selection, standing in for a wrong-chip cross-check.
    #[derive(Clone, Copy, Default)]
    struct RejectSelector;
    impl TargetSelector for RejectSelector {
        fn select<'t>(
            &mut self,
            _hint: ImageHint,
            _table: &'t [&'t dyn Target],
        ) -> Result<&'t dyn Target, SelectError> {
            Err(SelectError::ImageMismatch)
        }
    }

    /// A rejected `begin` drops the just-claimed session, so DAP commands still run afterwards.
    #[test]
    fn wrong_chip_rejection_releases_session() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            RejectSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            RejectSelector,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(
            be.begin_with_hint(Some(0x9999_9999)),
            Err(FlashPhase::Family)
        );
        assert_eq!(live.get(), 0, "the rejected begin must release the session");

        // The abort path then calls finish(false) with no manager open: a clean no-op.
        assert_eq!(be.finish(false), Ok(()));
        assert_eq!(
            live.get(),
            0,
            "finish after a rejected begin holds no session"
        );
    }

    /// Selector that always returns the second table row.
    #[derive(Clone, Copy, Default)]
    struct SecondSelector;
    impl TargetSelector for SecondSelector {
        fn select<'t>(
            &mut self,
            _hint: ImageHint,
            table: &'t [&'t dyn Target],
        ) -> Result<&'t dyn Target, SelectError> {
            table.get(1).copied().ok_or(SelectError::NoTarget)
        }
    }

    /// Inert delay.
    struct MockDelay;
    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// Arbiter whose sessions increment a counter while live, so a test can assert balance.
    struct CountingArbiter {
        live: &'static Cell<i32>,
    }
    struct CountingSession {
        live: &'static Cell<i32>,
    }
    impl FlashArbiter for CountingArbiter {
        type Session = CountingSession;
        fn begin_session(&self) -> Option<CountingSession> {
            self.live.set(self.live.get() + 1);
            Some(CountingSession { live: self.live })
        }
    }
    impl Drop for CountingSession {
        fn drop(&mut self) {
            self.live.set(self.live.get() - 1);
        }
    }

    type Backend<'a> = HwBackend<
        'a,
        MockSwd,
        MockDelay,
        MockTarget,
        DefaultCallbacks,
        MockProbe,
        FixedTarget,
        CountingArbiter,
        MockSwd,
    >;

    #[allow(clippy::too_many_arguments)]
    fn backend<'a>(
        fail_init: bool,
        fail_reset: bool,
        auto_reset: bool,
        target: &'a MockTarget,
        table: &'a [&'a dyn Target],
        buffer: &'a mut [u8],
        live: &'static Cell<i32>,
    ) -> Backend<'a> {
        HwBackend::new(
            MockSwd::new(fail_init),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset },
            FixedTarget,
            target,
            table,
            buffer,
            CountingArbiter { live },
            auto_reset,
        )
    }

    /// Two full cycles: the second can only run because the first handed the parts back.
    #[test]
    fn repeatable_begin_program_finish() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, true, &target, &table, &mut buf, live);

        for _ in 0..2 {
            be.begin().expect("begin");
            assert_eq!(live.get(), 1, "session live during the cycle");
            be.program(0x0800_0000, &[0xAA; 8]).expect("program");
            be.finish(true).expect("finish");
            assert_eq!(live.get(), 0, "session released after the cycle");
        }
    }

    #[test]
    fn buffer_boundaries_are_validated_before_debug_and_restored_unchanged() {
        for len in (0..=12).chain(251..=260).chain(1019..=1028) {
            let live = Box::leak(Box::new(Cell::new(0)));
            let releases = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut storage = std::vec![0xA5; len];
            let address = storage.as_ptr();
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).counting_releases(releases),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut storage,
                CountingArbiter { live },
                true,
            );
            for entry in 0..3 {
                let result = match entry {
                    0 => be.begin(),
                    1 => be.begin_with_hint(Some(0x1111_1111)),
                    _ => be.begin_start(None),
                };
                if len >= 4 && len % 4 == 0 {
                    result.expect("valid buffer");
                    be.settle().expect("finish opening");
                    assert!(be.manager.is_some());
                    be.finish(true).expect("finish");
                    assert_eq!(releases.get(), entry + 1);
                } else {
                    assert_eq!(result, Err(FlashPhase::Init), "length {len}");
                    assert!(be.manager.is_none());
                    assert!(matches!(be.deferred, Deferred::None));
                    assert_eq!(be.swd.as_ref().unwrap().last_tar, 0);
                    assert_eq!(releases.get(), 0);
                }
                assert_eq!(live.get(), 0);
                assert!(be.swd.is_some() && be.delay.is_some());
                let restored = be.buffer.as_ref().expect("buffer restored");
                assert_eq!(restored.len(), len);
                assert_eq!(restored.as_ptr(), address);
                assert!(restored.iter().all(|b| *b == 0xA5));
            }
        }
    }

    #[test]
    fn deferred_init_retains_the_exact_validated_slice_until_manager_construction() {
        for len in [4, 8, 12, 252, 256, 260, 1020, 1024, 1028] {
            let live = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut storage = std::vec![0xA5; len];
            let address = storage.as_ptr();
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).halting_after(3),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut storage,
                CountingArbiter { live },
                false,
            );
            for _ in 0..2 {
                be.begin_start(Some(0x1111_1111)).expect("validated begin");
                let mut now_us = 0;
                let mut init_steps = 0;
                for _ in 0..100 {
                    if let Deferred::Init(_, buffer, _) = &be.deferred {
                        init_steps += 1;
                        assert_eq!(buffer.as_ptr(), address);
                        assert_eq!(buffer.len(), len);
                        assert!(daplink_flash::page_buffer_ok(buffer.len()));
                        assert!(buffer.iter().all(|b| *b == 0xA5));
                    }
                    match be.step(now_us) {
                        FlashPoll::Pending { wake_at_us } => now_us = wake_at_us,
                        FlashPoll::Done => break,
                        FlashPoll::Err(phase) => panic!("valid length {len}: {phase:?}"),
                    }
                }
                assert!(init_steps > 1, "reset-and-halt must actually defer");
                assert!(be.manager.is_some());
                assert!(matches!(be.deferred, Deferred::None));
                assert_eq!(live.get(), 1);
                be.finish(true).expect("restore parts");
                let restored = be.buffer.as_ref().unwrap();
                assert_eq!((restored.len(), restored.as_ptr()), (len, address));
                assert_eq!(live.get(), 0);
            }
        }
    }

    #[derive(Clone)]
    struct InitCallbacks {
        handled: bool,
        fail_end: &'static Cell<bool>,
    }

    impl TargetCallbacks<MockSwd, MockDelay> for InitCallbacks {
        fn set_target_state(
            &self,
            _swd: &mut MockSwd,
            _delay: &mut MockDelay,
            _state: daplink_flash::target::TargetState,
        ) -> Result<daplink_flash::target::StateOutcome, Error> {
            Ok(if self.handled {
                daplink_flash::target::StateOutcome::Handled
            } else {
                daplink_flash::target::StateOutcome::NotHandled
            })
        }

        fn prerun_config(&self, _swd: &mut MockSwd, _delay: &mut MockDelay) -> Result<(), Error> {
            if self.fail_end.replace(false) {
                Err(Error::DebugInitFailed)
            } else {
                Ok(())
            }
        }
    }

    #[test]
    fn ready_and_reset_init_end_failures_restore_parts_before_retrying_construction() {
        for handled in [false, true] {
            let live = Box::leak(Box::new(Cell::new(0)));
            let fail_end = Box::leak(Box::new(Cell::new(true)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buffer = [0xA5; 4];
            let address = buffer.as_ptr();
            let mut be: HwBackend<_, _, _, _, _, _, _, MockSwd> = HwBackend::new(
                MockSwd::new(false),
                MockDelay,
                InitCallbacks { handled, fail_end },
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buffer,
                CountingArbiter { live },
                false,
            );
            be.begin_start(None).unwrap();
            match &be.deferred {
                Deferred::Init(_, slice, run) => {
                    assert_eq!((slice.len(), slice.as_ptr()), (4, address));
                    assert_eq!(run.is_none(), handled);
                }
                _ => panic!("init parts not deferred"),
            }
            assert_eq!(be.settle(), Err(FlashPhase::Init));
            assert!(be.manager.is_none());
            assert!(be.swd.is_some() && be.delay.is_some());
            let restored = be.buffer.as_ref().unwrap();
            assert_eq!((restored.len(), restored.as_ptr()), (4, address));
            assert_eq!(live.get(), 0);
            be.begin_start(None).unwrap();
            be.settle().unwrap();
            assert!(be.manager.is_some());
            be.finish(true).unwrap();
            assert_eq!(live.get(), 0);
            assert_eq!(be.buffer.as_ref().unwrap().as_ptr(), address);
        }
    }

    #[test]
    fn owning_sink_releases_failed_and_successful_sessions_once_and_can_reopen() {
        use daplink_vfs::decoder_set::{AllDecoders, DecoderSet};
        use daplink_vfs::transfer::{TransferOutcome, TransferSink};
        use daplink_vfs::{FlashSink, SectorPoll};

        let format = AllDecoders::ext_format(b"UF2").unwrap();

        for failure in ["begin", "prepare", "program", "finish", "range", "none"] {
            let live = Box::leak(Box::new(Cell::new(0)));
            let releases = Box::leak(Box::new(Cell::new(0)));
            let stray = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buffer = [0xA5; 256];
            let swd = MockSwd::new(failure == "begin")
                .counting_releases(releases)
                .watching(live, stray)
                .halting_after(2);
            let swd = match failure {
                "prepare" => swd.with_fail_erase(),
                "program" | "finish" => swd.with_fail_program(),
                _ => swd,
            };
            let be: Backend<'_> = HwBackend::new(
                swd,
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buffer,
                CountingArbiter { live },
                true,
            );
            let mut sink = FlashSink::new(be);
            sink.set_deferred(true);
            if failure == "prepare" {
                assert!(sink.set_sector_erase(false));
            }
            sink.begin(format, false);
            let addr = if failure == "range" {
                0x2000_0000
            } else {
                0x0800_0000
            };
            let data = [0xCC; 256];
            let len = if failure == "program" { 256 } else { 4 };
            sink.fragment(addr, &data[..len]).unwrap();
            let mut now_us = 0;
            for _ in 0..1000 {
                if sink.advance(now_us) == SectorPoll::Ready {
                    break;
                }
                now_us = sink.deferred_wake_us().unwrap_or(now_us + 10);
            }
            sink.end(TransferOutcome::Done);
            sink.end(TransferOutcome::Timeout);
            for _ in 0..1000 {
                if sink.advance(now_us) == SectorPoll::Ready {
                    break;
                }
                now_us = sink.deferred_wake_us().unwrap_or(now_us + 10);
            }
            let expected = match failure {
                "begin" => TransferOutcome::FlashError(FlashPhase::Init),
                "prepare" => TransferOutcome::FlashError(FlashPhase::Erase),
                "program" | "finish" => TransferOutcome::FlashError(FlashPhase::Program),
                "range" => TransferOutcome::FlashError(FlashPhase::Range),
                _ => TransferOutcome::Done,
            };
            assert_eq!(sink.outcome(), Some(expected), "{failure}");
            // A failure after the sector erase leaves the core halted; earlier ones still reset.
            let released = u32::from(!matches!(failure, "program" | "finish"));
            assert_eq!(
                (live.get(), stray.get(), releases.get()),
                (0, 0, released),
                "{failure}"
            );
            assert!(sink.backend().swd.is_some() && sink.backend().delay.is_some());
            assert_eq!(sink.backend().buffer.as_ref().unwrap().len(), 256);
            sink.end(TransferOutcome::Done);
            assert_eq!(sink.outcome(), Some(expected));
            assert_eq!(releases.get(), released);

            assert!(sink.set_sector_erase(true));
            sink.begin(format, false);
            sink.fragment(0x0800_0000, &[0xDD; 4]).unwrap();
            sink.end(TransferOutcome::Done);
            for _ in 0..1000 {
                if sink.advance(now_us) == SectorPoll::Ready {
                    break;
                }
                now_us = sink.deferred_wake_us().unwrap_or(now_us + 10);
            }
            assert_eq!(
                sink.outcome(),
                Some(TransferOutcome::Done),
                "reopen after {failure}"
            );
            assert_eq!(
                (live.get(), stray.get(), releases.get()),
                (0, 0, released + 1)
            );
        }
    }

    /// Erase, program and verify each park once per completion probe before the flush reports `Done`.
    #[test]
    fn deferred_program_parks_once_per_completion_poll() {
        const N: u32 = 4;
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).halting_after(N),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        let page = [0xAAu8; 256];
        let taken = be.program_chunk(0x0800_0000, &page).expect("stage");
        assert_eq!(taken, 256, "the whole page stages");

        let mut now_us = 0u64;
        let mut pendings = 0u32;
        loop {
            match be.step(now_us) {
                FlashPoll::Done => break,
                FlashPoll::Err(phase) => panic!("flush failed in {phase:?}"),
                FlashPoll::Pending { wake_at_us } => {
                    pendings += 1;
                    if wake_at_us > now_us {
                        now_us = wake_at_us;
                    }
                    assert!(pendings < 10_000, "the flush must terminate");
                }
            }
        }
        // Each operation parks about N times; one full poll set per operation is the lower bound.
        assert!(
            pendings >= 3 * N,
            "each of erase/program/verify parked once per poll: {pendings} pendings"
        );

        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0, "session released after the cycle");
    }

    /// A partial page staged at `finish_start` flushes at teardown as parked `Pending` steps.
    #[test]
    fn deferred_finish_flush_parks_once_per_completion_poll() {
        const N: u32 = 5;
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).halting_after(N),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        // A partial page never trips `flush_due`, so it stays staged for the teardown flush.
        let taken = be.program_chunk(0x0800_0000, &[0xAAu8; 8]).expect("stage");
        assert_eq!(
            taken, 8,
            "the partial page stages without a mid-stream flush"
        );

        be.finish_start(true, false).expect("finish_start");
        let mut now_us = 0u64;
        let mut pendings = 0u32;
        loop {
            match be.step(now_us) {
                FlashPoll::Done => break,
                FlashPoll::Err(phase) => panic!("teardown failed in {phase:?}"),
                FlashPoll::Pending { wake_at_us } => {
                    pendings += 1;
                    if wake_at_us > now_us {
                        now_us = wake_at_us;
                    }
                    assert!(pendings < 10_000, "the teardown must terminate");
                }
            }
        }
        // The teardown erase and program each park N times, so it never blocked through a syscall.
        assert!(
            pendings >= N,
            "the teardown flush parked once per completion probe: {pendings} pendings"
        );
        assert_eq!(live.get(), 0, "session released after the teardown");
    }

    /// The presence gate reaches the probe handle without opening a session.
    #[test]
    fn read_dpidr_reaches_the_probe_handle() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, true, &target, &table, &mut buf, live);

        assert_eq!(be.read_dpidr(), Some(0x0BC1_1477));
        assert_eq!(live.get(), 0, "the presence gate opens no session");
    }

    /// A session that programs nothing still releases the target `begin` halted.
    #[test]
    fn session_that_programs_nothing_releases_the_target() {
        for success in [true, false] {
            let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
            let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0u8; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).counting_releases(released),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                true,
            );

            be.begin().expect("begin");
            assert_eq!(released.get(), 0, "the core stays halted for the session");
            be.finish(success).expect("finish");
            assert_eq!(
                released.get(),
                1,
                "a session that programmed nothing still releases the core"
            );
            assert_eq!(live.get(), 0, "and holds no arbiter session afterwards");
        }
    }

    #[test]
    fn finish_uses_both_constructor_reset_policies() {
        for (auto_reset, success, expected_releases) in [
            (false, true, 0),
            (true, true, 1),
            (false, false, 0),
            (true, false, 1),
        ] {
            let live = Box::leak(Box::new(Cell::new(0)));
            let released = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).counting_releases(released),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                auto_reset,
            );
            be.begin().expect("begin");
            assert_eq!((live.get(), released.get()), (1, 0));
            be.finish(success).expect("finish");
            assert_eq!((live.get(), released.get()), (0, expected_releases));
            std::println!(
                "auto_reset={auto_reset}, success={success}: session={}, releases={}",
                live.get(),
                released.get()
            );
        }
    }

    /// The live reset policy overrides the composed default, leaving the core halted.
    #[test]
    fn live_reset_policy_overrides_the_composed_default() {
        for reset in [false, true] {
            let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
            let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0u8; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).counting_releases(released),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                true,
            );

            be.begin().expect("begin");
            be.program(0x0800_0000, &[0xCC; 8]).expect("program");
            be.finish_with_reset(true, reset).expect("finish");
            assert_eq!(
                released.get(),
                u32::from(reset),
                "the target is released only when the live policy asks for it"
            );
            assert_eq!(live.get(), 0, "and the session is released either way");
        }
    }

    /// A failed transfer still releases the target, whatever the reset argument says.
    #[test]
    fn failed_transfer_ignores_the_reset_argument_and_releases() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).counting_releases(released),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.finish_with_reset(false, false).expect("abort");
        assert_eq!(released.get(), 1, "an aborted session still releases");
        assert_eq!(live.get(), 0);
    }

    /// Once a sector is erased, a failed transfer leaves the core halted rather than running a torn image.
    #[test]
    fn a_failed_transfer_after_an_erase_leaves_the_core_halted() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).counting_releases(released),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.program(0x0800_0000, &[0x11; 256])
            .expect("a full page erases its sector");
        be.finish_with_reset(false, true).expect("abort");
        assert_eq!(released.get(), 0, "a torn image is not run");
        assert_eq!(live.get(), 0, "the session is released");
    }

    /// A failed connect releases the session, and the next `begin` succeeds.
    #[test]
    fn failed_init_then_retry_succeeds() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(true, false, false, &target, &table, &mut buf, live);

        assert_eq!(be.begin(), Err(FlashPhase::Init));
        assert_eq!(live.get(), 0, "failed begin releases the session");

        be.begin().expect("retry begin");
        assert_eq!(live.get(), 1);
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// An aborted session hands the parts back and can be reused.
    #[test]
    fn abort_then_reuse() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, false, &target, &table, &mut buf, live);

        be.begin().expect("begin");
        be.program(0x0800_0000, &[0x11; 4]).expect("program");
        be.finish(false).expect("abort");
        assert_eq!(live.get(), 0, "abort releases the session");

        be.begin().expect("reuse begin");
        assert_eq!(live.get(), 1);
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// Refuses a second session while one is live, as the mux does.
    struct ExclusiveArbiter {
        live: &'static Cell<i32>,
    }
    impl FlashArbiter for ExclusiveArbiter {
        type Session = CountingSession;
        fn begin_session(&self) -> Option<CountingSession> {
            (self.live.get() == 0).then(|| {
                self.live.set(1);
                CountingSession { live: self.live }
            })
        }
    }

    type Exclusive<'a> = HwBackend<
        'a,
        MockSwd,
        MockDelay,
        MockTarget,
        DefaultCallbacks,
        MockProbe,
        FixedTarget,
        ExclusiveArbiter,
        MockSwd,
    >;

    fn pooled<'a>(
        fail_init: bool,
        target: &'a MockTarget,
        table: &'a [&'a dyn Target],
        pool: &'a StagingPool<'a>,
        live: &'static Cell<i32>,
    ) -> PooledFlashBackend<'a, Exclusive<'a>> {
        let backend = HwBackend::new(
            MockSwd::new(fail_init),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            target,
            table,
            &mut [],
            ExclusiveArbiter { live },
            false,
        );
        PooledFlashBackend::new(backend, pool)
    }

    /// The length of the buffer the pool holds, left in place.
    fn pooled_len(pool: &StagingPool<'_>) -> Option<usize> {
        let buffer = pool.take()?;
        let len = buffer.len();
        pool.put(buffer);
        Some(len)
    }

    /// Two backends over one arbiter take turns with one pooled buffer.
    #[test]
    fn pooled_backends_take_turns_with_one_staging_buffer() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let pool = StagingPool::new();
        pool.put(&mut buf);
        let mut a = pooled(false, &target, &table, &pool, live);
        let mut b = pooled(false, &target, &table, &pool, live);

        a.begin().expect("begin");
        assert_eq!(pooled_len(&pool), None, "the session holds the buffer");
        assert_eq!(b.begin(), Err(FlashPhase::NoTarget), "busy arbiter");
        a.program(0x0800_0000, &[0xAA; 8]).expect("program");
        a.finish(true).expect("finish");
        assert_eq!(live.get(), 0);

        b.begin().expect("the buffer came back");
        b.program(0x0800_0000, &[0x55; 8]).expect("program");
        b.finish(true).expect("finish");
        assert_eq!(pooled_len(&pool), Some(256));
    }

    /// A failed bring-up hands the buffer back for the other backend.
    #[test]
    fn a_failed_pooled_begin_returns_the_staging_buffer() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let pool = StagingPool::new();
        pool.put(&mut buf);
        let mut a = pooled(true, &target, &table, &pool, live);
        let mut b = pooled(false, &target, &table, &pool, live);

        assert_eq!(a.begin(), Err(FlashPhase::Init));
        b.begin().expect("begin");
        b.finish(true).expect("finish");
        assert_eq!(pooled_len(&pool), Some(256));
    }

    /// A failed session programs its staged page only into a sector it already erased.
    #[test]
    fn abort_keeps_a_staged_page_only_in_an_erased_sector() {
        for (first, programs) in [(256usize, 2u32), (0, 0)] {
            let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
            let count: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0u8; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).counting_programs(count),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                true,
            );
            be.begin().expect("begin");
            if first > 0 {
                be.program(0x0800_0000, &[0x11; 256][..first])
                    .expect("full page");
            }
            be.program(0x0800_0000 + first as u32, &[0x22; 4])
                .expect("staged");
            be.finish(false).expect("abort");
            assert_eq!(count.get(), programs, "first page {first} bytes");
            assert_eq!(live.get(), 0, "the session is released");
        }
    }

    /// A failing post-program reset is non-fatal: `finish(true)` still reports success.
    #[test]
    fn auto_reset_failure_is_non_fatal() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_reset(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.program(0x0800_0000, &[0x22; 4]).expect("program");
        assert_eq!(
            be.finish(true),
            Ok(()),
            "reset failure must not fail finish"
        );
        assert_eq!(live.get(), 0);
    }

    /// A programming error on the final flush must not brick the backend for later sessions.
    #[test]
    fn failed_final_flush_then_reuse_succeeds() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_program(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.program(0x0800_0000, &[0xAA; 8]).expect("program");
        assert_eq!(
            be.finish(true),
            Err(FlashPhase::Program),
            "failed final flush reports a program-phase error"
        );
        assert_eq!(live.get(), 0, "failed finish still releases the session");

        be.begin().expect("reuse begin");
        assert_eq!(live.get(), 1);
        be.program(0x0800_0000, &[0xBB; 8]).expect("reuse program");
        be.finish(true).expect("reuse finish");
        assert_eq!(live.get(), 0);
    }

    /// A wrong-part image and a failed write have opposite remedies, so they get distinct phases.
    #[test]
    fn an_unmapped_address_and_a_failed_write_report_different_phases() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, true, &target, &table, &mut buf, live);
        be.begin().expect("begin");
        // 0x0900_0000 is outside this target's only region.
        be.program(0x0900_0000, &[0xAA; 8]).expect("staged");
        let unmapped = be.finish(true);
        assert_eq!(live.get(), 0);

        let live2: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target2 = MockTarget;
        let table2: [&dyn Target; 1] = [&target2];
        let mut buf2 = [0u8; 256];
        let mut be2: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_program(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target2,
            &table2,
            &mut buf2,
            CountingArbiter { live: live2 },
            true,
        );
        be2.begin().expect("begin");
        be2.program(0x0800_0000, &[0xAA; 8]).expect("staged");
        let wire_failure = be2.finish(true);
        assert_eq!(
            wire_failure,
            Err(FlashPhase::Program),
            "a write that reached the wire and failed stays a Program failure"
        );
        assert!(unmapped.is_err(), "an unmapped address must still fail");
        assert_ne!(
            unmapped, wire_failure,
            "an unmapped address must not report as a failed write"
        );
    }

    /// A page that programs cleanly and reads back wrong fails the transfer, at both flush sites.
    #[test]
    fn verify_mismatch_fails_the_transfer() {
        for fill_a_page in [false, true] {
            let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0u8; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).with_fail_verify(),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                true,
            );

            be.begin().expect("begin");
            if fill_a_page {
                assert_eq!(
                    be.program(0x0800_0000, &[0xAA; 256]),
                    Err(FlashPhase::Verify),
                    "a page-filling write reports the mismatch from program"
                );
                assert_eq!(be.finish(false), Ok(()), "the sink then aborts");
            } else {
                be.program(0x0800_0000, &[0xAA; 8]).expect("program");
                assert_eq!(
                    be.finish(true),
                    Err(FlashPhase::Verify),
                    "the final flush reports the mismatch from finish"
                );
            }
            assert_eq!(live.get(), 0, "a failed verify still releases the session");

            be.begin().expect("reuse begin");
            be.program(0x0800_0000, &[0xAA; 8]).expect("reuse program");
            be.finish(true).expect("reuse finish");
            assert_eq!(live.get(), 0);
        }
    }

    /// A target whose algorithm exports no `Verify` is verified by read-back, and fails the same.
    #[test]
    fn readback_verify_covers_a_target_without_a_verify_entry() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = ReadbackTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            ReadbackTarget,
            DefaultCallbacks,
            MockProbe,
            FixedTarget,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.program(0x0900_0000, &[0xFF; 8]).expect("program");
        be.finish(true)
            .expect("a page that reads back as written passes the read-back verify");

        be.begin().expect("begin");
        be.program(0x0900_0000, &[0xAA; 8]).expect("program");
        assert_eq!(
            be.finish(true),
            Err(FlashPhase::Verify),
            "a page that reads back wrong fails the transfer"
        );
        assert_eq!(live.get(), 0);
    }

    /// The window seams narrow from the composed union to the selected target, RAM included.
    #[test]
    fn window_seams_narrow_from_the_composed_union_to_the_selected_target() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let composed = MockTarget;
        let second = SecondTarget;
        let table: [&dyn Target; 2] = [&composed, &second];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            SecondSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            SecondSelector,
            &composed,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(
            be.candidate_windows(),
            &[
                TargetWindows {
                    flash: (0x0800_0000, 0x0800_0400),
                    ram: (0x2000_0000, 0x2000_2000),
                },
                TargetWindows {
                    flash: (0x1000_0000, 0x1000_0400),
                    ram: (0x2400_0000, 0x2408_0000),
                },
            ]
        );
        assert!(be.ram_ranges().is_empty(), "no target resolved yet");

        be.begin().expect("begin");
        assert_eq!(be.flash_ranges(), &[(0x1000_0000, 0x1000_0400)]);
        assert_eq!(be.ram_ranges(), &[(0x2400_0000, 0x2408_0000)]);
        assert_eq!(
            be.candidate_windows().len(),
            2,
            "the composed set is a fact about the firmware, not the session"
        );
        be.finish(true).expect("finish");
    }

    /// Family selection programs the non-primary entry and does not persist across transfers.
    #[test]
    fn selector_second_row_is_programmed() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let composed = MockTarget;
        let second = SecondTarget;
        let table: [&dyn Target; 2] = [&composed, &second];
        let mut buf = [0u8; 256];

        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            FixedTarget,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &composed,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );
        assert_eq!(
            be.expected_uf2_family(),
            None,
            "multiple targets have no fallback family"
        );
        assert!(be.can_select_uf2_family(0x2222_2222));
        assert!(!be.can_select_uf2_family(0x9999_9999));

        be.begin_with_hint(Some(0x2222_2222)).expect("begin");
        assert!(be.accepts_uf2_family(0x2222_2222));
        assert!(!be.accepts_uf2_family(0x1111_1111));
        assert_eq!(
            be.flash_ranges(),
            &[(0x1000_0000, 0x1000_0400)],
            "ranges follow the selected (second) target"
        );
        assert_eq!(
            be.expected_uf2_family(),
            Some(0x2222_2222),
            "expected family follows the selected (second) target"
        );

        be.program(0x1000_0000, &[0xAA; 8])
            .expect("program at the selected target's base");
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
        assert_eq!(be.begin(), Err(FlashPhase::Family));
        assert!(be.selected_target_name().is_none());
        assert!(be.flash_ranges().is_empty());
        assert_eq!(be.expected_uf2_family(), None);
    }

    #[test]
    fn multi_target_rejection_never_initializes_flash() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let second = SecondTarget;
        for (table, hint) in [
            ([&target as &dyn Target, &second as &dyn Target], None),
            (
                [&target as &dyn Target, &second as &dyn Target],
                Some(0x9999),
            ),
            (
                [&target as &dyn Target, &target as &dyn Target],
                Some(0x1111_1111),
            ),
        ] {
            let mut buf = [0xA5u8; 256];
            let mut be = backend(false, false, false, &target, &table, &mut buf, live);
            assert_eq!(be.begin_with_hint(hint), Err(FlashPhase::Family));
            assert_eq!(live.get(), 0);
            assert!(be.manager.is_none());
            let swd = be.swd.as_ref().expect("wire never lent");
            assert_eq!(swd.last_tar, 0);
            assert_eq!(swd.last_entry, 0);
            assert_eq!(be.erase_all(), Err(FlashPhase::Family));
            assert!(be
                .buffer
                .as_ref()
                .expect("buffer retained")
                .iter()
                .all(|b| *b == 0xA5));
        }
    }

    /// A mass-erase drives `EraseChip` and releases both target and session, opening no program run.
    #[test]
    fn erase_all_erases_the_chip_and_releases_the_session() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).counting_releases(released),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(be.erase_all(), Ok(()));
        assert_eq!(
            released.get(),
            1,
            "the mass-erase releases the target to run"
        );
        assert_eq!(live.get(), 0, "and holds no arbiter session afterwards");

        be.begin().expect("begin after erase_all");
        be.program(0x0800_0000, &[0xAA; 4]).expect("program");
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// A chip-erase that faults is reported in the `Erase` phase and still releases the session.
    #[test]
    fn erase_all_reports_erase_phase_on_algorithm_failure() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_erase(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(be.erase_all(), Err(FlashPhase::Erase));
        assert_eq!(
            live.get(),
            0,
            "a failed mass-erase still releases the session"
        );

        be.begin().expect("begin after failed erase_all");
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// A failed up-front chip erase tears the session down inside it, so the abort touches no wire.
    #[test]
    fn chip_erase_prepare_failure_then_abort_stays_inside_the_session() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let stray: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_erase().watching(live, stray),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );
        be.set_sector_erase(false).expect("chip erase");

        assert_eq!(be.begin(), Err(FlashPhase::Erase));
        assert_eq!(live.get(), 0, "the failed begin holds no session");
        assert_eq!(be.finish(false), Ok(()));
        assert_eq!(stray.get(), 0, "no wire access outside a session");
        assert_eq!(live.get(), 0);

        be.set_sector_erase(true).expect("sector erase");
        be.begin().expect("begin after the failed chip erase");
        be.program(0x0800_0000, &[0xAA; 4]).expect("program");
        be.finish(true).expect("finish");
        assert_eq!((live.get(), stray.get()), (0, 0));
    }

    /// After a failed prepare, `finish(false)` has no staged page to keep and nothing to tear down.
    #[test]
    fn prepare_failure_then_failed_finish_touches_nothing() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let stray: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let programs: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let releases: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false)
                .with_fail_erase()
                .watching(live, stray)
                .counting_programs(programs)
                .counting_releases(releases),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );
        be.set_sector_erase(false).expect("chip erase");

        assert_eq!(be.begin(), Err(FlashPhase::Erase));
        let released = releases.get();
        assert_eq!(be.finish(false), Ok(()));
        assert_eq!(programs.get(), 0, "nothing staged, nothing programmed");
        assert_eq!(releases.get(), released, "no second teardown");
        assert_eq!((live.get(), stray.get()), (0, 0));
    }

    /// `erase_all` releases the session on a selector rejection exactly as `begin` does.
    #[test]
    fn erase_all_wrong_chip_rejection_releases_session() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            RejectSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            RejectSelector,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(be.erase_all(), Err(FlashPhase::Family));
        assert_eq!(
            live.get(),
            0,
            "the rejected erase_all must release the session"
        );
    }
}
