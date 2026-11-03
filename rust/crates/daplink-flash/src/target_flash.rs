//! On-silicon [`FlashInterface`]: loads a target's algorithm into RAM and runs it over [`SwdHost`].

use core::marker::PhantomData;

use dap_rs::swd::Swd;
use embedded_hal::delay::DelayNs;

use crate::error::Error;
use crate::flash_algo::FlashAlgorithm;
use crate::flash_intf::FlashInterface;
use crate::reset_sequence::{ResetOps, ResetRun, StepPoll, UnlockPolicy};
use crate::swd_host::{SwdHost, SyscallCheck, SyscallRun};
use crate::target::{StateOutcome, Target, TargetCallbacks, TargetState};
use crate::ResetControl;

/// CMSIS flash-algorithm operation codes (r2 to `Init`).
const OP_ERASE: u32 = 1;
const OP_PROGRAM: u32 = 2;
const OP_VERIFY: u32 = 3;

/// What a resumable bring-up left for the caller to drive.
pub enum InitPhase {
    /// The session was already open; nothing to drive and nothing to finish.
    Already,
    /// Drive `0` to completion, then call [`TargetFlash::init_end`].
    Reset(ResetRun),
    /// The reset-and-halt is already done; call [`TargetFlash::init_end`].
    Ready,
}

/// Drives target flashing by loading and running a target's flash algorithm.
pub struct TargetFlash<'a, S, D, T: ?Sized, C, DEPS> {
    host: SwdHost<S, D, DEPS>,
    target: &'a T,
    callbacks: C,
    /// `algo_start` of the algorithm currently resident in target RAM, if any.
    loaded: Option<u32>,
    /// Whether a session is active (`init` called, `uninit` not yet).
    active: bool,
    /// Whether `uninit` drives a post-flash reset-to-run (`true`) or leaves the core halted.
    post_flash_reset: bool,
    /// Consent policy for a destructive access-port unlock, applied during `init`.
    unlock_policy: UnlockPolicy,
    _deps: PhantomData<DEPS>,
}

impl<'a, S, D, T, C, DEPS> TargetFlash<'a, S, D, T, C, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target + ?Sized,
    C: TargetCallbacks<S, D>,
{
    /// Build a driver over an SWD driver, delay, target description and callbacks.
    pub fn new(swd: S, delay: D, target: &'a T, callbacks: C) -> Self {
        Self {
            host: SwdHost::new(swd, delay),
            target,
            callbacks,
            loaded: None,
            active: false,
            post_flash_reset: true,
            unlock_policy: UnlockPolicy::Refuse,
            _deps: PhantomData,
        }
    }

    /// Select whether ending a session resets the target to run (`true`, default) or leaves it halted.
    pub fn set_post_flash_reset(&mut self, enabled: bool) {
        self.post_flash_reset = enabled;
    }

    /// Set the consent policy for a destructive access-port unlock (default [`UnlockPolicy::Refuse`]).
    pub fn set_unlock_policy(&mut self, policy: UnlockPolicy) {
        self.unlock_policy = policy;
    }

    /// Consume the driver, returning the SWD driver and delay source.
    pub fn into_parts(self) -> (S, D) {
        self.host.into_parts()
    }

    /// Begin a session: bring up debug and reset-and-halt; a failed bring-up releases the target.
    pub fn init(&mut self) -> Result<(), Error> {
        #[cfg(feature = "log")]
        log::info!("flash: init / target bring-up");
        let mut run = match self.init_begin() {
            Ok(InitPhase::Already) => return Ok(()),
            Ok(InitPhase::Ready) => None,
            Ok(InitPhase::Reset(run)) => Some(run),
            Err(e) => {
                #[cfg(feature = "defmt")]
                defmt::warn!("flash init: target bring-up failed: {}", e);
                return Err(e);
            }
        };
        if let Some(run) = run.as_mut() {
            if let Err(e) = self.drive_reset(run) {
                #[cfg(feature = "defmt")]
                defmt::warn!("flash init: target bring-up failed: {}", e);
                self.init_failed();
                return Err(e);
            }
        }
        self.init_end()
    }

    /// Drive `run` to completion inline, spending its deadlines in the host's delay source.
    fn drive_reset(&mut self, run: &mut ResetRun) -> Result<(), Error> {
        // A virtual clock: the run's deadlines are compared only against time already spent.
        let mut now_us = 0u64;
        loop {
            match self.advance_reset(run, now_us) {
                StepPoll::Done => return Ok(()),
                StepPoll::Err(e) => return Err(e),
                StepPoll::Pending { wake_at_us } => {
                    if wake_at_us > now_us {
                        let wait = (wake_at_us - now_us).min(u64::from(u32::MAX)) as u32;
                        ResetOps::delay_us(&mut self.host, wait)?;
                        now_us = wake_at_us;
                    }
                }
            }
        }
    }

    /// Open a session up to the reset-and-halt; drive the returned [`InitPhase`], then
    /// [`init_end`](Self::init_end), or [`init_failed`](Self::init_failed) on an error.
    pub fn init_begin(&mut self) -> Result<InitPhase, Error> {
        if self.active {
            return Ok(InitPhase::Already);
        }
        match self.init_connect() {
            Ok(phase) => Ok(phase),
            Err(e) => {
                self.reset_to_run();
                Err(e)
            }
        }
    }

    /// Bring-up as far as the reset-and-halt: pre-init callback, DP/AP power-up, family unlock.
    fn init_connect(&mut self) -> Result<InitPhase, Error> {
        // A host may have left nRESET asserted; best-effort, as a probe may not drive the pin.
        let _ = ResetOps::set_reset(&mut self.host, false);
        {
            let (swd, delay) = self.host.parts_mut();
            #[cfg(feature = "defmt")]
            defmt::info!("flash init: before_init_debug");
            self.callbacks.before_init_debug(swd, delay)?;
        }
        self.host.set_apsel(self.target.apsel());
        self.host.set_dp_config(self.target.debug_port());
        #[cfg(feature = "defmt")]
        defmt::info!("flash init: init_debug (DP/AP power-up)");
        self.host.init_debug()?;
        // Unlock gate runs before reset-and-halt and identity read: a locked part can't be fingerprinted.
        let sequence = self.target.reset_sequence();
        let _outcome = self
            .host
            .run_unlock_sequence(sequence, self.unlock_policy)?;
        let outcome = {
            let (swd, delay) = self.host.parts_mut();
            self.callbacks
                .set_target_state(swd, delay, TargetState::ResetProgram)?
        };
        if outcome == StateOutcome::Handled {
            return Ok(InitPhase::Ready);
        }
        let reset_type = self.target.reset_type();
        match sequence.resumable(TargetState::ResetProgram, reset_type) {
            Some(run) => Ok(InitPhase::Reset(run)),
            None => {
                // A family sequence that cannot be stepped still runs to completion inline.
                self.host
                    .run_reset_sequence(sequence, TargetState::ResetProgram, reset_type)?;
                Ok(InitPhase::Ready)
            }
        }
    }

    /// Spend `us` microseconds in the session's delay source.
    pub fn delay_us(&mut self, us: u32) {
        let _ = ResetOps::delay_us(&mut self.host, us);
    }

    /// Advance a reset state; its backend operations may block, as with [`ResetRun::advance`].
    pub fn advance_reset(&mut self, run: &mut ResetRun, now_us: u64) -> StepPoll {
        run.advance(&mut self.host, now_us)
    }

    /// Release the target after a bring-up that failed past [`init_begin`](Self::init_begin).
    pub fn init_failed(&mut self) {
        self.reset_to_run();
    }

    /// Finish bring-up after reset-and-halt: prerun config, identity check and algorithm gate.
    pub fn init_end(&mut self) -> Result<(), Error> {
        let r = self.init_check();
        if r.is_err() {
            self.reset_to_run();
        }
        r
    }

    fn init_check(&mut self) -> Result<(), Error> {
        {
            let (swd, delay) = self.host.parts_mut();
            #[cfg(feature = "defmt")]
            defmt::info!("flash init: prerun_config");
            self.callbacks.prerun_config(swd, delay)?;
        }
        // Verify identity before any algorithm is loaded or sector erased.
        let target = self.target;
        crate::identity::check_identity(target.identity_probes(), |addr| {
            self.host.read_word(addr)
        })?;
        if !self.target.flash_algorithm_available() {
            return Err(Error::AlgorithmUnavailable);
        }
        self.loaded = None;
        self.active = true;
        Ok(())
    }

    /// Begin a resumable teardown; returns the post-flash reset run, if any.
    pub fn uninit_begin(&mut self) -> Option<ResetRun> {
        if !self.active {
            return None;
        }
        self.uninit_loaded(OP_PROGRAM);
        if !self.post_flash_reset {
            return None;
        }
        let outcome = {
            let (swd, delay) = self.host.parts_mut();
            self.callbacks
                .set_target_state(swd, delay, TargetState::PostFlashReset)
        };
        if !matches!(outcome, Ok(StateOutcome::NotHandled)) {
            return None;
        }
        let sequence = self.target.reset_sequence();
        let reset_type = self.target.reset_type();
        match sequence.resumable(TargetState::PostFlashReset, reset_type) {
            Some(run) => Some(run),
            None => {
                // Best-effort: flashing already completed, so a failed reset must not fail uninit.
                let _ =
                    self.host
                        .run_reset_sequence(sequence, TargetState::PostFlashReset, reset_type);
                None
            }
        }
    }

    /// Close a session once any [`uninit_begin`](Self::uninit_begin) run has finished.
    pub fn uninit_end(&mut self) {
        self.loaded = None;
        self.active = false;
    }

    /// Drive the target back to running, best-effort.
    fn reset_to_run(&mut self) {
        let outcome = {
            let (swd, delay) = self.host.parts_mut();
            self.callbacks
                .set_target_state(swd, delay, TargetState::PostFlashReset)
        };
        if matches!(outcome, Ok(StateOutcome::NotHandled)) {
            let sequence = self.target.reset_sequence();
            let _ = self.host.run_reset_sequence(
                sequence,
                TargetState::PostFlashReset,
                self.target.reset_type(),
            );
        }
    }

    /// End the session best-effort; ignored teardown/reset errors mean `Ok` does not prove release.
    pub fn uninit(&mut self) -> Result<(), Error> {
        if !self.active {
            return Ok(());
        }
        #[cfg(feature = "log")]
        log::info!("flash: uninit (post_flash_reset={})", self.post_flash_reset);
        if let Some(mut run) = self.uninit_begin() {
            // Best-effort: flashing already completed, so a failed reset must not fail uninit.
            let _ = self.drive_reset(&mut run);
        }
        self.uninit_end();
        Ok(())
    }

    /// Drive a syscall run to completion inline and map its status through its [`SyscallCheck`].
    fn drive_syscall(&mut self, mut run: SyscallRun) -> Result<(), Error> {
        // A virtual clock: the deadlines are compared only against time already spent.
        let mut now_us = 0u64;
        loop {
            match run.advance(&mut self.host, now_us) {
                StepPoll::Done => break,
                StepPoll::Err(e) => return Err(e),
                StepPoll::Pending { wake_at_us } => {
                    if wake_at_us > now_us {
                        let wait = (wake_at_us - now_us).min(u64::from(u32::MAX)) as u32;
                        let _ = ResetOps::delay_us(&mut self.host, wait);
                        now_us = wake_at_us;
                    }
                }
            }
        }
        run.finish_checked(&mut self.host)
    }

    /// Begin a resumable erase of the sector containing `addr`.
    pub fn erase_sector_begin(&mut self, addr: u32) -> Result<SyscallRun, Error> {
        let algo = self.ensure_algo(addr, OP_ERASE)?;
        if !algo.supports_sector_erase() {
            return Err(Error::EraseNotSupported);
        }
        let entry = algo.algo_start + algo.erase_sector;
        #[cfg(feature = "log")]
        log::info!("flash: erase_sector addr={addr:#010x}");
        let run = self
            .host
            .flash_syscall_start(&algo.syscall, entry, [addr, 0, 0, 0])?;
        Ok(run.with_check(SyscallCheck::Zero(Error::EraseFailed)))
    }

    /// Erase the sector containing `addr`.
    pub fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        let run = self.erase_sector_begin(addr)?;
        self.drive_syscall(run)
    }

    /// Begin a resumable whole-chip erase.
    pub fn erase_chip_begin(&mut self) -> Result<SyscallRun, Error> {
        let region = self.default_region().ok_or(Error::AlgoMissing)?;
        let algo = *region.algo;
        self.ensure_algo(region.start, OP_ERASE)?;
        if !algo.supports_chip_erase() {
            return Err(Error::EraseNotSupported);
        }
        let entry = algo.algo_start + algo.erase_chip;
        #[cfg(feature = "log")]
        log::info!("flash: erase_chip (algo entry {entry:#010x})");
        let run = self
            .host
            .flash_syscall_start(&algo.syscall, entry, [0, 0, 0, 0])?;
        Ok(run.with_check(SyscallCheck::Zero(Error::EraseFailed)))
    }

    /// Erase the entire chip.
    pub fn erase_chip(&mut self) -> Result<(), Error> {
        let run = self.erase_chip_begin()?;
        self.drive_syscall(run)
    }

    /// Begin a resumable program of a single page of `data` at `addr`.
    pub fn program_page_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        if self.callbacks.security_bits_set(addr, data) {
            return Err(Error::SecurityBitsWouldBeSet);
        }
        let algo = self.ensure_algo(addr, OP_PROGRAM)?;
        let len = data.len() as u32;
        if len > algo.program_buffer_size {
            return Err(Error::DataTooLarge {
                size: len,
                max: algo.program_buffer_size,
            });
        }
        #[cfg(feature = "log")]
        log::info!("flash: program_page addr={addr:#010x} len={len}");
        self.host.write_memory(algo.program_buffer, data)?;
        let entry = algo.algo_start + algo.program_page;
        let buffer = algo.program_buffer;
        let run = self
            .host
            .flash_syscall_start(&algo.syscall, entry, [addr, len, buffer, 0])?;
        Ok(run.with_check(SyscallCheck::Zero(Error::WriteFailed)))
    }

    /// Program a single page of `data` at `addr`.
    pub fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        let run = self.program_page_begin(addr, data)?;
        self.drive_syscall(run)
    }

    /// Begin a resumable verify; a read-back comparison completes inline.
    pub fn verify_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        // Read-back must not (re-)init the algorithm: `Init` can disable a memory-mapped window.
        let algo = {
            let region = self
                .find_region(addr)
                .ok_or(Error::NoFlashRegion { addr })?;
            *region.algo
        };
        #[cfg(feature = "log")]
        log::info!(
            "flash: verify addr={:#010x} len={} ({})",
            addr,
            data.len(),
            if algo.verify.is_some() {
                "algo"
            } else {
                "readback"
            }
        );
        if let Some(verify_off) = algo.verify {
            // Algorithm-driven verify requires the algorithm resident and initialized.
            self.ensure_algo(addr, OP_VERIFY)?;
            let len = data.len() as u32;
            if len > algo.program_buffer_size {
                return Err(Error::DataTooLarge {
                    size: len,
                    max: algo.program_buffer_size,
                });
            }
            self.host.write_memory(algo.program_buffer, data)?;
            let entry = algo.algo_start + verify_off;
            let buffer = algo.program_buffer;
            let run =
                self.host
                    .flash_syscall_start(&algo.syscall, entry, [addr, len, buffer, 0])?;
            let check = if algo.verify_returns_address {
                // Returns addr+len on success, else the first failing address.
                SyscallCheck::VerifyAddr {
                    end: addr.wrapping_add(len),
                }
            } else {
                SyscallCheck::Verify { addr }
            };
            Ok(run.with_check(check))
        } else {
            // A resident algorithm can leave the mapped window disabled; retry once after teardown.
            match self.verify_readback(addr, data) {
                // One retry only: the teardown clears `loaded`, so this arm cannot fire again.
                Err(Error::VerifyFailed { .. }) if self.loaded.is_some() => {
                    self.uninit_loaded(OP_VERIFY);
                    self.verify_readback(addr, data)?;
                }
                result => result?,
            }
            Ok(SyscallRun::done(0))
        }
    }

    /// Verify `data` at `addr` via the algorithm's verify routine or a read-back.
    pub fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        let run = self.verify_begin(addr, data)?;
        self.drive_syscall(run)
    }

    /// Advance a run from a `*_begin` by one bounded completion probe.
    pub fn advance_syscall(&mut self, run: &mut SyscallRun, now_us: u64) -> StepPoll {
        run.advance(&mut self.host, now_us)
    }

    /// Read and interpret a completed run's status through its own [`SyscallCheck`].
    pub fn finish_syscall(&mut self, mut run: SyscallRun) -> Result<(), Error> {
        run.finish_checked(&mut self.host)
    }

    fn verify_readback(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        // Word alignment required for the block-read path.
        if !addr.is_multiple_of(4) || !data.len().is_multiple_of(4) {
            return Err(Error::AddressNotAligned { addr, alignment: 4 });
        }
        let mut failed = None;
        self.host.read_words(addr, data.len() / 4, &mut |a, got| {
            let off = (a - addr) as usize;
            let mut buf = [0u8; 4];
            buf.copy_from_slice(&data[off..off + 4]);
            if got == u32::from_le_bytes(buf) {
                return true;
            }
            failed = Some(a);
            false
        })?;
        match failed {
            Some(addr) => Err(Error::VerifyFailed { addr }),
            None => Ok(()),
        }
    }

    fn default_region(&self) -> Option<crate::flash_algo::FlashRegion<'a>> {
        let regions = self.target.flash_regions();
        regions
            .iter()
            .find(|r| r.is_default)
            .or_else(|| regions.first())
            .copied()
    }

    fn find_region(&self, addr: u32) -> Option<crate::flash_algo::FlashRegion<'a>> {
        self.target
            .flash_regions()
            .iter()
            .find(|r| r.contains(addr))
            .copied()
    }

    /// Tear down whatever algorithm is resident, best-effort, and record RAM as holding none.
    fn uninit_loaded(&mut self, op: u32) {
        let Some(start) = self.loaded else {
            return;
        };
        let resident = self
            .target
            .flash_regions()
            .iter()
            .find(|r| r.algo.algo_start == start)
            .map(|r| *r.algo);
        if let Some(algo) = resident {
            let _ = self.algo_uninit(&algo, op);
        }
        self.loaded = None;
    }

    /// Ensure the algorithm covering `addr` is resident and initialized for `op`.
    fn ensure_algo(&mut self, addr: u32, op: u32) -> Result<FlashAlgorithm<'a>, Error> {
        let region = self
            .find_region(addr)
            .ok_or(Error::NoFlashRegion { addr })?;
        let algo = *region.algo;
        if self.loaded != Some(algo.algo_start) {
            self.uninit_loaded(op);
            self.load_algorithm(&algo)?;
            self.algo_init(&algo, region.start, op)?;
            self.loaded = Some(algo.algo_start);
        }
        Ok(algo)
    }

    fn load_algorithm(&mut self, algo: &FlashAlgorithm<'a>) -> Result<(), Error> {
        // write_memory requires a word-multiple length.
        if !algo.algo_blob.len().is_multiple_of(4) {
            return Err(Error::AlgoDownloadFailed);
        }
        self.host
            .write_memory(algo.algo_start, algo.algo_blob)
            .map_err(|_| Error::AlgoDownloadFailed)
    }

    fn algo_init(&mut self, algo: &FlashAlgorithm<'a>, addr: u32, op: u32) -> Result<(), Error> {
        {
            let (swd, delay) = self.host.parts_mut();
            self.callbacks.prerun_config(swd, delay)?;
        }
        let entry = algo.algo_start + algo.init;
        let clock = self.target.flash_clock();
        let op = if algo.single_init_type {
            OP_PROGRAM
        } else {
            op
        };
        let status = self
            .host
            .flash_syscall(&algo.syscall, entry, [addr, clock, op, 0])?;
        if status != 0 {
            return Err(Error::InitFailed);
        }
        Ok(())
    }

    fn algo_uninit(&mut self, algo: &FlashAlgorithm<'a>, op: u32) -> Result<(), Error> {
        let entry = algo.algo_start + algo.uninit;
        let op = if algo.single_init_type {
            OP_PROGRAM
        } else {
            op
        };
        let status = self
            .host
            .flash_syscall(&algo.syscall, entry, [op, 0, 0, 0])?;
        if status != 0 {
            return Err(Error::UninitFailed);
        }
        Ok(())
    }
}

impl<S, D, T, C, DEPS> FlashInterface for TargetFlash<'_, S, D, T, C, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target + ?Sized,
    C: TargetCallbacks<S, D>,
{
    fn init(&mut self) -> Result<(), Error> {
        TargetFlash::init(self)
    }

    fn uninit(&mut self) -> Result<(), Error> {
        TargetFlash::uninit(self)
    }

    fn set_post_flash_reset(&mut self, enabled: bool) {
        TargetFlash::set_post_flash_reset(self, enabled)
    }

    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        TargetFlash::program_page(self, addr, data)
    }

    fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        TargetFlash::verify(self, addr, data)
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        TargetFlash::erase_sector(self, addr)
    }

    fn erase_chip(&mut self) -> Result<(), Error> {
        TargetFlash::erase_chip(self)
    }

    fn program_page_min_size(&self, addr: u32) -> u32 {
        // Page granularity is taken as the algorithm's staging-buffer size.
        self.find_region(addr)
            .map(|r| r.algo.program_buffer_size)
            .unwrap_or(0)
    }

    fn erase_sector_size(&self, addr: u32) -> Option<u32> {
        self.target
            .sectors()
            .iter()
            .find_map(|s| s.sector_size_at(addr))
    }

    fn erase_sector_base(&self, addr: u32) -> Option<u32> {
        self.target
            .sectors()
            .iter()
            .find_map(|s| s.sector_base_at(addr))
    }

    fn supports_chip_erase(&self) -> bool {
        self.default_region()
            .map(|r| r.algo.supports_chip_erase())
            .unwrap_or(false)
    }

    fn supports_sector_erase(&self) -> bool {
        self.default_region()
            .map(|r| r.algo.supports_sector_erase())
            .unwrap_or(false)
    }

    fn set_flash_algorithm(&mut self, addr: u32) -> Result<(), Error> {
        if self.active {
            self.ensure_algo(addr, OP_PROGRAM)?;
            Ok(())
        } else if self.find_region(addr).is_some() {
            Ok(())
        } else {
            Err(Error::NoFlashRegion { addr })
        }
    }

    fn erase_sector_begin(&mut self, addr: u32) -> Result<SyscallRun, Error> {
        TargetFlash::erase_sector_begin(self, addr)
    }

    fn erase_chip_begin(&mut self) -> Result<SyscallRun, Error> {
        TargetFlash::erase_chip_begin(self)
    }

    fn program_page_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        TargetFlash::program_page_begin(self, addr, data)
    }

    fn verify_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        TargetFlash::verify_begin(self, addr, data)
    }

    fn advance_syscall(&mut self, run: &mut SyscallRun, now_us: u64) -> StepPoll {
        TargetFlash::advance_syscall(self, run, now_us)
    }

    fn finish_syscall(&mut self, run: SyscallRun) -> Result<(), Error> {
        TargetFlash::finish_syscall(self, run)
    }

    fn delay_us(&mut self, us: u32) {
        TargetFlash::delay_us(self, us)
    }
}

#[cfg(test)]
mod tests {
    //! Reset-dispatch tests for the session lifecycle.

    extern crate std;

    use super::*;
    use crate::flash_algo::{FlashRegion, RamRegion, SectorInfo};
    use crate::reset_sequence::{ResetOps, ResetSequence};
    use crate::target::{ResetType, StateOutcome, Target, TargetCallbacks, TargetState};
    use core::cell::{Cell, RefCell};
    use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult, Swd};
    use std::boxed::Box;
    use std::vec::Vec;

    /// Shared config + observations for the mock target, callbacks and reset sequence.
    #[derive(Default)]
    struct Recorder {
        /// `set_reset` fails, as on a probe that cannot drive the pin.
        no_reset_pin: Cell<bool>,
        /// Callback claims the `ResetProgram` transition.
        handle_reset_program: Cell<bool>,
        /// Callback errors on the `PostFlashReset` transition.
        err_post_flash: Cell<bool>,
        /// `security_bits_set` reports the write would set security bits.
        security_bits: Cell<bool>,
        /// The target's declared reset type (defaults to `Hardware` when unset).
        reset_type: Cell<Option<ResetType>>,
        /// The target's custom sequence fails the `ResetProgram` transition.
        seq_fail_reset_program: Cell<bool>,
        /// The ordered `TargetState`s the callback was asked to drive.
        callback_states: RefCell<Vec<TargetState>>,
        /// The ordered `TargetState`s the target's custom [`ResetSequence`] was asked to drive.
        seq_states: RefCell<Vec<TargetState>>,
        /// How many times the target's custom [`ResetSequence`] ran.
        custom_seq_runs: Cell<u32>,
        /// The `ResetType` the custom sequence was last dispatched with.
        custom_seq_reset_type: Cell<Option<ResetType>>,
        /// Count of low-level bus reads/writes issued through the SWD driver.
        bus_ops: Cell<u32>,
        /// Whether the target sequence's `unlock` gate ran during `init`.
        unlock_ran: Cell<bool>,
        /// `bus_ops` count observed at the moment `unlock` ran.
        unlock_saw_bus: Cell<u32>,
        /// Each nRESET level driven, with the `bus_ops` count at that moment.
        resets: RefCell<Vec<(bool, u32)>>,
    }

    fn leak_rec() -> &'static Recorder {
        Box::leak(Box::new(Recorder::default()))
    }

    /// SWD driver mock: status reads are all-ones so polls succeed; every bus op bumps `bus_ops`.
    struct MockSwd {
        cfg: Config,
        last_tar: u32,
        rec: &'static Recorder,
    }

    impl Swd<MockSwd> for MockSwd {
        fn available(_deps: &MockSwd) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, _a: DPRegister) -> SwdResult<u32> {
            self.rec.bus_ops.set(self.rec.bus_ops.get() + 1);
            match apndp {
                // Posted AP read; value delivered by the following RDBUFF read.
                APnDP::AP => Ok(0),
                APnDP::DP => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            self.rec.bus_ops.set(self.rec.bus_ops.get() + 1);
            if apndp == APnDP::AP && a == DPRegister::CTRLSTAT {
                self.last_tar = data;
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    impl ResetControl for MockSwd {
        type Error = SwdErr;
        fn set_reset(&mut self, asserted: bool) -> Result<(), SwdErr> {
            self.rec
                .resets
                .borrow_mut()
                .push((asserted, self.rec.bus_ops.get()));
            if self.rec.no_reset_pin.get() {
                return Err(SwdErr::NoAck);
            }
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// Records each run and dispatched reset type without touching the bus.
    struct CustomSeq {
        rec: &'static Recorder,
    }
    impl ResetSequence for CustomSeq {
        fn set_target_state(
            &self,
            _ops: &mut dyn ResetOps,
            state: TargetState,
            reset_type: ResetType,
        ) -> Result<(), Error> {
            self.rec
                .custom_seq_runs
                .set(self.rec.custom_seq_runs.get() + 1);
            self.rec.custom_seq_reset_type.set(Some(reset_type));
            self.rec.seq_states.borrow_mut().push(state);
            if state == TargetState::ResetProgram && self.rec.seq_fail_reset_program.get() {
                return Err(Error::Timeout);
            }
            Ok(())
        }
    }

    /// Minimal target: no regions, configurable reset type, recording custom sequence.
    struct MockTarget {
        seq: CustomSeq,
        rec: &'static Recorder,
    }
    impl Target for MockTarget {
        fn flash_regions(&self) -> &[FlashRegion<'_>] {
            &[]
        }
        fn ram_regions(&self) -> &[RamRegion] {
            &[]
        }
        fn sectors(&self) -> &[SectorInfo] {
            &[]
        }
        fn reset_type(&self) -> ResetType {
            self.rec.reset_type.get().unwrap_or(ResetType::Hardware)
        }
        fn reset_sequence(&self) -> &dyn ResetSequence {
            &self.seq
        }
    }

    /// Recording callbacks configurable per the recorder.
    struct MockCallbacks {
        rec: &'static Recorder,
    }
    impl TargetCallbacks<MockSwd, NoDelay> for MockCallbacks {
        fn security_bits_set(&self, _addr: u32, _data: &[u8]) -> bool {
            self.rec.security_bits.get()
        }
        fn set_target_state(
            &self,
            _swd: &mut MockSwd,
            _delay: &mut NoDelay,
            state: TargetState,
        ) -> Result<StateOutcome, Error> {
            self.rec.callback_states.borrow_mut().push(state);
            match state {
                TargetState::ResetProgram if self.rec.handle_reset_program.get() => {
                    Ok(StateOutcome::Handled)
                }
                TargetState::PostFlashReset if self.rec.err_post_flash.get() => {
                    Err(Error::ResetFailed)
                }
                _ => Ok(StateOutcome::NotHandled),
            }
        }
    }

    type Flash<'a> = TargetFlash<'a, MockSwd, NoDelay, MockTarget, MockCallbacks, MockSwd>;

    fn flash<'a>(rec: &'static Recorder, target: &'a MockTarget) -> Flash<'a> {
        let swd = MockSwd {
            cfg: Config::default(),
            last_tar: 0,
            rec,
        };
        TargetFlash::new(swd, NoDelay, target, MockCallbacks { rec })
    }

    /// A callback that claims `ResetProgram` prevents the target's custom sequence from running.
    #[test]
    fn callback_handled_skips_target_sequence() {
        let rec = leak_rec();
        rec.handle_reset_program.set(true);
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);
        tf.init().expect("init");

        assert_eq!(
            rec.custom_seq_runs.get(),
            0,
            "handled: target sequence must not run"
        );
        assert_eq!(
            *rec.callback_states.borrow(),
            std::vec![TargetState::ResetProgram],
            "callback was consulted for the reset-and-halt"
        );
    }

    /// A declining callback lets the custom sequence run, dispatched with the target's reset type.
    #[test]
    fn not_handled_runs_custom_sequence_with_target_reset_type() {
        let rec = leak_rec();
        rec.reset_type.set(Some(ResetType::SoftwareSysReset));
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);
        tf.init().expect("init");

        assert_eq!(
            rec.custom_seq_runs.get(),
            1,
            "not-handled: target sequence runs"
        );
        assert_eq!(
            rec.custom_seq_reset_type.get(),
            Some(ResetType::SoftwareSysReset),
            "sequence dispatched on the target's reset type"
        );
    }

    /// A session releases nRESET before its first bus access, so a host-held reset cannot carry in.
    #[test]
    fn init_releases_nreset_before_any_bus_access() {
        let rec = leak_rec();
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);
        tf.init().expect("init");
        assert_eq!(*rec.resets.borrow(), [(false, 0)]);
    }

    #[test]
    fn init_survives_a_probe_without_an_nreset_pin() {
        let rec = leak_rec();
        rec.no_reset_pin.set(true);
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);
        tf.init().expect("init");
    }

    /// A second `init` while a session is active is a no-op.
    #[test]
    fn init_is_idempotent_while_active() {
        let rec = leak_rec();
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);
        tf.init().expect("first init");
        tf.init().expect("second init (idempotent)");

        assert_eq!(
            rec.custom_seq_runs.get(),
            1,
            "reset sequence runs exactly once"
        );
        assert_eq!(
            *rec.callback_states.borrow(),
            std::vec![TargetState::ResetProgram],
            "callback consulted once; the second init returned before re-driving state"
        );
    }

    /// A failing `PostFlashReset` callback does not fail `uninit`, and still clears `active`.
    #[test]
    fn uninit_best_effort_post_flash_reset_clears_active() {
        let rec = leak_rec();
        rec.err_post_flash.set(true);
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);
        tf.init().expect("init");
        assert_eq!(rec.custom_seq_runs.get(), 1);

        // uninit swallows the PostFlashReset error; the sequence is not additionally run.
        tf.uninit()
            .expect("uninit is best-effort and still succeeds");
        assert_eq!(
            rec.custom_seq_runs.get(),
            1,
            "errored callback is not followed by the generic sequence"
        );

        tf.init().expect("re-init after uninit");
        assert_eq!(
            rec.custom_seq_runs.get(),
            2,
            "re-init runs the reset sequence again, proving the session was released"
        );
    }

    /// A failed reset-and-halt still triggers a release, and `init` reports the original error.
    #[test]
    fn failed_reset_and_halt_releases_the_target() {
        let rec = leak_rec();
        rec.seq_fail_reset_program.set(true);
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);

        assert_eq!(tf.init().unwrap_err(), Error::Timeout, "original cause");
        assert_eq!(
            *rec.seq_states.borrow(),
            std::vec![TargetState::ResetProgram, TargetState::PostFlashReset],
            "the failed reset-and-halt is followed by a release"
        );
        tf.uninit().expect("uninit on an inactive session");
        assert_eq!(rec.seq_states.borrow().len(), 2);
    }

    /// The release also runs for a bring-up that fails after a successful reset-and-halt.
    #[test]
    fn failed_bringup_after_halt_releases_the_target() {
        struct NoAlgoTarget {
            seq: CustomSeq,
        }
        impl Target for NoAlgoTarget {
            fn flash_regions(&self) -> &[FlashRegion<'_>] {
                &[]
            }
            fn ram_regions(&self) -> &[RamRegion] {
                &[]
            }
            fn sectors(&self) -> &[SectorInfo] {
                &[]
            }
            fn reset_sequence(&self) -> &dyn ResetSequence {
                &self.seq
            }
            fn flash_algorithm_available(&self) -> bool {
                false
            }
        }

        let rec = leak_rec();
        let target = NoAlgoTarget {
            seq: CustomSeq { rec },
        };
        let swd = MockSwd {
            cfg: Config::default(),
            last_tar: 0,
            rec,
        };
        let mut tf: TargetFlash<'_, MockSwd, NoDelay, NoAlgoTarget, MockCallbacks, MockSwd> =
            TargetFlash::new(swd, NoDelay, &target, MockCallbacks { rec });

        assert_eq!(tf.init().unwrap_err(), Error::AlgorithmUnavailable);
        assert_eq!(
            *rec.seq_states.borrow(),
            std::vec![TargetState::ResetProgram, TargetState::PostFlashReset],
            "a halted core is released even though the halt itself succeeded"
        );
    }

    /// A mismatching identity probe aborts `init`; a matching one lets it complete.
    #[test]
    fn init_verifies_identity_and_aborts_on_mismatch() {
        use crate::identity::IdentityProbe;

        static MATCH: [IdentityProbe; 1] =
            [IdentityProbe::new(0xE004_2000, 0x0000_0FFF, 0x0000_0FFF)];
        static MISMATCH: [IdentityProbe; 1] =
            [IdentityProbe::new(0xE004_2000, 0x0000_0FFF, 0x0000_0410)];

        struct IdTarget {
            probes: &'static [IdentityProbe],
        }
        impl Target for IdTarget {
            fn flash_regions(&self) -> &[FlashRegion<'_>] {
                &[]
            }
            fn ram_regions(&self) -> &[RamRegion] {
                &[]
            }
            fn sectors(&self) -> &[SectorInfo] {
                &[]
            }
            fn identity_probes(&self) -> &[IdentityProbe] {
                self.probes
            }
        }

        let rec = leak_rec();
        let target = IdTarget { probes: &MATCH };
        let swd = MockSwd {
            cfg: Config::default(),
            last_tar: 0,
            rec,
        };
        let mut tf: TargetFlash<'_, MockSwd, NoDelay, IdTarget, MockCallbacks, MockSwd> =
            TargetFlash::new(swd, NoDelay, &target, MockCallbacks { rec });
        tf.init().expect("matching identity: init succeeds");

        let rec = leak_rec();
        let target = IdTarget { probes: &MISMATCH };
        let swd = MockSwd {
            cfg: Config::default(),
            last_tar: 0,
            rec,
        };
        let mut tf: TargetFlash<'_, MockSwd, NoDelay, IdTarget, MockCallbacks, MockSwd> =
            TargetFlash::new(swd, NoDelay, &target, MockCallbacks { rec });
        let err = tf.init().unwrap_err();
        assert_eq!(
            err,
            Error::IdentityMismatch {
                addr: 0xE004_2000,
                expected: 0x0000_0410,
                read: 0x0000_0FFF,
            }
        );
    }

    /// A locked access port aborts `init` before the identity check; an open one proceeds.
    #[test]
    fn init_runs_unlock_gate_before_identity() {
        use crate::reset_sequence::{UnlockOutcome, UnlockPolicy};

        /// Reports its port locked (or not) via `unlock`, recording when it ran.
        struct LockSeq {
            locked: bool,
            rec: &'static Recorder,
        }
        impl ResetSequence for LockSeq {
            fn unlock(
                &self,
                _ops: &mut dyn ResetOps,
                policy: UnlockPolicy,
            ) -> Result<UnlockOutcome, Error> {
                self.rec.unlock_ran.set(true);
                self.rec.unlock_saw_bus.set(self.rec.bus_ops.get());
                if self.locked {
                    let _ = policy;
                    return Err(Error::TargetLocked);
                }
                Ok(UnlockOutcome::NotLocked)
            }
        }

        struct LockTarget {
            seq: LockSeq,
        }
        impl Target for LockTarget {
            fn flash_regions(&self) -> &[FlashRegion<'_>] {
                &[]
            }
            fn ram_regions(&self) -> &[RamRegion] {
                &[]
            }
            fn sectors(&self) -> &[SectorInfo] {
                &[]
            }
            fn reset_sequence(&self) -> &dyn ResetSequence {
                &self.seq
            }
        }

        let rec = leak_rec();
        let target = LockTarget {
            seq: LockSeq { locked: true, rec },
        };
        let swd = MockSwd {
            cfg: Config::default(),
            last_tar: 0,
            rec,
        };
        let mut tf: TargetFlash<'_, MockSwd, NoDelay, LockTarget, MockCallbacks, MockSwd> =
            TargetFlash::new(swd, NoDelay, &target, MockCallbacks { rec });
        assert_eq!(tf.init().unwrap_err(), Error::TargetLocked);
        assert!(rec.unlock_ran.get(), "unlock gate ran");
        assert_eq!(
            *rec.callback_states.borrow(),
            std::vec![TargetState::PostFlashReset],
            "aborted before the reset-and-halt; the only transition driven is the release"
        );

        let rec = leak_rec();
        let target = LockTarget {
            seq: LockSeq { locked: false, rec },
        };
        let swd = MockSwd {
            cfg: Config::default(),
            last_tar: 0,
            rec,
        };
        let mut tf: TargetFlash<'_, MockSwd, NoDelay, LockTarget, MockCallbacks, MockSwd> =
            TargetFlash::new(swd, NoDelay, &target, MockCallbacks { rec });
        tf.init().expect("unlocked part inits");
        assert!(rec.unlock_ran.get());
    }

    /// `program_page` refuses a write that would set security bits, before any bus traffic.
    #[test]
    fn program_page_security_bits_rejected_before_bus() {
        let rec = leak_rec();
        rec.security_bits.set(true);
        let target = MockTarget {
            seq: CustomSeq { rec },
            rec,
        };
        let mut tf = flash(rec, &target);
        tf.init().expect("init");

        let before = rec.bus_ops.get();
        let err = tf.program_page(0x0800_0000, &[0xAA; 8]).unwrap_err();
        assert_eq!(err, Error::SecurityBitsWouldBeSet);
        assert_eq!(
            rec.bus_ops.get(),
            before,
            "the security-bit guard rejects before touching the bus"
        );
    }
}
