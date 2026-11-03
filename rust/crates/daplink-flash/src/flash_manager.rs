//! Page-buffering flash manager over the [`FlashInterface`] seam.

use crate::error::Error;
use crate::flash_intf::FlashInterface;
use crate::reset_sequence::StepPoll;
use crate::swd_host::SyscallRun;
use daplink_core::range;

/// Minimum acceptable page-buffer size in bytes.
const MIN_BUFFER: usize = 4;
/// Capacity of the merged erased-range table.
const MAX_ERASED_RANGES: usize = 16;

/// The error [`FlashManager::new`] reports for a page buffer of `len` bytes, if any.
fn page_buffer_error(len: usize) -> Option<Error> {
    if len < MIN_BUFFER {
        return Some(Error::BufferTooSmall {
            size: len,
            min: MIN_BUFFER,
        });
    }
    if !len.is_multiple_of(4) {
        return Some(Error::BufferTooSmall {
            size: len,
            min: (len + 3) & !3usize,
        });
    }
    None
}

/// Whether [`FlashManager::new`] accepts a page-staging buffer of `len` bytes.
pub fn page_buffer_ok(len: usize) -> bool {
    page_buffer_error(len).is_none()
}

/// How the manager erases flash before programming.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EraseMode {
    /// Erase the whole chip once, up front (falls back to sector erase if unsupported).
    Chip,
    /// Erase each sector lazily, just before its first program.
    Sector,
}

/// Staged-page erase/program/verify state from [`FlashManager::flush_begin`].
pub enum FlushRun {
    /// Erasing the not-yet-erased sectors under the staged page, up to `end`.
    Erase {
        /// The sector base currently erasing.
        base: u32,
        /// The staged page's end address; sectors are erased up to here.
        end: u32,
        /// The erase's completion poll.
        run: SyscallRun,
    },
    /// Programming the staged page.
    Program(SyscallRun),
    /// Verifying the staged page inline.
    Verify(SyscallRun),
    /// Nothing left to do.
    Done,
}

/// Fixed-capacity, allocation-free set of merged erased address ranges.
#[derive(Clone, Copy, Debug)]
struct ErasedSectorTracker {
    ranges: [(u32, u32); MAX_ERASED_RANGES],
    len: usize,
}

impl ErasedSectorTracker {
    fn new() -> Self {
        Self {
            ranges: [(0, 0); MAX_ERASED_RANGES],
            len: 0,
        }
    }

    fn is_erased(&self, addr: u32) -> bool {
        self.ranges[..self.len]
            .iter()
            .any(|&(s, e)| range::contains(s, e, addr))
    }

    /// Record `[start, end)` as erased, merging with adjacent/overlapping ranges.
    fn mark(&mut self, start: u32, end: u32) -> Result<(), Error> {
        if end <= start {
            return Ok(());
        }
        let mut start = start;
        let mut end = end;
        let mut i = 0;
        while i < self.len {
            let (s, e) = self.ranges[i];
            let disjoint = e < start || s > end;
            if disjoint {
                i += 1;
            } else {
                start = start.min(s);
                end = end.max(e);
                self.ranges[i] = self.ranges[self.len - 1];
                self.len -= 1;
            }
        }
        if self.len == MAX_ERASED_RANGES {
            return Err(Error::InvalidState);
        }
        self.ranges[self.len] = (start, end);
        self.len += 1;
        Ok(())
    }
}

/// Buffers writes into flash pages and drives erase/program through a [`FlashInterface`].
pub struct FlashManager<'b, F: FlashInterface> {
    flash: F,
    buffer: &'b mut [u8],
    erase_mode: EraseMode,
    tracker: ErasedSectorTracker,
    /// Start address of the bytes currently held in `buffer`.
    buf_addr: u32,
    /// Number of valid bytes currently held in `buffer`.
    buf_len: usize,
    /// Whether the manager has run its own lazy start-up (init, plus any up-front chip erase).
    initialized: bool,
    /// Whether a chip erase covered the whole device (skips per-sector erase).
    chip_erased: bool,
    /// Whether each page is verified inline, right after it is programmed.
    verify_each: bool,
    /// Whether the staged page must be flushed before any more bytes are staged.
    flush_due: bool,
    /// Whether a flush failed, so [`keep_accepted`](Self::keep_accepted) must not keep its page.
    faulted: bool,
}

impl<'b, F: FlashInterface> FlashManager<'b, F> {
    /// Create a manager over `flash` using `buffer` as the page-staging buffer.
    pub fn new(flash: F, buffer: &'b mut [u8]) -> Result<Self, Error> {
        if let Some(err) = page_buffer_error(buffer.len()) {
            return Err(err);
        }
        Ok(Self::over(flash, buffer))
    }

    fn over(flash: F, buffer: &'b mut [u8]) -> Self {
        Self {
            flash,
            buffer,
            erase_mode: EraseMode::Sector,
            tracker: ErasedSectorTracker::new(),
            buf_addr: 0,
            buf_len: 0,
            initialized: false,
            chip_erased: false,
            verify_each: false,
            flush_due: false,
            faulted: false,
        }
    }

    /// Set the erase strategy (only meaningful before the first write).
    pub fn set_erase_mode(&mut self, mode: EraseMode) {
        self.erase_mode = mode;
    }

    /// Enable inline per-page verification, right after each page is programmed.
    pub fn set_verify(&mut self, enabled: bool) {
        self.verify_each = enabled;
    }

    /// The current erase strategy.
    pub fn erase_mode(&self) -> EraseMode {
        self.erase_mode
    }

    /// Bytes one program covers at `addr`, capped to a power of two that fits the buffer.
    fn page_span(&self, addr: u32) -> usize {
        let cap = self.buffer.len();
        let page = self.flash.program_page_min_size(addr) as usize;
        let span = if page == 0 { cap } else { cap.min(page) };
        (1usize << span.ilog2()).max(MIN_BUFFER)
    }

    fn ensure_init(&mut self) -> Result<(), Error> {
        if self.initialized {
            return Ok(());
        }
        self.flash.init()?;
        self.initialized = true;
        if self.erase_mode == EraseMode::Chip && self.flash.supports_chip_erase() {
            self.flash.erase_chip()?;
            self.chip_erased = true;
        }
        Ok(())
    }

    /// Run the manager's lazy start-up now: the device's `init`, plus any up-front chip erase.
    pub fn prepare(&mut self) -> Result<(), Error> {
        self.ensure_init()
    }

    /// Whether a full page is staged and must be flushed before more bytes are staged.
    pub fn flush_due(&self) -> bool {
        self.flush_due
    }

    /// Stage bytes of `data` into the current page without touching the device; `0` means flush first.
    pub fn stage(&mut self, addr: u32, data: &[u8]) -> Result<usize, Error> {
        if self.flush_due || data.is_empty() {
            return Ok(0);
        }
        let span = self.page_span(addr);
        let base = addr - addr % span as u32;
        let staged_end = self.buf_addr + self.buf_len as u32;
        if self.buf_len > 0 && (base != self.buf_addr || addr < staged_end) {
            self.flush_due = true;
            return Ok(0);
        }
        if self.buf_len == 0 {
            self.buf_addr = base;
        }
        // Offset within the staged page: programs start at the page base, not the fragment address.
        let off = (addr - self.buf_addr) as usize;
        if off > span {
            return Err(Error::InvalidState);
        }
        for b in &mut self.buffer[self.buf_len..off] {
            *b = 0xFF;
        }
        let take = (span - off).min(data.len());
        self.buffer[off..off + take].copy_from_slice(&data[..take]);
        self.buf_len = off + take;
        if self.buf_len == span {
            self.flush_due = true;
        }
        Ok(take)
    }

    /// Erase and program the staged page in one device round trip; `Ok(true)` when it did work.
    pub fn flush_step(&mut self) -> Result<bool, Error> {
        self.flush_due = false;
        if self.buf_len == 0 {
            return Ok(false);
        }
        self.flush().inspect_err(|_| self.faulted = true)?;
        Ok(true)
    }

    /// Set whether the device's teardown resets the target to run.
    pub fn set_post_flash_reset(&mut self, reset: bool) {
        self.flash.set_post_flash_reset(reset);
    }

    /// Return the parts without tearing the device down; the caller owns the teardown.
    pub fn release(self) -> (F, &'b mut [u8]) {
        self.disassemble()
    }

    /// Blocking composition of [`stage`](Self::stage) and [`flush_step`](Self::flush_step).
    pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.ensure_init()?;
        let mut addr = addr;
        let mut data = data;
        while !data.is_empty() {
            if self.flush_due {
                self.flush_step()?;
            }
            let take = self.stage(addr, data)?;
            if take == 0 && !self.flush_due {
                return Err(Error::InvalidState);
            }
            addr += take as u32;
            data = &data[take..];
        }
        if self.flush_due {
            self.flush_step()?;
        }
        Ok(())
    }

    /// Erase (if needed) and program whatever is currently buffered.
    fn flush(&mut self) -> Result<(), Error> {
        // Blocking callers reach the device's blocking methods, never a run; `flush_begin` steps.
        if self.buf_len == 0 {
            return Ok(());
        }
        // Pad the tail with the erased value: the device always gets a whole page.
        let span = self.page_span(self.buf_addr);
        for b in &mut self.buffer[self.buf_len..span] {
            *b = 0xFF;
        }
        let addr = self.buf_addr;

        if !self.chip_erased {
            self.ensure_erased(addr, span as u32)?;
        }

        self.flash.program_page(addr, &self.buffer[..span])?;
        // Verify inline, while the expected bytes are still in the buffer.
        if self.verify_each {
            self.flash.verify(addr, &self.buffer[..span])?;
        }
        self.buf_len = 0;
        Ok(())
    }

    /// Erase every sector overlapping `[addr, addr+len)` that is not already erased.
    fn ensure_erased(&mut self, addr: u32, len: u32) -> Result<(), Error> {
        let end = addr + len;
        let mut a = addr;
        while a < end {
            let size = self
                .flash
                .erase_sector_size(a)
                .ok_or(Error::NoFlashRegion { addr: a })?;
            if size == 0 {
                return Err(Error::InvalidState);
            }
            // Sector boundaries are device-defined, not necessarily aligned to `a`.
            let base = self
                .flash
                .erase_sector_base(a)
                .ok_or(Error::NoFlashRegion { addr: a })?;
            if !self.tracker.is_erased(base) {
                self.flash.erase_sector(base)?;
                self.tracker.mark(base, base + size)?;
            }
            a = base + size;
        }
        Ok(())
    }

    /// The next not-yet-erased sector base in `[from, end)`.
    fn next_erase(&self, from: u32, end: u32) -> Result<Option<u32>, Error> {
        let mut a = from;
        while a < end {
            let size = self
                .flash
                .erase_sector_size(a)
                .ok_or(Error::NoFlashRegion { addr: a })?;
            if size == 0 {
                return Err(Error::InvalidState);
            }
            let base = self
                .flash
                .erase_sector_base(a)
                .ok_or(Error::NoFlashRegion { addr: a })?;
            if !self.tracker.is_erased(base) {
                return Ok(Some(base));
            }
            a = base + size;
        }
        Ok(None)
    }

    /// Begin a resumable flush of the staged page: pad it, then start its first operation.
    /// Drive the run on this manager before staging more; dropping it does not cancel the device.
    pub fn flush_begin(&mut self) -> Result<FlushRun, Error> {
        self.start_flush().inspect_err(|_| self.faulted = true)
    }

    fn start_flush(&mut self) -> Result<FlushRun, Error> {
        self.flush_due = false;
        if self.buf_len == 0 {
            return Ok(FlushRun::Done);
        }
        let span = self.page_span(self.buf_addr);
        // Pad the tail with the erased value: the device always gets a whole page.
        for b in &mut self.buffer[self.buf_len..span] {
            *b = 0xFF;
        }
        let addr = self.buf_addr;
        let end = addr + span as u32;
        if !self.chip_erased {
            if let Some(base) = self.next_erase(addr, end)? {
                let run = self.flash.erase_sector_begin(base)?;
                return Ok(FlushRun::Erase { base, end, run });
            }
        }
        let run = self.flash.program_page_begin(addr, &self.buffer[..span])?;
        Ok(FlushRun::Program(run))
    }

    /// Advance staged erase/program/verify work; backend calls may block. `Done` clears the page.
    /// `now_us` keeps one monotonic epoch across the run.
    pub fn advance_flush(&mut self, run: &mut FlushRun, now_us: u64) -> StepPoll {
        let poll = match run {
            FlushRun::Erase { run: sc, .. } | FlushRun::Program(sc) | FlushRun::Verify(sc) => {
                self.flash.advance_syscall(sc, now_us)
            }
            FlushRun::Done => return StepPoll::Done,
        };
        let poll = match poll {
            StepPoll::Pending { wake_at_us } => return StepPoll::Pending { wake_at_us },
            StepPoll::Err(e) => {
                *run = FlushRun::Done;
                StepPoll::Err(e)
            }
            StepPoll::Done => self.flush_next_op(run, now_us),
        };
        self.faulted |= matches!(poll, StepPoll::Err(_));
        poll
    }

    /// On one operation completing, interpret it and start the next stage of the flush.
    fn flush_next_op(&mut self, run: &mut FlushRun, now_us: u64) -> StepPoll {
        match core::mem::replace(run, FlushRun::Done) {
            FlushRun::Erase { base, end, run: sc } => {
                if let Err(e) = self.flash.finish_syscall(sc) {
                    return StepPoll::Err(e);
                }
                let size = match self.flash.erase_sector_size(base) {
                    Some(s) if s != 0 => s,
                    _ => return StepPoll::Err(Error::InvalidState),
                };
                if let Err(e) = self.tracker.mark(base, base + size) {
                    return StepPoll::Err(e);
                }
                match self.next_erase(base + size, end) {
                    Ok(Some(next)) => match self.flash.erase_sector_begin(next) {
                        Ok(sc) => {
                            *run = FlushRun::Erase {
                                base: next,
                                end,
                                run: sc,
                            };
                            StepPoll::Pending { wake_at_us: now_us }
                        }
                        Err(e) => StepPoll::Err(e),
                    },
                    Ok(None) => self.flush_begin_program(run, now_us),
                    Err(e) => StepPoll::Err(e),
                }
            }
            FlushRun::Program(sc) => {
                if let Err(e) = self.flash.finish_syscall(sc) {
                    return StepPoll::Err(e);
                }
                if self.verify_each {
                    let span = self.page_span(self.buf_addr);
                    let addr = self.buf_addr;
                    match self.flash.verify_begin(addr, &self.buffer[..span]) {
                        Ok(sc) => {
                            *run = FlushRun::Verify(sc);
                            StepPoll::Pending { wake_at_us: now_us }
                        }
                        Err(e) => StepPoll::Err(e),
                    }
                } else {
                    self.buf_len = 0;
                    StepPoll::Done
                }
            }
            FlushRun::Verify(sc) => {
                if let Err(e) = self.flash.finish_syscall(sc) {
                    return StepPoll::Err(e);
                }
                self.buf_len = 0;
                StepPoll::Done
            }
            FlushRun::Done => StepPoll::Done,
        }
    }

    /// Start the program of the staged page (all needed erases are done).
    fn flush_begin_program(&mut self, run: &mut FlushRun, now_us: u64) -> StepPoll {
        let span = self.page_span(self.buf_addr);
        let addr = self.buf_addr;
        match self.flash.program_page_begin(addr, &self.buffer[..span]) {
            Ok(sc) => {
                *run = FlushRun::Program(sc);
                StepPoll::Pending { wake_at_us: now_us }
            }
            Err(e) => StepPoll::Err(e),
        }
    }

    /// Begin lazy start-up resumably; returns the up-front chip erase run, if one is due.
    pub fn prepare_begin(&mut self) -> Result<Option<SyscallRun>, Error> {
        if !self.initialized {
            self.flash.init()?;
            self.initialized = true;
        }
        if self.erase_mode == EraseMode::Chip && self.flash.supports_chip_erase() {
            return Ok(Some(self.flash.erase_chip_begin()?));
        }
        Ok(None)
    }

    /// Advance a run from [`prepare_begin`](Self::prepare_begin) by one completion probe.
    pub fn advance_prepare(&mut self, run: &mut SyscallRun, now_us: u64) -> StepPoll {
        self.flash.advance_syscall(run, now_us)
    }

    /// Finish the chip erase begun by [`prepare_begin`](Self::prepare_begin).
    pub fn prepare_finish(&mut self, run: SyscallRun) -> Result<(), Error> {
        self.flash.finish_syscall(run)?;
        self.chip_erased = true;
        Ok(())
    }

    /// Spend `us` in the device's delay source, for a blocking caller driving a run's deadlines.
    pub fn delay_us(&mut self, us: u32) {
        self.flash.delay_us(us);
    }

    /// Move the wrapped flash and buffer out without running the drop-time teardown.
    fn disassemble(self) -> (F, &'b mut [u8]) {
        let this = core::mem::ManuallyDrop::new(self);
        // SAFETY: `this` is never used or dropped again, so each field is moved out exactly once.
        let flash = unsafe { core::ptr::read(&this.flash) };
        let buffer = unsafe { core::ptr::read(&this.buffer) };
        (flash, buffer)
    }

    /// Flush, apply `reset`, tear down, and return the parts on success and failure alike.
    pub fn finish(mut self, reset: bool) -> (Result<(), Error>, (F, &'b mut [u8])) {
        self.flash.set_post_flash_reset(reset);
        let flush_res = self.flush_step().map(|_| ());
        let uninit_res = self.flash.uninit();
        (flush_res.and(uninit_res), self.disassemble())
    }

    /// Whether any sector has been erased, so the target no longer holds the image it booted.
    pub fn erased_any(&self) -> bool {
        self.chip_erased || self.tracker.len > 0
    }

    /// Before a failed session's closing flush: keep the staged page only if it needs no erase
    /// and no flush has failed, so the target keeps what it accepted and loses no more sectors.
    pub fn keep_accepted(&mut self) {
        let end = self.buf_addr + self.page_span(self.buf_addr) as u32;
        let erased = self.chip_erased || matches!(self.next_erase(self.buf_addr, end), Ok(None));
        if self.faulted || !erased {
            self.buf_len = 0;
        }
        self.flush_due = false;
    }

    /// Abandon buffered data and return the parts, keeping the composed reset policy.
    pub fn abort(mut self) -> (F, &'b mut [u8]) {
        self.buf_len = 0;
        self.flush_due = false;
        let _ = self.flash.uninit();
        self.disassemble()
    }
}

impl<F: FlashInterface> Drop for FlashManager<'_, F> {
    fn drop(&mut self) {
        // Best-effort teardown if the caller neither finished nor aborted.
        let _ = self.flash.uninit();
    }
}

#[cfg(test)]
#[path = "manager_tests.rs"]
mod step_tests;
