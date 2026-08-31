//! Page-buffering flash manager over the [`FlashInterface`] seam.

use crate::error::Error;
use crate::flash_intf::FlashInterface;
use daplink_core::range;

/// Minimum acceptable page-buffer size in bytes.
const MIN_BUFFER: usize = 4;
/// Capacity of the merged erased-range table.
const MAX_ERASED_RANGES: usize = 16;

/// The page-buffer requirement as the error [`FlashManager::new`] reports, or `None` if `len` is
/// acceptable.
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

    /// Whether `addr` lies inside any recorded erased range.
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
}

impl<'b, F: FlashInterface> FlashManager<'b, F> {
    /// Create a manager over `flash` using `buffer` as the page-staging buffer.
    pub fn new(flash: F, buffer: &'b mut [u8]) -> Result<Self, Error> {
        if let Some(err) = page_buffer_error(buffer.len()) {
            return Err(err);
        }
        Ok(Self {
            flash,
            buffer,
            erase_mode: EraseMode::Sector,
            tracker: ErasedSectorTracker::new(),
            buf_addr: 0,
            buf_len: 0,
            initialized: false,
            chip_erased: false,
            verify_each: false,
        })
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

    /// Bytes one program covers at `addr`: the device's page granularity, reduced to the largest
    /// power of two the staging buffer can hold.
    fn page_span(&self, addr: u32) -> usize {
        let cap = self.buffer.len();
        let page = self.flash.program_page_min_size(addr) as usize;
        let span = if page == 0 { cap } else { cap.min(page) };
        (1usize << span.ilog2()).max(MIN_BUFFER)
    }

    /// Make the staged run the page containing `addr`, flushing first if needed. Returns the
    /// page span in force.
    fn open_page(&mut self, addr: u32) -> Result<usize, Error> {
        let span = self.page_span(addr);
        let base = addr - addr % span as u32;
        let staged_end = self.buf_addr + self.buf_len as u32;
        if self.buf_len > 0 && (base != self.buf_addr || addr < staged_end) {
            self.flush()?;
        }
        if self.buf_len == 0 {
            self.buf_addr = base;
        }
        let head = (addr - self.buf_addr) as usize;
        for b in &mut self.buffer[self.buf_len..head] {
            *b = 0xFF;
        }
        Ok(span)
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

    /// Buffer `data` to be written at `addr`.
    pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.ensure_init()?;
        let mut addr = addr;
        let mut data = data;
        while !data.is_empty() {
            let span = self.open_page(addr)?;
            // Offset within the staged page: programs start at the page base, not the fragment address.
            let off = (addr - self.buf_addr) as usize;
            let take = (span - off).min(data.len());
            self.buffer[off..off + take].copy_from_slice(&data[..take]);
            self.buf_len = off + take;
            addr += take as u32;
            data = &data[take..];
            if self.buf_len == span {
                self.flush()?;
            }
        }
        Ok(())
    }

    /// Erase (if needed) and program whatever is currently buffered.
    fn flush(&mut self) -> Result<(), Error> {
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

    /// Move the wrapped flash and buffer out without running the drop-time teardown.
    fn disassemble(self) -> (F, &'b mut [u8]) {
        let this = core::mem::ManuallyDrop::new(self);
        // SAFETY: `this` is never used or dropped again, so each field is moved out exactly once.
        let flash = unsafe { core::ptr::read(&this.flash) };
        let buffer = unsafe { core::ptr::read(&this.buffer) };
        (flash, buffer)
    }

    /// Flush pending data, tear down the session, and return the parts. `reset` is applied to the
    /// device just before teardown. Parts are returned on success and failure alike.
    pub fn finish(mut self, reset: bool) -> (Result<(), Error>, (F, &'b mut [u8])) {
        self.flash.set_post_flash_reset(reset);
        let flush_res = self.flush();
        let uninit_res = self.flash.uninit();
        (flush_res.and(uninit_res), self.disassemble())
    }

    /// Abandon any buffered data and return the parts; never errors. Takes no reset policy: an
    /// abort keeps whatever reset the device was composed with.
    pub fn abort(mut self) -> (F, &'b mut [u8]) {
        self.buf_len = 0;
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
