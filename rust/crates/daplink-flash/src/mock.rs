//! Host-side mock [`FlashInterface`] for testing the manager and page logic.

use alloc::vec;
use alloc::vec::Vec;

use crate::error::Error;
use crate::flash_algo::SectorInfo;
use crate::flash_intf::FlashInterface;

/// Program granularity reported by the mock.
const MOCK_PAGE: u32 = 256;

/// A flash operation the mock observed, in call order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlashOp {
    /// [`FlashInterface::init`].
    Init,
    /// [`FlashInterface::uninit`].
    Uninit,
    /// The release-to-run performed inside [`FlashInterface::uninit`] when the reset policy is on.
    PostFlashReset,
    /// [`FlashInterface::erase_chip`].
    EraseChip,
    /// [`FlashInterface::erase_sector`] at the given address.
    EraseSector(u32),
    /// [`FlashInterface::program_page`] of `len` bytes at `addr`.
    ProgramPage {
        /// Program address.
        addr: u32,
        /// Byte count.
        len: usize,
    },
}

/// A misuse of the flash protocol detected by the mock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Violation {
    /// A program touched a byte that had not been erased.
    WriteToNonErased,
    /// An access fell outside the flash bounds.
    OutOfBounds,
    /// An operation ran before `init`.
    NotInitialized,
    /// `init` ran while already initialized.
    AlreadyInitialized,
    /// An erase targeted an address that is not a valid sector base.
    InvalidSector,
}

/// Builder for a [`MockFlash`].
#[derive(Clone, Debug)]
pub struct MockFlashBuilder {
    base_addr: u32,
    size: u32,
    runs: Vec<SectorInfo>,
    chip_erase: bool,
    sector_erase: bool,
    fail_uninit: bool,
}

impl MockFlashBuilder {
    /// Start a builder for a flash of `size` bytes based at `base_addr`.
    pub fn new(base_addr: u32, size: u32) -> Self {
        Self {
            base_addr,
            size,
            runs: Vec::new(),
            chip_erase: true,
            sector_erase: true,
            fail_uninit: false,
        }
    }

    /// Lay out the whole flash as uniform sectors of `sector_size` bytes.
    pub fn uniform_sectors(mut self, sector_size: u32) -> Self {
        self.runs.clear();
        let count = self.size / sector_size;
        self.runs.push(SectorInfo {
            start: self.base_addr,
            size: sector_size,
            count,
        });
        self
    }

    /// Append one explicit run of sectors, for a geometry the named layouts do not describe.
    pub fn sector_run(mut self, run: SectorInfo) -> Self {
        self.runs.push(run);
        self
    }

    /// Apply the classic STM32F4 non-uniform layout (4x16K, 1x64K, then 128K sectors).
    pub fn stm32f4_layout(mut self) -> Self {
        self.runs.clear();
        let mut addr = self.base_addr;
        self.runs.push(SectorInfo {
            start: addr,
            size: 16 * 1024,
            count: 4,
        });
        addr += 4 * 16 * 1024;
        self.runs.push(SectorInfo {
            start: addr,
            size: 64 * 1024,
            count: 1,
        });
        addr += 64 * 1024;
        let remaining = (self.base_addr + self.size).saturating_sub(addr);
        let count = remaining / (128 * 1024);
        if count > 0 {
            self.runs.push(SectorInfo {
                start: addr,
                size: 128 * 1024,
                count,
            });
        }
        self
    }

    /// Mark chip erase as unsupported.
    pub fn no_chip_erase(mut self) -> Self {
        self.chip_erase = false;
        self
    }

    /// Mark sector erase as unsupported.
    pub fn no_sector_erase(mut self) -> Self {
        self.sector_erase = false;
        self
    }

    /// Make [`FlashInterface::uninit`] return an error (while still recording the op).
    pub fn fail_uninit(mut self) -> Self {
        self.fail_uninit = true;
        self
    }

    /// Finish building the mock flash.
    pub fn build(self) -> MockFlash {
        let mut runs = self.runs;
        if runs.is_empty() {
            runs.push(SectorInfo {
                start: self.base_addr,
                size: self.size,
                count: 1,
            });
        }
        MockFlash {
            base: self.base_addr,
            data: vec![0xFF; self.size as usize],
            erased: vec![false; self.size as usize],
            runs,
            chip_erase: self.chip_erase,
            sector_erase: self.sector_erase,
            fail_uninit: self.fail_uninit,
            ops: Vec::new(),
            violations: Vec::new(),
            initialized: false,
            post_flash_reset: true,
        }
    }
}

/// A virtual flash device implementing [`FlashInterface`] for host tests.
#[derive(Clone, Debug)]
pub struct MockFlash {
    base: u32,
    data: Vec<u8>,
    /// Per-byte erased flag (set by erase, cleared conceptually by program-to-non-0xFF).
    erased: Vec<bool>,
    runs: Vec<SectorInfo>,
    chip_erase: bool,
    sector_erase: bool,
    fail_uninit: bool,
    ops: Vec<FlashOp>,
    violations: Vec<Violation>,
    initialized: bool,
    /// Whether teardown releases the target to run. Defaults to `true`.
    post_flash_reset: bool,
}

impl MockFlash {
    /// Convenience: a uniform-sector flash.
    pub fn uniform(base_addr: u32, size: u32, sector_size: u32) -> Self {
        MockFlashBuilder::new(base_addr, size)
            .uniform_sectors(sector_size)
            .build()
    }

    /// The recorded operation log.
    pub fn operations(&self) -> &[FlashOp] {
        &self.ops
    }

    /// The recorded violations.
    pub fn violations(&self) -> &[Violation] {
        &self.violations
    }

    /// Whether a session is currently open (`init` seen, `uninit` not yet).
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Read `len` bytes at absolute `addr` from the virtual flash.
    pub fn read(&self, addr: u32, len: usize) -> Vec<u8> {
        let off = (addr - self.base) as usize;
        self.data[off..off + len].to_vec()
    }

    /// Whether `addr` is within the flash bounds.
    fn in_bounds(&self, addr: u32, len: u32) -> bool {
        addr >= self.base && (addr + len) <= self.base + self.data.len() as u32
    }

    /// The sector base for `addr`, if it lies in a run (the shared [`SectorInfo`] math).
    fn sector_base(&self, addr: u32) -> Option<u32> {
        self.runs.iter().find_map(|run| run.sector_base_at(addr))
    }

    /// Assert the exact operation sequence.
    pub fn assert_operations(&self, expected: &[FlashOp]) {
        assert_eq!(
            self.ops.as_slice(),
            expected,
            "flash operation sequence mismatch"
        );
    }

    /// Assert no protocol violations were recorded.
    pub fn assert_no_violations(&self) {
        assert!(
            self.violations.is_empty(),
            "unexpected flash violations: {:?}",
            self.violations
        );
    }

    /// Full sanity check: no violations were recorded during the session.
    pub fn verify(&self) {
        self.assert_no_violations();
    }
}

impl FlashInterface for MockFlash {
    fn init(&mut self) -> Result<(), Error> {
        self.ops.push(FlashOp::Init);
        if self.initialized {
            self.violations.push(Violation::AlreadyInitialized);
        }
        self.initialized = true;
        Ok(())
    }

    fn uninit(&mut self) -> Result<(), Error> {
        self.ops.push(FlashOp::Uninit);
        // Teardown with no session open is a no-op, but still recorded.
        if !self.initialized {
            return Ok(());
        }
        self.initialized = false;
        // Recorded before any injected failure: a failing teardown never suppresses the reset.
        if self.post_flash_reset {
            self.ops.push(FlashOp::PostFlashReset);
        }
        if self.fail_uninit {
            return Err(Error::UninitFailed);
        }
        Ok(())
    }

    fn set_post_flash_reset(&mut self, enabled: bool) {
        self.post_flash_reset = enabled;
    }

    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.ops.push(FlashOp::ProgramPage {
            addr,
            len: data.len(),
        });
        // Enforce the geometry reported through `program_page_min_size`.
        let len = data.len() as u32;
        if len == 0 || len > MOCK_PAGE || !MOCK_PAGE.is_multiple_of(len) {
            return Err(Error::DataTooLarge {
                size: len,
                max: MOCK_PAGE,
            });
        }
        if !addr.is_multiple_of(len) {
            return Err(Error::AddressNotAligned {
                addr,
                alignment: len,
            });
        }
        if !self.initialized {
            self.violations.push(Violation::NotInitialized);
        }
        if !self.in_bounds(addr, data.len() as u32) {
            self.violations.push(Violation::OutOfBounds);
            return Err(Error::AddressOutOfRange { addr });
        }
        let off = (addr - self.base) as usize;
        let mut hit_non_erased = false;
        for (i, byte) in data.iter().enumerate() {
            // 0xFF (erased value) is a no-op write, so padding never trips a violation.
            if *byte == 0xFF {
                continue;
            }
            if !self.erased[off + i] {
                hit_non_erased = true;
            }
            // AND-write semantics: programming can only clear bits.
            self.data[off + i] &= byte;
            self.erased[off + i] = false;
        }
        if hit_non_erased {
            self.violations.push(Violation::WriteToNonErased);
        }
        Ok(())
    }

    fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        if !self.initialized {
            self.violations.push(Violation::NotInitialized);
        }
        if !self.in_bounds(addr, data.len() as u32) {
            self.violations.push(Violation::OutOfBounds);
            return Err(Error::AddressOutOfRange { addr });
        }
        let off = (addr - self.base) as usize;
        if self.data[off..off + data.len()] != *data {
            return Err(Error::VerifyFailed { addr });
        }
        Ok(())
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        self.ops.push(FlashOp::EraseSector(addr));
        if !self.initialized {
            self.violations.push(Violation::NotInitialized);
        }
        match self.sector_base(addr) {
            Some(base) if base == addr => {
                let size = self
                    .runs
                    .iter()
                    .find(|r| r.contains(addr))
                    .map(|r| r.size)
                    .unwrap_or(0);
                let off = (addr - self.base) as usize;
                for i in 0..size as usize {
                    self.data[off + i] = 0xFF;
                    self.erased[off + i] = true;
                }
                Ok(())
            }
            _ => {
                self.violations.push(Violation::InvalidSector);
                Err(Error::EraseFailed)
            }
        }
    }

    fn erase_chip(&mut self) -> Result<(), Error> {
        self.ops.push(FlashOp::EraseChip);
        if !self.initialized {
            self.violations.push(Violation::NotInitialized);
        }
        for b in &mut self.data {
            *b = 0xFF;
        }
        for e in &mut self.erased {
            *e = true;
        }
        Ok(())
    }

    fn program_page_min_size(&self, _addr: u32) -> u32 {
        MOCK_PAGE
    }

    fn erase_sector_size(&self, addr: u32) -> Option<u32> {
        self.runs.iter().find(|r| r.contains(addr)).map(|r| r.size)
    }

    fn erase_sector_base(&self, addr: u32) -> Option<u32> {
        self.sector_base(addr)
    }

    fn supports_chip_erase(&self) -> bool {
        self.chip_erase
    }

    fn supports_sector_erase(&self) -> bool {
        self.sector_erase
    }
}
