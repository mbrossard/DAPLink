//! Flash-algorithm descriptors: entry points, memory regions and sector layout.

use daplink_core::range;

/// Runtime parameters the algorithm's syscall calling convention depends on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SyscallParams {
    /// Address the algorithm returns to when a routine completes (a breakpoint trap).
    pub breakpoint: u32,
    /// Value loaded into r9 (the algorithm's static base / GOT pointer).
    pub static_base: u32,
    /// Initial stack pointer for algorithm execution.
    pub stack_pointer: u32,
}

/// A downloadable flash algorithm: its RAM-resident code blob plus entry offsets.
#[derive(Clone, Copy, Debug)]
pub struct FlashAlgorithm<'a> {
    /// Offset of `Init`.
    pub init: u32,
    /// Offset of `UnInit`.
    pub uninit: u32,
    /// Offset of `EraseChip`.
    pub erase_chip: u32,
    /// Offset of `EraseSector`.
    pub erase_sector: u32,
    /// Offset of `ProgramPage`.
    pub program_page: u32,
    /// Offset of `Verify`, when the algorithm provides one.
    pub verify: Option<u32>,
    /// Syscall runtime parameters.
    pub syscall: SyscallParams,
    /// Address of the RAM staging buffer used to hand page data to `ProgramPage`.
    pub program_buffer: u32,
    /// Size of the staging buffer in bytes.
    pub program_buffer_size: u32,
    /// Load address of the algorithm code in target RAM.
    pub algo_start: u32,
    /// Raw algorithm code (position-independent, loaded at `algo_start`).
    pub algo_blob: &'a [u8],
    /// Whether `Verify` returns the first failing address rather than a status code.
    pub verify_returns_address: bool,
    /// Whether the algorithm uses a single `Init` type regardless of operation.
    pub single_init_type: bool,
    /// Whether chip erase should be skipped even when supported.
    pub skip_chip_erase: bool,
}

impl FlashAlgorithm<'_> {
    /// Whether this algorithm exposes a chip-erase routine that should be used.
    pub fn supports_chip_erase(&self) -> bool {
        self.erase_chip != 0 && !self.skip_chip_erase
    }

    /// Whether this algorithm exposes a sector-erase routine.
    pub fn supports_sector_erase(&self) -> bool {
        self.erase_sector != 0
    }
}

/// A programmable flash region described by an absolute `[start, end)` span.
#[derive(Clone, Copy, Debug)]
pub struct FlashRegion<'a> {
    /// Inclusive start address.
    pub start: u32,
    /// Exclusive end address.
    pub end: u32,
    /// Whether this is the default region for drag-n-drop programming.
    pub is_default: bool,
    /// Whether this region is secure.
    pub is_secure: bool,
    /// The algorithm that programs this region.
    pub algo: &'a FlashAlgorithm<'a>,
}

impl FlashRegion<'_> {
    /// Whether `addr` lies within `[start, end)`.
    pub fn contains(&self, addr: u32) -> bool {
        range::contains(self.start, self.end, addr)
    }

    /// Region size in bytes.
    pub fn size(&self) -> u32 {
        self.end.saturating_sub(self.start)
    }
}

/// A RAM region usable for algorithm code and buffers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RamRegion {
    /// Inclusive start address.
    pub start: u32,
    /// Exclusive end address.
    pub end: u32,
}

impl RamRegion {
    /// Whether `addr` lies within `[start, end)`.
    pub fn contains(&self, addr: u32) -> bool {
        range::contains(self.start, self.end, addr)
    }

    /// Region size in bytes.
    pub fn size(&self) -> u32 {
        self.end.saturating_sub(self.start)
    }
}

/// A run of uniformly-sized sectors starting at `start`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SectorInfo {
    /// Address of the first sector in this run.
    pub start: u32,
    /// Size of each sector in this run, in bytes.
    pub size: u32,
    /// Number of sectors in this run.
    pub count: u32,
}

impl SectorInfo {
    /// Exclusive end address of this run.
    pub fn end(&self) -> u32 {
        self.start
            .saturating_add(self.size.saturating_mul(self.count))
    }

    /// Whether `addr` falls within this run of sectors.
    pub fn contains(&self, addr: u32) -> bool {
        range::contains(self.start, self.end(), addr)
    }

    /// Sector size covering `addr` (constant across the run), if in range.
    pub fn sector_size_at(&self, addr: u32) -> Option<u32> {
        if self.contains(addr) {
            Some(self.size)
        } else {
            None
        }
    }

    /// Base address of the sector covering `addr`, if in range.
    pub fn sector_base_at(&self, addr: u32) -> Option<u32> {
        if !self.contains(addr) || self.size == 0 {
            return None;
        }
        let offset = addr - self.start;
        let index = offset / self.size;
        Some(self.start + index * self.size)
    }
}
