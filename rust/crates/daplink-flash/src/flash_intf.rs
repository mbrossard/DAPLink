//! The [`FlashInterface`] seam: the minimal flashing surface the manager drives.

use crate::reset_sequence::StepPoll;
use crate::swd_host::SyscallRun;
use crate::Error;

/// A programmable flash device, decoupled from how programming is performed.
pub trait FlashInterface {
    /// Begin a flashing session (connect / halt / prepare the algorithm). Idempotent.
    fn init(&mut self) -> Result<(), Error>;

    /// End a flashing session, releasing the target best-effort: `Ok` does not prove release.
    /// Idempotent, and valid with no session open.
    fn uninit(&mut self) -> Result<(), Error>;

    /// Whether [`uninit`](Self::uninit) releases the target to run or leaves it halted.
    fn set_post_flash_reset(&mut self, enabled: bool);

    /// Program `data` at `addr`: aligned to, and exactly, the program granularity in force.
    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error>;

    /// Verify that `data` is what is stored at absolute `addr` (the page just programmed).
    fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), Error>;

    /// Erase the sector containing `addr`.
    fn erase_sector(&mut self, addr: u32) -> Result<(), Error>;

    /// Erase the whole chip.
    fn erase_chip(&mut self) -> Result<(), Error>;

    /// Minimum program granularity (page size) covering `addr`.
    fn program_page_min_size(&self, addr: u32) -> u32;

    /// Sector size covering `addr`, if `addr` is in a flash region.
    fn erase_sector_size(&self, addr: u32) -> Option<u32>;

    /// Base address of the erase sector covering `addr`, if `addr` is in a flash region.
    fn erase_sector_base(&self, addr: u32) -> Option<u32>;

    /// Whether chip erase is available.
    fn supports_chip_erase(&self) -> bool;

    /// Whether sector erase is available.
    fn supports_sector_erase(&self) -> bool;

    /// Select the flash algorithm covering `addr` (no-op by default).
    fn set_flash_algorithm(&mut self, _addr: u32) -> Result<(), Error> {
        Ok(())
    }

    /// Begin a resumable sector erase; the default runs it to completion inline.
    /// Every `*_begin` may do its staging, such as loading a page into target RAM, synchronously.
    fn erase_sector_begin(&mut self, addr: u32) -> Result<SyscallRun, Error> {
        self.erase_sector(addr)?;
        Ok(SyscallRun::done(0))
    }

    /// Begin a resumable whole-chip erase (see [`erase_sector_begin`](Self::erase_sector_begin)).
    fn erase_chip_begin(&mut self) -> Result<SyscallRun, Error> {
        self.erase_chip()?;
        Ok(SyscallRun::done(0))
    }

    /// Begin a resumable program of a page (see [`erase_sector_begin`](Self::erase_sector_begin)).
    fn program_page_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        self.program_page(addr, data)?;
        Ok(SyscallRun::done(0))
    }

    /// Begin a resumable verify (see [`erase_sector_begin`](Self::erase_sector_begin)).
    fn verify_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        self.verify(addr, data)?;
        Ok(SyscallRun::done(0))
    }

    /// Advance a `*_begin` run by one bounded probe.
    fn advance_syscall(&mut self, run: &mut SyscallRun, now_us: u64) -> StepPoll {
        run.advance_inert(now_us)
    }

    /// Read and interpret a completed run's status.
    fn finish_syscall(&mut self, run: SyscallRun) -> Result<(), Error> {
        let _ = run;
        Ok(())
    }

    /// Spend `us` in the device's delay source; a no-op by default.
    fn delay_us(&mut self, _us: u32) {}
}

impl<T: FlashInterface> FlashInterface for &mut T {
    fn init(&mut self) -> Result<(), Error> {
        (**self).init()
    }

    fn uninit(&mut self) -> Result<(), Error> {
        (**self).uninit()
    }

    fn set_post_flash_reset(&mut self, enabled: bool) {
        (**self).set_post_flash_reset(enabled)
    }

    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        (**self).program_page(addr, data)
    }

    fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        (**self).verify(addr, data)
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        (**self).erase_sector(addr)
    }

    fn erase_chip(&mut self) -> Result<(), Error> {
        (**self).erase_chip()
    }

    fn program_page_min_size(&self, addr: u32) -> u32 {
        (**self).program_page_min_size(addr)
    }

    fn erase_sector_size(&self, addr: u32) -> Option<u32> {
        (**self).erase_sector_size(addr)
    }

    fn erase_sector_base(&self, addr: u32) -> Option<u32> {
        (**self).erase_sector_base(addr)
    }

    fn supports_chip_erase(&self) -> bool {
        (**self).supports_chip_erase()
    }

    fn supports_sector_erase(&self) -> bool {
        (**self).supports_sector_erase()
    }

    fn set_flash_algorithm(&mut self, addr: u32) -> Result<(), Error> {
        (**self).set_flash_algorithm(addr)
    }

    fn erase_sector_begin(&mut self, addr: u32) -> Result<SyscallRun, Error> {
        (**self).erase_sector_begin(addr)
    }

    fn erase_chip_begin(&mut self) -> Result<SyscallRun, Error> {
        (**self).erase_chip_begin()
    }

    fn program_page_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        (**self).program_page_begin(addr, data)
    }

    fn verify_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        (**self).verify_begin(addr, data)
    }

    fn advance_syscall(&mut self, run: &mut SyscallRun, now_us: u64) -> StepPoll {
        (**self).advance_syscall(run, now_us)
    }

    fn finish_syscall(&mut self, run: SyscallRun) -> Result<(), Error> {
        (**self).finish_syscall(run)
    }

    fn delay_us(&mut self, us: u32) {
        (**self).delay_us(us)
    }
}
