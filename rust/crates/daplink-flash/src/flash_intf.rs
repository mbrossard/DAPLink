//! The [`FlashInterface`] seam: the minimal flashing surface the manager drives.

use crate::Error;

/// A programmable flash device, decoupled from how programming is performed.
pub trait FlashInterface {
    /// Begin a flashing session (connect / halt / prepare the algorithm). Idempotent.
    fn init(&mut self) -> Result<(), Error>;

    /// End a flashing session (release the target). Idempotent, and valid with no session open.
    fn uninit(&mut self) -> Result<(), Error>;

    /// Select whether [`uninit`](Self::uninit) releases the target to run (`true`) or leaves it
    /// halted (`false`).
    fn set_post_flash_reset(&mut self, enabled: bool);

    /// Program `data` at absolute `addr`. `addr` must be aligned to the program granularity in
    /// force and `data` exactly that many bytes.
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
}
