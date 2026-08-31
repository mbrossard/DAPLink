//! Host tests for the page-buffering manager, driven through [`MockFlash`].

use alloc::rc::Rc;
use core::cell::RefCell;

use crate::error::Error;
use crate::flash_algo::SectorInfo;
use crate::flash_intf::FlashInterface;
use crate::flash_manager::{EraseMode, FlashManager};
use crate::mock::{FlashOp, MockFlash, MockFlashBuilder, Violation};

const BASE: u32 = 0x0800_0000;
const SIZE: u32 = 0x2000;
const SECTOR: u32 = 0x400;
/// Program granularity the mock reports.
const PAGE: u32 = 256;

fn mock() -> MockFlash {
    MockFlash::uniform(BASE, SIZE, SECTOR)
}

/// Forwards to a [`MockFlash`] shared behind `Rc<RefCell<_>>`, so a test can keep observing it
/// after the `FlashManager` that owned it by value has been dropped.
struct SharedMock(Rc<RefCell<MockFlash>>);

impl FlashInterface for SharedMock {
    fn init(&mut self) -> Result<(), Error> {
        self.0.borrow_mut().init()
    }

    fn uninit(&mut self) -> Result<(), Error> {
        self.0.borrow_mut().uninit()
    }

    fn set_post_flash_reset(&mut self, enabled: bool) {
        self.0.borrow_mut().set_post_flash_reset(enabled)
    }

    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.0.borrow_mut().program_page(addr, data)
    }

    fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        // Named explicitly: `MockFlash` also has an inherent `verify(&self)`.
        FlashInterface::verify(&mut *self.0.borrow_mut(), addr, data)
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        self.0.borrow_mut().erase_sector(addr)
    }

    fn erase_chip(&mut self) -> Result<(), Error> {
        self.0.borrow_mut().erase_chip()
    }

    fn program_page_min_size(&self, addr: u32) -> u32 {
        self.0.borrow().program_page_min_size(addr)
    }

    fn erase_sector_size(&self, addr: u32) -> Option<u32> {
        self.0.borrow().erase_sector_size(addr)
    }

    fn erase_sector_base(&self, addr: u32) -> Option<u32> {
        self.0.borrow().erase_sector_base(addr)
    }

    fn supports_chip_erase(&self) -> bool {
        self.0.borrow().supports_chip_erase()
    }

    fn supports_sector_erase(&self) -> bool {
        self.0.borrow().supports_sector_erase()
    }
}

#[test]
fn happy_write_then_finish_programs_erased_then_uninit() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0xAA; 100]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    flash.verify();

    assert_eq!(flash.read(BASE, 100), [0xAA; 100]);
    assert_eq!(flash.read(BASE + 100, 4), [0xFF; 4]);
}

#[test]
fn abort_does_not_flush_and_never_errors() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0x11; 40]).unwrap();
    let (flash, _buf) = mgr.abort();

    flash.assert_operations(&[FlashOp::Init, FlashOp::Uninit, FlashOp::PostFlashReset]);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 4), [0xFF; 4]);
}

#[test]
fn drop_without_finish_or_abort_uninits_exactly_once() {
    let shared = Rc::new(RefCell::new(mock()));
    let mut buf = [0u8; 256];
    {
        let mut mgr = FlashManager::new(SharedMock(shared.clone()), &mut buf).unwrap();
        mgr.write(BASE, &[0xAA; 16]).unwrap();
    }

    let flash = shared.borrow();
    flash.assert_operations(&[FlashOp::Init, FlashOp::Uninit, FlashOp::PostFlashReset]);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 16), [0xFF; 16]);
}

/// A session the caller opened, that then programs nothing, must still be torn down.
#[test]
fn caller_opened_session_is_torn_down_by_finish() {
    let mut flash = mock();
    flash.init().unwrap();
    let mut buf = [0u8; 256];
    let mgr = FlashManager::new(flash, &mut buf).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[FlashOp::Init, FlashOp::Uninit, FlashOp::PostFlashReset]);
    assert!(!flash.is_initialized(), "the session must be closed");
    flash.assert_no_violations();
}

/// The same for the abort exit.
#[test]
fn caller_opened_session_is_torn_down_by_abort() {
    let mut flash = mock();
    flash.init().unwrap();
    let mut buf = [0u8; 256];
    let mgr = FlashManager::new(flash, &mut buf).unwrap();
    let (flash, _buf) = mgr.abort();

    flash.assert_operations(&[FlashOp::Init, FlashOp::Uninit, FlashOp::PostFlashReset]);
    assert!(!flash.is_initialized(), "the session must be closed");
}

/// And for the drop exit, where neither `finish` nor `abort` is ever called.
#[test]
fn caller_opened_session_is_torn_down_by_drop() {
    let shared = Rc::new(RefCell::new(mock()));
    shared.borrow_mut().init().unwrap();
    let mut buf = [0u8; 256];
    {
        let _mgr = FlashManager::new(SharedMock(shared.clone()), &mut buf).unwrap();
    }

    let flash = shared.borrow();
    flash.assert_operations(&[FlashOp::Init, FlashOp::Uninit, FlashOp::PostFlashReset]);
    assert!(!flash.is_initialized(), "the session must be closed");
}

/// The reset-after-flash policy learned at the end of a transfer reaches the device: `finish`
/// applies it just before teardown.
#[test]
fn finish_threads_the_reset_policy_to_the_device() {
    const PROGRAM: FlashOp = FlashOp::ProgramPage {
        addr: BASE,
        len: PAGE as usize,
    };

    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0xAA; 16]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(false);
    res.unwrap();
    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        PROGRAM,
        FlashOp::Uninit,
    ]);
    flash.assert_no_violations();

    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0xAA; 16]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();
    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        PROGRAM,
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
}

/// A session that programmed nothing is still torn down when the policy says "no reset".
#[test]
fn no_reset_policy_still_tears_the_session_down() {
    let mut flash = mock();
    flash.init().unwrap();
    let mut buf = [0u8; 256];
    let mgr = FlashManager::new(flash, &mut buf).unwrap();
    let (res, (flash, _buf)) = mgr.finish(false);
    res.unwrap();

    flash.assert_operations(&[FlashOp::Init, FlashOp::Uninit]);
    assert!(!flash.is_initialized(), "the session must be closed");
}

#[test]
fn finish_then_reuse_parts_for_second_cycle() {
    let mut buf = [0u8; 256];
    let mgr = FlashManager::new(mock(), &mut buf).unwrap();
    let (flash, buf) = {
        let mut mgr = mgr;
        mgr.write(BASE, &[0x22; 16]).unwrap();
        let (res, parts) = mgr.finish(true);
        res.unwrap();
        parts
    };

    let mut mgr2 = FlashManager::new(flash, buf).unwrap();
    mgr2.write(BASE + SECTOR, &[0x33; 16]).unwrap();
    let (res, (flash, _buf)) = mgr2.finish(true);
    res.unwrap();

    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 16), [0x22; 16]);
    assert_eq!(flash.read(BASE + SECTOR, 16), [0x33; 16]);
}

#[test]
fn erase_mode_chip_uses_chip_erase() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.set_erase_mode(EraseMode::Chip);
    assert_eq!(mgr.erase_mode(), EraseMode::Chip);
    mgr.write(BASE, &[0x44; 8]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseChip,
        FlashOp::ProgramPage {
            addr: BASE,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
}

#[test]
fn erase_mode_chip_falls_back_to_sector_when_unsupported() {
    let flash = MockFlashBuilder::new(BASE, SIZE)
        .uniform_sectors(SECTOR)
        .no_chip_erase()
        .build();
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(flash, &mut buf).unwrap();
    mgr.set_erase_mode(EraseMode::Chip);
    mgr.write(BASE, &[0x55; 8]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
}

#[test]
fn write_spanning_two_sectors_erases_both() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    let start = BASE + SECTOR - 16;
    mgr.write(start, &[0x66; 32]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    // Each page is erased just before it is programmed, not both sectors up front.
    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE + SECTOR - PAGE,
            len: PAGE as usize,
        },
        FlashOp::EraseSector(BASE + SECTOR),
        FlashOp::ProgramPage {
            addr: BASE + SECTOR,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    assert_eq!(flash.read(start, 32), [0x66; 32]);
}

#[test]
fn non_contiguous_writes_auto_flush_between() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0x77; 4]).unwrap();
    mgr.write(BASE + PAGE, &[0x88; 4]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE,
            len: PAGE as usize,
        },
        FlashOp::ProgramPage {
            addr: BASE + PAGE,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 4), [0x77; 4]);
    assert_eq!(flash.read(BASE + PAGE, 4), [0x88; 4]);
}

#[test]
fn partial_page_padded_with_0xff() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0x01, 0x02, 0x03]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    assert_eq!(flash.read(BASE, 4), [0x01, 0x02, 0x03, 0xFF]);
    flash.assert_no_violations();
}

#[test]
fn correct_path_raises_no_violation_but_skipping_erase_does() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0x00; 8]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();
    flash.assert_no_violations();

    let mut raw = mock();
    raw.init().unwrap();
    raw.program_page(BASE, &[0x00; 8]).unwrap();
    assert_eq!(raw.violations(), &[Violation::WriteToNonErased]);
}

#[test]
fn buffer_too_small_is_rejected() {
    let mut tiny = [0u8; 2];
    let res = FlashManager::new(mock(), &mut tiny);
    assert!(matches!(
        res,
        Err(Error::BufferTooSmall { size: 2, min: 4 })
    ));
}

#[test]
fn unaligned_program_is_rejected() {
    let mut raw = mock();
    raw.init().unwrap();
    let err = raw.program_page(BASE + 1, &[0x00; 4]).unwrap_err();
    assert_eq!(
        err,
        Error::AddressNotAligned {
            addr: BASE + 1,
            alignment: 4,
        }
    );
}

/// A buffer too small to hold a page still programs aligned blocks, at the largest power of two
/// it can stage.
#[test]
fn buffer_full_auto_flushes() {
    let mut buf = [0u8; 8];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0x99; 20]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage { addr: BASE, len: 8 },
        FlashOp::ProgramPage {
            addr: BASE + 8,
            len: 8,
        },
        FlashOp::ProgramPage {
            addr: BASE + 16,
            len: 8,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 20), [0x99; 20]);
}

/// A buffer larger than the device's page is split into device-sized programs, not offered whole.
#[test]
fn buffer_larger_than_the_device_page_is_split() {
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0x5A; 1024]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE,
            len: 256,
        },
        FlashOp::ProgramPage {
            addr: BASE + 256,
            len: 256,
        },
        FlashOp::ProgramPage {
            addr: BASE + 512,
            len: 256,
        },
        FlashOp::ProgramPage {
            addr: BASE + 768,
            len: 256,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 1024), [0x5A; 1024]);
}

/// The same split holds when arriving fragments are smaller than the page: the manager still
/// accumulates up to the device's page rather than the buffer's capacity.
#[test]
fn small_fragments_accumulate_to_the_device_page() {
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    for i in 0..4u32 {
        mgr.write(BASE + i * 128, &[0xC3; 128]).unwrap();
    }
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE,
            len: 256,
        },
        FlashOp::ProgramPage {
            addr: BASE + 256,
            len: 256,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 512), [0xC3; 512]);
}

#[test]
fn flash_manager_rejects_non_word_buffer() {
    // A 6-byte buffer is not a whole number of 32-bit words; rejected at construction.
    let mut buf = [0u8; 6];
    let res = FlashManager::new(mock(), &mut buf);
    assert_eq!(res.err(), Some(Error::BufferTooSmall { size: 6, min: 8 }));
}

#[test]
fn flash_manager_finish_uninit_error_no_double() {
    let flash = MockFlashBuilder::new(BASE, SIZE)
        .uniform_sectors(SECTOR)
        .fail_uninit()
        .build();
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(flash, &mut buf).unwrap();
    mgr.write(BASE, &[0xAB; 16]).unwrap();
    let (res, _parts) = mgr.finish(true);
    let err = res.unwrap_err();
    assert_eq!(err, Error::UninitFailed);
}

/// A fragment that starts mid-page must still reach the device as whole, page-aligned programs.
#[test]
fn unaligned_fragment_is_programmed_as_aligned_pages() {
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE + 0x104, &[0x5A; 600]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    for op in flash.operations() {
        if let FlashOp::ProgramPage { addr, len } = *op {
            assert_eq!(
                addr % PAGE,
                0,
                "program at {addr:#010x} is not page-aligned"
            );
            let len = len as u32;
            assert_eq!(len % PAGE, 0, "program of {len} bytes is not a whole page");
        }
    }
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE + 0x104, 600), [0x5A; 600]);
    assert_eq!(flash.read(BASE + 0x100, 4), [0xFF; 4]);
    assert_eq!(flash.read(BASE + 0x104 + 600, 4), [0xFF; 4]);
}

/// The mock refuses what the manager must never emit: a page-sized run at a non-page-base
/// address, and a length that cannot be an aligned page block.
#[test]
fn mock_rejects_a_program_that_is_not_an_aligned_page_block() {
    let mut raw = mock();
    raw.init().unwrap();

    let err = raw.program_page(BASE + 0x104, &[0x00; 256]).unwrap_err();
    assert_eq!(
        err,
        Error::AddressNotAligned {
            addr: BASE + 0x104,
            alignment: PAGE,
        }
    );

    let err = raw.program_page(BASE, &[0x00; 100]).unwrap_err();
    assert_eq!(
        err,
        Error::DataTooLarge {
            size: 100,
            max: PAGE,
        }
    );
}

/// Two fragments in one page with a gap between them are programmed as a single page, the gap
/// filled with the erased value.
#[test]
fn gap_inside_one_page_is_filled_rather_than_reprogrammed() {
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(mock(), &mut buf).unwrap();
    mgr.write(BASE, &[0xA1; 4]).unwrap();
    mgr.write(BASE + 0x80, &[0xB2; 4]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(BASE),
        FlashOp::ProgramPage {
            addr: BASE,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, 4), [0xA1; 4]);
    assert_eq!(flash.read(BASE + 4, 4), [0xFF; 4]);
    assert_eq!(flash.read(BASE + 0x80, 4), [0xB2; 4]);
}

/// A sector run whose start is not a multiple of its own sector size: the base of the sector
/// covering an address is relative to the run's start, not to an absolute multiple of the size.
#[test]
fn offset_sector_run_erases_the_sector_it_programs() {
    const RUN_START: u32 = BASE + 0x400;
    const RUN_SECTOR: u32 = 0x800;
    let flash = MockFlashBuilder::new(BASE, SIZE)
        .sector_run(SectorInfo {
            start: RUN_START,
            size: RUN_SECTOR,
            count: 2,
        })
        .build();
    let mut buf = [0u8; 256];
    let mut mgr = FlashManager::new(flash, &mut buf).unwrap();
    let addr = RUN_START + RUN_SECTOR;
    mgr.write(addr, &[0x3C; 16]).unwrap();
    let (res, (flash, _buf)) = mgr.finish(true);
    res.unwrap();

    flash.assert_operations(&[
        FlashOp::Init,
        FlashOp::EraseSector(addr),
        FlashOp::ProgramPage {
            addr,
            len: PAGE as usize,
        },
        FlashOp::Uninit,
        FlashOp::PostFlashReset,
    ]);
    flash.assert_no_violations();
    assert_eq!(flash.read(addr, 16), [0x3C; 16]);
}

#[test]
fn stm32f4_layout_sector_sizes() {
    let flash = MockFlashBuilder::new(BASE, 512 * 1024)
        .stm32f4_layout()
        .build();
    assert_eq!(flash.erase_sector_size(BASE), Some(16 * 1024));
    assert_eq!(
        flash.erase_sector_size(BASE + 4 * 16 * 1024),
        Some(64 * 1024)
    );
    assert_eq!(
        flash.erase_sector_size(BASE + 4 * 16 * 1024 + 64 * 1024),
        Some(128 * 1024)
    );
}
