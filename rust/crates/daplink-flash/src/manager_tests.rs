//! `FlashManager` tests: blocking and stepped flushes against the mock flash.

extern crate alloc;

use super::*;
use crate::mock::{FlashOp, MockFlashBuilder};
use alloc::vec;
use alloc::vec::Vec;

const BASE: u32 = 0x0800_0000;
/// The mock's program granularity.
const PAGE: usize = 256;
/// Reference image size.
const IMAGE: usize = 62464;

#[test]
fn constructor_and_prevalidation_agree_at_buffer_boundaries() {
    for len in (0..=12).chain(251..=260).chain(1019..=1028) {
        let accepted = len >= 4 && len % 4 == 0;
        assert_eq!(page_buffer_ok(len), accepted, "length {len}");
        let flash = MockFlashBuilder::new(BASE, 1024).build();
        let mut buffer = vec![0xA5; len];
        let address = buffer.as_ptr();
        match FlashManager::new(flash, &mut buffer) {
            Ok(manager) => {
                assert!(accepted, "length {len}");
                let (_, restored) = manager.release();
                assert_eq!(restored.len(), len);
                assert_eq!(restored.as_ptr(), address);
                assert!(restored.iter().all(|b| *b == 0xA5));
            }
            Err(Error::BufferTooSmall { size, min }) => {
                assert!(!accepted, "length {len}");
                assert_eq!(size, len);
                assert_eq!(min, if len < 4 { 4 } else { (len + 3) & !3 });
            }
            Err(error) => panic!("unexpected constructor rejection: {error:?}"),
        };
    }
}

/// Drive `image` through the manager one step at a time, returning the flush count.
fn step_write(mgr: &mut FlashManager<'_, crate::mock::MockFlash>, image: &[u8]) -> usize {
    let mut addr = BASE;
    let mut data = image;
    let mut flushes = 0;
    while !data.is_empty() {
        if mgr.flush_due() {
            assert!(mgr.flush_step().expect("flush"), "a due flush does work");
            flushes += 1;
            continue;
        }
        let taken = mgr.stage(addr, data).expect("stage");
        assert!(
            taken > 0,
            "staging always makes progress when no flush is due"
        );
        addr += taken as u32;
        data = &data[taken..];
    }
    while mgr.flush_step().expect("flush") {
        flushes += 1;
    }
    flushes
}

/// One step programs one page; the F401 layout takes four lazy erases for the reference image.
#[test]
fn one_page_per_step_and_four_erases_across_the_reference_image() {
    let flash = MockFlashBuilder::new(BASE, 512 * 1024)
        .stm32f4_layout()
        .build();
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(flash, &mut buf).expect("manager");
    mgr.prepare().expect("prepare");
    let image: Vec<u8> = (0..IMAGE).map(|i| i as u8).collect();

    let flushes = step_write(&mut mgr, &image);
    assert_eq!(flushes, IMAGE / PAGE, "one page programmed per flush step");

    let (res, (flash, _buf)) = mgr.finish(true);
    res.expect("finish");
    let erases: Vec<u32> = flash
        .operations()
        .iter()
        .filter_map(|op| match op {
            FlashOp::EraseSector(addr) => Some(*addr),
            _ => None,
        })
        .collect();
    assert_eq!(
        erases,
        vec![BASE, BASE + 16 * 1024, BASE + 32 * 1024, BASE + 48 * 1024],
        "four 16 KiB sectors, erased lazily at the device sector size"
    );
    let programs = flash
        .operations()
        .iter()
        .filter(|op| matches!(op, FlashOp::ProgramPage { .. }))
        .count();
    assert_eq!(programs, IMAGE / PAGE);
    flash.assert_no_violations();
    flash.verify();
    assert_eq!(flash.read(BASE, IMAGE), image);
}

/// Each sector is erased immediately before the first page programmed into it.
#[test]
fn erases_interleave_with_the_programs_they_precede() {
    let flash = MockFlashBuilder::new(BASE, 512 * 1024)
        .stm32f4_layout()
        .build();
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(flash, &mut buf).expect("manager");
    mgr.prepare().expect("prepare");
    let image = vec![0xA5u8; IMAGE];
    step_write(&mut mgr, &image);
    let (res, (flash, _buf)) = mgr.finish(true);
    res.expect("finish");

    let mut expected_sector = BASE;
    for op in flash.operations() {
        match op {
            FlashOp::EraseSector(addr) => {
                assert_eq!(*addr, expected_sector, "sectors erase in order");
                expected_sector += 16 * 1024;
            }
            FlashOp::ProgramPage { addr, .. } => assert!(
                *addr < expected_sector,
                "a page is only programmed into an already erased sector"
            ),
            _ => {}
        }
    }
    assert_eq!(
        expected_sector,
        BASE + 64 * 1024,
        "all four erases happened"
    );
}

/// A fresh-sector page advances erase, program and verify separately, matching `flush_step`.
#[test]
fn flush_run_yields_between_erase_program_and_verify() {
    let page = vec![0x5Au8; PAGE];

    // Stepped, via `FlushRun`.
    let mut sbuf = [0u8; 1024];
    let mut stepped =
        FlashManager::new(MockFlashBuilder::new(BASE, 0x2000).build(), &mut sbuf).expect("manager");
    stepped.set_verify(true);
    stepped.prepare().expect("prepare");
    let mut off = 0usize;
    while off < page.len() && !stepped.flush_due() {
        off += stepped
            .stage(BASE + off as u32, &page[off..])
            .expect("stage");
    }
    assert!(stepped.flush_due(), "a full page is staged");

    let mut run = stepped.flush_begin().expect("flush_begin");
    assert!(matches!(run, FlushRun::Erase { .. }), "starts by erasing");
    // The distinct stages the run passes through, one visited per advance.
    let mut stages = Vec::new();
    let mut now_us = 0u64;
    let mut pendings = 0u32;
    loop {
        stages.push(match run {
            FlushRun::Erase { .. } => "erase",
            FlushRun::Program(_) => "program",
            FlushRun::Verify(_) => "verify",
            FlushRun::Done => "done",
        });
        match stepped.advance_flush(&mut run, now_us) {
            StepPoll::Done => break,
            StepPoll::Err(e) => panic!("flush run failed: {e:?}"),
            StepPoll::Pending { wake_at_us } => {
                pendings += 1;
                if wake_at_us > now_us {
                    now_us = wake_at_us;
                }
            }
        }
    }
    assert_eq!(
        stages,
        vec!["erase", "program", "verify"],
        "erase, then program, then verify, each its own advance"
    );
    assert!(pendings >= 2, "each stage transition yields");
    let (res, (stepped_flash, _buf)) = stepped.finish(true);
    res.expect("finish");

    // Blocking, via `flush_step`.
    let mut bbuf = [0u8; 1024];
    let mut blocking =
        FlashManager::new(MockFlashBuilder::new(BASE, 0x2000).build(), &mut bbuf).expect("manager");
    blocking.set_verify(true);
    blocking.prepare().expect("prepare");
    blocking.write(BASE, &page).expect("write");
    let (res, (blocking_flash, _buf)) = blocking.finish(true);
    res.expect("finish");

    assert_eq!(stepped_flash.operations(), blocking_flash.operations());
    assert_eq!(stepped_flash.read(BASE, PAGE), page);
    stepped_flash.verify();
}

/// The stepped write and the blocking `write` produce the identical device op stream.
#[test]
fn stepping_matches_the_blocking_write() {
    let image = vec![0x5Au8; 3 * PAGE + 17];

    let mut buf = [0u8; 1024];
    let mut mgr =
        FlashManager::new(MockFlashBuilder::new(BASE, 0x2000).build(), &mut buf).expect("manager");
    mgr.prepare().expect("prepare");
    step_write(&mut mgr, &image);
    let (res, (stepped, _buf)) = mgr.finish(true);
    res.expect("finish");

    let mut buf = [0u8; 1024];
    let mut mgr =
        FlashManager::new(MockFlashBuilder::new(BASE, 0x2000).build(), &mut buf).expect("manager");
    mgr.write(BASE, &image).expect("write");
    let (res, (blocking, _buf)) = mgr.finish(true);
    res.expect("finish");

    assert_eq!(stepped.operations(), blocking.operations());
    assert_eq!(
        stepped.read(BASE, image.len()),
        blocking.read(BASE, image.len())
    );
}

/// UF2 payload bytes per block.
const UF2_PAYLOAD: usize = 256;

/// Stage `image` one UF2 payload at a time, running each due flush as a `FlushRun`.
fn stage_blocks(mgr: &mut FlashManager<'_, crate::mock::MockFlash>, image: &[u8]) -> StepPoll {
    for (i, block) in image.chunks(UF2_PAYLOAD).enumerate() {
        let mut addr = BASE + (i * UF2_PAYLOAD) as u32;
        let mut data = block;
        while !data.is_empty() || mgr.flush_due() {
            if mgr.flush_due() {
                let mut run = match mgr.flush_begin() {
                    Ok(run) => run,
                    Err(e) => return StepPoll::Err(e),
                };
                loop {
                    match mgr.advance_flush(&mut run, 0) {
                        StepPoll::Pending { .. } => {}
                        StepPoll::Done => break,
                        err => return err,
                    }
                }
                continue;
            }
            let taken = mgr.stage(addr, data).expect("stage");
            addr += taken as u32;
            data = &data[taken..];
        }
    }
    StepPoll::Done
}

/// The F401 (16 KiB sectors, a 1 KiB staged page) under a 125-block drop refused at block `refused`.
fn f401_refused_drop(
    refused: usize,
    fail_program_at: Option<u32>,
) -> (crate::mock::MockFlash, Vec<u8>, StepPoll) {
    let mut flash = MockFlashBuilder::new(BASE, 512 * 1024)
        .stm32f4_layout()
        .page_size(1024);
    if let Some(addr) = fail_program_at {
        flash = flash.fail_program_at(addr);
    }
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(flash.build(), &mut buf).expect("manager");
    mgr.set_verify(true);
    mgr.prepare().expect("prepare");
    let image: Vec<u8> = (0..125 * UF2_PAYLOAD).map(|i| (i % 251) as u8).collect();
    let accepted = &image[..refused * UF2_PAYLOAD];
    let poll = stage_blocks(&mut mgr, accepted);
    mgr.keep_accepted();
    let mut run = mgr.flush_begin().expect("flush_begin");
    while let StepPoll::Pending { .. } = mgr.advance_flush(&mut run, 0) {}
    let (flash, _buf) = mgr.abort();
    (flash, accepted.to_vec(), poll)
}

/// A rejected drop keeps what it accepted in the sector it erased, and erases nothing more.
#[test]
fn a_kept_page_of_an_erased_sector_is_programmed() {
    let (flash, accepted, poll) = f401_refused_drop(62, None);
    assert_eq!(poll, StepPoll::Done);
    let lost = flash
        .read(BASE, accepted.len())
        .iter()
        .zip(&accepted)
        .position(|(a, b)| a != b);
    assert_eq!(lost, None, "first accepted byte not programmed");
    let tail = 16 * 1024 - accepted.len();
    let rest = flash.read(BASE + accepted.len() as u32, tail);
    assert!(
        rest.iter().all(|&b| b == 0xFF),
        "the sector tail stays erased"
    );
    let erases: Vec<&FlashOp> = flash
        .operations()
        .iter()
        .filter(|op| matches!(op, FlashOp::EraseSector(_)))
        .collect();
    assert_eq!(erases, [&FlashOp::EraseSector(BASE)]);
    flash.assert_no_violations();
}

/// A page whose program failed is not kept for a retry.
#[test]
fn a_failed_page_is_not_kept() {
    let (flash, _, poll) = f401_refused_drop(62, Some(BASE + 0x3800));
    assert!(matches!(poll, StepPoll::Err(_)));
    let at_failed = flash
        .operations()
        .iter()
        .filter(|op| matches!(op, FlashOp::ProgramPage { addr, .. } if *addr == BASE + 0x3800))
        .count();
    assert_eq!(at_failed, 1);
}

/// A staged page that would need a fresh erase is dropped, leaving that sector untouched.
#[test]
fn a_page_needing_an_erase_is_not_kept() {
    let (flash, accepted, poll) = f401_refused_drop(66, None);
    assert_eq!(poll, StepPoll::Done);
    assert_eq!(flash.read(BASE, 16 * 1024), accepted[..16 * 1024]);
    assert!(!flash
        .operations()
        .contains(&FlashOp::EraseSector(BASE + 16 * 1024)));
}

/// An nRF52 page (4 KiB page and sector) behind a 1 KiB buffer programs in aligned 1 KiB chunks.
#[test]
fn a_page_larger_than_the_buffer_is_programmed_in_chunks() {
    let flash = MockFlashBuilder::new(BASE, 64 * 1024)
        .uniform_sectors(4096)
        .page_size(4096)
        .build();
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(flash, &mut buf).expect("manager");
    mgr.set_verify(true);
    mgr.prepare().expect("prepare");
    let image: Vec<u8> = (0..8 * 1024).map(|i| (i % 251) as u8).collect();
    assert_eq!(stage_blocks(&mut mgr, &image), StepPoll::Done);
    let (res, (flash, _buf)) = mgr.finish(true);
    res.expect("finish");

    let mut expected = vec![FlashOp::Init];
    for sector in [BASE, BASE + 4096] {
        expected.push(FlashOp::EraseSector(sector));
        for chunk in 0..4 {
            expected.push(FlashOp::ProgramPage {
                addr: sector + chunk * 1024,
                len: 1024,
            });
        }
    }
    expected.extend([FlashOp::Uninit, FlashOp::PostFlashReset]);
    flash.assert_operations(&expected);
    flash.assert_no_violations();
    assert_eq!(flash.read(BASE, image.len()), image);
}

/// The blocking path's failed page is not kept either, whichever setter saw it fail.
#[test]
fn a_page_failing_a_blocking_flush_is_not_kept() {
    let addr = BASE + 0x3800;
    let flash = MockFlashBuilder::new(BASE, 512 * 1024)
        .stm32f4_layout()
        .page_size(1024)
        .fail_program_at(addr)
        .build();
    let mut buf = [0u8; 1024];
    let mut mgr = FlashManager::new(flash, &mut buf).expect("manager");
    mgr.set_verify(true);
    mgr.prepare().expect("prepare");
    let image: Vec<u8> = (0..62 * UF2_PAYLOAD).map(|i| (i % 251) as u8).collect();
    assert!(mgr.write(BASE, &image).is_err());
    mgr.keep_accepted();
    let (_, (flash, _buf)) = mgr.finish(false);
    let at_failed = flash
        .operations()
        .iter()
        .filter(|op| matches!(op, FlashOp::ProgramPage { addr: a, .. } if *a == addr))
        .count();
    assert_eq!(at_failed, 1);
}
