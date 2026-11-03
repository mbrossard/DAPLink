//! Host tests pinning the RP2040 record's values and its algorithm's structural invariants.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{ResetOps, ResetSequence, GENERIC_RESET_SEQUENCE};
use daplink_flash::target::{ResetType, TargetState};

use crate::algorithms::rp2040_rom::{RP2040_ROM_ALGO, RP2040_ROM_BLOB};
use crate::targets::rp2040::{RP2040, RP2040_UF2_FAMILY};
use crate::{check_identity, RamRegion, Target};

#[test]
fn uf2_family_is_official_rp2040_id() {
    // Microsoft uf2families.json: "Raspberry Pi RP2040" -> 0xe48bff56.
    assert_eq!(RP2040_UF2_FAMILY, 0xE48B_FF56);
    assert_eq!(RP2040.uf2_family(), Some(0xE48B_FF56));
}

#[test]
fn identity_probe_matches_bootrom_magic() {
    let probes = RP2040.identity_probes();
    assert_eq!(probes.len(), 1);
    let p = probes[0];
    // The bootrom magic word 'M','u',version=1 sits at offset 0x10; the top byte is masked off.
    assert_eq!(p.addr, 0x0000_0010);
    assert_eq!(p.mask, 0x00FF_FFFF);
    assert_eq!(p.expected, 0x0001_754D);

    // A genuine read (any top byte, correct 'M','u',1 low bytes) matches; a wrong magic aborts.
    assert!(check_identity(probes, |_| Ok(0xAB01_754D)).is_ok());
    assert!(check_identity(probes, |_| Ok(0x0002_754D)).is_err());
}

#[test]
fn flash_region_is_2m_qspi() {
    let target = RP2040;
    let regions = target.flash_regions();
    assert_eq!(regions.len(), 1);
    let r = &regions[0];
    // RP2040: 2 MiB of external QSPI flash at the XIP base.
    assert_eq!(r.start, 0x1000_0000);
    assert_eq!(r.end, 0x1020_0000);
    assert_eq!(r.end - r.start, 2 * 1024 * 1024);
    assert!(r.is_default);
}

#[test]
fn ram_region_pinned() {
    let target = RP2040;
    let ram = target.ram_regions();
    assert_eq!(ram.len(), 1);
    assert_eq!(ram[0].start, 0x2000_0000);
    assert_eq!(ram[0].end, 0x2004_2000);
}

#[test]
fn sectors_are_4k() {
    let target = RP2040;
    let sectors = target.sectors();
    assert_eq!(sectors.len(), 1);
    // Uniform 4 KiB sectors over the 2 MiB flash (512 of them).
    assert_eq!(sectors[0].start, 0x1000_0000);
    assert_eq!(sectors[0].size, 0x1000);
    assert_eq!(sectors[0].count, 512);
}

#[test]
fn rom_algo_entry_offsets_pinned() {
    let algo = &RP2040_ROM_ALGO;
    // Entry points are offsets into the blob, not absolute addresses; the stub sits at offset 0.
    assert_eq!(algo.init, 0x04);
    assert_eq!(algo.uninit, 0x2c);
    assert_eq!(algo.erase_chip, 0x80);
    assert_eq!(algo.erase_sector, 0x54);
    assert_eq!(algo.program_page, 0xa8);
    // The shim's `Verify` restores XIP read mode, compares, then restores the serial command state.
    assert_eq!(algo.verify, Some(0xd8));
}

#[test]
fn rom_blob_is_non_empty_and_word_sized() {
    // The generated blob is present and whole-word (the loader writes 32-bit words).
    assert!(!RP2040_ROM_BLOB.is_empty());
    assert_eq!(RP2040_ROM_BLOB.len() % 4, 0);
}

/// Entry points are load-relative offsets inside the referenced blob.
#[test]
fn rom_algo_entries_are_offsets_not_addresses() {
    let algo = &RP2040_ROM_ALGO;
    let blob_len = RP2040_ROM_BLOB.len() as u32;
    for offset in [
        algo.init,
        algo.uninit,
        algo.erase_chip,
        algo.erase_sector,
        algo.program_page,
    ]
    .into_iter()
    .chain(algo.verify)
    {
        // A value inside the blob is an offset; one near 0x2000_0000 is the absolute-address bug.
        assert!(offset < blob_len, "entry 0x{offset:x} is not a blob offset");
    }
}

/// RAM/layout invariants any RP2040 flash algorithm must satisfy.
fn assert_algo_ram_invariants(algo: &crate::FlashAlgorithm<'_>, blob_len: u32) {
    // The algorithm loads into main SRAM (a 16 KiB window from the SRAM base).
    let ram = RamRegion {
        start: 0x2000_0000,
        end: 0x2000_4000,
    };

    // Stack pointer, static base and the algorithm load address live in RAM.
    assert!(ram.contains(algo.syscall.stack_pointer));
    assert!(ram.contains(algo.syscall.static_base));
    assert!(ram.contains(algo.algo_start));

    assert!(ram.contains(algo.program_buffer));
    let pb_end = algo.program_buffer + algo.program_buffer_size;
    assert!(pb_end <= ram.end);
    // The RP2040 QSPI algorithm stages a full 4 KiB page at a time.
    assert_eq!(algo.program_buffer_size, 0x1000);

    // Program buffer must not overlap the loaded algorithm blob.
    let blob_start = algo.algo_start;
    let blob_end = algo.algo_start + blob_len;
    assert!(algo.program_buffer >= blob_end || pb_end <= blob_start);
}

#[test]
fn rom_algo_ram_invariants() {
    assert_algo_ram_invariants(&RP2040_ROM_ALGO, RP2040_ROM_BLOB.len() as u32);
}

/// The flash region resolves to the ROM-call shim.
#[test]
fn target_resolves_selected_algo_handle() {
    let target = RP2040;
    let regions = target.flash_regions();
    assert_eq!(regions.len(), 1);
    assert!(core::ptr::eq(regions[0].algo, &RP2040_ROM_ALGO));
}

/// Recording [`ResetOps`] mock: logs the ordered stream of operations a sequence performs.
#[derive(Default)]
struct RecOps {
    ops: std::vec::Vec<Op>,
    /// Scripted `(address, value)` reads; any other address reads 0.
    reads: std::vec::Vec<(u32, u32)>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Op {
    Write(u32, u32),
    Read(u32),
    Reset(ResetType),
    PollHalt,
    SetReset(bool),
    Halt,
}

impl ResetOps for RecOps {
    fn write_word(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        self.ops.push(Op::Write(addr, value));
        Ok(())
    }
    fn read_word(&mut self, addr: u32) -> Result<u32, Error> {
        self.ops.push(Op::Read(addr));
        Ok(self
            .reads
            .iter()
            .find(|&&(a, _)| a == addr)
            .map_or(0, |&(_, v)| v))
    }
    fn trigger_reset(&mut self, reset_type: ResetType) -> Result<(), Error> {
        self.ops.push(Op::Reset(reset_type));
        Ok(())
    }
    fn poll_halt(&mut self) -> Result<(), Error> {
        self.ops.push(Op::PollHalt);
        Ok(())
    }
    fn set_reset(&mut self, asserted: bool) -> Result<(), Error> {
        self.ops.push(Op::SetReset(asserted));
        Ok(())
    }
    fn halt(&mut self) -> Result<(), Error> {
        self.ops.push(Op::Halt);
        Ok(())
    }
}

fn run_sequence<F>(run: F) -> std::vec::Vec<Op>
where
    F: FnOnce(&mut RecOps),
{
    let mut ops = RecOps::default();
    run(&mut ops);
    ops.ops
}

/// Drive `ResetProgram` against the scripted `reads`, returning its result and op stream.
fn reset_program(reads: &[(u32, u32)]) -> (Result<(), Error>, std::vec::Vec<Op>) {
    let mut ops = RecOps {
        reads: reads.to_vec(),
        ..RecOps::default()
    };
    let r = RP2040.reset_sequence().set_target_state(
        &mut ops,
        TargetState::ResetProgram,
        ResetType::Hardware,
    );
    (r, ops.ops)
}

/// The RP2040 supplies its own reset sequence through `Target::reset_sequence`.
#[test]
fn target_supplies_custom_reset_sequence() {
    let target = RP2040;
    let seq = target.reset_sequence();
    // A distinct object from the generic shared instance.
    assert!(!core::ptr::eq(
        seq as *const _ as *const (),
        &GENERIC_RESET_SEQUENCE as *const _ as *const (),
    ));
}

/// RP2040 `RESETS.RESET_DONE` with DMA out of reset.
const DMA_DONE: (u32, u32) = (0x4000_C008, 1 << 2);
/// `DHCSR` with `S_REGRDY` set.
const REGRDY: (u32, u32) = (0xE000_EDF0, 1 << 16);

/// RP2040 `PSM.FRCE_OFF` with `PROC1` set.
const PROC1_OFF: (u32, u32) = (0x4001_0004, 1 << 16);

/// `ResetProgram` halts, then stops the watchdog, core 1 and DMA, without resetting the chip.
#[test]
fn rp2040_reset_program_halts_and_quiesces() {
    let (r, ops) = reset_program(&[PROC1_OFF, DMA_DONE, REGRDY]);
    r.expect("rp2040 sequence");
    assert_eq!(
        ops,
        [
            Op::Halt,
            // DEMCR: VC_HARDERR, so an algorithm fault halts.
            Op::Write(0xE000_EDFC, 1 << 10),
            // WATCHDOG.CTRL.ENABLE cleared through the CLR alias.
            Op::Write(0x4005_B000, 1 << 30),
            // PSM.FRCE_OFF.PROC1 set, read back, then released.
            Op::Write(0x4001_2004, 1 << 16),
            Op::Read(0x4001_0004),
            Op::Write(0x4001_3004, 1 << 16),
            // RESETS.RESET.DMA pulsed, then RESET_DONE.DMA awaited.
            Op::Write(0x4000_E000, 1 << 2),
            Op::Write(0x4000_F000, 1 << 2),
            Op::Read(0x4000_C008),
            // MPU_CTRL, then CONTROL and PRIMASK through DCRDR and DCRSR.
            Op::Write(0xE000_ED94, 0),
            Op::Write(0xE000_EDF8, 0),
            Op::Write(0xE000_EDF4, 0x1_0014),
            Op::Read(0xE000_EDF0),
        ]
    );
}

/// A `FRCE_OFF` write that never reads back fails the session, but is still released.
#[test]
fn rp2040_core1_stuck_fails_after_releasing_it() {
    let (r, ops) = reset_program(&[DMA_DONE]);
    assert_eq!(r, Err(Error::Timeout));
    assert_eq!(ops.last(), Some(&Op::Write(0x4001_3004, 1 << 16)));
}

/// `PSM.DONE` is never consulted, so neither reading of it can fail the session.
#[test]
fn rp2040_done_cannot_fail_the_quiesce() {
    for done in [0, 1 << 16] {
        let (r, ops) = reset_program(&[PROC1_OFF, DMA_DONE, REGRDY, (0x4001_000C, done)]);
        r.expect("rp2040 sequence");
        assert!(!ops.contains(&Op::Read(0x4001_000C)));
    }
}

/// The non-reset transitions match the generic op stream.
#[test]
fn rp2040_non_reset_transitions_are_generic() {
    let seq = RP2040.reset_sequence();
    // Non-reset transitions still match the generic op stream under the target's reset type.
    for state in [TargetState::ResetHold, TargetState::Halt] {
        let custom = run_sequence(|ops| {
            seq.set_target_state(ops, state, ResetType::Hardware)
                .expect("rp2040 sequence");
        });
        let generic = run_sequence(|ops| {
            GENERIC_RESET_SEQUENCE
                .set_target_state(ops, state, ResetType::Hardware)
                .expect("generic sequence");
        });
        assert_eq!(custom, generic, "state {state:?} must match generic");
    }
}

/// Reset-to-run is the generic release pinned to a software reset, never an nRESET pulse.
#[test]
fn rp2040_post_flash_reset_is_forced_software_reset() {
    // CoreSight debug-register values the release sequence writes (architectural constants).
    const DHCSR: u32 = 0xE000_EDF0;
    const DBGKEY: u32 = 0xA05F_0000;
    const DEMCR: u32 = 0xE000_EDFC;

    let target = RP2040;
    let seq = target.reset_sequence();

    for state in [TargetState::PostFlashReset, TargetState::ResetRun] {
        // Even a caller-supplied hardware reset type is overridden with the software reset.
        let ops = run_sequence(|ops| {
            seq.set_target_state(ops, state, ResetType::Hardware)
                .expect("rp2040 sequence");
        });
        assert_eq!(
            ops,
            [
                Op::Write(DEMCR, 0),
                Op::Write(DHCSR, DBGKEY),
                Op::Reset(ResetType::SoftwareSysReset),
            ],
            "state {state:?} must release the core and force a software reset"
        );
    }
}
