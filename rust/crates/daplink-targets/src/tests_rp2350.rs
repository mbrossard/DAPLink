//! Host tests pinning the RP2350 record's geometry, identity and debug-port connection policy.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{ResetOps, ResetSequence, GENERIC_RESET_SEQUENCE};
use daplink_flash::target::{ApAddressing, DpWake, ResetType, TargetState};

use crate::targets::rp2350::{
    RP2350, RP2350_ARM_NS_UF2_FAMILY, RP2350_ARM_S_UF2_FAMILY, RP2350_CHIP_ID_EXPECTED,
    RP2350_CORE0_AP_BASE, RP2350_CORE0_TARGETSEL, RP2350_RISCV_UF2_FAMILY,
};
use crate::{check_identity, Target};

#[test]
fn uf2_family_is_arm_secure_and_rejected_ids_are_distinct() {
    // Microsoft uf2families.json: "Raspberry Pi RP2350, Arm Secure" -> 0xe48bff59.
    assert_eq!(RP2350_ARM_S_UF2_FAMILY, 0xE48B_FF59);
    assert_eq!(RP2350.uf2_family(), Some(0xE48B_FF59));
    // The RISC-V and Arm-non-secure families exist but are not flashable by this Arm-secure entry.
    assert_eq!(RP2350_RISCV_UF2_FAMILY, 0xE48B_FF5A);
    assert_eq!(RP2350_ARM_NS_UF2_FAMILY, 0xE48B_FF5B);
    assert_ne!(RP2350_ARM_S_UF2_FAMILY, RP2350_RISCV_UF2_FAMILY);
    assert_ne!(RP2350_ARM_S_UF2_FAMILY, RP2350_ARM_NS_UF2_FAMILY);
}

#[test]
fn identity_probe_reads_chip_id_with_revision_masked() {
    let probes = RP2350.identity_probes();
    assert_eq!(probes.len(), 1);
    let p = probes[0];
    // SYSINFO.CHIP_ID lives at SYSINFO_BASE; the REVISION nibble [31:28] is masked off.
    assert_eq!(p.addr, 0x4000_0000);
    assert_eq!(p.mask, 0x0FFF_FFFF);
    assert_eq!(p.expected, RP2350_CHIP_ID_EXPECTED);
    // Pico 2 reads CHIP_ID = 0x2000_4927 (PART 0x0004, MANUFACTURER 0x493); revision masked off.
    assert_eq!(RP2350_CHIP_ID_EXPECTED, 0x0000_4927);

    // Any REVISION nibble still matches (A2/A3/A4 differ); a wrong PART/MANUFACTURER aborts.
    assert!(check_identity(probes, |_| Ok(0x2000_0000 | RP2350_CHIP_ID_EXPECTED)).is_ok());
    assert!(check_identity(probes, |_| Ok(0x0000_0000)).is_err());
}

#[test]
fn flash_geometry_is_4m_with_4k_sectors() {
    let regions = RP2350.flash_regions();
    assert_eq!(regions.len(), 1);
    let r = &regions[0];
    // Pico 2: 4 MiB external QSPI flash at the XIP base.
    assert_eq!(r.start, 0x1000_0000);
    assert_eq!(r.end, 0x1040_0000);
    assert_eq!(r.end - r.start, 4 * 1024 * 1024);
    assert!(r.is_default);

    let sectors = RP2350.sectors();
    assert_eq!(sectors.len(), 1);
    assert_eq!(sectors[0].size, 0x1000);
    assert_eq!(sectors[0].count, 1024);
    assert_eq!(sectors[0].end(), 0x1040_0000);
}

#[test]
fn ram_region_pinned() {
    let ram = RP2350.ram_regions();
    assert_eq!(ram.len(), 1);
    assert_eq!(ram[0].start, 0x2000_0000);
    assert_eq!(ram[0].end, 0x2008_2000);
}

#[test]
fn debug_port_is_dormant_wake_adiv6_multidrop_core0() {
    let dp = RP2350.debug_port();
    assert_eq!(dp.wake, DpWake::DormantToSwd);
    assert_eq!(
        dp.targetsel,
        Some(RP2350_CORE0_TARGETSEL),
        "RP2350's multidrop wire needs core-0 selected before DPIDR answers"
    );
    assert_eq!(RP2350_CORE0_TARGETSEL, 0x0004_0927);
    assert_eq!(
        dp.ap,
        ApAddressing::V6 {
            ap_base: RP2350_CORE0_AP_BASE
        }
    );
    assert_eq!(RP2350_CORE0_AP_BASE, 0x2000);
}

#[test]
fn flash_algorithm_is_available_and_wired() {
    // The RAM-resident bootrom-shim algorithm is present, so bring-up flows into a real flash run.
    assert!(RP2350.flash_algorithm_available());

    // The flash region references a non-empty algorithm blob loaded into the RAM window.
    let algo = RP2350.flash_regions()[0].algo;
    assert!(!algo.algo_blob.is_empty());
    assert_eq!(algo.algo_start, 0x2000_0000);

    // The shim's `Verify` restores XIP read mode, compares, then restores the serial command state.
    assert_eq!(algo.verify, Some(0xd6));
    assert!(algo.verify_returns_address);
    assert_eq!(algo.program_buffer, 0x2000_2000);
    assert_eq!(algo.syscall.stack_pointer, 0x2000_2000);
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
    let r = RP2350.reset_sequence().set_target_state(
        &mut ops,
        TargetState::ResetProgram,
        ResetType::Hardware,
    );
    (r, ops.ops)
}

/// The RP2350 supplies its own reset sequence through `Target::reset_sequence`.
#[test]
fn target_supplies_custom_reset_sequence() {
    let seq = RP2350.reset_sequence();
    assert!(!core::ptr::eq(
        seq as *const _ as *const (),
        &GENERIC_RESET_SEQUENCE as *const _ as *const (),
    ));
}

/// RP2350 `RESETS.RESET_DONE` with DMA out of reset.
const DMA_DONE: (u32, u32) = (0x4002_0008, 1 << 2);

/// RP2350 `PSM.FRCE_OFF` with `PROC1` set.
const PROC1_OFF: (u32, u32) = (0x4001_8004, 1 << 24);

/// `DHCSR` with `S_REGRDY` set.
const REGRDY: (u32, u32) = (0xE000_EDF0, 1 << 16);

/// `ResetProgram` halts, restores bus permissions, stops the watchdog, core 1 and DMA, and
/// switches a core halted Non-secure to Secure, all without a reset that would drop the DP.
#[test]
fn rp2350_reset_program_halts_and_quiesces() {
    // DSCSR as read: Non-secure (CDSKEY reads one), then already Secure.
    for (dscsr, cds_write) in [(1 << 17, Some(1 << 16)), ((1 << 17) | (1 << 16), None)] {
        let (r, ops) = reset_program(&[PROC1_OFF, DMA_DONE, REGRDY, (0xE000_EE08, dscsr)]);
        r.expect("rp2350 sequence");
        let mut expect = std::vec![
            Op::Halt,
            // ACCESSCTRL.CFGRESET, with the 0xacce write password.
            Op::Write(0x4006_0008, 0xACCE_0001),
            // DEMCR: every fault's catch, since an app may have enabled the faults.
            Op::Write(0xE000_EDFC, 0xFF0),
            Op::Write(0x400D_B000, 1 << 30),
            Op::Write(0x4001_A004, 1 << 24),
            Op::Read(0x4001_8004),
            Op::Write(0x4001_B004, 1 << 24),
            Op::Write(0x4002_2000, 1 << 2),
            Op::Write(0x4002_3000, 1 << 2),
            Op::Read(0x4002_0008),
            Op::Read(0xE000_EE08),
        ];
        expect.extend(cds_write.map(|v| Op::Write(0xE000_EE08, v)));
        // MSPLIM_S, then PSPLIM_S, written to zero through DCRDR and DCRSR.
        for reg in [0x1_001C, 0x1_001D] {
            expect.extend([
                Op::Write(0xE000_EDF8, 0),
                Op::Write(0xE000_EDF4, reg),
                Op::Read(0xE000_EDF0),
            ]);
        }
        // After CDS: both registers are banked, and the Secure copies are what the algorithm runs under.
        expect.extend([
            Op::Write(0xE000_ED94, 0),
            Op::Write(0xE000_EDF8, 0),
            Op::Write(0xE000_EDF4, 0x1_0014),
            Op::Read(0xE000_EDF0),
        ]);
        assert_eq!(ops, expect, "DSCSR {dscsr:#x}");
    }
}

/// A `FRCE_OFF` write that never reads back fails the session, but is still released.
#[test]
fn rp2350_core1_stuck_fails_after_releasing_it() {
    let (r, ops) = reset_program(&[DMA_DONE]);
    assert_eq!(r, Err(Error::Timeout));
    assert_eq!(ops.last(), Some(&Op::Write(0x4001_B004, 1 << 24)));
}

/// `PSM.DONE` is never consulted, so neither reading of it can fail the session.
#[test]
fn rp2350_done_cannot_fail_the_quiesce() {
    for done in [0, 1 << 24] {
        let (r, ops) = reset_program(&[PROC1_OFF, DMA_DONE, REGRDY, (0x4001_800C, done)]);
        r.expect("rp2350 sequence");
        assert!(!ops.contains(&Op::Read(0x4001_800C)));
    }
}

/// The non-reset transitions match the generic op stream.
#[test]
fn rp2350_non_reset_transitions_are_generic() {
    let seq = RP2350.reset_sequence();

    // Non-reset transitions still match the generic op stream under the target's reset type.
    for state in [TargetState::ResetHold, TargetState::Halt] {
        let custom = run_sequence(|ops| {
            seq.set_target_state(ops, state, ResetType::Hardware)
                .expect("rp2350 sequence");
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
fn rp2350_post_flash_reset_is_forced_software_reset() {
    // CoreSight debug-register values the release sequence writes (architectural constants).
    const DHCSR: u32 = 0xE000_EDF0;
    const DBGKEY: u32 = 0xA05F_0000;
    const DEMCR: u32 = 0xE000_EDFC;

    let seq = RP2350.reset_sequence();

    for state in [TargetState::PostFlashReset, TargetState::ResetRun] {
        let ops = run_sequence(|ops| {
            seq.set_target_state(ops, state, ResetType::Hardware)
                .expect("rp2350 sequence");
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
