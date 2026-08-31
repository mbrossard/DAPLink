//! Host tests pinning the RP2350 record's geometry, identity and debug-port connection policy.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{ResetOps, ResetSequence, GENERIC_RESET_SEQUENCE};
use daplink_flash::target::{ApAddressing, DpWake, ResetType, TargetState};

use crate::targets::rp235x::{
    RP235x as RP2350, RP2350_ARM_NS_UF2_FAMILY, RP2350_ARM_S_UF2_FAMILY, RP2350_CHIP_ID_EXPECTED,
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
        Ok(0)
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

/// The RP2350 supplies its own reset sequence (not the shared generic instance), proving the
/// per-target seam is wired through `Target::reset_sequence`.
#[test]
fn target_supplies_custom_reset_sequence() {
    let seq = RP2350.reset_sequence();
    assert!(!core::ptr::eq(
        seq as *const _ as *const (),
        &GENERIC_RESET_SEQUENCE as *const _ as *const (),
    ));
}

/// `ResetProgram` is halt-only and triggers no reset at all; the remaining non-reset transitions
/// match the generic op stream.
#[test]
fn rp2350_reset_program_is_halt_only_others_generic() {
    let seq = RP2350.reset_sequence();

    // ResetProgram just halts the running core, so the selected multidrop DP is never dropped.
    let reset_program = run_sequence(|ops| {
        seq.set_target_state(ops, TargetState::ResetProgram, ResetType::Hardware)
            .expect("rp2350 sequence");
    });
    assert_eq!(reset_program, [Op::Halt], "ResetProgram is halt-only");
    assert!(
        !reset_program.iter().any(|op| matches!(op, Op::Reset(_))),
        "ResetProgram must not trigger a reset that would drop the multidrop DP",
    );

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

/// Reset-to-run is the generic release with the reset type pinned to software, whatever the
/// caller passes -- never an nRESET pulse.
#[test]
fn rp2350_post_flash_reset_is_forced_software_reset() {
    // CoreSight debug-register values the release sequence writes (architectural constants).
    const DHCSR: u32 = 0xE000_EDF0;
    const DBGKEY: u32 = 0xA05F_0000;
    const DEMCR: u32 = 0xE000_EDFC;
    const TRCENA: u32 = 1 << 24;

    let seq = RP2350.reset_sequence();

    for state in [TargetState::PostFlashReset, TargetState::ResetRun] {
        let ops = run_sequence(|ops| {
            seq.set_target_state(ops, state, ResetType::Hardware)
                .expect("rp2350 sequence");
        });
        assert_eq!(
            ops,
            [
                Op::Write(DEMCR, TRCENA),
                Op::Write(DHCSR, DBGKEY),
                Op::Reset(ResetType::SoftwareSysReset),
            ],
            "state {state:?} must release the core and force a software reset"
        );
    }
}
