//! Host tests for the shared nRF52 NVMC algorithm and the family's CTRL-AP reset/unlock sequence.

use daplink_flash::error::Error;
use daplink_flash::reset_sequence::{
    ResetOps, ResetSequence, UnlockOutcome, UnlockPolicy, GENERIC_RESET_SEQUENCE,
};
use daplink_flash::target::{ResetType, TargetState};

use crate::algorithms::nrf52::{NRF52_ALGO, NRF52_BLOB};
use crate::families::nrf52::{
    Nrf52ResetSequence, CTRL_AP, CTRL_AP_APPROTECTSTATUS, CTRL_AP_RESET, NRF52_RESET_SEQUENCE,
};

#[test]
fn algo_entry_offsets_pinned() {
    let algo = &NRF52_ALGO;
    // FLM symbol offsets plus the 32-byte breakpoint header, with the Thumb bit set.
    assert_eq!(algo.init, 0x21);
    assert_eq!(algo.uninit, 0x61);
    assert_eq!(algo.erase_chip, 0xb9);
    assert_eq!(algo.erase_sector, 0xd5);
    assert_eq!(algo.program_page, 0x155);
    assert_eq!(algo.verify, Some(0x349));
    assert!(algo.verify_returns_address);
}

#[test]
fn algo_entries_are_offsets_not_addresses() {
    let algo = &NRF52_ALGO;
    let blob_len = NRF52_BLOB.len() as u32;
    for offset in [
        algo.init,
        algo.uninit,
        algo.erase_chip,
        algo.erase_sector,
        algo.program_page,
        algo.verify.unwrap(),
    ] {
        assert!(offset < blob_len, "entry 0x{offset:x} is not a blob offset");
    }
}

/// Blob, static data, stack and buffer sit in the first 8 KiB of RAM, in that order.
#[test]
fn algo_ram_layout_is_ordered_and_compact() {
    let algo = &NRF52_ALGO;
    let blob_end = algo.algo_start + NRF52_BLOB.len() as u32;
    assert_eq!(algo.algo_start, 0x2000_0000);
    assert!(algo.syscall.static_base >= algo.algo_start);
    assert!(algo.syscall.static_base < blob_end);
    assert!(algo.syscall.stack_pointer >= blob_end);
    assert!(algo.program_buffer >= algo.syscall.stack_pointer);
    assert_eq!(algo.program_buffer_size, 0x1000);
    assert_eq!(algo.program_buffer + algo.program_buffer_size, 0x2000_2000);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Op {
    ApRead(u8, u8),
    ApWrite(u8, u8, u32),
    Write(u32, u32),
    Read(u32),
    Reset(ResetType),
    PollHalt,
    SetReset(bool),
    Halt,
}

/// Records every operation; `approtect` answers `APPROTECTSTATUS` (0 = locked).
#[derive(Default)]
struct CtrlApMock {
    approtect: u32,
    ops: std::vec::Vec<Op>,
}

impl ResetOps for CtrlApMock {
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
    fn ap_read_raw(&mut self, apsel: u8, addr: u8) -> Result<u32, Error> {
        self.ops.push(Op::ApRead(apsel, addr));
        if apsel == CTRL_AP && addr == CTRL_AP_APPROTECTSTATUS {
            Ok(self.approtect)
        } else {
            Ok(0)
        }
    }
    fn ap_write_raw(&mut self, apsel: u8, addr: u8, value: u32) -> Result<(), Error> {
        self.ops.push(Op::ApWrite(apsel, addr, value));
        Ok(())
    }
}

/// A locked part is refused after one status read, with no write of any kind.
#[test]
fn locked_part_is_refused_under_every_policy() {
    for policy in [UnlockPolicy::Refuse, UnlockPolicy::EraseAll] {
        let mut ops = CtrlApMock::default();
        assert_eq!(
            Nrf52ResetSequence.unlock(&mut ops, policy),
            Err(Error::TargetLocked)
        );
        assert_eq!(ops.ops, [Op::ApRead(CTRL_AP, CTRL_AP_APPROTECTSTATUS)]);
    }
}

#[test]
fn open_part_reports_not_locked() {
    let mut ops = CtrlApMock {
        approtect: 1,
        ..CtrlApMock::default()
    };
    assert_eq!(
        NRF52_RESET_SEQUENCE.unlock(&mut ops, UnlockPolicy::Refuse),
        Ok(UnlockOutcome::NotLocked)
    );
    assert_eq!(ops.ops, [Op::ApRead(CTRL_AP, CTRL_AP_APPROTECTSTATUS)]);
}

/// A software reset cannot latch a hold, so `ResetHold` writes CTRL-AP `RESET = 1` instead.
#[test]
fn reset_hold_latches_ctrl_ap_reset() {
    let mut ops = CtrlApMock::default();
    NRF52_RESET_SEQUENCE
        .set_target_state(
            &mut ops,
            TargetState::ResetHold,
            ResetType::SoftwareSysReset,
        )
        .unwrap();
    assert_eq!(ops.ops, [Op::ApWrite(CTRL_AP, CTRL_AP_RESET, 1)]);
}

#[test]
fn other_states_match_the_generic_sequence() {
    for state in [
        TargetState::ResetProgram,
        TargetState::Halt,
        TargetState::ResetRun,
        TargetState::PostFlashReset,
    ] {
        let mut custom = CtrlApMock::default();
        NRF52_RESET_SEQUENCE
            .set_target_state(&mut custom, state, ResetType::SoftwareSysReset)
            .unwrap();
        let mut generic = CtrlApMock::default();
        GENERIC_RESET_SEQUENCE
            .set_target_state(&mut generic, state, ResetType::SoftwareSysReset)
            .unwrap();
        assert_eq!(custom.ops, generic.ops, "state {state:?}");
    }
}
