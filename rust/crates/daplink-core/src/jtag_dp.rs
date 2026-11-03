//! JTAG-DP ACK decoding for both JTAG DP protocols, learning each TAP's protocol from its ACKs.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering::Relaxed};

use crate::dap_wire::transfer_idle_cycles;

/// The only ACK a DPACC/APACC scan exits early on: the DP discards that scan's Update-DR.
pub const ACK_WAIT: u32 = 0b001;
/// Protocol 0's OK, protocol 1's FAULT.
pub const ACK_OK_FAULT: u32 = 0b010;

/// `DAP_Transfer` status: OK.
pub const STATUS_OK: u8 = 0x01;
/// `DAP_Transfer` status: WAIT.
pub const STATUS_WAIT: u8 = 0x02;
/// `DAP_Transfer` status: FAULT.
pub const STATUS_FAULT: u8 = 0x04;
/// `DAP_Transfer` status: no or invalid ACK.
pub const STATUS_NO_ACK: u8 = 0x07;

/// Per-TAP JTAG DP protocol, the last failure and the SWD idle cycles, shared by engine and probe.
pub struct JtagDp {
    protocol1: AtomicU32,
    failure: AtomicU8,
    swd_idle: AtomicU8,
    /// Set while a flash session holds the bus: its transfers clock no host idle cycles.
    flash: AtomicBool,
}

impl JtagDp {
    /// Every TAP protocol 0, no failure recorded.
    pub const fn new() -> Self {
        Self {
            protocol1: AtomicU32::new(0),
            failure: AtomicU8::new(0),
            swd_idle: AtomicU8::new(0),
            flash: AtomicBool::new(false),
        }
    }

    /// Decode TAP `index`'s ACK; `0b100` exists only in protocol 1, where `0b010` is FAULT.
    #[inline]
    pub fn status(&self, index: u8, ack: u32) -> u8 {
        let tap = 1u32.checked_shl(u32::from(index)).unwrap_or(0);
        let protocol1 = self.protocol1.load(Relaxed);
        let status = match ack {
            0b100 => {
                self.protocol1.store(protocol1 | tap, Relaxed);
                STATUS_OK
            }
            0b010 if protocol1 & tap != 0 => STATUS_FAULT,
            0b010 => STATUS_OK,
            ACK_WAIT => STATUS_WAIT,
            _ => STATUS_NO_ACK,
        };
        if matches!(status, STATUS_FAULT | STATUS_NO_ACK) {
            self.failure.store(status, Relaxed);
        }
        status
    }

    /// Whether any TAP is protocol 1.
    pub fn any_protocol1(&self) -> bool {
        self.protocol1.load(Relaxed) != 0
    }

    /// Forget every TAP's protocol, for a session that may face another target.
    pub fn reset(&self) {
        self.protocol1.store(0, Relaxed);
    }

    /// Keep the SWD idle cycles a TransferConfigure `report` sets, which dap-rs's `Swd` never sees.
    pub fn observe(&self, report: &[u8]) {
        if let Some(cycles) = transfer_idle_cycles(report) {
            self.swd_idle.store(cycles, Relaxed);
        }
    }

    /// The SWD idle cycles after each OK transfer: 0 until a host sets them, and 0 while flashing.
    #[inline(always)]
    pub fn swd_idle(&self) -> u8 {
        if self.flash.load(Relaxed) {
            0
        } else {
            self.swd_idle.load(Relaxed)
        }
    }

    /// Mark a flash session as holding the bus; the host's setting is kept for after it.
    pub fn set_flashing(&self, on: bool) {
        self.flash.store(on, Relaxed);
    }

    /// The status the last FAULT or no-ACK recorded since the previous take, or 0.
    pub fn take_failure(&self) -> u8 {
        let failure = self.failure.load(Relaxed);
        self.failure.store(0, Relaxed);
        failure
    }
}

/// Dependencies whose JTAG engine decodes ACKs through a [`JtagDp`].
pub trait JtagDpSource {
    /// The engine's JTAG-DP and SWD idle state; `None` when it keeps neither.
    fn jtag_dp(&self) -> Option<&'static JtagDp>;
}

impl Default for JtagDp {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_0_reads_010_as_ok_and_waits_on_001() {
        let dp = JtagDp::new();
        assert_eq!(dp.status(0, 0b010), STATUS_OK);
        assert_eq!(dp.status(0, 0b001), STATUS_WAIT);
        assert_eq!(dp.take_failure(), 0);
    }

    #[test]
    fn an_ok_of_100_makes_that_tap_protocol_1() {
        let dp = JtagDp::new();
        assert_eq!(dp.status(1, 0b100), STATUS_OK);
        assert_eq!(dp.status(1, 0b010), STATUS_FAULT);
        assert_eq!(dp.status(0, 0b010), STATUS_OK);
        assert_eq!(dp.take_failure(), STATUS_FAULT);
        assert_eq!(dp.take_failure(), 0);
        dp.reset();
        assert_eq!(dp.status(1, 0b010), STATUS_OK);
    }

    #[test]
    fn every_other_ack_is_no_ack() {
        let dp = JtagDp::new();
        for ack in [0b000, 0b011, 0b101, 0b110, 0b111] {
            assert_eq!(dp.status(0, ack), STATUS_NO_ACK);
            assert_eq!(dp.take_failure(), STATUS_NO_ACK);
        }
    }

    #[test]
    fn only_a_complete_transfer_configure_sets_the_swd_idle_cycles() {
        let dp = JtagDp::new();
        assert_eq!(dp.swd_idle(), 0);
        dp.observe(&[4, 3, 0, 0, 0]);
        dp.observe(&[5, 3, 0, 0, 0, 0]);
        assert_eq!(dp.swd_idle(), 0);
        dp.observe(&[4, 3, 0, 0, 0, 0]);
        assert_eq!(dp.swd_idle(), 3);
    }

    #[test]
    fn a_flash_session_mutes_the_idle_cycles_and_keeps_the_host_value() {
        let dp = JtagDp::new();
        dp.observe(&[4, 3, 0, 0, 0, 0]);
        dp.set_flashing(true);
        assert_eq!(dp.swd_idle(), 0);
        dp.observe(&[4, 5, 0, 0, 0, 0]);
        dp.set_flashing(false);
        assert_eq!(dp.swd_idle(), 5);
    }

    #[test]
    fn an_index_past_the_mask_stays_protocol_0() {
        let dp = JtagDp::new();
        assert_eq!(dp.status(40, 0b100), STATUS_OK);
        assert_eq!(dp.status(40, 0b010), STATUS_OK);
    }
}
