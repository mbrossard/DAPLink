//! `SwdHost` tests: reset sequences, connection, detection, recovery, syscalls, memory writes.

use super::*;

mod reset_sequence_tests {
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult};
    use std::vec::Vec;

    /// One thing the mock saw, in order (ordering, not just counts, is under test).
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum Ev {
        MemWrite(u32, u32),
        DhcsrPoll(u32),
        /// A DHCSR read the port refused (`AckWait`, from a stalled access port).
        DhcsrRefused,
        DapAbort,
        StickyClear,
    }

    /// Recording SWD mock, scriptable for the reset window (inert at defaults).
    struct RecMock {
        cfg: Config,
        last_tar: u32,
        writes: Vec<(u32, u32)>,
        resets: Vec<bool>,
        dhcsr_polls: u32,
        seq_count: u32,
        events: Vec<Ev>,
        /// An access-port transaction that never completed: only `DAPABORT` ends it.
        stalled: bool,
        /// The access port answers WAIT to everything and `DAPABORT` does not clear it.
        wait_always: bool,
        /// The AIRCR write faults and leaves the port stalled.
        stall_on_reset_write: bool,
        /// Answered DHCSR reads still to report as not-yet-halted before reporting `S_HALT`.
        polls_before_halt: u32,
        /// DHCSR reads to refuse (`AckWait`) once the reset has been requested.
        refuse_after_reset: u32,
        /// Refusals still owed, counted down one per refused read.
        refuse_polls: u32,
        /// Whether the read in progress is one of the refused ones.
        refusing: bool,
        /// The core never reports halted, however long it is polled.
        never_halts: bool,
        dap_aborts: u32,
    }

    impl RecMock {
        fn new() -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                writes: Vec::new(),
                resets: Vec::new(),
                dhcsr_polls: 0,
                seq_count: 0,
                events: Vec::new(),
                stalled: false,
                wait_always: false,
                stall_on_reset_write: false,
                polls_before_halt: 0,
                refuse_after_reset: 0,
                refuse_polls: 0,
                refusing: false,
                never_halts: false,
                dap_aborts: 0,
            }
        }

        fn wrote(&self, addr: u32, value: u32) -> bool {
            self.writes.iter().any(|&(a, v)| a == addr && v == value)
        }

        /// Index of the first recorded event matching `pred`, for ordering assertions.
        fn first(&self, pred: impl Fn(&Ev) -> bool) -> Option<usize> {
            self.events.iter().position(pred)
        }

        /// Whether the access port refuses everything.
        fn port_stalled(&self) -> bool {
            self.stalled || self.wait_always
        }

        /// Whether the read in progress is refused.
        fn ap_blocked(&self) -> bool {
            self.port_stalled() || self.refusing
        }
    }

    impl Swd<RecMock> for RecMock {
        fn available(_deps: &RecMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if self.ap_blocked() {
                        return Err(SwdErr::AckWait);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => {
                    let polling_dhcsr = self.last_tar == DHCSR;
                    if self.ap_blocked() {
                        if polling_dhcsr && !self.refusing {
                            self.events.push(Ev::DhcsrRefused);
                        }
                        return Err(SwdErr::AckWait);
                    }
                    if polling_dhcsr {
                        self.dhcsr_polls += 1;
                        let value = if self.never_halts || self.polls_before_halt > 0 {
                            self.polls_before_halt = self.polls_before_halt.saturating_sub(1);
                            C_DEBUGEN
                        } else {
                            S_HALT
                        };
                        self.events.push(Ev::DhcsrPoll(value));
                        Ok(value)
                    } else {
                        Ok(0)
                    }
                }
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_DAP => {
                    self.stalled = false;
                    self.dap_aborts += 1;
                    self.events.push(Ev::DapAbort);
                }
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.events.push(Ev::StickyClear);
                }
                (APnDP::AP, _) if self.port_stalled() => return Err(SwdErr::AckWait),
                // TAR: a refused-DHCSR-read is decided once per read, not in the retried read itself.
                (APnDP::AP, DPRegister::CTRLSTAT) => {
                    self.last_tar = data;
                    if data == DHCSR {
                        self.refusing = self.refuse_polls > 0;
                        if self.refusing {
                            self.refuse_polls -= 1;
                            self.events.push(Ev::DhcsrRefused);
                        }
                    }
                }
                (APnDP::AP, DPRegister::RDBUFF) => {
                    if self.last_tar == AIRCR {
                        self.refuse_polls = self.refuse_after_reset;
                    }
                    if self.stall_on_reset_write && self.last_tar == AIRCR {
                        self.stalled = true;
                        self.events.push(Ev::MemWrite(self.last_tar, data));
                        self.writes.push((self.last_tar, data));
                        return Err(SwdErr::AckFault);
                    }
                    self.writes.push((self.last_tar, data));
                    self.events.push(Ev::MemWrite(self.last_tar, data));
                }
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            self.seq_count += 1;
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    impl ResetControl for RecMock {
        type Error = SwdErr;
        fn set_reset(&mut self, asserted: bool) -> Result<(), SwdErr> {
            self.resets.push(asserted);
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// Delay source that records every wait instead of sleeping.
    #[derive(Default)]
    struct RecDelay {
        waits: Vec<u32>,
    }

    impl RecDelay {
        fn total_us(&self) -> u64 {
            self.waits.iter().map(|&us| u64::from(us)).sum()
        }
    }

    impl DelayNs for RecDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.waits.push(ns / 1000);
        }
        fn delay_us(&mut self, us: u32) {
            self.waits.push(us);
        }
    }

    fn run(state: TargetState, reset: ResetType) -> RecMock {
        let mut host = SwdHost::new(RecMock::new(), NoDelay);
        host.set_target_state(state, reset)
            .expect("set_target_state");
        host.into_parts().0
    }

    /// Drive a scripted mock through the software-system-reset reset-and-halt.
    fn run_scripted(m: RecMock, state: TargetState) -> (Result<(), Error>, RecMock, RecDelay) {
        let mut host = SwdHost::new(m, RecDelay::default());
        let outcome = host.set_target_state(state, ResetType::SoftwareSysReset);
        let (m, d) = host.into_parts();
        (outcome, m, d)
    }

    #[test]
    fn sysreset_program_writes_aircr_and_polls_without_reset_line() {
        let m = run(TargetState::ResetProgram, ResetType::SoftwareSysReset);
        assert!(
            m.wrote(DEMCR, VC_CORERESET | VC_HARDERR),
            "arm vector catch"
        );
        assert!(m.wrote(DHCSR, DBGKEY | C_DEBUGEN), "keep debug enabled");
        assert!(
            m.wrote(AIRCR, AIRCR_KEY | SYSRESETREQ),
            "system reset request"
        );
        assert!(m.wrote(DEMCR, VC_HARDERR), "disarm vector catch");
        assert!(m.dhcsr_polls >= 1, "polled S_HALT");
        assert!(m.resets.is_empty(), "software reset must not toggle nRESET");
    }

    #[test]
    fn vectreset_program_writes_vectreset() {
        let m = run(TargetState::ResetProgram, ResetType::SoftwareVectReset);
        assert!(m.wrote(DEMCR, VC_CORERESET | VC_HARDERR));
        assert!(
            m.wrote(AIRCR, AIRCR_KEY | VECTRESET),
            "vector reset request"
        );
        assert!(m.wrote(DEMCR, VC_HARDERR));
        assert!(m.dhcsr_polls >= 1);
        assert!(m.resets.is_empty());
    }

    #[test]
    fn hardware_program_toggles_reset_reinits_and_halts() {
        let m = run(TargetState::ResetProgram, ResetType::Hardware);
        assert!(m.wrote(DEMCR, VC_CORERESET | VC_HARDERR));
        assert!(m.wrote(DHCSR, DBGKEY | C_DEBUGEN));
        assert_eq!(
            m.resets,
            std::vec![true, false],
            "assert then release nRESET"
        );
        assert!(m.seq_count > 0, "re-init debug after the reset pulse");
        assert!(
            !m.writes.iter().any(|&(a, _)| a == AIRCR),
            "hardware reset must not write AIRCR"
        );
        assert!(m.dhcsr_polls >= 1, "polled S_HALT");
        assert!(m.wrote(DEMCR, VC_HARDERR), "disarm vector catch");
    }

    #[test]
    fn sysreset_post_flash_releases_and_resets_without_polling() {
        let m = run(TargetState::PostFlashReset, ResetType::SoftwareSysReset);
        assert!(m.wrote(DEMCR, 0), "clear any vector catch");
        assert!(m.wrote(DHCSR, DBGKEY), "release debug (C_DEBUGEN clear)");
        assert!(m.wrote(AIRCR, AIRCR_KEY | SYSRESETREQ));
        assert_eq!(m.dhcsr_polls, 0, "reset-to-run must not poll S_HALT");
        assert!(m.resets.is_empty());
    }

    #[test]
    fn hardware_post_flash_pulses_reset_without_polling() {
        let m = run(TargetState::PostFlashReset, ResetType::Hardware);
        assert!(m.wrote(DEMCR, 0));
        assert!(m.wrote(DHCSR, DBGKEY));
        assert_eq!(m.resets, std::vec![true, false]);
        assert_eq!(m.dhcsr_polls, 0, "reset-to-run must not poll S_HALT");
    }

    /// The halt poll waits between reads, starting short and doubling up to a cap.
    #[test]
    fn the_halt_poll_gives_the_target_time_between_reads() {
        let mut m = RecMock::new();
        m.polls_before_halt = 5;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(outcome, Ok(()), "a core that halts late still halts");
        assert!(m.dhcsr_polls > 5, "the poll kept asking");

        // Skip the pre-reset poll's own backoff; only the reset window's waits are checked.
        let settle = d
            .waits
            .iter()
            .position(|&us| us == crate::reset_sequence::RESET_SETTLE_US)
            .expect("the sequence settled after the reset");
        let polls: Vec<u32> = d.waits[settle + 1..].to_vec();
        assert!(!polls.is_empty(), "the poll waited between reads");
        assert_eq!(
            polls[0], HALT_POLL_START_US,
            "the first read comes after the starting delay, not immediately"
        );
        for pair in polls.windows(2) {
            let (prev, next) = (pair[0], pair[1]);
            assert!(
                next == HALT_POLL_MAX_US || next == prev * 2,
                "each wait doubles until the cap: {prev} -> {next}"
            );
            assert!(next <= HALT_POLL_MAX_US, "no wait exceeds the cap");
        }
    }

    /// A faulted reset write stalls the AP; the cancel must land before the first halt poll.
    #[test]
    fn the_reset_write_cancels_the_transaction_its_tolerated_fault_left() {
        let mut m = RecMock::new();
        m.stall_on_reset_write = true;
        let (outcome, m, _) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(outcome, Ok(()), "the reset-and-halt completes");
        let reset_write = m
            .first(|e| matches!(e, Ev::MemWrite(AIRCR, _)))
            .expect("the AIRCR write was attempted");
        let cancel = m
            .first(|e| matches!(e, Ev::DapAbort))
            .expect("the stalled transaction was cancelled");
        let first_poll = m
            .events
            .iter()
            .skip(reset_write)
            .position(|e| matches!(e, Ev::DhcsrPoll(_) | Ev::DhcsrRefused))
            .map(|i| i + reset_write)
            .expect("the halt poll ran after the reset");
        assert!(
            reset_write < cancel && cancel < first_poll,
            "the cancel falls between the reset write and the first halt poll \
             (events: {:?})",
            m.events
        );
    }

    /// The poll must survive refused (WAIT) reads after a reset rather than fail on the first one.
    #[test]
    fn the_halt_poll_waits_out_an_access_port_that_answers_wait() {
        let mut m = RecMock::new();
        m.refuse_after_reset = 3;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(outcome, Ok(()), "a port that comes back is not a failure");
        assert!(
            m.first(|e| matches!(e, Ev::DhcsrRefused)).is_some(),
            "the poll met the WAIT it is supposed to survive"
        );
        assert!(
            m.first(|e| matches!(e, Ev::DhcsrPoll(_))).is_some(),
            "and went on to read DHCSR once the port answered"
        );
        assert!(d.total_us() > 0, "time was actually given to the target");
    }

    /// A core that never halts is polled for the whole budget and then fails with a timeout.
    #[test]
    fn a_core_that_never_halts_exhausts_the_budget_and_fails() {
        let mut m = RecMock::new();
        m.never_halts = true;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(
            outcome,
            Err(Error::Timeout),
            "the poll fails rather than waiting forever"
        );
        assert!(
            d.total_us() >= HALT_POLL_BUDGET_US,
            "the whole budget was given to the target first: {} us",
            d.total_us()
        );
        assert!(
            m.dhcsr_polls < 1_000,
            "the budget is spent in time, not in transfers: {} polls",
            m.dhcsr_polls
        );
    }

    /// A port that never answers reports the WAIT as a transfer error, not silent success.
    #[test]
    fn a_port_that_only_ever_waits_fails_with_the_transfer_error() {
        let mut m = RecMock::new();
        m.wait_always = true;
        let (outcome, m, d) = run_scripted(m, TargetState::ResetProgram);

        assert_eq!(
            outcome,
            Err(Error::Swd(crate::error::SwdError::Wait)),
            "the exhausted WAIT is what the caller is told about"
        );
        assert!(
            d.total_us() >= HALT_POLL_BUDGET_US,
            "the port was given the whole budget: {} us",
            d.total_us()
        );
        assert!(
            m.dap_aborts > 0,
            "and the one recovery a WAIT-stalled port responds to was attempted"
        );
    }
}

mod connect_tests {
    //! Debug-port connection tests: wake preambles, multidrop TARGETSEL, ADIv6 addressing.
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use crate::target::{ApAddressing, DebugPort, DpWake};
    use dap_rs::swd::{APnDP, Config, DPRegister, Result as SwdResult};
    use std::vec::Vec;

    struct WireMock {
        cfg: Config,
        /// Every emitted raw sequence, as (num_bits, little-endian bytes).
        seqs: Vec<(usize, Vec<u8>)>,
        select_writes: Vec<u32>,
        /// Last SELECT value (its low nibble is the DP bank the next read observes).
        select: u32,
        dpidr: u32,
        targetid: u32,
        dlpidr: u32,
        clock_requests: Vec<u32>,
        clock_supported: bool,
    }

    impl WireMock {
        fn new() -> Self {
            Self {
                cfg: Config::default(),
                seqs: Vec::new(),
                select_writes: Vec::new(),
                select: 0,
                dpidr: 0x0BC1_1477,
                targetid: 0,
                dlpidr: 0,
                clock_requests: Vec::new(),
                clock_supported: true,
            }
        }
        /// Whether a raw sequence of exactly `num_bits` carrying `bytes` was emitted.
        fn emitted(&self, num_bits: usize, bytes: &[u8]) -> bool {
            self.seqs
                .iter()
                .any(|(n, b)| *n == num_bits && b.as_slice() == bytes)
        }
    }

    impl Swd<WireMock> for WireMock {
        fn available(_deps: &WireMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => Ok(0),
                (APnDP::DP, DPRegister::DPIDR) => Ok(self.dpidr),
                (APnDP::DP, DPRegister::CTRLSTAT) => Ok(match self.select & 0xF {
                    2 => self.targetid,
                    3 => self.dlpidr,
                    _ => CSYSPWRUPACK | CDBGPWRUPACK,
                }),
                (APnDP::DP, _) => Ok(0),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            if apndp == APnDP::DP && a == DPRegister::SELECT {
                self.select = data;
                self.select_writes.push(data);
            }
            Ok(())
        }
        fn set_clock(&mut self, max_frequency: u32) -> bool {
            self.clock_requests.push(max_frequency);
            self.clock_supported
        }
        fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> SwdResult<()> {
            self.seqs.push((num_bits, data.to_vec()));
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    fn host(m: WireMock) -> SwdHost<WireMock, NoDelay, WireMock> {
        SwdHost::new(m, NoDelay)
    }

    #[test]
    fn externally_configured_clock_does_not_block_debug_init() {
        let mut mock = WireMock::new();
        mock.clock_supported = false;
        let mut h = host(mock);
        h.init_debug().expect("externally configured transport");
        let (m, _) = h.into_parts();
        assert_eq!(m.clock_requests, [BRINGUP_HZ]);
        assert!(m.emitted(16, &0xE79Eu16.to_le_bytes()));
        assert!(!m.select_writes.is_empty());
    }

    #[test]
    fn classic_wake_emits_jtag_to_swd_switch_not_dormant_alert() {
        let mut h = host(WireMock::new());
        h.set_dp_config(DebugPort::default());
        h.init_debug().expect("classic connect");
        let (m, _) = h.into_parts();
        assert!(
            m.emitted(16, &0xE79Eu16.to_le_bytes()),
            "classic switch word present"
        );
        assert!(
            !m.emitted(64, &SELECTION_ALERT_LO.to_le_bytes()),
            "no dormant selection alert on the classic path"
        );
    }

    #[test]
    fn dormant_wake_emits_selection_alert_and_activation() {
        let mut h = host(WireMock::new());
        h.set_dp_config(DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: None,
            ap: ApAddressing::V6 { ap_base: 0x2000 },
        });
        h.init_debug().expect("dormant connect");
        let (m, _) = h.into_parts();
        assert!(
            m.emitted(31, &SWD_TO_DORMANT),
            "swd->dormant (31-bit select)"
        );
        assert!(m.emitted(64, &SELECTION_ALERT_LO.to_le_bytes()), "alert lo");
        assert!(m.emitted(64, &SELECTION_ALERT_HI.to_le_bytes()), "alert hi");
        assert!(m.emitted(12, &SWD_ACTIVATION.to_le_bytes()), "activation");
        assert!(
            !m.emitted(16, &0xE79Eu16.to_le_bytes()),
            "no classic switch on the dormant path"
        );
    }

    /// Blind 48-bit TARGETSEL matching the requested value, followed by a passing verify.
    #[test]
    fn multidrop_emits_blind_targetsel_and_verifies() {
        const TARGETSEL: u32 = 0x0100_2927;
        let mut m = WireMock::new();
        m.targetid = TARGETSEL & TARGETID_MASK;
        m.dlpidr = TARGETSEL & DLPIDR_MASK;
        let mut h = host(m);
        h.set_dp_config(DebugPort {
            wake: DpWake::JtagToSwd,
            targetsel: Some(TARGETSEL),
            ap: ApAddressing::V5,
        });
        h.init_debug().expect("multidrop connect");
        let (m, _) = h.into_parts();

        let parity = u64::from(TARGETSEL.count_ones() & 1);
        let seq = (parity << 45) | (u64::from(TARGETSEL) << 13) | 0x1F99;
        assert!(
            m.emitted(48, &seq.to_le_bytes()[..6]),
            "blind 48-bit TARGETSEL emitted"
        );
        assert!(
            m.select_writes.contains(&2),
            "selected DP bank 2 (TARGETID)"
        );
        assert!(m.select_writes.contains(&3), "selected DP bank 3 (DLPIDR)");
    }

    #[test]
    fn multidrop_verify_mismatch_fails() {
        const TARGETSEL: u32 = 0x0100_2927;
        let mut m = WireMock::new();
        m.targetid = 0x0BAD_BEEF & TARGETID_MASK;
        m.dlpidr = TARGETSEL & DLPIDR_MASK;
        let mut h = host(m);
        h.set_dp_config(DebugPort {
            wake: DpWake::JtagToSwd,
            targetsel: Some(TARGETSEL),
            ap: ApAddressing::V5,
        });
        assert_eq!(h.init_debug().unwrap_err(), Error::DebugInitFailed);
    }

    /// ADIv6 programs SELECT with the AP base ORed with the 0xD00 register page.
    #[test]
    fn adiv6_programs_ap_base_page_into_select() {
        let mut h = host(WireMock::new());
        h.set_dp_config(DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: None,
            ap: ApAddressing::V6 { ap_base: 0x2000 },
        });
        h.init_debug().expect("adiv6 connect");
        let _ = h.read_word(0x4000_0000).expect("mem read over adiv6");
        let (m, _) = h.into_parts();
        assert!(
            m.select_writes.contains(&0x2D00),
            "AP base 0x2000 | 0xD00 register page written to SELECT"
        );
        assert!(
            !m.select_writes.contains(&0x2000),
            "ADIv6 must target the 0xD00 register page, not the bare AP base 0x2000"
        );
        assert!(
            !m.select_writes.contains(&0),
            "ADIv6 must not fall back to the ADIv5 apsel<<24 = 0 SELECT"
        );
    }

    /// A dormant ADIv6 multidrop attach emits the line reset and blind TARGETSEL before DPIDR.
    #[test]
    fn dormant_wake_with_targetsel_emits_line_reset_and_blind_targetsel() {
        const TARGETSEL: u32 = 0x0004_0927;
        let mut m = WireMock::new();
        m.targetid = TARGETSEL & TARGETID_MASK;
        m.dlpidr = TARGETSEL & DLPIDR_MASK;
        let mut h = host(m);
        h.set_dp_config(DebugPort {
            wake: DpWake::DormantToSwd,
            targetsel: Some(TARGETSEL),
            ap: ApAddressing::V6 { ap_base: 0x2000 },
        });
        h.init_debug().expect("dormant + multidrop connect");
        let (m, _) = h.into_parts();

        assert!(m.emitted(64, &SELECTION_ALERT_LO.to_le_bytes()), "alert lo");
        assert!(
            m.emitted(54, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]),
            "terminated line reset before TARGETSEL / DPIDR"
        );
        let parity = u64::from(TARGETSEL.count_ones() & 1);
        let seq = (parity << 45) | (u64::from(TARGETSEL) << 13) | 0x1F99;
        assert!(
            m.emitted(48, &seq.to_le_bytes()[..6]),
            "blind 48-bit TARGETSEL emitted on the dormant path"
        );
        assert!(m.select_writes.contains(&2), "read TARGETID (DP bank 2)");
        assert!(m.select_writes.contains(&3), "read DLPIDR (DP bank 3)");
    }
}

mod detect_tests {
    //! Detection-read tests: a faulted read recovers and leaves the port usable.
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult};

    /// SWD mock with a poisoned address: reading it faults and latches a sticky error.
    struct FaultMock {
        cfg: Config,
        last_tar: u32,
        poison: u32,
        mem_value: u32,
        sticky: bool,
        abort_writes: u32,
        dap_aborts: u32,
    }

    impl FaultMock {
        fn new(poison: u32, mem_value: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                poison,
                mem_value,
                sticky: false,
                abort_writes: 0,
                dap_aborts: 0,
            }
        }
    }

    impl Swd<FaultMock> for FaultMock {
        fn available(_deps: &FaultMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if self.sticky {
                        return Err(SwdErr::AckFault);
                    }
                    if self.last_tar == self.poison {
                        self.sticky = true;
                        return Err(SwdErr::AckFault);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => {
                    if self.sticky {
                        Err(SwdErr::AckFault)
                    } else {
                        Ok(self.mem_value)
                    }
                }
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::AP, DPRegister::CTRLSTAT) => self.last_tar = data,
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.sticky = false;
                    self.abort_writes += 1;
                }
                // DAPABORT: nothing is stalled in this model, so it only records the cancel.
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_DAP => self.dap_aborts += 1,
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// A faulted detection read recovers the port; a following good-address read succeeds.
    #[test]
    fn detect_read_word_clears_sticky_and_recovers() {
        const POISON: u32 = 0x1000_0100;
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(FaultMock::new(POISON, 0x0000_0410), NoDelay);
        host.set_detect_budget(8, 64);

        assert!(
            host.detect_read_word(POISON).is_err(),
            "foreign read faults"
        );
        assert_eq!(
            host.core.swd.dap_aborts, 2,
            "each failed attempt ends by cancelling the transaction"
        );
        assert_eq!(
            host.core.swd.abort_writes, 4,
            "per attempt: one clear from the transfer retry, one from the detection recovery"
        );
        assert!(
            !host.core.swd.sticky,
            "sticky error cleared after the faulted read"
        );

        assert_eq!(
            host.detect_read_word(GOOD),
            Ok(0x0000_0410),
            "a good read succeeds after recovery"
        );
    }

    /// A persistently faulting address fails, but its recovery leaves the port usable.
    #[test]
    fn plain_read_self_heals_on_the_next_transfer() {
        const POISON: u32 = 0x1000_0100;
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(FaultMock::new(POISON, 0x0000_0410), NoDelay);

        assert!(host.read_word(POISON).is_err(), "poison read faults");
        assert!(
            host.core.swd.sticky,
            "the failed retry left the fault latched"
        );
        assert_eq!(host.read_word(GOOD), Ok(0x0000_0410));
        assert_eq!(
            host.core.swd.abort_writes, 2,
            "one ABORT per recovering transfer"
        );
    }

    /// MEM-AP mock: an unmapped read latches sticky flags and stalls the AP until `DAPABORT`.
    struct StallMock {
        cfg: Config,
        last_tar: u32,
        poison: u32,
        mem_value: u32,
        /// Latched error flags: acknowledged by an `ABORT_CLEAR_ALL` write.
        sticky: bool,
        /// An access-port transaction that never completed: cancelled only by `ABORT_DAP`.
        stalled: bool,
        sticky_clears: u32,
        dap_aborts: u32,
    }

    impl StallMock {
        fn new(poison: u32, mem_value: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                poison,
                mem_value,
                sticky: false,
                stalled: false,
                sticky_clears: 0,
                dap_aborts: 0,
            }
        }

        /// WAIT while a transaction is outstanding, FAULT while errors are merely latched.
        fn blocked(&self) -> Option<SwdErr> {
            if self.stalled {
                Some(SwdErr::AckWait)
            } else if self.sticky {
                Some(SwdErr::AckFault)
            } else {
                None
            }
        }
    }

    impl Swd<StallMock> for StallMock {
        fn available(_deps: &StallMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if let Some(e) = self.blocked() {
                        return Err(e);
                    }
                    if self.last_tar == self.poison {
                        self.sticky = true;
                        self.stalled = true;
                        return Err(SwdErr::AckFault);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => match self.blocked() {
                    Some(e) => Err(e),
                    None => Ok(self.mem_value),
                },
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_DAP => {
                    self.stalled = false;
                    self.dap_aborts += 1;
                }
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.sticky = false;
                    self.sticky_clears += 1;
                }
                (APnDP::AP, _) => {
                    if let Some(e) = self.blocked() {
                        return Err(e);
                    }
                    if a == DPRegister::CTRLSTAT {
                        self.last_tar = data;
                    }
                }
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    /// A stalled AP must be cancelled, not merely acknowledged, for the next identity read.
    #[test]
    fn a_stalled_access_port_is_cancelled_so_the_next_candidate_reads() {
        const POISON: u32 = 0x1000_0100;
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(StallMock::new(POISON, 0x0000_0413), NoDelay);
        host.set_detect_budget(8, 64);

        assert!(
            host.detect_read_word(POISON).is_err(),
            "the unmapped identity address faults"
        );
        assert!(
            !host.core.swd.stalled,
            "the outstanding transaction was cancelled, not just acknowledged"
        );
        assert!(host.core.swd.dap_aborts > 0, "the cancel was actually sent");

        assert_eq!(
            host.detect_read_word(GOOD),
            Ok(0x0000_0413),
            "the following candidate's identity read is answered"
        );
    }

    /// A detection read beginning on a port already stalled by an earlier access recovers it.
    #[test]
    fn detect_read_recovers_a_port_left_stalled_by_an_earlier_access() {
        const GOOD: u32 = 0xE004_2000;
        let mut host = SwdHost::new(StallMock::new(0xDEAD_0000, 0x0000_0413), NoDelay);
        host.set_detect_budget(8, 64);
        host.core.swd.stalled = true;

        assert_eq!(
            host.detect_read_word(GOOD),
            Ok(0x0000_0413),
            "a mapped address reads back despite the inherited stall"
        );
        assert_eq!(
            host.core.swd.dap_aborts, 1,
            "one cancel, then the read is retried over a usable port"
        );
    }
}

mod recovery_tests {
    //! Sticky-error recovery tests: one ABORT and re-arm, and no retry loop on a persistent fault.
    extern crate std;

    use super::*;
    use crate::error::SwdError;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult};
    use std::vec::Vec;

    #[test]
    fn sticky_clear_leaves_dapabort_and_stkcmpclr_clear() {
        assert_eq!(ABORT_CLEAR_ALL, 0x1C);
    }

    /// SWD mock whose AP reads fault while `faults_left` is non-zero, latching a sticky error.
    struct TransientMock {
        cfg: Config,
        last_tar: u32,
        mem_value: u32,
        faults_left: u32,
        sticky: bool,
        abort_writes: u32,
        select_writes: Vec<u32>,
        csw_writes: Vec<u32>,
    }

    impl TransientMock {
        fn new(faults_left: u32, mem_value: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                mem_value,
                faults_left,
                sticky: false,
                abort_writes: 0,
                select_writes: Vec::new(),
                csw_writes: Vec::new(),
            }
        }
    }

    impl Swd<TransientMock> for TransientMock {
        fn available(_deps: &TransientMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => {
                    if self.sticky {
                        return Err(SwdErr::AckFault);
                    }
                    if self.faults_left > 0 {
                        self.faults_left -= 1;
                        self.sticky = true;
                        return Err(SwdErr::AckFault);
                    }
                    Ok(0)
                }
                (APnDP::DP, DPRegister::RDBUFF) => {
                    if self.sticky {
                        Err(SwdErr::AckFault)
                    } else {
                        Ok(self.mem_value)
                    }
                }
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::DP, DPRegister::DPIDR) if data == ABORT_CLEAR_ALL => {
                    self.sticky = false;
                    self.abort_writes += 1;
                }
                (APnDP::DP, DPRegister::SELECT) => self.select_writes.push(data),
                (APnDP::AP, _) if self.sticky => return Err(SwdErr::AckFault),
                (APnDP::AP, DPRegister::DPIDR) => self.csw_writes.push(data),
                (APnDP::AP, DPRegister::CTRLSTAT) => self.last_tar = data,
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// One transient fault recovers with a single ABORT and SELECT/CSW re-arm.
    #[test]
    fn transient_fault_recovers_with_abort_select_and_retry() {
        let mut host = SwdHost::new(TransientMock::new(1, 0xDEAD_BEEF), NoDelay);
        assert_eq!(
            host.read_word(0x2000_0000),
            Ok(0xDEAD_BEEF),
            "retried read succeeds"
        );
        let m = host.into_parts().0;
        assert_eq!(m.abort_writes, 1, "recovery wrote ABORT once");
        assert_eq!(
            m.select_writes,
            std::vec![0, 0],
            "SELECT re-issued during recovery"
        );
        assert_eq!(m.csw_writes, std::vec![CSW_WORD_ADDRINC, CSW_WORD_ADDRINC]);
    }

    /// A persistent fault gets exactly one recovery attempt, then surfaces the original error.
    #[test]
    fn persistent_fault_returns_original_error_after_one_retry() {
        let mut host = SwdHost::new(TransientMock::new(u32::MAX, 0), NoDelay);
        assert_eq!(
            host.read_word(0x2000_0000),
            Err(Error::Swd(SwdError::Fault))
        );
        let m = host.into_parts().0;
        assert_eq!(m.abort_writes, 1, "exactly one recovery attempt, no loop");
    }
}

mod syscall_tests {
    //! Flash-syscall calling-convention tests: register setup, resume, S_HALT poll, timeout.
    extern crate std;

    use super::*;
    use crate::flash_algo::SyscallParams;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Result as SwdResult};
    use std::vec::Vec;

    /// Recording core-debug mock with fixed DHCSR and DCRDR (r0) values.
    struct CoreMock {
        cfg: Config,
        last_tar: u32,
        writes: Vec<(u32, u32)>,
        /// Value every DHCSR read reports (S_REGRDY, with or without S_HALT).
        dhcsr: u32,
        dcrdr: u32,
        dhcsr_polls: u32,
        /// The last DCRSR selector, and what r15 reads as.
        selected: u32,
        pc: u32,
    }

    impl CoreMock {
        fn new(dhcsr: u32, dcrdr: u32) -> Self {
            Self {
                cfg: Config::default(),
                last_tar: 0,
                writes: Vec::new(),
                dhcsr,
                dcrdr,
                dhcsr_polls: 0,
                selected: 0,
                pc: PARAMS.breakpoint & !1,
            }
        }

        /// How many times r15 was read through DCRSR.
        fn pc_reads(&self) -> usize {
            self.writes.iter().filter(|w| **w == (DCRSR, 15)).count()
        }
    }

    impl Swd<CoreMock> for CoreMock {
        fn available(_deps: &CoreMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
            match (apndp, a) {
                (APnDP::AP, _) => Ok(0),
                (APnDP::DP, DPRegister::RDBUFF) => match self.last_tar {
                    DHCSR => {
                        self.dhcsr_polls += 1;
                        Ok(self.dhcsr)
                    }
                    DCRDR if self.selected == 15 => Ok(self.pc),
                    DCRDR => Ok(self.dcrdr),
                    _ => Ok(0),
                },
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::AP, DPRegister::CTRLSTAT) => self.last_tar = data,
                (APnDP::AP, DPRegister::RDBUFF) => {
                    if self.last_tar == DCRSR {
                        self.selected = data;
                    }
                    self.writes.push((self.last_tar, data));
                }
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    const PARAMS: SyscallParams = SyscallParams {
        breakpoint: 0x2000_0004,
        static_base: 0x2000_0800,
        stack_pointer: 0x2000_1000,
    };
    const ENTRY: u32 = 0x2000_0040;

    /// The full calling convention, pinned as an ordered op-stream over the debug registers.
    #[test]
    fn flash_syscall_sets_registers_resumes_and_returns_r0() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY | S_HALT, 0x2A), NoDelay);
        let ret = host
            .flash_syscall(&PARAMS, ENTRY, [1, 2, 3, 4])
            .expect("syscall");
        assert_eq!(ret, 0x2A, "r0 read back as the return value");
        let m = host.into_parts().0;

        let stream: Vec<(u32, u32)> = m
            .writes
            .iter()
            .copied()
            .filter(|&(a, _)| a == DCRDR || a == DCRSR || a == DHCSR)
            .collect();
        let mut expect: Vec<(u32, u32)> = Vec::new();
        for (reg, val) in [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 4),
            (9, PARAMS.static_base),
            (13, PARAMS.stack_pointer),
            (14, PARAMS.breakpoint | 1),
            (15, ENTRY),
            (16, 0x0100_0000),
        ] {
            expect.push((DCRDR, val));
            expect.push((DCRSR, reg | (1 << 16)));
        }
        if MASK_INTERRUPTS {
            // C_MASKINTS is set while halted, kept across the resume, and cleared once halted again.
            expect.push((DHCSR, DBGKEY | C_DEBUGEN | C_HALT | (1 << 3)));
            expect.push((DHCSR, DBGKEY | C_DEBUGEN | (1 << 3)));
            expect.push((DCRSR, 15));
            expect.push((DCRSR, 0));
            expect.push((DHCSR, DBGKEY | C_DEBUGEN | C_HALT));
        } else {
            expect.push((DHCSR, DBGKEY | C_DEBUGEN));
            expect.push((DCRSR, 15));
            expect.push((DCRSR, 0));
        }
        assert_eq!(stream, expect);
        assert!(m.dhcsr_polls >= 1, "polled S_HALT after the resume");
    }

    /// A [`SyscallOps`] that reports the core still running for a fixed number of probes.
    struct PollMock {
        /// Probes still to answer "not halted" before reporting halted.
        polls_before_halt: u32,
        /// Probes taken.
        polls: u32,
        /// The r0 status `finish` reports once halted.
        status: u32,
    }

    impl SyscallOps for PollMock {
        fn syscall_halted(&mut self) -> Result<bool, Error> {
            self.polls += 1;
            if self.polls_before_halt > 0 {
                self.polls_before_halt -= 1;
                return Ok(false);
            }
            Ok(true)
        }
        fn syscall_abandon(&mut self) {}
        fn syscall_pc(&mut self) -> Result<u32, Error> {
            Ok(PARAMS.breakpoint & !1)
        }
        fn syscall_status(&mut self) -> Result<u32, Error> {
            Ok(self.status)
        }
    }

    /// The poll hands back one backing-off deadline per unhalted probe, then `Done`.
    #[test]
    fn syscall_run_yields_backing_off_deadlines_then_completes() {
        let mut ops = PollMock {
            polls_before_halt: 5,
            polls: 0,
            status: 0,
        };
        let mut run = SyscallRun::new(ENTRY, PARAMS.breakpoint & !1);
        let mut now_us = 0u64;
        let mut waits = Vec::new();
        let mut pendings = 0u32;
        loop {
            match run.advance(&mut ops, now_us) {
                StepPoll::Done => break,
                StepPoll::Err(e) => panic!("unexpected error: {e:?}"),
                StepPoll::Pending { wake_at_us } => {
                    pendings += 1;
                    if wake_at_us > now_us {
                        waits.push(wake_at_us - now_us);
                        now_us = wake_at_us;
                    }
                }
            }
        }
        // The first advance sets up the poll; then one probe per advance until the sixth halts.
        assert_eq!(ops.polls, 6, "one bounded probe per advance");
        assert!(pendings >= 5, "each not-yet-halted probe yields a deadline");
        assert_eq!(
            waits[0],
            u64::from(SYSCALL_POLL_START_US),
            "the first deadline keeps the starting backoff"
        );
        for pair in waits.windows(2) {
            let (prev, next) = (pair[0], pair[1]);
            assert!(
                next == u64::from(SYSCALL_POLL_MAX_US) || next == prev * 2,
                "each backoff doubles until the cap: {prev} -> {next}"
            );
        }
        assert_eq!(run.finish(&mut ops), Ok(0));
        run.check = SyscallCheck::Zero(Error::EraseFailed);
        assert_eq!(run.finish_checked(&mut ops), Ok(()));
    }

    /// A halt anywhere but the return trap is no result, even with r0 reading as success.
    #[test]
    fn a_halt_at_a_foreign_pc_is_an_error_not_a_result() {
        let mut mock = CoreMock::new(S_REGRDY | S_HALT, 0);
        mock.pc = 0x0000_1234;
        let mut host = SwdHost::new(mock, NoDelay);
        assert_eq!(
            host.flash_syscall(&PARAMS, ENTRY, [0; 4]),
            Err(Error::AlgoHalted { pc: 0x0000_1234 })
        );
    }

    /// The check costs one r15 read per completed call, and a good return is still a result.
    #[test]
    fn a_return_to_the_trap_reads_r15_once_and_reports_r0() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY | S_HALT, 7), NoDelay);
        assert_eq!(host.flash_syscall(&PARAMS, ENTRY, [0; 4]), Ok(7));
        assert_eq!(host.into_parts().0.pc_reads(), 1);
    }

    /// `C_MASKINTS` must not outlive a foreign halt either.
    #[cfg(feature = "mask-interrupts")]
    #[test]
    fn a_foreign_halt_still_clears_the_interrupt_mask() {
        let mut mock = CoreMock::new(S_REGRDY | S_HALT, 0);
        mock.pc = 0x0000_1234;
        let mut host = SwdHost::new(mock, NoDelay);
        assert!(host.flash_syscall(&PARAMS, ENTRY, [0; 4]).is_err());
        let m = host.into_parts().0;
        assert_eq!(m.writes.last(), Some(&(DHCSR, DBGKEY | C_DEBUGEN | C_HALT)));
    }

    /// A locked-up core fails at the first completion poll, not after the budget.
    #[test]
    fn a_locked_up_core_fails_at_the_first_poll() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY | S_LOCKUP, 0), NoDelay);
        assert_eq!(
            host.flash_syscall(&PARAMS, ENTRY, [0; 4]),
            Err(Error::AlgoLockup)
        );
        let m = host.into_parts().0;
        // Nine S_REGRDY waits for the register setup, then the one poll.
        assert_eq!(m.dhcsr_polls, 10);
        // Entering Debug state ends the lockup, so the teardown finds a halted core.
        assert_eq!(m.writes.last(), Some(&(DHCSR, DBGKEY | C_DEBUGEN | C_HALT)));
    }

    /// A timed-out core is halted before anything else touches it: the next call writes its registers.
    #[test]
    fn a_timeout_halts_the_core_it_gave_up_on() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY, 0), NoDelay);
        assert_eq!(
            host.flash_syscall(&PARAMS, ENTRY, [0; 4]),
            Err(Error::AlgoTimeout)
        );
        let m = host.into_parts().0;
        assert_eq!(m.writes.last(), Some(&(DHCSR, DBGKEY | C_DEBUGEN | C_HALT)));
    }

    /// Whatever an operation takes, its completion is seen within the cap, once past the ladder.
    #[test]
    fn a_long_operation_is_detected_within_the_poll_cap() {
        struct Timed {
            now_us: u64,
            done_at_us: u64,
        }
        impl SyscallOps for Timed {
            fn syscall_halted(&mut self) -> Result<bool, Error> {
                Ok(self.now_us >= self.done_at_us)
            }
            fn syscall_abandon(&mut self) {}
            fn syscall_pc(&mut self) -> Result<u32, Error> {
                Ok(0)
            }
            fn syscall_status(&mut self) -> Result<u32, Error> {
                Ok(0)
            }
        }
        // A 45 ms flash sector, then F4 sector erases around the ladder's old 100 ms rungs.
        for done_at_us in [45_000, 303_000, 350_000, 402_000] {
            let mut ops = Timed {
                now_us: 0,
                done_at_us,
            };
            let mut run = SyscallRun::new(ENTRY, 0);
            loop {
                let now_us = ops.now_us;
                match run.advance(&mut ops, now_us) {
                    StepPoll::Done => break,
                    StepPoll::Err(e) => panic!("{e:?}"),
                    StepPoll::Pending { wake_at_us } => ops.now_us = wake_at_us,
                }
            }
            let late = ops.now_us - done_at_us;
            assert!(late <= 2_000, "{done_at_us} us seen {late} us late");
        }
    }

    /// The halt keeps `C_MASKINTS` until `C_HALT` reads set, as the DHCSR rule requires.
    #[cfg(feature = "mask-interrupts")]
    #[test]
    fn a_masked_timeout_halts_before_it_clears_the_mask() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY, 0), NoDelay);
        assert!(host.flash_syscall(&PARAMS, ENTRY, [0; 4]).is_err());
        let m = host.into_parts().0;
        let halted = DBGKEY | C_DEBUGEN | C_HALT;
        assert_eq!(
            m.writes[m.writes.len() - 2..],
            [(DHCSR, halted | (1 << 3)), (DHCSR, halted)]
        );
    }

    /// A core that never reports S_HALT after resume exhausts the budget and surfaces `AlgoTimeout`.
    #[test]
    fn flash_syscall_times_out_when_core_never_halts() {
        let mut host = SwdHost::new(CoreMock::new(S_REGRDY, 0), NoDelay);
        assert_eq!(
            host.flash_syscall(&PARAMS, ENTRY, [0; 4]),
            Err(Error::AlgoTimeout)
        );
        let m = host.into_parts().0;
        assert!(
            m.dhcsr_polls > 100,
            "backoff kept polling until the budget elapsed"
        );
    }
}

mod write_memory_tests {
    //! `write_memory` must explicitly re-arm TAR when a transfer crosses a 1KiB boundary.
    extern crate std;

    use super::*;
    use crate::swd_host_core::*;
    use dap_rs::swd::{APnDP, Config, DPRegister, Result as SwdResult};
    use std::vec::Vec;

    /// One recorded AP register write, tagged by which CSW/TAR/DRW role it played.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ApWrite {
        Csw(u32),
        Tar(u32),
        Drw(u32),
    }

    /// SWD mock that records every AP register write in order.
    struct RecordingMock {
        cfg: Config,
        writes: Vec<ApWrite>,
    }

    impl RecordingMock {
        fn new() -> Self {
            Self {
                cfg: Config::default(),
                writes: Vec::new(),
            }
        }
    }

    impl Swd<RecordingMock> for RecordingMock {
        fn available(_deps: &RecordingMock) -> bool {
            true
        }
        fn config(&mut self) -> &mut Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, _apndp: APnDP, _a: DPRegister) -> SwdResult<u32> {
            Ok(0)
        }
        fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
            match (apndp, a) {
                (APnDP::AP, DPRegister::DPIDR) => self.writes.push(ApWrite::Csw(data)),
                (APnDP::AP, DPRegister::CTRLSTAT) => self.writes.push(ApWrite::Tar(data)),
                (APnDP::AP, DPRegister::RDBUFF) => self.writes.push(ApWrite::Drw(data)),
                _ => {}
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// A write spanning a 1KiB boundary re-arms TAR right at the crossing, not just at the start.
    #[test]
    fn write_memory_rearms_tar_at_1kib_boundary() {
        const BASE: u32 = 0x2000_0000;
        const BOUNDARY: u32 = BASE + 0x400;
        let start = BOUNDARY - 8;
        let data: [u8; 12] = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
        ];

        let mut host = SwdHost::new(RecordingMock::new(), NoDelay);
        host.write_memory(start, &data).expect("write_memory");
        let m = host.into_parts().0;

        assert_eq!(
            m.writes,
            std::vec![
                ApWrite::Csw(CSW_WORD_ADDRINC),
                ApWrite::Tar(start),
                ApWrite::Drw(0x0403_0201),
                ApWrite::Drw(0x0807_0605),
                // TAR explicitly re-armed at the boundary rather than trusting auto-increment.
                ApWrite::Tar(BOUNDARY),
                ApWrite::Drw(0x0C0B_0A09),
            ]
        );
    }
}
