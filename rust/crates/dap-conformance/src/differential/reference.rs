//! The reference: dap-rs plus the tree's JTAG-DP policy, independent of every `daplink-*` crate.

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::rc::Rc;

use dap_rs::dap::{Dap, TransferConfig};
use dap_rs::driver::bitbang::{BitbangAdapter, JtagAdapter, SwdAdapter};
use dap_rs::jtag::{self, Jtag, SequenceInfo, TapConfig, TransferResult};
use dap_rs::swd::{self, APnDP, DPRegister, RnW, Swd};
use dap_rs::swj::{Dependencies, Pins};
use dap_rs::swo::NoSwo;

use super::diff::{fire, Probe};
use super::pads::{NoLeds, SimDelay, SimPin};
use crate::adi::jtag_dp::{decode_ack, JtagAck, Protocol};
use crate::rule::RuleId;
use crate::target::{Line, Shared};

/// Scan-chain storage for one engine; leaked, as the engines want it `'static`.
pub fn scan_chain(slots: usize) -> &'static mut [TapConfig] {
    Box::leak(vec![TapConfig::INIT; slots].into_boxed_slice())
}

/// The reference's JTAG-DP state, and the rules each report's answers are normalized to.
#[derive(Debug, Default)]
pub struct Latch {
    /// Bit `i` set once TAP `i` answered `0b100`, which only a protocol-1 JTAG-DP sends.
    protocols: Cell<u32>,
    /// FAULT (4) or no ACK (7), recorded for the status dap-rs reports as 0x08.
    failure: Cell<u8>,
    /// Per report still to come: the rules it is normalized to, as [`RuleId`] bits.
    rules: RefCell<VecDeque<u64>>,
    /// The current report's.
    current: Cell<u64>,
    /// Whether the current report reached the SWD driver.
    swd: Cell<bool>,
}

impl Latch {
    /// Normalize the reports to come, in order, to `rule` where `reports` is true.
    pub fn follow(&self, rule: RuleId, reports: Vec<bool>) {
        let bits = reports
            .into_iter()
            .map(|on| if on { rule.bit() } else { 0 })
            .collect();
        self.follow_bits(bits);
    }

    pub(crate) fn follow_bits(&self, bits: Vec<u64>) {
        let mut rules = self.rules.borrow_mut();
        if rules.len() < bits.len() {
            rules.resize(bits.len(), 0);
        }
        for (r, b) in rules.iter_mut().zip(bits) {
            *r |= b;
        }
    }

    fn on(&self, rule: RuleId) -> bool {
        self.current.get() & rule.bit() != 0
    }

    /// The attempts a transfer makes for `retries` WAIT retries in the current report.
    fn attempts(&self, retries: usize) -> usize {
        retries + usize::from(self.on(RuleId::WaitAttempts))
    }

    /// Before `report`: a new session forgets every protocol, a transfer the last failure.
    pub fn before(&self, report: &[u8]) {
        let rules = self.rules.borrow_mut().pop_front();
        self.current.set(rules.unwrap_or(0));
        self.swd.set(false);
        match report.first() {
            Some(0x02 | 0x03 | 0x15) => self.protocols.set(0),
            Some(0x05 | 0x06) => self.failure.set(0),
            _ => {}
        }
    }

    /// After `report`: dap-rs's 0x08 becomes the recorded FAULT or no ACK.
    pub fn after(&self, report: &[u8], response: &mut [u8], len: usize) {
        let at = match report.first() {
            Some(0x05) => 2,
            Some(0x06) => 3,
            _ => return,
        };
        let failure = self.failure.replace(0);
        if at < len && failure != 0 && response[at] == 0x08 {
            response[at] = failure;
        }
        if at < len {
            let mut arm = response[..len].to_vec();
            self.arm_count(report, &mut arm);
            if arm != response[..len] {
                fire(RuleId::CountCompleted);
                if self.on(RuleId::CountCompleted) {
                    response[..len].copy_from_slice(&arm);
                }
            }
        }
    }

    /// dap-rs counts the block transfer that failed, Arm what completed; and Arm's empty answers.
    fn arm_count(&self, report: &[u8], response: &mut [u8]) {
        let len = response.len();
        if report[0] == 0x06 {
            if report[2..4] == [0, 0] {
                // Only seen unnormalized: a normalized empty block never reaches dap-rs.
                response.fill(0);
                response[0] = 0x06;
            } else if self.swd.get() && response[3] != 1 {
                let done = if report[4] & 2 != 0 {
                    (len - 4) / 4
                } else {
                    usize::from(u16::from_le_bytes([response[1], response[2]])) - 1
                };
                response[1..3].copy_from_slice(&(done as u16).to_le_bytes());
            }
            return;
        }
        if report[2] == 0 {
            response[2] = 0;
        } else if response[2] == 0 && response[1] > 0 {
            // Only match mask writes ran, and Arm answers those OK.
            response[2] = 1;
        }
    }
}

/// A command processor with the probe-level JTAG status and session handling of a [`Latch`].
pub struct Latched<P> {
    /// The processor.
    pub inner: P,
    /// Its JTAG transfers' state.
    pub latch: Rc<Latch>,
}

impl<P: Probe> Probe for Latched<P> {
    fn process(&mut self, report: &[u8], response: &mut [u8]) -> usize {
        self.latch.before(report);
        if self.latch.on(RuleId::CountCompleted) && report[0] == 0x06 && report[2..4] == [0, 0] {
            // Arm does nothing; an empty `DAP_Transfer` restores dap-rs's mode as the block would.
            fire(RuleId::CountCompleted);
            self.inner.process(&[0x05, report[1], 0], response);
            response[..4].copy_from_slice(&[0x06, 0, 0, 0]);
            return 4;
        }
        let len = self.inner.process(report, response);
        self.latch.after(report, response, len);
        len
    }
}

/// One DPACC/APACC scan of either JTAG-DP protocol, a bit per `sequence`; only WAIT leaves early.
pub fn jtag_transfer<DEPS, J: Jtag<DEPS>>(
    jtag: &mut J,
    latch: &Latch,
    r_nw: RnW,
    a2a3: u8,
    idle_cycles: u8,
    data: u32,
) -> TransferResult {
    let index = jtag.config().index;
    let count = jtag.config().device_count;
    let after = u32::from(count - index - 1);
    let cell = |jtag: &mut J, tdi: bool, tms: bool| {
        let mut tdo = 0u8;
        let info = SequenceInfo {
            n_bits: 1,
            capture: true,
            tms,
        };
        jtag.sequence(info, &[u8::from(tdi)], std::slice::from_mut(&mut tdo));
        u32::from(tdo & 1)
    };
    let bits = |jtag: &mut J, value: u64, n: u32, exit: bool| {
        (0..n).fold(0u64, |v, i| {
            let tms = exit && i + 1 == n;
            v | u64::from(cell(jtag, value >> i & 1 != 0, tms)) << i
        })
    };
    jtag.tms_sequence(&[true, false, false]);
    bits(jtag, u64::MAX, u32::from(index), false);
    let ack = bits(jtag, u64::from(a2a3) << 1 | r_nw as u64, 3, false) as u32;
    let captured = if ack == 0b001 {
        jtag.tms_sequence(&[true, true, false]);
        0
    } else {
        let v = bits(jtag, u64::from(data), 32, after == 0) as u32;
        if after > 0 {
            bits(jtag, u64::MAX, after, true);
        }
        jtag.tms_sequence(&[true, false]);
        v
    };
    bits(jtag, u64::MAX, u32::from(idle_cycles), false);
    let tap = 1u32 << index;
    let protocols = &latch.protocols;
    let protocol = if protocols.get() & tap == 0 {
        Protocol::Unknown
    } else {
        Protocol::One
    };
    if protocol.learn(ack as u8) == Protocol::One {
        protocols.set(protocols.get() | tap);
    }
    let failure = match decode_ack(ack as u8, protocol) {
        JtagAck::Ok | JtagAck::OkFault => return TransferResult::Ok(captured),
        JtagAck::Wait => return TransferResult::Wait,
        JtagAck::Fault => 0x04,
        JtagAck::Invalid(_) => 0x07,
    };
    latch.failure.set(failure);
    TransferResult::Fault
}

type Inner = BitbangAdapter<SimPin, SimDelay>;

/// The reference's dependencies: dap-rs's bit-bang adapter.
pub struct RefDeps(Inner, Rc<Latch>);
/// The reference's SWD driver: dap-rs's.
pub struct RefSwd(SwdAdapter<SimPin, SimDelay>, Rc<Latch>);
/// The reference's JTAG driver: dap-rs's, transfers through [`jtag_transfer`].
pub struct RefJtag(JtagAdapter<SimPin, SimDelay>, Rc<Latch>);

impl From<RefDeps> for RefSwd {
    fn from(d: RefDeps) -> Self {
        RefSwd(d.0.into(), d.1)
    }
}

impl From<RefSwd> for RefDeps {
    fn from(s: RefSwd) -> Self {
        RefDeps(s.0.into(), s.1)
    }
}

impl From<RefDeps> for RefJtag {
    fn from(d: RefDeps) -> Self {
        RefJtag(d.0.into(), d.1)
    }
}

impl From<RefJtag> for RefDeps {
    fn from(j: RefJtag) -> Self {
        RefDeps(j.0.into(), j.1)
    }
}

impl Dependencies<RefSwd, RefJtag> for RefDeps {
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.0.process_swj_pins(output, mask, wait_us)
    }
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.0.process_swj_sequence(data, nbits)
    }
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.0.process_swj_clock(max_frequency)
    }
    fn high_impedance_mode(&mut self) {
        self.0.high_impedance_mode()
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        self.0.swd_config()
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.0.jtag_config()
    }
}

impl RefSwd {
    /// `access` until it is not WAIT, `wait_retries` times as dap-rs counts or once more.
    fn retried<T>(
        &mut self,
        wait_retries: usize,
        mut access: impl FnMut(&mut Self) -> swd::Result<T>,
    ) -> swd::Result<T> {
        self.1.swd.set(true);
        if wait_retries == 0 {
            fire(RuleId::WaitAttempts);
        }
        for attempt in 1..=self.1.attempts(wait_retries) {
            match access(self) {
                Err(swd::Error::AckWait) => {
                    if attempt == wait_retries {
                        fire(RuleId::WaitAttempts);
                    }
                }
                x => return x,
            }
        }
        Err(swd::Error::AckWait)
    }
}

impl Swd<RefDeps> for RefSwd {
    fn available(_deps: &RefDeps) -> bool {
        true
    }
    fn config(&mut self) -> &mut swd::Config {
        self.0.config()
    }
    /// dap-rs calls this for an AP register only to re-read a value match; normalized, it collects.
    fn read(&mut self, wait_retries: usize, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        let v = self.retried(wait_retries, |s| s.read_inner(apndp, a))?;
        if apndp == APnDP::AP {
            fire(RuleId::ValueMatch);
            if self.1.on(RuleId::ValueMatch) {
                return self.retried(wait_retries, |s| {
                    s.read_inner(APnDP::DP, DPRegister::RDBUFF)
                });
            }
        }
        Ok(v)
    }
    fn read_ap(&mut self, wait_retries: usize, a: DPRegister) -> swd::Result<u32> {
        self.retried(wait_retries, |s| s.read_inner(APnDP::AP, a))
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        let r = self.0.read_inner(apndp, a);
        if r == Err(swd::Error::BadParity) {
            // dap-rs returns before the trailing turnaround, leaving the target mid-frame.
            fire(RuleId::TrailingTurnaround);
        }
        r
    }
    fn write(
        &mut self,
        wait_retries: usize,
        apndp: APnDP,
        a: DPRegister,
        data: u32,
    ) -> swd::Result<()> {
        self.retried(wait_retries, |s| s.write_inner(apndp, a, data))
    }
    /// dap-rs reports a write's no ACK as `AckUnknown(7)`, which it answers 0x0F.
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        match self.0.write_inner(apndp, a, data) {
            Err(swd::Error::AckUnknown(7)) => {
                fire(RuleId::NoAckStatus);
                if self.1.on(RuleId::NoAckStatus) {
                    return Err(swd::Error::NoAck);
                }
                Err(swd::Error::AckUnknown(7))
            }
            r => r,
        }
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_clock(max_frequency)
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.0.write_sequence(num_bits, data)
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.0.read_sequence(num_bits, data)
    }
}

impl Jtag<RefDeps> for RefJtag {
    fn available(_deps: &RefDeps) -> bool {
        true
    }
    fn config(&mut self) -> &mut jtag::Config {
        self.0.config()
    }
    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        self.0.sequence(info, tdi, rxbuf)
    }
    fn tms_sequence(&mut self, tms: &[bool]) {
        self.0.tms_sequence(tms)
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_clock(max_frequency)
    }
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let latch = self.1.clone();
        jtag_transfer(self, &latch, r_nw, a2a3, config.idle_cycles, data)
    }
}

/// The reference engine: dap-rs over the reference drivers.
pub type RefDap = Dap<'static, RefDeps, NoLeds, SimDelay, RefJtag, RefSwd, NoSwo>;

/// The reference: its engine with the probe-level JTAG handling.
pub type Reference = Latched<RefDap>;

/// dap-rs unchanged: its JTAG transfers speak protocol 0 only.
pub type DapRsOnly = Dap<
    'static,
    Inner,
    NoLeds,
    SimDelay,
    JtagAdapter<SimPin, SimDelay>,
    SwdAdapter<SimPin, SimDelay>,
    NoSwo,
>;

fn adapter(sim: &Shared) -> Inner {
    let pin = |line| SimPin::new(sim, line);
    BitbangAdapter::new(
        SimPin::open_drain(sim, Line::Nreset),
        pin(Line::Tdi),
        pin(Line::Swdio),
        pin(Line::Swclk),
        pin(Line::Tdo),
        SimDelay,
        scan_chain(8),
    )
}

/// The reference on `sim`, parked the way every firmware parks it at boot.
pub fn reference(sim: &Shared) -> Reference {
    let latch = Rc::new(Latch::default());
    let deps = RefDeps(adapter(sim), latch.clone());
    let mut dap = Dap::new(deps, NoLeds, SimDelay, NoSwo, "diff", 512);
    dap.suspend();
    Latched { inner: dap, latch }
}

/// dap-rs unchanged on `sim`, parked the same way.
pub fn dap_rs_only(sim: &Shared) -> DapRsOnly {
    let mut dap = Dap::new(adapter(sim), NoLeds, SimDelay, NoSwo, "diff", 512);
    dap.suspend();
    dap
}

impl Probe for RefDap {
    fn process(&mut self, report: &[u8], response: &mut [u8]) -> usize {
        self.process_command(report, response)
    }
}

impl Probe for DapRsOnly {
    fn process(&mut self, report: &[u8], response: &mut [u8]) -> usize {
        self.process_command(report, response)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::differential::diff::run;
    use crate::session::{jtag_select, Cmd, Item, Stream};
    use crate::target::{shared, Mode, Target, DPIDR_V3, FAULT_BASE};

    fn read(req: u8) -> Item {
        Item { req, word: None }
    }

    fn write(req: u8, word: u32) -> Item {
        Item {
            req,
            word: Some(word),
        }
    }

    /// The last response of `cmds` on `target` from the reference and from dap-rs unchanged.
    fn last(target: Target, transfer: Vec<Item>) -> (Vec<u8>, Vec<u8>) {
        let s = Stream {
            target,
            cmds: vec![
                Cmd::Connect(2),
                jtag_select(),
                Cmd::JtagSequence(vec![(0x46, vec![0x3F]), (0x01, vec![0x00])]),
                Cmd::JtagConfigure(vec![4]),
                Cmd::Transfer(0, transfer),
            ],
        };
        let sim = shared(&s.target);
        let ours = run(&s, &mut reference(&sim), &sim).pop().unwrap();
        let sim = shared(&s.target);
        let theirs = run(&s, &mut dap_rs_only(&sim), &sim).pop().unwrap();
        (ours.response, theirs.response)
    }

    fn adiv6() -> Target {
        Target {
            mode: Mode::Jtag,
            adiv6: true,
            ..Target::default()
        }
    }

    #[test]
    fn a_protocol_1_dp_reads_only_through_the_reference() {
        let (ours, theirs) = last(adiv6(), vec![read(0x02)]);
        let mut want = vec![0x05, 1, 1];
        want.extend(DPIDR_V3.to_le_bytes());
        assert_eq!(ours, want);
        assert_eq!(theirs, [0x05, 0, 2]);
    }

    #[test]
    fn a_protocol_1_fault_stops_the_transfer() {
        let items = vec![
            write(0x04, 0x5000_0020),
            write(0x08, 0),
            write(0x05, FAULT_BASE),
            read(0x0F),
        ];
        let (ours, _) = last(adiv6(), items);
        assert_eq!(ours, [0x05, 4, 0x04]);
    }

    #[test]
    fn a_marked_report_makes_one_attempt_more_than_its_wait_retries() {
        let s = Stream {
            target: Target {
                waits: vec![1],
                ..Target::default()
            },
            cmds: vec![
                Cmd::Connect(1),
                crate::session::swd_select(),
                Cmd::Configure(0, 1, 0),
                Cmd::Block(0, 1, 0x03, vec![]),
            ],
        };
        for (marked, packets, status) in [(false, 2, 2), (true, 3, 1)] {
            let sim = shared(&s.target);
            let mut dap = reference(&sim);
            dap.latch
                .follow(RuleId::WaitAttempts, vec![false, false, false, marked]);
            let last = run(&s, &mut dap, &sim).pop().unwrap();
            assert_eq!((last.rates.len(), last.response[3]), (packets, status));
        }
    }

    /// dap-rs a867ca7 compares the poll one read behind: 1, 1, 2 at two retries, 1, 1, 2, 3 at three.
    #[test]
    fn a_marked_value_match_compares_each_read_once() {
        for (retries, marked, packets, status) in [
            (2, false, 4, 0x11),
            (2, true, 6, 0x01),
            (3, false, 5, 0x01),
            (3, true, 6, 0x01),
        ] {
            let s = crate::session::drw_poll(retries);
            let sim = shared(&s.target);
            let mut dap = reference(&sim);
            dap.latch
                .follow(RuleId::ValueMatch, vec![false, false, false, false, marked]);
            let last = run(&s, &mut dap, &sim).pop().unwrap();
            assert_eq!(last.response, [0x05, 2, status]);
            assert_eq!(last.rates.len(), packets);
        }
    }

    #[test]
    fn no_target_is_no_ack_not_wait() {
        let target = Target {
            mode: Mode::Jtag,
            absent: true,
            ..Target::default()
        };
        let (ours, theirs) = last(target, vec![read(0x02)]);
        assert_eq!(ours, [0x05, 0, 0x07]);
        assert_eq!(theirs, [0x05, 0, 0x02]);
    }

    type Side = (Vec<u8>, usize);

    /// Each step's response and SWD packet count from the reference, unmarked and count-marked.
    fn counted(s: &Stream) -> Vec<(Side, Side)> {
        let side = |marked: bool| {
            let sim = shared(&s.target);
            let mut dap = reference(&sim);
            dap.latch
                .follow(RuleId::CountCompleted, vec![marked; s.cmds.len()]);
            run(s, &mut dap, &sim)
                .into_iter()
                .map(|t| (t.response, t.rates.len()))
        };
        side(false).zip(side(true)).collect()
    }

    /// dap-rs a867ca7 counts the block transfer that failed; marked, the block count is Arm's.
    #[test]
    fn a_count_marked_block_counts_what_completed() {
        let s = crate::session::sticky_fault();
        let steps = counted(&s);
        // A DAP_Transfer keeps dap-rs's count, which Arm's would answer 05 00 04 at step 4.
        let want: [(&[u8], &[u8]); 6] = [
            (&[5, 6, 4], &[5, 6, 4]),
            (&[5, 1, 4], &[5, 1, 4]),
            (&[5, 1, 0], &[5, 1, 1]),
            (&[6, 1, 0, 4], &[6, 0, 0, 0]),
            (&[6, 1, 0, 4], &[6, 0, 0, 4]),
            (&[6, 1, 0, 4], &[6, 0, 0, 4]),
        ];
        for (at, (plain, marked)) in want.into_iter().enumerate() {
            let (p, m) = &steps[3 + at];
            assert_eq!(
                (&p.0[..plain.len()], &m.0[..marked.len()]),
                (plain, marked),
                "step {}",
                3 + at
            );
        }
        // Only unmarked does an empty block post an AP read.
        assert_eq!((steps[6].0 .1, steps[6].1 .1), (1, 0));
    }

    #[test]
    fn a_count_marked_empty_jtag_transfer_is_ok() {
        let s = Stream {
            target: Target {
                mode: Mode::Jtag,
                ..Target::default()
            },
            cmds: vec![
                Cmd::Connect(2),
                jtag_select(),
                Cmd::JtagConfigure(vec![4]),
                Cmd::Transfer(0, vec![]),
            ],
        };
        let (plain, marked) = &counted(&s)[4];
        assert_eq!(
            (&plain.0[..], &marked.0[..]),
            (&[5, 0, 8][..], &[5, 0, 0][..])
        );
    }
}
