//! Bit-bang JTAG composed onto an SWD backend that lends it the shared SWCLK/SWDIO pads.

use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin};
use dap_rs::jtag::{self, Jtag, SequenceInfo};
use dap_rs::swd::{self, APnDP, DPRegister, Swd};
use dap_rs::swj::{Dependencies, Pins};

#[cfg(feature = "rp2040-jtag-accel")]
use dap_rs::dap::TransferConfig;
#[cfg(feature = "rp2040-jtag-accel")]
use dap_rs::jtag::TransferResult;
#[cfg(feature = "rp2040-jtag-accel")]
use dap_rs::swd::RnW;

use crate::late_jtag::{JtagLines, LateJtagShifter};

/// The mechanism seam: an SWD backend whose two shared SWJ pads move to software GPIO and back.
/// Implemented on the backend's `Dependencies` type, the role held while JTAG is selected.
pub trait SwjPadLoan {
    /// Quiesce the SWD peripheral and point SWCLK/SWDIO at software GPIO, moving function only.
    fn lend_pads(&mut self);

    /// Point the pads back and resume the peripheral in the state a fresh SWD request expects.
    fn reclaim_pads(&mut self);

    /// Reprogram the SWD pacing; restated here because the JTAG role names no `Swd` type.
    fn set_swd_clock(&mut self, max_frequency: u32) -> bool;

    /// The backend's own scan-chain configuration, so the composition owns no second storage.
    fn jtag_config_mut(&mut self) -> &mut jtag::Config;
}

/// The `Dependencies` role: pads are peripheral-owned, so SWJ traffic reaches the wire unchanged.
pub struct ComposedJtagDeps<D, IO, C> {
    swd: D,
    jtag: LateJtagShifter<IO, C>,
}

/// The active SWD role: the inner driver verbatim, carrying the JTAG shifter across the switch.
pub struct ComposedJtagSwd<S, IO, C> {
    swd: S,
    jtag: LateJtagShifter<IO, C>,
}

/// The active JTAG role: shifting over software GPIO with the SWD peripheral quiesced.
pub struct ComposedJtagDrv<D, IO, C> {
    swd: D,
    jtag: LateJtagShifter<IO, C>,
}

impl<D, IO, C> ComposedJtagDeps<D, IO, C>
where
    IO: InputOutputPin,
    C: DelayCycles,
{
    /// Compose the late-sample shifter onto a wired SWD backend; `lines` are second pad handles.
    pub fn new(swd: D, lines: JtagLines<IO>, delay: C) -> Self {
        Self {
            swd,
            jtag: LateJtagShifter::new(lines, delay),
        }
    }
}

impl<D, S, IO, C> From<ComposedJtagDeps<D, IO, C>> for ComposedJtagSwd<S, IO, C>
where
    S: From<D>,
{
    fn from(d: ComposedJtagDeps<D, IO, C>) -> Self {
        // Both roles keep the pads on the SWD peripheral; nothing to hand over.
        Self {
            swd: d.swd.into(),
            jtag: d.jtag,
        }
    }
}

impl<D, S, IO, C> From<ComposedJtagSwd<S, IO, C>> for ComposedJtagDeps<D, IO, C>
where
    D: From<S>,
{
    fn from(s: ComposedJtagSwd<S, IO, C>) -> Self {
        Self {
            swd: s.swd.into(),
            jtag: s.jtag,
        }
    }
}

impl<D, IO, C> From<ComposedJtagDeps<D, IO, C>> for ComposedJtagDrv<D, IO, C>
where
    D: SwjPadLoan,
    IO: InputOutputPin,
    C: DelayCycles,
{
    fn from(mut d: ComposedJtagDeps<D, IO, C>) -> Self {
        // Stage software levels before the loan, so no stale level reaches the wire at handover.
        d.jtag.lines_to_jtag();
        d.swd.lend_pads();
        Self {
            swd: d.swd,
            jtag: d.jtag,
        }
    }
}

impl<D, IO, C> From<ComposedJtagDrv<D, IO, C>> for ComposedJtagDeps<D, IO, C>
where
    D: SwjPadLoan,
    IO: InputOutputPin,
    C: DelayCycles,
{
    fn from(mut j: ComposedJtagDrv<D, IO, C>) -> Self {
        // Pads back first, so the line is never left with no owner.
        j.swd.reclaim_pads();
        j.jtag.release_lines();
        Self {
            swd: j.swd,
            jtag: j.jtag,
        }
    }
}

impl<D, S, IO, C> Dependencies<ComposedJtagSwd<S, IO, C>, ComposedJtagDrv<D, IO, C>>
    for ComposedJtagDeps<D, IO, C>
where
    D: Dependencies<S, D> + SwjPadLoan + From<S>,
    S: Swd<D>,
    IO: InputOutputPin,
    C: DelayCycles,
{
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        Dependencies::<S, D>::process_swj_pins(&mut self.swd, output, mask, wait_us)
    }
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        Dependencies::<S, D>::process_swj_sequence(&mut self.swd, data, nbits)
    }
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        // Recorded on both sides so a later mode switch shifts at this request's resolution.
        self.jtag.set_clock(max_frequency);
        Dependencies::<S, D>::process_swj_clock(&mut self.swd, max_frequency)
    }
    fn high_impedance_mode(&mut self) {
        Dependencies::<S, D>::high_impedance_mode(&mut self.swd)
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        Dependencies::<S, D>::swd_config(&mut self.swd)
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        Dependencies::<S, D>::jtag_config(&mut self.swd)
    }
}

/// SWD over [`ComposedJtagDeps`]: the inner driver verbatim, clock requests recorded for JTAG too.
impl<D, S, IO, C> Swd<ComposedJtagDeps<D, IO, C>> for ComposedJtagSwd<S, IO, C>
where
    D: Dependencies<S, D> + SwjPadLoan,
    S: Swd<D>,
    IO: InputOutputPin,
    C: DelayCycles,
{
    fn available(_deps: &ComposedJtagDeps<D, IO, C>) -> bool {
        true
    }
    fn config(&mut self) -> &mut swd::Config {
        self.swd.config()
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.swd.read_inner(apndp, a)
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        self.swd.write_inner(apndp, a, data)
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.jtag.set_clock(max_frequency);
        self.swd.set_clock(max_frequency)
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.swd.write_sequence(num_bits, data)
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.swd.read_sequence(num_bits, data)
    }
}

/// JTAG over [`ComposedJtagDeps`]: the late-sample bit cell on software GPIO, peripheral quiesced.
impl<D, IO, C> Jtag<ComposedJtagDeps<D, IO, C>> for ComposedJtagDrv<D, IO, C>
where
    D: SwjPadLoan,
    IO: InputOutputPin,
    C: DelayCycles,
{
    fn available(_deps: &ComposedJtagDeps<D, IO, C>) -> bool {
        true
    }
    fn config(&mut self) -> &mut jtag::Config {
        self.swd.jtag_config_mut()
    }
    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]) {
        self.jtag.sequence(info, tdi, rxbuf)
    }
    fn tms_sequence(&mut self, tms: &[bool]) {
        self.jtag.tms_sequence(tms)
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.jtag.set_clock(max_frequency);
        // The peripheral is quiesced here, so this only reprograms it for later.
        self.swd.set_swd_clock(max_frequency)
    }

    #[cfg(feature = "rp2040-jtag-accel")]
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let config = self.swd.jtag_config_mut();
        let before = config.index as u32;
        let after = (config.device_count as u32).saturating_sub(before + 1);
        self.jtag
            .transfer_word(r_nw, a2a3, transfer_config.idle_cycles, data, before, after)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;
    use dap_rs::dap::DelayNs;

    const STAGED: u8 = 1;
    const LEND: u8 = 2;
    const RECLAIM: u8 = 3;
    const RELEASED: u8 = 4;

    /// Per-test event log; the harness runs tests in parallel, so each test owns one.
    #[derive(Default)]
    struct Log {
        events: Cell<[u8; 16]>,
        len: Cell<usize>,
    }

    impl Log {
        fn push(&self, ev: u8) {
            let mut e = self.events.get();
            let n = self.len.get();
            e[n] = ev;
            self.events.set(e);
            self.len.set(n + 1);
        }

        fn taken(&self) -> [u8; 16] {
            self.events.get()
        }
    }

    struct MockPin<'a> {
        log: &'a Log,
        output: bool,
    }

    impl InputOutputPin for MockPin<'_> {
        fn set_as_output(&mut self) {
            if !self.output {
                self.output = true;
                self.log.push(STAGED);
            }
        }
        fn set_high(&mut self, _high: bool) {}
        fn set_as_input(&mut self) {
            if self.output {
                self.output = false;
                self.log.push(RELEASED);
            }
        }
        fn is_high(&mut self) -> bool {
            false
        }
    }

    struct MockDelay;
    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }
    impl DelayCycles for MockDelay {
        fn cpu_clock(&self) -> u32 {
            1_000_000
        }
        fn delay_cycles(&mut self, _cycles: u32) {}
    }

    struct MockBackend<'a> {
        log: &'a Log,
    }

    impl SwjPadLoan for MockBackend<'_> {
        fn lend_pads(&mut self) {
            self.log.push(LEND);
        }
        fn reclaim_pads(&mut self) {
            self.log.push(RECLAIM);
        }
        fn set_swd_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn jtag_config_mut(&mut self) -> &mut jtag::Config {
            unimplemented!("the ordering tests never reach the scan chain")
        }
    }

    fn deps(log: &Log) -> ComposedJtagDeps<MockBackend<'_>, MockPin<'_>, MockDelay> {
        let pin = || MockPin { log, output: false };
        ComposedJtagDeps::new(
            MockBackend { log },
            JtagLines::new(pin(), pin(), pin(), pin()),
            MockDelay,
        )
    }

    #[test]
    fn jtag_entry_stages_software_lines_before_lending_the_pads() {
        let log = Log::default();
        let _jtag = ComposedJtagDrv::from(deps(&log));
        // Three lines driven, then the loan; TDO was already an input, so it logs nothing.
        assert_eq!(&log.taken()[..4], &[STAGED, STAGED, STAGED, LEND]);
    }

    #[test]
    fn jtag_exit_reclaims_the_pads_before_releasing_the_software_drive() {
        let log = Log::default();
        let jtag = ComposedJtagDrv::from(deps(&log));
        let _back = ComposedJtagDeps::from(jtag);
        assert_eq!(
            &log.taken()[..8],
            &[STAGED, STAGED, STAGED, LEND, RECLAIM, RELEASED, RELEASED, RELEASED]
        );
    }
}
