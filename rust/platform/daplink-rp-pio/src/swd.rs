//! The dap-rs role triple over the PIO SWD engine, plus the pad-loan seam.

use daplink_rs::dap_rs::jtag::{self, Jtag, SequenceInfo};
use daplink_rs::dap_rs::swd::{self, APnDP, DPRegister, Swd};
use daplink_rs::dap_rs::swj::{Dependencies, Pins};
use daplink_rs::{ScanChainStorage, SwjPadLoan};

use crate::clock::{clock_divider, CYCLES_PER_BIT, MIN_DIVIDER};
use crate::program::ProgramEntries;
use crate::shifter::{PioShifter, SwjPinsSio};
use crate::transfer::{PioSwd, XferError};

/// State carried across the role conversions.
struct Inner<S> {
    engine: PioSwd<S>,
    sys_hz: u32,
    swd_config: swd::Config,
    jtag_config: jtag::Config,
}

impl<S: PioShifter> Inner<S> {
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        match clock_divider(self.sys_hz, max_frequency, CYCLES_PER_BIT, MIN_DIVIDER) {
            Some(div) => {
                self.engine.set_clock_divider(div);
                true
            }
            None => false,
        }
    }

    fn high_impedance_mode(&mut self) {
        self.engine.drain_and_release();
    }
}

/// The PIO SWD dependencies; also its own disabled JTAG driver, so `DAP_Info` advertises SWD only.
pub struct PioSwdDeps<S> {
    inner: Inner<S>,
}

/// The active PIO SWD driver, produced from [`PioSwdDeps`] by the role conversion.
pub struct PioSwdDrv<S> {
    inner: Inner<S>,
}

impl<S: PioShifter> PioSwdDeps<S> {
    /// Build the dependencies over a running state machine and its loaded entry PCs.
    pub fn new<const N: usize>(
        shifter: S,
        entries: ProgramEntries,
        sys_hz: u32,
        scan: &'static ScanChainStorage<N>,
    ) -> Self {
        Self {
            inner: Inner {
                engine: PioSwd::new(shifter, entries),
                sys_hz,
                swd_config: swd::Config::default(),
                jtag_config: jtag::Config::new(scan.init()),
            },
        }
    }

    /// Destructively clear a wedged state machine when the caller may discard queued work.
    pub fn recover(&mut self) {
        self.inner.engine.recover();
    }
}

impl<S> From<PioSwdDeps<S>> for PioSwdDrv<S> {
    fn from(d: PioSwdDeps<S>) -> Self {
        Self { inner: d.inner }
    }
}

impl<S> From<PioSwdDrv<S>> for PioSwdDeps<S> {
    fn from(s: PioSwdDrv<S>) -> Self {
        Self { inner: s.inner }
    }
}

/// Map the engine's transfer error onto the dap-rs SWD error type.
fn map_err(e: XferError) -> swd::Error {
    match e {
        XferError::AckWait => swd::Error::AckWait,
        XferError::AckFault => swd::Error::AckFault,
        XferError::NoAck => swd::Error::NoAck,
        XferError::BadParity => swd::Error::BadParity,
        XferError::Unknown(v) => swd::Error::AckUnknown(v),
    }
}

/// The pad-loan mechanism: what lets bit-bang JTAG compose onto this backend.
impl<S: PioShifter> SwjPadLoan for PioSwdDeps<S> {
    fn lend_pads(&mut self) {
        self.inner.engine.lend_pads();
    }

    fn reclaim_pads(&mut self) {
        self.inner.engine.reclaim_pads();
    }

    fn set_swd_clock(&mut self, max_frequency: u32) -> bool {
        // Only the divider, so this is safe with the pads on loan and the SM parked.
        self.inner.set_clock(max_frequency)
    }

    fn jtag_config_mut(&mut self) -> &mut jtag::Config {
        &mut self.inner.jtag_config
    }
}

impl<S: PioShifter + SwjPinsSio> Dependencies<PioSwdDrv<S>, PioSwdDeps<S>> for PioSwdDeps<S> {
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        self.inner.engine.lend_pads();
        let read = self
            .inner
            .engine
            .shifter_mut()
            .swj_pins(output, mask, wait_us);
        self.inner.engine.reclaim_pads();
        read
    }
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.inner.engine.set_direction(true);
        self.inner.engine.write_sequence(nbits, data);
    }
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_clock(max_frequency)
    }
    fn high_impedance_mode(&mut self) {
        self.inner.high_impedance_mode();
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        &mut self.inner.swd_config
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        &mut self.inner.jtag_config
    }
}

impl<S: PioShifter + SwjPinsSio> Swd<PioSwdDeps<S>> for PioSwdDrv<S> {
    fn available(_deps: &PioSwdDeps<S>) -> bool {
        true
    }
    fn config(&mut self) -> &mut swd::Config {
        &mut self.inner.swd_config
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.inner
            .engine
            .read_transfer(apndp == APnDP::AP, a as u8)
            .map_err(map_err)
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        self.inner
            .engine
            .write_transfer(apndp == APnDP::AP, a as u8, data)
            .map_err(map_err)
    }
    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_clock(max_frequency)
    }
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.inner.engine.set_direction(true);
        self.inner.engine.write_sequence(num_bits, data);
        Ok(())
    }
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.inner.engine.set_direction(false);
        self.inner.engine.read_sequence(num_bits, data);
        Ok(())
    }
}

/// Disabled JTAG: an `out` group cannot span TMS and TDI, so JTAG rides the bit-bang path.
impl<S: PioShifter + SwjPinsSio> Jtag<PioSwdDeps<S>> for PioSwdDeps<S> {
    fn available(_deps: &PioSwdDeps<S>) -> bool {
        false
    }
    fn config(&mut self) -> &mut jtag::Config {
        &mut self.inner.jtag_config
    }
    fn sequence(&mut self, _info: SequenceInfo, _tdi: &[u8], rxbuf: &mut [u8]) {
        // Read back zeros, not the previous response-buffer contents.
        rxbuf.fill(0);
    }
    fn tms_sequence(&mut self, _tms: &[bool]) {}
    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        false
    }
}

/// The accelerated DAP engine: PIO SWD as the SWD driver, bit-bang JTAG over the pad loan.
pub type PioSwdJtagDap<
    'a,
    S,
    IO,
    C,
    SWO = daplink_rs::NoSwo,
    V = daplink_rs::NoVendor,
    LEDS = daplink_rs::NoopLeds,
> = daplink_rs::ComposedJtagDap<'a, PioSwdDeps<S>, PioSwdDrv<S>, IO, C, SWO, V, LEDS>;

/// The SWD-only PIO DAP engine, for boards whose JTAG pins are not wired out.
pub type PioSwdOnlyDap<
    'a,
    S,
    WAIT,
    SWO = daplink_rs::NoSwo,
    V = daplink_rs::NoVendor,
    LEDS = daplink_rs::NoopLeds,
> = daplink_rs::DapProbe<'a, PioSwdDeps<S>, LEDS, WAIT, PioSwdDeps<S>, PioSwdDrv<S>, SWO, V>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;
    use std::vec::Vec;

    #[derive(Debug, PartialEq, Eq)]
    enum Op {
        Divider(u16),
        Enable(bool),
        OenReleased,
        SwclkLow,
        ClearFifos,
        Restart,
        Jmp(u8),
        PindirsIdle,
    }

    struct MockShifter {
        tx: Vec<u32>,
        ops: Vec<Op>,
    }

    impl PioShifter for MockShifter {
        fn put(&mut self, word: u32) {
            self.tx.push(word);
        }
        fn get(&mut self) -> u32 {
            0
        }
        fn set_clock_divider(&mut self, divider: u16) {
            self.ops.push(Op::Divider(divider));
        }
        fn set_enabled(&mut self, enabled: bool) {
            self.ops.push(Op::Enable(enabled));
        }
        fn clear_fifos(&mut self) {
            self.ops.push(Op::ClearFifos);
        }
        fn restart(&mut self) {
            self.ops.push(Op::Restart);
        }
        fn exec_jmp(&mut self, addr: u8) {
            self.ops.push(Op::Jmp(addr));
        }
        fn force_oen_released(&mut self) {
            self.ops.push(Op::OenReleased);
        }
        fn force_swclk_low(&mut self) {
            self.ops.push(Op::SwclkLow);
        }
        fn force_pindirs_idle(&mut self) {
            self.ops.push(Op::PindirsIdle);
        }
        fn pads_to_sio(&mut self) {}
        fn pads_to_pio(&mut self) {}
    }

    impl SwjPinsSio for MockShifter {
        fn swj_pins(&mut self, _output: Pins, _mask: Pins, _wait_us: u32) -> Pins {
            Pins::empty()
        }
    }

    fn deps(sys_hz: u32) -> PioSwdDeps<MockShifter> {
        PioSwdDeps {
            inner: Inner {
                engine: PioSwd::new(
                    MockShifter {
                        tx: Vec::new(),
                        ops: Vec::new(),
                    },
                    crate::program::DIRECT.entries(0),
                ),
                sys_hz,
                swd_config: swd::Config::default(),
                jtag_config: jtag::Config::new(&mut []),
            },
        }
    }

    fn set_clock(deps: &mut PioSwdDeps<MockShifter>, hz: u32) -> bool {
        Dependencies::<PioSwdDrv<MockShifter>, PioSwdDeps<MockShifter>>::process_swj_clock(deps, hz)
    }

    #[test]
    fn public_clock_hook_rejects_unrepresentable_stock_requests() {
        for sys_hz in [125_000_000, 150_000_000] {
            let capacity = u64::from(CYCLES_PER_BIT) * u64::from(crate::clock::MAX_DIVIDER);
            let accepted = (u64::from(sys_hz).div_ceil(capacity)) as u32;
            let rejected = accepted - 1;
            let encoded = clock_divider(sys_hz, accepted, CYCLES_PER_BIT, MIN_DIVIDER).unwrap();
            let mut deps = deps(sys_hz);
            assert!(set_clock(&mut deps, accepted));
            assert_eq!(
                deps.inner.engine.shifter_mut().ops,
                vec![Op::Divider(encoded)]
            );
            assert!(!set_clock(&mut deps, rejected));
            assert!(!set_clock(&mut deps, 0));
            assert_eq!(
                deps.inner.engine.shifter_mut().ops,
                vec![Op::Divider(encoded)]
            );
        }
    }

    #[test]
    fn public_clock_hook_writes_the_maximum_divider_encoding() {
        let mut deps = deps(CYCLES_PER_BIT * crate::clock::MAX_DIVIDER);
        assert!(set_clock(&mut deps, 1));
        assert_eq!(deps.inner.engine.shifter_mut().ops, vec![Op::Divider(0)]);
    }

    #[test]
    fn high_impedance_hook_uses_ordered_non_destructive_release() {
        let mut deps = deps(125_000_000);
        Dependencies::<PioSwdDrv<MockShifter>, PioSwdDeps<MockShifter>>::high_impedance_mode(
            &mut deps,
        );
        let entries = deps.inner.engine.entries();
        let shifter = deps.inner.engine.shifter_mut();
        assert!(shifter.ops.is_empty());
        assert_eq!(
            shifter.tx,
            vec![crate::codec::cmd_word(entries.release_cmd, false, 1)]
        );
    }
}
