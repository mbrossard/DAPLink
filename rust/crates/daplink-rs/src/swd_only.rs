//! SWD-only DAP dependencies: [`SwdOnlyDeps`] doubles as its own disabled JTAG driver.

use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin, SwdAdapter};
use dap_rs::jtag::{self, Jtag, SequenceInfo};
use dap_rs::swd::{self, APnDP, DPRegister, Swd};
use dap_rs::swj::{Dependencies, Pins};
use daplink_core::{JtagDp, JtagDpSource};

use crate::wired::{
    idle_cells, no_ack, park, read_retry, trailing_turnaround, write_retry, SwjPin, WiredAdapter,
};

/// SWD-only DAP dependencies; also serves as the disabled JTAG driver.
pub struct SwdOnlyDeps<IO: InputOutputPin, D: DelayCycles>(WiredAdapter<IO, D>, &'static JtagDp);

impl<IO: InputOutputPin, D: DelayCycles> SwdOnlyDeps<IO, D> {
    /// Wrap a configured bit-bang adapter; `dp` carries the host's SWD idle cycles.
    pub fn new(inner: WiredAdapter<IO, D>, dp: &'static JtagDp) -> Self {
        Self(inner, dp)
    }
}

impl<IO: InputOutputPin, D: DelayCycles> JtagDpSource for SwdOnlyDeps<IO, D> {
    fn jtag_dp(&self) -> Option<&'static JtagDp> {
        Some(self.1)
    }
}

/// Active SWD driver: delegates to the bit-bang `SwdAdapter`.
pub struct SwdOnlySwdDrv<IO: InputOutputPin, D: DelayCycles>(
    SwdAdapter<SwjPin<IO>, D>,
    &'static JtagDp,
);

impl<IO: InputOutputPin, D: DelayCycles> From<SwdOnlyDeps<IO, D>> for SwdOnlySwdDrv<IO, D> {
    fn from(d: SwdOnlyDeps<IO, D>) -> Self {
        SwdOnlySwdDrv(d.0.into(), d.1)
    }
}
impl<IO: InputOutputPin, D: DelayCycles> From<SwdOnlySwdDrv<IO, D>> for SwdOnlyDeps<IO, D> {
    fn from(s: SwdOnlySwdDrv<IO, D>) -> Self {
        SwdOnlyDeps(s.0.into(), s.1)
    }
}

impl<IO: InputOutputPin, D: DelayCycles> Dependencies<SwdOnlySwdDrv<IO, D>, SwdOnlyDeps<IO, D>>
    for SwdOnlyDeps<IO, D>
{
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
        park(&mut self.0)
    }
    fn swd_config(&mut self) -> &mut swd::Config {
        self.0.swd_config()
    }
    fn jtag_config(&mut self) -> &mut jtag::Config {
        self.0.jtag_config()
    }
}

impl<IO: InputOutputPin, D: DelayCycles> SwdOnlySwdDrv<IO, D> {
    /// What an OK read ACK still owes: the turnaround dap-rs skips on a bad parity, then idle.
    #[inline(always)]
    fn settle(&mut self, r: &swd::Result<u32>) {
        match r {
            Ok(_) => {}
            Err(swd::Error::BadParity) => trailing_turnaround(&mut self.0),
            Err(_) => return,
        }
        self.idle();
    }

    /// The host's idle cycles, after an OK ACK.
    #[inline(always)]
    fn idle(&mut self) {
        let cycles = self.1.swd_idle();
        if cycles != 0 {
            idle_cells(&mut self.0, cycles);
        }
    }
}

impl<IO: InputOutputPin, D: DelayCycles> Swd<SwdOnlyDeps<IO, D>> for SwdOnlySwdDrv<IO, D> {
    fn available(_deps: &SwdOnlyDeps<IO, D>) -> bool {
        true
    }
    fn read(&mut self, wait_retries: usize, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        let r = read_retry(&mut self.0, wait_retries, apndp, a);
        self.settle(&r);
        r
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        let r = self.0.read_inner(apndp, a);
        self.settle(&r);
        r
    }
    fn write(
        &mut self,
        wait_retries: usize,
        apndp: APnDP,
        a: DPRegister,
        data: u32,
    ) -> swd::Result<()> {
        let r = write_retry(&mut self.0, wait_retries, apndp, a, data).map_err(no_ack);
        if r.is_ok() {
            self.idle();
        }
        r
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        let r = self.0.write_inner(apndp, a, data).map_err(no_ack);
        if r.is_ok() {
            self.idle();
        }
        r
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
    fn config(&mut self) -> &mut swd::Config {
        self.0.config()
    }
}

/// Disabled JTAG: `available()` is `false`, so JTAG is never offered to the host.
impl<IO: InputOutputPin, D: DelayCycles> Jtag<SwdOnlyDeps<IO, D>> for SwdOnlyDeps<IO, D> {
    fn available(_deps: &SwdOnlyDeps<IO, D>) -> bool {
        false
    }
    fn config(&mut self) -> &mut jtag::Config {
        self.0.jtag_config()
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

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use self::std::rc::Rc;
    use core::cell::Cell;

    use crate::wired::{wired_swd_deps, ScanChainStorage};

    /// An open-drain line; the cell is `true` while it pulls low.
    #[derive(Clone)]
    struct Line(Rc<Cell<bool>>);
    impl InputOutputPin for Line {
        fn set_as_output(&mut self) {}
        fn set_high(&mut self, high: bool) {
            self.0.set(!high);
        }
        fn set_as_input(&mut self) {
            self.0.set(false);
        }
        fn is_high(&mut self) -> bool {
            !self.0.get()
        }
    }

    struct Delay;
    impl dap_rs::dap::DelayNs for Delay {
        fn delay_ns(&mut self, _ns: u32) {}
    }
    impl DelayCycles for Delay {
        fn cpu_clock(&self) -> u32 {
            1_000_000
        }
        fn delay_cycles(&mut self, _cycles: u32) {}
    }

    #[test]
    fn swd_entry_keeps_nreset_asserted_until_the_pads_park() {
        static SCAN: ScanChainStorage = ScanChainStorage::new();
        let low = Rc::new(Cell::new(false));
        let line = || Line(Rc::new(Cell::new(false)));
        let nreset = Line(low.clone());
        let mut deps = wired_swd_deps(nreset, line(), line(), line(), line(), Delay, &SCAN);
        deps.process_swj_pins(Pins::empty(), Pins::NRESET, 0);
        let mut deps: SwdOnlyDeps<_, _> = SwdOnlySwdDrv::from(deps).into();
        assert!(low.get());
        deps.high_impedance_mode();
        assert!(!low.get());
    }
}
