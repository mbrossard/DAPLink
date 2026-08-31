//! SWD-only DAP dependencies: [`Deps`] doubles as its own disabled JTAG driver.

use dap_rs::driver::bitbang::{BitbangAdapter, DelayCycles, InputOutputPin, SwdAdapter};
use dap_rs::jtag::{self, Jtag, SequenceInfo};
use dap_rs::swd::{self, APnDP, DPRegister, Swd};
use dap_rs::swj::{Dependencies, Pins};

/// SWD-only DAP dependencies; also serves as the disabled JTAG driver.
pub struct Deps<IO: InputOutputPin, D: DelayCycles>(BitbangAdapter<IO, D>);

impl<IO: InputOutputPin, D: DelayCycles> Deps<IO, D> {
    /// Wrap a configured bit-bang adapter.
    pub fn new(inner: BitbangAdapter<IO, D>) -> Self {
        Self(inner)
    }
}

/// Active SWD driver: delegates to the bit-bang `SwdAdapter`.
pub struct SwdDrv<IO: InputOutputPin, D: DelayCycles>(SwdAdapter<IO, D>);

impl<IO: InputOutputPin, D: DelayCycles> From<Deps<IO, D>> for SwdDrv<IO, D> {
    fn from(d: Deps<IO, D>) -> Self {
        SwdDrv(d.0.into())
    }
}
impl<IO: InputOutputPin, D: DelayCycles> From<SwdDrv<IO, D>> for Deps<IO, D> {
    fn from(s: SwdDrv<IO, D>) -> Self {
        Deps(s.0.into())
    }
}

impl<IO: InputOutputPin, D: DelayCycles> Dependencies<SwdDrv<IO, D>, Deps<IO, D>> for Deps<IO, D> {
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

impl<IO: InputOutputPin, D: DelayCycles> Swd<Deps<IO, D>> for SwdDrv<IO, D> {
    fn available(_deps: &Deps<IO, D>) -> bool {
        true
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        self.0.read_inner(apndp, a)
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        self.0.write_inner(apndp, a, data)
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
impl<IO: InputOutputPin, D: DelayCycles> Jtag<Deps<IO, D>> for Deps<IO, D> {
    fn available(_deps: &Deps<IO, D>) -> bool {
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
