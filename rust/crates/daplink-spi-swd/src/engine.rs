//! The session engine: the dependency object plus its two mode drivers (SPI SWD, GPIO JTAG).

use dap_rs::dap::TransferConfig;
use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin};
use dap_rs::jtag::{self, Jtag, SequenceInfo, TapConfig, TransferResult};
use dap_rs::swd::{self, APnDP, DPRegister, RnW};
use dap_rs::swj::{Dependencies, Pins};

use crate::port::{JtagPacing, SpiPortOps};
use crate::scan::JtagScan;
use crate::session::{
    ack_from_frame, combine_read_data, half_period_cycles, header_session, ladder_rung, parity32,
    park_session, read_ack_session, read_data_sessions, request_byte, wdata_bytes,
    write_ack_session, write_data_session, Dir, Prescaler, Session, TRN,
};

/// Injected hardware and state for the session SWD transport; all state (clock rungs, both
/// interface configurations) lives here, so every mode switch preserves it.
pub struct SpiSwdDeps<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    port: P,
    swclk: IO,
    swdio: IO,
    nreset: IO,
    delay: D,
    pacing: J,
    kernel_hz: u32,
    max_sck_hz: u32,
    mbr: u8,
    bit_cycles: u32,
    // JTAG has its own requested clock, so its half-period is distinct from `bit_cycles`.
    jtag_bit_cycles: u32,
    // True when at or below the injected ladder's threshold: the word path runs the ladder.
    jtag_fast: bool,
    config: swd::Config,
    jtag_config: jtag::Config,
}

impl<P, IO, D, J> SpiSwdDeps<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    /// Wire the session transport over a constructed `port` plus the three bidirectional probe
    /// lines. `kernel_hz` is the SPI kernel clock the rung ladder divides; `max_sck_hz` caps the
    /// ladder at the rung the part can drive; `delay` times every GPIO fallback path and the
    /// JTAG bit-banging; `scan_chain` is caller-owned `'static` storage.
    // A wiring constructor: naming every injected part beats bundling them into a struct.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mut port: P,
        swclk: IO,
        swdio: IO,
        nreset: IO,
        delay: D,
        pacing: J,
        kernel_hz: u32,
        max_sck_hz: u32,
        scan_chain: &'static mut [TapConfig],
    ) -> Self {
        port.configure();
        port.pads_to_spi();
        Self {
            port,
            swclk,
            swdio,
            nreset,
            delay,
            pacing,
            kernel_hz,
            max_sck_hz,
            mbr: Prescaler::SLOWEST.mbr_bits(),
            bit_cycles: 1,
            jtag_bit_cycles: 1,
            jtag_fast: false,
            config: swd::Config::default(),
            jtag_config: jtag::Config::new(scan_chain),
        }
    }

    /// Recompute the SPI baud rung and the GPIO-fallback bit period for a requested SWCLK;
    /// `false` when even the slowest rung is still faster than `max_frequency`.
    fn retune_clock(&mut self, max_frequency: u32) -> bool {
        match ladder_rung(self.kernel_hz, self.max_sck_hz, max_frequency) {
            Some((pre, achieved)) => {
                self.mbr = pre.mbr_bits();
                self.bit_cycles = half_period_cycles(self.delay.cpu_clock(), achieved);
                true
            }
            None => false,
        }
    }

    /// Recompute the JTAG half-period spin count and whether the injected fast ladder takes over
    /// at it; `false` only for a zero request.
    fn retune_jtag_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        // Floor the count at the ladder's threshold: above it the ladder is the pacing
        // authority, so the generic path must not run faster than the handoff point.
        self.jtag_bit_cycles =
            half_period_cycles(self.delay.cpu_clock(), max_frequency).max(J::FAST_HALF_CYCLES);
        self.jtag_fast = self.jtag_bit_cycles <= J::FAST_HALF_CYCLES;
        true
    }

    /// Bit-bang `num_bits` of `data` out of SWDIO on GPIO at the configured bit period.
    fn gpio_write_sequence(&mut self, mut num_bits: usize, data: &[u8]) {
        // Level before direction: leaving the alternate function drives whatever the output
        // register last held, so parking the idle level first prevents a stray SWCLK edge.
        self.swclk.set_high(true);
        self.swclk.set_as_output();
        self.swdio.set_as_output();
        for &byte in data {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            for i in 0..bits {
                self.swdio.set_high(byte & (1 << i) != 0);
                self.swclk.set_high(false);
                self.delay.delay_cycles(self.bit_cycles);
                self.swclk.set_high(true);
                self.delay.delay_cycles(self.bit_cycles);
            }
            num_bits -= bits;
        }
        self.port.pads_to_spi();
    }

    /// Bit-bang `num_bits` in from SWDIO on GPIO at the configured bit period.
    fn gpio_read_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) {
        // Idle level before direction, as in `gpio_write_sequence`.
        self.swclk.set_high(true);
        self.swclk.set_as_output();
        self.swdio.set_as_input();
        for b in data.iter_mut() {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            let mut v = 0u8;
            for i in 0..bits {
                self.swclk.set_high(false);
                self.delay.delay_cycles(self.bit_cycles);
                v |= (self.swdio.is_high() as u8) << i;
                self.swclk.set_high(true);
                self.delay.delay_cycles(self.bit_cycles);
            }
            *b = v;
            num_bits -= bits;
        }
        // Only the clock pad goes back to the SPI block here; see `clk_pad_to_spi`.
        self.port.clk_pad_to_spi();
    }

    /// Take the shared pads for GPIO JTAG on mode entry, then re-assert the dedicated lines.
    fn pins_to_jtag(&mut self) {
        // Both idle levels are latched before either pad leaves the alternate function, so mode
        // entry cannot clock the TAP with a stale TCK level.
        self.swclk.set_high(true); // TCK idle high (mode 3, matching the bit-bang engine)
        self.swclk.set_as_output();
        self.swdio.set_high(true); // TMS idle high
        self.swdio.set_as_output();
        self.port.jtag_pads_acquire();
    }

    /// Release the JTAG-only lines on mode exit; TCK/TMS are reclaimed by the next SPI session.
    fn pins_release_jtag(&mut self) {
        self.port.jtag_pads_release();
    }

    /// Counted pacing for the generic JTAG mechanics (`sequence`/`tms_sequence`); the injected
    /// ladder paces only the specialized scans, never this path.
    #[inline(always)]
    fn jtag_wait(&mut self) {
        self.delay.delay_cycles(self.jtag_bit_cycles);
    }

    /// One outgoing/sampling JTAG bit at the current TMS level: drive TDI, clock low, wait,
    /// sample TDO before the rising edge, clock high, wait.
    #[inline(always)]
    fn jtag_bit(&mut self, tdi: bool) -> u32 {
        self.port.tdi_set(tdi);
        self.port.tck_low();
        self.jtag_wait();
        let bit = self.port.tdo_get();
        self.port.tck_high();
        self.jtag_wait();
        bit
    }

    /// Shift up to 8 TDI bits LSB-first and capture TDO, at a fixed (already-set) TMS level.
    fn jtag_shift(&mut self, tdi: u8, bits: usize) -> u8 {
        let mut tdo = 0u32;
        for i in 0..bits {
            tdo |= self.jtag_bit(tdi & (1 << i) != 0) << i;
        }
        tdo as u8
    }

    /// Walk a TMS bit sequence (TAP-state walk) with TDI unchanged.
    fn jtag_tms_walk(&mut self, tms: &[bool]) {
        for &bit in tms {
            self.port.tms_set(bit);
            self.port.tck_low();
            self.jtag_wait();
            self.port.tck_high();
            self.jtag_wait();
        }
    }
}

/// The fast JTAG word path over this dependency object: the injected port seam, the
/// ladder-or-counted-spin pacing `retune_jtag_clock` decided, and the engine's configuration.
impl<P, IO, D, J> JtagScan for SpiSwdDeps<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    type Port = P;

    #[inline(always)]
    fn scan_port(&self) -> &P {
        &self.port
    }

    #[inline(always)]
    fn scan_config(&mut self) -> &jtag::Config {
        &self.jtag_config
    }

    #[inline(always)]
    fn scan_wait_low(&mut self) {
        if self.jtag_fast {
            self.pacing.pace_low();
        } else {
            self.delay.delay_cycles(self.jtag_bit_cycles);
        }
    }

    #[inline(always)]
    fn scan_wait_high(&mut self) {
        if self.jtag_fast {
            self.pacing.pace_high();
        } else {
            self.delay.delay_cycles(self.jtag_bit_cycles);
        }
    }
}

/// The session SPI-assisted SWD adapter: [`swd::Swd`] over [`SpiSwdDeps`].
pub struct SpiSwd<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    d: SpiSwdDeps<P, IO, D, J>,
}

impl<P, IO, D, J> From<SpiSwdDeps<P, IO, D, J>> for SpiSwd<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(d: SpiSwdDeps<P, IO, D, J>) -> Self {
        Self { d }
    }
}

impl<P, IO, D, J> SpiSwd<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    /// Run a transmit session, shifting `bytes[..frames]` LSB-first, one byte per frame; in
    /// half-duplex transmit the shifter produces no receive frames to drain.
    fn tx_session(&mut self, s: Session, bytes: &[u8]) {
        self.d.port.pads_to_spi();
        self.d.port.program(self.d.mbr, s.bits, s.frames, Dir::Tx);
        self.d.port.enable();
        for &b in bytes.iter().take(s.frames as usize) {
            self.d.port.push8(b);
        }
        self.d.port.start();
        self.d.port.wait_eot();
        self.d.port.disable();
        self.d.port.clear_flags();
    }

    /// Run a receive session, returning the last received frame right-aligned.
    fn rx_session(&mut self, s: Session) -> u32 {
        self.d.port.pads_to_spi();
        self.d.port.program(self.d.mbr, s.bits, s.frames, Dir::Rx);
        self.d.port.enable();
        self.d.port.start();
        self.d.port.wait_eot();
        let mut last = 0;
        for _ in 0..s.frames {
            last = self.d.port.pop32();
        }
        self.d.port.disable();
        self.d.port.clear_flags();
        last
    }
}

impl<P, IO, D, J> swd::Swd<SpiSwdDeps<P, IO, D, J>> for SpiSwd<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn available(_deps: &SpiSwdDeps<P, IO, D, J>) -> bool {
        true
    }

    fn config(&mut self) -> &mut swd::Config {
        &mut self.d.config
    }

    fn read_inner(&mut self, apndp: APnDP, addr: DPRegister) -> swd::Result<u32> {
        let req = request_byte(apndp, RnW::R, addr);
        self.tx_session(header_session(), &[req]);

        let ack = ack_from_frame(self.rx_session(read_ack_session()), TRN);
        if let Err(e) = swd::Ack::try_ok(ack) {
            // Non-OK ACK: idle-low park frame reclaims the bus.
            self.tx_session(park_session(), &[0]);
            return Err(e);
        }

        let [lo_s, hi_s] = read_data_sessions();
        let lo = self.rx_session(lo_s);
        let hi = self.rx_session(hi_s);

        let (data, parity) = combine_read_data(lo, hi);
        if parity == parity32(data) {
            Ok(data)
        } else {
            Err(swd::Error::BadParity)
        }
    }

    fn write_inner(&mut self, apndp: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        let req = request_byte(apndp, RnW::W, addr);
        self.tx_session(header_session(), &[req]);

        let ack = ack_from_frame(self.rx_session(write_ack_session()), TRN);
        swd::Ack::try_ok(ack)?;

        self.tx_session(write_data_session(), &wdata_bytes(data));
        Ok(())
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.d.retune_clock(max_frequency)
    }

    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        // Arbitrary sequences are always bit-banged on GPIO.
        self.d.gpio_write_sequence(num_bits, data);
        Ok(())
    }

    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.d.gpio_read_sequence(num_bits, data);
        Ok(())
    }
}

/// Back to the dependency object for the direct-pin `SWJ` commands; the round-trip preserves
/// all state, which lives in the dependency object.
impl<P, IO, D, J> From<SpiSwd<P, IO, D, J>> for SpiSwdDeps<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(s: SpiSwd<P, IO, D, J>) -> Self {
        s.d
    }
}

/// The GPIO-bit-bang JTAG mode driver over [`SpiSwdDeps`]; a distinct mode type so the `From`
/// conversions perform the pad handover once per mode switch.
pub struct SpiJtag<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    d: SpiSwdDeps<P, IO, D, J>,
}

impl<P, IO, D, J> From<SpiSwdDeps<P, IO, D, J>> for SpiJtag<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(mut d: SpiSwdDeps<P, IO, D, J>) -> Self {
        d.pins_to_jtag();
        Self { d }
    }
}

impl<P, IO, D, J> From<SpiJtag<P, IO, D, J>> for SpiSwdDeps<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(mut j: SpiJtag<P, IO, D, J>) -> Self {
        j.d.pins_release_jtag();
        j.d
    }
}

/// Direct-pin control surface: SWCLK/SWDIO/nRESET driven and read as plain GPIOs; the SPI block
/// reclaims the shared pads on the next transfer.
impl<P, IO, D, J> Dependencies<SpiSwd<P, IO, D, J>, SpiJtag<P, IO, D, J>>
    for SpiSwdDeps<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        // The requested level is latched before the pad becomes an output, on every line, so
        // enabling the drive cannot emit a stray edge or pulse a reset line.
        if mask.contains(Pins::SWCLK) {
            self.swclk.set_high(output.contains(Pins::SWCLK));
            self.swclk.set_as_output();
        }
        if mask.contains(Pins::SWDIO) {
            self.swdio.set_high(output.contains(Pins::SWDIO));
            self.swdio.set_as_output();
        }
        if mask.contains(Pins::NRESET) {
            self.nreset.set_high(output.contains(Pins::NRESET));
            self.nreset.set_as_output();
        }

        if wait_us != 0 {
            let cycles = wait_us.saturating_mul(self.delay.cpu_clock() / 1_000_000);
            self.delay.delay_cycles(cycles);
        }

        if mask.contains(Pins::TDI) {
            self.port.tdi_set(output.contains(Pins::TDI));
        }

        // Input reads reflect the pad level whether the line is driven or released; NTRST is
        // reported present-but-high like the bit-bang backend.
        let mut read = Pins::empty();
        read.set(Pins::SWCLK, self.swclk.is_high());
        read.set(Pins::SWDIO, self.swdio.is_high());
        read.set(Pins::NRESET, self.nreset.is_high());
        read.set(Pins::TDI, self.port.tdi_is_high());
        read.set(Pins::TDO, self.port.tdo_is_high());
        read.set(Pins::NTRST, true);
        read
    }

    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.gpio_write_sequence(nbits, data);
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        // Prime both timings, so a clock set before DAP_Connect(JTAG) carries over.
        let jtag_ok = self.retune_jtag_clock(max_frequency);
        let swd_ok = self.retune_clock(max_frequency);
        swd_ok || jtag_ok
    }

    fn high_impedance_mode(&mut self) {
        self.swclk.set_as_input();
        self.swdio.set_as_input();
        self.nreset.set_as_input();
    }

    fn swd_config(&mut self) -> &mut swd::Config {
        &mut self.config
    }

    fn jtag_config(&mut self) -> &mut jtag::Config {
        &mut self.jtag_config
    }
}

/// Real GPIO-bit-bang JTAG beside the SPI SWD path, so the composed engine advertises SWD+JTAG;
/// `transfer` and `shift_ir` are overridden with the specialized word path.
impl<P, IO, D, J> Jtag<SpiSwdDeps<P, IO, D, J>> for SpiJtag<P, IO, D, J>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn available(_deps: &SpiSwdDeps<P, IO, D, J>) -> bool {
        true
    }

    fn config(&mut self) -> &mut jtag::Config {
        &mut self.d.jtag_config
    }

    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], mut rxbuf: &mut [u8]) {
        // TMS is fixed for the whole sequence (dap-rs breaks TAP walks into `tms_sequence`).
        self.d.port.tms_set(info.tms);
        let mut n_bits = info.n_bits;
        for &byte in tdi {
            let bits = n_bits.min(8) as usize;
            n_bits -= bits as u8;
            let tdo = self.d.jtag_shift(byte, bits);
            if info.capture && !rxbuf.is_empty() {
                rxbuf[0] = tdo;
                rxbuf = &mut rxbuf[1..];
            }
        }
    }

    fn tms_sequence(&mut self, tms: &[bool]) {
        self.d.jtag_tms_walk(tms);
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.d.retune_jtag_clock(max_frequency)
    }

    /// The specialized DPACC/APACC scan; `write_abort`'s provided implementation bypasses this
    /// method and keeps the generic mechanics.
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        self.d
            .transfer_word(r_nw, a2a3, transfer_config.idle_cycles, data)
    }

    /// The specialized IR scan in place of the provided per-chunk `shift_ir`.
    fn shift_ir(&mut self, ir: u32) {
        self.d.shift_ir_word(ir);
    }
}
