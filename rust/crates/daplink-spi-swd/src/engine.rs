//! The session engine: the dependency object plus its two mode drivers (SPI SWD, GPIO JTAG).

use dap_rs::dap::TransferConfig;
use dap_rs::driver::bitbang::{DelayCycles, InputOutputPin};
use dap_rs::jtag::{self, Jtag, SequenceInfo, TapConfig, TransferResult};
use dap_rs::swd::{self, APnDP, DPRegister, RnW};
use dap_rs::swj::{Dependencies, Pins};
use daplink_core::dap_wire::IDLE_LOW;
use daplink_core::{JtagDp, JtagDpSource};

use crate::port::{spin, JtagPacing, PortResult, SpiFault, SpiPortOps};
use crate::scan::{scan_shift_ir_word, scan_transfer_word, JtagScanState, KeptAck};
use crate::session::{
    ack_from_frame, combine_read_data, half_period_cycles, header_session, parity32,
    read_ack_session, read_data_session, request_byte, spin_iterations, wdata_bytes, wdata_frames,
    write_ack_session, write_data_session, write_data_session_exact, Dir, Prescaler, Session,
    SpiLadder, TRN,
};
use crate::timing::{
    prepare_jtag_timing, HalfCycleProfile, JtagTimingPlan, PacingPlan, PacingProfile,
};

/// Which peripheral owns the shared SWCLK/SWDIO pads, so ordinary sessions skip mode-register traffic.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum PadOwner {
    /// Both shared pads are on the SPI alternate function.
    Spi,
    /// Only the clock pad is; the data pad is a GPIO input.
    ClkOnly,
    /// Both pads are plain GPIOs.
    Gpio,
}

/// A register-seam fault leaves the wire state unknown and the session's FIFO untrusted.
fn fault_error(_f: SpiFault) -> swd::Error {
    swd::Error::AckProtocol
}

/// One transfer, then up to `retries` more while it answers WAIT: CMSIS-DAP's count.
#[inline(always)]
fn wait_retry<T>(retries: usize, mut transfer: impl FnMut() -> swd::Result<T>) -> swd::Result<T> {
    let mut left = retries;
    loop {
        match transfer() {
            Err(swd::Error::AckWait) if left != 0 => left -= 1,
            r => return r,
        }
    }
}

/// Injected hardware and state for the session SWD transport, preserved across mode switches.
pub struct SpiSwdDeps<P, IO, D, J, const K: usize = 1>
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
    ladder: SpiLadder<P::Source, K>,
    pre: Prescaler,
    bit_cycles: u32,
    // The request is below the slowest rung, so every session is clocked on GPIO.
    gpio_only: bool,
    // ES0621 2.17.2: one SCK period of the current rung, as spin iterations.
    eot_spins: u32,
    pads: PadOwner,
    // Duplex only: whether the separate transmit pad is currently driving the data net.
    driving: bool,
    jtag_timing: JtagTimingPlan,
    config: swd::Config,
    jtag_config: jtag::Config,
    kept: &'static KeptAck,
}

impl<P, IO, D, J, const K: usize> SpiSwdDeps<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    /// Wire the session transport over `port` and the three bidirectional probe lines.
    // A wiring constructor: naming every injected part beats bundling them into a struct.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mut port: P,
        swclk: IO,
        swdio: IO,
        nreset: IO,
        delay: D,
        mut pacing: J,
        ladder: SpiLadder<P::Source, K>,
        scan_chain: &'static mut [TapConfig],
        kept: &'static KeptAck,
    ) -> Self {
        const { assert!(K != 0, "an SPI ladder needs at least one kernel") };
        port.configure();
        port.pads_to_spi();
        let cpu_hz = delay.cpu_clock();
        pacing.prepare_counted(1);
        Self {
            port,
            swclk,
            swdio,
            nreset,
            delay,
            pacing,
            ladder,
            pre: Prescaler::SLOWEST,
            bit_cycles: 1,
            gpio_only: false,
            eot_spins: spin_iterations(Prescaler::SLOWEST.sck_cycles(cpu_hz, ladder.kernels[0].1)),
            pads: PadOwner::Spi,
            driving: false,
            jtag_timing: {
                let scaled = PacingProfile::new(0, HalfCycleProfile::uncalibrated());
                prepare_jtag_timing(
                    0,
                    1,
                    1,
                    HalfCycleProfile::uncalibrated(),
                    scaled,
                    scaled,
                    scaled,
                )
                .expect("nonzero initial JTAG frequency")
            },
            config: swd::Config::default(),
            jtag_config: jtag::Config::new(scan_chain),
            kept,
        }
    }

    /// Recompute the SPI rung and GPIO bit period for a SWCLK; below the slowest rung, GPIO only.
    fn retune_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        let cpu_hz = self.delay.cpu_clock();
        let clock = self.ladder.choose(max_frequency);
        if let Some(c) = clock {
            self.port.route(c.source);
        }
        self.port.set_gpio_clock(max_frequency);
        match clock {
            Some(c) => {
                self.pre = c.pre;
                self.bit_cycles = half_period_cycles(cpu_hz, c.sck_hz);
                self.eot_spins = spin_iterations(c.pre.sck_cycles(cpu_hz, c.kernel_hz));
                self.gpio_only = false;
            }
            None => {
                self.bit_cycles = half_period_cycles(cpu_hz, max_frequency);
                self.gpio_only = true;
            }
        }
        true
    }

    /// Prepare scan and sequence pacing; a zero request leaves the clock unchanged.
    fn retune_jtag_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        let cpu_hz = self.delay.cpu_clock();
        let floor = half_period_cycles(cpu_hz, J::COUNTED_MAX_HZ);
        let counted = HalfCycleProfile::new(0, 0, floor, floor);
        let profile = PacingProfile::new(J::FAST_HALF_CYCLES, counted);
        self.jtag_timing = prepare_jtag_timing(
            cpu_hz,
            max_frequency,
            floor,
            counted,
            profile,
            profile,
            profile,
        )
        .expect("JTAG frequency checked nonzero");
        self.pacing
            .prepare_counted(self.jtag_timing.sequence_half_cycles());
        true
    }

    /// Take both pads for GPIO cells, SWCLK idling high and SWDIO driven or released.
    fn gpio_take(&mut self, drive: bool) {
        self.take_clk();
        if drive {
            self.swdio.set_as_output();
        } else {
            self.swdio.set_as_input();
        }
    }

    /// Take SWCLK for GPIO: latched high off the SPI block's idle level, else left where it is.
    fn take_clk(&mut self) {
        // Level before direction, so leaving the alternate function cannot emit a stray SWCLK edge.
        if self.pads != PadOwner::Gpio {
            self.swclk.set_high(true);
        }
        self.pads_to_gpio();
        self.swclk.set_as_output();
    }

    /// One driven GPIO cell, `bit` on SWDIO before the falling edge.
    #[inline(always)]
    fn gpio_bit_out(&mut self, bit: bool) {
        self.swdio.set_high(bit);
        self.swclk.set_high(false);
        self.delay.delay_cycles(self.bit_cycles);
        self.swclk.set_high(true);
        self.delay.delay_cycles(self.bit_cycles);
    }

    /// One GPIO cell sampling SWDIO at the end of the low half.
    #[inline(always)]
    fn gpio_bit_in(&mut self) -> bool {
        self.swclk.set_high(false);
        self.delay.delay_cycles(self.bit_cycles);
        let bit = self.swdio.is_high();
        self.swclk.set_high(true);
        self.delay.delay_cycles(self.bit_cycles);
        bit
    }

    /// Bit-bang `num_bits` of `data` out of SWDIO on GPIO at the configured bit period.
    fn gpio_write_sequence(&mut self, mut num_bits: usize, data: &[u8]) {
        self.gpio_take(true);
        for &byte in data {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            for i in 0..bits {
                self.gpio_bit_out(byte & (1 << i) != 0);
            }
            num_bits -= bits;
        }
        if !self.gpio_only {
            self.port.pads_to_spi();
            self.pads = PadOwner::Spi;
        }
    }

    /// Bit-bang `num_bits` in from SWDIO on GPIO at the configured bit period.
    fn gpio_read_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) {
        self.gpio_take(false);
        for b in data.iter_mut() {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            let mut v = 0u8;
            for i in 0..bits {
                v |= (self.gpio_bit_in() as u8) << i;
            }
            *b = v;
            num_bits -= bits;
        }
        // Only the clock pad goes back to the SPI block here; see `clk_pad_to_spi`.
        if !self.gpio_only {
            self.port.clk_pad_to_spi();
            self.pads = PadOwner::ClkOnly;
        }
    }

    /// A transmit session's frames as GPIO cells, LSB first, for a clock below the slowest rung.
    #[inline(never)]
    fn gpio_session_out(&mut self, bits: u8, frames: impl Iterator<Item = u32>) {
        self.gpio_take(true);
        for f in frames {
            for i in 0..bits {
                self.gpio_slow_out(f >> i & 1 != 0);
            }
        }
    }

    /// A receive session's frames as GPIO cells, right-aligned as the SPI block leaves them.
    #[inline(never)]
    fn gpio_session_in(&mut self, s: Session, out: &mut [u32]) {
        self.gpio_take(false);
        for i in 0..s.frames as usize {
            let mut v = 0u32;
            for b in 0..s.bits {
                v |= u32::from(self.gpio_slow_in()) << b;
            }
            if let Some(slot) = out.get_mut(i) {
                *slot = v;
            }
        }
    }

    /// [`gpio_bit_out`](Self::gpio_bit_out) out of line: a slow cell does not pay for its size.
    #[inline(never)]
    fn gpio_slow_out(&mut self, bit: bool) {
        self.gpio_bit_out(bit);
    }

    /// [`gpio_bit_in`](Self::gpio_bit_in) out of line.
    #[inline(never)]
    fn gpio_slow_in(&mut self) -> bool {
        self.gpio_bit_in()
    }

    /// Clock one released turnaround bit, which the SPI block cannot frame (minimum four bits).
    fn gpio_turnaround(&mut self) {
        self.gpio_read_sequence(1, &mut [0u8]);
    }

    /// The host's SWD idle cycles after an OK ACK, on GPIO with SWDIO low.
    #[inline(always)]
    fn idle(&mut self) {
        let cycles = self.kept.dp().swd_idle();
        if cycles != 0 {
            self.gpio_write_sequence(cycles.into(), &IDLE_LOW);
        }
    }

    /// Hand the shared pads back to the SPI block, only when something else took them.
    fn ensure_pads_to_spi(&mut self) {
        if self.pads != PadOwner::Spi {
            self.port.pads_to_spi();
            self.pads = PadOwner::Spi;
        }
    }

    /// Match the duplex transmit pad to a session's direction; compiled out in half-duplex.
    fn ensure_drive(&mut self, dir: Dir) {
        if !P::DUPLEX {
            return;
        }
        let want = dir == Dir::Tx;
        if self.driving != want {
            if want {
                self.port.drive();
            } else {
                self.port.release();
            }
            self.driving = want;
        }
    }

    /// Release the duplex transmit pad, whatever the session state.
    fn release_drive(&mut self) {
        if P::DUPLEX && self.driving {
            self.port.release();
            self.driving = false;
        }
    }

    /// Give the shared pads to GPIO, releasing the duplex transmit pad first.
    fn pads_to_gpio(&mut self) {
        self.release_drive();
        self.pads = PadOwner::Gpio;
    }

    /// Take the shared pads for GPIO JTAG on mode entry, then re-assert the dedicated lines.
    fn pins_to_jtag(&mut self) {
        // Both idle levels latch before either pad leaves the alternate function.
        self.take_clk();
        self.swdio.set_high(true); // TMS idle high
        self.swdio.set_as_output();
        self.port.jtag_pads_acquire();
    }

    /// Release the JTAG-only lines on mode exit; TCK/TMS are reclaimed by the next SPI session.
    fn pins_release_jtag(&mut self) {
        self.port.jtag_pads_release();
    }

    /// Pace the low half of a generic JTAG sequence.
    #[inline(always)]
    fn jtag_pace_low(&mut self) {
        if matches!(self.jtag_timing.read(), PacingPlan::Fast) {
            self.pacing.pace_low();
        } else {
            self.pacing
                .counted_low(&mut self.delay, self.jtag_timing.sequence_half_cycles());
        }
    }

    /// Pace the high half: TCK-high pulse width only.
    #[inline(always)]
    fn jtag_pace_high(&mut self) {
        if matches!(self.jtag_timing.read(), PacingPlan::Fast) {
            self.pacing.pace_high();
        } else {
            self.pacing
                .counted_high(&mut self.delay, self.jtag_timing.sequence_half_cycles());
        }
    }

    /// One JTAG bit: drive TDI, clock low, pace, sample TDO before the rising edge, clock high, pace.
    #[inline(always)]
    fn jtag_bit(&mut self, tdi: bool) -> u32 {
        self.port.tdi_set(tdi);
        self.port.tck_low();
        self.jtag_pace_low();
        let bit = self.port.tdo_get();
        self.port.tck_high();
        self.jtag_pace_high();
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
            self.jtag_pace_low();
            self.port.tck_high();
            self.jtag_pace_high();
        }
    }
}

impl<P, IO, D, J, const K: usize> JtagScanState for SpiSwdDeps<P, IO, D, J, K>
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
    fn scan_keep_ack(&mut self, ack: u32) {
        self.kept.keep(ack);
    }

    #[inline(always)]
    fn scan_complete(&mut self, data: u32) -> TransferResult {
        self.kept.complete(|| self.jtag_config.index, data)
    }

    #[inline(always)]
    fn scan_spin(&mut self, cycles: u32) {
        self.delay.delay_cycles(cycles);
    }

    #[inline(always)]
    fn scan_spin_low(&mut self, cycles: u32) {
        self.pacing.counted_low(&mut self.delay, cycles);
    }

    #[inline(always)]
    fn scan_spin_high(&mut self, cycles: u32) {
        self.pacing.counted_high(&mut self.delay, cycles);
    }

    #[inline(always)]
    fn scan_spin_output_low(&mut self, cycles: u32) {
        self.pacing.counted_output_low(&mut self.delay, cycles);
    }

    #[inline(always)]
    fn scan_pace_low(&self) {
        self.pacing.pace_low();
    }

    #[inline(always)]
    fn scan_pace_high(&self) {
        self.pacing.pace_high();
    }
}

/// The session SPI-assisted SWD adapter: [`swd::Swd`] over [`SpiSwdDeps`].
pub struct SpiSwd<P, IO, D, J, const K: usize = 1>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    d: SpiSwdDeps<P, IO, D, J, K>,
}

impl<P, IO, D, J, const K: usize> From<SpiSwdDeps<P, IO, D, J, K>> for SpiSwd<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(d: SpiSwdDeps<P, IO, D, J, K>) -> Self {
        Self { d }
    }
}

impl<P, IO, D, J, const K: usize> SpiSwd<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    /// Program and enable one session's geometry, reclaiming pads only if a GPIO path took them.
    fn begin(&mut self, s: Session, dir: Dir) {
        self.d.ensure_pads_to_spi();
        self.d.ensure_drive(dir);
        self.d
            .port
            .program(self.d.pre.mbr_bits(), s.bits, s.frames, dir);
        self.d.port.enable();
    }

    /// The request header; with SWCLK low on GPIO, its start bit is clocked there to add no edge.
    fn header(&mut self, req: u8) -> swd::Result<()> {
        if self.d.pads == PadOwner::Gpio && !self.d.gpio_only && !self.d.swclk.is_high() {
            self.d.gpio_write_sequence(1, &[req]);
            let tail = Session {
                bits: 7,
                frames: 1,
                dir: Dir::Tx,
            };
            return self.tx_session(tail, &[req >> 1]);
        }
        self.tx_session(header_session(), &[req])
    }

    /// Retire a session: erratum delay, disable and flag clearing; a faulted session yields no data.
    fn end(&mut self, r: PortResult<()>) -> swd::Result<()> {
        if P::EOT_DELAY {
            spin(self.d.eot_spins);
        }
        self.d.port.disable();
        let cleared = self.d.port.clear_flags();
        r.and(cleared).map_err(fault_error)
    }

    /// Run a transmit session, shifting `bytes[..frames]` LSB-first, one byte per frame.
    fn tx_session(&mut self, s: Session, bytes: &[u8]) -> swd::Result<()> {
        debug_assert!(bytes.len() == s.frames as usize);
        if self.d.gpio_only {
            self.d
                .gpio_session_out(s.bits, bytes.iter().map(|&b| u32::from(b)));
            return Ok(());
        }
        self.begin(s, Dir::Tx);
        let mut r = Ok(());
        for &b in bytes {
            r = self.d.port.push8(b);
            if r.is_err() {
                break;
            }
        }
        if r.is_ok() {
            self.d.port.start();
            r = self.d.port.wait_eot();
        }
        self.end(r)
    }

    /// [`tx_session`](Self::tx_session) for frames wider than a byte.
    fn tx_session16(&mut self, s: Session, halves: &[u16]) -> swd::Result<()> {
        debug_assert!(halves.len() == s.frames as usize);
        if self.d.gpio_only {
            self.d
                .gpio_session_out(s.bits, halves.iter().map(|&h| u32::from(h)));
            return Ok(());
        }
        self.begin(s, Dir::Tx);
        let mut r = Ok(());
        for &h in halves {
            r = self.d.port.push16(h);
            if r.is_err() {
                break;
            }
        }
        if r.is_ok() {
            self.d.port.start();
            r = self.d.port.wait_eot();
        }
        self.end(r)
    }

    /// Run a receive session into `out`, by count: `RXP` does not rise for a last partial packet.
    fn rx_session(&mut self, s: Session, out: &mut [u32]) -> swd::Result<()> {
        if self.d.gpio_only {
            self.d.gpio_session_in(s, out);
            return Ok(());
        }
        self.begin(s, Dir::Rx);
        self.d.port.start();
        let mut r = self.d.port.wait_eot();
        if r.is_ok() {
            for i in 0..s.frames as usize {
                match self.d.port.pop32() {
                    Ok(v) => {
                        if let Some(slot) = out.get_mut(i) {
                            *slot = v;
                        }
                    }
                    Err(e) => {
                        r = Err(e);
                        break;
                    }
                }
            }
        }
        self.end(r)
    }
}

impl<P, IO, D, J, const K: usize> swd::Swd<SpiSwdDeps<P, IO, D, J, K>> for SpiSwd<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn available(_deps: &SpiSwdDeps<P, IO, D, J, K>) -> bool {
        true
    }

    fn config(&mut self) -> &mut swd::Config {
        &mut self.d.config
    }

    fn read(&mut self, wait_retries: usize, apndp: APnDP, addr: DPRegister) -> swd::Result<u32> {
        wait_retry(wait_retries, || self.read_inner(apndp, addr))
    }

    fn read_inner(&mut self, apndp: APnDP, addr: DPRegister) -> swd::Result<u32> {
        let req = request_byte(apndp, RnW::R, addr);
        self.header(req)?;

        let mut ack_frame = [0u32; 1];
        self.rx_session(read_ack_session(), &mut ack_frame)?;
        if let Err(e) = swd::Ack::try_ok(ack_from_frame(ack_frame[0], TRN)) {
            // Non-OK ACK: one released turnaround clock and nothing more, as dap-rs clocks.
            self.d.gpio_turnaround();
            return Err(e);
        }

        let mut frames = [0u32; 2];
        self.rx_session(read_data_session(), &mut frames)?;

        let (data, parity) = combine_read_data(frames[0], frames[1]);
        self.d.idle();
        if parity == parity32(data) {
            Ok(data)
        } else {
            Err(swd::Error::BadParity)
        }
    }

    fn write(
        &mut self,
        wait_retries: usize,
        apndp: APnDP,
        addr: DPRegister,
        data: u32,
    ) -> swd::Result<()> {
        wait_retry(wait_retries, || self.write_inner(apndp, addr, data))
    }

    fn write_inner(&mut self, apndp: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        let req = request_byte(apndp, RnW::W, addr);
        self.header(req)?;

        // The write ACK session already clocks the trailing turnaround.
        let mut ack_frame = [0u32; 1];
        self.rx_session(write_ack_session(), &mut ack_frame)?;
        swd::Ack::try_ok(ack_from_frame(ack_frame[0], TRN))?;

        if P::EXACT_WRITE_FRAMING {
            self.tx_session16(write_data_session_exact(), &wdata_frames(data))?;
        } else {
            self.tx_session(write_data_session(), &wdata_bytes(data))?;
        }
        self.d.idle();
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

/// Back to the dependency object for the direct-pin `SWJ` commands; all state lives there.
impl<P, IO, D, J, const K: usize> From<SpiSwd<P, IO, D, J, K>> for SpiSwdDeps<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(s: SpiSwd<P, IO, D, J, K>) -> Self {
        s.d
    }
}

/// The GPIO bit-bang JTAG mode driver; a distinct type so `From` performs the pad handover.
pub struct SpiJtag<P, IO, D, J, const K: usize = 1>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    d: SpiSwdDeps<P, IO, D, J, K>,
}

impl<P, IO, D, J, const K: usize> From<SpiSwdDeps<P, IO, D, J, K>> for SpiJtag<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(mut d: SpiSwdDeps<P, IO, D, J, K>) -> Self {
        d.pins_to_jtag();
        Self { d }
    }
}

impl<P, IO, D, J, const K: usize> From<SpiJtag<P, IO, D, J, K>> for SpiSwdDeps<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn from(mut j: SpiJtag<P, IO, D, J, K>) -> Self {
        j.d.pins_release_jtag();
        j.d
    }
}

impl<P, IO, D, J, const K: usize> JtagDpSource for SpiSwdDeps<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn jtag_dp(&self) -> Option<&'static JtagDp> {
        Some(self.kept.dp())
    }
}

/// Direct-pin control surface; the SPI block reclaims the shared pads on the next transfer.
impl<P, IO, D, J, const K: usize> Dependencies<SpiSwd<P, IO, D, J, K>, SpiJtag<P, IO, D, J, K>>
    for SpiSwdDeps<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        // Each level latches before its pad becomes an output, so enabling cannot emit an edge.
        if mask.intersects(Pins::SWCLK | Pins::SWDIO) {
            self.pads_to_gpio();
        }
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

        // NTRST is reported present-but-high like the bit-bang backend.
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
        self.pads_to_gpio();
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

/// GPIO bit-bang JTAG beside the SPI SWD path, with `transfer` and `shift_ir` specialized.
impl<P, IO, D, J, const K: usize> Jtag<SpiSwdDeps<P, IO, D, J, K>> for SpiJtag<P, IO, D, J, K>
where
    P: SpiPortOps,
    IO: InputOutputPin,
    D: DelayCycles,
    J: JtagPacing,
{
    fn available(_deps: &SpiSwdDeps<P, IO, D, J, K>) -> bool {
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
        if !self.d.retune_jtag_clock(max_frequency) {
            return false;
        }
        self.d.retune_clock(max_frequency);
        // In JTAG mode SWJ sequences follow the request, rung or not; SWD keeps the rung's half.
        self.d.bit_cycles = half_period_cycles(self.d.delay.cpu_clock(), max_frequency);
        true
    }

    /// The specialized DPACC/APACC scan; `write_abort` bypasses it.
    fn transfer(
        &mut self,
        r_nw: RnW,
        a2a3: u8,
        transfer_config: &TransferConfig,
        data: u32,
    ) -> TransferResult {
        let plan = self.d.jtag_timing;
        scan_transfer_word(
            &mut self.d,
            plan,
            r_nw,
            a2a3,
            transfer_config.idle_cycles,
            data,
        )
    }

    /// The specialized IR scan in place of the provided per-chunk `shift_ir`.
    fn shift_ir(&mut self, ir: u32) {
        let plan = self.d.jtag_timing;
        scan_shift_ir_word(&mut self.d, plan, ir);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use self::std::boxed::Box;
    use self::std::cell::{Cell, RefCell};
    use self::std::collections::VecDeque;
    use self::std::rc::Rc;
    use self::std::vec;
    use self::std::vec::Vec;
    use crate::port::JtagPortOps;
    use crate::session::choose_prescaler;
    use crate::session::{parity32, wdata_frames, write_data_session_exact, Prescaler};
    use crate::NoFastJtag;
    use dap_rs::dap::DelayNs;
    use dap_rs::swd::Swd;

    /// One recorded seam call: everything the engine asks of the chip, in order.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    enum Op {
        Configure,
        Program(u8, u8, u16, Dir),
        Enable,
        Disable,
        Start,
        Push8(u8),
        Push16(u16),
        WaitEot,
        Pop32,
        ClearFlags,
        PadsToSpi,
        ClkPadToSpi,
        Drive,
        Release,
        Delay(u32),
        AsOutput(Role),
        AsInput(Role),
        TckLow,
        TckHigh,
        TmsSet(bool),
        TdiSet(bool),
        TdoGet,
        PaceLow,
        PaceHigh,
        JtagAcquire,
        JtagRelease,
    }

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    enum Role {
        Swclk,
        Swdio,
        Nreset,
    }

    type Log = Rc<RefCell<Vec<Op>>>;

    /// Register accesses one seam call costs on the STM32H5 implementation, with first-read polls.
    fn reg_cost(op: Op) -> u32 {
        match op {
            // CFG1 image, CR2, CR1 image.
            Op::Program(..) => 3,
            Op::Enable | Op::Disable | Op::Start => 1,
            // Status poll plus the data-register access.
            Op::Push8(_) | Op::Push16(_) | Op::Pop32 => 2,
            // The status read that ends the session also carries the overrun.
            Op::WaitEot => 1,
            Op::ClearFlags => 1,
            // Two mode-register read-modify-writes.
            Op::PadsToSpi => 4,
            Op::ClkPadToSpi | Op::Drive | Op::Release => 2,
            _ => 0,
        }
    }

    /// The register seam as a recorder, with scripted receive frames and injectable faults.
    struct MockPort<const DUPLEX: bool, const EXACT: bool = false> {
        log: Log,
        rx: RefCell<VecDeque<u32>>,
        /// Scripted TDO bits; an empty queue keeps the seam's constant high level.
        tdo: RefCell<VecDeque<u32>>,
        fail_poll: Cell<Option<usize>>,
        polls: Cell<usize>,
        ovr: Cell<bool>,
        gpio_hz: u32,
    }

    impl<const DUPLEX: bool, const EXACT: bool> MockPort<DUPLEX, EXACT> {
        fn new(log: Log) -> Self {
            Self {
                log,
                rx: RefCell::new(VecDeque::new()),
                tdo: RefCell::new(VecDeque::new()),
                fail_poll: Cell::new(None),
                polls: Cell::new(0),
                ovr: Cell::new(false),
                gpio_hz: 0,
            }
        }

        fn push(&self, op: Op) {
            self.log.borrow_mut().push(op);
        }

        /// Count this poll and fault it when it is the injected one.
        fn poll(&self) -> PortResult<()> {
            let n = self.polls.get();
            self.polls.set(n + 1);
            match self.fail_poll.get() {
                Some(k) if k == n => Err(SpiFault::Timeout),
                _ => Ok(()),
            }
        }
    }

    impl<const DUPLEX: bool, const EXACT: bool> JtagPortOps for MockPort<DUPLEX, EXACT> {
        fn jtag_pads_acquire(&mut self) {
            self.push(Op::JtagAcquire);
        }
        fn jtag_pads_release(&mut self) {
            self.push(Op::JtagRelease);
        }
        fn tck_low(&self) {
            self.push(Op::TckLow);
        }
        fn tck_high(&self) {
            self.push(Op::TckHigh);
        }
        fn tms_set(&self, high: bool) {
            self.push(Op::TmsSet(high));
        }
        fn tdi_set(&self, high: bool) {
            self.push(Op::TdiSet(high));
        }
        fn tdo_get(&self) -> u32 {
            self.push(Op::TdoGet);
            self.tdo.borrow_mut().pop_front().unwrap_or(1)
        }
        fn tdi_is_high(&mut self) -> bool {
            false
        }
        fn tdo_is_high(&mut self) -> bool {
            false
        }
    }

    impl<const DUPLEX: bool, const EXACT: bool> SpiPortOps for MockPort<DUPLEX, EXACT> {
        const DUPLEX: bool = DUPLEX;
        type Source = ();
        const EXACT_WRITE_FRAMING: bool = EXACT;

        fn set_gpio_clock(&mut self, requested_hz: u32) {
            self.gpio_hz = requested_hz;
        }

        fn configure(&mut self) {
            self.push(Op::Configure);
        }

        fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
            self.push(Op::Program(mbr, bits, frames, dir));
        }

        fn enable(&self) {
            self.push(Op::Enable);
        }

        fn disable(&self) {
            self.push(Op::Disable);
        }

        fn start(&self) {
            self.push(Op::Start);
        }

        fn push8(&self, byte: u8) -> PortResult<()> {
            self.poll()?;
            self.push(Op::Push8(byte));
            Ok(())
        }

        fn push16(&self, half: u16) -> PortResult<()> {
            self.poll()?;
            self.push(Op::Push16(half));
            Ok(())
        }

        fn wait_eot(&self) -> PortResult<()> {
            self.poll()?;
            self.push(Op::WaitEot);
            if self.ovr.replace(false) {
                return Err(SpiFault::Overrun);
            }
            Ok(())
        }

        fn pop32(&self) -> PortResult<u32> {
            self.poll()?;
            self.push(Op::Pop32);
            Ok(self.rx.borrow_mut().pop_front().unwrap_or(0))
        }

        fn clear_flags(&self) -> PortResult<()> {
            self.push(Op::ClearFlags);
            Ok(())
        }

        fn pads_to_spi(&mut self) {
            self.push(Op::PadsToSpi);
        }

        fn clk_pad_to_spi(&mut self) {
            self.push(Op::ClkPadToSpi);
        }

        fn drive(&mut self) {
            self.push(Op::Drive);
        }

        fn release(&mut self) {
            self.push(Op::Release);
        }
    }

    #[derive(Clone)]
    struct MockPin {
        log: Log,
        role: Role,
        high: bool,
    }

    impl InputOutputPin for MockPin {
        fn set_as_output(&mut self) {
            self.log.borrow_mut().push(Op::AsOutput(self.role));
        }

        fn set_high(&mut self, high: bool) {
            self.high = high;
        }

        fn set_as_input(&mut self) {
            self.log.borrow_mut().push(Op::AsInput(self.role));
        }

        fn is_high(&mut self) -> bool {
            self.high
        }
    }

    struct MockDelay {
        log: Log,
        cpu_hz: u32,
    }

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    impl DelayCycles for MockDelay {
        fn cpu_clock(&self) -> u32 {
            self.cpu_hz
        }

        fn delay_cycles(&mut self, cycles: u32) {
            self.log.borrow_mut().push(Op::Delay(cycles));
        }
    }

    /// The ST-Link ladder's arithmetic, handing off at `FAST` cycles; records each half's source.
    struct MockPacing<const FAST: u32 = 8, const CALIBRATED: bool = false> {
        log: Log,
        low: u32,
        high: u32,
        preparations: usize,
    }

    impl<const FAST: u32, const CALIBRATED: bool> JtagPacing for MockPacing<FAST, CALIBRATED> {
        const FAST_HALF_CYCLES: u32 = FAST;

        fn prepare_counted(&mut self, cycles: u32) {
            let half = cycles.div_ceil(2) + if cycles > 12 { 3 } else { 0 };
            self.low = half.max(6);
            self.high = half.max(3);
            self.preparations += 1;
        }

        fn counted_low<D: DelayCycles>(&self, delay: &mut D, cycles: u32) {
            delay.delay_cycles(if CALIBRATED { self.low } else { cycles });
        }

        fn counted_high<D: DelayCycles>(&self, delay: &mut D, cycles: u32) {
            delay.delay_cycles(if CALIBRATED { self.high } else { cycles });
        }

        fn counted_output_low<D: DelayCycles>(&self, delay: &mut D, cycles: u32) {
            self.counted_high(delay, cycles);
        }

        fn pace_low(&self) {
            self.log.borrow_mut().push(Op::PaceLow);
        }

        fn pace_high(&self) {
            self.log.borrow_mut().push(Op::PaceHigh);
        }
    }

    type Probe = SpiSwd<MockPort<false>, MockPin, MockDelay, NoFastJtag>;
    /// The same engine over a seam with a separate transmit pad (carrier mode M4).
    type DuplexProbe = SpiSwd<MockPort<true>, MockPin, MockDelay, NoFastJtag>;

    const KERNEL_HZ: u32 = 200_000_000;
    const CPU_HZ: u32 = 200_000_000;

    /// A probe on the mock seam, with the construction traffic already dropped from the log.
    fn probe_of<const DUPLEX: bool>(
        log: &Log,
    ) -> SpiSwd<MockPort<DUPLEX>, MockPin, MockDelay, NoFastJtag> {
        probe_framed::<DUPLEX, false>(log)
    }

    fn probe_framed<const DUPLEX: bool, const EXACT: bool>(
        log: &Log,
    ) -> SpiSwd<MockPort<DUPLEX, EXACT>, MockPin, MockDelay, NoFastJtag> {
        let chain: &'static mut [TapConfig] =
            Box::leak(vec![TapConfig::INIT; 1].into_boxed_slice());
        let pin = |role| MockPin {
            log: log.clone(),
            role,
            high: false,
        };
        let deps = SpiSwdDeps::new(
            MockPort::new(log.clone()),
            pin(Role::Swclk),
            pin(Role::Swdio),
            pin(Role::Nreset),
            MockDelay {
                log: log.clone(),
                cpu_hz: CPU_HZ,
            },
            NoFastJtag,
            SpiLadder {
                kernels: [((), KERNEL_HZ)],
                max_sck_hz: KERNEL_HZ / 2,
            },
            chain,
            Box::leak(Box::new(KeptAck::new(Box::leak(Box::default())))),
        );
        let p = deps.into();
        log.borrow_mut().clear();
        p
    }

    fn probe(log: &Log) -> Probe {
        probe_of::<false>(log)
    }

    fn duplex_probe(log: &Log) -> DuplexProbe {
        probe_of::<true>(log)
    }

    /// A JTAG-mode probe over the recording ladder, with setup traffic dropped from the log.
    fn jtag_probe(log: &Log) -> SpiJtag<MockPort<false>, MockPin, MockDelay, MockPacing> {
        jtag_probe_at::<8>(log, CPU_HZ)
    }

    fn jtag_probe_at<const FAST: u32>(
        log: &Log,
        cpu_hz: u32,
    ) -> SpiJtag<MockPort<false>, MockPin, MockDelay, MockPacing<FAST>> {
        jtag_probe_paced::<FAST, false>(log, cpu_hz)
    }

    fn jtag_probe_paced<const FAST: u32, const CALIBRATED: bool>(
        log: &Log,
        cpu_hz: u32,
    ) -> SpiJtag<MockPort<false>, MockPin, MockDelay, MockPacing<FAST, CALIBRATED>> {
        let chain: &'static mut [TapConfig] =
            Box::leak(vec![TapConfig::INIT; 3].into_boxed_slice());
        let pin = |role| MockPin {
            log: log.clone(),
            role,
            high: false,
        };
        let deps = SpiSwdDeps::new(
            MockPort::new(log.clone()),
            pin(Role::Swclk),
            pin(Role::Swdio),
            pin(Role::Nreset),
            MockDelay {
                log: log.clone(),
                cpu_hz,
            },
            MockPacing {
                log: log.clone(),
                low: 0,
                high: 0,
                preparations: 0,
            },
            SpiLadder {
                kernels: [((), KERNEL_HZ)],
                max_sck_hz: KERNEL_HZ / 2,
            },
            chain,
            Box::leak(Box::new(KeptAck::new(Box::leak(Box::default())))),
        );
        let j: SpiJtag<_, _, _, _> = deps.into();
        log.borrow_mut().clear();
        j
    }

    /// The two 17-bit read-data frames a target would shift for `data`.
    fn data_frames(data: u32) -> [u32; 2] {
        [
            data & 0x0001_FFFF,
            ((data >> 17) & 0x7FFF) | ((parity32(data) as u32) << 15),
        ]
    }

    /// Script an OK ACK followed by `data`, then run a read.
    fn read_ok<const DUPLEX: bool>(
        p: &mut SpiSwd<MockPort<DUPLEX>, MockPin, MockDelay, NoFastJtag>,
        data: u32,
    ) -> swd::Result<u32> {
        let [lo, hi] = data_frames(data);
        // The ACK frame is the turnaround bit then the three ACK bits.
        p.d.port.rx.borrow_mut().extend([0b001 << TRN, lo, hi]);
        p.read_inner(APnDP::DP, DPRegister::DPIDR)
    }

    fn ops(log: &Log) -> Vec<Op> {
        log.borrow().clone()
    }

    fn cost(log: &Log) -> u32 {
        ops(log).into_iter().map(reg_cost).sum()
    }

    #[test]
    fn a_clean_read_is_three_sessions_of_known_register_traffic() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        let mbr = Prescaler::SLOWEST.mbr_bits();
        assert_eq!(read_ok(&mut p, 0xDEAD_BEEF), Ok(0xDEAD_BEEF));

        assert_eq!(
            ops(&log),
            vec![
                // Header: one 8-bit transmit frame; no `PadsToSpi`, the block owns the pads.
                Op::Program(mbr, 8, 1, Dir::Tx),
                Op::Enable,
                Op::Push8(0xA5),
                Op::Start,
                Op::WaitEot,
                Op::Disable,
                Op::ClearFlags,
                // ACK: turnaround plus three bits, received.
                Op::Program(mbr, 4, 1, Dir::Rx),
                Op::Enable,
                Op::Start,
                Op::WaitEot,
                Op::Pop32,
                Op::Disable,
                Op::ClearFlags,
                // Data: one session of two 17-bit frames, drained by count.
                Op::Program(mbr, 17, 2, Dir::Rx),
                Op::Enable,
                Op::Start,
                Op::WaitEot,
                Op::Pop32,
                Op::Pop32,
                Op::Disable,
                Op::ClearFlags,
            ]
        );
        assert_eq!(cost(&log), 32);
    }

    #[test]
    fn read_data_reassembles_contrasting_halves_in_frame_order() {
        for data in [0x0000_FFFF, 0xFFFF_0000, 0x8000_0001, 0x1234_5678] {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut p = probe(&log);
            assert_eq!(read_ok(&mut p, data), Ok(data));
        }

        // The low frame is shifted first; swapping the two frames must not still parse.
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        let [lo, hi] = data_frames(0x1234_5678);
        p.d.port.rx.borrow_mut().extend([0b001 << TRN, hi, lo]);
        assert_ne!(p.read_inner(APnDP::DP, DPRegister::DPIDR), Ok(0x1234_5678));
    }

    #[test]
    fn a_flipped_parity_bit_fails_the_read() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        let [lo, hi] = data_frames(0xDEAD_BEEF);
        p.d.port
            .rx
            .borrow_mut()
            .extend([0b001 << TRN, lo, hi ^ (1 << 15)]);
        assert_eq!(
            p.read_inner(APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::BadParity)
        );
    }

    #[test]
    fn a_poll_timeout_faults_the_read_and_still_retires_the_block() {
        // Every poll of a clean read.
        for k in 0..7 {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut p = probe(&log);
            p.d.port.fail_poll.set(Some(k));
            assert_eq!(
                read_ok(&mut p, 0xDEAD_BEEF),
                Err(swd::Error::AckProtocol),
                "poll {k} did not surface as a protocol failure"
            );
            let seen = ops(&log);
            assert_eq!(
                seen.iter().filter(|o| **o == Op::Disable).count(),
                seen.iter().filter(|o| **o == Op::Enable).count(),
                "poll {k} left the block enabled"
            );
            assert_eq!(seen.last(), Some(&Op::ClearFlags), "poll {k}");
        }
    }

    #[test]
    fn an_overrun_faults_the_read_rather_than_returning_the_frames() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        p.d.port.ovr.set(true);
        assert_eq!(read_ok(&mut p, 0xDEAD_BEEF), Err(swd::Error::AckProtocol));
    }

    #[test]
    fn the_eot_delay_scales_with_the_rung() {
        let mut seen = Vec::new();
        for request in [KERNEL_HZ / 2, KERNEL_HZ / 8, KERNEL_HZ / 32] {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut p = probe(&log);
            assert!(p.set_clock(request));
            assert_eq!(read_ok(&mut p, 1), Ok(1));
            seen.push(p.d.eot_spins);
        }
        // Core clock equal to the kernel: the delay is the division factor in whole iterations.
        assert_eq!(
            seen,
            vec![spin_iterations(2), spin_iterations(8), spin_iterations(32)]
        );
        assert_eq!(seen, vec![1, 3, 11]);
    }

    #[test]
    fn a_non_ok_read_ack_clocks_only_the_turnaround() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        p.d.port.rx.borrow_mut().push_back(0b010 << TRN);
        assert_eq!(
            p.read_inner(APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::AckWait)
        );

        let seen = ops(&log);
        let ack_end = seen
            .iter()
            .position(|o| *o == Op::Program(Prescaler::SLOWEST.mbr_bits(), 4, 1, Dir::Rx))
            .unwrap();
        let tail = &seen[ack_end..];
        // One released GPIO clock with the data pad an input; no session follows it.
        assert!(tail.contains(&Op::AsInput(Role::Swdio)));
        assert!(tail.contains(&Op::ClkPadToSpi));
        assert_eq!(
            tail.iter().filter(|o| matches!(o, Op::Program(..))).count(),
            1
        );
        assert!(!tail.contains(&Op::PadsToSpi));
    }

    #[test]
    fn a_non_ok_write_ack_needs_no_extra_turnaround() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        // The 5-bit write ACK session already clocks the trailing turnaround released.
        p.d.port.rx.borrow_mut().push_back(0b100 << TRN);
        assert_eq!(
            p.write_inner(APnDP::DP, DPRegister::CTRLSTAT, 0),
            Err(swd::Error::AckFault)
        );

        let seen = ops(&log);
        assert!(seen.contains(&Op::Program(Prescaler::SLOWEST.mbr_bits(), 5, 1, Dir::Rx)));
        assert!(
            !seen.iter().any(|o| matches!(o, Op::AsInput(_))),
            "no GPIO turnaround on the write error path"
        );
        // No data phase and no park frame after a non-OK write ACK.
        assert_eq!(
            seen.iter().filter(|o| matches!(o, Op::Program(..))).count(),
            2
        );
    }

    #[test]
    fn a_clean_write_pushes_the_padded_data_phase() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        p.d.port.rx.borrow_mut().push_back(0b001 << TRN);
        let data = 0xAABB_CCDDu32;
        assert_eq!(p.write_inner(APnDP::AP, DPRegister::SELECT, data), Ok(()));

        let seen = ops(&log);
        assert!(seen.contains(&Op::Program(Prescaler::SLOWEST.mbr_bits(), 8, 5, Dir::Tx)));
        for b in wdata_bytes(data) {
            assert!(seen.contains(&Op::Push8(b)));
        }
    }

    #[test]
    fn exact_writes_retry_wait_without_padding_or_data_on_fault() {
        for data in [0, 1, 0x8000_0000, 0xDEAD_BEEF, u32::MAX] {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut p = probe_framed::<false, true>(&log);
            p.d.port
                .rx
                .borrow_mut()
                .extend([0b010 << TRN, 0b001 << TRN]);
            assert_eq!(
                Swd::write(&mut p, 2, APnDP::DP, DPRegister::CTRLSTAT, data),
                Ok(())
            );
            let seen = ops(&log);
            let frames: Vec<_> = seen
                .iter()
                .filter_map(|op| match op {
                    Op::Push16(v) => Some(*v),
                    _ => None,
                })
                .collect();
            assert_eq!(frames, wdata_frames(data));
            assert_eq!(
                seen.iter()
                    .filter(|op| matches!(op, Op::Program(_, 8, 1, Dir::Tx)))
                    .count(),
                2
            );
            assert!(!seen.iter().any(|op| matches!(op, Op::Program(_, 8, 5, _))));
        }
        for (ack, error) in [
            (0b010, swd::Error::AckWait),
            (0b100, swd::Error::AckFault),
            (0b111, swd::Error::NoAck),
        ] {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut p = probe_framed::<false, true>(&log);
            p.d.port.rx.borrow_mut().push_back(ack << TRN);
            assert_eq!(
                p.write_inner(APnDP::DP, DPRegister::CTRLSTAT, 1),
                Err(error)
            );
            assert_eq!(
                ops(&log)
                    .iter()
                    .filter(|op| matches!(op, Op::Program(..)))
                    .count(),
                2
            );
            assert!(!ops(&log).iter().any(|op| matches!(op, Op::Push16(_))));
        }
    }

    #[test]
    fn exact_write_poll_faults_retire_the_session_and_allow_retry() {
        for failed_poll in 0..8 {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut p = probe_framed::<false, true>(&log);
            p.d.port.rx.borrow_mut().push_back(0b001 << TRN);
            p.d.port.fail_poll.set(Some(failed_poll));
            assert_eq!(
                p.write_inner(APnDP::DP, DPRegister::CTRLSTAT, 1),
                Err(swd::Error::AckProtocol)
            );
            assert!(ops(&log).ends_with(&[Op::Disable, Op::ClearFlags]));
            p.d.port.fail_poll.set(None);
            p.d.port.rx.borrow_mut().clear();
            p.d.port.rx.borrow_mut().push_back(0b001 << TRN);
            assert_eq!(p.write_inner(APnDP::DP, DPRegister::CTRLSTAT, 1), Ok(()));
        }
    }

    #[test]
    fn gpio_clock_receives_the_request_not_the_spi_rung() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        for requested in [1_000_000, 4_000_000, 25_000_000] {
            assert!(Swd::set_clock(&mut p, requested));
            assert_eq!(p.d.port.gpio_hz, requested);
            assert_eq!(p.d.pre, choose_prescaler(KERNEL_HZ, requested).unwrap().0);
        }
        assert!(!Swd::set_clock(&mut p, 0));
        assert_eq!(p.d.port.gpio_hz, 25_000_000);
        assert!(Swd::set_clock(&mut p, 1));
        assert_eq!(p.d.port.gpio_hz, 1);
    }

    #[test]
    fn the_exact_write_framing_shifts_three_eleven_bit_frames() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        let data = 0xAABB_CCDDu32;
        assert_eq!(
            p.tx_session16(write_data_session_exact(), &wdata_frames(data)),
            Ok(())
        );

        let seen = ops(&log);
        assert_eq!(
            seen[0],
            Op::Program(Prescaler::SLOWEST.mbr_bits(), 11, 3, Dir::Tx)
        );
        let pushed: Vec<u16> = seen
            .iter()
            .filter_map(|o| match o {
                Op::Push16(h) => Some(*h),
                _ => None,
            })
            .collect();
        assert_eq!(pushed, wdata_frames(data).to_vec());
    }

    #[test]
    fn a_gpio_sequence_is_the_only_thing_that_costs_mode_register_traffic() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        // Back-to-back reads: the pads never leave the SPI block.
        for _ in 0..3 {
            assert_eq!(read_ok(&mut p, 0x1234_5678), Ok(0x1234_5678));
        }
        assert!(!ops(&log).contains(&Op::PadsToSpi));

        // A bit-banged sequence takes the pads, and exactly the next session takes them back.
        log.borrow_mut().clear();
        assert_eq!(p.write_sequence(8, &[0xFF]), Ok(()));
        assert_eq!(read_ok(&mut p, 1), Ok(1));
        assert_eq!(
            ops(&log).iter().filter(|o| **o == Op::PadsToSpi).count(),
            1,
            "the sequence hands the pads back once, and no session repeats it"
        );
    }

    #[test]
    fn half_duplex_never_touches_a_drive_pad() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        assert_eq!(read_ok(&mut p, 0xDEAD_BEEF), Ok(0xDEAD_BEEF));
        p.d.port.rx.borrow_mut().push_back(0b001 << TRN);
        assert_eq!(p.write_inner(APnDP::DP, DPRegister::SELECT, 7), Ok(()));
        assert_eq!(p.write_sequence(8, &[0xFF]), Ok(()));
        assert!(
            !ops(&log)
                .iter()
                .any(|o| matches!(o, Op::Drive | Op::Release)),
            "the half-duplex seam has no separate transmit pad"
        );
    }

    #[test]
    fn a_duplex_read_releases_the_drive_pad_before_the_ack_and_never_redrives_it() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = duplex_probe(&log);
        let mbr = Prescaler::SLOWEST.mbr_bits();
        assert_eq!(read_ok(&mut p, 0xDEAD_BEEF), Ok(0xDEAD_BEEF));

        assert_eq!(
            ops(&log),
            vec![
                // Header: the drive pad is taken before the block is enabled.
                Op::Drive,
                Op::Program(mbr, 8, 1, Dir::Tx),
                Op::Enable,
                Op::Push8(0xA5),
                Op::Start,
                Op::WaitEot,
                Op::Disable,
                Op::ClearFlags,
                // ACK: released before the session's clocks, so the turnaround finds it high-impedance.
                Op::Release,
                Op::Program(mbr, 4, 1, Dir::Rx),
                Op::Enable,
                Op::Start,
                Op::WaitEot,
                Op::Pop32,
                Op::Disable,
                Op::ClearFlags,
                // Read data: still released, and no pad traffic to repeat it.
                Op::Program(mbr, 17, 2, Dir::Rx),
                Op::Enable,
                Op::Start,
                Op::WaitEot,
                Op::Pop32,
                Op::Pop32,
                Op::Disable,
                Op::ClearFlags,
            ]
        );
    }

    #[test]
    fn a_duplex_write_redrives_only_after_the_ack_session() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = duplex_probe(&log);
        let mbr = Prescaler::SLOWEST.mbr_bits();
        p.d.port.rx.borrow_mut().push_back(0b001 << TRN);
        assert_eq!(
            p.write_inner(APnDP::AP, DPRegister::SELECT, 0xAABB_CCDD),
            Ok(())
        );

        let seen = ops(&log);
        let at = |o: Op| seen.iter().position(|x| *x == o).unwrap();
        // The pad is retaken only after the ACK session's trailing turnaround has retired.
        assert!(at(Op::Release) < at(Op::Program(mbr, 5, 1, Dir::Rx)));
        let ack_end = at(Op::Program(mbr, 5, 1, Dir::Rx));
        let redrive = seen.iter().rposition(|o| *o == Op::Drive).unwrap();
        assert!(redrive > ack_end);
        assert!(redrive < at(Op::Program(mbr, 8, 5, Dir::Tx)));
        assert_eq!(seen.iter().filter(|o| **o == Op::Release).count(), 1);
    }

    #[test]
    fn a_duplex_error_path_leaves_the_drive_pad_released() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = duplex_probe(&log);
        p.d.port.rx.borrow_mut().push_back(0b010 << TRN);
        assert_eq!(
            p.read_inner(APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::AckWait)
        );

        let seen = ops(&log);
        let turn = seen
            .iter()
            .position(|o| *o == Op::AsInput(Role::Swdio))
            .unwrap();
        assert!(
            seen[..turn].contains(&Op::Release),
            "the drive pad is released before the ACK"
        );
        assert!(
            !seen[turn..].contains(&Op::Drive),
            "nothing re-drives after the turnaround"
        );
    }

    #[test]
    fn a_duplex_gpio_path_and_suspend_leave_the_drive_pad_released() {
        for suspend in [false, true] {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut p = duplex_probe(&log);
            // A header session takes the pad; whatever follows must give it back.
            p.d.port.rx.borrow_mut().push_back(0b001 << TRN);
            assert_eq!(p.write_inner(APnDP::DP, DPRegister::SELECT, 0), Ok(()));
            log.borrow_mut().clear();

            let mut d: SpiSwdDeps<_, _, _, _> = p.into();
            if suspend {
                d.high_impedance_mode();
            } else {
                d.process_swj_sequence(&[0xFF], 8);
            }
            let seen = ops(&log);
            assert_eq!(seen.iter().filter(|o| **o == Op::Release).count(), 1);
            assert!(!seen.contains(&Op::Drive));
            // The release precedes any pad this path drives.
            assert!(
                seen.iter().position(|o| *o == Op::Release)
                    < seen
                        .iter()
                        .position(|o| matches!(o, Op::AsOutput(_) | Op::AsInput(_))),
                "the drive pad goes high-impedance before another pad takes the net"
            );
        }
    }
    #[test]
    fn a_top_rung_sequence_takes_the_ladder_and_samples_tdo_inside_the_low_half() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe(&log);
        assert!(Jtag::set_clock(&mut j, 25_000_000));
        log.borrow_mut().clear();
        let info = SequenceInfo {
            n_bits: 2,
            capture: true,
            tms: true,
        };
        let mut rx = [0u8];
        j.sequence(info, &[0b01], &mut rx);
        // The sample sits after the paced low half and before the rising edge, on every bit.
        assert_eq!(
            ops(&log),
            vec![
                Op::TmsSet(true),
                Op::TdiSet(true),
                Op::TckLow,
                Op::PaceLow,
                Op::TdoGet,
                Op::TckHigh,
                Op::PaceHigh,
                Op::TdiSet(false),
                Op::TckLow,
                Op::PaceLow,
                Op::TdoGet,
                Op::TckHigh,
                Op::PaceHigh,
            ]
        );
        assert_eq!(rx[0], 0b11);
    }

    #[test]
    fn a_top_rung_tms_walk_takes_the_same_ladder_halves() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe(&log);
        assert!(Jtag::set_clock(&mut j, 25_000_000));
        log.borrow_mut().clear();
        j.tms_sequence(&[true]);
        assert_eq!(
            ops(&log),
            vec![
                Op::TmsSet(true),
                Op::TckLow,
                Op::PaceLow,
                Op::TckHigh,
                Op::PaceHigh,
            ]
        );
    }

    #[test]
    fn the_generic_jtag_half_period_is_capped_at_the_pad_rate_below_the_ladder() {
        // Below the handoff the counted spin serves, never shorter than the pad cap.
        let cap = half_period_cycles(CPU_HZ, <MockPacing as JtagPacing>::COUNTED_MAX_HZ);
        for (request, low, high) in [
            (1_000_000, Op::Delay(100), Op::Delay(100)),
            (4_000_000, Op::Delay(25), Op::Delay(25)),
            (12_000_000, Op::Delay(cap), Op::Delay(cap)),
            (12_400_000, Op::Delay(cap), Op::Delay(cap)),
            (25_000_000, Op::PaceLow, Op::PaceHigh),
        ] {
            let log: Log = Rc::new(RefCell::new(Vec::new()));
            let mut j = jtag_probe(&log);
            assert!(Jtag::set_clock(&mut j, request));
            log.borrow_mut().clear();
            j.tms_sequence(&[false]);
            assert_eq!(
                ops(&log),
                vec![Op::TmsSet(false), Op::TckLow, low, Op::TckHigh, high],
                "request {request}"
            );
        }
    }

    #[test]
    fn a_zero_jtag_clock_request_leaves_the_pacing_alone() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe(&log);
        assert!(Jtag::set_clock(&mut j, 4_000_000));
        assert!(!Jtag::set_clock(&mut j, 0));
        log.borrow_mut().clear();
        j.tms_sequence(&[false]);
        assert_eq!(
            ops(&log),
            vec![
                Op::TmsSet(false),
                Op::TckLow,
                Op::Delay(25),
                Op::TckHigh,
                Op::Delay(25),
            ]
        );
    }

    /// The `(tms, tdi)` cell each TAP clock presented, recovered from the recorded seam ops.
    fn tap_cells(log: &Log) -> Vec<(bool, bool)> {
        let mut tms = false;
        let mut tdi = false;
        let mut cells = Vec::new();
        for op in ops(log) {
            match op {
                Op::TmsSet(v) => tms = v,
                Op::TdiSet(v) => tdi = v,
                Op::TckLow => cells.push((tms, tdi)),
                _ => {}
            }
        }
        cells
    }

    fn assert_jtag_pacing(log: &Log, low: Op, high: Op) {
        let clocks = ops(log).iter().filter(|op| **op == Op::TckLow).count();
        assert_jtag_halves(log, &vec![low; clocks], high);
    }

    fn assert_jtag_halves(log: &Log, lows: &[Op], high: Op) {
        let seen = ops(log);
        let mut clocks = 0;
        for (i, op) in seen.iter().enumerate() {
            if *op == Op::TckLow {
                assert_eq!(seen[i + 1], lows[clocks]);
                let rising = i + 2 + usize::from(seen[i + 2] == Op::TdoGet);
                assert_eq!(seen[rising], Op::TckHigh);
                assert_eq!(seen[rising + 1], high);
                clocks += 1;
            }
        }
        assert!(clocks > 0);
        assert_eq!(clocks, lows.len());
    }

    fn scan_event_cases<const FAST: u32, const CALIBRATED: bool>(
        cpu_hz: u32,
        rates: &[(u32, Op, Op)],
    ) {
        let data = 0xA5C3_781Eu32;
        let transfer_config = TransferConfig {
            idle_cycles: 2,
            wait_retries: 1,
            match_retries: 0,
            match_mask: 0,
        };
        for &(request, low, high) in rates {
            let output_low = if CALIBRATED && matches!(high, Op::Delay(_)) {
                high
            } else {
                low
            };
            for (devices, index) in [(1, 0), (3, 0), (3, 1), (3, 2)] {
                let before = index as usize;
                let after = (devices - index - 1) as usize;
                for ack in [0b010, 0b001, 0b111] {
                    for rnw in [RnW::R, RnW::W] {
                        let log: Log = Rc::new(RefCell::new(Vec::new()));
                        let mut j = jtag_probe_paced::<FAST, CALIBRATED>(&log, cpu_hz);
                        assert!(Jtag::set_clock(&mut j, request));
                        assert!(!Jtag::set_clock(&mut j, 0));
                        j.d.jtag_config.device_count = devices;
                        j.d.jtag_config.index = index;
                        j.d.port
                            .tdo
                            .borrow_mut()
                            .extend((0..3).map(|i| (ack >> i) & 1));
                        j.d.port
                            .tdo
                            .borrow_mut()
                            .extend((0..32).map(|i| (data >> i) & 1));
                        let result = Jtag::transfer(&mut j, rnw, 3, &transfer_config, data);
                        let complete = ack != 0b001;
                        let read = if rnw == RnW::R { data } else { 0 };
                        let want = match ack {
                            0b010 => TransferResult::Ok(read),
                            0b001 => TransferResult::Wait,
                            _ => TransferResult::Fault,
                        };
                        assert_eq!(result, want);
                        let mut expected = vec![(true, false), (false, false), (false, false)];
                        expected.extend(vec![(false, true); before]);
                        expected.extend([(false, rnw == RnW::R), (false, true), (false, true)]);
                        if complete {
                            expected.extend((0..32).map(|i| {
                                (i == 31 && after == 0, rnw == RnW::W && (data >> i) & 1 != 0)
                            }));
                            expected.extend((0..after).map(|i| (i + 1 == after, true)));
                            let last_tdi = expected.last().unwrap().1;
                            expected.extend([(true, last_tdi), (false, last_tdi)]);
                        } else {
                            expected.extend([(true, true), (true, true), (false, true)]);
                        }
                        expected.extend([(false, true); 2]);
                        assert_eq!(
                            tap_cells(&log),
                            expected,
                            "rate {request}, chain {devices}/{index}"
                        );
                        assert_eq!(
                            ops(&log).iter().filter(|op| **op == Op::TdoGet).count(),
                            if complete && rnw == RnW::R { 35 } else { 3 }
                        );
                        let mut lows = vec![low; expected.len()];
                        if complete && rnw == RnW::W {
                            lows[before + 6..before + 38].fill(output_low);
                        }
                        assert_jtag_halves(&log, &lows, high);
                        assert_eq!(j.d.pacing.preparations, 2);
                    }
                }

                let log: Log = Rc::new(RefCell::new(Vec::new()));
                let mut j = jtag_probe_paced::<FAST, CALIBRATED>(&log, cpu_hz);
                assert!(Jtag::set_clock(&mut j, request));
                j.d.jtag_config.device_count = devices;
                j.d.jtag_config.index = index;
                j.d.jtag_config.scan_chain[index as usize] = TapConfig {
                    ir_length: 4,
                    ir_before: (before * 5) as u16,
                    ir_after: (after * 5) as u16,
                };
                Jtag::shift_ir(&mut j, 0b1010);
                let mut expected =
                    vec![(true, false), (true, false), (false, false), (false, false)];
                expected.extend(vec![(false, true); before * 5]);
                expected.extend([
                    (false, false),
                    (false, true),
                    (false, false),
                    (after == 0, true),
                ]);
                expected.extend((0..after * 5).map(|i| (i + 1 == after * 5, true)));
                expected.extend([(true, true), (false, true)]);
                assert_eq!(tap_cells(&log), expected);
                assert!(!ops(&log).contains(&Op::TdoGet));
                let mut lows = vec![low; expected.len()];
                lows[4 + before * 5..8 + before * 5].fill(output_low);
                assert_jtag_halves(&log, &lows, high);
                assert_eq!(j.d.pacing.preparations, 2);
            }
        }
    }

    #[test]
    fn prepared_scans_preserve_events_at_the_stlink_handoff() {
        scan_event_cases::<2, false>(
            72_000_000,
            &[
                (1_000_000, Op::Delay(36), Op::Delay(36)),
                (4_000_000, Op::Delay(9), Op::Delay(9)),
                (12_000_000, Op::Delay(3), Op::Delay(3)),
                (17_999_999, Op::Delay(3), Op::Delay(3)),
                (18_000_000, Op::PaceLow, Op::PaceHigh),
                (25_000_000, Op::PaceLow, Op::PaceHigh),
                (u32::MAX, Op::PaceLow, Op::PaceHigh),
            ],
        );
    }

    #[test]
    fn prepared_scans_preserve_other_pacing_policies() {
        scan_event_cases::<8, false>(
            CPU_HZ,
            &[
                (12_499_999, Op::Delay(9), Op::Delay(9)),
                (12_500_000, Op::PaceLow, Op::PaceHigh),
            ],
        );
        scan_event_cases::<0, false>(
            72_000_000,
            &[
                (1_000_000, Op::Delay(36), Op::Delay(36)),
                (25_000_000, Op::Delay(3), Op::Delay(3)),
                (u32::MAX, Op::Delay(3), Op::Delay(3)),
            ],
        );
    }

    #[test]
    fn calibrated_scans_keep_low_and_high_pacing_separate() {
        scan_event_cases::<2, true>(
            72_000_000,
            &[
                (250_000, Op::Delay(75), Op::Delay(75)),
                (500_000, Op::Delay(39), Op::Delay(39)),
                (1_000_000, Op::Delay(21), Op::Delay(21)),
                (2_000_000, Op::Delay(12), Op::Delay(12)),
                (2_999_999, Op::Delay(10), Op::Delay(10)),
                (3_000_000, Op::Delay(6), Op::Delay(6)),
                (4_000_000, Op::Delay(6), Op::Delay(5)),
                (5_999_999, Op::Delay(6), Op::Delay(4)),
                (6_000_000, Op::Delay(6), Op::Delay(3)),
                (17_999_999, Op::Delay(6), Op::Delay(3)),
                (18_000_000, Op::PaceLow, Op::PaceHigh),
                (u32::MAX, Op::PaceLow, Op::PaceHigh),
            ],
        );
    }

    #[test]
    fn counted_calibration_is_jtag_only_and_applies_before_the_first_clock_request() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe_paced::<2, true>(&log, 72_000_000);
        assert_eq!(j.d.pacing.preparations, 1);
        assert!(!Jtag::set_clock(&mut j, 0));
        j.tms_sequence(&[false, true]);
        assert_jtag_pacing(&log, Op::Delay(6), Op::Delay(3));
        log.borrow_mut().clear();
        let mut rx = [0];
        j.sequence(
            SequenceInfo {
                n_bits: 3,
                capture: true,
                tms: false,
            },
            &[0b101],
            &mut rx,
        );
        assert_eq!(rx, [0b111]);
        assert_jtag_pacing(&log, Op::Delay(6), Op::Delay(3));
        assert_eq!(j.d.pacing.preparations, 1);

        assert!(Jtag::set_clock(&mut j, 1_000_000));
        assert_eq!(j.d.pacing.preparations, 2);
        log.borrow_mut().clear();
        j.tms_sequence(&[false, true]);
        assert_jtag_pacing(&log, Op::Delay(21), Op::Delay(21));
        assert!(!Jtag::set_clock(&mut j, 0));
        assert_eq!(j.d.pacing.preparations, 2);

        let mut d: SpiSwdDeps<_, _, _, _> = j.into();
        assert!(d.retune_clock(1_000_000));
        let swd_half = d.bit_cycles;
        log.borrow_mut().clear();
        d.process_swj_sequence(&[0xA5], 8);
        let delays: Vec<_> = ops(&log)
            .into_iter()
            .filter(|op| matches!(op, Op::Delay(_)))
            .collect();
        assert_eq!(delays, vec![Op::Delay(swd_half); 16]);
        log.borrow_mut().clear();
        d.process_swj_pins(Pins::NRESET, Pins::NRESET, 5);
        let delays: Vec<_> = ops(&log)
            .into_iter()
            .filter(|op| matches!(op, Op::Delay(_)))
            .collect();
        assert_eq!(delays, vec![Op::Delay(360)]);
    }

    #[test]
    fn jtag_clock_updates_swj_sequences_without_requiring_a_spi_rung() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe_paced::<2, true>(&log, 72_000_000);
        for request in [250_000, 1_000_000, 2_999_999, 3_000_000, 18_000_000, 1] {
            assert!(Jtag::set_clock(&mut j, request));
            let half = half_period_cycles(72_000_000, request);
            assert_eq!(j.d.bit_cycles, half);
            assert!(!Jtag::set_clock(&mut j, 0));
            log.borrow_mut().clear();
            j.d.process_swj_sequence(&[0xA5], 8);
            let delays: Vec<_> = ops(&log)
                .into_iter()
                .filter(|op| matches!(op, Op::Delay(_)))
                .collect();
            assert_eq!(delays, vec![Op::Delay(half); 16]);
        }

        let mut d: SpiSwdDeps<_, _, _, _> = j.into();
        assert!(d.process_swj_clock(3_000_000));
        let achieved = choose_prescaler(KERNEL_HZ, 3_000_000).unwrap().1;
        assert_eq!(d.bit_cycles, half_period_cycles(72_000_000, achieved));
        assert_ne!(d.bit_cycles, half_period_cycles(72_000_000, 3_000_000));
        let before = (d.bit_cycles, d.pre, d.jtag_timing, d.pacing.preparations);
        assert!(!d.process_swj_clock(0));
        assert_eq!(
            (d.bit_cycles, d.pre, d.jtag_timing, d.pacing.preparations),
            before
        );
    }

    #[test]
    fn a_wait_read_ack_turns_the_bus_around_and_the_retry_reissues_the_header() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        let [lo, hi] = data_frames(0xDEAD_BEEF);
        // A WAIT, then the retry's OK ACK and its data frames.
        p.d.port
            .rx
            .borrow_mut()
            .extend([0b010 << TRN, 0b001 << TRN, lo, hi]);
        assert_eq!(
            Swd::read(&mut p, 2, APnDP::DP, DPRegister::DPIDR),
            Ok(0xDEAD_BEEF)
        );

        let seen = ops(&log);
        let headers: Vec<usize> = seen
            .iter()
            .enumerate()
            .filter(|(_, o)| **o == Op::Push8(0xA5))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(headers.len(), 2, "the retry re-issues the same header");
        assert!(!seen.contains(&Op::Push8(0)), "no park frame");
        let turn = seen
            .iter()
            .position(|o| *o == Op::AsInput(Role::Swdio))
            .unwrap();
        assert!(turn < headers[1], "the turnaround precedes the retry");
        assert_eq!(
            seen.iter()
                .filter(|o| **o == Op::AsInput(Role::Swdio))
                .count(),
            1,
            "only the failed attempt turns the bus around"
        );
        assert_eq!(
            seen[turn..headers[1]]
                .iter()
                .filter(|o| matches!(o, Op::Program(..)))
                .count(),
            1,
            "only the retry's header session follows the turnaround"
        );
    }

    #[test]
    fn every_wait_retry_turns_around_once_and_the_exhausted_read_still_faults() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        p.d.port
            .rx
            .borrow_mut()
            .extend([0b010 << TRN, 0b010 << TRN, 0b010 << TRN]);
        assert_eq!(
            Swd::read(&mut p, 2, APnDP::DP, DPRegister::DPIDR),
            Err(swd::Error::AckWait)
        );

        let seen = ops(&log);
        assert_eq!(seen.iter().filter(|o| **o == Op::Push8(0xA5)).count(), 3);
        assert_eq!(seen.iter().filter(|o| **o == Op::Push8(0)).count(), 0);
        assert_eq!(
            seen.iter()
                .filter(|o| **o == Op::AsInput(Role::Swdio))
                .count(),
            3,
            "each failed attempt turns the bus around exactly once"
        );
    }

    #[test]
    fn a_wait_write_ack_retries_without_parking_or_turning_the_bus_around() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut p = probe(&log);
        p.d.port
            .rx
            .borrow_mut()
            .extend([0b010 << TRN, 0b001 << TRN]);
        assert_eq!(
            Swd::write(&mut p, 2, APnDP::DP, DPRegister::CTRLSTAT, 0x1234_5678),
            Ok(())
        );

        let seen = ops(&log);
        assert_eq!(
            seen.iter()
                .filter(|o| **o == Op::Push8(0xA5 | 0x04))
                .count(),
            0,
            "the write header is not a read header"
        );
        assert_eq!(
            seen.iter()
                .filter(|o| matches!(o, Op::Program(_, 8, 1, Dir::Tx)))
                .count(),
            2,
            "the retry re-issues the header"
        );
        assert!(
            !seen.iter().any(|o| matches!(o, Op::AsInput(_))),
            "the write ACK session already clocked the turnaround"
        );
        assert_eq!(
            seen.iter()
                .filter(|o| matches!(o, Op::Program(_, 4, 1, Dir::Tx)))
                .count(),
            0,
            "a WAIT write ACK needs no park frame"
        );
    }

    #[test]
    fn read_and_write_parity_hold_on_both_duplex_settings() {
        fn parity_case<const DUPLEX: bool>() {
            for data in [0u32, 1, 0x8000_0000, 0xDEAD_BEEF, 0xFFFF_FFFF] {
                let log: Log = Rc::new(RefCell::new(Vec::new()));
                let mut p = probe_of::<DUPLEX>(&log);
                assert_eq!(
                    read_ok(&mut p, data),
                    Ok(data),
                    "duplex {DUPLEX} read {data}"
                );

                // The same frames with the parity bit flipped must not parse.
                let log: Log = Rc::new(RefCell::new(Vec::new()));
                let mut p = probe_of::<DUPLEX>(&log);
                let [lo, hi] = data_frames(data);
                p.d.port
                    .rx
                    .borrow_mut()
                    .extend([0b001 << TRN, lo, hi ^ (1 << 15)]);
                assert_eq!(
                    p.read_inner(APnDP::DP, DPRegister::DPIDR),
                    Err(swd::Error::BadParity),
                    "duplex {DUPLEX} parity {data}"
                );

                // The write data phase carries the same parity in its trailing byte.
                let log: Log = Rc::new(RefCell::new(Vec::new()));
                let mut p = probe_of::<DUPLEX>(&log);
                p.d.port.rx.borrow_mut().push_back(0b001 << TRN);
                assert_eq!(p.write_inner(APnDP::AP, DPRegister::SELECT, data), Ok(()));
                let pushed: Vec<u8> = ops(&log)
                    .iter()
                    .filter_map(|o| match o {
                        Op::Push8(b) => Some(*b),
                        _ => None,
                    })
                    .collect();
                assert_eq!(
                    pushed[1..],
                    wdata_bytes(data),
                    "duplex {DUPLEX} write {data}"
                );
                assert_eq!(*pushed.last().unwrap(), parity32(data));
            }
        }

        parity_case::<false>();
        parity_case::<true>();
    }

    /// A one-TAP chain with a four-bit instruction register, so both scans have a final bit.
    fn one_tap_chain(j: &mut SpiJtag<MockPort<false>, MockPin, MockDelay, MockPacing>) {
        let config = Jtag::config(j);
        config.device_count = 1;
        config.index = 0;
        config.scan_chain[0] = TapConfig {
            ir_length: 4,
            ir_before: 0,
            ir_after: 0,
        };
    }

    #[test]
    fn an_ir_scan_exits_the_tap_on_its_final_bit_and_hands_it_back_in_run_test_idle() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe(&log);
        one_tap_chain(&mut j);
        log.borrow_mut().clear();
        Jtag::shift_ir(&mut j, 0b1010);

        let cells = tap_cells(&log);
        // Run-Test/Idle -> Shift-IR, the four IR bits LSB-first, then Exit1 -> Update -> Idle.
        assert_eq!(
            cells,
            vec![
                (true, false),
                (true, false),
                (false, false),
                (false, false),
                (false, false),
                (false, true),
                (false, false),
                (true, true),
                (true, true),
                (false, true),
            ]
        );
    }

    #[test]
    fn a_word_transfer_exits_on_its_last_data_bit_and_returns_to_run_test_idle() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe(&log);
        one_tap_chain(&mut j);
        // The three ACK bits are sampled LSB-first; 0b010 is the OK the scan needs.
        j.d.port.tdo.borrow_mut().extend([0, 1, 0]);
        log.borrow_mut().clear();
        let config = TransferConfig {
            idle_cycles: 0,
            wait_retries: 1,
            match_retries: 0,
            match_mask: 0,
        };
        assert!(matches!(
            Jtag::transfer(&mut j, RnW::R, 0, &config, 0),
            TransferResult::Ok(0xFFFF_FFFF)
        ));

        let cells = tap_cells(&log);
        // Three entry clocks, three ACK clocks, 32 data clocks, two handback clocks.
        assert_eq!(cells.len(), 3 + 3 + 32 + 2);
        assert_eq!(
            &cells[..3],
            &[(true, false), (false, false), (false, false)]
        );
        assert!(
            cells[6..37].iter().all(|c| !c.0),
            "the data phase stays in Shift-DR until its last bit"
        );
        assert!(cells[37].0, "the final data bit exits Shift-DR");
        assert!(cells[38].0, "Exit1-DR -> Update-DR");
        assert!(!cells[39].0, "Update-DR -> Run-Test/Idle");
    }

    #[test]
    fn idle_cycles_are_clocked_in_run_test_idle_after_the_handback() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe(&log);
        one_tap_chain(&mut j);
        j.d.port.tdo.borrow_mut().extend([0, 1, 0]);
        log.borrow_mut().clear();
        let config = TransferConfig {
            idle_cycles: 3,
            wait_retries: 1,
            match_retries: 0,
            match_mask: 0,
        };
        assert!(matches!(
            Jtag::transfer(&mut j, RnW::R, 0, &config, 0),
            TransferResult::Ok(_)
        ));

        let cells = tap_cells(&log);
        assert_eq!(cells.len(), 3 + 3 + 32 + 2 + 3);
        assert_eq!(
            &cells[40..],
            &[(false, true), (false, true), (false, true)],
            "idle cycles hold TMS low with the provided TDI fill"
        );
    }

    #[test]
    fn a_wait_ack_scan_walks_back_to_run_test_idle_without_a_data_phase() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let mut j = jtag_probe(&log);
        one_tap_chain(&mut j);
        j.d.port.tdo.borrow_mut().extend([1, 0, 0]);
        log.borrow_mut().clear();
        let config = TransferConfig {
            idle_cycles: 0,
            wait_retries: 1,
            match_retries: 0,
            match_mask: 0,
        };
        assert!(matches!(
            Jtag::transfer(&mut j, RnW::R, 0, &config, 0),
            TransferResult::Wait
        ));

        let cells = tap_cells(&log);
        assert_eq!(cells.len(), 3 + 3 + 3, "no data phase on a WAIT ACK");
        assert_eq!(
            &cells[6..],
            &[(true, false), (true, false), (false, false)],
            "Shift-DR -> Exit1 -> Update -> Run-Test/Idle"
        );
    }

    #[test]
    fn the_swj_sequence_bit_bangs_on_gpio_and_gives_the_pads_straight_back() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let p = probe(&log);
        let mut d: SpiSwdDeps<_, _, _, _> = p.into();
        // The 16-bit JTAG-to-SWD switching sequence.
        d.process_swj_sequence(&[0x9E, 0xE7], 16);

        let seen = ops(&log);
        assert_eq!(
            seen.iter()
                .position(|o| *o == Op::AsOutput(Role::Swclk))
                .unwrap(),
            0,
            "the clock pad is parked at its idle level before it is driven"
        );
        assert_eq!(seen[1], Op::AsOutput(Role::Swdio));
        assert_eq!(
            seen.iter().filter(|o| matches!(o, Op::Delay(_))).count(),
            32,
            "one low and one high half per bit"
        );
        assert_eq!(seen.last(), Some(&Op::PadsToSpi));
        assert!(
            !seen.contains(&Op::ClkPadToSpi),
            "a write reclaims both pads"
        );
    }

    #[test]
    fn the_high_impedance_park_releases_all_three_lines_and_leaves_the_pads_off_the_spi_block() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let p = probe(&log);
        let mut d: SpiSwdDeps<_, _, _, _> = p.into();
        d.high_impedance_mode();

        assert_eq!(
            ops(&log),
            vec![
                Op::AsInput(Role::Swclk),
                Op::AsInput(Role::Swdio),
                Op::AsInput(Role::Nreset),
            ]
        );

        // The pads stay on GPIO until a session wants them, and then exactly one hand-back.
        let mut p: SpiSwd<_, _, _, _> = d.into();
        p.d.swclk.high = true;
        log.borrow_mut().clear();
        assert_eq!(read_ok(&mut p, 1), Ok(1));
        assert_eq!(ops(&log)[0], Op::PadsToSpi);
        assert_eq!(ops(&log).iter().filter(|o| **o == Op::PadsToSpi).count(), 1);
    }

    #[test]
    fn a_header_after_swclk_is_parked_low_clocks_its_start_bit_on_gpio() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let p = probe(&log);
        let mut d: SpiSwdDeps<_, _, _, _> = p.into();
        d.process_swj_pins(Pins::SWDIO, Pins::SWCLK | Pins::SWDIO, 0);
        let mut p: SpiSwd<_, _, _, _> = d.into();
        log.borrow_mut().clear();
        assert_eq!(read_ok(&mut p, 1), Ok(1));

        let seen = ops(&log);
        let handback = seen.iter().position(|o| *o == Op::PadsToSpi).unwrap();
        assert_eq!(
            seen[..handback]
                .iter()
                .filter(|o| matches!(o, Op::Delay(_)))
                .count(),
            2,
            "one GPIO cell, the start bit, before the SPI block takes SWCLK"
        );
        let req = request_byte(APnDP::DP, RnW::R, DPRegister::DPIDR);
        assert_eq!(
            seen[handback + 1],
            Op::Program(Prescaler::SLOWEST.mbr_bits(), 7, 1, Dir::Tx)
        );
        assert!(seen.contains(&Op::Push8(req >> 1)));
        assert_eq!(seen.iter().filter(|o| **o == Op::PadsToSpi).count(), 1);
    }

    #[test]
    fn a_swd_to_jtag_round_trip_takes_the_pads_once_and_the_next_session_reclaims_them() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let p = probe(&log);
        let d: SpiSwdDeps<_, _, _, _> = p.into();
        let j: SpiJtag<_, _, _, _> = d.into();

        assert_eq!(
            ops(&log),
            vec![
                Op::AsOutput(Role::Swclk),
                Op::AsOutput(Role::Swdio),
                Op::JtagAcquire,
            ],
            "both idle levels are latched before the JTAG-only lines are taken"
        );

        log.borrow_mut().clear();
        let d: SpiSwdDeps<_, _, _, _> = j.into();
        assert_eq!(ops(&log), vec![Op::JtagRelease]);

        // Only the next session pays for the mode registers, and only once.
        let mut p: SpiSwd<_, _, _, _> = d.into();
        log.borrow_mut().clear();
        assert_eq!(read_ok(&mut p, 1), Ok(1));
        assert_eq!(read_ok(&mut p, 1), Ok(1));
        assert_eq!(ops(&log).iter().filter(|o| **o == Op::PadsToSpi).count(), 1);
    }

    #[test]
    fn swj_pins_latch_each_level_before_driving_it_and_report_every_line() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let p = probe(&log);
        let mut d: SpiSwdDeps<_, _, _, _> = p.into();
        let mask = Pins::SWCLK | Pins::SWDIO | Pins::NRESET;
        let read = d.process_swj_pins(Pins::SWCLK | Pins::NRESET, mask, 0);

        assert_eq!(
            ops(&log),
            vec![
                Op::AsOutput(Role::Swclk),
                Op::AsOutput(Role::Swdio),
                Op::AsOutput(Role::Nreset),
            ],
            "a pad becomes an output only after its level is latched"
        );
        // The read-back reflects the levels just driven; NTRST is reported present but high.
        assert!(read.contains(Pins::SWCLK));
        assert!(!read.contains(Pins::SWDIO));
        assert!(read.contains(Pins::NRESET));
        assert!(read.contains(Pins::NTRST));
        assert!(!read.contains(Pins::TDI) && !read.contains(Pins::TDO));
    }

    #[test]
    fn a_swj_pin_wait_spends_the_requested_microseconds_at_the_core_clock() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let p = probe(&log);
        let mut d: SpiSwdDeps<_, _, _, _> = p.into();
        d.process_swj_pins(Pins::NRESET, Pins::NRESET, 10);
        assert!(ops(&log).contains(&Op::Delay(10 * (CPU_HZ / 1_000_000))));
    }

    #[test]
    fn a_swj_clock_primes_both_the_swd_rung_and_the_jtag_pacing() {
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let p = probe(&log);
        let mut d: SpiSwdDeps<_, _, _, _> = p.into();
        assert!(d.process_swj_clock(KERNEL_HZ / 8));
        assert_eq!(d.pre, choose_prescaler(KERNEL_HZ, KERNEL_HZ / 8).unwrap().0);
        // The JTAG request is capped at the rate the shared pads carry.
        assert_eq!(
            d.jtag_timing.sequence_half_cycles(),
            half_period_cycles(CPU_HZ, <MockPacing as JtagPacing>::COUNTED_MAX_HZ)
        );

        // Zero satisfies neither timing, and neither is disturbed.
        let before = (d.pre, d.jtag_timing, d.eot_spins);
        assert!(!d.process_swj_clock(0));
        assert_eq!((d.pre, d.jtag_timing, d.eot_spins), before);
    }
}
