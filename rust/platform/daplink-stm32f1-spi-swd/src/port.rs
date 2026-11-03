//! F1 register seam for the SPI-session SWD engine: the block can clock and observe the wire, never drive it.

use core::cell::Cell;
use core::marker::PhantomData;

use daplink_rs::DelayCycles;
use daplink_spi_swd::session::half_period_cycles;
use daplink_spi_swd::{
    Dir, JtagPortOps, PortResult, SpiFault, SpiJtag, SpiPortOps, SpiSwd, SpiSwdDeps,
};

use crate::pac;
use daplink_rs::{DynCycleDelay, PreparedSpin};

/// CNF+MODE nibbles (RM0008); for input with pull, ODR selects up/down.
const CFG_OUTPUT_PP_50M: u32 = 0b0011;
const CFG_AF_PP_50M: u32 = 0b1011;
const CFG_INPUT_FLOATING: u32 = 0b0100;
const CFG_INPUT_PULL: u32 = 0b1000;

/// The per-board half of the seam, a zero-sized type so every member monomorphizes to a constant.
pub trait SpiSwdBoard {
    /// Pin index of SWCLK / TCK within [`clk_port`](Self::clk_port) -- also the SPI SCK pad.
    const CLK_PIN: usize;
    /// Pin index of SWDIO / TMS within [`clk_port`](Self::clk_port) -- also the SPI MISO pad.
    const IO_PIN: usize;
    /// Pin index of TDI within [`jtag_port`](Self::jtag_port).
    const TDI_PIN: usize;
    /// Pin index of TDO within [`jtag_port`](Self::jtag_port).
    const TDO_PIN: usize;

    /// GPIO block carrying SWCLK/SWDIO.
    fn clk_port() -> &'static pac::gpioa::RegisterBlock;
    /// GPIO block carrying TDI/TDO.
    fn jtag_port() -> &'static pac::gpioa::RegisterBlock;
    /// The SPI register block whose SCK/MISO pads are the two probe lines.
    fn spi() -> &'static pac::spi2::RegisterBlock;
    /// Ungate the SPI peripheral clock; there is no kernel mux on this family.
    fn init_clocks();

    /// GPIO half-period pacing; a board whose spin runs slower from flash overrides the default.
    fn prepare_gpio_half(delay: DynCycleDelay, cycles: u32) -> PreparedSpin {
        delay.prepare(cycles)
    }
}

/// Bound on every SPI status poll: an undrivable rung must fail the transfer, not the USB task.
const SPI_POLL_LIMIT: u32 = 200_000;

/// Receive frames one session can retain; the engine's largest receive session has two.
const RX_FRAMES: usize = 4;

/// F1 seam: `program` latches a session, `push8`/`push16`/`start` render it; the rest is bookkeeping.
/// It assumes sole use of the board's SPI block and probe pads.
pub struct F1SpiPort<M: SpiSwdBoard> {
    delay: DynCycleDelay,
    bits: Cell<u8>,
    frames: Cell<u16>,
    dir: Cell<Dir>,
    mbr: Cell<u8>,
    /// Prepared GPIO half-period, independent of the SPI divider.
    half: Cell<PreparedSpin>,
    rx: [Cell<u32>; RX_FRAMES],
    rx_count: Cell<usize>,
    rx_pos: Cell<usize>,
    /// The fault the clocking raised; latched here because `start` renders the session itself.
    fault: Cell<Option<SpiFault>>,
    _board: PhantomData<M>,
}

/// Rewrite one pin's CNF+MODE nibble in CRL/CRH.
fn set_cfg(port: &pac::gpioa::RegisterBlock, pin: usize, cfg: u32) {
    let shift = 4 * (pin % 8);
    if pin < 8 {
        port.crl()
            .modify(|r, w| unsafe { w.bits((r.bits() & !(0b1111 << shift)) | (cfg << shift)) });
    } else {
        port.crh()
            .modify(|r, w| unsafe { w.bits((r.bits() & !(0b1111 << shift)) | (cfg << shift)) });
    }
}

#[inline(always)]
fn set_pin(port: &pac::gpioa::RegisterBlock, pin: usize, high: bool) {
    let bit = if high { pin } else { pin + 16 };
    port.bsrr().write(|w| unsafe { w.bits(1 << bit) });
}

fn read_data_frames(data: u32, tail: u32) -> [u32; 2] {
    [data & 0x1_ffff, (data >> 17) | ((tail & 3) << 15)]
}

impl<M: SpiSwdBoard> F1SpiPort<M> {
    /// Configure mapped SPI; sessions stay within 32 bits and four retained RX frames.
    pub fn new(delay: DynCycleDelay, kernel_hz: u32) -> Self {
        M::init_clocks();

        set_pin(M::jtag_port(), M::TDI_PIN, false);
        set_cfg(M::jtag_port(), M::TDI_PIN, CFG_OUTPUT_PP_50M);
        set_cfg(M::jtag_port(), M::TDO_PIN, CFG_INPUT_FLOATING);

        Self {
            delay,
            bits: Cell::new(8),
            frames: Cell::new(0),
            dir: Cell::new(Dir::Tx),
            mbr: Cell::new(7),
            half: Cell::new(M::prepare_gpio_half(
                delay,
                half_period_cycles(delay.cpu_clock(), kernel_hz / 256),
            )),
            rx: [const { Cell::new(0) }; RX_FRAMES],
            rx_count: Cell::new(0),
            rx_pos: Cell::new(0),
            fault: Cell::new(None),
            _board: PhantomData,
        }
    }

    #[inline(always)]
    fn clk_high(&self) {
        set_pin(M::clk_port(), M::CLK_PIN, true);
    }

    #[inline(always)]
    fn clk_low(&self) {
        set_pin(M::clk_port(), M::CLK_PIN, false);
    }

    #[inline(always)]
    fn io_set(&self, high: bool) {
        set_pin(M::clk_port(), M::IO_PIN, high);
    }

    #[inline(always)]
    fn io_get(&self) -> u32 {
        (M::clk_port().idr().read().bits() >> M::IO_PIN) & 1
    }

    #[inline(always)]
    fn wait_half(&self) {
        self.half.get().run();
    }

    /// Park SWCLK as a driven-high GPIO (the mode-3 idle level, latched before any mode change).
    fn clk_to_gpio(&self) {
        self.clk_high();
        set_cfg(M::clk_port(), M::CLK_PIN, CFG_OUTPUT_PP_50M);
    }

    /// Shift `bits` of `word` out LSB-first on GPIO; the target samples the rising edge.
    fn gpio_tx(&self, word: u32, bits: usize) {
        for i in 0..bits {
            self.io_set(word & (1 << i) != 0);
            self.clk_low();
            self.wait_half();
            self.clk_high();
            self.wait_half();
        }
    }

    /// Clock `bits` in LSB-first on GPIO, sampling in the low half of each cell.
    fn gpio_rx(&self, bits: usize) -> u32 {
        let mut v = 0u32;
        for i in 0..bits {
            self.clk_low();
            self.wait_half();
            v |= self.io_get() << i;
            self.clk_high();
            self.wait_half();
        }
        v
    }

    fn spi_acquire(&self) {
        let s = M::spi();
        let cr1 = (1 << 0)
            | (1 << 1)
            | (1 << 2)
            | (u32::from(self.mbr.get()) << 3)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)
            | (1 << 9)
            | (1 << 11);
        s.cr1().write(|w| unsafe { w.bits(cr1 as u16) });
        // `DR` then `SR` drops what a bounded-out pass left and clears a latched overrun.
        let _ = s.dr().read();
        let _ = s.sr().read();

        // Mode 3 idles SCK high and the pad is parked driven-high, so the handover is edge-free.
        set_cfg(M::clk_port(), M::CLK_PIN, CFG_AF_PP_50M);
    }

    #[inline(always)]
    fn spi_rx16_frame(&self) -> PortResult<u32> {
        let s = M::spi();
        s.dr().write(|w| unsafe { w.bits(0) });
        let mut fault = self.poll(|sr| sr & 1 != 0).err();
        // Clocking is over once the frame lands, so that status word also settles the overrun.
        if fault.is_none() && s.sr().read().bits() & (1 << 6) != 0 {
            fault = Some(SpiFault::Overrun);
        }
        let v = u32::from(s.dr().read().bits());
        fault = fault.or(self.poll(|sr| sr & (1 << 7) == 0).err());
        match fault {
            Some(f) => Err(f),
            None => Ok(v),
        }
    }

    fn spi_retire(&self) {
        // A faulted session must not leave SCK on the AF.
        self.clk_to_gpio();
        M::spi()
            .cr1()
            .write(|w| unsafe { w.bits((1 << 2) | (1 << 8) | (1 << 9)) });
    }

    fn spi_rx16(&self) -> PortResult<u32> {
        self.spi_acquire();
        let result = self.spi_rx16_frame();
        self.spi_retire();
        // SPI retirement does not supply the GPIO tail's first high half.
        if result.is_ok() {
            self.wait_half();
        }
        result
    }

    fn spi_rx32(&self) -> PortResult<u32> {
        self.spi_acquire();
        let result = self
            .spi_rx16_frame()
            .and_then(|lo| self.spi_rx16_frame().map(|hi| lo | (hi << 16)));
        self.spi_retire();
        if result.is_ok() {
            self.wait_half();
        }
        result
    }

    /// Spin on a status-register predicate up to the poll bound.
    fn poll(&self, ready: impl Fn(u16) -> bool) -> PortResult<()> {
        for _ in 0..SPI_POLL_LIMIT {
            if ready(M::spi().sr().read().bits()) {
                return Ok(());
            }
        }
        Err(SpiFault::Timeout)
    }

    fn push_rx(&self, v: u32) {
        let n = self.rx_count.get();
        debug_assert!(n < RX_FRAMES, "receive session exceeds the frame cap");
        self.rx[n.min(RX_FRAMES - 1)].set(v);
        self.rx_count.set(n + 1);
    }
}

impl<M: SpiSwdBoard> SpiPortOps for F1SpiPort<M> {
    // One data pad, driven or released by `enable`, so the engine makes no ownership decision.
    const DUPLEX: bool = false;
    const EXACT_WRITE_FRAMING: bool = true;
    // SPI frames wait for BSY to clear before shutdown; GPIO frames complete synchronously.
    const EOT_DELAY: bool = false;
    type Source = ();

    fn set_gpio_clock(&mut self, requested_hz: u32) {
        self.half.set(M::prepare_gpio_half(
            self.delay,
            half_period_cycles(self.delay.cpu_clock(), requested_hz),
        ));
    }

    fn configure(&mut self) {
        M::spi().cr2().write(|w| w);
    }

    fn program(&self, mbr: u8, bits: u8, frames: u16, dir: Dir) {
        self.mbr.set(mbr);
        self.bits.set(bits);
        self.frames.set(frames);
        self.dir.set(dir);
    }

    fn enable(&self) {
        // The release to pull-up on receive is what serves an ACK session's leading turnaround.
        match self.dir.get() {
            Dir::Tx => set_cfg(M::clk_port(), M::IO_PIN, CFG_OUTPUT_PP_50M),
            Dir::Rx => {
                self.io_set(true);
                set_cfg(M::clk_port(), M::IO_PIN, CFG_INPUT_PULL);
            }
        }
    }

    fn disable(&self) {}

    fn start(&self) {
        if self.dir.get() == Dir::Rx {
            let bits = self.bits.get() as usize;
            if bits == 17 && self.frames.get() == 2 {
                match self.spi_rx32() {
                    Ok(data) => {
                        let tail = self.gpio_rx(2);
                        for frame in read_data_frames(data, tail) {
                            self.push_rx(frame);
                        }
                    }
                    Err(f) => self.fault.set(Some(f)),
                }
                return;
            }
            for _ in 0..self.frames.get() {
                // A 17-bit frame is 16 SPI bits plus one GPIO bit; narrower ones are all GPIO.
                let v = if bits == 17 {
                    match self.spi_rx16() {
                        Ok(v) => v | (self.gpio_rx(1) << 16),
                        Err(f) => {
                            self.fault.set(Some(f));
                            return;
                        }
                    }
                } else {
                    self.gpio_rx(bits)
                };
                self.push_rx(v);
            }
        }
    }

    fn push8(&self, byte: u8) -> PortResult<()> {
        // Transmit frames render immediately; the wire tolerates the inter-frame gap.
        if self.dir.get() == Dir::Tx {
            self.gpio_tx(u32::from(byte), (self.bits.get() as usize).min(8));
        }
        Ok(())
    }

    fn push16(&self, half: u16) -> PortResult<()> {
        if self.dir.get() == Dir::Tx {
            self.gpio_tx(u32::from(half), (self.bits.get() as usize).min(16));
        }
        Ok(())
    }

    fn wait_eot(&self) -> PortResult<()> {
        // The session was clocked out by `push8`/`start`, so only its latched fault is left.
        match self.fault.get() {
            Some(f) => Err(f),
            None => Ok(()),
        }
    }

    fn pop32(&self) -> PortResult<u32> {
        let i = self.rx_pos.get();
        self.rx_pos.set(i + 1);
        Ok(self.rx[i.min(RX_FRAMES - 1)].get())
    }

    fn clear_flags(&self) -> PortResult<()> {
        // The fault was read out by `wait_eot`; this only resets the retained frames.
        self.rx_count.set(0);
        self.rx_pos.set(0);
        self.fault.set(None);
        Ok(())
    }

    fn pads_to_spi(&mut self) {
        // Session idle: SWCLK a driven-high GPIO, SWDIO released to its pull-up.
        self.clk_to_gpio();
        self.io_set(true);
        set_cfg(M::clk_port(), M::IO_PIN, CFG_INPUT_PULL);
    }

    fn clk_pad_to_spi(&mut self) {
        self.clk_to_gpio();
    }

    // Half-duplex: there is no separate transmit pad, and the engine compiles both calls out.
    fn drive(&mut self) {}

    fn release(&mut self) {}
}

impl<M: SpiSwdBoard> JtagPortOps for F1SpiPort<M> {
    fn jtag_pads_acquire(&mut self) {
        set_pin(M::jtag_port(), M::TDI_PIN, false);
        set_cfg(M::jtag_port(), M::TDI_PIN, CFG_OUTPUT_PP_50M);
        set_cfg(M::jtag_port(), M::TDO_PIN, CFG_INPUT_FLOATING);
    }

    fn jtag_pads_release(&mut self) {
        set_pin(M::jtag_port(), M::TDI_PIN, true);
        set_cfg(M::jtag_port(), M::TDI_PIN, CFG_INPUT_PULL);
    }

    #[inline(always)]
    fn tck_low(&self) {
        self.clk_low();
    }

    #[inline(always)]
    fn tck_high(&self) {
        self.clk_high();
    }

    #[inline(always)]
    fn tms_set(&self, high: bool) {
        self.io_set(high);
    }

    #[inline(always)]
    fn tdi_set(&self, high: bool) {
        set_pin(M::jtag_port(), M::TDI_PIN, high);
    }

    #[inline(always)]
    fn tdo_get(&self) -> u32 {
        (M::jtag_port().idr().read().bits() >> M::TDO_PIN) & 1
    }

    fn tdi_is_high(&mut self) -> bool {
        M::jtag_port().idr().read().bits() & (1 << M::TDI_PIN) != 0
    }

    fn tdo_is_high(&mut self) -> bool {
        M::jtag_port().idr().read().bits() & (1 << M::TDO_PIN) != 0
    }
}

/// The SPI-session DAP engine over a board seam `M`: SPI-assisted SWD; JTAG is GPIO bit-bang only.
pub type SpiSwdJtagDap<
    'a,
    M,
    IO,
    D,
    J,
    SWO = daplink_rs::NoSwo,
    V = daplink_rs::NoVendor,
    LEDS = daplink_rs::NoopLeds,
> = daplink_rs::DapProbe<
    'a,
    SpiSwdDeps<F1SpiPort<M>, IO, D, J>,
    LEDS,
    D,
    SpiJtag<F1SpiPort<M>, IO, D, J>,
    SpiSwd<F1SpiPort<M>, IO, D, J>,
    SWO,
    V,
>;

#[cfg(test)]
mod tests {
    use super::{read_data_frames, RX_FRAMES};
    use daplink_spi_swd::session::{
        combine_read_data, header_session, read_ack_session, read_data_session, write_ack_session,
        write_data_session, write_data_session_exact,
    };
    use daplink_spi_swd::Dir;

    #[test]
    fn engine_sessions_fit_the_port_geometry() {
        for s in [read_ack_session(), write_ack_session(), read_data_session()] {
            assert_eq!(s.dir, Dir::Rx);
            assert!(usize::from(s.frames) <= RX_FRAMES, "{s:?}");
            assert!(s.bits <= 17, "{s:?}");
        }
        // `push8` renders at most eight bits a frame, `push16` sixteen.
        for s in [header_session(), write_data_session()] {
            assert_eq!(s.dir, Dir::Tx);
            assert!(s.bits <= 8, "{s:?}");
        }
        let exact = write_data_session_exact();
        assert_eq!(exact.dir, Dir::Tx);
        assert!(exact.bits <= 16);
        // The coalesced read path `start` takes.
        assert_eq!(
            (read_data_session().bits, read_data_session().frames),
            (17, 2)
        );
    }

    #[test]
    fn read_frames_preserve_walking_data_bits() {
        for bit in 0..32 {
            for data in [1u32 << bit, !(1u32 << bit)] {
                for tail in 0..4 {
                    let [lo, hi] = read_data_frames(data, tail);
                    let wire = u64::from(lo) | (u64::from(hi) << 17);
                    assert_eq!(wire, u64::from(data) | (u64::from(tail) << 32));
                    assert_eq!(combine_read_data(lo, hi), (data, (tail & 1) as u8));
                    assert!(lo < (1 << 17) && hi < (1 << 17));
                }
            }
        }
    }

    #[test]
    fn read_frames_keep_parity_and_turnaround_separate() {
        assert_eq!(read_data_frames(0, 0), [0, 0]);
        assert_eq!(read_data_frames(0, 1), [0, 1 << 15]);
        assert_eq!(read_data_frames(0, 2), [0, 1 << 16]);
        assert_eq!(read_data_frames(0, 3), [0, 3 << 15]);
        assert_eq!(read_data_frames(u32::MAX, 3), [0x1_ffff, 0x1_ffff]);
        assert_eq!(read_data_frames(0, !3), [0, 0]);
    }
}
