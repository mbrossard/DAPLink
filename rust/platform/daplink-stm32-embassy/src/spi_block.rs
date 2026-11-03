//! `DAP_Transfer` and `DAP_TransferBlock` as back-to-back SPI sessions ahead of dap-rs.

use core::marker::PhantomData;

use daplink_core::dap_wire::{
    encode_block_response, transfer_idle_cycles, transfer_wait_attempts, BlockRequest,
};
use daplink_rs::dap_rs::swd::{self, APnDP, DPRegister, RnW};
use daplink_spi_swd::session::{
    ack_from_frame, combine_read_data, half_period_cycles, parity32, request_byte, spin_iterations,
    wdata_frames, TRN,
};
use daplink_spi_swd::{spin, SpiLadder};
use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::Moder;
use embassy_stm32::pac::spi::regs::{Cfg1, Cr1};
use embassy_stm32::pac::spi::vals::Mbr;

use crate::block_common::{dp_register, plain_transfers, retry, run_length, status_byte, BlockEnd};
use crate::dap_mode::DapMode;
use crate::spi_swd::{SpiSwdBoard, SPI_POLL_LIMIT};

/// One SWD word each way.
pub trait WordOps {
    fn read(&mut self, req: u8) -> swd::Result<u32>;
    fn write(&mut self, req: u8, data: u32) -> swd::Result<()>;
    /// Clock `cycles` cells with SWDIO driven low.
    fn idle(&mut self, cycles: u8);
    /// Close whatever the last word left open; nothing by default.
    fn disarm(&mut self) {}
}

/// A read word, then the host's idle cycles after an OK ACK, a bad parity included.
#[inline(never)]
#[cfg_attr(target_os = "none", link_section = ".ramcode.spi_block")]
fn read_word_idle<W: WordOps>(w: &mut W, req: u8, idle: u8) -> swd::Result<u32> {
    let res = w.read(req);
    if matches!(res, Ok(_) | Err(swd::Error::BadParity)) {
        w.idle(idle);
    }
    res
}

/// A write word, then the host's idle cycles after an OK ACK.
#[inline(never)]
#[cfg_attr(target_os = "none", link_section = ".ramcode.spi_block")]
fn write_word_idle<W: WordOps>(w: &mut W, req: u8, data: u32, idle: u8) -> swd::Result<()> {
    let res = w.write(req, data);
    if res.is_ok() {
        w.idle(idle);
    }
    res
}

/// A block's read word: inline, or the resident word that clocks the idle cycles.
macro_rules! read_word {
    ($w:ident, $req:expr) => {
        $w.read($req)
    };
    ($w:ident, $req:expr, $idle:ident) => {
        read_word_idle($w, $req, $idle)
    };
}

/// A block's write word, as [`read_word`].
macro_rules! write_word {
    ($w:ident, $req:expr, $data:expr) => {
        $w.write($req, $data)
    };
    ($w:ident, $req:expr, $data:expr, $idle:ident) => {
        write_word_idle($w, $req, $data, $idle)
    };
}

/// The block loops, each defined twice: resident, and in flash over resident idle-cycle words.
macro_rules! read_block_fn {
    ($(#[$m:meta])* $name:ident $(, $idle:ident)?) => {
        $(#[$m])*
        fn $name<W: WordOps>(
            w: &mut W,
            req: u8,
            last: u8,
            posted: bool,
            count: usize,
            attempts: usize,
            out: &mut [u8],
            $($idle: u8,)?
        ) -> BlockEnd {
            if posted {
                if let Err(e) = retry!(attempts, read_word!(w, req $(, $idle)?)) {
                    return BlockEnd {
                        ran: 1,
                        status: status_byte(e),
                        words: 0,
                    };
                }
            }
            let words = &mut out[4..4 + 4 * count];
            let tail = words.len().saturating_sub(4);
            for (done, chunk) in words[..tail].chunks_exact_mut(4).enumerate() {
                match retry!(attempts, read_word!(w, req $(, $idle)?)) {
                    Ok(v) => chunk.copy_from_slice(&v.to_le_bytes()),
                    Err(e) => {
                        return BlockEnd {
                            ran: (done + 1) as u16,
                            status: status_byte(e),
                            words: done,
                        }
                    }
                }
            }
            match retry!(attempts, read_word!(w, last $(, $idle)?)) {
                Ok(v) => words[tail..].copy_from_slice(&v.to_le_bytes()),
                Err(e) => {
                    return BlockEnd {
                        ran: count as u16,
                        status: status_byte(e),
                        words: count - 1,
                    }
                }
            }
            BlockEnd {
                ran: count as u16,
                status: 1,
                words: count,
            }
        }
    };
}

read_block_fn!(
    /// A whole read block: one priming read for an AP, then `count` reads whose last is DP RDBUFF.
    #[inline(never)]
    #[cfg_attr(target_os = "none", link_section = ".ramcode.spi_block")]
    read_block
);

read_block_fn!(
    #[inline(never)]
    #[allow(clippy::too_many_arguments)]
    read_block_idle,
    idle
);

macro_rules! write_block_fn {
    ($(#[$m:meta])* $name:ident $(, $idle:ident)?) => {
        $(#[$m])*
        fn $name<W: WordOps>(
            w: &mut W,
            req: u8,
            attempts: usize,
            data: &[u8],
            $($idle: u8,)?
        ) -> BlockEnd {
            for (done, chunk) in data.chunks_exact(4).enumerate() {
                let v = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                if let Err(e) = retry!(attempts, write_word!(w, req, v $(, $idle)?)) {
                    return BlockEnd {
                        ran: (done + 1) as u16,
                        status: status_byte(e),
                        words: 0,
                    };
                }
            }
            BlockEnd {
                ran: (data.len() / 4) as u16,
                status: 1,
                words: 0,
            }
        }
    };
}

write_block_fn!(
    #[inline(never)]
    #[cfg_attr(target_os = "none", link_section = ".ramcode.spi_block")]
    write_block
);

write_block_fn!(
    #[inline(never)]
    write_block_idle,
    idle
);

macro_rules! transfer_items_fn {
    ($(#[$m:meta])* $name:ident $(, $idle:ident)?) => {
        $(#[$m])*
        fn $name<W: WordOps>(
            w: &mut W,
            count: usize,
            attempts: usize,
            items: &[u8],
            out: &mut [u8],
            $($idle: u8,)?
        ) -> usize {
            out[0] = 0x05;
            out[1] = 0;
            out[2] = 0;
            let mut at = 3;
            let mut pos = 0;
            let mut done = 0;
            'items: while done < count {
                out[1] = (done + 1) as u8;
                let treq = items[pos];
                pos += 1;
                let ap = treq & 1 != 0;
                let addr = dp_register(treq >> 2);
                if treq & 2 != 0 {
                    if ap {
                        let n = run_length(treq, count - done, items, pos);
                        pos += n - 1;
                        let post = request_byte(APnDP::AP, RnW::R, addr);
                        match retry!(attempts, read_word!(w, post $(, $idle)?)) {
                            Ok(_) => out[2] = 1,
                            Err(e) => {
                                out[2] = status_byte(e);
                                break 'items;
                            }
                        }
                        for k in 0..n {
                            out[1] = (done + k + 1) as u8;
                            let rq = if k + 1 == n {
                                request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF)
                            } else {
                                post
                            };
                            match retry!(attempts, read_word!(w, rq $(, $idle)?)) {
                                Ok(v) => {
                                    out[2] = 1;
                                    out[at..at + 4].copy_from_slice(&v.to_le_bytes());
                                    at += 4;
                                }
                                Err(e) => {
                                    out[2] = status_byte(e);
                                    break 'items;
                                }
                            }
                        }
                        done += n;
                        continue;
                    }
                    let req = request_byte(APnDP::DP, RnW::R, addr);
                    match retry!(attempts, read_word!(w, req $(, $idle)?)) {
                        Ok(v) => {
                            out[2] = 1;
                            out[at..at + 4].copy_from_slice(&v.to_le_bytes());
                            at += 4;
                        }
                        Err(e) => {
                            out[2] = status_byte(e);
                            break 'items;
                        }
                    }
                } else {
                    let word = [items[pos], items[pos + 1], items[pos + 2], items[pos + 3]];
                    pos += 4;
                    let port = if ap { APnDP::AP } else { APnDP::DP };
                    let req = request_byte(port, RnW::W, addr);
                    match retry!(
                        attempts,
                        write_word!(w, req, u32::from_le_bytes(word) $(, $idle)?)
                    ) {
                        Ok(()) => out[2] = 1,
                        Err(e) => {
                            out[2] = status_byte(e);
                            break 'items;
                        }
                    }
                }
                done += 1;
            }
            at
        }
    };
}

transfer_items_fn!(
    /// A whole `DAP_Transfer`; a run of AP reads posts once, RDBUFF last.
    #[inline(never)]
    #[cfg_attr(target_os = "none", link_section = ".ramcode.spi_block")]
    transfer_items
);

transfer_items_fn!(
    #[inline(never)]
    transfer_items_idle,
    idle
);

/// `CR1` bits the sessions write whole: `SSI` always, `HDDIR` for transmit.
const CR1_SSI: u32 = 1 << 12;
const CR1_HDDIR: u32 = 1 << 11;
const CR1_CSTART: u32 = 1 << 9;
const CR1_SPE: u32 = 1;

/// `SR` bits the sessions poll: `RXP`, `TXP`, `EOT` and `OVR`.
const SR_RXP: u32 = 1;
const SR_TXP: u32 = 1 << 1;
const SR_EOT: u32 = 1 << 3;
const SR_OVR: u32 = 1 << 6;

/// The rung the sessions run at, mirrored from the clock request dap-rs applies.
#[derive(Copy, Clone)]
struct Rung {
    mbr: u8,
    half_spins: u32,
    eot_spins: u32,
}

impl Rung {
    /// `CFG1` for `bits`-wide frames at this rung; every other field is zero.
    #[inline(always)]
    fn cfg1(&self, bits: u8) -> Cfg1 {
        let mut cfg1 = Cfg1(0);
        cfg1.set_mbr(Mbr::from_bits(self.mbr));
        cfg1.set_dsize(bits - 1);
        cfg1
    }
}

/// Half-duplex SPI sessions straight on the registers, one word per call.
struct StmSpiWords<M: SpiSwdBoard> {
    rung: Rung,
    // `CR1` as the last session left it (block off), so an unchanged direction is not rewritten.
    cr1: u32,
    // `TSIZE` as last programmed, rewritten only on change.
    tsize: u16,
    // A header session is enabled and started, clocking as soon as its byte is pushed.
    armed: bool,
    _board: PhantomData<M>,
}

impl<M: SpiSwdBoard> StmSpiWords<M> {
    #[inline(always)]
    fn regs() -> pac::spi::Spi {
        M::spi()
    }

    #[inline(always)]
    fn poll(bit: u32) -> swd::Result<u32> {
        let s = Self::regs();
        for _ in 0..SPI_POLL_LIMIT {
            let sr = s.sr().read().0;
            if sr & bit != 0 {
                return Ok(sr);
            }
        }
        Err(swd::Error::AckProtocol)
    }

    /// Program the geometry and enable, direction first while the block is off.
    #[inline(always)]
    fn begin(&mut self, bits: u8, frames: u16, tx: bool) -> u32 {
        let s = Self::regs();
        s.cfg1().write_value(self.rung.cfg1(bits));
        if frames != self.tsize {
            s.cr2().write(|w| w.set_tsize(frames));
            self.tsize = frames;
        }
        let base = CR1_SSI | if tx { CR1_HDDIR } else { 0 };
        if base != self.cr1 {
            s.cr1().write_value(Cr1(base));
            self.cr1 = base;
        }
        s.cr1().write_value(Cr1(base | CR1_SPE));
        base
    }

    /// One SCK after end of transfer (ES0621 2.17.2), then disable and clear the flags.
    #[inline(always)]
    fn finish(&self, base: u32, r: swd::Result<()>) -> swd::Result<()> {
        let s = Self::regs();
        spin(self.rung.eot_spins);
        s.cr1().write_value(Cr1(base));
        s.ifcr().write(|w| {
            w.set_eotc(true);
            w.set_txtfc(true);
            w.set_ovrc(true);
        });
        r
    }

    /// Program, enable and start a transmit session; it clocks on its first push.
    #[inline(always)]
    fn open_tx(&mut self, bits: u8, frames: u16) -> u32 {
        let base = self.begin(bits, frames, true);
        Self::regs()
            .cr1()
            .write_value(Cr1(base | CR1_SPE | CR1_CSTART));
        base
    }

    /// Open the next word's header session ahead of time.
    #[inline(always)]
    fn arm(&mut self) {
        if !self.armed {
            self.open_tx(8, 1);
            self.armed = true;
        }
    }

    /// The request header, through the armed session when there is one.
    #[inline(always)]
    fn header(&mut self, req: u8) -> swd::Result<()> {
        let base = if core::mem::take(&mut self.armed) {
            CR1_SSI | CR1_HDDIR
        } else {
            self.open_tx(8, 1)
        };
        self.load8(base, &[req])
    }

    /// Push byte frames into a started transmit session and retire it.
    #[inline(always)]
    fn load8(&self, base: u32, bytes: &[u8]) -> swd::Result<()> {
        let s = Self::regs();
        let mut r = Ok(());
        for &b in bytes {
            if let Err(e) = Self::poll(SR_TXP) {
                r = Err(e);
                break;
            }
            s.txdr8().write_value(b);
        }
        self.run(base, r)
    }

    /// A transmit session of frames wider than a byte.
    #[inline(always)]
    fn tx16(&mut self, bits: u8, halves: &[u16]) -> swd::Result<()> {
        let base = self.open_tx(bits, halves.len() as u16);
        let s = Self::regs();
        let mut r = Ok(());
        for &h in halves {
            if let Err(e) = Self::poll(SR_TXP) {
                r = Err(e);
                break;
            }
            s.txdr16().write_value(h);
        }
        self.run(base, r)
    }

    /// Retire a loaded transmit session.
    #[inline(always)]
    fn run(&self, base: u32, mut r: swd::Result<()>) -> swd::Result<()> {
        if r.is_ok() {
            r = Self::poll(SR_EOT).map(|_| ());
        }
        self.finish(base, r)
    }

    #[inline(always)]
    fn rx(&mut self, bits: u8, out: &mut [u32]) -> swd::Result<()> {
        let base = self.begin(bits, out.len() as u16, false);
        let s = Self::regs();
        s.cr1().write_value(Cr1(base | CR1_SPE | CR1_CSTART));
        let polled = Self::poll(SR_EOT);
        let mut r = polled.and_then(|sr| {
            if sr & SR_OVR != 0 {
                Err(swd::Error::AckProtocol)
            } else {
                Ok(())
            }
        });
        if r.is_ok() {
            for slot in out.iter_mut() {
                match Self::poll(SR_RXP) {
                    Ok(_) => *slot = s.rxdr32().read(),
                    Err(e) => {
                        r = Err(e);
                        break;
                    }
                }
            }
        }
        self.finish(base, r)
    }

    /// One released clock on the GPIO pads after a non-OK read ACK, as the engine does.
    fn turnaround(&mut self) {
        let g = M::clk_port();
        g.bsrr().write(|w| w.set_bs(M::CLK_PIN, true));
        g.moder().modify(|w| {
            w.set_moder(M::CLK_PIN, Moder::OUTPUT);
            w.set_moder(M::IO_PIN, Moder::INPUT);
        });
        g.bsrr().write(|w| w.set_br(M::CLK_PIN, true));
        spin(self.rung.half_spins);
        g.bsrr().write(|w| w.set_bs(M::CLK_PIN, true));
        spin(self.rung.half_spins);
        pads_to_spi::<M>();
    }
}

#[inline(always)]
fn pads_to_spi<M: SpiSwdBoard>() {
    M::clk_port().moder().modify(|w| {
        w.set_moder(M::CLK_PIN, Moder::ALTERNATE);
        w.set_moder(M::IO_PIN, Moder::ALTERNATE);
    });
}

impl<M: SpiSwdBoard> WordOps for StmSpiWords<M> {
    #[inline(always)]
    fn read(&mut self, req: u8) -> swd::Result<u32> {
        self.header(req)?;
        let mut ack = [0u32; 1];
        self.rx(TRN + 3, &mut ack)?;
        if let Err(e) = swd::Ack::try_ok(ack_from_frame(ack[0], TRN)) {
            self.turnaround();
            return Err(e);
        }
        let mut frames = [0u32; 2];
        self.rx(17, &mut frames)?;
        let (data, parity) = combine_read_data(frames[0], frames[1]);
        if parity == parity32(data) {
            self.arm();
            Ok(data)
        } else {
            Err(swd::Error::BadParity)
        }
    }

    #[inline(always)]
    fn write(&mut self, req: u8, data: u32) -> swd::Result<()> {
        // Exact framing: 33 clocks as three 11-bit frames, not five padded bytes.
        let frames = wdata_frames(data);
        self.header(req)?;
        let mut ack = [0u32; 1];
        self.rx(TRN + 3 + TRN, &mut ack)?;
        swd::Ack::try_ok(ack_from_frame(ack[0], TRN))?;
        self.tx16(11, &frames)?;
        self.arm();
        Ok(())
    }

    /// Idle cells on the GPIO pads with SWDIO low, closing an armed header session first.
    fn idle(&mut self, cycles: u8) {
        self.disarm();
        let g = M::clk_port();
        g.bsrr().write(|w| {
            w.set_bs(M::CLK_PIN, true);
            w.set_br(M::IO_PIN, true);
        });
        g.moder().modify(|w| {
            w.set_moder(M::CLK_PIN, Moder::OUTPUT);
            w.set_moder(M::IO_PIN, Moder::OUTPUT);
        });
        for _ in 0..cycles {
            g.bsrr().write(|w| w.set_br(M::CLK_PIN, true));
            spin(self.rung.half_spins);
            g.bsrr().write(|w| w.set_bs(M::CLK_PIN, true));
            spin(self.rung.half_spins);
        }
        pads_to_spi::<M>();
    }

    /// Close an armed header session that no word will use; nothing was clocked.
    fn disarm(&mut self) {
        if core::mem::take(&mut self.armed) {
            let _ = self.finish(CR1_SSI | CR1_HDDIR, Ok(()));
        }
    }
}

/// The SPI block path, taking transfers and blocks while dap-rs holds the SWD driver.
pub struct SpiBlock<M: SpiSwdBoard, const K: usize> {
    cpu_hz: u32,
    ladder: SpiLadder<M::Source, K>,
    wait_attempts: usize,
    idle_cycles: u8,
    dap: DapMode,
    rung: Option<Rung>,
    _board: PhantomData<M>,
}

impl<M: SpiSwdBoard, const K: usize> SpiBlock<M, K> {
    /// Idle until the host sets a clock: before that the rung dap-rs runs is not mirrored.
    pub fn new(cpu_hz: u32, ladder: SpiLadder<M::Source, K>) -> Self {
        Self {
            cpu_hz,
            ladder,
            wait_attempts: 6,
            idle_cycles: 0,
            dap: DapMode::new(),
            rung: None,
            _board: PhantomData,
        }
    }

    /// The rung dap-rs's engine lands on for `max_frequency`; none below the slowest, as there.
    fn set_clock(&mut self, max_frequency: u32) {
        self.rung = self.ladder.choose(max_frequency).map(|c| Rung {
            mbr: c.pre.mbr_bits(),
            half_spins: spin_iterations(half_period_cycles(self.cpu_hz, c.sck_hz)),
            eot_spins: spin_iterations(c.pre.sck_cycles(self.cpu_hz, c.kernel_hz)),
        });
    }

    /// Handle a report ahead of the engine; `None` leaves it to dap-rs unchanged.
    pub fn process(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        self.process_on(report, response, Self::words)
    }

    /// Follow dap-rs's mode through a report it answered.
    pub fn observe(&mut self, report: &[u8], response: &[u8]) {
        self.dap.observe(report, response);
    }

    /// dap-rs parked the pads; it re-enters SWD before the block runs again.
    pub fn suspend(&mut self) {
        self.dap.suspend();
    }

    /// [`process`](Self::process) over the words `words` opens at the mirrored rung.
    fn process_on<W: WordOps>(
        &mut self,
        report: &[u8],
        response: &mut [u8],
        words: impl FnOnce(Rung) -> W,
    ) -> Option<usize> {
        match *report.first()? {
            0x04 => {
                if let Some(attempts) = transfer_wait_attempts(report) {
                    self.wait_attempts = attempts;
                }
                if let Some(idle) = transfer_idle_cycles(report) {
                    self.idle_cycles = idle;
                }
                None
            }
            0x11 => {
                if report.len() >= 5 {
                    let hz = u32::from_le_bytes([report[1], report[2], report[3], report[4]]);
                    if hz != 0 {
                        self.set_clock(hz);
                    }
                }
                None
            }
            0x05 if self.dap.swd_held() => self.transfer(report, response, words),
            0x06 if self.dap.swd_held() => self.transfer_block(report, response, words),
            _ => None,
        }
    }

    fn words(rung: Rung) -> StmSpiWords<M> {
        pads_to_spi::<M>();
        StmSpiWords {
            rung,
            // Unknown after dap-rs's sessions: the first session always writes the direction.
            cr1: u32::MAX,
            tsize: 0,
            armed: false,
            _board: PhantomData,
        }
    }

    fn transfer<W: WordOps>(
        &self,
        report: &[u8],
        response: &mut [u8],
        words: impl FnOnce(Rung) -> W,
    ) -> Option<usize> {
        let count = usize::from(*report.get(2)?);
        let items = report.get(3..)?;
        if count == 0 {
            return None;
        }
        let reads = plain_transfers(count, items)?;
        if response.len() < 3 + 4 * reads {
            return None;
        }
        let mut w = words(self.rung?);
        let attempts = self.wait_attempts;
        let len = match self.idle_cycles {
            0 => transfer_items(&mut w, count, attempts, items, response),
            idle => transfer_items_idle(&mut w, count, attempts, items, response, idle),
        };
        w.disarm();
        Some(len)
    }

    fn transfer_block<W: WordOps>(
        &self,
        report: &[u8],
        response: &mut [u8],
        words: impl FnOnce(Rung) -> W,
    ) -> Option<usize> {
        let BlockRequest {
            count,
            request: treq,
        } = BlockRequest::parse(report, response.len())?;
        let ap = treq & 1 != 0;
        let port = if ap { APnDP::AP } else { APnDP::DP };
        let addr = DPRegister::try_from((treq >> 2) & 3).ok()?;
        let mut w = words(self.rung?);
        let end = if treq & 2 != 0 {
            let req = request_byte(port, RnW::R, addr);
            let last = if ap {
                request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF)
            } else {
                req
            };
            let attempts = self.wait_attempts;
            match self.idle_cycles {
                0 => read_block(&mut w, req, last, ap, count, attempts, response),
                idle => read_block_idle(&mut w, req, last, ap, count, attempts, response, idle),
            }
        } else {
            let req = request_byte(port, RnW::W, addr);
            let data = &report[5..5 + 4 * count];
            match self.idle_cycles {
                0 => write_block(&mut w, req, self.wait_attempts, data),
                idle => write_block_idle(&mut w, req, self.wait_attempts, data, idle),
            }
        };
        w.disarm();
        Some(encode_block_response(
            response, end.ran, end.status, end.words,
        ))
    }
}

#[cfg(test)]
#[path = "spi_block_differential_tests.rs"]
mod differential_tests;

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use self::std::vec;
    use self::std::vec::Vec;

    /// Scripted words: each call pops the next outcome and logs the request.
    struct Script {
        reads: Vec<swd::Result<u32>>,
        writes: Vec<swd::Result<()>>,
        log: Vec<(u8, Option<u32>)>,
        idles: Vec<u8>,
    }

    impl WordOps for Script {
        fn read(&mut self, req: u8) -> swd::Result<u32> {
            self.log.push((req, None));
            self.reads.remove(0)
        }
        fn write(&mut self, req: u8, data: u32) -> swd::Result<()> {
            self.log.push((req, Some(data)));
            self.writes.remove(0)
        }
        fn idle(&mut self, cycles: u8) {
            self.idles.push(cycles);
        }
    }

    fn script(reads: Vec<swd::Result<u32>>, writes: Vec<swd::Result<()>>) -> Script {
        Script {
            reads,
            writes,
            log: Vec::new(),
            idles: Vec::new(),
        }
    }

    const AP_RD0: u8 = 0x87; // AP read, A = 0
    const RDBUFF: u8 = 0xbd; // DP read, A = 0xC

    #[test]
    fn request_constants_match_dap_rs() {
        assert_eq!(request_byte(APnDP::AP, RnW::R, DPRegister::DPIDR), AP_RD0);
        assert_eq!(request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF), RDBUFF);
    }

    #[test]
    fn a_posted_ap_read_block_takes_rdbuff_last() {
        let mut w = script(vec![Ok(0), Ok(10), Ok(11), Ok(12)], vec![]);
        let mut out = [0u8; 16];
        let end = read_block(&mut w, AP_RD0, RDBUFF, true, 3, 5, &mut out);
        assert_eq!((end.ran, end.status, end.words), (3, 1, 3));
        assert_eq!(&out[4..8], &10u32.to_le_bytes());
        assert_eq!(&out[12..16], &12u32.to_le_bytes());
        let reqs: Vec<u8> = w.log.iter().map(|l| l.0).collect();
        assert_eq!(reqs, vec![AP_RD0, AP_RD0, AP_RD0, RDBUFF]);
    }

    #[test]
    fn a_wait_is_retried_then_exhausts_the_budget() {
        let mut w = script(vec![Err(swd::Error::AckWait), Ok(7)], vec![]);
        let mut out = [0u8; 8];
        let end = read_block(&mut w, RDBUFF, RDBUFF, false, 1, 2, &mut out);
        assert_eq!((end.ran, end.status, end.words), (1, 1, 1));
        let mut w = script(vec![Err(swd::Error::AckWait); 3], vec![]);
        let end = read_block(&mut w, RDBUFF, RDBUFF, false, 1, 3, &mut out);
        assert_eq!((end.ran, end.status, end.words), (1, 2, 0));
        assert_eq!(w.log.len(), 3);
    }

    #[test]
    fn a_fault_mid_block_keeps_the_words_already_read() {
        let mut w = script(vec![Ok(0), Ok(1), Err(swd::Error::AckFault)], vec![]);
        let mut out = [0u8; 16];
        let end = read_block(&mut w, AP_RD0, RDBUFF, true, 3, 5, &mut out);
        assert_eq!((end.ran, end.status, end.words), (2, 4, 1));
    }

    #[test]
    fn a_failed_write_names_the_transfer_it_stopped_on() {
        let mut w = script(vec![], vec![Ok(()), Err(swd::Error::AckFault)]);
        let data = [1u8, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0];
        let end = write_block(&mut w, 0xa3, 5, &data);
        assert_eq!((end.ran, end.status, end.words), (2, 4, 0));
    }

    #[test]
    fn a_transfer_run_of_ap_reads_posts_once() {
        let mut w = script(vec![Ok(0), Ok(21), Ok(22)], vec![Ok(())]);
        // DP write SELECT, then two AP reads of A = 0.
        let items = [0x08u8, 0, 0, 0, 0, 0x03, 0x03];
        let mut out = [0u8; 11];
        let len = transfer_items(&mut w, 3, 5, &items, &mut out);
        assert_eq!(len, 11);
        assert_eq!(&out[..3], &[5, 3, 1]);
        assert_eq!(&out[3..7], &21u32.to_le_bytes());
        assert_eq!(&out[7..11], &22u32.to_le_bytes());
        let reqs: Vec<u8> = w.log.iter().map(|l| l.0).collect();
        assert_eq!(reqs[1..], [AP_RD0, AP_RD0, RDBUFF]);
    }

    #[test]
    fn flagged_items_stay_with_dap_rs() {
        assert_eq!(plain_transfers(1, &[0x13]), None);
        assert_eq!(plain_transfers(2, &[0x02, 0x00, 1, 2, 3, 4]), Some(1));
    }

    use core::sync::atomic::{AtomicUsize, Ordering};
    use daplink_rs::dap_rs::jtag::{Jtag, TapConfig};
    use daplink_rs::dap_rs::swd::Swd;
    use daplink_rs::{DynCycleDelay, InputOutputPin};
    use daplink_spi_swd::{
        Dir, JtagPortOps, NoFastJtag, PortResult, SpiJtag, SpiPortOps, SpiSwd, SpiSwdDeps,
    };

    /// The kernel index the test board's mux last selected.
    static MUX: AtomicUsize = AtomicUsize::new(0);

    struct Board;
    impl SpiSwdBoard for Board {
        const CLK_PIN: usize = 10;
        const IO_PIN: usize = 15;
        const TDI_PIN: usize = 7;
        const TDO_PIN: usize = 6;
        const SPI_AF: u8 = 5;
        type Source = usize;
        fn clk_port() -> pac::gpio::Gpio {
            pac::GPIOB
        }
        fn jtag_port() -> pac::gpio::Gpio {
            pac::GPIOA
        }
        fn spi() -> pac::spi::Spi {
            pac::SPI2
        }
        fn init_clocks() {}
        fn route(index: usize) {
            MUX.store(index, Ordering::Relaxed);
        }
    }

    /// A port that only switches kernels, the way the STM32 port does.
    struct Port;
    impl JtagPortOps for Port {
        fn jtag_pads_acquire(&mut self) {}
        fn jtag_pads_release(&mut self) {}
        fn tck_low(&self) {}
        fn tck_high(&self) {}
        fn tms_set(&self, _: bool) {}
        fn tdi_set(&self, _: bool) {}
        fn tdo_get(&self) -> u32 {
            0
        }
        fn tdi_is_high(&mut self) -> bool {
            false
        }
        fn tdo_is_high(&mut self) -> bool {
            false
        }
    }
    impl SpiPortOps for Port {
        const DUPLEX: bool = false;
        type Source = usize;
        fn route(&mut self, index: usize) {
            Board::route(index)
        }
        fn configure(&mut self) {}
        fn program(&self, _: u8, _: u8, _: u16, _: Dir) {}
        fn enable(&self) {}
        fn disable(&self) {}
        fn start(&self) {}
        fn push8(&self, _: u8) -> PortResult<()> {
            Ok(())
        }
        fn push16(&self, _: u16) -> PortResult<()> {
            Ok(())
        }
        fn wait_eot(&self) -> PortResult<()> {
            Ok(())
        }
        fn pop32(&self) -> PortResult<u32> {
            Ok(0)
        }
        fn clear_flags(&self) -> PortResult<()> {
            Ok(())
        }
        fn pads_to_spi(&mut self) {}
        fn clk_pad_to_spi(&mut self) {}
        fn drive(&mut self) {}
        fn release(&mut self) {}
    }

    struct NoPin;
    impl InputOutputPin for NoPin {
        fn set_as_output(&mut self) {}
        fn set_high(&mut self, _: bool) {}
        fn set_as_input(&mut self) {}
        fn is_high(&mut self) -> bool {
            false
        }
    }

    type Deps = SpiSwdDeps<Port, NoPin, DynCycleDelay, NoFastJtag, 3>;

    #[test]
    fn a_jtag_clock_keeps_the_block_rung_on_the_selected_kernel() {
        const CPU: u32 = 250_000_000;
        const KERNELS: [u32; 3] = [64_000_000, 100_000_000, 80_000_000];
        const LADDER: SpiLadder<usize, 3> = SpiLadder {
            kernels: [(0, KERNELS[0]), (1, KERNELS[1]), (2, KERNELS[2])],
            max_sck_hz: 50_000_000,
        };
        let chain = std::boxed::Box::leak(vec![TapConfig::INIT; 1].into_boxed_slice());
        let deps = Deps::new(
            Port,
            NoPin,
            NoPin,
            NoPin,
            DynCycleDelay::new(CPU),
            NoFastJtag,
            LADDER,
            chain,
            std::boxed::Box::leak(std::boxed::Box::new(daplink_spi_swd::KeptAck::new(
                std::boxed::Box::leak(std::boxed::Box::default()),
            ))),
        );
        let mut block = SpiBlock::<Board, 3>::new(CPU, LADDER);
        let mut clock = |hz: u32| {
            let mut report = [0x11, 0, 0, 0, 0];
            report[1..].copy_from_slice(&hz.to_le_bytes());
            assert_eq!(block.process(&report, &mut [0; 8]), None);
        };

        let mut swd: SpiSwd<_, _, _, _, 3> = deps.into();
        assert!(Swd::set_clock(&mut swd, 25_000_000));
        clock(25_000_000);
        let mut jtag: SpiJtag<_, _, _, _, 3> = Deps::from(swd).into();
        assert!(Jtag::set_clock(&mut jtag, 32_000_000));
        clock(32_000_000);
        let _swd: SpiSwd<_, _, _, _, 3> = Deps::from(jtag).into();

        let sck = KERNELS[MUX.load(Ordering::Relaxed)] >> (block.rung.unwrap().mbr + 1);
        assert!(sck <= 32_000_000, "{sck} Hz");
    }

    #[test]
    fn configured_wait_retries_allow_one_attempt_more() {
        let ladder = SpiLadder {
            kernels: [(0, 64_000_000); 3],
            max_sck_hz: 50_000_000,
        };
        let mut block = SpiBlock::<Board, 3>::new(250_000_000, ladder);
        assert_eq!(block.wait_attempts, 6);
        for (retries, attempts) in [(0x1234u16, 0x1235), (65535, 65536), (0, 1)] {
            let [lo, hi] = retries.to_le_bytes();
            assert_eq!(block.process(&[4, 3, lo, hi, 0, 0], &mut [0; 8]), None);
            assert_eq!((block.wait_attempts, block.idle_cycles), (attempts, 3));
        }
        let mut w = script(vec![], vec![Err(swd::Error::AckWait)]);
        let end = write_block(&mut w, 0xa3, block.wait_attempts, &[1, 0, 0, 0]);
        assert_eq!((end.ran, end.status, end.words), (1, 2, 0));
        assert_eq!(w.log.len(), 1);
    }
}
