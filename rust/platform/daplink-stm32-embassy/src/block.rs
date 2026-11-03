//! `DAP_TransferBlock` fast path for the bit-bang SWD image, with dap-rs's framing and status semantics.

use core::marker::PhantomData;

use daplink_core::dap_wire::{
    encode_block_response, transfer_idle_cycles, transfer_wait_attempts, BlockRequest,
};

use daplink_rs::dap_rs::swd::{self, APnDP, DPRegister, RnW};
use daplink_spi_swd::pace::{ClassFit, Fixed, Floors};
use daplink_spi_swd::session::{parity32, request_byte};

use crate::block_common::{dp_register, plain_transfers, retry, run_length, status_byte, BlockEnd};
use crate::dap_mode::DapMode;
use crate::pace::{halves, in_register, pinned, Spin};
use crate::swj::SwjPads;

/// Firmware-supplied pacing constants; zero fixed work never runs a cell fast.
pub trait BlockPacing: Copy {
    /// The pads the fits were measured on.
    type Pads: SwjPads;
    const CONTROL: ClassFit = ClassFit {
        fixed: NO_WORK,
        floors: DRIVEN,
    };
    const ACK: ClassFit = ClassFit {
        fixed: NO_WORK,
        floors: SAMPLED,
    };
    const READ: ClassFit = ClassFit {
        fixed: NO_WORK,
        floors: SAMPLED,
    };
    const WRITE: ClassFit = ClassFit {
        fixed: NO_WORK,
        floors: DRIVEN,
    };
    /// Highest frequency the wire is driven at whatever the host asks for; zero uncaps it.
    const MAX_WIRE_HZ: u32 = 12_000_000;
}

const NO_WORK: Fixed = Fixed {
    low: 0,
    high: 0,
    period: 0,
};

/// Shortest cell the probe drives.
const DRIVEN: Floors = Floors {
    low_ps: 32_000,
    high_ps: 32_000,
    period_ps: 0,
};

/// Shortest cell whose low half ends in an `IDR` sample.
const SAMPLED: Floors = Floors {
    low_ps: 44_000,
    high_ps: 32_000,
    period_ps: 0,
};

/// The two SWD pads as the block drives them; the STM32 implementation is register stores.
pub trait Wire {
    fn clk(&mut self, high: bool);
    fn swdio(&mut self, bit: u32);
    /// Drive SWDIO and take SWCLK low together; one store where the pads share a port.
    fn data_clk_low(&mut self, bit: u32) {
        self.swdio(bit);
        self.clk(false);
    }
    /// The word [`clk_low_store`](Self::clk_low_store) takes to drive `bit` with SWCLK low.
    fn clk_low_word(bit: u32) -> u32 {
        bit & 1
    }
    fn clk_low_store(&mut self, word: u32) {
        self.data_clk_low(word);
    }
    fn sample(&mut self) -> u32;
    /// Sample SWDIO already at bit 31.
    fn sample_msb(&mut self) -> u32 {
        self.sample() << 31
    }
    /// Sample the pad's raw input word.
    fn sample_raw(&mut self) -> u32 {
        self.sample()
    }
    /// SWDIO at bit 31 from a [`sample_raw`](Self::sample_raw) word.
    fn raw_msb(raw: u32) -> u32 {
        raw << 31
    }
    /// Drive or release SWDIO; called only when the direction actually changes.
    fn set_out(&mut self, out: bool);
    /// The SWDIO direction found on entry, restored when the block ends.
    fn entry_out(&mut self) -> bool;
}

// Constant register addresses, so no cell reloads a pointer the adjacent store may alias.
#[inline(always)]
fn bsrr<M: SwjPads>() -> *mut u32 {
    M::swj_port().bsrr().as_ptr() as *mut u32
}

#[inline(always)]
fn idr<M: SwjPads>() -> *const u32 {
    M::swj_port().idr().as_ptr() as *const u32
}

#[inline(always)]
fn moder<M: SwjPads>() -> *mut u32 {
    M::swj_port().moder().as_ptr() as *mut u32
}

/// The SWDIO `MODER` words, resolved once per block.
pub struct H5Port<M: SwjPads> {
    moder_in: u32,
    moder_out: u32,
    entry_out: bool,
    pads: PhantomData<M>,
}

impl<M: SwjPads> Clone for H5Port<M> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<M: SwjPads> Copy for H5Port<M> {}

#[allow(clippy::new_without_default)] // Constructing one reads a live register.
impl<M: SwjPads> H5Port<M> {
    /// `BSRR` set bits; the reset bit for the same pad is sixteen places up.
    const SWCLK_SET: u32 = 1 << M::SWCLK;
    const SWDIO_SET: u32 = 1 << M::SWDIO;
    const SWDIO_MODE_MASK: u32 = 0b11 << (2 * M::SWDIO);
    const SWDIO_MODE_OUT: u32 = 0b01 << (2 * M::SWDIO);

    /// Snapshot SWDIO's mode; call once, at the head of a block.
    pub fn new() -> Self {
        // SAFETY: a resolved GPIO register; a plain read.
        let entry = unsafe { moder::<M>().read_volatile() };
        Self {
            moder_in: entry & !Self::SWDIO_MODE_MASK,
            moder_out: (entry & !Self::SWDIO_MODE_MASK) | Self::SWDIO_MODE_OUT,
            entry_out: entry & Self::SWDIO_MODE_MASK == Self::SWDIO_MODE_OUT,
            pads: PhantomData,
        }
    }
}

impl<M: SwjPads> Wire for H5Port<M> {
    #[inline(always)]
    fn clk(&mut self, high: bool) {
        let v = if high {
            Self::SWCLK_SET
        } else {
            Self::SWCLK_SET << 16
        };
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe { bsrr::<M>().write_volatile(v) };
    }

    #[inline(always)]
    fn swdio(&mut self, bit: u32) {
        // Branchless: bit 0 shifts the mask into the reset half of `BSRR`.
        let v = Self::SWDIO_SET << (((bit ^ 1) & 1) << 4);
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe { bsrr::<M>().write_volatile(v) };
    }

    #[inline(always)]
    fn data_clk_low(&mut self, bit: u32) {
        // SWCLK and SWDIO share a port, so the cell's low edge is one store.
        let v = (Self::SWDIO_SET << (((bit ^ 1) & 1) << 4)) | (Self::SWCLK_SET << 16);
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe { bsrr::<M>().write_volatile(v) };
    }

    #[inline(always)]
    fn clk_low_word(bit: u32) -> u32 {
        (Self::SWDIO_SET << (((bit ^ 1) & 1) << 4)) | (Self::SWCLK_SET << 16)
    }

    #[inline(always)]
    fn clk_low_store(&mut self, word: u32) {
        // SAFETY: a resolved GPIO register; `BSRR` is write-1-to-act.
        unsafe { bsrr::<M>().write_volatile(word) };
    }

    #[inline(always)]
    fn sample(&mut self) -> u32 {
        // SAFETY: a resolved GPIO register; `IDR` is read-only.
        (unsafe { idr::<M>().read_volatile() } >> M::SWDIO) & 1
    }

    #[inline(always)]
    fn sample_raw(&mut self) -> u32 {
        // SAFETY: as in `sample`.
        unsafe { idr::<M>().read_volatile() }
    }

    #[inline(always)]
    fn raw_msb(raw: u32) -> u32 {
        (raw >> M::SWDIO) << 31
    }

    #[inline(always)]
    fn sample_msb(&mut self) -> u32 {
        // The final shift discards everything above the pad, so no mask is needed.
        // SAFETY: as in `sample`.
        (unsafe { idr::<M>().read_volatile() } >> M::SWDIO) << 31
    }

    #[inline(always)]
    fn set_out(&mut self, out: bool) {
        let v = if out { self.moder_out } else { self.moder_in };
        // SAFETY: a resolved GPIO register; only SWDIO's mode bits differ from what was read.
        unsafe { moder::<M>().write_volatile(v) };
    }

    #[inline(always)]
    fn entry_out(&mut self) -> bool {
        self.entry_out
    }
}

/// A wire plus the SWDIO direction, so a redundant `MODER` write never reaches the pad.
struct Bus<W: Wire> {
    w: W,
    out: bool,
    entry: bool,
}

impl<W: Wire> Bus<W> {
    fn new(mut w: W) -> Self {
        let entry = w.entry_out();
        Self {
            w,
            out: entry,
            entry,
        }
    }

    #[inline(always)]
    fn dir(&mut self, out: bool) {
        if self.out != out {
            self.w.set_out(out);
            self.out = out;
        }
    }

    /// Restore SWDIO's input/output direction, not its exact original mode.
    fn restore(&mut self) {
        let entry = self.entry;
        self.dir(entry);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Halves {
    ctl_low: Spin,
    ctl_high: Spin,
    ack_low: Spin,
    ack_high: Spin,
    data_low: Spin,
    data_high: Spin,
}

/// One incoming ACK bit, sampled after the low half.
#[inline(always)]
fn ack_in<W: Wire>(b: &mut Bus<W>, h: &Halves) -> u32 {
    b.w.clk(false);
    h.ack_low.run();
    let bit = b.w.sample();
    b.w.clk(true);
    h.ack_high.run();
    bit
}

/// One clock with no data movement: the turnaround cells.
#[inline(always)]
fn clock_only<W: Wire>(b: &mut Bus<W>, h: &Halves) {
    b.w.clk(false);
    h.ack_low.run();
    b.w.clk(true);
    h.ack_high.run();
}

/// One incoming data bit, already at bit 31 so the accumulator is a shift and an or.
#[inline(always)]
fn data_in<W: Wire>(b: &mut Bus<W>, h: &Halves) -> u32 {
    b.w.clk(false);
    h.data_low.run();
    // Extracted in the low half and folded after the high spin, the same in every data cell.
    let bit = pinned(W::raw_msb(b.w.sample_raw()));
    b.w.clk(true);
    h.data_high.run();
    pinned(bit)
}

/// The parity cell of a read: one incoming bit on the data timing, as 0 or 1.
#[inline(always)]
fn parity_in<W: Wire>(b: &mut Bus<W>, h: &Halves) -> u32 {
    data_in(b, h) >> 31
}

#[inline(always)]
fn data_out<W: Wire>(b: &mut Bus<W>, h: &Halves, bit: u32) {
    b.w.data_clk_low(bit);
    h.data_low.run();
    b.w.clk(true);
    h.data_high.run();
}

/// Thirty-two identical cells, so the data phase carries no loop branch.
macro_rules! repeat32 {
    ($($body:tt)*) => {
        $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)*
        $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)*
        $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)*
        $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)* $($body)*
    };
}

/// A request's eight cells as the words that drive them, prepared once per block.
type Request = [u32; 8];

/// The drive words of `req`, LSB first.
#[inline(always)]
fn request<W: Wire>(req: u8) -> Request {
    core::array::from_fn(|i| W::clk_low_word(u32::from(req) >> i))
}

#[inline(always)]
fn request_out<W: Wire>(b: &mut Bus<W>, h: &Halves, req: &Request) {
    for &w in req {
        b.w.clk_low_store(w);
        h.ctl_low.run();
        b.w.clk(true);
        h.ctl_high.run();
    }
}

#[inline(always)]
fn read_word<W: Wire>(b: &mut Bus<W>, h: &Halves, req: u8) -> swd::Result<u32> {
    read_word_with(b, h, &request::<W>(req))
}

/// One SWD read word, cell for cell as the bit-bang engine drives it.
#[inline(always)]
fn read_word_with<W: Wire>(b: &mut Bus<W>, h: &Halves, req: &Request) -> swd::Result<u32> {
    b.dir(true);
    request_out(b, h, req);
    b.dir(false);
    clock_only(b, h);

    let ack = ack_in(b, h) | ack_in(b, h) << 1 | ack_in(b, h) << 2;
    if let Err(e) = swd::Ack::try_ok(ack as u8) {
        clock_only(b, h);
        return Err(e);
    }

    // Opaque, so the first data cell folds like the other 31.
    let mut data = pinned(0);
    // Folded in the cell, not deferred to one stall after the parity cell.
    repeat32!(data = in_register((data >> 1) | data_in(b, h)););

    let parity = parity_in(b, h);
    if parity != parity32(data) as u32 {
        return Err(bad_parity(b, h));
    }
    trailing_turnaround(b, h);
    Ok(data)
}

/// A read's trailing turnaround samples nothing, so it takes the driven pacing.
#[inline(always)]
fn trailing_turnaround<W: Wire>(b: &mut Bus<W>, h: &Halves) {
    b.w.clk(false);
    h.ctl_low.run();
    b.w.clk(true);
    h.ctl_high.run();
}

/// The turnaround a bad parity still owes, kept off the paced path.
#[cold]
#[inline(never)]
fn bad_parity<W: Wire>(b: &mut Bus<W>, h: &Halves) -> swd::Error {
    trailing_turnaround(b, h);
    swd::Error::BadParity
}

#[inline(always)]
fn write_word<W: Wire>(b: &mut Bus<W>, h: &Halves, req: u8, data: u32) -> swd::Result<()> {
    write_word_with(b, h, &request::<W>(req), data)
}

/// One SWD write word; a no-ACK is `NoAck` (0x07), as on a read.
#[inline(always)]
fn write_word_with<W: Wire>(
    b: &mut Bus<W>,
    h: &Halves,
    req: &Request,
    data: u32,
) -> swd::Result<()> {
    // A software popcount, pinned ahead of the request so it never lands in a paced half.
    let parity = pinned(parity32(data) as u32);
    b.dir(true);
    request_out(b, h, req);
    b.dir(false);
    clock_only(b, h);

    let ack = ack_in(b, h) | ack_in(b, h) << 1 | ack_in(b, h) << 2;
    clock_only(b, h);
    match ack {
        0b001 => {}
        0b010 => return Err(swd::Error::AckWait),
        0b100 => return Err(swd::Error::AckFault),
        0b111 => return Err(swd::Error::NoAck),
        _ => return Err(swd::Error::AckUnknown(ack as u8)),
    }

    b.dir(true);
    let mut v = data;
    repeat32!(
        data_out(b, h, v & 1);
        v >>= 1;
    );
    data_out(b, h, parity);
    Ok(())
}

/// A read word, then the host's idle cycles after an OK ACK, a bad parity included.
#[inline(never)]
#[cfg_attr(target_os = "none", link_section = ".ramcode.swd_block")]
fn read_word_idle<W: Wire>(
    b: &mut Bus<W>,
    h: &Halves,
    req: &Request,
    idle: u8,
) -> swd::Result<u32> {
    let res = read_word_with(b, h, req);
    if matches!(res, Ok(_) | Err(swd::Error::BadParity)) {
        idle_cells(b, h, idle);
    }
    res
}

/// A write word, then the host's idle cycles after an OK ACK.
#[inline(never)]
#[cfg_attr(target_os = "none", link_section = ".ramcode.swd_block")]
fn write_word_idle<W: Wire>(
    b: &mut Bus<W>,
    h: &Halves,
    req: &Request,
    data: u32,
    idle: u8,
) -> swd::Result<()> {
    let res = write_word_with(b, h, req, data);
    if res.is_ok() {
        idle_cells(b, h, idle);
    }
    res
}

/// Idle cells with SWDIO driven low, at the control pacing.
#[inline(always)]
fn idle_cells<W: Wire>(b: &mut Bus<W>, h: &Halves, cycles: u8) {
    b.dir(true);
    for _ in 0..cycles {
        b.w.data_clk_low(0);
        h.ctl_low.run();
        b.w.clk(true);
        h.ctl_high.run();
    }
}

#[inline(always)]
fn read_idle<W: Wire>(b: &mut Bus<W>, h: &Halves, req: u8, idle: u8) -> swd::Result<u32> {
    read_word_idle(b, h, &request::<W>(req), idle)
}

#[inline(always)]
fn write_idle<W: Wire>(
    b: &mut Bus<W>,
    h: &Halves,
    req: u8,
    data: u32,
    idle: u8,
) -> swd::Result<()> {
    write_word_idle(b, h, &request::<W>(req), data, idle)
}

/// The block loops, each defined twice: resident, and in flash over resident idle-cycle words.
macro_rules! read_block_fn {
    ($(#[$m:meta])* $name:ident, $read:ident $(, $idle:ident)?) => {
        $(#[$m])*
        fn $name<W: Wire>(
            b: &mut Bus<W>,
            h: &Halves,
            req: u8,
            last: u8,
            posted: bool,
            count: usize,
            attempts: usize,
            out: &mut [u8],
            $($idle: u8,)?
        ) -> BlockEnd {
            let (req, last) = (request::<W>(req), request::<W>(last));
            if posted {
                if let Err(e) = retry!(attempts, $read(b, h, &req $(, $idle)?)) {
                    return BlockEnd {
                        ran: 1,
                        status: status_byte(e),
                        words: 0,
                    };
                }
            }
            let words = &mut out[4..4 + 4 * count];
            // The last word is its own call site, so every loop cell drives from one fixed request.
            let tail = words.len().saturating_sub(4);
            for (done, chunk) in words[..tail].chunks_exact_mut(4).enumerate() {
                match retry!(attempts, $read(b, h, &req $(, $idle)?)) {
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
            if count > 0 {
                match retry!(attempts, $read(b, h, &last $(, $idle)?)) {
                    Ok(v) => words[tail..].copy_from_slice(&v.to_le_bytes()),
                    Err(e) => {
                        return BlockEnd {
                            ran: count as u16,
                            status: status_byte(e),
                            words: count - 1,
                        }
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
    #[cfg_attr(target_os = "none", link_section = ".ramcode.swd_block")]
    #[allow(clippy::too_many_arguments)] // Keep the resident entry's transfer state explicit.
    read_block,
    read_word_with
);

read_block_fn!(
    #[inline(never)]
    #[allow(clippy::too_many_arguments)]
    read_block_idle,
    read_word_idle,
    idle
);

macro_rules! write_block_fn {
    ($(#[$m:meta])* $name:ident, $write:ident $(, $idle:ident)?) => {
        $(#[$m])*
        fn $name<W: Wire>(
            b: &mut Bus<W>,
            h: &Halves,
            req: u8,
            attempts: usize,
            data: &[u8],
            $($idle: u8,)?
        ) -> BlockEnd {
            let req = request::<W>(req);
            for (done, chunk) in data.chunks_exact(4).enumerate() {
                let v = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                if let Err(e) = retry!(attempts, $write(b, h, &req, v $(, $idle)?)) {
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
    #[cfg_attr(target_os = "none", link_section = ".ramcode.swd_block")]
    write_block,
    write_word_with
);

write_block_fn!(
    #[inline(never)]
    write_block_idle,
    write_word_idle,
    idle
);

macro_rules! transfer_items_fn {
    ($(#[$m:meta])* $name:ident, $read:ident, $write:ident $(, $idle:ident)?) => {
        $(#[$m])*
        fn $name<W: Wire>(
            b: &mut Bus<W>,
            rh: &Halves,
            wh: &Halves,
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
                // dap-rs counts a transfer as executed before it runs, so a failure names itself.
                out[1] = (done + 1) as u8;
                let treq = items[pos];
                pos += 1;
                let ap = treq & 1 != 0;
                let addr = dp_register(treq >> 2);
                if treq & 2 != 0 {
                    if ap {
                        // A run of AP reads posts once, RDBUFF last; a failure names the item whose word was pending.
                        let n = run_length(treq, count - done, items, pos);
                        pos += n - 1;
                        let post = request_byte(APnDP::AP, RnW::R, addr);
                        match retry!(attempts, $read(b, rh, post $(, $idle)?)) {
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
                            match retry!(attempts, $read(b, rh, rq $(, $idle)?)) {
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
                    match retry!(attempts, $read(b, rh, req $(, $idle)?)) {
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
                        $write(b, wh, req, u32::from_le_bytes(word) $(, $idle)?)
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
    /// A whole `DAP_Transfer`, framed as dap-rs frames it.
    #[inline(never)]
    #[cfg_attr(target_os = "none", link_section = ".ramcode.swd_block")]
    transfer_items,
    read_word,
    write_word
);

transfer_items_fn!(
    #[inline(never)]
    #[allow(clippy::too_many_arguments)]
    transfer_items_idle,
    read_idle,
    write_idle,
    idle
);

/// One direction's halves: control and ACK shared, `data` the direction's own class.
fn prepare<P: BlockPacing>(cpu_hz: u32, request: u32, sampled: bool) -> Halves {
    let (ctl_low, ctl_high) = halves(&P::CONTROL, cpu_hz, request);
    let (ack_low, ack_high) = halves(&P::ACK, cpu_hz, request);
    let (data_low, data_high) = if sampled {
        halves(&P::READ, cpu_hz, request)
    } else {
        halves(&P::WRITE, cpu_hz, request)
    };
    Halves {
        ctl_low,
        ctl_high,
        ack_low,
        ack_high,
        data_low,
        data_high,
    }
}

/// `DAP_TransferBlock` ahead of dap-rs, as one straight-line loop while dap-rs holds SWD.
pub struct H5Block<P: BlockPacing> {
    cpu_hz: u32,
    dap: DapMode,
    wait_attempts: usize,
    idle_cycles: u8,
    read: Halves,
    write: Halves,
    pacing: PhantomData<P>,
}

impl<P: BlockPacing> H5Block<P> {
    /// Start at the pad cap and dap-rs's five WAIT retries until the host configures them.
    pub fn new(cpu_hz: u32) -> Self {
        let mut block = Self {
            cpu_hz,
            dap: DapMode::new(),
            wait_attempts: 6,
            idle_cycles: 0,
            read: prepare::<P>(cpu_hz, P::MAX_WIRE_HZ, true),
            write: prepare::<P>(cpu_hz, P::MAX_WIRE_HZ, false),
            pacing: PhantomData,
        };
        block.set_clock(u32::MAX);
        block
    }

    /// Re-prepare the plan for a clock request; `false` only for a zero frequency.
    pub fn set_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        // The pads are not characterized above the cap.
        let capped = if P::MAX_WIRE_HZ == 0 {
            max_frequency
        } else {
            max_frequency.min(P::MAX_WIRE_HZ)
        };
        self.read = prepare::<P>(self.cpu_hz, capped, true);
        self.write = prepare::<P>(self.cpu_hz, capped, false);
        true
    }

    /// Handle a report ahead of the engine; `None` leaves it to dap-rs unchanged.
    pub fn process(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        self.process_on(report, response, H5Port::<P::Pads>::new)
    }

    /// Follow dap-rs's mode through a report it answered.
    pub fn observe(&mut self, report: &[u8], response: &[u8]) {
        self.dap.observe(report, response);
        // Here, not in `process`, so what is answered during a flash session still lands.
        match report.first() {
            Some(0x04) if response == [0x04, 0] => {
                if let Some(attempts) = transfer_wait_attempts(report) {
                    self.wait_attempts = attempts;
                }
                if let Some(idle) = transfer_idle_cycles(report) {
                    self.idle_cycles = idle;
                }
            }
            Some(0x11) if response == [0x11, 0] && report.len() >= 5 => {
                self.set_clock(u32::from_le_bytes([
                    report[1], report[2], report[3], report[4],
                ]));
            }
            _ => {}
        }
    }

    /// dap-rs parked the pads; it re-enters SWD before the block runs again.
    pub fn suspend(&mut self) {
        self.dap.suspend();
    }

    /// [`process`](Self::process) on the wire `wire` resolves.
    fn process_on<W: Wire>(
        &mut self,
        report: &[u8],
        response: &mut [u8],
        wire: impl FnOnce() -> W,
    ) -> Option<usize> {
        match *report.first()? {
            0x05 if self.dap.swd_held() => self.transfer(report, response, wire),
            0x06 if self.dap.swd_held() => self.transfer_block(report, response, wire),
            _ => None,
        }
    }

    /// `DAP_Transfer` for the plain shapes only; a flagged item is left to dap-rs whole.
    fn transfer<W: Wire>(
        &self,
        report: &[u8],
        response: &mut [u8],
        wire: impl FnOnce() -> W,
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
        let (rh, wh) = (self.read, self.write);
        let mut b = Bus::new(wire());
        let attempts = self.wait_attempts;
        let len = match self.idle_cycles {
            0 => transfer_items(&mut b, &rh, &wh, count, attempts, items, response),
            idle => transfer_items_idle(&mut b, &rh, &wh, count, attempts, items, response, idle),
        };
        b.restore();
        Some(len)
    }

    /// The wire is resolved only once the report is known to be handled here.
    fn transfer_block<W: Wire>(
        &self,
        report: &[u8],
        response: &mut [u8],
        wire: impl FnOnce() -> W,
    ) -> Option<usize> {
        let BlockRequest {
            count,
            request: treq,
        } = BlockRequest::parse(report, response.len())?;
        let ap = treq & 1 != 0;
        let read = treq & 2 != 0;
        let port = if ap { APnDP::AP } else { APnDP::DP };
        let addr = DPRegister::try_from((treq >> 2) & 3).ok()?;
        let attempts = self.wait_attempts;
        let end = if read {
            let req = request_byte(port, RnW::R, addr);
            let last = if ap {
                request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF)
            } else {
                req
            };
            let h = self.read;
            let mut b = Bus::new(wire());
            let end = match self.idle_cycles {
                0 => read_block(&mut b, &h, req, last, ap, count, attempts, response),
                idle => read_block_idle(&mut b, &h, req, last, ap, count, attempts, response, idle),
            };
            b.restore();
            end
        } else {
            let h = self.write;
            let req = request_byte(port, RnW::W, addr);
            let mut b = Bus::new(wire());
            let data = &report[5..5 + 4 * count];
            let end = match self.idle_cycles {
                0 => write_block(&mut b, &h, req, attempts, data),
                idle => write_block_idle(&mut b, &h, req, attempts, data, idle),
            };
            b.restore();
            end
        };
        Some(encode_block_response(
            response, end.ran, end.status, end.words,
        ))
    }
}

#[cfg(test)]
#[path = "block_protocol_tests.rs"]
mod protocol_tests;

#[cfg(test)]
#[path = "block_differential_tests.rs"]
mod differential_tests;

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use self::std::boxed::Box;
    use self::std::cell::RefCell;
    use self::std::rc::Rc;
    use self::std::vec;
    use self::std::vec::Vec;
    use daplink_rs::dap_rs::driver::bitbang::{
        BitbangAdapter, DelayCycles, InputOutputPin, SwdAdapter,
    };
    use daplink_rs::dap_rs::jtag::TapConfig;

    /// One clock cell at the pads: SWDIO at the rising edge, and any bit sampled in the low half.
    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    pub(super) struct Cell {
        pub(super) driven: Option<bool>,
        sampled: Option<bool>,
    }

    /// One bus operation, so a case can count stores as well as edges.
    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    pub(super) enum Op {
        Clk(bool),
        Swdio,
        DataClkLow,
        Sample,
        Dir,
    }

    /// `BSRR` stores: the clock edges and the two ways SWDIO is driven.
    fn stores(ops: &[Op]) -> usize {
        ops.iter()
            .filter(|o| matches!(o, Op::Clk(_) | Op::Swdio | Op::DataClkLow))
            .count()
    }

    fn count(ops: &[Op], want: Op) -> usize {
        ops.iter().filter(|o| **o == want).count()
    }

    #[derive(Default)]
    pub(super) struct Trace {
        out: bool,
        level: bool,
        sampled: Option<bool>,
        pub(super) cells: Vec<Cell>,
        ops: Vec<Op>,
        script: Vec<bool>,
        pub(super) next: usize,
    }

    impl Trace {
        pub(super) fn new(script: &[bool]) -> Rc<RefCell<Self>> {
            Rc::new(RefCell::new(Trace {
                script: script.to_vec(),
                ..Trace::default()
            }))
        }

        fn rise(&mut self) {
            let driven = if self.out { Some(self.level) } else { None };
            let sampled = self.sampled.take();
            self.cells.push(Cell { driven, sampled });
        }

        fn take(&mut self) -> bool {
            let v = self.script[self.next];
            self.next += 1;
            self.sampled = Some(v);
            v
        }
    }

    /// The block's wire over a trace.
    pub(super) struct MockWire(pub(super) Rc<RefCell<Trace>>);

    impl Wire for MockWire {
        fn clk(&mut self, high: bool) {
            let mut t = self.0.borrow_mut();
            t.ops.push(Op::Clk(high));
            if high {
                t.rise();
            }
        }
        fn swdio(&mut self, bit: u32) {
            let mut t = self.0.borrow_mut();
            t.ops.push(Op::Swdio);
            t.level = bit != 0;
        }
        fn data_clk_low(&mut self, bit: u32) {
            let mut t = self.0.borrow_mut();
            t.ops.push(Op::DataClkLow);
            t.level = bit != 0;
        }
        fn sample(&mut self) -> u32 {
            let mut t = self.0.borrow_mut();
            t.ops.push(Op::Sample);
            t.take() as u32
        }
        fn set_out(&mut self, out: bool) {
            let mut t = self.0.borrow_mut();
            t.ops.push(Op::Dir);
            t.out = out;
        }
        fn entry_out(&mut self) -> bool {
            self.0.borrow().out
        }
    }

    /// dap-rs's own pin over the same trace, for comparable cells.
    #[derive(Clone)]
    pub(super) enum TracePin {
        Clk(Rc<RefCell<Trace>>),
        Swdio(Rc<RefCell<Trace>>),
        Nc,
    }

    impl InputOutputPin for TracePin {
        fn set_as_output(&mut self) {
            if let TracePin::Swdio(t) = self {
                t.borrow_mut().out = true;
            }
        }
        fn set_high(&mut self, high: bool) {
            match self {
                TracePin::Clk(t) if high => t.borrow_mut().rise(),
                TracePin::Swdio(t) => t.borrow_mut().level = high,
                _ => {}
            }
        }
        fn set_as_input(&mut self) {
            if let TracePin::Swdio(t) = self {
                t.borrow_mut().out = false;
            }
        }
        fn is_high(&mut self) -> bool {
            match self {
                TracePin::Swdio(t) => t.borrow_mut().take(),
                _ => false,
            }
        }
    }

    #[derive(Clone)]
    pub(super) struct MockDelay;
    impl daplink_rs::dap_rs::dap::DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }
    impl DelayCycles for MockDelay {
        fn cpu_clock(&self) -> u32 {
            250_000_000
        }
        fn delay_cycles(&mut self, _cycles: u32) {}
    }

    pub(super) fn adapter(trace: &Rc<RefCell<Trace>>) -> BitbangAdapter<TracePin, MockDelay> {
        let scan: &'static mut [TapConfig] = Box::leak(Box::new([TapConfig::INIT; 1]));
        BitbangAdapter::new(
            TracePin::Nc,
            TracePin::Nc,
            TracePin::Swdio(trace.clone()),
            TracePin::Clk(trace.clone()),
            TracePin::Nc,
            MockDelay,
            scan,
        )
    }

    /// The H523 probe pads, PB10 and PB15.
    pub(crate) struct TestPads;
    impl SwjPads for TestPads {
        const SWCLK: usize = 10;
        const SWDIO: usize = 15;
        fn swj_port() -> embassy_stm32::pac::gpio::Gpio {
            embassy_stm32::pac::GPIOB
        }
    }

    #[derive(Clone, Copy)]
    pub(super) struct P;
    impl BlockPacing for P {
        type Pads = TestPads;
    }

    /// No cap, so the floors are what bind.
    #[derive(Clone, Copy)]
    struct Uncapped;
    impl BlockPacing for Uncapped {
        type Pads = TestPads;
        const MAX_WIRE_HZ: u32 = 0;
    }

    /// dap-rs answering `report` as `response`, as the block sees it afterwards.
    pub(super) fn answered<P: BlockPacing>(block: &mut H5Block<P>, report: &[u8], response: &[u8]) {
        assert_eq!(block.process(report, &mut [0u8; 64]), None);
        block.observe(report, response);
    }

    fn plan() -> Halves {
        prepare::<P>(250_000_000, 1_000_000, true)
    }

    /// One direction's prepared halves at `request`, as iteration counts.
    fn spins<Q: BlockPacing>(request: u32, write: bool) -> [u32; 6] {
        let mut block: H5Block<Q> = H5Block::new(250_000_000);
        assert!(block.set_clock(request));
        let h = if write { block.write } else { block.read };
        [
            h.ctl_low,
            h.ctl_high,
            h.ack_low,
            h.ack_high,
            h.data_low,
            h.data_high,
        ]
        .map(Spin::get)
    }

    /// The bits a target returns for one read word: ACK, data LSB-first, parity.
    pub(super) fn read_script(ack: u32, data: u32, parity: bool) -> Vec<bool> {
        let mut v: Vec<bool> = (0..3).map(|i| ack & (1 << i) != 0).collect();
        v.extend((0..32).map(|i| data & (1 << i) != 0));
        v.push(parity);
        v
    }

    pub(super) fn ack_script(ack: u32) -> Vec<bool> {
        (0..3).map(|i| ack & (1 << i) != 0).collect()
    }

    /// Run a block read against a scripted target and return the cells and the response.
    fn block_read(script: &[bool], count: u16, treq: u8, retries: usize) -> (Vec<Cell>, Vec<u8>) {
        let trace = Trace::new(script);
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        block.wait_attempts = retries + 1;
        let mut resp = vec![0u8; 4 + 4 * count as usize];
        let req = [0x06u8, 0, count as u8, (count >> 8) as u8, treq];
        let len = block
            .transfer_block(&req, &mut resp, || MockWire(trace.clone()))
            .expect("the fast path takes a well-formed block");
        resp.truncate(len);
        let cells = trace.borrow().cells.clone();
        (cells, resp)
    }

    #[test]
    fn request_bytes_match_dap_rs() {
        for ap in [APnDP::DP, APnDP::AP] {
            for rnw in [RnW::R, RnW::W] {
                for a in 0..4u8 {
                    let addr = DPRegister::try_from(a).unwrap();
                    assert_eq!(
                        request_byte(ap, rnw, addr),
                        swd::make_request(ap, rnw, addr)
                    );
                }
            }
        }
    }

    #[test]
    fn read_word_matches_the_bitbang_adapter() {
        let script = read_script(0b001, 0xDEAD_BEEF, parity32(0xDEAD_BEEF) != 0);
        let mine = Trace::new(&script);
        let req = request_byte(APnDP::AP, RnW::R, DPRegister::CTRLSTAT);
        assert_eq!(
            read_word(&mut Bus::new(MockWire(mine.clone())), &plan(), req),
            Ok(0xDEAD_BEEF)
        );

        let theirs = Trace::new(&script);
        assert_eq!(
            adapter(&theirs).read(APnDP::AP, DPRegister::CTRLSTAT),
            Ok(0xDEAD_BEEF)
        );
        assert_eq!(mine.borrow().cells, theirs.borrow().cells);
        assert_eq!(mine.borrow().cells.len(), 46);
    }

    #[test]
    fn write_word_matches_the_bitbang_adapter() {
        let script = ack_script(0b001);
        let mine = Trace::new(&script);
        let req = request_byte(APnDP::AP, RnW::W, DPRegister::SELECT);
        assert_eq!(
            write_word(
                &mut Bus::new(MockWire(mine.clone())),
                &plan(),
                req,
                0x1234_5678
            ),
            Ok(())
        );

        let theirs = Trace::new(&script);
        assert_eq!(
            adapter(&theirs).write(APnDP::AP, DPRegister::SELECT, 0x1234_5678),
            Ok(())
        );
        assert_eq!(mine.borrow().cells, theirs.borrow().cells);
        assert_eq!(mine.borrow().cells.len(), 46);
    }

    #[test]
    fn a_driven_cell_is_two_stores_and_a_sampled_cell_two() {
        // A read word drives only the eight request bits; every other cell is clock-only.
        let script = read_script(0b001, 0xDEAD_BEEF, parity32(0xDEAD_BEEF) != 0);
        let trace = Trace::new(&script);
        let req = request_byte(APnDP::AP, RnW::R, DPRegister::CTRLSTAT);
        assert_eq!(
            read_word(&mut Bus::new(MockWire(trace.clone())), &plan(), req),
            Ok(0xDEAD_BEEF)
        );
        let ops = trace.borrow().ops.clone();
        assert_eq!(stores(&ops), 2 * 46);
        assert_eq!(count(&ops, Op::DataClkLow), 8);
        assert_eq!(count(&ops, Op::Swdio), 0);
        assert_eq!(count(&ops, Op::Sample), 36);

        // A write word drives the request, the thirty-two data cells and the parity cell.
        let trace = Trace::new(&ack_script(0b001));
        let req = request_byte(APnDP::AP, RnW::W, DPRegister::SELECT);
        assert_eq!(
            write_word(
                &mut Bus::new(MockWire(trace.clone())),
                &plan(),
                req,
                0x1234_5678
            ),
            Ok(())
        );
        let ops = trace.borrow().ops.clone();
        assert_eq!(stores(&ops), 2 * 46);
        assert_eq!(count(&ops, Op::DataClkLow), 41);
        assert_eq!(count(&ops, Op::Swdio), 0);
    }

    /// The bit-bang engine's SWD driver, which single transfers take, over a trace.
    fn engine(trace: &Rc<RefCell<Trace>>) -> daplink_rs::LateSwdDrv<TracePin, MockDelay> {
        let scan = Box::leak(Box::new(daplink_rs::ScanChainStorage::<1>::new()));
        let swdio = TracePin::Swdio(trace.clone());
        let swclk = TracePin::Clk(trace.clone());
        let nc = TracePin::Nc;
        daplink_rs::LateJtagDeps::wired(nc.clone(), nc.clone(), swdio, swclk, nc, MockDelay, scan)
            .into()
    }

    fn driven(trace: &Rc<RefCell<Trace>>) -> Vec<Option<bool>> {
        trace.borrow().cells.iter().map(|c| c.driven).collect()
    }

    #[test]
    fn known_divergence_a_bad_parity_clocks_the_trailing_turnaround() {
        use daplink_rs::dap_rs::swd::Swd;
        let data = 0x0000_00FFu32;
        let mut script = read_script(0b001, data, parity32(data) == 0);
        let mine = Trace::new(&script);
        let req = request_byte(APnDP::AP, RnW::R, DPRegister::CTRLSTAT);
        assert_eq!(
            read_word(&mut Bus::new(MockWire(mine.clone())), &plan(), req),
            Err(swd::Error::BadParity)
        );

        let theirs = Trace::new(&script);
        assert_eq!(
            adapter(&theirs).read(APnDP::AP, DPRegister::CTRLSTAT),
            Err(swd::Error::BadParity)
        );
        // dap-rs returns before the trailing turnaround.
        assert_eq!(theirs.borrow().cells.len(), 45);
        assert_eq!(mine.borrow().cells[..45], theirs.borrow().cells[..]);
        assert_eq!(mine.borrow().cells.len(), 46);
        assert_eq!(mine.borrow().cells[45].driven, None);

        // The engine clocks it as a released read cell, sampling one more bit.
        script.push(true);
        let single = Trace::new(&script);
        assert_eq!(
            engine(&single).read_inner(APnDP::AP, DPRegister::CTRLSTAT),
            Err(swd::Error::BadParity)
        );
        assert_eq!(driven(&single), driven(&mine));
    }

    #[test]
    fn known_divergence_a_write_no_ack_is_no_ack() {
        use daplink_rs::dap_rs::swd::Swd;
        let script = ack_script(0b111);
        let mine = Trace::new(&script);
        let req = request_byte(APnDP::AP, RnW::W, DPRegister::SELECT);
        assert_eq!(
            write_word(&mut Bus::new(MockWire(mine.clone())), &plan(), req, 0),
            Err(swd::Error::NoAck)
        );

        let theirs = Trace::new(&script);
        assert_eq!(
            adapter(&theirs).write(APnDP::AP, DPRegister::SELECT, 0),
            Err(swd::Error::AckUnknown(7))
        );
        assert_eq!(mine.borrow().cells, theirs.borrow().cells);
        assert_eq!(mine.borrow().cells.len(), 13);

        let single = Trace::new(&script);
        assert_eq!(
            engine(&single).write_inner(APnDP::AP, DPRegister::SELECT, 0),
            Err(swd::Error::NoAck)
        );
        assert_eq!(single.borrow().cells, mine.borrow().cells);
    }

    #[test]
    fn a_failed_ack_matches_the_bitbang_adapter() {
        for (ack, err) in [
            (0b010, swd::Error::AckWait),
            (0b100, swd::Error::AckFault),
            (0b111, swd::Error::NoAck),
        ] {
            let script = ack_script(ack);
            let mine = Trace::new(&script);
            let req = request_byte(APnDP::AP, RnW::R, DPRegister::CTRLSTAT);
            assert_eq!(
                read_word(&mut Bus::new(MockWire(mine.clone())), &plan(), req),
                Err(err)
            );
            let theirs = Trace::new(&script);
            assert_eq!(
                adapter(&theirs).read(APnDP::AP, DPRegister::CTRLSTAT),
                Err(err)
            );
            assert_eq!(mine.borrow().cells, theirs.borrow().cells);
            // Eight request cells, the turnaround, three ACK cells, one more turnaround.
            assert_eq!(mine.borrow().cells.len(), 13);
        }
    }

    #[test]
    fn a_posted_ap_read_block_frames_count_and_status() {
        let values = [0x1111_1111u32, 0x2222_2222, 0x3333_3333];
        let mut script = read_script(0b001, 0, parity32(0) != 0);
        for v in values {
            script.extend(read_script(0b001, v, parity32(v) != 0));
        }
        // AP, read, address 1.
        let (cells, resp) = block_read(&script, 3, 0b0111, 5);
        assert_eq!(&resp[..4], &[0x06, 3, 0, 1]);
        for (i, v) in values.iter().enumerate() {
            assert_eq!(&resp[4 + 4 * i..8 + 4 * i], &v.to_le_bytes());
        }
        // The priming read plus three words, 46 cells each.
        assert_eq!(cells.len(), 4 * 46);
        // The last read addresses RDBUFF, not the AP register.
        let last = request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF);
        let head = 3 * 46;
        for bit in 0..8 {
            assert_eq!(cells[head + bit].driven, Some(last & (1 << bit) != 0));
        }
    }

    #[test]
    fn a_dp_read_block_is_not_posted_and_keeps_its_address() {
        let v = 0xA5A5_0F0Fu32;
        let mut script = Vec::new();
        for _ in 0..2 {
            script.extend(read_script(0b001, v, parity32(v) != 0));
        }
        // DP, read, address 2.
        let (cells, resp) = block_read(&script, 2, 0b1010, 5);
        assert_eq!(&resp[..4], &[0x06, 2, 0, 1]);
        assert_eq!(cells.len(), 2 * 46);
        let req = request_byte(APnDP::DP, RnW::R, DPRegister::SELECT);
        for bit in 0..8 {
            assert_eq!(cells[46 + bit].driven, Some(req & (1 << bit) != 0));
        }
    }

    #[test]
    fn a_wait_is_retried_and_then_succeeds() {
        let v = 0x0000_0001u32;
        let mut script = Vec::new();
        // DP read: two WAITs, then the value.
        script.extend(ack_script(0b010));
        script.extend(ack_script(0b010));
        script.extend(read_script(0b001, v, parity32(v) != 0));
        let (cells, resp) = block_read(&script, 1, 0b0010, 5);
        assert_eq!(resp, [0x06, 1, 0, 1, 1, 0, 0, 0]);
        assert_eq!(cells.len(), 2 * 13 + 46);
    }

    #[test]
    fn an_exhausted_wait_budget_reports_the_retry_status() {
        let mut script = Vec::new();
        for _ in 0..4 {
            script.extend(ack_script(0b010));
        }
        let (cells, resp) = block_read(&script, 2, 0b0010, 3);
        assert_eq!(resp, [0x06, 0, 0, 2]);
        assert_eq!(cells.len(), 4 * 13);
    }

    #[test]
    fn a_zero_retry_budget_still_makes_one_attempt() {
        let (cells, resp) = block_read(&ack_script(0b010), 1, 0b0010, 0);
        assert_eq!(resp, [0x06, 0, 0, 2]);
        assert_eq!(cells.len(), 13);
    }

    #[test]
    fn a_fault_mid_block_keeps_the_words_already_read() {
        let v = 0x7777_7777u32;
        let mut script = read_script(0b001, v, parity32(v) != 0);
        script.extend(ack_script(0b100));
        let (_, resp) = block_read(&script, 3, 0b0010, 5);
        assert_eq!(&resp[..4], &[0x06, 1, 0, 4]);
        assert_eq!(resp.len(), 8);
        assert_eq!(&resp[4..], &v.to_le_bytes());
    }

    #[test]
    fn a_noack_on_the_priming_read_reports_no_transfer() {
        let (_, resp) = block_read(&ack_script(0b111), 4, 0b0111, 5);
        assert_eq!(resp, [0x06, 0, 0, 7]);
    }

    #[test]
    fn a_bad_parity_reports_the_protocol_status() {
        let mut script = read_script(0b001, 0, true);
        script.extend(read_script(0b001, 0, true));
        let (_, resp) = block_read(&script, 1, 0b0010, 5);
        assert_eq!(resp, [0x06, 0, 0, 0x0F]);
    }

    #[test]
    fn a_write_block_frames_count_and_status() {
        let values = [0x0102_0304u32, 0x0506_0708];
        let mut script = Vec::new();
        for _ in values {
            script.extend(ack_script(0b001));
        }
        let trace = Trace::new(&script);
        let block: H5Block<P> = H5Block::new(250_000_000);
        let mut req = vec![0x06u8, 0, 2, 0, 0b0100];
        for v in values {
            req.extend(v.to_le_bytes());
        }
        let mut resp = [0u8; 8];
        let len = block
            .transfer_block(&req, &mut resp, || MockWire(trace.clone()))
            .unwrap();
        assert_eq!(len, 4);
        assert_eq!(&resp[..4], &[0x06, 2, 0, 1]);
        let cells = trace.borrow().cells.clone();
        assert_eq!(cells.len(), 2 * 46);
        // The second word's data phase, LSB-first, then its parity.
        for bit in 0..32 {
            assert_eq!(
                cells[46 + 13 + bit].driven,
                Some(values[1] & (1 << bit) != 0)
            );
        }
        assert_eq!(cells[46 + 45].driven, Some(parity32(values[1]) != 0));
    }

    #[test]
    fn a_failed_write_counts_the_writes_before_it() {
        let mut script = ack_script(0b001);
        script.extend(ack_script(0b100));
        let trace = Trace::new(&script);
        let block: H5Block<P> = H5Block::new(250_000_000);
        let mut req = vec![0x06u8, 0, 2, 0, 0b0100];
        req.extend([0u8; 8]);
        let mut resp = [0u8; 8];
        let len = block
            .transfer_block(&req, &mut resp, || MockWire(trace))
            .unwrap();
        assert_eq!(len, 4);
        assert_eq!(&resp[..4], &[0x06, 1, 0, 4]);
    }

    #[test]
    fn the_wire_direction_returns_to_what_it_was() {
        let script = read_script(0b001, 0, false);
        let trace = Trace::new(&script);
        trace.borrow_mut().out = true;
        let block: H5Block<P> = H5Block::new(250_000_000);
        let req = [0x06u8, 0, 1, 0, 0b0010];
        let mut resp = [0u8; 8];
        block
            .transfer_block(&req, &mut resp, || MockWire(trace.clone()))
            .unwrap();
        assert!(trace.borrow().out);
    }

    #[test]
    fn only_a_block_on_a_live_swd_session_leaves_dap_rs() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        let mut resp = [0u8; 64];
        // Nothing is handled before a connect selects SWD.
        assert_eq!(block.process(&[0x06, 0, 1, 0, 0b0010], &mut resp), None);
        answered(&mut block, &[0x02, 2], &[0x02, 2]);
        assert_eq!(block.process(&[0x06, 0, 1, 0, 0b0010], &mut resp), None);
        // Every other command, and a malformed or empty block, stays with dap-rs.
        answered(&mut block, &[0x02, 1], &[0x02, 1]);
        assert_eq!(block.process(&[0x05, 0, 1], &mut resp), None);
        assert_eq!(block.process(&[0x06, 0, 0, 0, 0b0010], &mut resp), None);
        assert_eq!(block.process(&[0x06, 0], &mut resp), None);
        assert_eq!(block.process(&[0x06, 0, 1, 0, 0b0000], &mut resp), None);
        answered(&mut block, &[0x03], &[0x03, 0]);
        assert_eq!(block.process(&[0x06, 0, 1, 0, 0b0010], &mut resp), None);
    }

    #[test]
    fn a_short_response_buffer_stays_with_dap_rs() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        block.observe(&[0x02, 1], &[0x02, 1]);
        let mut resp = [0u8; 7];
        assert_eq!(block.process(&[0x06, 0, 1, 0, 0b0010], &mut resp), None);
    }

    #[test]
    fn the_wait_budget_follows_dap_rs_transfer_configure() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        assert_eq!(block.wait_attempts, 6);
        answered(&mut block, &[0x04, 2, 0x40, 0x01, 0x08, 0x00], &[0x04, 0]);
        assert_eq!((block.wait_attempts, block.idle_cycles), (0x0141, 2));
        // A truncated configure leaves the budget alone.
        answered(&mut block, &[0x04, 3, 0x00], &[0x04, 0]);
        assert_eq!((block.wait_attempts, block.idle_cycles), (0x0141, 2));
        // So does one dap-rs refused.
        answered(&mut block, &[0x04, 5, 1, 0, 0, 0], &[0xFF]);
        assert_eq!((block.wait_attempts, block.idle_cycles), (0x0141, 2));
    }

    /// A flash session keeps the block out of `process`; dap-rs still answers, so `observe` sees it.
    #[test]
    fn a_configure_answered_during_a_flash_session_reaches_the_block() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        block.observe(&[0x04, 3, 7, 0, 0, 0], &[0x04, 0]);
        assert_eq!((block.wait_attempts, block.idle_cycles), (8, 3));
    }

    #[test]
    fn a_zero_request_leaves_the_plan_standing() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        let boot = spins::<P>(u32::MAX, false);
        assert!(!block.set_clock(0));
        assert_eq!(block.read, prepare::<P>(250_000_000, P::MAX_WIRE_HZ, true));
        assert_eq!(boot, spins::<P>(12_000_000, false));
    }

    #[test]
    fn a_request_is_paced_as_one_period() {
        // 4 MHz on a 250 MHz core: 63 cycles, 32 low and 31 high, no fixed work credited.
        assert_eq!(spins::<P>(4_000_000, false), [11, 10, 11, 10, 11, 10]);
        assert_eq!(spins::<P>(4_000_000, true), [11, 10, 11, 10, 11, 10]);
    }

    #[test]
    fn a_request_above_the_cap_is_held_at_the_cap() {
        let read = spins::<P>(12_000_000, false);
        let write = spins::<P>(12_000_000, true);
        for request in [16_000_000u32, 25_000_000, u32::MAX] {
            assert_eq!(spins::<P>(request, false), read, "read at {request} Hz");
            assert_eq!(spins::<P>(request, true), write, "write at {request} Hz");
        }
    }

    #[test]
    fn an_uncapped_request_still_keeps_the_floors() {
        // Driven halves hold 32 ns (8 cycles), a sampled low 44 ns (11 cycles).
        assert_eq!(spins::<Uncapped>(25_000_000, false), [3, 3, 4, 3, 4, 3]);
        assert_eq!(spins::<Uncapped>(25_000_000, true), [3, 3, 4, 3, 3, 3]);
    }

    #[test]
    fn a_swj_clock_report_re_prepares_the_plan() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        let boot = block.read;
        let clock = [0x11, 0x40, 0x42, 0x0F, 0x00];
        answered(&mut block, &clock, &[0x11, 0xFF]);
        assert_eq!(block.read, boot, "a refused clock changes nothing");
        // 1 MHz again, through the report the host sends.
        answered(&mut block, &clock, &[0x11, 0]);
        assert_eq!(block.read, plan());
    }

    /// dap-rs's `CheckResult`, which its crate keeps private: the status byte and the value.
    fn check<T>(r: swd::Result<T>, status: &mut u8) -> Option<T> {
        match r {
            Ok(v) => {
                *status = 1;
                Some(v)
            }
            Err(swd::Error::AckWait) => {
                *status = 2;
                None
            }
            Err(swd::Error::AckFault) => {
                *status = 4;
                None
            }
            Err(swd::Error::NoAck) => {
                *status = 7;
                None
            }
            Err(_) => {
                *status = (1 << 3) | 7;
                None
            }
        }
    }

    /// Local plain-transfer dispatcher model over dap-rs SWD operations, making N + 1 attempts.
    fn dap_rs_transfer(script: &[bool], report: &[u8], retries: usize) -> (Vec<Cell>, Vec<u8>) {
        use daplink_rs::dap_rs::swd::Swd;
        let retries = retries + 1;
        let trace = Trace::new(script);
        let mut swd: SwdAdapter<TracePin, MockDelay> = adapter(&trace).into();
        let count = report[2] as usize;
        let mut resp = vec![0u8; 3 + 4 * count];
        resp[0] = 0x05;
        let mut at = 3;
        let mut pos = 3;
        for done in 0..count {
            resp[1] = (done + 1) as u8;
            let treq = report[pos];
            pos += 1;
            let apndp = if treq & 1 != 0 { APnDP::AP } else { APnDP::DP };
            let a = DPRegister::try_from((treq >> 2) & 3).unwrap();
            if treq & 2 != 0 {
                let value = if apndp == APnDP::AP {
                    if check(swd.read_ap(retries, a), &mut resp[2]).is_none() {
                        break;
                    }
                    match check(swd.read_dp(retries, DPRegister::RDBUFF), &mut resp[2]) {
                        Some(v) => v,
                        None => break,
                    }
                } else {
                    match check(swd.read_dp(retries, a), &mut resp[2]) {
                        Some(v) => v,
                        None => break,
                    }
                };
                resp[at..at + 4].copy_from_slice(&value.to_le_bytes());
                at += 4;
            } else {
                let v = u32::from_le_bytes(report[pos..pos + 4].try_into().unwrap());
                pos += 4;
                if check(swd.write(retries, apndp, a, v), &mut resp[2]).is_none() {
                    break;
                }
            }
        }
        resp.truncate(at);
        let cells = trace.borrow().cells.clone();
        (cells, resp)
    }

    /// Run a `DAP_Transfer` through the fast path against a scripted target.
    fn fast_transfer(script: &[bool], report: &[u8], retries: usize) -> (Vec<Cell>, Vec<u8>) {
        let trace = Trace::new(script);
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        block.wait_attempts = retries + 1;
        let mut resp = vec![0u8; 3 + 4 * report[2] as usize];
        let len = block
            .transfer(report, &mut resp, || MockWire(trace.clone()))
            .expect("the fast path takes a plain transfer");
        resp.truncate(len);
        let cells = trace.borrow().cells.clone();
        (cells, resp)
    }

    /// Both engines against the same scripted target: the cells and the response must agree.
    fn transfer_matches_dap_rs(script: &[bool], report: &[u8], retries: usize) -> Vec<u8> {
        let (mine, resp) = fast_transfer(script, report, retries);
        let (theirs, want) = dap_rs_transfer(script, report, retries);
        assert_eq!(mine, theirs);
        assert_eq!(resp, want);
        resp
    }

    /// A `DAP_Transfer` report: the index byte, the count, then the items.
    fn transfer_report(items: &[u8], count: u8) -> Vec<u8> {
        let mut req = vec![0x05u8, 0, count];
        req.extend_from_slice(items);
        req
    }

    #[test]
    fn a_single_ap_write_matches_dap_rs() {
        // AP write to address 1: the TAR write ahead of every block.
        let report = transfer_report(&[0b0101, 0x00, 0x10, 0x00, 0x20], 1);
        let (cells, _) = fast_transfer(&ack_script(0b001), &report, 5);
        assert_eq!(cells.len(), 46);
        assert_eq!(
            transfer_matches_dap_rs(&ack_script(0b001), &report, 5),
            [0x05, 1, 1]
        );
    }

    #[test]
    fn a_single_dp_write_matches_dap_rs() {
        // DP write to SELECT.
        let report = transfer_report(&[0b1000, 0x11, 0x22, 0x33, 0x44], 1);
        assert_eq!(
            transfer_matches_dap_rs(&ack_script(0b001), &report, 5),
            [0x05, 1, 1]
        );
    }

    #[test]
    fn a_dp_read_matches_dap_rs() {
        let v = 0xDEAD_BEEFu32;
        let script = read_script(0b001, v, parity32(v) != 0);
        // DP read of CTRLSTAT: not posted, so one word on the wire.
        let report = transfer_report(&[0b0110], 1);
        let (cells, resp) = fast_transfer(&script, &report, 5);
        assert_eq!(cells.len(), 46);
        assert_eq!(&resp[3..], &v.to_le_bytes());
        assert_eq!(transfer_matches_dap_rs(&script, &report, 5), resp);
    }

    #[test]
    fn a_posted_ap_read_takes_its_value_off_rdbuff() {
        let stale = 0x1111_1111u32;
        let v = 0xA5A5_0F0Fu32;
        let mut script = read_script(0b001, stale, parity32(stale) != 0);
        script.extend(read_script(0b001, v, parity32(v) != 0));
        // AP read of address 0.
        let report = transfer_report(&[0b0011], 1);
        let (cells, resp) = fast_transfer(&script, &report, 5);
        // The posted read plus the RDBUFF read, and only the RDBUFF value reaches the host.
        assert_eq!(cells.len(), 2 * 46);
        assert_eq!(resp, [0x05, 1, 1, 0x0F, 0x0F, 0xA5, 0xA5]);
        let rdbuff = request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF);
        for bit in 0..8 {
            assert_eq!(cells[46 + bit].driven, Some(rdbuff & (1 << bit) != 0));
        }
        assert_eq!(transfer_matches_dap_rs(&script, &report, 5), resp);
    }

    #[test]
    fn a_wait_then_ok_retries_like_dap_rs() {
        let v = 0x0000_0001u32;
        let mut script = ack_script(0b010);
        script.extend(read_script(0b001, v, parity32(v) != 0));
        let report = transfer_report(&[0b0110], 1);
        let (cells, resp) = fast_transfer(&script, &report, 5);
        assert_eq!(cells.len(), 13 + 46);
        assert_eq!(resp, [0x05, 1, 1, 1, 0, 0, 0]);
        assert_eq!(transfer_matches_dap_rs(&script, &report, 5), resp);

        // An exhausted budget reports the retry status and no word.
        let mut waits = Vec::new();
        for _ in 0..4 {
            waits.extend(ack_script(0b010));
        }
        assert_eq!(transfer_matches_dap_rs(&waits, &report, 3), [0x05, 1, 2]);
    }

    #[test]
    fn a_fault_mid_transfer_stops_where_dap_rs_stops() {
        let mut script = ack_script(0b001);
        script.extend(ack_script(0b100));
        // Three DP writes; the second faults, so the third never reaches the wire.
        let mut items = Vec::new();
        for _ in 0..3 {
            items.extend([0b1000u8, 0, 0, 0, 0]);
        }
        let report = transfer_report(&items, 3);
        let (cells, resp) = fast_transfer(&script, &report, 5);
        assert_eq!(cells.len(), 46 + 13);
        assert_eq!(resp, [0x05, 2, 4]);
        assert_eq!(transfer_matches_dap_rs(&script, &report, 5), resp);

        // A fault after a read keeps the word already placed.
        let v = 0x7777_7777u32;
        let mut script = read_script(0b001, v, parity32(v) != 0);
        script.extend(ack_script(0b100));
        let report = transfer_report(&[0b0110, 0b0110], 2);
        let resp = transfer_matches_dap_rs(&script, &report, 5);
        assert_eq!(&resp[..3], &[0x05, 2, 4]);
        assert_eq!(&resp[3..], &v.to_le_bytes());
    }

    #[test]
    fn a_flagged_transfer_stays_with_dap_rs() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        block.observe(&[0x02, 1], &[0x02, 1]);
        let mut resp = [0u8; 64];
        // Value match, match mask and timestamp each keep the whole report with dap-rs.
        for item in [
            vec![0b0110u8 | (1 << 4), 0, 0, 0, 0],
            vec![0b1000u8 | (1 << 5), 0, 0, 0, 0],
            vec![0b0110u8 | (1 << 7)],
            vec![0b1000u8 | (1 << 7), 0, 0, 0, 0],
        ] {
            assert_eq!(block.process(&transfer_report(&item, 1), &mut resp), None);
        }
        // A flag anywhere in the report refuses the whole report.
        let mut items = vec![0b1000u8, 0, 0, 0, 0];
        items.extend([0b0110u8 | (1 << 4), 0, 0, 0, 0]);
        assert_eq!(block.process(&transfer_report(&items, 2), &mut resp), None);
    }

    #[test]
    fn only_a_transfer_on_a_live_swd_session_leaves_dap_rs() {
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        let mut resp = [0u8; 64];
        let report = transfer_report(&[0b0101, 0, 0, 0, 0], 1);
        // Nothing before a connect selects SWD, and nothing after a disconnect.
        assert_eq!(block.process(&report, &mut resp), None);
        block.observe(&[0x02, 1], &[0x02, 1]);
        // A truncated, empty or short-buffered report also stays with dap-rs.
        assert_eq!(block.process(&[0x05, 0, 1], &mut resp), None);
        assert_eq!(block.process(&[0x05, 0], &mut resp), None);
        assert_eq!(block.process(&transfer_report(&[], 0), &mut resp), None);
        assert_eq!(
            block.process(&transfer_report(&[0b0101, 0, 0, 0], 1), &mut resp),
            None
        );
        assert_eq!(
            block.process(&transfer_report(&[0b0110], 1), &mut [0u8; 6]),
            None
        );
        block.observe(&[0x03], &[0x03, 0]);
        assert_eq!(block.process(&report, &mut resp), None);
    }

    /// A plain AP read of DRW: the item a mixed `DAP_Transfer` repeats.
    const AP_DRW_READ: u8 = 0b1111;

    /// How a scripted target refuses one transaction.
    #[derive(Clone, Copy)]
    enum Refusal {
        Wait,
        Fault,
        NoAck,
        Parity,
    }

    impl Refusal {
        /// The bits the target returns instead of a word; a WAIT is returned to every attempt.
        fn bits(self, retries: usize) -> Vec<bool> {
            match self {
                Refusal::Wait => {
                    let mut v = Vec::new();
                    for _ in 0..=retries {
                        v.extend(ack_script(0b010));
                    }
                    v
                }
                Refusal::Fault => ack_script(0b100),
                Refusal::NoAck => ack_script(0b111),
                // An accepted read whose parity bit contradicts its all-zero word.
                Refusal::Parity => read_script(0b001, 0, true),
            }
        }
    }

    /// One accepted read of `v`.
    fn ok_read(v: u32) -> Vec<bool> {
        read_script(0b001, v, parity32(v) != 0)
    }

    /// The bits a posted run sees; `fail` names the refusing transaction, 0 the priming read.
    fn posted_run_bits(
        stale: u32,
        values: &[u32],
        fail: Option<(usize, Refusal)>,
        retries: usize,
    ) -> Vec<bool> {
        let mut s = Vec::new();
        for k in 0..=values.len() {
            if let Some((at, r)) = fail {
                if at == k {
                    s.extend(r.bits(retries));
                    return s;
                }
            }
            s.extend(ok_read(if k == 0 { stale } else { values[k - 1] }));
        }
        s
    }

    /// The bits dap-rs's two-transactions-per-item shape sees; `fail` names the refusing item and read.
    fn dap_rs_run_bits(
        stale: u32,
        values: &[u32],
        fail: Option<(usize, bool, Refusal)>,
        retries: usize,
    ) -> Vec<bool> {
        let mut s = Vec::new();
        for (j, v) in values.iter().enumerate() {
            let posted = if j == 0 { stale } else { values[j - 1] };
            for (rdbuff, word) in [(false, posted), (true, *v)] {
                match fail {
                    Some((at, on_rdbuff, r)) if at == j && on_rdbuff == rdbuff => {
                        s.extend(r.bits(retries));
                        return s;
                    }
                    _ => s.extend(ok_read(word)),
                }
            }
        }
        s
    }

    /// Both engines over equivalent targets; the response bytes must agree.
    fn responses_agree(
        mine_script: &[bool],
        theirs_script: &[bool],
        report: &[u8],
        retries: usize,
    ) -> (Vec<Cell>, Vec<u8>) {
        let (cells, resp) = fast_transfer(mine_script, report, retries);
        let (_, want) = dap_rs_transfer(theirs_script, report, retries);
        assert_eq!(resp, want);
        (cells, resp)
    }

    #[test]
    fn a_run_of_two_posts_once_and_matches_dap_rs() {
        let values = [0x1000_0001u32, 0x2000_0002];
        let report = transfer_report(&[AP_DRW_READ, AP_DRW_READ], 2);
        let (cells, resp) = responses_agree(
            &posted_run_bits(0, &values, None, 5),
            &dap_rs_run_bits(0, &values, None, 5),
            &report,
            5,
        );
        // One priming read and one transaction per item, against dap-rs's four.
        assert_eq!(cells.len(), 3 * 46);
        assert_eq!(&resp[..3], &[0x05, 2, 1]);
        assert_eq!(&resp[3..7], &values[0].to_le_bytes());
        assert_eq!(&resp[7..], &values[1].to_le_bytes());
        // The run's last transaction addresses RDBUFF; the one before it re-posts the AP.
        let post = request_byte(APnDP::AP, RnW::R, DPRegister::RDBUFF);
        let rdbuff = request_byte(APnDP::DP, RnW::R, DPRegister::RDBUFF);
        for bit in 0..8 {
            assert_eq!(cells[46 + bit].driven, Some(post & (1 << bit) != 0));
            assert_eq!(cells[2 * 46 + bit].driven, Some(rdbuff & (1 << bit) != 0));
        }
    }

    #[test]
    fn a_tar_write_and_fifteen_ap_reads_match_dap_rs() {
        let values: Vec<u32> = (0..15).map(|i| 0x0BAD_0000 + i as u32).collect();
        let mut items = vec![0b0101u8, 0x00, 0x10, 0x00, 0x20];
        items.extend([AP_DRW_READ; 15]);
        let report = transfer_report(&items, 16);
        assert_eq!(report.len(), 23);

        let mut mine = ack_script(0b001);
        mine.extend(posted_run_bits(0, &values, None, 5));
        let mut theirs = ack_script(0b001);
        theirs.extend(dap_rs_run_bits(0, &values, None, 5));
        let (cells, resp) = responses_agree(&mine, &theirs, &report, 5);
        assert_eq!(resp.len(), 63);
        assert_eq!(&resp[..3], &[0x05, 16, 1]);
        for (i, v) in values.iter().enumerate() {
            assert_eq!(&resp[3 + 4 * i..7 + 4 * i], &v.to_le_bytes());
        }
        // The TAR write, one priming read and fifteen reads, against dap-rs's thirty-one words.
        assert_eq!(cells.len(), 17 * 46);
        let (theirs_cells, _) = dap_rs_transfer(&theirs, &report, 5);
        assert_eq!(theirs_cells.len(), 31 * 46);
    }

    #[test]
    fn a_run_interrupted_by_a_dp_read_splits_in_two() {
        let first = [0x1111_0001u32, 0x1111_0002];
        let dp = 0x0DDD_0000u32;
        let second = [0x2222_0001u32, 0x2222_0002];
        let items = [
            AP_DRW_READ,
            AP_DRW_READ,
            0b0110, // DP read of CTRL/STAT: not an AP read, so it ends the run.
            AP_DRW_READ,
            AP_DRW_READ,
        ];
        let report = transfer_report(&items, 5);

        let mut mine = posted_run_bits(0, &first, None, 5);
        mine.extend(ok_read(dp));
        mine.extend(posted_run_bits(0, &second, None, 5));
        let mut theirs = dap_rs_run_bits(0, &first, None, 5);
        theirs.extend(ok_read(dp));
        theirs.extend(dap_rs_run_bits(0, &second, None, 5));

        let (cells, resp) = responses_agree(&mine, &theirs, &report, 5);
        // Three transactions per run and one for the DP read, against dap-rs's nine.
        assert_eq!(cells.len(), 7 * 46);
        assert_eq!(&resp[..3], &[0x05, 5, 1]);
        for (i, v) in [first[0], first[1], dp, second[0], second[1]]
            .iter()
            .enumerate()
        {
            assert_eq!(&resp[3 + 4 * i..7 + 4 * i], &v.to_le_bytes());
        }
    }

    #[test]
    fn a_failure_in_a_run_names_the_item_whose_word_it_lost() {
        let values = [0xA000_0001u32, 0xA000_0002, 0xA000_0003, 0xA000_0004];
        let mut items = vec![0b0101u8, 0x00, 0x10, 0x00, 0x20];
        items.extend([AP_DRW_READ; 4]);
        let report = transfer_report(&items, 5);
        let retries = 3;

        // Map each transaction to the dap-rs item it is attributed to.
        for (at, item) in [(0usize, 0usize), (1, 0), (2, 1), (3, 2), (4, 3)] {
            for refusal in [
                Refusal::Wait,
                Refusal::Fault,
                Refusal::NoAck,
                Refusal::Parity,
            ] {
                let mut mine = ack_script(0b001);
                mine.extend(posted_run_bits(0, &values, Some((at, refusal)), retries));
                // dap-rs fails at a different transaction, but frames the same count, status and words.
                for on_rdbuff in [false, true] {
                    let mut theirs = ack_script(0b001);
                    theirs.extend(dap_rs_run_bits(
                        0,
                        &values,
                        Some((item, on_rdbuff, refusal)),
                        retries,
                    ));
                    let (_, resp) = responses_agree(&mine, &theirs, &report, retries);
                    // The TAR write is item 1, so the run's item `item` is transfer `item + 2`.
                    assert_eq!(resp[1], (item + 2) as u8);
                    // Only the words confirmed before the refusal are returned.
                    assert_eq!(resp.len(), 3 + 4 * item);
                    for (i, v) in values.iter().take(item).enumerate() {
                        assert_eq!(&resp[3 + 4 * i..7 + 4 * i], &v.to_le_bytes());
                    }
                }
            }
        }

        // A refusal on the TAR write ahead of the run stops both engines identically.
        let report = transfer_report(&items, 5);
        assert_eq!(
            transfer_matches_dap_rs(&ack_script(0b100), &report, retries),
            [0x05, 1, 4]
        );
    }

    #[test]
    fn a_run_costs_one_transaction_more_than_its_items() {
        for n in 1..=15usize {
            let values: Vec<u32> = (0..n).map(|i| 0xC000_0000 + i as u32).collect();
            let report = transfer_report(&vec![AP_DRW_READ; n], n as u8);
            let (cells, resp) = fast_transfer(&posted_run_bits(0, &values, None, 5), &report, 5);
            assert_eq!(cells.len(), (n + 1) * 46, "run of {n}");
            assert_eq!(resp.len(), 3 + 4 * n);
        }
    }
}
