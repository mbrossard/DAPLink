//! DMA0, owned by this crate because this firmware is its only user.

/// Every channel DMA0 has. The controller indexes the table at `SRAMBASE + 16 * channel`, so a
/// table shorter than the whole range sends it to fetch a descriptor from whatever follows.
const CHANNELS: usize = 23;

/// One channel descriptor: transfer configuration, last source and destination addresses, reload.
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub struct Descriptor {
    pub xfercfg: u32,
    pub src_end: u32,
    pub dst_end: u32,
    pub next: u32,
}

impl Descriptor {
    pub const fn zeroed() -> Self {
        Descriptor {
            xfercfg: 0,
            src_end: 0,
            dst_end: 0,
            next: 0,
        }
    }
}

/// Everything the controller reads, in one allocation: the head table it indexes by channel, the
/// descriptors channels reload from, and the buffers those fill. The `SRAMBASE` alignment is here.
#[repr(C, align(512))]
pub struct DmaMem {
    pub table: [Descriptor; CHANNELS],
    /// One reload descriptor and one lap of buffer per receiving UART.
    pub vcom: RxSlot,
    pub swo: SwoSlot,
    /// The transmit staging ring, one contiguous run the controller reads directly.
    pub tx: TxRing,
}

/// The transmit staging ring: a byte ring, so a run of bytes arms as one transfer however it was
/// staged.
#[repr(C, align(4))]
pub struct TxRing {
    pub buf: [u8; TX_RING_LEN],
}

pub const TX_RING_LEN: usize = 2048;

/// The longest transfer that can be armed: the counter encodes an exhausted descriptor and a full
/// 1024 identically, so 1023 is the largest unambiguous count.
pub const TX_MAX_XFER: usize = 1023;

/// The trace ring: a closed chain of descriptors over one buffer.
#[repr(C, align(16))]
pub struct SwoSlot {
    pub descs: [Descriptor; SWO_BLOCKS],
    pub buf: [u8; SWO_BUF_LEN],
}

/// A self-linked descriptor and the ring it fills.
#[repr(C, align(16))]
pub struct RxSlot {
    pub loop_desc: Descriptor,
    pub buf: [u8; RX_BUF_LEN],
}

/// Bytes one descriptor covers, capped by [`TX_MAX_XFER`]'s ceiling rather than by the buffer: a
/// longer ring is more descriptors, never a bigger one.
pub const RX_BUF_LEN: usize = 1023;

/// Bytes one trace block covers: a power of two under the counter's ceiling, and the amount a
/// single lost completion could misplace the cursor by.
pub const SWO_BLOCK_LEN: usize = 512;

/// Blocks the trace ring chains, sized to the *host's* polling interval rather than the probe's:
/// trace leaves only when the host asks for it.
pub const SWO_BLOCKS: usize = 16;
pub const SWO_BUF_LEN: usize = SWO_BLOCKS * SWO_BLOCK_LEN;

// `.dma_mem` pins the block's address, which this part's USB attach is sensitive to.
#[used]
#[link_section = ".dma_mem"]
static mut MEM: DmaMem = DmaMem {
    table: [Descriptor::zeroed(); CHANNELS],
    vcom: RxSlot::zeroed(),
    swo: SwoSlot::zeroed(),
    tx: TxRing::zeroed(),
};

impl TxRing {
    const fn zeroed() -> Self {
        TxRing {
            buf: [0; TX_RING_LEN],
        }
    }
}

impl SwoSlot {
    const fn zeroed() -> Self {
        SwoSlot {
            descs: [Descriptor::zeroed(); SWO_BLOCKS],
            buf: [0; SWO_BUF_LEN],
        }
    }
}

impl RxSlot {
    const fn zeroed() -> Self {
        RxSlot {
            loop_desc: Descriptor::zeroed(),
            buf: [0; RX_BUF_LEN],
        }
    }
}

/// The one allocation the controller reads from.
pub fn mem() -> *mut DmaMem {
    core::ptr::addr_of_mut!(MEM)
}

/// Clock DMA0 and point it at the table; the first call does everything and the rest nothing, so
/// a recovery re-entry cannot wipe memory under a live channel. The controller is left enabled
/// with no channel armed.
pub fn init() {
    use core::sync::atomic::{AtomicBool, Ordering};
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return;
    }
    // The block sits outside `.bss`, so startup has not zeroed it.
    // SAFETY: guarded above to run once, before any channel is armed or `mem()` has readers.
    unsafe { core::ptr::write_bytes(mem() as *mut u8, 0, core::mem::size_of::<DmaMem>()) };

    let syscon = unsafe { &*lpc55_hal::raw::SYSCON::ptr() };
    syscon.ahbclkctrl0.modify(|_, w| w.dma0().set_bit());

    let regs = unsafe { &*lpc55_hal::raw::DMA0::ptr() };
    regs.srambase.write(|w| unsafe { w.bits(mem() as u32) });
    regs.ctrl.write(|w| w.enable().set_bit());
}

/// The head descriptor a channel reloads from, for a caller that owns that channel.
pub fn descriptor(channel: usize) -> *mut Descriptor {
    // SAFETY: the table is reachable only here, and a caller owns the channel it names for as
    // long as that channel is programmed.
    unsafe { core::ptr::addr_of_mut!((*mem()).table[channel]) }
}

/// Transfer configuration a receive channel runs: `WIDTH` (bits 9:8) and `SRCINC` (13:12) at their
/// zero encodings, and `CLRTRIG` clear so the software trigger survives every reload.
const RX_XFERCFG: u32 = (1 << 0)          // CFGVALID
    | (1 << 1)                             // RELOAD
    | (1 << 4)                             // SETINTA
    | (0b01 << 14)                         // DSTINC = one byte
    | ((RX_BUF_LEN as u32 - 1) << 16); // XFERCOUNT, encoded as one less

/// The same configuration over a trace block, which is shorter than a VCOM block.
const SWO_XFERCFG: u32 = (RX_XFERCFG & !(0xFFFF << 16)) | ((SWO_BLOCK_LEN as u32 - 1) << 16);

fn regs() -> &'static lpc55_hal::raw::dma0::RegisterBlock {
    // SAFETY: this crate owns DMA0; every accessor here is per-channel.
    unsafe { &*lpc55_hal::raw::DMA0::ptr() }
}

/// A channel's own registers, reached by the documented layout rather than the PAC's one field per
/// channel: the per-channel block at `0x400 + 16 * channel`, holding CFG, CTLSTAT and XFERCFG.
fn channel_reg(channel: usize, offset: usize) -> *mut u32 {
    (lpc55_hal::raw::DMA0::PTR as usize + 0x400 + 16 * channel + offset) as *mut u32
}

/// Bound on the wait for a beat to finish; anything near it is a controller that will not answer.
const BUSY_SPIN_LIMIT: u32 = 100_000;

/// Attempts to read a position with no lap landing mid-read before giving up.
const LAP_SETTLE_TRIES: usize = 4;

const CFG: usize = 0x0;
const XFERCFG: usize = 0x8;

/// Arm `channel` to fill `slot` from `fiford`, and hand the peripheral's FIFO to it. The caller
/// owns both the channel and the peripheral.
pub(crate) fn arm_rx(channel: usize, slot: *mut RxSlot, fiford: u32, priority: u8) {
    let dma = regs();
    // SAFETY: the slot is one of `MEM`'s, owned by the seam that passes it, and the controller
    // is not reading it until the channel is armed below.
    unsafe {
        let loop_desc = core::ptr::addr_of_mut!((*slot).loop_desc);
        (*loop_desc) = Descriptor {
            xfercfg: RX_XFERCFG,
            src_end: fiford,
            dst_end: core::ptr::addr_of!((*slot).buf[RX_BUF_LEN - 1]) as u32,
            next: loop_desc as u32,
        };
        *descriptor(channel) = *loop_desc;
    }
    // Peripheral-driven, no hardware trigger, at the priority the caller chose.
    // SAFETY: `channel` is owned by the caller and the offsets are the documented block layout.
    unsafe {
        channel_reg(channel, CFG).write_volatile(1 | ((priority as u32 & 0x7) << 16));
        channel_reg(channel, XFERCFG).write_volatile(RX_XFERCFG);
    }
    dma.enableset0.write(|w| unsafe { w.bits(1 << channel) });
    // Enabling is not enough: a channel with `TRIG` clear does nothing, and a request cannot.
    dma.settrig0.write(|w| unsafe { w.bits(1 << channel) });
}

/// Stop `channel` and discard everything it had latched, so a restart begins clean.
pub(crate) fn stop(channel: usize) {
    let dma = regs();
    dma.enableclr0.write(|w| unsafe { w.bits(1 << channel) });
    // `BUSY` only, and bounded: `ACTIVE` stays set on a healthy idle channel and never clears.
    let mut spins = 0u32;
    while dma.busy0.read().bits() & (1 << channel) != 0 && spins < BUSY_SPIN_LIMIT {
        spins += 1;
    }
    // Disabling does not clear a latched trigger; only this discards it.
    dma.abort0.write(|w| unsafe { w.bits(1 << channel) });
    dma.inta0.write(|w| unsafe { w.bits(1 << channel) });
}

/// Units still to be written before the counter reloads, clamped to one VCOM block.
pub(crate) fn remaining(channel: usize) -> usize {
    remaining_in(channel, RX_BUF_LEN)
}

/// The same reading clamped to a caller-chosen block, which must be the block the channel was
/// armed with: too long a clamp lets `block - remaining` run backwards past zero.
pub(crate) fn remaining_in(channel: usize, block: usize) -> usize {
    // The field counts down and encodes one less, so 0x3FF at the reload boundary is a full ring.
    // SAFETY: a plain read of the channel's own transfer word.
    let cfg = unsafe { channel_reg(channel, XFERCFG).read_volatile() };
    let n = ((cfg >> 16) & 0x3FF) as usize;
    (n + 1).min(block)
}

/// Whether a lap is latched, without consuming it.
fn peek_lap(channel: usize) -> bool {
    regs().inta0.read().bits() & (1 << channel) != 0
}

/// Take and clear the lap flag; `INTA0` latches whether or not the interrupt is enabled.
pub(crate) fn take_lap(channel: usize) -> bool {
    let dma = regs();
    if dma.inta0.read().bits() & (1 << channel) == 0 {
        return false;
    }
    dma.inta0.write(|w| unsafe { w.bits(1 << channel) });
    true
}

/// Let a channel raise the DMA0 interrupt on completion, so a caller can re-arm from the
/// completion itself instead of from whatever poll comes next.
pub(crate) fn enable_int(channel: usize) {
    regs().intenset0.write(|w| unsafe { w.bits(1 << channel) });
}

/// The reverse of [`enable_int`], for a channel being torn down.
pub(crate) fn disable_int(channel: usize) {
    regs().intenclr0.write(|w| unsafe { w.bits(1 << channel) });
}

/// Take and clear a latched channel error.
pub(crate) fn take_error(channel: usize) -> bool {
    let dma = regs();
    if dma.errint0.read().bits() & (1 << channel) == 0 {
        return false;
    }
    dma.errint0.write(|w| unsafe { w.bits(1 << channel) });
    true
}

/// One poll of a receive channel: read the position and the lap flag in the order the ring's
/// accounting needs, and report a latched error for the owning seam to quiesce.
pub(crate) fn sample_rx(channel: usize) -> (usize, bool, bool) {
    let errored = take_error(channel);
    // The flag is set at completion, not at the reload: a lap counts only where the counter is
    // seen jumping up across it.
    for _ in 0..LAP_SETTLE_TRIES {
        let before = remaining(channel);
        let lapped = take_lap(channel);
        let after = remaining(channel);
        if peek_lap(channel) {
            // A further lap landed mid-read; nothing read here can be trusted to pair up.
            continue;
        }
        return (after, lapped && after > before, errored);
    }
    // Give up toward "no lap": losing bytes visibly beats duplicating them silently.
    (remaining(channel), false, errored)
}

/// Arm `channel` over a closed chain of descriptors covering `slot`'s buffer, each block
/// [`SWO_BLOCK_LEN`] long and each raising `INTA`, which the DMA0 interrupt counts.
pub(crate) fn arm_rx_chain(channel: usize, slot: *mut SwoSlot, fiford: u32, priority: u8) {
    let dma = regs();
    // SAFETY: the slot is `MEM`'s, owned by the seam that passes it, and the controller is not
    // reading it until the channel is armed below.
    unsafe {
        let descs = core::ptr::addr_of_mut!((*slot).descs);
        for i in 0..SWO_BLOCKS {
            let next = (i + 1) % SWO_BLOCKS;
            (*descs)[i] = Descriptor {
                xfercfg: SWO_XFERCFG,
                src_end: fiford,
                dst_end: core::ptr::addr_of!((*slot).buf[(i + 1) * SWO_BLOCK_LEN - 1]) as u32,
                next: core::ptr::addr_of!((*descs)[next]) as u32,
            };
        }
        *descriptor(channel) = (*descs)[0];
    }
    // SAFETY: `channel` is owned by the caller and the offsets are the documented block layout.
    unsafe {
        channel_reg(channel, CFG).write_volatile(1 | ((priority as u32 & 0x7) << 16));
        // The live word must state the same block the chain does; it is what runs first.
        channel_reg(channel, XFERCFG).write_volatile(SWO_XFERCFG);
    }
    dma.enableset0.write(|w| unsafe { w.bits(1 << channel) });
    dma.settrig0.write(|w| unsafe { w.bits(1 << channel) });
}

/// One poll of a chained channel, as an absolute count of units produced since it was armed,
/// plus a latched error for the owning seam to quiesce. `completed` is the block count the DMA0
/// interrupt maintains: the `INTA0` latch would charge several completions between polls as one.
/// `last_abs` is the caller's record of the previous report, and the return is clamped against
/// it: the reported position never steps backward, because a consumer folding it modulo the
/// ring would charge a backward step as most of a lap and re-deliver consumed data.
pub(crate) fn sample_rx_chain(
    channel: usize,
    completed: &core::sync::atomic::AtomicUsize,
    last_abs: &core::sync::atomic::AtomicUsize,
) -> (usize, bool) {
    use core::sync::atomic::Ordering;

    let errored = take_error(channel);
    let mut sampled = None;
    for _ in 0..LAP_SETTLE_TRIES {
        let blocks = completed.load(Ordering::Relaxed);
        let after = remaining_in(channel, SWO_BLOCK_LEN);
        // A completion pending or landing mid-read would pair the offset with a stale count.
        if peek_lap(channel) || completed.load(Ordering::Relaxed) != blocks {
            continue;
        }
        sampled = Some(blocks * SWO_BLOCK_LEN + (SWO_BLOCK_LEN - after));
        break;
    }
    // Exhausted retries fall back to the counted blocks alone: still never past the true
    // position, so losing a lap visibly beats inventing one.
    let raw = sampled.unwrap_or_else(|| completed.load(Ordering::Relaxed) * SWO_BLOCK_LEN);
    let abs = raw.max(last_abs.load(Ordering::Relaxed));
    last_abs.store(abs, Ordering::Relaxed);
    (abs, errored)
}

/// Arm `channel` to send `len` bytes from `src` into `fifowr`, once. No reload: a transmit is
/// finite, and the caller re-arms when it has more to send.
pub(crate) fn arm_tx(channel: usize, src: *const u8, len: usize, fifowr: u32, priority: u8) {
    let dma = regs();
    let xfercfg = (1 << 0)                  // CFGVALID
        | (1 << 4)                           // SETINTA
        | (0b01 << 12)                       // SRCINC = one byte
        | ((len as u32 - 1) << 16); // XFERCOUNT, encoded as one less
                                    // SAFETY: the buffer is `MEM`'s; the channel is the caller's.
    unsafe {
        *descriptor(channel) = Descriptor {
            xfercfg,
            src_end: src.add(len - 1) as u32,
            dst_end: fifowr,
            next: 0,
        };
        channel_reg(channel, CFG).write_volatile(1 | ((priority as u32 & 0x7) << 16));
        channel_reg(channel, XFERCFG).write_volatile(xfercfg);
    }
    dma.enableset0.write(|w| unsafe { w.bits(1 << channel) });
    dma.settrig0.write(|w| unsafe { w.bits(1 << channel) });
}

/// Whether `channel` has finished the transfer it was given, consuming the flag if so.
pub(crate) fn tx_complete(channel: usize) -> bool {
    take_lap(channel)
}
