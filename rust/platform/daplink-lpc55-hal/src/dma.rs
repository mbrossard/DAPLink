//! DMA0, owned by this crate because this firmware is its only user.
// Channel owners: 4 VCOM receive (`dma_rx`), 5 VCOM transmit (`dma_tx`), 8 SWO (`dma_swo`).

/// Every DMA0 channel: the controller indexes the table by channel, so it must cover them all.
const CHANNELS: usize = 23;

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

/// Everything the controller reads, in one `SRAMBASE`-aligned allocation.
#[repr(C, align(512))]
pub struct DmaMem {
    pub table: [Descriptor; CHANNELS],
    pub vcom: RxSlot,
    pub swo: SwoSlot,
    pub tx: TxRing,
}

/// The transmit staging ring: a byte ring, so a run arms as one transfer however it was staged.
#[repr(C, align(4))]
pub struct TxRing {
    pub buf: [u8; TX_RING_LEN],
}

pub const TX_RING_LEN: usize = 2048;

/// Longest armable transfer: 1024 encodes the same as an exhausted descriptor.
pub const TX_MAX_XFER: usize = 1023;

#[repr(C, align(16))]
pub struct SwoSlot {
    pub descs: [Descriptor; SWO_BLOCKS],
    pub buf: [u8; SWO_BUF_LEN],
}

#[repr(C, align(16))]
pub struct RxSlot {
    pub loop_desc: Descriptor,
    pub buf: [u8; RX_BUF_LEN],
}

/// Bytes one descriptor covers; a longer ring is more descriptors, never a bigger one.
pub const RX_BUF_LEN: usize = 1023;

/// Bytes one trace block covers: what a single lost completion could misplace the cursor by.
pub const SWO_BLOCK_LEN: usize = 512;

/// Trace ring blocks, sized to the host's polling interval: trace leaves only when the host asks.
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

pub fn mem() -> *mut DmaMem {
    #[cfg(test)]
    {
        stop_tests::mem()
    }
    #[cfg(not(test))]
    {
        core::ptr::addr_of_mut!(MEM)
    }
}

/// Clock DMA0 and point it at the table; only the first call acts, so a recovery re-entry cannot
/// wipe memory under a live channel. Leaves the controller enabled with no channel armed.
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
    // SAFETY: the table is reachable only here, and the caller owns the channel it names.
    unsafe { core::ptr::addr_of_mut!((*mem()).table[channel]) }
}

/// Receive channel configuration; `CLRTRIG` clear so the software trigger survives every reload.
const RX_XFERCFG: u32 = (1 << 0)          // CFGVALID
    | (1 << 1)                             // RELOAD
    | (1 << 4)                             // SETINTA
    | (0b01 << 14)                         // DSTINC = one byte
    | ((RX_BUF_LEN as u32 - 1) << 16); // XFERCOUNT, encoded as one less

/// The same configuration over a trace block, which is shorter than a VCOM block.
const SWO_XFERCFG: u32 = (RX_XFERCFG & !(0xFFFF << 16)) | ((SWO_BLOCK_LEN as u32 - 1) << 16);

fn regs() -> &'static lpc55_hal::raw::dma0::RegisterBlock {
    #[cfg(test)]
    {
        stop_tests::regs()
    }
    #[cfg(not(test))]
    // SAFETY: this crate owns DMA0; every accessor here is per-channel.
    unsafe {
        &*lpc55_hal::raw::DMA0::ptr()
    }
}

/// A channel's own registers, by the documented per-channel block at `0x400 + 16 * channel`.
fn channel_reg(channel: usize, offset: usize) -> *mut u32 {
    #[cfg(test)]
    {
        (regs() as *const _ as usize + 0x400 + 16 * channel + offset) as *mut u32
    }
    #[cfg(not(test))]
    {
        (lpc55_hal::raw::DMA0::PTR as usize + 0x400 + 16 * channel + offset) as *mut u32
    }
}

/// Bound on the wait for a beat to finish; anything near it is a controller that will not answer.
const BUSY_SPIN_LIMIT: u32 = 100_000;

/// Attempts to read a position with no lap landing mid-read before giving up.
const LAP_SETTLE_TRIES: usize = 4;

const CFG: usize = 0x0;
const XFERCFG: usize = 0x8;

/// Arm `channel` to fill `slot` from `fiford` and hand it the FIFO; the caller owns both.
pub(crate) fn arm_rx(channel: usize, slot: *mut RxSlot, fiford: u32, priority: u8) {
    #[cfg(test)]
    stop_tests::arm(channel);
    let dma = regs();
    // SAFETY: the slot is `MEM`'s, owned by the caller, and unread by the controller until armed.
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

/// Stop `channel` before reuse; an unresponsive controller halts with interrupts masked.
pub(crate) fn stop(channel: usize) {
    if try_stop(channel).is_err() {
        stop_failed();
    }
}

fn try_stop(channel: usize) -> Result<(), u32> {
    stop_write(0x028, 1 << channel);
    let mut busy = 0;
    for _ in 0..BUSY_SPIN_LIMIT {
        busy = stop_busy();
        if busy & (1 << channel) == 0 {
            stop_write(0x078, 1 << channel);
            stop_write(0x058, 1 << channel);
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            return Ok(());
        }
    }
    Err(busy)
}

#[cfg(not(test))]
fn stop_write(offset: usize, value: u32) {
    unsafe {
        (lpc55_hal::raw::DMA0::PTR.cast::<u8>().add(offset) as *mut u32).write_volatile(value)
    };
}

#[cfg(not(test))]
fn stop_busy() -> u32 {
    regs().busy0.read().bits()
}

#[cfg(not(test))]
fn stop_failed() -> ! {
    cortex_m::interrupt::disable();
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
use stop_tests::{stop_busy, stop_failed, stop_write};

pub(crate) fn masked<T>(f: impl FnOnce() -> T) -> T {
    #[cfg(not(test))]
    {
        cortex_m::interrupt::free(|_| f())
    }
    #[cfg(test)]
    {
        f()
    }
}

#[cfg(test)]
pub(crate) mod stop_tests;

/// Units still to be written before the counter reloads, clamped to one VCOM block.
pub(crate) fn remaining(channel: usize) -> usize {
    remaining_in(channel, RX_BUF_LEN)
}

/// As [`remaining`], clamped to the block the channel was armed with; a longer clamp lets
/// `block - remaining` run past zero.
pub(crate) fn remaining_in(channel: usize, block: usize) -> usize {
    // The field counts down and encodes one less, so 0x3FF at the reload boundary is a full ring.
    #[cfg(test)]
    let cfg = tests::xfercfg(channel);
    #[cfg(not(test))]
    // SAFETY: a plain read of the channel's own transfer word.
    let cfg = unsafe { channel_reg(channel, XFERCFG).read_volatile() };
    let n = ((cfg >> 16) & 0x3FF) as usize;
    (n + 1).min(block)
}

#[cfg(not(test))]
fn peek_lap(channel: usize) -> bool {
    regs().inta0.read().bits() & (1 << channel) != 0
}

/// Take and clear the lap flag; `INTA0` latches whether or not the interrupt is enabled.
#[cfg(not(test))]
pub(crate) fn take_lap(channel: usize) -> bool {
    let dma = regs();
    if dma.inta0.read().bits() & (1 << channel) == 0 {
        return false;
    }
    dma.inta0.write(|w| unsafe { w.bits(1 << channel) });
    true
}

/// Raise the DMA0 interrupt on completion, so a caller can re-arm from the completion itself.
pub(crate) fn enable_int(channel: usize) {
    regs().intenset0.write(|w| unsafe { w.bits(1 << channel) });
}

pub(crate) fn disable_int(channel: usize) {
    regs().intenclr0.write(|w| unsafe { w.bits(1 << channel) });
}

#[cfg(not(test))]
pub(crate) fn take_error(channel: usize) -> bool {
    let dma = regs();
    if dma.errint0.read().bits() & (1 << channel) == 0 {
        return false;
    }
    dma.errint0.write(|w| unsafe { w.bits(1 << channel) });
    true
}

/// One receive-channel poll: position, lap flag and latched error, in the ring's accounting order.
pub(crate) fn sample_rx(channel: usize) -> (usize, bool, bool) {
    let errored = take_error(channel);
    // The flag is set at completion, not at reload: a lap counts only where the counter jumps up.
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

/// Arm `channel` over a closed descriptor chain of [`SWO_BLOCK_LEN`] blocks, each raising `INTA`.
pub(crate) fn arm_rx_chain(channel: usize, slot: *mut SwoSlot, fiford: u32, priority: u8) {
    #[cfg(test)]
    stop_tests::arm(channel);
    let dma = regs();
    // SAFETY: the slot is `MEM`'s, owned by the caller, and unread by the controller until armed.
    unsafe {
        link_chain(slot, fiford);
        *descriptor(channel) = (*slot).descs[0];
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

/// Close `slot`'s descriptors into a ring, each filling the next block; `slot` must be unread.
#[inline(always)]
unsafe fn link_chain(slot: *mut SwoSlot, fiford: u32) {
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
}

/// One chained-channel poll as an absolute count since arming, clamped never to step backward.
pub(crate) fn sample_rx_chain(
    channel: usize,
    completed: impl FnMut() -> u64,
    last_abs: u64,
) -> (u64, bool) {
    let errored = take_error(channel);
    (chain_position(channel, completed, last_abs), errored)
}

/// Advisory reads share the coherent sampler without consuming errors or publishing movement.
pub(crate) fn chain_position(
    channel: usize,
    mut completed: impl FnMut() -> u64,
    last_abs: u64,
) -> u64 {
    let mut sampled = None;
    for _ in 0..LAP_SETTLE_TRIES {
        let blocks = completed();
        let after = remaining_in(channel, SWO_BLOCK_LEN);
        // A completion pending or landing mid-read would pair the offset with a stale count.
        if peek_lap(channel) || completed() != blocks {
            continue;
        }
        sampled = Some(chain_bytes(blocks, SWO_BLOCK_LEN - after));
        break;
    }
    // Exhausted retries fall back to the counted blocks alone, never past the true position.
    let raw = sampled.unwrap_or_else(|| chain_bytes(completed(), 0));
    raw.max(last_abs)
}

// The non-wrapping capture lifetime is fewer than 2^64 bytes, independent of host polling.
fn chain_bytes(blocks: u64, offset: usize) -> u64 {
    blocks
        .wrapping_mul(SWO_BLOCK_LEN as u64)
        .wrapping_add(offset as u64)
}

/// Arm `channel` to send `len` bytes from `src` into `fifowr`, once, without reload.
pub(crate) fn arm_tx(channel: usize, src: *const u8, len: usize, fifowr: u32, priority: u8) {
    #[cfg(test)]
    stop_tests::arm(channel);
    let dma = regs();
    let xfercfg = tx_xfercfg(len);
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

/// A single, non-reloading transfer of `len` bytes into a fixed FIFO address.
#[inline(always)]
const fn tx_xfercfg(len: usize) -> u32 {
    (1 << 0)                                // CFGVALID
        | (1 << 4)                           // SETINTA
        | (0b01 << 12)                       // SRCINC = one byte
        | ((len as u32 - 1) << 16) // XFERCOUNT, encoded as one less
}

/// Whether `channel` has finished the transfer it was given, consuming the flag if so.
pub(crate) fn tx_complete(channel: usize) -> bool {
    take_lap(channel)
}

// Host tests script the channel's counter and flags in place of DMA0.
#[cfg(test)]
pub(crate) use tests::error_sample;
#[cfg(test)]
pub(crate) use tests::{peek_lap, take_error, take_lap};

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    extern crate std;
    use core::cell::Cell;
    use std::boxed::Box;
    use std::cell::RefCell;
    use std::collections::VecDeque;

    /// What lands just after a scripted counter read.
    #[derive(Clone, Copy)]
    pub(crate) enum Then {
        Nothing,
        /// A completion latches `INTA` and is still pending.
        Flag,
        /// A completion is taken and counted by the DMA0 interrupt.
        Irq,
        SwoIrq,
    }

    #[derive(Default)]
    struct Fake {
        counts: VecDeque<(u32, Then)>,
        lap: bool,
        error: bool,
        completed: Option<&'static Cell<u64>>,
    }

    std::thread_local! {
        static FAKE: RefCell<Fake> = RefCell::new(Fake::default());
    }

    pub(crate) fn script(counts: &[(u32, Then)], lap: bool, error: bool) {
        FAKE.with(|f| {
            *f.borrow_mut() = Fake {
                counts: counts.iter().copied().collect(),
                lap,
                error,
                completed: None,
            }
        });
    }

    pub(crate) fn error_sample() {
        script(&[(99, Then::Nothing), (99, Then::Nothing)], false, true);
    }

    fn unread() -> usize {
        FAKE.with(|f| f.borrow().counts.len())
    }

    /// The live `XFERCFG` word for the next scripted `XFERCOUNT` field.
    pub(super) fn xfercfg(_channel: usize) -> u32 {
        let (word, swo_irq) = FAKE.with(|f| {
            let mut f = f.borrow_mut();
            let (field, then) = f.counts.pop_front().expect("unscripted counter read");
            match then {
                Then::Nothing => {}
                Then::Flag => f.lap = true,
                Then::Irq => {
                    let completed = f.completed.unwrap();
                    completed.set(completed.get().wrapping_add(1));
                }
                Then::SwoIrq => f.lap = true,
            }
            (
                (field << 16) | (RX_XFERCFG & 0xFFFF),
                matches!(then, Then::SwoIrq),
            )
        });
        if swo_irq {
            crate::dma_swo::swo_dma_service_irq();
        }
        word
    }

    pub(crate) fn peek_lap(_channel: usize) -> bool {
        FAKE.with(|f| f.borrow().lap)
    }

    pub(crate) fn take_lap(_channel: usize) -> bool {
        FAKE.with(|f| core::mem::take(&mut f.borrow_mut().lap))
    }

    pub(crate) fn take_error(_channel: usize) -> bool {
        FAKE.with(|f| core::mem::take(&mut f.borrow_mut().error))
    }

    /// The field holds one less than the units left, and a reload-boundary 0x3FF clamps to the block.
    #[test]
    fn the_counter_field_decodes_to_units_left_within_the_block() {
        script(&[(0, Then::Nothing), (499, Then::Nothing)], false, false);
        assert_eq!(remaining(0), 1);
        assert_eq!(remaining(0), 500);
        script(
            &[(0x3FF, Then::Nothing), (0x3FF, Then::Nothing)],
            false,
            false,
        );
        assert_eq!(remaining(0), RX_BUF_LEN);
        assert_eq!(remaining_in(0, SWO_BLOCK_LEN), SWO_BLOCK_LEN);
    }

    /// A lap counts only where the counter jumped up across the flag read.
    #[test]
    fn a_lap_is_reported_only_where_the_counter_reloaded() {
        script(&[(599, Then::Nothing), (499, Then::Nothing)], false, false);
        assert_eq!(sample_rx(0), (500, false, false));
        script(&[(9, Then::Nothing), (999, Then::Nothing)], true, false);
        assert_eq!(sample_rx(0), (1000, true, false));
        // A flag with no reload between the reads belongs to a completion already folded in.
        script(&[(999, Then::Nothing), (989, Then::Nothing)], true, false);
        assert_eq!(sample_rx(0), (990, false, false));
        assert!(!peek_lap(0), "the flag is consumed either way");
    }

    /// A completion landing mid-read forces a fresh pair; one that never stops gives up as no lap.
    #[test]
    fn a_lap_landing_mid_read_is_re_read_and_exhaustion_reports_no_lap() {
        script(
            &[
                (9, Then::Nothing),
                (4, Then::Flag),
                (999, Then::Nothing),
                (997, Then::Nothing),
            ],
            false,
            true,
        );
        assert_eq!(sample_rx(0), (998, false, true));
        assert_eq!(unread(), 0);

        let mut storm = std::vec::Vec::new();
        for _ in 0..LAP_SETTLE_TRIES {
            storm.extend([(9, Then::Nothing), (999, Then::Flag)]);
        }
        storm.push((899, Then::Nothing));
        script(&storm, true, false);
        assert_eq!(sample_rx(0), (900, false, false));
        assert_eq!(unread(), 0);
    }

    fn chain(counts: &[(u32, Then)], lap: bool, blocks: u64, last: u64) -> u64 {
        let completed: &'static Cell<u64> = Box::leak(Box::new(Cell::new(blocks)));
        script(counts, lap, false);
        FAKE.with(|f| f.borrow_mut().completed = Some(completed));
        let (abs, errored) = sample_rx_chain(0, || completed.get(), last);
        assert!(!errored);
        assert_eq!(unread(), 0);
        abs
    }

    /// The absolute count pairs the block count with the offset read between two equal loads.
    #[test]
    fn a_chain_position_pairs_the_block_count_with_a_stable_offset() {
        assert_eq!(chain(&[(99, Then::Nothing)], false, 3, 0), 1948);
        // Counted mid-read: 3 * 512 + 507 would misplace the cursor by a whole block.
        let counts = [(4, Then::Irq), (499, Then::Nothing)];
        assert_eq!(chain(&counts, false, 3, 0), 4 * 512 + 12);
    }

    /// A completion that stays pending falls back to whole blocks, and never below the last report.
    #[test]
    fn an_unsettled_chain_falls_back_to_whole_blocks_and_never_steps_back() {
        let pending = [(4, Then::Nothing); LAP_SETTLE_TRIES];
        assert_eq!(chain(&pending, true, 3, 0), 1536);
        assert_eq!(chain(&pending, true, 3, 1600), 1600);
    }

    #[test]
    fn wide_chain_crosses_old_byte_and_block_rollovers_without_a_poll_deadline() {
        for blocks in [1u64 << 23, 1u64 << 32, 1u64 << 48] {
            let boundary = blocks * SWO_BLOCK_LEN as u64;
            assert_eq!(
                chain(&[(499, Then::Nothing)], false, blocks, boundary - 16),
                boundary + 12
            );
            assert_eq!(
                chain(&[(499, Then::Nothing)], false, blocks, boundary + 12),
                boundary + 12
            );
            let reads = [(4, Then::Irq), (499, Then::Nothing)];
            assert_eq!(
                chain(&reads, false, blocks - 1, boundary - 16),
                boundary + 12
            );
            let pending = [(4, Then::Nothing); LAP_SETTLE_TRIES];
            assert_eq!(chain(&pending, true, blocks, boundary - 16), boundary);
            assert_eq!(chain(&pending, true, blocks, boundary + 12), boundary + 12);
        }
        assert_eq!(
            chain(&[(499, Then::Nothing)], false, 1 << 32, 12),
            (1u64 << 41) + 12
        );
    }

    #[test]
    fn completion_at_each_epoch_load_never_accepts_a_mixed_pair() {
        for when in 0..4 {
            let blocks = Cell::new((1u64 << 32) - 1);
            let loads = Cell::new(0);
            let boundary = (1u64 << 32) * SWO_BLOCK_LEN as u64;
            let expected = if when == 3 {
                boundary - 5
            } else {
                boundary + 12
            };
            let reads: &[(u32, Then)] = if when == 0 {
                &[(499, Then::Nothing)]
            } else if when == 3 {
                &[(4, Then::Nothing)]
            } else {
                &[(4, Then::Nothing), (499, Then::Nothing)]
            };
            script(reads, false, true);
            let (position, error) = sample_rx_chain(
                0,
                || {
                    let point = loads.get();
                    loads.set(point + 2);
                    if point == when {
                        blocks.set(blocks.get() + 1);
                    }
                    let value = blocks.get();
                    if point + 1 == when {
                        blocks.set(blocks.get() + 1);
                    }
                    value
                },
                boundary - 16,
            );
            assert_eq!(position, expected, "load boundary {when}");
            assert!(error);
            assert!(!take_error(0));
            assert_eq!(unread(), 0);
        }
    }

    #[test]
    fn advisory_sampling_does_not_consume_errors_or_completions() {
        script(&[(4, Then::Nothing); LAP_SETTLE_TRIES], true, true);
        assert_eq!(chain_position(0, || 1 << 32, 0), 1 << 41);
        assert_eq!(unread(), 0);
        assert!(peek_lap(0));
        assert!(take_error(0));
        assert!(!take_error(0));
    }

    /// Each descriptor fills the next block and the last links back to the first.
    #[test]
    fn the_trace_chain_closes_on_its_first_descriptor() {
        let mut slot = Box::new(SwoSlot::zeroed());
        let slot: *mut SwoSlot = &mut *slot;
        // SAFETY: a host allocation no controller reads.
        let slot = unsafe {
            link_chain(slot, 0x4008_B030);
            &*slot
        };
        for (i, d) in slot.descs.iter().enumerate() {
            assert_eq!((d.xfercfg, d.src_end), (SWO_XFERCFG, 0x4008_B030));
            let end = &slot.buf[(i + 1) * SWO_BLOCK_LEN - 1];
            assert_eq!(d.dst_end, end as *const u8 as u32);
            let next = &slot.descs[(i + 1) % SWO_BLOCKS];
            assert_eq!(d.next, next as *const Descriptor as u32);
        }
    }

    /// A transmit runs once from an incrementing source into the fixed FIFO word.
    #[test]
    fn the_transmit_word_is_one_shot_into_a_fixed_destination() {
        for len in [1, 2, 512, TX_MAX_XFER] {
            let w = tx_xfercfg(len);
            assert_eq!((w >> 16) & 0x3FF, len as u32 - 1);
            assert_eq!(w & 0b1_0011, 0b1_0001, "CFGVALID and SETINTA, no RELOAD");
            assert_eq!((w >> 12) & 0b11, 0b01, "SRCINC one byte");
            assert_eq!((w >> 14) & 0b11, 0, "DSTINC none");
        }
    }

    /// `XFERCOUNT` is 10 bits holding one less than the length, so every block must fit it.
    #[test]
    fn every_armed_block_fits_the_ten_bit_transfer_count() {
        for block in [RX_BUF_LEN, SWO_BLOCK_LEN, TX_MAX_XFER] {
            assert!(block >= 1);
            assert!(
                block - 1 <= 0x3FF,
                "block {block} does not fit XFERCOUNT's ten bits"
            );
        }
        // 1024 and an exhausted descriptor share an encoding, so 1023 is the ceiling.
        assert_eq!(TX_MAX_XFER, 0x3FF);
        assert_eq!(RX_BUF_LEN, TX_MAX_XFER);
    }

    /// The two receive configurations differ only in the block they count down from.
    #[test]
    fn the_receive_transfer_words_encode_one_less_than_their_block() {
        assert_eq!((RX_XFERCFG >> 16) & 0x3FF, RX_BUF_LEN as u32 - 1);
        assert_eq!((SWO_XFERCFG >> 16) & 0x3FF, SWO_BLOCK_LEN as u32 - 1);
        assert_eq!(RX_XFERCFG & 0xFFFF, SWO_XFERCFG & 0xFFFF);
        // CFGVALID, RELOAD and SETINTA, with CLRTRIG left clear so the trigger survives a reload.
        assert_eq!(RX_XFERCFG & 0b1_0011, 0b1_0011);
        assert_eq!(RX_XFERCFG & (1 << 2), 0, "CLRTRIG must stay clear");
    }

    /// The transmit ring wraps by masking, and no single transfer can cover a whole lap.
    #[test]
    fn the_transmit_ring_wraps_by_masking_and_needs_more_than_one_transfer_per_lap() {
        assert!(TX_RING_LEN.is_power_of_two());
        const { assert!(TX_RING_LEN > TX_MAX_XFER) };
        // Two maximum transfers do not quite cover a lap, so a full ring is always three.
        assert_eq!(TX_RING_LEN.div_ceil(TX_MAX_XFER), 3);
        for cursor in [0usize, 1, TX_RING_LEN - 1, TX_RING_LEN, usize::MAX] {
            assert_eq!(cursor & (TX_RING_LEN - 1), cursor % TX_RING_LEN);
        }
    }

    /// The trace ring is a whole number of blocks, so the chain closes on a block boundary.
    #[test]
    fn the_trace_ring_is_a_whole_number_of_blocks() {
        assert_eq!(SWO_BUF_LEN, SWO_BLOCKS * SWO_BLOCK_LEN);
        assert!(SWO_BLOCK_LEN.is_power_of_two());
    }
}
