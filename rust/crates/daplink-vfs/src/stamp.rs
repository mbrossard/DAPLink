//! Owned boot reports and disjoint writers for retained reset state.

use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering},
};

use crate::files::{FAULT_MAX, STATUS_MAX};

/// Consecutive faults that request reset rather than halt.
pub const FAULT_RESET_LIMIT: u32 = 3;
const MAGIC: u32 = 0x6472_7375;
/// Size in bytes of the v1 retained record.
pub const RETAINED_SIZE: usize = 136;

#[repr(C)]
struct RawStamp {
    magic: u32,
    reset_count: u32,
    failed: u32,
    status_len: u32,
    status: [u8; STATUS_MAX],
    fault_streak: u32,
    fault_len: u32,
    fault: [u8; FAULT_MAX],
}

const _: () = {
    assert!(core::mem::size_of::<RawStamp>() == RETAINED_SIZE);
    assert!(core::mem::align_of::<RawStamp>() == 4);
    assert!(core::mem::align_of::<AtomicU32>() <= 4);
    assert!(core::mem::size_of::<AtomicU32>() == 4);
    assert!(core::mem::offset_of!(RawStamp, reset_count) == 4);
    assert!(core::mem::offset_of!(RawStamp, failed) == 8);
    assert!(core::mem::offset_of!(RawStamp, status_len) == 12);
    assert!(core::mem::offset_of!(RawStamp, status) == 16);
    assert!(core::mem::offset_of!(RawStamp, fault_streak) == 80);
    assert!(core::mem::offset_of!(RawStamp, fault_len) == 84);
    assert!(core::mem::offset_of!(RawStamp, fault) == 88);
};

/// Initialized, exclusively borrowed storage; never registered with the terminal handlers.
pub struct InitializedStamp(UnsafeCell<RawStamp>);

impl InitializedStamp {
    /// Create initialized storage without an accepted stamp.
    pub fn zeroed() -> Self {
        Self::from_bytes([0; RETAINED_SIZE])
    }

    /// Decode initialized v1 bytes; boot validates their magic and lengths.
    pub fn from_bytes(bytes: [u8; RETAINED_SIZE]) -> Self {
        let word = |i| u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
        let mut status = [0; STATUS_MAX];
        let mut fault = [0; FAULT_MAX];
        status.copy_from_slice(&bytes[16..80]);
        fault.copy_from_slice(&bytes[88..136]);
        Self(UnsafeCell::new(RawStamp {
            magic: word(0),
            reset_count: word(4),
            failed: word(8),
            status_len: word(12),
            status,
            fault_streak: word(80),
            fault_len: word(84),
            fault,
        }))
    }

    /// Advance the boot count and split ownership until all tokens are dropped.
    pub fn boot(&mut self) -> Boot<'_> {
        // SAFETY: initialized storage remains exclusively borrowed until every token is dropped.
        unsafe { boot(self.0.get()) }
    }
}

/// A copied report; its slices never refer to live retained memory.
pub struct BootSnapshot(RawStamp);

impl BootSnapshot {
    /// Reset count, wrapping since the last boot without accepted magic.
    pub fn reset_count(&self) -> u32 {
        self.0.reset_count
    }
    /// Faults recorded since the last steady-state clear.
    pub fn fault_streak(&self) -> u32 {
        self.0.fault_streak
    }
    /// Previous transfer text and failed flag, absent for an empty or interrupted record.
    pub fn status(&self) -> Option<(&[u8], bool)> {
        (self.0.status_len != 0).then(|| {
            (
                &self.0.status[..self.0.status_len as usize],
                self.0.failed != 0,
            )
        })
    }
    /// Previous complete fault reason, if any.
    pub fn fault(&self) -> Option<&[u8]> {
        (self.0.fault_len != 0).then(|| &self.0.fault[..self.0.fault_len as usize])
    }
}

/// Owned boot report and non-cloneable permissions over disjoint retained fields.
pub struct Boot<'a> {
    /// Immutable copy taken before fault recording is armed.
    pub snapshot: BootSnapshot,
    /// The sole transfer-status writer.
    pub status: StatusWriter<'a>,
    /// One-shot steady-state milestone, spent by the volume's uptime countdown.
    pub steady: SteadyToken<'a>,
    /// The sole terminal-fault endpoint.
    pub fault: FaultEndpoint<'a>,
}

struct Region<'a> {
    ptr: *mut RawStamp,
    lifetime: PhantomData<&'a UnsafeCell<RawStamp>>,
}

/// The sole writer of retained transfer status, independent of fault recording.
pub struct StatusWriter<'a> {
    region: Region<'a>,
    reset_count: u32,
}

// SAFETY: this non-cloneable token writes only its disjoint status fields; it is not Sync.
unsafe impl Send for StatusWriter<'_> {}

impl StatusWriter<'_> {
    /// Count copied at boot; does not read live retained memory.
    pub fn reset_count(&self) -> u32 {
        self.reset_count
    }
    /// Invalidate, write at most 64 status bytes, then commit the text and outcome.
    pub fn store(&mut self, status: &[u8], failed: bool) {
        // SAFETY: this token exclusively owns the status fields for the region lifetime.
        unsafe { write_status(self.region.ptr, status, failed, &mut || {}) }
    }
}

/// Clears only the atomic fault streak once the firmware has stayed up past a mount.
pub struct SteadyToken<'a>(Region<'a>);

impl SteadyToken<'_> {
    /// Clear the streak without clearing the retained fault reason.
    pub fn reached(self) {
        // SAFETY: boot initializes this aligned field before any atomic view exists.
        unsafe { streak(self.0.ptr).store(0, Ordering::SeqCst) }
    }
}

/// May be transferred to the terminal handlers, but never used as a returning global writer.
pub struct FaultEndpoint<'a>(Region<'a>);

/// Admit externally initialized retained SRAM before creating or publishing any writers.
/// # Safety
/// `p` must exclusively own 136 readable initialized bytes, aligned to 4, for this entire boot.
/// Exclude runtime clearing, stack/heap/DMA/other-core access; uninitialized Rust storage is invalid.
pub unsafe fn boot_retained(p: *mut u8) -> Boot<'static> {
    boot(p.cast())
}

unsafe fn boot<'a>(p: *mut RawStamp) -> Boot<'a> {
    if ptr::addr_of!((*p).magic).read_volatile() == MAGIC {
        let count = ptr::addr_of!((*p).reset_count)
            .read_volatile()
            .wrapping_add(1);
        ptr::addr_of_mut!((*p).reset_count).write_volatile(count);
        let failed = u32::from(ptr::addr_of!((*p).failed).read_volatile() != 0);
        ptr::addr_of_mut!((*p).failed).write_volatile(failed);
        for (len, max) in [
            (ptr::addr_of_mut!((*p).status_len), STATUS_MAX),
            (ptr::addr_of_mut!((*p).fault_len), FAULT_MAX),
        ] {
            if len.read_volatile() as usize > max {
                len.write_volatile(0);
            }
        }
    } else {
        for i in 0..RETAINED_SIZE {
            p.cast::<u8>().add(i).write_volatile(0);
        }
        ptr::addr_of_mut!((*p).magic).write_volatile(MAGIC);
    }
    let snapshot = BootSnapshot(p.read_volatile());
    let region = || Region {
        ptr: p,
        lifetime: PhantomData,
    };
    Boot {
        status: StatusWriter {
            region: region(),
            reset_count: snapshot.reset_count(),
        },
        snapshot,
        steady: SteadyToken(region()),
        fault: FaultEndpoint(region()),
    }
}

unsafe fn streak<'a>(p: *mut RawStamp) -> &'a AtomicU32 {
    AtomicU32::from_ptr(ptr::addr_of_mut!((*p).fault_streak))
}

unsafe fn write_status(p: *mut RawStamp, status: &[u8], failed: bool, step: &mut impl FnMut()) {
    ptr::addr_of_mut!((*p).status_len).write_volatile(0);
    step();
    let n = status.len().min(STATUS_MAX);
    for (i, &b) in status[..n].iter().enumerate() {
        ptr::addr_of_mut!((*p).status)
            .cast::<u8>()
            .add(i)
            .write_volatile(b);
        step();
    }
    ptr::addr_of_mut!((*p).failed).write_volatile(u32::from(failed));
    step();
    ptr::addr_of_mut!((*p).status_len).write_volatile(n as u32);
    step();
}

static LATCH: AtomicPtr<RawStamp> = AtomicPtr::new(ptr::null_mut());
static RECORDING: AtomicBool = AtomicBool::new(false);

/// Publish the sole fault endpoint once, before terminal handlers can record into it.
/// # Safety
/// Call once per boot on one core with non-returning handlers; exclude DMA and other-core writers.
pub unsafe fn arm_terminal_faults(endpoint: FaultEndpoint<'static>) {
    LATCH.store(endpoint.0.ptr, Ordering::Release);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// Terminal action after attempting to record a fault.
pub enum FaultAction {
    /// Request a system reset.
    Reset,
    /// Stop without returning to interrupted code.
    Halt,
}

impl FaultAction {
    /// Whether the terminal handler should request reset.
    pub fn resets(self) -> bool {
        matches!(self, Self::Reset)
    }
}

#[doc(hidden)]
/// # Safety
/// Mask configurable interrupts first; reset or halt without returning, on the sole fault-writing core.
pub unsafe fn terminal_panic(location: Option<(&str, u32)>) -> FaultAction {
    terminal(
        LATCH.load(Ordering::Acquire),
        &RECORDING,
        |reason| reason.write_panic(location),
        &mut |_| {},
    )
}

#[doc(hidden)]
/// # Safety
/// Mask configurable interrupts first; reset or halt without returning, on the sole fault-writing core.
pub unsafe fn terminal_hard_fault(pc: u32) -> FaultAction {
    terminal(
        LATCH.load(Ordering::Acquire),
        &RECORDING,
        |reason| reason.write_hard_fault(pc),
        &mut |_| {},
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    BeforeClaim,
    Unclaimed,
    Claimed,
    Streak,
    Invalidated,
    Byte,
    Committed,
}

unsafe fn terminal(
    p: *mut RawStamp,
    recording: &AtomicBool,
    build_reason: impl FnOnce(&mut Reason),
    step: &mut impl FnMut(Step),
) -> FaultAction {
    if p.is_null() {
        return FaultAction::Reset;
    }
    step(Step::BeforeClaim);
    if recording.load(Ordering::SeqCst) {
        return FaultAction::Halt;
    }
    step(Step::Unclaimed);
    // A preempting claimant never returns, so the suspended handler cannot also claim.
    recording.store(true, Ordering::SeqCst);
    step(Step::Claimed);
    let count = streak(p).load(Ordering::Relaxed).saturating_add(1);
    streak(p).store(count, Ordering::SeqCst);
    step(Step::Streak);
    ptr::addr_of_mut!((*p).fault_len).write_volatile(0);
    step(Step::Invalidated);
    let mut reason = Reason::new();
    build_reason(&mut reason);
    for (i, &b) in reason.written().iter().enumerate() {
        ptr::addr_of_mut!((*p).fault)
            .cast::<u8>()
            .add(i)
            .write_volatile(b);
        step(Step::Byte);
    }
    ptr::addr_of_mut!((*p).fault_len).write_volatile(reason.len as u32);
    step(Step::Committed);
    if count > FAULT_RESET_LIMIT {
        FaultAction::Halt
    } else {
        FaultAction::Reset
    }
}

struct Reason {
    buf: [u8; FAULT_MAX],
    len: usize,
}

impl Reason {
    fn new() -> Self {
        Self {
            buf: [0; FAULT_MAX],
            len: 0,
        }
    }
    fn write_panic(&mut self, location: Option<(&str, u32)>) {
        self.bytes(b"panic");
        if let Some((file, line)) = location {
            self.bytes(b" ");
            let mut digits = [0; 10];
            let line = dec(&mut digits, line);
            let room = FAULT_MAX.saturating_sub(self.len + 1 + line.len());
            let bytes = file.as_bytes();
            self.bytes(&bytes[bytes.len().saturating_sub(room)..]);
            self.bytes(b":");
            self.bytes(line);
        }
    }
    fn write_hard_fault(&mut self, pc: u32) {
        self.bytes(b"hardfault pc=");
        let mut digits = [0; 8];
        self.bytes(hex(&mut digits, pc));
    }
    #[cfg(test)]
    fn panic(location: Option<(&str, u32)>) -> Self {
        let mut r = Self::new();
        r.write_panic(location);
        r
    }
    #[cfg(test)]
    fn hard_fault(pc: u32) -> Self {
        let mut r = Self::new();
        r.write_hard_fault(pc);
        r
    }
    fn bytes(&mut self, s: &[u8]) {
        let n = s.len().min(FAULT_MAX - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s[..n]);
        self.len += n;
    }
    fn written(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

fn dec(buf: &mut [u8; 10], mut v: u32) -> &[u8] {
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 || i == 0 {
            break;
        }
    }
    &buf[i..]
}

fn hex(buf: &mut [u8; 8], v: u32) -> &[u8] {
    for (i, b) in buf.iter_mut().enumerate() {
        let nibble = (v >> (28 - 4 * i)) & 0xf;
        *b = match nibble {
            0..=9 => b'0' + nibble as u8,
            _ => b'a' + (nibble - 10) as u8,
        };
    }
    &buf[..]
}

/// Reserve the v1 record in the runtime's NOLOAD region; invoke once in an Arm firmware root.
#[macro_export]
macro_rules! retained_stamp {
    () => {
        ::core::arch::global_asm!(
            ".pushsection .uninit.RESET_STAMP,\"aw\",%nobits",
            ".balign 4",
            ".global __daplink_retained_stamp",
            ".type __daplink_retained_stamp,%object",
            "__daplink_retained_stamp:",
            ".space {size}",
            ".size __daplink_retained_stamp, .-__daplink_retained_stamp",
            ".popsection",
            size = const $crate::stamp::RETAINED_SIZE,
        );
        extern "C" { static mut __daplink_retained_stamp: [u8; $crate::stamp::RETAINED_SIZE]; }
    };
}

#[cfg(test)]
mod tests;
