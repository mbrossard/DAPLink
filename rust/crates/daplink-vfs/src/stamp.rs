//! Reset-surviving probe state: the boot counter, the last transfer status, and a fault latch.

use core::sync::atomic::{AtomicPtr, Ordering};

use crate::files::{FAULT_MAX, STATUS_MAX};

/// Consecutive faults that still ask for a reset; past this the latch asks the fault path to halt
/// instead, so a deterministically faulting probe stops cycling on the bus.
pub const FAULT_RESET_LIMIT: u32 = 3;

/// Reset-surviving stamp: magic + boot counter + the last transfer status + the fault latch.
#[repr(C)]
pub struct ResetStamp {
    magic: u32,
    reset_count: u32,
    /// 0 = last transfer succeeded or none yet, 1 = failed. A `u32`, not a `bool`, so every
    /// surviving bit pattern is a valid value; normalized by `boot_in_place`.
    failed: u32,
    status_len: u32,
    status: [u8; STATUS_MAX],
    /// Faults latched since the last [`clear_fault_streak`], the reset-loop bound.
    fault_streak: u32,
    fault_len: u32,
    fault: [u8; FAULT_MAX],
}

impl ResetStamp {
    /// The validity marker `boot_in_place` stamps and checks. Bump it whenever the layout below
    /// changes, so an older image's stamp reads as power-on rather than as misplaced fields.
    const MAGIC: u32 = 0x6472_7375;

    /// Validate and advance the stamp in place, returning it initialized: a valid magic is a
    /// reset, anything else a power-on. Call once, as early in boot as the image allows.
    pub fn boot_in_place(
        slot: &'static mut core::mem::MaybeUninit<ResetStamp>,
    ) -> &'static mut ResetStamp {
        let p = slot.as_mut_ptr();
        // SAFETY: every field is a plain integer or byte array for which any bit pattern is valid,
        // and the power-on branch initializes every byte before a `&mut` is handed out.
        unsafe {
            if core::ptr::addr_of!((*p).magic).read_volatile() == Self::MAGIC {
                let count = core::ptr::addr_of!((*p).reset_count)
                    .read_volatile()
                    .wrapping_add(1);
                core::ptr::addr_of_mut!((*p).reset_count).write_volatile(count);
                let failed = u32::from(core::ptr::addr_of!((*p).failed).read_volatile() != 0);
                core::ptr::addr_of_mut!((*p).failed).write_volatile(failed);
                clamp_len(core::ptr::addr_of_mut!((*p).status_len), STATUS_MAX);
                clamp_len(core::ptr::addr_of_mut!((*p).fault_len), FAULT_MAX);
            } else {
                core::ptr::write_bytes(p as *mut u8, 0, core::mem::size_of::<ResetStamp>());
                core::ptr::addr_of_mut!((*p).magic).write_volatile(Self::MAGIC);
            }
            LATCH.store(p, Ordering::Release);
            slot.assume_init_mut()
        }
    }

    /// Number of probe resets since power-on (0 on the power-on boot itself).
    pub fn reset_count(&self) -> u32 {
        self.reset_count
    }

    /// The latched status text and failed flag, if a transfer has completed since power-on.
    pub fn status(&self) -> Option<(&[u8], bool)> {
        if self.status_len == 0 {
            None
        } else {
            Some((&self.status[..self.status_len as usize], self.failed != 0))
        }
    }

    /// The reason latched by the fault that ended a previous boot; survives until a power cycle.
    pub fn fault(&self) -> Option<&[u8]> {
        if self.fault_len == 0 {
            None
        } else {
            Some(&self.fault[..self.fault_len as usize])
        }
    }

    /// Faults latched since the last [`clear_fault_streak`] (see [`FAULT_RESET_LIMIT`]).
    pub fn fault_streak(&self) -> u32 {
        self.fault_streak
    }

    /// Latch a transfer status (truncated to the status field width) and its failed flag.
    pub fn store(&mut self, status: &[u8], failed: bool) {
        let n = core::cmp::min(status.len(), STATUS_MAX);
        self.status[..n].copy_from_slice(&status[..n]);
        self.status_len = n as u32;
        self.failed = u32::from(failed);
    }
}

/// Zero a surviving text length that no longer indexes its buffer, so partial corruption drops
/// the text rather than exposing what follows it.
///
/// # Safety
///
/// `p` must point at a live `u32` length field.
unsafe fn clamp_len(p: *mut u32, max: usize) {
    if p.read_volatile() as usize > max {
        p.write_volatile(0);
    }
}

/// The stamp a fault may write to, armed by [`ResetStamp::boot_in_place`] and null until then. A
/// raw pointer: the owning `&mut` is long since handed to the volume, and no reference is formed.
static LATCH: AtomicPtr<ResetStamp> = AtomicPtr::new(core::ptr::null_mut());

/// What a fault handler should do once the reason is latched.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FaultAction {
    /// Reset the part: the probe re-enumerates and reports the reason in `DETAILS.TXT`.
    Reset,
    /// Stop instead, because faults have kept recurring (see [`FAULT_RESET_LIMIT`]).
    Halt,
}

impl FaultAction {
    /// Whether the part should be reset.
    pub fn resets(self) -> bool {
        matches!(self, FaultAction::Reset)
    }
}

/// Record that this boot reached its steady state, clearing the fault streak. Call once, after
/// the firmware has composed itself; a no-op before a stamp is armed.
pub fn clear_fault_streak() {
    let p = LATCH.load(Ordering::Acquire);
    if p.is_null() {
        return;
    }
    // SAFETY: `p` came from `boot_in_place` on a `'static` slot; this writes one field whose
    // every bit pattern is valid, and forms no reference (see [`LATCH`]).
    unsafe {
        if core::ptr::addr_of!((*p).magic).read_volatile() == ResetStamp::MAGIC {
            core::ptr::addr_of_mut!((*p).fault_streak).write_volatile(0);
        }
    }
}

/// Latch a panic, optionally naming the source location, and return what the handler should do.
/// Never the message: rendering it needs the core formatting machinery.
pub fn latch_panic(location: Option<(&str, u32)>) -> FaultAction {
    let mut r = Reason::new();
    r.bytes(b"panic");
    if let Some((file, line)) = location {
        r.bytes(b" ");
        let mut digits = [0u8; 10];
        let line = dec(&mut digits, line);
        // The tail holds the file name and crate directory; the head is shared by every site.
        let room = FAULT_MAX.saturating_sub(r.len + 1 + line.len());
        r.bytes(tail(file.as_bytes(), room));
        r.bytes(b":");
        r.bytes(line);
    }
    latch(r.written())
}

/// Latch a hard fault's stacked program counter, returning what the handler should do next.
pub fn latch_hard_fault(pc: u32) -> FaultAction {
    let mut r = Reason::new();
    r.bytes(b"hardfault pc=");
    let mut digits = [0u8; 8];
    r.bytes(hex(&mut digits, pc));
    latch(r.written())
}

/// Write `reason` into the armed stamp and decide reset-or-halt. Unarmed, the reason is dropped
/// and the answer is still `Reset`: recovery is worth more than evidence.
fn latch(reason: &[u8]) -> FaultAction {
    let p = LATCH.load(Ordering::Acquire);
    if p.is_null() {
        return FaultAction::Reset;
    }
    // SAFETY: as `clear_fault_streak`; the magic check also rejects a destroyed slot.
    unsafe {
        if core::ptr::addr_of!((*p).magic).read_volatile() != ResetStamp::MAGIC {
            return FaultAction::Reset;
        }
        // Length last and cleared first: a fault mid-write leaves no reason, not a spliced one.
        core::ptr::addr_of_mut!((*p).fault_len).write_volatile(0);
        let dst = core::ptr::addr_of_mut!((*p).fault) as *mut u8;
        for (i, &b) in reason.iter().enumerate() {
            dst.add(i).write_volatile(b);
        }
        core::ptr::addr_of_mut!((*p).fault_len).write_volatile(reason.len() as u32);
        let streak = core::ptr::addr_of!((*p).fault_streak)
            .read_volatile()
            .saturating_add(1);
        core::ptr::addr_of_mut!((*p).fault_streak).write_volatile(streak);
        if streak > FAULT_RESET_LIMIT {
            FaultAction::Halt
        } else {
            FaultAction::Reset
        }
    }
}

/// A fixed-width cursor building the latch reason, silently dropping anything past the field.
struct Reason {
    buf: [u8; FAULT_MAX],
    len: usize,
}

impl Reason {
    fn new() -> Self {
        Self {
            buf: [0u8; FAULT_MAX],
            len: 0,
        }
    }

    fn bytes(&mut self, s: &[u8]) {
        let n = core::cmp::min(s.len(), FAULT_MAX - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s[..n]);
        self.len += n;
    }

    fn written(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// The last `n` bytes of `s` (all of it when shorter).
fn tail(s: &[u8], n: usize) -> &[u8] {
    &s[s.len().saturating_sub(n)..]
}

/// Render `v` as decimal into `buf`, returning the digits.
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

/// Render `v` as eight lowercase hex digits into `buf`.
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

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::MaybeUninit;

    fn leak_slot() -> &'static mut MaybeUninit<ResetStamp> {
        std::boxed::Box::leak(std::boxed::Box::new(MaybeUninit::zeroed()))
    }

    #[test]
    fn power_on_starts_fresh() {
        // Zeroed memory has no magic: power-on semantics.
        let s = ResetStamp::boot_in_place(leak_slot());
        assert_eq!(s.reset_count(), 0);
        assert!(s.status().is_none());
        assert!(s.fault().is_none());
        assert_eq!(s.fault_streak(), 0);
    }

    #[test]
    fn reset_advances_count_and_keeps_status() {
        let slot = leak_slot();
        let p = slot.as_mut_ptr();
        {
            let s = ResetStamp::boot_in_place(slot);
            s.store(b"Flash: 9984 bytes -> Done", false);
        }
        // SAFETY (test only): recreate the `&mut` after the first borrow ends, modelling a soft
        // reset that keeps the slot's contents.
        let slot2 = unsafe { &mut *(p as *mut MaybeUninit<ResetStamp>) };
        let s = ResetStamp::boot_in_place(slot2);
        assert_eq!(s.reset_count(), 1);
        let (text, failed) = s.status().unwrap();
        assert_eq!(text, b"Flash: 9984 bytes -> Done");
        assert!(!failed);
    }

    #[test]
    fn corrupt_length_is_clamped() {
        let slot = leak_slot();
        let p = slot.as_mut_ptr();
        {
            let s = ResetStamp::boot_in_place(slot);
            s.store(b"x", true);
        }
        // Corrupt both text lengths beyond their field widths: the next boot must clamp them.
        unsafe {
            core::ptr::addr_of_mut!((*p).status_len).write(10_000);
            core::ptr::addr_of_mut!((*p).fault_len).write(10_000);
        }
        let slot2 = unsafe { &mut *(p as *mut MaybeUninit<ResetStamp>) };
        let s = ResetStamp::boot_in_place(slot2);
        assert!(s.status().is_none());
        assert!(s.fault().is_none());
        assert_eq!(s.reset_count(), 1);
    }

    #[test]
    fn store_truncates_to_field_width() {
        let s = ResetStamp::boot_in_place(leak_slot());
        let long = [b'A'; 200];
        s.store(&long, true);
        let (text, failed) = s.status().unwrap();
        assert_eq!(text.len(), STATUS_MAX);
        assert!(failed);
    }

    #[test]
    fn decimal_and_hex_render_without_a_formatter() {
        let mut d = [0u8; 10];
        assert_eq!(dec(&mut d, 0), b"0");
        assert_eq!(dec(&mut d, 412), b"412");
        assert_eq!(dec(&mut d, u32::MAX), b"4294967295");
        let mut h = [0u8; 8];
        assert_eq!(hex(&mut h, 0x0800_a1b2), b"0800a1b2");
    }

    /// A long path loses its head, not the file name and line that identify the panic.
    #[test]
    fn a_long_panic_path_keeps_its_file_and_line() {
        let long = "/a/very/long/path/firmware/rp2040-pico-embassy/src/main.rs";
        let mut r = Reason::new();
        r.bytes(b"panic ");
        let mut digits = [0u8; 10];
        let line = dec(&mut digits, 412);
        let room = FAULT_MAX.saturating_sub(r.len + 1 + line.len());
        r.bytes(tail(long.as_bytes(), room));
        r.bytes(b":");
        r.bytes(line);
        let text = core::str::from_utf8(r.written()).unwrap();
        assert!(text.starts_with("panic "));
        assert!(text.ends_with("pico-embassy/src/main.rs:412"), "{text}");
        assert!(r.written().len() <= FAULT_MAX);
    }
}
