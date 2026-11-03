//! The const PIO instruction encoder: one function per instruction form the programs use.

/// A PIO instruction destination (the `out`/`set` target field, bits 7:5).
#[derive(Copy, Clone)]
pub(crate) enum Dest {
    Pins = 0,
    X = 1,
    Y = 2,
    PinDirs = 4,
    Pc = 5,
    #[cfg(any(test, feature = "rp2040", feature = "rp235x"))]
    Exec = 7,
}

/// A PIO `jmp` condition (bits 7:5 of a JMP instruction).
#[derive(Copy, Clone)]
pub(crate) enum Cond {
    Always = 0,
    XPostDec = 2,
    YZero = 3,
    #[cfg(any(test, feature = "rp2040", feature = "rp235x"))]
    XNotY = 5,
}

/// How the 5-bit delay/side-set field splits for a variant.
#[derive(Copy, Clone)]
pub(crate) struct SideSet {
    /// Side-set data bits; an enable bit is always present because both variants use `opt`.
    pub(crate) data_bits: u8,
}

impl SideSet {
    /// Total side-set field width, including the `opt` enable bit.
    const fn total_bits(self) -> u8 {
        self.data_bits + 1
    }

    /// Encode the 5-bit `[12:8]` delay/side-set field; `side` is `None` to leave the pins alone.
    const fn field(self, side: Option<u8>, delay: u8) -> u16 {
        let total = self.total_bits();
        let delay_bits = 5 - total;
        assert!(delay < 1 << delay_bits);
        let (enable, value) = match side {
            Some(v) => (1u16, v as u16),
            None => (0, 0),
        };
        // Side-set data sits just below the enable bit; delay fills the low bits.
        let mut field = (delay as u16) & ((1 << delay_bits) - 1);
        field |= value << delay_bits;
        field |= enable << (total - 1) << delay_bits;
        field
    }
}

/// Assemble one instruction: opcode in 15:13, delay/side-set in 12:8, opcode-specific low byte.
const fn instr(opcode: u16, field: u16, low: u16) -> u16 {
    (opcode << 13) | (field << 8) | low
}

const OP_JMP: u16 = 0b000;
const OP_IN: u16 = 0b010;
pub(crate) const OP_OUT: u16 = 0b011;
const OP_PUSHPULL: u16 = 0b100;
const OP_MOV: u16 = 0b101;

pub(crate) const fn jmp(ss: SideSet, cond: Cond, addr: u8, side: Option<u8>, delay: u8) -> u16 {
    instr(
        OP_JMP,
        ss.field(side, delay),
        ((cond as u16) << 5) | (addr as u16),
    )
}
pub(crate) const fn out(ss: SideSet, dest: Dest, count: u8, side: Option<u8>, delay: u8) -> u16 {
    // A count of 32 encodes as 0; every count here is below 32.
    instr(
        OP_OUT,
        ss.field(side, delay),
        ((dest as u16) << 5) | (count as u16),
    )
}
pub(crate) const fn in_pins(ss: SideSet, count: u8, side: Option<u8>, delay: u8) -> u16 {
    // Source = PINS (0).
    instr(OP_IN, ss.field(side, delay), count as u16)
}
pub(crate) const fn pull(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    // PULL, blocking: bit 7 = 1 (pull), bit 5 = 1 (block).
    instr(OP_PUSHPULL, ss.field(side, delay), 0b1010_0000)
}
pub(crate) const fn push(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    // PUSH, blocking: bit 7 = 0 (push), bit 5 = 1 (block).
    instr(OP_PUSHPULL, ss.field(side, delay), 0b0010_0000)
}
pub(crate) const fn nop(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    // Canonical NOP = `mov y, y`.
    instr(OP_MOV, ss.field(side, delay), (2 << 5) | 2)
}

pub(crate) const fn clear_isr(ss: SideSet) -> u16 {
    // `mov isr, null` clears both the ISR and its input shift counter.
    mov_isr_null(ss, None, 0)
}

pub(crate) const fn mov_isr_null(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    // `mov isr, null` clears both the ISR and its input shift counter.
    instr(OP_MOV, ss.field(side, delay), (6 << 5) | 3)
}

pub(crate) const fn mov_y_to_pins(ss: SideSet, side: Option<u8>, delay: u8) -> u16 {
    instr(OP_MOV, ss.field(side, delay), 2)
}

#[cfg(any(test, feature = "rp2040", feature = "rp235x"))]
pub(crate) const fn mov_reverse_isr_to_osr(ss: SideSet) -> u16 {
    // `mov osr, ::isr`: destination OSR, reverse operation, source ISR.
    instr(OP_MOV, ss.field(None, 0), (7 << 5) | (2 << 3) | 6)
}

#[cfg(any(test, feature = "rp2040", feature = "rp235x"))]
pub(crate) const fn irq(ss: SideSet, index: u8) -> u16 {
    assert!(index < 8);
    instr(0b110, ss.field(None, 0), index as u16)
}

pub(crate) const fn set(ss: SideSet, dest: Dest, data: u8, side: Option<u8>, delay: u8) -> u16 {
    assert!(data < 32);
    instr(
        0b111,
        ss.field(side, delay),
        ((dest as u16) << 5) | data as u16,
    )
}
