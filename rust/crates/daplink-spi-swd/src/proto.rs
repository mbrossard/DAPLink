//! SWD wire framing: request byte, parity, ACK decode, turnaround, bit timing.

/// Turnaround clocks between bus direction changes (the power-on `DLCR.TURNROUND` default).
pub const TRN: u8 = 1;

/// The 8-bit SWD request header, LSB first on the wire; `addr` carries `A[3:2]`.
pub const fn request_byte(ap: bool, read: bool, addr: u8) -> u8 {
    let req = 1 | ((ap as u8) << 1) | ((read as u8) << 2) | ((addr & 0x3) << 3) | (1 << 7);
    let parity = (req.count_ones() & 1) as u8;
    req | (parity << 5)
}

/// Even parity of a data word, as a single bit.
pub const fn parity32(v: u32) -> u8 {
    (v.count_ones() & 1) as u8
}

/// Extract the three ACK bits from a right-aligned acknowledge window sampled after `trn` clocks.
pub const fn ack_from_frame(raw: u32, trn: u8) -> u8 {
    ((raw >> trn) & 0b111) as u8
}

/// The write data phase as bytes: the four little-endian data bytes, then the parity byte.
pub const fn wdata_bytes(data: u32) -> [u8; 5] {
    let b = data.to_le_bytes();
    [b[0], b[1], b[2], b[3], parity32(data)]
}

/// Half-period spin count for a requested wire clock; rounds up (never overclocking the wire)
/// and floors at one cycle.
pub const fn half_period_cycles(cpu_hz: u32, max_frequency: u32) -> u32 {
    let hz = if max_frequency == 0 { 1 } else { max_frequency };
    let cycles = (cpu_hz / 2).div_ceil(hz);
    if cycles == 0 {
        1
    } else {
        cycles
    }
}
