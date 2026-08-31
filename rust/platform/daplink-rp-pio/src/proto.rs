//! SWD wire framing: request byte, data parity, ACK decode.

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

/// A decoded SWD acknowledge.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Ack {
    /// `0b001`: the target accepted the transfer.
    Ok,
    /// `0b010`: the target is busy; the transfer must be retried.
    Wait,
    /// `0b100`: a sticky fault is set and the transfer was not performed.
    Fault,
    /// `0b111`: no target responded, or the line is in protocol error.
    NoAck,
    /// Any other three-bit value; the protocol defines none.
    Unknown(u8),
}

impl Ack {
    /// Decode the three acknowledge bits; `0b111` is defined on writes as well as reads.
    pub const fn from_bits(bits: u8) -> Ack {
        match bits {
            0b001 => Ack::Ok,
            0b010 => Ack::Wait,
            0b100 => Ack::Fault,
            0b111 => Ack::NoAck,
            other => Ack::Unknown(other),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_byte_has_start_park_and_even_parity() {
        for (ap, read, addr) in [
            (false, true, 0u8),
            (false, false, 2),
            (true, true, 1),
            (true, false, 3),
        ] {
            let b = request_byte(ap, read, addr);
            assert_eq!(b & 1, 1, "start bit");
            assert_eq!(b >> 7, 1, "park bit");
            assert_eq!((b & 0b0011_1110).count_ones() % 2, 0, "even parity");
        }
    }

    #[test]
    fn parity32_counts_set_bits() {
        assert_eq!(parity32(0), 0);
        assert_eq!(parity32(1), 1);
        assert_eq!(parity32(0b11), 0);
        assert_eq!(parity32(u32::MAX), 0);
    }

    #[test]
    fn ack_decoding_skips_the_turnaround_bits() {
        for trn in 1..=4u8 {
            for bits in [0b001u32, 0b010, 0b100, 0b111] {
                assert_eq!(ack_from_frame(bits << trn, trn), bits as u8);
            }
        }
        // Bits above the ACK -- a write frame's trailing turnaround -- are discarded too.
        assert_eq!(ack_from_frame(0b10101, 1), 0b010);
        assert_eq!(Ack::from_bits(0b001), Ack::Ok);
        assert_eq!(Ack::from_bits(0b000), Ack::Unknown(0));
    }
}
