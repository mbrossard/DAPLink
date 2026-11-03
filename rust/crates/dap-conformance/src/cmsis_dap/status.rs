//! The `DAP_Transfer` and `DAP_TransferBlock` status byte, decoded whole.

use std::fmt;

/// Bits 2:0 of a status byte that is not a protocol error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ack {
    /// 1: OK on SWD; OK or FAULT on JTAG.
    Ok,
    /// 2: WAIT.
    Wait,
    /// 4: FAULT.
    Fault,
    /// 7: no response from the target.
    NoAck,
    /// 0, 3, 5 or 6: no transfer ran, or an ACK outside the four, returned raw.
    Other(u8),
}

impl Ack {
    /// The three bits.
    pub fn bits(self) -> u8 {
        match self {
            Ack::Ok => 1,
            Ack::Wait => 2,
            Ack::Fault => 4,
            Ack::NoAck => 7,
            Ack::Other(b) => b & 7,
        }
    }

    fn from_bits(b: u8) -> Ack {
        match b & 7 {
            1 => Ack::Ok,
            2 => Ack::Wait,
            4 => Ack::Fault,
            7 => Ack::NoAck,
            b => Ack::Other(b),
        }
    }
}

/// A transfer status byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// The last response from the target, and bit 4, value mismatch.
    Ack {
        /// Bits 2:0.
        ack: Ack,
        /// Bit 4.
        mismatch: bool,
    },
    /// Bit 3, an SWD protocol error: bits 2:0 are not an ACK.
    ProtocolError {
        /// Bits 2:0, which mean nothing defined.
        raw_ack_bits: u8,
        /// Bit 4.
        mismatch: bool,
    },
    /// A byte no status can be: bits 7:5 set, or bit 4 in a block's status.
    Malformed(u8),
}

const PROTOCOL_ERROR: u8 = 1 << 3;
const MISMATCH: u8 = 1 << 4;

impl Status {
    /// The status byte of a `DAP_Transfer` reply.
    pub fn transfer(byte: u8) -> Status {
        if byte & 0xE0 != 0 {
            return Status::Malformed(byte);
        }
        let mismatch = byte & MISMATCH != 0;
        if byte & PROTOCOL_ERROR != 0 {
            Status::ProtocolError {
                raw_ack_bits: byte & 7,
                mismatch,
            }
        } else {
            Status::Ack {
                ack: Ack::from_bits(byte),
                mismatch,
            }
        }
    }

    /// The status byte of a `DAP_TransferBlock` reply, which has no value mismatch.
    pub fn block(byte: u8) -> Status {
        if byte & MISMATCH != 0 {
            return Status::Malformed(byte);
        }
        Status::transfer(byte)
    }

    /// The byte.
    pub fn byte(self) -> u8 {
        let flag = |set: bool, bit: u8| if set { bit } else { 0 };
        match self {
            Status::Ack { ack, mismatch } => ack.bits() | flag(mismatch, MISMATCH),
            Status::ProtocolError {
                raw_ack_bits,
                mismatch,
            } => PROTOCOL_ERROR | raw_ack_bits & 7 | flag(mismatch, MISMATCH),
            Status::Malformed(b) => b,
        }
    }

    /// The ACK, unless the byte is a protocol error or malformed.
    pub fn ack(self) -> Option<Ack> {
        match self {
            Status::Ack { ack, .. } => Some(ack),
            _ => None,
        }
    }

    /// OK without a value mismatch: every transfer the reply counts completed.
    pub fn is_ok(self) -> bool {
        self == Status::Ack {
            ack: Ack::Ok,
            mismatch: false,
        }
    }
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mismatch = match *self {
            Status::Ack { ack, mismatch } => {
                match ack {
                    Ack::Ok => f.write_str("OK")?,
                    Ack::Wait => f.write_str("WAIT")?,
                    Ack::Fault => f.write_str("FAULT")?,
                    Ack::NoAck => f.write_str("NO_ACK")?,
                    Ack::Other(b) => write!(f, "ACK {b:#05b}")?,
                }
                mismatch
            }
            Status::ProtocolError { mismatch, .. } => {
                f.write_str("protocol error")?;
                mismatch
            }
            Status::Malformed(b) => return write!(f, "malformed status {b:#04x}"),
        };
        if mismatch {
            f.write_str(", value mismatch")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ack(ack: Ack, mismatch: bool) -> Status {
        Status::Ack { ack, mismatch }
    }

    #[test]
    fn no_ack_is_seven() {
        assert_eq!(Status::transfer(0x07), ack(Ack::NoAck, false));
        assert_eq!(Status::transfer(0x07).to_string(), "NO_ACK");
    }

    #[test]
    fn a_parity_error_is_not_an_ack_of_zero() {
        let s = Status::transfer(0x08);
        assert_eq!(
            s,
            Status::ProtocolError {
                raw_ack_bits: 0,
                mismatch: false
            }
        );
        assert_eq!(s.ack(), None);
        assert!(!s.is_ok());
        assert_eq!(s.to_string(), "protocol error");
    }

    #[test]
    fn a_protocol_error_with_ack_bits_has_no_ack() {
        // dap-rs answers 0x0F for a parity error and for an ACK outside the four.
        let s = Status::transfer(0x0F);
        assert_eq!(
            s,
            Status::ProtocolError {
                raw_ack_bits: 7,
                mismatch: false
            }
        );
        assert_eq!(s.ack(), None);
        assert_eq!(Status::block(0x0F), s);
    }

    #[test]
    fn a_mismatch_comes_with_any_ack() {
        assert_eq!(Status::transfer(0x11), ack(Ack::Ok, true));
        assert_eq!(Status::transfer(0x12), ack(Ack::Wait, true));
        assert_eq!(Status::transfer(0x14), ack(Ack::Fault, true));
        assert!(!Status::transfer(0x11).is_ok());
        assert_eq!(Status::transfer(0x11).to_string(), "OK, value mismatch");
        assert_eq!(
            Status::transfer(0x18),
            Status::ProtocolError {
                raw_ack_bits: 0,
                mismatch: true
            }
        );
    }

    #[test]
    fn a_block_has_no_mismatch_and_high_bits_are_malformed() {
        assert_eq!(Status::block(0x11), Status::Malformed(0x11));
        for b in [0x20, 0x41, 0x81, 0xFF] {
            assert_eq!(Status::transfer(b), Status::Malformed(b));
        }
    }

    #[test]
    fn raw_acks_stay_raw() {
        for b in [0, 3, 5, 6] {
            assert_eq!(Status::transfer(b), ack(Ack::Other(b), false));
        }
        assert_eq!(Status::transfer(3).to_string(), "ACK 0b011");
    }

    #[test]
    fn every_byte_round_trips() {
        for b in 0..=255u8 {
            assert_eq!(Status::transfer(b).byte(), b);
            assert_eq!(Status::block(b).byte(), b);
        }
    }
}
