//! Pure session-framing and clock-ladder logic: plain functions over integers, host-tested.

use dap_rs::swd::{APnDP, DPRegister, RnW};

pub use crate::proto::{ack_from_frame, half_period_cycles, parity32, wdata_bytes, TRN};

/// One power-of-two SPI master-baud rung; the value is the raw 3-bit `CFG1.MBR` field.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Prescaler(u8);

impl Prescaler {
    /// The `MBR[2:0]` bit pattern for this rung (0..=7).
    pub const fn mbr_bits(self) -> u8 {
        self.0
    }

    /// The division factor this rung applies to the kernel clock (2, 4, .. 256).
    pub const fn divisor(self) -> u32 {
        2u32 << self.0
    }

    /// The slowest rung (/256), the power-on default before the host sets a clock.
    pub const SLOWEST: Prescaler = Prescaler(7);
}

/// Pick the fastest rung whose SCK does not exceed `max_hz`, with the SCK it yields; `None` when
/// even /256 is still faster (the signal to fall back to bit-banging). Never rounds up.
pub fn choose_prescaler(kernel_hz: u32, max_hz: u32) -> Option<(Prescaler, u32)> {
    if kernel_hz == 0 || max_hz == 0 {
        return None;
    }
    for mbr in 0u8..=7 {
        let f = kernel_hz / (2u32 << mbr);
        if f <= max_hz {
            return Some((Prescaler(mbr), f));
        }
    }
    None
}

/// [`choose_prescaler`] with the request capped at `max_sck_hz`, the part's SPI clock ceiling.
pub fn ladder_rung(kernel_hz: u32, max_sck_hz: u32, requested_hz: u32) -> Option<(Prescaler, u32)> {
    choose_prescaler(kernel_hz, requested_hz.min(max_sck_hz))
}

/// Direction of one SPI session, mapped onto the half-duplex direction bit `CR1.HDDIR`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Dir {
    /// The probe drives the SWDIO data line.
    Tx,
    /// The target drives SWDIO; the probe generates the clock and samples the line.
    Rx,
}

/// A single SPI session: a frame width, a frame count, and a direction.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Session {
    /// Frame width in bits (`DSIZE` = `bits - 1`), always within 4..=32.
    pub bits: u8,
    /// Number of frames of this width (`TSIZE`).
    pub frames: u16,
    /// Transfer direction.
    pub dir: Dir,
}

impl Session {
    const fn new(bits: u8, frames: u16, dir: Dir) -> Self {
        Self { bits, frames, dir }
    }
}

/// The header session: one 8-bit transmit frame carrying the request byte.
pub const fn header_session() -> Session {
    Session::new(8, 1, Dir::Tx)
}

/// The read-ACK session: one receive frame of `trn + 3` bits.
pub const fn read_ack_session() -> Session {
    Session::new(TRN + 3, 1, Dir::Rx)
}

/// The write-ACK session: one receive frame of `trn + 3 + trn` bits.
pub const fn write_ack_session() -> Session {
    Session::new(TRN + 3 + TRN, 1, Dir::Rx)
}

/// The park session reclaiming the bus after a non-OK ACK: one 4-bit zero transmit frame.
pub const fn park_session() -> Session {
    Session::new(4, 1, Dir::Tx)
}

/// The two receive sessions of the read data phase: 17 bits, then `16 + trn` bits (the parity
/// bit and trailing turnaround folded in, since a standalone `trn`-bit frame is below the
/// block's minimum `DSIZE`). Post-ACK clock count matches the reference read framing.
pub const fn read_data_sessions() -> [Session; 2] {
    [
        Session::new(17, 1, Dir::Rx),
        Session::new(16 + TRN, 1, Dir::Rx),
    ]
}

/// The write data session: five 8-bit transmit frames (4 data bytes + parity byte).
pub const fn write_data_session() -> Session {
    Session::new(8, 5, Dir::Tx)
}

/// Split the two received data words into the 32-bit data value and the parity bit.
pub fn combine_read_data(lo: u32, hi: u32) -> (u32, u8) {
    let data = (lo & 0x0001_FFFF) | ((hi & 0x0000_7FFF) << 17);
    let parity = ((hi >> 15) & 1) as u8;
    (data, parity)
}

/// The 8-bit SWD request header for a transfer named in the engine's types.
pub fn request_byte(apndp: APnDP, rnw: RnW, addr: DPRegister) -> u8 {
    crate::proto::request_byte(apndp as u8 != 0, rnw as u8 != 0, addr as u8)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ladder_rounds_down_and_picks_fastest() {
        // 64 MHz kernel: /2 = 32 MHz fits a 32 MHz request.
        let (pre, f) = choose_prescaler(64_000_000, 32_000_000).unwrap();
        assert_eq!(pre.divisor(), 2);
        assert_eq!(f, 32_000_000);
        // 10 MHz request: /4 = 16 too fast, /8 = 8 fits.
        let (pre, f) = choose_prescaler(64_000_000, 10_000_000).unwrap();
        assert_eq!(pre.divisor(), 8);
        assert_eq!(f, 8_000_000);
    }

    #[test]
    fn ladder_rungs_on_a_160_mhz_kernel() {
        for (req, want_div, want_hz) in [
            (80_000_000u32, 2u32, 80_000_000u32),
            (40_000_000, 4, 40_000_000),
            (25_000_000, 8, 20_000_000),
            (10_000_000, 16, 10_000_000),
        ] {
            let (pre, f) = choose_prescaler(160_000_000, req).unwrap();
            assert_eq!(pre.divisor(), want_div);
            assert_eq!(f, want_hz);
        }
    }

    #[test]
    fn ladder_reports_unavailable_below_floor() {
        // Slowest rung /256 = 250 kHz still exceeds 100 kHz.
        assert!(choose_prescaler(64_000_000, 100_000).is_none());
        assert!(choose_prescaler(0, 1_000_000).is_none());
    }

    #[test]
    fn ladder_rung_clamps_the_request_to_the_cap() {
        // Cap binds: 160 MHz kernel capped at 40 MHz, a 100 MHz request lands on /4, not /2.
        let (pre, f) = ladder_rung(160_000_000, 40_000_000, 100_000_000).unwrap();
        assert_eq!(pre.divisor(), 4);
        assert_eq!(f, 40_000_000);
        // Request binds: below the cap the request still decides.
        let (_, f) = ladder_rung(160_000_000, 40_000_000, 10_000_000).unwrap();
        assert_eq!(f, 10_000_000);
        // Neither binds: a cap at exactly kernel/2 is inert (that is already the fastest rung).
        assert_eq!(
            ladder_rung(64_000_000, 64_000_000 / 2, 50_000_000),
            choose_prescaler(64_000_000, 50_000_000)
        );
    }

    #[test]
    fn slowest_rung_is_div_256() {
        assert_eq!(Prescaler::SLOWEST.divisor(), 256);
        assert_eq!(Prescaler::SLOWEST.mbr_bits(), 7);
    }

    #[test]
    fn read_data_split_covers_33_bits_plus_trailing_turnaround() {
        let [lo, hi] = read_data_sessions();
        assert_eq!(lo.bits, 17);
        assert_eq!(hi.bits, 16 + TRN);
        // 32 data + parity + trailing turnaround, matching the reference read framing.
        assert_eq!(lo.bits + hi.bits, 33 + TRN);
        assert_eq!(lo.bits + hi.bits, 34);
        for s in [lo, hi] {
            assert!((4..=32).contains(&s.bits));
        }
    }

    #[test]
    fn every_session_width_is_a_legal_frame_size() {
        let [lo, hi] = read_data_sessions();
        for s in [
            header_session(),
            read_ack_session(),
            write_ack_session(),
            park_session(),
            write_data_session(),
            lo,
            hi,
        ] {
            assert!((4..=32).contains(&s.bits), "illegal DSIZE for {s:?}");
            assert!(s.frames >= 1);
        }
    }

    #[test]
    fn combine_read_data_reassembles_word_and_parity() {
        let data = 0xDEAD_BEEFu32;
        let par = parity32(data);
        let lo = data & 0x0001_FFFF;
        let hi = ((data >> 17) & 0x7FFF) | ((par as u32) << 15);
        assert_eq!(combine_read_data(lo, hi), (data, par));
    }

    #[test]
    fn combine_read_data_ignores_the_trailing_turnaround_bits() {
        let data = 0x1234_5678u32;
        let par = parity32(data);
        let lo = data & 0x0001_FFFF;
        // Bit 16 of the second frame is the trailing turnaround; it must not leak.
        let hi = ((data >> 17) & 0x7FFF) | ((par as u32) << 15) | (1 << 16);
        assert_eq!(combine_read_data(lo, hi), (data, par));
    }

    #[test]
    fn write_bytes_are_little_endian_plus_parity() {
        let data = 0xAABB_CCDDu32;
        assert_eq!(
            wdata_bytes(data),
            [0xDD, 0xCC, 0xBB, 0xAA, parity32(data)],
            "write data session shifts LSB-first bytes then the parity byte"
        );
    }

    #[test]
    fn ack_frame_discards_turnaround() {
        // raw = [trn=1][ack=0b001] -> 0b001_1 with ack in bits 1..=3.
        let raw = (0b001u32 << TRN) | 0b1;
        assert_eq!(ack_from_frame(raw, TRN), 0b001);
    }

    #[test]
    fn ack_widths_follow_turnaround() {
        assert_eq!(read_ack_session().bits, TRN + 3);
        assert_eq!(write_ack_session().bits, TRN + 3 + TRN);
    }

    #[test]
    fn half_period_rounds_up_and_floors_at_one() {
        // 250 MHz core, 25 MHz request: 125 MHz / 25 MHz = 5 cycles.
        assert_eq!(half_period_cycles(250_000_000, 25_000_000), 5);
        // Rounding is up, never down (a too-short half-period would overclock the wire).
        assert_eq!(half_period_cycles(250_000_000, 30_000_000), 5);
        assert_eq!(half_period_cycles(250_000_000, 40_000_000), 4);
        // A request at or above half the core clock still spins at least one cycle.
        assert_eq!(half_period_cycles(160_000_000, 160_000_000), 1);
    }

    #[test]
    fn request_byte_matches_the_wire_rules() {
        for (apndp, rnw, addr) in [
            (APnDP::DP, RnW::R, DPRegister::DPIDR),
            (APnDP::AP, RnW::W, DPRegister::SELECT),
            (APnDP::DP, RnW::R, DPRegister::RDBUFF),
        ] {
            let b = request_byte(apndp, rnw, addr);
            assert_eq!(b & 1, 1, "start bit");
            assert_eq!((b >> 6) & 1, 0, "stop bit");
            assert_eq!((b >> 7) & 1, 1, "park bit");
            assert_eq!((b >> 1) & 1, apndp as u8, "APnDP bit");
            assert_eq!((b >> 2) & 1, rnw as u8, "RnW bit");
            assert_eq!((b >> 3) & 0b11, addr as u8, "address bits");
            assert_eq!(b.count_ones() & 1, 0, "request byte carries even parity");
        }
    }
}
