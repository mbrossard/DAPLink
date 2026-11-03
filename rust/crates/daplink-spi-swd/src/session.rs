//! Pure SWD framing, session geometry and clock-ladder logic: plain functions over integers, host-tested.

use dap_rs::swd::{APnDP, DPRegister, RnW};

/// Turnaround clocks between bus direction changes (the power-on `DLCR.TURNROUND` default).
pub const TRN: u8 = 1;

/// The 8-bit SWD request header, LSB first on the wire; `addr` carries `A[3:2]`.
const fn request_header(ap: bool, read: bool, addr: u8) -> u8 {
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

/// The write data phase as three 11-bit LSB-first frames: 32 data bits then parity.
pub const fn wdata_frames(data: u32) -> [u16; 3] {
    let v = (data as u64) | ((parity32(data) as u64) << 32);
    [
        (v & 0x7FF) as u16,
        ((v >> 11) & 0x7FF) as u16,
        ((v >> 22) & 0x7FF) as u16,
    ]
}

/// Core cycles one iteration of the engine's counted spin costs: `subs` plus a taken `bne`.
pub const SPIN_CYCLES: u32 = 3;

/// Spin iterations covering at least `cycles` core cycles; never zero, the loop is bottom-tested.
pub const fn spin_iterations(cycles: u32) -> u32 {
    let iterations = cycles.div_ceil(SPIN_CYCLES);
    if iterations == 0 {
        1
    } else {
        iterations
    }
}

/// CPU cycles in one SCK period at a kernel clock and division factor; floored at one.
pub const fn sck_period_cycles(cpu_hz: u32, kernel_hz: u32, divisor: u32) -> u32 {
    if kernel_hz == 0 {
        return 1;
    }
    let cycles = (cpu_hz as u64 * divisor as u64) / kernel_hz as u64;
    if cycles == 0 {
        1
    } else if cycles > u32::MAX as u64 {
        u32::MAX
    } else {
        cycles as u32
    }
}

/// Half-period spin count for a requested wire clock, rounded up and at least one.
pub const fn half_period_cycles(cpu_hz: u32, max_frequency: u32) -> u32 {
    let hz = if max_frequency == 0 { 1 } else { max_frequency };
    let cycles = (cpu_hz / 2).div_ceil(hz);
    if cycles == 0 {
        1
    } else {
        cycles
    }
}

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

    /// CPU cycles in one SCK period at this rung: the ES0621 2.17.2 EOT-to-disable delay.
    pub const fn sck_cycles(self, cpu_hz: u32, kernel_hz: u32) -> u32 {
        sck_period_cycles(cpu_hz, kernel_hz, self.divisor())
    }
}

/// The fastest rung whose SCK does not exceed `max_hz`, or `None` if even /256 is too fast.
pub const fn choose_prescaler(kernel_hz: u32, max_hz: u32) -> Option<(Prescaler, u32)> {
    if kernel_hz == 0 || max_hz == 0 {
        return None;
    }
    let mut mbr = 0u8;
    while mbr <= 7 {
        let f = kernel_hz / (2u32 << mbr);
        if f <= max_hz {
            return Some((Prescaler(mbr), f));
        }
        mbr += 1;
    }
    None
}

/// [`choose_prescaler`] with the request capped at `max_sck_hz`, the part's SPI clock ceiling.
pub const fn ladder_rung(
    kernel_hz: u32,
    max_sck_hz: u32,
    requested_hz: u32,
) -> Option<(Prescaler, u32)> {
    let capped = if requested_hz < max_sck_hz {
        requested_hz
    } else {
        max_sck_hz
    };
    choose_prescaler(kernel_hz, capped)
}

/// One clock an [`SpiLadder`] chose: the kernel source to route and the rung dividing it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SpiClock<S> {
    /// The kernel source the board routes.
    pub source: S,
    /// That source's rate, in Hz.
    pub kernel_hz: u32,
    /// The rung dividing it.
    pub pre: Prescaler,
    /// The SCK the rung yields, in Hz.
    pub sck_hz: u32,
}

/// An SPI block's clock plan: the kernel sources it can route, their rates, and the SCK cap.
#[derive(Copy, Clone, Debug)]
pub struct SpiLadder<S: Copy, const K: usize> {
    /// Each kernel's source and rate; the first is the one the block starts on.
    pub kernels: [(S, u32); K],
    /// The part's SPI clock ceiling.
    pub max_sck_hz: u32,
}

impl<S: Copy, const K: usize> SpiLadder<S, K> {
    /// The fastest nonzero SCK at or below the capped request; a tie keeps the earlier kernel.
    pub const fn choose(&self, requested_hz: u32) -> Option<SpiClock<S>> {
        let mut best: Option<SpiClock<S>> = None;
        let mut best_hz = 0;
        let mut i = 0;
        while i < K {
            let (source, kernel_hz) = self.kernels[i];
            if let Some((pre, sck_hz)) = ladder_rung(kernel_hz, self.max_sck_hz, requested_hz) {
                if sck_hz > best_hz {
                    best_hz = sck_hz;
                    best = Some(SpiClock {
                        source,
                        kernel_hz,
                        pre,
                        sck_hz,
                    });
                }
            }
            i += 1;
        }
        best
    }
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

/// The read data phase as one session: two 17-bit frames, data plus parity plus turnaround.
pub const fn read_data_session() -> Session {
    Session::new(17, 2, Dir::Rx)
}

/// The write data session: five 8-bit transmit frames (4 data bytes + parity byte).
pub const fn write_data_session() -> Session {
    Session::new(8, 5, Dir::Tx)
}

/// The exact write data session: three 11-bit transmit frames.
pub const fn write_data_session_exact() -> Session {
    Session::new(11, 3, Dir::Tx)
}

/// Split the two received data words into the 32-bit data value and the parity bit.
pub fn combine_read_data(lo: u32, hi: u32) -> (u32, u8) {
    let data = (lo & 0x0001_FFFF) | ((hi & 0x0000_7FFF) << 17);
    let parity = ((hi >> 15) & 1) as u8;
    (data, parity)
}

/// The 8-bit SWD request header for a transfer named in the engine's types.
pub fn request_byte(apndp: APnDP, rnw: RnW, addr: DPRegister) -> u8 {
    request_header(apndp as u8 != 0, rnw as u8 != 0, addr as u8)
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
    fn read_data_is_one_session_of_33_bits_plus_trailing_turnaround() {
        let s = read_data_session();
        assert_eq!(s.bits, 17);
        assert_eq!(s.frames, 2);
        // 32 data + parity + trailing turnaround, matching the reference read framing.
        assert_eq!(u16::from(s.bits) * s.frames, 33 + u16::from(TRN));
        assert_eq!(u16::from(s.bits) * s.frames, 34);
    }

    #[test]
    fn every_session_width_is_a_legal_frame_size() {
        for s in [
            header_session(),
            read_ack_session(),
            write_ack_session(),
            write_data_session(),
            write_data_session_exact(),
            read_data_session(),
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
    fn exact_write_frames_carry_33_bits_lsb_first() {
        let data = 0xDEAD_BEEFu32;
        let f = wdata_frames(data);
        let v = f[0] as u64 | ((f[1] as u64) << 11) | ((f[2] as u64) << 22);
        assert_eq!(v & 0xFFFF_FFFF, data as u64);
        assert_eq!((v >> 32) & 1, parity32(data) as u64);
        // Three 11-bit frames carry exactly the 33 bits.
        assert!(f.iter().all(|&h| h < (1 << 11)));
        assert_eq!(
            u16::from(write_data_session_exact().bits) * write_data_session_exact().frames,
            33
        );
    }

    #[test]
    fn sck_period_scales_with_the_rung() {
        // Core equal to the kernel: one SCK period is the division factor in core cycles.
        let fast = choose_prescaler(200_000_000, 100_000_000).unwrap().0;
        let slow = choose_prescaler(200_000_000, 25_000_000).unwrap().0;
        assert_eq!(fast.sck_cycles(200_000_000, 200_000_000), 2);
        assert_eq!(slow.divisor(), 4 * fast.divisor());
        assert_eq!(
            slow.sck_cycles(200_000_000, 200_000_000),
            4 * fast.sck_cycles(200_000_000, 200_000_000)
        );
        // The slowest rung, a faster core, and the one-cycle floor.
        assert_eq!(Prescaler::SLOWEST.sck_cycles(200_000_000, 200_000_000), 256);
        assert_eq!(fast.sck_cycles(250_000_000, 200_000_000), 2);
        assert_eq!(Prescaler::SLOWEST.sck_cycles(1, 200_000_000), 1);
    }

    #[test]
    fn a_spin_covers_the_period_it_is_sized_for() {
        // Whole iterations, rounded up, and never zero: the loop is bottom-tested.
        for cycles in 1..300u32 {
            let iterations = spin_iterations(cycles);
            assert!(iterations >= 1);
            assert!(iterations * SPIN_CYCLES >= cycles);
            assert!(iterations * SPIN_CYCLES - cycles < SPIN_CYCLES);
        }
        assert_eq!(spin_iterations(0), 1);
        assert_eq!(spin_iterations(u32::MAX), u32::MAX.div_ceil(SPIN_CYCLES));
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

    /// Index-based selection: the first kernel, replaced only by a strictly faster rung, then its rung.
    fn reference(kernels: &[u32], max: u32, req: u32) -> Option<SpiClock<usize>> {
        let rung = |k| ladder_rung(k, max, req).map_or(0, |(_, f)| f);
        let mut best = (0, kernels[0], rung(kernels[0]));
        for (i, &k) in kernels.iter().enumerate().skip(1) {
            let f = rung(k);
            if f > best.2 {
                best = (i, k, f);
            }
        }
        if best.2 == 0 {
            return None;
        }
        ladder_rung(best.1, max, req).map(|(pre, sck_hz)| SpiClock {
            source: best.0,
            kernel_hz: best.1,
            pre,
            sck_hz,
        })
    }

    /// The H523 SPI2 ladder: per_ck, PLL1 Q and PLL2 P, capped at 50 MHz.
    const H523: SpiLadder<usize, 3> = SpiLadder {
        kernels: [(0, 64_000_000), (1, 100_000_000), (2, 80_000_000)],
        max_sck_hz: 50_000_000,
    };

    /// The F1 ladder: one fixed kernel, SPI2 on a 36 MHz APB1.
    const F1: SpiLadder<(), 1> = SpiLadder {
        kernels: [((), 36_000_000)],
        max_sck_hz: 18_000_000,
    };

    // The plan is a const value and `choose` a const fn.
    const _: () = assert!(matches!(
        F1.choose(18_000_000),
        Some(SpiClock {
            sck_hz: 18_000_000,
            ..
        })
    ));

    #[test]
    fn spi_ladder_matches_the_index_selection_on_the_h523_ladder_for_every_request() {
        let kernels = H523.kernels.map(|(_, k)| k);
        for req in 1..=60_000_000 {
            assert_eq!(
                H523.choose(req),
                reference(&kernels, 50_000_000, req),
                "request {req}"
            );
        }
    }

    #[test]
    fn spi_ladder_matches_ladder_rung_on_the_f1_kernel_for_every_request() {
        for req in 1..=60_000_000 {
            let want = ladder_rung(36_000_000, 18_000_000, req).map(|(pre, sck_hz)| SpiClock {
                source: (),
                kernel_hz: 36_000_000,
                pre,
                sck_hz,
            });
            assert_eq!(F1.choose(req), want, "request {req}");
        }
    }

    #[test]
    fn spi_ladder_matches_ladder_rung_on_runtime_f1_kernels() {
        for kernel in [
            8_000_000, 16_000_000, 24_000_000, 32_000_000, 36_000_000, 48_000_000,
        ] {
            let ladder = SpiLadder {
                kernels: [((), kernel)],
                max_sck_hz: 18_000_000,
            };
            for req in (1..=60_000_000)
                .step_by(997)
                .chain([18_000_000, 18_000_001])
            {
                let want = ladder_rung(kernel, 18_000_000, req).map(|(pre, sck_hz)| SpiClock {
                    source: (),
                    kernel_hz: kernel,
                    pre,
                    sck_hz,
                });
                assert_eq!(ladder.choose(req), want, "kernel {kernel} request {req}");
            }
        }
    }

    #[test]
    fn the_three_kernel_ladder_is_never_faster_and_takes_the_best_rung() {
        let want = [
            (100_000_000, 50_000_000),
            (50_000_000, 50_000_000),
            (49_000_000, 40_000_000),
            (40_000_000, 40_000_000),
            (39_000_000, 32_000_000),
            (32_000_000, 32_000_000),
            (25_000_000, 25_000_000),
            (24_000_000, 20_000_000),
            (16_000_000, 16_000_000),
            (12_000_000, 10_000_000),
            (1_000_000, 1_000_000),
        ];
        for (req, f) in want {
            assert_eq!(H523.choose(req).unwrap().sck_hz, f, "request {req}");
        }
        for req in (250_000..60_000_000).step_by(37_000) {
            assert!(H523.choose(req).unwrap().sck_hz <= req.min(50_000_000));
        }
    }
}
