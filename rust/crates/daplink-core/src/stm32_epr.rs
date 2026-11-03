//! STM32 USB FS endpoint register (EPnR/CHEPnR) halt clear, as a pure step over the register value.

const CTR_RX: u32 = 1 << 15;
const DTOG_RX: u32 = 1 << 14;
const STAT_RX: u32 = 3 << 12;
const CTR_TX: u32 = 1 << 7;
const DTOG_TX: u32 = 1 << 6;
const STAT_TX: u32 = 3 << 4;

const DISABLED: u32 = 0;
const STALL: u32 = 1;
const NAK: u32 = 2;
const VALID: u32 = 3;

/// The write moving `epr` one step toward unhalted at DATA0, or `None` once there or disabled.
pub fn clear_halt_step(epr: u32, dir_in: bool) -> Option<u32> {
    let (stat_mask, dtog, shift) = if dir_in {
        (STAT_TX, DTOG_TX, 4)
    } else {
        (STAT_RX, DTOG_RX, 12)
    };
    let stat = (epr & stat_mask) >> shift;
    let want = match stat {
        DISABLED => return None,
        STALL if dir_in => NAK,
        STALL => VALID,
        other => other,
    };
    if stat == want && epr & dtog == 0 {
        return None;
    }
    // STAT and DTOG toggle on a written one; CTR clears on a written zero.
    let keep = epr & !(DTOG_RX | STAT_RX | DTOG_TX | STAT_TX);
    Some(keep | CTR_RX | CTR_TX | ((stat ^ want) << shift) | (epr & dtog))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One endpoint register with the hardware's write semantics.
    struct Model(u32);

    impl Model {
        fn write(&mut self, w: u32) {
            let toggles = DTOG_RX | STAT_RX | DTOG_TX | STAT_TX;
            let ctr = (CTR_RX | CTR_TX) & self.0 & w;
            let plain = w & !(toggles | CTR_RX | CTR_TX);
            self.0 = plain | ctr | ((self.0 ^ w) & toggles);
        }

        fn stat(&self, dir_in: bool) -> u32 {
            if dir_in {
                (self.0 & STAT_TX) >> 4
            } else {
                (self.0 & STAT_RX) >> 12
            }
        }

        /// A packet completes: VALID turns NAK, the toggle advances, CTR sets.
        fn complete(&mut self, dir_in: bool) {
            if self.stat(dir_in) != VALID {
                return;
            }
            if dir_in {
                self.0 = (self.0 & !STAT_TX) | (NAK << 4) | CTR_TX;
                self.0 ^= DTOG_TX;
            } else {
                self.0 = (self.0 & !STAT_RX) | (NAK << 12) | CTR_RX;
                self.0 ^= DTOG_RX;
            }
        }
    }

    /// Bulk endpoint 1 in `stat`, with the other direction busy so a step must leave it alone.
    fn start(dir_in: bool, stat: u32, dtog: bool) -> u32 {
        let base = 1 | (VALID << 12) | (VALID << 4) | DTOG_RX | DTOG_TX;
        let (mask, bit, shift) = if dir_in {
            (STAT_TX | DTOG_TX, DTOG_TX, 4)
        } else {
            (STAT_RX | DTOG_RX, DTOG_RX, 12)
        };
        (base & !mask) | (stat << shift) | if dtog { bit } else { 0 }
    }

    /// The register as `step` drives it, with a packet completing before write `race`.
    fn run(
        step: fn(u32, bool) -> Option<u32>,
        epr: u32,
        dir_in: bool,
        race: usize,
    ) -> Option<Model> {
        let mut reg = Model(epr);
        for i in 0..16 {
            let Some(w) = step(reg.0, dir_in) else {
                return Some(reg);
            };
            if i == race {
                reg.complete(dir_in);
            }
            reg.write(w);
        }
        None
    }

    #[test]
    fn clears_every_halt_state_to_data0_under_a_racing_completion() {
        for dir_in in [true, false] {
            for stat in [STALL, NAK, VALID] {
                for dtog in [false, true] {
                    for race in 0..4 {
                        let epr = start(dir_in, stat, dtog);
                        let reg = run(clear_halt_step, epr, dir_in, race)
                            .unwrap_or_else(|| panic!("no fixpoint from {epr:#06x}"));
                        let toggle = if dir_in { DTOG_TX } else { DTOG_RX };
                        assert_eq!(reg.0 & toggle, 0, "toggle from {epr:#06x}");
                        assert_eq!(reg.stat(!dir_in), VALID, "other direction from {epr:#06x}");
                        let expected = match (stat, dir_in) {
                            (STALL, true) => [NAK, NAK],
                            (STALL, false) => [VALID, VALID],
                            (NAK, _) => [NAK, NAK],
                            _ => [VALID, NAK],
                        };
                        assert!(expected.contains(&reg.stat(dir_in)), "{epr:#06x}");
                        assert_eq!(reg.0 & 0x0f, 1, "address kept");
                    }
                }
            }
        }
    }

    #[test]
    fn leaves_a_disabled_endpoint_alone() {
        for dir_in in [true, false] {
            assert_eq!(clear_halt_step(start(dir_in, DISABLED, true), dir_in), None);
        }
    }

    /// embassy-stm32 0.6.0's unstall loop body, transcribed.
    fn v060_step(epr: u32, dir_in: bool) -> Option<u32> {
        let (mask, shift, want) = if dir_in {
            (STAT_TX, 4, NAK)
        } else {
            (STAT_RX, 12, VALID)
        };
        let stat = (epr & mask) >> shift;
        if stat == DISABLED || stat == STALL {
            return None;
        }
        let keep = epr & !(DTOG_RX | STAT_RX | DTOG_TX | STAT_TX);
        Some(keep | CTR_RX | CTR_TX | ((stat ^ want) << shift))
    }

    #[test]
    fn the_embassy_0_6_loop_keeps_a_halt_and_spins_without_one() {
        for dir_in in [true, false] {
            let halted = run(v060_step, start(dir_in, STALL, true), dir_in, 99).unwrap();
            assert_eq!(halted.stat(dir_in), STALL);
            for stat in [NAK, VALID] {
                assert!(run(v060_step, start(dir_in, stat, false), dir_in, 99).is_none());
            }
        }
    }
}
