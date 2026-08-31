//! The wiring, and the pin mapping the state machine's `PINCTRL` is loaded from.

/// The wiring, by GPIO number; every pad must be below 32.
#[derive(Copy, Clone)]
pub struct PioSwdPins {
    /// SWCLK, the side-set pin.
    pub swclk: u8,
    /// SWDIO as driven; also the readback pad on a single bidirectional wiring.
    pub swdio_out: u8,
    /// Separate readback pad on a split wiring, live whether or not the probe drives.
    pub swdio_in: Option<u8>,
    /// Active-low output enable; it must be immediately below SWCLK for contiguous side-set.
    pub oen: Option<u8>,
    /// nRESET, driven open-drain from SIO for the life of the image.
    pub nreset: Option<u8>,
}

impl PioSwdPins {
    /// The readback pad: the separate one where the wiring splits, else the driven pad itself.
    pub const fn readback(&self) -> u8 {
        match self.swdio_in {
            Some(pad) => pad,
            None => self.swdio_out,
        }
    }

    /// Whether drive and readback land on different pads.
    pub const fn is_split(&self) -> bool {
        self.readback() != self.swdio_out
    }

    /// Whether every pad is valid, exclusively assigned, and matches the program side-set.
    pub const fn valid_side_set_layout(&self, side_set_bits: u8) -> bool {
        let input = self.readback();
        if self.swclk >= 32
            || self.swdio_out >= 32
            || input >= 32
            || self.swclk == self.swdio_out
            || self.swclk == input
        {
            return false;
        }
        if let Some(input) = self.swdio_in {
            if input == self.swdio_out {
                return false;
            }
        }
        if let Some(oe) = self.oen {
            if side_set_bits != 2
                || oe >= 31
                || self.swclk != oe + 1
                || oe == self.swdio_out
                || oe == input
            {
                return false;
            }
        } else if side_set_bits != 1 {
            return false;
        }
        if let Some(nreset) = self.nreset {
            if nreset >= 32
                || nreset == self.swclk
                || nreset == self.swdio_out
                || nreset == input
                || match self.oen {
                    Some(oe) => nreset == oe,
                    None => false,
                }
            {
                return false;
            }
        }
        true
    }
}

/// The state machine's pin mapping; only the bases vary, and `sideset_count` includes `opt`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PinMap {
    /// `OUT` base: the driven SWDIO pad, which `out pindirs,1` also turns around.
    pub out_base: u8,
    /// `SET` base: the same driven pad, for the idle-direction forcing.
    pub set_base: u8,
    /// `IN` base: the readback pad, which `in pins,1` samples.
    pub in_base: u8,
    /// Side-set base; the hardware takes side-set pins contiguously from here.
    pub sideset_base: u8,
    /// Side-set width, data bits plus the `opt` enable bit.
    pub sideset_count: u8,
}

impl PinMap {
    /// Derive a mapping, requiring OEN immediately below SWCLK for the two-bit variant.
    pub const fn new(pins: &PioSwdPins, side_set_bits: u8) -> Self {
        assert!(pins.valid_side_set_layout(side_set_bits));
        Self {
            out_base: pins.swdio_out,
            set_base: pins.swdio_out,
            in_base: pins.readback(),
            // `{OEn, SWCLK}` where a shifter is driven, SWCLK alone otherwise.
            sideset_base: match pins.oen {
                Some(oe) => oe,
                None => pins.swclk,
            },
            sideset_count: side_set_bits + 1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::program::{select_program, Variant, DIRECT};

    /// The Pico wiring: one bidirectional pad, no shifter.
    const PICO: PioSwdPins = PioSwdPins {
        swclk: 2,
        swdio_out: 3,
        swdio_in: None,
        oen: None,
        nreset: Some(1),
    };

    /// The Debug Probe wiring: SWCLK GP12, SWDIO driven on GP14, read back on GP13, no OE pad.
    const DEBUGPROBE: PioSwdPins = PioSwdPins {
        swclk: 12,
        swdio_out: 14,
        swdio_in: Some(13),
        oen: None,
        nreset: None,
    };

    #[test]
    fn single_pad_reads_back_the_driven_pad() {
        assert!(!PICO.is_split());
        assert_eq!(
            PinMap::new(&PICO, DIRECT.side_set_bits),
            PinMap {
                out_base: 3,
                set_base: 3,
                in_base: 3,
                sideset_base: 2,
                sideset_count: 2,
            }
        );
    }

    #[test]
    fn split_wiring_samples_the_readback_pad_and_turns_the_driven_one_around() {
        assert!(DEBUGPROBE.is_split());
        let map = PinMap::new(&DEBUGPROBE, DIRECT.side_set_bits);
        assert_eq!(
            map,
            PinMap {
                out_base: 14,
                set_base: 14,
                in_base: 13,
                sideset_base: 12,
                sideset_count: 2,
            }
        );
        // `out pindirs,1` targets the OUT base, so the turnaround releases the driven pad only.
        assert_eq!(map.out_base, DEBUGPROBE.swdio_out);
        assert_ne!(map.in_base, map.out_base);
    }

    #[test]
    fn split_wiring_runs_the_direct_program() {
        // Splitting readback is a mapping fact, not an instruction one: no OE pad, no OEN program.
        assert_eq!(
            select_program(DEBUGPROBE.oen.is_some()).variant,
            Variant::Direct
        );
        assert_eq!(select_program(PICO.oen.is_some()).variant, Variant::Direct);
    }

    #[test]
    fn an_output_enable_pad_takes_the_side_set_base() {
        let shifted = PioSwdPins {
            swclk: 3,
            swdio_out: 4,
            swdio_in: Some(5),
            oen: Some(2),
            nreset: None,
        };
        let prog = select_program(true);
        assert_eq!(prog.variant, Variant::Oen);
        assert_eq!(
            PinMap::new(&shifted, prog.side_set_bits),
            PinMap {
                out_base: 4,
                set_base: 4,
                in_base: 5,
                sideset_base: 2,
                sideset_count: 3,
            }
        );
    }

    #[test]
    fn side_set_layout_validation_covers_both_variants() {
        assert!(PICO.valid_side_set_layout(1));
        assert!(!PICO.valid_side_set_layout(2));

        let valid_oen = PioSwdPins {
            swclk: 3,
            swdio_out: 4,
            swdio_in: Some(5),
            oen: Some(2),
            nreset: None,
        };
        assert!(valid_oen.valid_side_set_layout(2));
        assert!(!valid_oen.valid_side_set_layout(1));

        let noncontiguous_oen = PioSwdPins {
            swclk: 4,
            ..valid_oen
        };
        assert!(!noncontiguous_oen.valid_side_set_layout(2));

        let overflowing_oen = PioSwdPins {
            swclk: 0,
            oen: Some(31),
            ..valid_oen
        };
        assert!(!overflowing_oen.valid_side_set_layout(2));
    }

    #[test]
    fn pin_validation_enforces_bounds_and_exclusive_ownership() {
        let direct = PioSwdPins {
            swclk: 2,
            swdio_out: 3,
            swdio_in: Some(4),
            oen: None,
            nreset: Some(1),
        };
        assert!(direct.valid_side_set_layout(1));

        for invalid in [
            PioSwdPins {
                swclk: 32,
                ..direct
            },
            PioSwdPins {
                swdio_out: 32,
                ..direct
            },
            PioSwdPins {
                swdio_in: Some(32),
                ..direct
            },
            PioSwdPins {
                nreset: Some(32),
                ..direct
            },
            PioSwdPins {
                swclk: direct.swdio_out,
                ..direct
            },
            PioSwdPins {
                swdio_in: Some(direct.swdio_out),
                ..direct
            },
            PioSwdPins {
                swdio_in: Some(direct.swclk),
                ..direct
            },
            PioSwdPins {
                nreset: Some(direct.swclk),
                ..direct
            },
            PioSwdPins {
                nreset: Some(direct.swdio_out),
                ..direct
            },
            PioSwdPins {
                nreset: direct.swdio_in,
                ..direct
            },
        ] {
            assert!(!invalid.valid_side_set_layout(1));
        }

        let oen = PioSwdPins {
            swclk: 3,
            swdio_out: 4,
            swdio_in: Some(5),
            oen: Some(2),
            nreset: Some(1),
        };
        assert!(oen.valid_side_set_layout(2));
        for invalid in [
            PioSwdPins {
                swdio_out: 2,
                ..oen
            },
            PioSwdPins {
                swdio_in: Some(2),
                ..oen
            },
            PioSwdPins {
                nreset: Some(2),
                ..oen
            },
        ] {
            assert!(!invalid.valid_side_set_layout(2));
        }
    }

    #[test]
    #[should_panic]
    fn pin_map_rejects_noncontiguous_oen_and_swclk() {
        let pins = PioSwdPins {
            swclk: 5,
            swdio_out: 6,
            swdio_in: None,
            oen: Some(2),
            nreset: None,
        };
        let _ = PinMap::new(&pins, 2);
    }
}
