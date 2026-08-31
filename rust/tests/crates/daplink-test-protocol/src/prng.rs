//! Deterministic byte stream both ends of a link generate independently to compare what crossed.

/// Seed substituted for 0, which is xorshift32's fixed point.
pub const ZERO_SEED: u32 = 0x9E37_79B9;

/// Marsaglia's xorshift32, the whole state transition.
const fn step(x: u32) -> u32 {
    let x = x ^ (x << 13);
    let x = x ^ (x >> 17);
    x ^ (x << 5)
}

/// Seeded xorshift32 byte stream: each 32-bit word yields four bytes, least significant first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Prng {
    seed: u32,
    state: u32,
    pos: u64,
}

impl Prng {
    /// Start the stream; seed 0 is substituted with [`ZERO_SEED`] so every seed is usable.
    pub const fn new(seed: u32) -> Self {
        let seed = if seed == 0 { ZERO_SEED } else { seed };
        Self {
            seed,
            state: seed,
            pos: 0,
        }
    }

    /// The effective seed, after the zero substitution.
    pub const fn seed(&self) -> u32 {
        self.seed
    }

    /// Bytes emitted so far, which is the stream offset the next byte carries.
    pub const fn position(&self) -> u64 {
        self.pos
    }

    /// The next byte of the stream.
    pub fn next_byte(&mut self) -> u8 {
        // `state` holds the word covering the current position; a new one is drawn every 4 bytes.
        let phase = self.pos & 3;
        if phase == 0 {
            self.state = step(self.state);
        }
        self.pos += 1;
        (self.state >> (8 * phase)) as u8
    }

    /// Fill `buf` with the next bytes of the stream.
    pub fn fill(&mut self, buf: &mut [u8]) {
        for slot in buf {
            *slot = self.next_byte();
        }
    }

    /// Advance `n` bytes without producing them.
    pub fn advance(&mut self, n: u64) {
        let mut left = n;
        while left > 0 && self.pos & 3 != 0 {
            self.next_byte();
            left -= 1;
        }
        // Word-aligned: stepping keeps the alignment, so whole words cost one step each.
        let words = left / 4;
        for _ in 0..words {
            self.state = step(self.state);
        }
        self.pos += words * 4;
        for _ in 0..(left - words * 4) {
            self.next_byte();
        }
    }

    /// Move to absolute stream offset `pos`, restarting from the seed when moving backwards.
    pub fn seek(&mut self, pos: u64) {
        if pos < self.pos {
            self.state = self.seed;
            self.pos = 0;
        }
        self.advance(pos - self.pos);
    }
}

impl Iterator for Prng {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        Some(self.next_byte())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- generator vectors, pinned: host and DUT streams must agree byte for byte ---

    const SEED_1_HEAD: [u8; 16] = [
        0x21, 0x20, 0x04, 0x00, 0x01, 0x06, 0x08, 0x04, 0xC5, 0xA8, 0xCC, 0x9D, 0x4F, 0x99, 0x55,
        0x12,
    ];

    #[test]
    fn seeded_stream_matches_its_pinned_head() {
        let mut prng = Prng::new(1);
        let mut got = [0u8; 16];
        prng.fill(&mut got);
        assert_eq!(got, SEED_1_HEAD);

        // Also the stream seed 0 is substituted with.
        let mut prng = Prng::new(ZERO_SEED);
        let mut got = [0u8; 16];
        prng.fill(&mut got);
        assert_eq!(
            got,
            [
                0x19, 0x46, 0x0C, 0x51, 0x3E, 0x55, 0x2E, 0xE0, 0x3A, 0x8F, 0xB9, 0x7B, 0xB5, 0xA8,
                0x83, 0x01
            ]
        );

        let mut prng = Prng::new(0x5A5A_5A5A);
        let mut got = [0u8; 16];
        prng.fill(&mut got);
        assert_eq!(
            got,
            [
                0x92, 0x48, 0x33, 0x33, 0xC2, 0x95, 0x05, 0x1E, 0x1C, 0xF0, 0x0D, 0x3B, 0xFB, 0x61,
                0xC8, 0x24
            ]
        );
    }

    #[test]
    fn seed_zero_is_the_substituted_stream_not_a_stuck_one() {
        let mut zero = Prng::new(0);
        assert_eq!(zero.seed(), ZERO_SEED);
        let mut explicit = Prng::new(ZERO_SEED);
        for _ in 0..64 {
            assert_eq!(zero.next_byte(), explicit.next_byte());
        }
        // The fixed point would emit nothing but zeroes.
        assert!(Prng::new(0).take(64).any(|b| b != 0));
    }

    #[test]
    fn iterator_and_next_byte_are_the_same_stream() {
        let by_iter: [u8; 16] = core::array::from_fn({
            let mut prng = Prng::new(1);
            move |_| prng.next().unwrap()
        });
        assert_eq!(by_iter, SEED_1_HEAD);
    }

    #[test]
    fn position_counts_the_bytes_emitted() {
        let mut prng = Prng::new(7);
        assert_eq!(prng.position(), 0);
        prng.next_byte();
        assert_eq!(prng.position(), 1);
        let mut buf = [0u8; 30];
        prng.fill(&mut buf);
        assert_eq!(prng.position(), 31);
        prng.advance(9);
        assert_eq!(prng.position(), 40);
    }

    #[test]
    fn advance_from_every_phase_lands_on_the_same_bytes() {
        // The word-at-a-time fast path must agree with byte-at-a-time for any offset alignment.
        for skipped in 0..24u64 {
            let mut skipping = Prng::new(3);
            skipping.advance(skipped);
            let mut walking = Prng::new(3);
            for _ in 0..skipped {
                walking.next_byte();
            }
            assert_eq!(skipping.position(), walking.position());
            for _ in 0..8 {
                assert_eq!(
                    skipping.next_byte(),
                    walking.next_byte(),
                    "advance {skipped}"
                );
            }
        }
    }

    #[test]
    fn seek_addresses_blocks_in_both_directions() {
        const BLOCK: u64 = 4096;
        let mut reference = Prng::new(1);
        reference.advance(3 * BLOCK);
        let block3 = reference.next_byte();

        // Forward from the start, as a receiver resynchronizing past a short block does.
        let mut forward = Prng::new(1);
        forward.seek(3 * BLOCK);
        assert_eq!(forward.next_byte(), block3);

        // Backward, which restarts from the seed.
        let mut backward = Prng::new(1);
        backward.advance(9 * BLOCK);
        backward.seek(3 * BLOCK);
        assert_eq!(backward.position(), 3 * BLOCK);
        assert_eq!(backward.next_byte(), block3);

        // Seeking to the current position is a no-op.
        let mut still = Prng::new(1);
        still.seek(3 * BLOCK);
        still.seek(3 * BLOCK);
        assert_eq!(still.next_byte(), block3);
    }

    #[test]
    fn distinct_seeds_give_distinct_streams() {
        // The two directions of a duplex run are seeded separately and must not coincide.
        let tx: [u8; 64] = core::array::from_fn({
            let mut prng = Prng::new(1);
            move |_| prng.next_byte()
        });
        let rx: [u8; 64] = core::array::from_fn({
            let mut prng = Prng::new(2);
            move |_| prng.next_byte()
        });
        assert_ne!(tx, rx);
    }
}
