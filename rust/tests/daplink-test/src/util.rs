//! Helpers shared by cases: error chains, line splitting, hex encoding, and a payload PRNG.

/// `e` followed by its `source()` chain, joined with `: `. probe-rs wrappers such as
/// `Error::Arm` display a bare sentence and carry the whole diagnosis on the chain, so
/// printing only the outermost error says nothing about the failure.
pub(crate) fn chain(e: impl std::error::Error) -> String {
    let mut out = e.to_string();
    let mut prev = out.clone();
    let mut src = e.source();
    while let Some(s) = src {
        let text = s.to_string();
        // A transparent wrapper displays its source's text; do not print it twice.
        if text != prev {
            out.push_str(": ");
            out.push_str(&text);
            prev = text;
        }
        src = s.source();
    }
    out
}

/// The first `\n`-framed line of a response (the whole slice if there is no newline).
#[allow(dead_code)]
pub(crate) fn first_line(resp: &[u8]) -> &[u8] {
    resp.split(|&b| b == b'\n').next().unwrap_or(resp)
}

/// Append the lowercase-hex encoding of `bytes` to `out`.
pub(crate) fn push_hex(out: &mut Vec<u8>, bytes: &[u8]) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for &b in bytes {
        out.push(HEX[(b >> 4) as usize]);
        out.push(HEX[(b & 0xf) as usize]);
    }
}

/// Lowercase-hex encode `bytes` as a `String`.
pub(crate) fn hex(bytes: &[u8]) -> String {
    let mut out = Vec::with_capacity(bytes.len() * 2);
    push_hex(&mut out, bytes);
    String::from_utf8(out).expect("hex digits are valid UTF-8")
}

/// Deterministic xorshift64 PRNG: reproducible synthetic payloads without a dependency.
pub(crate) struct Rng(u64);

impl Rng {
    /// Seed the generator; the low bit is forced set so the state is never all-zero.
    pub(crate) fn new(seed: u64) -> Self {
        Rng(seed | 1)
    }

    /// Advance and return the next 64-bit state.
    pub(crate) fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// A `len`-byte pseudo-random payload.
    pub(crate) fn payload(&mut self, len: usize) -> Vec<u8> {
        (0..len).map(|_| self.next_u64() as u8).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A wrapper displaying `msg`, whatever its source says.
    #[derive(Debug)]
    struct Opaque(&'static str, Box<dyn std::error::Error + 'static>);

    impl std::fmt::Display for Opaque {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.write_str(self.0)
        }
    }

    impl std::error::Error for Opaque {
        fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
            Some(self.1.as_ref())
        }
    }

    #[test]
    fn chain_appends_every_source() {
        let inner = Opaque(
            "SWD ack was FAULT",
            Box::new(std::io::Error::other("no ack")),
        );
        let outer = Opaque("An ARM specific error occurred.", Box::new(inner));
        assert_eq!(
            chain(outer),
            "An ARM specific error occurred.: SWD ack was FAULT: no ack"
        );
    }

    #[test]
    fn chain_skips_a_transparent_wrapper() {
        // A transparent wrapper displays its source's text; it must not appear twice.
        let e = Opaque("no ack", Box::new(std::io::Error::other("no ack")));
        assert_eq!(chain(e), "no ack");
    }

    #[test]
    fn hex_encodes_lowercase() {
        assert_eq!(hex(&[0xde, 0xad, 0xbe, 0xef]), "deadbeef");
        assert_eq!(hex(&[0xa5, 0x5a, 0xc3, 0x3c]), "a55ac33c");
    }

    #[test]
    fn first_line_splits_on_newline() {
        assert_eq!(first_line(b"OK PONG\nextra"), b"OK PONG");
        assert_eq!(first_line(b"no newline"), b"no newline");
    }

    #[test]
    fn rng_is_deterministic_and_sized() {
        let mut a = Rng::new(0x1234);
        let mut b = Rng::new(0x1234);
        assert_eq!(a.payload(16), b.payload(16));
        assert_eq!(a.payload(7).len(), 7);
    }
}
