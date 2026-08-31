//! A `core::fmt`-free status renderer appending into a caller-owned buffer, truncating at its
//! capacity so a renderer can never panic or overflow.

/// Appends bytes and small integer renderings into a fixed byte buffer, truncating at capacity.
pub struct StatusWriter<'b> {
    buf: &'b mut [u8],
    used: usize,
}

impl<'b> StatusWriter<'b> {
    /// Wrap `buf`, starting empty.
    pub fn new(buf: &'b mut [u8]) -> Self {
        Self { buf, used: 0 }
    }

    /// Number of bytes written so far.
    pub fn len(&self) -> usize {
        self.used
    }

    /// Whether nothing has been written yet.
    pub fn is_empty(&self) -> bool {
        self.used == 0
    }

    /// The bytes written so far.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.used]
    }

    /// Append `data`, silently truncating anything that does not fit.
    pub fn bytes(&mut self, data: &[u8]) {
        let cap = self.buf.len();
        if self.used >= cap {
            return;
        }
        let n = core::cmp::min(data.len(), cap - self.used);
        self.buf[self.used..self.used + n].copy_from_slice(&data[..n]);
        self.used += n;
    }

    /// Append the decimal rendering of `v`.
    pub fn dec_u32(&mut self, mut v: u32) {
        if v == 0 {
            self.bytes(b"0");
            return;
        }
        let mut tmp = [0u8; 10];
        let mut i = tmp.len();
        while v > 0 {
            i -= 1;
            tmp[i] = b'0' + (v % 10) as u8;
            v /= 10;
        }
        self.bytes(&tmp[i..]);
    }

    /// Append `v` as `0x` followed by eight lowercase hex digits.
    pub fn hex_u32(&mut self, v: u32) {
        self.bytes(b"0x");
        const DIGITS: &[u8; 16] = b"0123456789abcdef";
        let mut out = [0u8; 8];
        for (k, slot) in out.iter_mut().enumerate() {
            let shift = (7 - k) * 4;
            *slot = DIGITS[((v >> shift) & 0xf) as usize];
        }
        self.bytes(&out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_writer() {
        let mut buf = [0u8; 16];
        let w = StatusWriter::new(&mut buf);
        assert!(w.is_empty());
        assert_eq!(w.len(), 0);
        assert_eq!(w.as_bytes(), b"");
    }

    #[test]
    fn append_bytes_and_ints() {
        let mut buf = [0u8; 64];
        let mut w = StatusWriter::new(&mut buf);
        w.bytes(b"n=");
        w.dec_u32(1234);
        w.bytes(b" @");
        w.hex_u32(0x0800_00AB);
        assert_eq!(w.as_bytes(), b"n=1234 @0x080000ab");
    }

    #[test]
    fn dec_zero() {
        let mut buf = [0u8; 8];
        let mut w = StatusWriter::new(&mut buf);
        w.dec_u32(0);
        assert_eq!(w.as_bytes(), b"0");
    }

    #[test]
    fn truncation_is_silent() {
        let mut buf = [0u8; 4];
        let mut w = StatusWriter::new(&mut buf);
        w.bytes(b"abcdefgh");
        assert_eq!(w.len(), 4);
        assert_eq!(w.as_bytes(), b"abcd");
        // Further writes still do not panic and remain truncated.
        w.dec_u32(999);
        assert_eq!(w.as_bytes(), b"abcd");
    }
}
