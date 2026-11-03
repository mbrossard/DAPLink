//! Cold policy the two block engines share: WAIT retries, status bytes and request parsing.

use daplink_rs::dap_rs::swd::{self, DPRegister};

/// The same transfer up to `attempts` times while it answers WAIT, `AckWait` if they run out.
macro_rules! retry {
    ($attempts:expr, $e:expr) => {{
        let mut left = $attempts;
        let mut res = Err(swd::Error::AckWait);
        while left != 0 {
            res = $e;
            if !matches!(res, Err(swd::Error::AckWait)) {
                break;
            }
            left -= 1;
        }
        res
    }};
}
pub(crate) use retry;

/// How a block ended: the transfers run, a failed one included, the status byte and the words.
pub(crate) struct BlockEnd {
    pub(crate) ran: u16,
    pub(crate) status: u8,
    pub(crate) words: usize,
}

/// The transfer status byte as dap-rs encodes it.
#[inline(always)]
pub(crate) fn status_byte(e: swd::Error) -> u8 {
    match e {
        swd::Error::AckWait => 2,
        swd::Error::AckFault => 4,
        swd::Error::NoAck => 7,
        _ => (1 << 3) | 7,
    }
}

/// The two address bits as a register; the mask leaves no invalid value.
#[inline(always)]
pub(crate) fn dp_register(bits: u8) -> DPRegister {
    match bits & 3 {
        0 => DPRegister::DPIDR,
        1 => DPRegister::CTRLSTAT,
        2 => DPRegister::SELECT,
        _ => DPRegister::RDBUFF,
    }
}

/// Words a plain `DAP_Transfer` reads; `None` for value match, match mask or timestamp.
pub(crate) fn plain_transfers(count: usize, items: &[u8]) -> Option<usize> {
    let mut pos = 0;
    let mut reads = 0;
    for _ in 0..count {
        let treq = *items.get(pos)?;
        pos += 1;
        // Value match, match mask and timestamp need dap-rs's item handling.
        if treq & ((1 << 4) | (1 << 5) | (1 << 7)) != 0 {
            return None;
        }
        if treq & 2 != 0 {
            reads += 1;
        } else {
            pos += 4;
            if pos > items.len() {
                return None;
            }
        }
    }
    Some(reads)
}

/// Consecutive plain AP reads identical to `treq` starting at `items[pos]`, capped by `left`.
pub(crate) fn run_length(treq: u8, left: usize, items: &[u8], pos: usize) -> usize {
    let mut n = 1;
    while n < left && items.get(pos + n - 1) == Some(&treq) {
        n += 1;
    }
    n
}
