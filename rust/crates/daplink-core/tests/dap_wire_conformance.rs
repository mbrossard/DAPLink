//! CMSIS-DAP v2 request lengths: `dap_wire::command_len` against the spec layouts, and the slot size.

use dap_conformance::cmsis_dap::layout::{self, Length as Spec};
use daplink_core::dap_wire::command_len;
use daplink_core::{DapQueue, QueueError, Submit};

/// Deterministic xorshift64*; fixed seeds keep every failure reproducible.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }

    fn byte(&mut self) -> u8 {
        self.next() as u8
    }

    fn push(&mut self, out: &mut Vec<u8>, n: usize) {
        for _ in 0..n {
            out.push(self.byte());
        }
    }
}

/// The probe's own layout: the vendor flash stream, a count then that many bytes.
fn probe(r: &[u8], batched: bool) -> Option<Spec> {
    match r[0] {
        0x8c if !batched => Some(
            r.get(1)
                .map_or(Spec::Short, |&n| Spec::Len(2 + usize::from(n))),
        ),
        _ => None,
    }
}

fn spec(r: &[u8]) -> Spec {
    layout::request_with(r, &probe)
}

/// IDs `command_len` sizes at the top level; everything else reports one byte.
const CODE_SIZED: [u8; 10] = [0x05, 0x06, 0x12, 0x14, 0x15, 0x1d, 0x21, 0x7e, 0x7f, 0x8c];

/// Every ID the spec model sizes.
fn sized_ids() -> Vec<u8> {
    (0..=255u8)
        .filter(|&id| matches!(spec(&[id, 0, 0, 0, 0, 0, 0]), Spec::Len(_)))
        .collect()
}

/// A complete, valid request for a sized `id`.
fn gen(rng: &mut Rng, id: u8, depth: usize) -> Vec<u8> {
    let mut c = vec![id];
    match id {
        0x05 => {
            c.push(rng.byte());
            let count = rng.below(40);
            c.push(count as u8);
            for _ in 0..count {
                let req = rng.byte();
                c.push(req);
                if req & 0x02 == 0 || req & 0x10 != 0 {
                    rng.push(&mut c, 4);
                }
            }
        }
        0x06 => {
            c.push(rng.byte());
            let count = rng.below(160) as u16;
            c.extend_from_slice(&count.to_le_bytes());
            let req = rng.byte();
            c.push(req);
            if req & 0x02 == 0 {
                rng.push(&mut c, 4 * usize::from(count));
            }
        }
        0x12 => {
            let n = rng.byte();
            c.push(n);
            let bits = if n == 0 { 256 } else { usize::from(n) };
            rng.push(&mut c, bits.div_ceil(8));
        }
        0x14 | 0x1d => {
            let count = rng.below(12);
            c.push(count as u8);
            for _ in 0..count {
                let info = rng.byte();
                c.push(info);
                if id == 0x14 || info & 0x80 == 0 {
                    rng.push(&mut c, layout::sequence_bits(info).div_ceil(8));
                }
            }
        }
        0x15 | 0x8c => {
            let n = rng.byte();
            c.push(n);
            rng.push(&mut c, usize::from(n));
        }
        0x21 => {
            rng.push(&mut c, 2);
            let n = rng.below(300);
            c.extend_from_slice(&(n as u16).to_le_bytes());
            rng.push(&mut c, n);
        }
        0x7e | 0x7f => {
            let inner: Vec<u8> = sized_ids()
                .into_iter()
                .filter(|i| !matches!(i, 0x7e..=0x9f))
                .collect();
            let count = if depth == 0 { rng.below(6) } else { 0 };
            c.push(count as u8);
            for _ in 0..count {
                let i = inner[rng.below(inner.len())];
                c.extend(gen(rng, i, depth + 1));
            }
        }
        _ => rng.push(&mut c, layout::fixed_request(id).expect("a fixed id") - 1),
    }
    c
}

fn samples() -> impl Iterator<Item = Vec<u8>> {
    let mut rng = Rng(0x0dab_1e55_c0de_0001);
    let ids = sized_ids();
    (0..ids.len() * 200).map(move |i| gen(&mut rng, ids[i % ids.len()], 0))
}

#[test]
fn the_model_sizes_every_standard_command() {
    let unsized_ids: Vec<u8> = (0..=255u8).filter(|id| !sized_ids().contains(id)).collect();
    for id in unsized_ids {
        let vendor = (0x80..=0x9f).contains(&id);
        let undefined = (0x0b..=0x0f).contains(&id) || (0x24..=0x7d).contains(&id) || id >= 0xa0;
        assert!(vendor || undefined, "{id:#04x} left unsized");
    }
}

#[test]
fn every_id_reports_a_length_once_its_header_is_present() {
    for id in 0..=255u8 {
        let r = [id; 2048];
        let got = command_len(&r);
        if CODE_SIZED.contains(&id) {
            assert!(got.is_some(), "{id:#04x}");
        } else {
            // Simplification: fixed and unsized commands report one byte at the top level.
            assert_eq!(got, Some(1), "{id:#04x}");
        }
    }
    assert_eq!(command_len(&[]), None);
}

#[test]
fn generated_commands_match_the_model() {
    for c in samples() {
        let got = command_len(&c);
        if CODE_SIZED.contains(&c[0]) {
            assert_eq!(got, Some(c.len()), "{c:02x?}");
        } else {
            assert_eq!(got, Some(1), "{c:02x?}");
        }
    }
}

#[test]
fn truncated_commands_agree_with_the_model() {
    for c in samples().filter(|c| CODE_SIZED.contains(&c[0])) {
        for k in 0..c.len() {
            let want = match spec(&c[..k]) {
                Spec::Len(n) => Some(n),
                Spec::Short => None,
                Spec::Unsized => unreachable!(),
            };
            assert_eq!(command_len(&c[..k]), want, "{:02x?}", &c[..k]);
        }
    }
}

#[test]
fn trailing_bytes_do_not_change_the_length() {
    let mut rng = Rng(0x7a11_0000_0000_0003);
    for mut c in samples().filter(|c| CODE_SIZED.contains(&c[0])) {
        let len = c.len();
        let extra = 1 + rng.below(64);
        rng.push(&mut c, extra);
        assert_eq!(command_len(&c), Some(len), "{c:02x?}");
    }
}

#[test]
fn batches_sum_their_commands() {
    let mut rng = Rng(0xba7c_4000_0000_0005);
    let inner: Vec<u8> = sized_ids()
        .into_iter()
        .filter(|i| !matches!(i, 0x7e..=0x9f))
        .collect();
    for _ in 0..2000 {
        let id = if rng.below(2) == 0 { 0x7e } else { 0x7f };
        let count = rng.below(9);
        let mut c = vec![id, count as u8];
        for _ in 0..count {
            let i = inner[rng.below(inner.len())];
            c.extend(gen(&mut rng, i, 1));
        }
        assert_eq!(spec(&c), Spec::Len(c.len()));
        assert_eq!(command_len(&c), Some(c.len()), "{c:02x?}");
        let len = c.len();
        let extra = 1 + rng.below(8);
        rng.push(&mut c, extra);
        assert_eq!(command_len(&c), Some(len), "{c:02x?}");
    }
}

#[test]
fn no_command_completes_at_a_packet_boundary_inside_it() {
    for c in samples().filter(|c| c.len() <= 1024) {
        for k in (64..c.len()).step_by(64) {
            assert!(!command_len(&c[..k]).is_some_and(|n| n <= k), "{c:02x?}");
        }
    }
}

#[test]
fn a_long_jtag_configure_spans_two_packets() {
    let mut c = vec![0x15, 63];
    c.resize(65, 4);
    assert_eq!(spec(&c), Spec::Len(65));
    assert_eq!(command_len(&c[..64]), Some(65));
}

#[test]
fn a_long_uart_transfer_spans_two_packets() {
    let mut c = vec![0x21, 0x3e, 0, 60, 0];
    c.resize(65, 0x55);
    assert_eq!(spec(&c), Spec::Len(65));
    assert_eq!(command_len(&c[..64]), Some(65));
}

#[test]
fn a_uart_command_inside_a_batch_is_sized() {
    let mut c = vec![0x7f, 2, 0x23, 0x06, 0, 15, 0, 0x00];
    c.resize(68, 0xaa);
    assert_eq!(spec(&c), Spec::Len(68));
    assert_eq!(command_len(&c[..64]), Some(68));
}

#[test]
fn vendor_and_nested_batch_commands_are_unsized_inside_a_batch() {
    for inner in [0x7f, 0x80, 0x8c, 0x9f] {
        let c = [0x7f, 2, 0x00, 0x04, inner, 1, 2];
        assert_eq!(spec(&c), Spec::Unsized, "{inner:#04x}");
        assert_eq!(command_len(&c), Some(c.len()), "{inner:#04x}");
    }
}

fn check_slots<const N: usize, const S: usize>() {
    let mut q: DapQueue<N, S> = DapQueue::new();
    assert_eq!((q.depth(), q.packet_capacity()), (N, S));
    let mut queued = 0;
    for c in samples().take(3000) {
        let got = q.submit(&c);
        if c.len() > S {
            assert_eq!(got, Err(QueueError::Oversized), "{c:02x?}");
        } else if c[0] == 0x07 {
            assert_eq!(got, Ok(Submit::AbortIntercepted));
        } else if queued == N {
            assert_eq!(got, Err(QueueError::Full));
            q.reset();
            queued = 0;
        } else {
            assert_eq!(got, Ok(Submit::Enqueued));
            queued += 1;
        }
    }
    // The largest write block and read response fit the slot; one more word does not.
    let write = 5 + 4 * ((S - 5) / 4);
    let read = 4 + 4 * ((S - 4) / 4);
    assert!(write <= S && write + 4 > S && read <= S && read + 4 > S);
}

#[test]
fn a_slot_holds_exactly_one_dap_packet() {
    check_slots::<1, 64>();
    check_slots::<4, 64>();
    check_slots::<4, 512>();
    check_slots::<8, 512>();
}
