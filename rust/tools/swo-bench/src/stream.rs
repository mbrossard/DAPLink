//! Record framing and gap detection over the DUT's sequence-tagged ITM payload.

use daplink_test_protocol as proto;

/// Bytes per record.
pub const RECORD_LEN: usize = proto::SWO_STREAM_RECORD_LEN;

/// One break in the sequence: `missing` records never arrived after `after_seq`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gap {
    /// Sequence number of the last record before the break.
    pub after_seq: u32,
    /// Records the sequence says are missing.
    pub missing: u64,
    /// Byte offset into the payload where the record after the break starts.
    pub byte_offset: usize,
}

/// What one pass of [`scan`] found in a payload stream.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Framing {
    /// Well-formed records decoded.
    pub records: u64,
    /// Sequence number of the first record.
    pub first_seq: Option<u32>,
    /// Sequence number of the last record.
    pub last_seq: Option<u32>,
    /// Sequence breaks, in capture order.
    pub gaps: Vec<Gap>,
    /// Records the sequence breaks account for.
    pub lost_records: u64,
    /// Times the scanner had to hunt for the magic word again mid-stream: byte-level loss.
    pub resync_events: usize,
    /// Bytes discarded by those hunts.
    pub resync_bytes: usize,
    /// Bytes before the first record: the capture simply began mid-record.
    pub preamble_bytes: usize,
    /// Bytes after the last complete record: the window simply ended mid-record.
    pub trailing_bytes: usize,
    /// Records whose sequence did not advance (a repeat or a backwards jump).
    pub out_of_order: usize,
}

impl Framing {
    /// A tier is clean when records arrived and nothing was lost between them.
    pub fn is_clean(&self) -> bool {
        self.records > 0
            && self.gaps.is_empty()
            && self.resync_events == 0
            && self.out_of_order == 0
    }
}

/// Scan `payload` for records, reporting every break in the sequence and every resync.
pub fn scan(payload: &[u8]) -> Framing {
    let magic = proto::SWO_STREAM_MAGIC.to_le_bytes();
    let mut f = Framing::default();
    let mut i = 0;
    let mut skipped = 0usize;
    let mut prev: Option<u32> = None;

    while i + RECORD_LEN <= payload.len() {
        if payload[i..i + 4] != magic {
            i += 1;
            skipped += 1;
            continue;
        }
        if skipped > 0 {
            // Non-record bytes between records: data lost mid-record, or the ragged start.
            if prev.is_none() {
                f.preamble_bytes = skipped;
            } else {
                f.resync_events += 1;
                f.resync_bytes += skipped;
            }
            skipped = 0;
        }

        let seq = u32::from_le_bytes(payload[i + 4..i + RECORD_LEN].try_into().unwrap());
        f.records += 1;
        f.first_seq.get_or_insert(seq);
        f.last_seq = Some(seq);

        if let Some(previous) = prev {
            match seq.wrapping_sub(previous) {
                1 => {}
                0 => f.out_of_order += 1,
                delta if delta > u32::MAX / 2 => f.out_of_order += 1,
                delta => {
                    let missing = u64::from(delta - 1);
                    f.gaps.push(Gap {
                        after_seq: previous,
                        missing,
                        byte_offset: i,
                    });
                    f.lost_records += missing;
                }
            }
        }
        prev = Some(seq);
        i += RECORD_LEN;
    }

    // The remainder is the window's ragged tail, or the whole capture if nothing framed.
    let tail = payload.len() - i + skipped;
    if prev.is_none() {
        f.preamble_bytes = payload.len();
    } else {
        f.trailing_bytes = tail;
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build the payload the DUT would emit for `seqs`.
    fn payload(seqs: &[u32]) -> Vec<u8> {
        let mut v = Vec::new();
        for &s in seqs {
            v.extend_from_slice(&proto::SWO_STREAM_MAGIC.to_le_bytes());
            v.extend_from_slice(&s.to_le_bytes());
        }
        v
    }

    #[test]
    fn a_contiguous_stream_is_clean() {
        let f = scan(&payload(&[1, 2, 3, 4]));
        assert_eq!(f.records, 4);
        assert_eq!(f.first_seq, Some(1));
        assert_eq!(f.last_seq, Some(4));
        assert!(f.is_clean());
        assert_eq!(f.lost_records, 0);
    }

    #[test]
    fn a_dropped_record_is_reported_with_its_position() {
        let f = scan(&payload(&[1, 2, 9, 10]));
        assert_eq!(f.records, 4);
        assert_eq!(f.lost_records, 6);
        assert_eq!(
            f.gaps,
            vec![Gap {
                after_seq: 2,
                missing: 6,
                byte_offset: 2 * RECORD_LEN
            }]
        );
        assert!(!f.is_clean());
    }

    #[test]
    fn several_gaps_are_all_kept() {
        let f = scan(&payload(&[1, 3, 4, 8]));
        assert_eq!(f.gaps.len(), 2);
        assert_eq!(f.lost_records, 1 + 3);
    }

    #[test]
    fn mid_record_byte_loss_shows_up_as_a_resync() {
        let mut p = payload(&[1]);
        p.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02]); // record 2, mangled by the loss
        p.extend_from_slice(&payload(&[3]));
        let f = scan(&p);
        assert_eq!(f.records, 2);
        assert_eq!(f.resync_events, 1);
        assert_eq!(f.resync_bytes, 6);
        // The surviving tags still show the record that never landed.
        assert_eq!(f.lost_records, 1);
        assert!(!f.is_clean());
    }

    #[test]
    fn a_ragged_start_is_a_preamble_not_a_gap() {
        let mut p = vec![0x11, 0x22, 0x33];
        p.extend_from_slice(&payload(&[5, 6]));
        let f = scan(&p);
        assert_eq!(f.preamble_bytes, 3);
        assert_eq!(f.resync_events, 0);
        assert!(f.is_clean());
    }

    #[test]
    fn a_ragged_end_is_a_trailing_remainder() {
        let mut p = payload(&[1, 2]);
        p.extend_from_slice(&[0x5A, 0xA5]);
        let f = scan(&p);
        assert_eq!(f.records, 2);
        assert_eq!(f.trailing_bytes, 2);
        assert!(f.is_clean());
    }

    #[test]
    fn the_sequence_may_wrap_without_looking_like_a_gap() {
        let f = scan(&payload(&[u32::MAX - 1, u32::MAX, 0, 1]));
        assert_eq!(f.records, 4);
        assert!(f.is_clean());
    }

    #[test]
    fn a_repeated_or_backwards_tag_is_out_of_order() {
        let f = scan(&payload(&[5, 5, 4]));
        assert_eq!(f.out_of_order, 2);
        assert!(f.gaps.is_empty());
        assert!(!f.is_clean());
    }

    #[test]
    fn a_payload_with_no_record_at_all_is_not_clean() {
        let f = scan(&[0xAA; 32]);
        assert_eq!(f.records, 0);
        assert_eq!(f.preamble_bytes, 32);
        assert!(!f.is_clean());
        assert!(!scan(&[]).is_clean());
    }
}
