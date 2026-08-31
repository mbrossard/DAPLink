//! The probe's CMSIS-DAP v2 bulk interface: VCOM realized-baud and counter vendor queries.

use std::time::Duration;

use anyhow::{Context, Result};
use dap_bench::transport::{self, Probe, ProbeSelector};

use crate::stream::VcomCounters;

/// Vendor command id of the VCOM realized-baud query (`ID_DAP_Vendor30`).
const VCOM_BAUD_CMD: u8 = 0x9E;

/// Vendor command id of the VCOM counter query (`ID_DAP_Vendor29`).
const VCOM_STATS_CMD: u8 = 0x9D;

/// The two `0x9E` reply lengths, negotiated the same way: the realized rate alone (v1), and the
/// realized rate with the applied rate and the `flags` word (v2).
const VCOM_BAUD_V1_LEN: usize = 5;
const VCOM_BAUD_V2_LEN: usize = 13;

/// `flags` bit: the seam observes whether a requested rate has reached its divisor.
const VCOM_BAUD_APPLIED_VALID: u32 = 1 << 0;
/// `flags` bit: no rate change is in flight. Meaningful only with `VCOM_BAUD_APPLIED_VALID`.
const VCOM_BAUD_SETTLED: u32 = 1 << 1;

/// The three reply lengths, negotiated by length alone: six counters (v1), six plus the valid
/// word, and twelve plus the `valid`, `grade` and `units` words (v2).
const VCOM_STATS_V1_LEN: usize = 1 + 4 * 6;
const VCOM_STATS_V1_VALID_LEN: usize = VCOM_STATS_V1_LEN + 4;
const VCOM_STATS_V2_FIELDS: usize = 9;
const VCOM_STATS_V2_LEN: usize = 1 + 4 * VCOM_STATS_V2_FIELDS + 12;

const TIMEOUT: Duration = Duration::from_millis(500);

pub struct DapLink {
    probe: Probe,
    max_packet_size: usize,
    vendor: bool,
}

impl DapLink {
    /// Open the CMSIS-DAP v2 bulk interface matching `selector` (`VID:PID[:serial]`).
    /// With `vendor` false the vendor queries are never sent and read as unserved.
    pub fn open(selector: &str, vendor: bool) -> Result<DapLink> {
        let selector: ProbeSelector = selector
            .parse()
            .with_context(|| format!("parse probe selector '{selector}'"))?;
        let probes = transport::enumerate();
        let info = transport::select(&probes, &selector)?;
        let max_packet_size = info.max_packet_size;
        let probe = Probe::open(info.clone())?;
        Ok(DapLink {
            probe,
            max_packet_size,
            vendor,
        })
    }

    /// Bulk max packet size the probe enumerated with: 64 on full-speed, 512 on high-speed.
    pub fn max_packet_size(&self) -> usize {
        self.max_packet_size
    }

    /// `None` when the firmware does not serve the query (answers `0xFF`, or a short reply).
    pub fn vcom_baud(&mut self) -> Result<Option<VcomBaud>> {
        if !self.vendor {
            return Ok(None);
        }
        let resp = self.probe.transfer(&[VCOM_BAUD_CMD], TIMEOUT)?;
        Ok(parse_vcom_baud(&resp))
    }

    /// `None` when the firmware predates the query, so older images stay measurable.
    pub fn vcom_counters(&mut self) -> Result<Option<VcomCounters>> {
        if !self.vendor {
            return Ok(None);
        }
        let resp = self.probe.transfer(&[VCOM_STATS_CMD], TIMEOUT)?;
        Ok(parse_vcom_counters(&resp))
    }
}

/// The probe's answer to `0x9E`. A firmware predating the applied-ness reply leaves `settled`
/// `None`, which is the host's cue that it can only infer settling from a timeout.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct VcomBaud {
    /// The rate the divisor now on the wire produces, `None` before the first programming.
    pub realized: Option<u32>,
    /// The nominal rate that divisor was programmed for, `None` where the seam does not source it.
    pub applied: Option<u32>,
    /// Whether a rate change is in flight; `None` where the seam does not observe applied-ness.
    pub settled: Option<bool>,
}

fn parse_vcom_baud(resp: &[u8]) -> Option<VcomBaud> {
    if resp.first() != Some(&VCOM_BAUD_CMD) {
        return None;
    }
    // A longer reply than this host knows is read as far as it does know, so a firmware ahead of
    // the tool stays measurable in exactly the way an older one does.
    if resp.len() < VCOM_BAUD_V1_LEN {
        return None;
    }
    let word = |at: usize| u32::from_le_bytes(resp[at..at + 4].try_into().expect("length checked"));
    let realized = word(1);
    let (applied, settled) = if resp.len() >= VCOM_BAUD_V2_LEN {
        let flags = word(9);
        if flags & VCOM_BAUD_APPLIED_VALID != 0 {
            let applied = word(5);
            (
                (applied != 0).then_some(applied),
                Some(flags & VCOM_BAUD_SETTLED != 0),
            )
        } else {
            (None, None)
        }
    } else {
        (None, None)
    };
    Some(VcomBaud {
        realized: (realized != 0).then_some(realized),
        applied,
        settled,
    })
}

fn parse_vcom_counters(resp: &[u8]) -> Option<VcomCounters> {
    if resp.first() != Some(&VCOM_STATS_CMD)
        || !matches!(
            resp.len(),
            VCOM_STATS_V1_LEN | VCOM_STATS_V1_VALID_LEN | VCOM_STATS_V2_LEN
        )
    {
        return None;
    }

    let field = |i: usize| {
        u32::from_le_bytes(
            resp[1 + 4 * i..5 + 4 * i]
                .try_into()
                .expect("length checked"),
        )
    };
    let v2 = resp.len() == VCOM_STATS_V2_LEN;
    // A counter absent from the reply reads zero; its valid bit is what says so.
    let v2_field = |i: usize| if v2 { field(i) } else { 0 };
    let word = |at: usize| u32::from_le_bytes(resp[at..at + 4].try_into().expect("length checked"));
    let masks = 1 + 4 * VCOM_STATS_V2_FIELDS;
    Some(VcomCounters {
        uart_rx: field(0),
        cdc_in: field(1),
        host_dropped: field(2),
        ring_full: field(3),
        frame_err: field(4),
        overrun: field(5),
        cdc_out: v2_field(6),
        uart_tx: v2_field(7),
        rx_dma_err: v2_field(8),
        valid: match resp.len() {
            VCOM_STATS_V1_VALID_LEN => Some(word(VCOM_STATS_V1_LEN)),
            VCOM_STATS_V2_LEN => Some(word(masks)),
            _ => None,
        },
        grade: v2.then(|| word(masks + 4)),
        units: v2.then(|| word(masks + 8)),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn response(valid: Option<u32>) -> Vec<u8> {
        let mut response = vec![VCOM_STATS_CMD];
        for field in [
            0x0102_0304_u32,
            0x1112_1314,
            0x2122_2324,
            0x3132_3334,
            0x4142_4344,
            0x5152_5354,
        ] {
            response.extend_from_slice(&field.to_le_bytes());
        }
        if let Some(valid) = valid {
            response.extend_from_slice(&valid.to_le_bytes());
        }
        response
    }

    fn v2_response(valid: u32, grade: u32, units: u32) -> Vec<u8> {
        let mut response = vec![VCOM_STATS_CMD];
        for field in 1..=9u32 {
            response.extend_from_slice(&(field * 0x0101).to_le_bytes());
        }
        for word in [valid, grade, units] {
            response.extend_from_slice(&word.to_le_bytes());
        }
        response
    }

    fn expected(valid: Option<u32>) -> VcomCounters {
        VcomCounters {
            uart_rx: 0x0102_0304,
            cdc_in: 0x1112_1314,
            host_dropped: 0x2122_2324,
            ring_full: 0x3132_3334,
            frame_err: 0x4142_4344,
            overrun: 0x5152_5354,
            valid,
            ..VcomCounters::default()
        }
    }

    fn baud_v2(realized: u32, applied: u32, flags: u32) -> Vec<u8> {
        let mut response = vec![VCOM_BAUD_CMD];
        for word in [realized, applied, flags] {
            response.extend_from_slice(&word.to_le_bytes());
        }
        response
    }

    #[test]
    fn the_realized_only_reply_leaves_applied_ness_unknown() {
        let mut response = vec![VCOM_BAUD_CMD];
        response.extend_from_slice(&921_600u32.to_le_bytes());
        assert_eq!(response.len(), VCOM_BAUD_V1_LEN);
        assert_eq!(
            parse_vcom_baud(&response),
            Some(VcomBaud {
                realized: Some(921_600),
                applied: None,
                settled: None,
            })
        );
    }

    #[test]
    fn the_flags_reply_carries_applied_ness_both_ways() {
        let response = baud_v2(
            2_666_666,
            2_666_666,
            VCOM_BAUD_APPLIED_VALID | VCOM_BAUD_SETTLED,
        );
        assert_eq!(response.len(), VCOM_BAUD_V2_LEN);
        assert_eq!(
            parse_vcom_baud(&response),
            Some(VcomBaud {
                realized: Some(2_666_666),
                applied: Some(2_666_666),
                settled: Some(true),
            })
        );
        // The old rate is still on the wire while the new one waits for the transmitter.
        assert_eq!(
            parse_vcom_baud(&baud_v2(115_207, 115_200, VCOM_BAUD_APPLIED_VALID)),
            Some(VcomBaud {
                realized: Some(115_207),
                applied: Some(115_200),
                settled: Some(false),
            })
        );
    }

    #[test]
    fn a_seam_declining_applied_ness_says_so_rather_than_claiming_settled() {
        assert_eq!(
            parse_vcom_baud(&baud_v2(9_600, 9_600, 0)),
            Some(VcomBaud {
                realized: Some(9_600),
                applied: None,
                settled: None,
            })
        );
    }

    #[test]
    fn a_reply_longer_than_this_host_knows_is_read_as_far_as_it_does() {
        let mut response = baud_v2(9_600, 9_600, VCOM_BAUD_APPLIED_VALID | VCOM_BAUD_SETTLED);
        response.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(
            parse_vcom_baud(&response),
            Some(VcomBaud {
                realized: Some(9_600),
                applied: Some(9_600),
                settled: Some(true),
            })
        );
    }

    #[test]
    fn a_wrong_id_or_a_short_reply_is_no_answer_at_all() {
        assert_eq!(parse_vcom_baud(&[]), None);
        assert_eq!(parse_vcom_baud(&[0xFF, 0, 0, 0, 0]), None);
        assert_eq!(parse_vcom_baud(&[VCOM_BAUD_CMD, 0, 0]), None);
    }

    #[test]
    fn a_zero_rate_is_the_absence_of_a_reading() {
        assert_eq!(
            parse_vcom_baud(&baud_v2(0, 0, VCOM_BAUD_APPLIED_VALID)),
            Some(VcomBaud {
                realized: None,
                applied: None,
                settled: Some(false),
            })
        );
    }

    #[test]
    fn parses_legacy_response_with_unknown_coverage() {
        let response = response(None);
        assert_eq!(response.len(), 25);
        assert_eq!(parse_vcom_counters(&response), Some(expected(None)));
    }

    #[test]
    fn parses_extended_response_with_explicit_coverage() {
        let response = response(Some(0b001111));
        assert_eq!(response.len(), 29);
        assert_eq!(
            parse_vcom_counters(&response),
            Some(expected(Some(0b001111)))
        );
    }

    #[test]
    fn parses_the_v2_response_with_every_counter_and_mask() {
        let response = v2_response(0x1FF, 0xAAAAAA, 0b1010);
        assert_eq!(response.len(), 49);
        let parsed = parse_vcom_counters(&response).expect("parses");
        assert_eq!(parsed.uart_rx, 0x0101);
        assert_eq!(parsed.overrun, 6 * 0x0101);
        assert_eq!(parsed.cdc_out, 7 * 0x0101);
        assert_eq!(parsed.uart_tx, 8 * 0x0101);
        assert_eq!(parsed.rx_dma_err, 9 * 0x0101);
        assert_eq!(parsed.valid, Some(0x1FF));
        assert_eq!(parsed.grade, Some(0xAAAAAA));
        assert_eq!(parsed.units, Some(0b1010));
    }

    #[test]
    fn rejects_wrong_command_and_malformed_lengths() {
        let mut wrong_command = response(None);
        wrong_command[0] = 0xff;
        assert_eq!(parse_vcom_counters(&wrong_command), None);

        let v2 = v2_response(0x1FF, 0, 0);
        for len in [0, 1, 24, 26, 27, 28, 30, 36, 37, 41, 45, 48] {
            assert_eq!(parse_vcom_counters(&v2[..len]), None, "length {len}");
        }

        let mut too_long = v2;
        too_long.push(0);
        assert_eq!(parse_vcom_counters(&too_long), None);
    }
}
