//! VCOM conservation-counter query (`0x9D`): echoed id, nine free-running `u32` LE counters,
//! then the `valid`, `grade` and `units` mask words.

use daplink_core::vendor::VendorHandler;

/// Vendor command id for the VCOM counter query (`ID_DAP_Vendor29`).
pub const VCOM_STATS_CMD_ID: u8 = 0x9D;

/// Counters in the response, in wire order.
pub const VCOM_STATS_FIELDS: usize = 9;

/// Total response length: the echoed id, the little-endian counters, then `valid`, `grade` and
/// `units`.
pub const VCOM_STATS_RESPONSE_LEN: usize = 1 + 4 * VCOM_STATS_FIELDS + 12;
// The reply must stay one full-speed DAP packet; re-check this before assuming a slot is free.
const _: () = assert!(VCOM_STATS_RESPONSE_LEN == 49 && VCOM_STATS_RESPONSE_LEN <= 64);

/// Counter slots, in wire order; the first six are the v1 prefix an older host still reads.
pub const VCOM_SLOT_UART_RX: usize = 0;
pub const VCOM_SLOT_CDC_IN: usize = 1;
pub const VCOM_SLOT_HOST_DROPPED: usize = 2;
pub const VCOM_SLOT_RING_FULL: usize = 3;
pub const VCOM_SLOT_FRAME_ERR: usize = 4;
pub const VCOM_SLOT_OVERRUN: usize = 5;
pub const VCOM_SLOT_CDC_OUT: usize = 6;
pub const VCOM_SLOT_UART_TX: usize = 7;
pub const VCOM_SLOT_RX_DMA_ERR: usize = 8;

pub const VCOM_VALID_UART_RX: u32 = 1 << VCOM_SLOT_UART_RX;
pub const VCOM_VALID_CDC_IN: u32 = 1 << VCOM_SLOT_CDC_IN;
pub const VCOM_VALID_HOST_DROPPED: u32 = 1 << VCOM_SLOT_HOST_DROPPED;
pub const VCOM_VALID_RING_FULL: u32 = 1 << VCOM_SLOT_RING_FULL;
pub const VCOM_VALID_FRAME_ERR: u32 = 1 << VCOM_SLOT_FRAME_ERR;
pub const VCOM_VALID_OVERRUN: u32 = 1 << VCOM_SLOT_OVERRUN;
pub const VCOM_VALID_CDC_OUT: u32 = 1 << VCOM_SLOT_CDC_OUT;
pub const VCOM_VALID_UART_TX: u32 = 1 << VCOM_SLOT_UART_TX;
pub const VCOM_VALID_RX_DMA_ERR: u32 = 1 << VCOM_SLOT_RX_DMA_ERR;
pub const VCOM_VALID_IDENTITY: u32 =
    VCOM_VALID_UART_RX | VCOM_VALID_CDC_IN | VCOM_VALID_HOST_DROPPED;
pub const VCOM_VALID_ALL: u32 = (1 << VCOM_STATS_FIELDS) - 1;

/// `grade` values, two bits per slot and meaningful only where `valid` is set: how good the
/// evidence behind a sourced counter is, never how much care went into it.
pub const VCOM_GRADE_LOWER_BOUND: u32 = 1;
pub const VCOM_GRADE_EXACT: u32 = 2;

/// One slot's contribution to the `grade` word.
pub const fn vcom_grade(slot: usize, grade: u32) -> u32 {
    grade << (2 * slot)
}

/// One slot's contribution to the `units` word, marking its counter as episodes rather than bytes.
pub const fn vcom_episodes(slot: usize) -> u32 {
    1 << slot
}

/// The VCOM bridge's byte ledger, holding `uart_rx == cdc_in + host_dropped` on the receive leg
/// and `cdc_out >= uart_tx` on the transmit leg. A counter the firmware cannot observe reads zero
/// with its `valid` bit clear, so the wire format never varies.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct VcomCounters {
    /// Bytes the bridge took off the UART seam.
    pub uart_rx: u32,
    /// Bytes the CDC IN endpoint accepted for the host. A bridge that stages toward the endpoint
    /// credits them when it hands them to the CDC stream, and moves the credit to `host_dropped`
    /// if the stream discards the stage, so the field is exact once the stage drains.
    pub cdc_in: u32,
    /// Bytes taken off the seam that the host would not take, a discarded stage included.
    pub host_dropped: u32,
    /// Bytes the receive path dropped because its ring was full.
    pub ring_full: u32,
    /// Error-flagged reception, in the unit the `units` word declares.
    pub frame_err: u32,
    /// Receive overrun, in the unit the `units` word declares.
    pub overrun: u32,
    /// Bytes accepted from the CDC OUT endpoint.
    pub cdc_out: u32,
    /// Bytes the transmit path put on the wire.
    pub uart_tx: u32,
    /// Receive-DMA faults the seam recovered from.
    pub rx_dma_err: u32,
    /// Bit mask describing which counters this firmware genuinely observes.
    pub valid: u32,
    /// Two bits per slot: evidence quality of each sourced counter (`VCOM_GRADE_*`).
    pub grade: u32,
    /// One bit per slot: clear for bytes, set for episodes.
    pub units: u32,
}

impl VcomCounters {
    /// The counters in wire order, little-endian.
    pub fn to_le_bytes(&self) -> [u8; 4 * VCOM_STATS_FIELDS] {
        let mut out = [0u8; 4 * VCOM_STATS_FIELDS];
        for (slot, value) in out.chunks_exact_mut(4).zip([
            self.uart_rx,
            self.cdc_in,
            self.host_dropped,
            self.ring_full,
            self.frame_err,
            self.overrun,
            self.cdc_out,
            self.uart_tx,
            self.rx_dma_err,
        ]) {
            slot.copy_from_slice(&value.to_le_bytes());
        }
        out
    }
}

/// Answers [`VCOM_STATS_CMD_ID`] from the injected reader and declines everything else, so it
/// chains with the other vendor commands in the one vendor slot.
pub struct VcomStatsVendor {
    counters: fn() -> VcomCounters,
}

impl VcomStatsVendor {
    /// `counters` reads the bridge's live ledger.
    pub const fn new(counters: fn() -> VcomCounters) -> Self {
        Self { counters }
    }
}

impl VendorHandler for VcomStatsVendor {
    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        if report.first() != Some(&VCOM_STATS_CMD_ID) {
            return None;
        }
        let counters = (self.counters)();
        let end = 1 + 4 * VCOM_STATS_FIELDS;
        response[1..end].copy_from_slice(&counters.to_le_bytes());
        for (word, value) in response[end..VCOM_STATS_RESPONSE_LEN]
            .chunks_exact_mut(4)
            .zip([counters.valid, counters.grade, counters.units])
        {
            word.copy_from_slice(&value.to_le_bytes());
        }
        Some(VCOM_STATS_RESPONSE_LEN)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use daplink_core::vendor::VendorChain;

    fn sample() -> VcomCounters {
        VcomCounters {
            uart_rx: 0x0102_0304,
            cdc_in: 0x0102_0300,
            host_dropped: 4,
            ring_full: 5,
            frame_err: 6,
            overrun: 7,
            cdc_out: 8,
            uart_tx: 9,
            rx_dma_err: 10,
            valid: VCOM_VALID_ALL,
            grade: vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_EXACT)
                | vcom_grade(VCOM_SLOT_OVERRUN, VCOM_GRADE_LOWER_BOUND),
            units: vcom_episodes(VCOM_SLOT_OVERRUN),
        }
    }

    #[test]
    fn answers_the_query_with_every_counter_and_mask_in_wire_order() {
        let mut v = VcomStatsVendor::new(sample);
        let mut resp = [0u8; 64];
        resp[0] = VCOM_STATS_CMD_ID; // the probe echoes the id before the handler runs
        assert_eq!(
            v.handle(&[VCOM_STATS_CMD_ID], &mut resp),
            Some(VCOM_STATS_RESPONSE_LEN)
        );
        assert_eq!(&resp[1..5], &[0x04, 0x03, 0x02, 0x01]);
        let fields: [u32; VCOM_STATS_FIELDS] = core::array::from_fn(|i| {
            u32::from_le_bytes(resp[1 + 4 * i..5 + 4 * i].try_into().unwrap())
        });
        assert_eq!(fields, [0x0102_0304, 0x0102_0300, 4, 5, 6, 7, 8, 9, 10]);
        let word = |at: usize| u32::from_le_bytes(resp[at..at + 4].try_into().unwrap());
        assert_eq!(word(37), VCOM_VALID_ALL);
        assert_eq!(word(41), sample().grade);
        assert_eq!(word(45), sample().units);
    }

    #[test]
    fn the_v1_prefix_survives_the_bump() {
        // An old host reads the first six counters and the byte after them was the valid word;
        // both must keep their positions, which the slot constants pin.
        assert_eq!(VCOM_SLOT_UART_RX, 0);
        assert_eq!(VCOM_SLOT_OVERRUN, 5);
        let mut v = VcomStatsVendor::new(sample);
        let mut resp = [0u8; 64];
        v.handle(&[VCOM_STATS_CMD_ID], &mut resp);
        let old_fields: [u32; 6] = core::array::from_fn(|i| {
            u32::from_le_bytes(resp[1 + 4 * i..5 + 4 * i].try_into().unwrap())
        });
        assert_eq!(old_fields, [0x0102_0304, 0x0102_0300, 4, 5, 6, 7]);
    }

    #[test]
    fn the_identity_three_plus_every_seam_slot_is_the_whole_mask() {
        // What a bridge ORs onto its own four when its seam observes every remaining slot.
        let bridge = VCOM_VALID_IDENTITY | VCOM_VALID_CDC_OUT;
        let seam = VCOM_VALID_RING_FULL
            | VCOM_VALID_FRAME_ERR
            | VCOM_VALID_OVERRUN
            | VCOM_VALID_UART_TX
            | VCOM_VALID_RX_DMA_ERR;
        assert_eq!(bridge | seam, VCOM_VALID_ALL);
    }

    #[test]
    fn grade_and_unit_words_hold_every_slot() {
        let all_exact: u32 = (0..VCOM_STATS_FIELDS)
            .map(|slot| vcom_grade(slot, VCOM_GRADE_EXACT))
            .fold(0, |acc, bits| {
                assert_eq!(acc & bits, 0, "slots overlap");
                acc | bits
            });
        assert_eq!(all_exact.count_ones() as usize, VCOM_STATS_FIELDS);
        assert_eq!(vcom_grade(VCOM_SLOT_RX_DMA_ERR, VCOM_GRADE_EXACT) >> 16, 2);
        assert_eq!(vcom_episodes(VCOM_SLOT_RX_DMA_ERR), 1 << 8);
    }

    #[test]
    fn the_ledger_identity_holds_over_the_wire_format() {
        let c = sample();
        assert_eq!(c.uart_rx, c.cdc_in + c.host_dropped);
    }

    #[test]
    fn declines_every_other_command_id() {
        let mut v = VcomStatsVendor::new(VcomCounters::default);
        let mut resp = [0u8; 64];
        assert_eq!(v.handle(&[0x9E], &mut resp), None);
        assert_eq!(v.handle(&[0x00], &mut resp), None);
        assert_eq!(v.handle(&[], &mut resp), None);
    }

    #[test]
    fn chains_beside_the_realized_baud_query_without_shadowing_it() {
        let mut chain = VendorChain::new(
            crate::VcomBaudVendor::new(|| crate::VcomBaudState::deferred(115_207, 115_200, true)),
            VcomStatsVendor::new(sample),
        );
        let mut resp = [0u8; 64];
        assert_eq!(
            chain.handle(&[crate::VCOM_BAUD_CMD_ID], &mut resp),
            Some(crate::VCOM_BAUD_RESPONSE_LEN)
        );
        assert_eq!(
            chain.handle(&[VCOM_STATS_CMD_ID], &mut resp),
            Some(VCOM_STATS_RESPONSE_LEN)
        );
        assert_eq!(chain.handle(&[0x8A], &mut resp), None);
    }
}
