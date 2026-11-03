//! The probe's VCOM counter ledger: its slots, grades and units, and the conservation it brackets.

/// The probe's free-running VCOM counters; high-water marks give growth, not a count.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VcomCounters {
    /// Bytes the bridge took off the UART.
    pub uart_rx: u32,
    /// Bytes the CDC IN endpoint accepted; exact once any staged transfer drains.
    pub cdc_in: u32,
    /// Bytes taken off the UART that the host would not take, a discarded stage included.
    pub host_dropped: u32,
    /// Bytes dropped because the probe's receive ring was full.
    pub ring_full: u32,
    /// Error-flagged reception, in the unit the `units` word declares.
    pub frame_err: u32,
    /// Receive overrun, in the unit the `units` word declares.
    pub overrun: u32,
    /// Bytes the probe accepted from the CDC OUT endpoint.
    pub cdc_out: u32,
    /// Bytes the probe's transmit path put on the wire.
    pub uart_tx: u32,
    /// Receive-DMA faults the probe recovered from.
    pub rx_dma_err: u32,
    /// Counter coverage; `None` denotes a legacy response with unknown coverage.
    pub valid: Option<u32>,
    /// Two bits per slot, evidence quality per sourced counter; `None` before the v2 reply.
    pub grade: Option<u32>,
    /// One bit per slot, set where the counter is episodes rather than bytes; `None` before v2.
    pub units: Option<u32>,
}

/// The six receive-ledger slots every reply carries, and the identity three inside them.
pub const VALID_RX_LEDGER: u32 = 0b11_1111;
const VALID_IDENTITY: u32 = 0b000111;
pub const VALID_RX_DMA_ERR: u32 = 1 << SLOT_RX_DMA_ERR;

/// Slot indices into the mask words, in wire order.
pub const SLOT_RING_FULL: usize = 3;
pub const SLOT_FRAME_ERR: usize = 4;
pub const SLOT_OVERRUN: usize = 5;
pub const SLOT_CDC_OUT: usize = 6;
pub const SLOT_UART_TX: usize = 7;
pub const SLOT_RX_DMA_ERR: usize = 8;

pub const GRADE_LOWER_BOUND: u32 = 1;
pub const GRADE_EXACT: u32 = 2;

/// One slot's grade out of the two-bit-per-slot word; `None` on a reply without the word.
pub fn slot_grade(grade: Option<u32>, slot: usize) -> Option<u32> {
    grade.map(|g| (g >> (2 * slot)) & 0b11)
}

/// Whether a slot's counter is episodes rather than bytes; a reply without the word means bytes.
pub fn slot_episodes(units: Option<u32>, slot: usize) -> bool {
    units.is_some_and(|u| u & (1 << slot) != 0)
}

/// Whether the reply sources the slot; an unsourced slot reads zero and means unknown.
pub fn slot_sourced(valid: Option<u32>, slot: usize) -> bool {
    valid.is_none_or(|mask| mask & (1 << slot) != 0)
}

/// One counter rendered against its coverage, grade and unit.
pub fn render_counter(c: &VcomCounters, value: u32, slot: usize) -> String {
    if !slot_sourced(c.valid, slot) {
        return "unsourced".into();
    }
    let bound = match slot_grade(c.grade, slot) {
        Some(GRADE_LOWER_BOUND) => ">=",
        _ => "",
    };
    let unit = if slot_episodes(c.units, slot) {
        " episodes"
    } else {
        ""
    };
    format!("{bound}{value}{unit}")
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConservationClass {
    CompleteClean,
    PartialClean,
    ObservedLoss,
    IdentityFailed,
    /// A DMA channel was reinitialised under live traffic; the counters prove nothing.
    DmaFaulted,
    /// A monotonic counter read lower after than before: the probe restarted mid-bracket.
    Restart,
}

impl ConservationClass {
    pub fn key(self) -> &'static str {
        match self {
            Self::CompleteClean => "complete-clean",
            Self::PartialClean => "partial-clean",
            Self::ObservedLoss => "observed-loss",
            Self::IdentityFailed => "identity-failed",
            Self::DmaFaulted => "dma-faulted",
            Self::Restart => "restart",
        }
    }
}

impl VcomCounters {
    pub fn since(self, base: Self) -> Self {
        Self {
            uart_rx: self.uart_rx.wrapping_sub(base.uart_rx),
            cdc_in: self.cdc_in.wrapping_sub(base.cdc_in),
            host_dropped: self.host_dropped.wrapping_sub(base.host_dropped),
            ring_full: self.ring_full.wrapping_sub(base.ring_full),
            frame_err: self.frame_err.wrapping_sub(base.frame_err),
            overrun: self.overrun.wrapping_sub(base.overrun),
            cdc_out: self.cdc_out.wrapping_sub(base.cdc_out),
            uart_tx: self.uart_tx.wrapping_sub(base.uart_tx),
            rx_dma_err: self.rx_dma_err.wrapping_sub(base.rx_dma_err),
            valid: match (self.valid, base.valid) {
                (Some(a), Some(b)) => Some(a & b),
                _ => None,
            },
            // Coverage words are constants per firmware, so the closing sample's carry over.
            grade: base.grade.and(self.grade),
            units: base.units.and(self.units),
        }
    }

    /// Every byte taken off the UART reached the host or was charged to a loss site.
    pub fn identity_holds(self) -> bool {
        self.uart_rx == self.cdc_in.wrapping_add(self.host_dropped)
    }

    /// The probe charged a loss to a named site.
    pub fn lost_any(self) -> bool {
        self.host_dropped != 0 || self.ring_full != 0 || self.frame_err != 0 || self.overrun != 0
    }

    /// The sourced DMA-error counter moved: a channel was reinitialised under live traffic.
    pub fn dma_faulted(self) -> bool {
        let sourced = |bit| self.valid.is_none_or(|mask: u32| mask & bit != 0);
        self.rx_dma_err != 0 && sourced(VALID_RX_DMA_ERR)
    }
}

/// The probe's ledger sampled either side of every window a verb ran.
#[derive(Clone, Copy, Debug)]
pub struct Conservation {
    pub before: VcomCounters,
    pub after: VcomCounters,
}

impl Conservation {
    /// What the bracketed windows put through the bridge.
    pub fn delta(&self) -> VcomCounters {
        self.after.since(self.before)
    }

    /// The identity holds at the closing sample, and nothing inside the bracket was lost.
    pub fn clean(&self) -> bool {
        matches!(
            self.classification(),
            ConservationClass::CompleteClean | ConservationClass::PartialClean
        )
    }

    /// The first monotonic counter that went backward, meaning the probe restarted.
    pub fn restart_slot(&self) -> Option<(&'static str, u32, u32)> {
        let (b, a) = (self.before, self.after);
        [
            ("uart_rx", b.uart_rx, a.uart_rx),
            ("cdc_in", b.cdc_in, a.cdc_in),
            ("host_dropped", b.host_dropped, a.host_dropped),
            ("ring_full", b.ring_full, a.ring_full),
            ("frame_err", b.frame_err, a.frame_err),
            ("overrun", b.overrun, a.overrun),
            ("cdc_out", b.cdc_out, a.cdc_out),
            ("uart_tx", b.uart_tx, a.uart_tx),
            ("rx_dma_err", b.rx_dma_err, a.rx_dma_err),
        ]
        .into_iter()
        .find(|&(_, before, after)| after < before)
    }

    pub fn classification(&self) -> ConservationClass {
        // First: a restart makes every wrapped delta below meaningless.
        if self.restart_slot().is_some() {
            return ConservationClass::Restart;
        }
        let delta = self.delta();
        let valid = delta.valid;
        // A recovered fault means nothing else in the bracket is evidence.
        if delta.dma_faulted() {
            return ConservationClass::DmaFaulted;
        }
        if valid.is_some_and(|mask| mask & VALID_IDENTITY == VALID_IDENTITY)
            && !delta.identity_holds()
        {
            return ConservationClass::IdentityFailed;
        }
        let loss = [
            delta.host_dropped,
            delta.ring_full,
            delta.frame_err,
            delta.overrun,
        ];
        let loss_bits = [1 << 2, 1 << 3, 1 << 4, 1 << 5];
        if loss
            .into_iter()
            .zip(loss_bits)
            .any(|(n, bit)| n != 0 && valid.is_none_or(|mask| mask & bit != 0))
        {
            return ConservationClass::ObservedLoss;
        }
        // Complete also needs every receive-ledger slot graded exact; a v1 reply stays partial.
        let exact = delta
            .grade
            .is_some_and(|g| (0..6).all(|slot| (g >> (2 * slot)) & 0b11 == GRADE_EXACT));
        if valid.is_some_and(|mask| mask & VALID_RX_LEDGER == VALID_RX_LEDGER) && exact {
            ConservationClass::CompleteClean
        } else {
            ConservationClass::PartialClean
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A probe ledger that took `uart_rx` bytes off the UART and delivered all of them.
    pub(crate) fn ledger(uart_rx: u32) -> VcomCounters {
        VcomCounters {
            uart_rx,
            cdc_in: uart_rx,
            ..VcomCounters::default()
        }
    }

    #[test]
    fn free_running_counters_are_differenced_by_wrapping() {
        let before = ledger(u32::MAX - 100);
        let after = ledger(923);
        assert_eq!(after.since(before).uart_rx, 1024);
    }

    /// A grade word claiming exact on every receive-ledger slot.
    const EXACT_LEDGER: u32 = 0b10_10_10_10_10_10;

    #[test]
    fn conservation_classifies_full_partial_loss_and_identity_failure() {
        let classify = |after: VcomCounters| {
            Conservation {
                before: VcomCounters {
                    valid: after.valid,
                    grade: after.grade,
                    units: after.units,
                    ..VcomCounters::default()
                },
                after,
            }
            .classification()
        };
        assert_eq!(
            classify(VcomCounters {
                valid: Some(VALID_RX_LEDGER),
                grade: Some(EXACT_LEDGER),
                units: Some(0),
                ..ledger(10)
            }),
            ConservationClass::CompleteClean
        );
        // Coverage without exactness proves nothing was seen, not that nothing was lost.
        assert_eq!(
            classify(VcomCounters {
                valid: Some(VALID_RX_LEDGER),
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
        assert_eq!(
            classify(VcomCounters {
                valid: Some(VALID_RX_LEDGER),
                grade: Some(EXACT_LEDGER & !(0b11 << 6) | (GRADE_LOWER_BOUND << 6)),
                units: Some(0),
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
        assert_eq!(classify(ledger(10)), ConservationClass::PartialClean);
        assert_eq!(
            classify(VcomCounters {
                cdc_in: 9,
                host_dropped: 1,
                ..ledger(10)
            }),
            ConservationClass::ObservedLoss
        );
        assert_eq!(
            classify(VcomCounters {
                cdc_in: 9,
                valid: Some(VALID_IDENTITY),
                ..ledger(10)
            }),
            ConservationClass::IdentityFailed
        );
        assert_eq!(
            classify(VcomCounters {
                cdc_in: 9,
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
        // A recovered DMA fault outranks everything else the bracket seems to say.
        assert_eq!(
            classify(VcomCounters {
                rx_dma_err: 1,
                valid: Some(VALID_RX_LEDGER | VALID_RX_DMA_ERR),
                grade: Some(EXACT_LEDGER),
                units: Some(0),
                ..ledger(10)
            }),
            ConservationClass::DmaFaulted
        );
        // An unsourced DMA-error slot cannot fault a bracket.
        assert_eq!(
            classify(VcomCounters {
                rx_dma_err: 3,
                valid: Some(VALID_RX_LEDGER),
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
    }

    #[test]
    fn conservation_classifies_a_firmware_restart_over_wrapped_deltas() {
        // Every monotonic slot reads zero after a restart, well below what it read before.
        let before = VcomCounters {
            uart_rx: 500_000,
            cdc_in: 500_000,
            valid: Some(VALID_RX_LEDGER),
            grade: Some(EXACT_LEDGER),
            units: Some(0),
            ..VcomCounters::default()
        };
        let after = VcomCounters {
            valid: Some(VALID_RX_LEDGER),
            grade: Some(EXACT_LEDGER),
            units: Some(0),
            ..VcomCounters::default()
        };
        assert_eq!(
            Conservation { before, after }.classification(),
            ConservationClass::Restart
        );
    }

    #[test]
    fn a_wrapped_rx_dma_err_alone_classifies_as_restart_not_dma_faulted() {
        // Only the DMA-error slot goes backward; everything else holds still.
        let before = VcomCounters {
            rx_dma_err: 3,
            valid: Some(VALID_RX_LEDGER | VALID_RX_DMA_ERR),
            ..VcomCounters::default()
        };
        let after = VcomCounters {
            rx_dma_err: 1,
            valid: Some(VALID_RX_LEDGER | VALID_RX_DMA_ERR),
            ..VcomCounters::default()
        };
        assert_eq!(
            Conservation { before, after }.classification(),
            ConservationClass::Restart
        );
    }

    #[test]
    fn a_lower_bound_counter_renders_as_a_bound_and_an_unsourced_one_says_so() {
        let c = VcomCounters {
            valid: Some(VALID_RX_LEDGER),
            grade: Some(
                (GRADE_LOWER_BOUND << (2 * SLOT_RING_FULL)) | (GRADE_EXACT << (2 * SLOT_OVERRUN)),
            ),
            units: Some(1 << SLOT_OVERRUN),
            ..VcomCounters::default()
        };
        assert_eq!(render_counter(&c, 7, SLOT_RING_FULL), ">=7");
        assert_eq!(render_counter(&c, 3, SLOT_OVERRUN), "3 episodes");
        // Outside the coverage word the slot reads zero by contract, which is not a measurement.
        assert_eq!(render_counter(&c, 0, SLOT_UART_TX), "unsourced");
    }
}
