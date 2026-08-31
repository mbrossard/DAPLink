//! Vendor-command port: CMSIS-DAP ids `0x80..=0x9F`, routed to a [`VendorHandler`] before the engine.

/// First vendor-specific CMSIS-DAP command id (`ID_DAP_Vendor0`).
pub const VENDOR_CMD_FIRST: u8 = 0x80;
/// Last vendor-specific CMSIS-DAP command id (`ID_DAP_Vendor31`).
pub const VENDOR_CMD_LAST: u8 = 0x9F;

/// Handler for vendor-specific CMSIS-DAP commands.
pub trait VendorHandler {
    /// Whether this handler handles anything at all. [`NoVendor`] sets this `false` so the
    /// vendor-routing branch compiles out entirely on probes without vendor commands.
    const HANDLES: bool = true;

    /// Handle one vendor command; `response[0]` is already the echoed id, so fill from `response[1]`
    /// and return the TOTAL length (>= 1). `None` declines and falls through to the standard engine.
    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize>;
}

/// No vendor commands: everything falls through to the engine.
pub struct NoVendor;

impl VendorHandler for NoVendor {
    const HANDLES: bool = false;

    fn handle(&mut self, _report: &[u8], _response: &mut [u8]) -> Option<usize> {
        None
    }
}

/// Composes two [`VendorHandler`]s over disjoint command ids: `a` first, `b` only when `a` declines.
/// A firmware nests these to fit several vendor commands into the engine's one vendor slot.
pub struct VendorChain<A, B> {
    a: A,
    b: B,
}

impl<A, B> VendorChain<A, B> {
    /// Chain `a` before `b`.
    pub const fn new(a: A, b: B) -> Self {
        Self { a, b }
    }
}

impl<A: VendorHandler, B: VendorHandler> VendorHandler for VendorChain<A, B> {
    // Compiles the routing branch out only when neither side handles anything.
    const HANDLES: bool = A::HANDLES || B::HANDLES;

    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        self.a
            .handle(report, response)
            .or_else(|| self.b.handle(report, response))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Answers one fixed command id with a fixed byte and declines everything else.
    struct Fixed {
        id: u8,
        reply: u8,
    }

    impl VendorHandler for Fixed {
        fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
            if report.first() == Some(&self.id) {
                response[1] = self.reply;
                Some(2)
            } else {
                None
            }
        }
    }

    #[test]
    fn first_handler_wins_when_it_answers() {
        let mut chain =
            VendorChain::new(Fixed { id: 0x9F, reply: 1 }, Fixed { id: 0x8A, reply: 2 });
        let mut resp = [0u8; 4];
        assert_eq!(chain.handle(&[0x9F], &mut resp), Some(2));
        assert_eq!(resp[1], 1);
    }

    #[test]
    fn falls_through_to_the_second_handler() {
        let mut chain =
            VendorChain::new(Fixed { id: 0x9F, reply: 1 }, Fixed { id: 0x8A, reply: 2 });
        let mut resp = [0u8; 4];
        assert_eq!(chain.handle(&[0x8A], &mut resp), Some(2));
        assert_eq!(resp[1], 2);
    }

    #[test]
    fn declines_when_neither_side_answers() {
        let mut chain =
            VendorChain::new(Fixed { id: 0x9F, reply: 1 }, Fixed { id: 0x8A, reply: 2 });
        let mut resp = [0u8; 4];
        assert_eq!(chain.handle(&[0x00], &mut resp), None);
    }

    #[test]
    fn handles_is_true_whenever_either_side_handles() {
        let neither: bool = <VendorChain<NoVendor, NoVendor> as VendorHandler>::HANDLES;
        let first_only: bool = <VendorChain<Fixed, NoVendor> as VendorHandler>::HANDLES;
        let second_only: bool = <VendorChain<NoVendor, Fixed> as VendorHandler>::HANDLES;
        assert!(!neither);
        assert!(first_only);
        assert!(second_only);
    }
}
