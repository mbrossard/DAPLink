//! VCOM realized-baud query (`0x9E`): echoed id, realized and applied rates, then a `flags` word.

use daplink_core::vendor::VendorHandler;

/// Vendor command id for the VCOM realized-baud query (`ID_DAP_Vendor30`).
pub const VCOM_BAUD_CMD_ID: u8 = 0x9E;

/// The first reply form: echoed id and little-endian realized rate; hosts tell forms apart by length.
pub const VCOM_BAUD_V1_RESPONSE_LEN: usize = 5;

/// Total response length: the echoed id, `realized`, `applied`, then `flags`.
pub const VCOM_BAUD_RESPONSE_LEN: usize = 13;
const _: () = assert!(VCOM_BAUD_RESPONSE_LEN <= 64);

/// `flags` bit: the seam observes whether a requested rate reached the divisor.
pub const VCOM_BAUD_APPLIED_VALID: u32 = 1 << 0;
/// `flags` bit: no rate change is in flight; meaningful only with [`VCOM_BAUD_APPLIED_VALID`].
pub const VCOM_BAUD_SETTLED: u32 = 1 << 1;

/// What the probe answers `0x9E` with.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct VcomBaudState {
    /// Rate the divisor now on the wire produces, `0` before the first programming.
    pub realized: u32,
    /// Nominal rate that divisor was programmed for, `0` where unsourced or unset.
    pub applied: u32,
    /// `VCOM_BAUD_*` bits.
    pub flags: u32,
}

impl VcomBaudState {
    /// A seam that defers a rate change until its transmitter is idle.
    pub const fn deferred(realized: u32, applied: u32, settled: bool) -> Self {
        let settled = if settled { VCOM_BAUD_SETTLED } else { 0 };
        Self {
            realized,
            applied,
            flags: VCOM_BAUD_APPLIED_VALID | settled,
        }
    }

    /// The reply body after the echoed id, little-endian.
    pub fn to_le_bytes(&self) -> [u8; VCOM_BAUD_RESPONSE_LEN - 1] {
        let mut out = [0u8; VCOM_BAUD_RESPONSE_LEN - 1];
        for (word, value) in out
            .chunks_exact_mut(4)
            .zip([self.realized, self.applied, self.flags])
        {
            word.copy_from_slice(&value.to_le_bytes());
        }
        out
    }
}

/// Answers [`VCOM_BAUD_CMD_ID`] with the injected reader's state and declines everything else.
pub struct VcomBaudVendor {
    state: fn() -> VcomBaudState,
}

impl VcomBaudVendor {
    /// `state` reads the last published rate and whether a request is still pending.
    pub const fn new(state: fn() -> VcomBaudState) -> Self {
        Self { state }
    }
}

impl VendorHandler for VcomBaudVendor {
    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        if report.first() != Some(&VCOM_BAUD_CMD_ID) {
            return None;
        }
        response[1..VCOM_BAUD_RESPONSE_LEN].copy_from_slice(&(self.state)().to_le_bytes());
        Some(VCOM_BAUD_RESPONSE_LEN)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settled() -> VcomBaudState {
        VcomBaudState::deferred(2_666_666, 2_666_666, true)
    }

    fn in_flight() -> VcomBaudState {
        VcomBaudState::deferred(115_207, 115_200, false)
    }

    #[test]
    fn answers_the_query_with_the_readers_state_le() {
        let mut v = VcomBaudVendor::new(settled);
        let mut resp = [0u8; 16];
        resp[0] = VCOM_BAUD_CMD_ID; // the probe echoes the id before the handler runs
        let len = v.handle(&[VCOM_BAUD_CMD_ID], &mut resp);
        assert_eq!(len, Some(VCOM_BAUD_RESPONSE_LEN));
        assert_eq!(&resp[..5], &[0x9E, 0xAA, 0xB0, 0x28, 0x00]);
        assert_eq!(
            u32::from_le_bytes(resp[5..9].try_into().unwrap()),
            2_666_666
        );
        assert_eq!(
            u32::from_le_bytes(resp[9..13].try_into().unwrap()),
            VCOM_BAUD_APPLIED_VALID | VCOM_BAUD_SETTLED
        );
    }

    #[test]
    fn an_in_flight_change_clears_the_settled_bit_but_keeps_the_valid_one() {
        let mut v = VcomBaudVendor::new(in_flight);
        let mut resp = [0u8; 16];
        let len = v.handle(&[VCOM_BAUD_CMD_ID], &mut resp);
        assert_eq!(len, Some(VCOM_BAUD_RESPONSE_LEN));
        let flags = u32::from_le_bytes(resp[9..13].try_into().unwrap());
        assert_eq!(flags & VCOM_BAUD_APPLIED_VALID, VCOM_BAUD_APPLIED_VALID);
        assert_eq!(flags & VCOM_BAUD_SETTLED, 0);
    }

    #[test]
    fn an_unsourcing_seam_leaves_both_bits_clear() {
        fn unsourced() -> VcomBaudState {
            VcomBaudState {
                realized: 9_600,
                ..VcomBaudState::default()
            }
        }
        let mut v = VcomBaudVendor::new(unsourced);
        let mut resp = [0u8; 16];
        v.handle(&[VCOM_BAUD_CMD_ID], &mut resp);
        assert_eq!(u32::from_le_bytes(resp[9..13].try_into().unwrap()), 0);
    }

    #[test]
    fn declines_every_other_command_id() {
        let mut v = VcomBaudVendor::new(settled);
        let mut resp = [0u8; 16];
        assert_eq!(v.handle(&[0x9F], &mut resp), None);
        assert_eq!(v.handle(&[0x00], &mut resp), None);
        assert_eq!(v.handle(&[], &mut resp), None);
    }

    #[test]
    fn chains_behind_another_handler_without_shadowing_it() {
        struct Other;
        impl VendorHandler for Other {
            fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
                (report.first() == Some(&0x9F)).then(|| {
                    response[1] = 0x55;
                    2
                })
            }
        }
        let mut chain = (Other, VcomBaudVendor::new(settled));
        let mut resp = [0u8; 16];
        assert_eq!(chain.handle(&[0x9F], &mut resp), Some(2));
        assert_eq!(
            chain.handle(&[VCOM_BAUD_CMD_ID], &mut resp),
            Some(VCOM_BAUD_RESPONSE_LEN)
        );
        assert_eq!(chain.handle(&[0x8A], &mut resp), None);
    }
}
