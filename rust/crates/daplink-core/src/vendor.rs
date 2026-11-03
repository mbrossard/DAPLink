//! Vendor-command port: CMSIS-DAP ids `0x80..=0x9F`, routed to a [`VendorHandler`] before the engine.

/// First vendor-specific CMSIS-DAP command id (`ID_DAP_Vendor0`).
pub const VENDOR_CMD_FIRST: u8 = 0x80;
/// Last vendor-specific CMSIS-DAP command id (`ID_DAP_Vendor31`).
pub const VENDOR_CMD_LAST: u8 = 0x9F;

/// Handler for vendor-specific CMSIS-DAP commands.
pub trait VendorHandler {
    /// Whether this handler handles anything; `false` compiles vendor routing out.
    const HANDLES: bool = true;

    /// Handle one vendor command from `response[1]` on, returning the total length; `None` declines.
    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize>;

    /// The host session ended: retire whatever it left open.
    fn session_reset(&mut self) {}
}

impl<V: VendorHandler + ?Sized> VendorHandler for &mut V {
    const HANDLES: bool = V::HANDLES;

    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        (**self).handle(report, response)
    }

    fn session_reset(&mut self) {
        (**self).session_reset()
    }
}

/// No vendor commands: everything falls through to the engine.
pub struct NoVendor;

impl VendorHandler for NoVendor {
    const HANDLES: bool = false;

    fn handle(&mut self, _report: &[u8], _response: &mut [u8]) -> Option<usize> {
        None
    }
}

macro_rules! impl_vendor_tuple {
    ($($handler:ident:$field:tt),+ $(,)?) => {
        impl<$($handler: VendorHandler),+> VendorHandler for ($($handler,)+) {
            const HANDLES: bool = false $(|| $handler::HANDLES)+;

            fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
                $(
                    if let Some(len) = self.$field.handle(report, response) {
                        return Some(len);
                    }
                )+
                None
            }

            fn session_reset(&mut self) {
                $(self.$field.session_reset();)+
            }
        }
    };
}

impl_vendor_tuple!(A:0, B:1);
impl_vendor_tuple!(A:0, B:1, C:2);
impl_vendor_tuple!(A:0, B:1, C:2, D:3);

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
    fn tuple_dispatch_is_left_to_right() {
        let mut handlers = (
            Fixed { id: 0x9F, reply: 1 },
            Fixed { id: 0x9F, reply: 2 },
            Fixed { id: 0x8A, reply: 3 },
        );
        let mut resp = [0u8; 4];
        assert_eq!(handlers.handle(&[0x9F], &mut resp), Some(2));
        assert_eq!(resp[1], 1);
        assert_eq!(handlers.handle(&[0x8A], &mut resp), Some(2));
        assert_eq!(resp[1], 3);
        assert_eq!(handlers.handle(&[0x00], &mut resp), None);
    }

    struct Resets(u32);

    impl VendorHandler for Resets {
        fn handle(&mut self, _report: &[u8], _response: &mut [u8]) -> Option<usize> {
            None
        }

        fn session_reset(&mut self) {
            self.0 += 1;
        }
    }

    #[test]
    fn tuple_session_reset_reaches_every_handler() {
        let mut handlers = (Resets(0), Resets(0));
        handlers.session_reset();
        assert_eq!((handlers.0 .0, handlers.1 .0), (1, 1));
    }

    #[test]
    fn tuple_handles_is_the_union() {
        let none: bool = <(NoVendor, NoVendor, NoVendor) as VendorHandler>::HANDLES;
        let one: bool = <(NoVendor, Fixed, NoVendor, NoVendor) as VendorHandler>::HANDLES;
        assert!(!none);
        assert!(one);
    }

    #[test]
    fn a_borrowed_handler_answers_as_its_referent() {
        let borrowed: bool = <&mut NoVendor as VendorHandler>::HANDLES;
        assert!(!borrowed);
        let mut fixed = Fixed { id: 0x8A, reply: 7 };
        let mut handlers = (NoVendor, &mut fixed);
        let mut resp = [0u8; 4];
        assert_eq!(handlers.handle(&[0x8A], &mut resp), Some(2));
        assert_eq!(resp[1], 7);
    }
}
