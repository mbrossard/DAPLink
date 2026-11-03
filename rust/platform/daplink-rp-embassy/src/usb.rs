//! RP USB composition restoring DATA0 when a halt is cleared, and holding the halt until then.

use daplink_core::rp_buffer_control::{clear_in_word, clear_out_words, halt_words};
use driver::{Direction, EndpointAddress, EndpointAllocError, EndpointType, Event, Unsupported};
use embassy_rp::{pac, peripherals::USB, usb};
use embassy_usb_driver as driver;

/// Endpoint numbers the controller has; embassy-usb 0.6 masks wIndex to below this.
const EP_COUNT: usize = 16;

/// Wraps the embassy-rp driver to reset endpoint data toggles on a halt clear.
pub struct RpUsbDriver<'d> {
    inner: usb::Driver<'d, USB>,
    eps: Endpoints,
}

impl<'d> RpUsbDriver<'d> {
    /// Wrap the embassy-rp USB driver.
    pub fn new(inner: usb::Driver<'d, USB>) -> Self {
        Self {
            inner,
            eps: Endpoints {
                out_sizes: [0; EP_COUNT],
                in_used: 0,
            },
        }
    }
}

/// What the wrapper must know of each allocated endpoint; zero size means an absent OUT.
#[derive(Clone, Copy)]
struct Endpoints {
    out_sizes: [u16; EP_COUNT],
    in_used: u16,
}

/// Bus half of [`RpUsbDriver`].
pub struct RpUsbBus<'d> {
    inner: usb::Bus<'d, USB>,
    eps: Endpoints,
}

impl<'d> driver::Driver<'d> for RpUsbDriver<'d> {
    type EndpointOut = <usb::Driver<'d, USB> as driver::Driver<'d>>::EndpointOut;
    type EndpointIn = <usb::Driver<'d, USB> as driver::Driver<'d>>::EndpointIn;
    type ControlPipe = <usb::Driver<'d, USB> as driver::Driver<'d>>::ControlPipe;
    type Bus = RpUsbBus<'d>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        let ep = self
            .inner
            .alloc_endpoint_out(ep_type, ep_addr, max_packet_size, interval_ms)?;
        self.eps.out_sizes[driver::Endpoint::info(&ep).addr.index()] = max_packet_size;
        Ok(ep)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        let ep = self
            .inner
            .alloc_endpoint_in(ep_type, ep_addr, max_packet_size, interval_ms)?;
        self.eps.in_used |= 1 << driver::Endpoint::info(&ep).addr.index();
        Ok(ep)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let (inner, control) = self.inner.start(control_max_packet_size);
        (
            RpUsbBus {
                inner,
                eps: self.eps,
            },
            control,
        )
    }
}

impl driver::Bus for RpUsbBus<'_> {
    async fn enable(&mut self) {
        self.inner.enable().await
    }
    async fn disable(&mut self) {
        self.inner.disable().await
    }
    async fn poll(&mut self) -> Event {
        self.inner.poll().await
    }

    fn endpoint_set_enabled(&mut self, ep: EndpointAddress, enabled: bool) {
        self.inner.endpoint_set_enabled(ep, enabled)
    }

    fn endpoint_set_stalled(&mut self, ep: EndpointAddress, stalled: bool) {
        let n = ep.index();
        let dir_in = ep.direction() == Direction::In;
        let mps = self.eps.out_sizes[n];
        let allocated = if dir_in {
            self.eps.in_used & (1 << n) != 0
        } else {
            mps != 0
        };
        if n == 0 || !allocated {
            return self.inner.endpoint_set_stalled(ep, stalled);
        }
        let ctrl = if dir_in {
            pac::USB_DPRAM.ep_in_buffer_control(n)
        } else {
            pac::USB_DPRAM.ep_out_buffer_control(n)
        };
        // Keep endpoint tasks out until the word is whole; the inner call only wakes them.
        critical_section::with(|_| {
            let (first, second) = match (stalled, dir_in) {
                (true, _) => halt_words(dir_in, mps),
                (false, true) => (clear_in_word(), clear_in_word()),
                (false, false) => clear_out_words(mps),
            };
            ctrl.write(|w| w.0 = first);
            self.inner.endpoint_set_stalled(ep, stalled);
            if second != first {
                cortex_m::asm::delay(12);
                ctrl.write(|w| w.0 = second);
            }
        });
    }

    fn endpoint_is_stalled(&mut self, ep: EndpointAddress) -> bool {
        self.inner.endpoint_is_stalled(ep)
    }

    fn force_reset(&mut self) -> Result<(), Unsupported> {
        self.inner.force_reset()
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        self.inner.remote_wakeup().await
    }
}
