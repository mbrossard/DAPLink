//! F1 USB composition restoring DATA0 when endpoints are deconfigured or a halt is cleared.

use daplink_rs::daplink_core::stm32_epr::clear_halt_step;
use driver::{Direction, EndpointAddress, EndpointAllocError, EndpointType, Event, Unsupported};
use embassy_stm32::{pac, peripherals::USB, usb};
use embassy_usb_driver as driver;

/// Endpoint registers the peripheral has; embassy-usb 0.6 passes any wIndex through.
const EP_COUNT: usize = 8;

/// Wraps the F1 driver to reset endpoint data toggles on disable.
pub struct F1UsbDriver<'d>(usb::Driver<'d, USB>);

impl<'d> F1UsbDriver<'d> {
    /// Wrap the embassy-stm32 USB driver.
    pub fn new(inner: usb::Driver<'d, USB>) -> Self {
        Self(inner)
    }
}

/// Bus half of [`F1UsbDriver`].
pub struct F1UsbBus<'d>(usb::Bus<'d, USB>);

impl<'d> driver::Driver<'d> for F1UsbDriver<'d> {
    type EndpointOut = <usb::Driver<'d, USB> as driver::Driver<'d>>::EndpointOut;
    type EndpointIn = <usb::Driver<'d, USB> as driver::Driver<'d>>::EndpointIn;
    type ControlPipe = <usb::Driver<'d, USB> as driver::Driver<'d>>::ControlPipe;
    type Bus = F1UsbBus<'d>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.0
            .alloc_endpoint_out(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.0
            .alloc_endpoint_in(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let (bus, control) = self.0.start(control_max_packet_size);
        (F1UsbBus(bus), control)
    }
}

impl driver::Bus for F1UsbBus<'_> {
    async fn enable(&mut self) {
        self.0.enable().await
    }
    async fn disable(&mut self) {
        self.0.disable().await
    }
    async fn poll(&mut self) -> Event {
        self.0.poll().await
    }

    fn endpoint_set_enabled(&mut self, ep: EndpointAddress, enabled: bool) {
        // Keep endpoint tasks and the USB ISR out until the toggle repair finishes.
        critical_section::with(|_| {
            self.0.endpoint_set_enabled(ep, enabled);
            if !enabled {
                let reg = pac::USB.epr(ep.index());
                let r = reg.read();
                let mut w = r;
                // CTR clears on zero; STAT and DTOG toggle on one (RM0008 USB_EPnR).
                w.set_ctr_rx(true);
                w.set_ctr_tx(true);
                w.set_stat_rx(pac::usb::vals::Stat::from_bits(0));
                w.set_stat_tx(pac::usb::vals::Stat::from_bits(0));
                w.set_dtog_rx(false);
                w.set_dtog_tx(false);
                match ep.direction() {
                    Direction::In => w.set_dtog_tx(r.dtog_tx()),
                    Direction::Out => w.set_dtog_rx(r.dtog_rx()),
                }
                reg.write_value(w);
            }
        });
    }

    fn endpoint_set_stalled(&mut self, ep: EndpointAddress, stalled: bool) {
        // EP0 has no halt feature, and embassy-stm32 0.6's own unstall spins on an unhalted endpoint.
        if ep.index() == 0 || ep.index() >= EP_COUNT {
            return;
        }
        if stalled {
            return self.0.endpoint_set_stalled(ep, true);
        }
        critical_section::with(|_| {
            let reg = pac::USB.epr(ep.index());
            let dir_in = ep.direction() == Direction::In;
            while let Some(w) = clear_halt_step(reg.read().0, dir_in) {
                reg.write_value(pac::usb::regs::Epr(w));
            }
            // A register no-op at NAK that wakes a writer parked on the stall.
            if dir_in && reg.read().stat_tx() == pac::usb::vals::Stat::NAK {
                self.0.endpoint_set_enabled(ep, true);
            }
        });
    }

    fn endpoint_is_stalled(&mut self, ep: EndpointAddress) -> bool {
        ep.index() < EP_COUNT && self.0.endpoint_is_stalled(ep)
    }

    fn force_reset(&mut self) -> Result<(), Unsupported> {
        self.0.force_reset()
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        self.0.remote_wakeup().await
    }
}
