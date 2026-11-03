//! Revision-gated endpoint retirement for the RP2040 USB bus.

use rp2040_hal::{pac, usb::UsbBus as HalBus};
use usb_device::{
    bus::{PollResult, UsbBus as Bus},
    endpoint::{EndpointAddress, EndpointType},
    Result, UsbDirection, UsbError,
};

/// Retires halted endpoint buffers on B2+; B0/B1 retain HAL behavior because of RP2040-E2.
pub struct UsbBus {
    inner: HalBus,
    can_abort: bool,
    out_sizes: [u16; 16],
    allocated: u32,
}

/// One bit per endpoint direction, at the endpoint's DPRAM buffer-control index.
fn slot(ep: EndpointAddress) -> u32 {
    1 << (ep.index() * 2 + usize::from(ep.is_out()))
}

impl UsbBus {
    pub(crate) fn new(inner: HalBus) -> Self {
        // SYSINFO is read-only; the bus owns all accesses to the USB controller.
        let revision = unsafe { &*pac::SYSINFO::ptr() }
            .chip_id()
            .read()
            .revision()
            .bits();
        Self {
            inner,
            can_abort: revision >= 2,
            out_sizes: [0; 16],
            allocated: 0,
        }
    }

    fn retire(&self, ep: EndpointAddress, stalled: bool) {
        // The caller holds the same critical section used by the HAL.
        let regs = unsafe { &*pac::USBCTRL_REGS::ptr() };
        let ram = unsafe { &*pac::USBCTRL_DPRAM::ptr() };
        let index = ep.index() * 2 + usize::from(ep.is_out());
        let mask = 1u32 << index;
        regs.ep_abort()
            .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
        // Fail closed rather than touching a buffer still owned by hardware.
        let mut remaining = 100_000;
        while regs.ep_abort_done().read().bits() & mask == 0 {
            assert!(remaining != 0, "USB endpoint abort did not complete");
            remaining -= 1;
        }
        let control = ram.ep_buffer_control(index);
        control.write(|w| unsafe {
            w.stall().bit(stalled);
            // The HAL flips IN PID before writing; OUT consumes the stored PID directly.
            w.pid_0().bit(ep.is_in());
            w.length_0().bits(if ep.is_out() {
                self.out_sizes[ep.index()]
            } else {
                0
            })
        });
        regs.buff_status().write(|w| unsafe { w.bits(mask) });
        if ep.is_out() && !stalled {
            // Buffer metadata must reach clk_usb before AVAILABLE changes.
            cortex_m::asm::delay(12);
            control.modify(|_, w| w.available_0().set_bit());
        }
        regs.ep_abort_done().write(|w| unsafe { w.bits(mask) });
        regs.ep_abort()
            .modify(|r, w| unsafe { w.bits(r.bits() & !mask) });
    }
}

impl Bus for UsbBus {
    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = HalBus::QUIRK_SET_ADDRESS_BEFORE_STATUS;

    fn alloc_ep(
        &mut self,
        dir: UsbDirection,
        addr: Option<EndpointAddress>,
        kind: EndpointType,
        size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        let ep = self.inner.alloc_ep(dir, addr, kind, size, interval)?;
        self.allocated |= slot(ep);
        if ep.is_out() {
            self.out_sizes[ep.index()] = size;
        }
        Ok(ep)
    }
    fn enable(&mut self) {
        self.inner.enable();
    }
    fn reset(&self) {
        self.inner.reset();
    }
    fn set_device_address(&self, addr: u8) {
        self.inner.set_device_address(addr);
    }
    fn write(&self, ep: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if !self.can_abort || ep.index() == 0 {
            return self.inner.write(ep, buf);
        }
        critical_section::with(|_| {
            if self.inner.is_stalled(ep) {
                return Err(UsbError::WouldBlock);
            }
            self.inner.write(ep, buf)
        })
    }
    fn read(&self, ep: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !self.can_abort || ep.index() == 0 {
            return self.inner.read(ep, buf);
        }
        critical_section::with(|_| {
            if self.inner.is_stalled(ep) {
                return Err(UsbError::WouldBlock);
            }
            self.inner.read(ep, buf)
        })
    }
    /// # Panics
    /// Panics if B2+ hardware fails to acknowledge endpoint abort within the polling bound.
    fn set_stalled(&self, ep: EndpointAddress, stalled: bool) {
        // usb-device forwards any wIndex; an endpoint never allocated has nothing to retire.
        if self.allocated & slot(ep) == 0 {
            return;
        }
        if !self.can_abort || ep.index() == 0 {
            self.inner.set_stalled(ep, stalled);
            return;
        }
        critical_section::with(|_| self.retire(ep, stalled));
    }
    fn is_stalled(&self, ep: EndpointAddress) -> bool {
        self.inner.is_stalled(ep)
    }
    fn suspend(&self) {
        self.inner.suspend();
    }
    fn resume(&self) {
        self.inner.resume();
    }
    fn poll(&self) -> PollResult {
        self.inner.poll()
    }
    fn force_reset(&self) -> Result<()> {
        self.inner.force_reset()
    }
}

#[cfg(test)]
mod tests {
    use super::slot;
    use usb_device::endpoint::EndpointAddress;

    fn mask(eps: &[u8]) -> u32 {
        eps.iter()
            .fold(0, |m, &ep| m | slot(EndpointAddress::from(ep)))
    }

    #[test]
    fn a_twin_of_an_allocated_endpoint_is_not_allocated() {
        let vcom = mask(&[0x00, 0x80, 0x01, 0x81, 0x82, 0x02, 0x83]);
        for ep in [0x00, 0x80, 0x01, 0x81, 0x82, 0x02, 0x83] {
            assert_ne!(vcom & slot(EndpointAddress::from(ep)), 0, "{ep:#04x}");
        }
        for ep in [0x03, 0x84, 0x04, 0x0f, 0x8f] {
            assert_eq!(vcom & slot(EndpointAddress::from(ep)), 0, "{ep:#04x}");
        }
    }
}
