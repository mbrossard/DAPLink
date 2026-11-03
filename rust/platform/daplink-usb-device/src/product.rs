//! Compile-time selection of the trailing mass-storage interface.

use crate::MscClass;
use usb_device::bus::{UsbBus, UsbBusAllocator};
use usb_device::class::UsbClass;
use usb_device::device::UsbDevice;

/// A device without a mass-storage interface or its transport state.
pub struct NoMsc;

/// The optional final class, allocated after DAP and CDC to preserve interface numbering.
pub trait MassStorage<'a, B: UsbBus, const EP: usize>: Sized {
    fn new(alloc: &'a UsbBusAllocator<B>) -> Self;
    fn poll(
        &mut self,
        device: &mut UsbDevice<'a, B>,
        dap: &mut dyn UsbClass<B>,
        vendor: &mut dyn UsbClass<B>,
    ) -> bool;
    fn poll_vcom(
        &mut self,
        device: &mut UsbDevice<'a, B>,
        dap: &mut dyn UsbClass<B>,
        vendor: &mut dyn UsbClass<B>,
        serial: &mut dyn UsbClass<B>,
    ) -> bool;
}

impl<'a, B: UsbBus, const EP: usize> MassStorage<'a, B, EP> for NoMsc {
    fn new(_: &'a UsbBusAllocator<B>) -> Self {
        Self
    }

    fn poll(
        &mut self,
        device: &mut UsbDevice<'a, B>,
        dap: &mut dyn UsbClass<B>,
        vendor: &mut dyn UsbClass<B>,
    ) -> bool {
        device.poll(&mut [dap, vendor])
    }

    fn poll_vcom(
        &mut self,
        device: &mut UsbDevice<'a, B>,
        dap: &mut dyn UsbClass<B>,
        vendor: &mut dyn UsbClass<B>,
        serial: &mut dyn UsbClass<B>,
    ) -> bool {
        device.poll(&mut [dap, vendor, serial])
    }
}

impl<'a, B: UsbBus, const EP: usize> MassStorage<'a, B, EP> for MscClass<'a, B, EP> {
    fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        MscClass::new(alloc)
    }

    fn poll(
        &mut self,
        device: &mut UsbDevice<'a, B>,
        dap: &mut dyn UsbClass<B>,
        vendor: &mut dyn UsbClass<B>,
    ) -> bool {
        device.poll(&mut [dap, vendor, self])
    }

    fn poll_vcom(
        &mut self,
        device: &mut UsbDevice<'a, B>,
        dap: &mut dyn UsbClass<B>,
        vendor: &mut dyn UsbClass<B>,
        serial: &mut dyn UsbClass<B>,
    ) -> bool {
        device.poll(&mut [dap, vendor, serial, self])
    }
}
