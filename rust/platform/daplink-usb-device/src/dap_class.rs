//! Polled CMSIS-DAP v2 bulk class: one bulk OUT for commands, one bulk IN for responses.

use daplink_rs::daplink_core::DAP_V2_INTERFACE_CLASS;
use usb_device::class_prelude::*;
use usb_device::Result;

/// CMSIS-DAP v2 bulk interface; exactly one command or response per transfer, never reassembled.
pub struct CmsisDapV2<'a, B: UsbBus> {
    interface: InterfaceNumber,
    name: StringIndex,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    out_pending: bool,
}

impl<B: UsbBus> CmsisDapV2<'_, B> {
    /// Allocate the interface, its string, and the two bulk endpoints (read then write).
    pub fn new(max_packet_size: u16, alloc: &UsbBusAllocator<B>) -> CmsisDapV2<'_, B> {
        CmsisDapV2 {
            interface: alloc.interface(),
            name: alloc.string(),
            read_ep: alloc.bulk(max_packet_size),
            write_ep: alloc.bulk(max_packet_size),
            out_pending: false,
        }
    }

    /// Read a landed command packet into `buf`, returning its length, or `None` when none is pending.
    pub fn read_command(&mut self, buf: &mut [u8]) -> Option<usize> {
        // A speculative read can see a still-arriving packet against the previous byte count.
        if !core::mem::take(&mut self.out_pending) {
            return None;
        }
        match self.read_ep.read(buf) {
            Ok(size) if size > 0 => Some(size),
            _ => None,
        }
    }

    /// Write one response packet, returning `false` when it is oversized or the endpoint is busy.
    pub fn write(&mut self, data: &[u8]) -> bool {
        if data.len() > self.write_ep.max_packet_size() as usize {
            return false;
        }
        self.write_ep.write(data).is_ok()
    }
}

impl<B: UsbBus> UsbClass<B> for CmsisDapV2<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        let (class, subclass, protocol) = DAP_V2_INTERFACE_CLASS;
        writer.interface_alt(
            self.interface,
            0,
            class,
            subclass,
            protocol,
            Some(self.name),
        )?;
        writer.endpoint(&self.read_ep)?;
        writer.endpoint(&self.write_ep)
    }

    fn get_string(&self, index: StringIndex, _lang_id: LangID) -> Option<&str> {
        (index == self.name).then_some("CMSIS-DAP v2 Interface")
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        // Sticky until a read consumes it, so a backend reporting the event once cannot lose it.
        if addr == self.read_ep.address() {
            self.out_pending = true;
        }
    }

    fn reset(&mut self) {
        // Any packet the old session landed is gone with the endpoint's re-initialization.
        self.out_pending = false;
    }
}
