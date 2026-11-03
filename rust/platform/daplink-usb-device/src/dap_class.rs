//! Polled CMSIS-DAP v2 bulk class: one bulk OUT for commands, one bulk IN for responses.

use daplink_rs::daplink_core::{EndpointSize, PacketSize, DAP_V2_INTERFACE_CLASS};
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, Request, RequestType};
use usb_device::Result;

/// `packet`, failing the build if it exceeds `endpoint`: [`CmsisDapV2::write`] sends one transfer.
pub const fn single_transfer_packet(packet: PacketSize, endpoint: EndpointSize) -> PacketSize {
    assert!(
        packet.get() <= endpoint.get(),
        "DAP packet size exceeds the bulk endpoint"
    );
    packet
}

/// CMSIS-DAP v2 bulk interface; exactly one command or response per transfer, never reassembled.
pub struct CmsisDapV2<'a, B: UsbBus> {
    interface: InterfaceNumber,
    name: StringIndex,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    out_pending: bool,
    reset_pending: bool,
    /// The host halted the IN endpoint; some buses would still arm a write behind the stall.
    in_halted: bool,
}

/// Whether `req` sets (`Some(true)`) or clears (`Some(false)`) the halt on exactly `ep`.
pub(crate) fn halt_request(req: &Request, ep: EndpointAddress) -> Option<bool> {
    let halt = req.request_type == RequestType::Standard
        && req.recipient == Recipient::Endpoint
        && req.value == Request::FEATURE_ENDPOINT_HALT
        && req.index == u16::from(u8::from(ep));
    match req.request {
        Request::SET_FEATURE if halt => Some(true),
        Request::CLEAR_FEATURE if halt => Some(false),
        _ => None,
    }
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
            reset_pending: false,
            in_halted: false,
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
        if self.in_halted || data.len() > self.write_ep.max_packet_size() as usize {
            return false;
        }
        self.write_ep.write(data).is_ok()
    }

    pub(crate) fn take_reset(&mut self) -> bool {
        core::mem::take(&mut self.reset_pending)
    }
}

impl<B: UsbBus> UsbClass<B> for CmsisDapV2<'_, B> {
    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        if let Some(halted) = halt_request(req, self.write_ep.address()) {
            self.in_halted = halted;
        }
        if req.request_type == RequestType::Standard
            && req.recipient == Recipient::Device
            && req.request == Request::SET_CONFIGURATION
            && req.index == 0
            && req.length == 0
            && req.value <= 1
        {
            // Configuration changes retire both endpoint buffers, not just the software queue.
            self.read_ep.stall();
            self.write_ep.stall();
            if req.value == 1 {
                self.read_ep.unstall();
                self.write_ep.unstall();
            }
            self.out_pending = false;
            self.reset_pending = true;
            self.in_halted = false;
        }
        // Leave the standard request and its status stage to UsbDevice.
    }

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
        self.in_halted = false;
        // Coalesce resets until session cleanup observes them, without a wrapping counter.
        self.reset_pending = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use daplink_core::{DapQueue, Submit, DAP_TRANSFER_ABORT};
    use std::sync::{Arc, Mutex};
    use std::vec::Vec;
    use usb_device::bus::PollResult;
    use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
    use usb_device::{UsbDirection, UsbError};

    /// Hands the class one queued OUT packet per read and records every response written.
    struct CommandBus {
        out: Arc<Mutex<Vec<Vec<u8>>>>,
        sent: Arc<Mutex<Vec<Vec<u8>>>>,
        next: usize,
    }

    impl UsbBus for CommandBus {
        fn alloc_ep(
            &mut self,
            direction: UsbDirection,
            address: Option<EndpointAddress>,
            _kind: EndpointType,
            _size: u16,
            _interval: u8,
        ) -> Result<EndpointAddress> {
            Ok(address.unwrap_or_else(|| {
                let result = EndpointAddress::from_parts(self.next, direction);
                self.next += 1;
                result
            }))
        }
        fn enable(&mut self) {}
        fn reset(&self) {}
        fn set_device_address(&self, _address: u8) {}
        fn write(&self, endpoint: EndpointAddress, data: &[u8]) -> Result<usize> {
            assert_eq!(endpoint.index(), 2);
            self.sent.lock().unwrap().push(data.to_vec());
            Ok(data.len())
        }
        fn read(&self, endpoint: EndpointAddress, data: &mut [u8]) -> Result<usize> {
            if endpoint.index() != 1 {
                return Err(UsbError::WouldBlock);
            }
            let mut out = self.out.lock().unwrap();
            if out.is_empty() {
                return Err(UsbError::WouldBlock);
            }
            let packet = out.remove(0);
            data[..packet.len()].copy_from_slice(&packet);
            Ok(packet.len())
        }
        fn set_stalled(&self, _endpoint: EndpointAddress, _stalled: bool) {}
        fn is_stalled(&self, _endpoint: EndpointAddress) -> bool {
            false
        }
        fn suspend(&self) {}
        fn resume(&self) {}
        fn poll(&self) -> PollResult {
            PollResult::None
        }
    }

    #[test]
    fn transfer_abort_is_consumed_without_a_slot_or_a_response() {
        let out = Arc::new(Mutex::new(std::vec![
            std::vec![0x41u8, 1],
            std::vec![DAP_TRANSFER_ABORT],
            std::vec![0x42u8, 2],
        ]));
        let sent = Arc::new(Mutex::new(Vec::new()));
        let alloc = UsbBusAllocator::new(CommandBus {
            out: out.clone(),
            sent: sent.clone(),
            next: 1,
        });
        let mut class = CmsisDapV2::new(64, &alloc);
        // Endpoints only become usable once a built device has frozen the allocator.
        let _device = UsbDeviceBuilder::new(&alloc, UsbVidPid(0x0d28, 0x0204)).build();
        // Depth two: a third slot does not exist, so an abort that took one would refuse 0x42.
        let mut queue: DapQueue<2, 64> = DapQueue::new();

        let read_ep = EndpointAddress::from_parts(1, UsbDirection::Out);
        let mut outcomes = Vec::new();
        for _ in 0..3 {
            class.endpoint_out(read_ep);
            outcomes.push(
                queue
                    .receive_with(|slot| class.read_command(slot))
                    .expect("a free slot accepts one endpoint packet")
                    .expect("a landed packet is read"),
            );
        }
        assert_eq!(
            outcomes,
            [Submit::Enqueued, Submit::AbortIntercepted, Submit::Enqueued]
        );

        while queue
            .execute_next_with(|request, response| {
                response[0] = request[0] + 0x40;
                1
            })
            .expect("the response ring has room")
            .is_some()
        {}
        while let Some(response) = queue.pending_response() {
            assert!(class.write(response));
            queue.discard_response();
        }

        // Only the two real commands were answered, and in the order the host sent them.
        assert_eq!(
            *sent.lock().unwrap(),
            std::vec![std::vec![0x81u8], std::vec![0x82u8]]
        );
    }
}
