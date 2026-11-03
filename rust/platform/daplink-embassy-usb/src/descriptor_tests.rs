//! The DAP-only device's descriptors, read back through embassy-usb's control pipe.

use super::*;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Waker};
use daplink_rs::daplink_core::DapQueue;
use embassy_usb::driver::{
    Bus, ControlPipe, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event,
    Unsupported,
};
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec::Vec;

struct Ep(EndpointInfo);

impl Endpoint for Ep {
    fn info(&self) -> &EndpointInfo {
        &self.0
    }

    async fn wait_enabled(&mut self) {}
}

impl EndpointOut for Ep {
    async fn read(&mut self, _: &mut [u8]) -> Result<usize, EndpointError> {
        core::future::pending().await
    }
}

impl EndpointIn for Ep {
    async fn write(&mut self, _: &[u8]) -> Result<(), EndpointError> {
        core::future::pending().await
    }
}

/// The host's side of EP0: queued SETUP packets and each completed IN data stage.
#[derive(Default)]
struct Ep0 {
    setups: VecDeque<[u8; 8]>,
    replies: Vec<Vec<u8>>,
    stage: Vec<u8>,
}

struct Host(Rc<RefCell<Ep0>>);
struct Pipe(Rc<RefCell<Ep0>>);
struct IdleBus;

impl Driver<'static> for Host {
    type EndpointOut = Ep;
    type EndpointIn = Ep;
    type ControlPipe = Pipe;
    type Bus = IdleBus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        _: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Ep, EndpointAllocError> {
        Ok(Ep(EndpointInfo {
            addr: EndpointAddress::from(0x01),
            ep_type,
            max_packet_size,
            interval_ms,
        }))
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        _: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Ep, EndpointAllocError> {
        Ok(Ep(EndpointInfo {
            addr: EndpointAddress::from(0x81),
            ep_type,
            max_packet_size,
            interval_ms,
        }))
    }

    fn start(self, _: u16) -> (IdleBus, Pipe) {
        (IdleBus, Pipe(self.0))
    }
}

impl Bus for IdleBus {
    async fn enable(&mut self) {}
    async fn disable(&mut self) {}
    async fn poll(&mut self) -> Event {
        core::future::pending().await
    }
    fn endpoint_set_enabled(&mut self, _: EndpointAddress, _: bool) {}
    fn endpoint_set_stalled(&mut self, _: EndpointAddress, _: bool) {}
    fn endpoint_is_stalled(&mut self, _: EndpointAddress) -> bool {
        false
    }
    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

impl ControlPipe for Pipe {
    fn max_packet_size(&self) -> usize {
        64
    }
    async fn setup(&mut self) -> [u8; 8] {
        let next = self.0.borrow_mut().setups.pop_front();
        match next {
            Some(setup) => setup,
            None => core::future::pending().await,
        }
    }
    async fn data_out(&mut self, _: &mut [u8], _: bool, _: bool) -> Result<usize, EndpointError> {
        unreachable!("only IN requests are sent")
    }
    async fn data_in(&mut self, data: &[u8], _: bool, last: bool) -> Result<(), EndpointError> {
        let mut ep0 = self.0.borrow_mut();
        ep0.stage.extend_from_slice(data);
        if last {
            let stage = core::mem::take(&mut ep0.stage);
            ep0.replies.push(stage);
        }
        Ok(())
    }
    async fn accept(&mut self) {}
    async fn reject(&mut self) {
        unreachable!("a GET_DESCRIPTOR was rejected")
    }
    async fn accept_set_address(&mut self, _: u8) {}
}

/// GET_DESCRIPTOR for descriptor `kind`, index 0.
fn get_descriptor(kind: u8, len: u16) -> [u8; 8] {
    let [lo, hi] = len.to_le_bytes();
    [0x80, 0x06, 0x00, kind, 0x00, 0x00, lo, hi]
}

/// The only test that builds a device: the class handler is a process-wide `StaticCell`.
#[test]
fn the_dap_only_device_is_one_interface_without_an_iad() {
    let ep0 = Rc::new(RefCell::new(Ep0::default()));
    ep0.borrow_mut()
        .setups
        .extend([get_descriptor(1, 18), get_descriptor(2, 255)]);
    let cfg = DapConfig::daplink("TEST", DapQueue::<1, 64>::endpoint_size::<64>());
    let (mut usb, _class) = usb_device_dap_only(Host(ep0.clone()), &cfg);
    let mut run = pin!(usb.run_until_suspend());
    let mut cx = Context::from_waker(Waker::noop());
    for _ in 0..16 {
        if ep0.borrow().replies.len() == 2 {
            break;
        }
        assert!(run.as_mut().poll(&mut cx).is_pending());
    }
    let replies = ep0.borrow().replies.clone();
    let [device, config] = &replies[..] else {
        panic!("{replies:x?}");
    };
    assert_eq!(device[4..7], [0, 0, 0], "bDeviceClass/SubClass/Protocol");
    assert_eq!(config[4], 1, "bNumInterfaces");
    let mut kinds = Vec::new();
    let mut rest = &config[..];
    while let [len, kind, ..] = *rest {
        kinds.push(kind);
        rest = &rest[usize::from(len)..];
    }
    assert_eq!(
        kinds,
        [0x02, 0x04, 0x05, 0x05],
        "no IAD before the interface"
    );
}
