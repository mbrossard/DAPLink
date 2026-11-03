use super::*;
use core::sync::atomic::{AtomicU8, Ordering};
use daplink_core::DapQueue;
use std::sync::{Arc, Mutex};
use std::vec::Vec;
use usb_device::bus::PollResult;
use usb_device::class_prelude::*;
use usb_device::{Result, UsbDirection, UsbError};

const TEST_CFG: DapConfig = DapConfig::daplink("test", DapQueue::<1, 64>::endpoint_size::<64>());

#[derive(Clone, Copy)]
#[repr(u8)]
enum Event {
    Reset = 1,
    Suspend,
    Resume,
    Configure,
    Deconfigure,
}

struct TestBus {
    event: Arc<AtomicU8>,
    stalls: Arc<Mutex<Vec<(u8, bool)>>>,
    configuration: AtomicU8,
    next: usize,
}

impl UsbBus for TestBus {
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
    fn write(&self, _endpoint: EndpointAddress, data: &[u8]) -> Result<usize> {
        Ok(data.len())
    }
    fn read(&self, endpoint: EndpointAddress, data: &mut [u8]) -> Result<usize> {
        if endpoint.index() != 0 || data.len() < 8 {
            return Err(UsbError::WouldBlock);
        }
        data[..8].copy_from_slice(&[
            0,
            9,
            self.configuration.load(Ordering::Relaxed),
            0,
            0,
            0,
            0,
            0,
        ]);
        Ok(8)
    }
    fn set_stalled(&self, endpoint: EndpointAddress, stalled: bool) {
        if endpoint.index() != 0 {
            self.stalls.lock().unwrap().push((endpoint.into(), stalled));
        }
    }
    fn is_stalled(&self, _endpoint: EndpointAddress) -> bool {
        false
    }
    fn suspend(&self) {}
    fn resume(&self) {}
    fn poll(&self) -> PollResult {
        match self.event.swap(0, Ordering::Relaxed) {
            1 => PollResult::Reset,
            2 => PollResult::Suspend,
            3 => PollResult::Resume,
            configuration @ (4 | 5) => {
                self.configuration
                    .store(u8::from(configuration == 4), Ordering::Relaxed);
                PollResult::Data {
                    ep_out: 0,
                    ep_in_complete: 0,
                    ep_setup: 1,
                }
            }
            _ => PollResult::None,
        }
    }
}

macro_rules! session_tests {
    ($name:ident, $device:ident, $cdc:expr, $msc:expr) => {
        #[test]
        fn $name() {
            let event = Arc::new(AtomicU8::new(0));
            let stalls = Arc::new(Mutex::new(Vec::new()));
            let alloc = UsbBusAllocator::new(TestBus {
                event: event.clone(),
                stalls: stalls.clone(),
                configuration: AtomicU8::new(0),
                next: 1,
            });
            let mut usb = $device::<TestBus>::new(&alloc, &TEST_CFG);
            macro_rules! poll {
                ($kind:ident) => {{
                    event.store(Event::$kind as u8, Ordering::Relaxed);
                    usb.poll();
                }};
            }
            assert!(!usb.session_ended());
            poll!(Reset);
            assert!(usb.session_ended());
            assert!(!usb.session_ended());
            poll!(Configure);
            assert!(usb.configured());
            assert!(usb.session_ended());
            poll!(Suspend);
            assert!(!usb.session_ended());
            poll!(Resume);
            assert!(usb.configured());
            assert!(!usb.session_ended());

            let mut queue = DapQueue::<2, 64>::new();
            queue.post_response(&[0xaa]).unwrap();
            let generation = queue.generation();
            poll!(Suspend);
            assert!(!usb.session_ended());
            poll!(Reset);
            poll!(Reset);
            poll!(Configure);
            assert!(usb.configured());
            if usb.session_ended() {
                queue.reset();
            } else {
                panic!("lost reset");
            }
            assert!(queue.pending_response().is_none());
            assert_ne!(queue.generation(), generation);
            queue.post_response(&[0xbb]).unwrap();
            assert!(!queue.discard_response_for(generation));
            assert_eq!(queue.pending_response(), Some(&[0xbb][..]));
            assert!(!usb.session_ended());

            poll!(Suspend);
            assert!(!usb.session_ended());
            poll!(Deconfigure);
            assert!(usb.session_ended());
            assert!(!usb.session_ended());
            assert!(!usb.write_response(&[0xaa]));
            stalls.lock().unwrap().clear();
            poll!(Configure);
            let mut expected = std::vec![(1, true), (0x82, true), (1, false), (0x82, false)];
            if $cdc {
                expected.extend([
                    (4, true),
                    (0x85, true),
                    (0x83, true),
                    (4, false),
                    (0x85, false),
                    (0x83, false),
                ]);
            }
            let (msc_in, msc_out): (u8, u8) = $msc;
            if msc_in != 0 {
                expected.extend([
                    (msc_in, true),
                    (msc_out, true),
                    (msc_in, false),
                    (msc_out, false),
                ]);
            }
            assert_eq!(*stalls.lock().unwrap(), expected);
            assert!(usb.session_ended());
            poll!(Deconfigure);
            poll!(Configure);
            assert!(
                usb.session_ended(),
                "a fast reconfiguration must remain observable"
            );
            assert!(!usb.session_ended());
        }
    };
}

session_tests!(
    vcom_reset_survives_suspend_and_late_observation,
    DapUsb,
    true,
    (0, 0)
);
session_tests!(
    msc_vcom_reset_survives_suspend_and_late_observation,
    DapUsbMsc,
    true,
    (0x86, 7)
);
session_tests!(
    msc_dap_reset_survives_suspend_and_late_observation,
    DapUsbNoVcomMsc,
    false,
    (0x83, 4)
);
session_tests!(
    no_vcom_reset_survives_suspend_and_late_observation,
    DapUsbNoVcom,
    false,
    (0, 0)
);

#[derive(Default)]
struct Enumeration {
    setup: Option<[u8; 8]>,
    reply: Vec<u8>,
    in_complete: bool,
    stalled: bool,
}

struct DescriptorBus {
    enumeration: Arc<Mutex<Enumeration>>,
    next: usize,
}

impl UsbBus for DescriptorBus {
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
        assert_eq!(endpoint.index(), 0);
        let mut state = self.enumeration.lock().unwrap();
        state.reply.extend_from_slice(data);
        state.in_complete = true;
        Ok(data.len())
    }
    fn read(&self, endpoint: EndpointAddress, data: &mut [u8]) -> Result<usize> {
        if endpoint.index() != 0 {
            return Err(UsbError::WouldBlock);
        }
        let Some(setup) = self.enumeration.lock().unwrap().setup.take() else {
            return Err(UsbError::WouldBlock);
        };
        data[..8].copy_from_slice(&setup);
        Ok(8)
    }
    fn set_stalled(&self, _endpoint: EndpointAddress, stalled: bool) {
        self.enumeration.lock().unwrap().stalled = stalled;
    }
    fn is_stalled(&self, _endpoint: EndpointAddress) -> bool {
        self.enumeration.lock().unwrap().stalled
    }
    fn suspend(&self) {}
    fn resume(&self) {}
    fn poll(&self) -> PollResult {
        let mut state = self.enumeration.lock().unwrap();
        if state.setup.is_some() {
            PollResult::Data {
                ep_out: 0,
                ep_in_complete: 0,
                ep_setup: 1,
            }
        } else if core::mem::take(&mut state.in_complete) {
            PollResult::Data {
                ep_out: 0,
                ep_in_complete: 1,
                ep_setup: 0,
            }
        } else {
            PollResult::None
        }
    }
}

fn get_descriptor(
    state: &Mutex<Enumeration>,
    mut poll: impl FnMut(),
    kind: u8,
    index: u8,
) -> Vec<u8> {
    *state.lock().unwrap() = Enumeration {
        setup: Some([
            0x80,
            6,
            index,
            kind,
            if kind == 3 { 0x09 } else { 0 },
            if kind == 3 { 0x04 } else { 0 },
            0xff,
            0,
        ]),
        ..Enumeration::default()
    };
    for _ in 0..64 {
        poll();
        let state = state.lock().unwrap();
        assert!(!state.stalled, "GET_DESCRIPTOR stalled");
        if state.setup.is_none() && !state.in_complete {
            assert!(!state.reply.is_empty(), "GET_DESCRIPTOR returned no data");
            return state.reply.clone();
        }
    }
    panic!("GET_DESCRIPTOR did not complete");
}

fn check_configuration(bytes: &[u8], vcom: bool, msc: bool) -> u8 {
    assert_eq!(&bytes[..2], &[9, 2]);
    assert_eq!(
        u16::from_le_bytes([bytes[2], bytes[3]]) as usize,
        bytes.len()
    );
    assert_eq!(bytes[4], 1 + 2 * u8::from(vcom) + u8::from(msc));
    let mut interfaces = Vec::new();
    let mut endpoints = Vec::new();
    let mut iads = Vec::new();
    let mut remaining = &bytes[9..];
    while !remaining.is_empty() {
        let len = remaining[0] as usize;
        assert!(len >= 2 && len <= remaining.len());
        let descriptor = &remaining[..len];
        match descriptor[1] {
            4 => interfaces.push(descriptor),
            5 => endpoints.push((interfaces.last().unwrap()[2], descriptor)),
            11 => iads.push(descriptor),
            _ => {}
        }
        remaining = &remaining[len..];
    }
    let mut expected = std::vec![(0, 0xff, 0, 0, 2)];
    if vcom {
        expected.extend([(1, 2, 2, 0, 1), (2, 0x0a, 0, 0, 2)]);
    }
    if msc {
        expected.push((if vcom { 3 } else { 1 }, 8, 6, 0x50, 2));
    }
    let actual: Vec<_> = interfaces
        .iter()
        .map(|d| (d[2], d[5], d[6], d[7], d[4]))
        .collect();
    assert_eq!(actual, expected);
    assert_eq!(iads.len(), usize::from(vcom));
    if vcom {
        assert_eq!(&iads[0][2..7], &[1, 2, 2, 2, 0]);
    }
    let mut addresses = Vec::new();
    for (iface, _, _, _, count) in expected {
        let eps: Vec<_> = endpoints
            .iter()
            .filter(|(i, _)| *i == iface)
            .map(|(_, d)| *d)
            .collect();
        assert_eq!(eps.len(), count as usize);
        for ep in &eps {
            assert!(!addresses.contains(&ep[2]), "duplicate endpoint address");
            addresses.push(ep[2]);
        }
        if vcom && iface == 1 {
            assert_eq!(eps[0][3] & 3, 3);
            assert_ne!(eps[0][2] & 0x80, 0);
        } else {
            assert_eq!(eps[0][2] & 0x80 != 0, iface != 0);
            assert_eq!(eps[1][2] & 0x80 != 0, iface == 0);
            for ep in eps {
                assert_eq!(ep[3] & 3, 2);
                assert_eq!(u16::from_le_bytes([ep[4], ep[5]]), 64);
            }
        }
    }
    assert_eq!(endpoints[0].1[2], 1);
    assert_eq!(endpoints[1].1[2], 0x82);
    assert_ne!(interfaces[0][8], 0);
    interfaces[0][8]
}

macro_rules! descriptor_test {
    ($name:ident, $device:ident, $vcom:expr, $msc:expr) => {
        #[test]
        fn $name() {
            let state = Arc::new(Mutex::new(Enumeration::default()));
            let alloc = UsbBusAllocator::new(DescriptorBus {
                enumeration: state.clone(),
                next: 1,
            });
            let mut usb = $device::<DescriptorBus>::new(&alloc, &TEST_CFG);
            let device = get_descriptor(
                &state,
                || {
                    usb.poll();
                },
                1,
                0,
            );
            assert_eq!(&device[..2], &[18, 1]);
            assert_eq!(
                &device[4..7],
                if $vcom { &[0xef, 2, 1] } else { &[0, 0, 0] }
            );
            let configuration = get_descriptor(
                &state,
                || {
                    usb.poll();
                },
                2,
                0,
            );
            let string_index = check_configuration(&configuration, $vcom, $msc);
            let string = get_descriptor(
                &state,
                || {
                    usb.poll();
                },
                3,
                string_index,
            );
            assert_eq!(string[0] as usize, string.len());
            assert_eq!(string[1], 3);
            let units: Vec<_> = string[2..]
                .chunks_exact(2)
                .map(|b| u16::from_le_bytes([b[0], b[1]]))
                .collect();
            assert_eq!(
                std::string::String::from_utf16(&units).unwrap(),
                "CMSIS-DAP v2 Interface"
            );
        }
    };
}

descriptor_test!(dap_descriptors, DapUsbNoVcom, false, false);
descriptor_test!(dap_cdc_descriptors, DapUsb, true, false);
descriptor_test!(dap_msc_descriptors, DapUsbNoVcomMsc, false, true);
descriptor_test!(dap_cdc_msc_descriptors, DapUsbMsc, true, true);

/// Every bus call a request reaches, in order, plus EP0's last reply and stall.
#[derive(Default)]
struct HaltLog {
    setup: Option<[u8; 8]>,
    calls: Vec<std::string::String>,
    reply: Option<Vec<u8>>,
    ep0_stalled: bool,
    in_complete: bool,
}

static HALT_LOG: Mutex<Option<HaltLog>> = Mutex::new(None);
/// The halt tests share [`HALT_LOG`], so they run one at a time.
static HALT_TESTS: Mutex<()> = Mutex::new(());

fn log(entry: std::string::String) {
    HALT_LOG.lock().unwrap().as_mut().unwrap().calls.push(entry);
}

struct HaltBus {
    next: usize,
}

impl UsbBus for HaltBus {
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
        if endpoint.index() != 0 {
            log(std::format!("bus.write({:#04x})", u8::from(endpoint)));
            return Ok(data.len());
        }
        let mut guard = HALT_LOG.lock().unwrap();
        let state = guard.as_mut().unwrap();
        state
            .reply
            .get_or_insert_with(Vec::new)
            .extend_from_slice(data);
        state.in_complete = true;
        Ok(data.len())
    }
    fn read(&self, endpoint: EndpointAddress, data: &mut [u8]) -> Result<usize> {
        if endpoint.index() != 0 {
            return Err(UsbError::WouldBlock);
        }
        let setup = HALT_LOG.lock().unwrap().as_mut().unwrap().setup.take();
        let Some(setup) = setup else {
            return Err(UsbError::WouldBlock);
        };
        data[..8].copy_from_slice(&setup);
        Ok(8)
    }
    fn set_stalled(&self, endpoint: EndpointAddress, stalled: bool) {
        if endpoint.index() == 0 {
            HALT_LOG.lock().unwrap().as_mut().unwrap().ep0_stalled = stalled;
        } else {
            log(std::format!(
                "bus.set_stalled({:#04x}, {stalled})",
                u8::from(endpoint)
            ));
        }
    }
    fn is_stalled(&self, endpoint: EndpointAddress) -> bool {
        if endpoint.index() != 0 {
            log(std::format!("bus.is_stalled({:#04x})", u8::from(endpoint)));
        }
        false
    }
    fn suspend(&self) {}
    fn resume(&self) {}
    fn poll(&self) -> PollResult {
        let mut guard = HALT_LOG.lock().unwrap();
        let state = guard.as_mut().unwrap();
        if state.setup.is_some() {
            PollResult::Data {
                ep_out: 0,
                ep_in_complete: 0,
                ep_setup: 1,
            }
        } else if core::mem::take(&mut state.in_complete) {
            PollResult::Data {
                ep_out: 0,
                ep_in_complete: 1,
                ep_setup: 0,
            }
        } else {
            PollResult::None
        }
    }
}

/// Logs each repair, for a controller with registers 0..6 as on the LPC55 USB1 block.
struct SixRegisters;

impl EndpointRepair for SixRegisters {
    fn has(&self, ep: EndpointAddress) -> bool {
        ep.index() < 6
    }
    fn clear_halt(&self, ep: EndpointAddress) {
        log(std::format!("repair.clear_halt({:#04x})", u8::from(ep)));
    }
}

static SIX: SixRegisters = SixRegisters;

/// One control request to the polled device: the bus calls, EP0's stall, and any reply.
fn request(
    mut poll: impl FnMut(),
    setup: [u8; 8],
) -> (Vec<std::string::String>, bool, Option<Vec<u8>>) {
    *HALT_LOG.lock().unwrap() = Some(HaltLog {
        setup: Some(setup),
        ..HaltLog::default()
    });
    for _ in 0..8 {
        poll();
    }
    let state = HALT_LOG.lock().unwrap().take().unwrap();
    (state.calls, state.ep0_stalled, state.reply)
}

const fn feature(set: bool, ep: u8) -> [u8; 8] {
    [0x02, if set { 3 } else { 1 }, 0, 0, ep, 0, 0, 0]
}

const fn status(ep: u8) -> [u8; 8] {
    [0x82, 0, 0, 0, ep, 0, 2, 0]
}

const SET_CONFIGURATION: [u8; 8] = [0x00, 9, 1, 0, 0, 0, 0, 0];

/// usb-device hands any wIndex to the bus; stm32-usbd and the lpc55 fork index arrays with it.
#[test]
fn halt_requests_reach_the_bus_unchecked_without_a_repair() {
    let _serial = HALT_TESTS.lock().unwrap_or_else(|e| e.into_inner());
    let alloc = UsbBusAllocator::new(HaltBus { next: 1 });
    let mut usb = DapUsbNoVcom::<HaltBus>::new(&alloc, &TEST_CFG);
    let mut poll = || {
        usb.poll();
    };
    let (calls, stalled, _) = request(&mut poll, feature(false, 0x8f));
    assert_eq!(calls, ["bus.set_stalled(0x8f, false)"]);
    assert!(!stalled);
    let (calls, _, reply) = request(&mut poll, status(0x8f));
    assert_eq!(calls, ["bus.is_stalled(0x8f)"]);
    assert_eq!(reply.as_deref(), Some(&[0u8, 0][..]));
}

#[test]
fn a_repair_resets_the_toggle_first_and_refuses_unknown_endpoints() {
    let _serial = HALT_TESTS.lock().unwrap_or_else(|e| e.into_inner());
    let alloc = UsbBusAllocator::new(HaltBus { next: 1 });
    let mut usb = DapUsbNoVcom::<HaltBus>::new(&alloc, &TEST_CFG).with_endpoint_repair(&SIX);
    let mut poll = || {
        usb.poll();
    };
    let (calls, stalled, _) = request(&mut poll, feature(false, 0x82));
    assert_eq!(
        calls,
        ["repair.clear_halt(0x82)", "bus.set_stalled(0x82, false)"]
    );
    assert!(!stalled);
    let (calls, stalled, _) = request(&mut poll, feature(true, 0x01));
    assert_eq!(calls, ["bus.set_stalled(0x01, true)"]);
    assert!(!stalled);
    for setup in [
        feature(false, 0x8f),
        feature(true, 0x06),
        status(0x8f),
        feature(false, 0x12),
    ] {
        let (calls, stalled, _) = request(&mut poll, setup);
        assert!(calls.is_empty(), "{setup:02x?} reached the bus: {calls:?}");
        assert!(stalled, "{setup:02x?} was not refused");
    }
    let (calls, _, _) = request(&mut poll, SET_CONFIGURATION);
    assert!(calls.contains(&"repair.clear_halt(0x01)".into()));
    assert!(calls.contains(&"repair.clear_halt(0x82)".into()));
}

/// The bus calls `f` makes outside a control request.
fn bus_calls(f: impl FnOnce()) -> Vec<std::string::String> {
    *HALT_LOG.lock().unwrap() = Some(HaltLog::default());
    f();
    HALT_LOG.lock().unwrap().take().unwrap().calls
}

#[test]
fn a_halted_dap_in_endpoint_takes_no_response_until_cleared() {
    let _serial = HALT_TESTS.lock().unwrap_or_else(|e| e.into_inner());
    let alloc = UsbBusAllocator::new(HaltBus { next: 1 });
    let mut usb = DapUsbNoVcom::<HaltBus>::new(&alloc, &TEST_CFG);
    request(|| _ = usb.poll(), SET_CONFIGURATION);
    request(|| _ = usb.poll(), feature(true, 0x82));
    assert!(bus_calls(|| assert!(!usb.write_response(&[0x00]))).is_empty());
    request(|| _ = usb.poll(), feature(false, 0x82));
    let calls = bus_calls(|| assert!(usb.write_response(&[0x00])));
    assert_eq!(calls, ["bus.write(0x82)"]);
}

#[test]
fn a_halted_cdc_in_endpoint_holds_staged_bytes_until_cleared() {
    let _serial = HALT_TESTS.lock().unwrap_or_else(|e| e.into_inner());
    let alloc = UsbBusAllocator::new(HaltBus { next: 1 });
    let mut usb = DapUsb::<HaltBus>::new(&alloc, &TEST_CFG);
    request(|| _ = usb.poll(), SET_CONFIGURATION);
    request(|| _ = usb.poll(), feature(true, 0x85));
    assert!(bus_calls(|| assert_eq!(usb.vcom_write(b"abc"), 3)).is_empty());
    assert_eq!(usb.vcom_staged(), 3);
    request(|| _ = usb.poll(), feature(false, 0x85));
    assert_eq!(bus_calls(|| usb.vcom_flush()), ["bus.write(0x85)"]);
}

#[test]
fn a_cdc_halt_names_one_endpoint_and_ends_with_its_session() {
    let _serial = HALT_TESTS.lock().unwrap_or_else(|e| e.into_inner());
    let alloc = UsbBusAllocator::new(HaltBus { next: 1 });
    let mut usb = DapUsb::<HaltBus>::new(&alloc, &TEST_CFG);
    let wrote = |calls: Vec<std::string::String>| calls.contains(&"bus.write(0x85)".into());
    request(|| _ = usb.poll(), SET_CONFIGURATION);
    request(|| _ = usb.poll(), feature(true, 0x82));
    assert!(wrote(bus_calls(|| {
        usb.vcom_write(b"abc");
        usb.vcom_flush();
    })));
    request(|| _ = usb.poll(), feature(false, 0x82));
    request(|| _ = usb.poll(), feature(true, 0x85));
    UsbClass::<HaltBus>::reset(&mut usb.serial);
    request(|| _ = usb.poll(), SET_CONFIGURATION);
    assert!(wrote(bus_calls(|| {
        usb.vcom_write(b"abc");
        usb.vcom_flush();
    })));
}

#[test]
fn a_repair_leaves_ep0_alone() {
    let _serial = HALT_TESTS.lock().unwrap_or_else(|e| e.into_inner());
    let alloc = UsbBusAllocator::new(HaltBus { next: 1 });
    let mut usb = DapUsbNoVcom::<HaltBus>::new(&alloc, &TEST_CFG).with_endpoint_repair(&SIX);
    for ep in [0x00, 0x80] {
        let (calls, _, _) = request(|| _ = usb.poll(), feature(false, ep));
        assert!(
            !calls.iter().any(|c| c.starts_with("repair.clear_halt")),
            "{ep:#x}"
        );
    }
}
