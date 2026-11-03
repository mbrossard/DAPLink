//! Buffered CDC-ACM serial class with a firmware-chosen data-endpoint packet size.

use crate::dap_class::halt_request;
use daplink_rs::daplink_core::DEFAULT_VCOM_BAUD;
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, Request, RequestType};
use usb_device::descriptor::lang_id::LangID;
use usb_device::device::DEFAULT_ALTERNATE_SETTING;
use usb_device::Result;

/// Force a short packet after this many consecutive full packets, so the host sees data.
const SHORT_PACKET_INTERVAL: usize = 10;

/// CDC-ACM protocol constants (USB CDC 1.10, Abstract Control Model subclass).
const USB_CLASS_CDC: u8 = 0x02;
const USB_CLASS_CDC_DATA: u8 = 0x0a;
const CDC_SUBCLASS_ACM: u8 = 0x02;
const CDC_PROTOCOL_NONE: u8 = 0x00;

const CS_INTERFACE: u8 = 0x24;
const CDC_TYPE_HEADER: u8 = 0x00;
const CDC_TYPE_CALL_MANAGEMENT: u8 = 0x01;
const CDC_TYPE_ACM: u8 = 0x02;
const CDC_TYPE_UNION: u8 = 0x06;

const REQ_SEND_ENCAPSULATED_COMMAND: u8 = 0x00;
const REQ_SET_LINE_CODING: u8 = 0x20;
const REQ_GET_LINE_CODING: u8 = 0x21;
const REQ_SET_CONTROL_LINE_STATE: u8 = 0x22;

/// Type of the last written packet, so a full packet is always terminated by a short one.
enum WriteState {
    Idle,
    Short,
    /// A full packet is in flight; the count forces a short packet every [`SHORT_PACKET_INTERVAL`].
    Full(usize),
}

/// Byte buffer with `rpos <= wpos <= CAP`; read data is memmoved out only when a write needs room.
struct CdcBuffer<const CAP: usize> {
    store: [u8; CAP],
    rpos: usize,
    wpos: usize,
}

impl<const CAP: usize> CdcBuffer<CAP> {
    const fn new() -> Self {
        Self {
            store: [0u8; CAP],
            rpos: 0,
            wpos: 0,
        }
    }

    fn clear(&mut self) {
        self.rpos = 0;
        self.wpos = 0;
    }

    fn available_read(&self) -> usize {
        self.wpos - self.rpos
    }

    /// Bytes available for writing, counting space reclaimable by discarding read data.
    fn available_write(&self) -> usize {
        self.available_write_without_discard() + self.rpos
    }

    /// Bytes available for writing at `wpos` without discarding read data.
    fn available_write_without_discard(&self) -> usize {
        CAP - self.wpos
    }

    fn write(&mut self, data: &[u8]) -> usize {
        if data.len() > self.available_write_without_discard() && self.rpos > 0 {
            self.discard_already_read_data();
        }

        let count = core::cmp::min(self.available_write_without_discard(), data.len());
        if count == 0 {
            return 0;
        }

        self.store[self.wpos..self.wpos + count].copy_from_slice(&data[..count]);
        self.wpos += count;
        count
    }

    /// Reserve `max_count` bytes and let `f` fill them, committing what it reports.
    fn write_all<E>(
        &mut self,
        max_count: usize,
        f: impl FnOnce(&mut [u8]) -> core::result::Result<usize, E>,
    ) -> core::result::Result<usize, E> {
        if max_count > self.available_write_without_discard() {
            if max_count > self.available_write() {
                return Ok(0);
            }
            self.discard_already_read_data();
        }

        f(&mut self.store[self.wpos..self.wpos + max_count]).inspect(|&count| {
            self.wpos += count;
        })
    }

    /// Hand up to `max_count` readable bytes to `f`, discarding what it reports consumed.
    fn read<E>(
        &mut self,
        max_count: usize,
        f: impl FnOnce(&[u8]) -> core::result::Result<usize, E>,
    ) -> core::result::Result<usize, E> {
        let count = core::cmp::min(max_count, self.available_read());

        f(&self.store[self.rpos..self.rpos + count]).inspect(|&count| {
            self.rpos += count;
        })
    }

    /// Memmove unread bytes to the front, reclaiming the space read data occupied.
    fn discard_already_read_data(&mut self) {
        if self.rpos != 0 {
            self.store.copy_within(self.rpos..self.wpos, 0);
            self.wpos -= self.rpos;
            self.rpos = 0;
        }
    }
}

/// The CDC line-coding record; the format is pinned at 8N1, only the baud varies.
#[derive(Clone, Copy)]
struct LineCoding {
    data_rate: u32,
    stop_bits: u8,
    parity_type: u8,
    data_bits: u8,
}

impl LineCoding {
    /// The reset default: the UART's idle rate, so enumeration alone never retunes the wire.
    const DEFAULT: Self = Self {
        data_rate: DEFAULT_VCOM_BAUD,
        stop_bits: 0,
        parity_type: 0,
        data_bits: 8,
    };
}

/// Packet-level CDC-ACM function: the comm and data interfaces, plus the class requests.
struct CdcAcm<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    line_coding: LineCoding,
    dtr: bool,
    /// Set when DTR falls, cleared once the stream layer discarded what the closing host left.
    closed: bool,
}

impl<'a, B: UsbBus> CdcAcm<'a, B> {
    /// Allocate the interfaces and endpoints; at high speed `bInterval` 11 (128 ms) replaces 255.
    fn new(alloc: &'a UsbBusAllocator<B>, max_packet_size: u16) -> Self {
        let notify_interval = if max_packet_size > 64 { 11 } else { 255 };
        Self {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(8, notify_interval),
            data_if: alloc.interface(),
            read_ep: alloc.bulk(max_packet_size),
            write_ep: alloc.bulk(max_packet_size),
            line_coding: LineCoding::DEFAULT,
            dtr: false,
            closed: false,
        }
    }

    /// Whether the host has closed the port since this was last asked.
    fn take_closed(&mut self) -> bool {
        core::mem::replace(&mut self.closed, false)
    }

    /// Track DTR; a falling edge is the host closing the port.
    fn set_control_line_state(&mut self, dtr: bool) {
        self.closed |= self.dtr && !dtr;
        self.dtr = dtr;
    }

    fn max_packet_size(&self) -> u16 {
        self.read_ep.max_packet_size()
    }

    fn write_packet(&mut self, data: &[u8]) -> Result<usize> {
        self.write_ep.write(data)
    }

    fn read_packet(&mut self, data: &mut [u8]) -> Result<usize> {
        self.read_ep.read(data)
    }
}

impl<B: UsbBus> UsbClass<B> for CdcAcm<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.iad(
            self.comm_if,
            2,
            USB_CLASS_CDC,
            CDC_SUBCLASS_ACM,
            CDC_PROTOCOL_NONE,
            None,
        )?;

        writer.interface_alt(
            self.comm_if,
            DEFAULT_ALTERNATE_SETTING,
            USB_CLASS_CDC,
            CDC_SUBCLASS_ACM,
            CDC_PROTOCOL_NONE,
            None,
        )?;

        // Header (bcdCDC 1.10), ACM, union and call-management, byte-identical to `usbd-serial`.
        writer.write(CS_INTERFACE, &[CDC_TYPE_HEADER, 0x10, 0x01])?;
        writer.write(CS_INTERFACE, &[CDC_TYPE_ACM, 0x00])?;
        writer.write(
            CS_INTERFACE,
            &[CDC_TYPE_UNION, self.comm_if.into(), self.data_if.into()],
        )?;
        writer.write(
            CS_INTERFACE,
            &[CDC_TYPE_CALL_MANAGEMENT, 0x00, self.data_if.into()],
        )?;

        writer.endpoint(&self.comm_ep)?;

        writer.interface_alt(
            self.data_if,
            DEFAULT_ALTERNATE_SETTING,
            USB_CLASS_CDC_DATA,
            0x00,
            0x00,
            None,
        )?;

        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;

        Ok(())
    }

    fn reset(&mut self) {
        self.line_coding = LineCoding::DEFAULT;
        // A reset ends an open session as a close does.
        self.set_control_line_state(false);
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.comm_if) as u16)
        {
            return;
        }

        match req.request {
            REQ_GET_LINE_CODING if req.length == 7 => {
                let lc = self.line_coding;
                xfer.accept(|data| {
                    data[0..4].copy_from_slice(&lc.data_rate.to_le_bytes());
                    data[4] = lc.stop_bits;
                    data[5] = lc.parity_type;
                    data[6] = lc.data_bits;
                    Ok(7)
                })
                .ok();
            }
            _ => {
                xfer.reject().ok();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.comm_if) as u16)
        {
            return;
        }

        match req.request {
            // Not supported, but accepted for standards compatibility.
            REQ_SEND_ENCAPSULATED_COMMAND => {
                xfer.accept().ok();
            }
            REQ_SET_LINE_CODING if xfer.data().len() >= 7 => {
                let data = xfer.data();
                // The rate is followed whatever the framing; only 8N1 runs, so the rest is refused.
                self.line_coding.data_rate =
                    u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                if data[4..7] == [0, 0, 8] {
                    xfer.accept().ok();
                } else {
                    xfer.reject().ok();
                }
            }
            // A falling DTR is a close: stage and caller's backlog go, an armed packet stays.
            REQ_SET_CONTROL_LINE_STATE => {
                self.set_control_line_state(req.value & 1 != 0);
                xfer.accept().ok();
            }
            _ => {
                xfer.reject().ok();
            }
        }
    }
}

/// Stream-buffered CDC-ACM port: `PKT` sizes the data endpoints, `BUF` the stream buffers.
pub struct BufferedCdc<'a, B: UsbBus, const PKT: usize = 64, const BUF: usize = 128> {
    inner: CdcAcm<'a, B>,
    read_buf: CdcBuffer<BUF>,
    write_buf: CdcBuffer<BUF>,
    write_state: WriteState,
    /// Staged bytes a port close or a bus reset threw away, until the caller takes them.
    discarded: usize,
    /// Set when a close or bus reset ended a session, until the caller takes it.
    ended: bool,
    /// Set by any host-side event on the port since the caller last asked.
    touched: bool,
    configured: bool,
    /// The host halted the IN endpoint; staged bytes wait for the clear.
    in_halted: bool,
}

impl<'a, B: UsbBus, const PKT: usize, const BUF: usize> BufferedCdc<'a, B, PKT, BUF> {
    /// Allocate the class from `alloc`; a whole packet must fit the stream buffer.
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        const { assert!(BUF >= PKT, "CDC stream buffer smaller than one packet") };
        Self {
            inner: CdcAcm::new(alloc, PKT as u16),
            read_buf: CdcBuffer::new(),
            write_buf: CdcBuffer::new(),
            write_state: WriteState::Idle,
            discarded: 0,
            ended: false,
            touched: true,
            configured: false,
            in_halted: false,
        }
    }

    /// Whether the host touched the port (packet, completion, control) since the last call.
    pub fn take_activity(&mut self) -> bool {
        core::mem::take(&mut self.touched)
    }

    /// The baud the host last set via CDC line coding.
    pub fn line_coding_baud(&self) -> u32 {
        self.inner.line_coding.data_rate
    }

    /// Bytes [`write`](Self::write) is guaranteed to take whole.
    pub fn write_capacity(&self) -> usize {
        if self.configured {
            self.write_buf.available_write()
        } else {
            0
        }
    }

    /// Buffer bytes toward the host and start sending; `WouldBlock` when nothing fit.
    pub fn write(&mut self, data: &[u8]) -> Result<usize> {
        let count = self.write_buf.write(data);

        // `count` is already committed: failing the write here would have the caller re-send it.
        match self.flush() {
            Ok(_) | Err(UsbError::WouldBlock) => {}
            Err(err) if count == 0 => return Err(err),
            Err(_) => {}
        };

        if count == 0 {
            Err(UsbError::WouldBlock)
        } else {
            Ok(count)
        }
    }

    /// Buffer toward the host without offering the endpoint a packet, unlike [`write`](Self::write).
    pub fn write_buffered(&mut self, data: &[u8]) -> usize {
        self.write_buf.write(data)
    }

    /// Bytes buffered toward the host that the endpoint has not been offered yet.
    pub fn staged(&self) -> usize {
        self.write_buf.available_read()
    }

    /// Whether a packet sits at the IN endpoint awaiting completion.
    pub fn in_flight(&self) -> bool {
        !matches!(self.write_state, WriteState::Idle)
    }

    /// Take the staged bytes a close or bus reset discarded, for the caller to deduct.
    pub fn take_discarded(&mut self) -> usize {
        self.discard_on_close();
        core::mem::take(&mut self.discarded)
    }

    /// Whether a close or bus reset ended a session since asked; the caller drops its backlog.
    pub fn take_close_edge(&mut self) -> bool {
        self.discard_on_close();
        core::mem::take(&mut self.ended)
    }

    /// Clear software buffers on DTR fall, charging staged TX bytes; an armed IN packet may remain.
    fn discard_on_close(&mut self) {
        if self.inner.take_closed() {
            self.read_buf.clear();
            self.discarded += self.write_buf.available_read();
            self.write_buf.clear();
            self.write_state = WriteState::Idle;
            self.ended = true;
        }
    }

    /// Pull a pending OUT packet into the read buffer.
    fn poll(&mut self) -> Result<()> {
        if !self.configured {
            return Err(UsbError::WouldBlock);
        }
        self.discard_on_close();
        let Self {
            inner, read_buf, ..
        } = self;

        read_buf.write_all(inner.max_packet_size() as usize, |buf_data| {
            match inner.read_packet(buf_data) {
                Ok(c) => Ok(c),
                Err(UsbError::WouldBlock) => Ok(0),
                Err(err) => Err(err),
            }
        })?;

        Ok(())
    }

    /// Read buffered host bytes into `data`; `WouldBlock` when none are available.
    pub fn read(&mut self, data: &mut [u8]) -> Result<usize> {
        self.poll()?;

        if self.read_buf.available_read() == 0 {
            return Err(UsbError::WouldBlock);
        }

        self.read_buf.read(data.len(), |buf_data| {
            data[..buf_data.len()].copy_from_slice(buf_data);
            Ok(buf_data.len())
        })
    }

    /// Whether host bytes remain buffered beyond what [`read`](Self::read) has returned.
    pub fn read_pending(&self) -> bool {
        self.read_buf.available_read() > 0
    }

    /// Offer staged bytes to IN; `Ok` means staging is empty, not that the host received the bytes.
    pub fn flush(&mut self) -> Result<()> {
        if !self.configured || self.in_halted {
            return Err(UsbError::WouldBlock);
        }
        let buf = &mut self.write_buf;
        let inner = &mut self.inner;
        let write_state = &mut self.write_state;

        let full_count = match *write_state {
            WriteState::Full(c) => c,
            _ => 0,
        };

        if buf.available_read() > 0 {
            let max_write_size = if full_count >= SHORT_PACKET_INTERVAL {
                inner.max_packet_size() - 1
            } else {
                inner.max_packet_size()
            } as usize;

            buf.read(max_write_size, |buf_data| {
                inner.write_packet(buf_data)?;

                *write_state = if buf_data.len() == inner.max_packet_size() as usize {
                    WriteState::Full(full_count + 1)
                } else {
                    WriteState::Short
                };

                Ok(buf_data.len())
            })?;

            Err(UsbError::WouldBlock)
        } else if full_count != 0 {
            // Terminate a trailing full packet with a ZLP so the host sees the transfer end.
            inner.write_packet(&[])?;
            *write_state = WriteState::Short;
            Err(UsbError::WouldBlock)
        } else {
            *write_state = WriteState::Idle;
            Ok(())
        }
    }

    fn set_configuration(&mut self, configured: bool) {
        // Retire endpoint ownership before clearing software state or restoring DATA0.
        self.inner.read_ep.stall();
        self.inner.write_ep.stall();
        self.inner.comm_ep.stall();
        self.reset();
        self.configured = configured;
        if configured {
            self.inner.read_ep.unstall();
            self.inner.write_ep.unstall();
            self.inner.comm_ep.unstall();
        }
    }
}

impl<B: UsbBus, const PKT: usize, const BUF: usize> UsbClass<B> for BufferedCdc<'_, B, PKT, BUF> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        self.inner.get_configuration_descriptors(writer)
    }

    fn get_string(&self, index: StringIndex, lang_id: LangID) -> Option<&str> {
        self.inner.get_string(index, lang_id)
    }

    fn reset(&mut self) {
        self.configured = false;
        self.in_halted = false;
        self.inner.reset();
        self.read_buf.clear();
        self.discarded += self.write_buf.available_read();
        self.write_buf.clear();
        self.write_state = WriteState::Idle;
        self.touched = true;
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.inner.read_ep.address() {
            self.touched = true;
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.inner.write_ep.address() {
            self.touched = true;
            self.flush().ok();
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        self.inner.control_in(xfer);
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        self.touched = true;
        let req = xfer.request();
        if let Some(halted) = halt_request(req, self.inner.write_ep.address()) {
            self.in_halted = halted;
        }
        if req.request_type == RequestType::Standard
            && req.recipient == Recipient::Device
            && req.request == Request::SET_CONFIGURATION
            && req.index == 0
            && req.length == 0
            && req.value <= 1
        {
            self.set_configuration(req.value == 1);
            // UsbDevice owns the standard request's status stage.
            return;
        }
        self.inner.control_out(xfer);
        self.discard_on_close();
    }
}

#[cfg(test)]
mod tests {
    use super::{BufferedCdc, CdcBuffer};
    use core::convert::Infallible;
    use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
    use std::collections::VecDeque;
    use std::sync::Mutex;
    use std::vec;
    use std::vec::Vec;
    use usb_device::bus::{PollResult, UsbBus, UsbBusAllocator};
    use usb_device::class_prelude::{EndpointAddress, EndpointType, UsbClass, UsbError};
    use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
    use usb_device::{Result, UsbDirection};

    /// A write past the free tail discards already-read data first, then fits.
    #[test]
    fn buffer_write_read_discard_cycle() {
        const CAP: usize = 5;
        let data: [u8; 7] = [1, 2, 3, 4, 5, 6, 7];
        let mut b = CdcBuffer::<CAP>::new();

        assert_eq!(b.write(&data[0..4]), 4);
        b.read(2, |d| {
            assert_eq!(d, &data[0..2]);
            Ok::<usize, Infallible>(2)
        })
        .unwrap();
        assert_eq!(b.write(&data[4..7]), 3);
        b.read(5, |d| {
            assert_eq!(d, &data[2..7]);
            Ok::<usize, Infallible>(5)
        })
        .unwrap();
        assert_eq!(b.available_read(), 0);
    }

    /// `write_all` commits 0 when even a discard can't make room, else reserves `max_count`.
    #[test]
    fn buffer_write_all_reserves_or_refuses() {
        const CAP: usize = 4;
        let mut b = CdcBuffer::<CAP>::new();

        let n = b
            .write_all::<Infallible>(5, |_| panic!("must not run without room"))
            .unwrap();
        assert_eq!(n, 0);

        let n = b
            .write_all::<Infallible>(3, |slot| {
                assert_eq!(slot.len(), 3);
                slot[..2].copy_from_slice(&[0xAA, 0xBB]);
                Ok(2)
            })
            .unwrap();
        assert_eq!(n, 2);
        assert_eq!(b.available_read(), 2);
    }

    /// A bus answering every transfer; IN reports busy unless a test opens it.
    struct MockBus {
        next_ep: AtomicU8,
        accepts_in: AtomicBool,
        halted: AtomicU32,
    }

    impl MockBus {
        fn alloc(accepts_in: bool) -> UsbBusAllocator<Self> {
            UsbBusAllocator::new(Self {
                next_ep: AtomicU8::new(1),
                accepts_in: AtomicBool::new(accepts_in),
                halted: AtomicU32::new(0),
            })
        }
    }

    impl UsbBus for MockBus {
        fn alloc_ep(
            &mut self,
            ep_dir: UsbDirection,
            ep_addr: Option<EndpointAddress>,
            _ep_type: EndpointType,
            _max_packet_size: u16,
            _interval: u8,
        ) -> Result<EndpointAddress> {
            Ok(ep_addr.unwrap_or_else(|| {
                EndpointAddress::from_parts(
                    self.next_ep.fetch_add(1, Ordering::Relaxed) as usize,
                    ep_dir,
                )
            }))
        }

        fn enable(&mut self) {}
        fn reset(&self) {}
        fn set_device_address(&self, _addr: u8) {}

        fn write(&self, _ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
            if self.accepts_in.load(Ordering::Relaxed) {
                Ok(buf.len())
            } else {
                Err(UsbError::WouldBlock)
            }
        }

        fn read(&self, _ep_addr: EndpointAddress, _buf: &mut [u8]) -> Result<usize> {
            Err(UsbError::WouldBlock)
        }

        fn set_stalled(&self, ep: EndpointAddress, stalled: bool) {
            let mask = 1 << (ep.index() * 2 + usize::from(ep.is_out()));
            if stalled {
                self.halted.fetch_or(mask, Ordering::Relaxed);
            } else {
                self.halted.fetch_and(!mask, Ordering::Relaxed);
            }
        }
        fn is_stalled(&self, ep: EndpointAddress) -> bool {
            self.halted.load(Ordering::Relaxed) & (1 << (ep.index() * 2 + usize::from(ep.is_out())))
                != 0
        }
        fn suspend(&self) {}
        fn resume(&self) {}
        fn poll(&self) -> PollResult {
            PollResult::None
        }
    }

    /// Freeze the allocation, without which the endpoints panic on first use.
    fn device(alloc: &UsbBusAllocator<MockBus>) -> UsbDevice<'_, MockBus> {
        UsbDeviceBuilder::new(alloc, UsbVidPid(0x1209, 0x0001)).build()
    }

    /// The return leg's ledger: staging credits `cdc_in`, a discard moves it to `host_dropped`.
    #[derive(Default)]
    struct Ledger {
        uart_rx: u32,
        staged: u32,
        host_dropped: u32,
    }

    impl Ledger {
        fn cdc_in(&self) -> u32 {
            self.staged.wrapping_sub(self.host_dropped)
        }

        fn holds(&self) -> bool {
            self.uart_rx == self.cdc_in().wrapping_add(self.host_dropped)
        }
    }

    /// Stage `data` through the class, crediting it the way the firmware seam does.
    fn stage(cdc: &mut BufferedCdc<'_, MockBus>, ledger: &mut Ledger, data: &[u8]) {
        let staged = cdc.write_buffered(data);
        assert_eq!(staged, data.len());
        ledger.uart_rx += data.len() as u32;
        ledger.staged += staged as u32;
    }

    /// A falling DTR discards the stage, and the bytes land in `host_dropped` instead of vanishing.
    #[test]
    fn port_close_reports_the_discarded_stage() {
        let alloc = MockBus::alloc(false);
        let mut cdc = BufferedCdc::<MockBus>::new(&alloc);
        let _dev = device(&alloc);
        cdc.set_configuration(true);
        let mut ledger = Ledger::default();

        stage(&mut cdc, &mut ledger, &[0xA5; 10]);
        assert_eq!(cdc.staged(), 10);
        assert!(ledger.holds());

        cdc.inner.set_control_line_state(true);
        cdc.inner.set_control_line_state(false);

        ledger.host_dropped += cdc.take_discarded() as u32;
        assert_eq!(ledger.host_dropped, 10);
        assert_eq!(ledger.cdc_in(), 0);
        assert!(ledger.holds());
        assert_eq!(cdc.staged(), 0);
        // The report is a take: the next pass must not charge the same bytes again.
        assert_eq!(cdc.take_discarded(), 0);
    }

    /// The close settles at the falling edge: nothing reaches the endpoint before the discard.
    #[test]
    fn port_close_empties_the_stage_before_the_next_flush() {
        let alloc = MockBus::alloc(true);
        let mut cdc = BufferedCdc::<MockBus>::new(&alloc);
        let _dev = device(&alloc);
        cdc.set_configuration(true);
        let mut ledger = Ledger::default();

        stage(&mut cdc, &mut ledger, &[0xC3; 12]);
        cdc.inner.set_control_line_state(true);
        cdc.inner.set_control_line_state(false);

        ledger.host_dropped += cdc.take_discarded() as u32;
        assert_eq!(ledger.host_dropped, 12);
        assert_eq!(cdc.staged(), 0);
        // The endpoint accepts writes here, so a surviving byte would have gone out.
        assert!(cdc.flush().is_ok());
        assert!(ledger.holds());

        stage(&mut cdc, &mut ledger, &[0x7E; 4]);
        assert_eq!(cdc.take_discarded(), 0);
        assert_eq!(cdc.staged(), 4);
    }

    /// A bus reset discards the stage, and reports it the same way.
    #[test]
    fn bus_reset_reports_the_discarded_stage() {
        let alloc = MockBus::alloc(false);
        let mut cdc = BufferedCdc::<MockBus>::new(&alloc);
        let _dev = device(&alloc);
        cdc.set_configuration(true);
        let mut ledger = Ledger::default();

        stage(&mut cdc, &mut ledger, &[0x5A; 20]);
        UsbClass::reset(&mut cdc);

        ledger.host_dropped += cdc.take_discarded() as u32;
        assert_eq!(ledger.host_dropped, 20);
        assert_eq!(ledger.cdc_in(), 0);
        assert!(ledger.holds());
        assert_eq!(cdc.staged(), 0);
    }

    /// Bytes the endpoint took are gone from the stage, so a later close cannot charge them.
    #[test]
    fn accepted_bytes_are_never_charged_as_discarded() {
        let alloc = MockBus::alloc(true);
        let mut cdc = BufferedCdc::<MockBus>::new(&alloc);
        let _dev = device(&alloc);
        cdc.set_configuration(true);
        let mut ledger = Ledger::default();

        stage(&mut cdc, &mut ledger, &[0x11; 8]);
        cdc.flush().ok();
        assert_eq!(cdc.staged(), 0);

        stage(&mut cdc, &mut ledger, &[0x22; 5]);
        UsbClass::reset(&mut cdc);

        ledger.host_dropped += cdc.take_discarded() as u32;
        assert_eq!(ledger.host_dropped, 5);
        assert_eq!(ledger.cdc_in(), 8);
        assert!(ledger.holds());
    }

    /// One IN buffer the bridge arms and the host empties, as a device controller holds it.
    struct HostBus {
        next_ep: AtomicU8,
        armed: Mutex<Option<Vec<u8>>>,
    }

    impl UsbBus for HostBus {
        fn alloc_ep(
            &mut self,
            ep_dir: UsbDirection,
            ep_addr: Option<EndpointAddress>,
            _ep_type: EndpointType,
            _max_packet_size: u16,
            _interval: u8,
        ) -> Result<EndpointAddress> {
            Ok(ep_addr.unwrap_or_else(|| {
                EndpointAddress::from_parts(
                    self.next_ep.fetch_add(1, Ordering::Relaxed) as usize,
                    ep_dir,
                )
            }))
        }

        fn enable(&mut self) {}
        fn reset(&self) {}
        fn set_device_address(&self, _addr: u8) {}

        fn write(&self, _ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
            let mut armed = self.armed.lock().unwrap();
            if armed.is_some() {
                return Err(UsbError::WouldBlock);
            }
            *armed = Some(buf.to_vec());
            Ok(buf.len())
        }

        fn read(&self, _ep_addr: EndpointAddress, _buf: &mut [u8]) -> Result<usize> {
            Err(UsbError::WouldBlock)
        }

        fn set_stalled(&self, _ep: EndpointAddress, _stalled: bool) {}
        fn is_stalled(&self, _ep: EndpointAddress) -> bool {
            false
        }
        fn suspend(&self) {}
        fn resume(&self) {}
        fn poll(&self) -> PollResult {
            PollResult::None
        }
    }

    /// A polled firmware's return leg: its UART ring, the class, and what each opener received.
    struct PolledPort<'a, const PKT: usize, const BUF: usize> {
        cdc: BufferedCdc<'a, HostBus, PKT, BUF>,
        dev: UsbDevice<'a, HostBus>,
        ring: VecDeque<u8>,
        reading: bool,
        sessions: Vec<Vec<u8>>,
        discarded: usize,
        /// Ring bytes the firmware dropped at a close edge.
        ring_dropped: usize,
    }

    impl<const PKT: usize, const BUF: usize> PolledPort<'_, PKT, BUF> {
        /// One service pass as `vcom_return` runs it, then the host's IN token if a port is open.
        fn pass(&mut self) {
            let mut chunk = [0u8; PKT];
            if self.cdc.take_close_edge() {
                self.ring_dropped += self.ring.len();
                self.ring.clear();
            }
            loop {
                let n = self.cdc.write_capacity().min(PKT).min(self.ring.len());
                if n == 0 {
                    break;
                }
                for (slot, byte) in chunk.iter_mut().zip(self.ring.drain(..n)) {
                    *slot = byte;
                }
                assert_eq!(self.cdc.write_buffered(&chunk[..n]), n);
            }
            self.cdc.flush().ok();
            self.discarded += self.cdc.take_discarded();
            if self.reading {
                if let Some(packet) = self.dev.bus().armed.lock().unwrap().take() {
                    self.sessions.last_mut().unwrap().extend(packet);
                }
            }
        }

        fn passes(&mut self, n: usize) {
            for _ in 0..n {
                self.pass();
            }
        }

        fn open(&mut self) {
            self.cdc.inner.set_control_line_state(true);
            self.sessions.push(Vec::new());
            self.reading = true;
        }

        fn close(&mut self) {
            self.cdc.inner.set_control_line_state(false);
            self.reading = false;
        }
    }

    fn polled_port<const PKT: usize, const BUF: usize>(
        alloc: &UsbBusAllocator<HostBus>,
    ) -> PolledPort<'_, PKT, BUF> {
        let mut cdc = BufferedCdc::<HostBus, PKT, BUF>::new(alloc);
        let dev = UsbDeviceBuilder::new(alloc, UsbVidPid(0x1209, 0x0001)).build();
        cdc.set_configuration(true);
        PolledPort {
            cdc,
            dev,
            ring: VecDeque::new(),
            reading: false,
            sessions: Vec::new(),
            discarded: 0,
            ring_dropped: 0,
        }
    }

    fn host_bus() -> UsbBusAllocator<HostBus> {
        UsbBusAllocator::new(HostBus {
            next_ep: AtomicU8::new(1),
            armed: Mutex::new(None),
        })
    }

    /// A close discards the stage and the UART ring at the edge; only the armed packet survives it.
    fn close_discards_all_but_the_armed_packet<const PKT: usize, const BUF: usize>() {
        let alloc = host_bus();
        let mut port = polled_port::<PKT, BUF>(&alloc);
        port.open();
        port.passes(2);
        // The reader stops keeping up, then the port closes.
        port.reading = false;
        port.ring.extend(core::iter::repeat_n(b'A', 3 * BUF));
        port.passes(4);
        let held = 3 * BUF;
        port.close();
        port.passes(4);
        assert_eq!(port.discarded, BUF);
        assert_eq!(port.ring_dropped, held - BUF - PKT);
        port.open();
        port.passes(4 * BUF);

        assert_eq!(port.sessions[1], vec![b'A'; PKT]);
        assert!(port.ring.is_empty());
    }

    #[test]
    fn a_full_speed_close_discards_all_but_the_armed_packet() {
        close_discards_all_but_the_armed_packet::<64, 128>();
    }

    #[test]
    fn a_high_speed_close_discards_all_but_the_armed_packet() {
        close_discards_all_but_the_armed_packet::<512, 2048>();
    }

    /// A bus reset ends an open session, so the firmware drops its ring as at a close.
    #[test]
    fn a_bus_reset_of_an_open_port_is_a_close_edge() {
        let alloc = host_bus();
        let mut port = polled_port::<64, 128>(&alloc);
        port.passes(1);
        UsbClass::reset(&mut port.cdc);
        assert!(!port.cdc.take_close_edge());
        port.open();
        port.passes(1);
        UsbClass::reset(&mut port.cdc);
        assert!(port.cdc.take_close_edge());
        assert!(!port.cdc.take_close_edge());
    }

    /// Nothing gates the return leg while DTR is low, so post-close bytes wait for the next opener.
    fn post_close_bytes_reach_the_next_opener<const PKT: usize, const BUF: usize>() {
        let alloc = host_bus();
        let mut port = polled_port::<PKT, BUF>(&alloc);
        port.open();
        port.ring.extend([b'A'; 100]);
        port.passes(8);
        assert_eq!(port.sessions[0], vec![b'A'; 100]);
        port.close();
        port.passes(4);
        port.ring.extend(core::iter::repeat_n(b'B', 2 * BUF));
        port.passes(100);
        port.open();
        port.passes(4 * BUF);

        assert_eq!(port.discarded, 0);
        assert_eq!(port.ring_dropped, 0);
        assert_eq!(port.sessions[1], vec![b'B'; 2 * BUF]);
    }

    #[test]
    fn post_close_bytes_reach_the_next_full_speed_opener() {
        post_close_bytes_reach_the_next_opener::<64, 128>();
    }

    #[test]
    fn post_close_bytes_reach_the_next_high_speed_opener() {
        post_close_bytes_reach_the_next_opener::<512, 2048>();
    }

    #[test]
    fn configuration_boundary_retires_all_endpoints_and_staging() {
        let alloc = MockBus::alloc(true);
        let mut cdc = BufferedCdc::<MockBus>::new(&alloc);
        let _dev = device(&alloc);
        cdc.set_configuration(true);
        assert_eq!(cdc.write_buffered(&[1; 12]), 12);
        assert_eq!(cdc.read_buf.write(&[2; 8]), 8);
        cdc.write_state = super::WriteState::Full(3);
        cdc.set_configuration(false);
        assert!(_dev.bus().is_stalled(cdc.inner.read_ep.address()));
        assert!(_dev.bus().is_stalled(cdc.inner.write_ep.address()));
        assert!(_dev.bus().is_stalled(cdc.inner.comm_ep.address()));
        assert_eq!(cdc.take_discarded(), 12);
        assert_eq!(cdc.take_discarded(), 0);
        assert!(!cdc.read_pending());
        assert!(!cdc.in_flight());
        assert_eq!(cdc.write_capacity(), 0);
        assert_eq!(cdc.flush(), Err(UsbError::WouldBlock));
        assert_eq!(cdc.read(&mut [0; 8]), Err(UsbError::WouldBlock));
        cdc.set_configuration(true);
        assert!(!_dev.bus().is_stalled(cdc.inner.read_ep.address()));
        assert!(!_dev.bus().is_stalled(cdc.inner.write_ep.address()));
        assert!(!_dev.bus().is_stalled(cdc.inner.comm_ep.address()));
        assert!(cdc.write_capacity() > 0);
        assert_eq!(cdc.take_discarded(), 0);
        assert_eq!(cdc.flush(), Ok(()));
        assert_eq!(cdc.write_buffered(&[3; 7]), 7);
        cdc.set_configuration(true);
        assert_eq!(cdc.take_discarded(), 7);
        assert!(!cdc.read_pending());
    }

    /// A bus scripted from the host side: EP0 packets queued, EP0 IN writes and stalls recorded.
    struct ScriptBus {
        next_ep: AtomicU8,
        polls: Mutex<VecDeque<(u16, u16, u16)>>,
        ep0: Mutex<VecDeque<Vec<u8>>>,
        in_written: Mutex<Vec<Vec<u8>>>,
        halted: AtomicU32,
    }

    impl UsbBus for ScriptBus {
        fn alloc_ep(
            &mut self,
            ep_dir: UsbDirection,
            ep_addr: Option<EndpointAddress>,
            _ep_type: EndpointType,
            _max_packet_size: u16,
            _interval: u8,
        ) -> Result<EndpointAddress> {
            Ok(ep_addr.unwrap_or_else(|| {
                EndpointAddress::from_parts(
                    self.next_ep.fetch_add(1, Ordering::Relaxed) as usize,
                    ep_dir,
                )
            }))
        }
        fn enable(&mut self) {}
        fn reset(&self) {}
        fn set_device_address(&self, _addr: u8) {}
        fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
            if ep_addr.index() == 0 {
                self.in_written.lock().unwrap().push(buf.to_vec());
            }
            Ok(buf.len())
        }
        fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
            if ep_addr.index() != 0 {
                return Err(UsbError::WouldBlock);
            }
            match self.ep0.lock().unwrap().pop_front() {
                Some(packet) => {
                    buf[..packet.len()].copy_from_slice(&packet);
                    Ok(packet.len())
                }
                None => Err(UsbError::WouldBlock),
            }
        }
        fn set_stalled(&self, ep: EndpointAddress, stalled: bool) {
            let mask = 1 << (ep.index() * 2 + usize::from(ep.is_out()));
            if stalled {
                self.halted.fetch_or(mask, Ordering::Relaxed);
            } else {
                self.halted.fetch_and(!mask, Ordering::Relaxed);
            }
        }
        fn is_stalled(&self, ep: EndpointAddress) -> bool {
            self.halted.load(Ordering::Relaxed) & (1 << (ep.index() * 2 + usize::from(ep.is_out())))
                != 0
        }
        fn suspend(&self) {}
        fn resume(&self) {}
        fn poll(&self) -> PollResult {
            match self.polls.lock().unwrap().pop_front() {
                Some((ep_out, ep_in_complete, ep_setup)) => PollResult::Data {
                    ep_out,
                    ep_in_complete,
                    ep_setup,
                },
                None => PollResult::None,
            }
        }
    }

    fn script_bus() -> UsbBusAllocator<ScriptBus> {
        UsbBusAllocator::new(ScriptBus {
            next_ep: AtomicU8::new(1),
            polls: Mutex::new(VecDeque::new()),
            ep0: Mutex::new(VecDeque::new()),
            in_written: Mutex::new(Vec::new()),
            halted: AtomicU32::new(0),
        })
    }

    fn ep0_in() -> EndpointAddress {
        EndpointAddress::from_parts(0, UsbDirection::In)
    }

    /// A class request as the host controller delivers it; a new SETUP clears an earlier stall.
    fn class_request(
        dev: &mut UsbDevice<'_, ScriptBus>,
        cdc: &mut BufferedCdc<'_, ScriptBus>,
        request_type: u8,
        request: u8,
        value: u16,
        length: u16,
        data: &[u8],
    ) {
        let [v0, v1] = value.to_le_bytes();
        let [l0, l1] = length.to_le_bytes();
        let bus = dev.bus();
        bus.halted.store(0, Ordering::Relaxed);
        bus.ep0
            .lock()
            .unwrap()
            .push_back(vec![request_type, request, v0, v1, 0, 0, l0, l1]);
        bus.polls.lock().unwrap().push_back((0, 0, 1));
        if !data.is_empty() {
            bus.ep0.lock().unwrap().push_back(data.to_vec());
            bus.polls.lock().unwrap().push_back((1, 0, 0));
        }
        dev.poll(&mut [cdc]);
        dev.poll(&mut [cdc]);
    }

    fn set_line_coding(
        dev: &mut UsbDevice<'_, ScriptBus>,
        cdc: &mut BufferedCdc<'_, ScriptBus>,
        baud: u32,
        format: [u8; 3],
    ) {
        let mut record = baud.to_le_bytes().to_vec();
        record.extend_from_slice(&format);
        class_request(dev, cdc, 0x21, 0x20, 0, 7, &record);
    }

    fn get_line_coding(
        dev: &mut UsbDevice<'_, ScriptBus>,
        cdc: &mut BufferedCdc<'_, ScriptBus>,
    ) -> Vec<u8> {
        class_request(dev, cdc, 0xA1, 0x21, 0, 7, &[]);
        dev.bus()
            .in_written
            .lock()
            .unwrap()
            .last()
            .cloned()
            .unwrap_or_default()
    }

    fn coding(baud: u32, format: [u8; 3]) -> Vec<u8> {
        let mut record = baud.to_le_bytes().to_vec();
        record.extend_from_slice(&format);
        record
    }

    /// 8N1 at a new rate is accepted, followed, and echoed.
    #[test]
    fn an_8n1_line_coding_is_followed_and_echoed() {
        let alloc = script_bus();
        let mut cdc = BufferedCdc::<ScriptBus>::new(&alloc);
        let mut dev = UsbDeviceBuilder::new(&alloc, UsbVidPid(0x1209, 0x0001)).build();
        set_line_coding(&mut dev, &mut cdc, 230_400, [0, 0, 8]);
        assert!(!dev.bus().is_stalled(ep0_in()));
        assert_eq!(cdc.line_coding_baud(), 230_400);
        assert_eq!(
            get_line_coding(&mut dev, &mut cdc),
            coding(230_400, [0, 0, 8])
        );
    }

    /// 7E1 at a new rate: the framing is refused, the rate followed, and 8N1 reported.
    #[test]
    fn a_refused_framing_still_moves_the_baud() {
        let alloc = script_bus();
        let mut cdc = BufferedCdc::<ScriptBus>::new(&alloc);
        let mut dev = UsbDeviceBuilder::new(&alloc, UsbVidPid(0x1209, 0x0001)).build();
        set_line_coding(&mut dev, &mut cdc, 230_400, [0, 2, 7]);
        assert!(
            dev.bus().is_stalled(ep0_in()),
            "a framing other than 8N1 is refused"
        );
        assert_eq!(
            cdc.line_coding_baud(),
            230_400,
            "the rate in a refused record is lost with it"
        );
        assert_eq!(
            get_line_coding(&mut dev, &mut cdc),
            coding(230_400, [0, 0, 8])
        );
    }
}
