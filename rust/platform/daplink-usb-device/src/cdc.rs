//! Buffered CDC-ACM serial class with a firmware-chosen data-endpoint packet size.

use daplink_rs::daplink_core::DEFAULT_VCOM_BAUD;
use usb_device::class_prelude::*;
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

/// Byte buffer with `rpos <= wpos <= CAP`; read data is discarded (memmoved out) only when a
/// write would not otherwise fit.
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

    /// Append as much of `data` as fits, returning the count written.
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
    /// Allocate the interfaces and endpoints; at high speed `bInterval` is exponential, so 11
    /// (128 ms) replaces the full-speed 255.
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

    /// The bulk packet size (identical for both directions).
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
                // Only 8N1 ever reaches the wire; refuse the whole record rather than fake it.
                if data[4..7] != [0, 0, 8] {
                    xfer.reject().ok();
                    return;
                }
                self.line_coding.data_rate =
                    u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                xfer.accept().ok();
            }
            // A falling DTR is the host closing the port; what it left must not reach the next one.
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
        }
    }

    /// The baud the host last set via CDC line coding.
    pub fn line_coding_baud(&self) -> u32 {
        self.inner.line_coding.data_rate
    }

    /// Bytes [`write`](Self::write) is guaranteed to take whole.
    pub fn write_capacity(&self) -> usize {
        self.write_buf.available_write()
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

    /// Take the staged bytes a port close or a bus reset discarded; nothing else reports them, so
    /// a caller crediting bytes as it stages them must deduct these from what it called delivered.
    pub fn take_discarded(&mut self) -> usize {
        core::mem::take(&mut self.discarded)
    }

    /// Pull a pending OUT packet into the read buffer.
    fn poll(&mut self) -> Result<()> {
        let Self {
            inner,
            read_buf,
            write_buf,
            write_state,
            discarded,
        } = self;

        if inner.take_closed() {
            read_buf.clear();
            *discarded += write_buf.available_read();
            write_buf.clear();
            *write_state = WriteState::Idle;
        }

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

    /// Send as much of the write buffer as the IN endpoint accepts; `Ok` once fully drained.
    pub fn flush(&mut self) -> Result<()> {
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
}

impl<B: UsbBus, const PKT: usize, const BUF: usize> UsbClass<B> for BufferedCdc<'_, B, PKT, BUF> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        self.inner.get_configuration_descriptors(writer)
    }

    fn get_string(&self, index: StringIndex, lang_id: LangID) -> Option<&str> {
        self.inner.get_string(index, lang_id)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.read_buf.clear();
        self.discarded += self.write_buf.available_read();
        self.write_buf.clear();
        self.write_state = WriteState::Idle;
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.inner.write_ep.address() {
            self.flush().ok();
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        self.inner.control_in(xfer);
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        self.inner.control_out(xfer);
    }
}

#[cfg(test)]
mod tests {
    use super::{BufferedCdc, CdcBuffer};
    use core::convert::Infallible;
    use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
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

    /// A bus that hands out endpoint addresses and answers every transfer; the IN direction
    /// reports busy unless a test opens it, so staged bytes stay staged.
    struct MockBus {
        next_ep: AtomicU8,
        accepts_in: AtomicBool,
    }

    impl MockBus {
        fn alloc(accepts_in: bool) -> UsbBusAllocator<Self> {
            UsbBusAllocator::new(Self {
                next_ep: AtomicU8::new(1),
                accepts_in: AtomicBool::new(accepts_in),
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

        fn set_stalled(&self, _ep_addr: EndpointAddress, _stalled: bool) {}
        fn is_stalled(&self, _ep_addr: EndpointAddress) -> bool {
            false
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

    /// The return leg's ledger as a bridge firmware keeps it: staging credits `cdc_in`, and a
    /// discard moves that credit to `host_dropped`.
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
        let mut ledger = Ledger::default();

        stage(&mut cdc, &mut ledger, &[0xA5; 10]);
        assert_eq!(cdc.staged(), 10);
        assert!(ledger.holds());

        cdc.inner.set_control_line_state(true);
        cdc.inner.set_control_line_state(false);
        // Only a read drives the class poll that acts on the close.
        assert!(cdc.read(&mut [0u8; 8]).is_err());

        ledger.host_dropped += cdc.take_discarded() as u32;
        assert_eq!(ledger.host_dropped, 10);
        assert_eq!(ledger.cdc_in(), 0);
        assert!(ledger.holds());
        assert_eq!(cdc.staged(), 0);
        // The report is a take: the next pass must not charge the same bytes again.
        assert_eq!(cdc.take_discarded(), 0);
    }

    /// A bus reset discards the stage, and reports it the same way.
    #[test]
    fn bus_reset_reports_the_discarded_stage() {
        let alloc = MockBus::alloc(false);
        let mut cdc = BufferedCdc::<MockBus>::new(&alloc);
        let _dev = device(&alloc);
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
}
