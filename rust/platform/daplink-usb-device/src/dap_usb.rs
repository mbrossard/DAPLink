//! Polled DAP products, with CDC and MSC selected by composition.

use crate::cdc::BufferedCdc;
use crate::dap_class::CmsisDapV2;
use crate::dap_pass::DapTransport;
use crate::msc::MscClass;
use crate::product::{MassStorage, NoMsc};
use crate::winusb::{EndpointRepair, VendorDescriptors};
use daplink_rs::daplink_core::{DapConfig, USB_MAX_POWER_MA};
use usb_device::bus::{UsbBus, UsbBusAllocator};
use usb_device::device::{
    StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid,
};

#[cfg(test)]
mod tests;

/// The polled USB device, DAP v2 on interface 0; `CDC_BUF` (>= `CDC_EP`) sizes the serial buffers.
/// `MSC_EP` matters only when `M` is [`MscClass`].
pub struct DapUsb<
    'a,
    B: UsbBus,
    const MSC_EP: usize = 64,
    const CDC_EP: usize = 64,
    const CDC_BUF: usize = 128,
    M = NoMsc,
> {
    usb_dev: UsbDevice<'a, B>,
    dap_v2: CmsisDapV2<'a, B>,
    vendor: VendorDescriptors,
    serial: BufferedCdc<'a, B, CDC_EP, CDC_BUF>,
    /// The mass-storage class, allocated last so it never shifts the interfaces above it.
    msc: M,
    session: SessionTracker,
}

/// DAP, CDC, and MSC, with the same interface ordering as the non-MSC composition.
pub type DapUsbMsc<
    'a,
    B,
    const MSC_EP: usize = 64,
    const CDC_EP: usize = 64,
    const CDC_BUF: usize = 128,
> = DapUsb<'a, B, MSC_EP, CDC_EP, CDC_BUF, MscClass<'a, B, MSC_EP>>;

impl<'a, B: UsbBus, const MSC_EP: usize, const CDC_EP: usize, const CDC_BUF: usize, M>
    DapUsb<'a, B, MSC_EP, CDC_EP, CDC_BUF, M>
where
    M: MassStorage<'a, B, MSC_EP>,
{
    /// Build the device from a bus allocator and the probe's USB identity; call once per image.
    pub fn new(alloc: &'a UsbBusAllocator<B>, cfg: &DapConfig<'a>) -> Self {
        let dap_v2 = CmsisDapV2::new(cfg.max_packet_size.get(), alloc);
        // Interfaces 1 and 2 for the CDC-ACM function, after DAP v2's interface 0.
        let serial = BufferedCdc::new(alloc);
        // Mass storage takes interface 3, so every interface below it keeps its number.
        let msc = M::new(alloc);
        let usb_dev = UsbDeviceBuilder::new(alloc, UsbVidPid(cfg.vid, cfg.pid))
            .strings(&[StringDescriptors::default()
                .manufacturer(cfg.manufacturer)
                .product(cfg.product)
                .serial_number(cfg.serial)])
            .expect("string descriptor set is within the usb-device string limit")
            .max_packet_size_0(cfg.max_packet_size_0)
            .expect("max_packet_size_0 is a USB-legal EP0 size (8/16/32/64)")
            .max_power(USB_MAX_POWER_MA as usize)
            .expect("USB_MAX_POWER_MA is within the 500 mA bus-power limit")
            // The CDC-ACM function needs an IAD, which Windows only honors on an EF/02/01 device.
            .composite_with_iads()
            .build();
        Self {
            usb_dev,
            dap_v2,
            vendor: VendorDescriptors::default(),
            serial,
            msc,
            session: SessionTracker::new(),
        }
    }

    /// Refuse endpoint requests the bus lacks a register for, and repair its toggles through `repair`.
    pub fn with_endpoint_repair(mut self, repair: &'static dyn EndpointRepair) -> Self {
        self.vendor = VendorDescriptors::with_repair(repair);
        self
    }

    /// Poll every class; `true` indicates the USB stack's data-event path, not every bus event.
    pub fn poll(&mut self) -> bool {
        self.msc.poll_vcom(
            &mut self.usb_dev,
            &mut self.dap_v2,
            &mut self.vendor,
            &mut self.serial,
        )
    }

    /// Read a landed command packet into `buf`, returning its length, or `None` when none is pending.
    pub fn read_command(&mut self, buf: &mut [u8]) -> Option<usize> {
        self.configured()
            .then(|| self.dap_v2.read_command(buf))
            .flatten()
    }

    /// Write one response packet, returning `false` when it is oversized or the endpoint is busy.
    pub fn write_response(&mut self, data: &[u8]) -> bool {
        self.configured() && self.dap_v2.write(data)
    }

    /// Whether the host touched the VCOM port since the last call.
    pub fn vcom_activity(&mut self) -> bool {
        self.serial.take_activity()
    }

    /// Read bytes the host wrote to the VCOM serial port; `0` when none are waiting.
    pub fn vcom_read(&mut self, buf: &mut [u8]) -> usize {
        self.serial.read(buf).unwrap_or(0)
    }

    /// Whether host bytes remain past [`vcom_read`](Self::vcom_read); a packet can outsize a read.
    pub fn vcom_read_pending(&self) -> bool {
        self.serial.read_pending()
    }

    /// Bytes [`vcom_write`](Self::vcom_write) is guaranteed to take whole.
    pub fn vcom_write_capacity(&self) -> usize {
        self.serial.write_capacity()
    }

    /// Accept VCOM bytes into software staging; the count does not imply endpoint or host receipt.
    pub fn vcom_write(&mut self, data: &[u8]) -> usize {
        self.serial.write(data).unwrap_or(0)
    }

    /// Buffer VCOM bytes toward the host without offering them to the endpoint.
    pub fn vcom_write_buffered(&mut self, data: &[u8]) -> usize {
        self.serial.write_buffered(data)
    }

    /// VCOM bytes staged toward the host and not yet offered to the endpoint.
    pub fn vcom_staged(&self) -> usize {
        self.serial.staged()
    }

    /// Whether a VCOM packet sits at the IN endpoint awaiting completion.
    pub fn vcom_in_flight(&self) -> bool {
        self.serial.in_flight()
    }

    /// Take the staged VCOM bytes a port close or a bus reset discarded; the host never got them.
    pub fn vcom_take_discarded(&mut self) -> usize {
        self.serial.take_discarded()
    }

    /// Whether a port close or bus reset ended a VCOM session since the last call.
    pub fn vcom_take_close_edge(&mut self) -> bool {
        self.serial.take_close_edge()
    }

    /// Push staged VCOM bytes at the IN endpoint; nothing else empties a buffer that filled.
    pub fn vcom_flush(&mut self) {
        let _ = self.serial.flush();
    }

    /// The host's requested VCOM baud from the current CDC line coding.
    pub fn vcom_line_coding_baud(&self) -> u32 {
        self.serial.line_coding_baud()
    }

    /// Whether enumeration is over: addressed, descriptors read, and a configuration selected.
    pub fn configured(&self) -> bool {
        self.usb_dev.state() == UsbDeviceState::Configured
    }

    /// Take a reset or deconfiguration event; the caller retires queued work and suspends the engine.
    pub fn session_ended(&mut self) -> bool {
        let state = self.usb_dev.state();
        let reset = self.dap_v2.take_reset();
        self.session.observe(state, reset)
    }
}

/// DAP without CDC; the optional MSC class follows interface 0.
pub struct DapUsbNoVcom<'a, B: UsbBus, const MSC_EP: usize = 64, M = NoMsc> {
    usb_dev: UsbDevice<'a, B>,
    dap_v2: CmsisDapV2<'a, B>,
    vendor: VendorDescriptors,
    /// The mass-storage class, allocated last so it never shifts the interfaces above it.
    msc: M,
    session: SessionTracker,
}

/// DAP and MSC without CDC state or descriptors.
pub type DapUsbNoVcomMsc<'a, B, const MSC_EP: usize = 64> =
    DapUsbNoVcom<'a, B, MSC_EP, MscClass<'a, B, MSC_EP>>;

impl<'a, B: UsbBus, const MSC_EP: usize, M> DapUsbNoVcom<'a, B, MSC_EP, M>
where
    M: MassStorage<'a, B, MSC_EP>,
{
    /// Build the device from a bus allocator and the probe's USB identity; call once per image.
    pub fn new(alloc: &'a UsbBusAllocator<B>, cfg: &DapConfig<'a>) -> Self {
        let dap_v2 = CmsisDapV2::new(cfg.max_packet_size.get(), alloc);
        // Mass storage takes interface 1, so every interface below it keeps its number.
        let msc = M::new(alloc);
        let usb_dev = UsbDeviceBuilder::new(alloc, UsbVidPid(cfg.vid, cfg.pid))
            .strings(&[StringDescriptors::default()
                .manufacturer(cfg.manufacturer)
                .product(cfg.product)
                .serial_number(cfg.serial)])
            .expect("string descriptor set is within the usb-device string limit")
            .max_packet_size_0(cfg.max_packet_size_0)
            .expect("max_packet_size_0 is a USB-legal EP0 size (8/16/32/64)")
            .max_power(USB_MAX_POWER_MA as usize)
            .expect("USB_MAX_POWER_MA is within the 500 mA bus-power limit")
            // No CDC-ACM function, so no IAD and no composite device class.
            .build();
        Self {
            usb_dev,
            dap_v2,
            vendor: VendorDescriptors::default(),
            msc,
            session: SessionTracker::new(),
        }
    }

    /// Refuse endpoint requests the bus lacks a register for, and repair its toggles through `repair`.
    pub fn with_endpoint_repair(mut self, repair: &'static dyn EndpointRepair) -> Self {
        self.vendor = VendorDescriptors::with_repair(repair);
        self
    }

    /// Poll every class; `true` indicates the USB stack's data-event path, not every bus event.
    pub fn poll(&mut self) -> bool {
        self.msc
            .poll(&mut self.usb_dev, &mut self.dap_v2, &mut self.vendor)
    }

    /// Read a landed command packet into `buf`, returning its length, or `None` when none is pending.
    pub fn read_command(&mut self, buf: &mut [u8]) -> Option<usize> {
        self.configured()
            .then(|| self.dap_v2.read_command(buf))
            .flatten()
    }

    /// Write one response packet, returning `false` when it is oversized or the endpoint is busy.
    pub fn write_response(&mut self, data: &[u8]) -> bool {
        self.configured() && self.dap_v2.write(data)
    }

    /// Whether enumeration is over: addressed, descriptors read, and a configuration selected.
    pub fn configured(&self) -> bool {
        self.usb_dev.state() == UsbDeviceState::Configured
    }

    /// Take a reset or deconfiguration event; the caller retires queued work and suspends the engine.
    pub fn session_ended(&mut self) -> bool {
        let state = self.usb_dev.state();
        let reset = self.dap_v2.take_reset();
        self.session.observe(state, reset)
    }
}

impl<'a, B: UsbBus, const MSC_EP: usize, const CDC_EP: usize, const CDC_BUF: usize>
    DapUsbMsc<'a, B, MSC_EP, CDC_EP, CDC_BUF>
{
    /// Advance mass storage over the composed volume.
    pub fn msc_service<S: daplink_vfs::TransferSink, O: daplink_vfs::VfsObserver>(
        &mut self,
        vfs: &mut daplink_vfs::Vfs<'_, S, O>,
    ) {
        self.msc.service(vfs);
    }
}

impl<'a, B: UsbBus, const MSC_EP: usize> DapUsbNoVcomMsc<'a, B, MSC_EP> {
    /// Advance mass storage over the composed volume.
    pub fn msc_service<S: daplink_vfs::TransferSink, O: daplink_vfs::VfsObserver>(
        &mut self,
        vfs: &mut daplink_vfs::Vfs<'_, S, O>,
    ) {
        self.msc.service(vfs);
    }
}

impl<'a, B: UsbBus, const MSC_EP: usize, const CDC_EP: usize, const CDC_BUF: usize, M> DapTransport
    for DapUsb<'a, B, MSC_EP, CDC_EP, CDC_BUF, M>
where
    M: MassStorage<'a, B, MSC_EP>,
{
    fn session_ended(&mut self) -> bool {
        DapUsb::session_ended(self)
    }

    fn read_command(&mut self, slot: &mut [u8]) -> Option<usize> {
        DapUsb::read_command(self, slot)
    }

    fn write_response(&mut self, resp: &[u8]) -> bool {
        DapUsb::write_response(self, resp)
    }
}

impl<'a, B: UsbBus, const MSC_EP: usize, M> DapTransport for DapUsbNoVcom<'a, B, MSC_EP, M>
where
    M: MassStorage<'a, B, MSC_EP>,
{
    fn session_ended(&mut self) -> bool {
        DapUsbNoVcom::session_ended(self)
    }

    fn read_command(&mut self, slot: &mut [u8]) -> Option<usize> {
        DapUsbNoVcom::read_command(self, slot)
    }

    fn write_response(&mut self, resp: &[u8]) -> bool {
        DapUsbNoVcom::write_response(self, resp)
    }
}

/// The session-ended latch: a class reset, or `Configured` left for `Default` or `Addressed`.
struct SessionTracker {
    /// The last state seen outside `Suspend`, which does not end a session.
    last: UsbDeviceState,
}

impl SessionTracker {
    const fn new() -> Self {
        Self {
            last: UsbDeviceState::Default,
        }
    }

    fn observe(&mut self, state: UsbDeviceState, reset: bool) -> bool {
        let ended = self.last == UsbDeviceState::Configured
            && matches!(state, UsbDeviceState::Default | UsbDeviceState::Addressed);
        if state != UsbDeviceState::Suspend {
            self.last = state;
        }
        reset || ended
    }
}
