//! The polled USB device: CMSIS-DAP v2, the vendor descriptors, and -- on [`DapUsb`] -- a CDC-ACM
//! serial port. A board with no VCOM wiring composes [`DapUsbNoVcom`], which never links the class.

use crate::cdc::BufferedCdc;
use crate::dap_class::CmsisDapV2;
#[cfg(feature = "flash")]
use crate::msc::MscClass;
use crate::winusb::VendorDescriptors;
use daplink_rs::daplink_core::{DapConfig, USB_MAX_POWER_MA};
use usb_device::bus::{UsbBus, UsbBusAllocator};
use usb_device::device::{
    StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid,
};

/// The polled USB device; DAP v2 is allocated first, so it lands on interface 0. `MSC_EP` and
/// `CDC_EP` size the bulk pairs, `CDC_BUF` (>= `CDC_EP`) the serial stream buffers.
pub struct DapUsb<
    'a,
    B: UsbBus,
    const MSC_EP: usize = 64,
    const CDC_EP: usize = 64,
    const CDC_BUF: usize = 128,
> {
    usb_dev: UsbDevice<'a, B>,
    dap_v2: CmsisDapV2<'a, B>,
    vendor: VendorDescriptors,
    serial: BufferedCdc<'a, B, CDC_EP, CDC_BUF>,
    /// The mass-storage class, allocated last so it never shifts the interfaces above it.
    #[cfg(feature = "flash")]
    msc: MscClass<'a, B, MSC_EP>,
    last_state: UsbDeviceState,
}

impl<'a, B: UsbBus, const MSC_EP: usize, const CDC_EP: usize, const CDC_BUF: usize>
    DapUsb<'a, B, MSC_EP, CDC_EP, CDC_BUF>
{
    /// Build the device from a bus allocator and the probe's USB identity; call once per image.
    pub fn new(alloc: &'a UsbBusAllocator<B>, cfg: &DapConfig<'a>) -> Self {
        let dap_v2 = CmsisDapV2::new(cfg.max_packet_size, alloc);
        // Interfaces 1 and 2 for the CDC-ACM function, after DAP v2's interface 0.
        let serial = BufferedCdc::new(alloc);
        // Mass storage takes interface 3, so every interface below it keeps its number.
        #[cfg(feature = "flash")]
        let msc = MscClass::new(alloc);
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
            vendor: VendorDescriptors,
            serial,
            #[cfg(feature = "flash")]
            msc,
            last_state: UsbDeviceState::Default,
        }
    }

    /// Poll the device across every class, returning whether there was bus activity.
    #[cfg(not(feature = "flash"))]
    pub fn poll(&mut self) -> bool {
        self.usb_dev
            .poll(&mut [&mut self.dap_v2, &mut self.vendor, &mut self.serial])
    }

    /// Poll every class; mass storage polls last, so it appends its descriptors after the rest.
    #[cfg(feature = "flash")]
    pub fn poll(&mut self) -> bool {
        self.usb_dev.poll(&mut [
            &mut self.dap_v2,
            &mut self.vendor,
            &mut self.serial,
            &mut self.msc,
        ])
    }

    /// Advance the mass-storage transport over `vfs` as far as the endpoints allow.
    #[cfg(feature = "flash")]
    pub fn msc_service<S, O>(&mut self, vfs: &mut daplink_vfs::Vfs<'_, S, O>)
    where
        S: daplink_vfs::TransferSink,
        O: daplink_vfs::VfsObserver,
    {
        self.msc.service(vfs);
    }

    /// Read a landed command packet into `buf`, returning its length, or `None` when none is pending.
    pub fn read_command(&mut self, buf: &mut [u8]) -> Option<usize> {
        self.dap_v2.read_command(buf)
    }

    /// Write one response packet, returning `false` when it is oversized or the endpoint is busy.
    pub fn write_response(&mut self, data: &[u8]) -> bool {
        self.dap_v2.write(data)
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

    /// Write bytes to the VCOM serial port, returning how many the endpoint accepted.
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

    /// Take the staged VCOM bytes a port close or a bus reset discarded; the host never got them.
    pub fn vcom_take_discarded(&mut self) -> usize {
        self.serial.take_discarded()
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

    /// Whether the device left `Configured` since the last call; the caller suspends the engine on it.
    pub fn session_ended(&mut self) -> bool {
        let state = self.usb_dev.state();
        let ended = self.last_state == UsbDeviceState::Configured
            && matches!(state, UsbDeviceState::Default | UsbDeviceState::Addressed);
        self.last_state = state;
        ended
    }
}

/// The polled USB device for a board that wires no VCOM: DAP v2 on interface 0, mass storage after
/// it, and no CDC-ACM class linked at all. `MSC_EP` sizes the mass-storage bulk pair.
///
/// Its own type rather than a feature on [`DapUsb`], because Cargo features are additive across the
/// whole dependency graph: every platform crate takes this one with default features on, so a
/// default-on `vcom` feature could not be switched off by any firmware. Its own body too, so
/// [`DapUsb`]'s image stays byte-identical.
pub struct DapUsbNoVcom<'a, B: UsbBus, const MSC_EP: usize = 64> {
    usb_dev: UsbDevice<'a, B>,
    dap_v2: CmsisDapV2<'a, B>,
    vendor: VendorDescriptors,
    /// The mass-storage class, allocated last so it never shifts the interfaces above it.
    #[cfg(feature = "flash")]
    msc: MscClass<'a, B, MSC_EP>,
    last_state: UsbDeviceState,
}

impl<'a, B: UsbBus, const MSC_EP: usize> DapUsbNoVcom<'a, B, MSC_EP> {
    /// Build the device from a bus allocator and the probe's USB identity; call once per image.
    pub fn new(alloc: &'a UsbBusAllocator<B>, cfg: &DapConfig<'a>) -> Self {
        let dap_v2 = CmsisDapV2::new(cfg.max_packet_size, alloc);
        // Mass storage takes interface 1, so every interface below it keeps its number.
        #[cfg(feature = "flash")]
        let msc = MscClass::new(alloc);
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
            vendor: VendorDescriptors,
            #[cfg(feature = "flash")]
            msc,
            last_state: UsbDeviceState::Default,
        }
    }

    /// Poll the device across every class, returning whether there was bus activity.
    #[cfg(not(feature = "flash"))]
    pub fn poll(&mut self) -> bool {
        self.usb_dev.poll(&mut [&mut self.dap_v2, &mut self.vendor])
    }

    /// Poll every class; mass storage polls last, so it appends its descriptors after the rest.
    #[cfg(feature = "flash")]
    pub fn poll(&mut self) -> bool {
        self.usb_dev
            .poll(&mut [&mut self.dap_v2, &mut self.vendor, &mut self.msc])
    }

    /// Advance the mass-storage transport over `vfs` as far as the endpoints allow.
    #[cfg(feature = "flash")]
    pub fn msc_service<S, O>(&mut self, vfs: &mut daplink_vfs::Vfs<'_, S, O>)
    where
        S: daplink_vfs::TransferSink,
        O: daplink_vfs::VfsObserver,
    {
        self.msc.service(vfs);
    }

    /// Read a landed command packet into `buf`, returning its length, or `None` when none is pending.
    pub fn read_command(&mut self, buf: &mut [u8]) -> Option<usize> {
        self.dap_v2.read_command(buf)
    }

    /// Write one response packet, returning `false` when it is oversized or the endpoint is busy.
    pub fn write_response(&mut self, data: &[u8]) -> bool {
        self.dap_v2.write(data)
    }

    /// Whether enumeration is over: addressed, descriptors read, and a configuration selected.
    pub fn configured(&self) -> bool {
        self.usb_dev.state() == UsbDeviceState::Configured
    }

    /// Whether the device left `Configured` since the last call; the caller suspends the engine on it.
    pub fn session_ended(&mut self) -> bool {
        let state = self.usb_dev.state();
        let ended = self.last_state == UsbDeviceState::Configured
            && matches!(state, UsbDeviceState::Default | UsbDeviceState::Addressed);
        self.last_state = state;
        ended
    }
}
