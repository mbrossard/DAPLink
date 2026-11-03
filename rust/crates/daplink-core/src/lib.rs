//! Zero-dependency core: USB identity, DAP queue and wire framing, DMA ring cursors, the flash-sink seam.

#![no_std]

pub mod dap_queue;
pub mod dap_wire;
pub mod dma_ring;
pub mod dma_tx_cursor;
pub mod flash_sink;
pub mod jtag_dp;
#[cfg(any(test, feature = "bot-fixtures"))]
pub mod msc_bot_cases;
pub mod msc_proto;
pub mod probe;
pub mod range;
pub mod rp_buffer_control;
pub mod serial;
pub mod stm32_epr;
pub mod usb_reconnect;
pub mod vendor;
pub mod winusb;

pub use dap_queue::{
    CommandExecutor, DapQueue, EndpointSize, PacketSize, QueueError, RequestProducer,
    ResponseConsumer, Submit, DAP_TRANSFER_ABORT,
};
pub use dma_ring::{DmaRingCursor, DmaRxChannel, DmaRxRing};
pub use dma_tx_cursor::{Accepted, DmaTxCursor};
pub use flash_sink::{FlashBackend, FlashPhase, FlashPoll, TargetWindows};
pub use jtag_dp::{JtagDp, JtagDpSource};
pub use probe::{FlashArbiter, NoArbiter, ProbeInfo, ResetControl};
pub use serial::{format_serial, SERIAL_LEN};
pub use usb_reconnect::{NoUsbReconnect, UsbReconnect};
pub use vendor::{NoVendor, VendorHandler, VENDOR_CMD_FIRST, VENDOR_CMD_LAST};

/// Canonical Arm DAPLink USB vendor id -- host tools recognize the probe as a DAPLink.
pub const DAPLINK_VID: u16 = 0x0D28;
/// Canonical Arm DAPLink USB product id.
pub const DAPLINK_PID: u16 = 0x0204;

/// CMSIS-DAP protocol version: a dotted `x.y.z` triple hosts parse to gate features.
pub const DAP_PROTOCOL_VERSION: &str = "2.1.1";

/// `DAP_Info` `0x04` firmware version: the protocol triple plus a fourth revision field hosts ignore.
/// A dot, not a space: pyOCD integer-parses three dot fields, and a space demotes it to 2.0.0.
pub const DAP_FW_VERSION: &str = concat!("2.1.1", ".", env!("DAPLINK_FW_REVISION"));

/// The source-revision token inside [`DAP_FW_VERSION`]: short commit hash, `-dirty`, or `unknown`.
pub const DAP_FW_REVISION: &str = env!("DAPLINK_FW_REVISION");

/// `DAP_Info` `0x09` product firmware version: the firmware registry row, or `unknown`.
pub const DAP_FW_ROW: &str = env!("DAPLINK_FW_ROW");

/// CMSIS-DAP v2 bulk packet size, in bytes (full-speed USB).
pub const DAP_PACKET_SIZE: u16 = 64;

/// CMSIS-DAP v2 bulk packet size, in bytes, on a high-speed bus (USB mandates 512 there).
pub const DAP_HS_PACKET_SIZE: u16 = 512;

/// FAT/MSC logical sector size, in bytes.
pub const MSC_SECTOR_SIZE: usize = 512;

/// Logical sectors the virtual MSC volume advertises; oversized so hosts never see it full.
pub const VFS_TOTAL_SECTORS: u32 = 131_200;

/// Mass-storage interface triple: mass storage, SCSI transparent command set, Bulk-Only Transport.
pub const MSC_INTERFACE_CLASS: (u8, u8, u8) = (0x08, 0x06, 0x50);
/// Bulk-Only class request: GET MAX LUN (the device answers the highest LUN number).
pub const MSC_GET_MAX_LUN: u8 = 0xFE;
/// Bulk-Only class request: Bulk-Only Mass Storage Reset.
pub const MSC_BULK_ONLY_RESET: u8 = 0xFF;
/// SCSI INQUIRY vendor id (exactly 8 bytes, space-padded).
pub const SCSI_VENDOR_ID: &[u8; 8] = b"DAPLink ";
/// SCSI INQUIRY product id (exactly 16 bytes, space-padded).
pub const SCSI_PRODUCT_ID: &[u8; 16] = b"daplink-rs VFS  ";
/// SCSI INQUIRY product revision (exactly 4 bytes).
pub const SCSI_REVISION_ID: &[u8; 4] = b"1.0 ";

/// CDC-ACM bulk packet size, in bytes (full-speed USB).
pub const CDC_PACKET_SIZE: u16 = 64;

/// CDC-ACM bulk packet size, in bytes, on a high-speed bus (USB mandates 512 there).
pub const CDC_HS_PACKET_SIZE: u16 = 512;

/// Default VCOM baud until a host sets CDC line coding.
pub const DEFAULT_VCOM_BAUD: u32 = 115_200;

/// Vendor-specific class triple for the CMSIS-DAP v2 bulk interface.
pub const DAP_V2_INTERFACE_CLASS: (u8, u8, u8) = (0xFF, 0x00, 0x00);

/// USB configuration `bMaxPower`, in milliamps: the conventional bus-powered debug-probe budget.
pub const USB_MAX_POWER_MA: u16 = 100;

/// JTAG scan-chain slots; the configure bound is exclusive, so chains of up to seven TAPs.
pub const SCAN_CHAIN_SLOTS: usize = 8;

/// Expands to `daplink-rs CMSIS-DAP (<name>)`; keep the `CMSIS-DAP` substring so host matchers find it.
///
/// ```
/// # use daplink_core::{product_string, DapConfig, DapQueue, EndpointSize};
/// const EP: EndpointSize = DapQueue::<4, 64>::endpoint_size::<64>();
/// const CFG: DapConfig = DapConfig::daplink("5BCCF132445D5C49", EP)
///     .with_product(product_string!("rp2040"));
/// assert_eq!(CFG.product, "daplink-rs CMSIS-DAP (rp2040)");
/// ```
#[macro_export]
macro_rules! product_string {
    ($name:expr) => {
        concat!("daplink-rs CMSIS-DAP (", $name, ")")
    };
}

/// The identity strings a host reads back over `DAP_Info`; an empty field answers length 0.
#[derive(Clone, Copy)]
pub struct Identity {
    /// `DAP_Info` `0x01`: the probe vendor.
    pub vendor: &'static str,
    /// `DAP_Info` `0x02`: the product string, matching the USB descriptor.
    pub product: &'static str,
    /// `DAP_Info` `0x03`: the serial number, matching the USB descriptor.
    pub serial: &'static str,
    /// `DAP_Info` `0x07`: the board vendor.
    pub board_vendor: &'static str,
    /// `DAP_Info` `0x08`: the board name.
    pub board_name: &'static str,
}

impl Identity {
    /// Every field absent, so every identity id answers length 0.
    pub const EMPTY: Self = Self {
        vendor: "",
        product: "",
        serial: "",
        board_vendor: "",
        board_name: "",
    };
}

/// USB device identity and endpoint sizing for a CMSIS-DAP probe; firmware owns the string storage.
#[derive(Clone, Copy)]
pub struct DapConfig<'a> {
    /// USB vendor id.
    pub vid: u16,
    /// USB product id.
    pub pid: u16,
    /// Manufacturer string.
    pub manufacturer: &'a str,
    /// Product string; keep the `CMSIS-DAP` substring (see [`product_string!`]).
    pub product: &'a str,
    /// Serial-number string, built with [`serial::format_serial`].
    pub serial: &'a str,
    /// Max packet size for the DAP bulk endpoints, checked against the queue slot.
    pub max_packet_size: EndpointSize,
    /// Control-endpoint (EP0) max packet size.
    pub max_packet_size_0: u8,
}

impl<'a> DapConfig<'a> {
    /// The standard daplink-rs identity: canonical VID/PID, `daplink-rs` manufacturer.
    pub const fn daplink(serial: &'a str, max_packet_size: EndpointSize) -> Self {
        Self {
            vid: DAPLINK_VID,
            pid: DAPLINK_PID,
            manufacturer: "daplink-rs",
            product: "daplink-rs CMSIS-DAP",
            serial,
            max_packet_size,
            max_packet_size_0: 64,
        }
    }

    /// The same identity with a caller-chosen product string; stays `const`.
    pub const fn with_product(self, product: &'a str) -> Self {
        Self { product, ..self }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// frozen: the class triples host driver binding keys on.
    #[test]
    fn interface_class_triples_are_pinned() {
        assert_eq!(DAP_V2_INTERFACE_CLASS, (0xFF, 0x00, 0x00));
        assert_eq!(MSC_INTERFACE_CLASS, (0x08, 0x06, 0x50));
    }
}
