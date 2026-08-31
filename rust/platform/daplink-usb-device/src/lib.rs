//! Polled (`usb-device`) USB adapter for daplink-rs probe firmware.

#![no_std]

#[cfg(test)]
extern crate std;

mod cdc;
mod dap_class;
mod dap_usb;
#[cfg(feature = "flash")]
mod msc;
mod winusb;

pub use dap_class::CmsisDapV2;
pub use dap_usb::{DapUsb, DapUsbNoVcom};
#[cfg(feature = "flash")]
pub use msc::MscClass;
pub use winusb::VendorDescriptors;

/// The `usb-device` bus allocator, re-exported so firmware can name it without its own dependency.
pub use usb_device::bus::UsbBusAllocator;
