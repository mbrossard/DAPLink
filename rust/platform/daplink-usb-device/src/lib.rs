//! Polled (`usb-device`) USB adapter for daplink-rs probe firmware.

#![no_std]

#[cfg(test)]
extern crate std;

mod cdc;
mod dap_class;
pub mod dap_pass;
mod dap_usb;
mod msc;
mod product;
mod winusb;

pub use dap_class::{single_transfer_packet, CmsisDapV2};
pub use dap_pass::{dap_pass, DapPassHooks, DapTransport};
pub use dap_usb::{DapUsb, DapUsbMsc, DapUsbNoVcom, DapUsbNoVcomMsc};
pub use msc::MscClass;
pub use product::{MassStorage, NoMsc};
pub use usb_device::endpoint::EndpointAddress;
pub use winusb::{EndpointRepair, RegisterCount, VendorDescriptors};

/// The `usb-device` bus allocator, re-exported so firmware can name it without its own dependency.
pub use usb_device::bus::UsbBusAllocator;
