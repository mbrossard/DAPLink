//! lpc55-hal platform support for polled (`usb-device`) LPC55S69 daplink-rs probes.

#![no_std]

mod baud;
mod clocks;
mod dma;
mod dma_rx;
mod dma_swo;
mod dma_tx;
mod init;
#[cfg(feature = "masked-swd")]
mod masked;
mod pin;
mod power;
mod reboot;
mod swo;
mod uid;
mod usb_phy;
mod vcom;

pub use clocks::ClockProfile;
pub use daplink_rs::DynCycleDelay;
pub use dma_swo::swo_dma_service_irq;
pub use init::{probe_parts, Bus, ProbeDeps, ProbeParts};
#[cfg(feature = "masked-swd")]
pub use masked::{
    masked_deps, verify_fit, MaskedBlock, MaskedDeps, MaskedJtagDrv, MaskedPacing, MaskedSwdDap,
    MaskedSwdDrv,
};
// Re-exported so a firmware's pacing type needs no direct dependency on the shared crate.
#[cfg(feature = "masked-swd")]
pub use daplink_spi_swd::{nops, JtagPacing};
pub use pin::IoPin;
pub use reboot::{reboot_armed, Lpc55Boot};
pub use swo::Fc3Swo;
pub use vcom::{
    observe_rx_errors, vcom_baud_state, vcom_rx_discards, vcom_rx_dma_errors, vcom_service_irq,
    vcom_tx_sent, vcom_tx_service_irq, Vcom, VcomRxDiscards,
};
