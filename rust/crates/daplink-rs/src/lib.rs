//! Composition layer for the daplink-rs debug probe, built on the dap-rs CMSIS-DAP engine.

#![no_std]

pub use daplink_core;
pub use daplink_core::vendor::{
    NoVendor, VendorChain, VendorHandler, VENDOR_CMD_FIRST, VENDOR_CMD_LAST,
};

mod composed_jtag;
mod dap_probe;
mod flash_backend;
pub mod flash_mux;
#[cfg(feature = "guard")]
mod guard;
mod late_jtag;
mod leds;
mod ram_text;
mod reboot;
mod ring;
mod spin;
mod swd_mux;
mod swd_only;
mod swo;
mod vcom_baud;
mod vcom_stats;
mod wired;

pub use composed_jtag::{ComposedJtagDeps, ComposedJtagDrv, ComposedJtagSwd, SwjPadLoan};
pub use dap_probe::{BitbangDap, ComposedJtagDap, DapProbe, SwdOnlyDap};
pub use daplink_core::dma_ring::{DmaRingCursor, DmaRxChannel, DmaRxRing};
pub use flash_backend::HwBackend;
pub use late_jtag::{
    Deps as LateJtagDeps, JtagLines, LateJtag, LateJtagShifter, SwdDrv as LateSwdDrv,
};
pub use leds::{noop_activity, Activity, BankLed, NoopLeds, StatusLed};
pub use reboot::{
    BootloaderEntry, FlashBusy, NeverBusy, NoBootloader, RebootVendor, KIND_NONE, KIND_NXP_ISP,
    KIND_ST_DFU, KIND_UF2, REBOOT_CMD_ID, STATUS_ARMED, STATUS_BAD_MAGIC, STATUS_BUSY,
    STATUS_UNSUPPORTED,
};
pub use ring::{drain_fifo, ByteConsumer, ByteProducer, ByteRing, FIFO_DRAIN_BUDGET};
pub use spin::{spin_cycles, PreparedSpin, CYCLES_PER_ITERATION};
pub use swd_mux::{DapDeps, DapJtag, DapSwd, FlashSession, FlashSide, SwdMux};
pub use swd_only::{Deps as SwdOnlyDeps, SwdDrv as SwdOnlySwdDrv};
pub use swo::{RingSwo, SwoBackend, SwoSource};
pub use vcom_baud::{
    VcomBaudState, VcomBaudVendor, VCOM_BAUD_APPLIED_VALID, VCOM_BAUD_CMD_ID,
    VCOM_BAUD_RESPONSE_LEN, VCOM_BAUD_SETTLED, VCOM_BAUD_V1_RESPONSE_LEN,
};
pub use vcom_stats::{
    vcom_episodes, vcom_grade, VcomCounters, VcomStatsVendor, VCOM_GRADE_EXACT,
    VCOM_GRADE_LOWER_BOUND, VCOM_SLOT_CDC_IN, VCOM_SLOT_CDC_OUT, VCOM_SLOT_FRAME_ERR,
    VCOM_SLOT_HOST_DROPPED, VCOM_SLOT_OVERRUN, VCOM_SLOT_RING_FULL, VCOM_SLOT_RX_DMA_ERR,
    VCOM_SLOT_UART_RX, VCOM_SLOT_UART_TX, VCOM_STATS_CMD_ID, VCOM_STATS_FIELDS,
    VCOM_STATS_RESPONSE_LEN, VCOM_VALID_ALL, VCOM_VALID_CDC_IN, VCOM_VALID_CDC_OUT,
    VCOM_VALID_FRAME_ERR, VCOM_VALID_HOST_DROPPED, VCOM_VALID_IDENTITY, VCOM_VALID_OVERRUN,
    VCOM_VALID_RING_FULL, VCOM_VALID_RX_DMA_ERR, VCOM_VALID_UART_RX, VCOM_VALID_UART_TX,
};
pub use wired::{wired_deps, wired_swd_deps, ScanChainStorage};

pub use dap_rs;
pub use dap_rs::driver::bitbang::{
    BitbangAdapter, DelayCycles, InputOutputPin, JtagAdapter, SwdAdapter,
};
pub use dap_rs::swo::{NoSwo, Swo};
pub use embedded_hal::delay::DelayNs;
