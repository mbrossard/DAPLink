# daplink-rs

A Rust implementation of a CMSIS-DAP debug probe.

## Experimental status

This branch is an experimental Rust implementation of DAPLink. It was generated using AI coding
agents as part of a broader experiment pursuing several research objectives.

This work has required substantial human input throughout its development. The branch should be
evaluated as experimental software and subjected to the same scrutiny and validation as any
substantial new implementation.

## Layout

- `crates/` -- protocol, flashing and composition libraries: `daplink-core`, `daplink-rs`,
  `daplink-spi-swd`, `daplink-flash`, `daplink-targets`, `daplink-vfs`, `daplink-decoder`,
  `daplink-probe-rs`, `pl011-baud`, `firmware-row`;
- `platform/` -- chip and USB-stack support: `daplink-usb-device`, `daplink-embassy-usb`,
  `daplink-rp-hal`, `daplink-rp-embassy`, `daplink-rp-pio`, `daplink-rp-dma-uart`,
  `daplink-stm32-embassy`, `daplink-stm32f1-hal`, `daplink-stm32f1-embassy`,
  `daplink-stm32f1-spi-swd`, `daplink-lpc55-hal`;
- `firmware/` -- per-board probe images: `rp2040-pico-embassy`, `rp2040-pico-rtic`,
  `rp2350-pico2-embassy`, `debugprobe`, `mculink`, `stm32h523-embassy`, `stm32f103-embassy`,
  `stlinkv2`;
- `tools/` -- host bench tools: `dap-bench`, `swo-bench`, `vcom-bench`, `flash-probe-rs`,
  `probe-reboot`;
- `tests/` -- probe test tool ([daplink-test](tests/daplink-test/README.md)), its DUT images
  (`test-rp2040`, `test-rp2350`, `test-stm32f401`) and support crates;
- `xtask/` -- build orchestration ([xtask](xtask/README.md));
- `docs/` -- architecture and board documentation ([documentation index](docs/README.md)).

Each crate is its own Cargo workspace. Build a firmware from its own directory with
`cargo build --release --locked`; its `.cargo/config.toml` selects the embedded target. See
[building and HIL](docs/reference/building.md) for prerequisites, registered builds, flashing,
and hardware-in-the-loop testing.
