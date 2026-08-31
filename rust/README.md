# daplink-rs

A Rust implementation of a CMSIS-DAP debug probe.

## Experimental status

This branch is an experimental Rust implementation of DAPLink. It was generated using AI coding
agents as part of a broader experiment pursuing several research objectives.

This work has required substantial human input throughout its development. The branch should be
evaluated as experimental software and subjected to the same scrutiny and validation as any
substantial new implementation.

- `crates/` -- protocol and composition libraries (`daplink-core`, `daplink-rs`);
- `platform/` -- HAL and USB-stack support crates (`daplink-rp-hal`, `daplink-usb-device`,
  `daplink-rp-embassy`, `daplink-embassy-usb`);
- `firmware/` -- per-board probe images;
- `tests/` -- probe test tool ([daplink-test](tests/daplink-test/README.md)), its DUT images,
  and the shared test-protocol crate;
- `xtask/` -- build orchestration ([xtask](xtask/README.md));
- `docs/` -- architecture and board documentation ([documentation index](docs/README.md)).

Each crate is its own Cargo workspace. Build a firmware from its own directory with
`cargo build --release --locked`; its `.cargo/config.toml` selects the embedded target. See
[building and HIL](docs/reference/building.md) for prerequisites, registered builds, flashing,
and hardware-in-the-loop testing.
