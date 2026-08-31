# Crate reference

Each directory below is an independent Cargo workspace. Firmware crates depend on reusable and
platform crates; reusable crates never select a board.

| Crate | Responsibility |
|---|---|
| `daplink-core` | Shared identity, configuration, queue, vendor, and service contracts |
| `daplink-rs` | CMSIS-DAP engine composition, wire roles, arbitration, SWO, and reboot support |
| `daplink-decoder` | Streaming firmware-image detection and UF2 decoding |
| `daplink-flash` | Target-side flash-algorithm runtime over the `dap-rs` SWD contract |
| `daplink-targets` | Generated algorithms, target records, and target selection |
| `daplink-vfs` | Virtual FAT16 volume, transfer engine, MSC protocol, and vendor flashing |
| `daplink-spi-swd` | Runtime-independent SPI-assisted SWD and direct-GPIO JTAG |
| `daplink-probe-rs` | Host-only adapter from probe-rs raw DAP access to `daplink-flash` |
| `daplink-embassy-usb` | Async CMSIS-DAP, CDC-ACM, and optional MSC USB classes |
| `daplink-usb-device` | Polled CMSIS-DAP, CDC-ACM, and optional MSC USB classes |
| `daplink-rp-embassy` | Embassy RP clocks, pins, serial services, LEDs, and reboot integration |
| `daplink-rp-hal` | rp2040-hal equivalents for the RTIC firmware |
| `daplink-rp-pio` | Runtime-independent RP2040/RP235x PIO SWD backend |
| `daplink-stm32-embassy` | Embassy STM32H5-class platform integration |
| `daplink-stm32f1-embassy` | Embassy STM32F1 platform integration |
| `daplink-stm32f1-hal` | stm32f1xx-hal platform integration for polled firmware |
| `daplink-lpc55-hal` | lpc55-hal platform integration for polled LPC55S69 firmware |

Firmware workspaces under `firmware/` own final board composition. Host tools and test firmware
under `tools/` and `tests/` consume these crates but are not part of a probe image.
