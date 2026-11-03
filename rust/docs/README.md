# Rust documentation

Start with the [architecture overview](architecture.md) for the layers and dependency direction.

## Design

- [Firmware composition](design/composition.md)
- [Debug transport](design/debug-transport.md)
- [RP PIO wire engines](design/rp-pio-engines.md)
- [Execution models](design/execution-models.md)
- [Streaming interfaces](design/streaming-interfaces.md)
- [USB](design/usb.md)
- [Target flashing](design/target-flashing.md)
- [Verification system](design/verification.md)
- [Hardware-in-the-loop testing](design/hil-testing.md)
- [Performance measurement](design/performance-measurement.md)

## Development

- [Adding a probe board](development/adding-a-board.md)
- [Adding a flash target](development/adding-a-flash-target.md)
- [Adding a wire backend](development/adding-a-wire-backend.md)

## Wire internals

- [LPC55 masked-GPIO SWD](internals/lpc55/masked-swd.md)
- [STM32 SPI-assisted SWD](internals/stm32/spi-assisted-swd.md)

## Reference

- [Building and HIL](reference/building.md)
- [Linux host setup](reference/linux-udev.md)
- [Crate reference](reference/crates.md)
- [DAPLink features not carried forward](reference/daplink-differences.md)

## Boards

- [Raspberry Pi Pico](boards/rp2040-pico.md)
- [Raspberry Pi Pico 2](boards/rp2350-pico2.md)
- [Raspberry Pi Debug Probe](boards/debugprobe.md)
- [STM32F103 boards](boards/stm32f103.md)
- [STM32H523](boards/stm32h523.md)
- [MCU-Link](boards/mculink.md)
- [ST-Link/V2](boards/stlinkv2.md)

Contributor tooling is documented with [xtask](../xtask/README.md) and the
[hardware-in-the-loop test harness](../tests/daplink-test/README.md).
