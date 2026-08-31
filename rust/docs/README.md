# Rust documentation

Start with the [architecture overview](architecture.md) for the layers and dependency direction.

## Design

- [Firmware composition](design/composition.md)
- [Debug transport](design/debug-transport.md)
- [Execution models](design/execution-models.md)
- [Streaming interfaces](design/streaming-interfaces.md)
- [USB](design/usb.md)
- [Target flashing](design/target-flashing.md)

## Reference

- [Building and HIL](reference/building.md)
- [Crate reference](reference/crates.md)

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
