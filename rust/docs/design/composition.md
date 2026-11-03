# Firmware composition

A firmware crate is the only layer that knows the complete product: board wiring, MCU, runtime,
USB stack, wire backend, services, and target set. Reusable crates never select these.

The CMSIS-DAP engine is a generic `DapProbe` whose type roles supply dependencies, LEDs, delay,
JTAG, SWD, SWO, and vendor commands. Aliases such as `BitbangDap` and `SwdOnlyDap` name
recurring combinations; `NoSwo`, `NoVendor`, and `NoopLeds` stand for absent capabilities.

Composition proceeds in three stages:

1. A platform constructor claims peripherals and returns owned parts for the selected board.
2. The firmware supplies its pin map and chooses concrete engine and service types.
3. The selected USB loop owns the engine and routes complete command packets to it.

Board- or build-dependent values (clocks, pin maps, target tables, queue depth) are constructor
arguments. Cargo features select complete firmware variants; reusable crates do not mirror product
variants as features, because Cargo unifies features across consumers. The exception is an MCU or
core selection a PAC or HAL requires.

When services share the target link, composition adds an explicit owner: flash builds place the
adapter behind `SwdMux`, giving CMSIS-DAP a gated role and a flash session exclusive access.
