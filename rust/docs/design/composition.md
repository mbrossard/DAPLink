# Firmware composition

A firmware crate is the only layer that knows the complete product: board wiring, MCU, runtime,
USB stack, wire backend, services, and target set. It assembles those choices rather than asking a
reusable crate to discover or select them.

The CMSIS-DAP engine is a generic `DapProbe` whose type roles supply dependencies, LEDs, delay,
JTAG, SWD, SWO, and vendor commands. Aliases such as `BitbangDap`, `SwdOnlyDap`, and
`ComposedJtagDap` name recurring combinations. Explicit inert types (`NoSwo`, `NoVendor`, and
`NoopLeds`) represent capabilities a product does not provide.

Composition proceeds in three stages:

1. A platform constructor claims peripherals and returns owned parts for the selected board.
2. The firmware supplies its pin map and chooses concrete engine and service types.
3. The selected USB loop owns the engine and routes complete command packets to it.

Generic types carry compile-time ownership and remove unused roles. Constructor arguments inject
values that depend on a board or build, such as clocks, pin maps, target tables, and queue depth.
Cargo features include optional code and select complete firmware variants such as accelerated or
flash-enabled builds; they are not runtime policy switches inside a reusable mechanism.

When services need the same physical target link, composition adds an explicit owner. Flash builds,
for example, place the adapter behind `SwdMux`; CMSIS-DAP receives a gated role while a flash
session receives exclusive access. A build without flashing has no mux or flash front end.
