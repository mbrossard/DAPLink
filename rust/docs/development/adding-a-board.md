# Adding a probe board

A probe board is a [firmware composition](../design/composition.md), not a new portable
abstraction. Start from the existing firmware with the closest MCU family, runtime, USB controller,
and debug wiring. Reuse its platform crate unless the MCU requires a genuinely different hardware
seam.

## Define the product

Before writing code, record these choices:

- MCU, clock tree, memory map, and embedded target triple;
- SWD, JTAG, nRESET, SWO, VCOM, status LED, and USB pins;
- Embassy or RTIC [execution model](../design/execution-models.md);
- bit-bang or accelerated wire backend;
- supported optional variants, such as target flashing or diagnostics;
- USB identity and whether interface numbering must remain compatible with another image.

Add `docs/boards/<board>.md` with the physical pinout and user-visible limitations. Keep register
mechanics in the platform crate or an `internals/` page rather than the board page.

## Add the firmware composition

Create `firmware/<board>/` with its manifest, linker inputs, build script, and composition root.
The composition root should:

1. initialize clocks and peripherals through the selected platform crate;
2. construct the concrete wire, reset, SWO, VCOM, LED, and vendor-command roles;
3. use explicit `NoSwo`, `NoVendor`, or `NoopLeds` roles for absent capabilities;
4. build the identity every image publishes: `DapConfig::daplink(serial, endpoint)`
   `.with_product(product_string!("<board>"))`, taking `endpoint` from the firmware's own queue
   type (`Queue::endpoint_size::<64>()`, so an endpoint larger than a slot fails the build), then
   `Identity { vendor, product, serial, board_vendor, board_name }` before
   constructing `DapProbe` with it. Name the board itself (`stm32h523`, `rp2350-pico2`), adding a
   runtime suffix only where two images share one board (`rp2040-pico`, `rp2040-pico-rtic`).
   `DAP_Info` answers these strings back to the host;
5. construct the [USB interfaces](../design/usb.md) in their intended descriptor order;
6. run the runtime-specific USB and service tasks.

Publish identity storage exactly once during boot, before handing references to the engine or
USB stack; the static identity helpers do not guard against republication.

If the closest platform lacks the required wire mechanism, follow
[adding a wire backend](adding-a-wire-backend.md) rather than duplicating packet handling in
the root. Preserve the [queue/session](../design/execution-models.md) and
[UART completion](../design/streaming-interfaces.md) obligations when adding service tasks.

## Register and verify it

The registry is a family plus its variants, not one flat row per build. Define one
`FirmwareFamily` for the crate -- its path, target triple, probe-rs chip, default HIL profile,
UF2 family, and bootloader -- and list each buildable shape as a `FirmwareVariant` under it:

```rust
FAMILY.variant(FirmwareVariant::new("firmware/<name>").features(&["some-feature"]))
```

in `xtask/src/registry.rs::FIRMWARE`. A variant's `name` is also its build directory, derived
automatically; give every variant of a family a distinct name so their builds do not evict one
another.

Add a rig profile for the board in `tests/daplink-test/src/profile.rs`. If the board is also a
wired device under test, add a row to the `TARGETS` table in `registry.rs` and matching entries in
the harness's target table (`tests/daplink-test/src/target.rs`).

Run `cargo xtask build-all`, then `cargo xtask hil` on the wired rig, as described in
[building and HIL](../reference/building.md). Before making a new backend or feature the default,
run HIL on both the new row and the nearest unaffected rows.
