# Architecture

Each firmware crate is a composition root for one board or board family. It selects the runtime,
chip support, USB implementation, target-wire backend, optional services, and physical pin map.

```text
firmware composition root
|-- platform integration -> runtime, USB stack, MCU peripherals
`-- reusable mechanisms  -> dap-rs and protocol formats
```

| Layer | Responsibility |
|---|---|
| `firmware/` | Final board wiring, enabled capabilities, and task or interrupt assembly |
| `platform/` | Runtime, USB-stack, MCU-family, and peripheral integration |
| `crates/` | Portable composition, flashing, storage, target, and protocol mechanisms |
| `dap-rs` | CMSIS-DAP command semantics and the generic SWD/JTAG driver contracts |

Dependencies point toward portable mechanisms: firmware depends on platform and reusable crates,
while reusable crates do not select a board. Accelerated wire backends expose narrow register or
pin seams that platform and firmware code complete.

CMSIS-DAP, target flashing, VCOM, SWO, and mass storage share hardware only through explicit
composition and ownership boundaries. Firmware type aliases and constructors make those choices
visible at the composition root; a build feature selects a supported product variant rather than
hiding runtime policy inside a reusable crate.

The [crate reference](reference/crates.md) identifies each workspace. The
[composition design](design/composition.md) explains how a firmware fills these layers with
concrete types and values.
