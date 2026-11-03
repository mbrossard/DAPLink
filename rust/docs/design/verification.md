# Verification system

| Layer | What it covers |
|---|---|
| Host tests | Codecs, state machines, ring arithmetic and generated data |
| [Hardware-in-the-loop](hil-testing.md) | Workflows through a complete probe, USB and wired target |
| [Measurement tools](performance-measurement.md) | Workload-specific throughput and loss evidence |

`cargo xtask test` first refuses any tracked lockfile that no longer matches its manifests, then
runs formatting, lints, library docs and tests over each host-buildable workspace, and keeps going
past a failing one. Firmware roots need their embedded targets and are covered by builds, which
are `--locked`: to re-resolve a dependency on purpose, run cargo in that workspace first. Library
crates keep no tracked lockfile, so their host rows resolve on each machine.

`cargo xtask build-all` also fails a row whose deepest resolved call path exceeds its free RAM, or
that it cannot measure. The figure is a lower bound: unresolved calls are dropped, so passing proves
nothing.

Keep testable logic below register access where useful: DMA cursor arithmetic, byte rings and SWO
session logic run without hardware. Test against literal expected results, not the
implementation's own helpers. Some host tests touch USB devices; check a test's effects before
running it.

A hardware run has three roles: the host drives the probe under test over USB, and that probe
drives the device under test over SWD, JTAG, reset, UART and SWO.

```text
transport probe ----> probe under test ----> device under test
                          ^
                          |
                         USB
                          |
                         host
```

An optional transport probe attaches to the probe under test but never carries a case's traffic:
it installs firmware before a run and can capture the probe's own log during one.

HIL runs through `cargo xtask hil` and each measurement tool is its own binary; see the
[build reference](../reference/building.md) and the
[harness reference](../../tests/daplink-test/README.md).
