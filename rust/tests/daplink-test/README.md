# daplink-test

The probe test tool: named cases run from this directory against a probe wired to a live
target:

```text
cargo run -- --probe <vid:pid:serial> --target <name> [case ...]
```

`--probe` selects the probe under test (full selector; many probes share a VID:PID) and
`--target` names the wired target from the harness's own table (e.g. `stm32f401`).
`--dut-elf` supplies the image the `flash` case programs; without it that case is left out of
the default run. `--dut-uf2` supplies the image the `vendor-flash` case streams; without it that
case is left out of the default run. Naming no cases runs the default set; the exit code is
nonzero if any case fails.

`--profile <name>` names the rig wiring in use. A case whose requirement is unmet by profile ∩
target prints `skip (<requirement>)` or `skip (<requirement>: target)` instead of running and
never fails the run. Without `--profile` no case is gated. `--disable-caps <list>` withholds
abilities beyond that intersection, for a bench where a normally-supported ability is unwired
or temporarily broken; such a case prints `skip (<requirement>: disabled)`.

`vendor-flash` is not profile-gated: its first command's answer is its own capability probe, and
a firmware built without the vendor handler prints `skip (vendor-flash: firmware)`. `msc` is not
profile-gated either: a firmware built without flash support carries no mass-storage interface at
all, and that absence prints `skip (msc: firmware)`.

| Case | What it proves |
|---|---|
| `info` | the probe answers `DAP_Info` correctly over raw CMSIS-DAP v2 bulk |
| `connect` | attach through the probe and reach the target core |
| `swd` | halt, pattern the top of target RAM over SWD, restore the backup, resume the DUT |
| `flash` | program `--dut-elf` through the probe, verify the readback, reset the target |
| `rtt` | read the boot event left by the flash, then a protocol PING exchange over RTT |
| `nreset` | stamp DUT RAM, pulse the reset line over raw bulk, require a boot event and the stamp zeroed |
| `vcom` | echo a pattern through the probe's USB serial port and the DUT's UART, require it back |
| `jtag` | the same RAM roundtrip as `swd`, attached over JTAG instead |
| `vendor-flash` | stream `--dut-uf2` through the probe's vendor flash commands over raw bulk, verify the readback |
| `msc` | drop `--dut-uf2` onto the probe's mass-storage volume over raw SCSI/BOT, verify the readback (unavailable on macOS hosts) |
| `speed-swd` | block-read throughput over SWD, time-windowed per requested clock; reports, never judges |
| `speed-jtag` | the same measurement over a JTAG session |

Cases are ordered so each adds one layer over the previous. A `FAILED` case records and the
run continues; a `FATAL` one means the substrate later cases depend on is gone, and the run
stops there.

The target is halted and reset by several cases -- do not point this at a target whose state
matters.
