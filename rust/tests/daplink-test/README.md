# daplink-test

The probe test tool: named cases run from this directory against a probe wired to a live
target:

```text
cargo run -- --probe <vid:pid:serial> --target <name> [case ...]
```

`--probe` selects the probe under test (full selector; many probes share a VID:PID) and
`--target` names the wired target from the harness's own table (e.g. `stm32f401`).
`--dut-elf` supplies the image the `flash` case programs, and the RTT symbol every
DUT-dependent case attaches through; without it those cases are left out of the default run.
`--dut-uf2` supplies the image the drop-style cases stream; without it those cases are left out
of the default run. Naming no cases runs the default set; the exit code is
nonzero if any case fails.

`--profile <name>` names the rig wiring in use. A case whose requirement is unmet by profile and
target prints `skip (<requirement>)` or `skip (<requirement>: target)` instead of running and
never fails the run. Without `--profile` no case is gated. `--disable-caps <list>` withholds
abilities beyond that intersection, for a bench where a normally-supported ability is unwired
or temporarily broken; such a case prints `skip (<requirement>: disabled)`.

A recording run appends to one ledger, `$XDG_STATE_HOME/daplink/hil-runs.jsonl` or
`~/.local/state/daplink/hil-runs.jsonl`, wherever the binary was built. `--record <path>` writes a
second copy, `--no-record` writes neither, and a run that cannot write its first line stops there.
That line is `"event":"start"` with the `selected` case names: a selected case with no record and no
`fatal` means the run died. Case records carry `mode` (`full` or `list`), `series` (`--series
<token>`), `load1` at case start, and, when a case noted any, `waits` with each deadline's budget and
worst use.

Every record carries `firmware_row` and `firmware_row_source`: the row the image answers on
`DAP_Info` `0x09` (`image`), else `--firmware-row <row>` (`operator`), else null (`none`).
`cargo xtask hil` passes the row it was given. A disagreement, a value outside the firmware registry
or no row prints a warning, and the run goes on. `external:<name>` names an image outside the
registry.

`vendor-flash` is not profile-gated: its first command's answer is its own capability probe, and
a firmware built without the vendor handler prints `skip (vendor-flash: firmware)`. `msc` is not
profile-gated either: a firmware built without flash support carries no mass-storage interface at
all, and that absence prints `skip (msc: firmware)`.

`reattach-suspend` (Linux only) suspends the probe through sysfs before resetting it; where that
attribute is not writable it prints a warning and `skip`, and
[Linux host setup](../../docs/reference/linux-udev.md) gives the rule that lets it run.

Before the first case, the harness asks the DUT who it is over the test protocol (`ID`) and
prints the reply as the header's `dut` line. A DUT that cannot answer prints
`dut identity unavailable` and the run continues.

| Case | What it proves |
|---|---|
| `identity` | the probe's enumeration identity on the bus and its vendor descriptors over control |
| `info` | the probe answers `DAP_Info` correctly over raw CMSIS-DAP v2 bulk |
| `pipeline` | a burst past both of the probe's rings comes back in issue order with nothing lost or corrupted, a further request is refused or delayed rather than dropped, and `DAP_Disconnect` leaves nothing stale |
| `reattach` | after a USB reset of the probe, it re-enumerates and answers `DAP_Info` within five seconds, and not later |
| `reattach-suspend` | idle the probe into a real USB runtime suspend, reset it, require the first reply to answer the request just sent rather than one queued before the suspend (Linux only; skips on a mass-storage row, whose runtime-PM-forbidden SCSI disk keeps the probe awake) |
| `halt` | on every bulk endpoint, a halt clear completes within a second, a host halt holds until cleared, and traffic resumes with the data toggle reset, at both toggle parities; a reply armed before a halt is dropped and one armed before a clear is delivered once or not at all, at both parities; the control pipe and every absent twin, in both directions, answer a clear and a status request without wedging EP0; ends with a USB reset (MSC unavailable on macOS hosts) |
| `connect` | attach through the probe and reach the target core |
| `swd` | halt, pattern the top of target RAM over SWD, restore the backup, resume the DUT |
| `fault` | a read at an address the target does not map faults instead of returning data, the sticky flags clear on an `ABORT` write, and DPIDR and ordinary reads work again |
| `flash` | program `--dut-elf` through the probe, verify the readback, reset the target |
| `rtt` | read the boot event left by a preceding `flash`, or by its own reset, then a protocol PING exchange over RTT |
| `reset` | stamp DUT RAM, reset the core through the session, require a boot event and the stamp zeroed |
| `nreset` | stamp DUT RAM, pulse the reset line over raw bulk, require a boot event and the stamp zeroed |
| `vcom` | echo a pattern through the probe's USB serial port and the DUT's UART, require it back |
| `vcom-load` | the same echo at 921600 only, with a `read_32` loop running on the session throughout |
| `swo` | the DUT's ITM emits on port 0 and the host captures the framed payload back, at four ITM bauds |
| `swo-load` | the same capture at one ITM rate, with a `read_32` loop running on the session throughout |
| `jtag` | the same RAM roundtrip as `swd`, attached over JTAG instead |
| `jtag-chain` | read the whole chain's IDCODEs with one raw DR scan, require `DAP_JTAG_IDCODE` per index to agree with it, and require exactly one debug port among distinct IDCODEs |
| `vendor-flash` | stream `--dut-uf2` through the probe's vendor flash commands over raw bulk, verify the readback |
| `msc` | drop `--dut-uf2` onto the probe's mass-storage volume over raw SCSI/BOT, verify the readback (unavailable on macOS hosts) |
| `msc-corrupt` | drop a uf2 with one block's magic destroyed, require the wire released promptly, `DETAILS.TXT` to report the failure and every byte of the image range to be unchanged but the refused block's erased sector tail, then put the good image back and leave the DUT running it (unavailable on macOS hosts) |
| `speed-swd` | block-read throughput over SWD, time-windowed per requested clock (1, 4, 12, 25 MHz); reports, never judges |
| `speed-jtag` | the same measurement over a JTAG session |

`--speed-access <bytes>` (comma-separated or repeated, multiples of 4 up to 4096, default 4096)
sets the bytes per read or write call, each size swept in turn. `--speed-extended` sweeps 1 to
50 MHz instead; each step halts the core, times writes, then reads checked against a pattern in the
top 4 KiB of target RAM, then restores that RAM. A step the probe or target cannot run prints
`FAILED` and the sweep carries on; the case fails when no step moved data or a step that moved
data read back a mismatch.

A `FAILED` case records and the run continues; a `FATAL` one stops the run.

The target is halted and reset by several cases -- do not point this at a target whose state
matters.

DUT-side state outside the test protocol is asserted by whoever needs it, never assumed: a case
that depends on it (`swo` routes the trace pin, `jtag` unroutes it) writes it before use rather
than trusting the previous case, and a DUT image reasserts at boot whatever its own reset does
not restore.

## Connectivity scan

Build and install `stlinkv2` and `test-stm32f401` with `--features conntest`.
Keep their matching ELFs; the probe retains USB SWD but replaces JTAG/SWO with GPIO scanning.
A separate via probe must reach the ST-Link/V2's own SWD pads.

```text
cargo run -- --probe <PUT-selector> --via <via-selector> \
  --probe-elf <ST-Link-diagnostic-ELF> --dut-elf <F401-diagnostic-ELF> \
  --target stm32f401 --profile stlinkv2-conntest scan
```

This standalone case tests PA7 -> PA15 and PB3 -> PA6/PA10 with open-drain low/release
transitions. It neither flashes nor resets either board. Exit 0 means all expected edges
followed, 1 a wiring mismatch, and 2 incomplete execution. Restore both normal images after
testing, and do not run another debug client concurrently.
