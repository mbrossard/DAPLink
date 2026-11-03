# RP PIO wire engines

`daplink-rp-pio` supplies runtime-independent SWD/JTAG engines. Firmware reserves the PIO
blocks, state machines, instruction memory and pads before calling the raw constructors.

## Source map

| File | Role |
|---|---|
| [`pins.rs`](../../platform/daplink-rp-pio/src/pins.rs), [`proto.rs`](../../platform/daplink-rp-pio/src/proto.rs), [`codec.rs`](../../platform/daplink-rp-pio/src/codec.rs), [`clock.rs`](../../platform/daplink-rp-pio/src/clock.rs) | Wiring and `PINCTRL`, SWD framing, the FIFO command word, divider math |
| [`program.rs`](../../platform/daplink-rp-pio/src/program.rs), [`asm.rs`](../../platform/daplink-rp-pio/src/asm.rs) | Generic PIO0 SWD and JTAG programs; the const instruction encoder every program uses |
| [`shifter.rs`](../../platform/daplink-rp-pio/src/shifter.rs), [`sm.rs`](../../platform/daplink-rp-pio/src/sm.rs), [`regs.rs`](../../platform/daplink-rp-pio/src/regs.rs) | The state-machine seam, its raw-register implementation, block-wide registers |
| [`sequence.rs`](../../platform/daplink-rp-pio/src/sequence.rs), [`jtag.rs`](../../platform/daplink-rp-pio/src/jtag.rs) | Generic PIO0 SWD/SWJ sequence spans and JTAG sequences |
| [`generic_roles.rs`](../../platform/daplink-rp-pio/src/generic_roles.rs) | The PIO0 engine state and its three dap-rs role views |
| [`transaction.rs`](../../platform/daplink-rp-pio/src/transaction.rs), [`transaction_program.rs`](../../platform/daplink-rp-pio/src/transaction_program.rs), [`transaction_sm.rs`](../../platform/daplink-rp-pio/src/transaction_sm.rs) | PIO1 complete-transfer SWD: ACK decision, program, state machine |
| [`autonomous_program.rs`](../../platform/daplink-rp-pio/src/autonomous_program.rs), [`autonomous_sm.rs`](../../platform/daplink-rp-pio/src/autonomous_sm.rs) | PIO1 autonomous JTAG transfers: program and state machine |
| [`swd_transaction_hybrid.rs`](../../platform/daplink-rp-pio/src/swd_transaction_hybrid.rs) | Generic PIO0 SWJ beside the PIO1 SWD transfer engine |
| [`composition.rs`](../../platform/daplink-rp-pio/src/composition.rs), [`transfer_engine.rs`](../../platform/daplink-rp-pio/src/transfer_engine.rs) | The `accel` composition and the one PIO1 owner both protocols share |

## Operation routing

The `accel` composition keeps generic PIO0 for sequences and fallbacks beside one PIO1 engine
that runs complete transfers. The [Debug Probe](../boards/debugprobe.md) has no JTAG wiring, so
it composes only the SWD accelerator; Pico boards compose both protocols.

| Operation | Where it runs |
|---|---|
| SWD register read/write | PIO1 fixed request and ACK-continuation suffix; CPU chooses continuation |
| Arbitrary SWD/SWJ sequences | Generic PIO0 spans, after PIO1 releases the pads |
| SWJ pin access | SIO pad access, after PIO1 releases the pads |
| JTAG sequences, TMS sequences | Generic PIO0 JTAG |
| JTAG transfer, index 0 of two devices, zero idle cycles | Autonomous PIO1 transfer |
| Other JTAG transfers | Generic PIO0 JTAG; CPU frames the scan |
| JTAG IR scan | PIO1 only while already active, with IR geometry `(before, length, after) = (0, 4, 5)` and an encodable instruction; otherwise PIO0 |

Accelerated IR instructions are ABORT, DPACC, APACC and IDCODE. The fixed SWD suffix encodes one
turnaround cycle; the hybrid path does not support another configured turnaround.

After ACK, the CPU chooses read, write or release and PIO1 clocks the complete continuation, so a
successful write can return before its wire suffix finishes. The autonomous JTAG program handles
ACK in PIO and publishes its result after returning the TAP to Idle. Retry policy stays above the
wire engine.

## Firmware compositions

`accel` is the default on every RP root; disabling it selects CPU bit-bang.

| Firmware root | `accel` | Completion boundaries |
|---|---|---|
| `rp2040-pico-embassy` | PIO0 SWJ plus shared PIO1 SWD/JTAG | DAP fence; with `flash`, vendor and VFS backend fences |
| `rp2040-pico-rtic` | PIO0 SWJ plus shared PIO1 SWD/JTAG | DAP fence; with `flash`, vendor and VFS backend fences |
| `rp2350-pico2-embassy` | PIO0 SWJ plus shared PIO1 SWD/JTAG | DAP fence; with `flash`, vendor and VFS backend fences |
| `debugprobe` | PIO0 SWJ plus PIO1 SWD only | DAP fence; no flash composition or JTAG |

## Ownership and completion

PIO0 keeps the generic SWD program on SM0 and, where composed, generic JTAG on SM1. PIO1 SM0 holds
either the complete-transfer SWD program or the autonomous JTAG program, never both. Ownership
covers each entire block, since instruction memory, synchronizer bypass, GPIO base and IRQ flags
are shared by its state machines. nRESET remains SIO.

Pad handoff between blocks keeps the clock low throughout, so no floating clock or extra rising
edge appears. On a protocol change, [`RpPioTransferEngine`](../../platform/daplink-rp-pio/src/transfer_engine.rs)
reloads PIO1 with the new program and restores the requested clock ceiling before entering it.

Every `accel` root attaches a `CommandFence` to `DapProbe`: `DAP_Transfer`, `DAP_TransferBlock`,
`DAP_WriteABORT` and handled vendor replies wait for queued SWD suffixes. Flash-enabled roots also
wrap the vendor and VFS backends in `FencedFlashBackend`, which fences after blocking operations
and each `step`; `begin_start`, `program_chunk` and `finish_start` are forwarded unfenced and
complete through `step`. `SwdMux` shares the adapter but does not replace these fences. See
[flash_backend.rs](../../crates/daplink-rs/src/flash_backend.rs).

## Clock profiles

The roots select system clocks; the engine maps the requested wire ceiling to a divider.

| Root system profile | Generic SWD cell | Complete-transfer SWD | PIO JTAG cell |
|---|---|---|---|
| RP2040 default, 125 MHz | 4 cycles, integer divider >= 2 | 4 cycles, divider >= 1.25 | 5 cycles |
| RP2350 default, 150 MHz | 4 cycles, integer divider >= 2 | 4 cycles, divider >= 1.5 | 6 cycles |
| Debug Probe opt-in 180 MHz | 5 cycles, integer divider >= 2 | 4 cycles, divider >= 2 | not composed |
| Pico opt-in 200 MHz | 5 cycles, integer divider >= 2 | 4 cycles, divider >= 2 | 8 cycles |

PIO JTAG and complete-transfer SWD use 16.8 fractional dividers with a JTAG floor of 1; generic
SWD uses an integer divider. Each divider takes whole system clocks per bit first, then rounds
upward, so no single clock period is shorter than the request. A request below the slowest
divider runs at that divider on every engine and is still reported as an error. The source owner
is [clock.rs](../../platform/daplink-rp-pio/src/clock.rs).
