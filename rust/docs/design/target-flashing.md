# Target flashing

Target flashing has two host front ends: a virtual FAT16 mass-storage volume and CMSIS-DAP vendor
stream commands. Both feed decoded image fragments through the same `TransferSink` and share one
arbitrated target adapter.

```text
mass storage -> virtual volume --\
                                -> decoder -> FlashSink -> flash runtime -> target
vendor commands ---------------/
```

`daplink-targets` supplies algorithm blobs, memory descriptions, identities, reset sequences, and
UF2 family identifiers. A firmware composes only its supported target rows. `FixedTarget` accepts
a family-less image only with a single configured target; with several, every block must carry a
UF2 Family ID matching exactly one row, and a missing, unknown or ambiguous ID is rejected before
programming. There is no hardware autodetection and no fallback to the primary target.

`FlashSink` validates image family and address ranges, then drives a `FlashBackend` through begin,
program, and finish. `daplink-flash` runs the target algorithm, buffers pages, verifies writes, and
applies the target reset sequence.

Flash-enabled firmware places the SWD/JTAG adapter behind `SwdMux`: CMSIS-DAP reports the target
unavailable while a flash session owns it. The firmware decides whether a successful transfer
resets the target and whether a mass-storage remount reconnects USB.

## Failure

Programming is not transactional: a failure after earlier blocks were accepted leaves the target
partly erased or programmed.

- A decoder error ends an MSC transfer with `DecodeError`.
- A sink range or programming failure latches in `FlashSink`, which swallows later fragments, so
  the MSC stream continues until completion or timeout.
- The vendor front end marks a decoder error as failed but ends the sink only on explicit close.

With deferred work a sector operation can return `Pending`; its owner must retry the same request.
`QueuedIo` acceptance means enqueued, not programmed. Follow the path through
[DirectIo](../../crates/daplink-vfs/src/msc_io.rs), [FlashSink](../../crates/daplink-vfs/src/flash_sink.rs)
and the [flash manager](../../crates/daplink-flash/src/flash_manager.rs). PIO compositions must also
keep the [command and flash fences](rp-pio-engines.md#ownership-and-completion).
