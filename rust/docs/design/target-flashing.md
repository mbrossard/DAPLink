# Target flashing

Target flashing has two host front ends: a virtual FAT16 mass-storage volume and CMSIS-DAP vendor
stream commands. Each owns its protocol state and flash backend, but both feed decoded image
fragments through the same `TransferSink` and share one arbitrated target adapter.

```text
mass storage -> virtual volume --\
                                -> decoder -> FlashSink -> flash runtime -> target
vendor commands ---------------/
```

`daplink-targets` supplies generated algorithm blobs, target memory descriptions, identities,
reset sequences, and UF2 family identifiers. A firmware composes only its supported target rows;
current firmware uses `FixedTarget`, with the image family retained as a mismatch check.

`FlashSink` validates image family and address ranges, then drives a `FlashBackend` through begin,
program, and finish phases. `daplink-flash` loads and runs the selected target algorithm, manages
erase and page buffering, verifies writes, and applies the target reset sequence.

Flash-enabled firmware places the physical SWD/JTAG adapter behind `SwdMux`. CMSIS-DAP receives a
gated handle and reports the target unavailable while a flash session owns the adapter; the flash
backend receives exclusive access until it finishes or aborts. This priority prevents interleaved
debug commands from corrupting a programming operation.

Completion policy remains a composition choice. Firmware decides whether a successful transfer
resets the target and whether a mass-storage remount requires USB reconnection.
