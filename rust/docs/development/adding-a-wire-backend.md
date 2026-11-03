# Adding a wire backend

A backend implements the dependency, SWD and/or JTAG roles consumed by `DapProbe`; CMSIS-DAP
decoding stays in `dap-rs`. Start from the nearest existing mechanism:
[RP PIO](../design/rp-pio-engines.md), [LPC55 masked GPIO](../internals/lpc55/masked-swd.md),
or [STM32 SPI sessions](../internals/stm32/spi-assisted-swd.md).

1. Put testable protocol sequencing below a narrow hardware seam. Keep register ownership in the
   platform and pins, clocks and enabled roles in the firmware.
2. Define the valid transfer geometry and clock requests, what is rejected and what falls back.
   SWD acceleration does not imply JTAG; compose an existing JTAG role when needed.
3. Specify pad ownership across SWD, JTAG, SWO, reset and flash, and attach the required
   command/flash fences when queue acceptance precedes wire completion.
4. Preserve ACK/WAIT/FAULT behavior, parity, turnaround, idle cycles and error cleanup; a JTAG
   role decodes its ACKs through the dependency role's `JtagDp`.
5. Add a named firmware variant through the [board registration procedure](adding-a-board.md),
   without changing defaults.

Host tests should drive the real dispatcher or engine against literal expected wire results:
malformed commands, every ACK, parity failure, configuration changes, fallback and pad-release
ordering, and divider and geometry boundaries. Run the SWD/JTAG, reset, flash and streaming HIL
cases before making a backend the default.
