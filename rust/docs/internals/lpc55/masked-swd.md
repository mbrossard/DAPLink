# LPC55 masked-GPIO SWD

MCU-Link's `accel` path specializes SWD word transfers and JTAG scans over masked GPIO; the
firmware's `bitbang` feature overrides it. Board wiring and clocks stay in the
[MCU-Link root](../../../firmware/mculink/src/main.rs).

`MASK0` restricts `MPIN0` accesses to SWDIO. SWCLK uses set/clear registers, and the board's
direction-controlled buffer has a separate enable. The accelerated adapter and its generic fallback
drive the same pads, so no other GPIO user may touch that bank. Leaving SWD restores the
masked-access state; SWD, JTAG and SWO share the TDO/SWO pad.

The [masked engine](../../../platform/daplink-lpc55-hal/src/masked.rs) holds the word loops and a
packet-level block-transfer interceptor; generic handling covers what the interceptor declines,
pin control and sequences. The fallback must see the same protocol configuration as the
accelerated path.

Pacing is injected through `MaskedPacing`, selected outside the data bit loops. The linker places
the selected routines in SRAMX; check placement in the linked ELF. A read clocks the trailing
turnaround before checking parity, and a non-OK ACK still owes its release cycles.
