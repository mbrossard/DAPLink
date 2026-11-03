# STM32 SPI-assisted SWD

[daplink-spi-swd](../../../crates/daplink-spi-swd/src/lib.rs) owns SWD session sequencing;
`F1SpiPort` and `StmSpiPort` supply the family register operations, and a firmware `SpiSwdBoard`
selects SPI, GPIO, pins, alternate functions and clocks. The raw constructors do not reserve the
peripherals: the caller must exclude other register and pad users.

An SWD operation uses short SPI sessions for supported phase widths and GPIO for the remaining
edges and arbitrary sequences. Every session reacquires its pads before programming direction.
Session completion restores the family's clock-pad role: a GPIO clock on F1, the SPI alternate
function on H5. JTAG uses GPIO, with family-specific word scans where composed.

The SPI divider ladder respects both the requested ceiling and the board-supplied cap; GPIO
pacing is derived separately. Board clock constants stay out of the shared engine.

H5 also intercepts supported block transfers in
[block.rs](../../../platform/daplink-stm32-embassy/src/block.rs); the firmware's mirrored
configuration must stay aligned with the command handler, and flash-active dispatch bypasses that
observation. Cached GPIO mode words require serialization with other writers to that bank.

F1 frame operations take at most 32 bits and its receive helper holds at most four frames; H5's
alternate-function programming addresses pins 8 through 15.
