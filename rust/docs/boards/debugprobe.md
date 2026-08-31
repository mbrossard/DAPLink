# Raspberry Pi Debug Probe

Pinout for the `debugprobe` firmware, on the board's target-facing (D) connector:

| Signal | GPIO | Direction |
|---|---|---|
| SWCLK | GP12 | out, through the level shifter |
| SWDI | GP13 | in, the shifted readback of SWDIO |
| SWDIO | GP14 | out, tristated for the read turnaround |

SWD only: the board wires no reset line, no JTAG pins and no trace signal. VCOM rides UART1
(TX=GP4, RX=GP5). The `0x9F` vendor command reboots the probe into its ROM bootloader.

SWD runs on a PIO state machine by default; `--no-default-features --features bitbang,guard`
builds the same image with the wire driven by the CPU instead.

The opt-in xtask row `firmware/debugprobe-overclock-180mhz` runs the system at 180 MHz and uses a
five-cycle PIO SWD cell. It is not a default and exceeds the RP2040's specified maximum system
frequency.

The board has no debug header. `cargo xtask flash firmware/debugprobe --put <selector>` updates a
running image through its UF2 bootloader; holding BOOTSEL while plugging it in and copying the
`.uf2` onto `RPI-RP2` remains the recovery path.
