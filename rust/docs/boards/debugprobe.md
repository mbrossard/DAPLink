# Raspberry Pi Debug Probe

Pinout for the `debugprobe` firmware, on the board's target-facing (D) connector:

| Signal | GPIO | Direction |
|---|---|---|
| SWCLK | GP12 | out, through the level shifter |
| SWDI | GP13 | in, the shifted readback of SWDIO |
| SWDIO | GP14 | out, tristated for the read turnaround |

SWD only: the board wires no reset line, no JTAG pins and no trace signal. VCOM rides UART1
(TX=GP4, RX=GP5). The `0x9F` vendor command reboots the probe into its ROM bootloader.

SWD runs on the PIO engines by default: generic PIO0 for sequences, PIO1 for complete
transfers. `--no-default-features` instead drives the wire from the CPU.

The opt-in xtask row `firmware/debugprobe-overclock-180mhz` runs the system at 180 MHz, with the
PIO cells in its [clock profile](../design/rp-pio-engines.md#clock-profiles). It is not a default
and exceeds the RP2040's specified maximum system frequency. It also enables `dap-packet-512`, which advertises a 512-byte DAP packet.

The board has no debug header. `cargo xtask flash firmware/debugprobe --put <selector>` updates a
running image through its UF2 bootloader; for recovery, hold BOOTSEL while plugging it in and copy
the `.uf2` onto `RPI-RP2`.
