# MCU-Link probe

The `mculink` firmware targets the NXP MCU-Link (LPC55S69JBD64), the tree's first high-speed
USB probe: USB1 runs at high speed with 512-byte bulk endpoints and eight pipelined command
packets. The core runs at 150 MHz from PLL0 by default, refusing to boot on silicon older than
revision 1B; `core-96mhz` selects the on-chip 96 MHz FRO instead.
Check the revision before flashing a default image: `SYSCON.DIEID` bits 3:0 at `0x50000FFC`
must read `0x1` (1B), e.g. `probe-rs read b32 0x50000FFC 1` through the second probe.

By default the SWD word path runs on the masked-GPIO engine (`accel`: single-store SWDIO
access through `MASK0`/`MPIN0`, executed from zero-wait SRAMX) with the fast JTAG word scans;
`bitbang` wins the wire back to the CPU-driven engine. Both engines drive the board's
direction-controlled buffers (SWCLK/TCK on PIO0_0, SWDIO/TMS on PIO0_2 with TXEN PIO0_28,
nRESET on PIO0_19 with TXEN PIO0_13, TDI on PIO0_1, TDO on PIO0_3). The firmware adds
UART-mode SWO capture on Flexcomm3 (PIO0_3, runtime pad mux with TDO), a CDC-ACM VCOM bridge
to the DUT UART (Flexcomm0, PIO0_24 RX / PIO0_25 TX) following the host's line-coding baud,
the CMSIS-DAP v2 identity surface (`DAP_Info`, WinUSB, the hashed factory-UUID serial), and
the `0x9F` vendor reboot into the boot ROM's USB-HID ISP interface (`0x1FC9:0x0021`). An
opt-in `flash` build adds the drag-and-drop mass-storage drive (512-byte high-speed bulk
endpoints) and the vendor flash commands, with `flash-stm32f401` selecting the target on the
DUT header; `flash` always runs the bit-bang wire engine.

Build with `cargo build --release` in the firmware directory, then flash the ELF through the
board's SWD header with a second probe (`probe-rs download --chip LPC55S69JBD64 <elf>`).
