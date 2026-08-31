# Raspberry Pi Pico

Pinout for the `rp2040-pico-embassy` and `rp2040-pico-rtic` probe firmware:

| Signal | GPIO | Board pin |
|---|---|---|
| SWCLK | GP2 | 4 |
| SWDIO | GP3 | 5 |
| nRESET | GP1 | 2 |
| TDI | GP6 | 9 |
| TDO | GP7 | 10 |
| VCOM TX | GP4 | 6 |
| VCOM RX | GP5 | 7 |
| GND | -- | 3 |

VCOM bridges the USB CDC-ACM serial port to UART1, following the host's line-coding baud (8N1).
Both probe variants default to accelerated SWD and JTAG; their `-bitbang` xtask rows select the
generic CPU wire engine for comparison.

The opt-in `firmware/rp2040-pico-embassy-overclock-200mhz` and
`firmware/rp2040-pico-rtic-overclock-200mhz` xtask rows run the system at 200 MHz. They are not
defaults and exceed the RP2040's specified maximum system frequency.

As a device under test (`tests/firmwares/test-rp2040`) the board instead echoes on UART0 --
TX=GP0 (pin 1), RX=GP1 (pin 2) -- and blinks GP25; SWD comes in on the DEBUG header.

To install without a second probe, build from `rust/` with
`cargo xtask build firmware/rp2040-pico-embassy` (or `firmware/rp2040-pico-rtic`), which writes
a `.uf2` beside the ELF under `target/<name>/thumbv6m-none-eabi/release/`. Hold BOOTSEL
while plugging the board in, then copy the `.uf2` onto the `RPI-RP2` drive; the drive unmounts
and the board reboots straight into the new firmware.

Alternatively, build with `cargo build --release` in either firmware directory and flash the
ELF at `target/thumbv6m-none-eabi/release/<firmware>` through the DEBUG header
(`probe-rs download --chip RP2040 <elf>`).
