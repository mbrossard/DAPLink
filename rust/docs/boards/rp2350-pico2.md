# Raspberry Pi Pico 2

Pinout for the `rp2350-pico2-embassy` probe firmware.

| Signal | GPIO | Board pin |
|---|---|---|
| SWCLK | GP2 | 4 |
| SWDIO | GP3 | 5 |
| nRESET | GP1 | 2 |
| TDI | GP6 | 9 |
| TDO | GP7 | 10 |
| VCOM TX | GP4 | 6 |
| VCOM RX | GP5 | 7 |
| SWO | GP13 | 17 |
| GND | -- | 3 |

Against the `test-rp2040` device-under-test image on a Pico, VCOM cross-connects TX to the DUT's
GP1 and RX to the DUT's GP0 (the DUT echoes on UART0), with a common ground.

The probe defaults to PIO SWD and fused CPU JTAG; its `-bitbang` xtask row selects the generic CPU
wire engine for comparison.

The opt-in `-overclock-200mhz` xtask row runs the system at 200 MHz and uses a five-cycle PIO SWD
cell. It is not a default and exceeds the RP2350's specified maximum system frequency.

To install without a second probe, build from `rust/` with
`cargo xtask build firmware/rp2350-pico2-embassy`, which writes a `.uf2` beside the ELF under
`target/firmware/rp2350-pico2-embassy/thumbv8m.main-none-eabihf/release/`. Hold BOOTSEL while
plugging the board in, then copy the `.uf2` onto the `RP2350` drive; the drive unmounts and the
board reboots straight into the new firmware.

Alternatively, build with `cargo build --release` in the firmware directory and flash the ELF at
`target/thumbv8m.main-none-eabihf/release/rp2350-pico2-embassy` through the DEBUG header
(`probe-rs download --chip RP235x <elf>`).
