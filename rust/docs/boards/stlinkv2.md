# ST-Link/V2

Pinout for the `stlinkv2` probe firmware: the **original ST-Link/V2** -- the cased, standalone
probe with a 20-pin header, STM32F103**C8** (64 KiB) -- not a clone dongle and not the V2-1
Nucleo on-board probe (F103CB, VCOM, no TDI/TDO).

| Signal | Pin |
|---|---|
| SWCLK / TCK | PB13 |
| SWDIO / TMS | PB14 |
| nRESET | PB0 (open-drain) |
| TDI | PA7 |
| TDO | PA6 |
| SWO | PA10 (USART1 RX, DMA1 CH5) |
| LED | PA9 (active low) |
| USB D- / D+ | PA11 / PA12 |

The core runs at 72 MHz from an 8 MHz crystal; a pulse on D+ forces re-enumeration, and PA15
(`USB_RENUMn`) is deliberately left alone -- driving it keeps the board off the bus. No VCOM,
which is what frees USART1 for SWO and PA9 for the LED. The header carries TDO and SWO on one
pin, so JTAG and SWO are both composed but used alternately, the target's choice.

Build with `cargo build --release` in the firmware directory, then flash the ELF through the
SWD header (`probe-rs download --chip STM32F103C8 <elf>`).
