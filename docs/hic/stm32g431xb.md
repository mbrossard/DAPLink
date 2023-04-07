# stm32g431xb HIC

Based on STM32G431CBT6 chip ([Datasheet](https://www.st.com/resource/en/datasheet/stm32g431cb.pdf)).

- Cortex-M4 170 Mhz
- 128 KB Flash
- 32 KB RAM
- USB 2.0 full-speed interface with LPM and BCD support.

## Memory Map

| Region   |  Size  | Start       | End         |
|----------|--------|-------------|-------------|
| Flash    | 128 KB | 0x0800_0000 | 0x0002_0000 |
| SRAM     | 32 KB  | 0x2000_0000 | 0x2000_8000 |

Bootloader size is 46 KB

## DAPLink default pin assignment

| Signal      | I/O | Symbol  | Pin |
|-------------|:---:|---------|:---:|
| SWD / JTAG  |
| SWCLK / TCK |  O  | PB13    |  27 |
| SWDIO / TMS |  O  | PB14    |  28 |
| SWDIO / TMS |  I  | PB12    |  26 |
| SWO / TDO   |  I  | PA10    |  32 |
| nRESET      |  O  | PB0     |  16 |
| UART        |
| UART RX     |  I  | PA2     |  10 |
| UART TX     |  O  | PA3     |  11 |
| Button      |
| NF-RST But. |  I  | PB15    |  29 |
| LEDs        |
| Connect. LED|  O  | PB6     |  43 |
| HID LED     |  O  | PA9     |  31 |
| CDC LED     |  O  | PA8     |  30 |
| MSC LED     |  O  | PB11    |  25 |
