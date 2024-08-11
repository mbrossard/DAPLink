# stm32l412xx HIC

Based on STM32L412K8 chip ([Datasheet](https://www.st.com/resource/en/datasheet/stm32l412k8.pdf)):
- Cortex-M4 48 Mhz
- 64 KB Flash
- 40 KB RAM
- Full-speed USB 2.0 crystal less: up to 8 bi-directional endpoints including EP0 (*)
- LQFP32 packaging

(*) "Configurable number of endpoints from 1 to 8", "up to 16 mono-directional or 8 bidirectional endpoints can be used". (source: [RM0394](https://www.st.com/resource/en/reference_manual/rm0394-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf))

## Memory Map

| Region   |  Size  | Start       | End         |
|----------|--------|-------------|-------------|
| Flash    | 64 KB  | 0x0800_0000 | 0x0801_0000 |
| SRAM1    | 32 KB  | 0x2000_0000 | 0x2000_8000 |
| SRAM2    |  8 KB  | 0x1000_0000 | 0x1000_2000 |

Because of the flash size limitation the DAPLink bootloader is not used.

## DAPLink default pin assignment

| Signal      | I/O | Symbol  | Pin | Comment |
|-------------|:---:|---------|:---:|---------|
| SWD / JTAG  |
| SWCLK / TCK |  O  | PA4    |  10 | Target SWCLK |
| SWDIO / TMS |  O  | PA5    |  11 | Target SWDIO |
| SWDIO / TMS |  I  | PA6    |  12 | Connect to PA5 through 100R |
| SWO / TDO   |  I  | PA10   |  20 | Target SWO   |
| nRESET      |  O  | PB0    |  14 | Target nReset |
| UART        |
| UART RX     |  I  | PA2    |   8 | Target RX |
| UART TX     |  O  | PA3    |   9 | Target TX |
| Button      |
| NF-RST But. |  I  | PB5    |  28 | Not used |
| LEDs        |
| Connect. LED|  O  | PB6    |  29 |
| HID LED     |  O  | PA9    |  19 |
| CDC LED     |  O  | PA9    |  19 |
| MSC LED     |  O  | PA9    |  19 |
| USB         |
| USB DP      |     | PA12   |  22 | Connect 18pF to GND and 33R serie |
| USB DM      |     | PA11   |  21 | Connect 18pF to GND and 33R serie |
