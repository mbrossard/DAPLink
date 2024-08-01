# stm32l412xx HIC

Based on STM32L412K8 chip ([Datasheet](https://www.st.com/resource/en/datasheet/stm32l412k8.pdf)):
- Cortex-M4 48 Mhz
- 64 KB Flash
- 40 KB RAM
- USB 2.0 full-speed crystal less solution with LPM and BCD

## Memory Map

| Region   |  Size  | Start       | End         |
|----------|--------|-------------|-------------|
| Flash    | 64 KB  | 0x0800_0000 | 0x0801_0000 |
| SRAM1    | 32 KB  | 0x2000_0000 | 0x2000_8000 |
| SRAM2    |  8 KB  | 0x1000_0000 | 0x1000_2000 |

Because of the flash size limitation the DAPLink bootloader is not used.

## DAPLink default pin assignment

| Signal      | I/O | Symbol  | Pin |
|-------------|:---:|---------|:---:|
| SWD / JTAG  |
| SWCLK / TCK |  O  | PB13    |  26 | SPI2_SCK / USART3_CTS / TIM1_CH1N
| SWDIO / TMS |  O  | PB14    |  27 | SPI2_MISO / USRAT3_RTS / TIM1_CH2N
| SWDIO / TMS |  I  | PB12    |  25 | SPI2_NSS/ I1C1_SMBA/ USART3_CK / TIM1_BKIN
| SWO / TDO   |  I  | PA10    |  31 | USART1_RX / TIM1_CH3
| nRESET      |  O  | PB0     |  18 | ADC12_IN8 / TIM3_CH3 / TIM1_CH2N
| UART        |
| --UART RX     |  I  | PA2     |   8 | USART2_TX
| --UART TX     |  O  | PA3     |   9 | USART2_RX
| Button      |
| NF-RST But. |  I  | PB15    |  28 | SPI2_MOSI / TIM1_CH3N
| LEDs        |
| Connect. LED|  O  | PB6     |  42 | I2C1_SCL / TIM4_CH1 / USART1_TX
| HID LED     |  O  | PA9     |  30 | USART1_TX / TIM1_CH2
| CDC LED     |  O  | PA9     |  30 |
| MSC LED     |  O  | PA9     |  30 |
