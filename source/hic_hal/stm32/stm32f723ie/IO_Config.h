/**
 * @file    IO_Config.h
 * @brief   IO Configuration for stm32f723ie HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2021 Arm Limited, All Rights Reserved
 * Copyright (c) 2019 https://github.com/RadioOperator
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "device.h"
#include "compiler.h"
#include "daplink.h"

#include "stm32f7xx.h"
#include "stm32f7xx_hal_gpio.h"

// This GPIO configuration is only valid for the STM32F723IE HIC
COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_STM32F723IE);

#if defined(STLINKV3)

// Pin mapping from STLINK-V3MINI_BSP.h in
// https://github.com/RadioOperator/CMSIS-DAP_for_STLINK-V3MINI
// Debug Port I/O Pins for STM32F723, matched with STLINK-V3MINI board

// SWCLK/TCK (Input)            PB2
#define PIN_SWCLK_TCK_I_PORT    GPIOB
#define PIN_SWCLK_TCK_I         GPIO_PIN_2
#define PIN_SWCLK_TCK_I_Bit     2

// SWCLK/TCK (Output)           PH6
#define PIN_SWCLK_TCK_O_PORT    GPIOH
#define PIN_SWCLK_TCK_O         GPIO_PIN_6
#define PIN_SWCLK_TCK_O_Bit     6

// SWDIO/TMS (Input)            PH7
#define PIN_SWDIO_TMS_I_PORT    GPIOH
#define PIN_SWDIO_TMS_I         GPIO_PIN_7
#define PIN_SWDIO_TMS_I_Bit     7

// SWDIO OE (Output)            PF9
#define PIN_SWDIO_TMS_O_PORT    GPIOF
#define PIN_SWDIO_TMS_O         GPIO_PIN_9
#define PIN_SWDIO_TMS_O_Bit     9

// TDI (Input)                  PA1
#define PIN_TDI_I_PORT          GPIOA
#define PIN_TDI_I               GPIO_PIN_1
#define PIN_TDI_I_Bit           1

// TDI (Output)                 PF8
#define PIN_TDI_O_PORT          GPIOF
#define PIN_TDI_O               GPIO_PIN_8
#define PIN_TDI_O_Bit           8

// TDO (Input)                  PD2 (UART5_RX)
#define PIN_TDO_PORT            GPIOD
#define PIN_TDO                 GPIO_PIN_2
#define PIN_TDO_Bit             2

// nTRST Pin                    Not available
// #define PIN_nTRST_PORT
// #define PIN_nTRST
// #define PIN_nTRST_Bit

// nRESET Pin                   PA6
#define PIN_nRESET_PORT         GPIOA
#define PIN_nRESET              GPIO_PIN_6
#define PIN_nRESET_Bit          6

// nRESET Output Enable         Not available
// #define PIN_nRESET_OE_PORT
// #define PIN_nRESET_OE
// #define PIN_nRESET_OE_Bit

// Connected LED (Output)       PA10 - High, RED
#define LED_CONNECTED_PORT      GPIOA
#define LED_CONNECTED_BIT       GPIO_PIN_10

// Target Running LED (Output)  PA10 - Low,  GREEN
#define LED_RUNNING_PORT        GPIOA
#define LED_RUNNING_BIT         GPIO_PIN_10

#elif defined(HS_PROBE)

// SWCLK/TCK (Input)            PB3 (SPI1_CLK)
#define PIN_SWCLK_TCK_I_PORT    GPIOB
#define PIN_SWCLK_TCK_I         GPIO_PIN_3
#define PIN_SWCLK_TCK_I_Bit     3

// SWCLK/TCK (Output)           PI1 (SPI2_CLK)
#define PIN_SWCLK_TCK_O_PORT    GPIOI
#define PIN_SWCLK_TCK_O         GPIO_PIN_1
#define PIN_SWCLK_TCK_O_Bit     1

// SWDIO/TMS (Input)            PB4 (SPI1_MISO)
#define PIN_SWDIO_TMS_I_PORT    GPIOB
#define PIN_SWDIO_TMS_I         GPIO_PIN_4
#define PIN_SWDIO_TMS_I_Bit     4

// SWDIO/TMS (Output)           PB5 (SPI1_MOSI)
#define PIN_SWDIO_TMS_O_PORT    GPIOB
#define PIN_SWDIO_TMS_O         GPIO_PIN_5
#define PIN_SWDIO_TMS_O_Bit     5

// TDI (Input)                  Not available
// #define PIN_TDI_I_PORT
// #define PIN_TDI_I
// #define PIN_TDI_I_Bit

// TDI (Output)                 PI3 (SPI2_MOSI)
#define PIN_TDI_O_PORT          GPIOI
#define PIN_TDI_O               GPIO_PIN_3
#define PIN_TDI_O_Bit           3

// TDO (Input)                  PI2 (SPI2_MISO)
#define PIN_TDO_PORT            GPIOI
#define PIN_TDO                 GPIO_PIN_2
#define PIN_TDO_Bit             2

// nTRST Pin                    Not available
// #define PIN_nTRST_PORT
// #define PIN_nTRST
// #define PIN_nTRST_Bit

// nRESET Pin                   PA6
#define PIN_nRESET_PORT         GPIOA
#define PIN_nRESET              GPIO_PIN_6
#define PIN_nRESET_Bit          6

// nRESET Output Enable         Not available
// #define PIN_nRESET_OE_PORT
// #define PIN_nRESET_OE
// #define PIN_nRESET_OE_Bit

// DAP LED (Output)            PC10 (LED1)
#define PIN_DAP_LED_PORT       GPIOC
#define PIN_DAP_LED            GPIO_PIN_10
#define PIN_DAP_LED_Bit        10

// CDC LED (Output)            PB7 (LED2)
#define PIN_CDC_LED_PORT       GPIOB
#define PIN_CDC_LED            GPIO_PIN_7
#define PIN_CDC_LED_Bit        7

// MSC CDC (Output)            PE0 (LED3)
#define PIN_MSC_LED_PORT       GPIOE
#define PIN_MSC_LED            GPIO_PIN_0
#define PIN_MSC_LED_Bit        0

#elif defined(DISCOVERY)

// SWCLK/TCK (Input)            PE3 (ARD D7)
#define PIN_SWCLK_TCK_I_PORT    GPIOE
#define PIN_SWCLK_TCK_I         GPIO_PIN_3
#define PIN_SWCLK_TCK_I_Bit     3

// SWCLK/TCK (Output)           PE6 (ARD D6)
#define PIN_SWCLK_TCK_O_PORT    GPIOE
#define PIN_SWCLK_TCK_O         GPIO_PIN_6
#define PIN_SWCLK_TCK_O_Bit     6

// SWDIO/TMS (Input)            PB4 (ARD D12, MISO1)
#define PIN_SWDIO_TMS_I_PORT    GPIOB
#define PIN_SWDIO_TMS_I         GPIO_PIN_4
#define PIN_SWDIO_TMS_I_Bit     4

// SWDIO/TMS (Output)           PB5 (ARD D11, MOSI1)
#define PIN_SWDIO_TMS_O_PORT    GPIOB
#define PIN_SWDIO_TMS_O         GPIO_PIN_5
#define PIN_SWDIO_TMS_O_Bit     5

// TDI (Input)                  PH3 (ARD D4)
#define PIN_TDI_I_PORT          GPIOH
#define PIN_TDI_I               GPIO_PIN_4
#define PIN_TDI_I_Bit           4

// TDI (Output)                 PE5 (ARD D3)
#define PIN_TDI_O_PORT          GPIOE
#define PIN_TDI_O               GPIO_PIN_5
#define PIN_TDI_O_Bit           5

// TDO (Input)                  PA1 (ARD D10, NSS1, UART4_RX AF8)
#define PIN_TDO_PORT            GPIOA
#define PIN_TDO                 GPIO_PIN_1
#define PIN_TDO_Bit             1

// nRESET Pin                   PA6 (ARD A0)
#define PIN_nRESET_PORT         GPIOA
#define PIN_nRESET              GPIO_PIN_6
#define PIN_nRESET_Bit          6

// DAP LED (Output blue)       PA5 (ARDUINO)
#define PIN_DAP_LED_PORT       GPIOA
#define PIN_DAP_LED            GPIO_PIN_5
#define PIN_DAP_LED_Bit        5

// CDC LED (Output red)        PA7 (USER1)
#define PIN_CDC_LED_PORT       GPIOA
#define PIN_CDC_LED            GPIO_PIN_7
#define PIN_CDC_LED_Bit        7

// MSC CDC (Output green)      PB1 (USER2)
#define PIN_MSC_LED_PORT       GPIOB
#define PIN_MSC_LED            GPIO_PIN_1
#define PIN_MSC_LED_Bit        1

#endif

#endif
