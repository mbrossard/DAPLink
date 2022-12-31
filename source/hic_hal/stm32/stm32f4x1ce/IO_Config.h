/**
 * @file    IO_Config.h
 * @brief   IO Configuration for stm32f4x1ce HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2021 Arm Limited, All Rights Reserved
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

#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"

// nRESET OUT Pin
#define nRESET_PIN_PORT              GPIOB
#define nRESET_PIN                   GPIO_PIN_0
#define nRESET_PIN_Bit               0

// SWD
#define SWCLK_TCK_PIN_PORT           GPIOB
#define SWCLK_TCK_PIN                GPIO_PIN_13
#define SWCLK_TCK_PIN_Bit            13

#define SWDIO_OUT_PIN_PORT           GPIOB
#define SWDIO_OUT_PIN                GPIO_PIN_14
#define SWDIO_OUT_PIN_Bit            14

#define SWDIO_IN_PIN_PORT            GPIOB
#define SWDIO_IN_PIN                 GPIO_PIN_12
#define SWDIO_IN_PIN_Bit             12

// JTAG
#define JTAG_TCK_PIN_PORT            SWCLK_TCK_PIN_PORT
#define JTAG_TCK_PIN                 SWCLK_TCK_PIN
#define JTAG_TCK_PIN_Bit             SWCLK_TCK_PIN_Bit

#define JTAG_TMS_PIN_PORT            SWDIO_OUT_PIN_PORT
#define JTAG_TMS_PIN                 SWDIO_OUT_PIN
#define JTAG_TMS_PIN_Bit             SWDIO_OUT_PIN_Bit

#define JTAG_TDI_PIN_PORT            GPIOA
#define JTAG_TDI_PIN                 GPIO_PIN_7
#define JTAG_TDI_PIN_Bit             7

#define JTAG_TDO_PIN_PORT            GPIOA
#define JTAG_TDO_PIN                 GPIO_PIN_5
#define JTAG_TDO_PIN_Bit             5

// Status LED
// There is only one LED connected to a GPIO

// Running LED (blie)
#define RUNNING_LED_PIN_PORT         GPIOC
#define RUNNING_LED_PIN              GPIO_PIN_13
#define RUNNING_LED_PIN_Bit          13

// Connected LED (blue)
#define CONNECTED_LED_PIN_PORT       GPIOC
#define CONNECTED_LED_PIN            GPIO_PIN_13
#define CONNECTED_LED_PIN_Bit        13

// HID / DAP (blue)
#define HID_LED_PIN_PORT             GPIOC
#define HID_LED_PIN                  GPIO_PIN_13
#define HID_LED_PIN_Bit              13

// CDC (blue)
#define CDC_LED_PIN_PORT             GPIOC
#define CDC_LED_PIN                  GPIO_PIN_13
#define CDC_LED_PIN_Bit              13

// MSC (blue)
#define MSC_LED_PIN_PORT             GPIOC
#define MSC_LED_PIN                  GPIO_PIN_13
#define MSC_LED_PIN_Bit              13

#endif
