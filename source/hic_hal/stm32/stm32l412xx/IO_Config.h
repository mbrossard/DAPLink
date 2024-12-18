/**
 * @file    IO_Config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
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

#include "stm32l4xx.h"
#include "compiler.h"
#include "daplink.h"

COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_STM32L412XX);

//USB control pin
#define USB_CONNECT_PORT_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define USB_CONNECT_PORT_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()
#define USB_CONNECT_ON()             (USB->BCDR |= (uint16_t) USB_BCDR_DPPU)
#define USB_CONNECT_OFF()            (USB->BCDR &= (uint16_t) ~USB_BCDR_DPPU)
//USB data pin
#define USB_DM_PIN_PORT				 GPIOA
#define USB_DM_PIN                 	 GPIO_PIN_11
#define USB_DM_Bit                 	 11

#define USB_DP_PIN_PORT            	 GPIOA
#define USB_DP_PIN                 	 GPIO_PIN_12
#define USB_DP_Bit                	 12

//Connected LED
#define CONNECTED_LED_PORT           GPIOB
#define CONNECTED_LED_PIN            GPIO_PIN_7
#define CONNECTED_LED_PIN_Bit        7

//When bootloader, disable the target port(not used)
#define POWER_EN_PIN_PORT            GPIOA
#define POWER_EN_PIN                 GPIO_PIN_8
#define POWER_EN_Bit                 8

// nRESET OUT Pin
#define nRESET_PIN_PORT              GPIOB
#define nRESET_PIN                   GPIO_PIN_0
#define nRESET_PIN_Bit               0

//SWD
#define SWCLK_TCK_PIN_PORT           GPIOA
#define SWCLK_TCK_PIN                GPIO_PIN_4
#define SWCLK_TCK_PIN_Bit            4

#define SWDIO_OUT_PIN_PORT           GPIOA
#define SWDIO_OUT_PIN                GPIO_PIN_5
#define SWDIO_OUT_PIN_Bit            5

#define SWDIO_IN_PIN_PORT            GPIOA
#define SWDIO_IN_PIN                 GPIO_PIN_6
#define SWDIO_IN_PIN_Bit             6

//LEDs
//USB status LED
#define RUNNING_LED_PORT             GPIOB
#define RUNNING_LED_PIN              GPIO_PIN_6
#define RUNNING_LED_Bit              6

#define PIN_HID_LED_PORT             GPIOB
#define PIN_HID_LED                  GPIO_PIN_5
#define PIN_HID_LED_Bit              5

#define PIN_CDC_LED_PORT             GPIOB
#define PIN_CDC_LED                  GPIO_PIN_4
#define PIN_CDC_LED_Bit              4

#define PIN_MSC_LED_PORT             GPIOB
#define PIN_MSC_LED                  GPIO_PIN_3
#define PIN_MSC_LED_Bit              3


#endif
