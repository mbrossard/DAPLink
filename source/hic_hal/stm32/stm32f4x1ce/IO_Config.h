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

// Status LED
// There is only one LED connected to a GPIO

// Running LED (blue)
#define RUNNING_LED_PORT             GPIOC
#define RUNNING_LED_PIN              GPIO_PIN_13

// Connected LED (blue)
#define CONNECTED_LED_PORT           GPIOC
#define CONNECTED_LED_PIN            GPIO_PIN_13

// HID / DAP (blue)
#define PIN_HID_LED_PORT             GPIOB
#define PIN_HID_LED                  GPIO_PIN_13

// CDC (blue)
#define PIN_CDC_LED_PORT             GPIOC
#define PIN_CDC_LED                  GPIO_PIN_13

// MSC (blue)
#define PIN_MSC_LED_PORT             GPIOC
#define PIN_MSC_LED                  GPIO_PIN_13

#endif
