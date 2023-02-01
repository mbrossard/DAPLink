/**
 * @file    IO_Config.h
 * @brief   IO Configuration for nrf52820 HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2021, Arm Limited, All Rights Reserved
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

// Override all defines if IO_CONFIG_OVERRIDE is defined
#ifdef IO_CONFIG_OVERRIDE
#include "IO_Config_Override.h"
#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__
#endif
#endif

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "device.h"
#include "compiler.h"
#include "daplink.h"
#include "nrf_gpio.h"

#define GPIO_LED_ACTIVE_STATE 1

// SWDCLK (Output)
#define PIN_SWCLK         NRF_GPIO_PIN_MAP(0, 25)

// SWDIO (Input/Output)
#define PIN_SWDIO         NRF_GPIO_PIN_MAP(0, 27)

// nRESET Pin
#define PIN_nRESET        NRF_GPIO_PIN_MAP(1, 7)

// pin to override the target select switch on Nordic boards
#define PIN_SWD_CTRL      NRF_GPIO_PIN_MAP(0, 18)

// Target Running LED (Output)
#undef LED_RUNNING

// Connected LED (Output)
#undef LED_CONNECTED

// HID LED
#define LED_HID           NRF_GPIO_PIN_MAP(0, 26)

// MSC LED
#define LED_MSC           LED_HID

// CDC LED
#define LED_CDC           LED_HID

// POWER LED
#undef LED_PWR

// Reset button (SW_RESET)
#define RESET_BUTTON      NRF_GPIO_PIN_MAP(0, 24)

// UART
#define UART_TX_PIN       NRF_GPIO_PIN_MAP(0, 29)
#define UART_RX_PIN       NRF_GPIO_PIN_MAP(1, 4)

#define UART_RTS_PIN      NRF_GPIO_PIN_MAP(0, 13)
#define UART_CTS_PIN      NRF_GPIO_PIN_MAP(0, 16)

// I2C
#undef I2C_SCL_PIN
#undef I2C_SDA_PIN
#endif
