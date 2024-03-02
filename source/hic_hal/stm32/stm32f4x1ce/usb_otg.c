/**
 * @file    usb_otg.c
 * @brief   USB Driver HIC-specific
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2023, Mathias Brossard <mathias@brossard.org>
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

#include "usb_otg.h"

#if !defined(USB_OTG_USE_FS)
#error "stm32f4x1ce only supports FS"
#endif

void otg_usb_gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Pin USB_OTG_FS_DM : PA11
    // Pin USB_OTG_FS_DP : PA12
    GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
