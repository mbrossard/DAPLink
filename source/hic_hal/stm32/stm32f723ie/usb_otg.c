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

#include "usb_init.h"
#include "usbd_otg.h"

#if defined(USB_OTG_HS_USE_EXTERNAL_HS_PHY)
#error "stm32f723ie does not support external PHY"
#endif

void otg_usb_gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

#if defined(USB_OTG_USE_FS)
    // Pin USB_OTG_FS_DM : PA11
    // Pin USB_OTG_FS_DP : PA12
    GPIO_InitStruct.Pin       = (GPIO_PIN_11 | GPIO_PIN_12);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#elif  defined(USB_OTG_USE_HS) && defined(USB_OTG_HS_USE_EMBEDDED_HS_PHY)
    // Pin USB_OTG_HS_DM : PB14
    // Pin USB_OTG_HS_DP : PB15
    GPIO_InitStruct.Pin       = (GPIO_PIN_14 | GPIO_PIN_15);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#elif defined(USB_OTG_USE_HS) && defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)
    // Pin USB_OTG_HS_DM : PB14
    // Pin USB_OTG_HS_DP : PB15
    GPIO_InitStruct.Pin       = (GPIO_PIN_14 | GPIO_PIN_15);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#else
#error "Inconsistent configuration"
#endif
}
