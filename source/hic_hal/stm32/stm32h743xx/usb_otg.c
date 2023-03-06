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

#if defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)
#error "stm32h743xx does not support embedded HS PHY"
#endif

void otg_usb_gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

#if defined(USB_OTG_USE_FS)
    // Pin USB_OTG_FS_DM : PA11
    // Pin USB_OTG_FS_DP : PA12
    GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#elif  defined(USB_OTG_USE_HS) && defined(USB_OTG_HS_USE_EXTERNAL_HS_PHY)
    /*
     * All STM32H743 have ULPI signals DIR and NXT as PC2_C and PC3_C.
     * Packages with 169 pins and above add respectively PI11 and PH4 as
     * alternate pins.
     */
    /* PA3: D0, PA5: CLK */
    GPIO_InitStruct.Pin       = (GPIO_PIN_3 | GPIO_PIN_5);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB0: D1, PB1: D2, PB5: D7, PB10: D3, PB11: D4, PB12: D5, PB13: D6 */
    GPIO_InitStruct.Pin       = (GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_5  |
                                 GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                                 GPIO_PIN_13);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if defined(OPENH743I)
    /* PC0: STP, PC2_C: DIR, PC3_C: NXT */
    GPIO_InitStruct.Pin       = (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#elif defined(H743_DISCOVERY)
    /* PC0: STP */
    GPIO_InitStruct.Pin       = GPIO_PIN_0;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PH4: NXT */
    GPIO_InitStruct.Pin       = GPIO_PIN_4;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* PI11: DIR */
    GPIO_InitStruct.Pin       = GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
#else
#error "Unsupported"
#endif
#elif defined(USB_OTG_USE_HS) && defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)
    // Pin USB_OTG_HS_DM : PB14
    // Pin USB_OTG_HS_DP : PB15
    GPIO_InitStruct.Pin       = (GPIO_PIN_14 | GPIO_PIN_15);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_OTG1_FS;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#else
#error "Inconsistent configuration"
#endif
}
