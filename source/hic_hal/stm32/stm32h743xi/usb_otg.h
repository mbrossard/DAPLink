/**
 * @file    usb_otg.h
 * @brief   USB Driver Configuration
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2022, Mathias Brossard <mathias@brossard.org>
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

#ifndef __USB_OTG_H__
#define __USB_OTG_H__

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "util.h"

#define GPIO_AF10_OTG_FS GPIO_AF10_OTG1_FS
#define GPIO_AF10_OTG_HS GPIO_AF10_OTG1_HS
#define GPIO_AF12_OTG_HS_FS GPIO_AF12_OTG1_FS

#if defined(FORCE_USB_FS)

#define USB_OTG_USE_FS                  1
#define USB_OTG_ENDPOINT_NUM            8

#elif defined(USE_EMBEDDED_FS)

#define USB_OTG_USE_HS                  1
#define USB_OTG_HS_USE_EMBEDDED_FS_PHY  1
#define USB_OTG_ENDPOINT_NUM            8

#else

#define USB_OTG_USE_HS                  1
#define USB_OTG_HS_USE_EXTERNAL_HS_PHY  1
#define USB_OTG_ENDPOINT_NUM            8

#endif

#endif
