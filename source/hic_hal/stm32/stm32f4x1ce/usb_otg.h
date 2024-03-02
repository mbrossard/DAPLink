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

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define USB_OTG_USE_FS                  1
#define USB_OTG_ENDPOINT_NUM            3

#endif
