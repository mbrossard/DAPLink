/*
 * DAPLink Interface Firmware
 * Copyright (c) 2023 Arm Limited, All Rights Reserved
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

#ifndef _TUSB_HIC_CONFIG_H_
#define _TUSB_HIC_CONFIG_H_

#define CFG_TUSB_MCU          OPT_MCU_LPC55XX
#define CFG_TUSB_RHPORT1_MODE OPT_MODE_DEVICE
#define CFG_TUD_MAX_SPEED     OPT_MODE_HIGH_SPEED
#define CFG_TUSB_MEM_ALIGN    __attribute__((aligned(64)))
#define CFG_TUSB_MEM_SECTION  __attribute__((section(".usbram")))

#endif
