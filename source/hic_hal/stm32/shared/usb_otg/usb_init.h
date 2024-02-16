/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2020 Arm Limited (or its affiliates). All rights reserved.
 * Copyright (c) 2022 Mathias Brossard <mathias@brossard.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef __USB_INIT_H__
#define __USB_INIT_H__

#include "usb_otg.h"

void otg_usb_gpio_init();
void otg_usb_clocks_init();
void otg_usb_reset();
void otg_usb_phy_init();
void otg_usb_reset_core();
void otg_usb_configure();
void otg_usb_enable();

#endif