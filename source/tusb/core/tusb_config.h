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

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "tusb_hic_config.h"

#if defined(DAPLINK_DEBUG_TUSB)
#define CFG_TUSB_DEBUG_PRINTF daplink_debug_print
#if defined(DAPLINK_DEBUG_TUSB_LEVEL)
#define CFG_TUSB_DEBUG        DAPLINK_DEBUG_TUSB_LEVEL
#else
#define CFG_TUSB_DEBUG        3
#endif
#endif

#define CFG_TUD_ENDPOINT0_SIZE  64
#define CFG_TUSB_OS             OPT_OS_NONE

#ifdef HID_ENDPOINT_DISABLE
#undef HID_ENDPOINT
#define HID_ENDPOINT 0
#else
#ifndef HID_ENDPOINT
#define HID_ENDPOINT 0
#else
#define HID_ENDPOINT 1
#endif
#endif

#ifdef MSC_ENDPOINT_DISABLE
#undef MSC_ENDPOINT
#define MSC_ENDPOINT 0
#else
#ifndef MSC_ENDPOINT
#define MSC_ENDPOINT 0
#else
#define MSC_ENDPOINT 1
#endif
#endif

#ifdef CDC_ENDPOINT_DISABLE
#undef CDC_ENDPOINT
#define CDC_ENDPOINT 0
#else
#ifndef CDC_ENDPOINT
#define CDC_ENDPOINT 0
#else
#define CDC_ENDPOINT 1
#endif
#endif

#ifdef BULK_ENDPOINT_DISABLE
#undef BULK_ENDPOINT
#define BULK_ENDPOINT 0
#else
#ifndef BULK_ENDPOINT
#define BULK_ENDPOINT 0
#else
#define BULK_ENDPOINT 1
#endif
#endif

#ifdef WEBUSB_INTERFACE_DISABLE
#undef WEBUSB_INTERFACE
#define WEBUSB_INTERFACE 0
#else
#ifndef WEBUSB_INTERFACE
#define WEBUSB_INTERFACE 0
#else
#define WEBUSB_INTERFACE 1
#endif
#endif

#ifdef WINUSB_INTERFACE_DISABLE
#undef WINUSB_INTERFACE
#define WINUSB_INTERFACE 0
#else
#ifndef WINUSB_INTERFACE
#define WINUSB_INTERFACE 0
#else
#define WINUSB_INTERFACE 1
#endif
#endif

#define CFG_TUD_ENABLED         1
#define CFG_TUD_HID             HID_ENDPOINT
#define CFG_TUD_CDC             CDC_ENDPOINT
#define CFG_TUD_MSC             MSC_ENDPOINT
#define CFG_TUD_VENDOR          BULK_ENDPOINT
#define CFG_TUD_MIDI            0

#define CFG_TUD_CDC_RX_BUFSIZE    (TUD_OPT_HIGH_SPEED ? 1024 : 64)
#define CFG_TUD_CDC_TX_BUFSIZE    (TUD_OPT_HIGH_SPEED ? 1024 : 64)
#define CFG_TUD_VENDOR_RX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 1024 : 64)
#define CFG_TUD_VENDOR_TX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 1024 : 64)

#define CFG_TUD_MSC_EP_BUFSIZE    512

#ifndef CFG_TUSB_MCU
  #error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#endif

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
