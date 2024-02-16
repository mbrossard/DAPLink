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


#ifndef __USBD_OTG_H__
#define __USBD_OTG_H__

#include "usb_otg.h"

#if defined(USB_OTG_USE_FS) && defined(USB_OTG_USE_HS)
#error "USB_OTG_USE_FS and USB_OTG_USE_HS are mutually exclusive"
#elif !defined(USB_OTG_USE_FS) && !defined(USB_OTG_USE_HS)
#error "USB_OTG_USE_FS or USB_OTG_USE_HS must be defined"
#endif

#ifndef USB1_OTG_HS

#if defined(USB_OTG_USE_FS)
#define OTG                      USB_OTG_FS
#define USB_OTG_PERIPH_BASE      USB_OTG_FS_PERIPH_BASE
#define OTG_IRQn                 OTG_FS_IRQn
#ifndef USB_OTG_BUFFER_SIZE
#define USB_OTG_BUFFER_SIZE      1280U
#endif
#ifndef USBD_RX_FIFO_SIZE
#define USBD_RX_FIFO_SIZE        512U
#endif
#elif defined(USB_OTG_USE_HS)
#define OTG                      USB_OTG_HS
#define USB_OTG_PERIPH_BASE      USB_OTG_HS_PERIPH_BASE
#define OTG_IRQn                 OTG_HS_IRQn
#ifndef USB_OTG_BUFFER_SIZE
#define USB_OTG_BUFFER_SIZE      4096U
#endif
#ifndef USBD_RX_FIFO_SIZE
#define USBD_RX_FIFO_SIZE        1024U
#endif
#endif

#else

// STM32H7xx use a different combination where USB_OTG_FS
// is very similar to the USB_OTG_HS peripheral but without
// the option to connect to an HS PHY via ULPI.
#if defined(USB_OTG_USE_FS)
#define OTG                      USB2_OTG_FS
#define USB_OTG_PERIPH_BASE      USB2_OTG_FS_PERIPH_BASE
#define OTG_IRQn                 OTG_FS_IRQn
#ifndef USB_OTG_BUFFER_SIZE
#define USB_OTG_BUFFER_SIZE      4096U
#endif
#ifndef USBD_RX_FIFO_SIZE
#define USBD_RX_FIFO_SIZE        1024U
#endif
#elif defined(USB_OTG_USE_HS)
#define OTG                      USB1_OTG_HS
#define USB_OTG_PERIPH_BASE      USB1_OTG_HS_PERIPH_BASE
#define OTG_IRQn                 OTG_HS_IRQn
#ifndef USB_OTG_BUFFER_SIZE
#define USB_OTG_BUFFER_SIZE      4096U
#endif
#ifndef USBD_RX_FIFO_SIZE
#define USBD_RX_FIFO_SIZE        1024U
#endif
#endif


#endif

#ifndef USB_OTG_ENDPOINT_NUM
#error "USB_OTG_ENDPOINT_NUM must be defined"
#endif
#if     USB_OTG_ENDPOINT_NUM > 8
#error  Too many Endpoints, maximum IN/OUT Endpoint pairs that this driver supports is 8 !!!
#endif

#define OTG_DEV                  ((USB_OTG_DeviceTypeDef *)(USB_OTG_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define OTG_PCGCCTL              *(__IO uint32_t *)((uint32_t)USB_OTG_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)
#define EP_NUM(ep_addr)          ((ep_addr) & 0x0FU)
#define EP_DIR(ep_addr)          ((ep_addr & USB_ENDPOINT_DIRECTION_MASK) == USB_ENDPOINT_DIRECTION_MASK)
#define OTG_TX_FIFO(n)           *((volatile uint32_t *)(USB_OTG_PERIPH_BASE + USB_OTG_FIFO_BASE + ((n) * USB_OTG_FIFO_SIZE)))
#define OTG_RX_FIFO              *((volatile uint32_t *)(USB_OTG_PERIPH_BASE + USB_OTG_FIFO_BASE))
#define OTG_EP_IN(i)             ((USB_OTG_INEndpointTypeDef *)(USB_OTG_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define OTG_EP_OUT(i)            ((USB_OTG_OUTEndpointTypeDef *)(USB_OTG_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define OTG_DIEPTSIZ(ep_num)     ((OTG_EP_IN(ep_num))->DIEPTSIZ)
#define OTG_DIEPCTL(ep_num)      ((OTG_EP_IN(ep_num))->DIEPCTL)
#define OTG_DTXFSTS(ep_num)      ((OTG_EP_IN(ep_num))->DTXFSTS)
#define OTG_DOEPTSIZ(ep_num)     ((OTG_EP_OUT(ep_num))->DOEPTSIZ)
#define OTG_DOEPCTL(ep_num)      ((OTG_EP_OUT(ep_num))->DOEPCTL)
#define OTG_DIEPINT(ep_num)      ((OTG_EP_IN(ep_num))->DIEPINT)
#define OTG_DOEPINT(ep_num)      ((OTG_EP_OUT(ep_num))->DOEPINT)
#define OTG_EP_IN_TYPE(ep_num)   ((OTG_DIEPCTL(ep_num) >> 18) & 3U)
#define OTG_EP_OUT_TYPE(ep_num)  ((OTG_DOEPCTL(ep_num) >> 18) & 3U)
#define OTG_DAINT_IEPINT(in_ep)  ((uint32_t)((1U <<         in_ep)  & 0x0000FFFFU))
#define OTG_DAINT_OEPINT(out_ep) ((uint32_t)((1U << (16U + out_ep)) & 0xFFFF0000U))

#if defined(USB_OTG_USE_HS)
#if !defined(USB_OTG_HS_USE_EMBEDDED_HS_PHY) && \
    !defined(USB_OTG_HS_USE_EXTERNAL_HS_PHY) && \
    !defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)
#error "Must define USB_OTG_HS_USE_EMBEDDED_HS_PHY or USB_OTG_HS_USE_EXTERNAL_HS_PHY or USB_OTG_HS_USE_EMBEDDED_FS_PHY"
#endif
#endif

#ifndef USB_HS_PHYC_TUNE_VALUE
#define USB_HS_PHYC_TUNE_VALUE 0x00000F13U
#endif

#endif