/**
 * @file    daplink.c
 * @brief   DAPLink TinyUSB integration
 *
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

#include "tusb.h"
#include "cmsis_compiler.h"

#if BULK_ENPOINT
uint8_t TxDataBuffer[CFG_TUD_VENDOR_TX_BUFSIZE];
uint8_t RxDataBuffer[CFG_TUD_VENDOR_RX_BUFSIZE];
#endif

__WEAK void daplink_tusb_init(void) {}

extern void USBD_SignalHandler(void);
extern void USBD_IntEnable(void);

void usbd_init(void)
{
    tusb_init();
    daplink_tusb_init();
}

static uint32_t usbd_con = 0;
void usbd_connect(uint32_t con)
{
    usbd_con = con;
}

uint32_t usbd_configured(void)
{
    return usbd_con;
}

void cdc_process_event(void) {
}

void USBD_Handler(void)
{
#ifdef TUSB_DAPLINK_DEFERRED
    tud_int_handler(1);
#endif

    tud_task();

#if BULK_ENDPOINT
    if (tud_vendor_available()) {
        tud_vendor_read(RxDataBuffer, sizeof(RxDataBuffer));
        uint32_t = resp_len = DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);
        tud_vendor_write(TxDataBuffer, resp_len);
        tud_vendor_flush();
    }
#endif

#ifdef TUSB_DAPLINK_DEFERRED
    USBD_IntEnable();
#endif
}
