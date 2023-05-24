/**
 * DAPLink Interface Firmware
 * Copyright (c) 2023, Arm Limited
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
#include "gpio.h"
#include "info.h"
#include "sdk.h"
#include "DAP.h"

#if defined(DRAG_N_DROP_SUPPORT)
#include "virtual_fs.h"
#endif

unsigned int g_board_info;

#if (BULK_ENDPOINT)
uint8_t TxDataBuffer[CFG_TUD_VENDOR_TX_BUFSIZE];
uint8_t RxDataBuffer[CFG_TUD_VENDOR_RX_BUFSIZE];
#endif

int main(void) {
    CFG_TUSB_DEBUG_PRINTF("Starting\n");

    sdk_init();
    gpio_init();
    info_init();

#if defined(DRAG_N_DROP_SUPPORT)
    vfs_init();
    usbd_msc_init();
#endif

    tusb_init();

    DAP_Setup();

    while (1) {
        tud_task();

#if (BULK_ENDPOINT)
        if (tud_vendor_available()) {
            uint32_t resp_len = 0;
            tud_vendor_read(RxDataBuffer, sizeof(RxDataBuffer));
            resp_len = DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);
            tud_vendor_write(TxDataBuffer, resp_len);
        }
#endif
    }

    return 0;
}
