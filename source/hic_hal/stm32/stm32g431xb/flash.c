/**
 * @file    flash_hal_stm32f103xb.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
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

#include "flash_hal.h"        // FlashOS Structures
#include "target_config.h"    // target_device
#include "stm32g4xx.h"
#include "util.h"
#include "target_board.h"
#include "daplink_addr.h"

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
static uint32_t FlashInitStatus = 0;

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    FlashInitStatus |= 1 << fnc;
    if (FlashInitStatus)
    {
        HAL_FLASH_Unlock();
    }
    return 0;
}

uint32_t UnInit(uint32_t fnc)
{
    FlashInitStatus &= ~(1 << fnc);
    if (!FlashInitStatus)
    {
        HAL_FLASH_Lock();
    }
    return 0;
}

uint32_t EraseSector(uint32_t adr)
{
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t error;
    uint32_t ret = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = (adr - DAPLINK_ROM_START) / DAPLINK_SECTOR_SIZE;
    erase_init.NbPages = 1;
    erase_init.Banks = FLASH_BANK_1;

    if (HAL_FLASHEx_Erase(&erase_init, &error) != HAL_OK) {
        ret = 1;
    }

    return ret;
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    uint32_t i;
    uint32_t ret = 0;
    uint64_t *dbuf = (uint64_t *) buf;

    util_assert(sz % 8 == 0);

    for (i = 0; i < sz / 8; i++) {
        uint32_t addr = adr + i * 8;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dbuf[i]) != HAL_OK) {
            ret = 1;
            break;
        }
    }

    return ret;
}
