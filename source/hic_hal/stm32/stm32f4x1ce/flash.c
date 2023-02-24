/**
 * @file    flash.c
 * @brief   Flash access functions for stm32f4x1ce HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2021, Arm Limited, All Rights Reserved
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
#include "flash_blob.h"
#include "target_config.h"    // target_device
#include "stm32f4xx.h"
#include "util.h"
#include "string.h"
#include "target_board.h"
#include "target_config.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    //
    // No special init required
    //
    return (0);
}

uint32_t UnInit(uint32_t fnc)
{
    //
    // No special uninit required
    //
    return (0);
}

uint32_t EraseChip(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t error;
    uint32_t ret = 0;  // O.K.
    if (g_board_info.target_cfg) {
        HAL_FLASH_Unlock();

        memset(&erase_init, 0, sizeof(erase_init));
        erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
#ifdef DAPLINK_BL
        erase_init.Sector = 4;
        erase_init.NbSectors = 2;
#else
        erase_init.Sector = 0;
        erase_init.NbSectors = 4;
#endif
        if (HAL_FLASHEx_Erase(&erase_init, &error) != HAL_OK) {
            ret = 1;
        }

        HAL_FLASH_Lock();
    } else {
        ret = 1;
    }
    return ret;
}

uint32_t EraseSector(uint32_t adr)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t error, i, j;
    uint32_t ret = 0;  // O.K.

    if (g_board_info.target_cfg) {
        for (i = 0; i < g_board_info.target_cfg->sector_info_length; i++) {
            if (g_board_info.target_cfg->sectors_info[i].start == adr) {
                HAL_FLASH_Unlock();
                memset(&erase_init, 0, sizeof(erase_init));
                erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
                erase_init.Sector = i;
                erase_init.NbSectors = 1;
                if (HAL_FLASHEx_Erase(&erase_init, &error) != HAL_OK) {
                    ret = 1;
                }

                HAL_FLASH_Lock();
            } else if (g_board_info.target_cfg->sectors_info[i].start > adr) {
                break;
            }
        }
    }
    return ret;
}

static const sector_info_t sectors_info[] = {
    {0x08000000 + KB(0),   KB(16)},
    {0x08000000 + KB(16),  KB(16)},
    {0x08000000 + KB(32),  KB(16)},
    {0x08000000 + KB(48),  KB(16)},
    {0x08000000 + KB(64),  KB(64)},
    {0x08000000 + KB(128), KB(128)},
 };

#include "daplink_addr.h"

uint32_t EraseSectorSize(uint32_t adr)
{
    uint32_t i = 0, j = (sizeof(sectors_info) / sizeof(sector_info_t)) - 1;
    if ((adr < sectors_info[0].start) ||
        (adr >= (sectors_info[j].start + sectors_info[j].size))) {
        return 0;
    }

    uint32_t cur, next, size;
    for (; i < j - 1; i++) {
        cur = sectors_info[i].start;
        next = sectors_info[i + 1].start;

        if (adr > next) {
            continue;
        } else if (adr == next) {
            return sectors_info[i + 1].size;
        } else {
            return sectors_info[i].size;
        }
    }

    return sectors_info[j].size;
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    uint32_t i;
    uint32_t ret = 0;  // O.K.

    HAL_FLASH_Unlock();

    util_assert(sz % 4 == 0);
    for (i = 0; i < sz / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adr + i * 4, buf[i]) != HAL_OK) {
            ret = 1;
            break;
        }
    }

    HAL_FLASH_Lock();
    return ret;
}
