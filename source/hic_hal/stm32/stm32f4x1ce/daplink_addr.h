/**
 * @file    daplink_addr.h
 * @brief   Address map for stm32f4x1ce HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2021, Arm Limited
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

#ifndef DAPLINK_ADDR_H
#define DAPLINK_ADDR_H

/*
 * Flash module organization:
 * | Sector 0 | 0x0800_0000 - 0x0800_3FFF | 16 Kbytes  |
 * | Sector 1 | 0x0800_4000 - 0x0800_7FFF | 16 Kbytes  |
 * | Sector 2 | 0x0800_8000 - 0x0800_BFFF | 16 Kbytes  |
 * | Sector 3 | 0x0800_C000 - 0x0800_FFFF | 16 Kbytes  |
 * | Sector 4 | 0x0801_0000 - 0x0801_FFFF | 64 Kbytes  |
 * | Sector 5 | 0x0802_0000 - 0x0803_FFFF | 128 Kbytes |
 * | Sector 6 | 0x0804_0000 - 0x0805_FFFF | 128 Kbytes |
 * | Sector 7 | 0x0806_0000 - 0x0807_FFFF | 128 Kbytes |
 * Source: https://www.st.com/resource/en/reference_manual/dm00096844-stm32f401xb-c-and-stm32f401xd-e-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

/* Device sizes */

#define DAPLINK_ROM_START               0x08000000
#define DAPLINK_ROM_SIZE                0x00040000 /* Limit to 256 KB */

#define DAPLINK_RAM_START               0x20000000
#define DAPLINK_RAM_SIZE                0x00018000

/* ROM sizes */

#define DAPLINK_ROM_BL_START            0x08000000
#define DAPLINK_ROM_BL_SIZE             0x0000C000

#define DAPLINK_ROM_CONFIG_USER_START   0x0800C000
#define DAPLINK_ROM_CONFIG_USER_SIZE    0x00004000

#define DAPLINK_ROM_IF_START            0x08010000
#define DAPLINK_ROM_IF_SIZE             0x00030000

/* RAM sizes */

#define DAPLINK_RAM_APP_START           0x20000000
#define DAPLINK_RAM_APP_SIZE            0x00017F00

#define DAPLINK_RAM_SHARED_START        0x20017F00
#define DAPLINK_RAM_SHARED_SIZE         0x00000100

/* Flash Programming Info */

#define DAPLINK_SECTOR_SIZE             KB(16)
#define DAPLINK_MIN_WRITE_SIZE          KB(16)

/* Current build */

#if defined(DAPLINK_BL)

#define DAPLINK_ROM_APP_START            DAPLINK_ROM_BL_START
#define DAPLINK_ROM_APP_SIZE             DAPLINK_ROM_BL_SIZE
#define DAPLINK_ROM_UPDATE_START         DAPLINK_ROM_IF_START
#define DAPLINK_ROM_UPDATE_SIZE          DAPLINK_ROM_IF_SIZE

#elif defined(DAPLINK_IF)

#define DAPLINK_ROM_APP_START            DAPLINK_ROM_IF_START
#define DAPLINK_ROM_APP_SIZE             DAPLINK_ROM_IF_SIZE
#define DAPLINK_ROM_UPDATE_START         DAPLINK_ROM_BL_START
#define DAPLINK_ROM_UPDATE_SIZE          DAPLINK_ROM_BL_SIZE

#else

#error "Build must be either bootloader or interface"

#endif

#endif
