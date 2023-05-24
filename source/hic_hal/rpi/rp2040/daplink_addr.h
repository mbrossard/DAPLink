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

#ifndef DAPLINK_ADDR_H
#define DAPLINK_ADDR_H

// Flash layout:
//
//  [ 0x1000_0000 - 0x1000_0100 ]: 256 B  - bootloader
//  [ 0x1000_0100 - 0x1001_FC00 ]: 127 kB - interface
//  [ 0x1001_FC00 - 0x1001_FFFF ]:   1 kB - persistent config (cfgrom)
//
// Notes:
// 1. The interface does not extend to the end of flash because that makes the binary
//    firmware image large and slow to program.
// 2. Attempting to program flash starting at 0x96000 fails for some reason, even though
//    the UM states that 630 kB is available for the user (0x9d800).

/* Device sizes */

#define DAPLINK_ROM_START               0x10000000 //
#define DAPLINK_ROM_SIZE                0x00020000 //127 kB IF + 1 kB config

#define DAPLINK_RAM_START               0x20000000 //
#define DAPLINK_RAM_SIZE                0x00040000 // 256 kB

/* ROM sizes */

#define DAPLINK_ROM_BL_START            0x10000000
#define DAPLINK_ROM_BL_SIZE             0x00000000 //

#define DAPLINK_ROM_IF_START            0x10000000
#define DAPLINK_ROM_IF_SIZE             0x0001FC00 // 127 kB interface

#define DAPLINK_ROM_CONFIG_USER_START   0x1001FC00
#define DAPLINK_ROM_CONFIG_USER_SIZE    0x00000400 // 1 kB config

/* RAM sizes */

#define DAPLINK_RAM_APP_START           0x20000000
#define DAPLINK_RAM_APP_SIZE            0x0003FF00

#define DAPLINK_RAM_SHARED_START        0x2003FF00
#define DAPLINK_RAM_SHARED_SIZE         0x00000100

/* Flash Programming Info */

#define DAPLINK_SECTOR_SIZE             0x00001000
#define DAPLINK_MIN_WRITE_SIZE          0x00000100

/* USB RAM */
#define DAPLINK_USB_RAM_START           0x00000000
#define DAPLINK_USB_RAM_SIZE            0x00000000

/* Current build */

#if defined(DAPLINK_IF)

#define DAPLINK_ROM_APP_START            DAPLINK_ROM_IF_START
#define DAPLINK_ROM_APP_SIZE             DAPLINK_ROM_IF_SIZE
#define DAPLINK_ROM_UPDATE_START         DAPLINK_ROM_BL_START
#define DAPLINK_ROM_UPDATE_SIZE          0

#else

#error "Build must be interface"

#endif

#endif
