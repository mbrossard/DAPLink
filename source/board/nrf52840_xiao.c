/**
 * @file    nrf52820_bl.c
 * @brief   board ID, meta-data, and additional configuration for micro:bit
 *          nRF52820 HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2021, ARM Limited, All Rights Reserved
 * Copyright (c) 2021 Micro:bit Educational Foundation
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

#include "target_board.h"
#include "target_family.h"

extern target_cfg_t target_device_nrf52_64;

const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .board_id = "5220",
    .family_id = kNordic_Nrf52_FamilyID,
    .target_cfg = &target_device_nrf52_64,
    .board_vendor = "Seeed Studio",
    .board_name = "XIAO BLE",
};
