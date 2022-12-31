/**
 * @file    okdo_e1.c
 * @brief   board file for Okdo E1
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2022, ARM Limited, All Rights Reserved
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

#include "target_family.h"
#include "target_board.h"

const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .family_id = kNXP_LPC55xx_FamilyID,
    .flags = kEnablePageErase,
    .daplink_drive_name = "OKDO_E1",
    .daplink_target_url = "https://www.okdo.com/p/okdo-e1-development-board",
    .target_cfg = &target_device,
    .board_vendor = "Okdo",
    .board_name = "E1",
};
