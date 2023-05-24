/**
 * @file    read_uid.c
 * @brief   UID extraction function for rp2040 HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2023, Arm Limited, All Rights Reserved
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

#include "read_uid.h"
#include "pico.h"
#include "pico/unique_id.h"

void read_unique_id(uint32_t *id)
{
    pico_unique_board_id_t uID;
    pico_get_unique_board_id(&uID);

    id[0] = uID.id[7] << 24 | uID.id[6];
    id[1] = uID.id[5] << 24 | uID.id[4];
    id[2] = uID.id[3] << 24 | uID.id[2];
    id[3] = uID.id[1] << 24 | uID.id[0];
}
