/**
 * @file    read_uid.c
 * @brief   UID extraction function for stm32f723ie HIC
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

#include "stm32f723xx.h"
#include "read_uid.h"

void read_unique_id(uint32_t *id)
{
    uint32_t *uid_base = (uint32_t *)UID_BASE;

    id[0] = uid_base[0];
    id[1] = uid_base[1];
    id[2] = uid_base[2];
    id[3] = 0xA5A5A5A5;
}
