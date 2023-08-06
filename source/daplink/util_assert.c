/**
 * @file    util.c
 * @brief   Implementation of util.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
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

#include <string.h>

#include "util.h"
#include "settings.h"
#include "cortex_m.h"

//remove dependency from vfs_manager
__WEAK void vfs_mngr_fs_remount(void) {}

__WEAK void _util_assert(bool expression, const char *filename, uint16_t line)
{
    bool assert_set;
    cortex_int_state_t int_state;

    if (expression) {
        return;
    }

    int_state = cortex_int_get_and_disable();
    // Only write the assert if there is not already one
    assert_set = config_ram_get_assert(0, 0, 0, 0);

    if (!assert_set) {
        config_ram_set_assert(filename, line);
    }

    cortex_int_restore(int_state);

    // Start a remount if this is the first assert
    // Do not call vfs_mngr_fs_remount from an ISR!
    if (!assert_set && !cortex_in_isr()) {
        vfs_mngr_fs_remount();
    }
}

void util_assert_clear()
{
    cortex_int_state_t int_state;
    int_state = cortex_int_get_and_disable();
    config_ram_clear_assert();
    cortex_int_restore(int_state);
}
