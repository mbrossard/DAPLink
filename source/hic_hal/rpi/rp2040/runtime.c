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

#include "daplink_debug.h"
#include "util.h"

void __attribute__((noreturn)) panic(const char *fmt, ...) {
#if defined (DAPLINK_DEBUG)
    va_list arg;
    va_start(arg, fmt);
    daplink_debug_print(fmt, arg);
    va_end(arg);
#endif
    util_assert(0);
    while (1) {}
}

void __attribute__((noreturn)) panic_unsupported() {
    panic("not supported");
}