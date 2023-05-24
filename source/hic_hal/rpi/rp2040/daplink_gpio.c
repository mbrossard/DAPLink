/**
 * @file    gpio.c
 * @brief   GPIO handling
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

#include "DAP_config.h"
#include "gpio.h"
#include "daplink.h"

void gpio_init(void)
{
}

void gpio_set_board_power(bool powerEnabled)
{
}

void gpio_set_hid_led(gpio_led_state_t state)
{
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
}

void gpio_set_msc_led(gpio_led_state_t state)
{
}

__WEAK uint8_t gpio_get_reset_btn_no_fwrd(void)
{
    return 0;
}

__WEAK uint8_t gpio_get_reset_btn_fwrd(void)
{
    return 0;
}
