/**
 * @file    gpio.c
 * @brief   GPIO access functions for stm32f4x1ce HIC
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

#include "gpio.h"

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Configure GPIO pin */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = RUNNING_LED_PIN;
    HAL_GPIO_Init(RUNNING_LED_PIN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RUNNING_LED_PIN_PORT, RUNNING_LED_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = CONNECTED_LED_PIN;
    HAL_GPIO_Init(CONNECTED_LED_PIN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(CONNECTED_LED_PIN_PORT, CONNECTED_LED_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = HID_LED_PIN;
    HAL_GPIO_Init(HID_LED_PIN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(HID_LED_PIN_PORT, HID_LED_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = CDC_LED_PIN;
    HAL_GPIO_Init(CDC_LED_PIN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(CDC_LED_PIN_PORT, CDC_LED_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = MSC_LED_PIN;
    HAL_GPIO_Init(MSC_LED_PIN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MSC_LED_PIN_PORT, MSC_LED_PIN, GPIO_PIN_RESET);
}

void gpio_set_hid_led(gpio_led_state_t state)
{
    // LED is active low
    HAL_GPIO_WritePin(HID_LED_PIN_PORT, HID_LED_PIN, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    // LED is active low
    HAL_GPIO_WritePin(CDC_LED_PIN_PORT, CDC_LED_PIN, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    // LED is active low
    HAL_GPIO_WritePin(MSC_LED_PIN_PORT, MSC_LED_PIN, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t gpio_get_reset_btn_no_fwrd(void)
{
    return 0;
}

uint8_t gpio_get_reset_btn_fwrd(void)
{
    return 0;
}


uint8_t GPIOGetButtonState(void)
{
    return 0;
}

void target_forward_reset(bool assert_reset)
{
}

void gpio_set_board_power(bool powerEnabled)
{
}
