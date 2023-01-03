/**
 * @file    gpio.c
 * @brief   GPIO access functions for stm32f723ie HIC
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

#include "stm32f7xx_hal.h"

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    // PA10=Low,  Green LED on
    // PA10=High, Red   LED on
#if defined(LED_CONNECTED_PORT)
    GPIO_InitStruct.Pin   = LED_CONNECTED_BIT;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LED_CONNECTED_PORT, &GPIO_InitStruct);
    LED_CONNECTED_PORT->BSRR = LED_CONNECTED_BIT;
#endif

#if defined(PIN_DAP_LED_PORT)
    GPIO_InitStruct.Pin   = PIN_DAP_LED;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PIN_DAP_LED_PORT, &GPIO_InitStruct);
    PIN_DAP_LED_PORT->BSRR = PIN_DAP_LED;
#endif

#if defined(PIN_CDC_LED_PORT)
    GPIO_InitStruct.Pin   = PIN_CDC_LED;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PIN_CDC_LED_PORT, &GPIO_InitStruct);
    PIN_CDC_LED_PORT->BSRR = PIN_CDC_LED;
#endif

#if defined(PIN_MSC_LED_PORT)
    GPIO_InitStruct.Pin   = PIN_MSC_LED;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PIN_MSC_LED_PORT, &GPIO_InitStruct);
    PIN_MSC_LED_PORT->BSRR = PIN_MSC_LED;
#endif
}

void gpio_set_hid_led(gpio_led_state_t state)
{
#if defined(PIN_DAP_LED_PORT)
    // LED is active low
    HAL_GPIO_WritePin(PIN_DAP_LED_PORT, PIN_DAP_LED, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
#elif defined(LED_CONNECTED_PORT)
    // Low is green and high is red
    HAL_GPIO_WritePin(LED_CONNECTED_PORT, LED_CONNECTED_BIT, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
#if defined(PIN_CDC_LED_PORT)
    // LED is active low
    HAL_GPIO_WritePin(PIN_CDC_LED_PORT, PIN_CDC_LED, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
#elif defined(LED_CONNECTED_PORT)
    // Low is green and high is red
    HAL_GPIO_WritePin(LED_CONNECTED_PORT, LED_CONNECTED_BIT, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

void gpio_set_msc_led(gpio_led_state_t state)
{
#if defined(PIN_MSC_LED_PORT)
    // LED is active low
    HAL_GPIO_WritePin(PIN_MSC_LED_PORT, PIN_MSC_LED, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
#elif defined(LED_CONNECTED_PORT)
    // Low is green and high is red
    HAL_GPIO_WritePin(LED_CONNECTED_PORT, LED_CONNECTED_BIT, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
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
