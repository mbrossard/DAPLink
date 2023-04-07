/**
 * @file    uart.c
 * @brief
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

#include "string.h"

#include "stm32g4xx.h"
#include "uart.h"
#include "gpio.h"
#include "util.h"
#include "circ_buf.h"
#include "IO_Config.h"

// For usart

#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (512)

circ_buf_t write_buffer;
uint8_t write_buffer_data[BUFFER_SIZE];
circ_buf_t read_buffer;
uint8_t read_buffer_data[BUFFER_SIZE];

static UART_Configuration configuration = {
    .Baudrate = 9600,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE,
};
UART_HandleTypeDef uart_handle = {0};

static void clear_buffers(void)
{
    circ_buf_init(&write_buffer, write_buffer_data, sizeof(write_buffer_data));
    circ_buf_init(&read_buffer, read_buffer_data, sizeof(read_buffer_data));
}

int32_t uart_initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    uart_reset();

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /** USART2 GPIO Configuration
        PA3     ------> USART2_RX
        PA2     ------> USART2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_EnableIRQ(USART2_IRQn);

    return 1;
}

int32_t uart_uninitialize(void)
{
    __HAL_RCC_USART2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    HAL_NVIC_DisableIRQ(USART2_IRQn);

    return 1;
}

int32_t uart_reset(void)
{
    uart_handle.Instance = USART2;

    __HAL_UART_DISABLE_IT(&uart_handle, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&uart_handle, UART_IT_RXNE);

    clear_buffers();

    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    uart_handle.Instance = USART2;

    /* Data bits */
    if (config->DataBits != UART_DATA_BITS_8)
        return(0);

    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;

    /* Parity */
    switch (config->Parity)
    {
        case UART_PARITY_NONE:
            uart_handle.Init.Parity = HAL_UART_PARITY_NONE;
            break;
        case UART_PARITY_EVEN:
            uart_handle.Init.Parity = HAL_UART_PARITY_EVEN;
            uart_handle.Init.WordLength = UART_WORDLENGTH_9B;
            break;
        case UART_PARITY_ODD:
            uart_handle.Init.Parity = HAL_UART_PARITY_ODD;
            break;
        default:
            return (0);
    }

    /* Stop bits */
    switch (config->StopBits)
    {
        case UART_STOP_BITS_1:
            uart_handle.Init.StopBits = UART_STOPBITS_1;
            break;
        case UART_STOP_BITS_2:
            uart_handle.Init.StopBits = UART_STOPBITS_2;
            break;
        case UART_STOP_BITS_1_5:
            uart_handle.Init.StopBits = UART_STOPBITS_1_5;
            break;
        default:
            return (0);
    }

    uart_handle.Init.BaudRate = config->Baudrate;

    uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    uart_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart_handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart_handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    uart_reset();

    util_assert(HAL_OK == HAL_UART_DeInit(&uart_handle));
    util_assert(HAL_OK == HAL_UART_Init(&uart_handle));

    __HAL_UART_ENABLE_IT(&uart_handle, UART_IT_RXNE);

    return 1;
}

int32_t uart_get_configuration(UART_Configuration *config)
{
    config->Baudrate = configuration.Baudrate;
    config->DataBits = configuration.DataBits;
    config->Parity   = configuration.Parity;
    config->StopBits = configuration.StopBits;
    config->FlowControl = UART_FLOW_CONTROL_NONE;

    return 1;
}

void uart_set_control_line_state(uint16_t ctrl_bmp)
{
}

int32_t uart_write_free(void)
{
    return circ_buf_count_free(&write_buffer);
}

int32_t uart_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt = circ_buf_write(&write_buffer, data, size);

    __HAL_UART_ENABLE_IT(&uart_handle, UART_IT_TXE);

    return cnt;
}

int32_t uart_read_data(uint8_t *data, uint16_t size)
{
    return circ_buf_read(&read_buffer, data, size);
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_IT(&uart_handle, UART_IT_RXNE) != RESET) {
        uint8_t dat = uart_handle.Instance->RDR & 0xFF;
        uint32_t free = circ_buf_count_free(&read_buffer);
        if (free > RX_OVRF_MSG_SIZE) {
            circ_buf_push(&read_buffer, dat);
        } else if (RX_OVRF_MSG_SIZE == free) {
            circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else {
            // Drop character
        }
    }

    if (__HAL_UART_GET_IT(&uart_handle, UART_IT_TXE) != RESET) {
        if (circ_buf_count_used(&write_buffer) > 0) {
            uart_handle.Instance->TDR = circ_buf_pop(&write_buffer);
        } else {
            __HAL_UART_DISABLE_IT (&uart_handle, UART_IT_TXE);
        }
    }
}
