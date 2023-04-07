/*
 * Copyright (c) 2020 Màrius Montón <marius.monton@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Project:   CMSIS Driver implementation for STM32 devices
 *
 * This library manages USARTs for STM32 devices.
 * User should configure structs USART1_Resources, USART2_Resources, etc to
 * their routing and capabilities.
 *
 * Currently implemented:
 * - Implemented non-blocking mode for Send & Receive functions
 * - Implemented ARM_USART_GetModemStatus function
 *
 * To be implemented:
 * TODO: Implement transfer function
 * TODO: Implement the use of DMA for Send, Receive and Transfer functions.
 * TODO: Implement ARM_USART_GetStatus function.
 * TODO: Implement ARM_USART_SetModemControl function
 *
 */

#include "Driver_USART.h"

#include "stm32g4xx_hal.h"

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)   /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

typedef struct {
    GPIO_TypeDef *port;
    GPIO_InitTypeDef pin;
} STM32_PIN;

typedef struct {
    ARM_USART_CAPABILITIES capabilities;        // Capabilities
    /* Specific STM32 UART properties */
    UART_HandleTypeDef instance;
    STM32_PIN TxPin;
    STM32_PIN RxPin;
    ARM_USART_STATUS status;
    ARM_USART_MODEM_STATUS modem_status;
    ARM_USART_SignalEvent_t cb_event;
} STM32_USART_RESOURCES;

/* Driver Capabilities */
#ifdef USART1
static STM32_USART_RESOURCES USART1_Resources = {
    {
     1,                         /* supports UART (Asynchronous) mode */
     0,                         /* supports Synchronous Master mode */
     0,                         /* supports Synchronous Slave mode */
     0,                         /* supports UART Single-wire mode */
     0,                         /* supports UART IrDA mode */
     0,                         /* supports UART Smart Card mode */
     0,                         /* Smart Card Clock generator available */
     0,                         /* RTS Flow Control available */
     0,                         /* CTS Flow Control available */
     0,                         /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
     0,                         /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
     0,                         /* RTS Line: 0=not available, 1=available */
     0,                         /* CTS Line: 0=not available, 1=available */
     0,                         /* DTR Line: 0=not available, 1=available */
     0,                         /* DSR Line: 0=not available, 1=available */
     0,                         /* DCD Line: 0=not available, 1=available */
     0,                         /* RI Line: 0=not available, 1=available */
     0,                         /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
     0,                         /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
     0,                         /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
     0,                         /* Signal RI change event: \ref ARM_USART_EVENT_RI */
     0
     /* Reserved (must be zero) */
     },
    {.Instance = USART1,.Init.WordLength = UART_WORDLENGTH_8B,.Init.StopBits = UART_STOPBITS_1,.Init.Parity =
     HAL_UART_PARITY_NONE,.Init.HwFlowCtl = UART_HWCONTROL_NONE},
    {NULL},   /* Tx Pin */
    {GPIOA, {.Pin = GPIO_PIN_10,.Mode = GPIO_MODE_AF_OD,.Pull = GPIO_NOPULL,.Speed = GPIO_SPEED_FREQ_VERY_HIGH,.Alternate = GPIO_AF7_USART1}},   /* Rx Pin */
    {0},
    {0},
    NULL
};
#endif

// STM32 functions
static int32_t STM32_USART_Initialize(ARM_USART_SignalEvent_t cb_event, STM32_USART_RESOURCES * usart)
{
    if (usart->instance.Instance == NULL) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
#ifdef USART1
    else if (usart->instance.Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
#endif
#ifdef GPIOA
    if ((usart->TxPin.port == GPIOA) || (usart->RxPin.port == GPIOA)) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
#endif
    if (cb_event != NULL) {
        usart->cb_event = cb_event;
    }

    if (usart->TxPin.port)
        HAL_GPIO_Init(usart->TxPin.port, &usart->TxPin.pin);
    if (usart->RxPin.port)
        HAL_GPIO_Init(usart->RxPin.port, &usart->RxPin.pin);

    return ARM_DRIVER_OK;
}

static int32_t STM32_USART_Uninitialize(STM32_USART_RESOURCES * usart)
{
    HAL_UART_DeInit(&usart->instance);
    return ARM_DRIVER_OK;
}

static int32_t STM32_USART_PowerControl(ARM_POWER_STATE state, STM32_USART_RESOURCES const *usart)
{
    switch (state) {
    case ARM_POWER_OFF:
        break;
    case ARM_POWER_LOW:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
        break;
    case ARM_POWER_FULL:
        break;
    }
    return ARM_DRIVER_OK;
}

static int32_t STM32_USART_Send(const void *data, uint32_t num, STM32_USART_RESOURCES * usart)
{
    HAL_StatusTypeDef ret;

    if ((data == NULL) || (num == 0U)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    ret = HAL_UART_Transmit_IT(&usart->instance, (uint8_t *) data, num);

    if (ret == HAL_OK) {
        return ARM_DRIVER_OK;
    } else if (ret == HAL_BUSY) {
        return ARM_DRIVER_ERROR_BUSY;
    } else {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

static int32_t STM32_USART_Receive(void *data, uint32_t num, STM32_USART_RESOURCES * usart)
{
    if ((data == NULL) || (num == 0U)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    HAL_UART_Receive_IT(&usart->instance, (uint8_t *) data, num);

    return ARM_DRIVER_OK;
}

static int32_t STM32_USART_Transfer(const void *data_out, void *data_in,
                                    uint32_t num, STM32_USART_RESOURCES const *usart)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t STM32_USART_GetTxCount(STM32_USART_RESOURCES const *usart)
{
    return usart->instance.TxXferCount;
}

static uint32_t STM32_USART_GetRxCount(STM32_USART_RESOURCES const *usart)
{
    return usart->instance.RxXferCount;
}

static int32_t STM32_USART_Control(uint32_t control, uint32_t arg, STM32_USART_RESOURCES * usart)
{
    switch (control & ARM_USART_CONTROL_Msk) {
    case ARM_USART_MODE_ASYNCHRONOUS:
        usart->instance.Init.BaudRate = arg;
        break;
    case ARM_USART_CONTROL_TX:
        if (arg == 1) {
            usart->instance.Init.Mode |= UART_MODE_TX;
        } else {
            usart->instance.Init.Mode &= ~UART_MODE_TX;
        }
        HAL_UART_Init(&usart->instance);
        return ARM_DRIVER_OK;
        break;

    case ARM_USART_CONTROL_RX:
        if (arg == 1) {
            usart->instance.Init.Mode |= UART_MODE_RX;
        } else {
            usart->instance.Init.Mode &= ~UART_MODE_RX;
        }
        HAL_UART_Init(&usart->instance);
        return ARM_DRIVER_OK;
        break;

    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (control & ARM_USART_DATA_BITS_Msk) {
    case ARM_USART_DATA_BITS_5:
    case ARM_USART_DATA_BITS_6:
    case ARM_USART_DATA_BITS_7:
        return ARM_DRIVER_ERROR_PARAMETER;
    case ARM_USART_DATA_BITS_8:
        usart->instance.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case ARM_USART_DATA_BITS_9:
        usart->instance.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_NONE:
        usart->instance.Init.Parity = HAL_UART_PARITY_NONE;
        break;
    case ARM_USART_PARITY_ODD:
        usart->instance.Init.Parity = HAL_UART_PARITY_ODD;
        break;
    case ARM_USART_PARITY_EVEN:
        usart->instance.Init.Parity = HAL_UART_PARITY_EVEN;
        break;
    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1:
        usart->instance.Init.StopBits = UART_STOPBITS_1;
        break;
    case ARM_USART_STOP_BITS_2:
        usart->instance.Init.StopBits = UART_STOPBITS_2;
        break;
    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE:
        usart->instance.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        break;
    case ARM_USART_FLOW_CONTROL_RTS:
        usart->instance.Init.HwFlowCtl = UART_HWCONTROL_RTS;
        break;
    case ARM_USART_FLOW_CONTROL_CTS:
        usart->instance.Init.HwFlowCtl = UART_HWCONTROL_CTS;
        break;
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
        usart->instance.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
        break;
    default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    usart->instance.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&usart->instance) != HAL_OK) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

static ARM_USART_STATUS STM32_USART_GetStatus(STM32_USART_RESOURCES const
                                              *usart)
{
    return usart->status;
}

static int32_t STM32_USART_SetModemControl(ARM_USART_MODEM_CONTROL control, STM32_USART_RESOURCES const *usart)
{
    return 0;
}

static ARM_USART_MODEM_STATUS STM32_USART_GetModemStatus(STM32_USART_RESOURCES const *usart)
{
    return usart->modem_status;
}

//
//   Functions
//

static ARM_DRIVER_VERSION ARM_GetVersion(void)
{
    return DriverVersion;
}

// USART1
#ifdef USART1
static ARM_USART_CAPABILITIES USART1_GetCapabilities(void)
{
    return USART1_Resources.capabilities;
}

static int32_t USART1_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return STM32_USART_Initialize(cb_event, &USART1_Resources);
}

static int32_t USART1_Uninitialize(void)
{
    return STM32_USART_Uninitialize(&USART1_Resources);
}

static int32_t USART1_PowerControl(ARM_POWER_STATE state)
{
    return STM32_USART_PowerControl(state, &USART1_Resources);
}

static int32_t USART1_Send(const void *data, uint32_t num)
{
    return STM32_USART_Send(data, num, &USART1_Resources);
}

static int32_t USART1_Receive(void *data, uint32_t num)
{
    return STM32_USART_Receive(data, num, &USART1_Resources);
}

static int32_t USART1_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return STM32_USART_Transfer(data_out, data_in, num, &USART1_Resources);
}

static uint32_t USART1_GetTxCount(void)
{
    return STM32_USART_GetTxCount(&USART1_Resources);
}

static uint32_t USART1_GetRxCount(void)
{
    return STM32_USART_GetRxCount(&USART1_Resources);
}

static int32_t USART1_Control(uint32_t control, uint32_t arg)
{
    return STM32_USART_Control(control, arg, &USART1_Resources);
}

static ARM_USART_STATUS USART1_GetStatus(void)
{
    return STM32_USART_GetStatus(&USART1_Resources);
}

static int32_t USART1_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return STM32_USART_SetModemControl(control, &USART1_Resources);
}

static ARM_USART_MODEM_STATUS USART1_GetModemStatus(void)
{
    return STM32_USART_GetModemStatus(&USART1_Resources);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&USART1_Resources.instance);
}
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * UartHandle)
{
    uint32_t event = 0;

    event = ARM_USART_EVENT_TX_COMPLETE;

#ifdef USART1
    if (UartHandle->Instance == USART1_Resources.instance.Instance) {
        if (USART1_Resources.cb_event != NULL) {
            USART1_Resources.cb_event(event);
        }
    }
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * UartHandle)
{
    uint32_t event = 0;

    event = ARM_USART_EVENT_RECEIVE_COMPLETE;

#ifdef USART1
    if (UartHandle->Instance == USART1_Resources.instance.Instance) {
        if (USART1_Resources.cb_event != NULL) {
            USART1_Resources.cb_event(event);
        }
    }
#endif
}

#ifdef USART1
ARM_DRIVER_USART Driver_USART1 = {
    ARM_GetVersion,
    USART1_GetCapabilities,
    USART1_Initialize,
    USART1_Uninitialize,
    USART1_PowerControl,
    USART1_Send,
    USART1_Receive,
    USART1_Transfer,
    USART1_GetTxCount,
    USART1_GetRxCount,
    USART1_Control,
    USART1_GetStatus,
    USART1_SetModemControl,
    USART1_GetModemStatus
};
#endif