/* -----------------------------------------------------------------------------
 * Copyright (c) 2021 Arm Limited (or its affiliates).
 * All rights reserved.
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
 *
 * $Date:        27. October 2021
 * $Revision:    V1.0
 * Project:      UART Driver definitions for ST STM32G4xx
 * -------------------------------------------------------------------------- */

#ifndef __UART_STM32G4XX_H
#define __UART_STM32G4XX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_USART.h"
#include "stm32g4xx_hal.h"

#include "RTE_Components.h"
#include "IO_Config.h"

#define  VM_ASYNC                      (1UL)
#define  VM_SYNC                       (2UL)
#define  VM_IRDA                       (3UL)
#define  VM_SMARTCARD                  (4UL)
#define  Asynchronous                  VM_ASYNC
#define  IrDA                          VM_IRDA

#define UART_HAL_STATUS(stat)       ((stat == HAL_OK)      ? ARM_DRIVER_OK :            \
                                    ((stat == HAL_BUSY)    ? ARM_DRIVER_ERROR_BUSY :    \
                                    ((stat == HAL_TIMEOUT) ? ARM_DRIVER_ERROR_TIMEOUT : \
                                                             ARM_DRIVER_ERROR)))



#define UARTx_RESOURCE_ALLOC(x)     extern UART_HandleTypeDef    UART##x##_HANDLE;                        \
                                    static UART_INFO             UART##x##_Info;                  \
                                    static UART_TRANSFER_INFO    UART##x##_TransferInfo;          \
                                    static const UART_RESOURCES  UART##x##_Resources =  {         \
                                              &UART##x##_HANDLE,                                          \
                                              UART##x##_PERIPHERAL,                               \
                                              &UART##x##_Info,                                    \
                                              &UART##x##_TransferInfo,                            \
                                              UART##x##_DMA_USE_TX,                               \
                                              UART##x##_DMA_USE_RX,                               \
                                              0                                                   \
                                            }

#define UARTx_EXPORT_DRIVER(x)                                                                                                                                                          \
static int32_t                USART##x##_Initialize      (ARM_USART_SignalEvent_t cb_event)                  { return UART_Initialize (cb_event, &UART##x##_Resources); }               \
static int32_t                USART##x##_Uninitialize    (void)                                              { return UART_Uninitialize (&UART##x##_Resources); }                       \
static int32_t                USART##x##_PowerControl    (ARM_POWER_STATE state)                             { return UART_PowerControl (state, &UART##x##_Resources); }                \
static int32_t                USART##x##_Send            (const void *data, uint32_t num)                    { return UART_Send (data, num, &UART##x##_Resources); }                    \
static int32_t                USART##x##_Receive         (void *data, uint32_t num)                          { return UART_Receive (data, num, &UART##x##_Resources); }                 \
static int32_t                USART##x##_Transfer        (const void *data_out, void *data_in, uint32_t num) { return UART_Transfer (data_out, data_in, num, &UART##x##_Resources); }   \
static uint32_t               USART##x##_GetGetTxCount   (void)                                              { return UART_GetTxCount (&UART##x##_Resources); }                         \
static uint32_t               USART##x##_GetGetRxCount   (void)                                              { return UART_GetRxCount (&UART##x##_Resources); }                         \
static int32_t                USART##x##_Control         (uint32_t control, uint32_t arg)                    { return UART_Control (control, arg, &UART##x##_Resources); }              \
static ARM_USART_STATUS       USART##x##_GetStatus       (void)                                              { return UART_GetStatus (&UART##x##_Resources); }                          \
                                                                                                                                                                                      \
ARM_DRIVER_USART Driver_USART##x = {    \
  UART_GetVersion,                      \
  UART_GetCapabilities,                 \
  USART##x##_Initialize,                \
  USART##x##_Uninitialize,              \
  USART##x##_PowerControl,              \
  USART##x##_Send,                      \
  USART##x##_Receive,                   \
  USART##x##_Transfer,                  \
  USART##x##_GetGetTxCount,             \
  USART##x##_GetGetRxCount,             \
  USART##x##_Control,                   \
  USART##x##_GetStatus,                 \
  UART_SetModemControl,                 \
  UART_GetModemStatus                   \
}

// DMA Use
#define UART_DMA_USE_TX           (1U << 0)
#define UART_DMA_USE_RX           (1U << 1)
#define UART_DMA_USE_TX_RX        (USART_DMA_USE_TX | USART_DMA_USE_RX)

// UART1 Configuration
#ifdef MX_USART1
#if   (MX_USART1_VM == VM_ASYNC)

// Peripheral: USART1
#define UART1_PERIPHERAL          USART1

// Handle
#define UART1_HANDLE              huart1

// USART1 used in Asynchronous mode
#define USART1_MODE_ASYNC         1

// UART1 DMA USE
#ifdef MX_USART1_TX_DMA_Instance
  #define UART1_DMA_USE_TX        UART_DMA_USE_TX
#else
  #define UART1_DMA_USE_TX        0
#endif
#ifdef MX_USART1_RX_DMA_Instance
  #define UART1_DMA_USE_RX        UART_DMA_USE_RX
#else
  #define UART1_DMA_USE_RX        0
#endif
#define UART1_DMA_USE            (UART1_DMA_USE_TX | UART1_DMA_USE_RX)
#endif
#endif

// UART2 Configuration
#ifdef MX_USART2
#if   (MX_USART2_VM == VM_ASYNC)

// Peripheral: USART2
#define UART2_PERIPHERAL          USART2

// Handle
#define UART2_HANDLE              huart2

// USART2 used in Asynchronous mode
#define USART2_MODE_ASYNC         1

// UART2 DMA USE
#ifdef MX_USART2_TX_DMA_Instance
  #define UART2_DMA_USE_TX        UART_DMA_USE_TX
#else
  #define UART2_DMA_USE_TX        0
#endif
#ifdef MX_USART2_RX_DMA_Instance
  #define UART2_DMA_USE_RX        UART_DMA_USE_RX
#else
  #define UART2_DMA_USE_RX        0
#endif
#define UART2_DMA_USE            (UART2_DMA_USE_TX | UART2_DMA_USE_RX)
#endif
#endif

// UART3 Configuration
#ifdef MX_USART3
#if   (MX_USART3_VM == VM_ASYNC)

// Peripheral: USART3
#define UART3_PERIPHERAL          USART3

// Handle
#define UART3_HANDLE              huart3

// USART3 used in Asynchronous mode
#define USART3_MODE_ASYNC         1

// UART3 DMA USE
#ifdef MX_USART3_TX_DMA_Instance
  #define UART3_DMA_USE_TX        UART_DMA_USE_TX
#else
  #define UART3_DMA_USE_TX        0
#endif
#ifdef MX_USART3_RX_DMA_Instance
  #define UART3_DMA_USE_RX        UART_DMA_USE_RX
#else
  #define UART3_DMA_USE_RX        0
#endif
#define UART3_DMA_USE            (UART3_DMA_USE_TX | UART3_DMA_USE_RX)
#endif
#endif

// UART4 Configuration
#ifdef MX_UART4
#if   (MX_UART4_VM == VM_ASYNC)

// Peripheral: UART4
#define UART4_PERIPHERAL          UART4

// Handle
#define UART4_HANDLE              huart4

// UART4 used in Asynchronous mode
#define USART4_MODE_ASYNC         1

// UART4 DMA USE
#ifdef MX_UART4_TX_DMA_Instance
  #define UART4_DMA_USE_TX        UART_DMA_USE_TX
#else
  #define UART4_DMA_USE_TX        0
#endif
#ifdef MX_UART4_RX_DMA_Instance
  #define UART4_DMA_USE_RX        UART_DMA_USE_RX
#else
  #define UART4_DMA_USE_RX        0
#endif
#define UART4_DMA_USE            (UART4_DMA_USE_TX | UART4_DMA_USE_RX)
#endif
#endif

// UART5 Configuration
#ifdef MX_UART5
#if   (MX_UART5_VM == VM_ASYNC)

// Peripheral: UART5
#define UART5_PERIPHERAL          UART5

// Handle
#define UART5_HANDLE              huart5

// UART5 used in Asynchronous mode
#define USART5_MODE_ASYNC         1

// UART5 DMA USE
#ifdef MX_UART5_TX_DMA_Instance
  #define UART5_DMA_USE_TX        UART_DMA_USE_TX
#else
  #define UART5_DMA_USE_TX        0
#endif
#ifdef MX_UART5_RX_DMA_Instance
  #define UART5_DMA_USE_RX        UART_DMA_USE_RX
#else
  #define UART5_DMA_USE_RX        0
#endif
#define UART5_DMA_USE            (UART5_DMA_USE_TX | UART5_DMA_USE_RX)
#endif
#endif

// LPUART1 Configuration
#ifdef MX_LPUART1

// Peripheral: LPUART1
#define UART6_PERIPHERAL          LPUART1

// Handle
#define UART6_HANDLE              hlpuart1

// UART6 used in Asynchronous mode
#define USART6_MODE_ASYNC         1

// UART6 DMA USE
#ifdef MX_LPUART1_TX_DMA_Instance
  #define UART6_DMA_USE_TX        UART_DMA_USE_TX
#else
  #define UART6_DMA_USE_TX        0
#endif
#ifdef MX_LPUART1_RX_DMA_Instance
  #define UART6_DMA_USE_RX        UART_DMA_USE_RX
#else
  #define UART6_DMA_USE_RX        0
#endif
#define UART6_DMA_USE            (UART6_DMA_USE_TX | UART6_DMA_USE_RX)
#endif

#if defined(USART1_MODE_ASYNC) ||     \
    defined(USART2_MODE_ASYNC) ||     \
    defined(USART3_MODE_ASYNC) ||     \
    defined(USART4_MODE_ASYNC) ||     \
    defined(USART5_MODE_ASYNC) ||     \
    defined(USART6_MODE_ASYNC)

#define USARTx_MODE_ASYNC          1

// USART flags
#define UART_FLAG_INITIALIZED      ((uint8_t)(1U))
#define UART_FLAG_POWERED          ((uint8_t)(1U << 1))
#define UART_FLAG_CONFIGURED       ((uint8_t)(1U << 2))
#define UART_FLAG_TX_ENABLED       ((uint8_t)(1U << 3))
#define UART_FLAG_RX_ENABLED       ((uint8_t)(1U << 4))

// UART Transfer Information (Run-Time)
typedef struct {
  uint32_t              rx_num;         // Total number of receive data
  uint32_t              tx_num;         // Total number of transmit data
  uint32_t              rx_cnt;         // Number of data received
  uint32_t              tx_cnt;         // Number of data sent
  uint16_t              def_val;        // Default transfer value
  uint16_t              reserved;
} UART_TRANSFER_INFO;

typedef struct {
  uint8_t tx_busy;                      // Transmitter busy flag
  uint8_t rx_busy;                      // Receiver busy flag
  uint8_t tx_underflow;                 // Transmit data underflow detected (cleared on start of next send operation)
  uint8_t rx_overflow;                  // Receive data overflow detected (cleared on start of next receive operation)
  uint8_t rx_break;                     // Break detected on receive (cleared on start of next receive operation)
  uint8_t rx_framing_error;             // Framing error detected on receive (cleared on start of next receive operation)
  uint8_t rx_parity_error;              // Parity error detected on receive (cleared on start of next receive operation)
  uint8_t reserved;
} UART_STATUS;

// UART Information (Run-time)
typedef struct {
  ARM_USART_SignalEvent_t cb_event;     // Event Callback
  UART_STATUS             status;       // Status flags
  uint8_t                 flags;        // Current USART flags
  uint8_t                 reserved[3];
} UART_INFO;

// UART Resources definition
typedef const struct {
  UART_HandleTypeDef     *h;
  void                   *reg;          // UART peripheral pointer
  UART_INFO              *info;         // Run-Time Information
  UART_TRANSFER_INFO     *xfer;         // UART transfer information
  uint8_t                 dma_use_tx;
  uint8_t                 dma_use_rx;
  uint16_t                reserved;
} UART_RESOURCES;

// Global functions and variables exported by driver .c module
#ifdef USART1_MODE_ASYNC
extern ARM_DRIVER_USART Driver_USART1;
#endif

#ifdef USART2_MODE_ASYNC
extern ARM_DRIVER_USART Driver_USART2;
#endif

#ifdef USART3_MODE_ASYNC
extern ARM_DRIVER_USART Driver_USART3;
#endif

#ifdef USART4_MODE_ASYNC
extern ARM_DRIVER_USART Driver_USART4;
#endif

#ifdef USART5_MODE_ASYNC
extern ARM_DRIVER_USART Driver_USART5;
#endif

#ifdef USART6_MODE_ASYNC
extern ARM_DRIVER_USART Driver_USART6;
#endif

#endif
#endif /* __UART_STM32G4XX_H */
